use super::aim::FireOut;
use super::math::*;
use super::plan::MoveOut;
use super::{Brain, Goal, Mode, Tgt};
use crate::nav::{Nav, CELL};
use crate::proto::{ActorKind, BeamKind, Event, Obs, WeaponKind};
use crate::world::World;

#[derive(Clone, Copy, PartialEq, Eq, Debug)]
pub(crate) enum PkSt {
    Off,
    Hide,
    Out,
    Back,
}

#[derive(Clone, Debug)]
pub(crate) struct Peek {
    pub st: PkSt,
    pub tid: i64,
    pub hide: V3,
    pub peek: V3,
    pub plan_t: f64,
    pub st_t: f64,
    pub fail_t: f64,
    pub shots: u32,
    pub dbg: bool,
    pub last_st: PkSt,
    pub ebeam_t: f64,
    pub dodge: Option<(V3, f64)>,
}

impl Peek {
    pub fn new() -> Peek {
        Peek { st: PkSt::Off, tid: -1, hide: [0.0; 3], peek: [0.0; 3], plan_t: -1e9, st_t: 0.0, fail_t: -1e9, shots: 0, dbg: std::env::var("LZ_DEBUG").is_ok(), last_st: PkSt::Off, ebeam_t: -1e9, dodge: None }
    }
}

struct Node {
    q: V3,
    cov: bool,
    clr: bool,
    seen_by: u32,
}

impl Brain {
    fn laser_slot(&self, obs: &Obs) -> Option<usize> {
        obs.me.weapons.iter().position(|w| w.as_ref().map_or(false, |w| w.kind == WeaponKind::Laser))
    }

    pub(crate) fn laser_pre(&mut self, obs: &Obs, world: &World, nav: &Nav) {
        let p = self.p.clone();
        let t = obs.t;
        let pos = obs.me.pos;
        if p.lz_seek > 0.0 && self.laser_slot(obs).is_none() && !matches!(self.mode, Mode::Zone) && (self.mode != Mode::Retreat || p.lz_seek_retreat > 0.5) {
            if self.want(&obs.me, WeaponKind::Laser).is_some() {
                let z = &obs.zone;
                let hot = t - self.mem.last_damage_t < p.lz_hot_s || self.mem.tracks.iter().any(|tr| tr.kind == ActorKind::Mech && t - tr.last_seen < p.lz_hot_s && dist(tr.pos_at(t), pos) < p.lz_hot_d);
                let mut best: Option<(f64, i64, V3)> = None;
                for it in &self.mem.pickups {
                    if it.kind != WeaponKind::Laser || self.mem.blacklisted(it.id, t) {
                        continue;
                    }
                    let d = dist(it.pos, pos);
                    let lim = if hot { p.lz_seek_fight } else { p.lz_seek };
                    if self.sf_far_block(d) {
                        continue;
                    }
                    if d > lim || dist2v(it.pos, z.center) > z.radius - 8.0 {
                        continue;
                    }
                    if best.map_or(true, |b| d < b.0) {
                        best = Some((d, it.id, it.pos));
                    }
                }
                if let Some((_, id, q)) = best {
                    self.mode = Mode::Loot;
                    self.goal = Goal::Nav { pos: q, arrive: 1.2 };
                    if self.goal_id != id {
                        self.pursuit = (id, t);
                    }
                    self.goal_id = id;
                }
            }
        }
        if p.lz_peek <= 0.0 {
            self.pk.st = PkSt::Off;
            return;
        }
        let Some(ls) = self.laser_slot(obs) else {
            self.pk.st = PkSt::Off;
            return;
        };
        let _ = ls;
        let mut tid = match self.target {
            Tgt::Mech(id) => id,
            _ => -1,
        };
        if tid < 0 && p.lz_any_tgt > 0.5 {
            let mut best: Option<(f64, i64)> = None;
            for tr in &self.mem.tracks {
                if tr.kind != ActorKind::Mech || t - tr.last_seen > p.lz_mem_s {
                    continue;
                }
                let s = dist(tr.pos_at(t), pos) - if t - tr.last_dmg_t < 3.0 { 100.0 } else { 0.0 } - if tr.has(WeaponKind::Laser) { 50.0 } else { 0.0 };
                if best.map_or(true, |b| s < b.0) {
                    best = Some((s, tr.id));
                }
            }
            if let Some((_, id)) = best {
                tid = id;
            }
        }
        let tr = self.mem.tracks.iter().find(|tr| tr.id == tid && tr.kind == ActorKind::Mech).cloned();
        let ok = matches!(self.mode, Mode::Fight) || (self.mode == Mode::Retreat && p.lz_retreat > 0.5) || (p.lz_any_tgt > 0.5 && matches!(self.mode, Mode::Loot | Mode::Hunt | Mode::Idle | Mode::Drop)) || (self.pk.st != PkSt::Off && self.mode != Mode::Zone);
        let Some(tr) = tr.filter(|tr| ok && t - tr.last_seen < p.lz_mem_s) else {
            if self.pk.dbg && (t * 10.0).round() as i64 % 20 == 0 {
                eprintln!("LZ t={:.2} no tgt mode={:?} target={:?} ok={}", t, self.mode, self.target, ok);
            }
            self.pk.st = PkSt::Off;
            return;
        };
        let tp = tr.pos_at(t);
        let dt = dist(tp, pos);
        if dt < p.lz_min_d || dt > p.lz_max_d || obs.me.hp < obs.me.max_hp * p.lz_min_hp {
            if self.pk.dbg {
                eprintln!("LZR range {:.0}", dt);
            }
            self.pk.st = PkSt::Off;
            return;
        }
        if tr.id != self.pk.tid {
            self.pk.tid = tr.id;
            self.pk.plan_t = -1e9;
        }
        let replan = t - self.pk.plan_t >= p.lz_replan_s && self.pk.st != PkSt::Out;
        if replan && t - self.pk.fail_t > 0.5 {
            self.pk.plan_t = t;
            match self.peek_plan(obs, world, nav, tp, tr.id) {
                Some((h, q)) => {
                    self.pk.hide = h;
                    self.pk.peek = q;
                    if self.pk.st == PkSt::Off {
                        self.pk.st = PkSt::Hide;
                        self.pk.st_t = t;
                    }
                }
                None => {
                    if self.pk.dbg {
                        eprintln!("LZ t={:.2} plan fail d={:.0}", t, dt);
                    }
                    self.pk.fail_t = t;
                    self.pk.st = PkSt::Off;
                }
            }
        }
        if self.pk.st != PkSt::Off {
            self.goal = Goal::Nav { pos: self.pk.hide, arrive: 0.8 };
        }
    }

    fn peek_plan(&self, obs: &Obs, world: &World, nav: &Nav, tp: V3, tid: i64) -> Option<(V3, V3)> {
        let p = &self.p;
        let pos = obs.me.pos;
        let t = obs.t;
        let h = self.cfg.mech.height;
        let te = up(tp, self.cfg.mech.eye_height);
        let tm = up(tp, self.cfg.mech.muzzle_height);
        let tc = up(tp, h * 0.5);
        let others: Vec<V3> = self
            .mem
            .tracks
            .iter()
            .filter(|o| o.kind == ActorKind::Mech && o.id != tid && t - o.last_seen < 4.0)
            .map(|o| up(o.pos_at(t), self.cfg.mech.eye_height))
            .filter(|e| dist(*e, pos) < 260.0)
            .collect();
        let (cnx, cny, o) = nav.grid();
        let r = p.lz_r.max(4.0);
        let i0 = (((pos[0] - r - o[0]) / CELL).floor().max(0.0)) as usize;
        let j0 = (((pos[1] - r - o[1]) / CELL).floor().max(0.0)) as usize;
        let i1 = ((((pos[0] + r - o[0]) / CELL).ceil()).max(0.0) as usize).min(cnx.saturating_sub(1));
        let j1 = ((((pos[1] + r - o[1]) / CELL).ceil()).max(0.0) as usize).min(cny.saturating_sub(1));
        let mut nodes: Vec<Node> = Vec::new();
        let step = p.lz_cell.max(1.0) as usize;
        let mut cj = j0;
        while cj <= j1 {
            let mut ci = i0;
            while ci <= i1 {
                for nd in nav.cell_node_ids(ci, cj) {
                    let q = nav.node_pos(nd);
                    if (q[2] - pos[2]).abs() > 3.0 || dist(q, pos) > r {
                        continue;
                    }
                    let mut cov = world.ray(te, up(q, h * 0.5)).is_some() && world.ray(tm, up(q, h * 0.95)).is_some() && world.ray(te, up(q, h * 0.95)).is_some();
                    if cov && p.lz_margin > 0.0 {
                        let dx = q[0] - tp[0];
                        let dy = q[1] - tp[1];
                        let l = dx.hypot(dy).max(1e-6);
                        let (nx, ny) = (-dy / l * p.lz_margin, dx / l * p.lz_margin);
                        for sgn in [-1.0, 1.0] {
                            let e2 = [te[0] + nx * sgn, te[1] + ny * sgn, te[2]];
                            if world.ray(e2, up(q, h * 0.6)).is_none() {
                                cov = false;
                            }
                        }
                    }
                    let clr = !cov && world.ray(up(q, self.cfg.mech.muzzle_height), tc).is_none();
                    let mut seen_by = 0;
                    if cov || clr {
                        for e in &others {
                            if world.ray(*e, up(q, h * 0.6)).is_none() {
                                seen_by += 1;
                            }
                        }
                    }
                    nodes.push(Node { q, cov, clr, seen_by });
                }
                if nodes.len() > 900 || self.spent_ms() > p.lz_ms {
                    break;
                }
                ci += step;
            }
            cj += step;
        }
        let mut cands: Vec<(f64, usize, usize)> = Vec::new();
        let smax = p.lz_step;
        for (hi, hn) in nodes.iter().enumerate() {
            if !hn.cov {
                continue;
            }
            let dh = dist(hn.q, pos);
            let keep = if self.pk.st != PkSt::Off && dist(hn.q, self.pk.hide) < 1.5 { p.lz_keep } else { 0.0 };
            let far = if dist(hn.q, tp) < p.lz_min_d { 40.0 } else { 0.0 };
            for (pi, pn) in nodes.iter().enumerate() {
                if !pn.clr {
                    continue;
                }
                let s = dist(hn.q, pn.q);
                if s > smax || s < 0.8 {
                    continue;
                }
                let sc = dh * p.lz_w_move + s * p.lz_w_step + pn.seen_by as f64 * p.lz_w_seen + hn.seen_by as f64 * p.lz_w_seen * 1.5 + far - keep;
                cands.push((sc, hi, pi));
            }
        }
        if self.pk.dbg {
            eprintln!("LZP nodes={} cov={} clr={} cands={} ms={:.1}", nodes.len(), nodes.iter().filter(|n| n.cov).count(), nodes.iter().filter(|n| n.clr).count(), cands.len(), self.spent_ms());
        }
        cands.sort_by(|a, b| a.0.partial_cmp(&b.0).unwrap_or(std::cmp::Ordering::Equal));
        for (_, hi, pi) in cands.iter().take(12) {
            let hq = nodes[*hi].q;
            let pq = nodes[*pi].q;
            if (dist(hq, pos) < 1.0 || nav.clear(pos, hq)) && nav.clear(hq, pq) {
                return Some((hq, pq));
            }
            if self.spent_ms() > p.lz_ms + 3.0 {
                break;
            }
        }
        if p.lz_hide_only > 0.5 {
            let mut best: Option<(f64, V3)> = None;
            for n in nodes.iter().filter(|n| n.cov && n.seen_by == 0) {
                let keep = if self.pk.st != PkSt::Off && dist(n.q, self.pk.hide) < 1.5 { p.lz_keep } else { 0.0 };
                let s = dist(n.q, pos) - keep;
                if best.map_or(true, |b| s < b.0) {
                    best = Some((s, n.q));
                }
            }
            if let Some((_, q)) = best {
                if dist(q, pos) < 1.0 || nav.clear(pos, q) {
                    return Some((q, q));
                }
            }
        }
        None
    }

    pub(crate) fn laser_post(&mut self, obs: &Obs, world: &World, fc: &mut FireOut, mv: &mut MoveOut) {
        let p = self.p.clone();
        if p.lz_scan_s > 0.0 && fc.target.is_none() && !fc.charging && self.laser_slot(obs).is_none() {
            let t = obs.t;
            let quiet = t - self.mem.last_damage_t > 2.0;
            let per = p.lz_scan_every;
            let phase = if per > 0.0 { t % per } else { 1e9 };
            if quiet && !self.sf_quiet_block() && (t < p.lz_scan_s || phase < p.lz_scan_len) {
                fc.aim = [wrap(obs.me.aim[0] + 1.5), 0.0];
            }
        }
        for b in &obs.beams {
            if b.kind == BeamKind::Laser && b.by == self.pk.tid {
                self.pk.ebeam_t = obs.t - b.age.max(0.0);
            }
        }
        for e in &obs.events {
            if let Event::Damaged(d) = e {
                if d.weapon == "laser" && d.by == Some(self.pk.tid) {
                    self.pk.ebeam_t = obs.t;
                }
                if self.pk.dbg {
                    eprintln!("LZD {} {:?} {:.0} mode={:?}", d.weapon, self.pk.st, d.amount, self.mode);
                }
            }
        }
if p.lz_cl > 0.5 && self.pk.st == PkSt::Off {
            self.counter_laser(obs, world, mv);
        }
        if self.pk.dbg && self.pk.st != self.pk.last_st {
            eprintln!("LZ t={:.2} {:?}->{:?} pos={:.0?} hide={:.0?} peek={:.0?} hp={:.0}", obs.t, self.pk.last_st, self.pk.st, obs.me.pos, self.pk.hide, self.pk.peek, obs.me.hp);
        }
        self.pk.last_st = self.pk.st;
        if self.pk.st == PkSt::Off {
            return;
        }
        let t = obs.t;
        let me = &obs.me;
        let pos = me.pos;
        let Some(ls) = self.laser_slot(obs) else {
            self.pk.st = PkSt::Off;
            return;
        };
        let Some(w) = me.weapons[ls].clone() else { return };
        let Some(tr) = self.mem.tracks.iter().find(|tr| tr.id == self.pk.tid).cloned() else {
            self.pk.st = PkSt::Off;
            return;
        };
        let tp = tr.pos_at(t);
        let at = self.aim_target(&tr);
        let c = up(tp, at.height * 0.5);
        let vpos = add(pos, scale([me.vel[0], me.vel[1], 0.0], 0.05));
        let muzzle = up(vpos, me.stats.muzzle_height);
        let line = world.ray(muzzle, c).is_none() && world.ray(up(pos, me.stats.muzzle_height), c).is_none();
        let direct = dir_to_aim(sub(c, up(pos, me.stats.muzzle_height)));
        let full = w.stats.charge.max(0.05);
        let charging = w.charge > 0.0;
        let pred = slew(me.aim, direct, me.stats.turn_rate * 0.05, me.stats.pitch_rate * 0.05);
        let d = len(sub(c, muzzle)).max(1.0);
        let tol = (at.radius * 0.8).atan2(d);
        let aligned = adiff(pred[0], direct[0]).abs() <= tol && (pred[1] - direct[1]).abs() <= (at.height * 0.4).atan2(d);
        let to = |a: V3, b: V3| -> (V2, f64) {
            let dx = b[0] - a[0];
            let dy = b[1] - a[1];
            let l = dx.hypot(dy);
            if l < 1e-6 {
                ([0.0, 0.0], 0.0)
            } else {
                ([dx / l, dy / l], l)
            }
        };
        let gun_fire = fc.fire;
        match self.pk.st {
            PkSt::Hide => {
                let (dir, dh) = to(pos, self.pk.hide);
                if dh > 0.7 {
                    mv.dir = [dir[0] * (dh / 2.0).min(1.0), dir[1] * (dh / 2.0).min(1.0)];
                    mv.boost = dh > 6.0 && me.energy > 30.0 && !charging;
                } else {
                    mv.dir = [0.0, 0.0];
                    mv.boost = false;
                }
                mv.jump = false;
                fc.aim = direct;
                let te = up(tp, self.cfg.mech.eye_height);
                let exposed = world.ray(te, up(pos, at.height.max(1.0) * 0.5)).is_none() || world.ray(te, up(pos, self.cfg.mech.height * 0.95)).is_none();
                if exposed && p.lz_exp_rush > 0.5 && dh > 0.7 {
                    mv.dir = dir;
                    mv.boost = me.energy > 10.0 && !charging;
                }
                let near = dh < p.lz_charge_d && !(exposed && p.lz_exp_rush > 0.5);
                if charging {
                    if line && aligned && w.charge >= full - 0.06 {
                        fc.fire[ls] = false;
                        self.pk.shots += 1;
                        self.pk.st = PkSt::Back;
                        self.pk.st_t = t;
                    } else {
                        fc.fire[ls] = true;
                        let since = t - self.pk.ebeam_t;
                        let held_s = (w.charge - full).max(0.0);
                        let wait = p.lz_sync > 0.0 && tr.has(WeaponKind::Laser) && since > 1.3 && held_s < p.lz_sync;
                        if w.charge >= full - 0.12 && dh < 1.5 && !wait && dist(self.pk.peek, self.pk.hide) > 0.5 {
                            self.pk.st = PkSt::Out;
                            self.pk.st_t = t;
                        }
                    }
                } else {
                    fc.fire[ls] = near && w.cooldown <= 0.05;
                }
            }
            PkSt::Out => {
                let (dir, dp) = to(pos, self.pk.peek);
                mv.dir = if dp > 0.3 { dir } else { [0.0, 0.0] };
                mv.boost = false;
                mv.jump = false;
                fc.aim = direct;
                if !charging {
                    self.pk.st = PkSt::Back;
                    self.pk.st_t = t;
                } else if line && aligned {
                    fc.fire[ls] = false;
                    self.pk.shots += 1;
                    self.pk.st = PkSt::Back;
                    self.pk.st_t = t;
                } else {
                    fc.fire[ls] = true;
                    if t - self.pk.st_t > p.lz_out_max {
                        self.pk.st = PkSt::Hide;
                        self.pk.st_t = t;
                        self.pk.plan_t = -1e9;
                    }
                }
                if line && p.lz_gun > 0.5 {
                    for (i, wo) in me.weapons.iter().enumerate() {
                        if i != ls && wo.as_ref().map_or(false, |x| x.kind == WeaponKind::Gun) {
                            fc.fire[i] = gun_fire[i] || (wo.as_ref().map_or(false, |x| x.ammo > 0.0 && x.reload <= 0.0) && aligned);
                        }
                    }
                }
            }
            PkSt::Back => {
                let (dir, dh) = to(pos, self.pk.hide);
                mv.dir = if dh > 0.4 { dir } else { [0.0, 0.0] };
                mv.boost = dh > 1.0 && me.energy > 15.0 && p.lz_boost > 0.5;
                mv.jump = false;
                fc.fire[ls] = false;
                if dh < 0.9 || t - self.pk.st_t > 1.5 {
                    self.pk.st = PkSt::Hide;
                    self.pk.st_t = t;
                    self.pk.plan_t = -1e9;
                }
            }
            PkSt::Off => {}
        }
        fc.charging = fc.fire[ls];
    }
}

impl Brain {
    fn counter_laser(&mut self, obs: &Obs, world: &World, mv: &mut MoveOut) {
        let p = self.p.clone();
        let t = obs.t;
        let me = &obs.me;
        let pos = me.pos;
        if let Some((q, until)) = self.pk.dodge {
            if t > until {
                self.pk.dodge = None;
            } else {
                let dx = q[0] - pos[0];
                let dy = q[1] - pos[1];
                let l = dx.hypot(dy);
                if l > 0.5 {
                    mv.dir = [dx / l, dy / l];
                    mv.boost = me.energy > 8.0;
                    mv.jump = false;
                }
                return;
            }
        }
        let full = self.cfg.weapons.laser.charge.max(0.1);
        let h = self.cfg.mech.height;
        let mut thr: Option<(V3, f64)> = None;
        for tr in &self.mem.tracks {
            if tr.kind != ActorKind::Mech || !tr.charging || t - tr.last_seen > 0.15 || !tr.has(WeaponKind::Laser) {
                continue;
            }
            let tp = tr.pos_at(t);
            let d = dist(tp, pos);
            if d < 15.0 {
                continue;
            }
            let brg = (pos[1] - tp[1]).atan2(pos[0] - tp[0]);
            if adiff(tr.aim[0], brg).abs() > p.lz_cl_cone {
                continue;
            }
            let left = full - (t - tr.charge_since);
            if left < 0.05 || left > p.lz_cl_left {
                continue;
            }
            if thr.map_or(true, |x| left < x.1) {
                thr = Some((tp, left));
            }
        }
        if self.pk.dbg {
            let n = self.mem.tracks.iter().filter(|tr| tr.kind == ActorKind::Mech && tr.charging && t - tr.last_seen < 0.15).count();
            if n > 0 {
                eprintln!("LZC charging_seen={} thr={:?}", n, thr.map(|x| x.1));
            }
        }
        let Some((tp, left)) = thr else { return };
        let te = up(tp, self.cfg.mech.eye_height);
        if world.ray(te, up(pos, h * 0.5)).is_some() && world.ray(te, up(pos, h * 0.95)).is_some() {
            return;
        }
        let shared = self.shared.clone();
        let nav = &shared.nav;
        let speed = me.stats.boost_speed.max(me.stats.walk_speed).max(1.0);
        let r = (left * speed + 1.0).min(p.lz_cl_r);
        let (cnx, cny, o) = nav.grid();
        let i0 = (((pos[0] - r - o[0]) / CELL).floor().max(0.0)) as usize;
        let j0 = (((pos[1] - r - o[1]) / CELL).floor().max(0.0)) as usize;
        let i1 = ((((pos[0] + r - o[0]) / CELL).ceil()).max(0.0) as usize).min(cnx.saturating_sub(1));
        let j1 = ((((pos[1] + r - o[1]) / CELL).ceil()).max(0.0) as usize).min(cny.saturating_sub(1));
        let mut best: Option<(f64, V3)> = None;
        for cj in j0..=j1 {
            for ci in i0..=i1 {
                for nd in nav.cell_node_ids(ci, cj) {
                    let q = nav.node_pos(nd);
                    let d = dist(q, pos);
                    if d > r || (q[2] - pos[2]).abs() > 2.5 || best.map_or(false, |b| d >= b.0) {
                        continue;
                    }
                    if world.ray(te, up(q, h * 0.5)).is_some() && world.ray(te, up(q, h * 0.95)).is_some() && nav.clear(pos, q) {
                        best = Some((d, q));
                    }
                }
            }
            if self.spent_ms() > p.lz_ms + 4.0 {
                break;
            }
        }
        if self.pk.dbg {
            eprintln!("LZC2 found={}", best.is_some());
        }
        if let Some((_, q)) = best {
            self.pk.dodge = Some((q, t + left + 0.3));
            let dx = q[0] - pos[0];
            let dy = q[1] - pos[1];
            let l = dx.hypot(dy).max(1e-6);
            mv.dir = [dx / l, dy / l];
            mv.boost = me.energy > 8.0;
            mv.jump = false;
        }
    }
}
