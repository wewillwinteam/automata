use super::aim::FireOut;
use super::math::*;
use super::strategy::zone_at;
use super::{Brain, Goal, Mode, Tgt};
use crate::nav::Nav;
use crate::physics::{self, Body, Drive};
use crate::proto::{ActorKind, Obs, ProjKind, SelfObs, WeaponKind};
use crate::runtime::Budget;
use crate::world::World;

pub(crate) struct MoveOut {
    pub dir: V2,
    pub boost: bool,
    pub jump: bool,
}

struct Threat {
    eye: V3,
    aim_yaw: f64,
    gun: bool,
    laser: bool,
    lightning: bool,
    mortar: bool,
    ghost: bool,
    is_target: bool,
    charge_left: Option<f64>,
    los_now: bool,
    dist: f64,
}

struct Shot {
    p: V3,
    v: V3,
    dmg: f64,
    r: f64,
}

struct Blast {
    at: V3,
    t: f64,
    splash: f64,
    radius: f64,
    edge: f64,
    direct: f64,
}

struct Cand {
    dir: V2,
    speed: f64,
    boost: bool,
    pts: Vec<V3>,
    blocked: f64,
    score: f64,
}

const DT: f64 = 0.1;

fn simulate(world: &World, me: &SelfObs, dir: V2, speed: f64, mult: f64, boost: bool, n: usize) -> (Vec<V3>, f64) {
    let st = &me.stats;
    let ok = |v: V3| [fin(v[0], 0.0), fin(v[1], 0.0), fin(v[2], 0.0)];
    let mut body = Body { pos: ok(me.pos), vel: ok(me.vel), grounded: me.grounded, energy: fin(me.energy, 0.0), energy_wait: 0.0 };
    let mv = if speed > 0.0 { dir } else { [0.0, 0.0] };
    let d = Drive { mv, boost, jump: false, speed_mult: mult };
    let mut pts = Vec::with_capacity(n + 1);
    pts.push(body.pos);
    let mut blocked = 0.0;
    for _ in 0..n {
        let before = body.pos;
        let info = physics::step(world, st, &mut body, &d, DT);
        if info.blocked {
            blocked += if dist2(body.pos, before) < 0.3 * speed * DT { 1.0 } else { 0.3 };
        }
        pts.push(body.pos);
    }
    (pts, blocked)
}

impl Brain {
    fn threat_dps(&self, th: &Threat, d: f64) -> f64 {
        let p = &self.p;
        if th.ghost {
            return if d < 280.0 { p.ghost_dps } else { 0.0 };
        }
        let w = &self.cfg.weapons;
        let mut s = 0.0;
        let model = p.thr_model > 0.0;
        let squad = model && self.squad_mode();
        if th.gun && d < w.gun.range {
            s += if model { p.gun_dps * super::duel::gun_hit(squad, d) } else { p.gun_dps * (1.15 - d / 220.0).clamp(0.2, 1.0) };
        }
        if th.laser && d < w.laser.range {
            s += if model { p.laser_dps * super::duel::laser_hit(squad, d) / super::duel::LASER_REF } else { p.laser_dps };
        }
        if th.lightning && d < w.lightning.range + 6.0 {
            s += p.lightning_dps;
        }
        if th.mortar && d > 20.0 && d < 235.0 {
            s += p.mortar_dps;
        }
        s
    }

    fn threats(&self, obs: &Obs, world: &World, center: V3) -> Vec<Threat> {
        let t = obs.t;
        let mut v: Vec<Threat> = Vec::new();
        for tr in &self.mem.tracks {
            if tr.kind != ActorKind::Mech || t - tr.last_seen > 3.0 {
                continue;
            }
            let tp = tr.pos_at(t);
            let s = 1.0 + self.cfg.cubes.size_per_cube * tr.cubes.max(0.0);
            let fresh = t - tr.last_seen < self.p.fresh_s;
            let charge_left = if tr.charging && fresh { Some((self.cfg.weapons.laser.charge - (t - tr.charge_since)).clamp(0.1, 0.8)) } else { None };
            v.push(Threat {
                eye: up(tp, self.cfg.mech.eye_height * s),
                aim_yaw: tr.aim[0],
                gun: tr.has(WeaponKind::Gun) || tr.weapons.is_empty(),
                laser: tr.has(WeaponKind::Laser),
                lightning: tr.has(WeaponKind::Lightning),
                mortar: tr.has(WeaponKind::Mortar),
                ghost: false,
                is_target: self.target == Tgt::Mech(tr.id),
                charge_left,
                los_now: false,
                dist: dist(tp, center),
            });
        }
        if let Some(u) = &self.mem.unseen {
            if t - u.t < 4.0 {
                let r = self.p.ghost_range;
                let q = [u.from[0] + u.bearing.cos() * r, u.from[1] + u.bearing.sin() * r, u.from[2] + self.cfg.mech.eye_height];
                let explained = v.iter().any(|th| th.dist < 250.0 && adiff((th.eye[1] - center[1]).atan2(th.eye[0] - center[0]), u.bearing).abs() < 0.35);
                if !explained {
                    v.push(Threat { eye: q, aim_yaw: wrap(u.bearing + std::f64::consts::PI), gun: true, laser: u.laser, lightning: false, mortar: false, ghost: true, is_target: false, charge_left: None, los_now: false, dist: r });
                }
            }
        }
        v.sort_by(|a, b| a.dist.total_cmp(&b.dist));
        v.truncate(self.p.max_threats.max(0.0) as usize);
        for th in v.iter_mut() {
            th.los_now = world.sight(th.eye, center);
        }
        v
    }

    fn incoming(&self, obs: &Obs, world: &World, center: V3) -> (Vec<Shot>, Vec<Blast>) {
        let mut shots = Vec::new();
        let mut blasts = Vec::new();
        let m = &self.cfg.weapons.mortar;
        for pr in obs.projectiles.iter().take(64) {
            if !pr.pos.iter().chain(pr.vel.iter()).all(|x| x.is_finite()) {
                continue;
            }
            if pr.kind == ProjKind::Mortar {
                let g = m.gravity.max(1.0);
                let mut q = pr.pos;
                let mut v = pr.vel;
                let mut tt = 0.0;
                let h = 0.05;
                let mut hit = false;
                while tt < m.ttl.clamp(1.0, 8.0) {
                    v[2] -= g * h;
                    q = add(q, scale(v, h));
                    tt += h;
                    if world.solid(q[0], q[1], q[2]) || q[2] < world.terrain_z(q[0], q[1]) {
                        hit = true;
                        break;
                    }
                }
                if hit && dist(q, center) < 60.0 {
                    blasts.push(Blast { at: q, t: tt, splash: m.splash, radius: m.splash_radius.max(1.0), edge: m.splash_edge, direct: m.damage });
                }
            } else {
                let rel = sub(pr.pos, center);
                let (_, dxy, _) = closest_xy(rel, pr.vel, 0.0, 1.2);
                if dxy > 30.0 {
                    continue;
                }
                let (dmg, r) = if pr.kind == ProjKind::Npc { (self.cfg.npc.damage, self.cfg.projectile.npc_radius) } else { (self.cfg.weapons.gun.damage, self.cfg.projectile.gun_radius) };
                shots.push(Shot { p: pr.pos, v: pr.vel, dmg, r });
            }
        }
        (shots, blasts)
    }

    fn incoming_cost(&self, c: &Cand, shots: &[Shot], blasts: &[Blast], radius: f64, height: f64) -> f64 {
        let margin = self.p.proj_margin.max(0.1);
        let n = c.pts.len() - 1;
        let mut cost = 0.0;
        for s in shots {
            let mut worst: f64 = 0.0;
            for k in 0..n {
                let a = c.pts[k];
                let b = c.pts[k + 1];
                let ov = scale(sub(b, a), 1.0 / DT);
                let t0 = k as f64 * DT;
                let sp = add(s.p, scale(s.v, t0));
                let rel = sub(sp, up(a, height * 0.5));
                let rv = sub(s.v, ov);
                let (_, dxy, dz) = closest_xy(rel, rv, 0.0, DT);
                if dz.abs() > height * 0.5 + margin {
                    continue;
                }
                let reach = radius + s.r + margin;
                if dxy < reach {
                    worst = worst.max(((reach - dxy) / margin).clamp(0.0, 1.0));
                }
            }
            cost += worst * s.dmg;
        }
        for b in blasts {
            let kf = b.t / DT;
            let q = if kf >= n as f64 {
                let last = c.pts[n];
                let prev = c.pts[n - 1];
                let ext = (b.t - n as f64 * DT).min(0.8);
                add(last, scale(sub(last, prev), ext / DT))
            } else {
                let k = kf.floor() as usize;
                let f = kf - k as f64;
                add(scale(c.pts[k], 1.0 - f), scale(c.pts[(k + 1).min(n)], f))
            };
            let d = (dist2(q, b.at) - radius).max(0.0);
            if (q[2] - b.at[2]).abs() > height + b.radius {
                continue;
            }
            if d < b.radius + 3.0 {
                let fall = (d / b.radius).min(1.0);
                let mut dmg = b.splash * (1.0 - (1.0 - b.edge) * fall);
                if d <= 0.3 {
                    dmg += b.direct;
                }
                if d > b.radius {
                    dmg *= 0.3 * (1.0 - (d - b.radius) / 3.0);
                }
                cost += dmg;
            }
        }
        cost
    }

    pub(crate) fn plan_move(&mut self, obs: &Obs, world: &World, nav: &Nav, budget: &Budget, fc: &FireOut) -> MoveOut {
        let p = self.p.clone();
        let me = &obs.me;
        let st = &me.stats;
        let t = obs.t;
        let pos = me.pos;
        let n = (p.horizon_steps as usize).clamp(2, 16);
        let horizon = n as f64 * DT;
        let radius = st.radius.max(0.5);
        let height = st.height.max(1.0);
        let center = up(pos, height * 0.5);
        let walk = st.walk_speed.max(0.1);

        if let Some((tg, t0)) = self.jet {
            let dh = dist2(pos, tg);
            let landed = me.grounded && t - t0 > 0.35;
            if landed || t - t0 > 4.0 || (me.energy < 2.0 && me.grounded) {
                self.jet = None;
            } else {
                let d = if dh > 0.6 { unit2(tg[0] - pos[0], tg[1] - pos[1]) } else { [0.0, 0.0] };
                let rise = pos[2] < tg[2] + 1.5;
                if d[0].hypot(d[1]) > 0.1 {
                    self.last_dir = d;
                }
                return MoveOut { dir: d, boost: !me.grounded && rise, jump: me.grounded };
            }
        }

        if t < self.unstick_until {
            let d = self.unstick_dir;
            self.last_dir = d;
            let jet = self.unstick_level >= 2 && !me.grounded && me.energy > 15.0;
            return MoveOut { dir: d, boost: jet, jump: me.grounded };
        }

        // nav field
        let mut d0: Option<f64> = None;
        let mut navdir: Option<V2> = None;
        let mut goal_pos: Option<V3> = None;
        let mut arrive = 0.0;
        let mut fld: Option<std::sync::Arc<crate::nav::Field>> = None;
        let mut nav_wants = false;
        if let Goal::Field { center, arrive: a } = self.goal {
            arrive = a;
            let g = [center[0], center[1], pos[2]];
            goal_pos = Some(g);
            if let Some((_, _, f)) = &self.zone_field {
                if let Some(d) = nav.dist(f, pos).filter(|d| d.is_finite()) {
                    d0 = Some(d);
                    let stp = nav.step(f, pos);
                    if let Some(s) = stp {
                        if s.kind == crate::nav::EDGE_JET && me.grounded && me.energy >= s.energy + 8.0 && dist2(pos, s.target) < 14.0 {
                            self.jet = Some((s.target, t));
                            let dd = unit2(s.target[0] - pos[0], s.target[1] - pos[1]);
                            self.last_dir = dd;
                            return MoveOut { dir: dd, boost: false, jump: true };
                        }
                    }
                    navdir = stp.map(|s| s.dir).filter(|v| v[0].is_finite() && v[1].is_finite() && v[0].hypot(v[1]) > 0.2);
                    nav_wants = navdir.is_some();
                    fld = Some(f.clone());
                }
            }
            if d0.is_none() {
                d0 = Some(dist2(pos, g));
            }
            if navdir.is_none() {
                navdir = Some(unit2(g[0] - pos[0], g[1] - pos[1]));
            }
        }
        if let Goal::Nav { pos: g, arrive: a } = self.goal {
            goal_pos = Some(g);
            arrive = a;
            let calm = self.roof_jet || (t - self.mem.last_damage_t > 3.0 && !matches!(self.mode, Mode::Fight | Mode::Retreat));
            let jet_e = if calm { me.max_energy * 0.8 } else { -1.0 };
            let need = match &self.field {
                Some((fg, f, tb)) => {
                    let moved = dist(*fg, g);
                    (moved > p.field_regoal && (t - tb >= 0.3 || moved > 25.0)) || ((f.jet_limit() >= 0.0) != (jet_e >= 0.0) && t - tb >= 0.5)
                }
                None => true,
            };
            if need && budget.remaining_ms() > p.field_min_budget_ms && (self.field.is_none() || self.spent_ms() < 8.0) {
                let sg = nav.snap(g).filter(|q| q.iter().all(|x| x.is_finite())).unwrap_or(g);
                let lim = dist2(pos, sg) * 2.0 + 120.0;
                self.field = Some((g, std::sync::Arc::new(nav.field_limited(&[(sg, 0.0)], jet_e, lim)), t));
            }
            if let Some((fg, f, _)) = &self.field {
                let stale = dist(*fg, g);
                if stale <= 25.0 {
                    match nav.dist(f, pos) {
                        Some(d) if d.is_finite() => {
                            d0 = Some(d);
                            let stp = nav.step(f, pos);
                            if let Some(s) = stp {
                                if s.kind == crate::nav::EDGE_JET && me.grounded && me.energy >= s.energy + 8.0 && dist2(pos, s.target) < 14.0 {
                                    self.jet = Some((s.target, t));
                                    let dd = unit2(s.target[0] - pos[0], s.target[1] - pos[1]);
                                    self.last_dir = dd;
                                    return MoveOut { dir: dd, boost: false, jump: true };
                                }
                            }
                            navdir = stp.map(|s| s.dir).filter(|v| v[0].is_finite() && v[1].is_finite() && v[0].hypot(v[1]) > 0.2);
                            nav_wants = navdir.is_some();
                            fld = Some(f.clone());
                        }
                        _ => {
                            if stale <= p.field_regoal {
                                self.goal_unreachable = true;
                            }
                        }
                    }
                }
            }
            if d0.is_none() {
                d0 = Some(dist2(pos, g));
            }
            if navdir.is_none() {
                navdir = Some(unit2(g[0] - pos[0], g[1] - pos[1]));
            }
        }
        let reached = match (self.goal, d0) {
            (Goal::Nav { pos: g, .. }, Some(d)) => d <= arrive || dist2(pos, g) <= arrive.min(1.0),
            (Goal::Field { .. }, Some(d)) => d <= arrive,
            (Goal::Nav { .. }, None) | (Goal::Field { .. }, None) => false,
            _ => true,
        };

        // stuck
        let pursuing = matches!(self.goal, Goal::Nav { .. } | Goal::Field { .. }) && !reached && !matches!(self.mode, Mode::Fight | Mode::Retreat);
        if !(pursuing && (nav_wants || fld.is_none()) && d0.map_or(false, |d| d > arrive + 6.0)) {
            self.wants_since = t;
        }
        if !pursuing {
            self.moving_since = t;
        } else if t - self.moving_since >= p.stuck_window_s && (t - self.cmd_since >= p.stuck_window_s || t - self.wants_since >= p.stuck_window_s) {
            if let Some(mv) = self.mem.moved(t, p.stuck_window_s) {
                if mv < p.stuck_dist {
                    self.unstick_level = if t - self.unstick_until < 6.0 { self.unstick_level + 1 } else { 1 };
                    let base = navdir.filter(|v| v[0].hypot(v[1]) > 0.5).unwrap_or([1.0, 0.0]);
                    let turn = if self.unstick_level % 2 == 0 {
                        self.rng.range(-0.3, 0.3)
                    } else {
                        let sgn = if self.rng.f64() < 0.5 { -1.0 } else { 1.0 };
                        sgn * self.rng.range(0.9, 2.2)
                    };
                    let a = base[1].atan2(base[0]) + turn;
                    self.unstick_dir = [a.cos(), a.sin()];
                    self.unstick_until = t + p.unstick_s;
                    self.unstick_count += 1;
                    self.moving_since = t + p.unstick_s;
                    self.cmd_since = t + p.unstick_s;
                    self.wants_since = t + p.unstick_s;
                    self.last_dir = self.unstick_dir;
                    return MoveOut { dir: self.unstick_dir, boost: false, jump: me.grounded };
                }
            }
        }

        let mut threats = self.threats(obs, world, center);
        let (shots, blasts) = self.incoming(obs, world, center);
        let npcs: Vec<V3> = self
            .mem
            .tracks
            .iter()
            .filter(|tr| tr.kind == ActorKind::Npc && t - tr.last_seen < 2.0 && self.target != Tgt::Npc(tr.id))
            .map(|tr| tr.pos)
            .collect();

        // juke axis
        let mut juke: Option<V2> = None;
        for th in &threats {
            if th.ghost || !th.los_now || !(th.gun || th.mortar) || th.dist < p.juke_min_dist || (p.juke_max_dist > 0.0 && th.dist > p.juke_max_dist) {
                continue;
            }
            let to_us = (center[1] - th.eye[1]).atan2(center[0] - th.eye[0]);
            if adiff(th.aim_yaw, to_us).abs() < 0.35 {
                let u = unit2(center[0] - th.eye[0], center[1] - th.eye[1]);
                juke = Some([-u[1], u[0]]);
                break;
            }
        }
        if juke.is_some() && t >= self.juke_next {
            self.juke_side = -self.juke_side;
            self.juke_next = t + self.rng.range(p.juke_min_s, p.juke_max_s.max(p.juke_min_s));
        }
        let mut w_juke = p.w_juke;
        if p.w_cad > 0.0 {
            let mut pick: Option<(f64, V2)> = None;
            for m in &obs.mechs {
                if m.kind != ActorKind::Mech {
                    continue;
                }
                let d = dist2(m.pos, pos);
                if d < p.cad_min || d > p.cad_max {
                    continue;
                }
                if self.cad.due(m.id, t, p.cad_period, p.cad_win, p.cad_keep, p.cad_off, p.cad_reg.max(0.0) as u32) && pick.map_or(true, |x| d < x.0) {
                    let u = unit2(pos[0] - m.pos[0], pos[1] - m.pos[1]);
                    pick = Some((d, [-u[1], u[0]]));
                }
            }
            if let Some((_, ax)) = pick {
                if t >= self.cad_until - p.cad_period {
                    self.juke_side = -self.juke_side;
                }
                self.juke_next = t + p.cad_period + 0.05;
                self.cad_until = t + p.cad_period + 0.05;
                self.cad_axis = ax;
            }
            if t < self.cad_until {
                juke = Some(self.cad_axis);
                w_juke = p.w_cad;
            }
        }

        let under_fire = t - self.mem.last_damage_t < 1.0 || !shots.is_empty() || !blasts.is_empty() || threats.iter().any(|th| th.charge_left.is_some() && th.los_now);
        let has_lightning = me.weapons.iter().flatten().any(|w| w.kind == WeaponKind::Lightning);
        let reserve = if self.mode == Mode::Zone {
            3.0
        } else if has_lightning {
            p.energy_reserve_lightning
        } else {
            p.energy_reserve
        };
        let kite_boost = self.stand_off
            && p.mx_boost > 0.0
            && match self.goal {
                Goal::Range { pos: g, pref } => dist2(pos, g) < pref - p.range_band - p.mx_boost,
                _ => false,
            };
        let allow_boost = me.grounded && me.energy > reserve && (matches!(self.mode, Mode::Zone | Mode::Retreat | Mode::Drop | Mode::Roof) || under_fire || !me.in_zone || kite_boost);
        let slow = if fc.charging { st.charge_move_mult.clamp(0.1, 1.0) } else { 1.0 };

        // candidates
        let base = navdir.filter(|v| v[0].hypot(v[1]) > 0.5).unwrap_or(if self.last_dir[0].hypot(self.last_dir[1]) > 0.5 { self.last_dir } else { [1.0, 0.0] });
        let a0 = base[1].atan2(base[0]);
        let nd = (p.n_dirs as usize).clamp(4, 32);
        let nb = (p.n_boost_dirs as usize).clamp(0, 32);
        let mut cands: Vec<Cand> = Vec::with_capacity(nd + nb + 1);
        cands.push(Cand { dir: [0.0, 0.0], speed: 0.0, boost: false, pts: Vec::new(), blocked: 0.0, score: 0.0 });
        for k in 0..nd {
            let a = a0 + k as f64 * 2.0 * std::f64::consts::PI / nd as f64;
            cands.push(Cand { dir: [a.cos(), a.sin()], speed: walk * slow, boost: false, pts: Vec::new(), blocked: 0.0, score: 0.0 });
        }
        if allow_boost {
            for k in 0..nb {
                let a = a0 + k as f64 * 2.0 * std::f64::consts::PI / nb as f64;
                cands.push(Cand { dir: [a.cos(), a.sin()], speed: st.boost_speed.max(walk) * slow, boost: true, pts: Vec::new(), blocked: 0.0, score: 0.0 });
            }
        }

        let w_prog = match self.mode {
            Mode::Idle => p.w_progress_idle,
            Mode::Zone | Mode::Roof => p.w_progress_zone,
            Mode::Retreat => p.w_progress_retreat,
            Mode::Loot | Mode::Hunt => p.w_progress_loot,
            Mode::Drop => p.w_progress_drop,
            Mode::Fight => p.w_progress_fight,
        };
        let full_r = self.cfg.cubes.pickup_radius + 1.0;
        let avoid: Vec<V3> = if p.cube_avoid > 0.0 && me.cubes >= p.cube_avoid {
            self.mem.cubes.iter().filter(|c| dist2(c.pos, pos) < 30.0 && (c.pos[2] - pos[2]).abs() < 6.0).map(|c| c.pos).collect()
        } else {
            Vec::new()
        };
        let (zc, zr) = zone_at(&obs.zone, t, t + horizon, self.zone_b);
        let z = &obs.zone;
        let next_soon = z.shrink_start - t < 15.0;

        for c in cands.iter_mut() {
            let (pts, blocked) = simulate(world, me, c.dir, c.speed, slow, c.boost, n);
            c.pts = pts;
            c.blocked = blocked;
            let end = c.pts[n];
            let mut s = 0.0;
            match self.goal {
                Goal::Nav { .. } | Goal::Field { .. } => {
                    if !reached {
                        let g = goal_pos.unwrap_or(pos);
                        let start_d = d0.unwrap_or(0.0);
                        let de = match &fld {
                            Some(f) => nav.dist(f, end).filter(|d| d.is_finite()).unwrap_or(start_d + 6.0),
                            None => dist2(end, g),
                        };
                        s += w_prog * (start_d - de.max(arrive * 0.5)).clamp(-25.0, 25.0);
                        if let Some(ndv) = navdir {
                            s += p.w_navdir * (c.dir[0] * ndv[0] + c.dir[1] * ndv[1]) * (c.speed / walk).min(1.2);
                        }
                    } else {
                        s -= 0.6 * dist2(end, pos);
                    }
                }
                Goal::Range { pos: g, pref } => {
                    let e0 = ((dist2(pos, g) - pref).abs() - p.range_band).max(0.0);
                    let e1 = ((dist2(end, g) - pref).abs() - p.range_band).max(0.0);
                    // opening the distance comes before the side step while the target is inside the stand-off distance
                    let wr = if self.stand_off && dist2(pos, g) < pref - p.range_band { p.w_range * p.mx_w_back } else { p.w_range };
                    s += wr * (e0 - e1);
                }
                Goal::None => {
                    s -= 0.6 * dist2(end, pos);
                }
            }
            if c.speed == 0.0 {
                s += p.w_stand;
                if t - self.mem.last_damage_t < 1.5 {
                    s -= p.w_juke;
                }
            }
            s -= p.w_blocked * c.blocked;
            s += p.w_smooth * (c.dir[0] * self.last_dir[0] + c.dir[1] * self.last_dir[1]);
            if let Some(j) = juke {
                s += w_juke * self.juke_side * (c.dir[0] * j[0] + c.dir[1] * j[1]) * (c.speed / walk).min(1.3);
                if c.speed == 0.0 {
                    s -= w_juke * 0.6;
                }
            }
            if c.boost {
                s -= p.w_boost * st.boost_drain * horizon;
            }
            // zone
            let out = dist2v(end, zc) - zr + p.zone_inner;
            if out > 0.0 {
                let k = if self.mode == Mode::Zone || matches!(self.goal, Goal::Field { .. }) { p.zone_cost_zone_mode } else { p.zone_cost_mult };
                s -= k * z.dps * (horizon * (out / p.zone_inner.max(0.1)).min(1.0) + out / walk);
            }
            if next_soon && self.mode != Mode::Zone {
                let on = dist2v(end, z.next_center) - z.next_radius;
                let o0 = dist2v(pos, z.next_center) - z.next_radius;
                if on > 0.0 {
                    s -= p.w_next_zone * (on - o0.max(0.0));
                }
            }
            for q in &npcs {
                let d = dist(*q, end);
                if d < p.npc_avoid {
                    s -= p.w_npc * (p.npc_avoid - d) * 0.2 + p.w_npc * (dist(*q, pos) - d).max(0.0);
                }
            }
            for q in &avoid {
                if c.pts.iter().any(|pt| dist2(*pt, *q) < full_r && (pt[2] - q[2]).abs() < 4.5) {
                    s -= p.w_cube_avoid;
                }
            }
            s -= p.w_proj * self.incoming_cost(c, &shots, &blasts, radius, height);
            if p.w_antilead > 0.0 {
                s -= p.w_antilead * super::aimz::antilead(c.pts.as_slice(), DT, pos, me.vel, threats.iter().filter(|th| th.gun && !th.ghost && th.los_now).map(|th| (th.eye, th.aim_yaw)), radius + self.cfg.projectile.gun_radius, self.cfg.weapons.gun.speed.max(1.0), p.antilead_lat, p.antilead_sigma, self.cfg.weapons.gun.damage);
            }
            c.score = s;
        }

        // exposure pass, best partial scores first
        if !threats.is_empty() {
            let mut order: Vec<usize> = (0..cands.len()).collect();
            order.sort_by(|&a, &b| cands[b].score.total_cmp(&cands[a].score));
            let armed = fc.armed && fc.target.is_some();
            let laser_dmg = self.cfg.weapons.laser.damage;
            for th in threats.iter_mut() {
                th.dist = th.dist.max(1.0);
            }
            for &i in &order {
                let cheap = budget.remaining_ms() < p.expo_min_budget_ms || self.spent_ms() > 12.0;
                let c = &mut cands[i];
                let mut cost = 0.0;
                let mut bonus = 0.0;
                for th in &threats {
                    let w_mode = match self.mode {
                        Mode::Retreat => p.expo_retreat,
                        Mode::Zone => p.expo_zone,
                        Mode::Fight if th.is_target => {
                            if armed {
                                p.expo_fight_target
                            } else {
                                1.0
                            }
                        }
                        _ => 1.0,
                    };
                    let mut e = 0.0;
                    let mut los_end = th.los_now;
                    let mut los_mid = th.los_now;
                    for (j, &k) in [n / 2, n].iter().enumerate() {
                        let cpt = up(c.pts[k], height * 0.5);
                        let los = if cheap { th.los_now } else { world.sight(th.eye, cpt) };
                        if j == 0 {
                            los_mid = los;
                        } else {
                            los_end = los;
                        }
                        if los {
                            let d = dist(th.eye, cpt);
                            let to_us = (cpt[1] - th.eye[1]).atan2(cpt[0] - th.eye[0]);
                            let al = (1.0 - adiff(th.aim_yaw, to_us).abs()).clamp(0.0, 1.0);
                            e += 0.5 * self.threat_dps(th, d) * (0.4 + 0.6 * al);
                        }
                    }
                    cost += p.w_expo * w_mode * e * horizon;
                    if th.is_target && self.mode == Mode::Fight && armed && los_end {
                        bonus += p.w_fight_los;
                    }
                    if let Some(tl) = th.charge_left {
                        let hit = if tl <= horizon * 0.5 + 0.05 { los_mid } else { los_end };
                        if hit {
                            cost += p.w_charge * laser_dmg;
                        }
                    }
                }
                c.score += bonus - cost;
            }
        }

        #[cfg(test)]
        {
            self.dbg = format!("d0={:?} navdir={:?} reached={} field={} threats={}", d0, navdir, reached, fld.is_some(), threats.len());
            for c in &cands {
                let e = c.pts[n];
                self.dbg += &format!("\n    dir=({:.2},{:.2}) sp={:.1} b={} score={:.1} blocked={:.1} end=({:.1},{:.1},{:.1})", c.dir[0], c.dir[1], c.speed, c.boost, c.score, c.blocked, e[0], e[1], e[2]);
            }
        }
        let mut best = 0usize;
        for i in 1..cands.len() {
            if cands[i].score > cands[best].score {
                best = i;
            }
        }
        let c = &cands[best];
        let frac_speed = if c.speed > 0.0 { 1.0 } else { 0.0 };
        let dir = [c.dir[0] * frac_speed, c.dir[1] * frac_speed];
        if frac_speed > 0.0 {
            self.last_dir = c.dir;
        } else {
            self.cmd_since = t + DT;
            self.last_dir = [self.last_dir[0] * 0.5, self.last_dir[1] * 0.5];
        }
        MoveOut { dir, boost: c.boost, jump: false }
    }
}
