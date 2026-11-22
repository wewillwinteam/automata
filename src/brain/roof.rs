use super::math::*;
use super::strategy::ZoneInfo;
use super::{Brain, Goal, Mode, Tgt};
use crate::nav::{self, Nav};
use crate::proto::{ActorKind, Obs};
use crate::world::World;
use std::collections::HashMap;

pub(crate) struct Region {
    pub expo: f64,
    pub keep: V3,
    pub setback: f64,
    pub height: f64,
    pub top: f64,
}

pub(crate) struct RoofMap {
    pub regions: Vec<Region>,
    pub node_region: HashMap<u32, u32>,
}

impl RoofMap {
    pub fn build(world: &World, nav: &Nav, min_nodes: usize, min_setback: f64, height: f64) -> RoofMap {
        let n = nav.node_count();
        let mut comp: Vec<u32> = vec![u32::MAX; n];
        let mut regions = Vec::new();
        let mut node_region = HashMap::new();
        let mut next_id = 0u32;
        for s in 0..n as u32 {
            if !nav.node_roof(s) || comp[s as usize] != u32::MAX {
                continue;
            }
            let id = next_id;
            next_id += 1;
            comp[s as usize] = id;
            let mut stack = vec![s];
            let mut list = Vec::new();
            while let Some(a) = stack.pop() {
                list.push(a);
                let za = nav.node_pos(a)[2];
                for e in nav.edges(a) {
                    let b = e.to;
                    if e.kind != nav::EDGE_WALK || (b as usize) >= n || comp[b as usize] != u32::MAX || !nav.node_roof(b) {
                        continue;
                    }
                    if (nav.node_pos(b)[2] - za).abs() > 1.3 {
                        continue;
                    }
                    comp[b as usize] = id;
                    stack.push(b);
                }
            }
            if list.len() < min_nodes {
                continue;
            }
            let cnt: Vec<usize> = list
                .iter()
                .map(|&a| nav.edges(a).iter().filter(|e| e.kind == nav::EDGE_WALK && (e.to as usize) < n && comp[e.to as usize] == id).count())
                .collect();
            let maxc = cnt.iter().copied().max().unwrap_or(0);
            let bnd: Vec<V3> = list.iter().zip(cnt.iter()).filter(|(_, &c)| c < maxc).map(|(&a, _)| nav.node_pos(a)).collect();
            if bnd.is_empty() {
                continue;
            }
            let mut sb: Vec<(f64, u32)> = Vec::with_capacity(list.len());
            for &a in &list {
                let q = nav.node_pos(a);
                let mut m = f64::INFINITY;
                for b in &bnd {
                    m = m.min((q[0] - b[0]).hypot(q[1] - b[1]));
                }
                sb.push((m, a));
            }
            sb.sort_by(|x, y| y.0.total_cmp(&x.0));
            let top_sb = sb[0].0;
            if top_sb < min_setback {
                continue;
            }
            let cut = (top_sb * 0.5).max(min_setback);
            let cands: Vec<(f64, u32)> = sb.iter().copied().filter(|x| x.0 >= cut).collect();
            let stride = (cands.len() / 24).max(1);
            let mut best: Option<(f64, f64, u32)> = None;
            for (k, &(m, a)) in cands.iter().enumerate() {
                if k % stride != 0 {
                    continue;
                }
                let e = exposure(world, nav.node_pos(a), height);
                let s = e - 0.02 * m;
                if best.map_or(true, |b| s < b.0) {
                    best = Some((s, m, a));
                }
            }
            let Some((_, bm, ba)) = best else { continue };
            let best = (bm, ba);
            let keep = nav.node_pos(best.1);
            let ground = fin(world.terrain_z(keep[0], keep[1]), keep[2]);
            let ri = regions.len() as u32;
            for &a in &list {
                node_region.insert(a, ri);
            }
            let expo = exposure(world, keep, height);
            regions.push(Region { expo, keep, setback: best.0, height: keep[2] - ground, top: keep[2] });
        }
        RoofMap { regions, node_region }
    }

    pub fn region_of(&self, nav: &Nav, p: V3) -> Option<usize> {
        let n = nav.snap_node(p)?;
        let r = *self.node_region.get(&n)? as usize;
        if p[2] > self.regions[r].top - 2.0 {
            Some(r)
        } else {
            None
        }
    }
}

/// Weighted count of ground and roof observers within 170 m that see the top of a mech standing at p.
pub(crate) fn exposure(world: &World, p: V3, height: f64) -> f64 {
    let top = up(p, height * 0.95);
    let mid = up(p, height * 0.5);
    let mut e = 0.0;
    for (r, w) in [(50.0, 1.0), (90.0, 0.8), (130.0, 0.6), (170.0, 0.4)] {
        for k in 0..32 {
            let a = k as f64 * std::f64::consts::PI / 16.0 + r * 0.01;
            let x = p[0] + r * a.cos();
            let y = p[1] + r * a.sin();
            let fz = world.floor_below(x, y, 1.0e3);
            if !fz.is_finite() {
                continue;
            }
            let eye = [x, y, fz + 8.0];
            if world.ray(eye, mid).is_none() {
                e += 2.0 * w;
            } else if world.ray(eye, top).is_none() {
                e += w;
            }
        }
    }
    e
}

struct Foe {
    id: i64,
    pos: V3,
    hpf: f64,
    climber: bool,
    line: bool,
}

impl Brain {
    fn roof_choose(&self, obs: &Obs, zi: &ZoneInfo, rm: &RoofMap) -> Option<usize> {
        let p = &self.p;
        let z = &obs.zone;
        let pos = obs.me.pos;
        let mut best: Option<(f64, usize)> = None;
        for (i, r) in rm.regions.iter().enumerate() {
            if r.setback < p.roof_min_setback || r.height < p.roof_min_h || r.height > p.roof_max_h {
                continue;
            }
            if self.roof_ban.iter().any(|&(b, until)| b == i && obs.t < until) {
                continue;
            }
            let dn = dist2v(r.keep, z.next_center);
            let inner = z.next_radius - dn - p.roof_zone_margin;
            if inner < 0.0 || dist2v(r.keep, zi.cur_c) > zi.cur_r - p.roof_zone_margin {
                continue;
            }
            let d = dist2(r.keep, pos);
            let mut s = 2.0 * r.setback.min(45.0) - p.roof_w_expo * r.expo - p.roof_w_dist * d + 0.2 * inner.min(120.0) - 0.4 * (r.height - 20.0).abs();
            if self.roof_sel == Some(i) {
                s += 25.0;
            }
            if best.map_or(true, |b| s > b.0) {
                best = Some((s, i));
            }
        }
        best.map(|b| b.1)
    }

    fn roof_step_point(&mut self, world: &World, nav: &Nav, rm: &RoofMap, ri: usize, tid: i64, tp: V3, t: f64, muzzle_h: f64) -> Option<V3> {
        if let Some((id, q, t0)) = self.roof_step {
            if id == tid && t - t0 < 0.6 {
                return q;
            }
        }
        let r = &rm.regions[ri];
        let keep = r.keep;
        let tc = up(tp, self.cfg.mech.height * 0.5);
        let b0 = (tp[1] - keep[1]).atan2(tp[0] - keep[0]);
        let mut best: Option<(f64, V3)> = None;
        let maxd = r.setback + 12.0;
        let mut rays = 0;
        for db in [0.0, 0.45, -0.45, 0.9, -0.9] {
            let b = b0 + db;
            let mut k = 0.0;
            while k <= maxd && rays < 90 {
                let q0 = [keep[0] + b.cos() * k, keep[1] + b.sin() * k, keep[2] + 1.0];
                k += 2.5;
                let Some(n) = nav.snap_node(q0) else { continue };
                if rm.node_region.get(&n).map_or(true, |&x| x as usize != ri) {
                    continue;
                }
                let q = nav.node_pos(n);
                if best.map_or(false, |bb| dist2(q, keep) >= bb.0) {
                    break;
                }
                rays += 1;
                if world.ray(up(q, muzzle_h), tc).is_none() {
                    best = Some((dist2(q, keep), q));
                    break;
                }
            }
        }
        let q = best.map(|b| b.1);
        self.roof_step = Some((tid, q, t));
        q
    }

    pub(crate) fn roof_decide(&mut self, obs: &Obs, world: &World, nav: &Nav, zi: &ZoneInfo) {
        self.roof_hold = false;
        self.roof_jet = false;
        let p = self.p.clone();
        if p.roof_mode <= 0.0 {
            return;
        }
        let Some(rm) = self.roof.clone() else { return };
        let t = obs.t;
        let me = &obs.me;
        let pos = me.pos;
        let hpf = frac(me.hp, me.max_hp);
        for ev in &obs.events {
            if let crate::proto::Event::Kill(_) = ev {
                self.kills += 1;
            }
        }
        if (self.kills as f64) < p.roof_after_kills || t < p.roof_start_s || zi.urgent || self.held(t) {
            return;
        }
        if self.mode == Mode::Loot && self.roof_on.is_none() {
            if let Goal::Nav { pos: g, .. } = self.goal {
                if dist(g, pos) < p.roof_loot_first {
                    return;
                }
            }
        }
        if self.roof_sel.is_none() || t - self.roof_sel_t > 1.0 {
            let prev = self.roof_sel;
            self.roof_sel = self.roof_choose(obs, zi, &rm);
            self.roof_sel_t = t;
            if prev != self.roof_sel {
                self.roof_step = None;
                self.roof_rest = false;
            }
        }
        let Some(ri) = self.roof_sel else { return };
        let r = &rm.regions[ri];
        let on = rm.region_of(nav, pos) == Some(ri);
        self.roof_on = if on { Some(ri) } else { None };
        if on && self.roof_first_t < 0.0 {
            self.roof_first_t = t;
        }

        let center = up(pos, me.stats.height * 0.5);
        let mut foes: Vec<Foe> = Vec::new();
        for tr in &self.mem.tracks {
            if tr.kind != ActorKind::Mech || t - tr.last_seen > p.roof_memory_s {
                continue;
            }
            let tp = tr.pos_at(t);
            let near_roof = dist2(tp, r.keep) < p.roof_climb_r;
            let climber = near_roof && t - tr.last_seen < 1.5 && (tp[2] > r.top - 4.0 || (tr.vel[2] > 2.0 && tp[2] > r.top - r.height * 0.7));
            let line = t - tr.last_seen < 1.0 && foes.len() < 6 && world.sight(up(tp, self.cfg.mech.eye_height), center);
            foes.push(Foe { id: tr.id, pos: tp, hpf: frac(tr.hp, tr.max_hp), climber, line });
        }
        let lines = foes.iter().filter(|f| f.line).count();
        let n_climb = foes.iter().filter(|f| f.climber).count();

        if !on {
            if let Some(c) = foes.iter().filter(|f| f.climber).min_by(|a, b| dist(a.pos, pos).total_cmp(&dist(b.pos, pos))) {
                if self.roof_first_t >= 0.0 && dist(c.pos, pos) < 60.0 {
                    return;
                }
            }
            if self.roof_first_t >= 0.0 && self.mode == Mode::Loot {
                let safe = foes.iter().all(|f| dist(f.pos, pos) > p.roof_loot_safe);
                if safe && hpf > 0.5 {
                    return;
                }
            }
            if p.roof_travel_fight > 0.0 && t - self.mem.last_damage_t < 3.0 && matches!(self.mode, Mode::Fight | Mode::Retreat) {
                let att = self.mem.tracks.iter().any(|tr| tr.kind == ActorKind::Mech && t - tr.last_dmg_t < 3.0 && t - tr.last_seen < 1.0 && dist(tr.pos_at(t), pos) < 130.0);
                if att && dist2(r.keep, pos) > p.roof_travel_commit {
                    return;
                }
            }
            self.mode = Mode::Roof;
            self.goal = Goal::Nav { pos: r.keep, arrive: 2.5 };
            self.goal_id = -1;
            self.roof_jet = true;
            self.allow_crates = false;
            return;
        }

        if n_climb >= 2 && hpf < p.roof_leave_hp {
            self.roof_ban.push((ri, t + p.roof_ban_s));
            self.roof_sel = None;
            self.roof_on = None;
            return;
        }
        if hpf >= p.roof_fire_hp && lines < 2 {
            self.roof_rest = false;
        }
        if hpf < p.roof_rest_hp || lines >= 2 {
            self.roof_rest = true;
        }
        self.allow_crates = false;
        if let Some(c) = foes.iter().filter(|f| f.climber).min_by(|a, b| dist(a.pos, pos).total_cmp(&dist(b.pos, pos))) {
            self.mode = Mode::Fight;
            self.target = Tgt::Mech(c.id);
            self.goal = Goal::Nav { pos: r.keep, arrive: 6.0 };
            self.goal_id = -1;
            return;
        }
        if self.mode == Mode::Loot && !self.roof_rest {
            if let Goal::Nav { pos: g, .. } = self.goal {
                if rm.region_of(nav, g) == Some(ri) || foes.iter().all(|f| dist(f.pos, pos) > p.roof_loot_safe) && hpf > 0.7 {
                    return;
                }
            }
        }
        if !self.roof_rest {
            let mut best: Option<(f64, i64, V3)> = None;
            for f in &foes {
                let d = dist2(f.pos, r.keep);
                if d > p.roof_engage {
                    continue;
                }
                let mut s = -d + 80.0 * (1.0 - f.hpf);
                if self.target == Tgt::Mech(f.id) {
                    s += 20.0;
                }
                if best.map_or(true, |b| s > b.0) {
                    best = Some((s, f.id, f.pos));
                }
            }
            if let Some((_, id, tp)) = best {
                if let Some(q) = self.roof_step_point(world, nav, &rm, ri, id, tp, t, me.stats.muzzle_height) {
                    self.mode = Mode::Fight;
                    self.target = Tgt::Mech(id);
                    self.goal = Goal::Nav { pos: q, arrive: 1.2 };
                    self.goal_id = -1;
                    return;
                }
            }
        }
        self.mode = Mode::Roof;
        self.target = Tgt::None;
        self.goal = Goal::Nav { pos: r.keep, arrive: 2.0 };
        self.goal_id = -1;
        self.roof_hold = true;
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    #[ignore]
    fn regions_real_map() {
        let dir = std::path::Path::new(env!("CARGO_MANIFEST_DIR")).join("../kit/arena");
        let Ok(w) = World::load(&dir) else { return };
        let nav = Nav::build(&w, &crate::proto::MechCfg::default());
        let t0 = std::time::Instant::now();
        let rm = RoofMap::build(&w, &nav, 120, 7.0, 9.0);
        eprintln!("build {:.1} ms, {} regions", t0.elapsed().as_secs_f64() * 1e3, rm.regions.len());
        for r in &rm.regions {
            eprintln!("keep {:.0},{:.0},{:.1} setback {:.1} h {:.1} expo {:.1}", r.keep[0], r.keep[1], r.keep[2], r.setback, r.height, r.expo);
        }
    }
}
