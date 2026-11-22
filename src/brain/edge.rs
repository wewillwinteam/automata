//! Edge patrol (practice, zone 1): live in a band just inside the nearest boundary (map wall
//! or zone line), so hunters come from the inside, and fight in a zigzag band off the wall.

use super::math::*;
use super::{Brain, Mode};
use crate::proto::{ActorKind, Obs};
use crate::world::World;

impl Brain {
    /// Zone 1 of a practice match with edge_on.
    pub(crate) fn edge_mode(&self, obs: &Obs) -> bool {
        self.p.edge_on > 0.0 && self.squad_mode() && !self.ffa.on && obs.zone.next_radius > 300.0
    }

    /// Distance from q to the nearest boundary (map wall or current zone line), the outward
    /// unit normal there, and the distance to the second-nearest map wall (corner check).
    pub(crate) fn boundary(&self, obs: &Obs, world: &World, q: V3) -> (f64, V2, f64) {
        let b = &world.bounds;
        let mut w = [(q[0] - b.min_x, [-1.0, 0.0]), (b.max_x - q[0], [1.0, 0.0]), (q[1] - b.min_y, [0.0, -1.0]), (b.max_y - q[1], [0.0, 1.0])];
        w.sort_by(|a, c| a.0.total_cmp(&c.0));
        let z = &obs.zone;
        let dz = z.radius - dist2v(q, z.center);
        if dz < w[0].0 {
            (dz, unit2(q[0] - z.center[0], q[1] - z.center[1]), w[0].0)
        } else {
            (w[0].0, w[0].1, w[1].0)
        }
    }

    /// The next patrol point: edge_band_mid inside the nearest boundary, edge_patrol_step
    /// along it; turns back before a corner.
    pub(crate) fn edge_patrol_goal(&mut self, obs: &Obs, world: &World, nav: &crate::nav::Nav, zi: &super::strategy::ZoneInfo) -> Option<V3> {
        let me = obs.me.pos;
        let (d, n, second) = self.boundary(obs, world, me);
        let mut tdir = [-n[1] * self.edge_side, n[0] * self.edge_side];
        let ahead = [me[0] + tdir[0] * 40.0, me[1] + tdir[1] * 40.0, me[2]];
        let (_, _, second_ahead) = self.boundary(obs, world, ahead);
        if second < 60.0 && second_ahead < second {
            self.edge_side = -self.edge_side;
            tdir = [-tdir[0], -tdir[1]];
        }
        let mid = self.p.edge_band_mid;
        let step = self.p.edge_patrol_step;
        let x = me[0] + n[0] * (d - mid) + tdir[0] * step;
        let y = me[1] + n[1] * (d - mid) + tdir[1] * step;
        let q0 = [x, y, fin(world.floor_below(x, y, me[2] + 3.0), me[2])];
        let q = nav.snap(q0).filter(|q| q.iter().all(|v| v.is_finite()) && dist(*q, q0) < 12.0)?;
        if !self.in_play_pub(obs, zi, q, dist(q, me)) {
            self.edge_side = -self.edge_side;
            return None;
        }
        Some(q)
    }

    /// Fight band (zone 1): with 2+ recent attackers and the boundary within reach, the band
    /// [lo, hi] off the boundary that the planner should zigzag in; also picks the in/out
    /// wobble target every 1-2 s.
    pub(crate) fn edge_fight_update(&mut self, obs: &Obs, world: &World) {
        self.edge_fight = None;
        let p = &self.p;
        if !self.edge_mode(obs) || p.edge_hi <= 0.0 {
            return;
        }
        let t = obs.t;
        let att = self.mem.tracks.iter().filter(|tr| tr.kind == ActorKind::Mech && t - tr.last_dmg_t < 5.0).count();
        let (d, _, second) = self.boundary(obs, world, obs.me.pos);
        if att < 2 || d > p.edge_hi + 30.0 || second < 30.0 {
            return;
        }
        let (lo, hi) = (p.edge_lo, p.edge_hi);
        if t >= self.edge_wob.1 {
            let target = if self.rng.f64() < 0.5 { lo } else { hi };
            let next = t + self.rng.range(1.0, 2.0);
            self.edge_wob = (target, next);
        }
        self.edge_fight = Some((lo, hi));
    }

    /// Planner penalty for a trajectory ending at q while the fight band is on.
    pub(crate) fn edge_cost(&self, obs: &Obs, world: &World, q: V3) -> f64 {
        match self.edge_fight {
            Some((lo, hi)) if self.mode == Mode::Fight => {
                let (d, _, _) = self.boundary(obs, world, q);
                let target = self.edge_wob.0.clamp(lo, hi);
                self.p.edge_w * ((d - target).abs() - 2.5).max(0.0)
            }
            _ => 0.0,
        }
    }

    /// Retreating near the boundary: direction along it, away from the attackers' centre.
    pub(crate) fn edge_away_update(&mut self, obs: &Obs, world: &World, retreating: bool) {
        self.edge_away = None;
        if !retreating || !self.edge_mode(obs) {
            return;
        }
        let t = obs.t;
        let me = obs.me.pos;
        let (d, n, _) = self.boundary(obs, world, me);
        if d > 60.0 {
            return;
        }
        let att: Vec<V3> = self.mem.tracks.iter().filter(|tr| tr.kind == ActorKind::Mech && t - tr.last_dmg_t < 5.0 && t - tr.last_seen < 5.0).map(|tr| tr.pos_at(t)).collect();
        if att.is_empty() {
            return;
        }
        let (cx, cy) = att.iter().fold((0.0, 0.0), |a, q| (a.0 + q[0], a.1 + q[1]));
        let k = att.len() as f64;
        let from = [me[0] - cx / k, me[1] - cy / k];
        let tan = [-n[1], n[0]];
        let s = if tan[0] * from[0] + tan[1] * from[1] >= 0.0 { 1.0 } else { -1.0 };
        self.edge_away = Some([tan[0] * s, tan[1] * s]);
    }
}
