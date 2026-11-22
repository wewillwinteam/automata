use super::math::*;
use super::Brain;
use crate::nav::Nav;
use crate::proto::{Event, Obs};
use crate::world::World;

pub(crate) struct WaveState {
    pub kills: u32,
    pub prep: Option<(V3, f64)>,
    pub next_eval: f64,
    pub k: u32,
    pub last_kill_t: f64,
}

impl WaveState {
    pub fn new() -> WaveState {
        WaveState { kills: 0, prep: None, next_eval: 0.0, k: 0, last_kill_t: 0.0 }
    }
}

impl Brain {
    pub(crate) fn wave_count(&mut self, obs: &Obs) {
        for ev in &obs.events {
            if let Event::Kill(_) = ev {
                self.wave.kills += 1;
                self.wave.last_kill_t = obs.t;
            }
        }
    }

    fn wave_armed(&self) -> bool {
        if self.p.ready_once > 0.5 {
            return self.p.ready_kill >= 1.0 && (self.wave.kills as f64) == self.p.ready_kill - 1.0 && self.squad_mode();
        }
        self.p.ready_kill >= 1.0 && (self.wave.kills as f64) >= self.p.ready_kill - 1.0
    }

    pub(crate) fn ready(&self, obs: &Obs) -> bool {
        let p = &self.p;
        let me = &obs.me;
        if obs.t >= p.ready_deadline || (p.ready_once > 0.5 && obs.t >= self.wave.last_kill_t + p.ready_wait) {
            return true;
        }
        if me.cubes < p.ready_cubes.min(self.cfg.cubes.stat_cap.max(1.0)) || frac(me.hp, me.max_hp) < p.ready_hp {
            return false;
        }
        if p.ready_second > 0.0 && me.weapons.iter().any(|w| w.is_none()) {
            return false;
        }
        if p.ready_cover > 0.0 {
            match self.wave.prep {
                Some((q, _)) if dist(q, me.pos) <= p.ready_cover => {}
                _ => return false,
            }
        }
        true
    }

    pub(crate) fn gate_closed(&self, obs: &Obs) -> bool {
        self.wave_armed() && !self.ready(obs)
    }

    pub(crate) fn spare(&self, hp: f64, max_hp: f64) -> bool {
        frac(hp, max_hp) < self.p.ready_spare_frac || hp < self.p.ready_spare_hp
    }

    pub(crate) fn prep_goal(&self) -> Option<V3> {
        if self.p.prep_pos <= 0.0 || !(self.p.ready_kill < 1.0 || self.wave_armed()) {
            return None;
        }
        self.wave.prep.map(|x| x.0)
    }

    fn exposure(&self, world: &World, q: V3, h: f64) -> f64 {
        let n = 16;
        let mut seen = 0.0;
        let c = up(q, h * 0.6);
        for k in 0..n {
            let a = k as f64 * std::f64::consts::TAU / n as f64;
            for (r, w) in [(45.0, 1.0), (110.0, 0.7)] {
                let x = q[0] + r * a.cos();
                let y = q[1] + r * a.sin();
                let z = fin(world.terrain_z(x, y), q[2]);
                let e = [x, y, z + self.cfg.mech.eye_height];
                if world.ray(e, c).is_none() {
                    seen += w;
                }
            }
        }
        seen / (n as f64 * 1.7)
    }

    /// Share of long sight lines from a spot: an open spot lets the laser work on hunters while they cross open ground.
    fn sightlines(&self, world: &World, q: V3, h: f64) -> f64 {
        let n = 16;
        let mut seen = 0.0;
        let e = up(q, self.cfg.mech.eye_height.min(h));
        for k in 0..n {
            let a = k as f64 * std::f64::consts::TAU / n as f64;
            for (r, w) in [(110.0, 1.0), (170.0, 1.0)] {
                let x = q[0] + r * a.cos();
                let y = q[1] + r * a.sin();
                let z = fin(world.terrain_z(x, y), q[2]);
                if world.ray(e, [x, y, z + h * 0.6]).is_none() {
                    seen += w;
                }
            }
        }
        seen / (n as f64 * 2.0)
    }

    pub(crate) fn wave_update(&mut self, obs: &Obs, world: &World, nav: &Nav) {
        if self.p.prep_pos <= 0.0 {
            return;
        }
        let t = obs.t;
        let z = &obs.zone;
        if !(z.next_radius.is_finite() && z.next_center[0].is_finite()) {
            return;
        }
        let me = &obs.me;
        let keep_r = (z.next_radius - self.p.zone_margin - 10.0).max(z.next_radius * 0.4);
        if let Some((q, _)) = self.wave.prep {
            if dist2v(q, z.next_center) > keep_r + 2.0 {
                self.wave.prep = None;
            }
        }
        if t < self.wave.next_eval {
            return;
        }
        self.wave.next_eval = t + 0.3;
        let h = me.stats.height.max(1.0);
        for _ in 0..3 {
            if self.spent_ms() > 6.0 {
                break;
            }
            self.wave.k = self.wave.k.wrapping_add(1);
            let (cx, cy, rr) = if self.wave.k % 2 == 0 { (me.pos[0], me.pos[1], 120.0) } else { (z.next_center[0], z.next_center[1], keep_r) };
            let a = self.rng.f64() * std::f64::consts::TAU;
            let r = rr * self.rng.f64().sqrt();
            let x = cx + r * a.cos();
            let y = cy + r * a.sin();
            if (x - z.next_center[0]).hypot(y - z.next_center[1]) > keep_r {
                continue;
            }
            let q0 = [x, y, fin(world.terrain_z(x, y), me.pos[2])];
            let Some(q) = nav.snap(q0) else { continue };
            if dist(q, q0) > 4.0 || !q.iter().all(|v| v.is_finite()) || q[2] > q0[2] + 2.0 {
                continue;
            }
            let ex = if self.p.prep_open > 0.0 { 1.0 - self.sightlines(world, q, h) } else { self.exposure(world, q, h) };
            let dz = dist2v(q, z.next_center) / z.next_radius.max(1.0);
            let s = ex + 0.15 * dz + 0.0015 * dist(q, me.pos);
            if self.wave.prep.map_or(true, |(_, ps)| s < ps - 0.03) {
                self.wave.prep = Some((q, s));
            }
        }
    }
}
