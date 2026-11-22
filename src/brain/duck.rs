//! Peek and duck (practice): while the gun is empty or reloading and the laser is not
//! ready, step into the nearest spot within duck_r where the whole body (centre and top
//! height, both sides of its width) is hidden from every attacker by buildings or terrain
//! (trees do not stop shots); the normal fight logic brings us back out once reloaded.

use super::math::*;
use super::Brain;
use crate::nav::Nav;
use crate::proto::{ActorKind, Obs, WeaponKind};
use crate::world::World;

impl Brain {
    pub(crate) fn duck_goal(&mut self, obs: &Obs, world: &World, nav: &Nav) -> Option<V3> {
        let p = &self.p;
        let t = obs.t;
        let me = &obs.me;
        if p.duck_on <= 0.0 || !self.squad_mode() || self.ffa.on {
            self.duck = None;
            return None;
        }
        let Some(g) = me.weapons.iter().flatten().find(|w| w.kind == WeaponKind::Gun) else {
            self.duck = None;
            return None;
        };
        let empty = g.reload > 0.0 || g.ammo <= 0.0;
        let low = p.duck_ammo > 0.0 && g.ammo <= p.duck_ammo;
        let laser_ready = me.weapons.iter().flatten().any(|w| w.kind == WeaponKind::Laser && w.cooldown < 0.3);
        if !(empty || low) || laser_ready {
            self.duck = None;
            return None;
        }
        let eyes: Vec<V3> = self
            .mem
            .tracks
            .iter()
            .filter(|tr| tr.kind == ActorKind::Mech && t - tr.last_seen < 2.0 && (t - tr.last_dmg_t < 5.0 || dist(tr.pos_at(t), me.pos) < 150.0))
            .map(|tr| up(tr.pos_at(t), self.cfg.mech.eye_height * (1.0 + self.cfg.cubes.size_per_cube * tr.cubes.max(0.0))))
            // duck_los: only the mechs that have a line to us right now
            .filter(|e| p.duck_los <= 0.0 || world.sight_no_trees(*e, up(me.pos, me.stats.height * 0.5)) || world.sight_no_trees(*e, up(me.pos, me.stats.height * 0.92)))
            .collect();
        if eyes.is_empty() {
            self.duck = None;
            return None;
        }
        let h = me.stats.height.max(1.0);
        let r = self.cfg.mech.radius * (1.0 + self.cfg.cubes.size_per_cube * me.cubes.max(0.0));
        let hidden = |q: V3| -> bool {
            eyes.iter().all(|e| {
                let s = unit2(-(q[1] - e[1]), q[0] - e[0]);
                let pts = [up(q, h * 0.5), up(q, h * 0.92), [q[0] + s[0] * r, q[1] + s[1] * r, q[2] + h * 0.5], [q[0] - s[0] * r, q[1] - s[1] * r, q[2] + h * 0.5]];
                pts.iter().all(|pt| !world.sight_no_trees(*e, *pt))
            })
        };
        if let Some((q, t0)) = self.duck {
            if t - t0 < 4.0 && hidden(q) {
                return Some(q);
            }
        }
        let pos = me.pos;
        if hidden(pos) {
            crate::runtime::log(&format!("DUCK t={:.1} already covered", t));
            self.duck = Some((pos, t));
            return Some(pos);
        }
        let mut best: Option<(f64, V3)> = None;
        'outer: for &rr in &[3.0, 5.0, 7.5, 10.0, 14.0, 18.0] {
            if rr > p.duck_r {
                break;
            }
            for k in 0..12 {
                if self.spent_ms() > 10.0 {
                    break 'outer;
                }
                let a = k as f64 * std::f64::consts::PI / 6.0 + rr * 0.07;
                let (x, y) = (pos[0] + rr * a.cos(), pos[1] + rr * a.sin());
                let q0 = [x, y, fin(world.floor_below(x, y, pos[2] + 3.0), pos[2])];
                let Some(q) = nav.snap(q0) else { continue };
                if dist(q, q0) > 4.0 || !q.iter().all(|v| v.is_finite()) || !hidden(q) {
                    continue;
                }
                let c = dist(q, pos);
                if best.map_or(true, |b| c < b.0) {
                    best = Some((c, q));
                }
            }
            if best.is_some() {
                break;
            }
        }
        match best {
            Some((c, q)) => {
                crate::runtime::log(&format!("DUCK t={:.1} spot {:.1} m away", t, c));
                self.duck = Some((q, t));
                Some(q)
            }
            None => {
                crate::runtime::log(&format!("DUCKMISS t={:.1}", t));
                self.duck = None;
                None
            }
        }
    }
}
