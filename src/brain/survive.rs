use super::math::*;
use super::memory::Track;
use super::{Brain, Goal, Mode};
use crate::nav::Nav;
use crate::proto::{ActorKind, Obs, WeaponKind};
use crate::world::World;

pub(crate) struct Foe {
    pub eye: V3,
    pub w: f64,
    pub laser: bool,
    pub aimed_charge: Option<f64>,
    pub target: bool,
    pub d: f64,
}

impl Brain {
    pub(crate) fn foes(&self, obs: &Obs, target: Option<i64>) -> Vec<Foe> {
        let t = obs.t;
        let pos = obs.me.pos;
        let p = &self.p;
        let mut v: Vec<Foe> = Vec::new();
        let model = p.thr_model > 0.0;
        let squad = model && self.squad_mode();
        for tr in &self.mem.tracks {
            if tr.kind != ActorKind::Mech || t - tr.last_seen > p.sv_memory_s {
                continue;
            }
            let age = (t - tr.last_seen).min(1.5);
            let tp = tr.pos_at(tr.last_seen + age);
            if !tp.iter().all(|x| x.is_finite()) {
                continue;
            }
            let s = 1.0 + self.cfg.cubes.size_per_cube * tr.cubes.max(0.0);
            let d = dist(tp, pos);
            let mut w = 0.0;
            let gun = tr.has(WeaponKind::Gun) || tr.weapons.is_empty();
            if gun && d < self.cfg.weapons.gun.range {
                w += if model { p.gun_dps * super::duel::gun_hit(squad, d) } else { p.gun_dps * (1.15 - d / 220.0).clamp(0.2, 1.0) };
            }
            let laser = tr.has(WeaponKind::Laser) && d < self.cfg.weapons.laser.range;
            if laser {
                w += if model { p.laser_dps * p.sv_laser_mult * super::duel::laser_hit(squad, d) / super::duel::LASER_REF } else { p.laser_dps * p.sv_laser_mult };
            }
            if tr.has(WeaponKind::Lightning) && d < self.cfg.weapons.lightning.range + 10.0 {
                w += p.lightning_dps;
            }
            if tr.has(WeaponKind::Mortar) && d > 20.0 && d < 235.0 {
                w += p.mortar_dps * 0.5;
            }
            if t - tr.last_dmg_t < 3.0 {
                w *= 1.3;
            }
            let eye = up(tp, self.cfg.mech.eye_height * s);
            let aimed_charge = aimed_laser(tr, t, eye, pos, self.cfg.weapons.laser.charge, p.fresh_s);
            v.push(Foe { eye, w, laser, aimed_charge, target: target == Some(tr.id), d });
        }
        v.sort_by(|a, b| a.d.total_cmp(&b.d));
        v.truncate(6);
        v
    }

    pub(crate) fn spot_goal(&mut self, obs: &Obs, world: &World, nav: &Nav, tgt: Option<&Track>, pref: f64, cur: Goal) -> Goal {
        let p = self.p.clone();
        let t = obs.t;
        let me = &obs.me;
        let pos = me.pos;
        if p.sv_spot <= 0.0 {
            return cur;
        }
        let foes = self.foes(obs, tgt.map(|x| x.id));
        let others = foes.iter().filter(|f| !f.target && f.w > 0.0).count();
        let laser_evade = foes.iter().any(|f| f.aimed_charge.is_some());
        let need = match self.mode {
            Mode::Retreat => false,
            _ => others >= 1 || p.sv_spot >= 2.0 || laser_evade,
        };
        if !need {
            self.spot = None;
            return cur;
        }
        if let Some((q, tc, stay)) = self.spot {
            if t - tc < p.sv_period_s && !laser_evade {
                return if stay { cur } else { Goal::Nav { pos: q, arrive: 1.2 } };
            }
        }
        let h = me.stats.height.max(1.0);
        let walk = me.stats.walk_speed.max(1.0);
        let boost = me.stats.boost_speed.max(walk);
        let muzzle_h = me.stats.muzzle_height;
        let zi_c = obs.zone.center;
        let zi_r = obs.zone.radius;
        let mut cands: Vec<(V3, f64)> = vec![(pos, 0.0)];
        let rings: [(f64, usize); 4] = [(6.0, 8), (12.0, 12), (19.0, 12), (27.0, 12)];
        for (k0, &(r, n)) in rings.iter().enumerate() {
            for k in 0..n {
                let a = k as f64 * 2.0 * std::f64::consts::PI / n as f64 + k0 as f64 * 0.37;
                let x = pos[0] + r * a.cos();
                let y = pos[1] + r * a.sin();
                let q0 = [x, y, fin(world.floor_below(x, y, pos[2] + 3.0), pos[2])];
                let Some(q) = nav.snap(q0) else { continue };
                if dist(q, q0) > 4.0 || !q.iter().all(|v| v.is_finite()) || (q[2] - pos[2]).abs() > 6.0 {
                    continue;
                }
                if zi_r > 0.0 && dist2v(q, zi_c) > zi_r - 10.0 {
                    continue;
                }
                cands.push((q, dist2(q, pos)));
            }
        }
        if let Some((q, _, false)) = self.spot {
            if dist2(q, pos) < 30.0 {
                cands.push((q, dist2(q, pos)));
            }
        }
        let charge_left = foes.iter().filter_map(|f| f.aimed_charge).fold(f64::INFINITY, f64::min);
        let mut scored: Vec<(f64, V3, bool, f64)> = Vec::with_capacity(cands.len());
        for (i, &(q, dq)) in cands.iter().enumerate() {
            if self.spent_ms() > p.sv_budget_ms {
                break;
            }
            let c = up(q, h * 0.5);
            let mid = up([(q[0] + pos[0]) * 0.5, (q[1] + pos[1]) * 0.5, (q[2] + pos[2]) * 0.5], h * 0.5);
            let mut expo = 0.0;
            let mut path = 0.0;
            let mut hidden_all = true;
            let mut laser_hit = false;
            for f in &foes {
                let shot = world.ray(f.eye, c).is_none();
                if shot {
                    hidden_all = false;
                    expo += f.w * if f.target { p.sv_target_expo } else { 1.0 };
                    if f.laser {
                        laser_hit = true;
                    }
                    if f.aimed_charge.is_some() {
                        laser_hit = true;
                    }
                }
                if i > 0 && dq > 3.0 && world.ray(f.eye, mid).is_none() {
                    path += f.w * if f.target { p.sv_target_expo } else { 1.0 };
                }
            }
            let mut s = -p.sv_w_cov * expo / 100.0 - p.sv_w_path * path / 100.0 * (dq / walk).min(3.0) / 3.0;
            if hidden_all {
                s += p.sv_w_hidden;
            }
            if laser_hit {
                s -= p.sv_w_laser;
            }
            if let Some(tr) = tgt {
                let tc = up(tr.pos_at(t), self.cfg.mech.height * 0.5);
                let d = dist(q, tc);
                if world.ray(up(q, muzzle_h), tc).is_none() {
                    s += p.sv_w_shot - p.sv_w_range * ((d - pref).abs() - p.range_band).max(0.0);
                }
            }
            if charge_left.is_finite() && !laser_hit {
                if dq / boost <= charge_left + 0.1 {
                    s += p.sv_w_evade;
                }
            }
            s -= p.sv_w_travel * dq;
            if let Some((sq, _, _)) = self.spot {
                if dist2(sq, q) < 3.0 {
                    s += p.sv_hyst;
                }
            }
            scored.push((s, q, hidden_all, dq));
        }
        if scored.is_empty() {
            return cur;
        }
        let n = scored.len();
        for i in 0..n {
            let q = scored[i].1;
            let near = (0..n).any(|j| j != i && scored[j].2 && dist2(scored[j].1, q) < 6.5);
            if near || scored[i].2 {
                scored[i].0 += p.sv_w_cover;
            }
        }
        let mut best = 0usize;
        for i in 1..n {
            if scored[i].0 > scored[best].0 {
                best = i;
            }
        }
        let (bs, q, _, dq) = scored[best];
        let stay = best == 0 || dq < 2.5 || bs < scored[0].0 + p.sv_min_gain;
        self.spot = Some((q, t, stay));
        if stay {
            cur
        } else {
            Goal::Nav { pos: q, arrive: 1.2 }
        }
    }
}

impl Brain {
    pub(crate) fn threat_aim(&self, obs: &Obs) -> Option<V2> {
        if self.p.sv_torso <= 0.0 {
            return None;
        }
        let pos = obs.me.pos;
        let foes = self.foes(obs, None);
        let mut best: Option<(f64, V3)> = None;
        for f in &foes {
            if f.d > 220.0 || f.w <= 0.0 {
                continue;
            }
            let s = f.w / (1.0 + f.d / 80.0);
            if best.map_or(true, |b| s > b.0) {
                best = Some((s, f.eye));
            }
        }
        best.map(|(_, e)| [(e[1] - pos[1]).atan2(e[0] - pos[0]), 0.0])
    }

    pub(crate) fn hold_fire(&self, obs: &Obs, d: f64, hp: f64) -> bool {
        let p = &self.p;
        if p.sv_hold <= 0.0 || self.mode != Mode::Retreat {
            return false;
        }
        let lethal = hp > 0.0 && hp < p.sv_hold_lethal;
        let hit_recent = obs.t - self.mem.last_damage_t < p.sv_hold_hit_s;
        d > p.sv_hold_range && !lethal && !hit_recent
    }
}

fn aimed_laser(tr: &Track, t: f64, eye: V3, pos: V3, charge: f64, fresh: f64) -> Option<f64> {
    if !tr.charging || t - tr.last_seen > fresh.max(0.3) || !tr.has(WeaponKind::Laser) {
        return None;
    }
    let d = dist2(eye, pos).max(1.0);
    let to_us = (pos[1] - eye[1]).atan2(pos[0] - eye[0]);
    if adiff(tr.aim[0], to_us).abs() > (4.5f64).atan2(d) + 0.08 {
        return None;
    }
    Some((charge - (t - tr.charge_since)).clamp(0.1, 0.8))
}
