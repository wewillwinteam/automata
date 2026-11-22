use super::math::*;
use super::memory::Track;
use super::strategy::ZoneInfo;
use super::{Brain, Mode};
use crate::proto::{ActorKind, Obs, WeaponKind};

#[derive(Default, Clone, Debug)]
pub(crate) struct Gear {
    pub sweep_on: bool,
    pub sweep_acc: f64,
    pub sweep_yaw: f64,
    pub last_sweep: f64,
    pub laser_goal: bool,
    pub jet_scan: bool,
    pub pursue: (i64, f64),
}

impl Brain {
    pub(crate) fn gear_on(&self) -> bool {
        self.p.gear_on >= 0.5
    }

    pub(crate) fn gear_harmless(&self, tr: &Track, t: f64, d: f64) -> bool {
        let p = &self.p;
        if tr.kind != ActorKind::Mech {
            return true;
        }
        if self.sf_not_harmless(tr) {
            return false;
        }
        if self.order.dmg(tr.id) >= p.gear_harmless_dmg {
            return false;
        }
        if tr.has(WeaponKind::Laser) && d < p.gear_laser_threat_r {
            return false;
        }
        if t >= p.gear_late_s {
            return false;
        }
        true
    }

    pub(crate) fn gear_near_real(&self, obs: &Obs) -> f64 {
        let t = obs.t;
        let mut near = f64::INFINITY;
        for tr in &self.mem.tracks {
            if tr.kind != ActorKind::Mech || t - tr.last_seen > 1.5 {
                continue;
            }
            let d = dist(tr.pos_at(t), obs.me.pos);
            if !self.gear_harmless(tr, t, d) {
                near = near.min(d);
            }
        }
        near
    }

    pub(crate) fn gear_tgt_harmless(&self, obs: &Obs, tgt: Option<&Track>) -> bool {
        match tgt {
            None => true,
            Some(tr) => {
                let d = dist(tr.pos_at(obs.t), obs.me.pos);
                self.gear_harmless(tr, obs.t, d)
            }
        }
    }

    fn has_kind(obs: &Obs, k: WeaponKind) -> bool {
        obs.me.weapons.iter().flatten().any(|w| w.kind == k)
    }

    pub(crate) fn gear_goal(&mut self, obs: &Obs, zi: &ZoneInfo, tgt: Option<&Track>) -> Option<(i64, V3)> {
        self.gear.laser_goal = false;
        if !self.gear_on() || zi.urgent {
            return None;
        }
        let p = self.p.clone();
        let t = obs.t;
        let me = &obs.me;
        if !self.gear_tgt_harmless(obs, tgt) {
            return None;
        }
        let real = self.gear_near_real(obs);
        let tgt_d = tgt.map_or(f64::INFINITY, |tr| dist(tr.pos_at(t), me.pos));
        let mut best: Option<(f64, i64, V3, bool)> = None;
        if !Self::has_kind(obs, WeaponKind::Laser) && real > p.gear_safe_r {
            for it in &self.mem.pickups {
                if it.kind != WeaponKind::Laser || self.mem.blacklisted(it.id, t) || self.want(me, it.kind).is_none() {
                    continue;
                }
                let d = dist(it.pos, me.pos);
                if self.sf_far_block(d) {
                    continue;
                }
                if d > p.gear_laser_max || !self.in_play_pub(obs, zi, it.pos, d) {
                    continue;
                }
                let s = d - 400.0 - if it.id == self.goal_id { 15.0 } else { 0.0 };
                if best.map_or(true, |b| s < b.0) {
                    best = Some((s, it.id, it.pos, true));
                }
            }
        }
        if best.is_none() && me.weapons.iter().any(|w| w.is_none()) && real > p.gear_safe_r && tgt_d > p.gear_fight_keep {
            for it in &self.mem.pickups {
                if self.mem.blacklisted(it.id, t) || self.want(me, it.kind).is_none() {
                    continue;
                }
                let d = dist(it.pos, me.pos);
                if d > p.gear_weapon_max || !self.in_play_pub(obs, zi, it.pos, d) {
                    continue;
                }
                let s = d - if it.id == self.goal_id { 15.0 } else { 0.0 };
                if best.map_or(true, |b| s < b.0) {
                    best = Some((s, it.id, it.pos, false));
                }
            }
        }
        let cap = p.gear_cube_cap;
        let wants_cube = self.gear_wants_cube(me.cubes, cap);
        if best.is_none() && wants_cube && real > p.gear_cube_safe {
            for it in &self.mem.cubes {
                if self.mem.blacklisted(it.id, t) {
                    continue;
                }
                let d = dist(it.pos, me.pos);
                let lim = if tgt_d < p.gear_fight_keep { p.gear_cube_fight_r } else { p.gear_cube_r };
                if d > lim || !self.in_play_pub(obs, zi, it.pos, d) {
                    continue;
                }
                let s = d - if it.id == self.goal_id { 10.0 } else { 0.0 };
                if best.map_or(true, |b| s < b.0) {
                    best = Some((s, it.id, it.pos, false));
                }
            }
        }
        if best.is_none() && wants_cube && real > p.gear_cube_safe && p.gear_site_r > 0.0 && tgt_d > p.gear_fight_keep {
            for s in &self.econ.sites {
                let d = dist(s.0, me.pos);
                if t - s.1 < 1e8 && t - s.1 < p.gear_site_s && d < p.gear_site_r && d > 4.0 && self.in_play_pub(obs, zi, s.0, d) && best.map_or(true, |b| d < b.0) {
                    best = Some((d, -1, s.0, false));
                }
            }
        }
        if best.is_none() && wants_cube && p.gear_npc_goal_r > 0.0 && real > p.gear_safe_r && tgt_d > p.gear_fight_keep && frac(me.hp, me.max_hp) > 0.6 {
            for tr in &self.mem.tracks {
                if tr.kind != ActorKind::Npc || t - tr.last_seen > 20.0 || self.mem.blacklisted(tr.id, t) {
                    continue;
                }
                let d = dist(tr.pos, me.pos);
                if d > p.gear_npc_goal_r || !self.in_play_pub(obs, zi, tr.pos, d) || best.map_or(false, |b| d >= b.0) {
                    continue;
                }
                let k = if d > 1e-6 { (p.gear_npc_stand / d).min(1.0) } else { 0.0 };
                let q = [tr.pos[0] + (me.pos[0] - tr.pos[0]) * k, tr.pos[1] + (me.pos[1] - tr.pos[1]) * k, me.pos[2]];
                let q = if d <= p.gear_npc_stand + 8.0 { me.pos } else { q };
                best = Some((d, tr.id, q, false));
            }
        }
        if let Some(b) = best.filter(|b| b.1 >= 0 && !b.3 && p.gear_pursuit_s > 0.0) {
            if self.gear.pursue.0 != b.1 {
                self.gear.pursue = (b.1, t);
            } else if t - self.gear.pursue.1 > p.gear_pursuit_s {
                self.mem.ban(b.1, t + p.blacklist_s);
                self.gear.pursue = (-1, t);
                return None;
            }
        }
        if let Some(b) = best {
            self.gear.laser_goal = b.3;
            return Some((b.1, b.2));
        }
        None
    }

    pub(crate) fn gear_grab(&self, obs: &Obs, zi: &ZoneInfo) -> Option<(i64, V3)> {
        let p = &self.p;
        if !self.gear_on() || p.gear_grab_r <= 0.0 || zi.urgent {
            return None;
        }
        let t = obs.t;
        let me = &obs.me;
        let wants_cube = self.gear_wants_cube(me.cubes, p.gear_cube_cap);
        let mut best: Option<(f64, i64, V3)> = None;
        for it in &self.mem.pickups {
            if it.kind != WeaponKind::Laser || self.mem.blacklisted(it.id, t) || self.want(me, it.kind).is_none() || (it.pos[2] - me.pos[2]).abs() > 3.5 {
                continue;
            }
            let d = dist(it.pos, me.pos);
            if d < p.gear_grab_r && best.map_or(true, |b| d < b.0) {
                best = Some((d, it.id, it.pos));
            }
        }
        if best.is_none() && wants_cube {
            for it in &self.mem.cubes {
                if self.mem.blacklisted(it.id, t) || (it.pos[2] - me.pos[2]).abs() > 3.5 {
                    continue;
                }
                let d = dist(it.pos, me.pos);
                if d < p.gear_grab_r && best.map_or(true, |b| d < b.0) {
                    best = Some((d, it.id, it.pos));
                }
            }
        }
        best.map(|b| (b.1, b.2))
    }

    pub(crate) fn gear_npc_ok(&self, obs: &Obs, tr: &Track) -> bool {
        let p = &self.p;
        if !self.gear_on() || p.gear_npc_fire_r <= 0.0 || self.mode == Mode::Retreat {
            return false;
        }
        let me = &obs.me;
        let d = dist(tr.pos, me.pos);
        let wants_cube = self.gear_wants_cube(me.cubes, p.gear_cube_cap);
        wants_cube && d <= p.gear_npc_fire_r && frac(me.hp, me.max_hp) > 0.5
    }

    pub(crate) fn gear_drop_ok(&self, obs: &Obs, tgt: Option<&Track>) -> bool {
        self.gear_on() && self.p.gear_drop >= 0.5 && self.gear_tgt_harmless(obs, tgt) && self.gear_near_real(obs) > self.p.gear_safe_r
    }

    pub(crate) fn gear_farm_mode(&self, obs: &Obs, tgt: Option<&Track>) -> bool {
        if !self.gear_on() || self.p.gear_farm < 0.5 {
            return false;
        }
        let Some(tr) = tgt else { return false };
        let d = dist(tr.pos_at(obs.t), obs.me.pos);
        self.gear_harmless(tr, obs.t, d) && d > self.p.gear_farm_far && self.gear_near_real(obs) > self.p.gear_safe_r
    }

    pub(crate) fn gear_skip_fire(&self, tr: &Track, t: f64, d: f64, clear: bool) -> bool {
        if !self.gear_on() || self.p.gear_farm < 0.5 || !matches!(self.mode, Mode::Hunt | Mode::Loot) {
            return false;
        }
        if t - tr.last_dmg_t < 1.0 && d < self.p.gear_farm_far {
            return false;
        }
        self.gear_harmless(tr, t, d) && (!clear || d > self.p.gear_farm_far)
    }

    pub(crate) fn gear_range(&self, obs: &Obs, tr: &Track, pref: f64) -> f64 {
        if !self.gear_on() || self.p.gear_easy_range <= 0.0 {
            return pref;
        }
        let d = dist(tr.pos_at(obs.t), obs.me.pos);
        if self.gear_harmless(tr, obs.t, d) && !tr.has(WeaponKind::Laser) {
            pref.min(self.p.gear_easy_range)
        } else {
            pref
        }
    }

    pub(crate) fn gear_jet(&mut self, obs: &Obs) -> Option<(bool, bool)> {
        if !self.gear.sweep_on || !self.gear.jet_scan {
            return None;
        }
        let me = &obs.me;
        if me.energy < self.p.gear_jet_stop || self.sf_quiet_block() {
            self.gear.jet_scan = false;
            return None;
        }
        Some((me.grounded, !me.grounded))
    }

    pub(crate) fn gear_boost(&self, obs: &Obs) -> bool {
        let me = &obs.me;
        self.gear_on() && self.p.gear_boost_e > 0.0 && self.gear.laser_goal && self.mode == Mode::Loot && me.grounded && me.energy > self.p.gear_boost_e
    }

    pub(crate) fn gear_sweep(&mut self, obs: &Obs, busy: bool) -> Option<f64> {
        let p = self.p.clone();
        if !self.gear_on() || p.gear_sweep_every <= 0.0 {
            return None;
        }
        let t = obs.t;
        let yaw = obs.me.aim[0];
        let calm = !busy
            && !self.sf_quiet_block()
            && t - self.mem.last_damage_t > 3.0
            && !self.mem.tracks.iter().any(|tr| tr.kind == ActorKind::Mech && t - tr.last_seen < 2.0 && { let d = dist(tr.pos, obs.me.pos); d < p.gear_sweep_mech_r && (d < p.gear_sweep_any_r || !self.gear_harmless(tr, t, d)) })
            && matches!(self.mode, Mode::Idle | Mode::Loot | Mode::Hunt | Mode::Zone | Mode::Drop);
        if self.gear.sweep_on {
            if !calm {
                self.gear.sweep_on = false;
                return None;
            }
            self.gear.sweep_acc += adiff(yaw, self.gear.sweep_yaw).abs();
            self.gear.sweep_yaw = yaw;
            if self.gear.sweep_acc >= 2.0 * std::f64::consts::PI - 0.4 {
                self.gear.sweep_on = false;
                self.gear.last_sweep = t;
                return None;
            }
            return Some(wrap(yaw + 1.5));
        }
        let first = self.gear.last_sweep <= 0.0 && t < 3.0;
        let every = if p.gear_sweep_nolaser > 0.0 && !Self::has_kind(obs, WeaponKind::Laser) { p.gear_sweep_nolaser } else { p.gear_sweep_every };
        if calm && (first || t - self.gear.last_sweep > every) {
            self.gear.sweep_on = true;
            self.gear.sweep_acc = 0.0;
            self.gear.sweep_yaw = yaw;
            self.gear.last_sweep = t;
            self.gear.jet_scan = p.gear_jet_scan > 0.0 && obs.me.energy >= p.gear_jet_scan && (first || !Self::has_kind(obs, WeaponKind::Laser));
            return Some(wrap(yaw + 1.5));
        }
        None
    }
}

impl Brain {
    /// Gear goes for cubes below its cap, and from 13 to 15 for the next upgrade, unless a cube limit is set.
    pub(crate) fn gear_wants_cube(&self, cubes: f64, cap: f64) -> bool {
        if self.p.cube_avoid > 0.0 {
            return cubes < self.p.cube_avoid.min(cap);
        }
        cubes < cap || (cubes >= 13.0 && cubes < 15.0)
    }
}
