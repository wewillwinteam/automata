use super::aim::FireOut;
use super::math::*;
use super::memory::Track;
use super::{Brain, Mode, Tgt};
use super::duel::{self, Loadout};
use crate::proto::{ActorKind, Obs, WeaponKind};

impl Brain {
    /// Kill-order value (practice): its damage per second to us divided by the seconds we
    /// need to kill it. Killing the highest ratio first minimises the damage we take.
    pub(crate) fn ko_value(&self, obs: &Obs, tr: &Track) -> f64 {
        let p = &self.p;
        if p.ko_on <= 0.0 || !self.squad_mode() || self.ffa.on || tr.kind != ActorKind::Mech {
            return 0.0;
        }
        let me = &obs.me;
        let t = obs.t;
        let has = |k: WeaponKind| me.weapons.iter().flatten().any(|w| w.kind == k);
        let us = Loadout { gun: has(WeaponKind::Gun), laser: has(WeaponKind::Laser), lightning: has(WeaponKind::Lightning) };
        let them = Loadout { gun: tr.has(WeaponKind::Gun) || tr.weapons.is_empty(), laser: tr.has(WeaponKind::Laser), lightning: tr.has(WeaponKind::Lightning) };
        let tp = tr.pos_at(t);
        let d = dist(tp, me.pos).max(1.0);
        let cubes = me.cubes.min(self.cfg.cubes.stat_cap).max(0.0);
        let mut out = duel::outgoing(us, d) * (1.0 + self.cfg.cubes.damage_per_cube * cubes);
        let to_us = (me.pos[1] - tp[1]).atan2(me.pos[0] - tp[0]);
        if adiff(tr.aim[0], to_us).abs() > std::f64::consts::FRAC_PI_2 {
            out *= 1.5; // our shots land in its back
        }
        if us.laser && tr.has(WeaponKind::Mortar) {
            out *= 1.25; // laser cook-off on mortar carriers
        }
        if !(out > 1e-6) {
            return 0.0;
        }
        let ttk = (tr.hp.max(1.0) / out).max(0.5);
        let threat = duel::incoming(them, true, d) + if t - tr.last_dmg_t < 3.0 { 20.0 } else { 0.0 };
        (p.ko_w * threat / ttk).min(p.ko_cap)
    }

    /// Keep the current target while a laser charge is more than half full.
    pub(crate) fn ko_charging(&self, obs: &Obs) -> bool {
        self.p.ko_on > 0.0
            && obs.me.weapons.iter().flatten().any(|w| w.kind == WeaponKind::Laser && w.stats.charge > 0.0 && w.charge > 0.5 * w.stats.charge)
    }

    pub(crate) fn finish_bonus(&self, tr: &Track) -> f64 {
        let f = frac(tr.hp, tr.max_hp);
        if f < self.p.finish_hp {
            self.p.finish_bonus * (1.0 - f / self.p.finish_hp.max(0.01)).clamp(0.3, 1.0)
        } else {
            0.0
        }
    }

    pub(crate) fn rush_ok(&self, obs: &Obs, tr: &Track) -> bool {
        let t = obs.t;
        let tp = tr.pos_at(t);
        let mut others = 0;
        for o in &self.mem.tracks {
            if o.kind != ActorKind::Mech || o.id == tr.id || t - o.last_seen > 3.0 {
                continue;
            }
            if dist(o.pos_at(t), tp) < self.p.rush_alone_r || t - o.last_dmg_t < 2.5 {
                others += 1;
            }
        }
        if others > 0 {
            return false;
        }
        let need = (tr.hp.max(0.0) / 20.0).min(self.p.rush_energy_cap) * self.p.rush_energy_frac;
        obs.me.energy >= need
    }

    pub(crate) fn precharge(&self, obs: &Obs, out: &mut FireOut) {
        if self.p.precharge <= 0.0 || matches!(self.mode, Mode::Retreat | Mode::Zone) {
            return;
        }
        let me = &obs.me;
        let t = obs.t;
        let Some((i, w)) = me.weapons.iter().enumerate().find_map(|(i, w)| w.as_ref().filter(|w| w.kind == WeaponKind::Laser).map(|w| (i, w))) else { return };
        let full = w.stats.charge.max(0.05);
        let tid = match self.target {
            Tgt::Mech(id) => id,
            _ => -1,
        };
        let mut best: Option<(f64, V3)> = None;
        for tr in &self.mem.tracks {
            if tr.kind != ActorKind::Mech || t - tr.last_seen > self.p.precharge_memory_s {
                continue;
            }
            let tp = tr.pos_at(t.min(tr.last_seen + 1.0));
            let d = dist(tp, me.pos);
            if d > w.stats.range * 0.9 {
                continue;
            }
            let s = d - if tr.id == tid { 100.0 } else { 0.0 };
            if best.map_or(true, |b| s < b.0) {
                best = Some((s, tp));
            }
        }
        if w.charge > 0.0 {
            if w.charge < full + (w.stats.hold_s - 0.3).max(0.0) {
                out.fire[i] = true;
                out.charging = true;
            }
        } else if w.cooldown <= 0.1 && best.is_some() {
            out.fire[i] = true;
            out.charging = true;
        }
        if let Some((_, tp)) = best {
            let c = up(tp, self.cfg.mech.height * 0.5);
            let muzzle = up(me.pos, me.stats.muzzle_height);
            out.aim = dir_to_aim(sub(c, muzzle));
            out.armed = true;
        }
    }
}
