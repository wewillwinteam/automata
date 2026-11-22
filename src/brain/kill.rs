use super::aim::FireOut;
use super::math::*;
use super::memory::Track;
use super::{Brain, Mode, Tgt};
use crate::proto::{ActorKind, Obs, WeaponKind};

impl Brain {
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
