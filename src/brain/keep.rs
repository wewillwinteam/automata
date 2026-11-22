use super::aim::FireOut;
use super::math::*;
use super::{Brain, Mode, Tgt};
use crate::proto::{ActorKind, Obs, WeaponKind};
use crate::world::World;

impl Brain {
    pub(crate) fn hold_lost_charge(&self, obs: &Obs, out: &mut FireOut) {
        let mem_s = self.p.lz_hold_lost;
        if mem_s <= 0.0 || matches!(self.mode, Mode::Retreat | Mode::Zone) {
            return;
        }
        let me = &obs.me;
        let t = obs.t;
        let Some((i, w)) = me.weapons.iter().enumerate().find_map(|(i, w)| w.as_ref().filter(|w| w.kind == WeaponKind::Laser).map(|w| (i, w))) else { return };
        if !(w.charge > 0.0) || i > 1 {
            return;
        }
        let full = w.stats.charge.max(0.05);
        if w.charge >= full + (w.stats.hold_s - 0.3).max(0.0) {
            return;
        }
        let tid = match self.target {
            Tgt::Mech(id) => id,
            _ => -1,
        };
        let mut best: Option<(f64, V3)> = None;
        for tr in &self.mem.tracks {
            if tr.kind != ActorKind::Mech || t - tr.last_seen > mem_s {
                continue;
            }
            let tp = tr.pos_at(t.min(tr.last_seen + 1.0));
            let d = dist(tp, me.pos);
            if !(d <= w.stats.range) {
                continue;
            }
            let s = (t - tr.last_seen) * 100.0 + d * 0.1 - if tr.id == tid { 50.0 } else { 0.0 };
            if best.map_or(true, |b| s < b.0) {
                best = Some((s, tp));
            }
        }
        let Some((_, tp)) = best else { return };        out.fire[i] = true;
        out.charging = true;
        out.armed = true;
        let muzzle = up(me.pos, me.stats.muzzle_height);
        let a = dir_to_aim(sub(up(tp, self.cfg.mech.height * 0.5), muzzle));
        if a[0].is_finite() && a[1].is_finite() {
            out.aim = a;
        }
    }
}

impl Brain {
    /// Fire control has no target this tick but our laser is charging: keep the charge on the mech it was
    /// started on (or the freshest other mech in range) instead of letting it go blind. In every mode.
    pub(crate) fn lz_hold_target(&self, obs: &Obs, world: &World, out: &mut FireOut) {
        let me = &obs.me;
        let t = obs.t;
        let Some((i, w)) = me.weapons.iter().enumerate().find_map(|(i, w)| w.as_ref().filter(|w| w.kind == WeaponKind::Laser).map(|w| (i, w))) else { return };
        if !(w.charge > 0.0) || i > 1 {
            return;
        }
        let full = w.stats.charge.max(0.05);
        if w.charge >= full + (w.stats.hold_s - 0.3).max(0.0) {
            return;
        }
        let mut pick = self.mem.tracks.iter().find(|tr| tr.id == self.lz_tid && t - tr.last_seen <= self.p.lz_mem_s);
        if pick.is_none() {
            let mut best: Option<(f64, &super::memory::Track)> = None;
            for tr in &self.mem.tracks {
                if tr.kind != ActorKind::Mech || t - tr.last_seen > 2.0 {
                    continue;
                }
                let d = dist(tr.pos_at(t), me.pos);
                if !(d <= w.stats.range) {
                    continue;
                }
                let s = (t - tr.last_seen) * 100.0 + d * 0.1;
                if best.map_or(true, |b| s < b.0) {
                    best = Some((s, tr));
                }
            }
            pick = best.map(|b| b.1);
        }
        let Some(tr) = pick else { return };
        let mut at = self.aim_target(tr);
        at.feet = tr.pos_at(t);
        let muzzle = up(me.pos, me.stats.muzzle_height);
        let c = up(at.feet, at.height * 0.5);
        let rel = sub(c, muzzle);
        let d = len(rel).max(0.1);
        let direct = dir_to_aim(add(rel, scale(at.vel, self.p.laser_latency)));
        if !(direct[0].is_finite() && direct[1].is_finite()) {
            return;
        }
        let st = &me.stats;
        let pred = slew(me.aim, direct, st.turn_rate * 0.05, st.pitch_rate * 0.05);
        let tol_y = (at.radius * self.p.aim_tol).atan2(d);
        let tol_p = (at.height * 0.5 * self.p.aim_tol).atan2(d);
        let aligned = adiff(pred[0], direct[0]).abs() * direct[1].cos().max(0.2) <= tol_y && (pred[1] - direct[1]).abs() <= tol_p;
        let seen = t - tr.last_seen <= 0.15;
        let open = seen && d <= w.stats.range * 0.98 && world.ray(muzzle, c).is_none();
        let ready = w.charge >= full - 0.051;
        // a clear, aligned, full shot is taken; anything else stays held
        let fire = !(ready && aligned && open);
        out.fire[i] = fire;
        out.charging = fire;
        out.armed = true;
        out.aim = direct;
        out.target = Some(at);
    }
}
