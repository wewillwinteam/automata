use super::math::*;
use super::memory::Track;
use super::order::Role;
use super::Brain;
use crate::proto::{Action, ActorKind, Event, Obs, WeaponKind};
use crate::world::World;

#[derive(Default, Clone, Debug)]
pub(crate) struct Safety {
    pub dealt: Vec<(i64, f64)>,
    pub exposed: bool,
    pub threat: bool,
}

impl Brain {
    pub(crate) fn sf_update(&mut self, obs: &Obs, world: &World) {
        let p = self.p.clone();
        let t = obs.t;
        for ev in &obs.events {
            if let Event::Damaged(d) = ev {
                if let Some(by) = d.by {
                    if let Some(x) = self.sf.dealt.iter_mut().find(|x| x.0 == by) {
                        x.1 += d.amount.max(0.0);
                    } else {
                        self.sf.dealt.push((by, d.amount.max(0.0)));
                    }
                }
            }
        }
        self.sf.exposed = false;
        self.sf.threat = false;
        if p.sf_open_r <= 0.0 && p.sf_sweep < 0.5 {
            return;
        }
        let me = obs.me.pos;
        let eye = up(me, obs.me.stats.height * 0.6);
        let mut exposed = false;
        let mut threat = false;
        for tr in &self.mem.tracks {
            if tr.kind != ActorKind::Mech || t - tr.last_seen > p.sf_mem_s {
                continue;
            }
            let q = tr.pos_at(t);
            let d = dist(q, me);
            if d > p.sf_los_r.max(p.sf_sweep_r) || self.gear_harmless(tr, t, d) {
                continue;
            }
            if !world.sight(up(q, self.cfg.mech.eye_height), eye) {
                continue;
            }
            if d <= p.sf_los_r {
                exposed = true;
            }
            if d <= p.sf_sweep_r {
                threat = true;
            }
        }
        self.sf.exposed = exposed;
        self.sf.threat = threat;
    }

    pub(crate) fn sf_dealt(&self, id: i64) -> f64 {
        self.sf.dealt.iter().find(|x| x.0 == id).map_or(0.0, |x| x.1)
    }

    pub(crate) fn sf_not_harmless(&self, tr: &Track) -> bool {
        let p = &self.p;
        if p.sf_harm < 0.5 {
            return false;
        }
        tr.has(WeaponKind::Laser) || tr.cubes > p.sf_harm_cubes || self.sf_dealt(tr.id) >= p.sf_harm_dmg
    }

    fn sf_has_laser(obs: &Obs) -> bool {
        obs.me.weapons.iter().flatten().any(|w| w.kind == WeaponKind::Laser)
    }

    pub(crate) fn sf_hunter(&self, tr: &Track, t: f64) -> bool {
        t - tr.last_dmg_t < self.p.sf_hunt_s || self.order.role(tr.id, t, 0.0, self.p.sf_hunt_s) != Role::Passive
    }

    pub(crate) fn sf_hold(&self, obs: &Obs, tr: &Track) -> bool {
        let p = &self.p;
        if p.sf_nofirst < 0.5 || tr.kind != ActorKind::Mech || Self::sf_has_laser(obs) {
            return false;
        }
        let t = obs.t;
        if self.sf_hunter(tr, t) {
            return false;
        }
        let d = dist(tr.pos_at(t), obs.me.pos);
        tr.has(WeaponKind::Laser) || d > p.sf_nf_close
    }

    pub(crate) fn sf_no_mortar(&self) -> bool {
        self.p.sf_mortar >= 0.5
    }

    pub(crate) fn sf_drop(&self, obs: &Obs, act: &mut Action) {
        if self.p.sf_mortar < 1.5 || act.take.is_some() || act.drop.is_some() {
            return;
        }
        if let Some(i) = obs.me.weapons.iter().position(|w| w.as_ref().map_or(false, |w| w.kind == WeaponKind::Mortar)) {
            act.drop = Some(i as i64);
        }
    }

    pub(crate) fn sf_quiet_block(&self) -> bool {
        self.p.sf_sweep >= 0.5 && self.sf.threat
    }

    pub(crate) fn sf_far_block(&self, d: f64) -> bool {
        self.p.sf_open_r > 0.0 && d > self.p.sf_open_r && self.sf.exposed
    }
}
