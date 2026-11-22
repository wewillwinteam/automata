use std::sync::Arc;

use super::math::*;
use super::memory::Track;
use super::Brain;
use crate::nav::{Field, Nav};
use crate::proto::{ActorKind, Obs};

#[derive(Clone, Debug)]
pub(crate) struct Rec {
    pub id: i64,
    pub vr: f64,
    pub hunter_until: f64,
    pub eta: f64,
}

pub(crate) struct Book {
    pub recs: Vec<Rec>,
    pub field: Option<(V3, f64, Arc<Field>)>,
    pub shooters: usize,
    pub hunters: Vec<(i64, f64)>,
}

impl Book {
    pub fn new() -> Book {
        Book { recs: Vec::new(), field: None, shooters: 0, hunters: Vec::new() }
    }
    pub fn rec(&self, id: i64) -> Option<&Rec> {
        self.recs.iter().find(|r| r.id == id)
    }
    pub fn is_hunter(&self, id: i64, t: f64) -> bool {
        self.rec(id).map_or(false, |r| r.hunter_until > t)
    }
}

impl Brain {
    pub(crate) fn sq_update(&mut self, obs: &Obs, nav: &Nav) {
        let p = self.p.clone();
        if p.sq_on < 0.5 {
            return;
        }
        let t = obs.t;
        let pos = obs.me.pos;
        let rebuild = match &self.sq.field {
            Some((q, tf, _)) => t - tf > 0.5 && dist(*q, pos) > 4.0 || t - tf > 2.0,
            None => true,
        };
        if rebuild && self.spent_ms() < 4.0 {
            let f = nav.field_limited(&[(pos, 0.0)], -1.0, p.sq_eta_range);
            self.sq.field = Some((pos, t, Arc::new(f)));
        }
        let walk = obs.me.stats.walk_speed.max(1.0);
        let mut shooters = 0;
        for tr in &self.mem.tracks {
            if tr.kind != ActorKind::Mech {
                continue;
            }
            if t - tr.last_dmg_t < p.sq_shooter_s {
                shooters += 1;
            }
            if self.sq.rec(tr.id).is_none() {
                self.sq.recs.push(Rec { id: tr.id, vr: 0.0, hunter_until: -1.0, eta: f64::INFINITY });
            }
            let tp = tr.pos_at(t);
            let d = dist(tp, pos);
            let fresh = t - tr.last_seen < 0.15;
            let u = unit2(pos[0] - tp[0], pos[1] - tp[1]);
            let vr_now = tr.vel[0] * u[0] + tr.vel[1] * u[1];
            let path = self.sq.field.as_ref().and_then(|(_, _, f)| nav.dist(f, tp)).filter(|x| x.is_finite()).unwrap_or(d * 1.3);
            let eta = if fresh && t - tr.last_seen < 0.15 && d < p.sq_los_range { 0.0 } else { (path - p.sq_los_range * 0.5).max(0.0) / walk };
            let aim_at_us = fresh && adiff(tr.aim[0], (pos[1] - tp[1]).atan2(pos[0] - tp[0])).abs() < p.sq_aim_tol && d < 220.0;
            let r = self.sq.recs.iter_mut().find(|r| r.id == tr.id).unwrap();
            if fresh {
                r.vr = r.vr * (1.0 - p.sq_vr_alpha) + vr_now * p.sq_vr_alpha;
            }
            r.eta = eta;
            let damaged = t - tr.last_dmg_t < p.sq_dmg_memory_s;
            if damaged || (fresh && d < p.sq_hunter_range && r.vr > p.sq_hunter_vr && d > 35.0) || (aim_at_us && tr.charging) {
                r.hunter_until = t + p.sq_hunter_hold_s;
            }
        }
        self.sq.recs.retain(|r| r.hunter_until > t - 60.0 || r.eta.is_finite());
        if self.sq.recs.len() > 32 {
            let cut = self.sq.recs.len() - 32;
            self.sq.recs.drain(0..cut);
        }
        self.sq.shooters = shooters;
        let mut hs: Vec<(i64, f64)> = Vec::new();
        for tr in &self.mem.tracks {
            if tr.kind != ActorKind::Mech || t - tr.last_seen > p.sq_track_s {
                continue;
            }
            if let Some(r) = self.sq.rec(tr.id) {
                if r.hunter_until > t {
                    hs.push((tr.id, r.eta));
                }
            }
        }
        hs.sort_by(|a, b| a.1.total_cmp(&b.1));
        hs.truncate(4);
        self.sq.hunters = hs;
    }

    pub(crate) fn sq_ttk(&self, obs: &Obs, tr: &Track) -> f64 {
        let _ = obs;
        tr.hp.max(1.0) / self.p.sq_our_dps.max(1.0)
    }

    fn sq_isolated(&self, obs: &Obs, tr: &Track) -> bool {
        let t = obs.t;
        let tp = tr.pos_at(t);
        !self.mem.tracks.iter().any(|o| o.kind == ActorKind::Mech && o.id != tr.id && t - o.last_seen < self.p.sq_track_s && dist(o.pos_at(t), tp) < self.p.sq_iso_range)
    }

    pub(crate) fn sq_allow(&self, obs: &Obs, tr: &Track) -> bool {
        let p = &self.p;
        if p.sq_on < 0.5 {
            return true;
        }
        let t = obs.t;
        if t - tr.last_dmg_t < 5.0 || self.sq.is_hunter(tr.id, t) {
            return true;
        }
        if t > p.sq_late_t {
            return true;
        }
        let hpf = frac(obs.me.hp, obs.me.max_hp);
        let ttk = self.sq_ttk(obs, tr);
        let first_h = self.sq.hunters.first().map_or(f64::INFINITY, |h| h.1);
        let weak = frac(tr.hp, tr.max_hp) < p.sq_ambush_tgt_hp;
        hpf >= p.sq_ambush_hp && (weak || ttk < p.sq_ambush_ttk) && self.sq_isolated(obs, tr) && first_h > ttk + p.sq_gap_margin
    }

    pub(crate) fn sq_gate(&self, obs: &Obs, tgt: Option<&Track>) -> bool {
        let p = &self.p;
        if p.sq_on < 0.5 || p.sq_gate_on < 0.5 {
            return false;
        }
        let t = obs.t;
        let hpf = frac(obs.me.hp, obs.me.max_hp);
        if hpf > p.sq_gate_max_hp {
            return false;
        }
        let ttk = tgt.map_or(0.0, |tr| self.sq_ttk(obs, tr));
        let mut etas: Vec<f64> = self.sq.hunters.iter().filter(|h| tgt.map_or(true, |tr| tr.id != h.0)).map(|h| h.1).collect();
        if let Some(tr) = tgt {
            if t - tr.last_dmg_t < 3.0 && tr.hp < obs.me.hp * p.sq_finish_ratio {
                return false;
            }
        }
        etas.sort_by(|a, b| a.total_cmp(b));
        let n_soon = etas.iter().filter(|e| **e < ttk + p.sq_gap_margin).count();
        self.sq.shooters >= 2 || n_soon >= 1 && tgt.is_some()
    }

    pub(crate) fn sq_regen(&self, obs: &Obs) -> bool {
        let p = &self.p;
        if p.sq_on < 0.5 || p.sq_regen_on < 0.5 || !obs.me.in_zone {
            return false;
        }
        let t = obs.t;
        let hpf = frac(obs.me.hp, obs.me.max_hp);
        let fighting = self.mem.tracks.iter().any(|tr| tr.kind == ActorKind::Mech && t - tr.last_dmg_t < p.sq_regen_fight_s);
        hpf < p.sq_regen_hp && !fighting && t < p.sq_regen_until_t
    }

    pub(crate) fn sq_avoid(&self, obs: &Obs) -> bool {
        let p = &self.p;
        if p.sq_on < 0.5 || p.sq_avoid_on < 0.5 || obs.t > p.sq_late_t {
            return false;
        }
        let t = obs.t;
        let pos = obs.me.pos;
        self.mem.tracks.iter().any(|tr| tr.kind == ActorKind::Mech && t - tr.last_seen < 1.0 && !self.sq.is_hunter(tr.id, t) && dist(tr.pos_at(t), pos) < p.sq_avoid_range)
    }
}
