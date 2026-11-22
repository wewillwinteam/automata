use super::math::*;
use super::memory::Track;
use super::Brain;
use crate::proto::{ActorKind, Event, Obs};

#[derive(Clone, Debug, Default)]
pub(crate) struct Ffa {
    pub on: bool,
    pub on_t: f64,
    pub why: u8,
    alive0: i64,
    kills: i64,
    sig: u32,
    seen_hp: Vec<(i64, f64, f64)>,
    our_hits: Vec<(i64, f64)>,
    pub foreign: Vec<(i64, f64)>,
    beam_last: Vec<(i64, f64)>,
    pub seed: i64,
}

impl Ffa {
    pub fn new(seed: i64) -> Ffa {
        Ffa { seed, ..Default::default() }
    }

    pub fn foreign_t(&self, id: i64) -> f64 {
        self.foreign.iter().find(|x| x.0 == id).map_or(-1e9, |x| x.1)
    }

    fn mark(&mut self, id: i64, t: f64) {
        if let Some(x) = self.foreign.iter_mut().find(|x| x.0 == id) {
            x.1 = t;
        } else {
            self.foreign.push((id, t));
        }
    }

    fn set(&mut self, t: f64, why: u8) {
        if !self.on {
            self.on = true;
            self.on_t = t;
            self.why = why;
            if std::env::var_os("FFA_LOG").is_some() {
                eprintln!("FFA seed={} t={:.1} why={}", self.seed, t, why);
            }
        }
    }

    pub fn update(&mut self, obs: &Obs, tracks: &[Track], enabled: bool) {
        let t = obs.t;
        let me = &obs.me;
        if self.alive0 <= 0 {
            self.alive0 = obs.alive.max(1);
        }
        for ev in &obs.events {
            match ev {
                Event::Kill(_) => self.kills += 1,
                Event::Hit(h) => {
                    if let Some(x) = self.our_hits.iter_mut().find(|x| x.0 == h.target) {
                        x.1 = t;
                    } else {
                        self.our_hits.push((h.target, t));
                    }
                }
                _ => {}
            }
        }
        if self.our_hits.len() > 64 {
            self.our_hits.remove(0);
        }
        if !enabled {
            return;
        }
        let deaths = self.alive0 - obs.alive;
        if t < 75.0 && deaths >= self.kills + 2 {
            self.set(t, 1);
        }
        let is_npc = |id: i64| tracks.iter().any(|tr| tr.id == id && tr.kind == ActorKind::Npc) || obs.mechs.iter().any(|m| m.id == id && m.kind == ActorKind::Npc);
        for b in &obs.beams {
            if b.by == me.id || is_npc(b.by) {
                continue;
            }
            let Some(end) = b.path.last() else { continue };
            if !end.iter().all(|v| v.is_finite()) || dist(*end, me.pos) < 9.0 {
                continue;
            }
            let hit3 = obs.mechs.iter().find(|m| m.kind == ActorKind::Mech && m.id != b.by && m.id != me.id && dist(up(m.pos, 4.5), *end) < 5.5);
            if let Some(m) = hit3 {
                let last = self.beam_last.iter().find(|x| x.0 == b.by).map_or(-1e9, |x| x.1);
                if t - last > 1.0 {
                    self.beam_last.retain(|x| x.0 != b.by);
                    self.beam_last.push((b.by, t));
                    self.sig += 1;
                }
                let id = m.id;
                self.mark(id, t);
            }
        }
        let npcs: Vec<V3> = obs.mechs.iter().filter(|m| m.kind == ActorKind::Npc).map(|m| m.pos).collect();
        let zc = obs.zone.center;
        let zr = obs.zone.radius;
        for m in &obs.mechs {
            if m.kind != ActorKind::Mech || m.id == me.id {
                continue;
            }
            let Some(hp) = m.hp else { continue };
            let prev = self.seen_hp.iter().find(|x| x.0 == m.id).copied();
            if let Some((_, php, pt)) = prev {
                let drop = php - hp;
                let ours = self.our_hits.iter().any(|x| x.0 == m.id && t - x.1 < 1.2);
                let in_zone = dist2v(m.pos, zc) < zr - 2.0;
                let npc_near = npcs.iter().any(|q| dist(*q, m.pos) < 150.0);
                if t - pt < 0.6 && drop >= 45.0 && !ours && in_zone && !npc_near {
                    self.sig += 1;
                    self.mark(m.id, t);
                }
            }
            if let Some(x) = self.seen_hp.iter_mut().find(|x| x.0 == m.id) {
                x.1 = hp;
                x.2 = t;
            } else {
                self.seen_hp.push((m.id, hp, t));
            }
        }
        if self.seen_hp.len() > 32 {
            self.seen_hp.remove(0);
        }
        if self.sig >= 3 {
            self.set(t, 2);
        }
    }
}

impl Brain {
    pub(crate) fn ffa_passive(&self, obs: &Obs) -> bool {
        self.p.ffa_on > 0.5 && self.ffa.on && (obs.alive as f64) > self.p.ffa_fair_alive && obs.t < self.p.ffa_end_t
    }

    pub(crate) fn ffa_end_rival(&self, obs: &Obs, tr: &Track) -> bool {
        if self.p.ffa_end_bonus <= 0.0 || obs.t < self.p.ffa_end_t || obs.zone.next_radius > 30.0 {
            return false;
        }
        let c = obs.zone.next_center;
        let mine = dist2v(obs.me.pos, c) - obs.me.hp / 260.0;
        let theirs = dist2v(tr.pos_at(obs.t), c) - tr.hp / 260.0;
        theirs < mine
    }

    pub(crate) fn ffa_allow(&self, obs: &Obs, tr: &Track, attacker: bool, d: f64) -> bool {
        if !self.ffa_passive(obs) {
            return true;
        }
        let t = obs.t;
        let p = &self.p;
        let hpf = frac(obs.me.hp, obs.me.max_hp);
        if attacker && (d < p.ffa_defend_range || hpf > 0.5) {
            return true;
        }
        let others = self.mem.tracks.iter().any(|o| o.kind == ActorKind::Mech && o.id != tr.id && t - o.last_seen < 4.0 && dist(o.pos_at(t), obs.me.pos) < p.ffa_crowd_range);
        let tf = frac(tr.hp, tr.max_hp);
        if tf < p.ffa_sure_hp && hpf > p.ffa_min_hp && !others && d < 150.0 {
            return true;
        }
        if t - self.ffa.foreign_t(tr.id) < 3.0 && tf < p.ffa_vulture_hp && hpf > p.ffa_min_hp - 0.1 && d < 140.0 {
            return true;
        }
        false
    }

    pub(crate) fn ffa_fire_ok(&self, obs: &Obs, tr: &Track, attacker: bool, d: f64, clear: bool) -> bool {
        if attacker || self.target == super::Tgt::Mech(tr.id) || d < 60.0 {
            return true;
        }
        if !(self.ffa.on && self.p.ffa_on > 0.5) {
            return self.p.ffa_early_quiet_range <= 0.0 || obs.t >= 75.0 || d < self.p.ffa_early_quiet_range;
        }
        if !self.ffa_passive(obs) {
            return true;
        }
        let to_us = (obs.me.pos[1] - tr.pos[1]).atan2(obs.me.pos[0] - tr.pos[0]);
        clear && d < 110.0 && adiff(tr.aim[0], to_us).abs() < 0.4
    }

    pub(crate) fn ffa_avoid(&self, obs: &Obs) -> bool {
        if !self.ffa_passive(obs) || self.p.ffa_avoid_range <= 0.0 {
            return false;
        }
        let t = obs.t;
        self.mem.tracks.iter().any(|tr| tr.kind == ActorKind::Mech && t - tr.last_seen < 1.0 && dist(tr.pos_at(t), obs.me.pos) < self.p.ffa_avoid_range)
    }

    pub(crate) fn ffa_quiet(&self, obs: &Obs) -> f64 {
        if !self.ffa_passive(obs) || !obs.me.in_zone || obs.t - self.mem.last_damage_t < 4.0 {
            return 1.0;
        }
        let t = obs.t;
        if self.mem.tracks.iter().any(|tr| tr.kind == ActorKind::Mech && t - tr.last_seen < 1.0) {
            return 1.0;
        }
        let heard = self.mem.sounds.iter().any(|s| t - s.t < 4.0 && s.kind == "step");
        if !heard {
            return 1.0;
        }
        let walk = obs.me.stats.walk_speed.max(1.0);
        (self.cfg.sound.step_speed * 0.92 / walk).clamp(0.2, 1.0)
    }
}
