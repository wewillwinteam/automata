use super::math::*;
use super::strategy::ZoneInfo;
use super::Brain;
use crate::proto::{ActorKind, Event, Obs};

#[derive(Default, Clone, Debug)]
pub(crate) struct Econ {
    pub sites: Vec<(V3, f64)>,
    pub resting: bool,
}

impl Brain {
    pub(crate) fn econ_update(&mut self, obs: &Obs) {
        let t = obs.t;
        for ev in &obs.events {
            if let Event::Kill(k) = ev {
                if let Some(tr) = self.mem.tracks.iter().find(|tr| tr.kind == ActorKind::Mech && tr.id == k.target) {
                    self.econ.sites.push((tr.pos, t));
                }
            }
        }
        let keep = self.p.harvest_site_s;
        let me = obs.me.pos;
        self.econ.sites.retain(|s| t - s.1 < keep || t - s.1 > 1e8 && t - (s.1 + 1e9) < keep);
        for s in self.econ.sites.iter_mut() {
            if dist2(s.0, me) < 5.0 && t - s.1 < 1e8 {
                s.1 -= 1e9;
            }
        }
    }

    fn cube_wanted(&self, cubes: f64) -> bool {
        let cap = self.cfg.cubes.stat_cap;
        if self.p.cube_avoid > 0.0 {
            return cubes < self.p.cube_avoid.min(cap);
        }
        cubes < cap || (cubes >= 13.0 && cubes < 15.0)
    }

    pub(crate) fn harvest(&self, obs: &Obs, zi: &ZoneInfo, n_att: usize, near_mech: f64) -> Option<(i64, V3)> {
        let p = &self.p;
        if !p.harvest_on || !self.cube_wanted(obs.me.cubes) {
            return None;
        }
        let t = obs.t;
        let me = &obs.me;
        let calm = n_att == 0 && t - self.mem.last_damage_t > p.harvest_calm_s;
        let mut best: Option<(f64, i64, V3)> = None;
        for it in &self.mem.cubes {
            if self.mem.blacklisted(it.id, t) {
                continue;
            }
            let d = dist(it.pos, me.pos);
            let site = self.econ.sites.iter().any(|s| dist2(s.0, it.pos) < 18.0);
            let lim = if site && calm && near_mech > p.harvest_near { p.harvest_site_max } else if calm { p.harvest_calm_max } else { p.harvest_fight_max };
            if d > lim || !self.in_play_pub(obs, zi, it.pos, d) {
                continue;
            }
            let s = d - if it.id == self.goal_id { 10.0 } else { 0.0 };
            if best.map_or(true, |b| s < b.0) {
                best = Some((s, it.id, it.pos));
            }
        }
        if best.is_none() && calm && near_mech > p.harvest_near {
            for s in &self.econ.sites {
                let d = dist(s.0, me.pos);
                if t - s.1 < p.harvest_visit_s && d < p.harvest_site_max && d > 5.0 && self.in_play_pub(obs, zi, s.0, d) {
                    if best.map_or(true, |b| d < b.0) {
                        best = Some((d, -1, s.0));
                    }
                }
            }
        }
        best.map(|b| (b.1, b.2))
    }

    pub(crate) fn want_rest(&mut self, obs: &Obs, zi: &ZoneInfo, n_att: usize, near_mech: f64, hpf: f64) -> bool {
        let p = &self.p;
        let t = obs.t;
        let on = p.rest_on || (p.rest_after_k >= 1.0 && self.wave.kills as f64 >= p.rest_after_k && self.squad_mode());
        let ok = on && !zi.urgent && n_att == 0 && t - self.mem.last_damage_t > 1.0 && near_mech > p.rest_near;
        let r = if self.econ.resting { ok && hpf < p.rest_exit_hp } else { ok && hpf < p.rest_hp };
        self.econ.resting = r;
        r
    }
}
