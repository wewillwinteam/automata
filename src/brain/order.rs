use super::math::*;
use super::memory::Track;
use super::strategy::ZoneInfo;
use super::{Brain, Tgt};
use crate::proto::{ActorKind, Event, Obs};

#[derive(Clone, Copy, PartialEq, Eq, Debug)]
pub(crate) enum Role {
    Threat,
    Pest,
    Passive,
}

#[derive(Default)]
pub(crate) struct Order {
    hits: Vec<(i64, f64, f64)>,
    hostile: Vec<(i64, f64)>,
    came: Vec<(i64, f64)>,
    aim_since: Vec<(i64, f64, f64)>,
    pub halted: bool,
}

impl Order {
    pub fn update(&mut self, obs: &Obs) {
        let t = obs.t;
        let me = obs.me.pos;
        self.hits.retain(|h| t - h.1 < 10.0);
        for ev in &obs.events {
            if let Event::Damaged(d) = ev {
                if d.weapon == "zone" {
                    continue;
                }
                if let Some(by) = d.by {
                    self.hits.push((by, t, d.amount.max(0.0)));
                    self.mark(by, t);
                    Self::put(&mut self.came, by, t);
                }
            }
        }
        for a in &obs.mechs {
            if a.kind != ActorKind::Mech || a.id == obs.me.id || !a.pos.iter().all(|x| x.is_finite()) {
                continue;
            }
            let dx = me[0] - a.pos[0];
            let dy = me[1] - a.pos[1];
            let d = dx.hypot(dy);
            if d < 1e-6 {
                continue;
            }
            let vr = (a.vel[0] * dx + a.vel[1] * dy) / d;
            let aimd = adiff(a.aim[0], dy.atan2(dx)).abs();
            if (vr > 2.5 && d < 250.0) || (aimd < 0.2 && d < 230.0) {
                self.mark(a.id, t);
            }
            if aimd < 0.25 && vr > 0.5 && d < 250.0 {
                Self::put(&mut self.came, a.id, t);
            }
            if aimd < 0.25 && d < 150.0 {
                if !self.aim_since.iter().any(|x| x.0 == a.id && t - x.2 < 0.25) {
                    self.aim_since.retain(|x| x.0 != a.id);
                    self.aim_since.push((a.id, t, t));
                } else if let Some(x) = self.aim_since.iter_mut().find(|x| x.0 == a.id) {
                    x.2 = t;
                }
            }
        }
    }

    fn put(v: &mut Vec<(i64, f64)>, id: i64, t: f64) {
        if let Some(h) = v.iter_mut().find(|h| h.0 == id) {
            h.1 = t;
        } else {
            v.push((id, t));
        }
    }

    pub fn came_at(&self, id: i64, t: f64, hold_s: f64) -> bool {
        self.came.iter().any(|h| h.0 == id && t - h.1 < hold_s)
    }

    /// Distinct mechs that damaged us in the last 10 s or have held aim on us from within 150 m for 1 s.
    pub fn on_us(&self, t: f64) -> usize {
        let mut ids: Vec<i64> = self.hits.iter().map(|h| h.0).collect();
        for x in &self.aim_since {
            if t - x.2 < 0.25 && x.2 - x.1 >= 1.0 {
                ids.push(x.0);
            }
        }
        ids.sort_unstable();
        ids.dedup();
        ids.len()
    }

    fn mark(&mut self, id: i64, t: f64) {
        if let Some(h) = self.hostile.iter_mut().find(|h| h.0 == id) {
            h.1 = t;
        } else {
            self.hostile.push((id, t));
        }
    }

    pub fn dmg(&self, id: i64) -> f64 {
        self.hits.iter().filter(|h| h.0 == id).map(|h| h.2).sum()
    }

    pub fn role(&self, id: i64, t: f64, pest_dmg: f64, hold_s: f64) -> Role {
        if self.dmg(id) > pest_dmg {
            Role::Threat
        } else if self.hostile.iter().any(|h| h.0 == id && t - h.1 < hold_s) {
            Role::Pest
        } else {
            Role::Passive
        }
    }
}

impl Brain {
    pub(crate) fn order_on(&self) -> bool {
        self.p.order_mode >= 1.0
    }

    pub(crate) fn role_of(&self, id: i64, t: f64) -> Role {
        self.order.role(id, t, self.p.pest_dmg, self.p.order_hostile_s)
    }

    pub(crate) fn pest_ok(&self, tr: &Track, obs: &Obs) -> bool {
        let t = obs.t;
        if t >= self.p.order_late_s {
            return true;
        }
        let d = dist(tr.pos_at(t), obs.me.pos);
        if d < self.p.order_pest_close {
            return true;
        }
        if self.target == Tgt::Mech(tr.id) {
            return true;
        }
        self.p.order_spare < 0.5 && !self.order_has_passive
    }

    pub(crate) fn passive_ok(&self, tr: &Track, obs: &Obs) -> bool {
        let me = &obs.me;
        if frac(me.hp, me.max_hp) < self.p.order_abort_hp {
            return false;
        }
        if self.p.order_skip_laser >= 0.5 && tr.has(crate::proto::WeaponKind::Laser) && obs.t < self.p.order_late_s {
            return false;
        }
        if self.p.order_skip_cubes >= 0.5 && tr.cubes > me.cubes && obs.t < self.p.order_late_s {
            return false;
        }
        true
    }

    /// Whether fire control may shoot this mech when it is not the chosen target.
    pub(crate) fn order_fire_ok(&self, tr: &Track, obs: &Obs) -> bool {
        let t = obs.t;
        if !self.order_on() || self.target == Tgt::Mech(tr.id) {
            return true;
        }
        let role = self.role_of(tr.id, t);
        if self.p.order_mode >= 2.0 {
            return role == Role::Threat || self.order.came_at(tr.id, t, self.p.order_hostile_s);
        }
        match role {
            Role::Threat => true,
            Role::Pest => self.pest_ok(tr, obs),
            Role::Passive => self.passive_ok(tr, obs),
        }
    }

    pub(crate) fn choose_target_order(&mut self, obs: &Obs, zi: &ZoneInfo) -> Option<Track> {
        let p = self.p.clone();
        let t = obs.t;
        let pos = obs.me.pos;
        let z = &obs.zone;
        let live: Vec<(i64, V3)> = self.mem.tracks.iter().filter(|x| x.kind == ActorKind::Mech && t - x.last_seen < 3.0).map(|x| (x.id, x.pos_at(t))).collect();
        let mut best_threat: Option<(f64, &Track)> = None;
        let mut best_passive: Option<(f64, &Track)> = None;
        let mut best_pest: Option<(f64, &Track)> = None;
        for tr in &self.mem.tracks {
            if tr.kind != ActorKind::Mech {
                continue;
            }
            let role = self.order.role(tr.id, t, p.pest_dmg, p.order_hostile_s);
            if p.order_mode >= 2.0 && role != Role::Threat && !self.order.came_at(tr.id, t, p.order_hostile_s) {
                continue;
            }
            if role == Role::Passive && !self.passive_ok(tr, obs) {
                continue;
            }
            let mem_s = if role == Role::Passive { p.order_memory_s } else { p.target_memory_s };
            if t - tr.last_seen > mem_s {
                continue;
            }
            let tp = tr.pos_at(t);
            let d = dist(tp, pos);
            let attacker = t - tr.last_dmg_t < 5.0;
            if !attacker && self.mem.blacklisted(tr.id, t) {
                continue;
            }
            let reach = if role == Role::Passive { p.order_max_dist } else { p.engage_range };
            if d > reach && !attacker {
                continue;
            }
            if d > 60.0 {
                if dist2v(tp, zi.cur_c) > zi.cur_r - 2.0 {
                    continue;
                }
                if zi.closing && dist2v(tp, z.next_center) > z.next_radius + p.chase_out_next {
                    continue;
                }
            }
            let hpl = 1.0 - frac(tr.hp, tr.max_hp);
            let mut s = 60.0 * (1.0 - d / 200.0) + 80.0 * hpl;
            if self.target == Tgt::Mech(tr.id) {
                s += p.target_sticky;
            }
            if t - tr.last_seen < p.fresh_s {
                s += 30.0;
            }
            match role {
                Role::Threat => {
                    s += 100.0 + self.order.dmg(tr.id) * 0.1;
                    if best_threat.map_or(true, |b| s > b.0) {
                        best_threat = Some((s, tr));
                    }
                }
                Role::Passive => {
                    let to_us = (pos[1] - tp[1]).atan2(pos[0] - tp[0]);
                    s += 25.0 * (1.0 - adiff(tr.aim[0], to_us).cos());
                    let crowd = live.iter().filter(|(id, q)| *id != tr.id && dist(*q, tp) < 80.0).count();
                    if crowd == 0 {
                        s += 40.0;
                    }
                    if best_passive.map_or(true, |b| s > b.0) {
                        best_passive = Some((s, tr));
                    }
                }
                Role::Pest => {
                    if best_pest.map_or(true, |b| s > b.0) {
                        best_pest = Some((s, tr));
                    }
                }
            }
        }
        let has_passive = best_passive.is_some();
        let pick = if let Some(b) = best_threat {
            Some(b.1.clone())
        } else if p.order_mode >= 2.0 {
            best_pest.or(best_passive).map(|b| b.1.clone())
        } else {
            let late = t >= p.order_late_s;
            let close_pest = best_pest.filter(|b| dist(b.1.pos_at(t), pos) < p.order_pest_close);
            if let Some(b) = close_pest {
                Some(b.1.clone())
            } else if late {
                match (best_passive, best_pest) {
                    (Some(a), Some(b)) => Some(if a.0 >= b.0 { a.1.clone() } else { b.1.clone() }),
                    (Some(a), None) => Some(a.1.clone()),
                    (None, Some(b)) => Some(b.1.clone()),
                    _ => None,
                }
            } else if let Some(b) = best_passive {
                Some(b.1.clone())
            } else if p.order_spare < 0.5 {
                best_pest.map(|b| b.1.clone())
            } else {
                None
            }
        };
        self.order_has_passive = has_passive;
        pick
    }
}
