use std::collections::VecDeque;

use super::math::*;
use crate::proto::{ActorKind, Config, Event, Obs, SoundDist, TakeFailReason, WeaponKind};
use crate::world::World;

#[derive(Clone, Debug)]
pub struct Track {
    pub id: i64,
    pub kind: ActorKind,
    pub pos: V3,
    pub vel: V3,
    pub acc: V3,
    pub aim: V2,
    pub hp: f64,
    pub max_hp: f64,
    pub cubes: f64,
    pub weapons: Vec<WeaponKind>,
    pub charging: bool,
    pub charge_since: f64,
    pub last_seen: f64,
    pub dmg_to_us: f64,
    pub last_dmg_t: f64,
}

impl Track {
    pub fn has(&self, k: WeaponKind) -> bool {
        self.weapons.contains(&k)
    }
    pub fn pos_at(&self, t: f64) -> V3 {
        let dt = (t - self.last_seen).clamp(0.0, 1.0);
        add(self.pos, scale([self.vel[0], self.vel[1], 0.0], dt))
    }
}

#[derive(Clone, Debug)]
pub struct Item {
    pub id: i64,
    pub pos: V3,
    pub hp: f64,
    pub kind: WeaponKind,
    pub last_seen: f64,
    pub misses: u32,
}

#[derive(Clone, Debug)]
pub struct Heard {
    pub kind: String,
    pub bearing: f64,
    pub dist: SoundDist,
    pub t: f64,
}

#[derive(Clone, Debug)]
pub struct Unseen {
    pub bearing: f64,
    pub t: f64,
    pub from: V3,
    pub laser: bool,
}

pub struct Memory {
    pub tracks: Vec<Track>,
    pub crates: Vec<Item>,
    pub cubes: Vec<Item>,
    pub pickups: Vec<Item>,
    pub sounds: Vec<Heard>,
    pub last_damage_t: f64,
    pub last_dealt_t: f64,
    pub unseen: Option<Unseen>,
    pub hist: VecDeque<(f64, V3)>,
    pub blacklist: Vec<(i64, f64)>,
    pub last_t: f64,
    pub range_fails: Vec<(i64, u32)>,
    pub range_ban: f64,
}

fn should_see(obs: &Obs, world: &World, cfg: &Config, p: V3) -> bool {
    let me = &obs.me;
    let eye = up(me.pos, me.stats.eye_height);
    let d = dist(eye, p);
    let near = cfg.vision.near_range + me.stats.near_range_bonus - 3.0;
    let far = cfg.vision.range + me.stats.vision_bonus - 8.0;
    let in_range = if d <= near {
        true
    } else if d <= far {
        let yaw = (p[1] - eye[1]).atan2(p[0] - eye[0]);
        adiff(yaw, me.aim[0]).abs() < (cfg.vision.fov_deg * 0.5).to_radians() - 0.12
    } else {
        false
    };
    in_range && world.sight(eye, up(p, 1.0))
}

fn merge_items(list: &mut Vec<Item>, seen: &[(i64, V3, f64, WeaponKind)], obs: &Obs, world: &World, cfg: &Config, check_budget: &mut i32) {
    let t = obs.t;
    for &(id, pos, hp, kind) in seen {
        if let Some(it) = list.iter_mut().find(|i| i.id == id) {
            it.pos = pos;
            it.hp = hp;
            it.kind = kind;
            it.last_seen = t;
            it.misses = 0;
        } else {
            list.push(Item { id, pos, hp, kind, last_seen: t, misses: 0 });
        }
    }
    for it in list.iter_mut() {
        if it.last_seen < t && *check_budget > 0 {
            *check_budget -= 1;
            if should_see(obs, world, cfg, it.pos) {
                it.misses += 1;
            }
        }
    }
    list.retain(|i| i.misses < 2 && i.pos.iter().all(|x| x.is_finite()));
    if list.len() > 200 {
        let cut = list.len() - 200;
        list.drain(0..cut);
    }
}

impl Memory {
    pub fn new() -> Memory {
        Memory {
            tracks: Vec::new(),
            crates: Vec::new(),
            cubes: Vec::new(),
            pickups: Vec::new(),
            sounds: Vec::new(),
            last_damage_t: -1e9,
            last_dealt_t: -1e9,
            unseen: None,
            hist: VecDeque::new(),
            blacklist: Vec::new(),
            last_t: -1.0,
            range_fails: Vec::new(),
            range_ban: 0.0,
        }
    }

    pub fn blacklisted(&self, id: i64, t: f64) -> bool {
        self.blacklist.iter().any(|&(i, until)| i == id && until > t)
    }

    pub fn ban(&mut self, id: i64, until: f64) {
        self.blacklist.retain(|&(i, _)| i != id);
        self.blacklist.push((id, until));
        if self.blacklist.len() > 64 {
            self.blacklist.remove(0);
        }
    }

    pub fn update(&mut self, obs: &Obs, world: &World, cfg: &Config) {
        let t = obs.t;
        let me = &obs.me;
        let dt = if self.last_t >= 0.0 { (t - self.last_t).clamp(0.02, 2.0) } else { 0.1 };
        self.last_t = t;

        let decay = (-dt / 4.0f64).exp();
        for tr in self.tracks.iter_mut() {
            tr.dmg_to_us *= decay;
        }

        for a in &obs.mechs {
            if a.id == me.id || a.kind == ActorKind::Unknown || !a.pos.iter().all(|x| x.is_finite()) {
                continue;
            }
            let hp = a.hp.unwrap_or(if a.kind == ActorKind::Npc { cfg.npc.hp } else { cfg.mech.hp });
            let max_hp = a.max_hp.unwrap_or(if a.kind == ActorKind::Npc { cfg.npc.hp } else { cfg.mech.hp });
            if let Some(tr) = self.tracks.iter_mut().find(|x| x.id == a.id) {
                let dtr = t - tr.last_seen;
                if dtr > 1e-6 && dtr <= 0.25 {
                    let na = scale(sub(a.vel, tr.vel), 1.0 / dtr);
                    tr.acc = add(scale(tr.acc, 0.5), scale(na, 0.5));
                } else {
                    tr.acc = [0.0; 3];
                }
                if a.charging && !tr.charging {
                    tr.charge_since = t;
                }
                tr.kind = a.kind;
                tr.pos = a.pos;
                tr.vel = a.vel;
                tr.aim = a.aim;
                tr.hp = hp;
                tr.max_hp = max_hp;
                tr.cubes = a.cubes;
                tr.weapons = a.weapons.clone();
                tr.charging = a.charging;
                tr.last_seen = t;
            } else {
                self.tracks.push(Track {
                    id: a.id,
                    kind: a.kind,
                    pos: a.pos,
                    vel: a.vel,
                    acc: [0.0; 3],
                    aim: a.aim,
                    hp,
                    max_hp,
                    cubes: a.cubes,
                    weapons: a.weapons.clone(),
                    charging: a.charging,
                    charge_since: t,
                    last_seen: t,
                    dmg_to_us: 0.0,
                    last_dmg_t: -1e9,
                });
            }
        }

        for ev in &obs.events {
            match ev {
                Event::Damaged(d) => {
                    if d.weapon == "zone" || d.by.is_none() {
                        continue;
                    }
                    self.last_damage_t = t;
                    let by = d.by.unwrap_or(-1);
                    let mut fresh = false;
                    if let Some(tr) = self.tracks.iter_mut().find(|x| x.id == by) {
                        tr.dmg_to_us += d.amount.max(0.0);
                        tr.last_dmg_t = t;
                        fresh = t - tr.last_seen < 0.3;
                    }
                    if !fresh {
                        if let Some(b) = d.bearing {
                            if b.is_finite() {
                                self.unseen = Some(Unseen { bearing: b, t, from: me.pos, laser: d.weapon == "laser" });
                            }
                        }
                    }
                }
                Event::Hit(h) => {
                    self.last_dealt_t = t;
                    if let Some(c) = self.crates.iter_mut().find(|c| c.id == h.target) {
                        c.hp -= h.amount;
                    }
                }
                Event::Kill(k) => {
                    self.tracks.retain(|x| x.id != k.target);
                }
                Event::Took(k) => {
                    self.pickups.retain(|x| x.id != k.id);
                }
                Event::TakeFailed(f) => match f.reason {
                    TakeFailReason::Gone => self.pickups.retain(|x| x.id != f.id),
                    TakeFailReason::Range => {
                        if self.range_ban > 0.0 {
                            let n = match self.range_fails.iter_mut().find(|x| x.0 == f.id) {
                                Some(x) => {
                                    x.1 += 1;
                                    x.1
                                }
                                None => {
                                    self.range_fails.push((f.id, 1));
                                    if self.range_fails.len() > 32 {
                                        self.range_fails.remove(0);
                                    }
                                    1
                                }
                            };
                            if n as f64 >= self.range_ban {
                                self.range_fails.retain(|x| x.0 != f.id);
                                self.ban(f.id, t + 8.0);
                            }
                        }
                    }
                    _ => self.ban(f.id, t + 8.0),
                },
                _ => {}
            }
        }

        self.tracks.retain(|x| t - x.last_seen < 25.0 && !(x.kind == ActorKind::Npc && x.hp <= 0.0));
        self.tracks.sort_by_key(|x| x.id);

        let mut budget = 60i32;
        let crates: Vec<_> = obs.crates.iter().map(|c| (c.id, c.pos, c.hp.unwrap_or(cfg.crates.hp), WeaponKind::Unknown)).collect();
        merge_items(&mut self.crates, &crates, obs, world, cfg, &mut budget);
        let cubes: Vec<_> = obs.cubes.iter().map(|c| (c.id, c.pos, 0.0, WeaponKind::Unknown)).collect();
        merge_items(&mut self.cubes, &cubes, obs, world, cfg, &mut budget);
        let picks: Vec<_> = obs.pickups.iter().map(|c| (c.id, c.pos, 0.0, c.kind)).collect();
        merge_items(&mut self.pickups, &picks, obs, world, cfg, &mut budget);

        for s in &obs.sounds {
            if s.bearing.is_finite() {
                self.sounds.push(Heard { kind: s.kind.clone(), bearing: s.bearing, dist: s.dist, t: t - s.age.max(0.0) });
            }
        }
        self.sounds.retain(|s| t - s.t < 6.0);
        if self.sounds.len() > 64 {
            let cut = self.sounds.len() - 64;
            self.sounds.drain(0..cut);
        }
        if let Some(u) = &self.unseen {
            if t - u.t > 6.0 {
                self.unseen = None;
            }
        }
        self.blacklist.retain(|&(_, until)| until > t);

        self.hist.push_back((t, me.pos));
        while self.hist.front().map(|h| t - h.0 > 3.0).unwrap_or(false) {
            self.hist.pop_front();
        }
    }

    /// Displacement over the last w seconds, or None when history is shorter.
    pub fn moved(&self, t: f64, w: f64) -> Option<f64> {
        let first = self.hist.front()?;
        if t - first.0 < w * 0.9 {
            return None;
        }
        let old = self.hist.iter().find(|h| t - h.0 <= w)?;
        let cur = self.hist.back()?;
        Some(dist(old.1, cur.1))
    }
}
