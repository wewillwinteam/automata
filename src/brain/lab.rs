//! Closed-loop scenarios for tests: the real Brain against scripted opponents.
//! Our mech moves with physics::step as in a match (two 0.05 s ticks per decision).
//! This checks mechanics (geometry, timing, rules); it does not predict match scores.

use std::collections::VecDeque;
use std::sync::Arc;

use super::math::*;
use super::{Brain, Params};
use crate::nav::Field;
use crate::physics::{self, Body, Drive};
use crate::proto::{ActorKind, ActorObs, Config, DamagedEv, DropEv, DropObs, Event, Hello, HitEv, KillEv, Obs, ProjKind, ProjectileObs, WeaponKind, WeaponObs, WeaponStats, ZoneObs};
use crate::runtime::Budget;
use crate::Shared;

pub(crate) const DT: f64 = 0.05;

#[derive(Clone, Debug)]
pub(crate) struct Wpn {
    pub kind: WeaponKind,
    pub stats: WeaponStats,
    pub cooldown: f64,
    pub ammo: f64,
    pub reload: f64,
    pub charge: f64,
}

impl Wpn {
    pub fn new(kind: WeaponKind, cfg: &Config) -> Wpn {
        let stats = cfg.weapons.get(kind).cloned().unwrap_or_default();
        Wpn { kind, ammo: stats.mag, stats, cooldown: 0.0, reload: 0.0, charge: 0.0 }
    }
}

#[derive(Clone, Debug)]
pub(crate) enum Script {
    /// Stands still and shoots with a linear lead.
    Gunner { latency: f64, cadence: f64 },
    /// Walks between an exposed point and a hidden one, shooting while exposed.
    Ducker { open: V3, hide: V3, out: bool, until: f64 },
    /// Walks to us and shoots with noisy aim (sg, sl: aim noise in radians for gun and laser).
    Hunter { sg: f64, sl: f64, pref: f64 },
    /// Wanders around a home point, in and out of sight, shooting when it sees us.
    Walker { home: V3, goal: V3, until: f64 },
}

pub(crate) struct Foe {
    pub id: i64,
    pub body: Body,
    pub aim: V2,
    pub hp: f64,
    pub alive: bool,
    pub script: Script,
    pub gun: Wpn,
    pub laser: Option<Wpn>,
    pub seen_until: f64,
    side: f64,
    side_until: f64,
    path: Option<Arc<Field>>,
}

struct Shot {
    id: i64,
    pos: V3,
    vel: V3,
    by: i64,
    left: f64,
    dmg: f64,
}

#[derive(Default, Clone, Debug)]
pub(crate) struct Stats {
    pub secs: f64,
    pub shots_at_us: u32,
    pub hits_on_us: u32,
    pub lasers_at_us: u32,
    pub laser_hits_on_us: u32,
    pub dmg_taken: f64,
    pub gun_shots: u32,
    pub gun_hits: u32,
    pub laser_shots: u32,
    pub laser_hits: u32,
    pub laser_held: u32,
    pub laser_partial: u32,
    pub laser_blind: u32,
    pub charge_s: f64,
    pub dmg_dealt: f64,
    pub kills: u32,
    pub dist_sum: f64,
    pub dist_n: u32,
    pub boost_s: f64,
    pub air_s: f64,
    pub died_at: Option<f64>,
    pub drops: u32,
    pub hash: u64,
}

impl Stats {
    pub fn add(&mut self, o: &Stats) {
        self.secs += o.secs;
        self.shots_at_us += o.shots_at_us;
        self.hits_on_us += o.hits_on_us;
        self.lasers_at_us += o.lasers_at_us;
        self.laser_hits_on_us += o.laser_hits_on_us;
        self.dmg_taken += o.dmg_taken;
        self.gun_shots += o.gun_shots;
        self.gun_hits += o.gun_hits;
        self.laser_shots += o.laser_shots;
        self.laser_hits += o.laser_hits;
        self.laser_held += o.laser_held;
        self.laser_partial += o.laser_partial;
        self.laser_blind += o.laser_blind;
        self.charge_s += o.charge_s;
        self.dmg_dealt += o.dmg_dealt;
        self.kills += o.kills;
        self.dist_sum += o.dist_sum;
        self.dist_n += o.dist_n;
        self.boost_s += o.boost_s;
        self.air_s += o.air_s;
        self.drops += o.drops;
        self.hash = self.hash.rotate_left(7) ^ o.hash;
    }
}

pub(crate) struct Lab {
    pub shared: Arc<Shared>,
    pub cfg: Config,
    pub brain: Brain,
    pub body: Body,
    pub aim: V2,
    pub hp: f64,
    pub max_hp: f64,
    /// false: our HP stays full (the bot keeps its fight behaviour for the whole run).
    pub mortal: bool,
    pub foes_mortal: bool,
    pub wpn: [Option<Wpn>; 2],
    pub foes: Vec<Foe>,
    pub rng: Rng,
    pub tick: i64,
    /// Statistics start after this time (skips the opening scan).
    pub warmup: f64,
    pub st: Stats,
    pub zone: ZoneObs,
    /// Supply drop: position, landing time, capture progress.
    pub drop: Option<(V3, f64, f64)>,
    shots: Vec<Shot>,
    hist: VecDeque<(V3, V3)>,
    events: Vec<Event>,
    last_hurt: f64,
    next_id: i64,
    chase: Option<(f64, Arc<Field>)>,
}

/// First parameter in [0, 1] where the segment a..b is inside a vertical cylinder standing at the origin.
fn seg_cyl(a: V3, b: V3, r: f64, h: f64) -> Option<f64> {
    let d = sub(b, a);
    let (mut t0, mut t1) = (0.0f64, 1.0f64);
    let aa = d[0] * d[0] + d[1] * d[1];
    if aa < 1e-12 {
        if a[0] * a[0] + a[1] * a[1] > r * r {
            return None;
        }
    } else {
        let bb = 2.0 * (a[0] * d[0] + a[1] * d[1]);
        let cc = a[0] * a[0] + a[1] * a[1] - r * r;
        let disc = bb * bb - 4.0 * aa * cc;
        if disc < 0.0 {
            return None;
        }
        let s = disc.sqrt();
        t0 = t0.max((-bb - s) / (2.0 * aa));
        t1 = t1.min((-bb + s) / (2.0 * aa));
    }
    if d[2].abs() < 1e-12 {
        if a[2] < 0.0 || a[2] > h {
            return None;
        }
    } else {
        let za = -a[2] / d[2];
        let zb = (h - a[2]) / d[2];
        t0 = t0.max(za.min(zb));
        t1 = t1.min(za.max(zb));
    }
    if t0 <= t1 {
        Some(t0)
    } else {
        None
    }
}

fn gauss(rng: &mut Rng) -> f64 {
    let u = rng.f64().max(1e-12);
    let v = rng.f64();
    (-2.0 * u.ln()).sqrt() * (2.0 * std::f64::consts::PI * v).cos()
}

fn fnv(h: u64, bytes: &[u8]) -> u64 {
    let mut h = h;
    for b in bytes {
        h ^= *b as u64;
        h = h.wrapping_mul(0x0000_0100_0000_01B3);
    }
    h
}

pub(crate) const FNV0: u64 = 0xcbf2_9ce4_8422_2325;

impl Lab {
    pub fn new(shared: Arc<Shared>, params: Params, seed: i64, start: V3, loadout: &[WeaponKind]) -> Lab {
        let mut hello = Hello::default();
        hello.seed = seed;
        let cfg = hello.config.clone();
        let mut brain = Brain::with_params(&hello, shared.clone(), params);
        brain.frozen = true;
        let mut wpn: [Option<Wpn>; 2] = [None, None];
        for (i, k) in loadout.iter().take(2).enumerate() {
            wpn[i] = Some(Wpn::new(*k, &cfg));
        }
        let zone = ZoneObs { center: [41.3, -72.8], radius: 2000.0, next_center: [41.3, -72.8], next_radius: 2000.0, shrink_start: 0.0, shrink_end: 0.0, dps: 20.0 };
        Lab {
            body: Body::new(start, cfg.mech.energy),
            aim: [0.0, 0.0],
            hp: cfg.mech.hp,
            max_hp: cfg.mech.hp,
            mortal: false,
            foes_mortal: false,
            wpn,
            foes: Vec::new(),
            rng: Rng::new(seed as u64 ^ 0x51ED),
            tick: 0,
            warmup: 6.0,
            st: Stats { hash: FNV0, ..Default::default() },
            zone,
            drop: None,
            shots: Vec::new(),
            hist: VecDeque::new(),
            events: Vec::new(),
            last_hurt: -1e9,
            next_id: 300,
            chase: None,
            shared,
            cfg,
            brain,
        }
    }

    pub fn add_foe(&mut self, pos: V3, script: Script, laser: bool) -> i64 {
        let id = self.foes.len() as i64 + 1;
        let to_us = (self.body.pos[1] - pos[1]).atan2(self.body.pos[0] - pos[0]);
        self.foes.push(Foe {
            id,
            body: Body::new(pos, self.cfg.mech.energy),
            aim: [to_us, 0.0],
            hp: self.cfg.mech.hp,
            alive: true,
            script,
            gun: Wpn::new(WeaponKind::Gun, &self.cfg),
            laser: if laser { Some(Wpn::new(WeaponKind::Laser, &self.cfg)) } else { None },
            seen_until: -1.0,
            side: 1.0,
            side_until: 0.0,
            path: None,
        });
        id
    }

    pub fn t(&self) -> f64 {
        self.tick as f64 * DT
    }

    fn live(&self) -> bool {
        self.t() >= self.warmup
    }

    fn sees(&self, eye: V3, yaw: f64, p: V3) -> bool {
        let v = &self.cfg.vision;
        let d = dist(eye, p);
        if d <= v.near_range {
            return self.shared.world.sight_near(eye, p);
        }
        if d > v.range {
            return false;
        }
        let b = (p[1] - eye[1]).atan2(p[0] - eye[0]);
        adiff(b, yaw).abs() <= v.fov_deg.to_radians() * 0.5 && self.shared.world.sight(eye, p)
    }

    fn obs(&mut self) -> Obs {
        let t = self.t();
        let m = self.cfg.mech.clone();
        let mut o = Obs::default();
        o.tick = self.tick;
        o.t = t;
        o.alive = 1 + self.foes.iter().filter(|f| f.alive).count() as i64;
        o.me.id = 0;
        o.me.pos = self.body.pos;
        o.me.vel = self.body.vel;
        o.me.aim = self.aim;
        o.me.hp = self.hp;
        o.me.max_hp = self.max_hp;
        o.me.energy = self.body.energy;
        o.me.max_energy = m.energy;
        o.me.grounded = self.body.grounded;
        o.me.in_zone = true;
        o.me.stats = m.clone();
        for i in 0..2 {
            o.me.weapons[i] = self.wpn[i].as_ref().map(|w| WeaponObs { kind: w.kind, cooldown: w.cooldown, ammo: w.ammo, reload: w.reload, charge: w.charge, stats: w.stats.clone() });
        }
        let eye = up(self.body.pos, m.eye_height);
        let yaw = self.aim[0];
        let mem = self.cfg.vision.memory_s;
        for i in 0..self.foes.len() {
            if !self.foes[i].alive {
                continue;
            }
            let fp = self.foes[i].body.pos;
            if self.sees(eye, yaw, up(fp, m.height * 0.5)) || self.sees(eye, yaw, up(fp, m.height)) {
                self.foes[i].seen_until = t + mem;
            }
            let f = &self.foes[i];
            if t <= f.seen_until {
                let mut weapons = vec![WeaponKind::Gun];
                if f.laser.is_some() {
                    weapons.push(WeaponKind::Laser);
                }
                o.mechs.push(ActorObs { id: f.id, kind: ActorKind::Mech, pos: f.body.pos, vel: f.body.vel, aim: f.aim, hp: Some(f.hp), max_hp: Some(self.cfg.mech.hp), cubes: 0.0, weapons, charging: f.laser.as_ref().map_or(false, |l| l.charge > 0.0) });
            }
        }
        for s in &self.shots {
            if s.by != 0 && self.sees(eye, yaw, s.pos) {
                o.projectiles.push(ProjectileObs { id: s.id, kind: ProjKind::Gun, pos: s.pos, vel: s.vel });
            }
        }
        if let Some((q, lands, prog)) = self.drop {
            o.drops.push(DropObs { id: 20000, pos: q, radius: 8.0, lands_in: (lands - t).max(0.0), capture_s: 4.0, progress: prog, holder: if prog > 0.0 { Some(0) } else { None } });
        }
        o.events = std::mem::take(&mut self.events);
        o.zone = self.zone.clone();
        o
    }

    fn spawn_shot(&mut self, by: i64, pos: V3, aim: V2, st: &WeaponStats, noise: f64) {
        let s = st.spread_deg.to_radians();
        let yaw = aim[0] + s * gauss(&mut self.rng) + noise * gauss(&mut self.rng);
        let pitch = aim[1] + s * gauss(&mut self.rng) + 0.5 * noise * gauss(&mut self.rng);
        let m = &self.cfg.mech;
        let o = physics::shot_origin(&self.shared.world, pos, yaw, m.muzzle_height, m.radius + 0.5);
        self.next_id += 1;
        self.shots.push(Shot { id: self.next_id, pos: o, vel: scale(aim_dir(yaw, pitch), st.speed), by, left: st.range, dmg: st.damage });
    }

    fn hurt_us(&mut self, by: i64, from: V3, amount: f64, weapon: &str) {
        let t = self.t();
        let bearing = (from[1] - self.body.pos[1]).atan2(from[0] - self.body.pos[0]);
        let rear = adiff(bearing, self.aim[0]).abs() > self.cfg.mech.front_arc_deg.to_radians() * 0.5;
        let amount = amount * if rear { self.cfg.mech.rear_mult } else { 1.0 };
        self.events.push(Event::Damaged(DamagedEv { amount, weapon: weapon.into(), bearing: Some(bearing), by: Some(by), rear }));
        self.last_hurt = t;
        if self.live() {
            self.st.dmg_taken += amount;
        }
        if self.mortal {
            self.hp -= amount;
            if self.hp <= 0.0 && self.st.died_at.is_none() {
                self.st.died_at = Some(t);
            }
        }
    }

    fn hurt_foe(&mut self, i: usize, amount: f64, weapon: &str) {
        let t = self.t();
        let id = self.foes[i].id;
        self.events.push(Event::Hit(HitEv { target: id, amount, weapon: weapon.into(), ..Default::default() }));
        self.last_hurt = t;
        if self.live() {
            self.st.dmg_dealt += amount;
        }
        if self.foes_mortal {
            self.foes[i].hp -= amount;
            if self.foes[i].hp <= 0.0 {
                self.foes[i].alive = false;
                self.events.push(Event::Kill(KillEv { target: id }));
                self.st.kills += 1;
            }
        }
    }

    fn our_weapons(&mut self, fire: [bool; 2], reload: bool) {
        let m = self.cfg.mech.clone();
        for i in 0..2 {
            let Some(mut w) = self.wpn[i].take() else { continue };
            w.cooldown = (w.cooldown - DT).max(0.0);
            match w.kind {
                WeaponKind::Gun => {
                    if w.reload > 0.0 {
                        w.reload -= DT;
                        if w.reload <= 0.0 {
                            w.reload = 0.0;
                            w.ammo = w.stats.mag;
                        }
                    } else if w.ammo <= 0.0 || (reload && w.ammo < w.stats.mag) {
                        w.reload = w.stats.reload;
                    } else if fire[i] && w.cooldown <= 0.0 {
                        let (pos, aim) = (self.body.pos, self.aim);
                        self.spawn_shot(0, pos, aim, &w.stats, 0.0);
                        w.ammo -= 1.0;
                        w.cooldown = w.stats.cooldown;
                        if self.live() {
                            self.st.gun_shots += 1;
                        }
                    }
                }
                WeaponKind::Laser => {
                    let full = w.stats.charge.max(0.05);
                    let mut shoot = false;
                    if w.cooldown <= 0.0 {
                        if fire[i] {
                            w.charge += DT;
                            shoot = w.charge >= full + w.stats.hold_s;
                        } else if w.charge > 0.0 {
                            if w.charge >= w.stats.min_charge {
                                shoot = true;
                            } else {
                                w.charge += DT;
                            }
                        }
                    }
                    if w.charge > 0.0 && self.live() {
                        self.st.charge_s += DT;
                    }
                    if shoot {
                        let frac = w.stats.min_frac + (1.0 - w.stats.min_frac) * (w.charge / full).clamp(0.0, 1.0);
                        let o = physics::shot_origin(&self.shared.world, self.body.pos, self.aim[0], m.muzzle_height, m.radius + 0.5);
                        let e = add(o, scale(aim_dir(self.aim[0], self.aim[1]), w.stats.range));
                        let wall = self.shared.world.ray(o, e).unwrap_or(1.0);
                        let mut best: Option<(f64, usize)> = None;
                        let mut blind = true;
                        for (k, f) in self.foes.iter().enumerate() {
                            if !f.alive {
                                continue;
                            }
                            if self.shared.world.ray(o, up(f.body.pos, m.height * 0.5)).is_none() {
                                blind = false;
                            }
                            if let Some(tt) = seg_cyl(sub(o, f.body.pos), sub(e, f.body.pos), m.radius, m.height) {
                                if tt < wall && best.map_or(true, |b| tt < b.0) {
                                    best = Some((tt, k));
                                }
                            }
                        }
                        if self.live() {
                            self.st.laser_shots += 1;
                            if w.charge > full + 0.2 {
                                self.st.laser_held += 1;
                            }
                            if w.charge < full - 0.06 {
                                self.st.laser_partial += 1;
                            }
                            if blind {
                                self.st.laser_blind += 1;
                            }
                            if best.is_some() {
                                self.st.laser_hits += 1;
                            }
                        }
                        if let Some((_, k)) = best {
                            self.hurt_foe(k, w.stats.damage * frac, "laser");
                        }
                        w.cooldown = w.stats.cooldown;
                        w.charge = 0.0;
                    }
                }
                _ => {}
            }
            self.wpn[i] = Some(w);
        }
    }

    fn foe_tick(&mut self, i: usize) {
        let t = self.t();
        let m = self.cfg.mech.clone();
        let world = &self.shared.clone().world;
        let nav = &self.shared.clone().nav;
        let me = self.body.pos;
        let fpos = self.foes[i].body.pos;
        let muzzle = up(fpos, m.muzzle_height);
        let center = up(me, m.height * 0.5);
        let d = dist(muzzle, center);
        let los = world.ray(muzzle, center).is_none();
        let script = self.foes[i].script.clone();
        let (latency, cadence, noise_g, noise_l) = match script {
            Script::Gunner { latency, cadence } => (latency, cadence, 0.0, 0.0),
            Script::Ducker { .. } => (0.05, 0.24, 0.0, 0.0),
            Script::Hunter { sg, sl, .. } => (0.05, 0.24, sg, sl),
            Script::Walker { .. } => (0.05, 0.24, 0.0, 0.0),
        };
        let mut boost = false;
        // movement
        let mut mv = [0.0, 0.0];
        match script {
            Script::Gunner { .. } => {}
            Script::Ducker { open, hide, out, until } => {
                let goal = if out { open } else { hide };
                if dist2(fpos, goal) > 0.6 {
                    mv = unit2(goal[0] - fpos[0], goal[1] - fpos[1]);
                }
                if t >= until {
                    let stay = if out { self.rng.range(1.2, 2.6) } else { self.rng.range(1.6, 3.6) };
                    self.foes[i].script = Script::Ducker { open, hide, out: !out, until: t + stay };
                }
            }
            Script::Hunter { pref, .. } => {
                if t >= self.foes[i].side_until {
                    self.foes[i].side = -self.foes[i].side;
                    self.foes[i].side_until = t + self.rng.range(0.5, 1.4);
                }
                let u = unit2(me[0] - fpos[0], me[1] - fpos[1]);
                let s = self.foes[i].side;
                if !los || d > pref {
                    let fresh = self.chase.as_ref().map_or(false, |c| t - c.0 < 1.0);
                    if !fresh {
                        self.chase = Some((t, Arc::new(nav.field_limited(&[(me, 0.0)], -1.0, 900.0))));
                    }
                    if let Some((_, f)) = &self.chase {
                        mv = nav.step(f, fpos).map(|s| s.dir).unwrap_or(u);
                    }
                    if los && d < 210.0 {
                        // weave while closing in under fire
                        mv = unit2(mv[0] - 0.8 * u[1] * s, mv[1] + 0.8 * u[0] * s);
                    }
                    boost = d > pref + 40.0 && self.foes[i].body.energy > 25.0 && (t * 0.2 + self.foes[i].id as f64 * 0.37).fract() < 0.4;
                } else {
                    mv = [-u[1] * s, u[0] * s];
                }
            }
            Script::Walker { home, goal, until } => {
                if self.foes[i].path.is_none() || dist2(fpos, goal) < 3.0 || t >= until {
                    let mut g = goal;
                    for _ in 0..8 {
                        let an = self.rng.range(0.0, std::f64::consts::TAU);
                        let r = self.rng.range(15.0, 60.0);
                        let q0 = [home[0] + r * an.cos(), home[1] + r * an.sin(), home[2]];
                        if let Some(q) = nav.snap(q0) {
                            if dist(q, q0) < 3.0 && dist(q, fpos) > 12.0 {
                                g = q;
                                break;
                            }
                        }
                    }
                    self.foes[i].path = Some(Arc::new(nav.field_limited(&[(g, 0.0)], -1.0, 260.0)));
                    self.foes[i].script = Script::Walker { home, goal: g, until: t + 14.0 };
                }
                if let Some(f) = &self.foes[i].path {
                    mv = nav.step(f, fpos).map(|s| s.dir).unwrap_or([0.0, 0.0]);
                }
            }
        }
        let charging = self.foes[i].laser.as_ref().map_or(false, |l| l.charge > 0.0);
        let drive = Drive { mv, boost, jump: false, speed_mult: if charging { m.charge_move_mult } else { 1.0 } };
        physics::step(world, &m, &mut self.foes[i].body, &drive, DT);
        // aim with a linear lead on what it saw `latency` ago
        let k = ((latency / DT).round() as usize).min(self.hist.len().saturating_sub(1));
        let (pl, vl) = if self.hist.is_empty() { (me, [0.0; 3]) } else { self.hist[self.hist.len() - 1 - k] };
        let rel = sub(up(pl, m.height * 0.5), muzzle);
        let gun_speed = self.foes[i].gun.stats.speed.max(1.0);
        let want = lead(rel, [vl[0], vl[1], 0.0], [0.0; 3], gun_speed, latency, 0.0, 0.0).map(|x| dir_to_aim(x.0)).unwrap_or(dir_to_aim(rel));
        let direct = dir_to_aim(add(rel, scale([vl[0], vl[1], 0.0], latency)));
        let laser_final = self.foes[i].laser.as_ref().map_or(false, |l| l.charge >= l.stats.charge - 0.2 && l.charge > 0.0);
        let cmd = if laser_final { direct } else { want };
        self.foes[i].aim = physics::slew_aim(self.foes[i].aim, cmd, &m, 1.0, DT);
        let aim = self.foes[i].aim;
        let exposed = match self.foes[i].script {
            Script::Ducker { out, .. } => out,
            _ => true,
        };
        // gun
        let aligned = adiff(aim[0], want[0]).abs() <= (2.0f64).atan2(d) && (aim[1] - want[1]).abs() <= (3.5f64).atan2(d);
        let mut g = self.foes[i].gun.clone();
        g.cooldown = (g.cooldown - DT).max(0.0);
        if g.reload > 0.0 {
            g.reload -= DT;
            if g.reload <= 0.0 {
                g.reload = 0.0;
                g.ammo = g.stats.mag;
            }
        } else if g.ammo <= 0.0 {
            g.reload = g.stats.reload;
        } else if los && exposed && aligned && g.cooldown <= 0.0 && d < g.stats.range {
            let id = self.foes[i].id;
            self.spawn_shot(id, fpos, aim, &g.stats, noise_g);
            g.ammo -= 1.0;
            g.cooldown = g.stats.cooldown.max(cadence);
            if self.live() {
                self.st.shots_at_us += 1;
            }
        }
        self.foes[i].gun = g;
        // laser: charge with a line of sight, fire at full charge
        if let Some(mut l) = self.foes[i].laser.clone() {
            l.cooldown = (l.cooldown - DT).max(0.0);
            let full = l.stats.charge.max(0.05);
            if l.cooldown <= 0.0 && (l.charge > 0.0 || (los && exposed && d < l.stats.range)) {
                l.charge += DT;
                let dl = adiff(aim[0], direct[0]).abs() <= (2.0f64).atan2(d);
                if (l.charge >= full + 0.05 && los && dl) || l.charge >= full + l.stats.hold_s {
                    let yaw = aim[0] + noise_l * gauss(&mut self.rng);
                    let pitch = aim[1] + 0.3 * noise_l * gauss(&mut self.rng);
                    let o = physics::shot_origin(world, fpos, yaw, m.muzzle_height, m.radius + 0.5);
                    let e = add(o, scale(aim_dir(yaw, pitch), l.stats.range));
                    let wall = world.ray(o, e).unwrap_or(1.0);
                    let hit = seg_cyl(sub(o, me), sub(e, me), m.radius, m.height).map_or(false, |tt| tt < wall);
                    if self.live() {
                        self.st.lasers_at_us += 1;
                        if hit {
                            self.st.laser_hits_on_us += 1;
                        }
                    }
                    if hit {
                        let id = self.foes[i].id;
                        self.hurt_us(id, fpos, l.stats.damage, "laser");
                    }
                    l.cooldown = l.stats.cooldown;
                    l.charge = 0.0;
                }
            }
            self.foes[i].laser = Some(l);
        }
    }

    fn shots_tick(&mut self, me_prev: V3, foe_prev: &[V3]) {
        let m = self.cfg.mech.clone();
        let pad = self.cfg.projectile.gun_radius;
        let mut shots = std::mem::take(&mut self.shots);
        let mut keep = Vec::with_capacity(shots.len());
        for mut s in shots.drain(..) {
            let a = s.pos;
            let b = add(a, scale(s.vel, DT));
            let wall = self.shared.world.ray(a, b).unwrap_or(2.0);
            let mut gone = wall <= 1.0;
            if s.by != 0 {
                if let Some(tt) = seg_cyl(sub(a, me_prev), sub(b, self.body.pos), m.radius + pad, m.height) {
                    if tt < wall {
                        if self.live() {
                            self.st.hits_on_us += 1;
                        }
                        self.hurt_us(s.by, sub(a, scale(s.vel, 0.2)), s.dmg, "gun");
                        gone = true;
                    }
                }
            } else {
                for k in 0..self.foes.len() {
                    if !self.foes[k].alive {
                        continue;
                    }
                    if let Some(tt) = seg_cyl(sub(a, foe_prev[k]), sub(b, self.foes[k].body.pos), m.radius + pad, m.height) {
                        if tt < wall {
                            if self.live() {
                                self.st.gun_hits += 1;
                            }
                            self.hurt_foe(k, s.dmg, "gun");
                            gone = true;
                            break;
                        }
                    }
                }
            }
            s.pos = b;
            s.left -= len(s.vel) * DT;
            if !gone && s.left > 0.0 {
                keep.push(s);
            }
        }
        self.shots = keep;
    }

    /// One decision: observation, action, two ticks. Returns false when our mech is dead.
    pub fn decide(&mut self) -> bool {
        let o = self.obs();
        let a = self.brain.act(&o, &Budget::unlimited());
        self.st.hash = fnv(self.st.hash, serde_json::to_string(&a).unwrap_or_default().as_bytes());
        let m = self.cfg.mech.clone();
        let fire = a.fire.unwrap_or([false; 2]);
        let want = a.aim.unwrap_or(self.aim);
        for k in 0..2 {
            let t = self.t();
            self.aim = physics::slew_aim(self.aim, want, &m, 1.0, DT);
            self.our_weapons(fire, a.reload.unwrap_or(false) && k == 0);
            let charging = self.wpn.iter().flatten().any(|w| w.kind == WeaponKind::Laser && w.charge > 0.0);
            let drive = Drive { mv: a.move_.unwrap_or([0.0, 0.0]), boost: a.boost.unwrap_or(false), jump: a.jump.unwrap_or(false) && k == 0, speed_mult: if charging { m.charge_move_mult } else { 1.0 } };
            let me_prev = self.body.pos;
            let info = physics::step(&self.shared.world, &m, &mut self.body, &drive, DT);
            let foe_prev: Vec<V3> = self.foes.iter().map(|f| f.body.pos).collect();
            for i in 0..self.foes.len() {
                if self.foes[i].alive {
                    self.foe_tick(i);
                }
            }
            self.shots_tick(me_prev, &foe_prev);
            if self.mortal && t - self.last_hurt >= m.regen_delay && self.hp > 0.0 {
                self.hp = (self.hp + m.regen_per_s * DT).min(self.max_hp);
            }
            if let Some((q, lands, prog)) = self.drop {
                let inside = |p: V3| dist2(p, q) <= 8.0 && (p[2] - q[2]).abs() < 6.0;
                let others = self.foes.iter().any(|f| f.alive && inside(f.body.pos));
                let np = if t >= lands && inside(self.body.pos) && !others { prog + DT } else if others && inside(self.body.pos) { prog } else { 0.0 };
                if np >= 4.0 {
                    self.hp = self.max_hp;
                    self.body.energy = m.energy;
                    self.events.push(Event::Drop(DropEv { id: 20000, cubes: 3.0 }));
                    self.st.drops += 1;
                    self.drop = None;
                } else {
                    self.drop = Some((q, lands, np));
                }
            }
            self.hist.push_back((self.body.pos, self.body.vel));
            while self.hist.len() > 8 {
                self.hist.pop_front();
            }
            if self.live() {
                self.st.secs += DT;
                if info.boosted {
                    self.st.boost_s += DT;
                }
                if !self.body.grounded {
                    self.st.air_s += DT;
                }
            }
            self.tick += 1;
        }
        if self.live() {
            let near = self.foes.iter().filter(|f| f.alive).map(|f| dist(f.body.pos, self.body.pos)).fold(f64::INFINITY, f64::min);
            if near.is_finite() {
                self.st.dist_sum += near;
                self.st.dist_n += 1;
            }
        }
        self.st.died_at.is_none()
    }

    pub fn run(&mut self, secs: f64) -> Stats {
        while self.t() < secs {
            if !self.decide() {
                break;
            }
        }
        self.st.clone()
    }
}
