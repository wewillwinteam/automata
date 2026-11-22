//! Closed-loop checks of the laser fire control against one scripted mech.

use super::math::*;
use super::*;
use crate::physics::{self, Body, Drive};
use crate::proto::{ActorKind, ActorObs, DamagedEv, Event, Hello, Obs, WeaponObs, ZoneObs};
use std::sync::{Arc, OnceLock};

const EID: i64 = 5;

#[derive(Clone, Copy)]
struct Foe {
    pos: V3,
    vel: V3,
    shown: bool,
    hp: f64,
}

struct Shot {
    t: f64,
    charge: f64,
    hit: bool,
}

struct Rig {
    shared: Arc<Shared>,
    cfg: crate::proto::Config,
    brain: Brain,
    body: Body,
    aim: V2,
    hp: f64,
    charge: f64,
    cd: f64,
    tick: i64,
    log: Vec<String>,
    shots: Vec<Shot>,
    starts: Vec<(f64, Mode)>,
    held: usize,
    /// keep our feet where they are (exact geometry for the wall cases)
    pin: bool,
}

fn fnv(lines: &[String]) -> u64 {
    let mut h: u64 = 0xcbf2_9ce4_8422_2325;
    for l in lines {
        for b in l.as_bytes().iter().chain(b"\n".iter()) {
            h ^= *b as u64;
            h = h.wrapping_mul(0x0000_0100_0000_01b3);
        }
    }
    h
}

fn prod_params() -> Params {
    serde_json::from_str::<Params>(include_str!("../../bot.json")).expect("bot.json")
}

fn flat() -> Arc<Shared> {
    static S: OnceLock<Arc<Shared>> = OnceLock::new();
    S.get_or_init(|| {
        let cfg = crate::proto::Config::default();
        Arc::new(Shared { world: crate::world::World::empty(), nav: crate::nav::Nav::build(&crate::world::World::empty(), &cfg.mech) })
    })
    .clone()
}

/// Flat ground with one 30 m high block spanning x 140..170, y -60..-10.
fn walled() -> Arc<Shared> {
    static S: OnceLock<Arc<Shared>> = OnceLock::new();
    S.get_or_init(|| {
        let dir = std::env::temp_dir().join(format!("lz_map_{}", std::process::id()));
        let v2 = dir.join("v2");
        std::fs::create_dir_all(&v2).expect("dir");
        let (nx, ny) = (401usize, 301usize);
        let json = format!(
            r#"{{"bounds":{{"minX":-100,"maxX":300,"minY":-150,"maxY":150}},"terrain":{{"nx":{nx},"ny":{ny},"minX":-100,"minY":-150,"step":1,"scale":0.01,"file":"terrain.i16"}},"rules":{{}},"prisms":[{{"kind":"building","base":0,"top":30,"ring":[140,-10,170,-10,170,-60,140,-60]}}],"trees":{{"fields":[],"data":[]}}}}"#
        );
        std::fs::write(v2.join("world.json"), json).expect("world.json");
        std::fs::write(v2.join("terrain.i16"), vec![0u8; nx * ny * 2]).expect("terrain");
        let cfg = crate::proto::Config::default();
        let world = crate::world::World::load(&dir).expect("load");
        assert_eq!(world.prisms.len(), 1);
        let nav = crate::nav::Nav::build(&world, &cfg.mech);
        let _ = std::fs::remove_dir_all(&dir);
        Arc::new(Shared { world, nav })
    })
    .clone()
}

impl Rig {
    fn new(shared: Arc<Shared>, params: Params, start: V3) -> Rig {
        let mut hello = Hello::default();
        hello.seed = 7;
        hello.slot = 1;
        let cfg = hello.config.clone();
        let brain = Brain::with_params(&hello, shared.clone(), params);
        let body = Body::new(start, cfg.mech.energy);
        Rig { shared, hp: cfg.mech.hp, cfg, brain, body, aim: [0.0, 0.0], charge: 0.0, cd: 0.0, tick: 0, log: Vec::new(), shots: Vec::new(), starts: Vec::new(), held: 0, pin: false }
    }

    fn t(&self) -> f64 {
        self.tick as f64 * 0.05
    }

    fn obs(&self, foe: Option<Foe>, events: Vec<Event>) -> Obs {
        let cfg = &self.cfg;
        let mut o = Obs::default();
        o.tick = self.tick;
        o.t = self.t();
        o.alive = 16;
        o.me.id = 1;
        o.me.pos = self.body.pos;
        o.me.vel = self.body.vel;
        o.me.aim = self.aim;
        o.me.hp = self.hp;
        o.me.max_hp = cfg.mech.hp;
        o.me.energy = self.body.energy;
        o.me.max_energy = cfg.mech.energy;
        o.me.grounded = self.body.grounded;
        o.me.in_zone = true;
        o.me.stats = cfg.mech.clone();
        o.me.weapons = [
            Some(WeaponObs { kind: WeaponKind::Gun, ammo: 12.0, stats: cfg.weapons.gun.clone(), ..Default::default() }),
            Some(WeaponObs { kind: WeaponKind::Laser, charge: self.charge, cooldown: self.cd.max(0.0), stats: cfg.weapons.laser.clone(), ..Default::default() }),
        ];
        if let Some(f) = foe.filter(|f| f.shown) {
            let yaw = (self.body.pos[1] - f.pos[1]).atan2(self.body.pos[0] - f.pos[0]);
            o.mechs.push(ActorObs { id: EID, kind: ActorKind::Mech, pos: f.pos, vel: f.vel, aim: [yaw, 0.0], hp: Some(f.hp), max_hp: Some(cfg.mech.hp), cubes: 0.0, weapons: vec![WeaponKind::Gun, WeaponKind::Laser], charging: false });
        }
        o.events = events;
        o.zone = ZoneObs { center: [41.3, -72.8], radius: 380.0, next_center: [50.0, -30.0], next_radius: 240.0, shrink_start: 105.0, shrink_end: 145.0, dps: 35.0 };
        o
    }

    /// One decision: the brain acts, then two ticks of torso, laser and legs.
    fn step(&mut self, foe: Option<Foe>, events: Vec<Event>) -> crate::proto::Action {
        let o = self.obs(foe, events);
        let a = self.brain.act(&o, &Budget::unlimited());
        self.log.push(serde_json::to_string(&a).expect("action"));
        let fire = a.fire.unwrap_or([false; 2])[1];
        if fire && self.charge > 0.0 {
            self.held += 1;
        }
        let want = a.aim.unwrap_or(self.aim);
        let ls = self.cfg.weapons.laser.clone();
        let world = &self.shared.world;
        for k in 0..2 {
            self.aim = physics::slew_aim(self.aim, want, &self.cfg.mech, 1.0, 0.05);
            let mut shoot = false;
            if self.charge > 0.0 {
                if fire || self.charge < ls.min_charge {
                    self.charge += 0.05;
                    if self.charge >= ls.charge + ls.hold_s {
                        shoot = true;
                    }
                } else {
                    shoot = true;
                }
            } else if fire && self.cd <= 0.0 {
                self.charge = 0.05;
                self.starts.push((self.t(), self.brain.mode));
            }
            if shoot {
                let mut hit = false;
                if let Some(f) = foe {
                    let m = physics::shot_origin(world, self.body.pos, self.aim[0], self.cfg.mech.muzzle_height, self.cfg.mech.radius + 0.5);
                    let c = up(f.pos, self.cfg.mech.height * 0.5);
                    let d = aim_dir(self.aim[0], self.aim[1]);
                    let rel = sub(c, m);
                    let along = dot(rel, d);
                    let off = sub(rel, scale(d, along));
                    hit = along > 0.0 && off[0].hypot(off[1]) <= self.cfg.mech.radius && off[2].abs() <= self.cfg.mech.height * 0.5 && world.ray(m, c).is_none();
                }
                self.shots.push(Shot { t: self.t(), charge: self.charge, hit });
                self.charge = 0.0;
                self.cd = ls.cooldown;
            }
            self.cd = (self.cd - 0.05).max(0.0);
            let d = Drive { mv: a.move_.unwrap_or([0.0, 0.0]), boost: a.boost.unwrap_or(false), jump: a.jump.unwrap_or(false) && k == 0, speed_mult: if self.charge > 0.0 { self.cfg.mech.charge_move_mult } else { 1.0 } };
            if !self.pin {
                physics::step(world, &self.cfg.mech, &mut self.body, &d, 0.05);
            }
        }
        self.tick += 2;
        a
    }
}

fn hurt(by: i64, bearing: f64) -> Event {
    Event::Damaged(DamagedEv { amount: 45.0, weapon: "gun".into(), bearing: Some(bearing), by: Some(by), rear: false })
}

/// A strafing mech 100 m east. In turn: plain duel, the mech drops out of the observation in the middle of our
/// charges, an unseen third mech hits us in the middle of our charges, then we are badly hurt and chased.
fn script(rig: &mut Rig, secs: f64) {
    let n = (secs * 10.0).round() as i64;
    let mut hide_until = -1.0;
    let mut last_unseen = -1e9;
    for i in 0..n {
        let t = i as f64 * 0.1;
        let y = -30.0 + 12.0 * (0.9 * t).sin();
        let mid = rig.charge > 0.25 && rig.charge < 0.45;
        if (6.0..12.0).contains(&t) && mid && t > hide_until + 1.0 {
            hide_until = t + 1.5;
        }
        let foe = Foe { pos: [150.0, y, 0.0], vel: [0.0, 12.0 * 0.9 * (0.9 * t).cos(), 0.0], shown: t >= hide_until, hp: 1400.0 };
        let mut ev = Vec::new();
        if (12.0..18.0).contains(&t) && mid && t - last_unseen > 2.5 {
            last_unseen = t;
            ev.push(hurt(9, 2.5));
        }
        if t >= 18.0 {
            rig.hp = 300.0;
            if i % 5 == 0 {
                ev.push(hurt(EID, (foe.pos[1] - rig.body.pos[1]).atan2(foe.pos[0] - rig.body.pos[0])));
            }
        }
        rig.step(Some(foe), ev);
    }
}


/// Recorded on the commit before the laser flags existed.
const GOLDEN: u64 = 0x9206_8590_27a2_d048;

#[test]
fn flags_off_same_actions() {
    for (name, params) in [("default", Params::default()), ("bot.json", prod_params())] {
        let mut rig = Rig::new(flat(), params, [50.0, -30.0, 0.0]);
        script(&mut rig, 26.0);
        let h = fnv(&rig.log);
        let hits = rig.shots.iter().filter(|s| s.hit).count();
        let retreat = rig.starts.iter().filter(|s| s.1 == Mode::Retreat).count();
        eprintln!("{name}: hash {h:#018x} shots {} hits {hits} starts {} (in retreat {retreat}) held {}", rig.shots.len(), rig.starts.len(), rig.held);
        assert!(rig.shots.len() >= 8 && retreat >= 2, "{name}: the script must exercise the laser");
        assert_eq!(h, GOLDEN, "{name}: actions changed with every new flag off");
    }
}

#[test]
fn all_flags_on_fires_no_blind_shot() {
    let mut p = prod_params();
    p.lz_fc = 1.0;
    p.lz_early = 1.0;
    p.lz_noblind = 1.0;
    p.lz_no_retreat = 1.0;
    let mut rig = Rig::new(flat(), p, [50.0, -30.0, 0.0]);
    script(&mut rig, 26.0);
    for s in &rig.shots {
        eprintln!("   all on: shot t={:.2} charge={:.2} hit={}", s.t, s.charge, s.hit);
    }
    assert!(rig.shots.len() >= 6);
    assert!(rig.shots.iter().all(|s| s.hit), "a shot missed");
    assert_eq!(rig.starts.iter().filter(|s| s.1 == Mode::Retreat).count(), 0);
}

#[test]
fn no_new_charge_while_retreating() {
    let run = |p: Params| {
        let mut rig = Rig::new(flat(), p, [50.0, -30.0, 0.0]);
        script(&mut rig, 26.0);
        (rig.starts.iter().filter(|s| s.1 == Mode::Retreat).count(), rig.starts.len())
    };
    let (off, _) = run(prod_params());
    assert!(off > 0, "flag off: charges do start in retreat");
    let mut p = prod_params();
    p.lz_no_retreat = 1.0;
    let (on, all) = run(p);
    assert_eq!(on, 0);
    assert!(all >= 4);
}

/// The mech drops out of the observation for 1.2 s in the middle of a charge.
fn run_vanish(p: Params) -> (Rig, f64, f64) {
    let mut rig = Rig::new(flat(), p, [50.0, -30.0, 0.0]);
    let (mut from, mut until) = (-1.0, -1.0);
    for i in 0..80 {
        let t = i as f64 * 0.1;
        if from < 0.0 && t > 2.0 && rig.charge > 0.25 {
            from = t;
            until = t + 1.2;
        }
        let foe = Foe { pos: [150.0, -30.0 + 3.0 * t, 0.0], vel: [0.0, 3.0, 0.0], shown: !(from >= 0.0 && t >= from && t < until), hp: 1400.0 };
        rig.step(Some(foe), Vec::new());
    }
    (rig, from, until)
}

#[test]
fn held_charge_is_not_fired_blind() {
    let (rig, from, until) = run_vanish(prod_params());
    assert!(from > 0.0);
    assert!(rig.shots.iter().any(|s| s.t >= from && s.t < until && !s.hit), "flag off: the charge goes off when the target drops out");
    let mut p = prod_params();
    p.lz_noblind = 1.0;
    let (rig, from, until) = run_vanish(p);
    assert!(rig.shots.iter().all(|s| !(s.t >= from && s.t < until)), "flag on: nothing fired while the target is gone");
    let next = rig.shots.iter().find(|s| s.t >= until).expect("a shot once the target is back");
    assert!(next.hit && next.t < until + 0.5, "t={} hit={}", next.t, next.hit);
}

/// A third mech we cannot see hits us in the middle of a charge.
fn run_unseen(p: Params) -> (Rig, f64) {
    let mut rig = Rig::new(flat(), p, [50.0, -30.0, 0.0]);
    let mut at = -1.0;
    for i in 0..80 {
        let t = i as f64 * 0.1;
        let foe = Foe { pos: [150.0, -30.0 + 3.0 * t, 0.0], vel: [0.0, 3.0, 0.0], shown: true, hp: 1400.0 };
        let mut ev = Vec::new();
        if at < 0.0 && t > 2.0 && rig.charge > 0.25 {
            at = t;
            ev.push(hurt(9, 2.5));
        }
        rig.step(Some(foe), ev);
    }
    (rig, at)
}

#[test]
fn unseen_shooter_does_not_spill_the_charge() {
    let (rig, at) = run_unseen(prod_params());
    let s = rig.shots.iter().find(|s| s.t >= at).expect("shot");
    assert!(!s.hit && s.charge < 0.6, "flag off: charge {} hit {}", s.charge, s.hit);
    let mut p = prod_params();
    p.lz_noblind = 1.0;
    let (rig, at) = run_unseen(p);
    let s = rig.shots.iter().find(|s| s.t >= at).expect("shot");
    assert!(s.hit && s.charge >= 0.69, "flag on: charge {} hit {}", s.charge, s.hit);
}

fn tg(feet: V3, vel: V3) -> aim::AimTarget {
    aim::AimTarget { id: EID, feet, vel, acc: [0.0; 3], radius: 2.5, height: 9.0, hp: 1400.0, fixed: false }
}

/// From (50, -30) the block hides x = 185 for y below 0.
#[test]
fn forecast_sees_the_wall() {
    let sh = walled();
    let rig = Rig::new(sh.clone(), Params::default(), [50.0, -30.0, 0.0]);
    let o = rig.obs(None, Vec::new());
    let (w, b) = (&sh.world, &rig.brain);
    let south = tg([185.0, 7.5, 0.0], [0.0, -11.7, 0.0]);
    assert!(b.lz_clear_at(w, &o, &south, 0.0));
    assert!(b.lz_clear_at(w, &o, &south, 0.4));
    assert!(!b.lz_clear_at(w, &o, &south, 0.8));
    assert!(b.lz_clear_at(w, &o, &tg([185.0, 7.5, 0.0], [0.0; 3]), 0.8));
    assert!(!b.lz_clear_at(w, &o, &tg([185.0, -20.0, 0.0], [0.0; 3]), 0.0));
    // walking straight at the block: it cannot enter it, so this is not a target about to hide
    assert!(b.lz_clear_at(w, &o, &tg([135.0, -30.0, 0.0], [11.7, 0.0, 0.0]), 0.8));
    // our own drift counts too, capped at the charging speed
    let mut o2 = rig.obs(None, Vec::new());
    o2.me.vel = [0.0, -30.0, 0.0];
    assert!(b.lz_clear_at(w, &o2, &tg([185.0, 1.0, 0.0], [0.0; 3]), 0.0));
    assert!(!b.lz_clear_at(w, &o2, &tg([185.0, 1.0, 0.0], [0.0; 3]), 0.8));
}

/// The mech walks south along x = 185 and stops behind the block.
fn run_wall(p: Params, y0: f64) -> Rig {
    let mut rig = Rig::new(walled(), p, [50.0, -30.0, 0.0]);
    rig.pin = true;
    for i in 0..40 {
        let t = i as f64 * 0.1;
        let y = (y0 - 11.7 * t).max(-40.0);
        let foe = Foe { pos: [185.0, y, 0.0], vel: [0.0, if y > -40.0 { -11.7 } else { 0.0 }, 0.0], shown: true, hp: 1400.0 };
        rig.step(Some(foe), Vec::new());
    }
    for s in &rig.shots {
        eprintln!("   wall y0={y0}: shot t={:.2} charge={:.2} hit={}", s.t, s.charge, s.hit);
    }
    rig
}

#[test]
fn no_charge_on_a_target_about_to_hide() {
    let rig = run_wall(prod_params(), 7.5);
    assert!(rig.starts.first().map_or(false, |s| s.0 < 0.3), "flag off: the charge starts at once");
    assert!(!rig.shots.is_empty() && rig.shots.iter().all(|s| !s.hit), "flag off: the charge is wasted on the wall");
    let mut p = prod_params();
    p.lz_fc = 1.0;
    let rig = run_wall(p, 7.5);
    assert!(rig.starts.is_empty() && rig.shots.is_empty(), "flag on: no charge at all");
}

#[test]
fn early_partial_shot_before_the_wall() {
    let rig = run_wall(prod_params(), 5.5);
    assert!(rig.shots.first().map_or(true, |s| !s.hit), "flag off: no hit");
    let mut p = prod_params();
    p.lz_early = 1.0;
    let rig = run_wall(p, 5.5);
    let s = rig.shots.first().expect("shot");
    assert!(s.hit && s.charge >= 0.2 && s.charge < 0.65, "flag on: charge {} hit {}", s.charge, s.hit);
}

#[test]
fn planned_partial_shot() {
    let mut p = prod_params();
    p.lz_fc = 1.0;
    p.lz_early = 1.0;
    p.lz_fc_part = 1.0;
    let rig = run_wall(p, 7.5);
    let s = rig.shots.first().expect("shot");
    assert!(s.hit && s.charge >= 0.35 && s.charge < 0.7, "charge {} hit {}", s.charge, s.hit);
}

#[test]
fn lethal_shot_passes_the_gates() {
    // 150 hp left: a short charge kills, so the forecast gate lets it through
    let mut p = prod_params();
    p.lz_fc = 1.0;
    let mut rig = Rig::new(walled(), p, [50.0, -30.0, 0.0]);
    rig.pin = true;
    for i in 0..12 {
        let t = i as f64 * 0.1;
        let foe = Foe { pos: [185.0, 7.5 - 11.7 * t, 0.0], vel: [0.0, -11.7, 0.0], shown: true, hp: 150.0 };
        rig.step(Some(foe), Vec::new());
    }
    let s = rig.shots.first().expect("shot");
    assert!(s.hit && s.charge < 0.5, "charge {} hit {}", s.charge, s.hit);
}
