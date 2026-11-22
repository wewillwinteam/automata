//! Closed-loop checks of the engagement model: one scripted hunter walking at us on flat ground.

use std::sync::Arc;

use super::math::*;
use super::*;
use crate::proto::{ActorKind, ActorObs, Obs, WeaponKind, WeaponObs, ZoneObs};
use crate::runtime::Budget;

struct Out {
    /// mean distance to the hunter over the last third of the run
    mean: f64,
    min: f64,
    max: f64,
    laser_shots: u32,
    /// seconds until the hunter is within 80 m (the run length if it never gets there)
    t_close: f64,
}

fn run(params: Params, ffa: bool, approach: f64, secs: f64) -> Out {
    let hello = crate::proto::Hello::default();
    let cfg = hello.config.clone();
    let world = crate::world::World::empty();
    let shared = Arc::new(Shared { nav: crate::nav::Nav::build(&world, &cfg.mech), world });
    let mut brain = Brain::with_params(&hello, shared.clone(), params);
    brain.ffa.on = ffa;
    let mut body = crate::physics::Body::new([0.0, 0.0, 0.0], cfg.mech.energy);
    let mut aim = [0.0, 0.0];
    let mut foe = [120.0, 0.0, 0.0];
    let (mut charge, mut cooldown) = (0.0f64, 0.0f64);
    let full = cfg.weapons.laser.charge;
    let n = (secs / 0.1) as i64;
    let (mut sum, mut cnt, mut min, mut max, mut shots) = (0.0, 0.0, f64::INFINITY, 0.0f64, 0u32);
    let mut t_close = secs;
    for i in 0..n {
        let t = i as f64 * 0.1;
        let u = unit2(body.pos[0] - foe[0], body.pos[1] - foe[1]);
        let fv = [u[0] * approach, u[1] * approach, 0.0];
        let mut o = Obs::default();
        o.tick = i * 2;
        o.t = t;
        o.alive = 16;
        o.me.id = 0;
        o.me.pos = body.pos;
        o.me.vel = body.vel;
        o.me.aim = aim;
        o.me.hp = cfg.mech.hp;
        o.me.max_hp = cfg.mech.hp;
        o.me.energy = body.energy;
        o.me.max_energy = cfg.mech.energy;
        o.me.grounded = body.grounded;
        o.me.in_zone = true;
        o.me.stats = cfg.mech.clone();
        o.me.weapons = [
            Some(WeaponObs { kind: WeaponKind::Gun, ammo: 12.0, stats: cfg.weapons.gun.clone(), ..Default::default() }),
            Some(WeaponObs { kind: WeaponKind::Laser, charge, cooldown, stats: cfg.weapons.laser.clone(), ..Default::default() }),
        ];
        o.mechs.push(ActorObs { id: 5, kind: ActorKind::Mech, pos: foe, vel: fv, aim: [u[1].atan2(u[0]), 0.0], hp: Some(cfg.mech.hp), max_hp: Some(cfg.mech.hp), cubes: 0.0, weapons: vec![WeaponKind::Gun, WeaponKind::Laser], charging: false });
        o.zone = ZoneObs { center: [0.0, 0.0], radius: 560.0, next_center: [0.0, 0.0], next_radius: 380.0, shrink_start: 60.0, shrink_end: 110.0, dps: 20.0 };
        let a = brain.act(&o, &Budget::unlimited());
        let hold = a.fire.map_or(false, |f| f[1]);
        let drive = crate::physics::Drive { mv: a.move_.unwrap_or([0.0, 0.0]), boost: a.boost.unwrap_or(false), jump: false, speed_mult: if hold && cooldown <= 0.0 { cfg.mech.charge_move_mult } else { 1.0 } };
        for _ in 0..2 {
            crate::physics::step(&shared.world, &cfg.mech, &mut body, &drive, 0.05);
            aim = crate::physics::slew_aim(aim, a.aim.unwrap_or(aim), &cfg.mech, 1.0, 0.05);
        }
        if cooldown > 0.0 {
            cooldown = (cooldown - 0.1).max(0.0);
        } else if hold {
            charge += 0.1;
            if charge >= full + cfg.weapons.laser.hold_s {
                charge = 0.0;
                cooldown = cfg.weapons.laser.cooldown;
                shots += 1;
            }
        } else if charge > 0.0 {
            charge = 0.0;
            cooldown = cfg.weapons.laser.cooldown;
            shots += 1;
        }
        foe = [foe[0] + fv[0] * 0.1, foe[1] + fv[1] * 0.1, 0.0];
        let d = dist2(foe, body.pos);
        if std::env::var("DUEL_TRACE").is_ok() && i % 20 == 0 {
            eprintln!("  t={t:.0} d={d:.0} v=({:.1},{:.1}) fire={:?} charge={charge:.2} cd={cooldown:.2} | {}", body.vel[0], body.vel[1], a.fire, brain.state());
        }
        min = min.min(d);
        if d < 80.0 && t_close >= secs {
            t_close = t;
        }
        max = max.max(d);
        if i * 3 >= n * 2 {
            sum += d;
            cnt += 1.0;
        }
    }
    Out { mean: sum / cnt, min, max, laser_shots: shots, t_close }
}

fn on() -> Params {
    let mut p = Params::default();
    p.mode_detect = 1.0;
    p.thr_model = 1.0;
    p.mx_on = 1.0;
    p.hold_hunter = 1.0;
    p
}

#[test]
fn stand_off() {
    let old = run(Params::default(), false, 5.0, 45.0);
    let squad = run(on(), false, 5.0, 45.0);
    let team = run(on(), true, 5.0, 45.0);
    eprintln!("legacy:      mean {:.0} m (min {:.0}, max {:.0}), {} laser shots", old.mean, old.min, old.max, old.laser_shots);
    eprintln!("model squad: mean {:.0} m (min {:.0}, max {:.0}), {} laser shots", squad.mean, squad.min, squad.max, squad.laser_shots);
    eprintln!("model team:  mean {:.0} m (min {:.0}, max {:.0}), {} laser shots", team.mean, team.min, team.max, team.laser_shots);
    assert!(old.mean > 105.0 && old.mean < 155.0, "legacy stays near its 140 m, got {:.0}", old.mean);
    assert!(squad.mean > 162.0 && squad.mean < 195.0, "squad stand-off, got {:.0}", squad.mean);
    assert!((team.mean - old.mean).abs() < 12.0, "symmetric team duel stays at the legacy distance, got {:.0} against {:.0}", team.mean, old.mean);
    assert!(squad.laser_shots * 10 >= old.laser_shots * 7, "the laser keeps firing at the stand-off distance");
}

#[test]
fn fast_hunter() {
    // a hunter walking at our own speed cannot be held off, only slowed down
    let old = run(Params::default(), false, 11.7, 30.0);
    let squad = run(on(), false, 11.7, 30.0);
    let mut pb = on();
    pb.mx_boost = 10.0;
    let boost = run(pb, false, 11.7, 30.0);
    eprintln!("legacy:        within 80 m after {:.1} s, {} laser shots, min {:.0} m", old.t_close, old.laser_shots, old.min);
    eprintln!("model:         within 80 m after {:.1} s, {} laser shots, min {:.0} m", squad.t_close, squad.laser_shots, squad.min);
    eprintln!("model + boost: within 80 m after {:.1} s, {} laser shots, min {:.0} m", boost.t_close, boost.laser_shots, boost.min);
    assert!(squad.t_close >= old.t_close * 1.5, "model {:.1} s against legacy {:.1} s", squad.t_close, old.t_close);
    assert!(boost.t_close >= squad.t_close - 0.5, "boost {:.1} s against {:.1} s", boost.t_close, squad.t_close);
}
