//! Scenario runs on the duel lab. Heavy runs are opt-in:
//!   MECH_LAB=dodge,duck,hunt   which tables to print (cargo test --release lab_ -- --nocapture)
//!   MECH_MAP=<map dir>         real map (duck, hunt, part of the fingerprints)
//!   MECH_STREAM=<file>         decompressed recorded stream (part of the fingerprints)
//!   MECH_GOLDEN=<file>         fingerprint file: written when missing, compared when present
//!   LAB_SET="name:key=value,key=value;name2:..."   parameter sets on top of bot.json (default "base:")

use std::sync::{Arc, OnceLock};

use super::lab::{Lab, Script, Stats, FNV0};
use super::math::*;
use super::{Brain, Params};
use crate::proto::{Config, ServerMsg, WeaponKind};
use crate::runtime::Budget;
use crate::Shared;

fn base_json() -> serde_json::Value {
    serde_json::from_str(include_str!("../../bot.json")).expect("bot.json")
}

fn params_from(over: &[(String, String)]) -> Params {
    let mut v = base_json();
    for (k, val) in over {
        let j: serde_json::Value = serde_json::from_str(val).unwrap_or(serde_json::Value::String(val.clone()));
        v[k.as_str()] = j;
    }
    serde_json::from_value(v).expect("params")
}

fn sets() -> Vec<(String, Params)> {
    let spec = std::env::var("LAB_SET").unwrap_or_else(|_| "base:".to_string());
    let mut out = Vec::new();
    for part in spec.split(';') {
        let part = part.trim();
        if part.is_empty() {
            continue;
        }
        let (name, rest) = part.split_once(':').unwrap_or((part, ""));
        let over: Vec<(String, String)> = rest.split(',').filter_map(|kv| kv.split_once('=')).map(|(k, v)| (k.trim().to_string(), v.trim().to_string())).collect();
        out.push((name.to_string(), params_from(&over)));
    }
    out
}

fn wants(what: &str) -> bool {
    std::env::var("MECH_LAB").map_or(false, |s| s.split(',').any(|x| x.trim() == what || x.trim() == "all"))
}

fn empty_shared() -> Arc<Shared> {
    static S: OnceLock<Arc<Shared>> = OnceLock::new();
    S.get_or_init(|| {
        let cfg = Config::default();
        let world = crate::world::World::empty();
        let nav = crate::nav::Nav::build(&world, &cfg.mech);
        Arc::new(Shared { world, nav })
    })
    .clone()
}

fn map_shared() -> Option<Arc<Shared>> {
    static S: OnceLock<Option<Arc<Shared>>> = OnceLock::new();
    S.get_or_init(|| {
        let dir = std::env::var("MECH_MAP").ok()?;
        let s = Shared::load(std::path::Path::new(&dir), &Config::default());
        assert!(!s.world.prisms.is_empty(), "map not loaded from {dir}");
        Some(Arc::new(s))
    })
    .clone()
}

/// A gunner at distance d on flat ground; the bot is told to hold that distance.
fn dodge(params: &Params, d: f64, latency: f64, cadence: f64, seed: i64, secs: f64, laser: bool) -> Stats {
    let mut p = params.clone();
    p.range_gun = d;
    p.range_laser = d;
    p.gear_easy_range = 0.0;
    p.engage_range = p.engage_range.max(d + 30.0);
    let load: &[WeaponKind] = if laser { &[WeaponKind::Gun, WeaponKind::Laser] } else { &[WeaponKind::Gun] };
    let mut lab = Lab::new(empty_shared(), p, seed, [0.0, 0.0, 0.0], load);
    lab.zone.center = [0.0, 0.0];
    lab.zone.next_center = [0.0, 0.0];
    lab.add_foe([d, 0.0, 0.0], Script::Gunner { latency, cadence }, false);
    lab.run(secs)
}

/// Places where a target can step between a point we see and a point we do not.
fn duck_sites(shared: &Shared, n: usize, seed: u64) -> Vec<(V3, V3, V3)> {
    let mut rng = Rng::new(seed);
    let (world, nav) = (&shared.world, &shared.nav);
    let mut out = Vec::new();
    let mut tries = 0;
    while out.len() < n && tries < 40000 {
        tries += 1;
        let a0 = [41.3 + rng.range(-300.0, 300.0), -72.8 + rng.range(-300.0, 300.0), 0.0];
        let a0 = [a0[0], a0[1], world.terrain_z(a0[0], a0[1])];
        let Some(a) = nav.snap(a0) else { continue };
        if dist(a, a0) > 2.0 {
            continue;
        }
        let ang = rng.range(0.0, std::f64::consts::TAU);
        let r = rng.range(95.0, 150.0);
        let b0 = [a[0] + r * ang.cos(), a[1] + r * ang.sin(), 0.0];
        let b0 = [b0[0], b0[1], world.terrain_z(b0[0], b0[1])];
        let Some(b) = nav.snap(b0) else { continue };
        if dist(b, b0) > 2.0 {
            continue;
        }
        let (eye, muz) = (up(a, 8.0), up(a, 7.0));
        if world.ray(muz, up(b, 4.5)).is_some() || !world.sight(eye, up(b, 4.5)) {
            continue;
        }
        let mut best: Option<(f64, V3)> = None;
        for k in 0..24 {
            let an = k as f64 * std::f64::consts::TAU / 24.0;
            for rr in [5.0, 8.0, 11.0] {
                let h0 = [b[0] + rr * an.cos(), b[1] + rr * an.sin(), b[2]];
                let Some(h) = nav.snap(h0) else { continue };
                if dist(h, h0) > 2.0 || (h[2] - b[2]).abs() > 1.5 {
                    continue;
                }
                let hidden = world.ray(muz, up(h, 4.5)).is_some() && world.ray(muz, up(h, 8.6)).is_some() && world.ray(eye, up(h, 8.6)).is_some();
                let mut wide = hidden;
                for side in [-12.0, 12.0] {
                    let q = [muz[0] - side * ang.sin(), muz[1] + side * ang.cos(), muz[2]];
                    wide = wide && world.ray(q, up(h, 4.5)).is_some();
                }
                if wide && nav.clear(b, h) && best.map_or(true, |x| rr < x.0) {
                    best = Some((rr, h));
                }
            }
        }
        if let Some((_, h)) = best {
            out.push((a, b, h));
        }
    }
    out
}

/// A laser-armed target that steps out of cover and back.
fn duck(shared: Arc<Shared>, params: &Params, site: (V3, V3, V3), seed: i64, secs: f64) -> Stats {
    let (a, b, h) = site;
    let mut lab = Lab::new(shared, params.clone(), seed, a, &[WeaponKind::Gun, WeaponKind::Laser]);
    lab.zone.radius = 900.0;
    lab.zone.next_radius = 900.0;
    lab.aim = [(b[1] - a[1]).atan2(b[0] - a[0]), 0.0];
    lab.add_foe(b, Script::Ducker { open: b, hide: h, out: true, until: 9.0 }, true);
    lab.run(secs)
}

/// A laser-armed target that wanders near buildings, in and out of sight.
fn cross(shared: Arc<Shared>, params: &Params, site: (V3, V3, V3), seed: i64, secs: f64) -> Stats {
    let (a, b, _) = site;
    let mut lab = Lab::new(shared, params.clone(), seed, a, &[WeaponKind::Gun, WeaponKind::Laser]);
    lab.zone.radius = 900.0;
    lab.zone.next_radius = 900.0;
    lab.aim = [(b[1] - a[1]).atan2(b[0] - a[0]), 0.0];
    lab.add_foe(b, Script::Walker { home: b, goal: b, until: 0.0 }, true);
    lab.run(secs)
}

fn laser_line(name: &str, tot: &Stats) -> String {
    let min = tot.secs.max(1.0) / 60.0;
    format!(
        "{name:14} laser {:4.1}/min hit {:3.0}% held {:3.0}% partial {:3.0}% blind {:3.0}% charging {:2.0}% | gun hit {:3.0}% | dealt {:5.0}/min taken {:5.0}/min ratio {:4.2} | dist {:4.0} m",
        tot.laser_shots as f64 / min,
        pct(tot.laser_hits, tot.laser_shots),
        pct(tot.laser_held, tot.laser_shots),
        pct(tot.laser_partial, tot.laser_shots),
        pct(tot.laser_blind, tot.laser_shots),
        100.0 * tot.charge_s / tot.secs.max(1.0),
        pct(tot.gun_hits, tot.gun_shots),
        tot.dmg_dealt / min,
        tot.dmg_taken / min,
        tot.dmg_dealt / tot.dmg_taken.max(1.0),
        tot.dist_sum / tot.dist_n.max(1) as f64
    )
}

#[test]
fn lab_cross() {
    if !wants("cross") {
        return;
    }
    let Some(shared) = map_shared() else { return };
    let sites = duck_sites(&shared, 12, 99);
    eprintln!("cross: {} sites; our laser against a target wandering in and out of sight", sites.len());
    for (name, p) in sets() {
        let mut tot = Stats::default();
        for (i, s) in sites.iter().enumerate() {
            for seed in 1..=2 {
                tot.add(&cross(shared.clone(), &p, *s, seed * 100 + i as i64, 90.0));
            }
        }
        eprintln!("{}", laser_line(&name, &tot));
    }
}

/// Hunters walk to us one after the other, more of them as they fall.
fn hunt(shared: Arc<Shared>, params: &Params, seed: i64, secs: f64) -> Stats {
    let mut rng = Rng::new(seed as u64 ^ 0xA11CE);
    let (world, nav) = (&shared.world, &shared.nav);
    let spot = |rng: &mut Rng, c: V3, lo: f64, hi: f64| -> V3 {
        loop {
            let an = rng.range(0.0, std::f64::consts::TAU);
            let r = rng.range(lo, hi);
            let q0 = [c[0] + r * an.cos(), c[1] + r * an.sin(), 0.0];
            let q0 = [q0[0], q0[1], world.terrain_z(q0[0], q0[1])];
            if (q0[0] - 41.3).hypot(q0[1] + 72.8) > 520.0 {
                continue;
            }
            if let Some(q) = nav.snap(q0) {
                if dist(q, q0) < 2.0 {
                    return q;
                }
            }
        }
    };
    let start = spot(&mut rng, [41.3, -72.8, 0.0], 0.0, 250.0);
    let mut lab = Lab::new(shared.clone(), params.clone(), seed, start, &[WeaponKind::Gun, WeaponKind::Laser]);
    lab.mortal = true;
    lab.foes_mortal = true;
    lab.warmup = 0.0;
    lab.zone.radius = 560.0;
    lab.zone.next_radius = 560.0;
    let tiers: [(f64, f64, bool); 4] = [(0.09, 0.14, false), (0.07, 0.12, true), (0.055, 0.095, true), (0.045, 0.085, true)];
    let mut spawned = 0usize;
    while lab.t() < secs {
        let alive = lab.foes.iter().filter(|f| f.alive).count();
        let want = 1 + lab.st.kills as usize / 2;
        if alive < want && spawned < 15 {
            let tier = tiers[(spawned / 4).min(3)];
            let q = spot(&mut rng, lab.body.pos, 220.0, 320.0);
            lab.add_foe(q, Script::Hunter { sg: tier.0, sl: tier.1, pref: 70.0 }, tier.2);
            spawned += 1;
        }
        if !lab.decide() {
            break;
        }
    }
    lab.st.clone()
}

/// A pack of n hunters (medium aim, gun + laser) sets off together from one side.
fn pack(shared: Arc<Shared>, params: &Params, seed: i64, n: usize, secs: f64) -> Stats {
    let mut rng = Rng::new(seed as u64 ^ 0xBEE5);
    let (world, nav) = (&shared.world, &shared.nav);
    let spot = |rng: &mut Rng, c: V3, lo: f64, hi: f64, a0: f64, da: f64| -> V3 {
        for _ in 0..4000 {
            let an = a0 + rng.range(-da, da);
            let r = rng.range(lo, hi);
            let q0 = [c[0] + r * an.cos(), c[1] + r * an.sin(), 0.0];
            let q0 = [q0[0], q0[1], world.terrain_z(q0[0], q0[1])];
            if (q0[0] - 41.3).hypot(q0[1] + 72.8) > 380.0 {
                continue;
            }
            if let Some(q) = nav.snap(q0) {
                if dist(q, q0) < 2.0 {
                    return q;
                }
            }
        }
        c
    };
    let start = spot(&mut rng, [41.3, -72.8, 0.0], 0.0, 200.0, 0.0, std::f64::consts::PI);
    let mut lab = Lab::new(shared.clone(), params.clone(), seed, start, &[WeaponKind::Gun, WeaponKind::Laser]);
    lab.mortal = true;
    lab.foes_mortal = true;
    lab.warmup = 0.0;
    lab.zone.radius = 380.0;
    lab.zone.next_radius = 380.0;
    let bearing = rng.range(0.0, std::f64::consts::TAU);
    if let Some(lands) = std::env::var("LAB_DROP").ok().and_then(|v| v.parse::<f64>().ok()) {
        let q = spot(&mut rng, start, 40.0, 100.0, bearing + std::f64::consts::PI, 1.0);
        lab.drop = Some((q, lands, 0.0));
    }
    for k in 0..n {
        let q = spot(&mut rng, start, 190.0 + 25.0 * k as f64, 230.0 + 25.0 * k as f64, bearing, 0.35);
        lab.add_foe(q, Script::Hunter { sg: 0.06, sl: 0.1, pref: 70.0 }, true);
    }
    lab.run(secs)
}

#[test]
fn lab_pack() {
    if !wants("pack") {
        return;
    }
    let Some(shared) = map_shared() else { return };
    let runs: i64 = std::env::var("MECH_RUNS").ok().and_then(|s| s.parse().ok()).unwrap_or(30);
    let n: usize = std::env::var("LAB_PACK").ok().and_then(|s| s.parse().ok()).unwrap_or(3);
    eprintln!("pack: {runs} runs; {n} hunters (gun + laser) set off together from one side");
    for (name, p) in sets() {
        let mut tot = Stats::default();
        let mut life = 0.0;
        let mut wins = 0;
        for seed in 1..=runs {
            let s = pack(shared.clone(), &p, seed, n, 240.0);
            life += s.died_at.unwrap_or(240.0);
            if s.died_at.is_none() {
                wins += 1;
            }
            tot.add(&s);
        }
        eprintln!(
            "{name:14} kills {:4.2} life {:5.1}s alive at end {:2}/{runs} drops {:2} | dealt/taken {:4.2} dealt {:5.0} | their gun {:3.0}% laser {:3.0}% | our laser hit {:3.0}% gun hit {:3.0}% | dist {:4.0} m boost {:2.0}%",
            tot.kills as f64 / runs as f64,
            life / runs as f64,
            wins,
            tot.drops,
            tot.dmg_dealt / tot.dmg_taken.max(1.0),
            tot.dmg_dealt / runs as f64,
            pct(tot.hits_on_us, tot.shots_at_us),
            pct(tot.laser_hits_on_us, tot.lasers_at_us),
            pct(tot.laser_hits, tot.laser_shots),
            pct(tot.gun_hits, tot.gun_shots),
            tot.dist_sum / tot.dist_n.max(1) as f64,
            100.0 * tot.boost_s / tot.secs.max(1.0)
        );
    }
}

fn pct(a: u32, b: u32) -> f64 {
    if b == 0 {
        0.0
    } else {
        100.0 * a as f64 / b as f64
    }
}

#[test]
fn lab_dodge() {
    if !wants("dodge") {
        return;
    }
    let secs = 66.0;
    let dists: Vec<f64> = std::env::var("LAB_DIST").ok().map(|s| s.split(',').filter_map(|x| x.trim().parse().ok()).collect()).unwrap_or_else(|| vec![60.0, 100.0, 140.0, 180.0]);
    eprintln!("dodge: share of aimed gun shots that hit us (two shooter models), our own gun shots per minute");
    for (name, p) in sets() {
        for laser in [false, true] {
            let mut line = format!("{name:14} {}", if laser { "gun+laser" } else { "gun      " });
            let mut tot = Stats::default();
            for d in dists.iter().copied() {
                let mut cell = String::new();
                for (lat, cad) in [(0.05, 0.24), (0.1, 0.3)] {
                    let mut s = Stats::default();
                    for seed in 1..=3 {
                        s.add(&dodge(&p, d, lat, cad, seed, secs, laser));
                    }
                    cell += &format!(" {:3.0}%", pct(s.hits_on_us, s.shots_at_us));
                    tot.add(&s);
                }
                line += &format!(" | {d:3.0}m{cell}");
            }
            line += &format!(" | all {:4.1}% | our shots/min {:4.0} hit {:3.0}% | boost {:2.0}% air {:2.0}%", pct(tot.hits_on_us, tot.shots_at_us), tot.gun_shots as f64 * 60.0 / tot.secs.max(1.0), pct(tot.gun_hits, tot.gun_shots), 100.0 * tot.boost_s / tot.secs.max(1.0), 100.0 * tot.air_s / tot.secs.max(1.0));
            eprintln!("{line}");
        }
    }
}

#[test]
fn lab_duck() {
    if !wants("duck") {
        return;
    }
    let Some(shared) = map_shared() else { return };
    let sites = duck_sites(&shared, 10, 77);
    eprintln!("duck: {} sites; our laser against a target stepping in and out of cover", sites.len());
    for (name, p) in sets() {
        let mut tot = Stats::default();
        for (i, s) in sites.iter().enumerate() {
            for seed in 1..=2 {
                tot.add(&duck(shared.clone(), &p, *s, seed * 100 + i as i64, 70.0));
            }
        }
        let min = tot.secs.max(1.0) / 60.0;
        eprintln!(
            "{name:14} laser {:4.1}/min hit {:3.0}% held {:3.0}% partial {:3.0}% blind {:3.0}% charging {:2.0}% | gun hit {:3.0}% | dealt {:5.0}/min taken {:5.0}/min ratio {:4.2} | dist {:4.0} m",
            tot.laser_shots as f64 / min,
            pct(tot.laser_hits, tot.laser_shots),
            pct(tot.laser_held, tot.laser_shots),
            pct(tot.laser_partial, tot.laser_shots),
            pct(tot.laser_blind, tot.laser_shots),
            100.0 * tot.charge_s / tot.secs.max(1.0),
            pct(tot.gun_hits, tot.gun_shots),
            tot.dmg_dealt / min,
            tot.dmg_taken / min,
            tot.dmg_dealt / tot.dmg_taken.max(1.0),
            tot.dist_sum / tot.dist_n.max(1) as f64
        );
    }
}

#[test]
fn lab_hunt() {
    if !wants("hunt") {
        return;
    }
    let Some(shared) = map_shared() else { return };
    let runs: i64 = std::env::var("MECH_RUNS").ok().and_then(|s| s.parse().ok()).unwrap_or(12);
    eprintln!("hunt: {runs} runs; hunters with measured aim noise walk to us, 1 + kills/2 at a time");
    for (name, p) in sets() {
        let mut tot = Stats::default();
        let (mut life, mut kills) = (0.0, Vec::new());
        for seed in 1..=runs {
            let s = hunt(shared.clone(), &p, seed, 300.0);
            life += s.died_at.unwrap_or(300.0);
            kills.push(s.kills);
            tot.add(&s);
        }
        kills.sort();
        eprintln!(
            "{name:14} kills {:4.2} (median {}) life {:5.1}s | dealt/taken {:4.2} | their gun {:3.0}% laser {:3.0}% | our laser hit {:3.0}% held {:3.0}% gun hit {:3.0}% | dist {:4.0} m boost {:2.0}%",
            tot.kills as f64 / runs as f64,
            kills[kills.len() / 2],
            life / runs as f64,
            tot.dmg_dealt / tot.dmg_taken.max(1.0),
            pct(tot.hits_on_us, tot.shots_at_us),
            pct(tot.laser_hits_on_us, tot.lasers_at_us),
            pct(tot.laser_hits, tot.laser_shots),
            pct(tot.laser_held, tot.laser_shots),
            pct(tot.gun_hits, tot.gun_shots),
            tot.dist_sum / tot.dist_n.max(1) as f64,
            100.0 * tot.boost_s / tot.secs.max(1.0)
        );
    }
}

fn stream_hash(shared: Arc<Shared>, lines: &[String], params: &Params) -> u64 {
    let mut brain: Option<Brain> = None;
    let mut h = FNV0;
    for l in lines {
        match serde_json::from_str::<ServerMsg>(l) {
            Ok(ServerMsg::Hello(hello)) => {
                let mut b = Brain::with_params(&hello, shared.clone(), params.clone());
                b.frozen = true;
                brain = Some(b);
            }
            Ok(ServerMsg::Obs(o)) => {
                let a = brain.as_mut().expect("hello first").act(&o, &Budget::unlimited());
                for b in serde_json::to_string(&a).unwrap_or_default().as_bytes() {
                    h ^= *b as u64;
                    h = h.wrapping_mul(0x0000_0100_0000_01B3);
                }
            }
            _ => {}
        }
    }
    h
}

/// Fingerprints of every action on fixed runs. With all new switches off they must not change.
#[test]
fn lab_golden() {
    let base = params_from(&[]);
    let mut out: Vec<String> = Vec::new();
    for (i, (d, laser)) in [(100.0, false), (120.0, true)].iter().enumerate() {
        let s = dodge(&base, *d, 0.05, 0.24, 11 + i as i64, 25.0, *laser);
        out.push(format!("dodge{i} {:016x}", s.hash));
    }
    if let Ok(path) = std::env::var("MECH_STREAM") {
        let text = std::fs::read_to_string(path).expect("stream");
        let lines: Vec<String> = text.lines().map(|s| s.to_string()).collect();
        out.push(format!("stream default empty {:016x}", stream_hash(empty_shared(), &lines, &Params::default())));
        out.push(format!("stream bot.json empty {:016x}", stream_hash(empty_shared(), &lines, &base)));
        if let Some(shared) = map_shared() {
            out.push(format!("stream default map {:016x}", stream_hash(shared.clone(), &lines, &Params::default())));
            out.push(format!("stream bot.json map {:016x}", stream_hash(shared, &lines, &base)));
        }
    }
    if let Some(shared) = map_shared() {
        let sites = duck_sites(&shared, 3, 77);
        for (i, s) in sites.iter().enumerate() {
            out.push(format!("duck{i} {:016x}", duck(shared.clone(), &base, *s, 5 + i as i64, 40.0).hash));
        }
        for (i, s) in sites.iter().enumerate().take(2) {
            out.push(format!("cross{i} {:016x}", cross(shared.clone(), &base, *s, 9 + i as i64, 60.0).hash));
        }
        for seed in 1..=2 {
            out.push(format!("hunt{seed} {:016x}", hunt(shared.clone(), &base, seed, 90.0).hash));
        }
    }
    let text = out.join("\n") + "\n";
    eprintln!("{text}");
    if let Ok(path) = std::env::var("MECH_GOLDEN") {
        match std::fs::read_to_string(&path) {
            Ok(old) => assert_eq!(old.replace("\r\n", "\n"), text, "fingerprints changed"),
            Err(_) => std::fs::write(&path, &text).expect("write fingerprints"),
        }
    }
}

#[test]
fn seg_and_noise() {
    let mut rng = Rng::new(7);
    let mut inside = 0;
    for _ in 0..2000 {
        let u = rng.f64().max(1e-12);
        let v = rng.f64();
        let g = (-2.0 * u.ln()).sqrt() * (2.0 * std::f64::consts::PI * v).cos();
        if g.abs() < 1.0 {
            inside += 1;
        }
    }
    assert!((inside as f64 / 2000.0 - 0.683).abs() < 0.04);
}
