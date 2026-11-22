use super::math::*;
use super::*;
use std::f64::consts::PI;

#[test]
fn angle_wrap() {
    assert!((wrap(3.0 * PI) - PI).abs() < 1e-9);
    assert!((wrap(-3.0 * PI) - PI).abs() < 1e-9);
    assert!((adiff(3.1, -3.1) - (6.2 - 2.0 * PI)).abs() < 1e-9);
    assert!((adiff(-3.1, 3.1) + (6.2 - 2.0 * PI)).abs() < 1e-9);
    assert_eq!(wrap(f64::NAN), 0.0);
    let s = slew([3.1, 0.0], [-3.1, 0.5], 0.2, 0.15);
    assert!((adiff(s[0], -3.1)).abs() < 1e-9 && (s[1] - 0.15).abs() < 1e-9);
    let s = slew([0.0, 0.0], [2.0, -1.0], 0.2, 0.15);
    assert!((s[0] - 0.2).abs() < 1e-9 && (s[1] + 0.15).abs() < 1e-9);
    let a = dir_to_aim(aim_dir(-2.5, 0.4));
    assert!((a[0] + 2.5).abs() < 1e-9 && (a[1] - 0.4).abs() < 1e-9);
}

#[test]
fn intercept() {
    let p = [100.0, 0.0, 0.0];
    let v = [0.0, 11.7, 0.0];
    let t = intercept_time(p, v, 252.0).unwrap();
    let hit = add(p, scale(v, t));
    assert!((len(hit) - 252.0 * t).abs() < 1e-6);
    assert!(t > 0.39 && t < 0.41);
    // receding faster than the shot
    assert!(intercept_time([10.0, 0.0, 0.0], [300.0, 0.0, 0.0], 252.0).is_none());
    // approaching
    let t = intercept_time([100.0, 0.0, 0.0], [-20.0, 0.0, 0.0], 252.0).unwrap();
    assert!((t - 100.0 / 272.0).abs() < 1e-9);
    let (pt, tf) = lead(p, v, [0.0; 3], 252.0, 0.05, 0.0, 0.3).unwrap();
    let truth = add(p, scale(v, 0.05 + tf));
    assert!(dist(pt, truth) < 1e-6);
    let (pt, tf) = lead(p, v, [0.0, -20.0, 0.0], 252.0, 0.0, 1.0, 1.0).unwrap();
    let truth = add(add(p, scale(v, tf)), scale([0.0, -20.0, 0.0], 0.5 * tf * tf));
    assert!(dist(pt, truth) < 0.05);
}

#[test]
fn mortar() {
    for &(d, dz) in &[(80.0, 0.0), (150.0, -9.0), (40.0, 12.0), (200.0, 5.0)] {
        for high in [false, true] {
            let (pitch, tf) = mortar_solution(d, dz, 72.0, 22.0, high).unwrap();
            let (mut x, mut z, mut t) = (0.0, 0.0, 0.0);
            let (vx, mut vz) = (72.0 * pitch.cos(), 72.0 * pitch.sin());
            let h = 1e-4;
            while t < tf {
                x += vx * h;
                z += vz * h - 0.5 * 22.0 * h * h;
                vz -= 22.0 * h;
                t += h;
            }
            assert!((x - d).abs() < 0.1, "x {x} d {d}");
            assert!((z - dz).abs() < 0.1, "z {z} dz {dz}");
            if high {
                assert!(pitch > PI / 4.0 - 0.1);
            }
        }
    }
    assert!(mortar_solution(400.0, 0.0, 72.0, 22.0, false).is_none());
    assert!(mortar_solution(0.0, 0.0, 72.0, 22.0, false).is_none());
}

#[test]
fn closest_approach() {
    let (t, d, dz) = closest_xy([-10.0, 3.0, 1.0], [100.0, 0.0, -5.0], 0.0, 1.0);
    assert!((t - 0.1).abs() < 1e-9 && (d - 3.0).abs() < 1e-9 && (dz - 0.5).abs() < 1e-9);
    let (t, d, _) = closest_xy([-10.0, 3.0, 0.0], [100.0, 0.0, 0.0], 0.0, 0.05);
    assert!((t - 0.05).abs() < 1e-9 && (d - (25.0f64 + 9.0).sqrt()).abs() < 1e-9);
    let (t, d, _) = closest_xy([4.0, 3.0, 0.0], [0.0, 0.0, 0.0], 0.0, 1.0);
    assert!(t == 0.0 && (d - 5.0).abs() < 1e-9);
}

#[test]
fn rng_is_deterministic() {
    let mut a = Rng::new(7);
    let mut b = Rng::new(7);
    for _ in 0..100 {
        let x = a.f64();
        assert_eq!(x, b.f64());
        assert!((0.0..1.0).contains(&x));
    }
}

fn check_action(a: &crate::proto::Action, tick: i64) {
    assert_eq!(a.tick, tick);
    let m = a.move_.expect("move");
    assert!(m[0].is_finite() && m[1].is_finite() && m[0].hypot(m[1]) <= 1.0 + 1e-9);
    let aim = a.aim.expect("aim");
    assert!(aim[0].is_finite() && aim[1].is_finite());
    assert!(aim[0].abs() <= PI + 1e-9 && aim[1] >= -0.7 - 1e-9 && aim[1] <= 1.3 + 1e-9);
    assert!(a.fire.is_some());
    if let Some(pk) = a.pick {
        assert!((0..=2).contains(&pk));
    }
}

fn run_stream(shared: Arc<Shared>, lines: &[String]) -> usize {
    let mut brain: Option<Brain> = None;
    let mut n = 0;
    let mut moved = 0;
    for l in lines {
        match serde_json::from_str::<crate::proto::ServerMsg>(l) {
            Ok(crate::proto::ServerMsg::Hello(h)) => brain = Some(Brain::with_params(&h, shared.clone(), Params::default())),
            Ok(crate::proto::ServerMsg::Obs(o)) => {
                let b = brain.as_mut().expect("hello first");
                let t0 = std::time::Instant::now();
                let a = b.act(&o, &Budget::unlimited());
                let ms = t0.elapsed().as_secs_f64() * 1000.0;
                if std::env::var("MECH_TRACE").is_ok() {
                    eprintln!("{} {:.2}ms hp={:.0} e={:.0} mv={:?} b={:?} aim={:?}/{:?} fire={:?} rl={:?} take={:?} | {}", o.tick, ms, o.me.hp, o.me.energy, a.move_.map(|m| [(m[0] * 100.0).round() / 100.0, (m[1] * 100.0).round() / 100.0]), a.boost, a.aim.map(|m| [(m[0] * 100.0).round() / 100.0, (m[1] * 100.0).round() / 100.0]), [(o.me.aim[0] * 100.0).round() / 100.0, (o.me.aim[1] * 100.0).round() / 100.0], a.fire, a.reload, a.take, b.state());
                }
                check_action(&a, o.tick);
                if a.move_.map_or(false, |m| m[0].hypot(m[1]) > 0.5) {
                    moved += 1;
                }
                n += 1;
            }
            _ => {}
        }
    }
    assert!(moved > 0 || n == 0);
    n
}

/// Set MECH_STREAM to a decompressed recording (and MECH_MAP to the map dir) to run.
#[test]
fn recorded_stream() {
    let Ok(path) = std::env::var("MECH_STREAM") else { return };
    let text = std::fs::read_to_string(path).expect("stream");
    let lines: Vec<String> = text.lines().map(|s| s.to_string()).collect();
    let cfg = crate::proto::Config::default();
    let empty = Arc::new(Shared { world: crate::world::World::empty(), nav: crate::nav::Nav::build(&crate::world::World::empty(), &cfg.mech) });
    let n = run_stream(empty, &lines);
    assert!(n >= 300, "only {n} observations");
    if let Ok(dir) = std::env::var("MECH_MAP") {
        let shared = Arc::new(Shared::load(std::path::Path::new(&dir), &cfg));
        let t0 = std::time::Instant::now();
        let n = run_stream(shared, &lines);
        eprintln!("map run: {n} obs, {:.3} ms/obs", t0.elapsed().as_secs_f64() * 1000.0 / n as f64);
    }
}

#[test]
fn degenerate_obs() {
    let hello = crate::proto::Hello::default();
    let cfg = &hello.config;
    let shared = Arc::new(Shared { world: crate::world::World::empty(), nav: crate::nav::Nav::build(&crate::world::World::empty(), &cfg.mech) });
    let mut b = Brain::with_params(&hello, shared, Params::default());
    let o: crate::proto::Obs = serde_json::from_str("{}").unwrap();
    let a = b.act(&o, &Budget::unlimited());
    check_action(&a, 0);
    let o: crate::proto::Obs = serde_json::from_str(
        r#"{"tick":2,"t":0.1,"self":{"id":1,"pos":[0,0,0],"hp":0,"max_hp":0,"weapons":[null,null],"stats":{"walk_speed":0,"radius":0,"height":0,"turn_rate":0}},
        "mechs":[{"id":2,"kind":"mech","pos":[0,0,0]},{"id":3,"kind":"weird","pos":[1e308,1e308,1e308],"vel":[1e308,-1e308,0]}],
        "projectiles":[{"id":9,"kind":"mortar","pos":[0,0,50],"vel":[0,0,0]},{"id":10,"kind":"gun","pos":[0,0,0],"vel":[0,0,0]}],
        "zone":{"radius":0,"next_radius":0},"offer":{"id":1,"choices":[]},
        "drops":[{"id":20000,"pos":[0,0,0],"radius":0}],"crates":[{"id":1000,"pos":[5,5,0]}],"cubes":[{"id":5000,"pos":[1,1,0]}],
        "pickups":[{"id":8000,"kind":"laser","pos":[0,1,0]}],"events":[{"type":"damaged","amount":10,"weapon":"gun","bearing":1.0,"by":2},{"type":"nonsense"}]}"#,
    )
    .unwrap();
    let a = b.act(&o, &Budget::unlimited());
    check_action(&a, 2);
    assert_eq!(a.take, Some(8000));
}

fn zone_obs(z: &[([f64; 2], f64)], cfg: &crate::proto::Config, i: usize, t: f64, t0: f64, live: bool) -> crate::proto::ZoneObs {
    let ph = &cfg.zone.phases[i.min(cfg.zone.phases.len() - 1)];
    let ss = t0 + ph.hold_s;
    let se = ss + ph.shrink_s;
    let (c0, r0) = z[i];
    let (c1, r1) = z[i + 1];
    let k = if live { ((t - ss) / (se - ss)).clamp(0.0, 1.0) } else { 0.0 };
    crate::proto::ZoneObs { center: [c0[0] + (c1[0] - c0[0]) * k, c0[1] + (c1[1] - c0[1]) * k], radius: r0 + (r1 - r0) * k, next_center: c1, next_radius: r1, shrink_start: ss, shrink_end: se, dps: ph.dps }
}

/// Needs MECH_MAP. MECH_RUNS sets the number of runs.
#[test]
fn zone_walk() {
    let Ok(dir) = std::env::var("MECH_MAP") else { return };
    let hello = crate::proto::Hello::default();
    let cfg = hello.config.clone();
    let shared = Arc::new(Shared::load(std::path::Path::new(&dir), &cfg));
    let runs: usize = std::env::var("MECH_RUNS").ok().and_then(|s| s.parse().ok()).unwrap_or(4);
    let mut rng = Rng::new(12345);
    let mut worst_ms: f64 = 0.0;
    let (mut total_early, mut total_unstick, mut bad_runs) = (0.0, 0u32, 0usize);
    for run in 0..runs {
        let live = run % 2 == 0;
        let mut z = vec![(cfg.zone.start_center, cfg.zone.start_radius)];
        for ph in &cfg.zone.phases {
            let (c, r) = *z.last().unwrap();
            let a = rng.range(0.0, 2.0 * PI);
            let d = (r - ph.radius).max(0.0) * rng.range(0.0, 0.95);
            z.push(([c[0] + d * a.cos(), c[1] + d * a.sin()], ph.radius));
        }
        let start = loop {
            let a = rng.range(0.0, 2.0 * PI);
            let d = cfg.zone.start_radius * 0.8 * rng.f64().sqrt();
            let (x, y) = (z[0].0[0] + d * a.cos(), z[0].0[1] + d * a.sin());
            let q = [x, y, shared.world.terrain_z(x, y)];
            if let Some(s) = shared.nav.snap(q) {
                if dist(s, q) < 3.0 {
                    break s;
                }
            }
        };
        if std::env::var("MECH_ONLY").ok().and_then(|s| s.parse::<usize>().ok()).map_or(false, |o| o != run) {
            continue;
        }
        let mut h = hello.clone();
        h.seed = run as i64 * 77 + 5;
        let mut brain = Brain::with_params(&h, shared.clone(), Params::default());
        let mut body = crate::physics::Body::new(start, cfg.mech.energy);
        let mut aim = [0.0, 0.0];
        let (mut phase, mut t0) = (0usize, 0.0);
        let mut hp = cfg.mech.hp;
        let (mut out_s, mut zone_dmg, mut still_s, mut early_dmg) = (0.0, 0.0, 0.0, 0.0);
        let mut tick = 0i64;
        let end_t: f64 = cfg.zone.phases.iter().map(|p| p.hold_s + p.shrink_s).sum::<f64>() - 12.0;
        while (tick as f64) * 0.05 < end_t {
            let t = tick as f64 * 0.05;
            while phase < cfg.zone.phases.len() - 1 && t >= t0 + cfg.zone.phases[phase].hold_s + cfg.zone.phases[phase].shrink_s {
                t0 += cfg.zone.phases[phase].hold_s + cfg.zone.phases[phase].shrink_s;
                phase += 1;
            }
            let zl = zone_obs(&z, &cfg, phase, t, t0, true);
            let zo = zone_obs(&z, &cfg, phase, t, t0, live);
            let inside = dist2v(body.pos, zl.center) <= zl.radius;
            let mut o = crate::proto::Obs::default();
            o.tick = tick;
            o.t = t;
            o.alive = 16;
            o.me.pos = body.pos;
            o.me.vel = body.vel;
            o.me.aim = aim;
            o.me.hp = hp;
            o.me.max_hp = cfg.mech.hp;
            o.me.energy = body.energy;
            o.me.max_energy = cfg.mech.energy;
            o.me.grounded = body.grounded;
            o.me.in_zone = inside;
            o.me.stats = cfg.mech.clone();
            o.me.weapons = [Some(crate::proto::WeaponObs { kind: WeaponKind::Gun, ammo: 12.0, stats: cfg.weapons.gun.clone(), ..Default::default() }), None];
            o.zone = zo;
            let w0 = std::time::Instant::now();
            let a = brain.act(&o, &Budget::from_now_ms(30.0));
            worst_ms = worst_ms.max(w0.elapsed().as_secs_f64() * 1000.0);
            check_action(&a, tick);
            let drive = crate::physics::Drive { mv: a.move_.unwrap_or([0.0, 0.0]), boost: a.boost.unwrap_or(false), jump: a.jump.unwrap_or(false), speed_mult: 1.0 };
            let before = body.pos;
            for k in 0..2 {
                let mut d = drive;
                d.jump = d.jump && k == 0;
                crate::physics::step(&shared.world, &cfg.mech, &mut body, &d, 0.05);
                aim = crate::physics::slew_aim(aim, a.aim.unwrap_or(aim), &cfg.mech, 1.0, 0.05);
            }
            if std::env::var("MECH_ZTRACE").is_ok() && tick % 20 == 0 {
                eprintln!("  t={:.0} pos=({:.1},{:.1},{:.1}) g={} dn={:.1}/{:.1} dc={:.1}/{:.1} mv={:?} b={:?} j={:?} us={} | {}", t, body.pos[0], body.pos[1], body.pos[2], body.grounded, dist2v(body.pos, zl.next_center), zl.next_radius, dist2v(body.pos, zl.center), zl.radius, a.move_.map(|m| [(m[0] * 100.0).round() / 100.0, (m[1] * 100.0).round() / 100.0]), a.boost, a.jump, brain.unstick_count, brain.state());
            }
            if std::env::var("MECH_ZDBG").ok().and_then(|s| s.parse::<i64>().ok()) == Some(tick) {
                eprintln!("DBG tick {tick} pos {:?}\n{}", body.pos, brain.dbg);
            }
            if !inside {
                out_s += 0.1;
                if phase + 1 < cfg.zone.phases.len() {
                    early_dmg += zl.dps * 0.1;
                }
                zone_dmg += zl.dps * 0.1;
                hp = (hp - zl.dps * 0.1).max(1.0);
            }
            if dist(before, body.pos) < 0.02 && dist2v(body.pos, zl.next_center) > zl.next_radius {
                still_s += 0.1;
            }
            tick += 2;
        }
        let zl = zone_obs(&z, &cfg, phase, end_t, t0, true);
        eprintln!("run {run} live={live} start=({:.0},{:.0}) outside {:.1}s zone_dmg {:.0} (before last phase {early_dmg:.0}) still_outside_next {:.1}s unstick {} final_dist {:.1} (r {:.1}) worst {:.1}ms", start[0], start[1], out_s, zone_dmg, still_s, brain.unstick_count, dist2v(body.pos, zl.next_center), zl.radius, worst_ms);
        total_early += early_dmg;
        if early_dmg > 100.0 {
            bad_runs += 1;
        }
        total_unstick += brain.unstick_count;
    }
    eprintln!("zone_walk: {runs} runs, {bad_runs} with zone damage before the last phase ({total_early:.0} in total), {total_unstick} unsticks, worst {worst_ms:.1} ms");
    assert!(bad_runs * 10 <= runs, "{bad_runs} of {runs} runs took zone damage early");
}

/// Needs MECH_MAP.
#[test]
fn stress_timing() {
    let Ok(dir) = std::env::var("MECH_MAP") else { return };
    let hello = crate::proto::Hello::default();
    let cfg = hello.config.clone();
    let shared = Arc::new(Shared::load(std::path::Path::new(&dir), &cfg));
    let mut rng = Rng::new(99);
    let mut brain = Brain::with_params(&hello, shared.clone(), Params::default());
    let mut times: Vec<f64> = Vec::new();
    let mut fired = 0;
    let c0 = [60.0, -20.0];
    for i in 0..400i64 {
        let t = i as f64 * 0.1;
        let x = c0[0] + 30.0 * (t * 0.2).cos();
        let y = c0[1] + 30.0 * (t * 0.2).sin();
        let me_pos = shared.nav.snap([x, y, shared.world.terrain_z(x, y)]).unwrap_or([x, y, 0.0]);
        let mut o = crate::proto::Obs::default();
        o.tick = i * 2;
        o.t = t;
        o.alive = 16;
        o.me.pos = me_pos;
        o.me.aim = [t * 0.5, 0.0];
        o.me.max_hp = 1400.0;
        o.me.hp = if i < 200 { 1400.0 } else { 300.0 };
        o.me.energy = 80.0;
        o.me.max_energy = 100.0;
        o.me.grounded = true;
        o.me.in_zone = true;
        o.me.stats = cfg.mech.clone();
        let second = match (i / 100) % 3 {
            0 => Some(crate::proto::WeaponObs { kind: WeaponKind::Laser, charge: if i % 10 > 2 { 0.1 * (i % 10) as f64 } else { 0.0 }, stats: cfg.weapons.laser.clone(), ..Default::default() }),
            1 => Some(crate::proto::WeaponObs { kind: WeaponKind::Mortar, stats: cfg.weapons.mortar.clone(), ..Default::default() }),
            _ => Some(crate::proto::WeaponObs { kind: WeaponKind::Lightning, stats: cfg.weapons.lightning.clone(), ..Default::default() }),
        };
        o.me.weapons = [Some(crate::proto::WeaponObs { kind: WeaponKind::Gun, ammo: 12.0, stats: cfg.weapons.gun.clone(), ..Default::default() }), second];
        for k in 0..15i64 {
            let a = k as f64 * 0.42 + t * 0.05;
            let d = 30.0 + 12.0 * k as f64;
            let (ex, ey) = (me_pos[0] + d * a.cos(), me_pos[1] + d * a.sin());
            let ez = shared.world.terrain_z(ex, ey);
            o.mechs.push(crate::proto::ActorObs { id: k + 1, kind: crate::proto::ActorKind::Mech, pos: [ex, ey, ez], vel: [5.0 * a.sin(), -5.0 * a.cos(), 0.0], aim: [a + PI, 0.0], hp: Some(900.0), max_hp: Some(1400.0), cubes: 0.0, weapons: vec![WeaponKind::Gun, WeaponKind::Laser], charging: k % 3 == 0 });
            for j in 0..3 {
                let s = 0.3 + 0.2 * j as f64;
                o.projectiles.push(crate::proto::ProjectileObs { id: 300 + k * 3 + j, kind: if j == 2 { crate::proto::ProjKind::Mortar } else { crate::proto::ProjKind::Gun }, pos: [ex - s * d * a.cos(), ey - s * d * a.sin(), ez + 7.0], vel: if j == 2 { [-40.0 * a.cos(), -40.0 * a.sin(), 20.0] } else { [-252.0 * a.cos(), -252.0 * a.sin(), 0.0] } });
            }
        }
        for k in 0..12i64 {
            let (px, py) = (me_pos[0] + rng.range(-80.0, 80.0), me_pos[1] + rng.range(-80.0, 80.0));
            let pz = shared.world.terrain_z(px, py);
            o.crates.push(crate::proto::ItemObs { id: 1000 + k, pos: [px, py, pz], hp: Some(300.0) });
            o.cubes.push(crate::proto::ItemObs { id: 5000 + k, pos: [px + 3.0, py, pz], hp: None });
            o.pickups.push(crate::proto::PickupObs { id: 8000 + k, kind: WeaponKind::Laser, pos: [px, py + 3.0, pz] });
        }
        if i % 7 == 0 {
            o.events.push(crate::proto::Event::Damaged(crate::proto::DamagedEv { amount: 45.0, weapon: "gun".into(), bearing: Some(1.0), by: Some(1 + i % 15), rear: false }));
        }
        o.zone = crate::proto::ZoneObs { center: [41.3, -72.8], radius: 380.0, next_center: [50.0, -30.0], next_radius: 240.0, shrink_start: 105.0, shrink_end: 145.0, dps: 35.0 };
        let w0 = std::time::Instant::now();
        let a = brain.act(&o, &Budget::from_now_ms(30.0));
        times.push(w0.elapsed().as_secs_f64() * 1000.0);
        check_action(&a, o.tick);
        if a.fire.map_or(false, |f| f[0] || f[1]) {
            fired += 1;
        }
    }
    times.sort_by(|a, b| a.total_cmp(b));
    eprintln!("stress: p50 {:.2} ms  p90 {:.2}  p99 {:.2}  max {:.2}  fired on {fired}/400", times[200], times[360], times[396], times[399]);
    assert!(times[399] < 40.0);
}
