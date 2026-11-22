use automata::world::{PrismKind, World};
use std::path::PathBuf;
use std::time::Instant;

fn map_dir() -> PathBuf {
    if let Ok(p) = std::env::var("MECH_MAP") {
        return PathBuf::from(p);
    }
    let root = PathBuf::from(env!("CARGO_MANIFEST_DIR"));
    for rel in ["../kit/arena", "../../mechfight-kit/arena"] {
        if root.join(rel).join("v2/world.json").exists() {
            return root.join(rel);
        }
    }
    root.join("../kit/arena")
}

fn load() -> World {
    let dir = map_dir();
    let t = Instant::now();
    let w = World::load(&dir).expect("load");
    eprintln!("load {:.1} ms, prisms {}, trees {}", t.elapsed().as_secs_f64() * 1e3, w.prisms.len(), w.trees.len());
    w
}

struct Rng(u64);
impl Rng {
    fn next(&mut self) -> f64 {
        self.0 ^= self.0 << 13;
        self.0 ^= self.0 >> 7;
        self.0 ^= self.0 << 17;
        (self.0 >> 11) as f64 / (1u64 << 53) as f64
    }
    fn range(&mut self, a: f64, b: f64) -> f64 {
        a + (b - a) * self.next()
    }
}

fn brute_solid(w: &World, x: f64, y: f64, z: f64) -> bool {
    z < w.terrain_z(x, y) || w.prisms.iter().any(|p| z >= p.base && z < p.top && p.contains_xy(x, y))
}

#[test]
fn terrain_vertices() {
    let w = load();
    let dir = map_dir().join("v2/terrain.i16");
    let raw = std::fs::read(dir).unwrap();
    let (x0, y0, nx, ny) = w.terrain_grid();
    let mut r = Rng(7);
    for _ in 0..20000 {
        let i = (r.next() * nx as f64) as usize;
        let j = (r.next() * ny as f64) as usize;
        let k = j * nx + i;
        let v = i16::from_le_bytes([raw[2 * k], raw[2 * k + 1]]) as f64 * 0.01;
        let z = w.terrain_z(x0 + i as f64, y0 + j as f64);
        assert!((z - v).abs() < 1e-5, "{i} {j} {z} {v}");
    }
}

#[test]
fn points_vs_brute() {
    let w = load();
    let mut r = Rng(11);
    let mut n_in = 0;
    let mut bad = 0;
    for _ in 0..200000 {
        // half near a random prism
        let (x, y) = if r.next() < 0.7 {
            let p = &w.prisms[(r.next() * w.prisms.len() as f64) as usize];
            (r.range(p.min_x - 3.0, p.max_x + 3.0), r.range(p.min_y - 3.0, p.max_y + 3.0))
        } else {
            (r.range(w.bounds.min_x, w.bounds.max_x), r.range(w.bounds.min_y, w.bounds.max_y))
        };
        let z = w.terrain_z(x, y) + r.range(-1.0, 40.0);
        let a = w.solid(x, y, z);
        let b = brute_solid(&w, x, y, z);
        if a != b {
            bad += 1;
            if bad < 4 {
                let mut cols = Vec::new();
                w.columns(x, y, &mut cols);
                let br: Vec<_> = w.prisms.iter().enumerate().filter(|(_, p)| p.contains_xy(x, y)).map(|(i, p)| (i, p.base, p.top, p.kind)).collect();
                eprintln!("solid mismatch x={x} y={y} z={z} fast={a} cols={cols:?} brute={br:?}");
            }
        }
        let mut cols = Vec::new();
        w.columns(x, y, &mut cols);
        let nb = w.prisms.iter().filter(|p| p.contains_xy(x, y)).count();
        if cols.len() != nb {
            bad += 1;
        }
        if nb > 0 {
            n_in += 1;
        }
        // floor_below brute
        let mut fb = w.terrain_z(x, y);
        for p in &w.prisms {
            if p.top <= z + 1e-6 && p.top > fb && p.contains_xy(x, y) {
                fb = p.top;
            }
        }
        if (fb - w.floor_below(x, y, z)).abs() > 1e-9 {
            bad += 1;
        }
    }
    eprintln!("points: {n_in} inside some footprint, {bad} mismatches");
    assert!(bad == 0);
}

#[test]
fn roof_and_bridge() {
    let w = load();
    // a building: pick the highest roof, find an interior point
    let mut r = Rng(3);
    let mut checked = 0;
    for (pi, p) in w.prisms.iter().enumerate() {
        if p.kind != PrismKind::Building {
            continue;
        }
        for _ in 0..50 {
            let x = r.range(p.min_x, p.max_x);
            let y = r.range(p.min_y, p.max_y);
            if !p.contains_xy(x, y) {
                continue;
            }
            let mut on_top = true;
            w.prisms_near(x, y, 0.0, |qi, q| if qi != pi && q.top > p.top && q.base <= p.top + 9.0 { on_top = false });
            if !on_top {
                break;
            }
            assert!((w.floor_below(x, y, p.top + 5.0) - p.top).abs() < 1e-9);
            assert!((w.floor_below(x, y, p.top) - p.top).abs() < 1e-9);
            assert!(w.solid(x, y, p.top - 0.1));
            assert!(!w.solid(x, y, p.top + 0.01));
            checked += 1;
            break;
        }
    }
    eprintln!("roofs checked {checked}");
    assert!(checked > 100);
    // bridges with clearance: shoot horizontally under
    let mut under = 0;
    for p in w.prisms.iter().filter(|p| p.kind == PrismKind::Bridge) {
        let cx = 0.5 * (p.min_x + p.max_x);
        let cy = 0.5 * (p.min_y + p.max_y);
        if !p.contains_xy(cx, cy) {
            continue;
        }
        let g = w.terrain_z(cx, cy);
        if p.base - g < 4.0 {
            continue;
        }
        let z = 0.5 * (g + p.base);
        assert!((w.ceiling_above(cx, cy, z) - p.base).abs() < 1e-9);
        assert!(w.solid(cx, cy, p.base + 0.01));
        // vertical ray up from under hits the base
        let t = w.ray([cx, cy, z], [cx, cy, z + 100.0]).expect("hit underside");
        assert!((z + 100.0 * t - p.base).abs() < 1e-6);
        // horizontal ray along the long axis at z, short range: check against brute
        let len = 40.0;
        for k in 0..8 {
            let ang = k as f64 * std::f64::consts::PI / 8.0;
            let a = [cx - ang.cos() * len / 2.0, cy - ang.sin() * len / 2.0, z];
            let b = [cx + ang.cos() * len / 2.0, cy + ang.sin() * len / 2.0, z];
            if brute_ray(&w, a, b, 0.05).is_none() {
                assert!(w.ray(a, b).is_none());
                under += 1;
            }
        }
    }
    eprintln!("bridge under-rays passing {under}");
    assert!(under > 0);
}

fn brute_ray(w: &World, a: [f64; 3], b: [f64; 3], step: f64) -> Option<f64> {
    let d = [b[0] - a[0], b[1] - a[1], b[2] - a[2]];
    let len = (d[0] * d[0] + d[1] * d[1] + d[2] * d[2]).sqrt();
    let n = (len / step).ceil().max(1.0) as usize;
    for k in 0..=n {
        let t = k as f64 / n as f64;
        if brute_solid(w, a[0] + d[0] * t, a[1] + d[1] * t, a[2] + d[2] * t) {
            return Some(t);
        }
    }
    None
}

fn rand_seg(w: &World, r: &mut Rng) -> ([f64; 3], [f64; 3]) {
    let x = r.range(w.bounds.min_x, w.bounds.max_x);
    let y = r.range(w.bounds.min_y, w.bounds.max_y);
    let z = w.floor_below(x, y, 200.0) + r.range(0.5, 12.0);
    let len = r.range(5.0, 200.0);
    let ang = r.range(0.0, 6.2832);
    let bx = (x + ang.cos() * len).clamp(w.bounds.min_x, w.bounds.max_x);
    let by = (y + ang.sin() * len).clamp(w.bounds.min_y, w.bounds.max_y);
    let bz = w.floor_below(bx, by, 200.0) + r.range(0.5, 12.0);
    ([x, y, z], [bx, by, bz])
}

#[test]
fn ray_vs_brute() {
    let w = load();
    let mut r = Rng(12345);
    let n = 20000;
    let (mut agree, mut miss_ray, mut miss_brute, mut tdiff, mut hits, mut starts) = (0, 0, 0, 0, 0, 0);
    let mut worst = 0.0f64;
    let mut unconfirmed = 0;
    for _ in 0..n {
        let (a, b) = rand_seg(&w, &mut r);
        let len = ((b[0] - a[0]).powi(2) + (b[1] - a[1]).powi(2) + (b[2] - a[2]).powi(2)).sqrt();
        let fast = w.ray(a, b);
        let slow = brute_ray(&w, a, b, 0.05);
        if slow == Some(0.0) {
            starts += 1;
        }
        match (fast, slow) {
            (None, None) => agree += 1,
            (Some(tf), Some(ts)) => {
                hits += 1;
                let e = (ts - tf) * len;
                if e >= -1e-6 && e <= 0.05 + 1e-6 {
                    agree += 1;
                } else {
                    tdiff += 1;
                    worst = worst.max(e.abs());
                    let fine = fine_check(&w, a, b, tf, len);
                    if !fine {
                        eprintln!("tdiff NOT CONFIRMED by fine sampling a={a:?} b={b:?} fast={tf} slow={ts} err={e}");
                        unconfirmed += 1;
                    }
                }
            }
            (Some(tf), None) => {
                miss_brute += 1;
                if !fine_check(&w, a, b, tf, len) {
                    eprintln!("ray-only hit NOT CONFIRMED a={a:?} b={b:?} t={tf}");
                    unconfirmed += 1;
                }
            }
            (None, Some(ts)) => {
                miss_ray += 1;
                if miss_ray < 10 {
                    eprintln!("RAY MISSED a={a:?} b={b:?} slow={ts}");
                }
            }
        }
    }
    eprintln!("ray vs brute: n={n} agree={agree} hits={hits} start_inside={starts} t_mismatch={tdiff} (worst {worst:.3} m) ray_only={miss_brute} brute_only={miss_ray}");
    eprintln!("early/ray-only hits not confirmed by 0.1 mm sampling: {unconfirmed}");
    assert!(miss_ray == 0);
}

/// solid() somewhere within 5 cm after t (0.1 mm steps): the hit is real but thinner than the brute step
fn fine_check(w: &World, a: [f64; 3], b: [f64; 3], t: f64, len: f64) -> bool {
    let d = [b[0] - a[0], b[1] - a[1], b[2] - a[2]];
    (0..=500).any(|k| {
        let tt = (t + k as f64 * 0.0001 / len).min(1.0);
        brute_solid(w, a[0] + d[0] * tt, a[1] + d[1] * tt, a[2] + d[2] * tt)
    })
}

#[test]
fn bench() {
    let w = load();
    let mut r = Rng(99);
    let pts: Vec<[f64; 3]> = (0..100000)
        .map(|_| {
            let x = r.range(w.bounds.min_x, w.bounds.max_x);
            let y = r.range(w.bounds.min_y, w.bounds.max_y);
            [x, y, w.terrain_z(x, y) + r.range(-1.0, 30.0)]
        })
        .collect();
    let mut acc = 0.0;
    let t = Instant::now();
    for p in &pts {
        acc += w.floor_below(p[0], p[1], p[2]);
    }
    let fb = t.elapsed().as_secs_f64() * 1e9 / pts.len() as f64;
    let t = Instant::now();
    let mut ns = 0;
    for p in &pts {
        ns += w.solid(p[0], p[1], p[2]) as usize;
    }
    let so = t.elapsed().as_secs_f64() * 1e9 / pts.len() as f64;
    let t = Instant::now();
    for p in &pts {
        acc += w.ceiling_above(p[0], p[1], p[2]).min(1e3);
    }
    let ce = t.elapsed().as_secs_f64() * 1e9 / pts.len() as f64;
    let t = Instant::now();
    for p in &pts {
        acc += w.terrain_z(p[0], p[1]);
    }
    let tz = t.elapsed().as_secs_f64() * 1e9 / pts.len() as f64;
    // rays of exactly 200 m horizontal-ish at mech eye height
    let segs: Vec<([f64; 3], [f64; 3])> = (0..20000)
        .map(|_| {
            let x = r.range(-300.0, 300.0);
            let y = r.range(-200.0, 200.0);
            let ang = r.range(0.0, 6.2832);
            let z = w.floor_below(x, y, 200.0) + 8.0;
            let bx = x + ang.cos() * 200.0;
            let by = y + ang.sin() * 200.0;
            ([x, y, z], [bx, by, w.floor_below(bx, by, 200.0) + 4.5])
        })
        .collect();
    let t = Instant::now();
    let mut nh = 0;
    for (a, b) in &segs {
        nh += w.ray(*a, *b).is_some() as usize;
    }
    let ry = t.elapsed().as_secs_f64() * 1e9 / segs.len() as f64;
    let t = Instant::now();
    let mut nsg = 0;
    for (a, b) in &segs {
        nsg += w.sight(*a, *b) as usize;
    }
    let si = t.elapsed().as_secs_f64() * 1e9 / segs.len() as f64;
    // open-field rays: only the ones that are clear (worst case: full traversal)
    let clear: Vec<_> = segs.iter().filter(|(a, b)| w.ray(*a, *b).is_none()).cloned().collect();
    let t = Instant::now();
    for (a, b) in &clear {
        nh += w.ray(*a, *b).is_some() as usize;
    }
    let ryc = t.elapsed().as_secs_f64() * 1e9 / clear.len().max(1) as f64;
    let t = Instant::now();
    let mut ncl = 0.0;
    for (a, b) in &clear {
        ncl += w.canopy_len(*a, *b);
        nsg += w.sight(*a, *b) as usize;
    }
    let sic = t.elapsed().as_secs_f64() * 1e9 / clear.len().max(1) as f64;
    let t = Instant::now();
    let mut np = 0;
    for p in pts.iter().take(20000) {
        w.prisms_near(p[0], p[1], 30.0, |_, _| np += 1);
    }
    let pn = t.elapsed().as_secs_f64() * 1e9 / 20000.0;
    eprintln!(
        "ns/call: terrain_z {tz:.1} floor_below {fb:.1} solid {so:.1} ceiling_above {ce:.1} | ray200 {ry:.0} (hit {:.0}%) ray200_clear {ryc:.0} | sight200 {si:.0} sight+canopy_len clear {sic:.0} (blocked by trees {:.0}%) | prisms_near r30 {pn:.0} ({:.1} avg) [{acc:.0} {ns} {ncl:.0}]",
        100.0 * nh as f64 / segs.len() as f64 / 1.0,
        0.0,
        np as f64 / 20000.0
    );
}
