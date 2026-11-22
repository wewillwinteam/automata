use super::*;
use std::time::Instant;

struct Rng(u64);
impl Rng {
    fn next(&mut self) -> u64 {
        self.0 ^= self.0 << 13;
        self.0 ^= self.0 >> 7;
        self.0 ^= self.0 << 17;
        self.0
    }
    fn below(&mut self, n: usize) -> usize {
        (self.next() % n as u64) as usize
    }
}

fn map_dir() -> Option<std::path::PathBuf> {
    let mut c = vec![];
    if let Ok(p) = std::env::var("NAV_MAP") {
        c.push(std::path::PathBuf::from(p));
    }
    c.push(std::path::Path::new(env!("CARGO_MANIFEST_DIR")).join("../kit/arena"));
    c.push(std::path::Path::new(env!("CARGO_MANIFEST_DIR")).join("../../mechfight-kit/arena"));
    if let Ok(p) = std::env::var("MECH_MAP") {
        c.insert(0, std::path::PathBuf::from(p));
    }
    c.push(std::path::PathBuf::from("/arena"));
    c.into_iter().find(|p| p.join("v2/world.json").exists())
}

fn reach(nav: &Nav, root: u32, jet: f64, reverse: bool) -> Vec<bool> {
    let mut seen = vec![false; nav.node_count()];
    let mut st = vec![root];
    seen[root as usize] = true;
    while let Some(v) = st.pop() {
        let es = if reverse { nav.edges_in(v) } else { nav.edges(v) };
        for e in es {
            if e.kind == EDGE_JET && e.energy as f64 > jet {
                continue;
            }
            if !seen[e.to as usize] {
                seen[e.to as usize] = true;
                st.push(e.to);
            }
        }
    }
    seen
}

fn scc(nav: &Nav, root: u32, jet: f64) -> Vec<bool> {
    let a = reach(nav, root, jet, false);
    let b = reach(nav, root, jet, true);
    a.iter().zip(b.iter()).map(|(x, y)| *x && *y).collect()
}

#[test]
fn flat_world() {
    let w = world::World::empty();
    let m = proto::MechCfg::default();
    let nav = Nav::build_rect(&w, &m, [-50.0, -50.0, 50.0, 50.0]);
    assert!(nav.node_count() > 2000);
    let f = nav.field([20.0, 0.0, 0.0]);
    let d = nav.dist(&f, [-20.0, 0.0, 0.0]).unwrap();
    assert!((d - 40.0).abs() < 0.5, "{d}");
    let s = nav.dir(&f, [-20.0, 1.0, 0.0]).unwrap();
    assert!(s[0] > 0.99, "{s:?}");
    assert!(nav.clear([-20.0, 0.0, 0.0], [20.0, 10.0, 0.0]));
}

#[test]
fn real_map() {
    let Some(dir) = map_dir() else {
        eprintln!("map not found, skipped");
        return;
    };
    let t0 = Instant::now();
    let w = world::World::load(&dir).expect("world");
    let t_world = t0.elapsed().as_secs_f64() * 1e3;
    let m = proto::MechCfg::default();
    let t0 = Instant::now();
    let nav = Nav::build(&w, &m);
    let t_build = t0.elapsed().as_secs_f64() * 1e3;
    let n = nav.node_count();
    let mut kinds = [0usize; 3];
    for i in 0..n {
        for e in nav.edges(i as u32) {
            kinds[e.kind as usize] += 1;
        }
    }
    let roof = (0..n).filter(|&i| nav.node_roof(i as u32)).count();
    let root = nav.snap_node([0.0, 0.0, 0.0]).unwrap();
    let main = scc(&nav, root, -1.0);
    let mainj = scc(&nav, root, m.energy);
    let ground = n - roof;
    let g_main = (0..n).filter(|&i| !nav.node_roof(i as u32) && main[i]).count();
    let r_main = (0..n).filter(|&i| nav.node_roof(i as u32) && main[i]).count();
    let r_mainj = (0..n).filter(|&i| nav.node_roof(i as u32) && mainj[i]).count();
    let g_mainj = (0..n).filter(|&i| !nav.node_roof(i as u32) && mainj[i]).count();
    eprintln!("world load {t_world:.0} ms, nav build {t_build:.0} ms, nodes {n} (ground {ground}, roof {roof}), edges walk {} drop {} jet {}", kinds[0], kinds[1], kinds[2]);
    eprintln!("main SCC walk: ground {g_main} ({:.2}%), roof {r_main}; with jets: ground {g_mainj} ({:.2}%), roof {r_mainj} ({:.2}% of roof)",
        100.0 * g_main as f64 / ground as f64, 100.0 * g_mainj as f64 / ground as f64, 100.0 * r_mainj as f64 / roof.max(1) as f64);

    if let Ok(p) = std::env::var("NAV_DUMP") {
        let (cnx, cny, o) = nav.grid();
        let mut out = format!("{cnx} {cny} {} {} {}\n", o[0], o[1], CELL).into_bytes();
        for cj in 0..cny {
            for ci in 0..cnx {
                let mut c = 0u8;
                let mut zt = f32::NEG_INFINITY;
                for nd in nav.cell_node_ids(ci, cj) {
                    let z = nav.node_pos(nd)[2] as f32;
                    if z < zt { continue; }
                    zt = z;
                    let r = nav.node_roof(nd);
                    c = match (r, main[nd as usize], mainj[nd as usize]) {
                        (false, true, _) => 1,
                        (true, true, _) => 2,
                        (true, false, true) => 3,
                        (false, false, true) => 6,
                        (false, false, false) => 4,
                        (true, false, false) => 5,
                    };
                }
                out.push(c);
            }
        }
        std::fs::write(p, out).unwrap();
    }

    // random pairs among main-SCC nodes and among all ground nodes
    let mains: Vec<u32> = (0..n as u32).filter(|&i| main[i as usize]).collect();
    let grounds: Vec<u32> = (0..n as u32).filter(|&i| !nav.node_roof(i)).collect();
    let mut rng = Rng(0x9e3779b97f4a7c15);
    let pairs = 1000;
    let (mut reach_g, mut reach_gj, mut ratios, mut tf, mut tfj) = (0, 0, vec![], 0.0, 0.0);
    for _ in 0..pairs {
        let a = grounds[rng.below(grounds.len())];
        let b = grounds[rng.below(grounds.len())];
        let (pa, pb) = (nav.node_pos(a), nav.node_pos(b));
        let t = Instant::now();
        let f = nav.field(pb);
        tf += t.elapsed().as_secs_f64();
        if let Some(d) = nav.dist(&f, pa) {
            reach_g += 1;
            let s = hyp(pa, pb);
            if s > 20.0 {
                ratios.push(d / s);
            }
        }
        if reach_g % 5 == 0 {
            let t = Instant::now();
            let fj = nav.field_with(pb, m.energy);
            tfj += t.elapsed().as_secs_f64();
            if nav.dist(&fj, pa).is_some() {
                reach_gj += 1;
            }
        }
    }
    ratios.sort_by(|a, b| a.partial_cmp(b).unwrap());
    let q = |p: f64| ratios[((ratios.len() - 1) as f64 * p) as usize];
    eprintln!("ground pairs reachable walk {reach_g}/{pairs}; field {:.2} ms avg; jet field {:.2} ms avg; path/straight median {:.3} p90 {:.3} p99 {:.3} max {:.3}",
        tf * 1e3 / pairs as f64, tfj * 1e3 / (pairs / 5).max(1) as f64, q(0.5), q(0.9), q(0.99), ratios[ratios.len() - 1]);
    let _ = reach_gj;

    // follow dir() in 1 m steps
    let trials = 300;
    let (mut ok, mut stuck, mut offgraph, mut osc_total, mut steps_total, mut tstep, mut nstep) = (0, 0, 0, 0usize, 0usize, 0.0, 0usize);
    let mut excess = vec![];
    for _ in 0..trials {
        let a = mains[rng.below(mains.len())];
        let b = mains[rng.below(mains.len())];
        let (mut p, pb) = (nav.node_pos(a), nav.node_pos(b));
        let f = nav.field(pb);
        let d0 = nav.dist(&f, p).unwrap();
        let mut walked = 0.0;
        let mut prev: Option<[f64; 2]> = None;
        let mut best = d0;
        let mut since = 0;
        let mut done = false;
        let mut failed_off = false;
        for _ in 0..((d0 * 3.0) as usize + 100) {
            let t = Instant::now();
            let s = nav.step(&f, p).unwrap();
            tstep += t.elapsed().as_secs_f64();
            nstep += 1;
            if s.dist < 1.5 || hyp(p, pb) < 1.5 {
                done = true;
                break;
            }
            let stepl = 1.0f64.min(hyp(p, s.target).max(0.3));
            if let Some(pd) = prev {
                if pd[0] * s.dir[0] + pd[1] * s.dir[1] < -0.5 {
                    osc_total += 1;
                }
            }
            prev = Some(s.dir);
            let mut np = [p[0] + s.dir[0] * stepl, p[1] + s.dir[1] * stepl, p[2]];
            np[2] = w.floor_below(np[0], np[1], p[2] + m.step_up);
            let sn = nav.snap_node(np).unwrap();
            let q = nav.node_pos(sn);
            if s.kind != EDGE_DROP && (hyp(q, np) > 1.5 || np[2] > q[2] as f64 + m.step_up + 0.5) {
                failed_off = true;
                if offgraph < 6 { eprintln!("off: p {:?} np {:?} q {:?} kind {} target {:?} roof {}", p, np, q, s.kind, s.target, nav.node_roof(sn)); }
                break;
            }
            p = np;
            walked += stepl;
            steps_total += 1;
            let dn = nav.dist(&f, p).unwrap_or(f64::INFINITY);
            if dn < best - 0.2 {
                best = dn;
                since = 0;
            } else {
                since += 1;
                if since > 40 {
                    break;
                }
            }
        }
        if done {
            ok += 1;
            excess.push(walked / d0.max(1.0));
        } else if failed_off {
            offgraph += 1;
        } else {
            stuck += 1;
        }
    }
    excess.sort_by(|a, b| a.partial_cmp(b).unwrap());
    eprintln!("follow: ok {ok}/{trials}, stuck {stuck}, off-graph {offgraph}, direction reversals {osc_total} in {steps_total} steps; walked/field-dist median {:.3} max {:.3}; step() {:.1} us avg",
        excess[excess.len() / 2], excess[excess.len() - 1], tstep * 1e6 / nstep as f64);

    // snap and dist timing, snap of off-graph positions
    let t = Instant::now();
    let f = nav.field([0.0, 0.0, 0.0]);
    let mut s = 0.0;
    for k in 0..100000 {
        let a = mains[k % mains.len()];
        let p = nav.node_pos(a);
        s += nav.dist(&f, [p[0] + 0.7, p[1] - 0.4, p[2] + 3.0]).unwrap_or(0.0);
    }
    eprintln!("dist() incl. snap {:.2} us avg ({s:.0})", t.elapsed().as_secs_f64() * 1e6 / 1e5 - 0.0);
    assert!(g_main as f64 > 0.9 * ground as f64);
    assert!(ok as f64 >= 0.97 * trials as f64);
}

#[test]
fn bench_field() {
    let Some(dir) = map_dir() else { return };
    let w = world::World::load(&dir).expect("world");
    let m = proto::MechCfg::default();
    let mut bt = vec![];
    let mut nav = Nav::build(&w, &m);
    for _ in 0..3 {
        let t = Instant::now();
        nav = Nav::build(&w, &m);
        bt.push(t.elapsed().as_secs_f64() * 1e3);
    }
    let mut rng = Rng(12345);
    let mut ts = vec![];
    let mut tj = vec![];
    for k in 0..60 {
        let g = nav.node_pos(rng.below(nav.node_count()) as u32);
        let t = Instant::now();
        let f = if k % 2 == 0 { nav.field(g) } else { nav.field_with(g, m.energy) };
        let e = t.elapsed().as_secs_f64() * 1e3;
        if k % 2 == 0 { ts.push(e) } else { tj.push(e) }
        std::hint::black_box(&f);
    }
    ts.sort_by(|a, b| a.partial_cmp(b).unwrap());
    tj.sort_by(|a, b| a.partial_cmp(b).unwrap());
    eprintln!("BENCH build {bt:.0?} ms; field min {:.2} med {:.2} max {:.2} ms; jet field min {:.2} med {:.2} max {:.2} ms", ts[0], ts[15], ts[29], tj[0], tj[15], tj[29]);
}
