//! Grid navigation graph over standable surfaces.

use crate::{proto, world};

/// Cell size of the nav grid in metres.
pub const CELL: f64 = 2.0;
/// Map rectangle [min_x, min_y, max_x, max_y] used when no other rect is given.
pub const MAP_RECT: [f64; 4] = [-512.38, -333.09, 555.08, 388.64];

pub const EDGE_WALK: u8 = 0;
pub const EDGE_DROP: u8 = 1;
pub const EDGE_JET: u8 = 2;

const INF: u32 = u32::MAX;
const SAMPLE: f64 = 1.0; // gap sample spacing; CELL must be an integer multiple
const K: usize = 2; // CELL / SAMPLE
const LONG: i32 = 3; // reach of drop and jet edges, in cells
const LOOKAHEAD: usize = 12;
const PACK_COST_BITS: u32 = 12;

#[derive(Clone, Copy, Debug)]
struct Gap {
    s: f32,
    c: f32,
    ter: bool,
}

/// Directed edge. In the reverse list, to is the source node.
#[derive(Clone, Copy, Debug)]
pub struct Edge {
    pub to: u32,
    /// decimetres of equivalent walking distance
    pub cost: u16,
    /// energy needed (jet edges)
    pub energy: u8,
    pub kind: u8,
}

pub struct Nav {
    ox: f64,
    oy: f64,
    cnx: usize,
    cny: usize,
    // gap samples
    snx: usize,
    sny: usize,
    pad: usize,
    gap_first: Vec<u32>,
    gaps: Vec<Gap>,
    cell_first: Vec<u32>,
    node_cell: Vec<u32>,
    node_z: Vec<f32>,
    node_roof: Vec<bool>,
    fwd_first: Vec<u32>,
    fwd: Vec<Edge>,
    rev_first: Vec<u32>,
    rev: Vec<Edge>,
    // packed reverse walk/drop edges (from << PACK_COST_BITS | cost) and reverse jet edges
    rp_first: Vec<u32>,
    rp: Vec<u32>,
    rj_first: Vec<u32>,
    rj: Vec<Edge>,
    max_cost: u32,
    pub walk_speed: f64,
    pub step_up: f64,
    pub height: f64,
}

/// Distance-to-goal field over the nav graph.
pub struct Field {
    goal: [f64; 3],
    goal_node: Option<u32>,
    d: Vec<u32>,
    jet: f64,
}

impl Field {
    pub fn goal(&self) -> [f64; 3] {
        self.goal
    }
    pub fn goal_node(&self) -> Option<u32> {
        self.goal_node
    }
    /// Max jet energy this field allows per jet edge (negative: no jet edges).
    pub fn jet_limit(&self) -> f64 {
        self.jet
    }
    /// Distance in metres from node i to the goal.
    pub fn node_dist(&self, i: u32) -> Option<f64> {
        match self.d.get(i as usize) {
            Some(&v) if v != INF => Some(v as f64 * 0.1),
            _ => None,
        }
    }
}

/// One step of guidance from Nav::step.
#[derive(Clone, Copy, Debug)]
pub struct Step {
    /// unit horizontal direction (zero at the goal)
    pub dir: [f64; 2],
    /// point the direction aims at (feet position)
    pub target: [f64; 3],
    /// EDGE_WALK, EDGE_DROP or EDGE_JET: what the next move is
    pub kind: u8,
    /// remaining distance in metres
    pub dist: f64,
    /// energy the next edge needs (jet), else 0
    pub energy: f64,
}

fn hyp(a: [f64; 3], b: [f64; 3]) -> f64 {
    (a[0] - b[0]).hypot(a[1] - b[1])
}

/// (time s, energy) for jump+jet to rise dz then cross dh horizontally.
fn jet_cost(m: &proto::MechCfg, dz: f64, dh: f64) -> Option<(f64, f64)> {
    let dt = 0.05;
    let (mut z, mut vz, mut t, mut e) = (0.0f64, m.jump_speed.max(0.0), 0.0f64, 0.0f64);
    let need = dz + 0.5;
    while z < need {
        vz = (vz + m.jet_accel * dt).min(m.max_rise_speed);
        vz -= m.gravity * dt;
        z += vz * dt;
        t += dt;
        e += m.jet_drain * dt;
        if t > 30.0 || (vz <= 0.0 && t > 1.0) {
            return None;
        }
    }
    let th = 2.0 * (dh.max(0.5) / m.air_accel.max(0.5)).sqrt();
    let time = t + 0.6 * th + 0.2;
    let energy = e + m.jet_drain * 0.4 * th;
    Some((time, energy))
}

impl Nav {
    pub fn build(world: &world::World, mech: &proto::MechCfg) -> Nav {
        let b = &world.bounds;
        let rect = if b.max_x - b.min_x < 1.0e5 && b.max_y - b.min_y < 1.0e5 && b.max_x > b.min_x && b.max_y > b.min_y {
            [b.min_x, b.min_y, b.max_x, b.max_y]
        } else {
            MAP_RECT
        };
        Self::build_rect(world, mech, rect)
    }

    pub fn build_rect(world: &world::World, mech: &proto::MechCfg, rect: [f64; 4]) -> Nav {
        let rad = mech.radius.max(0.5) + 0.5;
        let height = mech.height.max(1.0);
        let step_up = mech.step_up.max(0.0);
        let tan = mech.max_slope_deg.clamp(0.0, 89.0).to_radians().tan();
        let pad = rad.ceil() as usize + 1;
        let ox = ((rect[0] + rad) / CELL).ceil() * CELL;
        let oy = ((rect[1] + rad) / CELL).ceil() * CELL;
        let cnx = (((rect[2] - rad - ox) / CELL).floor().max(0.0)) as usize + 1;
        let cny = (((rect[3] - rad - oy) / CELL).floor().max(0.0)) as usize + 1;
        let sx0 = ox - pad as f64 * SAMPLE;
        let sy0 = oy - pad as f64 * SAMPLE;
        let snx = (cnx - 1) * K + 2 * pad + 1;
        let sny = (cny - 1) * K + 2 * pad + 1;

        // 1. free gaps per sample column
        let mut gap_first = Vec::with_capacity(snx * sny + 1);
        let mut gaps: Vec<Gap> = Vec::with_capacity(snx * sny + snx * sny / 8);
        for j in 0..sny {
            for i in 0..snx {
                gap_first.push(gaps.len() as u32);
                let x = sx0 + i as f64 * SAMPLE;
                let y = sy0 + j as f64 * SAMPLE;
                if x < rect[0] || x > rect[2] || y < rect[1] || y > rect[3] {
                    continue;
                }
                let t = world.terrain_z(x, y);
                let start = gaps.len();
                let mut z = 1.0e4;
                for _ in 0..16 {
                    let s = world.floor_below(x, y, z);
                    if !s.is_finite() {
                        break;
                    }
                    let ter = s <= t + 1e-3;
                    if !world.solid(x, y, s + 0.05) {
                        let c = world.ceiling_above(x, y, s);
                        gaps.push(Gap { s: s as f32, c: c.min(1.0e6) as f32, ter });
                    }
                    if ter {
                        break;
                    }
                    z = s - 0.02;
                }
                gaps[start..].reverse(); // ascending s
            }
        }
        gap_first.push(gaps.len() as u32);

        let mut nav = Nav {
            ox, oy, cnx, cny, snx, sny, pad,
            gap_first, gaps,
            cell_first: Vec::with_capacity(cnx * cny + 1),
            node_cell: vec![], node_z: vec![], node_roof: vec![],
            fwd_first: vec![], fwd: vec![], rev_first: vec![], rev: vec![],
            rp_first: vec![], rp: vec![], rj_first: vec![], rj: vec![],
            max_cost: 1,
            walk_speed: mech.walk_speed.max(0.1),
            step_up, height,
        };

        // disk of sample offsets
        let r_i = rad.ceil() as i32;
        let mut disk: Vec<(i32, i32, f32)> = vec![];
        for dj in -r_i..=r_i {
            for di in -r_i..=r_i {
                let d = ((di * di + dj * dj) as f64).sqrt() * SAMPLE;
                if d <= rad + 1e-9 {
                    disk.push((di, dj, d as f32));
                }
            }
        }
        let (step_f, h_f, tan_f) = (step_up as f32, height as f32, tan as f32);

        // 2. nodes
        let (mut cf, mut ncell, mut nz, mut nroof) = (Vec::with_capacity(cnx * cny + 1), vec![], vec![], vec![]);
        for cj in 0..cny {
            for ci in 0..cnx {
                cf.push(nz.len() as u32);
                let (si, sj) = (ci * K + pad, cj * K + pad);
                let cg = nav.col(si, sj);
                for g in cg {
                    let z = g.s;
                    if g.c - z < h_f {
                        continue;
                    }
                    let ok = disk.iter().all(|&(di, dj, d)| {
                        let q = nav.col((si as i32 + di) as usize, (sj as i32 + dj) as usize);
                        q.iter().any(|h| {
                            h.c >= z + h_f - 0.01
                                && (h.s <= z + step_f || (h.ter && h.s <= z + step_f + d * tan_f))
                        })
                    });
                    if ok {
                        ncell.push((cj * cnx + ci) as u32);
                        nz.push(z);
                        nroof.push(!g.ter);
                    }
                }
            }
        }
        cf.push(nz.len() as u32);
        nav.cell_first = cf;
        nav.node_cell = ncell;
        nav.node_z = nz;
        nav.node_roof = nroof;
        let n = nav.node_z.len();

        // 3. edges
        let mut edges: Vec<(u32, Edge)> = Vec::with_capacity(n * 9);
        let mut tmp: Vec<Edge> = vec![];
        for a in 0..n {
            tmp.clear();
            let (ci, cj) = nav.cell_ij(a as u32);
            let za = nav.node_z[a];
            let (sa_i, sa_j) = (ci * K + nav.pad, cj * K + nav.pad);
            // walk edges
            for dj in -1i32..=1 {
                for di in -1i32..=1 {
                    if di == 0 && dj == 0 {
                        continue;
                    }
                    let (bi, bj) = (ci as i32 + di, cj as i32 + dj);
                    if bi < 0 || bj < 0 || bi >= cnx as i32 || bj >= cny as i32 {
                        continue;
                    }
                    let mut z = za;
                    let mut ok = true;
                    for k in 1..=K as i32 {
                        let q = nav.col((sa_i as i32 + di * k) as usize, (sa_j as i32 + dj * k) as usize);
                        match q.iter().filter(|h| h.s <= z + step_f + 1e-3 && h.c >= h.s + h_f).map(|h| h.s).fold(None, |m: Option<f32>, s| Some(m.map_or(s, |m| m.max(s)))) {
                            Some(s) => z = s,
                            None => { ok = false; break; }
                        }
                    }
                    if !ok {
                        continue;
                    }
                    let bcell = bj as usize * cnx + bi as usize;
                    let len = CELL * ((di * di + dj * dj) as f64).sqrt();
                    for b in nav.cell_nodes(bcell) {
                        let zb = nav.node_z[b as usize];
                        if (zb - z).abs() < 0.05 {
                            let rise = (zb - za) as f64;
                            if rise > step_up.max(len * tan) + 1e-6 {
                                continue;
                            }
                            tmp.push(Edge { to: b, cost: (len * 10.0).round() as u16, energy: 0, kind: EDGE_WALK });
                        }
                    }
                }
            }
            // long edges: drops and jets
            let jet_col_top: f32 = {
                // lowest ceiling over the body column at a (jet room)
                let mut top = f32::INFINITY;
                for &(di, dj, d) in &disk {
                    if d > mech.radius as f32 + 0.01 {
                        continue;
                    }
                    let q = nav.col((sa_i as i32 + di) as usize, (sa_j as i32 + dj) as usize);
                    let c = q.iter().filter(|h| h.c >= za + h_f - 0.01 && (h.s <= za + step_f || (h.ter && h.s <= za + step_f + d * tan_f))).map(|h| h.c).fold(0.0f32, f32::max);
                    top = top.min(c);
                }
                top
            };
            for dj in -LONG..=LONG {
                for di in -LONG..=LONG {
                    if di == 0 && dj == 0 {
                        continue;
                    }
                    let (bi, bj) = (ci as i32 + di, cj as i32 + dj);
                    if bi < 0 || bj < 0 || bi >= cnx as i32 || bj >= cny as i32 {
                        continue;
                    }
                    let dh = CELL * ((di * di + dj * dj) as f64).sqrt();
                    if dh > LONG as f64 * CELL + 0.1 {
                        continue;
                    }
                    let bcell = bj as usize * cnx + bi as usize;
                    for b in nav.cell_nodes(bcell) {
                        if tmp.iter().any(|e| e.to == b) {
                            continue;
                        }
                        let zb = nav.node_z[b as usize];
                        let ns = ((dh / (0.5 * SAMPLE)).ceil() as i32).max(2);
                        if zb < za - step_f - 0.3 {
                            // drop: cross at roof height, then fall at b
                            let lo = za + 0.3;
                            let mut ok = true;
                            for k in 1..ns {
                                let f = k as f64 / ns as f64;
                                let si = (sa_i as f64 + di as f64 * K as f64 * f).round() as usize;
                                let sj = (sa_j as f64 + dj as f64 * K as f64 * f).round() as usize;
                                let q = nav.col(si, sj);
                                if !q.iter().any(|h| h.s <= lo && h.c >= za + h_f) {
                                    ok = false;
                                    break;
                                }
                            }
                            let bg = nav.col(bi as usize * K + nav.pad, bj as usize * K + nav.pad);
                            if ok && bg.iter().any(|h| (h.s - zb).abs() < 0.01 && h.c >= za + h_f) {
                                let fall = (2.0 * (za - zb) as f64 / mech.gravity.max(1.0)).sqrt();
                                let cost = dh + 0.3 * fall * nav.walk_speed;
                                tmp.push(Edge { to: b, cost: (cost * 10.0).round().min(60000.0) as u16, energy: 0, kind: EDGE_DROP });
                            }
                        } else if zb > za + step_f {
                            let dz = (zb - za) as f64;
                            if jet_col_top < zb + h_f + 0.3 {
                                continue;
                            }
                            let Some((time, energy)) = jet_cost(mech, dz, dh) else { continue };
                            if energy > mech.energy || energy > 255.0 {
                                continue;
                            }
                            let lo = zb + 0.5;
                            let mut ok = true;
                            for k in 1..ns {
                                let f = k as f64 / ns as f64;
                                let si = (sa_i as f64 + di as f64 * K as f64 * f).round() as usize;
                                let sj = (sa_j as f64 + dj as f64 * K as f64 * f).round() as usize;
                                let q = nav.col(si, sj);
                                if !q.iter().any(|h| h.s <= lo && h.c >= zb + h_f) {
                                    ok = false;
                                    break;
                                }
                            }
                            if ok {
                                let cost = time * nav.walk_speed;
                                tmp.push(Edge { to: b, cost: (cost * 10.0).round().min(60000.0) as u16, energy: energy.ceil() as u8, kind: EDGE_JET });
                            }
                        }
                    }
                }
            }
            for e in &tmp {
                edges.push((a as u32, *e));
            }
        }
        // CSR forward and reverse
        let mut fwd_first = vec![0u32; n + 1];
        let mut rev_first = vec![0u32; n + 1];
        for (a, e) in &edges {
            fwd_first[*a as usize + 1] += 1;
            rev_first[e.to as usize + 1] += 1;
        }
        for i in 0..n {
            fwd_first[i + 1] += fwd_first[i];
            rev_first[i + 1] += rev_first[i];
        }
        let blank = Edge { to: 0, cost: 0, energy: 0, kind: 0 };
        let mut fwd = vec![blank; edges.len()];
        let mut rev = vec![blank; edges.len()];
        let mut fp = fwd_first.clone();
        let mut rp = rev_first.clone();
        let mut max_cost = 1u32;
        for (a, e) in &edges {
            fwd[fp[*a as usize] as usize] = *e;
            fp[*a as usize] += 1;
            rev[rp[e.to as usize] as usize] = Edge { to: *a, ..*e };
            rp[e.to as usize] += 1;
            max_cost = max_cost.max(e.cost as u32);
        }
        nav.fwd_first = fwd_first;
        nav.fwd = fwd;
        nav.rev_first = rev_first;
        nav.rev = rev;
        nav.max_cost = max_cost;
        // packed reverse lists for the field search
        let mut rp_first = Vec::with_capacity(n + 1);
        let mut rj_first = Vec::with_capacity(n + 1);
        let mut rp = Vec::with_capacity(nav.rev.len());
        let mut rj = vec![];
        let packable = (n as u64) < (1u64 << (32 - PACK_COST_BITS));
        for v in 0..n {
            rp_first.push(rp.len() as u32);
            rj_first.push(rj.len() as u32);
            for e in &nav.rev[nav.rev_first[v] as usize..nav.rev_first[v + 1] as usize] {
                if e.kind != EDGE_JET && packable && (e.cost as u32) < (1 << PACK_COST_BITS) {
                    rp.push(e.to << PACK_COST_BITS | e.cost as u32);
                } else {
                    rj.push(*e);
                }
            }
        }
        rp_first.push(rp.len() as u32);
        rj_first.push(rj.len() as u32);
        nav.rp_first = rp_first;
        nav.rp = rp;
        nav.rj_first = rj_first;
        nav.rj = rj;
        nav
    }

    #[inline]
    fn col(&self, i: usize, j: usize) -> &[Gap] {
        if i >= self.snx || j >= self.sny {
            return &[];
        }
        let k = j * self.snx + i;
        &self.gaps[self.gap_first[k] as usize..self.gap_first[k + 1] as usize]
    }

    #[inline]
    fn cell_ij(&self, node: u32) -> (usize, usize) {
        let c = self.node_cell[node as usize] as usize;
        (c % self.cnx, c / self.cnx)
    }

    #[inline]
    fn cell_nodes(&self, cell: usize) -> std::ops::Range<u32> {
        self.cell_first[cell]..self.cell_first[cell + 1]
    }

    pub fn node_count(&self) -> usize {
        self.node_z.len()
    }
    pub fn edge_count(&self) -> usize {
        self.fwd.len()
    }
    /// Feet position of node i.
    pub fn node_pos(&self, i: u32) -> [f64; 3] {
        let (ci, cj) = self.cell_ij(i);
        [self.ox + ci as f64 * CELL, self.oy + cj as f64 * CELL, self.node_z[i as usize] as f64]
    }
    /// True when node i stands on a prism top (roof, bridge deck, stadium).
    pub fn node_roof(&self, i: u32) -> bool {
        self.node_roof[i as usize]
    }
    pub fn edges(&self, i: u32) -> &[Edge] {
        &self.fwd[self.fwd_first[i as usize] as usize..self.fwd_first[i as usize + 1] as usize]
    }
    pub fn edges_in(&self, i: u32) -> &[Edge] {
        &self.rev[self.rev_first[i as usize] as usize..self.rev_first[i as usize + 1] as usize]
    }
    /// Grid size (cells x, cells y) and the centre of cell (0, 0).
    pub fn grid(&self) -> (usize, usize, [f64; 2]) {
        (self.cnx, self.cny, [self.ox, self.oy])
    }
    /// Nodes of cell (ci, cj).
    pub fn cell_node_ids(&self, ci: usize, cj: usize) -> std::ops::Range<u32> {
        if ci >= self.cnx || cj >= self.cny {
            return 0..0;
        }
        self.cell_nodes(cj * self.cnx + ci)
    }

    /// Nearest standable node to p (feet), preferring the surface under p.
    pub fn snap_node(&self, p: [f64; 3]) -> Option<u32> {
        if !(p[0].is_finite() && p[1].is_finite()) {
            return None;
        }
        let pz = if p[2].is_finite() { p[2] } else { 1.0e4 };
        let fi = ((p[0] - self.ox) / CELL).round().clamp(-1.0e6, 1.0e6) as i64;
        let fj = ((p[1] - self.oy) / CELL).round().clamp(-1.0e6, 1.0e6) as i64;
        let mut best: Option<(f64, u32)> = None;
        for r in 0..=8i64 {
            if let Some((s, _)) = best {
                let rmin = (r as f64 - 1.0) * CELL;
                if rmin > 0.0 && rmin * rmin > s {
                    break;
                }
            }
            for dj in -r..=r {
                for di in -r..=r {
                    if di.abs().max(dj.abs()) != r {
                        continue;
                    }
                    let (ci, cj) = (fi + di, fj + dj);
                    if ci < 0 || cj < 0 || ci >= self.cnx as i64 || cj >= self.cny as i64 {
                        continue;
                    }
                    for nd in self.cell_nodes(cj as usize * self.cnx + ci as usize) {
                        let q = self.node_pos(nd);
                        let h2 = (q[0] - p[0]).powi(2) + (q[1] - p[1]).powi(2);
                        let dz = pz - q[2];
                        let pen = if dz < -(self.step_up + 0.6) {
                            (dz * 3.0).powi(2) + 100.0
                        } else if dz > 0.0 {
                            (dz * 0.3).powi(2)
                        } else {
                            dz * dz
                        };
                        let s = h2 + pen;
                        if best.map_or(true, |(b, _)| s < b) {
                            best = Some((s, nd));
                        }
                    }
                }
            }
        }
        best.map(|(_, n)| n)
    }

    pub fn snap(&self, p: [f64; 3]) -> Option<[f64; 3]> {
        self.snap_node(p).map(|n| self.node_pos(n))
    }

    pub fn field(&self, goal: [f64; 3]) -> Field {
        self.field_with(goal, -1.0)
    }

    /// Field that may use jet edges needing at most max_jet_energy each (negative: none).
    pub fn field_with(&self, goal: [f64; 3], max_jet_energy: f64) -> Field {
        self.field_multi(&[(goal, 0.0)], max_jet_energy)
    }

    /// Field from several goals, each with a start cost in metres (for instance nearest of many items).
    /// goal() of the result is the first goal.
    pub fn field_multi(&self, goals: &[([f64; 3], f64)], max_jet_energy: f64) -> Field {
        self.field_limited(goals, max_jet_energy, f64::INFINITY)
    }

    /// field_multi that stops expanding past max_m metres (nodes beyond stay unreachable). Much cheaper for local goals.
    pub fn field_limited(&self, goals: &[([f64; 3], f64)], max_jet_energy: f64, max_m: f64) -> Field {
        let limit: u32 = if max_m.is_finite() { (max_m.max(0.0) * 10.0).min(4.0e9) as u32 } else { u32::MAX - 70000 };
        let n = self.node_count();
        let mut d = vec![INF; n];
        let mut seeds: Vec<(u32, u32)> = goals
            .iter()
            .filter_map(|(g, c)| self.snap_node(*g).map(|nd| (nd, (c.max(0.0) * 10.0).round().min(1.0e9) as u32)))
            .collect();
        seeds.sort_by_key(|s| s.1);
        let goal = goals.first().map(|g| g.0).unwrap_or([0.0; 3]);
        let goal_node = goals.first().and_then(|g| self.snap_node(g.0));
        let jl = max_jet_energy;
        let bsz = self.max_cost as usize + 1;
        let mut buckets: Vec<Vec<u32>> = vec![Vec::new(); bsz];
        let mut pending = 0usize;
        let mut sp = 0usize;
        let mut cur: u32 = seeds.first().map_or(0, |s| s.1);
        loop {
            while sp < seeds.len() && (seeds[sp].1 as u64) < cur as u64 + bsz as u64 {
                let (nd, c) = seeds[sp];
                if c < d[nd as usize] {
                    d[nd as usize] = c;
                    buckets[c as usize % bsz].push(nd);
                    pending += 1;
                }
                sp += 1;
            }
            if pending == 0 {
                if sp >= seeds.len() {
                    break;
                }
                cur = seeds[sp].1;
                continue;
            }
            if cur > limit {
                break;
            }
            let bi = cur as usize % bsz;
            let Some(v) = buckets[bi].pop() else {
                cur += 1;
                continue;
            };
            pending -= 1;
            if d[v as usize] != cur {
                continue;
            }
            let lo = self.rp_first[v as usize] as usize;
            let hi = self.rp_first[v as usize + 1] as usize;
            for &pe in &self.rp[lo..hi] {
                let nd = cur + (pe & ((1 << PACK_COST_BITS) - 1));
                let u = (pe >> PACK_COST_BITS) as usize;
                // SAFETY: packed sources are node indices < n
                let du = unsafe { d.get_unchecked_mut(u) };
                if nd < *du {
                    *du = nd;
                    buckets[nd as usize % bsz].push(u as u32);
                    pending += 1;
                }
            }
            let lo = self.rj_first[v as usize] as usize;
            let hi = self.rj_first[v as usize + 1] as usize;
            for e in &self.rj[lo..hi] {
                if e.kind == EDGE_JET && (e.energy as f64) > jl {
                    continue;
                }
                let nd = cur + e.cost as u32;
                let u = e.to as usize;
                // SAFETY: edge targets are node indices < n
                let du = unsafe { d.get_unchecked_mut(u) };
                if nd < *du {
                    *du = nd;
                    buckets[nd as usize % bsz].push(e.to);
                    pending += 1;
                }
            }
        }
        Field { goal, goal_node, d, jet: max_jet_energy }
    }

    pub fn dist(&self, f: &Field, p: [f64; 3]) -> Option<f64> {
        let n = self.snap_node(p)?;
        let v = *f.d.get(n as usize)?;
        if v == INF {
            return None;
        }
        Some(v as f64 * 0.1 + hyp(p, self.node_pos(n)))
    }

    #[inline]
    fn edge_ok(&self, f: &Field, e: &Edge) -> bool {
        e.kind != EDGE_JET || (e.energy as f64) <= f.jet
    }

    fn best_next(&self, f: &Field, cur: u32) -> Option<Edge> {
        let dc = f.d[cur as usize];
        let mut best: Option<(u32, Edge)> = None;
        for e in self.edges(cur) {
            if !self.edge_ok(f, e) {
                continue;
            }
            let dt = f.d[e.to as usize];
            if dt == INF || dt >= dc {
                continue;
            }
            let s = dt + e.cost as u32;
            if best.map_or(true, |(b, _)| s < b) {
                best = Some((s, *e));
            }
        }
        best.map(|b| b.1)
    }

    /// Node path from p along the field descent (at most max_nodes nodes, starting at the snapped node).
    pub fn path(&self, f: &Field, p: [f64; 3], max_nodes: usize) -> Vec<[f64; 3]> {
        let mut out = vec![];
        let Some(mut cur) = self.snap_node(p) else { return out };
        if f.d.get(cur as usize).map_or(true, |&v| v == INF) {
            return out;
        }
        out.push(self.node_pos(cur));
        while out.len() < max_nodes {
            match self.best_next(f, cur) {
                Some(e) => {
                    cur = e.to;
                    out.push(self.node_pos(cur));
                }
                None => break,
            }
        }
        out
    }

    /// Guidance from p: smoothed direction plus what kind of move comes next.
    pub fn step(&self, f: &Field, p: [f64; 3]) -> Option<Step> {
        let n = self.snap_node(p)?;
        let dn = *f.d.get(n as usize)?;
        if dn == INF {
            return None;
        }
        let dist = dn as f64 * 0.1 + hyp(p, self.node_pos(n));
        let mk = |t: [f64; 3], kind: u8, energy: f64| {
            let (dx, dy) = (t[0] - p[0], t[1] - p[1]);
            let l = dx.hypot(dy);
            let dir = if l < 0.3 { [0.0, 0.0] } else { [dx / l, dy / l] };
            Step { dir, target: t, kind, dist, energy }
        };
        if dn == 0 {
            let g = f.goal;
            let t = if self.snap_node(g) == Some(n) { [g[0], g[1], self.node_z[n as usize] as f64] } else { self.node_pos(n) };
            return Some(mk(t, EDGE_WALK, 0.0));
        }
        let mut path: Vec<u32> = Vec::with_capacity(LOOKAHEAD);
        let mut cur = n;
        let mut first_kind = EDGE_WALK;
        let mut first_energy = 0.0;
        while path.len() < LOOKAHEAD {
            let Some(e) = self.best_next(f, cur) else { break };
            if e.kind != EDGE_WALK {
                if path.is_empty() {
                    first_kind = e.kind;
                    first_energy = e.energy as f64;
                    path.push(e.to);
                }
                break;
            }
            path.push(e.to);
            cur = e.to;
        }
        if path.is_empty() {
            return Some(mk(self.node_pos(n), EDGE_WALK, 0.0));
        }
        if first_kind != EDGE_WALK {
            return Some(mk(self.node_pos(path[0]), first_kind, first_energy));
        }
        for k in (1..path.len()).rev() {
            let q = self.node_pos(path[k]);
            if self.clear_from(n, p, q) {
                return Some(mk(q, EDGE_WALK, 0.0));
            }
        }
        Some(mk(self.node_pos(path[0]), EDGE_WALK, 0.0))
    }

    pub fn dir(&self, f: &Field, p: [f64; 3]) -> Option<[f64; 2]> {
        self.step(f, p).map(|s| s.dir)
    }

    pub fn clear(&self, a: [f64; 3], b: [f64; 3]) -> bool {
        match self.snap_node(a) {
            Some(n) => self.clear_from(n, a, b),
            None => false,
        }
    }

    fn clear_from(&self, start: u32, a: [f64; 3], b: [f64; 3]) -> bool {
        let len = hyp(a, b);
        let ns = (len / 0.5).ceil().max(1.0) as usize;
        let mut cur = start;
        let (mut ci, mut cj) = self.cell_ij(cur);
        for k in 1..=ns {
            let f = k as f64 / ns as f64;
            let x = a[0] + (b[0] - a[0]) * f;
            let y = a[1] + (b[1] - a[1]) * f;
            let ni = ((x - self.ox) / CELL).round();
            let nj = ((y - self.oy) / CELL).round();
            if ni < 0.0 || nj < 0.0 || ni >= self.cnx as f64 || nj >= self.cny as f64 {
                return false;
            }
            let (ni, nj) = (ni as usize, nj as usize);
            if ni == ci && nj == cj {
                continue;
            }
            let zc = self.node_z[cur as usize];
            let mut nxt: Option<(f32, u32)> = None;
            for e in self.edges(cur) {
                if e.kind != EDGE_WALK {
                    continue;
                }
                let (ei, ej) = self.cell_ij(e.to);
                if ei == ni && ej == nj {
                    let dz = (self.node_z[e.to as usize] - zc).abs();
                    if nxt.map_or(true, |(b, _)| dz < b) {
                        nxt = Some((dz, e.to));
                    }
                }
            }
            match nxt {
                Some((_, t)) => {
                    cur = t;
                    ci = ni;
                    cj = nj;
                }
                None => return false,
            }
        }
        if b[2].is_finite() {
            let zc = self.node_z[cur as usize] as f64;
            if (zc - b[2]).abs() > self.step_up + 1.5 {
                return false;
            }
        }
        true
    }
}

#[cfg(test)]
mod tests;
