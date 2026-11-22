//! Mech body kinematics, one tick at a time.

use crate::proto::MechCfg;
use crate::world::{Prism, World};

pub const TICK: f64 = 0.05;
/// Sideways slide per tick off a roof edge or out from under an eave.
pub const SLIDE: f64 = 0.6;
const EPS: f64 = 1e-6;

#[derive(Clone, Copy, Debug, PartialEq)]
pub struct Body {
    /// feet
    pub pos: [f64; 3],
    pub vel: [f64; 3],
    pub grounded: bool,
    pub energy: f64,
    /// seconds before energy regeneration resumes
    pub energy_wait: f64,
}

impl Body {
    pub fn new(pos: [f64; 3], energy: f64) -> Body {
        Body { pos, vel: [0.0; 3], grounded: true, energy, energy_wait: 0.0 }
    }
    /// Spend energy outside of movement (restarts the regeneration delay).
    pub fn spend(&mut self, st: &MechCfg, amount: f64, dt: f64) {
        self.energy = (self.energy - amount).max(0.0);
        self.energy_wait = st.energy_regen_delay + dt;
    }
}

#[derive(Clone, Copy, Debug)]
pub struct Drive {
    pub mv: [f64; 2],
    pub boost: bool,
    pub jump: bool,
    pub speed_mult: f64,
}

impl Default for Drive {
    fn default() -> Self {
        Drive { mv: [0.0, 0.0], boost: false, jump: false, speed_mult: 1.0 }
    }
}

#[derive(Clone, Copy, Debug, Default)]
pub struct StepInfo {
    pub boosted: bool,
    pub jetted: bool,
    pub jumped: bool,
    pub landed: bool,
    pub blocked: bool,
}

fn fin(v: f64, d: f64) -> f64 {
    if v.is_finite() { v } else { d }
}

pub fn wrap_angle(a: f64) -> f64 {
    if !a.is_finite() { return 0.0; }
    let mut a = a % std::f64::consts::TAU;
    if a > std::f64::consts::PI { a -= std::f64::consts::TAU; }
    if a < -std::f64::consts::PI { a += std::f64::consts::TAU; }
    a
}

pub fn aim_dir(aim: [f64; 2]) -> [f64; 3] {
    let cp = aim[1].cos();
    [cp * aim[0].cos(), cp * aim[0].sin(), aim[1].sin()]
}

/// Torso aim after one tick of slewing toward want.
pub fn slew_aim(aim: [f64; 2], want: [f64; 2], st: &MechCfg, turn_mult: f64, dt: f64) -> [f64; 2] {
    let wy = fin(want[0], aim[0]);
    let wp = fin(want[1], aim[1]).clamp(st.min_pitch.min(st.max_pitch), st.max_pitch.max(st.min_pitch));
    let my = (st.turn_rate * turn_mult * dt).max(0.0);
    let mp = (st.pitch_rate * turn_mult * dt).max(0.0);
    let dy = wrap_angle(wy - aim[0]);
    let yaw = wrap_angle(aim[0] + dy.clamp(-my, my));
    let dp = wp - aim[1];
    let pitch = (aim[1] + dp.clamp(-mp, mp)).clamp(st.min_pitch.min(st.max_pitch), st.max_pitch.max(st.min_pitch));
    [yaw, pitch]
}

/// Where a shot leaves: out along the aim yaw at the given height above the feet, pulled back if a wall is closer.
pub fn shot_origin(world: &World, pos: [f64; 3], yaw: f64, up: f64, out: f64) -> [f64; 3] {
    let c = [pos[0], pos[1], pos[2] + up];
    let o = [c[0] + out * yaw.cos(), c[1] + out * yaw.sin(), c[2]];
    match world.ray(c, o) {
        Some(t) => {
            let t = (t - 0.02).max(0.0);
            [c[0] + (o[0] - c[0]) * t, c[1] + (o[1] - c[1]) * t, c[2]]
        }
        None => o,
    }
}

fn blocks(base: f64, top: f64, z: f64, hgt: f64, step: f64) -> bool {
    top > z + step && base < z + hgt
}

/// Indices of the prisms close to a body, gathered once per step.
struct Near {
    n: usize,
    idx: [u32; 24],
    extra: Vec<u32>,
}

impl Near {
    fn gather(world: &World, x: f64, y: f64, r: f64) -> Near {
        let mut s = Near { n: 0, idx: [0; 24], extra: Vec::new() };
        world.prisms_near(x, y, r, |i, _| {
            if s.n < s.idx.len() {
                s.idx[s.n] = i as u32;
                s.n += 1;
            } else {
                s.extra.push(i as u32);
            }
        });
        s
    }
    fn each<'w>(&self, world: &'w World, mut f: impl FnMut(&'w Prism)) {
        for k in 0..self.n {
            if let Some(p) = world.prisms.get(self.idx[k] as usize) { f(p); }
        }
        for &i in &self.extra {
            if let Some(p) = world.prisms.get(i as usize) { f(p); }
        }
    }
    fn is_empty(&self) -> bool {
        self.n == 0
    }
}

fn resolve_in(world: &World, near: &Near, p: &mut [f64; 2], z: f64, r: f64, hgt: f64, step: f64, vel: &mut [f64; 2]) -> bool {
    if near.is_empty() { return true; }
    for _ in 0..4 {
        let mut moved = false;
        let (mut x, mut y) = (p[0], p[1]);
        let mut v = *vel;
        near.each(world, |pr| {
            if !blocks(pr.base, pr.top, z, hgt, step) { return; }
            if x < pr.min_x - r || x > pr.max_x + r || y < pr.min_y - r || y > pr.max_y + r { return; }
            let (d, cp, _) = pr.dist_xy(x, y);
            let d = d.abs();
            if !d.is_finite() { return; }
            let inside = pr.contains_xy(x, y);
            if !inside && d >= r { return; }
            let mut n = if d > 1e-9 {
                [(x - cp[0]) / d, (y - cp[1]) / d]
            } else {
                let s = v[0].hypot(v[1]);
                if s > 1e-9 { [-v[0] / s, -v[1] / s] } else { [1.0, 0.0] }
            };
            if inside && d > 1e-9 { n = [-n[0], -n[1]]; }
            x = cp[0] + n[0] * (r + 1e-4);
            y = cp[1] + n[1] * (r + 1e-4);
            let vn = v[0] * n[0] + v[1] * n[1];
            if vn < 0.0 {
                v[0] -= vn * n[0];
                v[1] -= vn * n[1];
            }
            moved = true;
        });
        *p = [x, y];
        *vel = v;
        if !moved { return true; }
    }
    free_in(world, near, *p, z, r, hgt, step)
}

fn free_in(world: &World, near: &Near, p: [f64; 2], z: f64, r: f64, hgt: f64, step: f64) -> bool {
    let mut free = true;
    near.each(world, |pr| {
        if !free || !blocks(pr.base, pr.top, z, hgt, step) { return; }
        if p[0] < pr.min_x - r || p[0] > pr.max_x + r || p[1] < pr.min_y - r || p[1] > pr.max_y + r { return; }
        if pr.contains_xy(p[0], p[1]) || pr.dist_xy(p[0], p[1]).0.abs() < r - 1e-3 { free = false; }
    });
    free
}

/// Pushes the cylinder centre p out of every wall. False if it is still inside one afterwards.
pub fn resolve_walls(world: &World, p: &mut [f64; 2], z: f64, r: f64, hgt: f64, step: f64, vel: &mut [f64; 2]) -> bool {
    let near = Near::gather(world, p[0], p[1], 2.0 * r + 1.0);
    resolve_in(world, &near, p, z, r, hgt, step, vel)
}

/// True if a cylinder at p touches no wall.
pub fn is_free(world: &World, p: [f64; 2], z: f64, r: f64, hgt: f64, step: f64) -> bool {
    let near = Near::gather(world, p[0], p[1], r);
    free_in(world, &near, p, z, r, hgt, step)
}

struct Support {
    /// surface under the centre
    s: f64,
    /// roof edge the cylinder overhangs (top at or below the feet), or -inf
    edge: f64,
    edge_cp: [f64; 2],
    edge_d: f64,
}

fn support(world: &World, near: &Near, x: f64, y: f64, z: f64, r: f64, step: f64) -> Support {
    let mut out = Support { s: world.terrain_z(x, y), edge: f64::NEG_INFINITY, edge_cp: [x, y], edge_d: 0.0 };
    near.each(world, |p| {
        if p.top > z + step + EPS { return; }
        if p.top <= out.s && p.top <= out.edge { return; }
        if x < p.min_x - r || x > p.max_x + r || y < p.min_y - r || y > p.max_y + r { return; }
        if p.contains_xy(x, y) {
            if p.top > out.s { out.s = p.top; }
        } else if p.top <= z + EPS && p.top > out.edge {
            let (d, cp, _) = p.dist_xy(x, y);
            let d = d.abs();
            if d < r - 1e-3 {
                out.edge = p.top;
                out.edge_cp = cp;
                out.edge_d = d;
            }
        }
    });
    out
}

fn slide_from(world: &World, near: &Near, pos: &mut [f64; 3], cp: [f64; 2], d: f64, r: f64, hgt: f64) {
    if d < 1e-9 { return; }
    let n = [(pos[0] - cp[0]) / d, (pos[1] - cp[1]) / d];
    let push = (r - d + 1e-3).clamp(0.0, SLIDE);
    let mut q = [pos[0] + n[0] * push, pos[1] + n[1] * push];
    let mut v = [0.0, 0.0];
    if resolve_in(world, near, &mut q, pos[2], r, hgt, EPS, &mut v) {
        let moved = (q[0] - pos[0]) * n[0] + (q[1] - pos[1]) * n[1];
        if moved > 0.0 {
            pos[0] = q[0];
            pos[1] = q[1];
        }
    }
}

/// Terrain at (x, y) is above feet height zt and steeper than lim there.
fn air_terrain_blocks(world: &World, x: f64, y: f64, zt: f64, lim: f64) -> bool {
    if world.terrain_z(x, y) <= zt + EPS { return false; }
    let gx = world.terrain_z(x + 0.5, y) - world.terrain_z(x - 0.5, y);
    let gy = world.terrain_z(x, y + 0.5) - world.terrain_z(x, y - 0.5);
    gx.hypot(gy) > lim
}

/// Advances one mech body by one tick.
pub fn step(world: &World, st: &MechCfg, body: &mut Body, d: &Drive, dt: f64) -> StepInfo {
    let prev = *body;
    let mut info = StepInfo::default();
    let dt = if dt.is_finite() && dt > 0.0 { dt } else { TICK };
    let r = fin(st.radius, 2.5).max(0.05);
    let hgt = fin(st.height, 9.0).max(0.05);
    let mut mv = [fin(d.mv[0], 0.0), fin(d.mv[1], 0.0)];
    let ml = mv[0].hypot(mv[1]);
    if ml > 1.0 {
        mv = [mv[0] / ml, mv[1] / ml];
    }
    let moving = ml > 1e-9;
    let mult = fin(d.speed_mult, 1.0).max(0.0);
    let was_g = body.grounded;
    let boost_ok = d.boost && body.energy > 0.0;
    let gboost = was_g && boost_ok && moving;

    // horizontal velocity (boost held raises the target speed in the air too)
    let speed = (if boost_ok && moving { st.boost_speed } else { st.walk_speed }) * mult;
    let acc = (if was_g { st.ground_accel } else { st.air_accel }) * dt;
    let (dx, dy) = (mv[0] * speed - body.vel[0], mv[1] * speed - body.vel[1]);
    let dl = dx.hypot(dy);
    if dl <= acc || dl < 1e-12 {
        body.vel[0] = mv[0] * speed;
        body.vel[1] = mv[1] * speed;
    } else {
        body.vel[0] += dx / dl * acc;
        body.vel[1] += dy / dl * acc;
    }

    // vertical velocity and energy
    let mut drained = false;
    let jumped = d.jump && was_g;
    let air = !was_g || jumped;
    if jumped {
        body.vel[2] = st.jump_speed;
        info.jumped = true;
    }
    if air {
        if boost_ok {
            body.vel[2] = (body.vel[2] + st.jet_accel * dt).min(st.max_rise_speed);
            body.energy -= st.jet_drain * dt;
            drained = true;
            info.jetted = true;
        }
        body.vel[2] -= st.gravity * dt;
    } else if gboost {
        body.energy -= st.boost_drain * dt;
        drained = true;
        info.boosted = true;
    }
    if body.energy < 0.0 {
        body.energy = 0.0;
    }
    if drained {
        body.energy_wait = st.energy_regen_delay + dt;
    } else if body.energy_wait > 1e-9 {
        body.energy_wait -= dt;
    } else {
        body.energy = (body.energy + st.energy_regen * dt).min(st.energy.max(body.energy));
    }

    // horizontal move
    let z = body.pos[2];
    let mut disp = [body.vel[0] * dt, body.vel[1] * dt];
    let lim = st.max_slope_deg.clamp(1.0, 89.0).to_radians().tan();
    if was_g {
        let tz = world.terrain_z(body.pos[0], body.pos[1]);
        let dlen = disp[0].hypot(disp[1]);
        if z - tz < 0.3 && dlen > 1e-9 {
            let (nx, ny) = (body.pos[0] + disp[0], body.pos[1] + disp[1]);
            let rise = world.terrain_z(nx, ny) - tz;
            if rise > 0.0 && rise / dlen > lim {
                let gx = world.terrain_z(nx + 0.5, ny) - world.terrain_z(nx - 0.5, ny);
                let gy = world.terrain_z(nx, ny + 0.5) - world.terrain_z(nx, ny - 0.5);
                let g2 = gx * gx + gy * gy;
                if g2 > 1e-12 {
                    let k = (disp[0] * gx + disp[1] * gy) / g2;
                    if k > 0.0 {
                        disp = [disp[0] - gx * k, disp[1] - gy * k];
                        let kv = (body.vel[0] * gx + body.vel[1] * gy) / g2;
                        body.vel[0] -= gx * kv;
                        body.vel[1] -= gy * kv;
                    }
                }
                let d2 = disp[0].hypot(disp[1]);
                let rise2 = world.terrain_z(body.pos[0] + disp[0], body.pos[1] + disp[1]) - tz;
                if d2 < 1e-9 || (rise2 > 0.0 && rise2 / d2 > lim) {
                    disp = [0.0, 0.0];
                    body.vel[0] = 0.0;
                    body.vel[1] = 0.0;
                }
                info.blocked = true;
            }
        }
    }
    if air {
        // steep terrain above the feet is a wall in the air
        let zt = z.max(z + body.vel[2] * dt);
        let dlen = disp[0].hypot(disp[1]);
        if dlen > 1e-9 && air_terrain_blocks(world, body.pos[0] + disp[0], body.pos[1] + disp[1], zt, lim) {
            let (nx, ny) = (body.pos[0] + disp[0], body.pos[1] + disp[1]);
            let gx = world.terrain_z(nx + 0.5, ny) - world.terrain_z(nx - 0.5, ny);
            let gy = world.terrain_z(nx, ny + 0.5) - world.terrain_z(nx, ny - 0.5);
            let g2 = gx * gx + gy * gy;
            if g2 > 1e-12 {
                let k = (disp[0] * gx + disp[1] * gy) / g2;
                if k > 0.0 {
                    disp = [disp[0] - gx * k, disp[1] - gy * k];
                    let kv = (body.vel[0] * gx + body.vel[1] * gy) / g2;
                    if kv > 0.0 {
                        body.vel[0] -= gx * kv;
                        body.vel[1] -= gy * kv;
                    }
                }
            }
            if disp[0].hypot(disp[1]) < 1e-9 || air_terrain_blocks(world, body.pos[0] + disp[0], body.pos[1] + disp[1], zt, lim) {
                disp = [0.0, 0.0];
                body.vel[0] = 0.0;
                body.vel[1] = 0.0;
            }
            info.blocked = true;
        }
    }
    let step_h = if air { EPS } else { st.step_up.max(0.0) };
    let dlen = disp[0].hypot(disp[1]);
    let near = Near::gather(world, body.pos[0], body.pos[1], 2.0 * r + dlen + 2.0 * SLIDE + 0.5);
    let mut p = [body.pos[0], body.pos[1]];
    let mut v2 = [body.vel[0], body.vel[1]];
    if dlen > 1e-12 {
        let n = (dlen.ceil() as usize).clamp(1, 8);
        for _ in 0..n {
            let mut q = [p[0] + disp[0] / n as f64, p[1] + disp[1] / n as f64];
            if resolve_in(world, &near, &mut q, z, r, hgt, step_h, &mut v2) {
                p = q;
            } else {
                v2 = [0.0, 0.0];
                info.blocked = true;
                break;
            }
        }
    }
    let b = &world.bounds;
    if b.max_x - b.min_x > 2.0 * r && b.max_y - b.min_y > 2.0 * r {
        if p[0] < b.min_x + r { p[0] = b.min_x + r; v2[0] = v2[0].max(0.0); }
        if p[0] > b.max_x - r { p[0] = b.max_x - r; v2[0] = v2[0].min(0.0); }
        if p[1] < b.min_y + r { p[1] = b.min_y + r; v2[1] = v2[1].max(0.0); }
        if p[1] > b.max_y - r { p[1] = b.max_y - r; v2[1] = v2[1].min(0.0); }
    }
    if (v2[0] - body.vel[0]).abs() + (v2[1] - body.vel[1]).abs() > 1e-9 {
        info.blocked = true;
    }
    body.vel[0] = v2[0];
    body.vel[1] = v2[1];

    // vertical move
    if !air {
        let mut sup = support(world, &near, p[0], p[1], z, r, step_h);
        if sup.s > z + EPS && world.ceiling_above(p[0], p[1], sup.s) < sup.s + hgt {
            // no head room on the step up: stay
            p = [prev.pos[0], prev.pos[1]];
            body.vel[0] = 0.0;
            body.vel[1] = 0.0;
            info.blocked = true;
            sup = support(world, &near, p[0], p[1], z, r, step_h);
        }
        body.pos[0] = p[0];
        body.pos[1] = p[1];
        if sup.edge > sup.s && sup.edge >= z - step_h {
            body.pos[2] = sup.edge;
            body.vel[2] = 0.0;
            let toward = mv[0] * (sup.edge_cp[0] - p[0]) + mv[1] * (sup.edge_cp[1] - p[1]);
            if toward <= 1e-9 {
                slide_from(world, &near, &mut body.pos, sup.edge_cp, sup.edge_d, r, hgt);
            }
        } else if sup.s >= z - step_h {
            body.pos[2] = sup.s;
            body.vel[2] = 0.0;
        } else {
            body.grounded = false;
            body.vel[2] = 0.0;
        }
    } else {
        body.pos[0] = p[0];
        body.pos[1] = p[1];
        let mut zn = z + body.vel[2] * dt;
        if body.vel[2] > 0.0 {
            let (h0, h1) = (z + hgt, zn + hgt);
            let mut ceil = f64::INFINITY;
            let mut eave: Option<([f64; 2], f64)> = None;
            near.each(world, |pr| {
                if pr.base < h0 - EPS || pr.base >= h1 || pr.base >= ceil { return; }
                if p[0] < pr.min_x - r || p[0] > pr.max_x + r || p[1] < pr.min_y - r || p[1] > pr.max_y + r { return; }
                if pr.contains_xy(p[0], p[1]) {
                    ceil = pr.base;
                    eave = None;
                } else {
                    let (dd, cp, _) = pr.dist_xy(p[0], p[1]);
                    if dd.abs() < r - 1e-3 {
                        ceil = pr.base;
                        eave = Some((cp, dd.abs()));
                    }
                }
            });
            if ceil.is_finite() {
                zn = (ceil - hgt).max(z);
                body.vel[2] = 0.0;
                body.pos[2] = zn;
                if let Some((cp, dd)) = eave {
                    slide_from(world, &near, &mut body.pos, cp, dd, r, hgt);
                }
            }
        }
        let sup = support(world, &near, body.pos[0], body.pos[1], z, r, EPS);
        let floor = sup.s.max(sup.edge);
        if zn <= floor && (body.vel[2] <= 0.0 || zn < floor - EPS) {
            body.pos[2] = floor;
            body.vel[2] = 0.0;
            body.grounded = true;
            info.landed = !was_g;
            if sup.edge > sup.s {
                let toward = mv[0] * (sup.edge_cp[0] - body.pos[0]) + mv[1] * (sup.edge_cp[1] - body.pos[1]);
                if toward <= 1e-9 {
                    slide_from(world, &near, &mut body.pos, sup.edge_cp, sup.edge_d, r, hgt);
                }
            }
        } else {
            body.pos[2] = zn;
            body.grounded = false;
        }
    }
    let tz = world.terrain_z(body.pos[0], body.pos[1]);
    if body.pos[2] < tz {
        body.pos[2] = tz;
        body.vel[2] = 0.0;
        body.grounded = true;
    }
    let ok = body.pos.iter().chain(body.vel.iter()).all(|v| v.is_finite()) && body.energy.is_finite() && body.energy_wait.is_finite();
    if !ok {
        *body = prev;
        body.vel = [0.0; 3];
        if !body.pos.iter().all(|v| v.is_finite()) {
            body.pos = [0.0; 3];
        }
        if !body.energy.is_finite() { body.energy = 0.0; }
        if !body.energy_wait.is_finite() { body.energy_wait = 0.0; }
    }
    info
}
