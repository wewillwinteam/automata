use std::f64::consts::PI;

pub type V2 = [f64; 2];
pub type V3 = [f64; 3];

pub fn wrap(a: f64) -> f64 {
    if !a.is_finite() {
        return 0.0;
    }
    let x = (a + PI).rem_euclid(2.0 * PI) - PI;
    if x <= -PI {
        x + 2.0 * PI
    } else {
        x
    }
}

pub fn adiff(a: f64, b: f64) -> f64 {
    wrap(a - b)
}

pub fn fin(x: f64, d: f64) -> f64 {
    if x.is_finite() {
        x
    } else {
        d
    }
}

pub fn add(a: V3, b: V3) -> V3 {
    [a[0] + b[0], a[1] + b[1], a[2] + b[2]]
}
pub fn sub(a: V3, b: V3) -> V3 {
    [a[0] - b[0], a[1] - b[1], a[2] - b[2]]
}
pub fn scale(a: V3, k: f64) -> V3 {
    [a[0] * k, a[1] * k, a[2] * k]
}
pub fn dot(a: V3, b: V3) -> f64 {
    a[0] * b[0] + a[1] * b[1] + a[2] * b[2]
}
pub fn len(a: V3) -> f64 {
    dot(a, a).sqrt()
}
pub fn dist(a: V3, b: V3) -> f64 {
    len(sub(a, b))
}
pub fn dist2(a: V3, b: V3) -> f64 {
    (a[0] - b[0]).hypot(a[1] - b[1])
}
pub fn dist2v(a: V3, b: V2) -> f64 {
    (a[0] - b[0]).hypot(a[1] - b[1])
}
pub fn up(a: V3, h: f64) -> V3 {
    [a[0], a[1], a[2] + h]
}
pub fn unit2(x: f64, y: f64) -> V2 {
    let l = x.hypot(y);
    if l > 1e-9 && l.is_finite() {
        [x / l, y / l]
    } else {
        [0.0, 0.0]
    }
}

pub fn aim_dir(yaw: f64, pitch: f64) -> V3 {
    [pitch.cos() * yaw.cos(), pitch.cos() * yaw.sin(), pitch.sin()]
}

pub fn dir_to_aim(d: V3) -> V2 {
    let h = d[0].hypot(d[1]);
    if h < 1e-9 && d[2].abs() < 1e-9 {
        return [0.0, 0.0];
    }
    [d[1].atan2(d[0]), d[2].atan2(h)]
}

pub fn slew(cur: V2, cmd: V2, dyaw: f64, dpitch: f64) -> V2 {
    let ey = adiff(cmd[0], cur[0]);
    let ep = cmd[1] - cur[1];
    [wrap(cur[0] + ey.clamp(-dyaw, dyaw)), cur[1] + ep.clamp(-dpitch, dpitch)]
}

pub struct Rng(u64);

impl Rng {
    pub fn new(seed: u64) -> Rng {
        Rng(seed ^ 0x9E37_79B9_7F4A_7C15)
    }
    pub fn next_u64(&mut self) -> u64 {
        self.0 = self.0.wrapping_add(0x9E37_79B9_7F4A_7C15);
        let mut z = self.0;
        z = (z ^ (z >> 30)).wrapping_mul(0xBF58_476D_1CE4_E5B9);
        z = (z ^ (z >> 27)).wrapping_mul(0x94D0_49BB_1331_11EB);
        z ^ (z >> 31)
    }
    pub fn f64(&mut self) -> f64 {
        (self.next_u64() >> 11) as f64 / (1u64 << 53) as f64
    }
    pub fn range(&mut self, a: f64, b: f64) -> f64 {
        a + (b - a) * self.f64()
    }
}

/// Smallest t > 0 with |p + v t| = s t.
pub fn intercept_time(p: V3, v: V3, s: f64) -> Option<f64> {
    if !(s > 0.0) {
        return None;
    }
    let a = dot(v, v) - s * s;
    let b = 2.0 * dot(p, v);
    let c = dot(p, p);
    if a.abs() < 1e-9 {
        if b < -1e-12 {
            let t = -c / b;
            return if t > 0.0 && t.is_finite() { Some(t) } else { None };
        }
        return None;
    }
    let disc = b * b - 4.0 * a * c;
    if disc < 0.0 || !disc.is_finite() {
        return None;
    }
    let sq = disc.sqrt();
    let t1 = (-b - sq) / (2.0 * a);
    let t2 = (-b + sq) / (2.0 * a);
    let (lo, hi) = if t1 < t2 { (t1, t2) } else { (t2, t1) };
    if lo > 1e-9 {
        Some(lo)
    } else if hi > 1e-9 {
        Some(hi)
    } else {
        None
    }
}

/// Aim point (relative to the shooter) and flight time for a constant-speed shot.
pub fn lead(p: V3, v: V3, acc: V3, s: f64, latency: f64, acc_gain: f64, acc_t: f64) -> Option<(V3, f64)> {
    let p0 = add(p, scale(v, latency));
    let mut t = intercept_time(p0, v, s)?;
    let mut pt = add(p0, scale(v, t));
    if acc_gain > 0.0 {
        for _ in 0..3 {
            let ta = t.min(acc_t);
            pt = add(add(p0, scale(v, t)), scale(acc, 0.5 * acc_gain * ta * ta));
            t = len(pt) / s;
        }
    }
    if t.is_finite() && pt.iter().all(|x| x.is_finite()) {
        Some((pt, t))
    } else {
        None
    }
}

/// Launch pitch and flight time to hit a point at horizontal distance d and height dz.
pub fn mortar_solution(d: f64, dz: f64, v: f64, g: f64, high: bool) -> Option<(f64, f64)> {
    if !(d > 1e-6) || !(v > 0.0) || !(g > 0.0) {
        return None;
    }
    let v2 = v * v;
    let disc = v2 * v2 - g * (g * d * d + 2.0 * dz * v2);
    if disc < 0.0 || !disc.is_finite() {
        return None;
    }
    let sq = disc.sqrt();
    let tan = if high { (v2 + sq) / (g * d) } else { (v2 - sq) / (g * d) };
    let pitch = tan.atan();
    let c = pitch.cos();
    if c < 1e-6 {
        return None;
    }
    Some((pitch, d / (v * c)))
}

/// Closest horizontal approach of a point moving as p + v t for t in [t0, t1]. Returns (t, horizontal distance, dz).
pub fn closest_xy(p: V3, v: V3, t0: f64, t1: f64) -> (f64, f64, f64) {
    let vv = v[0] * v[0] + v[1] * v[1];
    let mut t = if vv > 1e-12 { -(p[0] * v[0] + p[1] * v[1]) / vv } else { t0 };
    if !t.is_finite() {
        t = t0;
    }
    let t = t.clamp(t0, t1.max(t0));
    let x = p[0] + v[0] * t;
    let y = p[1] + v[1] * t;
    (t, x.hypot(y), p[2] + v[2] * t)
}

pub fn frac(a: f64, b: f64) -> f64 {
    if b > 1e-9 && a.is_finite() {
        (a / b).clamp(0.0, 1.0)
    } else {
        1.0
    }
}
