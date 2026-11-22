use std::collections::VecDeque;

use super::math::*;
use crate::proto::{ActorKind, Obs, ProjKind};

#[derive(Clone, Copy)]
struct Smp {
    t: f64,
    pos: V3,
    s: f64,
    e: Option<V2>,
    ground: bool,
    sp: f64,
}

type Disp = [[f32; 2]; 12];

struct Hist {
    id: i64,
    last_rev: f64,
    ref_e: Option<V2>,
    smp: VecDeque<Smp>,
    table: Vec<VecDeque<Disp>>,
    shot: VecDeque<(f64, Disp)>,
}

const NB: usize = 44;

pub(crate) struct AimZ {
    fired: VecDeque<(i64, f64)>,
    gshot: VecDeque<(f64, Disp)>,
    hist: Vec<Hist>,
    table: Vec<VecDeque<Disp>>,
}

fn bin(s: &Smp) -> usize {
    ((s.s / 0.1).floor().clamp(0.0, 21.0) as usize) * 2 + if s.sp > 15.0 { 1 } else { 0 }
}

impl AimZ {
    pub(crate) fn new() -> AimZ {
        AimZ { fired: VecDeque::new(), gshot: VecDeque::new(), hist: Vec::new(), table: (0..NB).map(|_| VecDeque::new()).collect() }
    }

    pub(crate) fn update(&mut self, obs: &Obs) {
        let t = obs.t;
        for m in &obs.mechs {
            if m.kind != ActorKind::Mech || !m.pos.iter().chain(m.vel.iter()).all(|x| x.is_finite()) {
                continue;
            }
            let hi = match self.hist.iter().position(|h| h.id == m.id) {
                Some(i) => i,
                None => {
                    if self.hist.len() >= 24 {
                        let (old, _) = self.hist.iter().enumerate().min_by(|a, b| {
                            let ta = a.1.smp.back().map_or(0.0, |s| s.t);
                            let tb = b.1.smp.back().map_or(0.0, |s| s.t);
                            ta.total_cmp(&tb)
                        }).unwrap();
                        self.hist.remove(old);
                    }
                    self.hist.push(Hist { id: m.id, last_rev: t - 2.0, ref_e: None, shot: VecDeque::new(), smp: VecDeque::new(), table: (0..NB).map(|_| VecDeque::new()).collect() });
                    self.hist.len() - 1
                }
            };
            let h = &mut self.hist[hi];
            if let Some(l) = h.smp.back() {
                if t - l.t < 0.085 {
                    continue;
                }
                if t - l.t > 0.5 {
                    h.smp.clear();
                }
            }
            let sp = m.vel[0].hypot(m.vel[1]);
            if sp > 3.0 {
                let e = [m.vel[0] / sp, m.vel[1] / sp];
                match h.ref_e {
                    Some(r) => {
                        let c = r[0] * e[0] + r[1] * e[1];
                        if c < 0.34 {
                            h.last_rev = t;
                            h.ref_e = Some(e);
                        } else if c > 0.87 {
                            h.ref_e = Some(e);
                        }
                    }
                    None => h.ref_e = Some(e),
                }
            }
            let e = if sp > 2.0 { Some([m.vel[0] / sp, m.vel[1] / sp]) } else { None };
            let ground = m.vel[2].abs() < 0.3;
            h.smp.push_back(Smp { t, pos: m.pos, s: (t - h.last_rev).min(2.1), e, ground, sp });
            while h.smp.len() > 14 {
                h.smp.pop_front();
            }
            if h.smp.len() >= 13 {
                let base = h.smp[h.smp.len() - 13];
                if let (Some(e), true) = (base.e, base.ground) {
                    let mut ok = true;
                    let mut d: Disp = [[0.0; 2]; 12];
                    for j in 1..=12 {
                        let q = h.smp[h.smp.len() - 13 + j];
                        if (q.t - base.t - 0.1 * j as f64).abs() > 0.04 {
                            ok = false;
                            break;
                        }
                        let dx = q.pos[0] - base.pos[0];
                        let dy = q.pos[1] - base.pos[1];
                        d[j - 1] = [(dx * e[0] + dy * e[1]) as f32, (-dx * e[1] + dy * e[0]) as f32];
                    }
                    if ok {
                        let id = h.id;
                        if self.fired.iter().any(|f| f.0 == id && (f.1 - base.t).abs() < 0.06) {
                            h.shot.push_back((base.sp, d));
                            if h.shot.len() > 40 {
                                h.shot.pop_front();
                            }
                            self.gshot.push_back((base.sp, d));
                            if self.gshot.len() > 80 {
                                self.gshot.pop_front();
                            }
                        }
                        let b = bin(&base);
                        h.table[b].push_back(d);
                        if h.table[b].len() > 60 {
                            h.table[b].pop_front();
                        }
                        self.table[b].push_back(d);
                        if self.table[b].len() > 160 {
                            self.table[b].pop_front();
                        }
                    }
                }
            }
        }
    }

    pub(crate) fn note_fire(&mut self, now: f64, feet: V3) {
        if let Some(h) = self.hist.iter().find(|h| h.smp.back().map_or(false, |s| now - s.t < 0.15 && dist2(s.pos, feet) < 0.5)) {
            let st = h.smp.back().map_or(now, |s| s.t);
            if self.fired.back().map_or(true, |f| f.0 != h.id || (f.1 - st).abs() > 0.01) {
                self.fired.push_back((h.id, st));
            }
        }
        while self.fired.front().map_or(false, |f| now - f.1 > 2.0) {
            self.fired.pop_front();
        }
    }

    /// Lateral offset (along nn) of the best aim point after tt seconds, and the share of samples it covers.
    pub(crate) fn query(&self, now: f64, feet: V3, tt: f64, nn: V2, hw: f64, shrink: f64, min_n: usize, shot_min: usize, gshot: bool) -> Option<(f64, f64)> {
        if !(tt > 0.0) || tt > 1.2 {
            return None;
        }
        let h = self.hist.iter().find(|h| h.smp.back().map_or(false, |s| now - s.t < 0.15 && dist2(s.pos, feet) < 0.5));
        let h = h?;
        let s = *h.smp.back()?;
        let e = s.e?;
        if !s.ground {
            return None;
        }
        let b = bin(&s);
        let mut pool: Vec<&Disp> = Vec::new();
        if shot_min > 0 {
            pool.extend(h.shot.iter().filter(|x| (x.0 > 15.0) == (s.sp > 15.0)).map(|x| &x.1));
            if pool.len() < shot_min && gshot {
                pool.clear();
                pool.extend(self.gshot.iter().filter(|x| (x.0 > 15.0) == (s.sp > 15.0)).map(|x| &x.1));
            }
        }
        if pool.len() < shot_min.max(1) {
          pool.clear();
          for src in [&h.table, &self.table] {
            pool.clear();
            pool.extend(src[b].iter());
            for k in [2usize, 4] {
                if pool.len() >= min_n {
                    break;
                }
                if b >= k {
                    pool.extend(src[b - k].iter());
                }
                if b + k < NB {
                    pool.extend(src[b + k].iter());
                }
            }
            if pool.len() >= min_n {
                break;
            }
          }
        }
        if pool.len() < min_n {
            return None;
        }
        let j = (tt / 0.1).clamp(1.0, 12.0);
        let j0 = (j.floor() as usize).clamp(1, 12);
        let fr = j - j0 as f64;
        let j1 = (j0 + 1).min(12);
        let mut lat: Vec<f64> = pool
            .iter()
            .map(|d| {
                let a = d[j0 - 1];
                let c = d[j1 - 1];
                let x = a[0] as f64 * (1.0 - fr) + c[0] as f64 * fr;
                let y = a[1] as f64 * (1.0 - fr) + c[1] as f64 * fr;
                let wx = x * e[0] - y * e[1];
                let wy = x * e[1] + y * e[0];
                wx * nn[0] + wy * nn[1]
            })
            .collect();
        lat.sort_by(|a, b| a.total_cmp(b));
        let w = 2.0 * hw * shrink;
        let mut best = 0usize;
        let mut bx = 0.0;
        let mut lo = 0usize;
        for i in 0..lat.len() {
            while lat[i] - lat[lo] > w {
                lo += 1;
            }
            let c = i - lo + 1;
            if c > best {
                best = c;
                bx = 0.5 * (lat[i] + lat[lo]);
            }
        }
        let n = lat.len() as f64;
        let cover = lat.iter().filter(|x| (*x - bx).abs() < hw).count() as f64 / n;
        Some((bx, cover))
    }
}

fn phi(x: f64) -> f64 {
    let t = 1.0 / (1.0 + 0.2316419 * x.abs());
    let d = 0.3989423 * (-x * x / 2.0).exp();
    let p = d * t * (0.3193815 + t * (-0.3565638 + t * (1.781478 + t * (-1.821256 + t * 1.330274))));
    if x >= 0.0 {
        1.0 - p
    } else {
        p
    }
}

pub(crate) fn antilead<I: Iterator<Item = (V3, f64)>>(pts: &[V3], dt: f64, pos: V3, vel: V3, shooters: I, hw: f64, speed: f64, lat: f64, sigma: f64, dmg: f64) -> f64 {
    let n = pts.len();
    if n < 2 {
        return 0.0;
    }
    let mut cost = 0.0;
    for (eye, aim_yaw) in shooters {
        let dx = pos[0] - eye[0];
        let dy = pos[1] - eye[1];
        let d = dx.hypot(dy);
        if !(d > 30.0 && d < 230.0) {
            continue;
        }
        let bearing = dy.atan2(dx);
        if adiff(aim_yaw, bearing).abs() > 0.6 {
            continue;
        }
        let tt = lat + d / speed;
        let kf = tt / dt;
        let k = kf.floor() as usize;
        let q = if k + 1 < n {
            let f = kf - k as f64;
            [pts[k][0] * (1.0 - f) + pts[k + 1][0] * f, pts[k][1] * (1.0 - f) + pts[k + 1][1] * f]
        } else {
            let a = pts[n - 2];
            let b = pts[n - 1];
            let ex = tt - (n - 1) as f64 * dt;
            [b[0] + (b[0] - a[0]) / dt * ex, b[1] + (b[1] - a[1]) / dt * ex]
        };
        let pl = [pos[0] + vel[0] * tt, pos[1] + vel[1] * tt];
        let nn = [-dy / d.max(0.1), dx / d.max(0.1)];
        let sep = (q[0] - pl[0]) * nn[0] + (q[1] - pl[1]) * nn[1];
        let sg = (sigma * d).max(0.2);
        let ph = phi((hw - sep) / sg) - phi((-hw - sep) / sg);
        cost += ph * dmg;
    }
    cost
}

pub(crate) struct Cadence {
    known: VecDeque<i64>,
    pub(crate) last: Vec<(i64, f64, f64, u32)>,
}

impl Cadence {
    pub(crate) fn new() -> Cadence {
        Cadence { known: VecDeque::new(), last: Vec::new() }
    }

    pub(crate) fn update(&mut self, obs: &Obs, period: f64, snap: f64) {
        let t = obs.t;
        let me = obs.me.pos;
        let mc = [me[0], me[1], me[2] + obs.me.stats.height * 0.5];
        for pr in &obs.projectiles {
            if pr.kind != ProjKind::Gun || self.known.contains(&pr.id) {
                continue;
            }
            if !pr.pos.iter().chain(pr.vel.iter()).all(|x| x.is_finite()) {
                continue;
            }
            self.known.push_back(pr.id);
            if self.known.len() > 160 {
                self.known.pop_front();
            }
            let sp = len(pr.vel);
            if sp < 50.0 {
                continue;
            }
            let u = scale(pr.vel, 1.0 / sp);
            let rel = sub(mc, pr.pos);
            let along = dot(rel, u);
            if along < 0.0 || len(sub(rel, scale(u, along))) > 12.0 {
                continue;
            }
            let mut best: Option<(f64, i64, f64)> = None;
            for m in &obs.mechs {
                if m.kind != ActorKind::Mech {
                    continue;
                }
                let w = sub([m.pos[0], m.pos[1], m.pos[2] + obs.me.stats.muzzle_height], pr.pos);
                let s = -dot(w, u);
                if s < 0.0 || s > 400.0 {
                    continue;
                }
                let perp = len(add(w, scale(u, s)));
                if perp < 4.0 && best.map_or(true, |b| perp < b.0) {
                    best = Some((perp, m.id, s / sp));
                }
            }
            if let Some((_, id, age)) = best {
                let raw = t - age;
                let ft = if snap > 0.0 { (raw / snap).round() * snap } else { raw };
                match self.last.iter_mut().find(|x| x.0 == id) {
                    Some(x) => {
                        let iv = raw - x.2;
                        if iv > 0.05 {
                            let k = (iv / period).round();
                            if k >= 1.0 && k <= 3.0 && (iv - k * period).abs() < 0.025 {
                                x.3 += 1;
                            } else {
                                x.3 = 0;
                            }
                            x.1 = ft;
                            x.2 = raw;
                        }
                    }
                    None => self.last.push((id, ft, raw, 0)),
                }
            }
        }
        self.last.retain(|x| t - x.1 < 3.0 + period);
    }

    pub(crate) fn due(&self, id: i64, t: f64, period: f64, win: f64, keep: f64, off: f64, reg: u32) -> bool {
        let Some(x) = self.last.iter().find(|x| x.0 == id) else { return false };
        if x.3 < reg {
            return false;
        }
        let since = t - (x.1 - off);
        if since < period * 0.5 || since > keep {
            return false;
        }
        let k = (since / period).round();
        (since - k * period).abs() <= win
    }
}
