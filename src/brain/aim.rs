use super::laser::PkSt;
use super::math::*;
use super::memory::Track;
use super::{Brain, Mode, Tgt};
use crate::proto::{ActorKind, Obs, SoundDist, WeaponKind, WeaponObs};
use crate::world::World;

#[derive(Clone, Debug)]
pub(crate) struct AimTarget {
    pub id: i64,
    pub feet: V3,
    pub vel: V3,
    pub acc: V3,
    pub radius: f64,
    pub height: f64,
    pub hp: f64,
    pub fixed: bool,
}

pub(crate) struct FireOut {
    pub aim: V2,
    pub fire: [bool; 2],
    pub reload: bool,
    pub target: Option<AimTarget>,
    pub armed: bool,
    pub charging: bool,
}

impl Brain {
    pub(crate) fn aim_target(&self, tr: &Track) -> AimTarget {
        let (radius, height) = if tr.kind == ActorKind::Npc {
            (self.cfg.npc.radius, self.cfg.npc.height)
        } else {
            let s = 1.0 + self.cfg.cubes.size_per_cube * tr.cubes.max(0.0);
            (self.cfg.mech.radius * s, self.cfg.mech.height * s)
        };
        AimTarget { id: tr.id, feet: tr.pos, vel: tr.vel, acc: tr.acc, radius, height, hp: tr.hp, fixed: false }
    }

    fn idle_aim(&self, obs: &Obs) -> V2 {
        let t = obs.t;
        if let Some(u) = &self.mem.unseen {
            if t - u.t < 3.0 {
                return [u.bearing, 0.0];
            }
        }
        let mut best: Option<(f64, f64)> = None;
        for s in &self.mem.sounds {
            if t - s.t > 2.0 {
                continue;
            }
            let k = match s.kind.as_str() {
                "gun" | "laser" | "laser_charge" | "lightning" | "mortar" | "step" | "boost" | "jet" | "reload" => 1.0,
                _ => 0.0,
            };
            let d = match s.dist {
                SoundDist::Near => 3.0,
                SoundDist::Mid => 1.5,
                _ => 0.0,
            };
            let pr = k * d - (t - s.t) * 0.3;
            if k * d > 0.0 && best.map_or(true, |b| pr > b.0) {
                best = Some((pr, s.bearing));
            }
        }
        if let Some((_, b)) = best {
            if let Some(a) = self.threat_aim(obs) {
                return a;
            }
            return [b, 0.0];
        }
        if let Some(a) = self.threat_aim(obs) {
            return a;
        }
        let base = if self.last_dir[0].hypot(self.last_dir[1]) > 0.1 { self.last_dir[1].atan2(self.last_dir[0]) } else { obs.me.aim[0] };
        let ph = 2.0 * std::f64::consts::PI * t / self.p.scan_period.max(0.5);
        [wrap(base + self.p.scan_amp * ph.sin()), 0.0]
    }

    fn pick_fire_target(&self, obs: &Obs, world: &World, muzzle: V3) -> Option<(AimTarget, bool)> {
        let t = obs.t;
        let me = &obs.me;
        let mut max_range: f64 = 0.0;
        for w in me.weapons.iter().flatten() {
            let r = match w.kind {
                WeaponKind::Gun => w.stats.range * self.p.gun_range_frac,
                WeaponKind::Mortar => {
                    if w.stats.gravity > 0.0 {
                        w.stats.speed * w.stats.speed / w.stats.gravity * 0.95
                    } else {
                        0.0
                    }
                }
                _ => w.stats.range,
            };
            max_range = max_range.max(r);
        }
        let mut best: Option<(f64, AimTarget, bool)> = None;
        let held = self.held(t);
        let gate = self.gate_closed(obs);
        for tr in &self.mem.tracks {
            if held || tr.kind != ActorKind::Mech || t - tr.last_seen > 0.15 {
                continue;
            }
            if gate && self.spare(tr.hp, tr.max_hp) {
                continue;
            }
            if !self.order_fire_ok(tr, obs) {
                continue;
            }
            if self.roof_hold && t - tr.last_dmg_t > 1.5 {
                continue;
            }
            let at = self.aim_target(tr);
            let c = up(at.feet, at.height * 0.5);
            let d = dist(c, muzzle);
            let clear = world.ray(muzzle, c).is_none();
            let attacker = t - tr.last_dmg_t < 3.0;
            if self.sf_hold(obs, tr) || self.gear_skip_fire(tr, t, d, clear) {
                continue;
            }
            if !self.ffa_fire_ok(obs, tr, attacker, d, clear) {
                continue;
            }
            let mut s = -0.3 * d + 40.0 * (1.0 - frac(tr.hp, tr.max_hp));
            if clear {
                s += 100.0;
            }
            if attacker {
                s += 40.0;
            }
            if self.target == Tgt::Mech(tr.id) {
                s += 50.0;
            }
            if d < max_range {
                s += 50.0;
            }
            s += self.finish_bonus(tr);
            let to_us = (me.pos[1] - tr.pos[1]).atan2(me.pos[0] - tr.pos[0]);
            let aligned = adiff(tr.aim[0], to_us).abs() < 0.5;
            let allow = (if self.econ.resting { attacker || d < 45.0 } else { self.mode != Mode::Retreat || attacker || (aligned && clear) || d < 60.0 }) && !self.hold_fire(obs, d, tr.hp);
            if best.as_ref().map_or(true, |b| s > b.0) {
                best = Some((s, at, allow));
            }
        }
        if let Some((_, at, allow)) = best {
            return Some((at, allow));
        }
        let mut bn: Option<(f64, AimTarget, bool)> = None;
        for tr in &self.mem.tracks {
            if tr.kind != ActorKind::Npc || t - tr.last_seen > 0.15 {
                continue;
            }
            let attacker = t - tr.last_dmg_t < 3.0;
            let hunted = self.target == Tgt::Npc(tr.id) || self.gear_npc_ok(obs, tr);
            if (!attacker && !hunted) || (self.roof_hold && t - tr.last_dmg_t > 1.5) {
                continue;
            }
            let d = dist(tr.pos, me.pos);
            let s = -d + if hunted { 50.0 } else { 0.0 };
            if bn.as_ref().map_or(true, |b| s > b.0) {
                bn = Some((s, self.aim_target(tr), self.mode != Mode::Retreat || attacker));
            }
        }
        if let Some((_, at, allow)) = bn {
            return Some((at, allow));
        }
        if !self.allow_crates {
            return None;
        }
        let mut bc: Option<(f64, AimTarget)> = None;
        let mut rays = 0;
        for it in &self.mem.crates {
            if t - it.last_seen > 0.3 || it.hp <= 0.0 || self.mem.blacklisted(it.id, t) {
                continue;
            }
            let c = up(it.pos, self.cfg.crates.height * 0.5);
            let d = dist(c, muzzle);
            if d > max_range.min(170.0) {
                continue;
            }
            let s = -d + if self.target == Tgt::Crate(it.id) { 60.0 } else { 0.0 };
            if bc.as_ref().map_or(true, |b| s > b.0) && rays < 8 {
                rays += 1;
                if world.ray(muzzle, c).is_none() {
                    let at = AimTarget { id: it.id, feet: it.pos, vel: [0.0; 3], acc: [0.0; 3], radius: self.cfg.crates.radius, height: self.cfg.crates.height, hp: it.hp, fixed: true };
                    bc = Some((s, at));
                }
            }
        }
        bc.map(|b| (b.1, true))
    }

    fn mortar_solve(&self, obs: &Obs, world: &World, tg: &AimTarget, w: &WeaponObs) -> Option<V2> {
        let p = &self.p;
        let st = &obs.me.stats;
        let tube = up(obs.me.pos, st.height + 0.5);
        let v = w.stats.speed;
        let g = w.stats.gravity;
        if !(v > 0.0) || !(g > 0.0) {
            return None;
        }
        let vxy = [tg.vel[0], tg.vel[1], 0.0];
        let speed = vxy[0].hypot(vxy[1]);
        if speed > p.mortar_max_target_speed {
            return None;
        }
        for high in [false, true] {
            let mut tf = dist2(tg.feet, tube) / (v * 0.9);
            let mut sol: Option<(f64, V3, f64)> = None;
            for _ in 0..3 {
                let tp = add(up(tg.feet, 0.5), scale(vxy, tf + p.latency));
                let d = dist2(tp, tube);
                match mortar_solution(d, tp[2] - tube[2], v, g, high) {
                    Some((pitch, t2)) => {
                        tf = t2;
                        sol = Some((pitch, tp, d));
                    }
                    None => {
                        sol = None;
                        break;
                    }
                }
            }
            let Some((pitch, tp, d)) = sol else { continue };
            if d < p.mortar_min_range || pitch < st.min_pitch + 0.01 || pitch > st.max_pitch - 0.01 {
                continue;
            }
            if tf > p.mortar_max_flight && speed > 2.0 && !tg.fixed {
                continue;
            }
            if w.stats.ttl > 0.0 && tf > w.stats.ttl * 0.95 {
                continue;
            }
            let yaw = (tp[1] - tube[1]).atan2(tp[0] - tube[0]);
            let vel = scale(aim_dir(yaw, pitch), v);
            let mut prev = tube;
            let mut clear = true;
            let n = 10;
            for k in 1..=n {
                let tt = tf * 0.93 * k as f64 / n as f64;
                let q = [tube[0] + vel[0] * tt, tube[1] + vel[1] * tt, tube[2] + vel[2] * tt - 0.5 * g * tt * tt];
                if world.ray(prev, q).is_some() {
                    clear = false;
                    break;
                }
                prev = q;
            }
            if clear {
                return Some([yaw, pitch]);
            }
        }
        None
    }

    /// Whether the laser line from our muzzle to the target centre is still free after tau seconds,
    /// both extrapolated along their horizontal velocity (ours capped at the charging speed).
    pub(crate) fn lz_clear_at(&self, world: &World, obs: &Obs, tg: &AimTarget, tau: f64) -> bool {
        let me = &obs.me;
        let st = &me.stats;
        let tau = fin(tau, 0.0).max(0.0);
        let mut v = [fin(me.vel[0], 0.0), fin(me.vel[1], 0.0)];
        let cap = (st.walk_speed * st.charge_move_mult.clamp(0.1, 1.0)).max(0.0);
        let sp = v[0].hypot(v[1]);
        if sp > cap && sp > 1e-9 {
            v = [v[0] / sp * cap, v[1] / sp * cap];
        }
        let mut m = [me.pos[0] + v[0] * tau, me.pos[1] + v[1] * tau, me.pos[2] + st.muzzle_height];
        if world.solid(m[0], m[1], m[2]) {
            m = up(me.pos, st.muzzle_height);
        }
        let c = [tg.feet[0] + fin(tg.vel[0], 0.0) * tau, tg.feet[1] + fin(tg.vel[1], 0.0) * tau, tg.feet[2] + tg.height * 0.5];
        if world.solid(c[0], c[1], c[2]) {
            // the target cannot walk into a wall: it will slide along the face we already see
            return true;
        }
        world.ray(m, c).is_none()
    }

    pub(crate) fn fire_control(&mut self, obs: &Obs, world: &World) -> FireOut {
        let p = self.p.clone();
        let me = &obs.me;
        let t = obs.t;
        let st = &me.stats;
        let muzzle = up(me.pos, st.muzzle_height);
        let mut out = FireOut { aim: self.idle_aim(obs), fire: [false; 2], reload: false, target: None, armed: false, charging: false };
        let mut tgt = self.pick_fire_target(obs, world, muzzle);
        let lz_on = self.pk.st == PkSt::Off;
        let noblind = lz_on && p.lz_noblind > 0.0;
        let my_charge = noblind && me.weapons.iter().flatten().any(|w| w.kind == WeaponKind::Laser && w.charge > 0.0);
        if noblind && !my_charge {
            self.lz_tid = -1;
        }
        if let Some(u) = &self.mem.unseen {
            let attacked_by_tgt = tgt.is_some() && self.mem.tracks.iter().any(|tr| t - tr.last_dmg_t < 1.5 && t - tr.last_seen < 0.3);
            if t - u.t < 1.2 && !attacked_by_tgt {
                if let Some((tg, _)) = &tgt {
                    let c = up(tg.feet, tg.height * 0.5);
                    if !my_charge && (world.ray(muzzle, c).is_some() || dist(c, muzzle) > 60.0) {
                        tgt = None;
                    }
                }
                if tgt.is_none() {
                    out.aim = [u.bearing, 0.0];
                }
            }
        }

        if self.held(t) {
            let mut face: Option<(f64, V3)> = None;
            for tr in &self.mem.tracks {
                if tr.kind != ActorKind::Mech || t - tr.last_seen > 2.5 {
                    continue;
                }
                let tp = tr.pos_at(t);
                let d = dist(tp, me.pos);
                let s = d - if t - tr.last_dmg_t < 3.0 { 200.0 } else { 0.0 };
                if t - tr.last_dmg_t < 3.0 && face.map_or(true, |f| s < f.0) {
                    face = Some((s, tp));
                }
            }
            if let Some((_, tp)) = face {
                tgt = None;
                out.aim = dir_to_aim(sub(up(tp, self.cfg.mech.height * 0.5), muzzle));
            }
        }
        let Some((tg, allow)) = tgt else {
            self.hold_lost_charge(obs, &mut out);
            if noblind {
                self.lz_hold_target(obs, world, &mut out);
            }
            self.precharge(obs, &mut out);
            for w in me.weapons.iter().flatten() {
                if w.kind == WeaponKind::Gun && w.reload <= 0.0 && w.ammo < w.stats.mag {
                    if w.ammo <= w.stats.mag * p.reload_frac || t - self.last_target_t > 1.5 {
                        out.reload = true;
                    }
                }
            }
            return out;
        };
        self.last_target_t = t;
        let c = up(tg.feet, tg.height * 0.5);
        let rel = sub(c, muzzle);
        let d = len(rel).max(0.1);
        let direct = dir_to_aim(add(rel, scale(tg.vel, p.laser_latency)));
        let los = world.ray(muzzle, c).is_none();
        let tol_y = (tg.radius * p.aim_tol).atan2(d);
        let tol_p = (tg.height * 0.5 * p.aim_tol).atan2(d);

        let mut gun: Option<(usize, V2, bool)> = None;
        let mut laser: Option<usize> = None;
        let mut lightning: Option<usize> = None;
        let mut mortar: Option<(usize, V2)> = None;
        for (i, w) in me.weapons.iter().enumerate() {
            let Some(w) = w else { continue };
            match w.kind {
                WeaponKind::Gun => {
                    let acc = if tg.fixed { [0.0; 3] } else { tg.acc };
                    let zon = p.aimz_on > 0.0 && !tg.fixed;
                    let lg = if zon { 0.0 } else { p.acc_gain };
                    if let Some((mut pt, tf)) = lead(rel, tg.vel, acc, w.stats.speed.max(1.0), p.latency, lg, p.acc_time) {
                        let mut pz_ok = true;
                        if zon {
                            let hl = pt[0].hypot(pt[1]).max(0.1);
                            let nn = [-pt[1] / hl, pt[0] / hl];
                            let hw = tg.radius + self.cfg.projectile.gun_radius;
                            if let Some((lat, pr)) = self.aimz.query(t, tg.feet, tf + p.latency, nn, hw, p.aimz_shrink, p.aimz_min_n.max(3.0) as usize, p.aimz_shot_min.max(0.0) as usize, p.aimz_gshot > 0.0) {
                                let cur = (pt[0] - rel[0]) * nn[0] + (pt[1] - rel[1]) * nn[1];
                                pt[0] += nn[0] * (lat - cur);
                                pt[1] += nn[1] * (lat - cur);
                                pz_ok = pr >= p.aimz_pmin;
                            }
                        }
                        let ok = pz_ok && len(pt) <= w.stats.range * p.gun_range_frac && world.ray(muzzle, add(muzzle, pt)).is_none();
                        gun = Some((i, dir_to_aim(pt), ok));
                    }
                }
                WeaponKind::Laser => laser = Some(i),
                WeaponKind::Lightning => lightning = Some(i),
                WeaponKind::Mortar => {
                    if w.cooldown <= 0.25 && allow {
                        mortar = self.mortar_solve(obs, world, &tg, w).map(|a| (i, a));
                    }
                    if w.cooldown <= 1.0 {
                        out.armed = true;
                    }
                }
                WeaponKind::Unknown => {}
            }
        }

        let laser_w = laser.and_then(|i| me.weapons[i].as_ref().map(|w| (i, w)));
        let laser_in = laser_w.map_or(false, |(_, w)| d <= w.stats.range * 0.98 && los);
        let laser_charging = laser_w.map_or(false, |(_, w)| w.charge > 0.0);
        let gun_ok = gun.map_or(false, |g| g.2);
        let mut cmd = match gun {
            Some((_, a, _)) => a,
            None => direct,
        };
        let mut laser_final = false;
        let mut early = false;
        // lz_early 2: early release only once free-for-all play is detected
        let early_on = p.lz_early > 0.0 && (p.lz_early < 1.5 || !self.squad_mode());
        if lz_on && early_on && laser_charging && laser_in {
            if let Some((_, w)) = laser_w {
                let full = w.stats.charge.max(0.05);
                let fr = w.stats.min_frac + (1.0 - w.stats.min_frac) * (w.charge / full).clamp(0.0, 1.0);
                early = w.charge >= w.stats.min_charge + 0.05 && fr >= p.lz_early_frac && !self.lz_clear_at(world, obs, &tg, p.lz_early_t);
            }
        }
        if let Some((_, w)) = laser_w {
            let full = w.stats.charge.max(0.05);
            if laser_charging && (w.charge >= full - 0.2 || early) {
                cmd = direct;
                laser_final = true;
            } else if !gun_ok && (laser_charging || laser_in) {
                cmd = direct;
            }
        }
        let mut mortar_primary = false;
        if !laser_final {
            if let Some((i, a)) = mortar {
                if me.weapons[i].as_ref().map_or(false, |w| w.cooldown <= 0.11) || !gun_ok {
                    cmd = a;
                    mortar_primary = true;
                }
            }
        }
        if !gun_ok && !mortar_primary && !laser_final {
            if let Some(i) = lightning {
                if me.weapons[i].as_ref().map_or(false, |w| d <= w.stats.range * 1.3) {
                    cmd = direct;
                }
            }
        }
        let pred = slew(me.aim, cmd, st.turn_rate * 0.05, st.pitch_rate * 0.05);
        let on = |a: V2, ty: f64, tp: f64| adiff(pred[0], a[0]).abs() * a[1].cos().max(0.2) <= ty && (pred[1] - a[1]).abs() <= tp;

        if let Some((i, a, ok)) = gun {
            if let Some(w) = me.weapons[i].as_ref() {
                if w.ammo <= 0.0 && w.reload <= 0.0 {
                    out.reload = true;
                }
                if !ok && w.reload <= 0.0 && w.ammo <= w.stats.mag * p.reload_frac && w.ammo < w.stats.mag {
                    out.reload = true;
                }
                if w.ammo > 0.0 && w.reload <= 0.7 {
                    out.armed = true;
                }
                if allow && ok && w.ammo > 0.0 && w.reload <= 0.0 && on(a, tol_y, tol_p) {
                    out.fire[i] = true;
                    if p.aimz_on > 0.0 && !tg.fixed {
                        self.aimz.note_fire(t, tg.feet);
                    }
                }
            }
        }
        if let Some((i, w)) = laser_w {
            let full = w.stats.charge.max(0.05);
            if w.cooldown <= 1.0 {
                out.armed = true;
            }
            if !laser_charging {
                if allow && laser_in && w.cooldown <= 0.1 {
                    let mut go = true;
                    if lz_on && p.lz_no_retreat > 0.0 && self.mode == Mode::Retreat {
                        go = tg.hp > 0.0 && w.stats.damage >= tg.hp * 1.05;
                    }
                    if go && lz_on && p.lz_fc > 0.0 {
                        let part = |c: f64| w.stats.damage * (w.stats.min_frac + (1.0 - w.stats.min_frac) * (c / full).clamp(0.0, 1.0));
                        let t2 = p.lz_fc_t2.min(full + 0.05).max(0.0);
                        let t1 = p.lz_fc_t1.clamp(0.0, t2);
                        let c1 = self.lz_clear_at(world, obs, &tg, t1);
                        if !(c1 && self.lz_clear_at(world, obs, &tg, t2)) {
                            // charge time left before the line closes
                            let te = (w.stats.min_charge + 0.05).max(0.1);
                            let avail = if c1 {
                                t1
                            } else if te < t1 && self.lz_clear_at(world, obs, &tg, te) {
                                te
                            } else {
                                0.0
                            };
                            let lethal = avail > 0.0 && tg.hp > 0.0 && part(avail) >= tg.hp * 1.05;
                            let planned = p.lz_fc_part > 0.0 && early_on && c1 && part(t1) >= w.stats.damage * p.lz_early_frac;
                            go = lethal || planned;
                        }
                    }
                    if go {
                        out.fire[i] = true;
                    }
                }
            } else {
                let ready = w.charge >= full - 0.051;
                let aligned = on(direct, tol_y, tol_p);
                let part = w.stats.damage * (w.stats.min_frac + (1.0 - w.stats.min_frac) * (w.charge / full).clamp(0.0, 1.0));
                let lethal = w.charge >= w.stats.min_charge && tg.hp > 0.0 && part >= tg.hp * 1.05;
                // the early shot leaves on the next tick: the torso must already be on the line
                let now_on = early && adiff(me.aim[0], direct[0]).abs() * direct[1].cos().max(0.2) <= tol_y && (me.aim[1] - direct[1]).abs() <= tol_p;
                let release = ((ready || lethal) && aligned && laser_in) || (now_on && laser_in);
                let give_up = !laser_in && w.charge >= full + (w.stats.hold_s - 0.4).max(0.0);
                out.fire[i] = !(release || give_up);
            }
            out.charging = out.fire[i];
            if noblind && out.fire[i] {
                self.lz_tid = tg.id;
            }
        }
        if let Some(i) = lightning {
            if let Some(w) = me.weapons[i].as_ref() {
                let in_range = d <= w.stats.range * 0.95 && dist(c, up(me.pos, st.height * 0.5)) <= w.stats.range * 0.95;
                if in_range {
                    out.armed = true;
                }
                let yaw_to = rel[1].atan2(rel[0]);
                let cone = w.stats.cone_deg.to_radians() * p.lightning_cone_frac;
                if allow && in_range && los && adiff(pred[0], yaw_to).abs() <= cone && me.energy >= p.lightning_min_energy.max(w.stats.energy) {
                    out.fire[i] = true;
                }
            }
        }
        if mortar_primary {
            if let Some((i, a)) = mortar {
                if on(a, (tg.radius + 1.5).atan2(d), 0.02) {
                    out.fire[i] = true;
                }
            }
        }
        out.aim = cmd;
        out.target = Some(tg);
        out
    }
}
