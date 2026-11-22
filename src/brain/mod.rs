mod aim;
mod aimz;
mod duel;
#[cfg(test)]
mod duel_tests;
mod kill;
mod keep;
mod laser;
mod econ;
mod gear;
mod ffa;
mod math;
mod memory;
mod order;
mod params;
mod plan;
mod squad;
mod roof;
mod safety;
mod strategy;
mod survive;
mod tele;
#[cfg(test)]
mod lab;
#[cfg(test)]
mod lab_tests;
#[cfg(test)]
mod tests;
#[cfg(test)]
mod laser_tests;
mod wave;

pub use params::Params;

use std::sync::Arc;

use crate::proto::{self, Action, MechCfg, Obs, Offer, SelfObs, WeaponKind};
use crate::runtime::Budget;
use crate::{nav, Pilot, Shared};
use math::*;
use memory::Memory;

#[derive(Clone, Copy, PartialEq, Eq, Debug)]
pub(crate) enum Mode {
    Idle,
    Zone,
    Fight,
    Retreat,
    Loot,
    Drop,
    Hunt,
    Roof,
}

#[derive(Clone, Copy, Debug)]
pub(crate) enum Goal {
    None,
    Nav { pos: V3, arrive: f64 },
    Field { center: V2, arrive: f64 },
    Range { pos: V3, pref: f64 },
}

#[derive(Clone, Copy, PartialEq, Eq, Debug)]
pub(crate) enum Tgt {
    None,
    Mech(i64),
    Npc(i64),
    Crate(i64),
}

pub struct Brain {
    pub(crate) cfg: proto::Config,
    pub(crate) p: Arc<Params>,
    pub(crate) shared: Arc<Shared>,
    pub(crate) rng: Rng,
    pub(crate) mem: Memory,
    pub(crate) mode: Mode,
    pub(crate) goal: Goal,
    pub(crate) goal_id: i64,
    pub(crate) pursuit: (i64, f64),
    pub(crate) target: Tgt,
    pub(crate) allow_crates: bool,
    pub(crate) field: Option<(V3, Arc<nav::Field>, f64)>,
    pub(crate) zone_field: Option<(V2, f64, Arc<nav::Field>)>,
    pub(crate) goal_unreachable: bool,
    pub(crate) cover: Option<(V3, f64)>,
    pub(crate) last_dir: V2,
    pub(crate) juke_side: f64,
    pub(crate) juke_next: f64,
    pub(crate) unstick_until: f64,
    pub(crate) unstick_dir: V2,
    pub(crate) last_target_t: f64,
    pub(crate) zone_b: bool,
    pub(crate) unstick_count: u32,
    pub(crate) unstick_level: u32,
    pub(crate) cmd_since: f64,
    pub(crate) wants_since: f64,
    pub(crate) moving_since: f64,
    pub(crate) t_act: std::time::Instant,
    pub(crate) no_los: (i64, f64),
    pub(crate) jet: Option<(V3, f64)>,
    pub(crate) sq: squad::Book,
    pub(crate) econ: econ::Econ,
    pub(crate) gear: gear::Gear,
    pub(crate) sf: safety::Safety,
    pub(crate) wave: wave::WaveState,
    pub(crate) ffa: ffa::Ffa,
    pub(crate) spot: Option<(V3, f64, bool)>,
    pub(crate) order: order::Order,
    pub(crate) order_has_passive: bool,
    pub(crate) roof: Option<Arc<roof::RoofMap>>,
    pub(crate) roof_sel: Option<usize>,
    pub(crate) roof_sel_t: f64,
    pub(crate) roof_on: Option<usize>,
    pub(crate) roof_rest: bool,
    pub(crate) roof_hold: bool,
    pub(crate) roof_jet: bool,
    pub(crate) roof_ban: Vec<(usize, f64)>,
    pub(crate) roof_step: Option<(i64, Option<V3>, f64)>,
    pub(crate) roof_first_t: f64,
    /// the engagement model picked a distance beyond the legacy one for the current target
    pub(crate) stand_off: bool,
    pub(crate) kills: u32,
    pub(crate) pk: laser::Peek,
    pub(crate) lz_tid: i64,
    pub(crate) tele: tele::Tele,
    pub(crate) aimz: aimz::AimZ,
    pub(crate) cad: aimz::Cadence,
    pub(crate) cad_until: f64,
    pub(crate) cad_axis: V2,
    #[cfg(test)]
    pub(crate) dbg: String,
    #[cfg(test)]
    pub(crate) frozen: bool,
}

impl Brain {
    pub fn new(hello: &proto::Hello, shared: Arc<Shared>) -> Brain {
        Brain::with_params(hello, shared, Params::load())
    }

    pub fn with_params(hello: &proto::Hello, shared: Arc<Shared>, params: Params) -> Brain {
        let mut rng = Rng::new(hello.seed as u64 ^ ((hello.slot as u64) << 40) ^ (params.salt.max(0.0) as u64).wrapping_mul(0x9E37_79B9_7F4A_7C15));
        let side = if rng.f64() < 0.5 { -1.0 } else { 1.0 };
        let roof = if params.roof_mode > 0.0 {
            let (mn, ms) = (params.roof_min_nodes.max(1.0) as usize, params.roof_min_setback * 0.5);
            std::panic::catch_unwind(std::panic::AssertUnwindSafe(|| Arc::new(roof::RoofMap::build(&shared.world, &shared.nav, mn, ms, hello.config.mech.height)))).ok()
        } else {
            None
        };
        Brain {
            cfg: hello.config.clone(),
            p: Arc::new(params),
            shared,
            rng,
            mem: Memory::new(),
            mode: Mode::Idle,
            goal: Goal::None,
            goal_id: -1,
            pursuit: (-1, 0.0),
            target: Tgt::None,
            allow_crates: false,
            field: None,
            zone_field: None,
            goal_unreachable: false,
            cover: None,
            last_dir: [0.0, 0.0],
            juke_side: side,
            juke_next: 0.0,
            unstick_until: -1.0,
            unstick_dir: [0.0, 0.0],
            last_target_t: -1e9,
            zone_b: false,
            unstick_count: 0,
            unstick_level: 0,
            cmd_since: 0.0,
            wants_since: 0.0,
            moving_since: 0.0,
            t_act: std::time::Instant::now(),
            no_los: (-1, 0.0),
            jet: None,
            sq: squad::Book::new(),
            econ: econ::Econ::default(),
            gear: gear::Gear::default(),
            sf: safety::Safety::default(),
            wave: wave::WaveState::new(),
            ffa: ffa::Ffa::new(hello.seed),
            spot: None,
            order: order::Order::default(),
            order_has_passive: false,
            roof,
            roof_sel: None,
            roof_sel_t: -1e9,
            roof_on: None,
            roof_rest: false,
            roof_hold: false,
            roof_jet: false,
            roof_ban: Vec::new(),
            roof_step: None,
            roof_first_t: -1.0,
            stand_off: false,
            kills: 0,
            pk: laser::Peek::new(),
            lz_tid: -1,
            tele: tele::Tele::default(),
            aimz: aimz::AimZ::new(),
            cad: aimz::Cadence::new(),
            cad_until: -1.0,
            cad_axis: [1.0, 0.0],
            #[cfg(test)]
            dbg: String::new(),
            #[cfg(test)]
            frozen: false,
        }
    }

    pub fn act(&mut self, obs: &Obs, budget: &Budget) -> Action {
        self.t_act = std::time::Instant::now();
        let mut act = Action::new(obs.tick);
        let shared = self.shared.clone();
        let world = &shared.world;
        let nav = &shared.nav;
        self.econ_update(obs);
        self.wave_count(obs);
        self.mem.range_ban = self.p.take_range_ban;
        self.mem.update(obs, world, &self.cfg);
        self.sf_update(obs, world);
        if self.p.aimz_on > 0.0 {
            self.aimz.update(obs);
        }
        if self.p.w_cad > 0.0 {
            self.cad.update(obs, self.p.cad_period, self.p.cad_snap);
        }
        self.wave_update(obs, world, nav);
        self.ffa.update(obs, &self.mem.tracks, self.p.ffa_on > 0.5 || self.p.mode_detect > 0.5);
        self.order.update(obs);
        self.tele_update(obs);
        if self.p.halt_multi >= 1.0 && !self.order.halted && self.order.on_us(obs.t) as f64 >= self.p.halt_multi {
            self.order.halted = true;
        }
        if let Some(of) = &obs.offer {
            act.pick = Some(self.pick_upgrade(of));
        }
        self.decide(obs, world, nav);
        self.laser_pre(obs, world, nav);
        let mut fc = self.fire_control(obs, world);
        if let Some(y) = self.gear_sweep(obs, fc.target.is_some() || fc.fire[0] || fc.fire[1] || fc.charging) {
            fc.aim = [y, 0.0];
        }
        let mut mv = self.plan_move(obs, world, nav, budget, &fc);
        if !mv.boost && self.gear_boost(obs) && mv.dir[0].hypot(mv.dir[1]) > 0.5 {
            mv.boost = true;
        }
        if let Some((j, b)) = self.gear_jet(obs) {
            mv.jump = mv.jump || j;
            mv.boost = mv.boost || b;
        }
        self.laser_post(obs, world, &mut fc, &mut mv);
        self.loot_action(obs, &mut act);
        self.sf_drop(obs, &mut act);
        let q = if mv.boost || self.mode != Mode::Idle { 1.0 } else { self.ffa_quiet(obs) };
        act.move_ = Some([mv.dir[0] * q, mv.dir[1] * q]);
        act.boost = Some(mv.boost);
        act.jump = Some(mv.jump);
        act.aim = Some(fc.aim);
        act.fire = Some(fc.fire);
        act.reload = Some(fc.reload);
        if (self.p.halt_after >= 0.0 && obs.t >= self.p.halt_after) || self.order.halted {
            act.move_ = Some([0.0, 0.0]);
            act.boost = Some(false);
            act.jump = Some(false);
            act.fire = Some([false, false]);
            act.reload = Some(false);
            act.take = None;
            act.drop = None;
        }
        sanitize(&mut act, &obs.me.stats);
        act
    }

    /// Opponents are assumed to be one squad until free-for-all play is detected.
    pub(crate) fn squad_mode(&self) -> bool {
        !(self.p.mode_detect > 0.5 && self.ffa.on)
    }

    pub(crate) fn held(&self, t: f64) -> bool {
        (self.p.hold_until >= 0.0 && t < self.p.hold_until) || (self.p.hold_k >= 1.0 && self.wave.kills as f64 >= self.p.hold_k && self.squad_mode())
    }

    /// Short state line for logs and tools.
    pub fn state(&self) -> String {
        format!("{:?} {:?} {:?} tracks={} items={}/{}/{} roof={:?}/{:?}/{} first={:.1}", self.mode, self.goal, self.target, self.mem.tracks.len(), self.mem.crates.len(), self.mem.cubes.len(), self.mem.pickups.len(), self.roof_sel, self.roof_on, self.roof_rest, self.roof_first_t)
    }

    pub(crate) fn spent_ms(&self) -> f64 {
        #[cfg(test)]
        if self.frozen {
            return 0.0;
        }
        self.t_act.elapsed().as_secs_f64() * 1000.0
    }

    pub fn unsticks(&self) -> u32 {
        self.unstick_count
    }

    fn pick_upgrade(&self, of: &Offer) -> i64 {
        for id in &self.p.upgrade_priority {
            if let Some(i) = of.choices.iter().take(3).position(|c| &c.id == id) {
                return i as i64;
            }
        }
        0
    }

    pub(crate) fn pref_of(&self, k: WeaponKind) -> f64 {
        match k {
            WeaponKind::Gun => self.p.pref_gun,
            WeaponKind::Laser => self.p.pref_laser,
            WeaponKind::Lightning => self.p.pref_lightning,
            WeaponKind::Mortar => self.p.pref_mortar,
            WeaponKind::Unknown => -1.0,
        }
    }

    /// None: not wanted. Some(None): goes into an empty slot. Some(Some(s)): swap out slot s.
    pub(crate) fn want(&self, me: &SelfObs, kind: WeaponKind) -> Option<Option<i64>> {
        if kind == WeaponKind::Mortar && self.sf_no_mortar() {
            return None;
        }
        if kind == WeaponKind::Unknown || me.weapons.iter().flatten().any(|w| w.kind == kind) {
            return None;
        }
        if me.weapons.iter().any(|w| w.is_none()) {
            return Some(None);
        }
        let mut worst: Option<(f64, usize)> = None;
        for (i, w) in me.weapons.iter().enumerate() {
            let Some(w) = w else { continue };
            if w.kind == WeaponKind::Laser && w.charge > 0.0 {
                continue;
            }
            let pv = self.pref_of(w.kind);
            if worst.map_or(true, |x| pv < x.0) {
                worst = Some((pv, i));
            }
        }
        match worst {
            Some((pv, i)) if pv + 0.5 < self.pref_of(kind) => Some(Some(i as i64)),
            _ => None,
        }
    }

    fn loot_action(&mut self, obs: &Obs, act: &mut Action) {
        let me = &obs.me;
        let reach = (self.cfg.loot.pickup_radius - 0.3).max(0.5);
        let mut best: Option<(f64, i64, Option<i64>)> = None;
        for pk in &obs.pickups {
            if dist2(pk.pos, me.pos) > reach || (pk.pos[2] - me.pos[2]).abs() > 3.7 {
                continue;
            }
            if self.mem.blacklisted(pk.id, obs.t) {
                continue;
            }
            if let Some(slot) = self.want(me, pk.kind) {
                let v = self.pref_of(pk.kind);
                if best.map_or(true, |b| v > b.0) {
                    best = Some((v, pk.id, slot));
                }
            }
        }
        if let Some((_, id, slot)) = best {
            act.take = Some(id);
            act.drop = slot;
        }
    }
}

fn sanitize(act: &mut Action, st: &MechCfg) {
    let mut m = act.move_.unwrap_or([0.0, 0.0]);
    if !m[0].is_finite() || !m[1].is_finite() {
        m = [0.0, 0.0];
    }
    let l = m[0].hypot(m[1]);
    if l > 1.0 {
        m = [m[0] / l * 0.999_999, m[1] / l * 0.999_999];
    }
    act.move_ = Some(m);
    let a = act.aim.unwrap_or([0.0, 0.0]);
    let (lo, hi) = if st.min_pitch < st.max_pitch { (st.min_pitch, st.max_pitch) } else { (-0.7, 1.3) };
    act.aim = Some([wrap(a[0]), fin(a[1], 0.0).clamp(lo, hi)]);
    if act.fire.is_none() {
        act.fire = Some([false, false]);
    }
    if let Some(pk) = act.pick {
        act.pick = Some(pk.clamp(0, 2));
    }
}

impl Pilot for Brain {
    fn act(&mut self, obs: &Obs) -> Action {
        Brain::act(self, obs, &Budget::unlimited())
    }
}
