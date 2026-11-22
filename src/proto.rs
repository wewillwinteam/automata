//! mechfight/1 message types.

use serde::{Deserialize, Deserializer, Serialize};

pub type V2 = [f64; 2];
pub type V3 = [f64; 3];

// ---------------------------------------------------------------- lenient integer parsing

fn de_i64<'de, D: Deserializer<'de>>(d: D) -> Result<i64, D::Error> {
    let v: Option<f64> = Option::deserialize(d)?;
    Ok(v.map(|x| x as i64).unwrap_or(0))
}

fn de_opt_i64<'de, D: Deserializer<'de>>(d: D) -> Result<Option<i64>, D::Error> {
    let v: Option<f64> = Option::deserialize(d)?;
    Ok(v.map(|x| x as i64))
}

fn de_f64<'de, D: Deserializer<'de>>(d: D) -> Result<f64, D::Error> {
    let v: Option<f64> = Option::deserialize(d)?;
    Ok(v.unwrap_or(0.0))
}

fn is_false(b: &bool) -> bool {
    !*b
}

// ---------------------------------------------------------------- enums

#[derive(Serialize, Deserialize, Clone, Copy, Debug, PartialEq, Eq, Hash, Default)]
#[serde(rename_all = "lowercase")]
pub enum WeaponKind {
    #[default]
    Gun,
    Mortar,
    Laser,
    Lightning,
    #[serde(other)]
    Unknown,
}

impl WeaponKind {
    pub fn as_str(self) -> &'static str {
        match self {
            WeaponKind::Gun => "gun",
            WeaponKind::Mortar => "mortar",
            WeaponKind::Laser => "laser",
            WeaponKind::Lightning => "lightning",
            WeaponKind::Unknown => "unknown",
        }
    }
}

#[derive(Serialize, Deserialize, Clone, Copy, Debug, PartialEq, Eq, Hash, Default)]
#[serde(rename_all = "lowercase")]
pub enum ActorKind {
    #[default]
    Mech,
    Npc,
    #[serde(other)]
    Unknown,
}

#[derive(Serialize, Deserialize, Clone, Copy, Debug, PartialEq, Eq, Hash, Default)]
#[serde(rename_all = "lowercase")]
pub enum ProjKind {
    #[default]
    Gun,
    Mortar,
    Npc,
    #[serde(other)]
    Unknown,
}

#[derive(Serialize, Deserialize, Clone, Copy, Debug, PartialEq, Eq, Hash, Default)]
#[serde(rename_all = "lowercase")]
pub enum BeamKind {
    #[default]
    Laser,
    Lightning,
    #[serde(other)]
    Unknown,
}

#[derive(Serialize, Deserialize, Clone, Copy, Debug, PartialEq, Eq, Hash, Default)]
#[serde(rename_all = "lowercase")]
pub enum SoundDist {
    #[default]
    Near,
    Mid,
    Far,
    #[serde(other)]
    Unknown,
}

#[derive(Serialize, Deserialize, Clone, Copy, Debug, PartialEq, Eq, Hash)]
#[serde(rename_all = "lowercase")]
pub enum EffectName {
    Cookoff,
    Short,
    Concuss,
    #[serde(other)]
    Unknown,
}

#[derive(Serialize, Deserialize, Clone, Copy, Debug, PartialEq, Eq, Hash, Default)]
#[serde(rename_all = "lowercase")]
pub enum TakeFailReason {
    #[default]
    Gone,
    Range,
    Have,
    Full,
    Busy,
    #[serde(other)]
    Unknown,
}

// ---------------------------------------------------------------- hello / config

#[derive(Serialize, Deserialize, Clone, Debug)]
#[serde(default)]
pub struct Hello {
    pub protocol: String,
    #[serde(deserialize_with = "de_i64")]
    pub slot: i64,
    #[serde(deserialize_with = "de_i64")]
    pub seed: i64,
    pub map: MapInfo,
    pub config: Config,
}

impl Default for Hello {
    fn default() -> Self {
        Hello { protocol: "mechfight/1".into(), slot: 0, seed: 0, map: MapInfo::default(), config: Config::default() }
    }
}

#[derive(Serialize, Deserialize, Clone, Debug)]
#[serde(default)]
pub struct MapInfo {
    pub dir: String,
    pub files: Vec<String>,
}

impl Default for MapInfo {
    fn default() -> Self {
        MapInfo { dir: "/arena".into(), files: vec!["v2/world.json".into(), "v2/terrain.i16".into()] }
    }
}

#[derive(Serialize, Deserialize, Clone, Debug, Default)]
#[serde(default)]
pub struct Config {
    #[serde(rename = "match")]
    pub match_: MatchCfg,
    pub bot: BotCfg,
    pub mech: MechCfg,
    pub vision: VisionCfg,
    pub sound: SoundCfg,
    pub weapons: WeaponsCfg,
    pub projectile: ProjectileCfg,
    pub cubes: CubesCfg,
    pub crates: CratesCfg,
    pub loot: LootCfg,
    pub npc: NpcCfg,
    #[serde(skip_serializing_if = "Option::is_none")]
    pub drops: Option<DropsCfg>,
    pub zone: ZoneCfg,
    pub replay: ReplayCfg,
    pub debug: DebugCaps,
    pub upgrades: Vec<UpgradeDef>,
}

#[derive(Serialize, Deserialize, Clone, Debug)]
#[serde(default)]
pub struct MatchCfg {
    pub tick_hz: f64,
    pub decide_every: f64,
    pub duration_s: f64,
    pub mechs: f64,
    pub spawn_min_separation: f64,
    pub spawn_zone_fraction: f64,
    pub arena_margin: f64,
}

impl Default for MatchCfg {
    fn default() -> Self {
        MatchCfg { tick_hz: 20.0, decide_every: 2.0, duration_s: 480.0, mechs: 16.0, spawn_min_separation: 130.0, spawn_zone_fraction: 0.8, arena_margin: 30.0 }
    }
}

#[derive(Serialize, Deserialize, Clone, Debug)]
#[serde(default)]
pub struct BotCfg {
    pub tick_budget_ms: f64,
    pub hello_budget_ms: f64,
    pub coast_s: f64,
    pub offer_timeout_s: f64,
    pub max_line_bytes: f64,
}

impl Default for BotCfg {
    fn default() -> Self {
        BotCfg { tick_budget_ms: 50.0, hello_budget_ms: 20000.0, coast_s: 1.0, offer_timeout_s: 5.0, max_line_bytes: 65536.0 }
    }
}

/// config.mech and obs.self.stats.
#[derive(Serialize, Deserialize, Clone, Debug, PartialEq)]
#[serde(default)]
pub struct MechCfg {
    pub radius: f64,
    pub height: f64,
    pub eye_height: f64,
    pub muzzle_height: f64,
    pub hp: f64,
    pub walk_speed: f64,
    pub boost_speed: f64,
    pub ground_accel: f64,
    pub air_accel: f64,
    pub gravity: f64,
    pub jump_speed: f64,
    pub jet_accel: f64,
    pub max_rise_speed: f64,
    pub step_up: f64,
    pub max_slope_deg: f64,
    pub energy: f64,
    pub energy_regen: f64,
    pub energy_regen_delay: f64,
    pub boost_drain: f64,
    pub jet_drain: f64,
    pub turn_rate: f64,
    pub pitch_rate: f64,
    pub min_pitch: f64,
    pub max_pitch: f64,
    pub regen_delay: f64,
    pub regen_per_s: f64,
    pub charge_move_mult: f64,
    pub self_damage_mult: f64,
    pub armor: f64,
    pub rear_mult: f64,
    pub front_arc_deg: f64,
    pub near_range_bonus: f64,
    pub vision_bonus: f64,
    pub hearing: f64,
}

impl Default for MechCfg {
    fn default() -> Self {
        MechCfg {
            radius: 2.5,
            height: 9.0,
            eye_height: 8.0,
            muzzle_height: 7.0,
            hp: 1400.0,
            walk_speed: 11.7,
            boost_speed: 19.5,
            ground_accel: 58.5,
            air_accel: 12.0,
            gravity: 22.0,
            jump_speed: 11.0,
            jet_accel: 34.0,
            max_rise_speed: 10.0,
            step_up: 1.2,
            max_slope_deg: 50.0,
            energy: 100.0,
            energy_regen: 18.0,
            energy_regen_delay: 0.6,
            boost_drain: 17.0,
            jet_drain: 15.0,
            turn_rate: 4.0,
            pitch_rate: 3.0,
            min_pitch: -0.7,
            max_pitch: 1.3,
            regen_delay: 5.0,
            regen_per_s: 45.0,
            charge_move_mult: 0.5,
            self_damage_mult: 0.5,
            armor: 0.0,
            rear_mult: 1.5,
            front_arc_deg: 180.0,
            near_range_bonus: 0.0,
            vision_bonus: 0.0,
            hearing: 1.0,
        }
    }
}

#[derive(Serialize, Deserialize, Clone, Debug)]
#[serde(default)]
pub struct VisionCfg {
    pub range: f64,
    pub fov_deg: f64,
    pub near_range: f64,
    pub canopy_blocks: bool,
    pub show_enemy_hp: bool,
    pub memory_s: f64,
}

impl Default for VisionCfg {
    fn default() -> Self {
        VisionCfg { range: 200.0, fov_deg: 120.0, near_range: 45.0, canopy_blocks: true, show_enemy_hp: true, memory_s: 5.0 }
    }
}

#[derive(Serialize, Deserialize, Clone, Debug)]
#[serde(default)]
pub struct SoundCfg {
    pub bearing_noise_deg: f64,
    pub near: f64,
    pub mid: f64,
    pub step_speed: f64,
    pub step_every: f64,
    pub range: SoundRange,
}

impl Default for SoundCfg {
    fn default() -> Self {
        SoundCfg { bearing_noise_deg: 7.0, near: 60.0, mid: 160.0, step_speed: 5.0, step_every: 0.6, range: SoundRange::default() }
    }
}

#[derive(Serialize, Deserialize, Clone, Debug)]
#[serde(default)]
pub struct SoundRange {
    pub step: f64,
    pub boost: f64,
    pub reload: f64,
    pub gun: f64,
    pub mortar: f64,
    pub explosion: f64,
    pub laser_charge: f64,
    pub laser: f64,
    pub lightning: f64,
    pub jet: f64,
    pub death: f64,
    #[serde(rename = "crate")]
    pub crate_: f64,
    pub npc_gun: f64,
    pub drop: f64,
}

impl Default for SoundRange {
    fn default() -> Self {
        SoundRange {
            step: 40.0,
            boost: 90.0,
            reload: 60.0,
            gun: 320.0,
            mortar: 420.0,
            explosion: 420.0,
            laser_charge: 140.0,
            laser: 380.0,
            lightning: 160.0,
            jet: 90.0,
            death: 600.0,
            crate_: 220.0,
            npc_gun: 200.0,
            drop: 600.0,
        }
    }
}

impl SoundRange {
    pub fn get(&self, kind: &str) -> Option<f64> {
        Some(match kind {
            "step" => self.step,
            "boost" => self.boost,
            "reload" => self.reload,
            "gun" => self.gun,
            "mortar" => self.mortar,
            "explosion" => self.explosion,
            "laser_charge" => self.laser_charge,
            "laser" => self.laser,
            "lightning" => self.lightning,
            "jet" => self.jet,
            "death" => self.death,
            "crate" => self.crate_,
            "npc_gun" => self.npc_gun,
            "drop" => self.drop,
            _ => return None,
        })
    }
}

/// Stats of any weapon (config.weapons.* and obs.self.weapons[i].stats). Fields that do not apply are 0.
#[derive(Serialize, Deserialize, Clone, Debug, Default, PartialEq)]
#[serde(default)]
pub struct WeaponStats {
    pub damage: f64,
    pub cooldown: f64,
    pub speed: f64,
    pub range: f64,
    pub spread_deg: f64,
    pub mag: f64,
    pub reload: f64,
    pub splash: f64,
    pub splash_radius: f64,
    pub splash_edge: f64,
    pub gravity: f64,
    pub ttl: f64,
    pub knock: f64,
    pub knock_up: f64,
    pub drain: f64,
    pub charge: f64,
    pub min_charge: f64,
    pub min_frac: f64,
    pub hold_s: f64,
    pub pierce: f64,
    pub charge_turn_mult: f64,
    pub cookoff: f64,
    pub cone_deg: f64,
    pub chain: f64,
    pub chain_range: f64,
    pub chain_falloff: f64,
    pub energy: f64,
    pub short_s: f64,
}

impl WeaponStats {
    pub fn default_gun() -> Self {
        WeaponStats { damage: 45.0, cooldown: 0.24, speed: 252.0, range: 230.0, spread_deg: 0.6, mag: 12.0, reload: 2.4, ..Default::default() }
    }
    pub fn default_mortar() -> Self {
        WeaponStats {
            damage: 200.0,
            splash: 160.0,
            splash_radius: 7.0,
            splash_edge: 0.3,
            cooldown: 2.6,
            speed: 72.0,
            gravity: 22.0,
            ttl: 7.0,
            knock: 16.0,
            knock_up: 9.0,
            drain: 45.0,
            ..Default::default()
        }
    }
    pub fn default_laser() -> Self {
        WeaponStats {
            damage: 315.0,
            charge: 0.75,
            min_charge: 0.15,
            min_frac: 0.35,
            hold_s: 2.0,
            cooldown: 1.65,
            range: 280.0,
            pierce: 0.0,
            charge_turn_mult: 1.0,
            cookoff: 0.6,
            ..Default::default()
        }
    }
    pub fn default_lightning() -> Self {
        WeaponStats {
            damage: 60.0,
            cooldown: 0.25,
            range: 45.0,
            cone_deg: 30.0,
            chain: 2.0,
            chain_range: 14.0,
            chain_falloff: 0.7,
            energy: 3.0,
            short_s: 2.0,
            ..Default::default()
        }
    }
    pub fn default_for(kind: WeaponKind) -> Self {
        match kind {
            WeaponKind::Gun => Self::default_gun(),
            WeaponKind::Mortar => Self::default_mortar(),
            WeaponKind::Laser => Self::default_laser(),
            WeaponKind::Lightning => Self::default_lightning(),
            WeaponKind::Unknown => Self::default(),
        }
    }
    /// Missing fields take the values of base.
    fn merged(base: WeaponStats, raw: serde_json::Map<String, serde_json::Value>) -> WeaponStats {
        let mut v = serde_json::to_value(base.clone()).unwrap_or_default();
        if let Some(m) = v.as_object_mut() {
            for (k, x) in raw {
                if x.is_number() {
                    m.insert(k, x);
                }
            }
        }
        serde_json::from_value(v).unwrap_or(base)
    }
}

macro_rules! de_weapon {
    ($name:ident, $base:expr) => {
        fn $name<'de, D: Deserializer<'de>>(d: D) -> Result<WeaponStats, D::Error> {
            let raw: Option<serde_json::Map<String, serde_json::Value>> = Option::deserialize(d)?;
            Ok(WeaponStats::merged($base, raw.unwrap_or_default()))
        }
    };
}
de_weapon!(de_gun, WeaponStats::default_gun());
de_weapon!(de_mortar, WeaponStats::default_mortar());
de_weapon!(de_laser, WeaponStats::default_laser());
de_weapon!(de_lightning, WeaponStats::default_lightning());

#[derive(Serialize, Deserialize, Clone, Debug)]
#[serde(default)]
pub struct WeaponsCfg {
    #[serde(deserialize_with = "de_gun")]
    pub gun: WeaponStats,
    #[serde(deserialize_with = "de_mortar")]
    pub mortar: WeaponStats,
    #[serde(deserialize_with = "de_laser")]
    pub laser: WeaponStats,
    #[serde(deserialize_with = "de_lightning")]
    pub lightning: WeaponStats,
}

impl Default for WeaponsCfg {
    fn default() -> Self {
        WeaponsCfg {
            gun: WeaponStats::default_gun(),
            mortar: WeaponStats::default_mortar(),
            laser: WeaponStats::default_laser(),
            lightning: WeaponStats::default_lightning(),
        }
    }
}

impl WeaponsCfg {
    pub fn get(&self, kind: WeaponKind) -> Option<&WeaponStats> {
        match kind {
            WeaponKind::Gun => Some(&self.gun),
            WeaponKind::Mortar => Some(&self.mortar),
            WeaponKind::Laser => Some(&self.laser),
            WeaponKind::Lightning => Some(&self.lightning),
            WeaponKind::Unknown => None,
        }
    }
}

#[derive(Serialize, Deserialize, Clone, Debug)]
#[serde(default)]
pub struct ProjectileCfg {
    pub gun_radius: f64,
    pub npc_radius: f64,
}

impl Default for ProjectileCfg {
    fn default() -> Self {
        ProjectileCfg { gun_radius: 0.3, npc_radius: 0.3 }
    }
}

#[derive(Serialize, Deserialize, Clone, Debug)]
#[serde(default)]
pub struct CubesCfg {
    pub pickup_radius: f64,
    pub hp_per_cube: f64,
    pub damage_per_cube: f64,
    pub stat_cap: f64,
    pub death_drop_extra: f64,
    pub size_per_cube: f64,
    pub upgrade_thresholds: Vec<f64>,
}

impl Default for CubesCfg {
    fn default() -> Self {
        CubesCfg {
            pickup_radius: 3.5,
            hp_per_cube: 0.04,
            damage_per_cube: 0.04,
            stat_cap: 10.0,
            death_drop_extra: 2.0,
            size_per_cube: 0.05,
            upgrade_thresholds: vec![1.0, 3.0, 6.0, 10.0, 15.0, 21.0, 28.0, 36.0, 45.0],
        }
    }
}

#[derive(Serialize, Deserialize, Clone, Debug)]
#[serde(default)]
pub struct CratesCfg {
    pub count: f64,
    pub roof_fraction: f64,
    pub hp: f64,
    pub radius: f64,
    pub height: f64,
    pub cubes: f64,
}

impl Default for CratesCfg {
    fn default() -> Self {
        CratesCfg { count: 36.0, roof_fraction: 0.25, hp: 300.0, radius: 1.4, height: 2.4, cubes: 1.0 }
    }
}

#[derive(Serialize, Deserialize, Clone, Debug)]
#[serde(default)]
pub struct LootWeights {
    pub gun: f64,
    pub mortar: f64,
    pub laser: f64,
    pub lightning: f64,
}

impl Default for LootWeights {
    fn default() -> Self {
        LootWeights { gun: 0.0, mortar: 1.0, laser: 1.0, lightning: 1.0 }
    }
}

#[derive(Serialize, Deserialize, Clone, Debug)]
#[serde(default)]
pub struct LootCfg {
    pub pickup_radius: f64,
    pub equip_s: f64,
    pub floor_count: f64,
    pub floor_roof_fraction: f64,
    pub crate_weapon_chance: f64,
    pub death_drop: f64,
    pub weights: LootWeights,
}

impl Default for LootCfg {
    fn default() -> Self {
        LootCfg {
            pickup_radius: 4.0,
            equip_s: 1.0,
            floor_count: 40.0,
            floor_roof_fraction: 0.25,
            crate_weapon_chance: 0.5,
            death_drop: 1.0,
            weights: LootWeights::default(),
        }
    }
}

#[derive(Serialize, Deserialize, Clone, Debug)]
#[serde(default)]
pub struct NpcCfg {
    pub count: f64,
    pub hp: f64,
    pub radius: f64,
    pub height: f64,
    pub speed: f64,
    pub leash: f64,
    pub chase_leash: f64,
    pub aggro_range: f64,
    pub deaggro_range: f64,
    pub spawn_clear: f64,
    pub damage: f64,
    pub cooldown: f64,
    pub projectile_speed: f64,
    pub spread_deg: f64,
    pub cubes: f64,
}

impl Default for NpcCfg {
    fn default() -> Self {
        NpcCfg {
            count: 12.0,
            hp: 350.0,
            radius: 1.8,
            height: 3.5,
            speed: 5.0,
            leash: 30.0,
            chase_leash: 60.0,
            aggro_range: 45.0,
            deaggro_range: 90.0,
            spawn_clear: 0.0,
            damage: 25.0,
            cooldown: 1.0,
            projectile_speed: 90.0,
            spread_deg: 2.5,
            cubes: 2.0,
        }
    }
}

#[derive(Serialize, Deserialize, Clone, Debug)]
#[serde(default)]
pub struct DropsCfg {
    pub times: Vec<f64>,
    pub fall_s: f64,
    pub radius: f64,
    pub capture_s: f64,
    pub cubes: f64,
    pub repair: f64,
    pub spread: f64,
}

impl Default for DropsCfg {
    fn default() -> Self {
        DropsCfg { times: vec![70.0, 140.0, 210.0], fall_s: 25.0, radius: 8.0, capture_s: 4.0, cubes: 3.0, repair: 1.0, spread: 0.7 }
    }
}

#[derive(Serialize, Deserialize, Clone, Debug, Default)]
#[serde(default)]
pub struct ZonePhase {
    pub hold_s: f64,
    pub shrink_s: f64,
    pub radius: f64,
    pub dps: f64,
}

#[derive(Serialize, Deserialize, Clone, Debug)]
#[serde(default)]
pub struct ZoneCfg {
    pub start_center: V2,
    pub start_radius: f64,
    pub phases: Vec<ZonePhase>,
}

impl Default for ZoneCfg {
    fn default() -> Self {
        let p = |hold_s, shrink_s, radius, dps| ZonePhase { hold_s, shrink_s, radius, dps };
        ZoneCfg {
            start_center: [41.3, -72.8],
            start_radius: 560.0,
            phases: vec![
                p(60.0, 50.0, 380.0, 20.0),
                p(45.0, 40.0, 240.0, 35.0),
                p(35.0, 35.0, 140.0, 60.0),
                p(30.0, 30.0, 70.0, 100.0),
                p(25.0, 25.0, 25.0, 160.0),
                p(15.0, 25.0, 0.0, 260.0),
            ],
        }
    }
}

#[derive(Serialize, Deserialize, Clone, Debug)]
#[serde(default)]
pub struct ReplayCfg {
    pub frame_every: f64,
}

impl Default for ReplayCfg {
    fn default() -> Self {
        ReplayCfg { frame_every: 2.0 }
    }
}

#[derive(Serialize, Deserialize, Clone, Debug)]
#[serde(default)]
pub struct DebugCaps {
    pub max_lines: f64,
    pub max_points: f64,
    pub max_text: f64,
    pub max_text_chars: f64,
    pub max_bytes: f64,
    pub max_match_bytes: f64,
    pub hold_s: f64,
}

impl Default for DebugCaps {
    fn default() -> Self {
        DebugCaps { max_lines: 64.0, max_points: 64.0, max_text: 16.0, max_text_chars: 80.0, max_bytes: 8192.0, max_match_bytes: 8_000_000.0, hold_s: 0.5 }
    }
}

#[derive(Serialize, Deserialize, Clone, Debug, Default)]
#[serde(default)]
pub struct UpgradeDef {
    pub id: String,
    pub name: String,
    pub desc: String,
    #[serde(skip_serializing_if = "Option::is_none")]
    pub requires: Option<WeaponKind>,
    pub max: f64,
    pub effects: Vec<UpgradeEffect>,
}

#[derive(Serialize, Deserialize, Clone, Debug, Default)]
#[serde(default)]
pub struct UpgradeEffect {
    /// e.g. "mech.hp" or "weapons.gun.damage"
    pub path: String,
    #[serde(skip_serializing_if = "Option::is_none")]
    pub mul: Option<f64>,
    #[serde(skip_serializing_if = "Option::is_none")]
    pub add: Option<f64>,
}

// ---------------------------------------------------------------- observation

#[derive(Serialize, Deserialize, Clone, Debug, Default)]
#[serde(default)]
pub struct Obs {
    #[serde(deserialize_with = "de_i64")]
    pub tick: i64,
    #[serde(deserialize_with = "de_f64")]
    pub t: f64,
    #[serde(deserialize_with = "de_i64")]
    pub alive: i64,
    #[serde(rename = "self")]
    pub me: SelfObs,
    pub mechs: Vec<ActorObs>,
    pub projectiles: Vec<ProjectileObs>,
    pub crates: Vec<ItemObs>,
    pub cubes: Vec<ItemObs>,
    pub pickups: Vec<PickupObs>,
    pub sounds: Vec<SoundObs>,
    pub beams: Vec<BeamObs>,
    pub events: Vec<Event>,
    pub zone: ZoneObs,
    pub offer: Option<Offer>,
    pub drops: Vec<DropObs>,
}

#[derive(Serialize, Deserialize, Clone, Debug, Default)]
#[serde(default)]
pub struct SelfObs {
    #[serde(deserialize_with = "de_i64")]
    pub id: i64,
    pub pos: V3,
    pub vel: V3,
    pub aim: V2,
    pub hp: f64,
    pub max_hp: f64,
    pub energy: f64,
    pub max_energy: f64,
    pub grounded: bool,
    pub cubes: f64,
    pub upgrades: Vec<String>,
    pub weapons: [Option<WeaponObs>; 2],
    pub in_zone: bool,
    pub stats: MechCfg,
}

#[derive(Serialize, Deserialize, Clone, Debug, Default)]
#[serde(default)]
pub struct WeaponObs {
    pub kind: WeaponKind,
    pub cooldown: f64,
    /// gun only
    pub ammo: f64,
    /// gun only: seconds of reload left
    pub reload: f64,
    /// laser only: seconds charged so far
    pub charge: f64,
    pub stats: WeaponStats,
}

#[derive(Serialize, Deserialize, Clone, Debug, Default)]
#[serde(default)]
pub struct ActorObs {
    #[serde(deserialize_with = "de_i64")]
    pub id: i64,
    pub kind: ActorKind,
    pub pos: V3,
    pub vel: V3,
    pub aim: V2,
    #[serde(skip_serializing_if = "Option::is_none")]
    pub hp: Option<f64>,
    #[serde(skip_serializing_if = "Option::is_none")]
    pub max_hp: Option<f64>,
    pub cubes: f64,
    #[serde(skip_serializing_if = "Vec::is_empty")]
    pub weapons: Vec<WeaponKind>,
    #[serde(skip_serializing_if = "is_false")]
    pub charging: bool,
}

#[derive(Serialize, Deserialize, Clone, Debug, Default)]
#[serde(default)]
pub struct ProjectileObs {
    #[serde(deserialize_with = "de_i64")]
    pub id: i64,
    pub kind: ProjKind,
    pub pos: V3,
    pub vel: V3,
}

/// Crates (with hp) and cubes (no hp).
#[derive(Serialize, Deserialize, Clone, Debug, Default)]
#[serde(default)]
pub struct ItemObs {
    #[serde(deserialize_with = "de_i64")]
    pub id: i64,
    pub pos: V3,
    #[serde(skip_serializing_if = "Option::is_none")]
    pub hp: Option<f64>,
}

#[derive(Serialize, Deserialize, Clone, Debug, Default)]
#[serde(default)]
pub struct PickupObs {
    #[serde(deserialize_with = "de_i64")]
    pub id: i64,
    pub kind: WeaponKind,
    pub pos: V3,
}

#[derive(Serialize, Deserialize, Clone, Debug, Default)]
#[serde(default)]
pub struct SoundObs {
    pub kind: String,
    pub bearing: f64,
    pub dist: SoundDist,
    pub age: f64,
}

#[derive(Serialize, Deserialize, Clone, Debug, Default)]
#[serde(default)]
pub struct BeamObs {
    #[serde(deserialize_with = "de_i64")]
    pub by: i64,
    pub kind: BeamKind,
    pub path: Vec<V3>,
    pub age: f64,
}

#[derive(Serialize, Deserialize, Clone, Debug, Default)]
#[serde(default)]
pub struct DropObs {
    #[serde(deserialize_with = "de_i64")]
    pub id: i64,
    pub pos: V3,
    pub radius: f64,
    pub lands_in: f64,
    pub capture_s: f64,
    pub progress: f64,
    #[serde(deserialize_with = "de_opt_i64")]
    pub holder: Option<i64>,
}

#[derive(Serialize, Deserialize, Clone, Debug, Default)]
#[serde(default)]
pub struct ZoneObs {
    pub center: V2,
    pub radius: f64,
    pub next_center: V2,
    pub next_radius: f64,
    pub shrink_start: f64,
    pub shrink_end: f64,
    pub dps: f64,
}

#[derive(Serialize, Deserialize, Clone, Debug, Default)]
#[serde(default)]
pub struct Offer {
    #[serde(deserialize_with = "de_i64")]
    pub id: i64,
    pub choices: Vec<OfferChoice>,
    pub expires: f64,
}

#[derive(Serialize, Deserialize, Clone, Debug, Default)]
#[serde(default)]
pub struct OfferChoice {
    pub id: String,
    pub name: String,
    pub desc: String,
}

// ---------------------------------------------------------------- events

#[derive(Serialize, Deserialize, Clone, Debug)]
#[serde(tag = "type", rename_all = "snake_case")]
pub enum Event {
    Damaged(DamagedEv),
    Hit(HitEv),
    Effect(EffectEv),
    Drop(DropEv),
    Kill(KillEv),
    Pickup(PickupEv),
    Upgrade(UpgradeEv),
    Took(TookEv),
    Dropped(DroppedEv),
    TakeFailed(TakeFailedEv),
    #[serde(other)]
    Unknown,
}

#[derive(Serialize, Deserialize, Clone, Debug, Default)]
#[serde(default)]
pub struct DamagedEv {
    pub amount: f64,
    /// gun, mortar, laser, lightning, npc or zone
    pub weapon: String,
    /// null for the zone
    pub bearing: Option<f64>,
    /// null for the zone
    #[serde(deserialize_with = "de_opt_i64")]
    pub by: Option<i64>,
    #[serde(skip_serializing_if = "is_false")]
    pub rear: bool,
}

#[derive(Serialize, Deserialize, Clone, Debug, Default)]
#[serde(default)]
pub struct HitEv {
    #[serde(deserialize_with = "de_i64")]
    pub target: i64,
    pub amount: f64,
    pub weapon: String,
    #[serde(skip_serializing_if = "Option::is_none")]
    pub effect: Option<EffectName>,
    #[serde(skip_serializing_if = "is_false")]
    pub rear: bool,
}

#[derive(Serialize, Deserialize, Clone, Debug)]
#[serde(default)]
pub struct EffectEv {
    pub effect: EffectName,
    #[serde(deserialize_with = "de_opt_i64")]
    pub by: Option<i64>,
}

impl Default for EffectEv {
    fn default() -> Self {
        EffectEv { effect: EffectName::Unknown, by: None }
    }
}

#[derive(Serialize, Deserialize, Clone, Debug, Default)]
#[serde(default)]
pub struct DropEv {
    #[serde(deserialize_with = "de_i64")]
    pub id: i64,
    pub cubes: f64,
}

#[derive(Serialize, Deserialize, Clone, Debug, Default)]
#[serde(default)]
pub struct KillEv {
    #[serde(deserialize_with = "de_i64")]
    pub target: i64,
}

#[derive(Serialize, Deserialize, Clone, Debug, Default)]
#[serde(default)]
pub struct PickupEv {
    pub cubes: f64,
}

#[derive(Serialize, Deserialize, Clone, Debug, Default)]
#[serde(default)]
pub struct UpgradeEv {
    pub id: String,
}

#[derive(Serialize, Deserialize, Clone, Debug, Default)]
#[serde(default)]
pub struct TookEv {
    #[serde(deserialize_with = "de_i64")]
    pub id: i64,
    pub kind: WeaponKind,
    #[serde(deserialize_with = "de_i64")]
    pub slot: i64,
}

#[derive(Serialize, Deserialize, Clone, Debug, Default)]
#[serde(default)]
pub struct DroppedEv {
    #[serde(deserialize_with = "de_i64")]
    pub id: i64,
    pub kind: WeaponKind,
    #[serde(deserialize_with = "de_i64")]
    pub slot: i64,
}

#[derive(Serialize, Deserialize, Clone, Debug, Default)]
#[serde(default)]
pub struct TakeFailedEv {
    #[serde(deserialize_with = "de_i64")]
    pub id: i64,
    pub reason: TakeFailReason,
}

// ---------------------------------------------------------------- end

#[derive(Serialize, Deserialize, Clone, Debug, Default)]
#[serde(default)]
pub struct End {
    #[serde(deserialize_with = "de_i64")]
    pub place: i64,
    #[serde(deserialize_with = "de_i64")]
    pub kills: i64,
}

/// Every server message. Serializing adds "type".
#[derive(Serialize, Deserialize, Clone, Debug)]
#[serde(tag = "type", rename_all = "snake_case")]
pub enum ServerMsg {
    Hello(Box<Hello>),
    Obs(Box<Obs>),
    End(End),
    #[serde(other)]
    Unknown,
}

// ---------------------------------------------------------------- bot messages

#[derive(Serialize, Deserialize, Clone, Debug)]
#[serde(default)]
pub struct Ready {
    #[serde(rename = "type")]
    pub ty: String,
    pub name: String,
}

impl Ready {
    pub fn new(name: &str) -> Ready {
        Ready { ty: "ready".into(), name: name.into() }
    }
}

impl Default for Ready {
    fn default() -> Self {
        Ready::new("")
    }
}

#[derive(Serialize, Deserialize, Clone, Debug, Default, PartialEq)]
#[serde(default)]
pub struct Action {
    #[serde(deserialize_with = "de_i64")]
    pub tick: i64,
    #[serde(rename = "move", skip_serializing_if = "Option::is_none")]
    pub move_: Option<V2>,
    #[serde(skip_serializing_if = "Option::is_none")]
    pub boost: Option<bool>,
    #[serde(skip_serializing_if = "Option::is_none")]
    pub jump: Option<bool>,
    #[serde(skip_serializing_if = "Option::is_none")]
    pub aim: Option<V2>,
    #[serde(skip_serializing_if = "Option::is_none")]
    pub fire: Option<[bool; 2]>,
    #[serde(skip_serializing_if = "Option::is_none")]
    pub reload: Option<bool>,
    #[serde(skip_serializing_if = "Option::is_none", deserialize_with = "de_opt_i64")]
    pub pick: Option<i64>,
    #[serde(skip_serializing_if = "Option::is_none", deserialize_with = "de_opt_i64")]
    pub take: Option<i64>,
    #[serde(skip_serializing_if = "Option::is_none", deserialize_with = "de_opt_i64")]
    pub drop: Option<i64>,
    #[serde(skip_serializing_if = "Option::is_none")]
    pub debug: Option<DebugDraw>,
}

impl Action {
    pub fn new(tick: i64) -> Action {
        Action { tick, ..Default::default() }
    }
}

#[derive(Serialize, Deserialize, Clone, Debug, Default, PartialEq)]
#[serde(default)]
pub struct DebugDraw {
    #[serde(skip_serializing_if = "Vec::is_empty")]
    pub lines: Vec<DebugLine>,
    #[serde(skip_serializing_if = "Vec::is_empty")]
    pub points: Vec<DebugPoint>,
    #[serde(skip_serializing_if = "Vec::is_empty")]
    pub text: Vec<DebugText>,
    #[serde(skip_serializing_if = "Option::is_none")]
    pub label: Option<String>,
}

#[derive(Serialize, Deserialize, Clone, Debug, Default, PartialEq)]
#[serde(default)]
pub struct DebugLine {
    pub a: V3,
    pub b: V3,
    #[serde(skip_serializing_if = "Option::is_none")]
    pub color: Option<String>,
}

#[derive(Serialize, Deserialize, Clone, Debug, Default, PartialEq)]
#[serde(default)]
pub struct DebugPoint {
    pub at: V3,
    #[serde(skip_serializing_if = "Option::is_none")]
    pub color: Option<String>,
    #[serde(skip_serializing_if = "Option::is_none")]
    pub size: Option<f64>,
}

#[derive(Serialize, Deserialize, Clone, Debug, Default, PartialEq)]
#[serde(default)]
pub struct DebugText {
    pub text: String,
    #[serde(skip_serializing_if = "Option::is_none")]
    pub at: Option<V3>,
    #[serde(skip_serializing_if = "Option::is_none")]
    pub color: Option<String>,
}
