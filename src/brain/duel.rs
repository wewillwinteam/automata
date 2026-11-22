//! Hit rates by distance and an expected-exchange model for the engagement distance.

/// Share of aimed shots that land, by distance (m), piecewise linear.
const SQUAD_GUN: [(f64, f64); 6] = [(20.0, 0.89), (60.0, 0.64), (100.0, 0.36), (140.0, 0.25), (180.0, 0.16), (220.0, 0.04)];
const SQUAD_LASER: [(f64, f64); 6] = [(20.0, 0.74), (60.0, 0.38), (100.0, 0.20), (140.0, 0.10), (180.0, 0.06), (220.0, 0.0)];
const TEAM_GUN: [(f64, f64); 6] = [(25.0, 0.94), (70.0, 0.75), (110.0, 0.48), (155.0, 0.32), (200.0, 0.17), (240.0, 0.08)];
const TEAM_LASER: [(f64, f64); 3] = [(0.0, 0.72), (200.0, 0.72), (250.0, 0.4)];

/// Laser hit rate the legacy threat weights were tuned for.
pub(crate) const LASER_REF: f64 = 0.72;

pub(crate) const GUN_DPS: f64 = 102.0;
pub(crate) const LASER_DPS: f64 = 131.0;
pub(crate) const LIGHTNING_DPS: f64 = 240.0;
pub(crate) const LIGHTNING_RANGE: f64 = 45.0;
const LIGHTNING_MARGIN: f64 = 6.0;
const GRID: f64 = 10.0;
/// The model must beat the legacy distance by this factor to replace it.
const GAIN: f64 = 0.9;

fn interp(c: &[(f64, f64)], d: f64) -> f64 {
    let n = c.len();
    if n == 0 || !d.is_finite() {
        return 0.0;
    }
    if d <= c[0].0 {
        return c[0].1;
    }
    for k in 1..n {
        if d <= c[k].0 {
            let (x0, y0) = c[k - 1];
            let (x1, y1) = c[k];
            let f = if x1 > x0 { (d - x0) / (x1 - x0) } else { 1.0 };
            return y0 + (y1 - y0) * f;
        }
    }
    c[n - 1].1
}

/// Gun hit rate at distance d for a squad (house) shooter or a team shooter.
pub(crate) fn gun_hit(squad: bool, d: f64) -> f64 {
    if squad {
        interp(&SQUAD_GUN, d)
    } else {
        interp(&TEAM_GUN, d)
    }
}

/// Laser hit rate at distance d for a squad (house) shooter or a team shooter.
pub(crate) fn laser_hit(squad: bool, d: f64) -> f64 {
    if squad {
        interp(&SQUAD_LASER, d)
    } else {
        interp(&TEAM_LASER, d)
    }
}

#[derive(Clone, Copy, Debug, Default, PartialEq)]
pub(crate) struct Loadout {
    pub gun: bool,
    pub laser: bool,
    pub lightning: bool,
}

/// Damage per second we expect to land at distance d (our shots follow the team curves).
pub(crate) fn outgoing(us: Loadout, d: f64) -> f64 {
    let mut s = 0.0;
    if us.gun {
        s += GUN_DPS * gun_hit(false, d);
    }
    if us.laser {
        s += LASER_DPS * laser_hit(false, d);
    }
    if us.lightning && d <= LIGHTNING_RANGE {
        s += LIGHTNING_DPS;
    }
    s
}

/// Damage per second one opponent is expected to land on us at distance d.
pub(crate) fn incoming(them: Loadout, squad: bool, d: f64) -> f64 {
    let mut s = 0.0;
    if them.gun {
        s += GUN_DPS * gun_hit(squad, d);
    }
    if them.laser {
        s += LASER_DPS * laser_hit(squad, d);
    }
    if them.lightning && d <= LIGHTNING_RANGE + LIGHTNING_MARGIN {
        s += LIGHTNING_DPS;
    }
    s
}

/// Cost of fighting at distance d: damage taken (plus a time cost) per point of damage dealt. Lower is better.
pub(crate) fn score(us: Loadout, them: Loadout, others: &[Loadout], squad: bool, d: f64, kappa: f64) -> f64 {
    let out = outgoing(us, d);
    if !(out > 1e-6) {
        return f64::INFINITY;
    }
    let mut inc = incoming(them, squad, d);
    for o in others {
        inc += incoming(*o, squad, d);
    }
    (inc + kappa.max(0.0)) / out
}

/// Grid distance with the lowest score, if any distance lets us deal damage.
pub(crate) fn raw_best(us: Loadout, them: Loadout, others: &[Loadout], squad: bool, legacy: f64, min: f64, max: f64, kappa: f64) -> Option<(f64, f64)> {
    let lo = if min.is_finite() { min.max(5.0) } else { 30.0 };
    let hi = if max.is_finite() { max } else { 180.0 };
    let mut best: Option<(f64, f64)> = None;
    let mut d = lo;
    while d <= hi + 1e-9 {
        let s = score(us, them, others, squad, d, kappa);
        if s.is_finite() {
            let better = match best {
                None => true,
                Some((bd, bs)) => s < bs - 1e-12 || ((s - bs).abs() <= 1e-12 && (d - legacy).abs() < (bd - legacy).abs()),
            };
            if better {
                best = Some((d, s));
            }
        }
        d += GRID;
    }
    best
}

/// Preferred engagement distance: the model's best distance when it clearly beats the legacy one, else the legacy one.
pub(crate) fn best_range(us: Loadout, them: Loadout, others: &[Loadout], squad: bool, legacy: f64, min: f64, max: f64, kappa: f64) -> f64 {
    let Some((d, s)) = raw_best(us, them, others, squad, legacy, min, max, kappa) else { return legacy };
    let base = score(us, them, others, squad, legacy, kappa);
    if s < GAIN * base {
        d
    } else {
        legacy
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    const GL: Loadout = Loadout { gun: true, laser: true, lightning: false };
    const G: Loadout = Loadout { gun: true, laser: false, lightning: false };

    #[test]
    fn curves() {
        assert!((gun_hit(true, 0.0) - 0.89).abs() < 1e-12);
        assert!((gun_hit(true, 80.0) - 0.50).abs() < 1e-12);
        assert!((gun_hit(true, 400.0) - 0.04).abs() < 1e-12);
        assert!((laser_hit(true, 60.0) - 0.38).abs() < 1e-12);
        assert!((laser_hit(true, 300.0)).abs() < 1e-12);
        assert!((gun_hit(false, 25.0) - 0.94).abs() < 1e-12);
        assert!((gun_hit(false, 90.0) - 0.615).abs() < 1e-12);
        assert!((laser_hit(false, 120.0) - 0.72).abs() < 1e-12);
        assert!((laser_hit(false, 225.0) - 0.56).abs() < 1e-12);
        assert_eq!(gun_hit(true, f64::NAN), 0.0);
        let mut prev = 1.0;
        for k in 0..60 {
            let v = gun_hit(true, k as f64 * 5.0);
            assert!(v <= prev + 1e-12);
            prev = v;
        }
    }

    #[test]
    fn matchups() {
        // laser + gun against a squad hunter with gun + laser: stand off
        let d = best_range(GL, GL, &[], true, 140.0, 30.0, 180.0, 30.0);
        eprintln!("laser+gun vs squad gun+laser: {d}");
        assert!(d >= 170.0);
        let d2 = best_range(GL, GL, &[GL, GL], true, 140.0, 30.0, 180.0, 30.0);
        eprintln!("laser+gun vs three squad gun+laser: {d2}");
        assert!(d2 >= 170.0);
        // laser + gun against a team bot with only a gun: stand off
        let d = best_range(GL, G, &[], false, 140.0, 30.0, 180.0, 30.0);
        eprintln!("laser+gun vs team gun: {d}");
        assert!(d >= 150.0);
        // symmetric team duel: unchanged
        let d = best_range(GL, GL, &[], false, 140.0, 30.0, 180.0, 30.0);
        eprintln!("team gun+laser vs gun+laser: {d}");
        assert_eq!(d, 140.0);
        let d = best_range(G, G, &[], false, 85.0, 30.0, 180.0, 30.0);
        eprintln!("team gun vs gun: {d}");
        assert_eq!(d, 85.0);
        // gun only against a squad hunter with gun + laser: the optimum is a little further out
        let (rd, rs) = raw_best(G, GL, &[], true, 85.0, 30.0, 180.0, 30.0).unwrap();
        let d = best_range(G, GL, &[], true, 85.0, 30.0, 180.0, 30.0);
        eprintln!("gun vs squad gun+laser: optimum {rd} (score {rs:.3}, legacy {:.3}), chosen {d}", score(G, GL, &[], true, 85.0, 30.0));
        assert!((90.0..=120.0).contains(&rd));
        assert!(d == 85.0 || (90.0..=120.0).contains(&d));
    }

    #[test]
    fn edge_cases() {
        // nothing to shoot with: keep the legacy distance
        let none = Loadout::default();
        assert_eq!(best_range(none, GL, &[], true, 90.0, 30.0, 180.0, 30.0), 90.0);
        // lightning in hand pulls the model inside its range
        let zap = Loadout { gun: true, laser: false, lightning: true };
        let d = best_range(zap, G, &[], true, 85.0, 30.0, 180.0, 30.0);
        assert!(d <= LIGHTNING_RANGE);
        // never outside the allowed band
        for legacy in [55.0, 85.0, 140.0] {
            for squad in [false, true] {
                let d = best_range(GL, GL, &[GL], squad, legacy, 30.0, 180.0, 30.0);
                assert!(d == legacy || (30.0..=180.0).contains(&d));
            }
        }
        // degenerate inputs
        assert_eq!(best_range(GL, GL, &[], true, 140.0, f64::NAN, f64::NAN, f64::NAN), best_range(GL, GL, &[], true, 140.0, 30.0, 180.0, 0.0));
        assert_eq!(best_range(GL, GL, &[], true, 140.0, 200.0, 100.0, 30.0), 140.0);
    }
}
