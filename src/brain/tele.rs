use crate::proto::{Event, Obs, WeaponKind};

use super::Brain;

#[derive(Default)]
pub(crate) struct Tele {
    kills: u32,
    nl_done: bool,
    wl_done: bool,
}

impl Brain {
    pub(crate) fn tele_update(&mut self, obs: &Obs) {
        for ev in &obs.events {
            if let Event::Kill(_) = ev {
                self.tele.kills += 1;
            }
        }
        let has = obs.me.weapons.iter().flatten().any(|w| w.kind == WeaponKind::Laser);
        if self.p.halt_nl >= 0.0 && !self.tele.nl_done && obs.t >= self.p.halt_nl {
            self.tele.nl_done = true;
            if !has {
                self.order.halted = true;
            }
        }
        if self.p.halt_wl >= 0.0 && !self.tele.wl_done && obs.t >= self.p.halt_wl {
            self.tele.wl_done = true;
            if has {
                self.order.halted = true;
            }
        }
        if self.p.halt_k >= 1.0 && self.tele.kills as f64 >= self.p.halt_k {
            self.order.halted = true;
        }
    }
}
