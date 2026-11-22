pub mod brain;
pub mod nav;
pub mod physics;
pub mod proto;
pub mod runtime;
pub mod world;

use std::panic::{catch_unwind, AssertUnwindSafe};

pub struct Shared {
    pub world: world::World,
    pub nav: nav::Nav,
}

impl Shared {
    /// Never fails: falls back to World::empty().
    pub fn load(map_dir: &std::path::Path, cfg: &proto::Config) -> Shared {
        let world = match catch_unwind(AssertUnwindSafe(|| world::World::load(map_dir))) {
            Ok(Ok(w)) => w,
            Ok(Err(e)) => {
                runtime::log(&format!("world load failed: {e}"));
                world::World::empty()
            }
            Err(_) => {
                runtime::log("world load panicked");
                world::World::empty()
            }
        };
        let nav = match catch_unwind(AssertUnwindSafe(|| nav::Nav::build(&world, &cfg.mech))) {
            Ok(n) => n,
            Err(_) => {
                runtime::log("nav build panicked, using empty world");
                nav::Nav::build(&world::World::empty(), &cfg.mech)
            }
        };
        Shared { world, nav }
    }
}

pub trait Pilot: Send {
    fn act(&mut self, obs: &proto::Obs) -> proto::Action;
}
