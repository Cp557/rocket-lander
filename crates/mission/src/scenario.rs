//! Starting conditions.

use fsw::Phase;
use glam::{DQuat, DVec3};
use serde::{Deserialize, Serialize};
use sim::{State, Vehicle};

#[derive(Clone, Copy, Debug, PartialEq, Eq, Serialize, Deserialize)]
pub enum Scenario {
    /// Falling base-first at 5 km, offset from the pad: aero steering then the landing burn.
    Landing,
    /// Stage separation at 70 km, 35 km downrange and still climbing: the whole return.
    FullReturn,
    /// Stage separation at 70 km with a drone ship waiting 140 km downrange, near where
    /// the booster would fall anyway: only a short boostback, then a landing at sea.
    DroneShip,
}

pub struct Start {
    pub state: State,
    pub phase: Phase,
    pub fins_deployed: bool,
}

impl Scenario {
    pub const ALL: [Scenario; 3] = [Scenario::Landing, Scenario::FullReturn, Scenario::DroneShip];

    pub fn label(self) -> &'static str {
        match self {
            Scenario::Landing => "Landing",
            Scenario::FullReturn => "Full return",
            Scenario::DroneShip => "Drone ship",
        }
    }

    /// Whether the booster lands on the drone ship rather than the landing pad.
    pub fn at_sea(self) -> bool {
        self == Scenario::DroneShip
    }

    pub fn start(self, vehicle: &Vehicle) -> Start {
        let pointing = |dir: DVec3| DQuat::from_rotation_arc(DVec3::Z, dir.normalize());
        match self {
            Scenario::Landing => {
                let vel = DVec3::new(-8.0, 4.0, -230.0);
                Start {
                    state: State {
                        pos: DVec3::new(250.0, -100.0, 5000.0),
                        vel,
                        att: pointing(-vel),
                        omega: DVec3::ZERO,
                        mass: vehicle.dry_mass + 4500.0,
                    },
                    phase: Phase::AeroDescent,
                    fins_deployed: true,
                }
            }
            Scenario::FullReturn | Scenario::DroneShip => {
                let vel = DVec3::new(750.0, 0.0, 650.0);
                // Missions that land at sea keep less propellant for the way back.
                let (downrange, propellant) = if self.at_sea() {
                    (-140_000.0, 26_000.0)
                } else {
                    (35_000.0, 32_000.0)
                };
                Start {
                    state: State {
                        pos: DVec3::new(downrange, 0.0, 70_000.0),
                        vel,
                        att: pointing(vel),
                        omega: DVec3::ZERO,
                        mass: vehicle.dry_mass + propellant,
                    },
                    phase: Phase::Flip,
                    fins_deployed: false,
                }
            }
        }
    }
}
