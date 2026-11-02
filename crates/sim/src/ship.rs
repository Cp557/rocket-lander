//! The drone ship: a barge holding station at the origin whose deck heaves, pitches and
//! rolls in the swell. The booster lands on the deck or falls into the sea.

use glam::{DQuat, DVec3, EulerRot};
use rand::RngExt;
use rand::rngs::StdRng;
use serde::{Deserialize, Serialize};

/// Half the deck's length (along x) and width (along y).
pub const DECK_HALF_LENGTH: f64 = 45.0;
pub const DECK_HALF_WIDTH: f64 = 26.0;
/// The sea surface sits this far below the deck's mean height.
pub const FREEBOARD: f64 = 4.0;

/// One sinusoidal motion of the deck.
#[derive(Clone, Copy, Debug, PartialEq, Serialize, Deserialize)]
pub struct Swell {
    pub amplitude: f64,
    pub period: f64,
    pub phase: f64,
}

impl Swell {
    /// Value and rate of change at time `t`.
    fn at(&self, t: f64) -> (f64, f64) {
        let w = std::f64::consts::TAU / self.period;
        let angle = w * t + self.phase;
        (
            self.amplitude * angle.sin(),
            self.amplitude * w * angle.cos(),
        )
    }
}

#[derive(Clone, Copy, Debug, PartialEq, Serialize, Deserialize)]
pub struct Ship {
    /// Deck height (m), pitch about y and roll about x (rad).
    pub heave: Swell,
    pub pitch: Swell,
    pub roll: Swell,
}

/// Where the deck is and how it moves at an instant. The deck is the x-y plane of `att`
/// through `origin`.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct DeckPose {
    pub origin: DVec3,
    pub att: DQuat,
    pub vel: DVec3,
    /// Angular velocity, world frame.
    pub omega: DVec3,
}

impl DeckPose {
    /// Velocity of the deck at a world point.
    pub fn velocity_at(&self, point: DVec3) -> DVec3 {
        self.vel + self.omega.cross(point - self.origin)
    }
}

impl Ship {
    /// A moderate swell, used for undispersed flights.
    pub fn nominal() -> Self {
        Self {
            heave: Swell {
                amplitude: 0.5,
                period: 8.0,
                phase: 0.0,
            },
            pitch: Swell {
                amplitude: 1f64.to_radians(),
                period: 7.0,
                phase: 1.0,
            },
            roll: Swell {
                amplitude: 2f64.to_radians(),
                period: 9.5,
                phase: 2.0,
            },
        }
    }

    /// A random sea state, rougher with `stress`.
    pub fn sample(rng: &mut StdRng, stress: f64) -> Self {
        let nominal = Self::nominal();
        let mut swell = |s: Swell| Swell {
            amplitude: s.amplitude * rng.random_range(0.5..1.5) * (1.0 + stress),
            period: s.period * rng.random_range(0.8..1.2),
            phase: rng.random_range(0.0..std::f64::consts::TAU),
        };
        Self {
            heave: swell(nominal.heave),
            pitch: swell(nominal.pitch),
            roll: swell(nominal.roll),
        }
    }

    pub fn pose(&self, t: f64) -> DeckPose {
        let (heave, heave_rate) = self.heave.at(t);
        let (pitch, pitch_rate) = self.pitch.at(t);
        let (roll, roll_rate) = self.roll.at(t);
        DeckPose {
            origin: DVec3::new(0.0, 0.0, heave),
            att: DQuat::from_euler(EulerRot::YXZ, pitch, roll, 0.0),
            vel: DVec3::new(0.0, 0.0, heave_rate),
            omega: DVec3::new(roll_rate, pitch_rate, 0.0),
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn deck_rates_match_its_motion() {
        let ship = Ship::nominal();
        let (t, dt) = (3.0, 1e-4);
        let (a, b) = (ship.pose(t), ship.pose(t + dt));
        assert!(((b.origin - a.origin) / dt - a.vel).length() < 1e-3);
        // Small angles: the angular rates are the time derivatives of roll and pitch.
        let (pitch_a, roll_a, _) = a.att.to_euler(EulerRot::YXZ);
        let (pitch_b, roll_b, _) = b.att.to_euler(EulerRot::YXZ);
        assert!(((pitch_b - pitch_a) / dt - a.omega.y).abs() < 1e-3);
        assert!(((roll_b - roll_a) / dt - a.omega.x).abs() < 1e-3);
    }
}
