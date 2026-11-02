//! Truth-side simulation of the booster: rigid-body dynamics, engines, aerodynamics,
//! contact with the ground or the drone ship, wind and sensors. Knows nothing about flight
//! software.
//!
//! World frame: x east, y north, z up, origin on the ground at the landing pad center.
//! Body frame: origin at the center of mass, +z from engines toward the nose.

pub mod aero;
pub mod contact;
pub mod dispersion;
pub mod dynamics;
pub mod env;
pub mod sensors;
pub mod ship;
pub mod vehicle;
pub mod world;

pub use dispersion::Dispersion;
pub use dynamics::State;
pub use sensors::{DeckFix, GpsFix, SensorFrame};
pub use ship::Ship;
pub use vehicle::Vehicle;
pub use world::{Actuators, Commands, World};
