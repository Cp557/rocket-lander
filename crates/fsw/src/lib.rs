//! Flight software: navigation, guidance and control.
//!
//! Inputs are `sim::SensorFrame`s and outputs are `sim::Commands`; the flight software
//! never reads the true state. It models the vehicle with `Vehicle::nominal()`.

pub mod computer;
pub mod control;
pub mod gfold;
pub mod guidance;
pub mod nav;
pub mod predict;
pub mod terminal;

pub use computer::{FlightComputer, Phase};
pub use gfold::Plan;
pub use nav::NavState;
