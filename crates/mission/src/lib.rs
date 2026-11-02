//! Closed-loop flights: couples the truth simulation to the flight software, records a
//! flight log, judges the landing, and runs Monte Carlo batches.

pub mod flight;
pub mod log;
pub mod montecarlo;
pub mod scenario;

pub use flight::{Flight, FlightConfig, Outcome, Touchdown};
pub use log::{Event, FlightLog, Frame, PlanRecord};
pub use scenario::Scenario;
