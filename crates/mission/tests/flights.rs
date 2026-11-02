//! End-to-end flights: the nominal scenarios must land, and flights must be reproducible.

use std::sync::atomic::AtomicUsize;

use mission::montecarlo;
use mission::{Flight, FlightConfig, FlightLog, Scenario};

fn fly(scenario: Scenario, seed: u64, dispersed: bool) -> FlightLog {
    Flight::new(FlightConfig {
        scenario,
        seed,
        dispersed,
        stress: 0.0,
    })
    .run_to_end()
}

#[test]
fn nominal_landing_lands_on_the_pad() {
    let outcome = fly(Scenario::Landing, 1, false).outcome.expect("finished");
    assert!(outcome.landed, "{}", outcome.verdict);
}

#[test]
fn nominal_full_return_lands_on_the_pad() {
    let log = fly(Scenario::FullReturn, 1, false);
    let outcome = log.outcome.as_ref().expect("finished");
    assert!(outcome.landed, "{}", outcome.verdict);
    // Every phase of the return happens, in order.
    let phases: Vec<&str> = log.events.iter().map(|e| e.text.as_str()).collect();
    let order = [
        "Boostback",
        "Coast",
        "Entry burn",
        "Aero descent",
        "Landing burn",
        "Landed",
    ];
    let mut at = 0;
    for phase in order {
        at += phases[at..]
            .iter()
            .position(|p| *p == phase)
            .unwrap_or_else(|| panic!("missing {phase} in {phases:?}"));
    }
}

#[test]
fn nominal_drone_ship_landing_lands_on_the_deck() {
    let log = fly(Scenario::DroneShip, 1, false);
    assert!(log.ship.is_some());
    let outcome = log.outcome.expect("finished");
    assert!(outcome.landed, "{}", outcome.verdict);
}

#[test]
fn full_return_survives_an_engine_out_in_the_entry_burn() {
    // At stress 1 this seed loses an outer engine five seconds into the entry burn.
    let log = Flight::new(FlightConfig {
        scenario: Scenario::FullReturn,
        seed: 13,
        dispersed: true,
        stress: 1.0,
    })
    .run_to_end();
    assert!(log.events.iter().any(|e| e.text == "Engine out"));
    let outcome = log.outcome.expect("finished");
    assert!(outcome.landed, "{}", outcome.verdict);
}

#[test]
fn flight_computer_reads_the_wind_before_ignition() {
    let (mut error, mut wind) = (0.0, 0.0);
    for seed in 1..=6 {
        let log = fly(Scenario::Landing, seed, true);
        let ignition = log
            .frames
            .iter()
            .find(|f| f.phase == fsw::Phase::LandingBurn)
            .expect("landing burn");
        error += (ignition.est_wind - ignition.wind).truncate().length();
        wind += ignition.wind.truncate().length();
    }
    // Gusts the estimator lags behind leave some error, but far less than assuming calm air.
    assert!(error < 0.5 * wind, "error {error:.1} vs wind {wind:.1}");
}

#[test]
fn same_seed_flies_the_same_flight() {
    let a = fly(Scenario::Landing, 7, true);
    let b = fly(Scenario::Landing, 7, true);
    assert_eq!(a.frames.len(), b.frames.len());
    assert_eq!(a.frames.last(), b.frames.last());
    assert_eq!(a.outcome, b.outcome);
}

#[test]
fn log_survives_a_save_and_load() {
    let log = fly(Scenario::Landing, 3, false);
    let path = std::env::temp_dir().join("rocket-lander-log-test.json");
    log.save(&path).expect("save");
    let loaded = FlightLog::load(&path).expect("load");
    std::fs::remove_file(&path).ok();
    assert_eq!(loaded.frames.len(), log.frames.len());
    assert_eq!(loaded.outcome, log.outcome);
}

#[test]
fn monte_carlo_reports_every_run() {
    let progress = AtomicUsize::new(0);
    let results = montecarlo::run(Scenario::Landing, 8, 100, 0.0, &progress);
    assert_eq!(results.len(), 8);
    assert_eq!(progress.into_inner(), 8);
    let summary = montecarlo::summarize(&results);
    assert_eq!(summary.runs, 8);
}
