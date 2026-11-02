//! Batches of dispersed flights run in parallel without graphics.

use std::sync::atomic::{AtomicUsize, Ordering};

use rayon::prelude::*;
use serde::{Deserialize, Serialize};

use crate::flight::{Flight, FlightConfig, Outcome};
use crate::scenario::Scenario;

#[derive(Clone, Debug, PartialEq, Serialize, Deserialize)]
pub struct RunResult {
    pub seed: u64,
    pub outcome: Outcome,
}

/// Fly `runs` dispersed flights at `stress` with seeds `first_seed..`. `progress` counts
/// finished runs.
pub fn run(
    scenario: Scenario,
    runs: usize,
    first_seed: u64,
    stress: f64,
    progress: &AtomicUsize,
) -> Vec<RunResult> {
    (0..runs as u64)
        .into_par_iter()
        .map(|i| {
            let seed = first_seed + i;
            let log = Flight::new(FlightConfig {
                scenario,
                seed,
                dispersed: true,
                stress,
            })
            .run_to_end();
            progress.fetch_add(1, Ordering::Relaxed);
            RunResult {
                seed,
                outcome: log.outcome.expect("flights always end with an outcome"),
            }
        })
        .collect()
}

#[derive(Clone, Copy, Debug, Default, PartialEq)]
pub struct Summary {
    pub runs: usize,
    pub landed: usize,
    /// Miss distance over runs that touched down.
    pub mean_miss: f64,
    pub p95_miss: f64,
    pub max_vertical_speed: f64,
    pub min_propellant: f64,
}

pub fn summarize(results: &[RunResult]) -> Summary {
    let touchdowns: Vec<_> = results.iter().filter_map(|r| r.outcome.touchdown).collect();
    let mut misses: Vec<f64> = touchdowns.iter().map(|t| t.miss()).collect();
    misses.sort_by(f64::total_cmp);
    let percentile = |p: f64| {
        misses
            .get(((misses.len() as f64 - 1.0) * p).round() as usize)
            .copied()
            .unwrap_or(0.0)
    };
    Summary {
        runs: results.len(),
        landed: results.iter().filter(|r| r.outcome.landed).count(),
        mean_miss: if misses.is_empty() {
            0.0
        } else {
            misses.iter().sum::<f64>() / misses.len() as f64
        },
        p95_miss: percentile(0.95),
        max_vertical_speed: touchdowns
            .iter()
            .map(|t| t.vertical_speed)
            .fold(0.0, f64::max),
        min_propellant: touchdowns
            .iter()
            .map(|t| t.propellant)
            .fold(f64::INFINITY, f64::min),
    }
}
