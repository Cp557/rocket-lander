//! Monte Carlo batches run on a background thread so the viewer stays responsive.

use std::sync::Arc;
use std::sync::atomic::{AtomicUsize, Ordering};
use std::thread::JoinHandle;

use bevy::prelude::*;
use mission::Scenario;
use mission::montecarlo::{self, RunResult, Summary};

pub struct Job {
    pub scenario: Scenario,
    pub stress: f64,
    pub total: usize,
    progress: Arc<AtomicUsize>,
    handle: JoinHandle<Vec<RunResult>>,
}

impl Job {
    pub fn done(&self) -> usize {
        self.progress.load(Ordering::Relaxed)
    }
}

pub struct Batch {
    pub scenario: Scenario,
    pub stress: f64,
    pub runs: Vec<RunResult>,
    pub summary: Summary,
}

#[derive(Resource, Default)]
pub struct MonteCarlo {
    pub job: Option<Job>,
    pub batch: Option<Batch>,
}

impl MonteCarlo {
    pub fn start(&mut self, scenario: Scenario, total: usize, first_seed: u64, stress: f64) {
        let progress = Arc::new(AtomicUsize::new(0));
        let counter = progress.clone();
        let handle = std::thread::spawn(move || {
            montecarlo::run(scenario, total, first_seed, stress, &counter)
        });
        self.job = Some(Job {
            scenario,
            stress,
            total,
            progress,
            handle,
        });
    }

    /// Collect a finished job's results.
    pub fn poll(&mut self) {
        if !self
            .job
            .as_ref()
            .is_some_and(|job| job.handle.is_finished())
        {
            return;
        }
        let job = self.job.take().expect("checked above");
        let runs = job.handle.join().expect("Monte Carlo thread panicked");
        let summary = montecarlo::summarize(&runs);
        self.batch = Some(Batch {
            scenario: job.scenario,
            stress: job.stress,
            runs,
            summary,
        });
    }
}
