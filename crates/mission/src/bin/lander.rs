//! Headless command line: fly one flight, a Monte Carlo batch, or a stress sweep.
//!
//!   lander fly   [landing|full|ship] [--seed N] [--dispersed] [--stress S] [--save PATH]
//!   lander mc    [landing|full|ship] [--runs N] [--seed N] [--stress S] [--require PERCENT]
//!   lander sweep [landing|full|ship] [--runs N] [--seed N]

use std::path::PathBuf;
use std::sync::atomic::AtomicUsize;
use std::time::Instant;

use mission::montecarlo::{self, RunResult, Summary, summarize};
use mission::{Flight, FlightConfig, Scenario};

const USAGE: &str =
    "usage: lander fly [landing|full|ship] [--seed N] [--dispersed] [--stress S] [--save PATH]
       lander mc [landing|full|ship] [--runs N] [--seed N] [--stress S] [--require PERCENT]
       lander sweep [landing|full|ship] [--runs N] [--seed N]";
/// Stress levels `sweep` steps through.
const SWEEP: [f64; 5] = [0.0, 0.5, 1.0, 1.5, 2.0];

struct Args {
    command: String,
    scenario: Scenario,
    seed: u64,
    runs: usize,
    dispersed: bool,
    stress: f64,
    /// Exit with an error if fewer than this percentage of `mc` runs land.
    require: Option<f64>,
    save: Option<PathBuf>,
}

fn parse() -> Result<Args, String> {
    let mut args = std::env::args().skip(1);
    let command = args.next().ok_or("missing command: fly, mc or sweep")?;
    let mut parsed = Args {
        command,
        scenario: Scenario::FullReturn,
        seed: 1,
        runs: 100,
        dispersed: false,
        stress: 0.0,
        require: None,
        save: None,
    };
    while let Some(arg) = args.next() {
        let mut value = || args.next().ok_or(format!("{arg} needs a value"));
        match arg.as_str() {
            "landing" => parsed.scenario = Scenario::Landing,
            "full" => parsed.scenario = Scenario::FullReturn,
            "ship" => parsed.scenario = Scenario::DroneShip,
            "--seed" => parsed.seed = value()?.parse().map_err(|e| format!("--seed: {e}"))?,
            "--runs" => parsed.runs = value()?.parse().map_err(|e| format!("--runs: {e}"))?,
            "--stress" => {
                parsed.stress = value()?.parse().map_err(|e| format!("--stress: {e}"))?;
            }
            "--require" => {
                let percent = value()?.parse().map_err(|e| format!("--require: {e}"))?;
                parsed.require = Some(percent);
            }
            "--save" => parsed.save = Some(PathBuf::from(value()?)),
            "--dispersed" => parsed.dispersed = true,
            other => return Err(format!("unknown argument {other}")),
        }
    }
    Ok(parsed)
}

fn main() {
    let args = match parse() {
        Ok(args) => args,
        Err(e) => {
            eprintln!("{e}\n{USAGE}");
            std::process::exit(2);
        }
    };
    match args.command.as_str() {
        "fly" => fly(&args),
        "mc" => monte_carlo(&args),
        "sweep" => sweep(&args),
        other => {
            eprintln!("unknown command {other}\n{USAGE}");
            std::process::exit(2);
        }
    }
}

fn fly(args: &Args) {
    let started = Instant::now();
    let log = Flight::new(FlightConfig {
        scenario: args.scenario,
        seed: args.seed,
        dispersed: args.dispersed,
        stress: args.stress,
    })
    .run_to_end();
    for e in &log.events {
        println!("T+{:7.2}  {}", e.t, e.text);
    }
    if let Some(td) = log.outcome.as_ref().and_then(|o| o.touchdown) {
        println!(
            "touchdown: vz {:.2} m/s, vh {:.2} m/s, tilt {:.2} deg, rate {:.2} deg/s, miss {:.2} m, propellant {:.0} kg",
            td.vertical_speed,
            td.horizontal_speed,
            td.tilt_deg,
            td.tilt_rate_deg,
            td.miss(),
            td.propellant
        );
    }
    println!(
        "simulated {:.1} s in {:.2} s",
        log.duration(),
        started.elapsed().as_secs_f64()
    );
    if let Some(path) = &args.save {
        log.save(path).expect("save flight log");
        println!("saved {}", path.display());
    }
}

fn batch(args: &Args, stress: f64) -> Vec<RunResult> {
    montecarlo::run(
        args.scenario,
        args.runs,
        args.seed,
        stress,
        &AtomicUsize::new(0),
    )
}

fn landed_percent(s: &Summary) -> f64 {
    100.0 * s.landed as f64 / s.runs.max(1) as f64
}

fn monte_carlo(args: &Args) {
    let started = Instant::now();
    let results = batch(args, args.stress);
    let s = summarize(&results);
    println!(
        "{} (stress {}): {}/{} landed ({:.1}%), miss mean {:.1} m p95 {:.1} m, max vz {:.2} m/s, min propellant {:.0} kg, {:.1} s",
        args.scenario.label(),
        args.stress,
        s.landed,
        s.runs,
        landed_percent(&s),
        s.mean_miss,
        s.p95_miss,
        s.max_vertical_speed,
        s.min_propellant,
        started.elapsed().as_secs_f64()
    );
    for r in results.iter().filter(|r| !r.outcome.landed) {
        println!("  seed {}: {}", r.seed, r.outcome.verdict);
    }
    if let Some(required) = args.require
        && landed_percent(&s) < required
    {
        eprintln!("landed fewer than the required {required}%");
        std::process::exit(1);
    }
}

/// Success rate as the stress grows, with the most common way the failures went.
fn sweep(args: &Args) {
    println!(
        "{}, {} runs per level\nstress  landed  miss mean/p95  most common failure",
        args.scenario.label(),
        args.runs
    );
    for stress in SWEEP {
        let results = batch(args, stress);
        let s = summarize(&results);
        println!(
            "{stress:6.1}  {:5.1}%  {:5.1} / {:5.1} m  {}",
            landed_percent(&s),
            s.mean_miss,
            s.p95_miss,
            most_common_failure(&results)
        );
    }
}

fn most_common_failure(results: &[RunResult]) -> String {
    // Group verdicts by their kind, dropping the numbers ("sliding (3.1 m/s)" -> "sliding").
    let kind = |verdict: &str| {
        let mut words = verdict.split(|c: char| c.is_ascii_digit() || c == '(');
        words.next().unwrap_or_default().trim().to_string()
    };
    let mut counts: Vec<(String, usize)> = Vec::new();
    for r in results.iter().filter(|r| !r.outcome.landed) {
        let k = kind(&r.outcome.verdict);
        match counts.iter_mut().find(|(name, _)| *name == k) {
            Some((_, n)) => *n += 1,
            None => counts.push((k, 1)),
        }
    }
    counts
        .into_iter()
        .max_by_key(|(_, n)| *n)
        .map_or("-".to_string(), |(name, n)| format!("{name} ({n})"))
}
