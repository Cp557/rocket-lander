//! Deck motion prediction for the drone ship. The ship reports its deck a little late and
//! with noise; each motion (heave, pitch, roll) is close to a single swell, so a sinusoid
//! fitted to the recent reports predicts it seconds ahead.

use std::collections::VecDeque;

use glam::{DQuat, DVec2, DVec3, EulerRot};
use nalgebra::{Matrix3, Vector3};
use sim::DeckFix;

/// Reports kept for the fit (s), and how much history a fit needs.
const WINDOW: f64 = 40.0;
const MIN_HISTORY: f64 = 20.0;
const REFIT_PERIOD: f64 = 1.0;
/// Swell periods searched (s): a coarse grid, then a golden-section refinement.
const PERIOD_RANGE: (f64, f64) = (4.0, 16.0);
const PERIOD_GRID: usize = 120;
const REFINE_STEPS: usize = 25;

/// y(t) = offset + a sin(w t) + b cos(w t).
#[derive(Clone, Copy, Debug, Default)]
struct Sinusoid {
    offset: f64,
    a: f64,
    b: f64,
    omega: f64,
}

impl Sinusoid {
    /// Value, rate and acceleration at `t`.
    fn at(&self, t: f64) -> (f64, f64, f64) {
        let (s, c) = (self.omega * t).sin_cos();
        let w = self.omega;
        (
            self.offset + self.a * s + self.b * c,
            w * (self.a * c - self.b * s),
            -w * w * (self.a * s + self.b * c),
        )
    }
}

/// Least-squares sinusoid at one frequency, and its squared residual.
fn fit_at(samples: &[(f64, f64)], omega: f64) -> (Sinusoid, f64) {
    let (mut m, mut r) = (Matrix3::<f64>::zeros(), Vector3::<f64>::zeros());
    for &(t, y) in samples {
        let (s, c) = (omega * t).sin_cos();
        let basis = Vector3::new(1.0, s, c);
        m += basis * basis.transpose();
        r += basis * y;
    }
    let Some(x) = m.try_inverse().map(|inv| inv * r) else {
        return (Sinusoid::default(), f64::INFINITY);
    };
    let fit = Sinusoid {
        offset: x[0],
        a: x[1],
        b: x[2],
        omega,
    };
    let residual = samples
        .iter()
        .map(|&(t, y)| (y - fit.at(t).0).powi(2))
        .sum();
    (fit, residual)
}

/// The sinusoid that best fits the samples, searching the swell periods.
fn fit(samples: &[(f64, f64)]) -> Sinusoid {
    let (lo, hi) = PERIOD_RANGE;
    let period = |i: usize| lo + (hi - lo) * i as f64 / (PERIOD_GRID - 1) as f64;
    let cost = |p: f64| fit_at(samples, std::f64::consts::TAU / p).1;
    let costs: Vec<f64> = (0..PERIOD_GRID).map(|i| cost(period(i))).collect();
    let best = (0..PERIOD_GRID)
        .min_by(|&i, &j| costs[i].total_cmp(&costs[j]))
        .expect("nonempty grid");
    // Golden-section search between the neighbors of the best grid period.
    let (mut a, mut b) = (
        period(best.saturating_sub(1)),
        period((best + 1).min(PERIOD_GRID - 1)),
    );
    let golden = 0.5 * (5f64.sqrt() - 1.0);
    for _ in 0..REFINE_STEPS {
        let (c, d) = (b - golden * (b - a), a + golden * (b - a));
        if cost(c) < cost(d) {
            b = d;
        } else {
            a = c;
        }
    }
    fit_at(samples, std::f64::consts::TAU / (0.5 * (a + b))).0
}

/// The deck at an instant, as predicted.
#[derive(Clone, Copy, Debug)]
pub struct DeckState {
    /// Deck center.
    pub pos: DVec3,
    pub vel_z: f64,
    pub accel_z: f64,
    /// Tilt of the deck's normal toward +x and +y (rad), as a booster tilt is measured.
    pub tilt: DVec2,
}

#[derive(Default)]
pub struct DeckPredictor {
    /// Measurement time, then heave, pitch and roll.
    samples: VecDeque<(f64, [f64; 3])>,
    fits: Option<[Sinusoid; 3]>,
    last_fit: f64,
    center: DVec3,
}

impl DeckPredictor {
    /// Add a report measured at `measured_at` (the report time minus its latency).
    pub fn report(&mut self, measured_at: f64, fix: &DeckFix) {
        let (pitch, roll, _) = fix.att.to_euler(EulerRot::YXZ);
        self.samples
            .push_back((measured_at, [fix.pos.z, pitch, roll]));
        while self
            .samples
            .front()
            .is_some_and(|&(t, _)| t < measured_at - WINDOW)
        {
            self.samples.pop_front();
        }
        self.center = fix.pos.truncate().extend(0.0);
    }

    /// Refit the swell to the latest reports if a refit is due at time `now`.
    pub fn update(&mut self, now: f64) {
        let (Some(&(newest, _)), Some(&(oldest, _))) = (self.samples.back(), self.samples.front())
        else {
            return;
        };
        let due = self.fits.is_none() || now - self.last_fit >= REFIT_PERIOD;
        if newest - oldest >= MIN_HISTORY && due {
            let channel = |i: usize| -> Vec<(f64, f64)> {
                self.samples.iter().map(|&(t, y)| (t, y[i])).collect()
            };
            self.fits = Some([fit(&channel(0)), fit(&channel(1)), fit(&channel(2))]);
            self.last_fit = now;
        }
    }

    /// The deck at time `t`, from the fits once there is enough history (see `update`),
    /// otherwise as last reported. `None` before any report.
    pub fn at(&self, t: f64) -> Option<DeckState> {
        let &(_, latest) = self.samples.back()?;
        let ((heave, vel_z, accel_z), pitch, roll) = match self.fits {
            Some([h, p, r]) => (h.at(t), p.at(t).0, r.at(t).0),
            None => ((latest[0], 0.0, 0.0), latest[1], latest[2]),
        };
        let normal = DQuat::from_euler(EulerRot::YXZ, pitch, roll, 0.0) * DVec3::Z;
        Some(DeckState {
            pos: self.center + DVec3::Z * heave,
            vel_z,
            accel_z,
            tilt: DVec2::new(normal.x.atan2(normal.z), normal.y.atan2(normal.z)),
        })
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use sim::Ship;

    #[test]
    fn predicts_the_swell_seconds_ahead() {
        let ship = Ship::nominal();
        let mut predictor = DeckPredictor::default();
        // Noise-free reports at 10 Hz for 40 s.
        for i in 0..400 {
            let t = i as f64 * 0.1;
            let pose = ship.pose(t);
            predictor.report(
                t,
                &DeckFix {
                    pos: pose.origin,
                    vel: pose.vel,
                    att: pose.att,
                },
            );
        }
        let ahead = 45.0;
        let truth = ship.pose(ahead);
        predictor.update(40.0);
        let guess = predictor.at(ahead).expect("predicts");
        assert!((guess.pos.z - truth.origin.z).abs() < 0.02, "{guess:?}");
        assert!((guess.vel_z - truth.vel.z).abs() < 0.02, "{guess:?}");
        let normal = truth.att * DVec3::Z;
        assert!(
            (guess.tilt.x - normal.x.atan2(normal.z)).abs() < 1e-3,
            "{guess:?}"
        );
        assert!(
            (guess.tilt.y - normal.y.atan2(normal.z)).abs() < 1e-3,
            "{guess:?}"
        );
    }
}
