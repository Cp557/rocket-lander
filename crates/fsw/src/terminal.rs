//! Terminal guidance: model-predictive control of the sideways motion over the last few
//! hundred meters. It plans the tilt for the time left so the booster arrives upright (or
//! square to a tilted deck) with no sideways speed, near the pad, within tilt limits that
//! narrow toward the ground, knowing that the attitude takes time to follow a tilt command
//! and that the air pushes back on a body leaning across the flow.
//!
//! Each horizontal axis is planned on its own, with the state position p, velocity v, tilt
//! theta (of the body and thrust toward that axis) and tilt rate omega:
//!
//!   p' = v
//!   v' = (a - k) theta - (k / V) (v - w)
//!   theta' = omega
//!   omega' = kp (u - theta) - kd omega
//!
//! where a is the thrust acceleration, k = q S C_N' / m the body's normal force per radian
//! of angle of attack (small angles, falling base-first at air speed V through wind w),
//! and the last line is the closed attitude loop following the tilt command u. States are
//! eliminated (a condensed QP over u only, with box limits), and Clarabel solves it every
//! 0.1 s (the shortest plan step); the first command is flown until the next solve.

use clarabel::algebra::CscMatrix;
use clarabel::solver::{DefaultSettings, DefaultSolver, IPSolver, SolverStatus, SupportedConeT};
use nalgebra::{DMatrix, DVector, Matrix4, Matrix6, Vector4};

/// Height above touchdown below which terminal guidance takes over the sideways motion.
pub const HEIGHT: f64 = 700.0;
/// Plan steps: at least this long, and stretched so the plan reaches touchdown in at most
/// `MAX_STEPS`.
const MIN_STEP: f64 = 0.1;
const MIN_STEPS: usize = 5;
const MAX_STEPS: usize = 80;
/// Bryson-style scales: running position and velocity (per second of flight), and what
/// counts as large at touchdown.
const RUNNING_POSITION: f64 = 4.0;
const RUNNING_VELOCITY: f64 = 3.0;
const FINAL_POSITION: f64 = 2.0;
const FINAL_VELOCITY: f64 = 0.2;
const FINAL_TILT: f64 = 0.01;
const FINAL_TILT_RATE: f64 = 0.05;
/// Tilt command size (per second) and rate of change (rad/s) considered large.
const TILT: f64 = 0.3;
const TILT_CHANGE_RATE: f64 = 0.3;

/// One horizontal axis now: offset from the pad, velocity, tilt and tilt rate, and the
/// wind along it.
#[derive(Clone, Copy, Debug)]
pub struct Axis {
    pub position: f64,
    pub velocity: f64,
    pub tilt: f64,
    pub tilt_rate: f64,
    pub wind: f64,
}

/// The vehicle as terminal guidance sees it for the rest of the descent.
#[derive(Clone, Copy, Debug)]
pub struct Model {
    /// Thrust acceleration (m/s^2).
    pub accel: f64,
    /// Closed attitude loop: tilt acceleration per radian of tilt error and per rad/s.
    pub kp: f64,
    pub kd: f64,
    /// Body normal force per radian of angle of attack, as an acceleration, and the speed
    /// through the air.
    pub aero: f64,
    pub air_speed: f64,
    /// Seconds to touchdown.
    pub time_to_go: f64,
}

/// Exact discretization over one step: x+ = Phi x + Gamma u + Delta w, where w is the
/// wind (held constant).
fn discretize(m: &Model) -> (Matrix4<f64>, Vector4<f64>, Vector4<f64>) {
    let drag = m.aero / m.air_speed.max(1.0);
    let mut c = Matrix6::zeros();
    c[(0, 1)] = 1.0;
    c[(1, 1)] = -drag;
    c[(1, 2)] = m.accel - m.aero;
    c[(1, 5)] = drag;
    c[(2, 3)] = 1.0;
    c[(3, 2)] = -m.kp;
    c[(3, 3)] = -m.kd;
    c[(3, 4)] = m.kp;
    c *= step(m.time_to_go);
    let (mut exp, mut term) = (Matrix6::identity(), Matrix6::identity());
    for k in 1..30 {
        term = term * c / k as f64;
        exp += term;
    }
    (
        exp.fixed_view::<4, 4>(0, 0).into_owned(),
        exp.fixed_view::<4, 1>(0, 4).into_owned(),
        exp.fixed_view::<4, 1>(0, 5).into_owned(),
    )
}

/// Steps in the plan for the time left.
pub fn steps(time_to_go: f64) -> usize {
    ((time_to_go / step(time_to_go)).round() as usize).clamp(MIN_STEPS, MAX_STEPS)
}

/// Length of each plan step for the time left.
fn step(time_to_go: f64) -> f64 {
    (time_to_go / MAX_STEPS as f64).max(MIN_STEP)
}

/// The tilt to command now for one axis, given the tilt limit at each step of the plan and
/// the tilt to arrive at (a drone ship's deck may be tilted). `None` if the solver fails.
pub fn tilt_command(axis: Axis, model: &Model, limits: &[f64], final_tilt: f64) -> Option<f64> {
    let n = limits.len();
    let dt = step(model.time_to_go);
    let (phi, gamma, delta) = discretize(model);
    let x0 = Vector4::new(axis.position, axis.velocity, axis.tilt, axis.tilt_rate);
    let running = Matrix4::from_diagonal(&Vector4::new(
        dt / RUNNING_POSITION.powi(2),
        dt / RUNNING_VELOCITY.powi(2),
        0.0,
        0.0,
    ));
    let last = Matrix4::from_diagonal(&Vector4::new(
        FINAL_POSITION.powi(-2),
        FINAL_VELOCITY.powi(-2),
        FINAL_TILT.powi(-2),
        FINAL_TILT_RATE.powi(-2),
    ));

    // Cost u' H u + 2 f' u: x_k = Phi^k x0 + sum_j Phi^(k-1-j) Gamma u_j.
    let mut h = DMatrix::<f64>::zeros(n, n);
    let mut f = DVector::<f64>::zeros(n);
    let mut free = x0;
    let target = Vector4::new(0.0, 0.0, final_tilt, 0.0);
    // Columns of the input-to-state map for the current step, newest input last.
    let mut columns: Vec<Vector4<f64>> = Vec::with_capacity(n);
    for k in 1..=n {
        for col in &mut columns {
            *col = phi * *col;
        }
        columns.push(gamma);
        free = phi * free + delta * axis.wind;
        let (q, miss) = if k == n {
            (running + last, last * (free - target) + running * free)
        } else {
            (running, running * free)
        };
        for (i, ci) in columns.iter().enumerate() {
            f[i] += ci.dot(&miss);
            let qci = q * ci;
            for (j, cj) in columns.iter().enumerate() {
                h[(i, j)] += qci.dot(cj);
            }
        }
    }
    let effort = dt / TILT.powi(2);
    let change = (TILT_CHANGE_RATE * dt).powi(-2);
    for k in 0..n {
        let neighbors = (k > 0) as u8 + (k + 1 < n) as u8;
        h[(k, k)] += effort + change * neighbors as f64;
        if k + 1 < n {
            h[(k, k + 1)] -= change;
            h[(k + 1, k)] -= change;
        }
    }

    let (mut pi, mut pj, mut pv) = (Vec::new(), Vec::new(), Vec::new());
    for j in 0..n {
        for i in 0..=j {
            pi.push(i);
            pj.push(j);
            pv.push(2.0 * h[(i, j)]);
        }
    }
    let p = CscMatrix::new_from_triplets(n, n, pi, pj, pv);
    let q: Vec<f64> = f.iter().map(|v| 2.0 * v).collect();
    // |u_k| <= limit_k as u_k <= limit_k and -u_k <= limit_k.
    let rows: Vec<usize> = (0..2 * n).collect();
    let cols: Vec<usize> = (0..n).chain(0..n).collect();
    let vals: Vec<f64> = std::iter::repeat_n(1.0, n)
        .chain(std::iter::repeat_n(-1.0, n))
        .collect();
    let a = CscMatrix::new_from_triplets(2 * n, n, rows, cols, vals);
    let b: Vec<f64> = limits.iter().chain(limits).copied().collect();
    let cones = [SupportedConeT::NonnegativeConeT(2 * n)];
    let settings = DefaultSettings {
        verbose: false,
        max_iter: 50,
        ..Default::default()
    };
    let mut solver = DefaultSolver::new(&p, &q, &a, &b, &cones, settings).ok()?;
    solver.solve();
    matches!(
        solver.solution.status,
        SolverStatus::Solved | SolverStatus::AlmostSolved
    )
    .then(|| solver.solution.x[0])
}

#[cfg(test)]
mod tests {
    use super::*;

    fn model() -> Model {
        Model {
            accel: 12.0,
            kp: 3.0,
            kd: 3.0,
            aero: 0.0,
            air_speed: 50.0,
            time_to_go: 5.0,
        }
    }

    /// Fly the model with the controller in the loop; return the final state.
    fn fly(mut axis: Axis, m: &Model) -> Axis {
        let (phi, gamma, _) = discretize(m);
        let n = steps(m.time_to_go);
        for k in 0..n {
            let limits = vec![0.3; n - k];
            let u = tilt_command(axis, m, &limits, 0.0).expect("solves");
            let x = phi * Vector4::new(axis.position, axis.velocity, axis.tilt, axis.tilt_rate)
                + gamma * u;
            axis = Axis {
                position: x[0],
                velocity: x[1],
                tilt: x[2],
                tilt_rate: x[3],
                wind: 0.0,
            };
        }
        axis
    }

    #[test]
    fn arrives_still_and_upright() {
        let start = Axis {
            position: 25.0,
            velocity: -4.0,
            tilt: 0.05,
            tilt_rate: 0.0,
            wind: 0.0,
        };
        let end = fly(start, &model());
        // Well inside the 2 m/s sideways landing limit, and nearly upright.
        assert!(end.velocity.abs() < 1.0, "{end:?}");
        assert!(end.tilt.abs() < 0.03, "{end:?}");
        assert!(end.position.abs() < start.position.abs(), "{end:?}");
    }

    #[test]
    fn respects_the_tilt_limit() {
        let far = Axis {
            position: 200.0,
            velocity: 0.0,
            tilt: 0.0,
            tilt_rate: 0.0,
            wind: 0.0,
        };
        let u = tilt_command(far, &model(), &[0.1; 50], 0.0).expect("solves");
        assert!(u.abs() <= 0.1 + 1e-6, "{u}");
        // Toward the pad: a positive offset needs a negative tilt.
        assert!(u < 0.0);
    }

    #[test]
    fn leans_harder_against_the_air() {
        // With the body's lift pushing back on a lean, the same correction needs more tilt.
        let axis = Axis {
            position: 60.0,
            velocity: 0.0,
            tilt: 0.0,
            tilt_rate: 0.0,
            wind: 0.0,
        };
        let still = tilt_command(axis, &model(), &[0.4; 50], 0.0).expect("solves");
        let windy = Model {
            aero: 5.0,
            ..model()
        };
        let pushed = tilt_command(axis, &windy, &[0.4; 50], 0.0).expect("solves");
        assert!(pushed < still, "{pushed} vs {still}");
    }
}
