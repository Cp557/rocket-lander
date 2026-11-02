//! Fuel-optimal powered-descent guidance by convex optimization (G-FOLD style).
//!
//! Decision variables at each node k: position r, velocity v, thrust acceleration u = T/m
//! and a slack sigma >= |u|. The non-convex lower thrust bound |T| >= T_min is replaced by
//! sigma >= T_min/m; "lossless convexification" (Acikmese & Blackmore) shows the optimum
//! still has |u| = sigma, so the relaxed second-order cone program is exact.
//!
//! Simplifications: the mass along the trajectory is bounded with a conservative profile
//! instead of being a variable, and drag is a linear term fitted at the current speed.
//! A small penalty on thrust changes between nodes turns the pure fuel optimum (bang-bang
//! throttle) into ramps that feedback can track while the plan is re-solved in flight.

use clarabel::algebra::CscMatrix;
use clarabel::solver::{DefaultSettings, DefaultSolver, IPSolver, SolverStatus, SupportedConeT};
use glam::DVec3;

pub const INTERVALS: usize = 30;
/// Weight of the squared thrust-acceleration change between nodes.
const SMOOTHING: f64 = 0.05;
/// The last nodes are held to `terminal_thrust_max` so the final meters are gentle.
const TERMINAL_NODES: usize = 3;

#[derive(Clone, Copy, Debug)]
pub struct Problem {
    pub pos: DVec3,
    pub vel: DVec3,
    pub mass: f64,
    pub target_pos: DVec3,
    pub target_vel: DVec3,
    pub thrust_min: f64,
    pub thrust_max: f64,
    /// Thrust ceiling for the last `TERMINAL_NODES` nodes.
    pub terminal_thrust_max: f64,
    /// Propellant mass flow per newton of thrust.
    pub flow_per_newton: f64,
    pub gravity: f64,
    /// Linear drag model at each node: drag acceleration = -drag_rate * (velocity - wind).
    pub drag_rate: [f64; INTERVALS + 1],
    /// Expected wind at each node.
    pub wind: [DVec3; INTERVALS + 1],
    /// Maximum thrust angle from vertical at each node.
    pub max_tilt: [f64; INTERVALS + 1],
    /// Air-relative retrograde direction at the start; thrust must stay near it while dynamic
    /// pressure is high.
    pub retrograde: DVec3,
    /// Allowed thrust angle from `retrograde` at each node.
    pub aero_angle: [f64; INTERVALS + 1],
    /// Minimum elevation angle of the vehicle as seen from the target.
    pub glideslope: f64,
}

#[derive(Clone, Debug, PartialEq)]
pub struct Plan {
    /// Flight-computer time of the first node.
    pub start: f64,
    pub dt: f64,
    pub pos: Vec<DVec3>,
    pub vel: Vec<DVec3>,
    /// Thrust acceleration.
    pub accel: Vec<DVec3>,
    /// Integral of thrust acceleration (proportional to propellant used).
    pub cost: f64,
}

impl Plan {
    pub fn end(&self) -> f64 {
        self.start + self.dt * INTERVALS as f64
    }

    /// Linear interpolation of (position, velocity, thrust acceleration) at time `t`.
    pub fn sample(&self, t: f64) -> (DVec3, DVec3, DVec3) {
        let s = ((t - self.start) / self.dt).clamp(0.0, INTERVALS as f64);
        let k = (s.floor() as usize).min(INTERVALS - 1);
        let f = s - k as f64;
        let lerp = |v: &[DVec3]| v[k].lerp(v[k + 1], f);
        (lerp(&self.pos), lerp(&self.vel), lerp(&self.accel))
    }
}

const VARS_PER_NODE: usize = 10;
fn r(k: usize, a: usize) -> usize {
    k * VARS_PER_NODE + a
}
fn v(k: usize, a: usize) -> usize {
    k * VARS_PER_NODE + 3 + a
}
fn u(k: usize, a: usize) -> usize {
    k * VARS_PER_NODE + 6 + a
}
fn sigma(k: usize) -> usize {
    k * VARS_PER_NODE + 9
}

/// Rows of `A x + s = b`, grouped by cone in the order they are added.
#[derive(Default)]
struct Constraints {
    rows: usize,
    i: Vec<usize>,
    j: Vec<usize>,
    a: Vec<f64>,
    b: Vec<f64>,
    cones: Vec<SupportedConeT<f64>>,
}

impl Constraints {
    fn row(&mut self, terms: &[(usize, f64)], rhs: f64) {
        for &(col, val) in terms {
            self.i.push(self.rows);
            self.j.push(col);
            self.a.push(val);
        }
        self.b.push(rhs);
        self.rows += 1;
    }
}

/// Solve with a fixed flight time. Returns `None` when infeasible.
pub fn solve(p: &Problem, duration: f64, start: f64) -> Option<Plan> {
    let n = INTERVALS;
    let h = duration / n as f64;
    let vars = (n + 1) * VARS_PER_NODE;
    let g = DVec3::new(0.0, 0.0, -p.gravity);
    let (pos, vel) = (p.pos.to_array(), p.vel.to_array());
    let (target, target_vel) = (p.target_pos.to_array(), p.target_vel.to_array());
    let mut c = Constraints::default();

    // Equalities: boundary conditions and trapezoidal dynamics with linear drag.
    let first = c.rows;
    for a in 0..3 {
        c.row(&[(r(0, a), 1.0)], pos[a]);
        c.row(&[(v(0, a), 1.0)], vel[a]);
        c.row(&[(r(n, a), 1.0)], target[a]);
        c.row(&[(v(n, a), 1.0)], target_vel[a]);
    }
    // Land with the thrust (and so the body) vertical.
    c.row(&[(u(n, 0), 1.0)], 0.0);
    c.row(&[(u(n, 1), 1.0)], 0.0);
    for k in 0..n {
        let (d0, d1) = (0.5 * h * p.drag_rate[k], 0.5 * h * p.drag_rate[k + 1]);
        for a in 0..3 {
            let terms = [
                (v(k + 1, a), 1.0 + d1),
                (v(k, a), -1.0 + d0),
                (u(k, a), -0.5 * h),
                (u(k + 1, a), -0.5 * h),
            ];
            // Gravity plus the constant part of the drag, which pushes along the wind.
            c.row(&terms, h * g[a] + d0 * p.wind[k][a] + d1 * p.wind[k + 1][a]);
            c.row(
                &[
                    (r(k + 1, a), 1.0),
                    (r(k, a), -1.0),
                    (v(k, a), -0.5 * h),
                    (v(k + 1, a), -0.5 * h),
                ],
                0.0,
            );
        }
    }
    c.cones.push(SupportedConeT::ZeroConeT(c.rows - first));

    // Linear inequalities: thrust magnitude bounds (via sigma) and two pointing cones
    // (u . axis >= cos(angle) * sigma is linear because sigma >= |u|).
    // The heaviest possible mass bounds the top, the lightest bounds the bottom.
    let first = c.rows;
    for k in 0..=n {
        let t = k as f64 * h;
        let heaviest = p.mass - p.flow_per_newton * p.thrust_min * t;
        let lightest = p.mass - p.flow_per_newton * p.thrust_max * t;
        let ceiling = if k + TERMINAL_NODES > n {
            p.terminal_thrust_max
        } else {
            p.thrust_max
        };
        c.row(&[(sigma(k), 1.0)], ceiling / heaviest);
        c.row(&[(sigma(k), -1.0)], -p.thrust_min / lightest);
        c.row(&[(sigma(k), p.max_tilt[k].cos()), (u(k, 2), -1.0)], 0.0);
        let d = p.retrograde;
        c.row(
            &[
                (sigma(k), p.aero_angle[k].cos()),
                (u(k, 0), -d.x),
                (u(k, 1), -d.y),
                (u(k, 2), -d.z),
            ],
            0.0,
        );
    }
    c.cones
        .push(SupportedConeT::NonnegativeConeT(c.rows - first));

    // Second-order cones: |u_k| <= sigma_k, and the glideslope.
    for k in 0..=n {
        c.row(&[(sigma(k), -1.0)], 0.0);
        for a in 0..3 {
            c.row(&[(u(k, a), -1.0)], 0.0);
        }
        c.cones.push(SupportedConeT::SecondOrderConeT(4));
    }
    let cot = 1.0 / p.glideslope.tan();
    for k in 1..n {
        c.row(&[(r(k, 2), -cot)], -target[2] * cot);
        c.row(&[(r(k, 0), -1.0)], -target[0]);
        c.row(&[(r(k, 1), -1.0)], -target[1]);
        c.cones.push(SupportedConeT::SecondOrderConeT(3));
    }

    let mut q = vec![0.0; vars];
    for k in 0..=n {
        q[sigma(k)] = if k == 0 || k == n { 0.5 * h } else { h };
    }
    let a = CscMatrix::new_from_triplets(c.rows, vars, c.i, c.j, c.a);
    let p_matrix = smoothing_matrix(n, vars);
    let settings = DefaultSettings {
        verbose: false,
        max_iter: 100,
        ..Default::default()
    };
    let mut solver = DefaultSolver::new(&p_matrix, &q, &a, &c.b, &c.cones, settings).ok()?;
    solver.solve();
    if !matches!(
        solver.solution.status,
        SolverStatus::Solved | SolverStatus::AlmostSolved
    ) {
        return None;
    }
    let x = &solver.solution.x;
    let node = |f: fn(usize, usize) -> usize, k| DVec3::new(x[f(k, 0)], x[f(k, 1)], x[f(k, 2)]);
    Some(Plan {
        start,
        dt: h,
        pos: (0..=n).map(|k| node(r, k)).collect(),
        vel: (0..=n).map(|k| node(v, k)).collect(),
        accel: (0..=n).map(|k| node(u, k)).collect(),
        cost: solver.solution.obj_val,
    })
}

/// Upper triangle of P for SMOOTHING * sum_k |u_{k+1} - u_k|^2 (as 1/2 x' P x).
fn smoothing_matrix(n: usize, vars: usize) -> CscMatrix<f64> {
    let (mut i, mut j, mut val) = (Vec::new(), Vec::new(), Vec::new());
    for k in 0..=n {
        let neighbors = if k == 0 || k == n { 1.0 } else { 2.0 };
        for a in 0..3 {
            i.push(u(k, a));
            j.push(u(k, a));
            val.push(2.0 * SMOOTHING * neighbors);
            if k < n {
                i.push(u(k, a));
                j.push(u(k + 1, a));
                val.push(-2.0 * SMOOTHING);
            }
        }
    }
    CscMatrix::new_from_triplets(vars, vars, i, j, val)
}

/// Free final time: scan a range of flight times, then golden-section refine the cheapest.
pub fn solve_free_time(p: &Problem, start: f64) -> Option<Plan> {
    let max_decel = (p.thrust_max / p.mass - p.gravity).max(1.0);
    let reference = (p.vel.length() / max_decel).max(1.0);
    let step = 0.2 * reference;
    let scan = (0..12).map(|i| reference * 0.8 + step * i as f64);
    let (best_time, mut best) = scan
        .filter_map(|tf| solve(p, tf, start).map(|plan| (tf, plan)))
        .min_by(|a, b| a.1.cost.total_cmp(&b.1.cost))?;

    let ratio = 0.618_033_988_75;
    let cost = |c: &Option<Plan>| c.as_ref().map_or(f64::INFINITY, |p| p.cost);
    let (mut lo, mut hi) = (best_time - step, best_time + step);
    for _ in 0..6 {
        let (t1, t2) = (hi - ratio * (hi - lo), lo + ratio * (hi - lo));
        let (c1, c2) = (solve(p, t1, start), solve(p, t2, start));
        if cost(&c1) < cost(&c2) {
            hi = t2
        } else {
            lo = t1
        }
        for plan in [c1, c2].into_iter().flatten() {
            if plan.cost < best.cost {
                best = plan;
            }
        }
    }
    Some(best)
}

#[cfg(test)]
mod tests {
    use super::*;

    fn landing_problem() -> Problem {
        Problem {
            pos: DVec3::new(300.0, -100.0, 2000.0),
            vel: DVec3::new(-10.0, 5.0, -200.0),
            mass: 28_000.0,
            target_pos: DVec3::new(0.0, 0.0, 15.5),
            target_vel: DVec3::new(0.0, 0.0, -1.5),
            thrust_min: 0.4 * 845e3,
            thrust_max: 845e3,
            terminal_thrust_max: 0.5 * 845e3,
            flow_per_newton: 1.0 / (282.0 * 9.80665),
            gravity: 9.80665,
            drag_rate: [0.0; INTERVALS + 1],
            wind: [DVec3::ZERO; INTERVALS + 1],
            max_tilt: [25f64.to_radians(); INTERVALS + 1],
            retrograde: DVec3::new(10.0, -5.0, 200.0).normalize(),
            aero_angle: [std::f64::consts::PI; INTERVALS + 1],
            glideslope: 10f64.to_radians(),
        }
    }

    #[test]
    fn finds_a_landing_that_meets_every_constraint() {
        let p = landing_problem();
        let plan = solve_free_time(&p, 0.0).expect("feasible");
        let last = INTERVALS;
        assert!((plan.pos[last] - p.target_pos).length() < 1e-3);
        assert!((plan.vel[last] - p.target_vel).length() < 1e-3);
        for (k, u) in plan.accel.iter().enumerate() {
            let t = k as f64 * plan.dt;
            let thrust_light = u.length() * (p.mass - p.flow_per_newton * p.thrust_max * t);
            let thrust_heavy = u.length() * (p.mass - p.flow_per_newton * p.thrust_min * t);
            assert!(
                thrust_light >= p.thrust_min * 0.99,
                "node {k} below min thrust"
            );
            assert!(
                thrust_heavy <= p.thrust_max * 1.01,
                "node {k} above max thrust"
            );
            assert!(
                u.z >= u.length() * p.max_tilt[k].cos() - 1e-6,
                "node {k} too tilted"
            );
        }
    }

    #[test]
    fn relaxation_is_lossless() {
        let p = landing_problem();
        let plan = solve_free_time(&p, 0.0).expect("feasible");
        // Every node must actually fly at least the minimum thrust: |u| >= T_min / m.
        for (k, u) in plan.accel.iter().enumerate() {
            let lightest = p.mass - p.flow_per_newton * p.thrust_max * k as f64 * plan.dt;
            assert!(
                u.length() >= p.thrust_min / lightest * 0.99,
                "node {k}: |u| {} below the bound",
                u.length()
            );
        }
    }

    #[test]
    fn too_late_is_infeasible() {
        let mut p = landing_problem();
        p.pos.z = 300.0;
        assert!(solve_free_time(&p, 0.0).is_none());
    }
}
