//! Attitude control and actuator allocation.
//!
//! Each body axis is a double integrator driven through lagging actuators: the gimbal
//! and grid fins are rate-limited, so the angular acceleration they deliver trails the
//! command. Gains come from a discrete LQR on that three-state model (attitude, rate,
//! delivered acceleration) with Bryson's-rule weights, looked up in a table over the
//! authority and lag currently available (gimbal, grid fins, cold-gas thrusters).

use std::sync::OnceLock;

use glam::{DQuat, DVec2, DVec3};
use nalgebra::{Matrix3, Matrix4, Vector3};
use sim::Vehicle;

/// Attitude error and rate considered "large" in Bryson's rule.
const ATTITUDE_ERROR_SCALE: f64 = 0.1;
const ATTITUDE_RATE_SCALE: f64 = 0.2;
/// Below this thrust (N) the gimbal is left centered.
const MIN_GIMBAL_THRUST: f64 = 50e3;
/// The flight computer's tick, which the attitude gains are designed for.
pub const CONTROL_PERIOD: f64 = 0.02;
/// Gain table: authority (rad/s^2) on a log scale, and actuator lag (s).
const TABLE_AUTHORITY: (f64, f64) = (5e-3, 20.0);
const TABLE_AUTHORITY_POINTS: usize = 37;
const TABLE_LAG: (f64, f64) = (0.01, 0.41);
const TABLE_LAG_POINTS: usize = 9;
const RICCATI_ITERATIONS: usize = 20_000;

#[derive(Clone, Copy, Debug, PartialEq)]
pub struct Gains {
    pub kp: f64,
    pub kd: f64,
}

/// Continuous LQR for x'' = u with Q = diag(1/e^2, 1/r^2) and R = 1/u^2.
/// Solving the Riccati equation by hand gives kp = sqrt(q1/R) and kd = sqrt(q2/R + 2 kp).
pub fn double_integrator_lqr(max_error: f64, max_rate: f64, max_input: f64) -> Gains {
    let kp = max_input / max_error;
    let kd = ((max_input / max_rate).powi(2) + 2.0 * kp).sqrt();
    Gains { kp, kd }
}

/// Feedback on attitude, rate and the acceleration the actuators are delivering.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct LagGains {
    pub kp: f64,
    pub kd: f64,
    pub ka: f64,
}

/// Zero-order-hold model over one control period of x = (angle, rate, delivered
/// acceleration) with x' = (rate, delivered, (u - delivered) / lag): returns (Phi, Gamma).
fn lagged_double_integrator(lag: f64) -> (Matrix3<f64>, Vector3<f64>) {
    let mut m = Matrix4::zeros();
    m[(0, 1)] = 1.0;
    m[(1, 2)] = 1.0;
    m[(2, 2)] = -1.0 / lag;
    m[(2, 3)] = 1.0 / lag;
    m *= CONTROL_PERIOD;
    // exp(M) by its Taylor series; |M| <= 4 here, so 40 terms are plenty.
    let (mut exp, mut term) = (Matrix4::identity(), Matrix4::identity());
    for k in 1..40 {
        term = term * m / k as f64;
        exp += term;
    }
    (
        exp.fixed_view::<3, 3>(0, 0).into_owned(),
        exp.fixed_view::<3, 1>(0, 3).into_owned(),
    )
}

/// Discrete LQR for the lagged double integrator with Q = diag(1/e^2, 1/r^2, 0) and
/// R = 1/u^2, found by iterating the Riccati equation to convergence. The control is
/// u = -kp angle - kd rate - ka delivered.
pub fn lagged_double_integrator_lqr(
    max_error: f64,
    max_rate: f64,
    max_input: f64,
    lag: f64,
) -> LagGains {
    let (phi, gamma) = lagged_double_integrator(lag);
    let q = Matrix3::from_diagonal(&Vector3::new(max_error.powi(-2), max_rate.powi(-2), 0.0));
    let r = max_input.powi(-2);
    let gain = |p: &Matrix3<f64>| {
        let pg = p * gamma;
        (pg.transpose() * phi) / (r + gamma.dot(&pg))
    };
    let mut p = q;
    for _ in 0..RICCATI_ITERATIONS {
        let k = gain(&p);
        let next = q + phi.transpose() * p * phi - phi.transpose() * p * gamma * k;
        let converged = (next - p).norm() <= 1e-12 * next.norm();
        p = next;
        if converged {
            break;
        }
    }
    let k = gain(&p);
    LagGains {
        kp: k[0],
        kd: k[1],
        ka: k[2],
    }
}

/// `lagged_double_integrator_lqr` with the attitude weights, tabulated once over the
/// authority and lag ranges the vehicle sees and interpolated (clamped at the edges).
pub fn attitude_gains(max_input: f64, lag: f64) -> LagGains {
    static TABLE: OnceLock<Vec<LagGains>> = OnceLock::new();
    let (a0, a1) = (TABLE_AUTHORITY.0.ln(), TABLE_AUTHORITY.1.ln());
    let (n, m) = (TABLE_AUTHORITY_POINTS - 1, TABLE_LAG_POINTS - 1);
    let table = TABLE.get_or_init(|| {
        (0..=n)
            .flat_map(|i| (0..=m).map(move |j| (i, j)))
            .map(|(i, j)| {
                let authority = (a0 + (a1 - a0) * i as f64 / n as f64).exp();
                let lag = TABLE_LAG.0 + (TABLE_LAG.1 - TABLE_LAG.0) * j as f64 / m as f64;
                lagged_double_integrator_lqr(
                    ATTITUDE_ERROR_SCALE,
                    ATTITUDE_RATE_SCALE,
                    authority,
                    lag,
                )
            })
            .collect()
    });
    // Fractional table coordinates.
    let x = ((max_input.ln() - a0) / (a1 - a0) * n as f64).clamp(0.0, n as f64);
    let y = ((lag - TABLE_LAG.0) / (TABLE_LAG.1 - TABLE_LAG.0) * m as f64).clamp(0.0, m as f64);
    let (i, j) = ((x as usize).min(n - 1), (y as usize).min(m - 1));
    let (fx, fy) = (x - i as f64, y - j as f64);
    let at = |i: usize, j: usize| table[i * (m + 1) + j];
    let blend = |f: fn(&LagGains) -> f64| {
        let low = f(&at(i, j)) * (1.0 - fy) + f(&at(i, j + 1)) * fy;
        let high = f(&at(i + 1, j)) * (1.0 - fy) + f(&at(i + 1, j + 1)) * fy;
        low * (1.0 - fx) + high * fx
    };
    LagGains {
        kp: blend(|g| g.kp),
        kd: blend(|g| g.kd),
        ka: blend(|g| g.ka),
    }
}

/// What the flight computer believes it can push on right now.
#[derive(Clone, Copy, Debug, Default)]
pub struct Effectors {
    /// Total engine thrust (N).
    pub thrust: f64,
    /// Fin force per radian of deflection: dynamic pressure * area * lift slope; 0 when stowed.
    pub fin_gain: f64,
    /// Height of the center of mass above the base, which sets the gimbal and fin levers.
    pub com_height: f64,
    /// Whether the cold-gas thrusters have gas left.
    pub cold_gas: bool,
}

/// Maximum angular acceleration each effector gives about each body axis.
struct Authorities {
    gimbal: DVec3,
    fins: DVec3,
    rcs: DVec3,
}

fn authorities(vehicle: &Vehicle, eff: &Effectors, inertia: DVec3) -> Authorities {
    let gimbal = if eff.thrust > MIN_GIMBAL_THRUST {
        eff.thrust * eff.com_height * vehicle.gimbal_max.sin()
    } else {
        0.0
    };
    let fin_arm = vehicle.fin_height - eff.com_height;
    let fins = eff.fin_gain * vehicle.fin_max * fin_arm;
    let fin_roll = eff.fin_gain * vehicle.fin_max * vehicle.fin_roll_arm;
    Authorities {
        gimbal: DVec3::new(gimbal, gimbal, 0.0) / inertia,
        fins: DVec3::new(fins, fins, fin_roll) / inertia,
        rcs: if eff.cold_gas {
            vehicle.rcs_torque / inertia
        } else {
            DVec3::ZERO
        },
    }
}

/// Maximum angular acceleration about each body axis.
pub fn authority(vehicle: &Vehicle, eff: &Effectors, inertia: DVec3) -> DVec3 {
    let a = authorities(vehicle, eff, inertia);
    a.gimbal + a.fins + a.rcs
}

/// How long the actuators take to deliver a new command about each axis: each effector's
/// time to slew to half deflection, weighted by its share of the authority (the cold-gas
/// thrusters respond at once), plus half a tick for the IMU's averaging.
pub fn actuator_lag(vehicle: &Vehicle, eff: &Effectors, inertia: DVec3) -> DVec3 {
    let a = authorities(vehicle, eff, inertia);
    let gimbal_lag = 0.5 * vehicle.gimbal_max / vehicle.gimbal_rate;
    let fin_lag = 0.5 * vehicle.fin_max / vehicle.fin_rate;
    let total = (a.gimbal + a.fins + a.rcs).max(DVec3::splat(1e-9));
    (a.gimbal * gimbal_lag + a.fins * fin_lag) / total + DVec3::splat(0.5 * CONTROL_PERIOD)
}

/// Body angular acceleration that turns the nose toward `axis_cmd` (world frame), no
/// faster than `max_rate` (rad/s), and damps all rates, given what the actuators are still
/// delivering from earlier commands. Roll is only damped.
pub fn attitude_accel(
    att: DQuat,
    omega: DVec3,
    axis_cmd: DVec3,
    authority: DVec3,
    lag: DVec3,
    delivered: DVec3,
    max_rate: f64,
) -> DVec3 {
    let turn = DQuat::from_rotation_arc(att * DVec3::Z, axis_cmd.normalize());
    let mut error = att.inverse() * turn * att;
    if error.w < 0.0 {
        error = -error;
    }
    let error = error.to_scaled_axis();
    let axis = |e: f64, w: f64, a: f64, max: f64, lag: f64| {
        let g = attitude_gains(max, lag);
        // Cascaded form of the law, with the rate request capped so the axis can always
        // brake in time when the error is large.
        let cap = (max * e.abs()).sqrt().min(max_rate);
        let rate_cmd = (g.kp / g.kd * e).clamp(-cap, cap);
        (g.kd * (rate_cmd - w) - g.ka * a).clamp(-max, max)
    };
    DVec3::new(
        axis(error.x, omega.x, delivered.x, authority.x, lag.x),
        axis(error.y, omega.y, delivered.y, authority.y, lag.y),
        axis(error.z, omega.z, delivered.z, authority.z, lag.z),
    )
}

#[derive(Clone, Copy, Debug, Default, PartialEq)]
pub struct Allocation {
    pub gimbal: DVec2,
    pub fins: DVec3,
    pub rcs: DVec3,
}

/// Split a body torque request across gimbal, then fins, then cold-gas thrusters.
pub fn allocate(vehicle: &Vehicle, eff: &Effectors, torque: DVec3) -> Allocation {
    let mut remaining = torque;

    let mut gimbal = DVec2::ZERO;
    if eff.thrust > MIN_GIMBAL_THRUST {
        let lever = eff.thrust * eff.com_height;
        let sines =
            (DVec2::new(-torque.y, torque.x) / lever).clamp_length_max(vehicle.gimbal_max.sin());
        gimbal = DVec2::new(sines.x.asin(), sines.y.asin());
        remaining -= DVec3::new(lever * sines.y, -lever * sines.x, 0.0);
    }

    let mut fins = DVec3::ZERO;
    if eff.fin_gain > 1e-3 {
        let arm = vehicle.fin_height - eff.com_height;
        let (k, max) = (eff.fin_gain, vehicle.fin_max);
        fins = DVec3::new(
            (remaining.y / (arm * k)).clamp(-max, max),
            (-remaining.x / (arm * k)).clamp(-max, max),
            (remaining.z / (k * vehicle.fin_roll_arm)).clamp(-max, max),
        );
        remaining -= DVec3::new(
            -arm * k * fins.y,
            arm * k * fins.x,
            k * vehicle.fin_roll_arm * fins.z,
        );
    }

    let rcs = if eff.cold_gas {
        remaining.clamp(-vehicle.rcs_torque, vehicle.rcs_torque)
    } else {
        DVec3::ZERO
    };
    Allocation { gimbal, fins, rcs }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn lqr_gains_solve_the_riccati_equation() {
        let (e, r, u) = (2.0, 1.0, 3.0);
        let g = double_integrator_lqr(e, r, u);
        let (q1, q2, rr) = (1.0 / (e * e), 1.0 / (r * r), 1.0 / (u * u));
        // P = [[p11, p12], [p12, p22]] with K = R^-1 B^T P = [p12, p22] / R.
        let (p12, p22) = (g.kp * rr, g.kd * rr);
        assert!((q1 - p12 * p12 / rr).abs() < 1e-12);
        assert!((2.0 * p12 + q2 - p22 * p22 / rr).abs() < 1e-12);
    }

    #[test]
    fn allocation_reproduces_requested_torque_within_limits() {
        let v = Vehicle::nominal();
        let eff = Effectors {
            thrust: 800e3,
            fin_gain: 40e3,
            com_height: 14.0,
            cold_gas: true,
        };
        let torque = DVec3::new(300e3, -500e3, 10e3);
        let a = allocate(&v, &eff, torque);
        let lever = eff.thrust * eff.com_height;
        let arm = v.fin_height - eff.com_height;
        let produced = DVec3::new(lever * a.gimbal.y.sin(), -lever * a.gimbal.x.sin(), 0.0)
            + DVec3::new(
                -arm * eff.fin_gain * a.fins.y,
                arm * eff.fin_gain * a.fins.x,
                eff.fin_gain * v.fin_roll_arm * a.fins.z,
            )
            + a.rcs;
        assert!((produced - torque).length() < 1.0, "{produced:?}");
    }

    #[test]
    fn lagged_lqr_matches_the_double_integrator_when_the_lag_vanishes() {
        let (e, r, u) = (0.1, 0.2, 0.3);
        let fast = lagged_double_integrator_lqr(e, r, u, 0.005);
        let ideal = double_integrator_lqr(e, r, u);
        assert!(
            (fast.kp - ideal.kp).abs() / ideal.kp < 0.1,
            "{fast:?} vs {ideal:?}"
        );
        assert!(
            (fast.kd - ideal.kd).abs() / ideal.kd < 0.1,
            "{fast:?} vs {ideal:?}"
        );
    }

    #[test]
    fn lagged_lqr_settles_a_lagging_axis() {
        // Closed loop on the design model: a 0.1 rad error dies out without ringing.
        let lag = 0.2;
        let g = lagged_double_integrator_lqr(0.1, 0.2, 0.3, lag);
        let (phi, gamma) = lagged_double_integrator(lag);
        let mut x = Vector3::new(0.1, 0.0, 0.0);
        let mut overshoot: f64 = 0.0;
        for _ in 0..1000 {
            let u = (-g.kp * x[0] - g.kd * x[1] - g.ka * x[2]).clamp(-0.3, 0.3);
            x = phi * x + gamma * u;
            overshoot = overshoot.max(-x[0]);
        }
        assert!(x.norm() < 1e-4, "{x:?}");
        assert!(overshoot < 0.02, "overshoot {overshoot}");
    }

    #[test]
    fn gain_table_interpolates_the_lqr() {
        let (u, lag) = (0.37, 0.17);
        let table = attitude_gains(u, lag);
        let exact = lagged_double_integrator_lqr(ATTITUDE_ERROR_SCALE, ATTITUDE_RATE_SCALE, u, lag);
        assert!(
            (table.kp - exact.kp).abs() / exact.kp < 0.03,
            "{table:?} vs {exact:?}"
        );
        assert!(
            (table.kd - exact.kd).abs() / exact.kd < 0.03,
            "{table:?} vs {exact:?}"
        );
    }

    #[test]
    fn attitude_command_turns_toward_target() {
        let att = DQuat::IDENTITY;
        let accel = attitude_accel(
            att,
            DVec3::ZERO,
            DVec3::X,
            DVec3::splat(0.1),
            DVec3::splat(0.1),
            DVec3::ZERO,
            f64::INFINITY,
        );
        // Rotating +z toward +x is a positive rotation about +y.
        assert!(accel.y > 0.0 && accel.x.abs() < 1e-9);
    }
}
