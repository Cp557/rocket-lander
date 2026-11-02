//! Guidance laws for each descent phase.

use glam::{DVec2, DVec3};
use sim::Vehicle;
use sim::aero::normal_coefficient;
use sim::env::{Wind, density, gravity, pressure};

use crate::gfold::{INTERVALS, Problem};
use crate::predict::{Impact, drag_accel};

/// Largest angle of attack used to steer with body lift.
const MAX_STEERING_ALPHA: f64 = 0.2618; // 15 degrees
/// Zero-effort-miss navigation gain.
const ZEM_GAIN: f64 = 3.0;
const LANDING_MAX_TILT: f64 = 0.4363; // 25 degrees
const MIN_AERO_ANGLE: f64 = 0.0175; // 1 degree
/// Typical landing-burn throttle, used to size the useful thrust angle.
const TYPICAL_LANDING_THROTTLE: f64 = 0.8;
/// Dynamic pressure at which the landing burn's braking axis is halfway between
/// retrograde and vertical.
const LIFT_PRESSURE: f64 = 1000.0;
/// Height above touchdown below which the allowed tilt shrinks toward vertical.
const TERMINAL_HEIGHT: f64 = 80.0;
const TERMINAL_MIN_TILT: f64 = 0.0524; // 3 degrees
/// Final plan nodes over which the planned tilt limit narrows, mirroring `tilt_limit`.
const TAPER_NODES: usize = 8;
const GLIDESLOPE: f64 = 1.2217; // 70 degrees
const TOUCHDOWN_SPEED: f64 = 1.5;
/// The plan ends this far below touchdown height, so the legs touch before it runs out.
const TARGET_SINK: f64 = 0.3;
/// Final-approach thrust ceiling relative to the planned minimum: arrive nearly coasting.
const TERMINAL_THRUST_RATIO: f64 = 1.2;
/// Plan inside the engine's real throttle range so feedback has room to correct.
const PLAN_MIN_THRUST_MARGIN: f64 = 1.1;
const PLAN_MAX_THRUST_MARGIN: f64 = 0.85;

/// Nose direction for base-first flight: opposite the velocity.
pub fn retrograde(vel: DVec3) -> DVec3 {
    if vel.length() > 1.0 {
        -vel.normalize()
    } else {
        DVec3::Z
    }
}

pub fn horizontal_toward(from: DVec2, to: DVec2) -> DVec3 {
    (to - from).normalize_or_zero().extend(0.0)
}

/// Aerodynamic steering: zero-effort-miss guidance on the predicted landing point, turned
/// into an angle of attack. Lift only helps until the landing burn lights, so the miss is
/// nulled by then. A base-first cylinder is pushed away from the side its nose leans
/// toward, so to accelerate along `e` the nose leans along `-e`. `air` is the velocity
/// relative to the air. Returns the nose direction and the angle of attack.
pub fn steering_axis(
    vehicle: &Vehicle,
    pos: DVec3,
    air: DVec3,
    mass: f64,
    impact: &Impact,
    target: DVec2,
) -> (DVec3, f64) {
    let back = retrograde(air);
    let t_go = impact.ignition.unwrap_or(impact.time).max(2.0);
    let accel = ((target - impact.point) * ZEM_GAIN / (t_go * t_go)).extend(0.0);
    let across = accel - back * accel.dot(back);
    let qs = 0.5 * density(pos.z) * air.length_squared() * vehicle.ref_area();
    if across.length() < 1e-6 || qs < 1.0 {
        return (back, 0.0);
    }
    let needed = mass * across.length() / qs;
    let alpha = alpha_for_normal_coefficient(vehicle, needed, MAX_STEERING_ALPHA);
    ((back - across.normalize() * alpha.sin()).normalize(), alpha)
}

/// Smallest angle of attack (up to `max`) whose normal coefficient reaches `cn`.
pub fn alpha_for_normal_coefficient(vehicle: &Vehicle, cn: f64, max: f64) -> f64 {
    let (mut lo, mut hi) = (0.0, max);
    if normal_coefficient(vehicle, hi) <= cn {
        return hi;
    }
    for _ in 0..30 {
        let mid = 0.5 * (lo + hi);
        if normal_coefficient(vehicle, mid) < cn {
            lo = mid
        } else {
            hi = mid
        }
    }
    hi
}

/// Net sideways force (toward the lean side) when thrust and body lean `lean` off the flow
/// line: thrust adds `T sin(lean)`, but the body then flies at that angle of attack and
/// its normal force pushes the other way.
fn lean_force(vehicle: &Vehicle, thrust: f64, qs: f64, lean: f64) -> f64 {
    thrust * lean.sin() - qs * normal_coefficient(vehicle, lean.abs()) * lean.signum()
}

/// End of the useful lean range, signed. While thrust wins (`T > c1 q S`), leaning toward
/// the target helps until the net force peaks near `(T - c1 q S) / (2 c2 q S)`. While lift
/// wins (high dynamic pressure), leaning *away* from the target helps, exactly as in the
/// unpowered steering.
fn max_lean(vehicle: &Vehicle, thrust: f64, qs: f64) -> f64 {
    let slope = thrust - vehicle.normal_coeff_slope * qs;
    if slope >= 0.0 {
        (slope / (2.0 * vehicle.normal_coeff_crossflow * qs.max(1.0))).min(LANDING_MAX_TILT)
    } else {
        -MAX_STEERING_ALPHA
    }
}

/// Signed lean off the flow line that produces `force` sideways, saturating at `max_lean`.
fn lean_for(vehicle: &Vehicle, thrust: f64, qs: f64, force: f64) -> f64 {
    let limit = max_lean(vehicle, thrust, qs);
    if lean_force(vehicle, thrust, qs, limit) <= force {
        return limit;
    }
    // The net force grows monotonically from 0 to `limit`.
    let (mut lo, mut hi) = (0.0, limit);
    for _ in 0..40 {
        let mid = 0.5 * (lo + hi);
        if lean_force(vehicle, thrust, qs, mid) < force {
            lo = mid
        } else {
            hi = mid
        }
    }
    hi
}

/// Turn a requested landing-burn thrust acceleration into the one to fly: brake along the
/// flow line, and deliver the sideways part by leaning, through thrust or body lift
/// depending on which dominates at the current dynamic pressure. `air` is the velocity
/// relative to the air.
pub fn shape_thrust(
    vehicle: &Vehicle,
    accel: DVec3,
    air: DVec3,
    altitude: f64,
    mass: f64,
) -> DVec3 {
    let q = 0.5 * density(altitude) * air.length_squared();
    // The flow line only matters while there is air to push on; otherwise brake vertically.
    let w = q / (q + LIFT_PRESSURE);
    let axis = (retrograde(air) * w + DVec3::Z * (1.0 - w)).normalize();
    let along = accel.dot(axis).max(0.1);
    let side = accel - axis * accel.dot(axis);
    let lean = lean_for(
        vehicle,
        mass * along,
        q * vehicle.ref_area(),
        mass * side.length(),
    );
    (axis * lean.cos() + side.normalize_or_zero() * lean.sin()) * (along / lean.cos())
}

/// The planner's view of the same physics: the thrust cone half-angle about retrograde
/// that matches the sideways acceleration the vehicle can actually produce.
fn aero_angle(vehicle: &Vehicle, altitude: f64, dynamic_pressure: f64) -> f64 {
    let thrust = TYPICAL_LANDING_THROTTLE * vehicle.engine_thrust(pressure(altitude));
    let qs = dynamic_pressure * vehicle.ref_area();
    let lean = max_lean(vehicle, thrust, qs);
    (lean_force(vehicle, thrust, qs, lean).abs() / thrust)
        .atan()
        .clamp(MIN_AERO_ANGLE, std::f64::consts::FRAC_PI_2)
}

/// Allowed tilt from vertical, narrowing over the last meters so the legs meet the ground upright.
pub fn tilt_limit(height_above_touchdown: f64) -> f64 {
    (LANDING_MAX_TILT * height_above_touchdown / TERMINAL_HEIGHT)
        .clamp(TERMINAL_MIN_TILT, LANDING_MAX_TILT)
}

pub fn landing_problem(
    vehicle: &Vehicle,
    pos: DVec3,
    vel: DVec3,
    wind: &Wind,
    mass: f64,
    pad: DVec3,
) -> Problem {
    let thrust = vehicle.engine_thrust(pressure(pos.z));
    let thrust_min = vehicle.min_throttle * thrust * PLAN_MIN_THRUST_MARGIN;
    let air = vel - wind.at(pos.z);
    let speed = air.length().max(1.0);
    let target_pos = pad + DVec3::Z * (vehicle.standing_height(mass) - TARGET_SINK);
    // A roughly constant deceleration takes height down with the square of the speed.
    let node_height = |k: usize| {
        target_pos.z + (pos.z - target_pos.z).max(0.0) * (1.0 - k as f64 / INTERVALS as f64).powi(2)
    };
    let rho = density(pos.z);
    // Assume the speed falls roughly linearly to zero over the burn.
    let node_speed = |k: usize| speed * (1.0 - k as f64 / INTERVALS as f64);
    let aero_angle = std::array::from_fn(|k| {
        self::aero_angle(vehicle, pos.z, 0.5 * rho * node_speed(k).powi(2))
    });
    // Drag as a linear rate at each node: the drag at that node's expected speed, scaled
    // down by the fraction of speed left, so the plan never leans on drag that fades
    // sooner than assumed.
    let drag_rate =
        std::array::from_fn(|k| drag_accel(vehicle, pos.z, node_speed(k), mass) / speed);
    Problem {
        pos,
        vel,
        mass,
        target_pos,
        target_vel: DVec3::new(0.0, 0.0, -TOUCHDOWN_SPEED),
        thrust_min,
        thrust_max: thrust * PLAN_MAX_THRUST_MARGIN,
        terminal_thrust_max: thrust_min * TERMINAL_THRUST_RATIO,
        flow_per_newton: vehicle.engine_mass_flow() / thrust,
        gravity: gravity(pos.z),
        drag_rate,
        wind: std::array::from_fn(|k| wind.at(node_height(k))),
        max_tilt: std::array::from_fn(|k| {
            let nodes_left = (INTERVALS - k) as f64 / TAPER_NODES as f64;
            (LANDING_MAX_TILT * nodes_left).clamp(TERMINAL_MIN_TILT, LANDING_MAX_TILT)
        }),
        retrograde: retrograde(air),
        aero_angle,
        glideslope: GLIDESLOPE,
    }
}

/// Thrust acceleration without a plan: brake to touchdown speed at touchdown height and
/// null horizontal drift over the pad.
pub fn fallback_accel(vehicle: &Vehicle, pos: DVec3, vel: DVec3, mass: f64, pad: DVec3) -> DVec3 {
    let height = (pos.z - pad.z - vehicle.standing_height(mass)).max(0.5);
    let brake = (vel.z * vel.z - TOUCHDOWN_SPEED * TOUCHDOWN_SPEED).max(0.0) / (2.0 * height);
    let drift = -(pos - pad) * 0.1 - vel * 0.8;
    DVec3::new(
        drift.x,
        drift.y,
        gravity(pos.z) + if vel.z < 0.0 { brake } else { 0.0 },
    )
}

/// Keep a thrust acceleration inside a cone of half-angle `angle` about `axis` by trimming
/// only its sideways part, so a large sideways demand never adds braking.
pub fn cone_limit(accel: DVec3, axis: DVec3, angle: f64) -> DVec3 {
    let along = accel.dot(axis).max(0.1);
    let side = accel - axis * accel.dot(axis);
    axis * along + side.clamp_length_max(along * angle.tan())
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn steering_leans_nose_away_from_target() {
        let v = Vehicle::nominal();
        let pos = DVec3::new(0.0, 0.0, 5000.0);
        let vel = DVec3::new(0.0, 0.0, -250.0);
        // Predicted landing point is 500 m west of the pad, so the vehicle must move east (+x).
        let impact = Impact {
            point: DVec2::new(-500.0, 0.0),
            time: 30.0,
            ignition: Some(20.0),
        };
        let (axis, alpha) = steering_axis(&v, pos, vel, 30_000.0, &impact, DVec2::ZERO);
        assert!(alpha > 0.0);
        assert!(axis.x < 0.0, "nose leans west to be pushed east: {axis:?}");
    }

    #[test]
    fn cone_limit_trims_only_horizontal() {
        let a = cone_limit(DVec3::new(30.0, 30.0, 10.0), DVec3::Z, 0.3);
        assert!((a.z - 10.0).abs() < 1e-12);
        assert!((a.truncate().length() - 10.0 * 0.3f64.tan()).abs() < 1e-12);
        assert!((a.x - a.y).abs() < 1e-12);
    }
}
