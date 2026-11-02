//! Body aerodynamics and grid fins.

use glam::DVec3;

use crate::env::{density, speed_of_sound};
use crate::vehicle::Vehicle;

/// Axial drag relative to subsonic, by Mach number: flat below Mach 0.6, peaking just past
/// Mach 1 as the shock forms, then easing off supersonic.
const MACH_DRAG: [(f64, f64); 7] = [
    (0.0, 1.0),
    (0.6, 1.0),
    (0.9, 1.15),
    (1.1, 1.5),
    (1.5, 1.4),
    (2.5, 1.25),
    (5.0, 1.15),
];

#[derive(Clone, Copy, Debug, Default)]
pub struct AeroLoads {
    /// Body-frame force.
    pub force: DVec3,
    /// Body-frame torque about the CoM.
    pub torque: DVec3,
    pub dynamic_pressure: f64,
    /// Total angle between the velocity and the body axis line, 0..=90 degrees.
    pub alpha: f64,
}

pub fn mach_drag_factor(mach: f64) -> f64 {
    let at = MACH_DRAG.partition_point(|&(m, _)| m <= mach);
    match (MACH_DRAG.get(at.wrapping_sub(1)), MACH_DRAG.get(at)) {
        (Some(&(m0, f0)), Some(&(m1, f1))) => f0 + (f1 - f0) * (mach - m0) / (m1 - m0),
        (Some(&(_, f)), None) | (None, Some(&(_, f))) => f,
        (None, None) => 1.0,
    }
}

pub fn axial_coefficient(vehicle: &Vehicle, base_first: bool, mach: f64) -> f64 {
    let subsonic = if base_first {
        vehicle.axial_coeff_base_first
    } else {
        vehicle.axial_coeff_nose_first
    };
    subsonic * mach_drag_factor(mach)
}

/// Normal-force coefficient: slender-body lift plus crossflow drag.
pub fn normal_coefficient(vehicle: &Vehicle, alpha: f64) -> f64 {
    let (s, c) = alpha.sin_cos();
    vehicle.normal_coeff_slope * s * c + vehicle.normal_coeff_crossflow * s * s
}

/// Loads for a body of `mass` moving at `v_air` (relative to the air, body frame) at an
/// altitude. `fins` are pitch/yaw/roll deflections (rad); `fin_deploy` is 0..=1.
pub fn loads(
    vehicle: &Vehicle,
    v_air: DVec3,
    altitude: f64,
    mass: f64,
    fins: DVec3,
    fin_deploy: f64,
) -> AeroLoads {
    let speed = v_air.length();
    if speed < 1e-6 {
        return AeroLoads::default();
    }
    let q = 0.5 * density(altitude) * speed * speed;
    let mach = speed / speed_of_sound(altitude);
    let qs = q * vehicle.ref_area();
    let crossflow = DVec3::new(v_air.x, v_air.y, 0.0);
    let crossflow_speed = crossflow.length();
    let alpha = crossflow_speed.atan2(v_air.z.abs());

    let axial =
        DVec3::Z * (-qs * axial_coefficient(vehicle, v_air.z < 0.0, mach) * v_air.z / speed);
    let normal = if crossflow_speed > 1e-9 {
        -crossflow / crossflow_speed * (qs * normal_coefficient(vehicle, alpha))
    } else {
        DVec3::ZERO
    };

    // Fins are ideal lateral-force generators at the top of the vehicle.
    let fin_gain = q * vehicle.fin_area * vehicle.fin_lift_slope * fin_deploy;
    let fin_force = DVec3::new(fins.x, fins.y, 0.0) * fin_gain;
    let fin_roll = DVec3::Z * (fins.z * fin_gain * vehicle.fin_roll_arm);

    AeroLoads {
        force: axial + normal + fin_force,
        torque: vehicle.cp_offset(mass).cross(normal)
            + vehicle.fin_offset(mass).cross(fin_force)
            + fin_roll,
        dynamic_pressure: q,
        alpha,
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn base_first_descent_is_weathervane_stable() {
        let v = Vehicle::nominal();
        // Falling base-first with the body drifting toward +x relative to the air.
        let loads = loads(
            &v,
            DVec3::new(5.0, 0.0, -100.0),
            2000.0,
            30e3,
            DVec3::ZERO,
            0.0,
        );
        assert!(loads.force.z > 0.0, "axial drag opposes the fall");
        assert!(loads.force.x < 0.0, "normal force opposes crossflow");
        // CP above the CoM: the torque (about -y) rotates the body back toward the flow line.
        assert!(loads.torque.y < 0.0);
    }

    #[test]
    fn drag_rises_through_the_sound_barrier() {
        assert_eq!(mach_drag_factor(0.3), 1.0);
        assert!((mach_drag_factor(1.1) - 1.5).abs() < 1e-12);
        assert!(mach_drag_factor(1.0) > 1.15 && mach_drag_factor(1.0) < 1.5);
        assert_eq!(mach_drag_factor(9.0), 1.15);
    }
}
