//! Physical parameters of a Falcon-like first stage.

use std::f64::consts::PI;

use glam::DVec3;
use serde::{Deserialize, Serialize};

use crate::env::{G0, SEA_LEVEL_PRESSURE};

/// Body frame: origin at the center of mass, +z from engines toward the nose.
/// Heights below are measured from the base (engine plane) along +z. The center of mass
/// moves up as the propellant, which sits low in the tanks, burns away, so offsets from it
/// depend on the current mass.
#[derive(Clone, Copy, Debug, Serialize, Deserialize)]
pub struct Vehicle {
    pub dry_mass: f64,
    pub length: f64,
    pub radius: f64,
    /// Center of mass of the empty stage.
    pub dry_com_height: f64,
    /// Center of mass of the propellant still aboard.
    pub propellant_com_height: f64,
    /// Center of pressure for base-first flight; above the CoM, so descent is weathervane-stable.
    pub cp_height: f64,

    pub engine_thrust_vac: f64,
    pub engine_thrust_sl: f64,
    pub isp_vac: f64,
    pub min_throttle: f64,
    pub throttle_time_constant: f64,
    pub gimbal_max: f64,
    pub gimbal_rate: f64,
    /// Radius of the outer engine ring. The three-engine group is the center engine plus
    /// the outer pair on the body x axis.
    pub outer_engine_radius: f64,

    pub fin_height: f64,
    /// Effective fin area per control axis.
    pub fin_area: f64,
    /// Fin normal-force slope per radian of deflection.
    pub fin_lift_slope: f64,
    pub fin_roll_arm: f64,
    pub fin_max: f64,
    pub fin_rate: f64,
    pub fin_deploy_time: f64,

    /// Maximum cold-gas thruster torque about body x, y, z.
    pub rcs_torque: DVec3,

    pub axial_coeff_base_first: f64,
    pub axial_coeff_nose_first: f64,
    /// Slender-body normal-force slope (per radian, small angles).
    pub normal_coeff_slope: f64,
    /// Crossflow drag coefficient, scaled to the base reference area.
    pub normal_coeff_crossflow: f64,

    pub leg_radius: f64,
    /// How far the deployed feet sit below the base.
    pub leg_drop: f64,
    pub leg_deploy_time: f64,
}

impl Vehicle {
    pub fn nominal() -> Self {
        Self {
            dry_mass: 25_600.0,
            length: 40.0,
            radius: 1.85,
            dry_com_height: 15.3,
            propellant_com_height: 10.0,
            cp_height: 19.0,

            engine_thrust_vac: 914e3,
            engine_thrust_sl: 845e3,
            isp_vac: 311.0,
            min_throttle: 0.4,
            throttle_time_constant: 0.15,
            gimbal_max: 6f64.to_radians(),
            gimbal_rate: 20f64.to_radians(),
            outer_engine_radius: 1.2,

            fin_height: 38.0,
            fin_area: 4.0,
            fin_lift_slope: 2.0,
            fin_roll_arm: 2.5,
            fin_max: 20f64.to_radians(),
            fin_rate: 40f64.to_radians(),
            fin_deploy_time: 2.0,

            rcs_torque: DVec3::new(150e3, 150e3, 30e3),

            axial_coeff_base_first: 1.0,
            axial_coeff_nose_first: 0.5,
            normal_coeff_slope: 2.0,
            normal_coeff_crossflow: 10.0,

            leg_radius: 9.0,
            leg_drop: 1.5,
            leg_deploy_time: 2.0,
        }
    }

    pub fn ref_area(&self) -> f64 {
        PI * self.radius * self.radius
    }

    /// Principal moments of inertia (solid cylinder approximation) about the CoM.
    pub fn inertia(&self, mass: f64) -> DVec3 {
        let r2 = self.radius * self.radius;
        let lateral = mass * (3.0 * r2 + self.length * self.length) / 12.0;
        DVec3::new(lateral, lateral, 0.5 * mass * r2)
    }

    /// Thrust of one engine at full throttle; nozzle losses scale with ambient pressure.
    pub fn engine_thrust(&self, pressure: f64) -> f64 {
        self.engine_thrust_vac
            - (self.engine_thrust_vac - self.engine_thrust_sl) * pressure / SEA_LEVEL_PRESSURE
    }

    /// Propellant mass flow of one engine at full throttle.
    pub fn engine_mass_flow(&self) -> f64 {
        self.engine_thrust_vac / (self.isp_vac * G0)
    }

    /// Height of the center of mass above the base at a total mass.
    pub fn com_height(&self, mass: f64) -> f64 {
        let propellant = (mass - self.dry_mass).max(0.0);
        (self.dry_mass * self.dry_com_height + propellant * self.propellant_com_height)
            / (self.dry_mass + propellant)
    }

    /// Where the lit engines' combined thrust acts, body frame. Two engines lit means the
    /// three-engine group lost its outer engine at -x, so the thrust centers halfway out
    /// toward the survivor at +x.
    pub fn thrust_point(&self, engines: u8, mass: f64) -> DVec3 {
        let shift = if engines == 2 {
            0.5 * self.outer_engine_radius
        } else {
            0.0
        };
        DVec3::new(shift, 0.0, -self.com_height(mass))
    }

    pub fn fin_offset(&self, mass: f64) -> DVec3 {
        DVec3::new(0.0, 0.0, self.fin_height - self.com_height(mass))
    }

    pub fn cp_offset(&self, mass: f64) -> DVec3 {
        DVec3::new(0.0, 0.0, self.cp_height - self.com_height(mass))
    }

    /// Height of the CoM above flat ground when standing on deployed legs.
    pub fn standing_height(&self, mass: f64) -> f64 {
        self.com_height(mass) + self.leg_drop
    }

    pub fn leg_feet(&self, mass: f64) -> [DVec3; 4] {
        let z = -self.standing_height(mass);
        let r = self.leg_radius;
        [
            DVec3::new(r, 0.0, z),
            DVec3::new(-r, 0.0, z),
            DVec3::new(0.0, r, z),
            DVec3::new(0.0, -r, z),
        ]
    }

    /// Points on the base and nose rims; any of them touching the ground is a crash.
    pub fn hull_points(&self, mass: f64) -> [DVec3; 8] {
        let com = self.com_height(mass);
        let (r, base, nose) = (self.radius, -com, self.length - com);
        [
            DVec3::new(r, 0.0, base),
            DVec3::new(-r, 0.0, base),
            DVec3::new(0.0, r, base),
            DVec3::new(0.0, -r, base),
            DVec3::new(r, 0.0, nose),
            DVec3::new(-r, 0.0, nose),
            DVec3::new(0.0, r, nose),
            DVec3::new(0.0, -r, nose),
        ]
    }
}
