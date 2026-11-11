//! Random variations of the true vehicle and environment for Monte Carlo runs, and the
//! harsher conditions and hardware faults added on top of them under stress.
//! The flight software always assumes the nominal values.

use glam::{DVec2, DVec3};
use rand::RngExt;
use rand::rngs::StdRng;
use rand_distr::StandardNormal;

use crate::env::{NOMINAL_SHEAR_EXPONENT, Wind};
use crate::vehicle::Vehicle;

/// GPS goes silent for `duration` seconds once the vehicle descends through `altitude`.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct GpsOutage {
    pub altitude: f64,
    pub duration: f64,
}

/// The accelerometer bias jumps by `jump` once the vehicle descends through `altitude`.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct BiasJump {
    pub altitude: f64,
    pub jump: DVec3,
}

#[derive(Clone, Copy, Debug, Default, PartialEq)]
pub struct Faults {
    /// Seconds of three-engine firing while descending before the outer engine at body -x
    /// fails for good.
    pub engine_out_after: Option<f64>,
    pub gps_outage: Option<GpsOutage>,
    pub accel_bias_jump: Option<BiasJump>,
}

#[derive(Clone, Copy, Debug)]
pub struct Dispersion {
    pub dry_mass_scale: f64,
    pub isp_scale: f64,
    pub thrust_scale: f64,
    pub drag_scale: f64,
    /// Fixed thrust-axis error added to the gimbal angles (rad).
    pub thrust_misalignment: DVec2,
    pub wind: Wind,
    pub throttle_lag_scale: f64,
    pub position_error: DVec3,
    pub velocity_error: DVec3,
    pub faults: Faults,
}

impl Dispersion {
    pub fn none() -> Self {
        Self {
            dry_mass_scale: 1.0,
            isp_scale: 1.0,
            thrust_scale: 1.0,
            drag_scale: 1.0,
            thrust_misalignment: DVec2::ZERO,
            wind: Wind::default(),
            throttle_lag_scale: 1.0,
            position_error: DVec3::ZERO,
            velocity_error: DVec3::ZERO,
            faults: Faults::default(),
        }
    }

    pub fn sample(rng: &mut StdRng) -> Self {
        let mut normal = |sigma: f64| rng.sample::<f64, _>(StandardNormal) * sigma;
        let misalignment = DVec2::new(normal(0.1f64.to_radians()), normal(0.1f64.to_radians()));
        let position_error = DVec3::new(normal(100.0), normal(100.0), normal(100.0));
        let velocity_error = DVec3::new(normal(3.0), normal(3.0), normal(3.0));
        let (dry_mass_scale, isp_scale, thrust_scale, drag_scale) = (
            1.0 + normal(0.02),
            1.0 + normal(0.005),
            1.0 + normal(0.01),
            1.0 + normal(0.05),
        );
        let wind_heading = rng.random_range(0.0..std::f64::consts::TAU);
        let wind_speed = rng.random_range(0.0..15.0);
        Self {
            dry_mass_scale,
            isp_scale,
            thrust_scale,
            drag_scale,
            thrust_misalignment: misalignment,
            wind: Wind {
                gust_sigma: rng.random_range(0.0..2.0),
                ..Wind::steady(DVec2::from_angle(wind_heading) * wind_speed)
            },
            position_error,
            velocity_error,
            ..Self::none()
        }
    }

    /// Harsher conditions scaled by `severity` (0 leaves the dispersion unchanged; 1 is a
    /// bad day): stronger, gustier wind whose profile and direction change with height, a
    /// slower throttle response, and, each with probability `severity / 2`, an engine
    /// failure in the entry burn, a GPS outage and an accelerometer bias jump.
    pub fn stressed(mut self, severity: f64, rng: &mut StdRng) -> Self {
        if severity <= 0.0 {
            return self;
        }
        let s = severity;
        let mut uniform = |lo: f64, hi: f64| rng.random_range(lo..hi);
        self.wind.reference *= 1.0 + 0.5 * s * uniform(0.0, 1.0);
        self.wind.gust_sigma += s * uniform(0.0, 2.0);
        self.wind.shear_exponent =
            (NOMINAL_SHEAR_EXPONENT + 0.15 * s * uniform(-1.0, 1.0)).max(0.05);
        self.wind.veer = s * uniform(-45.0, 45.0).to_radians();
        self.throttle_lag_scale = 1.0 + s * uniform(0.0, 1.0);
        let chance = (0.5 * s).min(1.0);
        let engine_out_after = uniform(0.0, 8.0);
        let outage = GpsOutage {
            altitude: uniform(300.0, 4000.0),
            duration: s * uniform(5.0, 15.0),
        };
        let jump_heading = uniform(0.0, std::f64::consts::TAU);
        let jump = BiasJump {
            altitude: uniform(300.0, 4000.0),
            jump: DVec2::from_angle(jump_heading).extend(uniform(-1.0, 1.0)) * 0.05 * s,
        };
        let mut happens = || rng.random_bool(chance);
        self.faults = Faults {
            engine_out_after: happens().then_some(engine_out_after),
            gps_outage: happens().then_some(outage),
            accel_bias_jump: happens().then_some(jump),
        };
        self
    }

    pub fn apply(&self, nominal: &Vehicle) -> Vehicle {
        Vehicle {
            dry_mass: nominal.dry_mass * self.dry_mass_scale,
            isp_vac: nominal.isp_vac * self.isp_scale,
            throttle_time_constant: nominal.throttle_time_constant * self.throttle_lag_scale,
            engine_thrust_vac: nominal.engine_thrust_vac * self.thrust_scale,
            engine_thrust_sl: nominal.engine_thrust_sl * self.thrust_scale,
            axial_coeff_base_first: nominal.axial_coeff_base_first * self.drag_scale,
            axial_coeff_nose_first: nominal.axial_coeff_nose_first * self.drag_scale,
            normal_coeff_slope: nominal.normal_coeff_slope * self.drag_scale,
            normal_coeff_crossflow: nominal.normal_coeff_crossflow * self.drag_scale,
            ..*nominal
        }
    }
}
