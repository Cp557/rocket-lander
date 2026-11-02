//! Planet environment: gravity, the 1976 US Standard Atmosphere, and wind.

use glam::{DVec2, DVec3};
use rand::RngExt;
use rand::rngs::StdRng;
use rand_distr::StandardNormal;
use serde::{Deserialize, Serialize};

pub const G0: f64 = 9.80665;
pub const EARTH_RADIUS: f64 = 6_371_000.0;
pub const SEA_LEVEL_PRESSURE: f64 = 101_325.0;
/// Specific gas constant of air (J/(kg K)) and its ratio of specific heats.
const GAS_CONSTANT: f64 = 287.053;
const HEAT_RATIO: f64 = 1.4;
/// Earth radius the standard atmosphere uses to turn altitude into geopotential height.
const GEOPOTENTIAL_RADIUS: f64 = 6_356_766.0;
/// Layers of the 1976 US Standard Atmosphere: base geopotential height (m), base
/// temperature (K), lapse rate (K/m) and base pressure (Pa). The last layer is extended
/// isothermally above 86 km.
const LAYERS: [(f64, f64, f64, f64); 8] = [
    (0.0, 288.15, -0.0065, 101_325.0),
    (11_000.0, 216.65, 0.0, 22_632.06),
    (20_000.0, 216.65, 0.001, 5_474.889),
    (32_000.0, 228.65, 0.0028, 868.0187),
    (47_000.0, 270.65, 0.0, 110.9063),
    (51_000.0, 270.65, -0.0028, 66.938_87),
    (71_000.0, 214.65, -0.002, 3.956_420),
    (84_852.0, 186.946, 0.0, 0.373_384),
];

/// Gravitational acceleration magnitude (inverse-square) at an altitude.
pub fn gravity(altitude: f64) -> f64 {
    let r = EARTH_RADIUS / (EARTH_RADIUS + altitude.max(0.0));
    G0 * r * r
}

pub fn gravity_vec(altitude: f64) -> DVec3 {
    DVec3::new(0.0, 0.0, -gravity(altitude))
}

/// Temperature (K) and pressure (Pa) of the standard atmosphere.
fn standard_atmosphere(altitude: f64) -> (f64, f64) {
    let h = altitude.max(0.0);
    let height = GEOPOTENTIAL_RADIUS * h / (GEOPOTENTIAL_RADIUS + h);
    let &(base, base_temperature, lapse, base_pressure) = LAYERS
        .iter()
        .rev()
        .find(|layer| layer.0 <= height)
        .expect("the first layer starts at the ground");
    let temperature = base_temperature + lapse * (height - base);
    let pressure = if lapse == 0.0 {
        base_pressure * (-G0 * (height - base) / (GAS_CONSTANT * base_temperature)).exp()
    } else {
        base_pressure * (base_temperature / temperature).powf(G0 / (GAS_CONSTANT * lapse))
    };
    (temperature, pressure)
}

pub fn temperature(altitude: f64) -> f64 {
    standard_atmosphere(altitude).0
}

pub fn pressure(altitude: f64) -> f64 {
    standard_atmosphere(altitude).1
}

pub fn density(altitude: f64) -> f64 {
    let (temperature, pressure) = standard_atmosphere(altitude);
    pressure / (GAS_CONSTANT * temperature)
}

pub fn speed_of_sound(altitude: f64) -> f64 {
    (HEAT_RATIO * GAS_CONSTANT * temperature(altitude)).sqrt()
}

/// Altitude the steady wind is specified at.
pub const WIND_REFERENCE_ALTITUDE: f64 = 10_000.0;
/// Power-law exponent of the steady wind below the reference altitude.
pub const NOMINAL_SHEAR_EXPONENT: f64 = 0.2;

/// Steady wind speed at an altitude as a fraction of the reference: a power law in the
/// boundary layer, fading above the reference altitude.
pub fn wind_profile(altitude: f64, exponent: f64) -> f64 {
    let h = altitude.max(1.0);
    if h < WIND_REFERENCE_ALTITUDE {
        (h / WIND_REFERENCE_ALTITUDE).powf(exponent)
    } else {
        (-(h - WIND_REFERENCE_ALTITUDE) / WIND_REFERENCE_ALTITUDE).exp()
    }
}

/// Horizontal wind: a steady altitude profile plus first-order Gauss-Markov gusts.
#[derive(Clone, Copy, Debug, Serialize, Deserialize)]
pub struct Wind {
    /// Steady wind velocity at the reference altitude.
    pub reference: DVec2,
    pub shear_exponent: f64,
    /// How far the steady wind direction turns from the reference altitude down to the
    /// ground (rad, counter-clockwise seen from above).
    pub veer: f64,
    pub gust_sigma: f64,
    pub gust: DVec2,
}

const GUST_TIME_CONSTANT: f64 = 2.0;

impl Default for Wind {
    fn default() -> Self {
        Self::steady(DVec2::ZERO)
    }
}

impl Wind {
    /// The nominal profile through `reference`, without veer or gusts.
    pub fn steady(reference: DVec2) -> Self {
        Self {
            reference,
            shear_exponent: NOMINAL_SHEAR_EXPONENT,
            veer: 0.0,
            gust_sigma: 0.0,
            gust: DVec2::ZERO,
        }
    }

    pub fn at(&self, altitude: f64) -> DVec3 {
        let depth = (1.0 - altitude.max(0.0) / WIND_REFERENCE_ALTITUDE).max(0.0);
        let steady = DVec2::from_angle(self.veer * depth).rotate(self.reference)
            * wind_profile(altitude, self.shear_exponent);
        (steady + self.gust).extend(0.0)
    }

    pub fn update_gust(&mut self, dt: f64, rng: &mut StdRng) {
        let decay = (-dt / GUST_TIME_CONSTANT).exp();
        let drive = self.gust_sigma * (1.0 - decay * decay).sqrt();
        let noise = DVec2::new(rng.sample(StandardNormal), rng.sample(StandardNormal));
        self.gust = self.gust * decay + noise * drive;
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn standard_atmosphere_matches_the_tables() {
        // (altitude m, density kg/m^3, speed of sound m/s) from the 1976 tables.
        for (h, rho, a) in [
            (0.0, 1.2250, 340.29),
            (5_000.0, 0.73643, 320.53),
            (11_000.0, 0.36392, 295.07),
            (25_000.0, 0.040084, 298.39),
            (50_000.0, 0.0010269, 329.80),
            (80_000.0, 1.8458e-5, 282.54),
        ] {
            assert!(
                (density(h) - rho).abs() / rho < 0.01,
                "{h} m: {}",
                density(h)
            );
            assert!(
                (speed_of_sound(h) - a).abs() < 0.5,
                "{h} m: {}",
                speed_of_sound(h)
            );
        }
    }

    #[test]
    fn veer_turns_the_wind_toward_the_ground_only() {
        let wind = Wind {
            veer: std::f64::consts::FRAC_PI_2,
            ..Wind::steady(DVec2::new(10.0, 0.0))
        };
        let aloft = wind.at(WIND_REFERENCE_ALTITUDE);
        let low = wind.at(1.0);
        assert!((aloft - DVec3::new(10.0, 0.0, 0.0)).length() < 1e-9);
        // Near the ground the steady wind has turned a quarter turn and weakened.
        assert!(low.x.abs() < 1e-3 && low.y > 0.0 && low.y < 10.0, "{low:?}");
    }
}
