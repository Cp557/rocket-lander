//! Noisy onboard sensors. The flight software only ever sees a `SensorFrame`.

use glam::{DQuat, DVec3};
use rand::RngExt;
use rand::rngs::StdRng;
use rand_distr::StandardNormal;

use crate::dispersion::Faults;
use crate::dynamics::State;

pub const ACCEL_NOISE: f64 = 0.05;
pub const ACCEL_BIAS: f64 = 0.02;
pub const GYRO_NOISE: f64 = 5e-4;
pub const GYRO_BIAS: f64 = 2e-4;
pub const GPS_POS_NOISE: f64 = 2.0;
pub const GPS_VEL_NOISE: f64 = 0.1;
pub const GPS_PERIOD: f64 = 0.1;
pub const ALTIMETER_NOISE: f64 = 0.3;
pub const ALTIMETER_RANGE: f64 = 3000.0;
pub const ALTIMETER_PERIOD: f64 = 0.05;
pub const STAR_TRACKER_NOISE: f64 = 5e-4;
pub const STAR_TRACKER_PERIOD: f64 = 0.1;
/// Drone ship deck reports: rate, latency and noise (position m, velocity m/s, attitude
/// rad per axis).
pub const DECK_PERIOD: f64 = 0.1;
pub const DECK_DELAY: f64 = 0.2;
pub const DECK_POS_NOISE: f64 = 0.1;
pub const DECK_VEL_NOISE: f64 = 0.05;
pub const DECK_ATT_NOISE: f64 = 0.002;

#[derive(Clone, Copy, Debug)]
pub struct GpsFix {
    pub pos: DVec3,
    pub vel: DVec3,
}

/// The drone ship's report of where its deck center was `DECK_DELAY` ago, how it was
/// moving, and how it was tilted.
#[derive(Clone, Copy, Debug)]
pub struct DeckFix {
    pub pos: DVec3,
    pub vel: DVec3,
    pub att: DQuat,
}

#[derive(Clone, Copy, Debug)]
pub struct SensorFrame {
    /// Time covered by the IMU averages.
    pub dt: f64,
    /// Mean specific force over `dt`, body frame.
    pub accel: DVec3,
    /// Mean angular rate over `dt`, body frame.
    pub gyro: DVec3,
    pub gps: Option<GpsFix>,
    /// Radar altitude of the CoM above the ground, when in range.
    pub altimeter: Option<f64>,
    /// Absolute body-to-world attitude.
    pub star_tracker: Option<DQuat>,
    pub leg_contact: bool,
    /// Engine health monitoring reports a failed engine.
    pub engine_out: bool,
    /// Deck telemetry from the drone ship, when landing on one.
    pub deck: Option<DeckFix>,
    /// Cold gas left in the thruster tank (kg).
    pub rcs_gas: f64,
}

pub struct Sensors {
    accel_bias: DVec3,
    gyro_bias: DVec3,
    velocity_sum: DVec3,
    angle_sum: DVec3,
    elapsed: f64,
    next_gps: f64,
    next_altimeter: f64,
    next_star_tracker: f64,
    faults: Faults,
    /// When the GPS outage began, once it has.
    gps_outage_start: Option<f64>,
    /// Whether the GPS outage held at the latest read.
    pub gps_out: bool,
    pub bias_jumped: bool,
}

pub(crate) fn noise3(rng: &mut StdRng, sigma: f64) -> DVec3 {
    DVec3::new(
        rng.sample(StandardNormal),
        rng.sample(StandardNormal),
        rng.sample(StandardNormal),
    ) * sigma
}

fn noise(rng: &mut StdRng, sigma: f64) -> f64 {
    rng.sample::<f64, _>(StandardNormal) * sigma
}

impl Sensors {
    pub fn new(rng: &mut StdRng, faults: Faults) -> Self {
        Self {
            accel_bias: noise3(rng, ACCEL_BIAS),
            gyro_bias: noise3(rng, GYRO_BIAS),
            velocity_sum: DVec3::ZERO,
            angle_sum: DVec3::ZERO,
            elapsed: 0.0,
            next_gps: 0.0,
            next_altimeter: 0.0,
            next_star_tracker: 0.0,
            faults,
            gps_outage_start: None,
            gps_out: false,
            bias_jumped: false,
        }
    }

    /// Integrate one physics step of true specific force and rate (both body frame).
    pub fn accumulate(&mut self, specific_force: DVec3, omega: DVec3, dt: f64) {
        self.velocity_sum += specific_force * dt;
        self.angle_sum += omega * dt;
        self.elapsed += dt;
    }

    pub fn read(
        &mut self,
        t: f64,
        state: &State,
        leg_contact: bool,
        rng: &mut StdRng,
    ) -> SensorFrame {
        let descending_below = |altitude: f64| state.vel.z < 0.0 && state.pos.z < altitude;
        if let Some(jump) = self.faults.accel_bias_jump
            && descending_below(jump.altitude)
        {
            self.accel_bias += jump.jump;
            self.faults.accel_bias_jump = None;
            self.bias_jumped = true;
        }
        if let Some(outage) = self.faults.gps_outage
            && self.gps_outage_start.is_none()
            && descending_below(outage.altitude)
        {
            self.gps_outage_start = Some(t);
        }
        self.gps_out = self
            .faults
            .gps_outage
            .zip(self.gps_outage_start)
            .is_some_and(|(outage, start)| t - start < outage.duration);

        let dt = self.elapsed.max(1e-9);
        let accel = self.velocity_sum / dt + self.accel_bias + noise3(rng, ACCEL_NOISE);
        let gyro = self.angle_sum / dt + self.gyro_bias + noise3(rng, GYRO_NOISE);
        self.velocity_sum = DVec3::ZERO;
        self.angle_sum = DVec3::ZERO;
        self.elapsed = 0.0;

        let gps = (due(t, &mut self.next_gps, GPS_PERIOD) && !self.gps_out).then(|| GpsFix {
            pos: state.pos + noise3(rng, GPS_POS_NOISE),
            vel: state.vel + noise3(rng, GPS_VEL_NOISE),
        });
        let in_range = state.pos.z < ALTIMETER_RANGE && state.tilt() < 30f64.to_radians();
        let altimeter = (due(t, &mut self.next_altimeter, ALTIMETER_PERIOD) && in_range)
            .then(|| state.pos.z + noise(rng, ALTIMETER_NOISE));
        let star_tracker = due(t, &mut self.next_star_tracker, STAR_TRACKER_PERIOD)
            .then(|| state.att * DQuat::from_scaled_axis(noise3(rng, STAR_TRACKER_NOISE)));

        SensorFrame {
            dt,
            accel,
            gyro,
            gps,
            altimeter,
            star_tracker,
            leg_contact,
            engine_out: false,
            deck: None,
            rcs_gas: 0.0,
        }
    }
}

/// True when a periodic sensor is due at time `t`, scheduling its next sample.
pub(crate) fn due(t: f64, next: &mut f64, period: f64) -> bool {
    if t + 1e-9 < *next {
        return false;
    }
    *next = t + period;
    true
}
