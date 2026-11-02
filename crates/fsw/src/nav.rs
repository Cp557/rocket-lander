//! Navigation: attitude from gyro integration blended with star-tracker fixes, and an
//! 11-state Kalman filter driven by the IMU: position, velocity, accelerometer bias, and
//! the steady wind at the 10 km reference altitude. GPS and the radar altimeter correct
//! it, and so does the air-relative crossflow the flight computer reads from the side
//! force, which is what makes the wind observable. The crossflow only corrects the wind:
//! its errors (aero and thrust model, gimbal) are slow and correlated, so letting it move
//! velocity would corrupt navigation whenever GPS is out. The wind follows the nominal
//! altitude profile, so the estimate weakens toward the ground as the vehicle descends.
//!
//! Given the attitude estimate, the models are linear, so the extended Kalman filter
//! reduces to a standard Kalman filter with a time-varying model.

use std::ops::Range;

use glam::{DMat3, DQuat, DVec2, DVec3};
use nalgebra::{Matrix3, SMatrix, SVector};
use sim::SensorFrame;
use sim::env::{NOMINAL_SHEAR_EXPONENT, Wind, gravity_vec, wind_profile};
use sim::sensors::{ALTIMETER_NOISE, GPS_POS_NOISE, GPS_VEL_NOISE};

const STATES: usize = 11;
const ALL_STATES: Range<usize> = 0..STATES;
const WIND_STATES: Range<usize> = 9..STATES;
type Vec11 = SVector<f64, STATES>;
type Mat11 = SMatrix<f64, STATES, STATES>;

/// Fraction of each star-tracker fix blended into the attitude (complementary filter).
const STAR_TRACKER_GAIN: f64 = 0.05;
/// Unmodeled acceleration (attitude error times thrust, drag), m/s^2 per sqrt(Hz).
const ACCEL_PROCESS_NOISE: f64 = 0.3;
const BIAS_PROCESS_NOISE: f64 = 1e-3;
const INITIAL_POS_SIGMA: f64 = 150.0;
const INITIAL_VEL_SIGMA: f64 = 5.0;
const INITIAL_BIAS_SIGMA: f64 = 0.05;
const INITIAL_WIND_SIGMA: f64 = 10.0;
/// How fast the local wind wanders from the profile (gusts, and the real profile's shape
/// and turning), m/s per sqrt(s).
const WIND_PROCESS_NOISE: f64 = 0.5;
/// Smallest profile factor used to map local wind changes to the reference wind.
const MIN_PROFILE: f64 = 0.1;

#[derive(Clone, Copy, Debug)]
pub struct NavState {
    pub pos: DVec3,
    pub vel: DVec3,
    pub att: DQuat,
    pub omega: DVec3,
    pub accel_bias: DVec3,
    /// One-sigma position uncertainty (root of the covariance trace).
    pub pos_sigma: f64,
    /// Steady wind through the reference altitude, with the nominal profile.
    pub wind: Wind,
}

impl NavState {
    /// Estimated wind at the current altitude.
    pub fn local_wind(&self) -> DVec3 {
        self.wind.at(self.pos.z)
    }
}

pub struct Navigator {
    x: Vec11,
    p: Mat11,
    att: DQuat,
    omega: DVec3,
}

impl Navigator {
    pub fn new(pos: DVec3, vel: DVec3, att: DQuat) -> Self {
        let mut x = Vec11::zeros();
        x.fixed_rows_mut::<3>(0).copy_from(&na(pos));
        x.fixed_rows_mut::<3>(3).copy_from(&na(vel));
        let sigmas = [INITIAL_POS_SIGMA; 3]
            .into_iter()
            .chain([INITIAL_VEL_SIGMA; 3])
            .chain([INITIAL_BIAS_SIGMA; 3])
            .chain([INITIAL_WIND_SIGMA; 2]);
        let p = Mat11::from_diagonal(&Vec11::from_iterator(sigmas.map(|s| s * s)));
        Self {
            x,
            p,
            att,
            omega: DVec3::ZERO,
        }
    }

    pub fn state(&self) -> NavState {
        NavState {
            pos: glam3(&self.x, 0),
            vel: glam3(&self.x, 3),
            att: self.att,
            omega: self.omega,
            accel_bias: glam3(&self.x, 6),
            pos_sigma: (self.p[(0, 0)] + self.p[(1, 1)] + self.p[(2, 2)]).sqrt(),
            wind: Wind::steady(DVec2::new(self.x[9], self.x[10])),
        }
    }

    pub fn update(&mut self, s: &SensorFrame) {
        self.att = (self.att * DQuat::from_scaled_axis(s.gyro * s.dt)).normalize();
        if let Some(fix) = s.star_tracker {
            self.att = self.att.slerp(fix, STAR_TRACKER_GAIN).normalize();
        }
        self.omega = s.gyro;

        self.predict(s.accel, s.dt);
        if let Some(gps) = s.gps {
            let mut z = SVector::<f64, 6>::zeros();
            z.fixed_rows_mut::<3>(0).copy_from(&na(gps.pos));
            z.fixed_rows_mut::<3>(3).copy_from(&na(gps.vel));
            let mut h = SMatrix::<f64, 6, STATES>::zeros();
            h.fixed_view_mut::<6, 6>(0, 0).fill_with_identity();
            let r = SVector::<f64, 6>::from_iterator(
                [GPS_POS_NOISE; 3].into_iter().chain([GPS_VEL_NOISE; 3]),
            );
            self.correct(z, h, r.map(|s| s * s), ALL_STATES);
        }
        if let Some(altitude) = s.altimeter {
            let mut h = SMatrix::<f64, 1, STATES>::zeros();
            h[(0, 2)] = 1.0;
            self.correct(
                SVector::from([altitude]),
                h,
                SVector::from([ALTIMETER_NOISE * ALTIMETER_NOISE]),
                ALL_STATES,
            );
        }
    }

    /// Correct with the air-relative crossflow `(x, y)` in the body frame, with noise
    /// `sigma` per axis: the body-frame sideways part of velocity minus wind.
    pub fn correct_crossflow(&mut self, crossflow: DVec2, sigma: f64) {
        let to_body = rotation(self.att).transpose();
        let profile = wind_profile(self.x[2], NOMINAL_SHEAR_EXPONENT);
        let mut h = SMatrix::<f64, 2, STATES>::zeros();
        for row in 0..2 {
            for col in 0..3 {
                h[(row, 3 + col)] = to_body[(row, col)];
            }
            for col in 0..2 {
                h[(row, 9 + col)] = -profile * to_body[(row, col)];
            }
        }
        self.correct(
            SVector::from([crossflow.x, crossflow.y]),
            h,
            SVector::from([sigma * sigma; 2]),
            WIND_STATES,
        );
    }

    fn predict(&mut self, specific_force: DVec3, dt: f64) {
        let pos = glam3(&self.x, 0);
        let vel = glam3(&self.x, 3);
        let bias = glam3(&self.x, 6);
        let accel = self.att * (specific_force - bias) + gravity_vec(pos.z);
        self.x
            .fixed_rows_mut::<3>(0)
            .copy_from(&na(pos + vel * dt + accel * (0.5 * dt * dt)));
        self.x
            .fixed_rows_mut::<3>(3)
            .copy_from(&na(vel + accel * dt));

        let r = rotation(self.att);
        let mut phi = Mat11::identity();
        phi.fixed_view_mut::<3, 3>(0, 3)
            .copy_from(&(Matrix3::identity() * dt));
        phi.fixed_view_mut::<3, 3>(0, 6)
            .copy_from(&(-r * (0.5 * dt * dt)));
        phi.fixed_view_mut::<3, 3>(3, 6).copy_from(&(-r * dt));

        let qa = ACCEL_PROCESS_NOISE * ACCEL_PROCESS_NOISE;
        let qb = BIAS_PROCESS_NOISE * BIAS_PROCESS_NOISE;
        // Local wind changes map to the reference wind through the profile.
        let profile = wind_profile(pos.z, NOMINAL_SHEAR_EXPONENT).max(MIN_PROFILE);
        let qw = (WIND_PROCESS_NOISE / profile).powi(2);
        let q = [qa * dt.powi(3) / 3.0; 3]
            .into_iter()
            .chain([qa * dt; 3])
            .chain([qb * dt; 3])
            .chain([qw * dt; 2]);
        self.p = phi * self.p * phi.transpose() + Mat11::from_diagonal(&Vec11::from_iterator(q));
    }

    /// Kalman measurement update with diagonal measurement noise that only moves the
    /// `updated` states (the others are considered, not corrected: a Schmidt update). The
    /// Joseph form keeps the covariance right for that suboptimal gain.
    fn correct<const M: usize>(
        &mut self,
        z: SVector<f64, M>,
        h: SMatrix<f64, M, STATES>,
        noise: SVector<f64, M>,
        updated: Range<usize>,
    ) {
        let r = SMatrix::<f64, M, M>::from_diagonal(&noise);
        let innovation = z - h * self.x;
        let Some(s_inv) = (h * self.p * h.transpose() + r).try_inverse() else {
            return;
        };
        let mut k = self.p * h.transpose() * s_inv;
        for row in (0..STATES).filter(|row| !updated.contains(row)) {
            k.row_mut(row).fill(0.0);
        }
        self.x += k * innovation;
        let i_kh = Mat11::identity() - k * h;
        self.p = i_kh * self.p * i_kh.transpose() + k * r * k.transpose();
    }
}

fn na(v: DVec3) -> nalgebra::Vector3<f64> {
    nalgebra::Vector3::new(v.x, v.y, v.z)
}

fn glam3(x: &Vec11, start: usize) -> DVec3 {
    DVec3::new(x[start], x[start + 1], x[start + 2])
}

fn rotation(att: DQuat) -> Matrix3<f64> {
    let m = DMat3::from_quat(att);
    Matrix3::from_fn(|row, col| m.col(col)[row])
}

#[cfg(test)]
mod tests {
    use super::*;
    use sim::GpsFix;

    #[test]
    fn converges_on_noisy_gps_in_free_fall() {
        let mut true_pos = DVec3::new(0.0, 0.0, 20_000.0);
        let mut true_vel = DVec3::new(10.0, 0.0, -100.0);
        let mut nav = Navigator::new(
            true_pos + DVec3::new(120.0, -80.0, 60.0),
            true_vel,
            DQuat::IDENTITY,
        );
        let mut seed = 1u64;
        // Deterministic uniform noise in [-1, 1].
        let mut noise = || {
            seed = seed
                .wrapping_mul(6364136223846793005)
                .wrapping_add(1442695040888963407);
            ((seed >> 11) as f64 / (1u64 << 53) as f64) * 2.0 - 1.0
        };
        let dt = 0.02;
        for i in 0..1500 {
            true_vel += gravity_vec(true_pos.z) * dt;
            true_pos += true_vel * dt;
            let gps = (i % 5 == 0).then(|| GpsFix {
                pos: true_pos + DVec3::new(noise(), noise(), noise()) * GPS_POS_NOISE,
                vel: true_vel + DVec3::new(noise(), noise(), noise()) * GPS_VEL_NOISE,
            });
            // Free fall: the accelerometer reads zero specific force.
            let frame = SensorFrame {
                dt,
                accel: DVec3::ZERO,
                gyro: DVec3::ZERO,
                gps,
                altimeter: None,
                star_tracker: None,
                leg_contact: false,
                engine_out: false,
                deck: None,
            };
            nav.update(&frame);
        }
        let est = nav.state();
        assert!(est.pos_sigma < 3.0, "sigma {}", est.pos_sigma);
        assert!(
            (est.pos - true_pos).length() < 3.0,
            "pos error {}",
            (est.pos - true_pos).length()
        );
        assert!(
            (est.vel - true_vel).length() < 0.5,
            "vel error {}",
            (est.vel - true_vel).length()
        );
    }

    #[test]
    fn crossflow_reveals_the_wind() {
        // Falling straight down at 2 km through a wind of (6, -3) m/s there, level body:
        // the air streams past sideways at minus the wind. GPS pins the velocity, so the
        // crossflow is put down to wind.
        let (pos, vel) = (DVec3::new(0.0, 0.0, 2000.0), DVec3::new(0.0, 0.0, -150.0));
        let local = DVec2::new(6.0, -3.0);
        let mut nav = Navigator::new(pos, vel, DQuat::IDENTITY);
        let mut gps_velocity = SMatrix::<f64, 3, STATES>::zeros();
        gps_velocity
            .fixed_view_mut::<3, 3>(0, 3)
            .fill_with_identity();
        for _ in 0..200 {
            nav.correct(
                na(vel),
                gps_velocity,
                SVector::from([GPS_VEL_NOISE.powi(2); 3]),
                ALL_STATES,
            );
            nav.correct_crossflow(-local, 1.0);
        }
        let est = nav.state();
        assert!(
            (est.local_wind().truncate() - local).length() < 0.2,
            "{:?}",
            est.local_wind()
        );
        // The profile carries the estimate up and down: stronger aloft, weaker below.
        assert!(est.wind.at(8000.0).length() > est.wind.at(100.0).length());
    }
}
