//! Attitude estimation: a multiplicative extended Kalman filter (MEKF). The gyro carries the
//! attitude forward; each star-tracker fix corrects a small three-angle attitude error and
//! the gyro bias, which the filter learns from how the gyro drifts between fixes.
//!
//! The quaternion itself is the reference; the filter's state is the error about it
//! (body-frame small angles) and the gyro bias, so the covariance never has to respect the
//! quaternion's unit norm.

use glam::{DQuat, DVec3};
use nalgebra::{Matrix3, Matrix6, SMatrix, Vector3, Vector6};
use sim::sensors::{GYRO_NOISE, STAR_TRACKER_NOISE};

const INITIAL_ATTITUDE_SIGMA: f64 = 0.01;
const INITIAL_BIAS_SIGMA: f64 = 1e-3;
/// How fast the gyro bias may wander, rad/s per sqrt(s).
const BIAS_DRIFT: f64 = 1e-6;

pub struct AttitudeFilter {
    att: DQuat,
    bias: DVec3,
    /// Covariance of (attitude error, bias error).
    p: Matrix6<f64>,
    omega: DVec3,
}

fn na(v: DVec3) -> Vector3<f64> {
    Vector3::new(v.x, v.y, v.z)
}

fn skew(v: DVec3) -> Matrix3<f64> {
    Matrix3::new(0.0, -v.z, v.y, v.z, 0.0, -v.x, -v.y, v.x, 0.0)
}

impl AttitudeFilter {
    pub fn new(att: DQuat) -> Self {
        let sigmas = Vector6::new(
            INITIAL_ATTITUDE_SIGMA,
            INITIAL_ATTITUDE_SIGMA,
            INITIAL_ATTITUDE_SIGMA,
            INITIAL_BIAS_SIGMA,
            INITIAL_BIAS_SIGMA,
            INITIAL_BIAS_SIGMA,
        );
        Self {
            att,
            bias: DVec3::ZERO,
            p: Matrix6::from_diagonal(&sigmas.component_mul(&sigmas)),
            omega: DVec3::ZERO,
        }
    }

    pub fn att(&self) -> DQuat {
        self.att
    }

    /// Body rate with the estimated gyro bias removed.
    pub fn omega(&self) -> DVec3 {
        self.omega
    }

    pub fn bias(&self) -> DVec3 {
        self.bias
    }

    /// Carry the attitude forward by a gyro reading averaged over `dt`.
    pub fn propagate(&mut self, gyro: DVec3, dt: f64) {
        self.omega = gyro - self.bias;
        self.att = (self.att * DQuat::from_scaled_axis(self.omega * dt)).normalize();
        // The error rotates against the body rate and grows with the bias error.
        let mut phi = Matrix6::identity();
        phi.fixed_view_mut::<3, 3>(0, 0)
            .copy_from(&(Matrix3::identity() - skew(self.omega) * dt));
        phi.fixed_view_mut::<3, 3>(0, 3)
            .copy_from(&(-Matrix3::identity() * dt));
        let (angle, drift) = ((GYRO_NOISE * dt).powi(2), BIAS_DRIFT.powi(2) * dt);
        let q = Matrix6::from_diagonal(&Vector6::new(angle, angle, angle, drift, drift, drift));
        self.p = phi * self.p * phi.transpose() + q;
    }

    /// Correct with a star-tracker attitude fix.
    pub fn correct(&mut self, fix: DQuat) {
        let mut error = self.att.inverse() * fix;
        if error.w < 0.0 {
            error = -error;
        }
        let residual = na(DVec3::new(error.x, error.y, error.z) * 2.0);
        let mut h = SMatrix::<f64, 3, 6>::zeros();
        h.fixed_view_mut::<3, 3>(0, 0).fill_with_identity();
        let r = Matrix3::identity() * STAR_TRACKER_NOISE.powi(2);
        let Some(s_inv) = (h * self.p * h.transpose() + r).try_inverse() else {
            return;
        };
        let k = self.p * h.transpose() * s_inv;
        let dx = k * residual;
        self.att =
            (self.att * DQuat::from_scaled_axis(DVec3::new(dx[0], dx[1], dx[2]))).normalize();
        self.bias += DVec3::new(dx[3], dx[4], dx[5]);
        let i_kh = Matrix6::identity() - k * h;
        self.p = i_kh * self.p * i_kh.transpose() + k * r * k.transpose();
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn learns_the_gyro_bias_from_star_fixes() {
        let rate = DVec3::new(0.02, -0.01, 0.005);
        let bias = DVec3::new(4e-4, -3e-4, 2e-4);
        let mut truth = DQuat::IDENTITY;
        let mut filter =
            AttitudeFilter::new(DQuat::from_scaled_axis(DVec3::new(0.005, 0.0, -0.004)));
        let mut seed = 7u64;
        // Deterministic uniform noise in [-1, 1].
        let mut noise = || {
            seed = seed
                .wrapping_mul(6364136223846793005)
                .wrapping_add(1442695040888963407);
            ((seed >> 11) as f64 / (1u64 << 53) as f64) * 2.0 - 1.0
        };
        let dt = 0.02;
        for i in 0..6000 {
            truth = (truth * DQuat::from_scaled_axis(rate * dt)).normalize();
            let gyro = rate + bias + DVec3::new(noise(), noise(), noise()) * GYRO_NOISE;
            filter.propagate(gyro, dt);
            if i % 5 == 0 {
                let error = DVec3::new(noise(), noise(), noise()) * STAR_TRACKER_NOISE;
                filter.correct(truth * DQuat::from_scaled_axis(error));
            }
        }
        assert!(
            (filter.bias() - bias).length() < 5e-5,
            "{:?}",
            filter.bias()
        );
        let error = (filter.att().inverse() * truth).to_scaled_axis().length();
        assert!(error < 5e-4, "attitude error {error}");
    }
}
