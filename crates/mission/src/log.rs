//! Flight log: one frame per flight-computer tick, plus events and landing plans.
//! The live telemetry view, replay and saved files all read this one structure.

use std::fs;
use std::io;
use std::path::Path;

use fsw::Phase;
use glam::{Quat, Vec2, Vec3};
use serde::{Deserialize, Serialize};
use sim::Ship;

use crate::flight::{FlightConfig, Outcome};

/// Snapshot of truth, estimate and actuators. Single precision keeps logs small.
#[derive(Clone, Debug, PartialEq, Serialize, Deserialize)]
pub struct Frame {
    pub t: f32,
    pub phase: Phase,
    pub pos: Vec3,
    pub vel: Vec3,
    pub att: Quat,
    pub mass: f32,
    pub est_pos: Vec3,
    pub est_mass: f32,
    pub pos_sigma: f32,
    /// Flight computer's horizontal wind estimate.
    pub est_wind: Vec3,
    pub engines: u8,
    /// Total thrust in units of one full-throttle engine.
    pub thrust_level: f32,
    pub throttle_cmd: f32,
    pub thrust: f32,
    pub gimbal: Vec2,
    pub fins: Vec3,
    pub rcs: Vec3,
    pub fin_deploy: f32,
    pub leg_deploy: f32,
    pub dynamic_pressure: f32,
    pub alpha: f32,
    pub wind: Vec3,
    /// Flight computer's predicted impact point.
    pub impact: Option<Vec2>,
    /// Cold gas left (kg).
    #[serde(default)]
    pub rcs_gas: f32,
}

#[derive(Clone, Debug, PartialEq, Serialize, Deserialize)]
pub struct Event {
    pub t: f64,
    pub text: String,
}

/// A landing plan as adopted by the flight computer at time `t`.
#[derive(Clone, Debug, PartialEq, Serialize, Deserialize)]
pub struct PlanRecord {
    pub t: f64,
    pub points: Vec<Vec3>,
}

#[derive(Clone, Debug, PartialEq, Serialize, Deserialize)]
pub struct FlightLog {
    pub config: FlightConfig,
    pub frames: Vec<Frame>,
    pub events: Vec<Event>,
    pub plans: Vec<PlanRecord>,
    pub outcome: Option<Outcome>,
    /// The drone ship, whose motion replays from its swell.
    #[serde(default)]
    pub ship: Option<Ship>,
}

impl FlightLog {
    pub fn duration(&self) -> f64 {
        self.frames.last().map_or(0.0, |f| f.t as f64)
    }

    /// The two frames around `t` and the blend fraction between them.
    pub fn frames_around(&self, t: f64) -> Option<(&Frame, &Frame, f32)> {
        let after = self.frames.partition_point(|f| (f.t as f64) <= t);
        let a = &self.frames[after
            .saturating_sub(1)
            .min(self.frames.len().checked_sub(1)?)];
        let b = self.frames.get(after).unwrap_or(a);
        let span = b.t - a.t;
        let f = if span > 0.0 {
            ((t as f32 - a.t) / span).clamp(0.0, 1.0)
        } else {
            0.0
        };
        Some((a, b, f))
    }

    /// The latest plan adopted at or before `t`, while it is still being flown.
    pub fn plan_at(&self, t: f64) -> Option<&PlanRecord> {
        let i = self.plans.partition_point(|p| p.t <= t);
        self.plans[..i].last()
    }

    pub fn save(&self, path: &Path) -> io::Result<()> {
        if let Some(dir) = path.parent() {
            fs::create_dir_all(dir)?;
        }
        fs::write(path, serde_json::to_vec(self).map_err(io::Error::other)?)
    }

    pub fn load(path: &Path) -> io::Result<Self> {
        serde_json::from_slice(&fs::read(path)?).map_err(io::Error::other)
    }
}
