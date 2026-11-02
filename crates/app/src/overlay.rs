//! In-scene telemetry drawn with gizmos: trail, velocity and thrust vectors, the landing
//! plan, the predicted impact point and the pad target ring.

use std::f32::consts::FRAC_PI_2;

use bevy::color::palettes::css::{AQUA, GOLD, MAGENTA, ORANGE_RED, WHITE};
use bevy::prelude::*;
use mission::flight::PAD_RADIUS;

use crate::playback::Playback;
use crate::scene::{rotation_to_bevy, to_bevy};

const MAX_TRAIL_POINTS: usize = 800;
/// Arrow length per m/s of velocity.
const VELOCITY_SCALE: f32 = 0.15;
/// Arrow length per m/s^2 of thrust acceleration.
const THRUST_SCALE: f32 = 2.0;

/// Whether the in-scene telemetry is drawn; off gives a clean cinematic view.
#[derive(Resource)]
pub struct Overlays {
    pub show: bool,
}

impl Default for Overlays {
    fn default() -> Self {
        Self { show: true }
    }
}

pub fn draw(playback: Res<Playback>, overlays: Res<Overlays>, mut gizmos: Gizmos) {
    if !overlays.show {
        return;
    }
    let Some(frame) = playback.frame() else {
        return;
    };
    let log = playback.log();
    let flat = |center: Vec3| Isometry3d::new(center, Quat::from_rotation_x(FRAC_PI_2));

    gizmos.circle(flat(Vec3::Y * 0.4), PAD_RADIUS as f32, GOLD);

    let shown = log
        .frames
        .partition_point(|f| (f.t as f64) <= playback.cursor);
    let step = (shown / MAX_TRAIL_POINTS).max(1);
    let trail = log.frames[..shown]
        .iter()
        .step_by(step)
        .map(|f| to_bevy(f.pos))
        .chain([to_bevy(frame.pos)]);
    gizmos.linestrip(trail, WHITE.with_alpha(0.6));

    let pos = to_bevy(frame.pos);
    gizmos.arrow(pos, pos + to_bevy(frame.vel) * VELOCITY_SCALE, AQUA);
    if frame.thrust > 0.0 {
        let thrust_dir = rotation_to_bevy(frame.att)
            * to_bevy(Vec3::new(frame.gimbal.x.tan(), frame.gimbal.y.tan(), 1.0).normalize());
        let accel = frame.thrust / frame.mass;
        gizmos.arrow(pos, pos + thrust_dir * accel * THRUST_SCALE, ORANGE_RED);
    }

    if let Some(impact) = frame.impact {
        let ground = to_bevy(impact.extend(0.0)) + Vec3::Y * 0.5;
        gizmos.circle(flat(ground), 25.0, ORANGE_RED);
    }
    if frame.phase == fsw::Phase::LandingBurn
        && let Some(plan) = log.plan_at(playback.cursor)
    {
        gizmos.linestrip(plan.points.iter().map(|p| to_bevy(*p)), MAGENTA);
    }
}
