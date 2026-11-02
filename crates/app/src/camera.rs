//! Two cameras: the 3D view (orbiting the booster, watching from the pad, riding on the
//! booster, or cutting between those) and an overlay camera that only renders the egui
//! panels across the whole window.
//!
//! The 3D camera renders in HDR with a physically based sky, a sun-scale exposure and
//! bloom, so the engine plumes glow.

use bevy::camera::visibility::RenderLayers;
use bevy::camera::{CameraOutputMode, Exposure};
use bevy::core_pipeline::tonemapping::Tonemapping;
use bevy::input::mouse::{AccumulatedMouseMotion, AccumulatedMouseScroll};
use bevy::light::AtmosphereEnvironmentMapLight;
use bevy::pbr::AtmosphereSettings;
use bevy::post_process::bloom::Bloom;
use bevy::prelude::*;
use bevy::render::render_resource::BlendState;
use bevy_egui::input::EguiWantsInput;
use bevy_egui::{EguiGlobalSettings, PrimaryEguiContext};
use fsw::Phase;
use mission::Frame;
use sim::Vehicle;

use crate::playback::Playback;
use crate::scene::{rotation_to_bevy, to_bevy};

#[derive(Component)]
pub struct MainCamera;

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum CameraMode {
    /// Orbit around the booster.
    Chase,
    /// Track the booster through a telephoto lens from a camera site near the pad.
    Pad,
    /// Onboard, on the engine section looking down past the engines.
    Engine,
    /// Onboard, high on the body looking down its length.
    Booster,
    /// Cut between the others to follow the flight.
    Auto,
}

impl CameraMode {
    pub const ALL: [CameraMode; 5] = [
        CameraMode::Auto,
        CameraMode::Chase,
        CameraMode::Pad,
        CameraMode::Engine,
        CameraMode::Booster,
    ];

    pub fn label(self) -> &'static str {
        match self {
            CameraMode::Chase => "Chase",
            CameraMode::Pad => "Pad",
            CameraMode::Engine => "Engine",
            CameraMode::Booster => "Booster",
            CameraMode::Auto => "Auto",
        }
    }
}

#[derive(Resource)]
pub struct Orbit {
    pub mode: CameraMode,
    /// The view actually shown: `mode`, or the director's pick in `Auto`.
    pub active: CameraMode,
    pub yaw: f32,
    pub pitch: f32,
    pub distance: f32,
}

impl Default for Orbit {
    fn default() -> Self {
        Self {
            mode: CameraMode::Chase,
            active: CameraMode::Chase,
            yaw: 0.6,
            pitch: -0.15,
            distance: 140.0,
        }
    }
}

/// Tracking camera site, 1.2 km south-west of the pad so the descent is seen from the side.
pub const PAD_CAMERA: Vec3 = Vec3::new(-850.0, 6.0, 850.0);
/// Height of the view the pad camera's telephoto lens frames around the booster (m).
const PAD_FRAMING: f32 = 80.0;
const DEFAULT_FOV: f32 = std::f32::consts::FRAC_PI_4;
const MIN_FOV: f32 = 0.01;
const ONBOARD_FOV: f32 = 1.4;
/// Onboard camera mounts, between two legs: height above the base, distance out from the
/// skin, and how far the view leans in toward the axis (positive) or out (negative).
const ENGINE_MOUNT: (f32, f32, f32) = (4.0, 1.6, 0.3);
const BOOSTER_MOUNT: (f32, f32, f32) = (31.0, 0.35, -0.12);
/// Peak camera shake on the onboard views (rad), at full thrust or high dynamic pressure.
const SHAKE: f32 = 0.004;
const SHAKE_PRESSURE: f32 = 20_000.0;
/// Keep the chase camera above the ground.
const MIN_CAMERA_HEIGHT: f32 = 2.0;

pub fn setup(mut commands: Commands, mut egui: ResMut<EguiGlobalSettings>) {
    egui.auto_create_primary_context = false;
    commands.spawn((
        MainCamera,
        Camera3d::default(),
        Projection::Perspective(PerspectiveProjection {
            near: 0.5,
            far: 2_000_000.0,
            ..default()
        }),
        Transform::from_xyz(0.0, 100.0, 200.0).looking_at(Vec3::ZERO, Vec3::Y),
        AtmosphereSettings {
            aerial_view_lut_max_distance: 200_000.0,
            ..default()
        },
        AtmosphereEnvironmentMapLight::default(),
        Exposure { ev100: 13.0 },
        Tonemapping::AcesFitted,
        Bloom::NATURAL,
    ));
    commands.spawn((
        PrimaryEguiContext,
        Camera2d,
        RenderLayers::none(),
        Camera {
            order: 1,
            output_mode: CameraOutputMode::Write {
                blend_state: Some(BlendState::ALPHA_BLENDING),
                clear_color: ClearColorConfig::None,
            },
            clear_color: ClearColorConfig::Custom(Color::NONE),
            ..default()
        },
    ));
}

/// Drag to orbit, scroll to zoom, unless the pointer is busy with the panels.
pub fn control(
    buttons: Res<ButtonInput<MouseButton>>,
    motion: Res<AccumulatedMouseMotion>,
    scroll: Res<AccumulatedMouseScroll>,
    egui: Res<EguiWantsInput>,
    mut orbit: ResMut<Orbit>,
) {
    if egui.wants_any_pointer_input() {
        return;
    }
    if buttons.pressed(MouseButton::Left) || buttons.pressed(MouseButton::Right) {
        orbit.yaw -= motion.delta.x * 0.005;
        orbit.pitch = (orbit.pitch - motion.delta.y * 0.005).clamp(-1.5, 1.5);
    }
    orbit.distance = (orbit.distance * (1.0 - scroll.delta.y * 0.1)).clamp(20.0, 20_000.0);
}

/// The director's cut: the wide chase view up high, the engine camera through the entry
/// burn, the body camera early in the fall, then the pad camera, with a look down past the
/// engines in the middle of the landing burn.
fn directed(frame: &Frame) -> CameraMode {
    let altitude = frame.pos.z;
    match frame.phase {
        Phase::Flip | Phase::Boostback | Phase::Coast => CameraMode::Chase,
        Phase::EntryBurn => CameraMode::Engine,
        Phase::AeroDescent if altitude > 3000.0 => CameraMode::Booster,
        Phase::LandingBurn if (150.0..1000.0).contains(&altitude) => CameraMode::Engine,
        _ => CameraMode::Pad,
    }
}

/// An onboard camera on `mount`, looking down the body with the outward direction up in
/// the picture.
fn onboard(frame: &Frame, (height, standoff, lean): (f32, f32, f32), time: f32) -> Transform {
    let v = Vehicle::nominal();
    let out = Vec3::new(1.0, 1.0, 0.0).normalize();
    let com = v.com_height(frame.mass as f64) as f32;
    let mount = out * (v.radius as f32 + standoff) + Vec3::Z * (height - com);
    let look = (-out * lean - Vec3::Z).normalize();
    let body =
        Transform::from_translation(to_bevy(frame.pos)).with_rotation(rotation_to_bevy(frame.att));
    let camera =
        Transform::from_translation(to_bevy(mount)).looking_to(to_bevy(look), to_bevy(out));
    // The airframe shakes with the engines and in thick, fast air.
    let engines = frame.engines.max(1) as f32;
    let strength =
        (frame.thrust_level / engines + frame.dynamic_pressure / SHAKE_PRESSURE).min(1.5);
    let wobble =
        |rate: f32, phase: f32| (time * rate + phase).sin() + 0.5 * (time * 2.7 * rate).sin();
    let shake = Quat::from_euler(
        EulerRot::XYZ,
        SHAKE * strength * wobble(31.0, 0.0),
        SHAKE * strength * wobble(37.0, 1.3),
        0.0,
    );
    body * camera.with_rotation(camera.rotation * shake)
}

pub fn follow(
    time: Res<Time>,
    mut orbit: ResMut<Orbit>,
    playback: Res<Playback>,
    camera: Single<(&mut Transform, &mut Projection), With<MainCamera>>,
) {
    let Some(frame) = playback.frame() else {
        return;
    };
    let mode = match orbit.mode {
        CameraMode::Auto => directed(&frame),
        mode => mode,
    };
    orbit.active = mode;
    let (mut transform, mut projection) = camera.into_inner();
    let target = to_bevy(frame.pos);
    // The pad camera zooms like a tracking telescope, keeping the booster the same size.
    if let Projection::Perspective(lens) = projection.as_mut() {
        (lens.fov, lens.near) = match mode {
            CameraMode::Pad => (
                (2.0 * (0.5 * PAD_FRAMING / PAD_CAMERA.distance(target)).atan())
                    .clamp(MIN_FOV, DEFAULT_FOV),
                0.5,
            ),
            CameraMode::Engine | CameraMode::Booster => (ONBOARD_FOV, 0.05),
            CameraMode::Chase | CameraMode::Auto => (DEFAULT_FOV, 0.5),
        };
    }
    let t = time.elapsed_secs();
    *transform = match mode {
        CameraMode::Pad => Transform::from_translation(PAD_CAMERA).looking_at(target, Vec3::Y),
        CameraMode::Engine => onboard(&frame, ENGINE_MOUNT, t),
        CameraMode::Booster => onboard(&frame, BOOSTER_MOUNT, t),
        CameraMode::Chase | CameraMode::Auto => {
            let offset = Quat::from_euler(EulerRot::YXZ, orbit.yaw, orbit.pitch, 0.0)
                * Vec3::Z
                * orbit.distance;
            let mut eye = target + offset;
            eye.y = eye.y.max(MIN_CAMERA_HEIGHT);
            Transform::from_translation(eye).looking_at(target, Vec3::Y)
        }
    };
}
