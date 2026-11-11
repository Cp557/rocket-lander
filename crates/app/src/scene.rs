//! The world around the booster: sun, physically based sky, stars, curved land and ocean,
//! and the landing zone with its floodlights. The time of day moves the sun; as it sets,
//! the camera exposure opens up, the floodlights come on and the stars come out.
//!
//! Sim coordinates are x east, y north, z up; Bevy is y up, so `to_bevy` maps (x, y, z)
//! to (x, z, -y).

use std::f32::consts::{FRAC_PI_2, FRAC_PI_4, PI, TAU};

use bevy::asset::RenderAssetUsages;
use bevy::camera::Exposure;
use bevy::light::atmosphere::ScatteringMedium;
use bevy::light::light_consts::lux;
use bevy::light::{Atmosphere, CascadeShadowConfigBuilder, SunDisk};
use bevy::mesh::{Indices, PrimitiveTopology};
use bevy::prelude::*;

use crate::camera::MainCamera;
use crate::sea::{LandSite, Ocean};
use crate::terrain;

pub fn to_bevy(v: Vec3) -> Vec3 {
    Vec3::new(v.x, v.z, -v.y)
}

pub fn from_bevy(v: Vec3) -> Vec3 {
    Vec3::new(v.x, -v.z, v.y)
}

pub fn rotation_to_bevy(q: Quat) -> Quat {
    let basis = Quat::from_rotation_x(-FRAC_PI_2);
    basis * q * basis.inverse()
}

/// Shown (unless an ancestor is hidden) or hidden.
pub fn shown(on: bool) -> Visibility {
    if on {
        Visibility::Inherited
    } else {
        Visibility::Hidden
    }
}

/// Local solar time (hours) that sets where the sun is.
#[derive(Resource)]
pub struct TimeOfDay {
    pub hour: f32,
}

impl Default for TimeOfDay {
    fn default() -> Self {
        Self { hour: 9.5 }
    }
}

const SUNRISE: f32 = 6.0;
const SUNSET: f32 = 18.0;
const NOON_ELEVATION: f32 = 60f32.to_radians();
/// Camera exposure (EV100) by day and at night.
const DAY_EXPOSURE: f32 = 13.0;
const NIGHT_EXPOSURE: f32 = 8.5;
/// Light from each floodlight at night (lm).
const FLOODLIGHT: f32 = 2.5e7;
const FLOODLIGHT_RANGE: f32 = 400.0;
const MAST_HEIGHT: f32 = 25.0;
const STARS: usize = 2500;
/// Stars sit on a sphere this far out, around the camera.
const STAR_RADIUS: f32 = 1.0e6;
/// Angular size of the brightest stars (rad).
const STAR_SIZE: f32 = 1.6e-3;
/// Narrower than this field of view the stars would be blown up into disks; hide them.
const STAR_MIN_FOV: f32 = 0.3;

/// The sun's angle above the horizon (rad): 60 degrees at noon, as far below at midnight.
fn sun_elevation(hour: f32) -> f32 {
    NOON_ELEVATION * ((hour - SUNRISE) / (SUNSET - SUNRISE) * PI).sin()
}

/// Sun light pointing for an hour of the day: rising in the east, peaking in the south at
/// noon, setting in the west.
fn sun_transform(hour: f32) -> Transform {
    let day = (hour - SUNRISE) / (SUNSET - SUNRISE);
    let azimuth = FRAC_PI_2 + day * PI; // from north, clockwise
    let elevation = sun_elevation(hour);
    let toward_sun = Vec3::new(
        azimuth.sin() * elevation.cos(),
        azimuth.cos() * elevation.cos(),
        elevation.sin(),
    );
    Transform::default().looking_to(-to_bevy(toward_sun), Vec3::Y)
}

/// 0 by day, 1 at night, fading through twilight as the sun sinks from 2 degrees above
/// the horizon to 6 below.
pub fn darkness(hour: f32) -> f32 {
    ((2f32.to_radians() - sun_elevation(hour)) / 8f32.to_radians()).clamp(0.0, 1.0)
}

#[derive(Component)]
pub struct Floodlight;

#[derive(Component)]
pub struct Stars;

#[derive(Resource)]
pub struct NightAssets {
    lamp: Handle<StandardMaterial>,
    stars: Handle<StandardMaterial>,
}

/// Light the world for the time of day: the sun (which goes out once it sets), the
/// exposure, the floodlights, and stars kept centered on the camera.
#[allow(clippy::type_complexity)] // Bevy system parameters are naturally nested.
pub fn update_sky(
    time: Res<TimeOfDay>,
    assets: Res<NightAssets>,
    mut materials: ResMut<Assets<StandardMaterial>>,
    sun: Single<(&mut Transform, &mut DirectionalLight)>,
    camera: Single<
        (&Transform, &Projection, &mut Exposure),
        (With<MainCamera>, Without<DirectionalLight>),
    >,
    mut floodlights: Query<&mut SpotLight, With<Floodlight>>,
    stars: Single<
        (&mut Transform, &mut Visibility),
        (With<Stars>, Without<MainCamera>, Without<DirectionalLight>),
    >,
) {
    let night = darkness(time.hour);
    let (mut sun_pose, mut sun) = sun.into_inner();
    *sun_pose = sun_transform(time.hour);
    // The sky takes its light from the sun, so keep a little through twilight for the glow.
    sun.illuminance = lux::RAW_SUNLIGHT * (1.0 - night);

    let (eye, projection, mut exposure) = camera.into_inner();
    exposure.ev100 = DAY_EXPOSURE + (NIGHT_EXPOSURE - DAY_EXPOSURE) * night;
    for mut light in &mut floodlights {
        light.intensity = FLOODLIGHT * night;
    }
    if let Some(mut lamp) = materials.get_mut(&assets.lamp) {
        lamp.emissive = LinearRgba::rgb(40.0, 36.0, 30.0) * night;
    }
    if let Some(mut star) = materials.get_mut(&assets.stars) {
        star.base_color = Color::linear_rgb(night, night, night);
    }
    let wide = matches!(projection, Projection::Perspective(lens) if lens.fov >= STAR_MIN_FOV);
    let (mut sky, mut visibility) = stars.into_inner();
    sky.translation = eye.translation;
    *visibility = shown(night > 0.0 && wide);
}

/// Small quads facing the center on a sphere, above the horizon, with a few bright stars
/// and many faint ones (brightness in the vertex color).
fn star_mesh() -> Mesh {
    let mut seed = 0x2545_f491u32;
    let mut random = || {
        seed ^= seed << 13;
        seed ^= seed >> 17;
        seed ^= seed << 5;
        seed as f32 / u32::MAX as f32
    };
    let (mut positions, mut colors, mut indices) = (Vec::new(), Vec::new(), Vec::new());
    for i in 0..STARS as u32 {
        let (z, angle) = (random(), random() * TAU);
        let ring = (1.0 - z * z).sqrt();
        let dir = Vec3::new(ring * angle.cos(), z, ring * angle.sin());
        let brightness = random().powi(6);
        let half = STAR_RADIUS * STAR_SIZE * (0.3 + 0.7 * brightness) * 0.5;
        let side = dir.any_orthonormal_vector();
        let up = dir.cross(side);
        let center = dir * STAR_RADIUS;
        for (a, b) in [(-1.0, -1.0), (1.0, -1.0), (1.0, 1.0), (-1.0, 1.0)] {
            positions.push((center + (side * a + up * b) * half).to_array());
            let level = 0.15 + 0.85 * brightness;
            colors.push([level, level, level * 1.1, 1.0]);
        }
        let k = 4 * i;
        indices.extend([k, k + 1, k + 2, k, k + 2, k + 3]);
    }
    Mesh::new(
        PrimitiveTopology::TriangleList,
        RenderAssetUsages::RENDER_WORLD,
    )
    .with_inserted_attribute(Mesh::ATTRIBUTE_POSITION, positions)
    .with_inserted_attribute(Mesh::ATTRIBUTE_COLOR, colors)
    .with_inserted_indices(Indices::U32(indices))
}

/// Top of the landing pad surface; the legs' contact plane (z = 0) sits just under it.
const PAD_TOP: f32 = 0.08;
const PAD_RADIUS: f32 = 45.0;

pub fn setup(
    mut commands: Commands,
    mut meshes: ResMut<Assets<Mesh>>,
    mut materials: ResMut<Assets<StandardMaterial>>,
    mut images: ResMut<Assets<Image>>,
    mut media: ResMut<Assets<ScatteringMedium>>,
) {
    commands.spawn(Atmosphere::earth(
        media.add(ScatteringMedium::earth(256, 256)),
    ));
    commands.insert_resource(GlobalAmbientLight::NONE);
    commands.spawn((
        DirectionalLight {
            illuminance: lux::RAW_SUNLIGHT,
            shadow_maps_enabled: true,
            ..default()
        },
        SunDisk::EARTH,
        sun_transform(TimeOfDay::default().hour),
        CascadeShadowConfigBuilder {
            maximum_distance: 4_000.0,
            first_cascade_far_bound: 120.0,
            ..default()
        }
        .build(),
    ));

    let detail = images.add(terrain::detail_texture());
    commands.spawn((
        LandSite,
        Mesh3d(meshes.add(terrain::land_mesh())),
        MeshMaterial3d(materials.add(StandardMaterial {
            base_color_texture: Some(detail),
            perceptual_roughness: 0.95,
            reflectance: 0.2,
            ..default()
        })),
    ));
    commands.spawn((
        Ocean,
        Transform::default(),
        Mesh3d(meshes.add(terrain::ocean_mesh())),
        MeshMaterial3d(materials.add(StandardMaterial {
            base_color: Color::srgb(0.02, 0.09, 0.15),
            perceptual_roughness: 0.12,
            reflectance: 0.6,
            ..default()
        })),
    ));

    let stars = materials.add(StandardMaterial {
        unlit: true,
        alpha_mode: AlphaMode::Add,
        cull_mode: None,
        ..default()
    });
    commands.spawn((
        Stars,
        Mesh3d(meshes.add(star_mesh())),
        MeshMaterial3d(stars.clone()),
        Transform::default(),
        Visibility::Hidden,
    ));
    let lamp = materials.add(StandardMaterial {
        base_color: Color::srgb(0.2, 0.2, 0.2),
        ..default()
    });
    commands.insert_resource(NightAssets {
        lamp: lamp.clone(),
        stars,
    });

    landing_zone(&mut commands, &mut meshes, &mut materials, &lamp);
}

fn landing_zone(
    commands: &mut Commands,
    meshes: &mut Assets<Mesh>,
    materials: &mut Assets<StandardMaterial>,
    lamp: &Handle<StandardMaterial>,
) {
    let mut material = |c: Color, roughness: f32| {
        materials.add(StandardMaterial {
            base_color: c,
            perceptual_roughness: roughness,
            ..default()
        })
    };
    let apron = material(Color::srgb(0.62, 0.62, 0.60), 0.9);
    let pad = material(Color::srgb(0.72, 0.72, 0.70), 0.85);
    let asphalt = material(Color::srgb(0.16, 0.16, 0.17), 0.95);
    let white = material(Color::srgb(0.95, 0.95, 0.95), 0.6);
    let yellow = material(Color::srgb(0.95, 0.78, 0.10), 0.6);
    let building = material(Color::srgb(0.85, 0.86, 0.88), 0.5);
    let dark = material(Color::srgb(0.25, 0.27, 0.30), 0.6);

    let mut slab = |mesh: Mesh, material: &Handle<StandardMaterial>, transform: Transform| {
        commands.spawn((
            LandSite,
            Mesh3d(meshes.add(mesh)),
            MeshMaterial3d(material.clone()),
            transform,
        ));
    };
    slab(
        Cuboid::new(160.0, 0.2, 160.0).into(),
        &apron,
        Transform::from_xyz(0.0, PAD_TOP - 0.14, 0.0),
    );
    slab(
        Cylinder::new(PAD_RADIUS, 0.3).into(),
        &pad,
        Transform::from_xyz(0.0, PAD_TOP - 0.15, 0.0),
    );
    // Access road running west to the hangars.
    slab(
        Cuboid::new(1_200.0, 0.2, 9.0).into(),
        &asphalt,
        Transform::from_xyz(-680.0, PAD_TOP - 0.15, 0.0),
    );

    // Pad markings lie flat just above the surface.
    let flat =
        |y: f32| Transform::from_xyz(0.0, y, 0.0).with_rotation(Quat::from_rotation_x(-FRAC_PI_2));
    slab(
        Annulus::new(40.0, 42.0).into(),
        &yellow,
        flat(PAD_TOP + 0.01),
    );
    slab(
        Annulus::new(14.0, 15.5).into(),
        &white,
        flat(PAD_TOP + 0.01),
    );
    for angle in [FRAC_PI_4, -FRAC_PI_4] {
        slab(
            Cuboid::new(26.0, 0.02, 2.5).into(),
            &white,
            Transform::from_xyz(0.0, PAD_TOP + 0.01, 0.0)
                .with_rotation(Quat::from_rotation_y(angle)),
        );
    }

    // A few buildings for scale.
    for (x, z, size) in [
        (-1_250.0, -60.0, Vec3::new(60.0, 22.0, 40.0)),
        (-1_250.0, 60.0, Vec3::new(40.0, 14.0, 30.0)),
        (-1_180.0, 0.0, Vec3::new(14.0, 30.0, 14.0)),
    ] {
        slab(
            Cuboid::from_size(size).into(),
            &building,
            Transform::from_xyz(x, size.y / 2.0, z),
        );
    }
    // Light masts, each with a floodlight aimed at the pad center.
    let mut heads = Vec::new();
    for (x, z) in [(-60.0, -60.0), (60.0, -60.0), (-60.0, 60.0), (60.0, 60.0)] {
        slab(
            Cylinder::new(0.4, MAST_HEIGHT).into(),
            &dark,
            Transform::from_xyz(x, MAST_HEIGHT / 2.0, z),
        );
        let head = Transform::from_xyz(x, MAST_HEIGHT + 0.6, z).looking_at(Vec3::ZERO, Vec3::Y);
        slab(Cuboid::new(2.4, 1.2, 0.5).into(), lamp, head);
        heads.push(head);
    }
    for head in heads {
        commands.spawn((
            LandSite,
            Floodlight,
            SpotLight {
                intensity: 0.0,
                color: Color::srgb(1.0, 0.94, 0.84),
                range: FLOODLIGHT_RANGE,
                outer_angle: 0.55,
                inner_angle: 0.3,
                shadow_maps_enabled: false,
                ..default()
            },
            head,
        ));
    }
}
