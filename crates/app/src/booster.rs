//! The booster: a model built by `blender/booster.py`, embedded in the binary, with its
//! named parts posed from the playback frame. The viewer adds what the model does not
//! carry: glowing engine plumes on the gimbal pivot and the light they cast, the legs'
//! pusher struts, the exhaust pushed back around the body when firing into fast air, and
//! the engine section's heat glow.
//!
//! The local frame is the sim body frame mapped like the world (`scene::to_bevy`), so the
//! long axis is local +Y with the origin at the center of mass.

use std::f32::consts::{FRAC_PI_2, PI, TAU};

use bevy::asset::RenderAssetUsages;
use bevy::mesh::{Indices, PrimitiveTopology};
use bevy::prelude::*;
use fsw::Phase;
use mission::{FlightLog, Frame};
use sim::Vehicle;
use sim::env::{SEA_LEVEL_PRESSURE, pressure};

use crate::particles::{linear_fade_texture, radial_fade_texture};
use crate::playback::Playback;
use crate::scene::{from_bevy, rotation_to_bevy, to_bevy};

/// Registered with `embedded_asset!` in `main`.
const MODEL: &str = "embedded://app/models/booster.glb";

#[derive(Component)]
pub struct Booster;

/// The model's `engines` pivot, tilted by the gimbal angles.
#[derive(Component)]
pub struct EngineCluster;

#[derive(Clone, Copy, PartialEq)]
pub enum PlumeLayer {
    Glow,
    Core,
    /// One of the shock diamonds along the core, counted from the nozzle.
    Diamond(usize),
}

/// One layer of one engine's plume. Engine 0 is the center engine, 1..=8 the outer ring.
#[derive(Component)]
pub struct Plume {
    engine: usize,
    layer: PlumeLayer,
}

/// A model `fin_N` hinge; `rest` is its authored rotation (deployed).
#[derive(Component)]
pub struct GridFin {
    index: usize,
    rest: Quat,
}

/// A model `leg_N` hinge; `rest` is its authored rotation (stowed).
#[derive(Component)]
pub struct Leg {
    rest: Quat,
    hinge: Vec3,
    strut: Entity,
}

/// Pusher strut between the body and a leg, re-aimed every frame.
#[derive(Component)]
pub struct Strut;

/// Exhaust pushed back around the lower body by the oncoming air.
#[derive(Component)]
pub struct Envelope;

/// Glowing shell over the engine section after the entry burn heats it.
#[derive(Component)]
pub struct HeatGlow;

/// The light the plumes cast on the booster, the ground and the smoke.
#[derive(Component)]
pub struct PlumeLight;

#[derive(Resource)]
pub struct BoosterAssets {
    plume_cone: Handle<Mesh>,
    glow: Handle<StandardMaterial>,
    core: Handle<StandardMaterial>,
    diamond_mesh: Handle<Mesh>,
    diamond: Handle<StandardMaterial>,
    strut_mesh: Handle<Mesh>,
    strut: Handle<StandardMaterial>,
    envelope: Handle<StandardMaterial>,
    heat: Handle<StandardMaterial>,
}

const ENGINES: usize = 9;
const ENGINE_RING: f32 = 1.2;
/// Distance below the base where the exhaust leaves the nozzles (matches the model).
pub const NOZZLE_EXIT_BELOW_BASE: f32 = 2.0;
/// Height of the model's origin above the base (`BASE` in `blender/booster.py`). The
/// center of mass moves with the propellant load, so the model is placed from it.
const MODEL_ORIGIN_HEIGHT: f32 = 15.0;
const LEG_HINGE_ABOVE_BASE: f32 = 2.0;
/// Where the pusher strut meets the body, above the leg hinge.
const STRUT_BODY_ABOVE_HINGE: f32 = 6.5;
/// Where the pusher strut meets the leg, as a fraction of the leg's length.
const STRUT_ON_LEG: f32 = 0.55;
const SHOCK_DIAMONDS: usize = 4;
const DIAMOND_SPACING: f32 = 1.7;
/// Dynamic over ambient pressure at which a retro-firing plume is fully pushed back around
/// the body: thin air lets the exhaust balloon out, and fast air folds it back.
const RETRO_FULL_RATIO: f32 = 5.0;
const ENVELOPE_GLOW: LinearRgba = LinearRgba::rgb(14.0, 5.0, 1.2);
/// Time to heat the engine section fully during the entry burn, and to cool it after (s).
const HEAT_TIME: f64 = 8.0;
const COOL_TIME: f64 = 40.0;
const HEAT_GLOW: LinearRgba = LinearRgba::rgb(6.0, 1.5, 0.2);
/// Plume light per full-throttle engine (lm): bright enough to light the pad at night,
/// a warm tint by day.
const PLUME_LIGHT: f32 = 4e7;
const PLUME_LIGHT_RANGE: f32 = 3000.0;
/// How far below the nozzles the plume light sits.
const PLUME_LIGHT_BELOW_NOZZLES: f32 = 5.0;

fn engine_position(engine: usize) -> Vec2 {
    if engine == 0 {
        return Vec2::ZERO;
    }
    Vec2::from_angle((engine - 1) as f32 * TAU / 8.0 + TAU / 16.0) * ENGINE_RING
}

/// Which engines burn for an engine count: the center first, then an opposite outer pair.
fn is_lit(engine: usize, engines: u8) -> bool {
    match engines {
        0 => false,
        1 => engine == 0,
        // The three-engine group is the center and an opposite pair; two lit means the
        // pair lost its engine on the body -x side.
        2 => matches!(engine, 0 | 1),
        3 => matches!(engine, 0 | 1 | 5),
        _ => true,
    }
}

struct LegGeometry {
    length: f32,
    /// Swing from stowed (along the body) to deployed.
    deployed_angle: f32,
}

/// The leg reaches from its hinge on the body wall to the foot position the physics uses.
fn leg_geometry(v: &Vehicle) -> LegGeometry {
    let out = (v.leg_radius - v.radius) as f32;
    let down = LEG_HINGE_ABOVE_BASE + v.leg_drop as f32;
    LegGeometry {
        length: out.hypot(down),
        deployed_angle: FRAC_PI_2 + down.atan2(out),
    }
}

pub fn setup(
    mut commands: Commands,
    asset_server: Res<AssetServer>,
    mut meshes: ResMut<Assets<Mesh>>,
    mut materials: ResMut<Assets<StandardMaterial>>,
    mut images: ResMut<Assets<Image>>,
) {
    // Plume light is all emissive (unlit materials would ignore it), on a black base so
    // the sun adds nothing. Cone UVs are polar with the tip at the center, so a radial
    // fade texture makes the glow fade out along the plume's length.
    let flame =
        |opacity: f32, emissive: LinearRgba, texture: Option<Handle<Image>>| StandardMaterial {
            base_color: Color::srgba(0.0, 0.0, 0.0, opacity),
            base_color_texture: texture,
            emissive,
            perceptual_roughness: 1.0,
            reflectance: 0.0,
            alpha_mode: AlphaMode::Blend,
            cull_mode: None,
            ..default()
        };
    let fade = images.add(radial_fade_texture(false));
    let linear_fade = images.add(linear_fade_texture());
    // Pushed-back exhaust and heat glow add light without hiding what is behind them.
    let glow = |emissive: LinearRgba| StandardMaterial {
        base_color: Color::BLACK,
        base_color_texture: Some(linear_fade.clone()),
        emissive,
        perceptual_roughness: 1.0,
        reflectance: 0.0,
        alpha_mode: AlphaMode::Add,
        cull_mode: None,
        ..default()
    };
    let envelope = materials.add(glow(ENVELOPE_GLOW));
    let heat = materials.add(glow(LinearRgba::BLACK));
    commands.insert_resource(BoosterAssets {
        plume_cone: meshes.add(open_cone(32)),
        glow: materials.add(flame(0.6, LinearRgba::rgb(14.0, 5.0, 1.0), Some(fade))),
        core: materials.add(flame(0.85, LinearRgba::rgb(40.0, 24.0, 10.0), None)),
        diamond_mesh: meshes.add(Sphere::new(1.0)),
        diamond: materials.add(flame(0.7, LinearRgba::rgb(60.0, 45.0, 30.0), None)),
        strut_mesh: meshes.add(Cylinder::new(0.11, 1.0)),
        strut: materials.add(StandardMaterial {
            base_color: Color::srgb(0.55, 0.55, 0.57),
            metallic: 0.9,
            perceptual_roughness: 0.35,
            ..default()
        }),
        envelope: envelope.clone(),
        heat: heat.clone(),
    });
    let base = -MODEL_ORIGIN_HEIGHT;
    // Unit bulb from below the nozzles up around the body; scaled every frame.
    let bulb = [
        (0.0, -0.3),
        (0.55, -0.25),
        (0.85, -0.1),
        (1.0, 0.1),
        (0.95, 0.35),
        (0.8, 0.65),
        (0.62, 1.0),
    ];
    // Hugs the nozzles and the lowest few meters of the body.
    let engine_section = [(1.75, -NOZZLE_EXIT_BELOW_BASE), (1.9, 0.0), (1.9, 3.0)];
    commands
        .spawn((Booster, Transform::default(), Visibility::default()))
        .with_child(WorldAssetRoot(
            asset_server.load(GltfAssetLabel::Scene(0).from_asset(MODEL)),
        ))
        .with_child((
            Envelope,
            Mesh3d(meshes.add(lathe(&bulb, 48))),
            MeshMaterial3d(envelope),
            Transform::from_xyz(0.0, base, 0.0),
            Visibility::Hidden,
        ))
        .with_child((
            HeatGlow,
            Mesh3d(meshes.add(lathe(&engine_section, 48))),
            MeshMaterial3d(heat),
            Transform::from_xyz(0.0, base, 0.0),
            Visibility::Hidden,
        ))
        .with_child((
            PlumeLight,
            PointLight {
                intensity: 0.0,
                color: Color::srgb(1.0, 0.62, 0.32),
                range: PLUME_LIGHT_RANGE,
                radius: 2.0,
                shadow_maps_enabled: false,
                ..default()
            },
            Transform::from_xyz(
                0.0,
                base - NOZZLE_EXIT_BELOW_BASE - PLUME_LIGHT_BELOW_NOZZLES,
                0.0,
            ),
        ));
}

fn part_index(name: &str, prefix: &str) -> Option<usize> {
    name.strip_prefix(prefix)?.parse().ok()
}

/// Tag the model's named pivots as the glTF scene spawns, and attach the plumes and
/// struts the model does not carry.
pub fn bind_parts(
    mut commands: Commands,
    parts: Query<(Entity, &Name, &Transform), Added<Name>>,
    booster: Single<Entity, With<Booster>>,
    assets: Res<BoosterAssets>,
) {
    for (entity, name, transform) in &parts {
        let name = name.as_str();
        if name == "engines" {
            commands
                .entity(entity)
                .insert(EngineCluster)
                .with_children(|e| {
                    let layers = [PlumeLayer::Glow, PlumeLayer::Core]
                        .into_iter()
                        .chain((0..SHOCK_DIAMONDS).map(PlumeLayer::Diamond));
                    for engine in 0..ENGINES {
                        for layer in layers.clone() {
                            let (mesh, material) = match layer {
                                PlumeLayer::Glow => (&assets.plume_cone, &assets.glow),
                                PlumeLayer::Core => (&assets.plume_cone, &assets.core),
                                PlumeLayer::Diamond(_) => (&assets.diamond_mesh, &assets.diamond),
                            };
                            e.spawn((
                                Plume { engine, layer },
                                Mesh3d(mesh.clone()),
                                MeshMaterial3d(material.clone()),
                                Transform::default(),
                                Visibility::Hidden,
                            ));
                        }
                    }
                });
        } else if let Some(index) = part_index(name, "fin_") {
            commands.entity(entity).insert(GridFin {
                index,
                rest: transform.rotation,
            });
        } else if part_index(name, "leg_").is_some() {
            let strut = commands
                .spawn((
                    Strut,
                    Mesh3d(assets.strut_mesh.clone()),
                    MeshMaterial3d(assets.strut.clone()),
                    Transform::default(),
                ))
                .id();
            commands.entity(*booster).add_child(strut);
            commands.entity(entity).insert(Leg {
                rest: transform.rotation,
                hinge: transform.translation,
                strut,
            });
        }
    }
}

/// Unit cone (radius 1, height 1, tip at +Y) without a base cap, for plumes. UVs are
/// polar like Bevy's cone: the tip maps to the texture center, the rim to its edge.
fn open_cone(segments: u32) -> Mesh {
    let mut positions = vec![[0.0, 0.5, 0.0]];
    let mut uvs = vec![[0.5, 0.5]];
    for i in 0..=segments {
        let (sin, cos) = (i as f32 / segments as f32 * TAU).sin_cos();
        positions.push([cos, -0.5, sin]);
        uvs.push([0.5 + 0.5 * cos, 0.5 + 0.5 * sin]);
    }
    let normals = vec![[0.0, 1.0, 0.0]; positions.len()];
    let indices = (1..=segments).flat_map(|i| [0, i + 1, i]).collect();
    Mesh::new(
        PrimitiveTopology::TriangleList,
        RenderAssetUsages::RENDER_WORLD,
    )
    .with_inserted_attribute(Mesh::ATTRIBUTE_POSITION, positions)
    .with_inserted_attribute(Mesh::ATTRIBUTE_NORMAL, normals)
    .with_inserted_attribute(Mesh::ATTRIBUTE_UV_0, uvs)
    .with_inserted_indices(Indices::U32(indices))
}

/// Surface of revolution about +Y through `(radius, height)` points, open at both ends.
/// UV v runs along the profile from 0 at the first point to 1 at the last.
fn lathe(profile: &[(f32, f32)], segments: u32) -> Mesh {
    let rows = profile.len() as u32;
    let (mut positions, mut normals, mut uvs) = (Vec::new(), Vec::new(), Vec::new());
    for (k, &(radius, height)) in profile.iter().enumerate() {
        for i in 0..=segments {
            let u = i as f32 / segments as f32;
            let (sin, cos) = (u * TAU).sin_cos();
            positions.push([radius * cos, height, radius * sin]);
            normals.push([cos, 0.0, sin]);
            uvs.push([u, k as f32 / (rows - 1) as f32]);
        }
    }
    let ring = segments + 1;
    let indices = (0..rows - 1)
        .flat_map(|k| {
            (0..segments).flat_map(move |i| {
                let (a, b) = (k * ring + i, (k + 1) * ring + i);
                [a, b, a + 1, a + 1, b, b + 1]
            })
        })
        .collect();
    Mesh::new(
        PrimitiveTopology::TriangleList,
        RenderAssetUsages::RENDER_WORLD,
    )
    .with_inserted_attribute(Mesh::ATTRIBUTE_POSITION, positions)
    .with_inserted_attribute(Mesh::ATTRIBUTE_NORMAL, normals)
    .with_inserted_attribute(Mesh::ATTRIBUTE_UV_0, uvs)
    .with_inserted_indices(Indices::U32(indices))
}

/// How far the exhaust is pushed back around the body (0..=1): engines firing into fast,
/// thin oncoming air, as in the entry burn.
fn retro_strength(frame: &Frame) -> f32 {
    if frame.thrust_level < 0.02 {
        return 0.0;
    }
    let nose = frame.att * Vec3::Z;
    let into_air = (-nose.dot(frame.vel.normalize_or_zero())).clamp(0.0, 1.0);
    let ratio = frame.dynamic_pressure / pressure(frame.pos.z as f64) as f32;
    ((ratio - 1.0) / (RETRO_FULL_RATIO - 1.0)).clamp(0.0, 1.0) * into_air
}

/// How hot the engine section is (0..=1): it heats while the entry burn fires into the
/// oncoming air and cools over the following minutes.
fn engine_heat(log: &FlightLog, t: f64) -> f32 {
    let at = |phase: Phase| {
        log.events
            .iter()
            .find(|e| e.text == phase.label())
            .map(|e| e.t)
    };
    let Some(start) = at(Phase::EntryBurn).filter(|&start| start <= t) else {
        return 0.0;
    };
    let end = at(Phase::AeroDescent).unwrap_or(f64::INFINITY).min(t);
    let peak = ((end - start) / HEAT_TIME).min(1.0);
    (peak * (-(t - end) / COOL_TIME).exp()) as f32
}

/// Size the pushed-back exhaust, light the engine section's heat glow, and set the light
/// the plumes cast.
pub fn glow(
    playback: Res<Playback>,
    assets: Res<BoosterAssets>,
    mut materials: ResMut<Assets<StandardMaterial>>,
    envelope: Single<(&mut Transform, &mut Visibility), With<Envelope>>,
    heat: Single<&mut Visibility, (With<HeatGlow>, Without<Envelope>)>,
    mut plume_light: Single<&mut PointLight, With<PlumeLight>>,
) {
    let Some(frame) = playback.frame() else {
        return;
    };
    let shown = |on: bool| {
        if on {
            Visibility::Visible
        } else {
            Visibility::Hidden
        }
    };
    let strength = retro_strength(&frame);
    let (mut transform, mut visibility) = envelope.into_inner();
    *visibility = shown(strength > 0.02);
    let radius = 2.6 + 3.0 * strength;
    transform.scale = Vec3::new(radius, 6.0 + 24.0 * strength, radius);
    if let Some(mut m) = materials.get_mut(&assets.envelope) {
        m.emissive = ENVELOPE_GLOW * strength.sqrt();
    }

    let warmth = engine_heat(playback.log(), playback.cursor);
    *heat.into_inner() = shown(warmth > 0.01);
    if let Some(mut m) = materials.get_mut(&assets.heat) {
        m.emissive = HEAT_GLOW * warmth;
    }
    plume_light.intensity = PLUME_LIGHT * frame.thrust_level;
}

/// How far each leg's swing must back off to keep its foot on the surface (rad): a foot
/// pressed `depth` below the ground (or the drone ship's deck, given as its sim-frame
/// origin and attitude) by the legs' sag rises by about the leg's horizontal reach per
/// radian.
fn leg_compression(frame: &Frame, hinge: Vec3, v: &Vehicle, deck: Option<(Vec3, Quat)>) -> f32 {
    if frame.leg_deploy < 1.0 {
        return 0.0;
    }
    let out = Vec3::new(hinge.x, 0.0, hinge.z).normalize_or_zero();
    let foot = out * v.leg_radius as f32 - Vec3::Y * v.standing_height(frame.mass as f64) as f32;
    let world = from_bevy(to_bevy(frame.pos) + rotation_to_bevy(frame.att) * foot);
    let depth = match deck {
        Some((origin, att)) => -(att.inverse() * (world - origin)).z,
        None => -world.z,
    };
    depth.max(0.0) / (v.leg_radius - v.radius) as f32
}

#[allow(clippy::type_complexity)] // Bevy system parameters are naturally nested.
pub fn sync(
    time: Res<Time>,
    playback: Res<Playback>,
    mut parts: ParamSet<(
        Query<&mut Transform, With<Booster>>,
        Query<&mut Transform, With<EngineCluster>>,
        Query<(&mut Transform, &mut Visibility, &Plume)>,
        Query<(&mut Transform, &GridFin)>,
        Query<(&mut Transform, &Leg)>,
        Query<&mut Transform, With<Strut>>,
    )>,
) {
    let Some(frame) = playback.frame() else {
        return;
    };

    // The root sits at the model's origin, which is above or below the center of mass
    // depending on how much propellant is left.
    let com = Vehicle::nominal().com_height(frame.mass as f64) as f32;
    for mut t in &mut parts.p0() {
        t.rotation = rotation_to_bevy(frame.att);
        t.translation = to_bevy(frame.pos) + t.rotation * Vec3::Y * (MODEL_ORIGIN_HEIGHT - com);
    }
    // Thrust leaves along the body axis tilted by the gimbal angles (body x, y).
    let thrust_dir =
        to_bevy(Vec3::new(frame.gimbal.x.tan(), frame.gimbal.y.tan(), 1.0).normalize());
    for mut t in &mut parts.p1() {
        t.rotation = Quat::from_rotation_arc(Vec3::Y, thrust_dir);
    }

    // Plumes stretch with throttle and balloon out as the air thins; in thick air the core
    // shows a train of shock diamonds.
    let engines = frame.engines.max(1);
    let level = (frame.thrust_level / engines as f32).clamp(0.0, 1.0);
    let vacuum = 1.0 - (pressure(frame.pos.z as f64) / SEA_LEVEL_PRESSURE) as f32;
    // Fast oncoming air squashes the plumes back toward the nozzles.
    let squash = 1.0 - 0.6 * retro_strength(&frame);
    let exit = -NOZZLE_EXIT_BELOW_BASE;
    let t_now = time.elapsed_secs();
    for (mut t, mut visibility, plume) in &mut parts.p2() {
        let burning = is_lit(plume.engine, frame.engines) && frame.thrust_level > 0.02;
        let shown = burning && !(matches!(plume.layer, PlumeLayer::Diamond(_)) && vacuum > 0.3);
        *visibility = if shown {
            Visibility::Visible
        } else {
            Visibility::Hidden
        };
        if !shown {
            continue;
        }
        let e = plume.engine as f32;
        let flicker = 1.0 + 0.06 * (t_now * 53.0 + e).sin() + 0.04 * (t_now * 89.0 + 2.0 * e).sin();
        let p = engine_position(plume.engine);
        let (width, length, offset, rotation) = match plume.layer {
            PlumeLayer::Glow => {
                let length = (10.0 + 22.0 * level) * (1.0 + 1.2 * vacuum) * flicker * squash;
                (
                    (1.1 + 0.7 * level + 5.0 * vacuum * level) * squash,
                    length,
                    length / 2.0,
                    Quat::IDENTITY,
                )
            }
            PlumeLayer::Core => {
                let length = (4.0 + 8.0 * level) * (1.0 - 0.5 * vacuum) * flicker * squash;
                (0.8, length, length / 2.0, Quat::from_rotation_x(PI))
            }
            PlumeLayer::Diamond(i) => {
                let size = 0.32 * (1.0 - 0.18 * i as f32) * (0.6 + 0.4 * level);
                (
                    size,
                    size * 1.6,
                    (i as f32 + 0.8) * DIAMOND_SPACING * (0.7 + 0.3 * level),
                    Quat::IDENTITY,
                )
            }
        };
        t.translation = Vec3::new(p.x, exit - offset, p.y);
        t.rotation = rotation;
        t.scale = Vec3::new(width, length, width);
    }

    for (mut t, fin) in &mut parts.p3() {
        // Fins fold up flat against the body when stowed and swing out to horizontal.
        let roll = frame.fins.z;
        let deflection = match fin.index {
            0 => frame.fins.y + roll,
            1 => frame.fins.x + roll,
            2 => frame.fins.y - roll,
            _ => frame.fins.x - roll,
        };
        let fold = Quat::from_rotation_z((1.0 - frame.fin_deploy) * FRAC_PI_2);
        t.rotation = fin.rest * fold * Quat::from_rotation_x(deflection);
    }

    // Stowed legs lie along the body; deployed ones swing out and down about their hinge,
    // pushed by a strut from higher up the body, and give a little as the legs take the
    // weight so the feet stay on the ground.
    let vehicle = Vehicle::nominal();
    let leg = leg_geometry(&vehicle);
    let deck = playback.log().ship.map(|ship| {
        let pose = ship.pose(playback.cursor);
        (pose.origin.as_vec3(), pose.att.as_quat())
    });
    let mut struts = Vec::with_capacity(4);
    for (mut t, l) in &mut parts.p4() {
        let swing = frame.leg_deploy * leg.deployed_angle
            - leg_compression(&frame, l.hinge, &vehicle, deck);
        t.rotation = l.rest * Quat::from_rotation_z(-swing);
        let on_body = l.hinge + Vec3::Y * STRUT_BODY_ABOVE_HINGE;
        let on_leg = l.hinge + t.rotation * Vec3::new(0.25, STRUT_ON_LEG * leg.length, 0.0);
        let span = on_leg - on_body;
        let strut = Transform::from_translation((on_body + on_leg) / 2.0)
            .with_rotation(Quat::from_rotation_arc(Vec3::Y, span.normalize()))
            .with_scale(Vec3::new(1.0, span.length(), 1.0));
        struts.push((l.strut, strut));
    }
    let mut strut_query = parts.p5();
    for (entity, transform) in struts {
        if let Ok(mut t) = strut_query.get_mut(entity) {
            *t = transform;
        }
    }
}
