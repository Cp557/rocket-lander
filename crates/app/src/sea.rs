//! The drone ship's surroundings: the ship, whose deck moves as it did in the flight, and
//! a patch of swell around it. While a flight lands at sea, the land, the pad and its
//! floodlights are hidden and the ocean drops to the ship's waterline.

use bevy::asset::RenderAssetUsages;
use bevy::mesh::{Indices, PrimitiveTopology, VertexAttributeValues};
use bevy::prelude::*;
use sim::ship::{DECK_HALF_LENGTH, DECK_HALF_WIDTH, FREEBOARD};

use crate::playback::Playback;
use crate::scene::{rotation_to_bevy, to_bevy};

/// Everything that belongs to the landing site ashore.
#[derive(Component)]
pub struct LandSite;

/// The planet-wide ocean disc, which sits 1 m below the land.
#[derive(Component)]
pub struct Ocean;

#[derive(Component)]
pub struct SeaSite;

#[derive(Component)]
pub struct ShipModel;

#[derive(Resource)]
pub struct WaveMesh(Handle<Mesh>);

/// Half-size of the swell patch around the ship, its grid resolution, and the band at its
/// edge over which the waves die down to the flat ocean beyond.
const WAVE_HALF_SIZE: f32 = 1200.0;
const WAVE_GRID: u32 = 160;
const WAVE_FADE: f32 = 400.0;
/// Wave trains (direction from north, clockwise, in degrees; share of the ship's heave
/// amplitude; period relative to the heave period).
const WAVE_TRAINS: [(f32, f32, f32); 3] =
    [(20.0, 0.7, 1.0), (55.0, 0.35, 0.8), (-40.0, 0.25, 1.25)];
const GRAVITY: f32 = 9.81;
/// The ocean disc's height near the origin (`terrain::sea_level`) above the waterline.
const OCEAN_DISC_OFFSET: f32 = FREEBOARD as f32 - 1.0;

pub fn setup(
    mut commands: Commands,
    mut meshes: ResMut<Assets<Mesh>>,
    mut materials: ResMut<Assets<StandardMaterial>>,
) {
    let mut material = |color: Color, roughness: f32| {
        materials.add(StandardMaterial {
            base_color: color,
            perceptual_roughness: roughness,
            ..default()
        })
    };
    let hull = material(Color::srgb(0.06, 0.07, 0.08), 0.6);
    let deck = material(Color::srgb(0.28, 0.29, 0.30), 0.85);
    let white = material(Color::srgb(0.92, 0.92, 0.92), 0.6);
    let yellow = material(Color::srgb(0.95, 0.78, 0.10), 0.6);
    let (length, width) = (2.0 * DECK_HALF_LENGTH as f32, 2.0 * DECK_HALF_WIDTH as f32);
    let hull_depth = 8.0;
    let flat = |y: f32| {
        Transform::from_xyz(0.0, y, 0.0)
            .with_rotation(Quat::from_rotation_x(-std::f32::consts::FRAC_PI_2))
    };
    let mut parts: Vec<(Mesh, Handle<StandardMaterial>, Transform)> = vec![
        (
            Cuboid::new(length + 6.0, hull_depth, width + 4.0).into(),
            hull,
            Transform::from_xyz(0.0, -hull_depth / 2.0 - 0.3, 0.0),
        ),
        (
            Cuboid::new(length, 0.3, width).into(),
            deck,
            Transform::from_xyz(0.0, -0.15, 0.0),
        ),
        (Annulus::new(14.0, 15.5).into(), white.clone(), flat(0.02)),
        (Annulus::new(24.0, 25.0).into(), yellow, flat(0.02)),
    ];
    // An X across the target, and blast walls at both ends of the deck.
    for angle in [std::f32::consts::FRAC_PI_4, -std::f32::consts::FRAC_PI_4] {
        parts.push((
            Cuboid::new(30.0, 0.02, 2.5).into(),
            white.clone(),
            Transform::from_xyz(0.0, 0.02, 0.0).with_rotation(Quat::from_rotation_y(angle)),
        ));
    }
    let wall = material(Color::srgb(0.12, 0.13, 0.14), 0.7);
    for end in [-1.0, 1.0] {
        parts.push((
            Cuboid::new(1.5, 4.0, width).into(),
            wall.clone(),
            Transform::from_xyz(end * (length / 2.0 + 0.75), 2.0, 0.0),
        ));
    }
    commands
        .spawn((SeaSite, ShipModel, Transform::default(), Visibility::Hidden))
        .with_children(|ship| {
            for (mesh, material, transform) in parts {
                ship.spawn((
                    Mesh3d(meshes.add(mesh)),
                    MeshMaterial3d(material),
                    transform,
                ));
            }
        });

    let waves = meshes.add(wave_grid());
    commands.spawn((
        SeaSite,
        Mesh3d(waves.clone()),
        MeshMaterial3d(materials.add(StandardMaterial {
            base_color: Color::srgb(0.02, 0.09, 0.15),
            perceptual_roughness: 0.12,
            reflectance: 0.6,
            ..default()
        })),
        Transform::from_xyz(0.0, -(FREEBOARD as f32), 0.0),
        Visibility::Hidden,
    ));
    commands.insert_resource(WaveMesh(waves));
}

/// Flat grid of the swell patch, with positions and normals filled in every frame.
fn wave_grid() -> Mesh {
    let n = WAVE_GRID;
    let count = ((n + 1) * (n + 1)) as usize;
    let indices = (0..n)
        .flat_map(|row| {
            (0..n).flat_map(move |col| {
                let a = row * (n + 1) + col;
                let b = a + n + 1;
                [a, b, a + 1, a + 1, b, b + 1]
            })
        })
        .collect();
    Mesh::new(
        PrimitiveTopology::TriangleList,
        RenderAssetUsages::RENDER_WORLD | RenderAssetUsages::MAIN_WORLD,
    )
    .with_inserted_attribute(Mesh::ATTRIBUTE_POSITION, vec![[0.0f32; 3]; count])
    .with_inserted_attribute(Mesh::ATTRIBUTE_NORMAL, vec![[0.0f32, 1.0, 0.0]; count])
    .with_inserted_indices(Indices::U32(indices))
}

/// Swell height and slope (d/dx, d/dz) at a Bevy-frame point and time, for a heave
/// amplitude and period.
fn swell(x: f32, z: f32, t: f32, amplitude: f32, period: f32) -> (f32, f32, f32) {
    let (mut height, mut dx, mut dz) = (0.0, 0.0, 0.0);
    for (i, (heading, share, relative_period)) in WAVE_TRAINS.into_iter().enumerate() {
        let period = period * relative_period;
        let omega = std::f32::consts::TAU / period;
        // Deep-water waves: wavenumber from the dispersion relation omega^2 = g k.
        let k = omega * omega / GRAVITY;
        let heading = heading.to_radians();
        // From north clockwise, in Bevy x (east) and z (south).
        let (kx, kz) = (k * heading.sin(), -k * heading.cos());
        let phase = kx * x + kz * z - omega * t + i as f32 * 1.7;
        let a = amplitude * share;
        height += a * phase.sin();
        dx += a * kx * phase.cos();
        dz += a * kz * phase.cos();
    }
    (height, dx, dz)
}

#[allow(clippy::type_complexity)] // Bevy system parameters are naturally nested.
pub fn update(
    playback: Res<Playback>,
    waves: Res<WaveMesh>,
    mut meshes: ResMut<Assets<Mesh>>,
    mut shown_at_sea: Local<Option<bool>>,
    mut sites: ParamSet<(
        Query<&mut Visibility, With<LandSite>>,
        Query<&mut Visibility, With<SeaSite>>,
        Query<&mut Transform, With<Ocean>>,
        Query<&mut Transform, With<ShipModel>>,
    )>,
) {
    let ship = playback.log().ship;
    let at_sea = ship.is_some();
    if *shown_at_sea != Some(at_sea) {
        *shown_at_sea = Some(at_sea);
        let shown = |on: bool| {
            if on {
                Visibility::Inherited
            } else {
                Visibility::Hidden
            }
        };
        for mut v in &mut sites.p0() {
            *v = shown(!at_sea);
        }
        for mut v in &mut sites.p1() {
            *v = shown(at_sea);
        }
        for mut t in &mut sites.p2() {
            t.translation.y = if at_sea { -OCEAN_DISC_OFFSET } else { 0.0 };
        }
    }
    let Some(ship) = ship else {
        return;
    };
    let t = playback.cursor;
    let pose = ship.pose(t);
    for mut transform in &mut sites.p3() {
        transform.translation = to_bevy(pose.origin.as_vec3());
        transform.rotation = rotation_to_bevy(pose.att.as_quat());
    }

    let Some(mut mesh) = meshes.get_mut(&waves.0) else {
        return;
    };
    let (amplitude, period) = (ship.heave.amplitude as f32, ship.heave.period as f32);
    let n = WAVE_GRID;
    let step = 2.0 * WAVE_HALF_SIZE / n as f32;
    let (mut positions, mut normals) = (Vec::new(), Vec::new());
    for row in 0..=n {
        for col in 0..=n {
            let (x, z) = (
                -WAVE_HALF_SIZE + col as f32 * step,
                -WAVE_HALF_SIZE + row as f32 * step,
            );
            let edge = WAVE_HALF_SIZE - x.abs().max(z.abs());
            let fade = (edge / WAVE_FADE).clamp(0.0, 1.0);
            let (height, dx, dz) = swell(x, z, t as f32, amplitude, period);
            positions.push([x, height * fade, z]);
            normals.push(
                Vec3::new(-dx * fade, 1.0, -dz * fade)
                    .normalize()
                    .to_array(),
            );
        }
    }
    mesh.insert_attribute(
        Mesh::ATTRIBUTE_POSITION,
        VertexAttributeValues::Float32x3(positions),
    );
    mesh.insert_attribute(
        Mesh::ATTRIBUTE_NORMAL,
        VertexAttributeValues::Float32x3(normals),
    );
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn swell_slope_matches_its_height() {
        let (x, z, t, h) = (130.0, -75.0, 12.0, 0.01);
        let (y, dx, dz) = swell(x, z, t, 0.8, 8.0);
        let (yx, _, _) = swell(x + h, z, t, 0.8, 8.0);
        let (yz, _, _) = swell(x, z + h, t, 0.8, 8.0);
        assert!(((yx - y) / h - dx).abs() < 1e-3, "{} vs {dx}", (yx - y) / h);
        assert!(((yz - y) / h - dz).abs() < 1e-3, "{} vs {dz}", (yz - y) / h);
    }
}
