//! Cosmetic particles: exhaust smoke in the lower atmosphere, dust blown off the ground
//! near touchdown, and cold-gas thruster puffs. They are soft, lit camera-facing billboards
//! that advance with flight time, so they pause with playback, speed up with it, and are
//! cleared when the cursor jumps.

use bevy::light::NotShadowCaster;
use bevy::prelude::*;
use sim::Vehicle;

use crate::booster::NOZZLE_EXIT_BELOW_BASE;
use crate::camera::MainCamera;
use crate::playback::Playback;
use crate::scene::{TimeOfDay, darkness, rotation_to_bevy, to_bevy};
use crate::terrain::{fbm, rgba_image};

#[derive(Clone, Copy, PartialEq, Eq)]
enum Kind {
    Smoke,
    Dust,
    Gas,
}

#[derive(Component)]
pub struct Particle {
    kind: Kind,
    vel: Vec3,
    age: f32,
    life: f32,
    start_size: f32,
    end_size: f32,
    /// Fixed roll of the billboard, so neighboring puffs look different.
    spin: f32,
}

/// Shared mesh and a ladder of increasingly transparent materials per particle kind, so
/// fading a particle is just swapping its material handle.
#[derive(Resource)]
pub struct ParticleAssets {
    mesh: Handle<Mesh>,
    /// Indexed by `Kind`.
    ladders: [Vec<Handle<StandardMaterial>>; 3],
}

const FADE_STEPS: usize = 8;
/// How brightly the particles glow with scattered sunlight by day, as a fraction of their
/// color.
const DAY_GLOW: f32 = 0.7;
const MAX_PARTICLES: usize = 800;
/// No visible exhaust smoke above this altitude.
const SMOKE_CEILING: f32 = 25_000.0;
/// Engine-exit height below which the exhaust raises dust.
const DUST_HEIGHT: f32 = 45.0;
/// Cold-gas torque above which a thruster puff is shown.
const RCS_PUFF_TORQUE: f32 = 20e3;
/// Rate at which particles take on the wind's velocity.
const AIR_DRAG: f32 = 1.2;

/// White texture whose alpha falls off from the center to the edge; `noisy` breaks the
/// edge up into a puff.
pub fn radial_fade_texture(noisy: bool) -> Image {
    const SIZE: usize = 64;
    let mut data = Vec::with_capacity(SIZE * SIZE * 4);
    for y in 0..SIZE {
        for x in 0..SIZE {
            let (u, v) = (
                (x as f32 + 0.5) / SIZE as f32 - 0.5,
                (y as f32 + 0.5) / SIZE as f32 - 0.5,
            );
            let r = 2.0 * u.hypot(v);
            let wobble = if noisy {
                0.35 * (fbm(u * 6.0 + 7.0, v * 6.0 + 3.0, 3, 0) - 0.5)
            } else {
                0.0
            };
            let falloff = if noisy { 1.5 } else { 0.8 };
            let alpha = (1.0 - r + wobble).clamp(0.0, 1.0).powf(falloff);
            data.extend_from_slice(&[255, 255, 255, (alpha * 255.0) as u8]);
        }
    }
    rgba_image(SIZE as u32, SIZE as u32, data)
}

/// Opaque at the bottom row (v = 0) and fading out toward the top, for surfaces of
/// revolution whose UV v runs along their profile.
pub fn linear_fade_texture() -> Image {
    const SIZE: usize = 64;
    let data = (0..SIZE)
        .flat_map(|y| {
            let alpha = (1.0 - (y as f32 + 0.5) / SIZE as f32).powf(1.5);
            [255, 255, 255, (alpha * 255.0) as u8]
        })
        .collect();
    rgba_image(1, SIZE as u32, data)
}

pub fn setup(
    mut commands: Commands,
    mut meshes: ResMut<Assets<Mesh>>,
    mut materials: ResMut<Assets<StandardMaterial>>,
    mut images: ResMut<Assets<Image>>,
) {
    let puff = images.add(radial_fade_texture(true));
    let mut ladder = |r: f32, g: f32, b: f32, opacity: f32| -> Vec<Handle<StandardMaterial>> {
        (0..FADE_STEPS)
            .map(|i| {
                let fade = 1.0 - i as f32 / FADE_STEPS as f32;
                materials.add(StandardMaterial {
                    base_color: Color::srgba(r, g, b, opacity * fade),
                    base_color_texture: Some(puff.clone()),
                    alpha_mode: AlphaMode::Blend,
                    // Lit, so the sun, the plume and the floodlights all show on the smoke.
                    perceptual_roughness: 1.0,
                    reflectance: 0.0,
                    double_sided: true,
                    cull_mode: None,
                    ..default()
                })
            })
            .collect()
    };
    let ladders = [
        ladder(0.78, 0.78, 0.76, 0.35),
        ladder(0.60, 0.53, 0.42, 0.5),
        ladder(0.95, 0.95, 0.97, 0.45),
    ];
    let mesh = meshes.add(Rectangle::new(1.0, 1.0));
    commands.insert_resource(ParticleAssets { mesh, ladders });
}

/// By day the particles glow with scattered sunlight; at night only lights show them.
pub fn daylight(
    time: Res<TimeOfDay>,
    assets: Res<ParticleAssets>,
    mut materials: ResMut<Assets<StandardMaterial>>,
    mut shown_hour: Local<Option<f32>>,
) {
    if *shown_hour == Some(time.hour) {
        return;
    }
    *shown_hour = Some(time.hour);
    let day = 1.0 - darkness(time.hour);
    for handle in assets.ladders.iter().flatten() {
        if let Some(mut m) = materials.get_mut(handle) {
            let c = m.base_color.to_linear();
            m.emissive = LinearRgba::rgb(c.red, c.green, c.blue) * day * DAY_GLOW;
        }
    }
}

#[derive(Default)]
pub struct EmitterState {
    last_cursor: f64,
    carry: [f32; 3],
    seed: u64,
}

impl EmitterState {
    fn random(&mut self) -> f32 {
        self.seed = self
            .seed
            .wrapping_mul(6364136223846793005)
            .wrapping_add(1442695040888963407);
        (self.seed >> 40) as f32 / (1u64 << 24) as f32
    }

    fn random_dir(&mut self) -> Vec3 {
        Vec3::new(
            self.random() - 0.5,
            self.random() - 0.5,
            self.random() - 0.5,
        )
        .normalize_or_zero()
    }

    /// Whole particles due this step for an emitter at `rate` per second.
    fn due(&mut self, slot: usize, rate: f32, dt: f32) -> usize {
        self.carry[slot] += rate * dt;
        let n = self.carry[slot].floor();
        self.carry[slot] -= n;
        n as usize
    }
}

pub fn update(
    mut commands: Commands,
    playback: Res<Playback>,
    assets: Res<ParticleAssets>,
    camera: Single<&Transform, (With<MainCamera>, Without<Particle>)>,
    mut particles: Query<(
        Entity,
        &mut Particle,
        &mut Transform,
        &mut MeshMaterial3d<StandardMaterial>,
    )>,
    mut state: Local<EmitterState>,
) {
    let dt = (playback.cursor - state.last_cursor) as f32;
    state.last_cursor = playback.cursor;
    if !(0.0..1.0).contains(&dt) {
        for (entity, ..) in &particles {
            commands.entity(entity).despawn();
        }
        return;
    }
    if dt == 0.0 {
        return;
    }
    let Some(frame) = playback.frame() else {
        return;
    };
    let wind = to_bevy(frame.wind);

    let blend = 1.0 - (-AIR_DRAG * dt).exp();
    for (entity, mut p, mut t, mut material) in &mut particles {
        p.age += dt;
        if p.age >= p.life {
            commands.entity(entity).despawn();
            continue;
        }
        let air = if p.kind == Kind::Dust {
            Vec3::ZERO
        } else {
            wind
        };
        p.vel = p.vel.lerp(air, blend);
        t.translation += p.vel * dt;
        let progress = p.age / p.life;
        t.scale = Vec3::splat(p.start_size + (p.end_size - p.start_size) * progress);
        t.rotation = camera.rotation * Quat::from_rotation_z(p.spin);
        let step = ((progress * FADE_STEPS as f32) as usize).min(FADE_STEPS - 1);
        material.0 = assets.ladders[p.kind as usize][step].clone();
    }

    let mut budget = MAX_PARTICLES.saturating_sub(particles.iter().len());
    let facing = camera.rotation;
    let mut spawn = |commands: &mut Commands,
                     kind: Kind,
                     pos: Vec3,
                     vel: Vec3,
                     life: f32,
                     sizes: (f32, f32),
                     spin: f32| {
        if budget == 0 {
            return;
        }
        budget -= 1;
        commands.spawn((
            Particle {
                kind,
                vel,
                age: 0.0,
                life,
                start_size: sizes.0,
                end_size: sizes.1,
                spin,
            },
            Mesh3d(assets.mesh.clone()),
            MeshMaterial3d(assets.ladders[kind as usize][0].clone()),
            NotShadowCaster,
            Transform::from_translation(pos)
                .with_scale(Vec3::splat(sizes.0))
                .with_rotation(facing * Quat::from_rotation_z(spin)),
        ));
    };

    let v = Vehicle::nominal();
    let rotation = rotation_to_bevy(frame.att);
    let center = to_bevy(frame.pos);
    let velocity = to_bevy(frame.vel);
    let com = v.com_height(frame.mass as f64) as f32;
    let nozzle = center + rotation * Vec3::new(0.0, -(com + NOZZLE_EXIT_BELOW_BASE), 0.0);
    let exhaust =
        rotation * -to_bevy(Vec3::new(frame.gimbal.x.tan(), frame.gimbal.y.tan(), 1.0).normalize());
    let burning = frame.thrust_level > 0.05;

    if burning && frame.pos.z < SMOKE_CEILING {
        for _ in 0..state.due(0, 20.0 * frame.thrust_level, dt) {
            let jitter = state.random_dir() * 2.0;
            let spread = state.random_dir() * 10.0;
            // Spread spawns along the path flown this step so fast flight leaves a trail.
            let back = velocity * dt * state.random();
            let pos = nozzle + exhaust * (8.0 + 12.0 * state.random()) + jitter - back;
            let life = 3.0 + 4.0 * state.random();
            let spin = state.random() * std::f32::consts::TAU;
            spawn(
                &mut commands,
                Kind::Smoke,
                pos,
                velocity * 0.5 + exhaust * 60.0 + spread,
                life,
                (5.0, 30.0),
                spin,
            );
        }
    }
    if burning && nozzle.y < DUST_HEIGHT {
        let strength = 1.0 - nozzle.y.max(0.0) / DUST_HEIGHT;
        let ground = Vec3::new(nozzle.x, 0.6, nozzle.z);
        for _ in 0..state.due(1, 120.0 * strength, dt) {
            let angle = state.random() * std::f32::consts::TAU;
            let out = Vec3::new(angle.cos(), 0.0, angle.sin());
            let pos = ground + out * (2.0 + 6.0 * state.random());
            let vel = out * (15.0 + 25.0 * state.random()) + Vec3::Y * (1.0 + 4.0 * state.random());
            let life = 2.5 + 2.5 * state.random();
            let spin = state.random() * std::f32::consts::TAU;
            spawn(&mut commands, Kind::Dust, pos, vel, life, (3.0, 18.0), spin);
        }
    }
    if frame.rcs.length() > RCS_PUFF_TORQUE {
        let nose = center + rotation * Vec3::Y * (v.length as f32 - com - 6.0);
        for _ in 0..state.due(2, 40.0, dt) {
            let out = state.random_dir();
            let pos = nose + out * (v.radius as f32 + 0.3);
            let spin = state.random() * std::f32::consts::TAU;
            spawn(
                &mut commands,
                Kind::Gas,
                pos,
                velocity + out * 25.0,
                0.8,
                (0.6, 4.0),
                spin,
            );
        }
    }
}
