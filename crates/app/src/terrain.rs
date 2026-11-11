//! Procedural ground: a curved land disc with a coastline east of the pad, a matching
//! ocean surface, and a tiling detail texture. Purely visual; physics uses flat ground.

use bevy::asset::RenderAssetUsages;
use bevy::image::{ImageAddressMode, ImageFilterMode, ImageSampler, ImageSamplerDescriptor};
use bevy::mesh::{Indices, PrimitiveTopology};
use bevy::prelude::*;
use bevy::render::render_resource::{Extent3d, TextureDimension, TextureFormat};

/// Must match `Atmosphere::earth`, whose planet center sits this far below the origin.
const PLANET_RADIUS: f32 = 6_360_000.0;
const GROUND_RADIUS: f32 = 1_200_000.0;
const SEGMENTS: usize = 384;
/// Ground covered by one repeat of the detail texture.
pub const TEXTURE_TILE: f32 = 400.0;
const TEXTURE_SIZE: usize = 512;
/// The sea sits a little below the land, and sinks further with distance so the two never
/// fight over depth near the horizon (invisible at that range).
fn sea_level(r: f32) -> f32 {
    -1.0 - 0.002 * r
}
const SEABED_DEPTH: f32 = 30.0;
/// The seabed shelves down from the shore at this slope, so the waterline crosses the
/// mesh smoothly instead of zigzagging along a cliff.
const SHELF_SLOPE: f32 = 0.05;
const BEACH_WIDTH: f32 = 250.0;

/// How far a sphere's surface falls below its tangent plane at distance `r`.
fn curvature_drop(r: f32) -> f32 {
    r * r / (2.0 * PLANET_RADIUS)
}

/// East-west position of the shoreline (Bevy x) for a northing (Bevy z); ocean lies east.
fn shore(z: f32) -> f32 {
    4_000.0 + 1_500.0 * (z / 7_000.0).sin() + 600.0 * (z / 2_300.0 + 1.0).sin()
}

/// Ring radii: dense near the pad, growing geometrically toward the horizon.
fn ring_radii() -> Vec<f32> {
    let mut radii = vec![0.0];
    let mut step = 25.0;
    while radii[radii.len() - 1] < GROUND_RADIUS {
        let last = radii[radii.len() - 1];
        if last >= 1_000.0 {
            step *= 1.06;
        }
        radii.push(last + step);
    }
    radii
}

/// A disc following the planet's curvature, with per-vertex height offset and color.
fn curved_disc(height: impl Fn(f32, f32) -> f32, color: impl Fn(f32, f32) -> [f32; 4]) -> Mesh {
    let radii = ring_radii();
    let (mut positions, mut normals, mut uvs, mut colors) =
        (Vec::new(), Vec::new(), Vec::new(), Vec::new());
    for &r in &radii {
        for s in 0..SEGMENTS {
            let a = s as f32 / SEGMENTS as f32 * std::f32::consts::TAU;
            let (x, z) = (r * a.cos(), r * a.sin());
            let y = height(x, z) - curvature_drop(r);
            positions.push([x, y, z]);
            normals.push(Vec3::new(x, y + PLANET_RADIUS, z).normalize().to_array());
            uvs.push([x / TEXTURE_TILE, z / TEXTURE_TILE]);
            colors.push(color(x, z));
        }
    }
    // Triangles wind counter-clockwise seen from above, so the surface faces up.
    let mut indices = Vec::new();
    for ring in 0..radii.len() as u32 - 1 {
        for s in 0..SEGMENTS as u32 {
            let next = (s + 1) % SEGMENTS as u32;
            let (a, b) = (ring * SEGMENTS as u32 + s, ring * SEGMENTS as u32 + next);
            let (c, d) = (a + SEGMENTS as u32, b + SEGMENTS as u32);
            indices.extend_from_slice(&[a, b, c, b, d, c]);
        }
    }
    Mesh::new(
        PrimitiveTopology::TriangleList,
        RenderAssetUsages::RENDER_WORLD,
    )
    .with_inserted_attribute(Mesh::ATTRIBUTE_POSITION, positions)
    .with_inserted_attribute(Mesh::ATTRIBUTE_NORMAL, normals)
    .with_inserted_attribute(Mesh::ATTRIBUTE_UV_0, uvs)
    .with_inserted_attribute(Mesh::ATTRIBUTE_COLOR, colors)
    .with_inserted_indices(Indices::U32(indices))
}

pub fn land_mesh() -> Mesh {
    curved_disc(
        |x, z| {
            let offshore = (x - shore(z)).max(0.0);
            (-offshore * SHELF_SLOPE).max(sea_level(x.hypot(z)) - SEABED_DEPTH)
        },
        |x, z| {
            let shore_distance = shore(z) - x;
            let fields = fbm(x / 6_000.0, z / 6_000.0, 4, 0);
            let patches = fbm(x / 900.0, z / 900.0, 3, 0);
            let grass = Vec3::new(0.16, 0.26, 0.09);
            let dry = Vec3::new(0.34, 0.33, 0.19);
            let forest = Vec3::new(0.06, 0.13, 0.05);
            let sand = Vec3::new(0.78, 0.72, 0.56);
            let mut c = grass.lerp(dry, smoothstep(0.5, 0.75, fields) * 0.7);
            c = c.lerp(forest, smoothstep(0.55, 0.75, patches) * 0.8);
            c = c.lerp(sand, 1.0 - smoothstep(0.0, BEACH_WIDTH, shore_distance));
            [c.x, c.y, c.z, 1.0]
        },
    )
}

pub fn ocean_mesh() -> Mesh {
    curved_disc(|x, z| sea_level(x.hypot(z)), |_, _| [1.0; 4])
}

/// Grayscale detail texture (tileable), multiplied over the land colors.
pub fn detail_texture() -> Image {
    let mut data = Vec::with_capacity(TEXTURE_SIZE * TEXTURE_SIZE * 4);
    for y in 0..TEXTURE_SIZE {
        for x in 0..TEXTURE_SIZE {
            let (u, v) = (
                x as f32 / TEXTURE_SIZE as f32,
                y as f32 / TEXTURE_SIZE as f32,
            );
            let n = fbm(u * 8.0, v * 8.0, 5, 8);
            let value = (170.0 + 85.0 * n) as u8;
            data.extend_from_slice(&[value, value, value, 255]);
        }
    }
    let mut image = rgba_image(TEXTURE_SIZE as u32, TEXTURE_SIZE as u32, data);
    image.sampler = ImageSampler::Descriptor(ImageSamplerDescriptor {
        address_mode_u: ImageAddressMode::Repeat,
        address_mode_v: ImageAddressMode::Repeat,
        mag_filter: ImageFilterMode::Linear,
        min_filter: ImageFilterMode::Linear,
        mipmap_filter: ImageFilterMode::Linear,
        anisotropy_clamp: 8,
        ..default()
    });
    image
}

/// An sRGB RGBA8 texture from rows of pixels.
pub fn rgba_image(width: u32, height: u32, data: Vec<u8>) -> Image {
    Image::new(
        Extent3d {
            width,
            height,
            depth_or_array_layers: 1,
        },
        TextureDimension::D2,
        data,
        TextureFormat::Rgba8UnormSrgb,
        RenderAssetUsages::RENDER_WORLD,
    )
}

fn smoothstep(edge0: f32, edge1: f32, x: f32) -> f32 {
    let t = ((x - edge0) / (edge1 - edge0)).clamp(0.0, 1.0);
    t * t * (3.0 - 2.0 * t)
}

/// Fractal value noise in 0..1. With `period > 0` it tiles every `period` units of the
/// first octave.
pub fn fbm(x: f32, y: f32, octaves: u32, period: i32) -> f32 {
    let (mut sum, mut amplitude, mut total, mut scale) = (0.0, 1.0, 0.0, 1.0);
    for octave in 0..octaves {
        sum += amplitude * value_noise(x * scale, y * scale, period * scale as i32, octave);
        total += amplitude;
        amplitude *= 0.5;
        scale *= 2.0;
    }
    sum / total
}

fn value_noise(x: f32, y: f32, period: i32, seed: u32) -> f32 {
    let (x0, y0) = (x.floor(), y.floor());
    let (fx, fy) = (x - x0, y - y0);
    let lattice = |i: i32, j: i32| {
        let (i, j) = if period > 0 {
            (i.rem_euclid(period), j.rem_euclid(period))
        } else {
            (i, j)
        };
        hash(i, j, seed)
    };
    let (i, j) = (x0 as i32, y0 as i32);
    let (sx, sy) = (fx * fx * (3.0 - 2.0 * fx), fy * fy * (3.0 - 2.0 * fy));
    let top = lattice(i, j) + (lattice(i + 1, j) - lattice(i, j)) * sx;
    let bottom = lattice(i, j + 1) + (lattice(i + 1, j + 1) - lattice(i, j + 1)) * sx;
    top + (bottom - top) * sy
}

fn hash(i: i32, j: i32, seed: u32) -> f32 {
    let mut h = (i as u32).wrapping_mul(0x8da6_b343)
        ^ (j as u32).wrapping_mul(0xd816_3841)
        ^ seed.wrapping_mul(0xcb1a_b31f);
    h ^= h >> 13;
    h = h.wrapping_mul(0x5bd1_e995);
    h ^= h >> 15;
    (h & 0xffff) as f32 / 65_535.0
}
