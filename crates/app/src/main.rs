//! Mission-control viewer: 3D scene, live telemetry plots, replay and Monte Carlo, or,
//! with `--record`, a video of one flight (see `record`).
//!
//! The simulation itself lives in the `mission` crate; this app only advances it and
//! draws whatever frame the playback cursor points at.

mod audio;
mod booster;
mod camera;
mod montecarlo;
mod overlay;
mod particles;
mod playback;
mod record;
mod scene;
mod sea;
mod terrain;
mod ui;

use bevy::asset::embedded_asset;
use bevy::prelude::*;
use bevy::window::WindowResolution;
use bevy_egui::{EguiPlugin, EguiPrimaryContextPass};
use mission::{FlightConfig, Scenario};

fn main() {
    let args: Vec<String> = std::env::args().collect();
    let recording = match record::parse(&args) {
        Ok(recording) => recording,
        Err(e) => {
            eprintln!("{e}");
            std::process::exit(2);
        }
    };
    let first_flight = FlightConfig {
        scenario: Scenario::FullReturn,
        seed: 1,
        dispersed: false,
        stress: 0.0,
    };
    let (width, height) = recording.as_ref().map_or((1600, 900), |r| r.size);
    let mut resolution = WindowResolution::new(width, height);
    if recording.is_some() {
        // Exact video pixels whatever the display scaling.
        resolution = resolution.with_scale_factor_override(1.0);
    }
    let mut app = App::new();
    app.add_plugins(DefaultPlugins.set(WindowPlugin {
        primary_window: Some(Window {
            title: "Rocket Lander".into(),
            resolution,
            resizable: recording.is_none(),
            ..default()
        }),
        ..default()
    }))
    .add_plugins(EguiPlugin::default())
    .insert_resource(playback::Playback::live(first_flight))
    .init_resource::<montecarlo::MonteCarlo>()
    .init_resource::<camera::Orbit>()
    .init_resource::<overlay::Overlays>()
    .init_resource::<scene::TimeOfDay>()
    .insert_resource(ClearColor(Color::BLACK))
    .add_systems(
        Startup,
        (
            scene::setup,
            sea::setup,
            booster::setup,
            particles::setup,
            camera::setup,
        ),
    )
    .add_systems(
        Update,
        (
            playback::advance,
            booster::bind_parts,
            booster::sync,
            booster::glow,
            particles::update,
            particles::daylight,
            camera::control,
            camera::follow,
            scene::update_sky,
            sea::update,
            overlay::draw,
        )
            .chain(),
    );
    match recording {
        // A clean picture: no panels (the egui pass must still exist), no sound.
        Some(options) => {
            record::add(&mut app, options);
            app.add_systems(EguiPrimaryContextPass, || {});
        }
        None => {
            app.add_plugins(audio::plugin)
                .add_systems(EguiPrimaryContextPass, ui::draw);
        }
    }
    // Built by blender/booster.py; embedded so the binary runs from anywhere.
    embedded_asset!(app, "models/booster.glb");
    app.run();
}
