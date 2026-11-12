//! Mission-control panels: key numbers across the top, the main telemetry plots on the
//! right (the rest folded under "More plots"), the controls and event log at the bottom,
//! and a Monte Carlo window. The 3D camera's viewport is fitted to the space left in the
//! middle.

use std::f64::consts::TAU;

use bevy::camera::Viewport;
use bevy::prelude::*;
use bevy::window::PrimaryWindow;
use bevy_egui::{EguiContexts, egui};
use egui::{Color32, LayerId, RichText, Ui, UiBuilder};
use egui_plot::{Legend, Line, Plot, PlotPoints, PlotUi, Points, VLine};
use mission::flight::{MAX_HORIZONTAL_SPEED, MAX_VERTICAL_SPEED, PAD_RADIUS, Touchdown};
use mission::montecarlo::RunResult;
use mission::{FlightConfig, Frame, Scenario};
use sim::Vehicle;

use crate::audio::Sound;
use crate::camera::{CameraMode, MainCamera, Orbit};
use crate::montecarlo::MonteCarlo;
use crate::playback::{FIRST_FLIGHT, Playback};

const SPEEDS: [f64; 7] = [0.5, 1.0, 2.0, 5.0, 10.0, 20.0, 50.0];
const MAX_PLOT_POINTS: usize = 1500;
const PLOT_HEIGHT: f32 = 140.0;
const TRUTH: Color32 = Color32::from_rgb(90, 170, 255);
const ESTIMATE: Color32 = Color32::from_rgb(255, 170, 60);
const GOOD: Color32 = Color32::from_rgb(80, 200, 120);
const BAD: Color32 = Color32::from_rgb(230, 80, 80);

pub struct UiState {
    config: FlightConfig,
    monte_carlo_open: bool,
    monte_carlo_runs: usize,
}

impl Default for UiState {
    fn default() -> Self {
        Self {
            config: FIRST_FLIGHT,
            monte_carlo_open: false,
            monte_carlo_runs: 100,
        }
    }
}

#[allow(clippy::too_many_arguments)] // Bevy system parameters.
pub fn draw(
    mut contexts: EguiContexts,
    mut playback: ResMut<Playback>,
    mut monte_carlo: ResMut<MonteCarlo>,
    mut orbit: ResMut<Orbit>,
    mut sound: ResMut<Sound>,
    mut state: Local<UiState>,
    mut camera: Single<&mut Camera, With<MainCamera>>,
    window: Single<&Window, With<PrimaryWindow>>,
) -> Result {
    let ctx = contexts.ctx_mut()?.clone();
    monte_carlo.poll();
    let mut root = Ui::new(
        ctx.clone(),
        "root".into(),
        UiBuilder::new()
            .layer_id(LayerId::background())
            .max_rect(ctx.viewport_rect()),
    );

    let top = egui::Panel::top("status")
        .show(&mut root, |ui| status_bar(ui, &playback))
        .response
        .rect
        .height();
    let bottom = egui::Panel::bottom("controls")
        .resizable(true)
        .default_size(150.0)
        .show(&mut root, |ui| {
            controls(ui, &mut playback, &mut state, &mut orbit, &mut sound)
        })
        .response
        .rect
        .height();
    let right = egui::Panel::right("plots")
        .resizable(true)
        .default_size(480.0)
        .show(&mut root, |ui| plots(ui, &playback))
        .response
        .rect
        .width();
    if state.monte_carlo_open {
        monte_carlo_window(&ctx, &mut state, &mut monte_carlo, &mut playback);
    }
    fit_viewport(&mut camera, &window, top, bottom, right);
    Ok(())
}

fn fit_viewport(camera: &mut Camera, window: &Window, top: f32, bottom: f32, right: f32) {
    let scale = window.scale_factor();
    let position = UVec2::new(0, (top * scale) as u32);
    let size = UVec2::new(
        window
            .physical_width()
            .saturating_sub((right * scale) as u32),
        window
            .physical_height()
            .saturating_sub(((top + bottom) * scale) as u32),
    );
    if size.x > 0 && size.y > 0 {
        camera.viewport = Some(Viewport {
            physical_position: position,
            physical_size: size,
            ..default()
        });
    }
}

fn tilt_deg(frame: &Frame) -> f64 {
    ((frame.att * Vec3::Z).z.clamp(-1.0, 1.0) as f64)
        .acos()
        .to_degrees()
}

fn status_bar(ui: &mut Ui, playback: &Playback) {
    let Some(f) = playback.frame() else { return };
    let dry = Vehicle::nominal().dry_mass as f32;
    let engines = match f.engines {
        0 => "off".to_string(),
        n => format!("{n} at {:.0}%", f.thrust_level / n as f32 * 100.0),
    };
    ui.horizontal(|ui| {
        let item = |ui: &mut Ui, label: &str, value: String| {
            ui.label(RichText::new(label).weak());
            ui.label(RichText::new(value).monospace().strong().size(15.0));
            ui.add_space(12.0);
        };
        item(ui, "T+", format!("{:6.1} s", f.t));
        ui.label(
            RichText::new(f.phase.label().to_uppercase())
                .monospace()
                .strong()
                .size(15.0)
                .color(ESTIMATE),
        );
        ui.add_space(10.0);
        item(ui, "Altitude", format!("{:6.0} m", f.pos.z));
        item(ui, "Speed", format!("{:6.1} m/s", f.vel.length()));
        item(ui, "Engines", engines);
        item(
            ui,
            "Propellant",
            format!("{:6.0} kg", (f.mass - dry).max(0.0)),
        );
        if let Some(outcome) = &playback.log().outcome {
            let (text, color) = if outcome.landed {
                ("LANDED".to_string(), GOOD)
            } else {
                (format!("FAILED: {}", outcome.verdict), BAD)
            };
            ui.label(RichText::new(text).strong().size(15.0).color(color));
        }
    });
}

fn controls(
    ui: &mut Ui,
    playback: &mut Playback,
    state: &mut UiState,
    orbit: &mut Orbit,
    sound: &mut Sound,
) {
    let duration = playback.log().duration();
    ui.horizontal(|ui| {
        let label = if playback.playing { "Pause" } else { "Play" };
        if ui.button(label).clicked() {
            if !playback.playing && playback.is_finished() && playback.cursor >= duration {
                playback.cursor = 0.0;
            }
            playback.playing = !playback.playing;
        }
        egui::ComboBox::from_id_salt("speed")
            .width(60.0)
            .selected_text(format!("{}x", playback.speed))
            .show_ui(ui, |ui| {
                for speed in SPEEDS {
                    ui.selectable_value(&mut playback.speed, speed, format!("{speed}x"));
                }
            });
        ui.spacing_mut().slider_width = (ui.available_width() - 90.0).max(100.0);
        let mut cursor = playback.cursor;
        let slider = egui::Slider::new(&mut cursor, 0.0..=duration.max(0.02))
            .custom_formatter(|t, _| format!("{t:.1} s"));
        // Only a drag or click moves the cursor; the slider may otherwise nudge the value.
        let response = ui.add(slider);
        if response.changed() && (response.dragged() || response.clicked()) {
            playback.cursor = cursor;
        }
    });
    ui.horizontal(|ui| {
        egui::ComboBox::from_id_salt("scenario")
            .selected_text(state.config.scenario.label())
            .show_ui(ui, |ui| {
                for scenario in Scenario::ALL {
                    ui.selectable_value(&mut state.config.scenario, scenario, scenario.label());
                }
            });
        ui.checkbox(&mut state.config.dispersed, "Random conditions")
            .on_hover_text(
                "Wind, vehicle and starting point drawn from the seed. \
                 Stress adds rougher weather and faults.",
            );
        if state.config.dispersed {
            ui.add(egui::DragValue::new(&mut state.config.seed).prefix("seed "));
            ui.add(stress_input(&mut state.config.stress));
        }
        if ui.button("Fly").clicked() {
            *playback = Playback::live(state.config);
        }
        ui.separator();
        egui::ComboBox::from_id_salt("camera")
            .selected_text(format!("Camera: {}", orbit.mode.label()))
            .show_ui(ui, |ui| {
                for mode in CameraMode::ALL {
                    ui.selectable_value(&mut orbit.mode, mode, mode.label());
                }
            });
        ui.checkbox(&mut sound.enabled, "Sound");
        ui.separator();
        ui.toggle_value(&mut state.monte_carlo_open, "Monte Carlo");
    });
    ui.separator();
    let mut jump = None;
    egui::ScrollArea::vertical()
        .auto_shrink(false)
        .stick_to_bottom(true)
        .show(ui, |ui| {
            for event in &playback.log().events {
                let reached = event.t <= playback.cursor;
                let text = RichText::new(format!("T+{:7.2}  {}", event.t, event.text)).monospace();
                let text = if reached { text } else { text.weak() };
                if ui.selectable_label(false, text).clicked() {
                    jump = Some(event.t);
                }
            }
        });
    if let Some(t) = jump {
        playback.cursor = t;
    }
}

fn stress_input(stress: &mut f64) -> egui::DragValue<'_> {
    egui::DragValue::new(stress)
        .range(0.0..=3.0)
        .speed(0.05)
        .fixed_decimals(1)
        .prefix("stress ")
}

/// One plotted line: name, color and the value it takes from each frame.
type Series<'a> = (&'a str, Color32, &'a dyn Fn(&Frame) -> f64);

fn plots(ui: &mut Ui, playback: &Playback) {
    let dry = Vehicle::nominal().dry_mass;
    let propellant = |f: &Frame| f.mass as f64 - dry;
    let propellant_estimate = |f: &Frame| f.est_mass as f64 - dry;
    let main: [(&str, Vec<Series>); 4] = [
        (
            "Altitude [m]",
            vec![
                ("truth", TRUTH, &|f| f.pos.z as f64),
                ("estimate", ESTIMATE, &|f| f.est_pos.z as f64),
            ],
        ),
        (
            "Speed [m/s]",
            vec![
                ("descent", TRUTH, &|f| -f.vel.z as f64),
                ("horizontal", ESTIMATE, &|f| {
                    f.vel.truncate().length() as f64
                }),
            ],
        ),
        (
            "Throttle command [%]",
            vec![("per engine", TRUTH, &|f| f.throttle_cmd as f64 * 100.0)],
        ),
        (
            "Navigation [m]",
            vec![
                ("position error", TRUTH, &|f| {
                    (f.pos - f.est_pos).length() as f64
                }),
                ("filter 1-sigma", ESTIMATE, &|f| f.pos_sigma as f64),
            ],
        ),
    ];
    let more: [(&str, Vec<Series>); 7] = [
        (
            "Thrust [MN]",
            vec![("thrust", TRUTH, &|f| f.thrust as f64 / 1e6)],
        ),
        (
            "Attitude [deg]",
            vec![
                ("tilt", TRUTH, &tilt_deg),
                ("angle of attack", ESTIMATE, &|f| {
                    (f.alpha as f64).to_degrees()
                }),
            ],
        ),
        (
            "Dynamic pressure [kPa]",
            vec![("q", TRUTH, &|f| f.dynamic_pressure as f64 / 1000.0)],
        ),
        (
            "Actuators [deg]",
            vec![
                ("gimbal", TRUTH, &|f| {
                    (f.gimbal.length() as f64).to_degrees()
                }),
                ("grid fins", ESTIMATE, &|f| {
                    (f.fins.abs().max_element() as f64).to_degrees()
                }),
            ],
        ),
        (
            "Wind speed [m/s]",
            vec![
                ("truth", TRUTH, &|f| f.wind.truncate().length() as f64),
                ("flight computer", ESTIMATE, &|f| {
                    f.est_wind.truncate().length() as f64
                }),
            ],
        ),
        (
            "Cold gas [kg]",
            vec![("left", TRUTH, &|f| f.rcs_gas as f64)],
        ),
        (
            "Propellant [kg]",
            vec![
                ("truth", TRUTH, &propellant),
                ("flight computer", ESTIMATE, &propellant_estimate),
            ],
        ),
    ];
    egui::ScrollArea::vertical().show(ui, |ui| {
        for (title, series) in &main {
            chart(ui, playback, title, series);
        }
        egui::CollapsingHeader::new("More plots").show(ui, |ui| {
            for (title, series) in &more {
                chart(ui, playback, title, series);
            }
        });
    });
}

/// One titled plot of `series` against flight time, with a cursor line.
fn chart(ui: &mut Ui, playback: &Playback, title: &str, series: &[Series]) {
    let log = playback.log();
    let step = (log.frames.len() / MAX_PLOT_POINTS).max(1);
    ui.label(RichText::new(title).strong());
    let plot = Plot::new(title)
        .height(PLOT_HEIGHT)
        .legend(Legend::default())
        .link_axis("time", [true, false])
        .allow_scroll(false);
    plot.show(ui, |plot| {
        for &(name, color, value) in series {
            let points: Vec<[f64; 2]> = log
                .frames
                .iter()
                .step_by(step)
                .map(|f| [f.t as f64, value(f)])
                .collect();
            plot.line(Line::new(name, PlotPoints::from(points)).color(color));
        }
        // Unnamed, so it stays out of the legend.
        plot.vline(VLine::new("", playback.cursor).color(Color32::YELLOW));
    });
}

fn monte_carlo_window(
    ctx: &egui::Context,
    state: &mut UiState,
    monte_carlo: &mut MonteCarlo,
    playback: &mut Playback,
) {
    let mut open = state.monte_carlo_open;
    let mut replay = None;
    egui::Window::new("Monte Carlo")
        .open(&mut open)
        .default_width(440.0)
        .show(ctx, |ui| {
            ui.horizontal(|ui| {
                ui.label(format!(
                    "{} with dispersions",
                    state.config.scenario.label()
                ));
                ui.add(stress_input(&mut state.config.stress));
                ui.add(
                    egui::DragValue::new(&mut state.monte_carlo_runs)
                        .range(10..=2000)
                        .prefix("runs "),
                );
                if ui
                    .add_enabled(monte_carlo.job.is_none(), egui::Button::new("Run"))
                    .clicked()
                {
                    monte_carlo.start(
                        state.config.scenario,
                        state.monte_carlo_runs,
                        1,
                        state.config.stress,
                    );
                }
            });
            if let Some(job) = &monte_carlo.job {
                let done = job.done();
                ui.add(
                    egui::ProgressBar::new(done as f32 / job.total as f32)
                        .text(format!("{done}/{} flights", job.total)),
                );
                ctx.request_repaint();
            }
            let Some(batch) = &monte_carlo.batch else {
                return;
            };
            let s = batch.summary;
            ui.label(format!(
                "{} at stress {:.1}: {}/{} landed ({:.0}%). Miss mean {:.1} m, p95 {:.1} m.",
                batch.scenario.label(),
                batch.stress,
                s.landed,
                s.runs,
                100.0 * s.landed as f64 / s.runs.max(1) as f64,
                s.mean_miss,
                s.p95_miss
            ));
            let (landed, failed): (Vec<_>, Vec<_>) =
                batch.runs.iter().partition(|r| r.outcome.landed);
            ui.label(RichText::new("Where they touched down [m]").strong());
            Plot::new("touchdowns")
                .height(280.0)
                .data_aspect(1.0)
                .legend(Legend::default())
                .show(ui, |plot| {
                    let pad: Vec<[f64; 2]> = (0..=64)
                        .map(|i| {
                            let a = i as f64 / 64.0 * TAU;
                            [PAD_RADIUS * a.cos(), PAD_RADIUS * a.sin()]
                        })
                        .collect();
                    plot.line(Line::new("pad", PlotPoints::from(pad)).color(Color32::GOLD));
                    scatter(plot, &landed, &failed, |t| [t.position.x, t.position.y]);
                });
            ui.label(RichText::new("Touchdown speed [m/s]: horizontal vs. descent").strong());
            Plot::new("touchdown speeds")
                .height(200.0)
                .include_x(0.0)
                .include_y(0.0)
                .show(ui, |plot| {
                    // The landing limits, as a box from the origin.
                    let (h, v) = (MAX_HORIZONTAL_SPEED, MAX_VERTICAL_SPEED);
                    let limits = vec![[h, 0.0], [h, v], [0.0, v]];
                    plot.line(Line::new("limits", PlotPoints::from(limits)).color(Color32::GOLD));
                    scatter(plot, &landed, &failed, |t| {
                        [t.horizontal_speed, t.vertical_speed]
                    });
                });
            ui.label(RichText::new("Failures").strong());
            egui::ScrollArea::vertical()
                .max_height(160.0)
                .show(ui, |ui| {
                    for run in &failed {
                        ui.horizontal(|ui| {
                            if ui.small_button("Replay").clicked() {
                                replay = Some(FlightConfig {
                                    scenario: batch.scenario,
                                    seed: run.seed,
                                    dispersed: true,
                                    stress: batch.stress,
                                });
                            }
                            ui.label(format!("seed {}: {}", run.seed, run.outcome.verdict));
                        });
                    }
                });
        });
    state.monte_carlo_open = open;
    if let Some(config) = replay {
        *playback = Playback::live(config);
        state.config = config;
    }
}

/// Touchdowns of the landed runs in green and the failed ones in red, placed by `point`.
fn scatter(
    plot: &mut PlotUi<'_>,
    landed: &[&RunResult],
    failed: &[&RunResult],
    point: impl Fn(Touchdown) -> [f64; 2],
) {
    for (name, color, runs) in [("landed", GOOD, landed), ("failed", BAD, failed)] {
        let points: Vec<[f64; 2]> = runs
            .iter()
            .filter_map(|r| r.outcome.touchdown)
            .map(&point)
            .collect();
        plot.points(
            Points::new(name, PlotPoints::from(points))
                .color(color)
                .radius(3.0),
        );
    }
}
