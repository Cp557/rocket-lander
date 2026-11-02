//! Video export: fly a flight, step the playback cursor by exactly one video frame per
//! rendered frame, screenshot each one, and encode the frames with ffmpeg (MP4 with the
//! sound as heard at the camera, or a silent GIF for a `.gif` output) when the flight or
//! the requested span ends. Then quit.
//!
//!   app --record out.mp4 [landing|full|ship] [--seed N] [--dispersed] [--stress S]
//!       [--camera auto|chase|pad|engine|booster] [--hour H] [--from T] [--to T]
//!       [--fps N] [--speed X] [--size WxH]

use std::path::PathBuf;
use std::process::Command;

use bevy::app::AppExit;
use bevy::prelude::*;
use bevy::render::view::screenshot::{Screenshot, save_to_disk};
use mission::{FlightConfig, Scenario};

use crate::audio::{self, Moment};
use crate::booster::EngineCluster;
use crate::camera::{CameraMode, MainCamera, Orbit};
use crate::overlay::Overlays;
use crate::playback::Playback;
use crate::scene::{TimeOfDay, from_bevy};

/// Wall-clock time to let shaders compile and the model, shadows and sky settle before
/// capturing (s).
const WARMUP: f32 = 8.0;
/// Longest wait, in rendered frames, for the last screenshots to reach the disk.
const FLUSH_FRAMES: u32 = 600;
/// Seconds of footage kept after the flight ends.
const TAIL: f64 = 3.0;
const GIF_FPS: u32 = 12;
const GIF_WIDTH: u32 = 640;

pub struct Options {
    pub out: PathBuf,
    pub config: FlightConfig,
    pub camera: CameraMode,
    pub hour: f32,
    pub from: f64,
    pub to: Option<f64>,
    pub fps: u32,
    pub speed: f64,
    pub size: (u32, u32),
}

/// `Ok(None)` when the arguments do not ask for a recording.
pub fn parse(args: &[String]) -> Result<Option<Options>, String> {
    let Some(at) = args.iter().position(|a| a == "--record") else {
        return Ok(None);
    };
    let out = args.get(at + 1).ok_or("--record needs an output path")?;
    let mut options = Options {
        out: PathBuf::from(out),
        config: FlightConfig {
            scenario: Scenario::FullReturn,
            seed: 1,
            dispersed: false,
            stress: 0.0,
        },
        camera: CameraMode::Auto,
        hour: TimeOfDay::default().hour,
        from: 0.0,
        to: None,
        fps: 30,
        speed: 1.0,
        size: (1280, 720),
    };
    let mut rest = args[at + 2..].iter();
    while let Some(arg) = rest.next() {
        let mut value = || rest.next().ok_or(format!("{arg} needs a value"));
        let number = |text: &String| text.parse::<f64>().map_err(|e| format!("{arg}: {e}"));
        match arg.as_str() {
            "landing" => options.config.scenario = Scenario::Landing,
            "full" => options.config.scenario = Scenario::FullReturn,
            "ship" => options.config.scenario = Scenario::DroneShip,
            "--dispersed" => options.config.dispersed = true,
            "--seed" => options.config.seed = number(value()?)? as u64,
            "--stress" => options.config.stress = number(value()?)?,
            "--hour" => options.hour = number(value()?)? as f32,
            "--from" => options.from = number(value()?)?,
            "--to" => options.to = Some(number(value()?)?),
            "--fps" => options.fps = number(value()?)? as u32,
            "--speed" => options.speed = number(value()?)?,
            "--camera" => {
                let name = value()?;
                options.camera = CameraMode::ALL
                    .into_iter()
                    .find(|m| m.label().eq_ignore_ascii_case(name))
                    .ok_or(format!("unknown camera {name}"))?;
            }
            "--size" => {
                let size = value()?;
                let (w, h) = size.split_once('x').ok_or("--size wants WxH")?;
                let parse = |s: &str| s.parse::<u32>().map_err(|e| format!("--size: {e}"));
                options.size = (parse(w)?, parse(h)?);
            }
            other => return Err(format!("unknown argument {other}")),
        }
    }
    Ok(Some(options))
}

#[derive(Resource)]
pub struct Recording {
    options: Options,
    frames_dir: PathBuf,
    /// Rendered frames so far, including the warmup.
    rendered: u32,
    captured: u32,
    /// Frames rendered since the last capture, once capturing is over.
    flushed: Option<u32>,
    /// Where the camera was at each captured frame, for the soundtrack.
    moments: Vec<Moment>,
}

/// Set the app up to record `options` instead of showing the viewer.
pub fn add(app: &mut App, options: Options) {
    {
        let frames_dir = options.out.with_extension("frames");
        app.insert_resource(Playback {
            playing: false,
            ..Playback::live(options.config)
        })
        .insert_resource(Orbit {
            mode: options.camera,
            ..default()
        })
        .insert_resource(Overlays { show: false })
        .insert_resource(TimeOfDay { hour: options.hour })
        .insert_resource(Recording {
            options,
            frames_dir,
            rendered: 0,
            captured: 0,
            flushed: None,
            moments: Vec::new(),
        })
        .add_systems(Update, step.before(crate::playback::advance));
    }
}

#[allow(clippy::too_many_arguments)] // Bevy system parameters.
fn step(
    mut commands: Commands,
    time: Res<Time<Real>>,
    mut recording: ResMut<Recording>,
    mut playback: ResMut<Playback>,
    orbit: Res<Orbit>,
    camera: Single<&Transform, With<MainCamera>>,
    model: Query<(), With<EngineCluster>>,
    mut exit: MessageWriter<AppExit>,
) {
    let r = &mut *recording;
    r.rendered += 1;
    if let Some(flushed) = &mut r.flushed {
        *flushed += 1;
        // Screenshots are written a few frames after they are taken, later under load.
        let written = std::fs::read_dir(&r.frames_dir).map_or(0, |dir| dir.count());
        if written >= r.captured as usize || *flushed >= FLUSH_FRAMES {
            encode(
                &r.options,
                &r.frames_dir,
                r.captured,
                playback.log(),
                &r.moments,
            );
            exit.write(AppExit::Success);
        }
        return;
    }
    if time.elapsed_secs() < WARMUP || model.is_empty() {
        playback.simulate_to(r.options.from);
        playback.cursor = r.options.from;
        return;
    }
    let o = &r.options;
    let t = o.from + r.captured as f64 / o.fps as f64 * o.speed;
    playback.simulate_to(t);
    let end = match o.to {
        Some(to) => to,
        None if playback.is_finished() => playback.log().duration() + TAIL,
        None => f64::INFINITY,
    };
    if t > end {
        r.flushed = Some(0);
        return;
    }
    playback.cursor = t.min(playback.log().duration());
    if r.captured == 0 {
        std::fs::create_dir_all(&r.frames_dir).expect("create the frames folder");
    }
    r.captured += 1;
    // The camera has not moved for this frame yet; a frame's lag is inaudible.
    r.moments.push(Moment {
        t,
        listener: from_bevy(camera.translation),
        pad_view: orbit.active == CameraMode::Pad,
    });
    let path = r.frames_dir.join(format!("frame_{:05}.png", r.captured));
    commands
        .spawn(Screenshot::primary_window())
        .observe(save_to_disk(path));
}

fn encode(
    options: &Options,
    frames_dir: &std::path::Path,
    frames: u32,
    log: &mission::FlightLog,
    moments: &[Moment],
) {
    let input = frames_dir.join("frame_%05d.png");
    let gif = options.out.extension().is_some_and(|e| e == "gif");
    let mut ffmpeg = Command::new("ffmpeg");
    ffmpeg
        .args(["-y", "-loglevel", "error", "-framerate"])
        .arg(options.fps.to_string())
        .arg("-i")
        .arg(&input);
    let sound = frames_dir.join("sound.wav");
    if !gif && audio::write_wav(&sound, &audio::soundtrack(log, moments, options.fps)).is_ok() {
        ffmpeg
            .arg("-i")
            .arg(&sound)
            .args(["-c:a", "aac", "-b:a", "160k", "-shortest"]);
    }
    if gif {
        ffmpeg.arg("-vf").arg(format!(
            "fps={GIF_FPS},scale={GIF_WIDTH}:-1:flags=lanczos,split[a][b];[a]palettegen=stats_mode=diff[p];[b][p]paletteuse=dither=bayer:bayer_scale=4"
        ));
    } else {
        ffmpeg.args(["-c:v", "libx264", "-pix_fmt", "yuv420p", "-crf", "18"]);
    }
    ffmpeg.arg(&options.out);
    match ffmpeg.status() {
        Ok(status) if status.success() => {
            let _ = std::fs::remove_dir_all(frames_dir);
            println!("recorded {frames} frames to {}", options.out.display());
        }
        Ok(status) => eprintln!(
            "ffmpeg failed ({status}); the frames are in {}",
            frames_dir.display()
        ),
        Err(e) => eprintln!(
            "could not run ffmpeg ({e}); the {frames} frames are in {} (encode them with: ffmpeg -framerate {} -i {} out.mp4)",
            frames_dir.display(),
            options.fps,
            input.display()
        ),
    }
}
