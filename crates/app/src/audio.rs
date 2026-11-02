//! Sound as heard at the camera: engine roar, and sonic booms at the pad camera.
//!
//! Sound travels at `SPEED_OF_SOUND`, so a distant camera hears the engines as they were
//! when the sound left them. A booster falling toward the pad camera faster than sound
//! outruns its own noise: everything it emitted while closing in supersonically arrives
//! at once, when the arrival time of its sound stops falling, and that is the boom.
//!
//! Both sounds are synthesized: the roar is filtered noise generated on the audio thread,
//! muffled with distance through a cutoff the main thread sets; the boom is a triple
//! N-wave (body, legs and engines each shed a shock) with an outdoor echo tail. The same
//! synthesis renders a recorded video's soundtrack offline (`soundtrack`).

use std::f32::consts::TAU;
use std::io::{self, Write};
use std::path::Path;
use std::sync::Arc;
use std::sync::atomic::{AtomicU32, Ordering};

use bevy::audio::{AddAudioSource, AudioSinkPlayback, Decodable, Source, Volume};
use bevy::prelude::*;
use mission::FlightLog;

use crate::camera::{CameraMode, MainCamera, Orbit, PAD_CAMERA};
use crate::playback::Playback;
use crate::scene::from_bevy;

pub const SPEED_OF_SOUND: f32 = 340.0;
const SAMPLE_RATE: u32 = 44_100;
/// Distance inside which the engines sound at full strength (m).
const NEAR_FIELD: f32 = 300.0;
/// Loudness falls as (near field / distance) to this power: gentler than 1/r, so the
/// distant phases stay audible.
const LOUDNESS_FALLOFF: f32 = 0.8;
/// Roar cutoff close by, and the floor that distance muffles it to (Hz).
const NEAR_CUTOFF: f32 = 4000.0;
const FAR_CUTOFF: f32 = 150.0;
const MASTER_VOLUME: f32 = 0.6;
/// Thrust of one engine at sea level, the loudness reference (N).
const REFERENCE_THRUST: f32 = 845e3;
/// Gaps between the three shocks of a boom (s).
const BOOM_SPACING: [f32; 3] = [0.0, 0.32, 0.6];
/// Frames between samples when searching the log for booms.
const BOOM_SEARCH_STRIDE: usize = 5;

/// Whether the viewer makes sound.
#[derive(Resource)]
pub struct Sound {
    pub enabled: bool,
}

impl Default for Sound {
    fn default() -> Self {
        Self { enabled: true }
    }
}

/// Live parameters of the roar, set by the main thread and read on the audio thread.
#[derive(Default)]
struct RoarControls {
    cutoff: AtomicU32,
    crackle: AtomicU32,
}

fn store(cell: &AtomicU32, value: f32) {
    cell.store(value.to_bits(), Ordering::Relaxed);
}

fn load(cell: &AtomicU32) -> f32 {
    f32::from_bits(cell.load(Ordering::Relaxed))
}

/// Endless engine noise.
#[derive(Asset, TypePath)]
struct Roar {
    controls: Arc<RoarControls>,
}

/// The engine noise generator, one sample at a time.
struct RoarSynth {
    seed: u32,
    low: [f32; 2],
    brown: f32,
    pop: f32,
}

impl RoarSynth {
    fn new() -> Self {
        Self {
            seed: 0x9e37_79b9,
            low: [0.0; 2],
            brown: 0.0,
            pop: 0.0,
        }
    }

    fn white(&mut self) -> f32 {
        // xorshift32
        self.seed ^= self.seed << 13;
        self.seed ^= self.seed >> 17;
        self.seed ^= self.seed << 5;
        self.seed as f32 / u32::MAX as f32 * 2.0 - 1.0
    }

    fn sample(&mut self, cutoff: f32, crackle: f32) -> f32 {
        let cutoff = cutoff.max(FAR_CUTOFF);
        let a = 1.0 - (-TAU * cutoff / SAMPLE_RATE as f32).exp();
        let white = self.white();
        // Two one-pole low-passes for the roar, scaled so its loudness does not depend on
        // the cutoff, over brown noise for the rumble.
        self.low[0] += a * (white - self.low[0]);
        self.low[1] += a * (self.low[0] - self.low[1]);
        let roar = self.low[1] * (3.0 * (2.0 - a) / a).sqrt() * 0.5;
        self.brown = (self.brown + 0.02 * white) / 1.02;
        // Crackle: sparse sharp pops, only heard close by.
        if self.white() > 1.0 - 2.0 * 40.0 / SAMPLE_RATE as f32 {
            self.pop = self.white();
        }
        self.pop *= 0.97;
        0.4 * roar + 3.0 * self.brown + 0.6 * crackle * self.pop
    }
}

struct RoarDecoder {
    controls: Arc<RoarControls>,
    synth: RoarSynth,
}

impl Iterator for RoarDecoder {
    type Item = f32;

    fn next(&mut self) -> Option<f32> {
        let (cutoff, crackle) = (load(&self.controls.cutoff), load(&self.controls.crackle));
        Some(self.synth.sample(cutoff, crackle))
    }
}

impl Source for RoarDecoder {
    fn current_span_len(&self) -> Option<usize> {
        None
    }

    fn channels(&self) -> bevy::audio::ChannelCount {
        bevy::audio::ChannelCount::MIN
    }

    fn sample_rate(&self) -> bevy::audio::SampleRate {
        bevy::audio::SampleRate::new(SAMPLE_RATE).expect("nonzero")
    }

    fn total_duration(&self) -> Option<std::time::Duration> {
        None
    }
}

impl Decodable for Roar {
    type Decoder = RoarDecoder;

    fn decoder(&self) -> RoarDecoder {
        RoarDecoder {
            controls: self.controls.clone(),
            synth: RoarSynth::new(),
        }
    }
}

/// A prerendered clip.
#[derive(Asset, TypePath)]
struct Clip {
    samples: Arc<[f32]>,
}

struct ClipDecoder {
    samples: Arc<[f32]>,
    at: usize,
}

impl Iterator for ClipDecoder {
    type Item = f32;

    fn next(&mut self) -> Option<f32> {
        let sample = self.samples.get(self.at).copied();
        self.at += 1;
        sample
    }
}

impl Source for ClipDecoder {
    fn current_span_len(&self) -> Option<usize> {
        Some(self.samples.len().saturating_sub(self.at))
    }

    fn channels(&self) -> bevy::audio::ChannelCount {
        bevy::audio::ChannelCount::MIN
    }

    fn sample_rate(&self) -> bevy::audio::SampleRate {
        bevy::audio::SampleRate::new(SAMPLE_RATE).expect("nonzero")
    }

    fn total_duration(&self) -> Option<std::time::Duration> {
        Some(std::time::Duration::from_secs_f32(
            self.samples.len() as f32 / SAMPLE_RATE as f32,
        ))
    }
}

impl Decodable for Clip {
    type Decoder = ClipDecoder;

    fn decoder(&self) -> ClipDecoder {
        ClipDecoder {
            samples: self.samples.clone(),
            at: 0,
        }
    }
}

/// Three N-waves (a sharp rise, a linear fall through zero, a sharp return) with a
/// decaying rumble of echoes off the ground and buildings.
fn boom_samples() -> Vec<f32> {
    const N_WAVE: f32 = 0.12;
    const TAIL: f32 = 2.5;
    let rate = SAMPLE_RATE as f32;
    let length = ((BOOM_SPACING[2] + N_WAVE + TAIL) * rate) as usize;
    let mut seed = 12345u32;
    let mut white = || {
        seed ^= seed << 13;
        seed ^= seed >> 17;
        seed ^= seed << 5;
        seed as f32 / u32::MAX as f32 * 2.0 - 1.0
    };
    let (mut low, mut echo) = (0.0, 0.0);
    (0..length)
        .map(|i| {
            let t = i as f32 / rate;
            let wave: f32 = BOOM_SPACING
                .iter()
                .zip([1.0, 0.8, 0.65])
                .map(|(&start, gain)| {
                    let s = (t - start) / N_WAVE;
                    if (0.0..1.0).contains(&s) {
                        gain * (1.0 - 2.0 * s)
                    } else {
                        0.0
                    }
                })
                .sum();
            // Distance rounds off the shock fronts.
            low += 0.25 * (wave - low);
            echo += 0.01 * (white() - echo);
            let tail = 4.0 * echo * (-(t - BOOM_SPACING[0]).max(0.0) / 0.7).exp();
            (0.9 * low + 0.3 * tail).clamp(-1.0, 1.0)
        })
        .collect()
}

#[derive(Component)]
struct EngineSound;

#[derive(Resource)]
struct AudioAssets {
    controls: Arc<RoarControls>,
    boom: Handle<Clip>,
}

pub fn plugin(app: &mut App) {
    app.add_audio_source::<Roar>()
        .add_audio_source::<Clip>()
        .init_resource::<Sound>()
        .add_systems(Startup, setup)
        .add_systems(Update, (roar, booms).after(crate::camera::follow));
}

fn setup(mut commands: Commands, mut roars: ResMut<Assets<Roar>>, mut clips: ResMut<Assets<Clip>>) {
    let controls = Arc::new(RoarControls::default());
    commands.spawn((
        EngineSound,
        AudioPlayer(roars.add(Roar {
            controls: controls.clone(),
        })),
        PlaybackSettings::LOOP.with_volume(Volume::SILENT),
    ));
    commands.insert_resource(AudioAssets {
        controls,
        boom: clips.add(Clip {
            samples: boom_samples().into(),
        }),
    });
}

/// When the sound now reaching `listener` (sim frame) at `t` left the booster: the latest
/// emission time whose arrival, t_emit + distance(t_emit) / c, is not after `t`. A
/// supersonic dive makes arrivals run backwards, so step back from `t` until one has
/// arrived, then bisect. `None` before any sound has arrived.
fn emission_time(log: &FlightLog, listener: Vec3, t: f64) -> Option<f64> {
    const STEP_BACK: f64 = 0.5;
    let arrival = |emitted: f64| -> Option<f64> {
        let (a, b, f) = log.frames_around(emitted)?;
        Some(emitted + (listener.distance(a.pos.lerp(b.pos, f)) / SPEED_OF_SOUND) as f64)
    };
    let (mut lo, mut hi) = (t, t);
    loop {
        lo = (lo - STEP_BACK).max(0.0);
        if arrival(lo)? <= t {
            break;
        }
        if lo == 0.0 {
            return None;
        }
        hi = lo;
    }
    for _ in 0..30 {
        let mid = 0.5 * (lo + hi);
        if arrival(mid)? <= t {
            lo = mid;
        } else {
            hi = mid;
        }
    }
    Some(lo)
}

/// How the roar sounds at a listener: loudness (0 when nothing is heard), cutoff and
/// crackle.
#[derive(Clone, Copy, Debug, Default)]
struct Heard {
    volume: f32,
    cutoff: f32,
    crackle: f32,
}

/// The roar at `listener` (sim frame) at flight time `t`, from the engines as they were
/// when the sound left them.
fn heard(log: &FlightLog, listener: Vec3, t: f64) -> Heard {
    let Some((a, b, f)) = emission_time(log, listener, t).and_then(|t| log.frames_around(t)) else {
        return Heard::default();
    };
    let (pos, thrust) = (a.pos.lerp(b.pos, f), a.thrust + (b.thrust - a.thrust) * f);
    if thrust <= 0.0 {
        return Heard::default();
    }
    let closeness = (NEAR_FIELD / listener.distance(pos).max(NEAR_FIELD)).min(1.0);
    Heard {
        volume: (thrust / REFERENCE_THRUST).sqrt().min(1.5) * closeness.powf(LOUDNESS_FALLOFF),
        cutoff: FAR_CUTOFF + NEAR_CUTOFF * closeness.powf(0.6),
        crackle: closeness,
    }
}

fn roar(
    playback: Res<Playback>,
    sound: Res<Sound>,
    assets: Res<AudioAssets>,
    camera: Single<&Transform, With<MainCamera>>,
    mut sink: Single<&mut AudioSink, With<EngineSound>>,
) {
    let h = heard(
        playback.log(),
        from_bevy(camera.translation),
        playback.cursor,
    );
    store(&assets.controls.cutoff, h.cutoff);
    store(&assets.controls.crackle, h.crackle);
    let on = sound.enabled && playback.playing;
    sink.set_volume(Volume::Linear(if on {
        MASTER_VOLUME * h.volume
    } else {
        0.0
    }));
}

/// One video frame's view, for its soundtrack: flight time, where the camera is (sim
/// frame) and whether it is the pad camera.
#[derive(Clone, Copy, Debug)]
pub struct Moment {
    pub t: f64,
    pub listener: Vec3,
    pub pad_view: bool,
}

/// The sound of a recorded video at `fps` frames per second, one `Moment` per frame: the
/// roar with its parameters blended across each frame, and booms as their arrivals pass
/// while the pad camera is the view.
pub fn soundtrack(log: &FlightLog, moments: &[Moment], fps: u32) -> Vec<f32> {
    let per_frame = (SAMPLE_RATE / fps) as usize;
    let mut out = vec![0.0f32; moments.len() * per_frame];
    let mut synth = RoarSynth::new();
    let sounds: Vec<Heard> = moments
        .iter()
        .map(|m| heard(log, m.listener, m.t))
        .collect();
    for (k, pair) in sounds.windows(2).enumerate() {
        for i in 0..per_frame {
            let f = i as f32 / per_frame as f32;
            let blend = |a: f32, b: f32| a + (b - a) * f;
            let (a, b) = (pair[0], pair[1]);
            let volume = blend(a.volume, b.volume);
            let sample = synth.sample(blend(a.cutoff, b.cutoff), blend(a.crackle, b.crackle));
            out[k * per_frame + i] = MASTER_VOLUME * volume * sample;
        }
    }
    let boom = boom_samples();
    let arrivals = boom_arrivals(log, from_bevy(PAD_CAMERA));
    for (k, pair) in moments.windows(2).enumerate() {
        let passed = arrivals.iter().any(|&t| pair[0].t < t && t <= pair[1].t);
        if passed && pair[1].pad_view {
            let start = ((k + 1) * per_frame).min(out.len());
            for (o, b) in out[start..].iter_mut().zip(&boom) {
                *o += MASTER_VOLUME * b;
            }
        }
    }
    for o in &mut out {
        *o = o.clamp(-1.0, 1.0);
    }
    out
}

/// Write mono samples as a 16-bit PCM WAV file.
pub fn write_wav(path: &Path, samples: &[f32]) -> io::Result<()> {
    let data = (samples.len() * 2) as u32;
    let mut bytes = Vec::with_capacity(44 + data as usize);
    bytes.extend_from_slice(b"RIFF");
    bytes.extend_from_slice(&(36 + data).to_le_bytes());
    bytes.extend_from_slice(b"WAVEfmt ");
    bytes.extend_from_slice(&16u32.to_le_bytes());
    bytes.extend_from_slice(&1u16.to_le_bytes()); // PCM
    bytes.extend_from_slice(&1u16.to_le_bytes()); // mono
    bytes.extend_from_slice(&SAMPLE_RATE.to_le_bytes());
    bytes.extend_from_slice(&(SAMPLE_RATE * 2).to_le_bytes());
    bytes.extend_from_slice(&2u16.to_le_bytes());
    bytes.extend_from_slice(&16u16.to_le_bytes());
    bytes.extend_from_slice(b"data");
    bytes.extend_from_slice(&data.to_le_bytes());
    for s in samples {
        bytes.extend_from_slice(&((s * i16::MAX as f32) as i16).to_le_bytes());
    }
    std::fs::File::create(path)?.write_all(&bytes)
}

/// Times at which booms reach `listener`: the local minima of the sound's arrival time
/// t + distance / c over the flight so far.
fn boom_arrivals(log: &FlightLog, listener: Vec3) -> Vec<f64> {
    let arrival: Vec<f64> = log
        .frames
        .iter()
        .step_by(BOOM_SEARCH_STRIDE)
        .map(|f| f.t as f64 + (listener.distance(f.pos) / SPEED_OF_SOUND) as f64)
        .collect();
    arrival
        .windows(3)
        .filter(|w| w[1] < w[0] && w[1] <= w[2])
        .map(|w| w[1])
        .collect()
}

/// Play a boom whenever the cursor passes one's arrival at the pad camera, while that is
/// the view.
fn booms(
    mut commands: Commands,
    playback: Res<Playback>,
    sound: Res<Sound>,
    orbit: Res<Orbit>,
    assets: Res<AudioAssets>,
    mut last_cursor: Local<f64>,
) {
    let (from, to) = (*last_cursor, playback.cursor);
    *last_cursor = to;
    if !sound.enabled || !playback.playing || orbit.active != CameraMode::Pad || to <= from {
        return;
    }
    let listener = from_bevy(PAD_CAMERA);
    if boom_arrivals(playback.log(), listener)
        .iter()
        .any(|&t| from < t && t <= to)
    {
        commands.spawn((
            AudioPlayer(assets.boom.clone()),
            PlaybackSettings::DESPAWN.with_volume(Volume::Linear(MASTER_VOLUME)),
        ));
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::scene::to_bevy;
    use mission::{FlightConfig, Frame, Scenario};

    /// A log whose booster follows `path` (sim frame) for `seconds`.
    fn log_of(path: impl Fn(f32) -> Vec3, seconds: f32) -> FlightLog {
        let mut flight = mission::Flight::new(FlightConfig {
            scenario: Scenario::Landing,
            seed: 1,
            dispersed: false,
            stress: 0.0,
        });
        flight.tick();
        let mut log = flight.log().clone();
        let blank = log.frames[0].clone();
        log.frames = (0..(seconds * 50.0) as usize)
            .map(|i| {
                let t = i as f32 * 0.02;
                Frame {
                    t,
                    pos: path(t),
                    ..blank.clone()
                }
            })
            .collect();
        log
    }

    #[test]
    fn a_supersonic_dive_toward_the_listener_booms_once() {
        // Straight down at 600 m/s from 30 km, braking to 50 m/s over the last 20 km.
        let height = |t: f32| {
            let fast = 10_000.0 / 600.0;
            if t < fast {
                30_000.0 - 600.0 * t
            } else {
                20_000.0 - 50.0 * (t - fast)
            }
        };
        let log = log_of(|t| Vec3::new(0.0, 0.0, height(t).max(0.0)), 120.0);
        let booms = boom_arrivals(&log, Vec3::ZERO);
        assert_eq!(booms.len(), 1, "{booms:?}");
        // The boom carries the sound from where the dive turned subsonic, 20 km up.
        let expected = 10_000.0 / 600.0 + 20_000.0 / SPEED_OF_SOUND;
        assert!(
            (booms[0] - expected as f64).abs() < 0.5,
            "{booms:?} vs {expected}"
        );
    }

    #[test]
    fn subsonic_flight_never_booms() {
        let log = log_of(|t| Vec3::new(0.0, 0.0, 5000.0 - 200.0 * t), 20.0);
        assert!(boom_arrivals(&log, Vec3::ZERO).is_empty());
        let heard = emission_time(&log, Vec3::ZERO, 18.0).expect("sound has arrived");
        // Sound left at t_e from 5000 - 200 t_e meters away: t_e + d / c = 18.
        let expected = (18.0 - 5000.0 / SPEED_OF_SOUND) / (1.0 - 200.0 / SPEED_OF_SOUND);
        assert!(
            (heard - expected as f64).abs() < 0.05,
            "{heard} vs {expected}"
        );
    }

    #[test]
    fn boom_clip_stays_in_range() {
        let samples = boom_samples();
        assert!(samples.iter().all(|s| s.abs() <= 1.0));
        assert!(samples.iter().any(|s| s.abs() > 0.5));
    }

    #[test]
    fn soundtrack_has_the_engines_and_the_boom() {
        // A booster diving supersonically at the pad camera with its engine running.
        let pad = from_bevy(PAD_CAMERA);
        let mut log = log_of(
            |t| pad + Vec3::new(0.0, 0.0, (30_000.0 - 600.0 * t).max(2_000.0)),
            70.0,
        );
        for f in &mut log.frames {
            f.thrust = 845e3;
        }
        let fps = 10;
        let moments: Vec<Moment> = (0..700)
            .map(|k| Moment {
                t: k as f64 / fps as f64,
                listener: pad,
                pad_view: true,
            })
            .collect();
        let audio = soundtrack(&log, &moments, fps);
        assert_eq!(audio.len(), 700 * (SAMPLE_RATE / fps) as usize);
        let loudest = |from: f64, to: f64| {
            let at = |t: f64| (t * SAMPLE_RATE as f64) as usize;
            audio[at(from)..at(to)]
                .iter()
                .fold(0.0f32, |m, s| m.max(s.abs()))
        };
        // The dive outruns its own sound: nothing is heard until the boom, when it slows
        // to a hover at 2 km, and then the roar of the hover.
        let boom = 28_000.0 / 600.0 + 2_000.0 / SPEED_OF_SOUND as f64;
        assert_eq!(loudest(0.0, boom - 1.0), 0.0);
        assert!(loudest(boom + 3.0, boom + 8.0) > 0.01);
        assert!(loudest(boom - 0.1, boom + 0.6) > loudest(boom + 3.0, boom + 8.0));
    }

    #[test]
    fn listener_conversion_round_trips() {
        let v = Vec3::new(1.0, 2.0, 3.0);
        assert_eq!(from_bevy(to_bevy(v)), v);
    }
}
