//! The playback cursor. Live flights are simulated just far enough ahead of the cursor;
//! recorded flights are only scrubbed. Every view reads the frame at the cursor.

use std::time::{Duration, Instant};

use bevy::prelude::*;
use mission::{Flight, FlightConfig, FlightLog, Frame};

/// Wall-clock time per rendered frame that may be spent simulating ahead.
const SIM_BUDGET: Duration = Duration::from_millis(25);

pub enum Source {
    Live(Box<Flight>),
    Recorded(Box<FlightLog>),
}

#[derive(Resource)]
pub struct Playback {
    pub source: Source,
    /// Flight time shown everywhere, in seconds.
    pub cursor: f64,
    pub playing: bool,
    /// Flight seconds per real second.
    pub speed: f64,
}

impl Playback {
    pub fn live(config: FlightConfig) -> Self {
        Self {
            source: Source::Live(Box::new(Flight::new(config))),
            cursor: 0.0,
            playing: true,
            speed: 1.0,
        }
    }

    pub fn recorded(log: FlightLog) -> Self {
        Self {
            source: Source::Recorded(Box::new(log)),
            cursor: 0.0,
            playing: true,
            speed: 1.0,
        }
    }

    pub fn log(&self) -> &FlightLog {
        match &self.source {
            Source::Live(flight) => flight.log(),
            Source::Recorded(log) => log.as_ref(),
        }
    }

    /// Simulate a live flight until its log covers `t` or the flight ends.
    pub fn simulate_to(&mut self, t: f64) {
        if let Source::Live(flight) = &mut self.source {
            while flight.log().duration() < t && !flight.is_done() {
                flight.tick();
            }
        }
    }

    pub fn is_finished(&self) -> bool {
        match &self.source {
            Source::Live(flight) => flight.is_done(),
            Source::Recorded(_) => true,
        }
    }

    /// The frame at the cursor, blended between its neighbors so motion stays smooth.
    pub fn frame(&self) -> Option<Frame> {
        let (a, b, f) = self.log().frames_around(self.cursor)?;
        let mut frame = a.clone();
        frame.t = a.t + (b.t - a.t) * f;
        frame.pos = a.pos.lerp(b.pos, f);
        frame.vel = a.vel.lerp(b.vel, f);
        frame.att = a.att.slerp(b.att, f);
        frame.est_pos = a.est_pos.lerp(b.est_pos, f);
        frame.thrust_level = a.thrust_level + (b.thrust_level - a.thrust_level) * f;
        frame.gimbal = a.gimbal.lerp(b.gimbal, f);
        frame.fins = a.fins.lerp(b.fins, f);
        frame.fin_deploy = a.fin_deploy + (b.fin_deploy - a.fin_deploy) * f;
        frame.leg_deploy = a.leg_deploy + (b.leg_deploy - a.leg_deploy) * f;
        Some(frame)
    }
}

pub fn advance(time: Res<Time>, mut playback: ResMut<Playback>) {
    if !playback.playing {
        return;
    }
    let target = playback.cursor + time.delta_secs_f64() * playback.speed;
    if let Source::Live(flight) = &mut playback.source {
        let started = Instant::now();
        // The log, not the flight clock, bounds the cursor: the newest frame trails the
        // flight's own time by one tick.
        while flight.log().duration() < target
            && !flight.is_done()
            && started.elapsed() < SIM_BUDGET
        {
            flight.tick();
        }
    }
    let end = playback.log().duration();
    playback.cursor = target.min(end);
    if playback.is_finished() && playback.cursor >= end {
        playback.playing = false;
    }
}
