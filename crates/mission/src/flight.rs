//! One closed-loop flight: sensors -> flight computer -> actuators -> physics.

use fsw::{FlightComputer, Phase};
use glam::{DVec2, DVec3};
use rand::rngs::StdRng;
use rand::{RngExt, SeedableRng};
use serde::{Deserialize, Serialize};
use sim::{Actuators, Commands, Dispersion, Ship, Vehicle, World};

use crate::log::{Event, FlightLog, Frame, PlanRecord};
use crate::scenario::Scenario;

pub const PHYSICS_DT: f64 = 0.002;
pub const STEPS_PER_TICK: usize = 10;
/// How long to watch the vehicle after touchdown before calling it landed.
const SETTLE_TIME: f64 = 5.0;
const TIME_LIMIT: f64 = 900.0;

pub const MAX_VERTICAL_SPEED: f64 = 4.0;
pub const MAX_HORIZONTAL_SPEED: f64 = 2.0;
pub const MAX_TILT_DEG: f64 = 5.0;
pub const MAX_TILT_RATE_DEG: f64 = 10.0;
pub const PAD_RADIUS: f64 = 15.0;

#[derive(Clone, Copy, Debug, PartialEq, Serialize, Deserialize)]
pub struct FlightConfig {
    pub scenario: Scenario,
    pub seed: u64,
    /// Randomize the true vehicle, wind and starting state (Monte Carlo).
    pub dispersed: bool,
    /// With `dispersed`, how much harsher weather and how many faults to add
    /// (`Dispersion::stressed`; 0 is the standard dispersion).
    #[serde(default)]
    pub stress: f64,
}

/// Seed offset for the stress draws, so that adding stress leaves a seed's standard
/// dispersion (and its sensor noise) unchanged.
const STRESS_STREAM: u64 = 0x9e37_79b9_7f4a_7c15;
/// Seed offset for the sea state, kept apart for the same reason.
const SEA_STREAM: u64 = 0x5851_f42d_4c95_7f2d;

/// True state at first ground contact.
#[derive(Clone, Copy, Debug, PartialEq, Serialize, Deserialize)]
pub struct Touchdown {
    pub t: f64,
    pub position: DVec2,
    pub vertical_speed: f64,
    pub horizontal_speed: f64,
    pub tilt_deg: f64,
    pub tilt_rate_deg: f64,
    pub propellant: f64,
}

impl Touchdown {
    pub fn miss(&self) -> f64 {
        self.position.length()
    }

    /// The first landing limit this touchdown breaks, if any.
    fn violation(&self) -> Option<String> {
        let checks = [
            (
                self.vertical_speed > MAX_VERTICAL_SPEED,
                format!("hard touchdown ({:.1} m/s)", self.vertical_speed),
            ),
            (
                self.horizontal_speed > MAX_HORIZONTAL_SPEED,
                format!("sliding ({:.1} m/s)", self.horizontal_speed),
            ),
            (
                self.tilt_deg > MAX_TILT_DEG,
                format!("tilted {:.1} deg", self.tilt_deg),
            ),
            (
                self.tilt_rate_deg > MAX_TILT_RATE_DEG,
                format!("rotating {:.1} deg/s", self.tilt_rate_deg),
            ),
            (
                self.miss() > PAD_RADIUS,
                format!("missed the pad by {:.0} m", self.miss()),
            ),
        ];
        checks
            .into_iter()
            .find_map(|(failed, why)| failed.then_some(why))
    }
}

#[derive(Clone, Debug, PartialEq, Serialize, Deserialize)]
pub struct Outcome {
    pub landed: bool,
    pub verdict: String,
    pub touchdown: Option<Touchdown>,
}

pub struct Flight {
    world: World,
    fsw: FlightComputer,
    log: FlightLog,
    last_phase: Phase,
    last_plan_revision: u32,
    fins_announced: bool,
    legs_announced: bool,
    engine_out_announced: bool,
    gps_out: bool,
    bias_jump_announced: bool,
    gas_out_announced: bool,
    touchdown: Option<Touchdown>,
}

impl Flight {
    pub fn new(config: FlightConfig) -> Self {
        let mut rng = StdRng::seed_from_u64(config.seed);
        let dispersion = if config.dispersed {
            let mut stress_rng = StdRng::seed_from_u64(config.seed ^ STRESS_STREAM);
            Dispersion::sample(&mut rng).stressed(config.stress, &mut stress_rng)
        } else {
            Dispersion::none()
        };
        let nominal = Vehicle::nominal();
        let start = config.scenario.start(&nominal);

        let mut truth = start.state;
        truth.pos += dispersion.position_error;
        truth.vel += dispersion.velocity_error;
        truth.mass += nominal.dry_mass * (dispersion.dry_mass_scale - 1.0);
        let actuators = Actuators {
            fin_deploy: if start.fins_deployed { 1.0 } else { 0.0 },
            ..Default::default()
        };
        let mut world = World::new(
            dispersion.apply(&nominal),
            truth,
            actuators,
            dispersion.wind,
            dispersion.thrust_misalignment,
            dispersion.faults,
            rng.random(),
        );
        world.ship = config.scenario.at_sea().then(|| {
            if config.dispersed {
                Ship::sample(
                    &mut StdRng::seed_from_u64(config.seed ^ SEA_STREAM),
                    config.stress,
                )
            } else {
                Ship::nominal()
            }
        });

        let fsw = FlightComputer::new(nominal, &start.state, start.phase, start.fins_deployed);
        let mut flight = Self {
            log: FlightLog {
                config,
                frames: Vec::new(),
                events: Vec::new(),
                plans: Vec::new(),
                outcome: None,
                ship: world.ship,
            },
            world,
            fsw,
            last_phase: start.phase,
            last_plan_revision: 0,
            fins_announced: start.fins_deployed,
            legs_announced: false,
            engine_out_announced: false,
            gps_out: false,
            bias_jump_announced: false,
            gas_out_announced: false,
            touchdown: None,
        };
        flight.event(format!("Start: {}", start.phase.label()));
        flight
    }

    pub fn log(&self) -> &FlightLog {
        &self.log
    }

    pub fn is_done(&self) -> bool {
        self.log.outcome.is_some()
    }

    /// Advance one flight-computer period.
    pub fn tick(&mut self) {
        if self.is_done() {
            return;
        }
        let sensors = self.world.sense();
        let cmd = self.fsw.step(&sensors);
        // Record truth and estimate for the same instant, before the world moves on.
        self.record(&cmd);
        self.announce();
        for _ in 0..STEPS_PER_TICK {
            self.world.step(&cmd, PHYSICS_DT);
        }
        self.judge();
    }

    pub fn run_to_end(mut self) -> FlightLog {
        while !self.is_done() {
            self.tick();
        }
        self.log
    }

    fn event(&mut self, text: String) {
        self.log.events.push(Event {
            t: self.world.t,
            text,
        });
    }

    fn record(&mut self, cmd: &Commands) {
        let (s, a, info) = (&self.world.state, &self.world.act, &self.world.info);
        let nav = self.fsw.nav();
        let impact = matches!(
            self.fsw.phase(),
            Phase::Flip | Phase::Boostback | Phase::Coast | Phase::EntryBurn | Phase::AeroDescent
        )
        .then(|| self.fsw.impact().map(|i| i.point.as_vec2()))
        .flatten();
        self.log.frames.push(Frame {
            t: self.world.t as f32,
            phase: self.fsw.phase(),
            pos: s.pos.as_vec3(),
            vel: s.vel.as_vec3(),
            att: s.att.as_quat(),
            mass: s.mass as f32,
            est_pos: nav.pos.as_vec3(),
            est_mass: self.fsw.mass() as f32,
            pos_sigma: nav.pos_sigma as f32,
            est_wind: self.fsw.wind().as_vec3(),
            engines: a.engines,
            thrust_level: a.thrust_level as f32,
            throttle_cmd: cmd.throttle as f32,
            thrust: info.thrust as f32,
            gimbal: a.gimbal.as_vec2(),
            fins: a.fins.as_vec3(),
            rcs: a.rcs.as_vec3(),
            fin_deploy: a.fin_deploy as f32,
            leg_deploy: a.leg_deploy as f32,
            dynamic_pressure: info.dynamic_pressure as f32,
            alpha: info.alpha as f32,
            wind: info.wind.as_vec3(),
            impact,
            rcs_gas: self.world.rcs_gas as f32,
        });
    }

    fn announce(&mut self) {
        let phase = self.fsw.phase();
        if phase != self.last_phase {
            self.last_phase = phase;
            self.event(phase.label().to_string());
        }
        if self.fsw.plan_revision() != self.last_plan_revision {
            self.last_plan_revision = self.fsw.plan_revision();
            let plan = self.fsw.plan().expect("a revision implies a plan");
            let points = plan.pos.iter().map(|p| p.as_vec3()).collect();
            self.log.plans.push(PlanRecord {
                t: self.world.t,
                points,
            });
        }
        if !self.fins_announced && self.world.act.fin_deploy >= 1.0 {
            self.fins_announced = true;
            self.event("Grid fins deployed".into());
        }
        if !self.legs_announced && self.world.act.leg_deploy >= 1.0 {
            self.legs_announced = true;
            self.event("Legs deployed".into());
        }
        if !self.engine_out_announced && self.world.engine_failed() {
            self.engine_out_announced = true;
            self.event("Engine out".into());
        }
        let sensors = self.world.sensors();
        if sensors.gps_out != self.gps_out {
            self.gps_out = sensors.gps_out;
            let text = if self.gps_out { "GPS lost" } else { "GPS back" };
            self.event(text.into());
        }
        if !self.gas_out_announced && self.world.rcs_gas <= 0.0 {
            self.gas_out_announced = true;
            self.event("Cold gas out".into());
        }
        if !self.bias_jump_announced && self.world.sensors().bias_jumped {
            self.bias_jump_announced = true;
            self.event("Accelerometer bias jump".into());
        }
    }

    fn judge(&mut self) {
        let (s, info) = (self.world.state, self.world.info);
        let touching = info.legs_touching > 0 || info.hull_touching > 0;
        if self.touchdown.is_none() && touching {
            // On the drone ship, position, speeds and tilt count relative to the deck: its
            // surface under the booster, and its normal.
            let (center, deck_vel, up) =
                self.world
                    .ship
                    .map_or((DVec3::ZERO, DVec3::ZERO, DVec3::Z), |ship| {
                        let pose = ship.pose(self.world.t);
                        let local = pose.att.inverse() * (s.pos - pose.origin);
                        let below = pose.origin + pose.att * local.truncate().extend(0.0);
                        (pose.origin, pose.velocity_at(below), pose.att * DVec3::Z)
                    });
            let relative = s.vel - deck_vel;
            let td = Touchdown {
                t: self.world.t,
                position: (s.pos - center).truncate(),
                vertical_speed: -relative.dot(up),
                horizontal_speed: (relative - up * relative.dot(up)).length(),
                tilt_deg: s.axis().angle_between(up).to_degrees(),
                tilt_rate_deg: s.omega.truncate().length().to_degrees(),
                propellant: s.mass - self.world.vehicle.dry_mass,
            };
            self.touchdown = Some(td);
            self.event(format!(
                "Touchdown at {:.1} m/s, {:.1} m from pad center",
                td.vertical_speed,
                td.miss()
            ));
        }

        let hull_contact = info.hull_touching > 0;
        let verdict = match self.touchdown {
            _ if info.in_water > 0 => Some((false, "crashed: fell into the sea".to_string())),
            Some(_) if hull_contact => Some((false, "crashed: hull hit the ground".to_string())),
            Some(td) if self.world.t - td.t >= SETTLE_TIME => Some(
                td.violation()
                    .map_or((true, "landed".to_string()), |why| (false, why)),
            ),
            None if self.world.t > TIME_LIMIT => Some((false, "timed out".to_string())),
            _ => None,
        };
        if let Some((landed, verdict)) = verdict {
            self.event(format!("Outcome: {verdict}"));
            self.log.outcome = Some(Outcome {
                landed,
                verdict,
                touchdown: self.touchdown,
            });
        }
    }
}
