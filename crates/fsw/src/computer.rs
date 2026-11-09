//! The flight computer: one `step` per sensor frame runs navigation, the phase state
//! machine (guidance) and attitude control, and returns actuator commands.

use glam::{DVec2, DVec3};
use serde::{Deserialize, Serialize};
use sim::env::{density, gravity, pressure};
use sim::sensors::DECK_DELAY;
use sim::{Commands, SensorFrame, State, Vehicle};

use crate::control::{self, CONTROL_PERIOD, Effectors, double_integrator_lqr};
use crate::deck::{DeckPredictor, DeckState};
use crate::gfold::{self, Plan};
use crate::guidance;
use crate::nav::{NavState, Navigator};
use crate::predict::{
    self, ENTRY_BURN_END_SPEED, ENTRY_BURN_ENGINES, ENTRY_BURN_START_ALTITUDE, EntryBurn, Impact,
};
use crate::terminal;

#[derive(Clone, Copy, Debug, PartialEq, Eq, Serialize, Deserialize)]
pub enum Phase {
    Flip,
    Boostback,
    Coast,
    EntryBurn,
    AeroDescent,
    LandingBurn,
    Landed,
}

impl Phase {
    pub fn label(self) -> &'static str {
        match self {
            Phase::Flip => "Flip",
            Phase::Boostback => "Boostback",
            Phase::Coast => "Coast",
            Phase::EntryBurn => "Entry burn",
            Phase::AeroDescent => "Aero descent",
            Phase::LandingBurn => "Landing burn",
            Phase::Landed => "Landed",
        }
    }
}

const FLIP_ALIGNMENT: f64 = 0.0873; // 5 degrees
const FLIP_RATE: f64 = 0.0175; // 1 degree/s
/// Below this predicted miss, boostback trims with one engine at minimum throttle.
const BOOSTBACK_TRIM_MISS: f64 = 3000.0;
const BOOSTBACK_CUTOFF_MISS: f64 = 50.0;
const LEG_DEPLOY_ALTITUDE: f64 = 500.0;
/// Cut the engine this close above leg-contact height. One engine at minimum throttle
/// still outweighs a near-empty booster, so if it slows to a stop above the ground it
/// climbs away instead of settling; dropping the last half meter is gentle.
const TERMINAL_CUTOFF_HEIGHT: f64 = 0.5;
/// Rising again this close to the ground means the burn stopped short. One engine at
/// minimum throttle would carry the booster away until the propellant ran out, so cut it
/// and drop the last meters instead.
const CLIMB_CUTOFF_HEIGHT: f64 = 5.0;
/// If terminal guidance fails to solve, just null the sideways velocity at this rate (/s).
const VELOCITY_ONLY_GAIN: f64 = 0.8;
/// Ticks between terminal guidance solves. Its plan holds each tilt command for at least
/// 0.1 s (five ticks), so the planned tilt is held in between.
const TERMINAL_EVERY: u64 = 5;
const REPLAN_PERIOD: f64 = 0.5;
/// Stop replanning this close to the end of the plan; the last seconds are tracked as-is.
const REPLAN_CUTOFF: f64 = 2.0;
/// While coasting over the top, let the attitude drift this far from its target (rad) and
/// turn this slowly (rad/s) before spending cold gas on it.
const COAST_DEADBAND: f64 = 0.0873; // 5 degrees
const COAST_DEADBAND_RATE: f64 = 0.0087; // 0.5 degrees/s
/// Turn rates on cold gas alone (rad/s): the gas a turn costs grows with how fast it
/// spins up. The flip is pressed for time; the coast has minutes.
const FLIP_TURN_RATE: f64 = 0.0873; // 5 degrees/s
const COAST_TURN_RATE: f64 = 0.0262; // 1.5 degrees/s
/// Ticks between impact predictions outside boostback.
const PREDICT_EVERY: u64 = 5;
/// If no plan is found by this fraction of the ignition altitude, ignite on the fallback law.
const FORCED_IGNITION: f64 = 0.8;
/// While no landing plan is feasible yet, look again this often (s). A heavy booster
/// falling fast through thin air can reach its estimated ignition altitude long before
/// drag has slowed it enough for a plan.
const IGNITION_RETRY: f64 = 0.2;
/// At sea, below this height above the deck the touchdown is timed for a calm moment: the
/// deck level and not rising, as predicted from its reports.
const CALM_HEIGHT: f64 = 40.0;
/// Scales of a bad moment: deck tilt (rad) and the deck rising (m/s).
const CALM_TILT: f64 = 0.02;
const CALM_RISE: f64 = 0.3;
/// Speed toward the deck at touchdown.
const DECK_TOUCHDOWN_SPEED: f64 = 1.5;
/// Planning bounds for the timed approach: times up to three times a steady approach are
/// tried, keeping the acceleration this far inside what the engine can do (m/s^2), up to
/// this throttle.
const CALM_CANDIDATES: usize = 90;
const CALM_ACCEL_MARGIN: f64 = 1.0;
const PLAN_THROTTLE: f64 = 0.85;
/// Over the last fraction of a second, just hold the touchdown speed relative to the deck.
const DECK_FINAL_TIME: f64 = 0.3;
const DECK_MATCH_GAIN: f64 = 2.0;
const MIN_FIN_PRESSURE: f64 = 100.0;
/// Below this dynamic pressure the side force is too weak to read the crossflow from.
const CROSSFLOW_MIN_PRESSURE: f64 = 500.0;
/// Largest angle of attack inferred from a side force.
const CROSSFLOW_MAX_ALPHA: f64 = std::f64::consts::FRAC_PI_6;
/// Unmodeled sideways specific force (accelerometer noise, thrust misalignment, aero and
/// fin model errors) when reading the crossflow, m/s^2.
const CROSSFLOW_FORCE_NOISE: f64 = 0.3;
/// Floor on the crossflow measurement noise, m/s.
const CROSSFLOW_MIN_NOISE: f64 = 0.3;

/// What guidance wants this tick; attitude control turns `axis` into actuator commands.
struct Intent {
    engines: u8,
    throttle: f64,
    /// Desired nose direction, world frame.
    axis: DVec3,
}

impl Intent {
    fn unpowered(axis: DVec3) -> Self {
        Self {
            engines: 0,
            throttle: 0.0,
            axis,
        }
    }
}

pub struct FlightComputer {
    vehicle: Vehicle,
    nav: Navigator,
    phase: Phase,
    t: f64,
    ticks: u64,
    mass: f64,
    /// Modeled engine output in full-throttle engines (the engines lag the commands).
    thrust_level: f64,
    /// Modeled gimbal and grid fin positions (both slew at a limited rate).
    gimbal: DVec2,
    fins: DVec3,
    pad: DVec3,
    fins_deployed_at: Option<f64>,
    deploy_legs: bool,
    boostback_heading: DVec3,
    impact: Option<Impact>,
    plan: Option<Plan>,
    plan_revision: u32,
    last_replan: f64,
    last_cmd: Commands,
    accel_cmd: DVec3,
    /// Engine health monitoring has reported a failed engine in the three-engine group.
    engine_out: bool,
    /// The drone ship's deck motion, learned from its reports.
    deck: Option<DeckPredictor>,
    /// When to touch down on the deck, once chosen.
    touchdown_at: Option<f64>,
    /// The tilt terminal guidance last planned, held between solves.
    held_tilt: Option<DVec2>,
    /// Cold gas left in the thruster tank (kg), as reported.
    rcs_gas: f64,
    /// Modeled body angular acceleration the lagging actuators are delivering from the
    /// attitude commands so far.
    delivered: DVec3,
}

impl FlightComputer {
    /// `initial` is the state handed over at start (the flight computer's first estimate).
    pub fn new(vehicle: Vehicle, initial: &State, phase: Phase, fins_deployed: bool) -> Self {
        Self {
            vehicle,
            nav: Navigator::new(initial.pos, initial.vel, initial.att),
            phase,
            t: 0.0,
            ticks: 0,
            mass: initial.mass,
            thrust_level: 0.0,
            gimbal: DVec2::ZERO,
            fins: DVec3::ZERO,
            pad: DVec3::ZERO,
            fins_deployed_at: fins_deployed.then_some(f64::NEG_INFINITY),
            deploy_legs: false,
            boostback_heading: DVec3::ZERO,
            impact: None,
            plan: None,
            plan_revision: 0,
            last_replan: 0.0,
            last_cmd: Commands::default(),
            accel_cmd: DVec3::ZERO,
            engine_out: false,
            deck: None,
            touchdown_at: None,
            held_tilt: None,
            rcs_gas: vehicle.rcs_gas,
            delivered: DVec3::ZERO,
        }
    }

    pub fn phase(&self) -> Phase {
        self.phase
    }

    pub fn nav(&self) -> NavState {
        self.nav.state()
    }

    pub fn mass(&self) -> f64 {
        self.mass
    }

    pub fn impact(&self) -> Option<Impact> {
        self.impact
    }

    pub fn plan(&self) -> Option<&Plan> {
        self.plan.as_ref()
    }

    /// Increments each time a new landing plan is adopted.
    pub fn plan_revision(&self) -> u32 {
        self.plan_revision
    }

    /// Thrust acceleration requested during the landing burn.
    pub fn accel_cmd(&self) -> DVec3 {
        self.accel_cmd
    }

    /// Estimated horizontal wind at the current altitude.
    pub fn wind(&self) -> DVec3 {
        self.nav.state().local_wind()
    }

    pub fn step(&mut self, sensors: &SensorFrame) -> Commands {
        self.engine_out |= sensors.engine_out;
        self.rcs_gas = sensors.rcs_gas;
        // On a drone ship the landing target is wherever its deck is: as reported on the
        // way down, and predicted from the swell during the landing burn.
        if let Some(fix) = sensors.deck {
            self.deck
                .get_or_insert_with(DeckPredictor::default)
                .report(self.t - DECK_DELAY, &fix);
        }
        if let Some(deck) = &mut self.deck {
            if self.phase == Phase::LandingBurn {
                deck.update(self.t);
            }
            if let Some(now) = deck.at(self.t) {
                self.pad = now.pos;
            }
        }
        self.model_actuators(sensors.dt);
        self.t += sensors.dt;
        self.ticks += 1;
        self.nav.update(sensors);
        if matches!(self.phase, Phase::AeroDescent | Phase::LandingBurn) {
            self.read_crossflow(sensors);
        }
        let est = self.nav.state();
        let intent = self.guide(&est, sensors.leg_contact);
        let cmd = self.control(&est, &intent);
        self.last_cmd = cmd;
        cmd
    }

    /// Engines that light for a commanded count, given any reported failure.
    fn lit_engines(&self, commanded: u8) -> u8 {
        if self.engine_out && commanded == 3 {
            2
        } else {
            commanded
        }
    }

    /// Track the engines' first-order throttle lag and the gimbal and fin slew, and
    /// dead-reckon the propellant used.
    fn model_actuators(&mut self, dt: f64) {
        let target = self.lit_engines(self.last_cmd.engines) as f64 * self.last_cmd.throttle;
        let v = &self.vehicle;
        self.thrust_level +=
            (target - self.thrust_level) * (1.0 - (-dt / v.throttle_time_constant).exp());
        self.mass = (self.mass - self.thrust_level * v.engine_mass_flow() * dt).max(v.dry_mass);
        let slew = |at: f64, to: f64, rate: f64| at + (to - at).clamp(-rate * dt, rate * dt);
        let gimbal = self.last_cmd.gimbal.clamp_length_max(v.gimbal_max);
        self.gimbal = DVec2::new(
            slew(self.gimbal.x, gimbal.x, v.gimbal_rate),
            slew(self.gimbal.y, gimbal.y, v.gimbal_rate),
        );
        let fins = self.last_cmd.fins;
        self.fins = DVec3::new(
            slew(self.fins.x, fins.x, v.fin_rate),
            slew(self.fins.y, fins.y, v.fin_rate),
            slew(self.fins.z, fins.z, v.fin_rate),
        );
    }

    /// Read the crossflow from the side force and hand it to navigation, which turns it into
    /// wind. Whatever the accelerometers feel sideways beyond thrust and fins is body normal
    /// force, which gives the angle of attack and so how fast the air streams past sideways.
    fn read_crossflow(&mut self, sensors: &SensorFrame) {
        let est = self.nav.state();
        let v = &self.vehicle;
        let air = est.vel - est.local_wind();
        let q = 0.5 * density(est.pos.z) * air.length_squared();
        if q < CROSSFLOW_MIN_PRESSURE {
            return;
        }
        let gimbal = self.gimbal;
        let thrust = DVec3::new(gimbal.x.tan(), gimbal.y.tan(), 1.0).normalize()
            * self.thrust_level
            * v.engine_thrust(pressure(est.pos.z));
        let fins = self.fins.truncate() * self.fin_gain(q);
        let force = (sensors.accel - est.accel_bias) * self.mass;
        let normal = force.truncate() - thrust.truncate() - fins;
        let qs = q * v.ref_area();
        let alpha =
            guidance::alpha_for_normal_coefficient(v, normal.length() / qs, CROSSFLOW_MAX_ALPHA);
        // The normal force opposes the crossflow.
        let crossflow = -normal.normalize_or_zero() * air.length() * alpha.sin();
        // A sideways force error maps to an angle of attack error through the slope of C_N.
        let slope = v.normal_coeff_slope * (2.0 * alpha).cos()
            + v.normal_coeff_crossflow * (2.0 * alpha).sin();
        let sigma =
            air.length() * self.mass * CROSSFLOW_FORCE_NOISE / (qs * slope) + CROSSFLOW_MIN_NOISE;
        self.nav.correct_crossflow(crossflow, sigma);
    }

    fn predict(&mut self, est: &NavState, burn: EntryBurn, every_tick: bool) -> Impact {
        if every_tick || self.impact.is_none() || self.ticks.is_multiple_of(PREDICT_EVERY) {
            self.impact = Some(predict::impact(
                &self.vehicle,
                est.pos,
                est.vel,
                &est.wind,
                self.mass,
                burn,
            ));
        }
        self.impact.expect("set above")
    }

    fn guide(&mut self, est: &NavState, leg_contact: bool) -> Intent {
        let target = self.pad.truncate();
        let retrograde = guidance::retrograde(est.vel);
        match self.phase {
            Phase::Flip => {
                let impact = self.predict(est, EntryBurn::Pending, true);
                let heading = guidance::horizontal_toward(impact.point, target);
                let nose = est.att * DVec3::Z;
                if nose.angle_between(heading) < FLIP_ALIGNMENT && est.omega.length() < FLIP_RATE {
                    self.boostback_heading = heading;
                    self.phase = Phase::Boostback;
                }
                Intent::unpowered(heading)
            }
            Phase::Boostback => {
                let impact = self.predict(est, EntryBurn::Pending, true);
                let miss = target - impact.point;
                if miss.length() < BOOSTBACK_CUTOFF_MISS
                    || miss.extend(0.0).dot(self.boostback_heading) < 0.0
                {
                    self.phase = Phase::Coast;
                    self.fins_deployed_at = Some(self.t);
                    return Intent::unpowered(retrograde);
                }
                let heading = guidance::horizontal_toward(impact.point, target);
                if miss.length() > BOOSTBACK_TRIM_MISS {
                    Intent {
                        engines: 3,
                        throttle: 1.0,
                        axis: heading,
                    }
                } else {
                    Intent {
                        engines: 1,
                        throttle: self.vehicle.min_throttle,
                        axis: heading,
                    }
                }
            }
            Phase::Coast => {
                self.predict(est, EntryBurn::Pending, false);
                if predict::entry_burn_should_start(est.pos, est.vel) {
                    self.phase = Phase::EntryBurn;
                } else if est.pos.z < ENTRY_BURN_START_ALTITUDE && est.vel.z < 0.0 {
                    self.phase = Phase::AeroDescent;
                }
                // Hold the attitude the entry burn will need (retrograde to the velocity at
                // its altitude on this ballistic arc) rather than chase the turning
                // velocity over the top, which would spend the cold gas.
                let drop = (est.pos.z - ENTRY_BURN_START_ALTITUDE).max(0.0);
                let entry_sink = (est.vel.z.powi(2) + 2.0 * gravity(est.pos.z) * drop).sqrt();
                Intent::unpowered(guidance::retrograde(est.vel.truncate().extend(-entry_sink)))
            }
            Phase::EntryBurn => {
                self.predict(est, EntryBurn::Burning, false);
                if est.vel.length() < ENTRY_BURN_END_SPEED {
                    self.phase = Phase::AeroDescent;
                    return Intent::unpowered(retrograde);
                }
                Intent {
                    engines: ENTRY_BURN_ENGINES,
                    throttle: 1.0,
                    axis: retrograde,
                }
            }
            Phase::AeroDescent => {
                let impact = self.predict(est, EntryBurn::Done, false);
                let (axis, _) = guidance::steering_axis(
                    &self.vehicle,
                    est.pos,
                    est.vel - est.local_wind(),
                    self.mass,
                    &impact,
                    target,
                );
                let ignition =
                    predict::ignition_altitude(&self.vehicle, est.pos, est.vel, self.mass);
                let forced = est.pos.z < FORCED_IGNITION * ignition;
                let retry_due = self.t - self.last_replan >= IGNITION_RETRY;
                if est.pos.z < ignition && (retry_due || forced) {
                    self.last_replan = self.t;
                    let problem = guidance::landing_problem(
                        &self.vehicle,
                        est.pos,
                        est.vel,
                        &est.wind,
                        self.mass,
                        self.pad,
                    );
                    let plan = gfold::solve_free_time(&problem, self.t);
                    if plan.is_some() || forced {
                        self.adopt(plan);
                        self.phase = Phase::LandingBurn;
                        return self.landing(est, leg_contact);
                    }
                }
                Intent::unpowered(axis)
            }
            Phase::LandingBurn => self.landing(est, leg_contact),
            Phase::Landed => Intent::unpowered(est.att * DVec3::Z),
        }
    }

    fn adopt(&mut self, plan: Option<Plan>) {
        if plan.is_some() {
            self.plan_revision += 1;
        }
        self.plan = plan;
        self.last_replan = self.t;
    }

    fn landing(&mut self, est: &NavState, leg_contact: bool) -> Intent {
        let height = est.pos.z - self.pad.z - self.vehicle.standing_height(self.mass);
        let deck_now = self.deck.as_ref().and_then(|d| d.at(self.t));
        let closing = est.vel.z - deck_now.map_or(0.0, |d| d.vel_z);
        let climbing_back = height < CLIMB_CUTOFF_HEIGHT && closing > 0.0;
        if leg_contact || height < TERMINAL_CUTOFF_HEIGHT || climbing_back {
            self.phase = Phase::Landed;
            return Intent::unpowered(est.att * DVec3::Z);
        }
        if est.pos.z < LEG_DEPLOY_ALTITUDE {
            self.deploy_legs = true;
        }
        let v = self.vehicle;
        let replan_due = self.t - self.last_replan >= REPLAN_PERIOD;
        if let Some(remaining) = self.plan.as_ref().map(|p| p.end() - self.t)
            && replan_due
            && remaining > REPLAN_CUTOFF
        {
            let problem =
                guidance::landing_problem(&v, est.pos, est.vel, &est.wind, self.mass, self.pad);
            let plan = [1.0, 1.05, 0.95, 1.1, 0.9]
                .iter()
                .find_map(|f| gfold::solve(&problem, remaining * f, self.t));
            match plan {
                Some(plan) => self.adopt(Some(plan)),
                None => self.last_replan = self.t,
            }
        }

        // Vertical tracking can be brisk (throttle responds in ~0.15 s); horizontal tracking
        // must stay well below the attitude loop's bandwidth, since it works by tilting.
        let vertical = double_integrator_lqr(2.0, 1.0, 3.0);
        let horizontal = double_integrator_lqr(5.0, 2.0, 2.0);
        let accel = match &self.plan {
            Some(plan) if self.t <= plan.end() => {
                let (pos, vel, thrust_accel) = plan.sample(self.t);
                let (dp, dv) = (pos - est.pos, vel - est.vel);
                let feedback = (dp.truncate() * horizontal.kp + dv.truncate() * horizontal.kd)
                    .extend(dp.z * vertical.kp + dv.z * vertical.kd);
                thrust_accel + feedback
            }
            _ => guidance::fallback_accel(&v, est.pos, est.vel, self.mass, self.pad),
        };
        let height = est.pos.z - self.pad.z - v.standing_height(self.mass);
        // Near the ground terminal guidance plans the sideways motion, aiming above all for
        // an upright touchdown without sideways speed. Higher up, fly the plan, leaning off
        // the flow line as the aerodynamics allow.
        // Over a drone ship's deck, the last meters are flown relative to the deck, to
        // touch down at a moment when it is level and not rising.
        let vertical = match deck_now {
            Some(deck) if height < CALM_HEIGHT => {
                let touchdown = match self.touchdown_at {
                    Some(t) => t,
                    None => *self.touchdown_at.insert(self.calm_moment(height, -closing)),
                };
                let to_go = touchdown - self.t;
                // Arrive at the deck at the chosen time and speed: the minimum-effort
                // (zero-effort miss and velocity) law on height and speed above the deck.
                let relative = if to_go > DECK_FINAL_TIME {
                    let miss = -(height + closing * to_go);
                    let speed_error = -DECK_TOUCHDOWN_SPEED - closing;
                    6.0 * miss / to_go.powi(2) - 2.0 * speed_error / to_go
                } else {
                    DECK_MATCH_GAIN * (-DECK_TOUCHDOWN_SPEED - closing)
                };
                gravity(est.pos.z) + deck.accel_z + relative
            }
            _ => accel.z,
        };
        let accel = if height < terminal::HEIGHT {
            if self.held_tilt.is_none() || self.ticks.is_multiple_of(TERMINAL_EVERY) {
                self.held_tilt = self.terminal_tilt(est, vertical, height);
            }
            let sideways = self
                .held_tilt
                .map_or(-est.vel.truncate() * VELOCITY_ONLY_GAIN, |tilt| {
                    DVec2::new(tilt.x.tan(), tilt.y.tan()) * vertical.max(1.0)
                });
            sideways.extend(vertical)
        } else {
            self.held_tilt = None;
            let air = est.vel - est.local_wind();
            guidance::shape_thrust(&v, accel, air, est.pos.z, self.mass)
        };
        let accel = guidance::cone_limit(accel, DVec3::Z, guidance::tilt_limit(height));
        self.accel_cmd = accel;
        let throttle = self.mass * accel.length() / v.engine_thrust(pressure(est.pos.z));
        Intent {
            engines: 1,
            throttle,
            axis: accel.normalize(),
        }
    }

    /// When to touch down on the deck, `height` above it and closing at `speed`: the
    /// calmest predicted moment (deck level and not rising) among the times the approach
    /// can be flown. Flown by the minimum-effort law, the acceleration relative to the deck
    /// changes linearly from start to touchdown; both ends must stay within what the
    /// engine can do, which rules out dawdling (at minimum throttle a light booster still
    /// slows) as well as rushing.
    fn calm_moment(&self, height: f64, speed: f64) -> f64 {
        let v = &self.vehicle;
        let thrust = v.engine_thrust(pressure(0.0)) / self.mass;
        let (low, high) = (
            v.min_throttle * thrust - gravity(0.0) + CALM_ACCEL_MARGIN,
            PLAN_THROTTLE * thrust - gravity(0.0) - CALM_ACCEL_MARGIN,
        );
        let feasible = |time: f64| {
            // Upward relative acceleration at the start and at touchdown.
            let excess = (speed + DECK_TOUCHDOWN_SPEED) * time / 2.0 - height;
            let jerk = -12.0 * excess / time.powi(3);
            let start = (speed - DECK_TOUCHDOWN_SPEED) / time - jerk * time / 2.0;
            let end = start + jerk * time;
            (low..=high).contains(&start) && (low..=high).contains(&end)
        };
        let badness = |t: f64| {
            self.deck
                .as_ref()
                .and_then(|d| d.at(t))
                .map_or(0.0, |d: DeckState| {
                    d.tilt.length_squared() / CALM_TILT.powi(2)
                        + d.vel_z.max(0.0).powi(2) / CALM_RISE.powi(2)
                })
        };
        let steady = 2.0 * height / (speed.max(DECK_TOUCHDOWN_SPEED) + DECK_TOUCHDOWN_SPEED);
        (1..=CALM_CANDIDATES)
            .map(|i| steady * 3.0 * i as f64 / CALM_CANDIDATES as f64)
            .filter(|&time| feasible(time))
            .map(|time| self.t + time)
            .min_by(|&a, &b| badness(a).total_cmp(&badness(b)))
            .unwrap_or(self.t + steady)
    }

    /// The tilt toward each horizontal axis from terminal guidance, given the vertical
    /// thrust acceleration and the height above touchdown.
    fn terminal_tilt(&self, est: &NavState, vertical: f64, height: f64) -> Option<DVec2> {
        let (eff, inertia) = self.effectors(est);
        let v = &self.vehicle;
        let authority = control::authority(v, &eff, inertia);
        let lag = control::actuator_lag(v, &eff, inertia);
        let gains = control::attitude_gains(authority.x, lag.x);
        // Braking at a steady rate to touchdown takes twice as long as coasting the
        // height at the current speed; over a deck, the touchdown may be timed.
        let time_to_go = match self.touchdown_at {
            Some(t) if height < CALM_HEIGHT => (t - self.t).max(0.1),
            _ => 2.0 * height / est.vel.z.abs().max(1.0),
        };
        // Land square to the deck: arrive tilted as it will be.
        let final_tilt = self
            .deck
            .as_ref()
            .and_then(|d| d.at(self.t + time_to_go))
            .map_or(DVec2::ZERO, |d| d.tilt);
        let accel = vertical.max(1.0);
        let wind = est.local_wind();
        let air_speed = (est.vel - wind).length();
        let q = 0.5 * density(est.pos.z) * air_speed * air_speed;
        let model = terminal::Model {
            accel,
            kp: gains.kp,
            kd: gains.kd,
            aero: q * v.ref_area() * v.normal_coeff_slope / self.mass,
            air_speed,
            time_to_go,
        };
        let n = terminal::steps(time_to_go);
        let limits: Vec<f64> = (0..n)
            .map(|k| guidance::tilt_limit(height * (1.0 - (k + 1) as f64 / n as f64).powi(2)))
            .collect();
        let nose = est.att * DVec3::Z;
        // Rotating about world +y tilts the nose toward +x; about +x, toward -y.
        let rate = est.att * est.omega;
        let axis =
            |offset: f64, velocity: f64, lean: f64, tilt_rate: f64, wind: f64| terminal::Axis {
                position: offset,
                velocity,
                tilt: lean.atan2(nose.z),
                tilt_rate,
                wind,
            };
        let offset = est.pos - self.pad;
        let x = axis(offset.x, est.vel.x, nose.x, rate.y, wind.x);
        let y = axis(offset.y, est.vel.y, nose.y, -rate.x, wind.y);
        Some(DVec2::new(
            terminal::tilt_command(x, &model, &limits, final_tilt.x)?,
            terminal::tilt_command(y, &model, &limits, final_tilt.y)?,
        ))
    }

    /// What the flight computer believes it can push on, and the inertia it pushes.
    fn effectors(&self, est: &NavState) -> (Effectors, DVec3) {
        let v = &self.vehicle;
        let thrust = self.thrust_level * v.engine_thrust(pressure(est.pos.z));
        let v_air = est.att.inverse() * (est.vel - est.local_wind());
        let q = 0.5 * density(est.pos.z) * v_air.length_squared();
        let eff = Effectors {
            thrust,
            fin_gain: self.fin_gain(q),
            com_height: v.com_height(self.mass),
            cold_gas: self.rcs_gas > 0.0,
        };
        (eff, v.inertia(self.mass))
    }

    /// Fin side force per radian of deflection.
    fn fin_gain(&self, q: f64) -> f64 {
        let v = &self.vehicle;
        let fins_ready = self
            .fins_deployed_at
            .is_some_and(|t0| self.t - t0 >= v.fin_deploy_time);
        if fins_ready && q > MIN_FIN_PRESSURE {
            q * v.fin_area * v.fin_lift_slope
        } else {
            0.0
        }
    }

    fn control(&mut self, est: &NavState, intent: &Intent) -> Commands {
        let v = &self.vehicle;
        let deploy_fins = self.fins_deployed_at.is_some();
        if self.phase == Phase::Landed {
            return Commands {
                deploy_fins,
                deploy_legs: self.deploy_legs,
                ..Default::default()
            };
        }
        let throttle = if intent.engines > 0 {
            intent.throttle.clamp(v.min_throttle, 1.0)
        } else {
            0.0
        };
        // Steer with the thrust the engines are actually producing, not the new command.
        let (eff, inertia) = self.effectors(est);
        let v_air = est.att.inverse() * (est.vel - est.local_wind());
        let lag = control::actuator_lag(v, &eff, inertia);
        let accel = control::attitude_accel(
            est.att,
            est.omega,
            intent.axis,
            control::authority(v, &eff, inertia),
            lag,
            self.delivered,
            match self.phase {
                Phase::Flip => FLIP_TURN_RATE,
                Phase::Coast => COAST_TURN_RATE,
                _ => f64::INFINITY,
            },
        );
        // The actuators close on the new command over the next tick.
        self.delivered += (accel - self.delivered) * (DVec3::ONE - (-CONTROL_PERIOD / lag).exp());
        // Feed forward the expected body aerodynamic torque (fins excluded: they are an
        // actuator) and the off-center push of an engine group that has lost an engine.
        let aero_torque = sim::aero::loads(v, v_air, est.pos.z, self.mass, DVec3::ZERO, 0.0).torque;
        let lit = self.lit_engines(self.last_cmd.engines);
        let offset = v.thrust_point(lit, self.mass) - v.thrust_point(1, self.mass);
        let thrust_torque = offset.cross(DVec3::Z * eff.thrust);
        let torque =
            inertia * accel + est.omega.cross(inertia * est.omega) - aero_torque - thrust_torque;
        let mut alloc = control::allocate(v, &eff, torque);
        // Nothing pushes the booster around above the air, so on the way over the top let
        // the attitude drift within a deadband rather than spend cold gas holding it.
        let drifting = est.att * DVec3::Z;
        let within_band = drifting.angle_between(intent.axis) < COAST_DEADBAND
            && est.omega.length() < COAST_DEADBAND_RATE;
        if self.phase == Phase::Coast && within_band {
            alloc.rcs = DVec3::ZERO;
        }

        Commands {
            engines: intent.engines,
            throttle,
            gimbal: alloc.gimbal,
            fins: alloc.fins,
            rcs: alloc.rcs,
            deploy_fins,
            deploy_legs: self.deploy_legs,
        }
    }
}
