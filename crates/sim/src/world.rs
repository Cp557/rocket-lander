//! The simulated world: true state, actuator dynamics, wind and sensors.

use glam::{DQuat, DVec2, DVec3};
use rand::SeedableRng;
use rand::rngs::StdRng;

use crate::dispersion::Faults;
use crate::dynamics::{Inputs, State, rk4};
use crate::env::{Wind, gravity_vec, pressure};
use crate::sensors::{
    DECK_ATT_NOISE, DECK_DELAY, DECK_PERIOD, DECK_POS_NOISE, DECK_VEL_NOISE, DeckFix, SensorFrame,
    Sensors, due, noise3,
};
use crate::ship::Ship;
use crate::vehicle::Vehicle;
use crate::{aero, contact};

/// What the flight computer asks the hardware to do.
#[derive(Clone, Copy, Debug, Default, PartialEq)]
pub struct Commands {
    /// Number of engines to light (0, 1 or 3).
    pub engines: u8,
    /// Per-engine throttle; the hardware clamps it to `min_throttle..=1`.
    pub throttle: f64,
    /// Engine gimbal angles (rad): x tilts thrust toward body +x, y toward body +y.
    pub gimbal: DVec2,
    /// Grid fin pitch/yaw/roll deflections (rad).
    pub fins: DVec3,
    /// Cold-gas thruster torque, body frame.
    pub rcs: DVec3,
    pub deploy_fins: bool,
    pub deploy_legs: bool,
}

/// Actual actuator positions after lags and rate limits.
#[derive(Clone, Copy, Debug, Default, PartialEq)]
pub struct Actuators {
    /// Engines actually lit: 2 when the three-engine group has lost an engine.
    pub engines: u8,
    /// Total thrust in units of one full-throttle engine.
    pub thrust_level: f64,
    pub gimbal: DVec2,
    pub fins: DVec3,
    pub rcs: DVec3,
    /// Deployment fractions, 0..=1.
    pub fin_deploy: f64,
    pub leg_deploy: f64,
}

/// Derived quantities from the latest step, for telemetry and scoring.
#[derive(Clone, Copy, Debug, Default)]
pub struct WorldInfo {
    pub thrust: f64,
    pub dynamic_pressure: f64,
    pub alpha: f64,
    pub wind: DVec3,
    pub legs_touching: usize,
    pub hull_touching: usize,
    pub in_water: usize,
}

pub struct World {
    pub vehicle: Vehicle,
    pub state: State,
    pub act: Actuators,
    pub wind: Wind,
    pub misalignment: DVec2,
    pub t: f64,
    pub info: WorldInfo,
    /// The drone ship to land on; `None` lands on the ground.
    pub ship: Option<Ship>,
    /// Cold gas left (kg); the thrusters fall silent when it runs out.
    pub rcs_gas: f64,
    engine_out_after: Option<f64>,
    /// Time spent firing three engines while descending, which is when the engine can fail.
    descending_burn_time: f64,
    next_deck_report: f64,
    sensors: Sensors,
    rng: StdRng,
}

impl World {
    pub fn new(
        vehicle: Vehicle,
        state: State,
        act: Actuators,
        wind: Wind,
        misalignment: DVec2,
        faults: Faults,
        seed: u64,
    ) -> Self {
        let mut rng = StdRng::seed_from_u64(seed);
        let sensors = Sensors::new(&mut rng, faults);
        let mut world = Self {
            vehicle,
            state,
            act,
            wind,
            misalignment,
            t: 0.0,
            info: WorldInfo::default(),
            ship: None,
            rcs_gas: vehicle.rcs_gas,
            engine_out_after: faults.engine_out_after,
            descending_burn_time: 0.0,
            next_deck_report: 0.0,
            sensors,
            rng,
        };
        world.info = world.compute_info(0.0);
        world
    }

    pub fn step(&mut self, cmd: &Commands, dt: f64) {
        let v = self.vehicle;
        let altitude = self.state.pos.z;
        let has_propellant = self.state.mass > v.dry_mass;
        self.update_actuators(cmd, has_propellant, dt);
        self.wind.update_gust(dt, &mut self.rng);

        let a = &self.act;
        let thrust = a.thrust_level * v.engine_thrust(pressure(altitude));
        let gimbal = a.gimbal + self.misalignment;
        let inputs = Inputs {
            thrust,
            thrust_dir: DVec3::new(gimbal.x.tan(), gimbal.y.tan(), 1.0).normalize(),
            thrust_point: v.thrust_point(a.engines, self.state.mass),
            mass_flow: a.thrust_level * v.engine_mass_flow(),
            fins: a.fins,
            fin_deploy: a.fin_deploy,
            rcs: a.rcs,
            legs_deployed: a.leg_deploy >= 1.0,
            wind: self.wind.at(altitude),
            deck: self.ship.map(|ship| ship.pose(self.t)),
        };

        self.rcs_gas = (self.rcs_gas - v.rcs_gas_flow(a.rcs) * dt).max(0.0);
        let before = self.state;
        self.state = rk4(&v, &before, &inputs, dt);
        self.state.mass = self.state.mass.max(v.dry_mass);
        self.t += dt;

        let specific_force = (self.state.vel - before.vel) / dt - gravity_vec(altitude);
        self.sensors.accumulate(
            self.state.att.inverse() * specific_force,
            self.state.omega,
            dt,
        );
        self.info = self.compute_info(thrust);
    }

    pub fn sense(&mut self) -> SensorFrame {
        let frame = self.sensors.read(
            self.t,
            &self.state,
            self.info.legs_touching > 0,
            &mut self.rng,
        );
        let deck_due = due(self.t, &mut self.next_deck_report, DECK_PERIOD);
        let deck = self.ship.filter(|_| deck_due).map(|ship| {
            let pose = ship.pose((self.t - DECK_DELAY).max(0.0));
            DeckFix {
                pos: pose.origin + noise3(&mut self.rng, DECK_POS_NOISE),
                vel: pose.vel + noise3(&mut self.rng, DECK_VEL_NOISE),
                att: pose.att * DQuat::from_scaled_axis(noise3(&mut self.rng, DECK_ATT_NOISE)),
            }
        });
        SensorFrame {
            engine_out: self.engine_failed(),
            deck,
            rcs_gas: self.rcs_gas,
            ..frame
        }
    }

    pub fn sensors(&self) -> &Sensors {
        &self.sensors
    }

    /// The outer engine at body -x has failed.
    pub fn engine_failed(&self) -> bool {
        self.engine_out_after
            .is_some_and(|after| self.descending_burn_time >= after)
    }

    fn update_actuators(&mut self, cmd: &Commands, has_propellant: bool, dt: f64) {
        if cmd.engines == 3 && self.state.vel.z < 0.0 {
            self.descending_burn_time += dt;
        }
        let engines = if self.engine_failed() && cmd.engines == 3 {
            2
        } else {
            cmd.engines
        };
        let gas_left = self.rcs_gas > 0.0;
        let v = &self.vehicle;
        let a = &mut self.act;
        let lit = has_propellant && engines > 0;
        let target = if lit {
            engines as f64 * cmd.throttle.clamp(v.min_throttle, 1.0)
        } else {
            0.0
        };
        a.thrust_level +=
            (target - a.thrust_level) * (1.0 - (-dt / v.throttle_time_constant).exp());
        if !has_propellant {
            a.thrust_level = 0.0;
        }
        if lit {
            a.engines = engines;
        } else if a.thrust_level < 0.01 {
            a.engines = 0;
        }
        let step = v.gimbal_rate * dt;
        a.gimbal += (cmd.gimbal.clamp_length_max(v.gimbal_max) - a.gimbal)
            .clamp(DVec2::splat(-step), DVec2::splat(step));
        if cmd.deploy_fins {
            a.fin_deploy = (a.fin_deploy + dt / v.fin_deploy_time).min(1.0);
        }
        if cmd.deploy_legs {
            a.leg_deploy = (a.leg_deploy + dt / v.leg_deploy_time).min(1.0);
        }
        let fin_target = if a.fin_deploy >= 1.0 {
            cmd.fins
                .clamp(-DVec3::splat(v.fin_max), DVec3::splat(v.fin_max))
        } else {
            DVec3::ZERO
        };
        let step = v.fin_rate * dt;
        a.fins += (fin_target - a.fins).clamp(DVec3::splat(-step), DVec3::splat(step));
        a.rcs = if gas_left {
            cmd.rcs.clamp(-v.rcs_torque, v.rcs_torque)
        } else {
            DVec3::ZERO
        };
    }

    fn compute_info(&self, thrust: f64) -> WorldInfo {
        let altitude = self.state.pos.z;
        let wind = self.wind.at(altitude);
        let v_air = self.state.att.inverse() * (self.state.vel - wind);
        let air = aero::loads(
            &self.vehicle,
            v_air,
            altitude,
            self.state.mass,
            self.act.fins,
            self.act.fin_deploy,
        );
        let deck = self.ship.map(|ship| ship.pose(self.t));
        let contact = contact::loads(
            &self.vehicle,
            &self.state,
            self.act.leg_deploy >= 1.0,
            deck.as_ref(),
        );
        WorldInfo {
            thrust,
            dynamic_pressure: air.dynamic_pressure,
            alpha: air.alpha,
            wind,
            legs_touching: contact.legs_touching,
            hull_touching: contact.hull_touching,
            in_water: contact.in_water,
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::env::{G0, gravity};
    use glam::DQuat;

    const DT: f64 = 0.002;

    fn world_at(pos: DVec3, vel: DVec3, mass: f64, act: Actuators) -> World {
        let state = State {
            pos,
            vel,
            att: DQuat::IDENTITY,
            omega: DVec3::ZERO,
            mass,
        };
        World::new(
            Vehicle::nominal(),
            state,
            act,
            Wind::default(),
            DVec2::ZERO,
            Faults::default(),
            1,
        )
    }

    fn run(world: &mut World, cmd: &Commands, seconds: f64) {
        for _ in 0..(seconds / DT).round() as usize {
            world.step(cmd, DT);
        }
    }

    #[test]
    fn free_fall_in_vacuum_matches_analytic() {
        let h0 = 300_000.0;
        let mut world = world_at(
            DVec3::new(0.0, 0.0, h0),
            DVec3::ZERO,
            30_000.0,
            Actuators::default(),
        );
        run(&mut world, &Commands::default(), 10.0);
        let expected = h0 - 0.5 * gravity(h0) * 100.0;
        assert!(
            (world.state.pos.z - expected).abs() < 0.05,
            "z = {}, expected {}",
            world.state.pos.z,
            expected
        );
    }

    #[test]
    fn burn_matches_rocket_equation() {
        let h0 = 300_000.0;
        let m0 = 40_000.0;
        let mut world = world_at(
            DVec3::new(0.0, 0.0, h0),
            DVec3::ZERO,
            m0,
            Actuators::default(),
        );
        let cmd = Commands {
            engines: 1,
            throttle: 1.0,
            ..Default::default()
        };
        let seconds = 20.0;
        run(&mut world, &cmd, seconds);
        let v = world.vehicle;
        let gravity_dv = -gravity(h0) * seconds;
        let measured = world.state.vel.z - gravity_dv;
        let expected = v.isp_vac * G0 * (m0 / world.state.mass).ln();
        assert!(
            (measured - expected).abs() / expected < 0.01,
            "dv {measured} vs {expected}"
        );
    }

    #[test]
    fn rests_upright_on_deployed_legs() {
        let v = Vehicle::nominal();
        let act = Actuators {
            leg_deploy: 1.0,
            ..Default::default()
        };
        let mut world = world_at(
            DVec3::new(0.0, 0.0, v.standing_height(27_000.0)),
            DVec3::ZERO,
            27_000.0,
            act,
        );
        run(&mut world, &Commands::default(), 5.0);
        // The leg shock absorbers give about 0.17 m under 27 t.
        let sag = v.standing_height(27_000.0) - world.state.pos.z;
        assert!(sag > 0.1 && sag < 0.25, "sag {sag}");
        assert!(world.state.vel.length() < 0.01);
        assert!(world.state.tilt() < 1e-3);
        assert_eq!(world.info.legs_touching, 4);
        assert_eq!(world.info.hull_touching, 0);
    }

    #[test]
    fn torque_free_spin_conserves_angular_momentum() {
        let mut world = world_at(
            DVec3::new(0.0, 0.0, 300_000.0),
            DVec3::ZERO,
            30_000.0,
            Actuators::default(),
        );
        world.state.omega = DVec3::new(0.1, 0.0, 0.5);
        let momentum =
            |w: &World| (w.state.att * (w.vehicle.inertia(w.state.mass) * w.state.omega)).length();
        let before = momentum(&world);
        run(&mut world, &Commands::default(), 20.0);
        assert!((momentum(&world) - before).abs() / before < 1e-6);
    }

    #[test]
    fn engine_out_leaves_two_engines_pushing_off_center() {
        let mut world = world_at(
            DVec3::new(0.0, 0.0, 30_000.0),
            DVec3::new(0.0, 0.0, -500.0),
            60_000.0,
            Actuators::default(),
        );
        world.engine_out_after = Some(0.5);
        let cmd = Commands {
            engines: 3,
            throttle: 1.0,
            ..Default::default()
        };
        run(&mut world, &cmd, 1.0);
        assert!(world.engine_failed() && world.sense().engine_out);
        assert_eq!(world.act.engines, 2);
        assert!((world.act.thrust_level - 2.0).abs() < 0.05);
        // The survivor at +x pushes off center, pitching the body about -y.
        assert!(world.state.omega.y < -1e-3 && world.state.omega.x.abs() < 1e-9);
    }

    #[test]
    fn thrusters_fall_silent_when_the_gas_runs_out() {
        let mut world = world_at(
            DVec3::new(0.0, 0.0, 300_000.0),
            DVec3::ZERO,
            30_000.0,
            Actuators::default(),
        );
        world.rcs_gas = 1.0;
        let cmd = Commands {
            rcs: DVec3::new(150e3, 0.0, 0.0),
            ..Default::default()
        };
        // Full pitch torque spends 150e3 / 18 N of 70 s gas: about 12 kg/s.
        run(&mut world, &cmd, 1.0);
        assert_eq!(world.rcs_gas, 0.0);
        assert_eq!(world.act.rcs, DVec3::ZERO);
        let spun = world.state.omega.x;
        run(&mut world, &cmd, 1.0);
        assert!((world.state.omega.x - spun).abs() < 1e-9);
    }

    #[test]
    fn imu_reads_gravity_reaction_at_rest() {
        let v = Vehicle::nominal();
        let act = Actuators {
            leg_deploy: 1.0,
            ..Default::default()
        };
        let mut world = world_at(
            DVec3::new(0.0, 0.0, v.standing_height(27_000.0) - 0.17),
            DVec3::ZERO,
            27_000.0,
            act,
        );
        run(&mut world, &Commands::default(), 3.0);
        world.sense();
        run(&mut world, &Commands::default(), 1.0);
        let frame = world.sense();
        assert!((frame.accel.z - G0).abs() < 0.2, "accel {:?}", frame.accel);
    }
}
