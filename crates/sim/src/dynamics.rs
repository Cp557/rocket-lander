//! 6-DOF rigid-body state and RK4 integration.

use glam::{DQuat, DVec3};

use crate::ship::DeckPose;
use crate::vehicle::Vehicle;
use crate::{aero, contact, env};

#[derive(Clone, Copy, Debug, PartialEq)]
pub struct State {
    /// CoM position, world frame.
    pub pos: DVec3,
    pub vel: DVec3,
    /// Rotation from body to world.
    pub att: DQuat,
    /// Angular velocity, body frame.
    pub omega: DVec3,
    pub mass: f64,
}

impl State {
    /// Nose direction in the world frame.
    pub fn axis(&self) -> DVec3 {
        self.att * DVec3::Z
    }

    /// Angle between the nose and vertical.
    pub fn tilt(&self) -> f64 {
        self.axis().z.clamp(-1.0, 1.0).acos()
    }
}

/// Everything held constant over one integration step.
#[derive(Clone, Copy, Debug)]
pub struct Inputs {
    pub thrust: f64,
    /// Unit thrust direction in the body frame.
    pub thrust_dir: DVec3,
    /// Where the thrust acts, body frame.
    pub thrust_point: DVec3,
    pub mass_flow: f64,
    pub fins: DVec3,
    pub fin_deploy: f64,
    pub rcs: DVec3,
    pub legs_deployed: bool,
    pub wind: DVec3,
    /// The drone ship's deck, when landing on it rather than on the ground.
    pub deck: Option<DeckPose>,
}

struct Derivative {
    pos: DVec3,
    vel: DVec3,
    att: DQuat,
    omega: DVec3,
    mass: f64,
}

fn derivative(vehicle: &Vehicle, s: &State, inp: &Inputs) -> Derivative {
    let altitude = s.pos.z;
    let v_air = s.att.inverse() * (s.vel - inp.wind);
    let aero = aero::loads(vehicle, v_air, altitude, s.mass, inp.fins, inp.fin_deploy);
    let contact = contact::loads(vehicle, s, inp.legs_deployed, inp.deck.as_ref());

    let thrust = inp.thrust_dir * inp.thrust;
    let body_force = thrust + aero.force;
    let torque = inp.thrust_point.cross(thrust) + aero.torque + contact.torque + inp.rcs;
    let inertia = vehicle.inertia(s.mass);

    Derivative {
        pos: s.vel,
        vel: (s.att * body_force + contact.force) / s.mass + env::gravity_vec(altitude),
        att: s.att * DQuat::from_xyzw(s.omega.x, s.omega.y, s.omega.z, 0.0) * 0.5,
        omega: (torque - s.omega.cross(inertia * s.omega)) / inertia,
        mass: -inp.mass_flow,
    }
}

fn advance(s: &State, d: &Derivative, dt: f64) -> State {
    State {
        pos: s.pos + d.pos * dt,
        vel: s.vel + d.vel * dt,
        att: s.att + d.att * dt,
        omega: s.omega + d.omega * dt,
        mass: s.mass + d.mass * dt,
    }
}

pub fn rk4(vehicle: &Vehicle, s: &State, inp: &Inputs, dt: f64) -> State {
    let k1 = derivative(vehicle, s, inp);
    let k2 = derivative(vehicle, &advance(s, &k1, dt / 2.0), inp);
    let k3 = derivative(vehicle, &advance(s, &k2, dt / 2.0), inp);
    let k4 = derivative(vehicle, &advance(s, &k3, dt), inp);
    let blend = Derivative {
        pos: (k1.pos + 2.0 * k2.pos + 2.0 * k3.pos + k4.pos) / 6.0,
        vel: (k1.vel + 2.0 * k2.vel + 2.0 * k3.vel + k4.vel) / 6.0,
        att: (k1.att + k2.att * 2.0 + k3.att * 2.0 + k4.att) * (1.0 / 6.0),
        omega: (k1.omega + 2.0 * k2.omega + 2.0 * k3.omega + k4.omega) / 6.0,
        mass: (k1.mass + 2.0 * k2.mass + 2.0 * k3.mass + k4.mass) / 6.0,
    };
    let mut next = advance(s, &blend, dt);
    next.att = next.att.normalize();
    next
}
