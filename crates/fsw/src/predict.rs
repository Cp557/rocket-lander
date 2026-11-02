//! Landing-point prediction: a fast point-mass simulation of the rest of the descent
//! (gravity, base-first drag, the planned entry burn and a retrograde landing burn; no lift).
//! The landing burn is flown near retrograde while dynamic pressure is high, so a
//! retrograde burn at `IGNITION_THROTTLE` is a good stand-in for where it ends up.

use glam::{DVec2, DVec3};
use sim::Vehicle;
use sim::aero::axial_coefficient;
use sim::env::{Wind, density, gravity, gravity_vec, pressure, speed_of_sound};

pub const ENTRY_BURN_START_ALTITUDE: f64 = 45_000.0;
pub const ENTRY_BURN_END_SPEED: f64 = 450.0;
pub const ENTRY_BURN_ENGINES: u8 = 3;
/// Single-engine throttle assumed when picking the ignition altitude, leaving margin for G-FOLD.
const IGNITION_THROTTLE: f64 = 0.7;

const STEP: f64 = 0.2;
const HORIZON: f64 = 1200.0;

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum EntryBurn {
    Pending,
    Burning,
    Done,
}

pub fn entry_burn_should_start(pos: DVec3, vel: DVec3) -> bool {
    pos.z < ENTRY_BURN_START_ALTITUDE && vel.z < 0.0 && vel.length() > ENTRY_BURN_END_SPEED
}

/// Drag deceleration of the base-first body.
pub fn drag_accel(vehicle: &Vehicle, altitude: f64, speed: f64, mass: f64) -> f64 {
    let mach = speed / speed_of_sound(altitude);
    0.5 * density(altitude)
        * speed
        * speed
        * axial_coefficient(vehicle, true, mach)
        * vehicle.ref_area()
        / mass
}

/// Altitude at which a single-engine burn at `IGNITION_THROTTLE`, helped by about half of
/// the current drag (drag fades as the burn slows the vehicle), stops at touchdown height.
pub fn ignition_altitude(vehicle: &Vehicle, pos: DVec3, vel: DVec3, mass: f64) -> f64 {
    let thrust = IGNITION_THROTTLE * vehicle.engine_thrust(pressure(pos.z)) / mass;
    let decel = thrust + 0.5 * drag_accel(vehicle, pos.z, vel.length(), mass) - gravity(pos.z);
    vehicle.standing_height(mass) + vel.z.min(0.0).powi(2) / (2.0 * decel.max(0.5))
}

#[derive(Clone, Copy, Debug, PartialEq)]
pub struct Impact {
    /// Where the vehicle stops (or hits the ground).
    pub point: DVec2,
    /// Time until then.
    pub time: f64,
    /// Time until landing-burn ignition, if the descent reaches it.
    pub ignition: Option<f64>,
}

/// Drag acts on the velocity relative to `wind` at each altitude.
pub fn impact(
    vehicle: &Vehicle,
    mut pos: DVec3,
    mut vel: DVec3,
    wind: &Wind,
    mut mass: f64,
    mut burn: EntryBurn,
) -> Impact {
    let mut t = 0.0;
    let mut prev = pos;
    let mut ignition = None;
    while pos.z > 0.0 && t < HORIZON {
        let speed = vel.length();
        let dir = if speed > 1e-6 {
            vel / speed
        } else {
            DVec3::ZERO
        };
        let air = vel - wind.at(pos.z);
        let air_speed = air.length().max(1e-6);
        let mut accel =
            gravity_vec(pos.z) - air / air_speed * drag_accel(vehicle, pos.z, air_speed, mass);
        // Same decision the flight computer makes when leaving coast.
        if burn == EntryBurn::Pending && pos.z < ENTRY_BURN_START_ALTITUDE && vel.z < 0.0 {
            burn = if entry_burn_should_start(pos, vel) {
                EntryBurn::Burning
            } else {
                EntryBurn::Done
            };
        }
        if burn == EntryBurn::Burning {
            if speed < ENTRY_BURN_END_SPEED || mass <= vehicle.dry_mass {
                burn = EntryBurn::Done;
            } else {
                let engines = ENTRY_BURN_ENGINES as f64;
                accel -= dir * engines * vehicle.engine_thrust(pressure(pos.z)) / mass;
                mass -= engines * vehicle.engine_mass_flow() * STEP;
            }
        }
        let can_land = burn == EntryBurn::Done && mass > vehicle.dry_mass && vel.z < 0.0;
        if ignition.is_none() && can_land && pos.z < ignition_altitude(vehicle, pos, vel, mass) {
            ignition = Some(t);
        }
        if ignition.is_some() && mass > vehicle.dry_mass {
            if vel.z >= 0.0 || pos.z <= vehicle.standing_height(mass) {
                return Impact {
                    point: pos.truncate(),
                    time: t,
                    ignition,
                };
            }
            accel -= dir * IGNITION_THROTTLE * vehicle.engine_thrust(pressure(pos.z)) / mass;
            mass -= IGNITION_THROTTLE * vehicle.engine_mass_flow() * STEP;
        }
        prev = pos;
        vel += accel * STEP;
        pos += vel * STEP;
        t += STEP;
    }
    // Interpolate the last step to the ground crossing.
    let f = if prev.z > pos.z {
        (prev.z / (prev.z - pos.z)).clamp(0.0, 1.0)
    } else {
        1.0
    };
    let hit = prev + (pos - prev) * f;
    Impact {
        point: hit.truncate(),
        time: t - STEP * (1.0 - f),
        ignition,
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn unpowered_arc_matches_fine_integration() {
        // Compare the coarse unpowered arc (no propellant, so no landing burn) against a
        // fine-step reference.
        let v = Vehicle::nominal();
        let mass = v.dry_mass;
        let pos = DVec3::new(0.0, 0.0, 120_000.0);
        let vel = DVec3::new(300.0, 0.0, 200.0);
        let coarse = impact(&v, pos, vel, &Wind::default(), mass, EntryBurn::Done);
        let (mut p, mut u, mut t) = (pos, vel, 0.0);
        let dt = 0.001;
        while p.z > 0.0 {
            let s = u.length();
            u += (gravity_vec(p.z) - u / s * drag_accel(&v, p.z, s, mass)) * dt;
            p += u * dt;
            t += dt;
        }
        assert!(
            (coarse.point.x - p.x).abs() < 50.0,
            "{} vs {}",
            coarse.point.x,
            p.x
        );
        assert!((coarse.time - t).abs() < 0.5);
    }

    #[test]
    fn wind_carries_the_impact_point_downwind() {
        let v = Vehicle::nominal();
        let (pos, vel) = (DVec3::new(0.0, 0.0, 5000.0), DVec3::new(0.0, 0.0, -250.0));
        let fall = |wind| impact(&v, pos, vel, &wind, v.dry_mass, EntryBurn::Done).point;
        let drift = fall(Wind::steady(DVec2::new(10.0, 0.0))) - fall(Wind::default());
        assert!(drift.x > 10.0 && drift.y.abs() < 1e-6, "{drift:?}");
    }
}
