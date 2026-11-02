//! Spring-damper contact with the landing surface: flat ground at z = 0, or the moving
//! deck of the drone ship, beside which is the sea.

use glam::DVec3;

use crate::dynamics::State;
use crate::ship::{DECK_HALF_LENGTH, DECK_HALF_WIDTH, DeckPose, FREEBOARD};
use crate::vehicle::Vehicle;

const STIFFNESS: f64 = 1.5e6;
const DAMPING: f64 = 2e5;
/// Per leg: the shock absorbers give about 0.2 m under the weight of a near-empty booster
/// and more at touchdown, overdamped so the vehicle does not bounce.
const LEG_STIFFNESS: f64 = 4e5;
const LEG_DAMPING: f64 = 1.5e5;
const FRICTION: f64 = 0.6;
/// Viscous cap on friction so the force fades smoothly as slip goes to zero.
const FRICTION_DAMPING: f64 = 2e5;
/// A point further than this under the deck has gone past its edge, not into it.
const MAX_DECK_PENETRATION: f64 = 2.0;

#[derive(Clone, Copy, Debug, Default)]
pub struct ContactLoads {
    /// World-frame force.
    pub force: DVec3,
    /// Body-frame torque about the CoM.
    pub torque: DVec3,
    pub legs_touching: usize,
    pub hull_touching: usize,
    /// Leg and hull points under the sea surface.
    pub in_water: usize,
}

/// The surface a point can push on: its outward normal, how deep the point is in it, and
/// the surface's velocity there.
struct Touch {
    normal: DVec3,
    depth: f64,
    surface_vel: DVec3,
}

fn touch(point: DVec3, deck: Option<&DeckPose>) -> Option<Touch> {
    match deck {
        None => (point.z < 0.0).then_some(Touch {
            normal: DVec3::Z,
            depth: -point.z,
            surface_vel: DVec3::ZERO,
        }),
        Some(deck) => {
            let local = deck.att.inverse() * (point - deck.origin);
            let on_deck = local.x.abs() <= DECK_HALF_LENGTH && local.y.abs() <= DECK_HALF_WIDTH;
            (on_deck && (0.0..MAX_DECK_PENETRATION).contains(&-local.z)).then(|| Touch {
                normal: deck.att * DVec3::Z,
                depth: -local.z,
                surface_vel: deck.velocity_at(point),
            })
        }
    }
}

/// Loads from every leg foot (when deployed) and hull point touching the surface: the
/// deck if `deck` is given, otherwise the ground.
pub fn loads(
    vehicle: &Vehicle,
    state: &State,
    legs_deployed: bool,
    deck: Option<&DeckPose>,
) -> ContactLoads {
    let mut out = ContactLoads::default();
    let mut add = |offset: DVec3, stiffness: f64, damping: f64| -> bool {
        let point = state.pos + state.att * offset;
        if deck.is_some() && point.z < -FREEBOARD {
            out.in_water += 1;
        }
        let Some(touch) = touch(point, deck) else {
            return false;
        };
        let vel = state.vel + state.att * state.omega.cross(offset);
        let force = surface_force(&touch, vel, stiffness, damping);
        out.force += force;
        out.torque += offset.cross(state.att.inverse() * force);
        true
    };
    let legs = if legs_deployed {
        vehicle
            .leg_feet(state.mass)
            .into_iter()
            .filter(|&p| add(p, LEG_STIFFNESS, LEG_DAMPING))
            .count()
    } else {
        0
    };
    let hull = vehicle
        .hull_points(state.mass)
        .into_iter()
        .filter(|&p| add(p, STIFFNESS, DAMPING))
        .count();
    out.legs_touching = legs;
    out.hull_touching = hull;
    out
}

/// Spring-damper push along the surface normal plus capped friction against the slip.
fn surface_force(touch: &Touch, vel: DVec3, stiffness: f64, damping: f64) -> DVec3 {
    let relative = vel - touch.surface_vel;
    let closing = relative.dot(touch.normal);
    let normal = (stiffness * touch.depth - damping * closing).max(0.0);
    let slip = relative - touch.normal * closing;
    let slip_speed = slip.length();
    let friction = if slip_speed > 1e-9 {
        -slip / slip_speed * (FRICTION * normal).min(FRICTION_DAMPING * slip_speed)
    } else {
        DVec3::ZERO
    };
    friction + touch.normal * normal
}
