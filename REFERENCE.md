# Rocket Lander — Technical Reference

A Falcon-style booster return, to a landing pad or a drone ship, simulated in Rust: 6-DOF truth physics, flight software
(navigation, guidance, control) that only sees noisy sensors, a headless Monte Carlo
runner, and a Bevy mission-control viewer with live telemetry and replay.

## Workspace

| Crate | Role | Depends on |
|---|---|---|
| `crates/sim` | Truth world: vehicle, rigid-body dynamics, atmosphere, wind, aero, contact with the ground or the drone ship, actuators, sensors, dispersions | — |
| `crates/fsw` | Flight software: navigation, phase state machine, guidance (predictor, aero steering, G-FOLD, terminal MPC), attitude control and allocation | `sim` (only the `Vehicle` model, `SensorFrame`, `Commands`, `State` for the initial handover, and shared environment/aero formulas) |
| `crates/mission` | Closed loop (`Flight`), flight log, landing judgement, Monte Carlo, `lander` CLI | `sim`, `fsw` |
| `crates/app` | Bevy viewer: 3D scene, gizmo overlays, egui telemetry panels, replay, Monte Carlo window, sound, video export. `blender/booster.py` generates the embedded booster model | all of the above |

Key libraries: `glam` 0.33 (f64 math in sim/fsw, f32 in logs and Bevy), `nalgebra` (Kalman
filter matrices), `clarabel` (conic solver for G-FOLD), `rayon` (Monte Carlo), Bevy 0.20,
`bevy_egui` 0.43 and `egui_plot` 0.37 (egui 0.36).

## Running

```bash
cargo run --release -p app                                   # viewer
cargo run --release -p mission --bin lander -- fly full      # one flight, prints the event timeline
cargo run --release -p mission --bin lander -- fly landing --seed 7 --dispersed --save flights/x.json
cargo run --release -p mission --bin lander -- mc full --runs 100   # Monte Carlo summary + failures
cargo run --release -p mission --bin lander -- mc landing --stress 1 --require 80   # exit 1 below 80% landed
cargo run --release -p mission --bin lander -- sweep full --runs 200   # landing rate vs stress 0..2
cargo run --release -p mission --bin lander -- mc ship --runs 100       # drone ship landings
cargo run --release -p app -- --record out.mp4 landing --camera auto --hour 20   # video (needs ffmpeg)
cargo test --workspace
blender -b --factory-startup --python crates/app/blender/booster.py   # regenerate the booster model
```

`[profile.dev]` uses `opt-level = 1` for workspace crates and `3` for dependencies; debug
builds are usable, but the viewer and Monte Carlo are much faster in `--release`.

## Conventions

- **World frame:** x east, y north, z up (m). Origin is the landing pad center on flat ground at z = 0.
- **Body frame:** origin at the center of mass (CoM), +z from the engines toward the nose.
  `State::att` rotates body to world. Angular velocity is in the body frame.
- **Bevy frame:** y up. `scene::to_bevy` maps sim (x, y, z) to Bevy (x, z, -y), and
  `rotation_to_bevy` conjugates attitudes by the same basis change. The booster mesh is
  built in that mapped body frame (long axis = local +Y).
- **Units:** SI throughout; angles in radians in code, degrees only in UI and printouts.
- **Timing:** physics RK4 at 500 Hz (`PHYSICS_DT = 0.002`), flight computer and log at
  50 Hz (`TICK = 0.02`, 10 physics steps per tick).
- **Determinism:** a `FlightConfig { scenario, seed, dispersed, stress }` fully determines a
  flight. All randomness (dispersions, sensor noise, gusts) comes from
  `StdRng::seed_from_u64(seed)`; the stress draws and the sea state come from separate
  streams (`seed ^ STRESS_STREAM`, `seed ^ SEA_STREAM`), so adding stress leaves a seed's
  standard dispersion and sensor noise unchanged.

## sim

### Vehicle (`Vehicle::nominal()`)

Falcon-like first stage: dry mass 25.6 t, length 40 m, radius 1.85 m, center of pressure
19 m above the base. Inertia is a solid cylinder scaled by the current mass.

- **Center of mass:** `com_height(mass)` blends the empty stage's (15.3 m above the base)
  with the propellant's (10 m, settled low in the tanks), so it rises from 12.4 m with 32 t
  aboard to 15.3 m empty. The body frame is centered on it, so every offset from it
  (thrust point, center of pressure, fins, legs, hull points) takes the current mass.

- **Engines:** each produces 914 kN in vacuum and 845 kN at sea level, with vacuum Isp 311 s
  and a minimum throttle of 40%. Throttle follows a first-order lag (τ = 0.15 s). Nozzle
  losses scale with ambient pressure. Flow is `level × T_vac / (Isp g0)`. Engines run in
  groups of 0, 1 or 3: the center engine, or it plus the outer pair on the body x axis
  (ring radius 1.2 m). Thrust stops when propellant runs out.
- **Engine out:** if the outer engine at body −x fails, the three-engine group runs on two
  (`Actuators::engines` = 2) and the thrust acts halfway out toward +x
  (`Vehicle::thrust_point`), torquing the body.
- **Gimbal:** ±6°, slews at 20°/s. All engines gimbal together about a point at the base.
- **Grid fins:** at 38 m; deploy over 2 s. ±20°, slew 40°/s. Modeled as ideal lateral-force
  generators (`q × 4 m² × 2/rad × deflection`) plus a roll moment. Effective only once
  fully deployed.
- **RCS:** cold-gas torque of up to 150 kN·m about pitch/yaw and 30 kN·m about roll.
  Pure couple, no propellant.
- **Legs:** 4 feet at 9 m radius, 1.5 m below the base when deployed; deploy over 2 s.
  `standing_height(mass)` = `com_height(mass)` + 1.5 m is the CoM height when standing
  (16.8 m empty, before the shock absorbers give).

### Dynamics and environment

- `dynamics::rk4` integrates position, velocity, attitude quaternion, body rates and mass.
  Euler's equations include the gyroscopic term. Inputs are held constant over a step.
- `env`: inverse-square gravity; the 1976 US Standard Atmosphere (temperature, pressure,
  density and speed of sound from its seven layers by geopotential height, extended
  isothermally above 86 km); and wind:
  - Steady part: the 10 km reference wind times `wind_profile` (a power law below 10 km,
    exponent 0.2 nominally; decaying above). Its direction turns by `veer` (radians)
    linearly from 10 km down to the ground.
  - Plus first-order Gauss–Markov gusts (τ = 2 s).
  - `Wind::steady(reference)` is the nominal shape without veer or gusts.
- `aero::loads`: axial drag `C_A` (1.0 base-first, 0.5 nose-first, times
  `mach_drag_factor`: 1 below Mach 0.6, peaking at 1.5 at Mach 1.1, easing to 1.15 by
  Mach 5), plus a normal force
  `C_N(α) = 2 sinα cosα + 10 sin²α` acting at the center of pressure. Base-first descent
  is weathervane-stable (tested). Fin forces act at the fin station.
- `contact`: spring–dampers with capped Coulomb friction (μ = 0.6):
  - Leg feet (only when fully deployed): 0.4 MN/m and 0.15 MN·s/m each, standing in for
    the crush-core shock absorbers. A near-empty booster sags about 0.17 m, overdamped.
  - 8 hull points (base and nose rims): 1.5 MN/m and 0.2 MN·s/m.
  - The surface is flat ground at z = 0, or the drone ship's deck (see below): a 90 × 52 m
    plane moving with the ship, pushing along its normal and rubbing against the
    velocity relative to it. A point more than 2 m under the deck plane has gone past its
    edge. Any point more than 4 m below z = 0 near the ship is in the sea.

### Drone ship (`ship.rs`)

- A barge holding station at the origin. Its deck heaves, pitches and rolls as three
  sinusoids (`Swell`): nominally 0.5 m over 8 s, 1° over 7 s and 2° over 9.5 s. The deck's
  mean height is z = 0 and the sea surface is 4 m below (`FREEBOARD`).
- `Ship::pose(t)` gives the deck's origin, attitude, velocity and angular velocity;
  `World::ship` holds the ship (`None` lands on the ground).
- Dispersed flights draw the sea state: each amplitude ×0.5–1.5 and period ×0.8–1.2,
  random phases, and amplitudes ×(1 + stress).

### Actuators and commands

`Commands` is the hardware interface: engines lit, per-engine throttle (clamped to
`min_throttle..=1` by the hardware), gimbal angles, fin deflections, RCS torque, and
deploy flags. `World::step` applies the lags and rate limits into `Actuators`. A fixed
thrust misalignment (from the dispersions) is added to the gimbal.

### Sensors (`SensorFrame`, produced once per tick by `World::sense`)

| Sensor | Model | Rate |
|---|---|---|
| IMU accelerometer | Mean specific force over the tick (body), + fixed bias (σ 0.02 m/s²) + noise (σ 0.05) | 50 Hz |
| IMU gyro | Mean body rate + fixed bias (σ 2e-4 rad/s) + noise (σ 5e-4) | 50 Hz |
| GPS | Position σ 2 m, velocity σ 0.1 m/s | 10 Hz |
| Radar altimeter | CoM altitude σ 0.3 m; valid below 3 km and with tilt < 30° | 20 Hz |
| Star tracker | Attitude, σ 5e-4 rad per axis | 10 Hz |
| Leg contact | Any foot touching | 50 Hz |
| Engine health | `engine_out` once an engine has failed | 50 Hz |
| Deck telemetry | The drone ship's deck center position and velocity, exact | 50 Hz |

Faults (stress only): GPS goes silent for a while once the vehicle descends through a set
altitude, and the accelerometer bias jumps once it descends through another.

### Dispersions (`Dispersion::sample`, Monte Carlo only)

- Mass and propulsion: dry mass ±2%, Isp ±0.5%, thrust ±1% (all 1σ).
- Aerodynamics: drag and normal-force coefficients ±5% (1σ).
- Thrust misalignment: 0.1° per axis (1σ).
- Wind: 10 km reference wind 0–15 m/s from any heading; gust σ 0–2 m/s.
- Starting state: position error σ 100 m and velocity error σ 3 m/s per axis.

`Dispersion::stressed(severity)` adds harsher conditions on top (severity 0 changes
nothing; 1 is a bad day). Each scales with the severity s:

- Weather: the reference wind grows by up to 50%·s, gust σ by up to 2s m/s, the profile
  exponent moves by up to ±0.15s, and the wind veers by up to ±45°·s.
- Throttle: the time constant grows by up to 100%·s.
- Faults, each with probability s/2:
  - The outer engine at −x fails 0–8 s into three-engine firing while descending (the
    entry burn).
  - A GPS outage of 5–15·s seconds starting between 300 m and 4 km.
  - An accelerometer bias jump of 0.05·s m/s² between 300 m and 4 km.

The flight software always assumes the nominal vehicle and starts from the nominal state.

## fsw

`FlightComputer::step(&SensorFrame) -> Commands` runs, in order: engine/propellant model,
navigation, guidance (phase state machine), then attitude control and allocation.

### Navigation (`nav.rs`)

- **Attitude:** integrate the gyro, then blend in each star-tracker fix with gain 0.05
  (a complementary filter).
- **Translation and wind:** an 11-state Kalman filter over position, velocity,
  accelerometer bias, and the horizontal wind at the 10 km reference altitude.
  - Prediction: IMU specific force rotated by the attitude estimate, plus gravity. The wind
    is a random walk: 0.5 m/s per √s at the local altitude, mapped to the reference through
    the nominal profile.
  - Updates: GPS (6 measurements) and the altimeter (1), using the Joseph-form covariance
    update.
  - Crossflow update (`correct_crossflow`, from the flight computer): the body-frame sideways
    air velocity, modeled as the sideways part of `R⁻¹(v − profile(h)·w_ref)`. It is a
    Schmidt update that only corrects the wind states: its errors are slow and correlated,
    and letting it move velocity corrupted navigation during GPS outages.
- `NavState::wind` is the estimate as a `Wind::steady` (nominal profile), so the wind at
  any altitude follows from it; `local_wind()` is the wind at the current altitude.
- Initial estimate: the nominal start state, with σ 150 m in position, 5 m/s in velocity
  and 10 m/s in the reference wind.
- `NavState::pos_sigma` is the root of the position covariance trace.

### Actuator model

The flight software keeps its own copy of the actuators: the throttle lag
(`thrust_level`, which counts two engines in the three-engine group after an engine out)
and the gimbal and fin slew rates. It uses the modeled thrust for gimbal allocation and to
dead-reckon propellant, and the modeled positions to read the side force.

### Crossflow reading (`FlightComputer::read_crossflow`)

- Runs in AeroDescent and LandingBurn while the air-relative dynamic pressure is ≥ 500 Pa.
- The body-frame side force is the accelerometer reading (minus the estimated bias) times
  mass, minus the modeled thrust (through the modeled gimbal) and fin forces. What remains
  is body normal force.
- Inverting `C_N` (up to 30°) gives the angle of attack. The normal force points away from
  the crossflow, so this gives the air-relative sideways velocity in the body frame.
- Its noise is a 0.3 m/s² sideways force error mapped through the slope of
  `C_N` (σ = V·m·0.3 / (q S C_N′(α)) + 0.3 m/s), so it counts for little at low dynamic
  pressure.
- Guidance and control fly relative to the air (`est.vel − local wind`): steering, the
  landing plan's flow line and drag, thrust shaping, fin gain and the aero torque
  feedforward. The predictor and G-FOLD use the wind at each altitude.

### Phases (`computer.rs`)

| Phase | Engines | Attitude target | Exit |
|---|---|---|---|
| Flip | off | horizontal heading from the predicted landing point toward the pad | aligned within 5° and rates < 1°/s |
| Boostback | 3 at full, then 1 at minimum throttle once the predicted miss is < 3 km | same heading | miss < 50 m or the predicted point passes the pad; fins deploy |
| Coast | off | retrograde (engines into the flow) | below 45 km and descending: EntryBurn if speed > 450 m/s, otherwise AeroDescent |
| EntryBurn | 3 at full | retrograde | speed < 450 m/s |
| AeroDescent | off | retrograde leaned for body lift (see below) | below the ignition altitude with a feasible G-FOLD plan, looked for every 0.2 s (or forced at 80% of it) |
| LandingBurn | 1 | from the shaped thrust command | leg contact, or the engine cuts 0.5 m above leg-contact height (one engine at minimum throttle would otherwise hover a near-empty booster), or within 5 m of it if the booster starts climbing again; legs deploy below 500 m |
| Landed | off | — (all actuators idle) | — |

### Guidance

- **Landing-point predictor** (`predict.rs`): a point mass with base-first drag at 0.2 s steps.
  Drag acts on the velocity relative to the estimated wind at each altitude; burns stay
  ground-retrograde.
  - It flies the same phase logic as the flight computer: an entry burn of 3 engines
    retrograde until 450 m/s, then a landing burn modeled as retrograde thrust at 70%
    throttle from the ignition altitude.
  - It returns where the vehicle stops, the time to get there, and the time until ignition.
  - Boostback aims this point at the pad.
- **Ignition altitude:** the stopping height for a single engine at 70% throttle, plus half
  of the current drag, minus gravity.
- **Aero steering** (`guidance::steering_axis`, given the air-relative velocity):
  - Zero-effort-miss guidance (gain 3) on the predicted landing point, with time-to-go equal
    to the time until ignition.
  - The required sideways acceleration becomes an angle of attack off the air-relative
    retrograde by inverting `C_N` (capped at 15°).
  - The nose leans *away* from the desired direction (body lift pushes opposite the lean).
- **G-FOLD** (`gfold.rs`): a convex landing plan solved with Clarabel. 30 intervals, with
  variables r, v, u = T/m and σ ≥ |u| at each node.
  - **Dynamics:** trapezoidal, with linear drag `−k_k (v − w_k)` per node, where `w_k` is
    the estimated wind at that node's expected height (height falling with the square of
    the remaining speed). Speed is assumed to fall linearly over the burn (`v_k = v₀(1 − k/30)`),
    and `k_k = D(v_k)/v₀`: the drag at that node's speed, scaled down by the fraction of
    speed left so the plan never leans on drag that fades sooner than assumed. (A constant
    rate overestimates late drag; matching `D(v_k)/v_k` exactly was less robust.)
  - **Thrust bounds:** σ ≥ T_min/m and σ ≤ T_max/m (lossless convexification). The mass
    profile is bounded conservatively instead of being a variable. The plan stays inside
    110%–85% of the real throttle range. The last 3 nodes are capped at 1.2× the minimum
    for a gentle touchdown.
  - **Pointing cones:** a tilt limit from vertical (25°, narrowing over the last 8 nodes to 3°),
    and a cone about the starting air-relative retrograde direction sized from the lateral authority
    available at each node's estimated dynamic pressure (`guidance::aero_angle`).
  - **Glideslope:** a 70° elevation cone above the target.
  - **Endpoint:** 0.3 m below standing height at −1.5 m/s, with horizontal thrust zero at the
    final node.
  - **Objective:** Σσ·dt (fuel) + 0.05·Σ|u_{k+1} − u_k|² (smoothing). That makes it a QP
    with second-order cones.
  - **Final time:** free at ignition (a 12-point scan, then golden-section refinement).
    Re-solved every 0.5 s with the remaining time (trying ±5%/±10%). Replanning stops 2 s
    before the plan ends.
- **Tracking:** plan feedforward plus LQR feedback on the position/velocity error.
  - Gains: vertical Bryson (2 m, 1 m/s, 3 m/s²); horizontal (5 m, 2 m/s, 2 m/s²).
    Horizontal is deliberately slower than the attitude loop.
  - `guidance::shape_thrust` brakes along the flow line (air-relative retrograde blended
    toward vertical as dynamic pressure falls).
  - It delivers the sideways part by leaning. While thrust dominates (`T > c1 q S`) it leans
    toward the target, up to where net force peaks. While body lift dominates it leans
    away, up to 15°.
  - Below 200 m above touchdown, terminal guidance takes over the sideways motion (see
    below); the vertical keeps following the plan.
  - On a drone ship, below 15 m above the deck the vertical command follows the deck
    instead: a sink rate relative to it of `sqrt(1.5² + 2·2·h)` m/s, tracked at 2 /s on
    top of gravity plus 2 m/s², so the legs meet a heaving deck gently.
  - A final tilt cone narrows from 25° at 80 m above touchdown to 3° at touchdown.
- **Terminal guidance** (`terminal.rs`): model-predictive control of the sideways motion.
  - Each horizontal axis on its own: position, velocity, tilt and tilt rate, with
    `v' = a·tilt` (a = the vertical thrust acceleration) and the closed attitude loop
    `tilt'' = kp (u − tilt) − kd tilt'` (kp, kd from `control::attitude_gains` at the current
    authority and lag). Zero-order hold at 0.1 s.
  - Horizon: the time to go at a steady braking rate (2h/|v_z|), 5–80 steps. The tilt
    command at each step is boxed by the tilt limit at that step's expected height.
  - Cost (Bryson scales): running position 4 m and velocity 3 m/s per second; at
    touchdown position 2 m, velocity 0.2 m/s, tilt 0.01 rad and tilt rate 0.05 rad/s;
    tilt command 0.3 rad per second and 0.03 rad change per step.
  - States are eliminated (a condensed QP in the commands only) and Clarabel solves it
    every tick; the first command becomes the sideways thrust acceleration `a·tan(u)`. If
    it fails, the sideways velocity is just damped at 0.8 /s.
  - It replaced a velocity-only rule below 40 m. Starting it higher (400 m) did worse:
    its model has no aerodynamics.
- **Fallback:** if no plan exists, brake to touchdown speed at touchdown height and damp drift over the pad.

### Control (`control.rs`)

- Each axis is a double integrator driven through a first-order actuator lag:
  state (angle, rate, delivered angular acceleration). The gains come from a discrete LQR
  at the 50 Hz tick (zero-order hold) with Bryson weights (attitude 0.1 rad, rate
  0.2 rad/s, input = the current authority, no weight on the actuator state), solved by
  Riccati iteration.
  - `attitude_gains` tabulates them once (37 authorities, log-spaced 0.005–20 rad/s²,
    × 9 lags, 0.01–0.41 s) and interpolates.
  - `actuator_lag`: each effector's time to slew to half deflection (gimbal 0.15 s, fins
    0.25 s, thrusters 0), weighted by its share of the authority, plus half a tick.
  - The flight computer models the delivered acceleration with the same lag and feeds it
    back (`ka`).
  - The law runs in cascaded form, with the rate request capped at `sqrt(α_max |e|)` so
    large turns brake in time.
  - Roll is only damped: the attitude target is the minimal rotation of the nose.
  - Stiffer weights (attitude 0.05 rad) landed less often.
- Torque = I·α + ω×Iω − predicted body aero torque (feedforward, from the air-relative
  velocity) − the off-center push of a two-engine group after an engine out.
- **Allocation:** gimbal first (only above 50 kN thrust), then grid fins (once deployed and
  q > 100 Pa), then RCS for the remainder, each saturating.

## mission

- **`Flight::tick`:** sense → `fsw.step` → record the frame (truth and estimate at the same
  instant) → 10 physics steps → judge.
- **`FlightLog`:** one `Frame` per tick (f32: truth, estimate including the wind estimate,
  actuators, aero, predicted impact, guidance acceleration command), plus `events` (phase changes, deployments,
  touchdown, outcome), `plans` (each adopted G-FOLD trajectory with its adoption time),
  and `outcome`.
  - Save and load as JSON.
  - `frames_around(t)` and `plan_at(t)` serve replay.
- **Judgement:** the first ground contact records a `Touchdown` from truth. On the drone
  ship, its position and speeds are relative to the deck.
  - Any hull contact is an immediate crash, and so is any point in the sea ("fell into
    the sea").
  - Otherwise the flight ends 5 s after touchdown and is graded against the limits:
    vertical ≤ 4 m/s, horizontal ≤ 2 m/s, tilt ≤ 5°, tilt rate ≤ 10°/s, within 15 m of the
    pad center. The first violated limit becomes the verdict.
  - Flights time out at 900 s.
- **Fault events:** "Engine out", "GPS lost" / "GPS back", and "Accelerometer bias
  jump" join the event log when they happen.
- **Scenarios:**
  - `Landing`: 5 km, falling at 230 m/s base-first, 270 m off the pad, 4.5 t of propellant,
    fins deployed, starting in AeroDescent.
  - `FullReturn`: separation at 70 km, 35 km downrange, (750, 0, 650) m/s, 32 t of
    propellant, starting in Flip.
  - `DroneShip`: the same separation state, but with the ship 140 km downrange, near
    where the booster falls without a boostback (a short retro trim remains), and 26 t of
    propellant (missions that land at sea keep less for the way back). `at_sea()`.
- **`FlightLog::ship`:** the drone ship, so replays can pose the deck.
- **`montecarlo::run`:** dispersed flights at a stress level with consecutive seeds, run in
  parallel with rayon. Logs are dropped; only outcomes are kept. `summarize` gives the
  success count and miss-distance statistics.
- **`lander` CLI:** `fly`, `mc` (`--require P` exits with an error below P% landed, for CI)
  and `sweep` (stress 0, 0.5, 1, 1.5, 2 with the success rate and most common failure).

## app

- **`playback::Playback`:** the single cursor (flight time) that every view reads.
  `simulate_to(t)` ticks a live flight until its log covers `t`.
  - A `Source::Live(Flight)` is ticked until its log covers the cursor, within a 25 ms
    per-frame budget. The log, not the flight clock, is the bound: the newest frame trails
    the flight's own time by one tick.
  - A `Source::Recorded(FlightLog)` is only scrubbed.
  - `frame()` blends neighboring frames for smooth motion.
- **`scene`:** the world.
  - Sun: `RAW_SUNLIGHT` with a sun disk and cascaded shadows. The `TimeOfDay` resource
    (0–24 h, default 9.5) places it: rising in the east at 6, 60° high in the south at
    noon, setting in the west at 18, as far below the horizon at midnight. The atmosphere
    reddens low sun on its own.
  - Night (`update_sky`): `darkness` goes from 0 to 1 as the sun sinks from 2° above the
    horizon to 6° below. With it the sun's light fades (the sky keeps a twilight glow from
    what is left), the exposure opens from EV 13 to 8.5, four floodlights (spot lights on
    the light masts, aimed at the pad) and their lamp heads come on, and 2,500 stars come
    out. The stars are additive quads on a 1,000 km sphere centered on the camera, hidden
    under a 0.3 rad field of view so the telephoto pad camera does not blow them up.
  - Sky: Bevy's physically based `Atmosphere::earth`, which also gives aerial-perspective
    haze. The planet center sits 6,360 km below the origin.
  - The landing zone: apron, pad with markings, access road, buildings and floodlight masts.
- **`terrain`:** procedural ground (visual only; the physics uses flat ground).
  - A 1,200 km disc that follows the planet's curvature: dense rings near the pad,
    geometric growth outward, and counter-clockwise winding so it faces up.
  - Vertex colors: fields, forest and beach, from value noise.
  - A coastline about 4 km east of the pad. The seabed shelves down from it at 5% to below
    a separate glossy ocean disc, so the waterline is smooth.
  - The sea sinks slightly with distance to avoid depth fighting at the horizon.
  - A tiling 400 m grayscale detail texture.
- **`sea`:** the drone ship's surroundings.
  - While the log has a ship, the land, the pad and its floodlights (`LandSite`) are hidden,
    the ocean disc drops 3 m to the ship's waterline, and the ship and swell (`SeaSite`)
    show.
  - The ship (hull, deck with target markings, blast walls at both ends) is posed from
    `Ship::pose` at the cursor.
  - The swell is a 2.4 km grid around it, displaced every frame by three deep-water wave
    trains scaled from the ship's heave (period, and wavelength by `ω² = g k`), fading to
    flat at its edge.
- **Booster model:** `blender/booster.py` builds it in Blender and exports
  `src/models/booster.glb`. `main` embeds the GLB with `embedded_asset!`
  (`embedded://app/models/booster.glb`), so the binary runs from any folder. The `.glb`
  is committed; Blender is only needed to regenerate it.
  - **Geometry:**
    - Lathed body with streaky soot vertex colors. The soot is exported as COLOR_0 because
      the material reads it through a color-attribute node.
    - Panel seams, a hollow interstage with a tank dome inside, raceway and thruster pods.
    - Octaweb with nine flared, double-sided bells.
    - Lattice grid fins, and tapered carbon legs whose foot pads come out level when deployed.
  - **Interface with the viewer:** the empties `engines`, `fin_0..3` and `leg_0..3`, with
    their pivots and rest poses (fins deployed, legs stowed). Body dimensions, the 2.0 m
    nozzle-exit depth, the engine ring and the leg geometry must match `booster.rs`
    and `sim::Vehicle::nominal()`.
- **`booster`:** spawns the model under the `Booster` root and poses its parts.
  - `bind_parts` tags the named empties as the glTF scene spawns: `EngineCluster`, and
    `GridFin`/`Leg` with their rest rotations. It attaches what the model lacks: plumes
    on the engine pivot, and a pusher strut per leg.
  - The model's origin is 15 m above its base (`MODEL_ORIGIN_HEIGHT`, matching the
    script), so the root is placed that far up the body from the base, which sits
    `com_height(mass)` below the center of mass.
  - `sync` drives the parts:
    - Gimbal: rotation from the gimbal angles.
    - Fins: `rest * fold * deflection`.
    - Legs: `rest * swing`.
    - Struts: re-aimed each frame from 6.5 m up the body to 55% along the leg.
  - Lit engines: 1 → center; 3 → center plus an opposite pair; 2 (engine out) → center
    plus the survivor of the pair.
  - Legs back off their swing by however far their feet would sink below the ground (or
    the deck), so they visibly give as the shock absorbers take the weight.
  - Per-engine plumes: a glow cone plus a core cone, as capless meshes with polar UVs, a
    radial fade texture, and emissive-only lit materials so bloom picks them up.
    - They lengthen with throttle, balloon with altitude, and flicker.
    - Four shock diamonds trail the core while the air is still thick.
  - `glow` drives two additive shells (surfaces of revolution with a linear fade texture):
    - Envelope: when the engines fire into fast, thin air (q / ambient pressure from 1 to
      5, nose against the velocity), a glowing bulb grows from below the nozzles up the
      body and the plumes squash. In practice this is the entry burn.
    - Heat glow: the engine section glows after the entry burn, heating over 8 s of burn
      and cooling with τ = 40 s, timed from the log's phase events.
  - `glow` also sets a point light 5 m below the nozzles (4·10⁷ lm per full-throttle
    engine, no shadows) that lights the booster, the pad and the smoke, mostly noticeable
    at night.
- **`particles`:** soft camera-facing billboards with a noisy puff texture, fading through
  a ladder of shared materials. They are lit (so the plume light colors the smoke at
  night) but cast no shadows, and glow at 70% of their color by daylight
  (`particles::daylight`) so they do not go dark against the sun.
  - Exhaust smoke below 25 km, dust when the nozzles are below 45 m, cold-gas puffs
    when the RCS torque exceeds 20 kN·m.
  - Driven by cursor time: they pause with playback and clear on a jump. Capped at 800.
- **`camera`:** two cameras.
  - The 3D camera has five modes (`CameraMode`):
    - Chase: orbits the booster (drag to rotate, scroll to zoom; kept above the ground).
    - Pad: tracks it from a site 1.2 km south-west of the pad with a telephoto lens whose
      field of view keeps an 80 m frame around the booster.
    - Engine and Booster: onboard, between two legs, looking down the body with an 80°
      lens: from a bracket 1.6 m out at 4 m up, leaning in toward the plume, and from the
      skin at 31 m up. They shake with thrust and dynamic pressure.
    - Auto: a director that cuts by phase and altitude: chase high up, the engine camera
      in the entry burn, the body camera above 3 km in the fall, then the pad camera, with
      the engine camera again between 1 km and 150 m in the landing burn.
    - `Orbit::active` is the view actually shown.
  - It renders in HDR with atmosphere settings (aerial view to 200 km), atmosphere-driven
    environment light, exposure EV100 13, ACES tonemapping and bloom.
  - A second camera renders only the egui panels over the whole window (bevy_egui's
    side-panel pattern). The 3D camera's viewport is fitted to the free area each frame.
- **`overlay`:** gizmos for the trail, velocity (cyan), thrust (red), the predicted impact
  ring, the current G-FOLD plan (magenta) and the pad ring. The `Overlays` resource
  toggles them.
- **`audio`:** the sound at the camera, synthesized (no audio files).
  - Engine roar: filtered noise generated on the audio thread (`Roar`, a custom
    `Decodable`), its cutoff and crackle set from the main thread through atomics. Sound
    travels at 340 m/s: `emission_time` finds, by bisection, when the sound now reaching
    the camera left the booster (the latest emission that has arrived, stepping back
    then bisecting, since a supersonic dive makes arrivals run backwards), and that
    frame's thrust and distance set the loudness
    (full within 300 m, then falling as distance^-0.8) and how muffled it is.
  - Sonic booms: for the pad camera site, the arrival time t + d/c of the booster's sound
    over the log has a local minimum wherever its approach drops below the speed of sound;
    a triple N-wave with an echo tail plays as the cursor passes each one, while the pad
    camera is the view.
  - Silent while paused; a Sound checkbox turns it off.
- **`record`:** `app --record out.mp4|out.gif [scenario] [--seed N] [--dispersed]
  [--stress S] [--camera auto|chase|pad|engine|booster] [--hour H] [--from T] [--to T]
  [--fps N] [--speed X] [--size WxH]`.
  - No panels, overlays or sound. The window is the video size at scale factor 1.
  - After an 8 s wall-clock warmup (shaders, model, sky), the cursor steps exactly one
    video frame per rendered frame and each frame is screenshotted to `out.frames/`.
  - Each frame also notes where the camera is and whether it is the pad camera.
  - At the end (the flight plus 3 s, or `--to`) it waits until every frame is on disk, then
    ffmpeg encodes H.264 MP4 with an AAC soundtrack, or a silent palette GIF (12 fps,
    640 px wide), and the frames are deleted. Without ffmpeg the frames stay.
  - The soundtrack (`audio::soundtrack`) uses the live synthesis offline: the roar's
    loudness, cutoff and crackle at each frame's camera, blended across the frame, plus a
    boom wherever an arrival passes while the pad camera is the view.
- **`ui`:** a mission-control layout.
  - Top: status strip.
  - Right: linked-time telemetry plots, decimated to ≤ 1500 points, with a cursor line.
  - Bottom: play/pause, speed, timeline slider, scenario/seed/dispersed/stress, Fly,
    Save/Load (`flights/*.json`), camera mode, overlay and sound toggles, time of day,
    Monte Carlo toggle, and a clickable event log.
  - Plots include wind speed, truth vs. the flight computer's estimate.
  - The timeline slider only moves the cursor on a drag or click. Otherwise the slider's
    own value nudging would fight playback.
  - **Monte Carlo window:** runs a batch at the chosen stress on a background thread, then
    shows the success rate, a touchdown scatter around the pad, touchdown speeds against
    the landing limits, and failed seeds with Replay.

## Invariants

- The flight software never reads `World::state`. Its only inputs are `SensorFrame`s and
  its initial handover state.
- Frame `k` holds truth and estimate for the same instant `t_k`, and the commands issued then.
- A flight is a pure function of its `FlightConfig`.
- Every finished flight has an `outcome`. `Flight::tick` is a no-op after that.

## Tests

- **`sim`:** free fall, rocket equation, resting on legs, angular momentum conservation,
  IMU at rest, aero stability, Mach drag rise, the standard atmosphere against its
  tables, engine out (two engines, off-center torque), wind veer, deck rates.
- **`fsw`:** terminal MPC arrives slow and upright and respects its tilt limit,
  LQR solves the Riccati equation, the lagged LQR reduces to it as the lag
  vanishes and settles a lagging axis, the gain table matches the LQR, allocation
  reproduces the requested torque, attitude command sign, Kalman filter convergence,
  crossflow reveals the wind, predictor vs. fine integration,
  predictor wind drift,
  G-FOLD (constraints, lossless relaxation, infeasible when too late), steering sign,
  cone limit.
- **`mission`:** nominal Landing, FullReturn and DroneShip land (FullReturn passes through
  every phase in order), a stressed FullReturn lands after an engine out in the entry burn, the wind
  estimate at ignition beats assuming calm air, determinism, log save/load, Monte Carlo
  bookkeeping.
- **`app`:** sound arrival delay, one boom for a supersonic dive and none subsonic, boom
  clip range, the soundtrack (silent until the boom, then the roar), frame conversion
  round trip, swell slopes.
- **CI** (`.github/workflows/ci.yml`): fmt, clippy with warnings denied, tests, and Monte
  Carlo gates (Landing 200 runs, Full return and Drone ship 100 runs each must land
  ≥ 95%).

## Measured performance

Reproduce with `lander mc <scenario> --runs N`. The nominal flights of all scenarios land.

| Scenario | Dispersed runs | Landed | Miss (mean / p95) |
|---|---|---|---|
| Landing | 300 | 100% | 1.9 / 5.5 m |
| Full return | 300 | 100% | 2.0 / 5.3 m |
| Drone ship | 300 | 99.7% | 1.0 / 2.8 m |

The wind estimate is off by about 1 m/s at landing-burn ignition (1.6 m/s at stress 1).
Terminal guidance trades a few meters of miss (well inside the 15 m pad radius) for
touchdowns without sideways speed.

Under stress (`lander sweep`, 200 runs per level):

| Stress | 0 | 0.5 | 1 | 1.5 | 2 |
|---|---|---|---|---|---|
| Landing | 100% | 100% | 97.5% | 88.5% | 70.0% |
| Full return | 100% | 99.5% | 98.0% | 90.0% | 73.5% |
| Drone ship | 99.5% | 97.0% | 97.0% | 88.0% | 48.5% |

- On land the failures are misses past 15 m: in strong wind the booster is still well off
  the pad when terminal guidance takes over at 200 m.
- At sea they are hard or sliding touchdowns on the heaving, rolling deck, which at
  stress 2 swells three times as high as nominal.

## Simplifications

- Flat Earth in the physics (the viewer draws the ground curved).
- Mach number only changes axial drag (not the normal force or fins).
- No plume–aero interaction in the physics (the viewer only draws it).
- The center of mass moves along the axis only; inertia is a cylinder scaled by mass.
- The drone ship holds station exactly and moves in three pure sinusoids; its deck
  telemetry is exact. The visual swell only resembles its motion.
- Fins are ideal force generators.
- RCS is continuous torque with no propellant.
- Attitude comes from a complementary filter rather than a full attitude EKF.
- The flight software assumes the nominal wind profile shape (no veer, exponent 0.2) and
  only estimates its strength and direction.
- Engine health is reported perfectly and at once.
