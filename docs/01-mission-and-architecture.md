# 1. Mission, requirements and architecture

## 1.1 The mission

Build a ~5 ft rocket that:

1. ignites on the pad and climbs to **~500 ft (150 m)**,
2. falls back, **lights a second motor near the ground and catches itself**,
3. **flips its landing legs down** just before touchdown, and
4. lands upright, reusable.

![Mission profile](img/mission_profile.svg)

| # | Event | Nominal value (simulation) | Trigger |
|---|---|---|---|
| 1 | Liftoff | 1.39 kg, T/W 5.5 peak / 3.2 average | Launch controller fires ascent motor |
| 2 | Ascent burnout | 2.1 s, ~65 m, 47 m/s | Accelerometer goes negative |
| 3 | Apogee | **152 m (498 ft)** at 6.3 s | Kalman-filter velocity < -0.5 m/s for 50 ms |
| 3a | Drogue out | 9 in drogue from the nose | Same event |
| 4 | Tail-first descent | ~22.5 m/s terminal | - |
| 5 | Landing-motor FIRE command | ~21 m altitude | Guidance predictor (every 5 ms) |
| 5a | Ignition | ~0.3 s later (scatter +/-0.08 s assumed) | Accelerometer jump |
| 6 | Legs flip down | 8 m | Altitude |
| 7 | Touchdown | **1.2 m/s** vertical (MC mean 1.5, p95 2.3) | Lidar / altitude |
| 8 | Safe | drogue cut, paddles closed, motor burns out | - |

Run it yourself: `python sim/scripts/run_nominal.py`.

## 1.2 Requirements

| ID | Requirement | Source / rationale | How it is met |
|---|---|---|---|
| R1 | Length ~5 ft | User | 60 in (1.524 m) |
| R2 | Apogee ~500 ft | User | 498 ft nominal; 138-169 m (p5-p95) |
| R3 | Propulsive landing with a "throttleable" motor | User | Solid landing motor + exhaust-paddle throttle (k = 0.25-1.0) + TVC |
| R4 | Legs that flip down near the ground | User | 4 spring legs released by a servo pin at 8 m |
| R5 | Liftoff mass <= 1500 g | FAA Class 1 / NAR model-rocket limit ([§101.22](https://www.ecfr.gov/current/title-14/chapter-I/subchapter-F/part-101/subpart-C/section-101.22)) | 1387 g (113 g margin) - **weigh everything** |
| R6 | Total propellant <= 125 g | Same | 107.6 g (2 x 53.8 g) |
| R7 | Total impulse <= 320 N·s, each motor < 62.5 g propellant, < 80 N average | NAR code, CPSC 16 CFR 1500.85 | 198 N·s; G40W is 53.8 g / 41 N |
| R8 | Touchdown <= 2.5 m/s vertical, <= 1.5 m/s lateral, <= 10° tilt | Leg stroke and tip-over sizing ([docs/07](07-landing-legs.md)) | 90% of 300 Monte Carlo flights; 100% in wind < 2 m/s |
| R9 | Fail-safe recovery | NAR safety code requires a recovery system | Backup 30 in main chute on abort; drogue always deployed |
| R10 | Landing motor can only ignite inside a safe envelope | Safety | Arm key + pad arming + in-flight envelope checks ([docs/06](06-gnc.md#67-safety-interlocks)) |

## 1.3 The hard problem, and the architecture that solves it

**Hobby solid motors cannot be throttled or shut down.** Once lit, a solid
motor follows its thrust curve until it burns out. Joe Barnard (BPS.space) took
seven years to land a model rocket this way. His first success, in July 2022,
was "a little more slam than hover", because "ignition variation and a fixed
burn time" forced hard touchdowns
([BPS on X](https://twitter.com/bps_space/status/1553103061135945728),
[Hackaday](https://hackaday.com/2022/08/05/bps-space-succesfully-lands-a-model-rocket/)).
What finally worked for him was a **mechanical throttle: ceramic "pincers" that
block part of the exhaust**, reportedly cutting up to 80% of thrust
([The New Stack](https://thenewstack.io/one-mans-dream-to-build-a-working-spacex-style-model-rocket/)).

The simulator in this repo reproduces the problem. A 0.1 s error in ignition
delay moves the landing altitude by ~2 m, which is enough to either stop in
mid-air and drop or hit hard. It then shows which design choices fix it:

![Ignition-delay sensitivity](img/ignition_delay_sensitivity.png)

### Options for the "thrustable" landing motor

| Option | Throttle? | Thrust-to-weight at ~1.4 kg | Mass penalty | Verdict |
|---|---|---|---|---|
| Estes F15 black powder | No | ~1.0 (14.4 N avg) | low | **Too weak** for a 5 ft vehicle |
| **AeroTech G40W + exhaust paddles + TVC** | **Partial: k = 0.25-1.0** | 3.0 avg | ~50 g paddles | **Baseline**: proven approach (BPS Scout F) |
| AeroTech G40W, ignition timing only | No | 3.0 | none | **Not viable**: +/-0.08 s ignition scatter at 22 m/s is +/-1.8 m of altitude error, which means 6-8 m/s impacts |
| 80-90 mm EDF on 6S LiPo | Yes, fully | ~2.5 static | +0.75-1.2 kg | Real throttle, but blows the 1.5 kg model-rocket limit; great as a GNC test bed |
| Cold gas (CO2) | Yes | < 0.3 | tank | Attitude control only |
| Throttled hybrid / liquid | Yes | high | tank, valves | Research-grade, high-power rules |
| RC turbojet (JetCat P20) | Yes | ~1.6 | 350 g + fuel | Seconds of spool-up; EUR 2,500 |

Sources: [docs/04-propulsion.md](04-propulsion.md).

### Baseline architecture (SLR-1)

![Vehicle layout](img/vehicle_layout.svg)

1. **Two identical AeroTech G40W motors side by side in one 2-axis TVC gimbal.**
   One flies the ascent; the other lands. Each sits 16.5 mm off-axis, and the
   gimbal trims about 2° to put the thrust line through the CG.
2. **Exhaust-paddle throttle** on the landing motor: two ceramic paddles swing
   into the plume on one servo. The design requirement is to keep only 25%
   of thrust when closed (k_min = 0.25). BPS reported up to 80% blockage.
3. **Small aft fins** give the rocket passive stability while it coasts up with
   the motor out and the TVC has no authority.
4. **Drogue from the nose at apogee.** With the fins, the bare rocket is
   unstable falling tail-first (-4.3 calibers). A 9 in drogue on a nose leash
   makes it stable (+1.2 calibers) and sets a steady ~22.5 m/s descent speed,
   so the landing burn always starts from nearly the same state. The drogue
   stays on through the burn, which adds stability, and is cut at touchdown.
5. **Throttle-ramp landing guidance.** The computer fires the landing motor when
   a constant-throttle plan would land mid-window. During the burn it re-solves
   a throttle ramp, k(t) = k0 + k1·t, 50 times a second. That ramp absorbs
   ignition-delay and impulse scatter ([docs/06](06-gnc.md)).
6. **Four Falcon-style legs** are stowed along the body and released at 8 m by
   a servo pin; springs swing them down to 55°.
7. **Backup main chute** for aborts: no ignition, out-of-envelope, or excessive tilt.

### Design decisions you might question

- **Why not land on the parachute?** The landing burn is the whole point. The
  chute is there so a failed burn is a recoverable event, not a crash.
- **Why two identical motors?** The G40W lands this mass well (see the
  [propulsion doc](04-propulsion.md)). A G40W ascent also happens to give
  exactly 500 ft. One part number also means one set of static fires to
  characterise.
- **Why a drogue instead of SpaceX-style grid fins?** To beat the
  destabilising fins, nose flaps would need about 4 x 100 x 100 mm of area. A
  9 in drogue does more for 35 g and also fixes the descent speed. Grid fins
  are a good v2 upgrade.
- **Why not finless like BPS?** A finless rocket is unstable both nose-first
  and tail-first. That is fine while a motor is burning, but this vehicle
  coasts unpowered for ~4 s up and ~7 s down. BPS Scout E reportedly used
  flip-out fins for the coast for the same reason.

## 1.4 Honest status

- **Done here:** the design, sizing, a 3-DOF simulator with Monte Carlo, and
  flight software. The software is a C++ core with 22 native unit and SIL tests
  that match the Python simulator, plus a Teensy hardware layer that
  compile-checks.
- **Not done:** nothing has been built or flown. The firmware has not run on
  real hardware.
- **Biggest risks**, in order:
  1. Paddle authority (k_min) and paddle survival.
  2. Landing-motor ignition-delay scatter. Measure it.
  3. Mass growth past 1500 g.
  4. Wind. Fly landing attempts in winds under 2.5 m/s (about 5 mph).

  The [test plan](09-test-plan.md) retires each of these before a full flight.
