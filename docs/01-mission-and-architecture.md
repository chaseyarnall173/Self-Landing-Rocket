# 1. Mission, requirements and architecture

## 1.1 The mission

Build a ~5 ft rocket that:

1. shoots up on a solid motor to **~200 ft**,
2. stays pointed tail-down as it falls back,
3. lights a second, thrust-controlled motor that **brakes hard at first, then
   simmers down until it is barely holding the rocket up**,
4. flips its legs down just before touchdown and lands upright, **with no
   parachute in a normal flight**.

![Mission profile](img/mission_profile.svg)

| # | Event | Nominal value (simulation) | Trigger |
|---|---|---|---|
| 1 | Liftoff | 1.31 kg, T/W 5.3 peak / 2.9 average, AeroTech F42T | Club launch controller |
| 2 | Ascent burnout | 1.4 s, 28 m, 27 m/s | Accelerometer goes negative |
| 3 | Apogee | **63 m (207 ft)** at 4.0 s | Kalman-filter velocity < −0.5 m/s for 50 ms |
| 3a | Top airbrakes open | 4 petals spring to 90° | Same event |
| 4 | Tail-first fall | ~18.5 m/s | — |
| 5 | Landing-motor FIRE command | ~15 m (49 ft) | Guidance predictor (every 5 ms) |
| 5a | Legs flip down | 12 m | Altitude |
| 5b | Ignition, AeroTech F25W | ~0.3 s after FIRE | Accelerometer jump |
| 6 | Burn: 49 → 35 → 30 → 24 → 20 → 16 N | ~1.4 s | Throttle re-planned 50x/s |
| 7 | Touchdown | **1.1 m/s** (Monte Carlo mean 1.35, p95 2.0) | Lidar / altitude |
| 8 | Safe | paddles closed, motor burns out | — |

Run it yourself: `python sim/scripts/run_nominal.py`.

## 1.2 Requirements

| ID | Requirement | Source / rationale | How it is met |
|---|---|---|---|
| R1 | Length ~5 ft | User | 60 in (1.524 m) |
| R2 | Apogee ~200 ft | User | 207 ft nominal (186-233 ft p5-p95) |
| R3 | Solid motor up, thrust-controlled motor down | User | F42T up; F25W down with exhaust-paddle throttle (k = 0.25-1.0) + TVC |
| R4 | Burn is strong first, then simmers to barely firing | User | "Exponential-decay" guidance: net braking A·e^(−t/τ) + C, ending at ~hover thrust |
| R5 | No drogue: engine-only recovery | User | Top airbrakes (rigid petals) for attitude only; the engine does all braking |
| R6 | Legs flip down near the ground | User | 4 spring legs released by a servo pin at 12 m |
| R7 | Liftoff mass ≤ 1500 g, propellant ≤ 125 g | FAA Class 1 ([§101.22](https://www.ecfr.gov/current/title-14/chapter-I/subchapter-F/part-101/subpart-C/section-101.22)) | 1314 g (186 g margin), 65.8 g |
| R8 | Touchdown ≤ 2.5 m/s vertical, ≤ 1.5 m/s lateral, ≤ 10° tilt | Leg sizing ([docs/07](07-landing-legs.md)) | 93% of 300 Monte Carlo flights, 0 crashes |
| R9 | Fail-safe if the landing motor doesn't light | NAR code, MA law, safety | Backup 30 in chute fires **only** on an abort |
| R10 | Landing motor can only ignite inside a safe envelope | Safety | Arm key + pad arming + in-flight envelope checks ([docs/06](06-gnc.md#67-safety-interlocks)) |

## 1.3 Why this architecture

**Solid motors can't be throttled or shut down.** BPS.space (Joe Barnard)
spent seven years landing a model rocket on solids. The breakthrough was
**ceramic paddles that block part of the exhaust**
([Hackaday](https://hackaday.com/2022/08/05/bps-space-succesfully-lands-a-model-rocket/),
[The New Stack](https://thenewstack.io/one-mans-dream-to-build-a-working-spacex-style-model-rocket/)).
SLR-1 does the same, and adds three choices that made the simulator happy.

1. **A landing motor whose thrust already decays.** The AeroTech F25W starts
   at 58 N and falls to 34 N at 1 s, 23 N at 1.5 s and 10 N at 2 s. That is
   naturally "strong, then simmer". The paddles only have to fine-tune it,
   and closed paddles can never lift a landed rocket back off its legs after
   0.4 s of burn.
2. **Guidance shaped like the motor.** Instead of a constant braking plan, the
   computer solves for braking that decays exponentially and ends with thrust
   ≈ weight. In 300 dispersed flights this gave 93% clean landings, against
   89% for a linear-ramp plan.
3. **A low flight and a short fall.** At 200 ft the fall is short and slow
   (~18.5 m/s), and wind drift is small (p5-p95 of the landing spot is
   −19 to +22 m from the pad).

### Why not finless, like the videos?

With the engine off, the gimbal can't steer. Something passive has to keep
the rocket pointed while it coasts up and while it falls back. The simulator
tested every option:

| Configuration | Coasting up | Falling tail-first | Clean landings |
|---|---|---|---|
| Finless, nothing on top | tumbles | tumbles | 0% |
| Finless + top airbrakes | tumbles | stable | 0% (tumbles on the way up) |
| 85% fins + top airbrakes | marginal (+0.05 cal) | stable | ~50% (44% tumble and abort) |
| **Full fins + top airbrakes** | **stable (+0.8 cal)** | **stable (+0.5 cal)** | **93%** |
| Fins + drogue chute (rev A) | stable | stable | 90% (but uses a parachute) |

Finless landers like BPS Scout and Aryan Kapoor's fly short, low hops where
the unpowered coast lasts about a second. A 200 ft flight coasts ~2.6 s up
and ~3.7 s down, too long without passive stability.

### Options for the "thrust-controlled" landing motor

| Option | Throttle? | Verdict |
|---|---|---|
| **Solid + exhaust paddles + TVC** (F25W) | Partial, k = 0.25-1.0 | **Baseline.** Proven by BPS; works in the 1.5 kg model-rocket class |
| Solid, ignition timing only | No | Not viable: ±0.08 s of ignition scatter at 18 m/s is ±1.5 m of altitude error |
| Long-burn solid (AeroTech G25W reload, 5.3 s) + paddles | Partial | Works too (96% in Monte Carlo); ~2.9 s controlled burn lit at ~80 ft. Heavier: metal case, 1.42 kg |
| 80-90 mm electric ducted fan | Full, can hover | Adds 0.8-1.2 kg → high-power rocket; great as a tethered test bed |
| Throttled hybrid / liquid | Full | Research-grade, high-power rules |

Sources and details: [docs/04-propulsion.md](04-propulsion.md).

### Baseline architecture (SLR-1 rev B)

![Vehicle layout](img/vehicle_layout.svg)

1. **F42T + F25W side by side in one 2-axis TVC gimbal.** Each motor sits
   16.5 mm off-axis; the gimbal trims ~2° to put the thrust line through the
   CG.
2. **Exhaust-paddle throttle** on the F25W: two ceramic paddles on one servo.
3. **Four fins** for passive stability on the way up.
4. **Four top airbrake petals**, each a quadrant of the body tube 80 x 125 mm,
   hinged at their forward edge and sprung open to 90° at apogee. They make
   tail-first flight stable (+0.5 cal; −4.3 without them) and keep the fall
   steady at ~18.5 m/s, so the burn always starts from about the same state.
5. **"Strong then simmer" landing guidance** ([docs/06](06-gnc.md)).
6. **Four flip-down legs**, released at 12 m.
7. **Backup main chute**, which deploys only on an abort.

## 1.4 Status

- **Done:**
  - design, sizing and a 3-DOF simulator with Monte Carlo;
  - flight software: a C++ core with 28 native unit and SIL tests that match
    the Python simulator, plus a Teensy hardware layer that compile-checks.
- **Not done:** nothing has been built or flown, and the firmware has not run
  on real hardware.
- **Biggest risks, in order:**
  1. **Wind.** Landing attempts only in surface wind under 3 m/s (~6.7 mph);
     success falls to 78% at 3-4 m/s.
  2. **Airbrake petals:** they must open reliably and give the drag the
     simulator assumes. Check on flight F1.
  3. **Landing-motor ignition delay.** Measure it.
  4. **Paddle survival in the exhaust.**
  5. **Mass growth.**

  The [test plan](09-test-plan.md) retires each one before a full flight.
