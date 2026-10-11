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
| 1 | Liftoff | 1.47 kg, T/W 4.2 peak / 3.5 average, AeroTech F52C | Club launch controller |
| 2 | Ascent burnout | 1.3 s, 23 m, 32 m/s | Accelerometer goes negative |
| 3 | Apogee | **69 m (226 ft)** at 4.3 s | Kalman-filter velocity < −0.5 m/s for 50 ms |
| 3a | Top airbrakes open | 4 petals spring to 90° | Same event |
| 4 | Tail-first fall | ~18 m/s | — |
| 5 | Landing-motor FIRE command | ~15 m (49 ft) | Guidance predictor (every 5 ms) |
| 5a | Legs flip down | 12 m | Altitude |
| 5b | Ignition, AeroTech F25W | ~0.3 s after FIRE | Accelerometer jump |
| 6 | Burn: 48 → 39 → 31 → 25 → 22 → 19 N | ~1.5 s | Throttle re-planned 50x/s |
| 7 | Touchdown | **1.1 m/s** at 10.3 s (Monte Carlo mean 1.4, p95 2.0) | Lidar / altitude |
| 8 | Safe | paddles closed, motor burns out | — |

Run it yourself: `python sim/scripts/run_nominal.py`.

## 1.2 Requirements

| ID | Requirement | Source / rationale | How it is met |
|---|---|---|---|
| R1 | Length ~5 ft | User | 60 in (1.524 m) |
| R2 | Apogee ~200 ft | User | 226 ft nominal (202-254 ft p5-p95) |
| R3 | Solid motor up, thrust-controlled motor down | User | F52C up; F25W down with exhaust-paddle throttle (k = 0.25-1.0) + TVC |
| R4 | Burn is strong first, then simmers to barely firing | User | "Exponential-decay" guidance: net braking A·e^(−t/τ) + C, ending at ~hover thrust |
| R5 | No drogue: engine-only recovery | User | Top airbrakes (rigid petals) for attitude only; the engine does all braking |
| R6 | Legs flip down near the ground | User | 4 spring legs released by a servo pin at 12 m |
| R7 | Liftoff mass ≤ 1500 g, propellant ≤ 125 g | FAA Class 1 ([§101.22](https://www.ecfr.gov/current/title-14/chapter-I/subchapter-F/part-101/subpart-C/section-101.22)) | 1472 g from the CAD (**28 g margin**), 68.8 g |
| R8 | Touchdown ≤ 2.5 m/s vertical, ≤ 1.5 m/s lateral, ≤ 10° tilt | Leg sizing ([docs/07](07-landing-legs.md)) | 93% of 300 Monte Carlo flights, 0 crashes |
| R11 | Every part fits, clears and can be made | Build | Full CAD ([docs/13](13-cad.md)): 0 clashes, gimbal free through ±7° |
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
   0.1 s of burn.
2. **Guidance shaped like the motor.** Instead of a constant braking plan, the
   computer solves for braking that decays exponentially and ends with thrust
   near weight. In 300 dispersed flights this gave 93% clean landings, against
   90% for a linear-ramp plan.
3. **A low flight and a short fall.** At 200 ft the fall is short and slow
   (~18 m/s), and wind drift is small (p5-p95 of the landing spot is
   −20 to +20 m from the pad).

### Why not finless, like the videos?

With the engine off, the gimbal can't steer. Something passive has to keep
the rocket pointed while it coasts up and while it falls back. The simulator
tested every option:

| Configuration | Coasting up | Falling tail-first | Clean landings |
|---|---|---|---|
| Finless, nothing on top | tumbles | tumbles | 0% |
| Finless + top airbrakes | tumbles | stable | 0% (tumbles on the way up) |
| Rev B fins + 125 mm petals, at the real (CAD) mass | stable (+1.2 cal) | marginal (+0.07 cal) | 85% (2% crash) |
| **90 % fins + 150 mm petals (rev C)** | **stable (+0.8 cal)** | **stable (+1.0 cal)** | **93%** |
| Fins + drogue chute (rev A) | stable | stable | 90% (but uses a parachute) |

Finless landers like BPS Scout and Aryan Kapoor's fly short, low hops where
the unpowered coast lasts about a second. A 200 ft flight coasts ~3 s up
and ~4 s down, too long without passive stability.

### Options for the "thrust-controlled" landing motor

| Option | Throttle? | Verdict |
|---|---|---|
| **Solid + exhaust paddles + TVC** (F25W) | Partial, k = 0.25-1.0 | **Baseline.** Proven by BPS; works in the 1.5 kg model-rocket class |
| Solid, ignition timing only | No | Not viable: ±0.08 s of ignition scatter at 18 m/s is ±1.5 m of altitude error |
| Long-burn solid (AeroTech G25W reload, 5.3 s) + paddles | Partial | Lands as well (94% in Monte Carlo, ~2.9 s controlled burn lit at ~80 ft), but its metal case puts rev C at 1570 g, **over the 1500 g limit** |
| 80-90 mm electric ducted fan | Full, can hover | Adds 0.8-1.2 kg → high-power rocket; great as a tethered test bed |
| Throttled hybrid / liquid | Full | Research-grade, high-power rules |

Sources and details: [docs/04-propulsion.md](04-propulsion.md).

### Baseline architecture (SLR-1 rev C)

![Vehicle layout](img/vehicle_layout.svg)

1. **F52C + F25W side by side in one 2-axis TVC gimbal.** Each motor sits
   16.5 mm off-axis; the gimbal trims ~2° to put the thrust line through the
   CG.
2. **Exhaust-paddle throttle** on the F25W: two ceramic paddles on one servo.
3. **Four fins** for passive stability on the way up.
4. **Four top airbrake petals**, each a quadrant of the body tube 80 x 150 mm,
   hinged at their forward edge and sprung open to 90° at apogee. They make
   tail-first flight stable (+1.0 cal; −4.9 without them) and keep the fall
   steady at ~18 m/s, so the burn always starts from about the same state.
5. **"Strong then simmer" landing guidance** ([docs/06](06-gnc.md)).
6. **Four flip-down legs**, released at 12 m.
7. **Backup main chute**, which deploys only on an abort.

## 1.4 Status

- **Done:**
  - design, sizing and a 3-DOF simulator with Monte Carlo;
  - a full 3D CAD model (133 parts) with STEP, STL and DXF files, a clash
    check and a gimbal sweep ([docs/13](13-cad.md));
  - flight software: a C++ core with 29 native unit and SIL tests that match
    the Python simulator, plus a Teensy hardware layer that compile-checks.
- **Not done:** nothing has been built or flown, and the firmware has not run
  on real hardware.
- **Biggest risks, in order:**
  1. **Wind.** Landing attempts only in surface wind under 3 m/s (~6.7 mph);
     success falls to 84% at 3-4 m/s.
  2. **Mass.** The CAD puts liftoff at 1472 g, 28 g under the 1500 g limit.
     Weigh every part; [docs/03](03-vehicle-design.md#32-mass-budget-target--1500-g-at-liftoff)
     lists where to cut.
  3. **Airbrake petals:** they must open reliably and give the drag the
     simulator assumes. Check on flight F1.
  4. **Landing-motor ignition delay.** Measure it.
  5. **Paddle survival in the exhaust.**

  The [test plan](09-test-plan.md) retires each one before a full flight.
