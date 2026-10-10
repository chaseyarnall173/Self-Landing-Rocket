# Self-Landing Rocket (SLR-1)

A complete design package for a **5 ft (1.52 m) hobby rocket**. A solid motor
shoots it to **~200 ft**. On the way down, a second, thrust-controlled motor
**catches it**: braking hard at first, then simmering down to about the
rocket's weight. **Landing legs flip down** before an upright touchdown.
**No parachute is used in a normal flight. The engine does the recovery.**

![Mission profile](docs/img/mission_profile.svg)

## The design in one paragraph

SLR-1 is a 4 in x 60 in rocket weighing 1.31 kg at liftoff.

- **Motors:** two single-use AeroTech F motors share one 2-axis gimbal that
  steers the thrust (TVC): an **F42T** for the climb and an **F25W** for the
  landing.
- **Climb:** the F42T flies it to **63 m (207 ft)**. Small fins keep it pointed
  while it coasts up with the motor out.
- **Fall:** at apogee, four airbrake petals cut from the body tube spring open
  near the top, like Falcon 9's grid fins. They keep the rocket tail-down and
  falling straight at ~18.5 m/s.
- **Landing burn:**
  - The flight computer lights the F25W at about 15 m (49 ft).
  - Two ceramic paddles in the exhaust **throttle** it. The computer plans a
    burn that starts strong and fades out exponentially, so thrust tapers from
    ~49 N to ~16 N (the rocket weighs 12.3 N) right at touchdown.
  - The gimbal holds it upright and cancels sideways drift.
- **Touchdown:** the legs flip down at 12 m and it lands at ~1.1 m/s.
- **Backup chute:** fires only if the landing motor fails to light.

![Vehicle layout](docs/img/vehicle_layout.svg)

| | |
|---|---|
| Liftoff mass | 1314 g (model-rocket limit 1500 g) |
| Motors | AeroTech F42T (climb) + AeroTech F25W (landing), both single-use |
| Propellant / total impulse | 65.8 g / 127 N·s → **FAA Class 1 model rocket, no certification, 1000 ft field** |
| Apogee | 63 m / 207 ft (Monte Carlo p5-p95: 57-71 m) |
| Landing burn | lit ~15 m up, ~1.4 s long, thrust 49 N → 16 N ("strong, then simmer") |
| Touchdown | 1.1 m/s nominal; Monte Carlo mean 1.35 m/s, p95 2.0 m/s |
| Monte Carlo (300 flights) | **93% clean landings, 0 crashes**; 98-100% in wind under 3 m/s |
| Est. cost | ≈ $1,850 including motors, test gear and spares |

## What's in this repo

| Path | What |
|---|---|
| [`docs/01-mission-and-architecture.md`](docs/01-mission-and-architecture.md) | Mission, requirements, trade study, **why this architecture** |
| [`docs/02-regulations-and-safety.md`](docs/02-regulations-and-safety.md) | FAA Part 101, NAR/Tripoli/NFPA, Massachusetts, clubs, safety rules |
| [`docs/03-vehicle-design.md`](docs/03-vehicle-design.md) | Geometry, **mass budget**, stability nose-first *and* tail-first, airbrakes, materials |
| [`docs/04-propulsion.md`](docs/04-propulsion.md) | Motor selection, **exhaust-paddle throttle**, dual-motor TVC gimbal, ignition |
| [`docs/05-avionics.md`](docs/05-avionics.md) | Flight computer, sensors, **pyro circuit**, power, pin map |
| [`docs/06-gnc.md`](docs/06-gnc.md) | State machine, estimation, TVC control, **"strong then simmer" landing guidance**, simulation results |
| [`docs/07-landing-legs.md`](docs/07-landing-legs.md) | Leg geometry, tip-over analysis, flip-down mechanism, damping |
| [`docs/08-build-guide.md`](docs/08-build-guide.md) | **Step-by-step build procedure** |
| [`docs/09-test-plan.md`](docs/09-test-plan.md) | Development ladder: bench → static fires → ascent-only → "virtual ground" → full landing |
| [`docs/10-flight-ops.md`](docs/10-flight-ops.md) | **Printable checklists**, go/no-go, misfire and abort procedures |
| [`docs/11-bom-and-budget.md`](docs/11-bom-and-budget.md) | Bill of materials and cost |
| [`docs/12-references.md`](docs/12-references.md) | Sources |
| [`sim/`](sim/) | Python 3-DOF mission simulator + Monte Carlo ([README](sim/README.md)) |
| [`firmware/`](firmware/) | Teensy 4.1 flight software ([README](firmware/README.md)) |

## Quick start

```bash
# simulator (Python 3.10+, numpy, matplotlib)
pip install numpy matplotlib
python sim/scripts/mass_budget.py          # mass, CG, static margins
python sim/scripts/run_nominal.py          # one flight + plots in docs/img/
python sim/scripts/run_montecarlo.py 300   # dispersion campaign
python sim/scripts/ignition_sensitivity.py

# flight software
make -C firmware test            # native unit tests + full-mission SIL (C++ vs Python goldens)
make -C firmware compile-check   # type-check the Teensy build against stub headers
cd firmware && pio run -e teensy41   # real build (PlatformIO)
```

## Status and honest caveats

- **This is a design, a simulator and flight software. Nothing has been
  built or flown, and the firmware has not run on real hardware yet.**
- The simulated success rate rests on assumptions you must replace with
  measurements, following [docs/09](docs/09-test-plan.md):
  - landing-motor ignition delay (assumed 0.30 ± 0.08 s);
  - paddle throttle authority;
  - airbrake drag;
  - final mass and inertia.
- Read [docs/02](docs/02-regulations-and-safety.md) before buying motors.
- Fly only at a club launch, with RSO approval and the backup parachute
  installed.
- Earlier design (rev A): 500 ft on two G40W motors with a drogue. It is kept
  in git history and remains selectable in the simulator
  (`slr1(drag_device="drogue", ascent="AeroTech_G40W", landing="AeroTech_G40W")`).
