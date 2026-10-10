# rocketsim - SLR-1 mission simulator

A planar 3-DOF (x, z, pitch) simulation of the whole flight, with the flight
computer's guidance in the loop. Requires Python 3.10+ with numpy and
matplotlib.

| File | Contents |
|---|---|
| `rocketsim/motors.py` | RASP `.eng` loader; thrust, impulse and mass vs time |
| `rocketsim/vehicle.py` | Geometry, mass properties, component aerodynamics for nose-first **and** tail-first flow |
| `rocketsim/designs.py` | The SLR-1 baseline: **itemised mass budget rolled up from the CAD** (`cad/out/mass_items.json`), fins (Barrowman), top airbrake petals, motors (also the rev A drogue / G40W variant) |
| `rocketsim/guidance.py` | Flight-computer logic: state machine, ignition decision, "strong then simmer" (exponential-decay) landing plan, TVC control (mirrors `firmware/src/core`) |
| `rocketsim/sim.py` | Equations of motion, actuators (servo lag/rate, paddle lag), wind, sensor errors, ground contact, outcome classification |
| `rocketsim/montecarlo.py` | Dispersion model + parallel runner |
| `rocketsim/plots.py` | Figures for the docs |
| `data/motors/*.eng` | Official thrust curves from ThrustCurve.org |
| `scripts/*.py` | `mass_budget`, `run_nominal`, `run_montecarlo`, `ignition_sensitivity`, `motor_table` |

## Changing the design

1. Edit `SLR1_ITEMS` in `rocketsim/designs.py` with real part masses (or
   re-run `python cad/build.py` after a CAD change and copy
   `cad/out/mass_items.json`).
2. Edit the motor names in `slr1()`. Drop any new `.eng` file into
   `data/motors/`.
3. Edit `DispersionModel` in `rocketsim/montecarlo.py` with measured
   ignition-delay and paddle numbers.
4. Run `scripts/run_montecarlo.py`. Then **mirror** any guidance or parameter
   change into `firmware/src/core/params.h`.
5. Run `python firmware/tools/gen_tables.py && make -C firmware test` to
   confirm the C++ still matches the Python.
