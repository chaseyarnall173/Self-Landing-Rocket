# 9. Test plan - the development ladder

Don't fly the full mission first. Each rung retires a specific risk and
produces a number that goes back into `params.h` / `designs.py` before the
Monte Carlo is re-run. BPS.space spent years on drone drops, Echo tests and
several Scout versions. Budget for multiple flights.

| # | Test | Retires | Pass criteria | Outputs |
|---|---|---|---|---|
| T0 | **Software**: `make -C firmware test`, `make -C firmware compile-check`, `python sim/scripts/run_montecarlo.py` | Logic bugs | 22/22 tests; MC ≥ 85% clean with your parameters | — |
| T1 | **Bench / HIL**: flight computer on the bench, dummy pyro loads | Wiring, axis signs, state machine | Correct phases when you tilt or shake the board; pyro fires only when armed; key-out disarms | `IMU_MAP` verified |
| T2 | **Gimbal calibration** (laser pointer at 2 m) | Linkage nonlinearity, backlash | Backlash < 0.2°; ±7° reached; full-scale step settles in < 60 ms (high-speed phone video) | `GIMBAL_*_US_PER_RAD`, centres |
| T3 | **Mass properties** | Mass growth, CG, inertia | ≤ 1500 g with motors (target 1400); CG ≈ 1.03 m | `DRY_MASS_KG`, `INERTIA_KGM2`, `GIMBAL_LEVER_M` |
| T4 | **Static fires of the landing motor** (3-5 G40Ws), load cell at ≥ 1 kHz, with the flight igniters fired by the flight computer | **Ignition delay, impulse scatter** | Delay σ ≤ 0.08 s (the MC assumption); impulse within ±5% | `IGNITION_DELAY_NOMINAL_S`, the measured σ fed into the MC |
| T5 | **Paddle-throttle static fires**: paddles stepped through 4 positions during a burn | **Paddle authority k_min**, paddle survival | Closed ≤ 0.30 of thrust; no paddle fracture; < 1 mm erosion per burn | `PADDLE_US[]`, `THROTTLE_MIN` |
| T6 | **TVC static fire**: vehicle on a 2-axis swivel (CG pivot) with the motor burning, closed loop | Attitude loop on real hardware | Holds vertical ±2° through the burn; no oscillation; gimbal unsaturated after 0.3 s | Gain adjustments |
| T7 | **Leg drop tests** ([docs/07](07-landing-legs.md#74-tests)) | Bounce, tip-over | No bounce > 3 cm at 2.5 m/s; no tip at 1.5 m/s lateral | — |
| T8 | **Recovery ground tests**: drogue and main ejection by servo | Recovery reliability | 10/10 deploys for each | — |
| F1 | **Flight 1: ascent only.** G40W ascent with TVC; drogue at apogee, **main chute at 60 m**; landing motor = **inert dummy of equal mass** | Ascent TVC, apogee, drogue descent, attitude under the drogue | Apogee 130-175 m; tilt < 10° under the drogue; descent 20-25 m/s; clean log | Real drag (descent speed) → `CDA_PRE_M2` |
| F2 | **Flight 2: "virtual ground" landing burn.** Same as F1, but the flight computer treats **60 m AGL as the ground** (`VIRTUAL_GROUND_M = 60` in `params.h`). It fires the live landing motor, runs the whole burn, then deploys the main chute | Ignition in flight, guidance, paddles in flight | Touchdown at the virtual ground at ≤ 2.5 m/s (from the log); chute deploys | Guidance validation with no risk to the vehicle |
| F3 | **Flight 3: full mission** in wind < 2.5 m/s at the surface | Everything | Upright landing, ≤ 2.5 m/s | 🎉 |

## Notes on the key tests

**T4 - ignition delay (most important).**

Set-up:
- Motor nozzle **down** into a deflector, as it flies in descent. Load cell
  (ADS1220 or ADS1256 class ADC at ≥ 1 kHz) logged on the same clock as the
  FIRE command.
- Use the *exact* flight igniter type, the dual-igniter installation and the
  2S battery.

Measurements:
- t(FIRE) → t(thrust > 5 N) for each motor;
- the full thrust curve, to compare against the `.eng`.

Variant: also try a light airflow blowing *up into the nozzle* (a leaf blower)
to mimic the ram air of a tail-first descent. Composite motors may light slower
with air flowing in.

**T5 - paddle authority.** If k_min comes out at 0.40 instead of 0.25:

- re-run the Monte Carlo (expect ~74% clean and 5% crashes);
- improve the paddles: larger area, steeper angle, a third paddle; or
- reduce the landing mass, which moves the hop window earlier.

**F2 - virtual-ground flight.** This is how you test the landing logic without
betting the vehicle.
- Set `params::VIRTUAL_GROUND_M = 60`. The state machine then flies the
  entire landing against a ground 60 m up: the fire decision, guidance,
  legs and terminal phase.
- At the virtual touchdown it logs `VIRTUAL TOUCHDOWN vz=...` and deploys
  the main chute.
- Lidar updates are ignored in this mode.
- You need an apogee of ≥ 150 m so the drogue descent reaches terminal
  speed before the 81 m fire point.

## Lessons from people who've done this

- **Bouncing landing legs** ended early BPS attempts. Damp the legs
  ([Hackaday](https://hackaday.com/2022/08/05/bps-space-succesfully-lands-a-model-rocket/)).
- **Horizontal velocity** from bad GPS/navigation made Scout E tip at
  touchdown ([Hackaday 2020](https://hackaday.com/2020/11/27/so-close-to-landing-a-model-rocket-on-its-tail/)).
- **A barometer error** wrecked Aryan Kapoor's first flight. **An ascent motor
  that failed to eject** caused a bounce. He landed on his 5th attempt after 3
  years ([Hackaday 2024](https://hackaday.com/2024/07/10/model-rocket-nails-vertical-landing-after-three-year-effort/)).
- **Barometers are unreliable in high relative wind** near the ground. One
  builder switched to lidar for the descent trigger
  ([Timucin Erbas](https://www.timucinerbas.com/3d-printed-rocket)). We use
  lidar for the last 7-10 m.
