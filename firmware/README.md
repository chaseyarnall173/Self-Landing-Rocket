# SLR-1 flight software (Teensy 4.1)

```
src/core/      hardware-independent C++ (runs on the Teensy AND in native tests)
  params.h            every tunable, mirrored from sim/rocketsim/guidance.py
  vec.h               vector / quaternion math
  motor_curve.*       thrust-curve table + mass model
  motor_tables.h      AUTO-GENERATED from sim/data/motors/*.eng
  estimator.*         gyro-quaternion attitude, 3-state altitude Kalman filter
  tvc_controller.*    2-axis dynamic-inversion PID with offset trim, anti-windup
  landing_guidance.*  touchdown window, throttle-ramp plan, fire decision, braking law
  flight_fsm.*        state machine, envelope checks, aborts
src/hal/       Teensy-specific
  board.h             pin map + calibration constants
  sensors.*           BMI088, BMP390 (non-blocking, datasheet compensation), TFmini-S
  actuators.*         servo PWM, paddles, mechanisms, pyro channels (one shot, key-gated)
  logger.*            100 Hz RAM ring log -> microSD after landing
src/main.cpp   1 kHz IMU / 50 Hz baro+GPS / 200 Hz control loop
test/native/   unit tests + full-mission software-in-the-loop test
tools/gen_tables.py   regenerate motor tables + Python "golden" values
```

## Build and test

```bash
make -C firmware test            # 22 tests: motor model, C++ vs Python guidance,
                                 # TVC closed loop, Kalman filter, attitude, SIL missions
make -C firmware compile-check   # host type-check of the Teensy code (stub Arduino headers)
cd firmware && pio run -e teensy41 -t upload
```

`compile-check` is not a substitute for the real toolchain. The HAL drivers
are written from the datasheets and **have not run on hardware yet**. Bring
them up on the bench (docs/08 §8.5) before anything else.

## Before every flight

- Measured values → `src/core/params.h`:
  - `DRY_MASS_KG`, `INERTIA_KGM2`, `GIMBAL_LEVER_M`
  - `IGNITION_DELAY_NOMINAL_S`, `THROTTLE_MIN`
  - `CDA_PRE_M2` (from the flight-1 descent speed)
- Calibrations → `src/hal/board.h`:
  - gimbal centres and gains, `PADDLE_US[]`
  - `IMU_MAP`, `PAD_HEADING_DEG`
- `VIRTUAL_GROUND_M`: 0 for a real landing, 60 for test flight F2.
- Re-run the Python Monte Carlo with the same numbers.
