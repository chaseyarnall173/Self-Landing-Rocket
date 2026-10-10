# 4. Propulsion: motors, TVC gimbal, exhaust-paddle throttle

## 4.1 Motor data (official ThrustCurve.org curves)

The `.eng` files live in `sim/data/motors/`, from
[ThrustCurve.org](https://www.thrustcurve.org). Run
`python sim/scripts/motor_table.py`.

| Motor | Type | Total impulse | Avg / max thrust | Burn | Propellant / total | Use |
|---|---|---|---|---|---|---|
| **AeroTech F42T** | Blue Thunder, single-use, 29 x 83 mm | 53-56 N·s | 38-42 / 69 N | 1.3-1.5 s | 27 / 76 g | **Ascent → 207 ft** |
| **AeroTech F25W** | White Lightning, single-use (molded case), 29 x 98 mm | 71-78 N·s | 26 / 58 N | 2.2-2.7 s | 36-39 / 90-97 g | **Landing** |
| AeroTech F52C | Classic, single-use | 66 N·s | 50 / 61 N | 1.3 s | 30 / 81 g | Ascent alternative: ~276 ft (also used for test flight F2) |
| AeroTech G25W | White Lightning, **reload** (RMS-29 case) | 117.5 N·s | 22 / 41 N | 5.3 s | 62.5 g + case | Long-burn landing alternative: ~2.9 s controlled burn |
| AeroTech G40W | White Lightning, single-use | 97-99 N·s | 41 / 74 N | 2.3 s | 53.8 / 123 g | Rev A (500 ft) design |
| Estes F15 | Black powder | 49.6 N·s | 14.4 / 25 N | 3.45 s | 60 / 102 g | Too weak for a 5 ft vehicle (below its weight) |

Ranges show the ThrustCurve database value versus the sim-file value. Use the
`.eng` file, and weigh your motors.

Sources: [ThrustCurve search](https://www.thrustcurve.org/motors/search.html),
[AeroTech](https://aerotech-rocketry.com),
[NAR F15 cert](https://www.nar.org/wp-content/uploads/2014/08/F15.pdf).

**The F25W's thrust curve is already "strong then simmer":**

| Time after ignition | 0.04 s | 0.5 s | 1.0 s | 1.5 s | 2.0 s | 2.2 s |
|---|---|---|---|---|---|---|
| Thrust (N) | 57.6 | 47.9 | 33.8 | 22.9 | 10.1 | 4.5 |
| vs rocket weight (12.4 N) | 4.6x | 3.9x | 2.7x | 1.8x | 0.8x | 0.4x |

What the regressive curve buys you:

- **Strong braking comes first**, when the rocket is falling fastest.
- **It fades toward hover** near the ground.
- **No hop after touchdown.** From 0.4 s of burn onward, even with the paddles
  fully closed (25% of thrust), the motor can't lift a landed rocket off its
  legs. The touchdown window runs from 0.40 s to 1.79 s of burn, much wider
  than the G40W's (1.12-1.94 s).

**Totals for the flight:**
- 65.8 g of propellant and 127 N·s: Class 1, in the NAR "G" site row (1000 ft).
- Each motor has less than 62.5 g of propellant and less than 80 N·s, so both
  are CPSC-exempt consumer model-rocket motors.

## 4.2 Why a solid motor still needs a throttle

A landing burn must end at **h = 0 and v ≈ 0 at the same moment**. Without a
throttle you control only the ignition time, so the motor's ignition-delay
scatter goes straight into the touchdown error:

- Δh ≈ 18.5 m/s x 0.08 s = **1.5 m** (1σ) → impacts of 5-6 m/s.
- Tilting the gimbal to waste thrust only removes 1 − cos θ: 0.75% at 7°.
- A **mechanical exhaust throttle** gives a real second control.

Results from 300 dispersed flights (wind 0-4 m/s, ignition delay 0.30 ± 0.08 s):

| Variant | Clean landings | Crashes |
|---|---|---|
| **Baseline: paddles leave 25% of thrust, "strong then simmer" guidance** | **93%** | **0%** |
| Weak paddles (leave 40% of thrust) | 93% | 0% |
| Tightly characterised ignition (σ = 0.04 s) | 93% | <1% |
| Linear-ramp guidance instead of exponential | 89% | 0% |

With the F25W, paddle strength stops being the critical driver, because the
motor's own decay does most of the work. The remaining failures are mostly
sideways drift in winds of 3-4 m/s.

## 4.3 Exhaust-paddle throttle design

```
          nozzle exit (landing motor)
               ||  ||
     paddle A  \\  //  paddle B        Two paddles on a common axle each side;
                \\//                   one servo drives both through a
           ----> plume <----           scissor link so they close symmetrically
                                       (no net side force).
```

- **Geometry:**
  - two flat paddles 20 x 28 mm, pivoting 10 mm below the nozzle exit;
  - fully open (k = 1) they sit outside the plume;
  - closed, they meet at 30-40° to the axis and deflect most of the jet
    sideways in opposite directions.
- **Materials:**
  - paddle faces: machinable alumina-silicate ceramic, or 4-5 mm graphite;
  - arms: 304 stainless;
  - pivots: steel pins in bronze bushings;
  - the servo sits behind a heat shield (aluminium sheet + ceramic paper).
  - Nothing 3D-printed goes near the plume.
- **Actuator:** KST X08H HV digital servo, 0.09 s/60°
  ([servodatabase](https://servodatabase.com/servo/kst/x08h)).
- **Calibration:** on the load-cell stand, map servo pulse to throttle k at
  1.0, 0.75, 0.5 and 0.25. Put the numbers in `PADDLE_US[]`
  (`firmware/src/hal/board.h`) and `THROTTLE_MIN` (`params.h`).
- **Wear:** inspect after every burn; budget a new pair every 2-3 burns.

## 4.4 Dual-motor TVC gimbal

- **Layout:**
  - two aluminium 29 mm motor sleeves (125 mm long, so they fit both motor
    lengths), 33 mm centre-to-centre, in an inner ring;
  - a middle ring (axis Y) and an outer ring fixed to the airframe (axis Z);
  - pivots at station ~1.46 m, about 0.45 m from the CG at landing.
- **Range:** ±7° (BPS uses ±5°,
  [BPS TVC](https://bps.space/products/thrust-vector-control)).
- **Servos:** 2 x KST X08H with ball-link pushrods, about a 2:1 ratio.
- **Offset thrust:** each motor is 16.5 mm off the centreline. The software
  feeds forward a ~2.1° trim and leans the body the other way so the trim
  doesn't push the vehicle sideways.
- **Retention:** screw-cap or clip retainers **on each sleeve**. Thrust goes
  through the sleeves and pivots into the aft thrust plate.
- **Spacer:** the F42T is 15 mm shorter than the F25W. Add a printed spacer so
  both nozzles exit at the same station.

## 4.5 Ignition

| Item | Choice | Why |
|---|---|---|
| Ascent igniter | The igniter supplied with the motor | Fired from the club launch controller |
| Landing igniters | **Two** in the F25W, on two MOSFET channels fired together | A no-ignition failure is the worst case |
| Firing current | ≥ 2 A per igniter for Estes-type starters (0.5 A no-fire / 2 A all-fire); 1 A for MJG Firewire-type | ([Estes starter](https://www.erockets.biz/estes-model-rocket-star-tech-starters-igniters-for-flying-model-rockets-est-2303/), [MJG](https://www.erockets.biz/mjg-technology-36-starters-initiators-dipped-2pk-mjg-03)) |
| Pulse | 1.0 s, once per boot | `PYRO_PULSE_MS` |
| Continuity test current | < 10 mA | 10 kΩ sense divider ([docs/05](05-avionics.md)) |

**Measure the ignition delay.**
- Static-fire 3-5 F25Ws with the exact flight igniters, logging a load cell at
  ≥ 1 kHz (ADS1220-class ADC; an HX711 is too slow,
  [Arduino blog](https://blog.arduino.cc/2018/12/17/an-arduino-based-rocket-motor-thrust-test-stand/)).
- Set `IGNITION_DELAY_NOMINAL_S` to your measured mean, then re-run the
  Monte Carlo with your measured σ.

## 4.6 Ejection charges and delays

- The NAR code says to use motors only as the manufacturer instructs.
- **Both motors need a configuration with no ejection charge**, such as a
  plugged or "P" variant, or one where the instructions allow the charge to be
  removed for electronic deployment.
- Ask AeroTech or your vendor which F42T and F25W configurations allow this.
  Never drill or modify a motor.

## 4.7 Truly throttleable alternative: EDF test bed

An 80 mm EDF on 6S gives ~36 N
([Motion RC](https://motionrc.com/products/freewing-80mm-12-blade-edf-6s-power-system-w-3658-2150kv-inrunner-motor-e72314)),
but the fan, ESC and battery weigh 0.75-0.8 kg. That would push SLR-1 into
high power.

Best use: a **tethered EDF hopper** running the *same flight computer and TVC
code*, to practise landings for the price of a battery charge
([EDF lander example](https://github.com/UrosKukovic/TVC_Self_Landing_Rocket)).
