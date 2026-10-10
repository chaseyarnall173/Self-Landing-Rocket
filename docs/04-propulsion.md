# 4. Propulsion: motors, TVC gimbal, exhaust-paddle throttle

## 4.1 Motor data (official ThrustCurve.org curves)

The `.eng` files live in `sim/data/motors/` (from
[ThrustCurve.org](https://www.thrustcurve.org)). Run
`python sim/scripts/motor_table.py`.

| Motor | Type | Total impulse | Avg / max thrust | Burn | Propellant / total | Use |
|---|---|---|---|---|---|---|
| **AeroTech G40W** | APCP White Lightning, single-use, 29 x 124 mm | 97-99 N·s | 41-43 / 74 N | 2.3 s | 53.8 / 123 g | **Ascent AND landing** |
| AeroTech G80T | Blue Thunder, single-use | 133 N·s | 74 / 102 N | 1.8 s | 62.5 / 128 g | Ascent alternative: apogee ~240 m (790 ft) |
| AeroTech G64W | White Lightning, **reload** (RMS-29/40-120) | 118 N·s | 48 / 108 N | 2.4 s | 62.5 / 151 g | Metal reload case: heavy, and "substantial metal" |
| AeroTech G25W | White Lightning, long burn | 117.5 N·s | 22 / 41 N | 5.3 s | 62.5 / 120 g | T/W too low for this vehicle |
| Estes F15 | Black powder | 49.6 N·s | 14.4 / 25.3 N | 3.45 s | 60 / 102 g | Below the vehicle's weight (13.6 N) - **too weak** |

Sources: [ThrustCurve G40W](https://www.thrustcurve.org/motors/AeroTech/G40W),
[AeroTech G40W](https://aerotech-rocketry.com/products/product_e54bba8f-a733-e5ba-b4f6-3db6a2d05b19),
[AeroTech G80T](https://aerotech-rocketry.com/products/product_12ac810d-0472-e322-6e54-c5d35b4eb0e5),
[NAR F15 cert](https://www.nar.org/wp-content/uploads/2014/08/F15.pdf).

**Why G40W for landing.**
- Its regressive thrust curve starts at 74 N, falls through ~37 N at 1.5 s, and
  tails off to ~13 N (about the vehicle's weight) by 2.0 s. That is nearly the
  shape you want for a landing.
- Strong braking comes first, when the vehicle is fast. Weak thrust comes at
  the end, so closed paddles (25% of thrust) can't lift the vehicle off its
  legs after touchdown once the burn clock passes 1.12 s.
- It holds 53.8 g of propellant, with < 80 N average thrust: a model-rocket motor.

**Why G40W for ascent.** It gives a 152 m (498 ft) apogee at 1387 g. Two
identical motors also means one part to buy, store and characterise.

Total for the flight: 107.6 g propellant, 198 N·s. That is Class 1, "two Gs".

## 4.2 Why a solid motor needs a throttle (the core physics)

A landing burn must end at **h = 0 and v ≈ 0 at the same moment**. That is two
conditions. Without a throttle you control only one variable, the ignition
time, so the ignition delay's scatter goes straight into the touchdown error:

- Δh ≈ v_descent x Δt_delay = 22.5 m/s x 0.08 s = **1.8 m** (1σ).
- Stopping 1.8 m high and then free-falling ends at ~6 m/s. Arriving 1.8 m late
  hits harder still.

BPS.space found the same: ignition timing "simply weren't repeatable enough"
([Hackaday 2022](https://hackaday.com/2022/08/05/bps-space-succesfully-lands-a-model-rocket/)).

**Tilting the gimbal to waste thrust is weak:** it only removes 1 - cos θ of
the thrust, 0.75% at 7° and 6% at 20°. A **mechanical exhaust throttle** that
blocks or deflects part of the plume gives a real second control. With it the
guidance can re-plan the burn continuously (see [docs/06](06-gnc.md)).

The authority you need, from the Monte Carlo (300 flights, 0-4 m/s wind,
ignition delay 0.30 ± 0.08 s):

| Paddles closed keep... | k_min | Clean landings | Crashes | Notes |
|---|---|---|---|---|
| 25% of thrust (75% cut) | **0.25** | **90%** | **0%** | Design requirement |
| 40% of thrust (60% cut) | 0.40 | 74% | 5% | Degraded paddles: late ignitions hop up to 4 m |
| 100% (no throttle) | 1.0 | 0% | — | Not viable |

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
  - two flat paddles 20 x 28 mm, pivoting on axes 10 mm below the nozzle exit,
    perpendicular to the motor axis;
  - fully open they sit outside the plume (k = 1);
  - closed they meet at 30-40° to the axis, deflecting most of the jet
    sideways in opposite directions, so the side forces cancel.
- **Materials:**
  - paddle faces: machinable alumina-silicate ceramic ("lava"), or graphite
    plate 4-5 mm thick;
  - arms: 304 stainless;
  - pivots: steel pins in bronze bushings;
  - the servo sits forward of the aft bulkhead behind a heat shield (aluminium
    sheet + ceramic paper).
  - APCP exhaust is ~2000 °C+. 3D-printed parts anywhere in or near the plume
    will fail.
- **Actuator:** KST X08H (or similar HV digital servo) on 2S, 0.09 s/60°,
  2.8 kg·cm ([servodatabase](https://servodatabase.com/servo/kst/x08h)). The
  full open-to-closed travel is ~45° of servo, about 70 ms.
- **Calibration:** on the load-cell stand, map servo pulse to throttle k at
  four points (1.0 / 0.75 / 0.5 / 0.25). Put the numbers in
  `firmware/src/hal/board.h` (`PADDLE_US`) and `THROTTLE_MIN` in
  `firmware/src/core/params.h`.
- **Wear:** inspect after every burn; budget a new pair every 2-3 burns.
- **The paddles only throttle the LANDING motor.** They sit under its nozzle,
  not the ascent motor's.

## 4.4 Dual-motor TVC gimbal

- **Layout:**
  - two aluminium 29 mm motor sleeves bonded into an inner ring, 33 mm
    centre-to-centre;
  - the inner ring pivots on a middle ring (axis Y), which pivots on the outer
    ring fixed to the airframe (axis Z);
  - pivots sit 20 mm forward of the nozzle exits, at station ~1.46 m, so the
    lever to the CG is ~0.45 m at landing.
- **Range:** ±7°. BPS uses ±5° on its mount
  ([BPS TVC](https://bps.space/products/thrust-vector-control)); the BPS
  community wiki works an example at 6.91°
  ([gimbal design](https://github.com/bps-space/bps-space.github.io/blob/master/mechanical/gimbal-design.md)).
- **Servos:** 2 x KST X08H on 2S, pushrods with ball links, about a 2:1 servo-
  to-gimbal ratio.
- **Response:** measure the linkage ratio with the laser-pointer method
  ([docs/09](09-test-plan.md)).
- **Offset thrust:** each motor is 16.5 mm off the centreline. The flight
  software feeds forward a 2.1° trim (asin(e/L)) and leans the body slightly
  the other way so the trim doesn't push the vehicle sideways.
- **Retention:** each motor needs a positive retainer (screw-cap or clip) **on
  the gimbal sleeve**. The thrust ring transfers load through the sleeve and
  pivots into the aft thrust plate.

## 4.5 Ignition

| Item | Choice | Why |
|---|---|---|
| Ascent igniter | Igniter supplied with the motor (FirstFire Jr) | Fired from the club launch controller |
| Landing igniters | **Two** igniters in the landing motor, on two separate MOSFET channels fired together | A no-ignition failure is the worst case; redundancy halves the odds |
| Firing current | >= 2 A per igniter for Estes-type starters (0.5 A no-fire / 2 A all-fire); 1 A for MJG Firewire-type (0.3 A no-fire / 0.6 A all-fire) | ([Estes starter](https://www.erockets.biz/estes-model-rocket-star-tech-starters-igniters-for-flying-model-rockets-est-2303/), [MJG](https://www.erockets.biz/mjg-technology-36-starters-initiators-dipped-2pk-mjg-03)) |
| Pulse | 1.0 s, once per boot | `PYRO_PULSE_MS` |
| Continuity test current | < 10 mA (well below no-fire) | Through a 10 kΩ sense resistor ([docs/05](05-avionics.md)) |

**Ignition delay is the single most important number you must measure.**
- Data source: static-fire 3-5 G40Ws with the exact flight igniters. Log the
  FIRE command and the load cell at >= 1 kHz.
  - Use an ADS1220-class ADC; the HX711's 10-80 Hz is far too slow
    ([Arduino blog](https://blog.arduino.cc/2018/12/17/an-arduino-based-rocket-motor-thrust-test-stand/)).
- Composite (APCP) motors light more slowly and less repeatably than black
  powder. The simulator's assumption of 0.30 ± 0.08 s is a placeholder.
- Set `IGNITION_DELAY_NOMINAL_S` to your measured mean.
- Re-run `sim/scripts/run_montecarlo.py` with your measured σ.

## 4.6 Ejection charges and delays

- The NAR code says: use motors only as the manufacturer instructs.
  - **Landing motor:** choose a configuration with **no ejection charge** that
    the manufacturer permits (e.g. a plugged / "P" variant, or removing the
    charge where the instructions allow it for electronic deployment).
  - **Ascent motor:** its ejection charge would fire into the motor bay
    mid-flight. Use a plugged configuration there too.
- **Ask AeroTech or your vendor** which G40W configuration supports this.
  Never drill, cut or modify a motor.
- If a plugged G40 isn't available, the fallback is the Estes-branded G40 or
  another 29 mm G with a plugged option. Re-run the sim with its `.eng` file.

## 4.7 Truly throttleable alternative: EDF test bed

An 80 mm, 12-blade EDF on 6S gives 3.7 kg (36 N) of thrust
([Motion RC](https://motionrc.com/products/freewing-80mm-12-blade-edf-6s-power-system-w-3658-2150kv-inrunner-motor-e72314)).
The fan, ESC and battery together weigh about 0.75-0.8 kg. That makes a full
rocket with an EDF lander heavier than 1.5 kg, so it becomes high power.

**Recommended use:** build a small, tethered "EDF hopper" with the *same
flight computer and TVC code* to practise hovering and landing for the price
of a battery charge. Examples:
[UrosKukovic EDF lander](https://github.com/UrosKukovic/TVC_Self_Landing_Rocket),
[Hack Club Starkit](https://hardware.hackclub.com/projects/mikuel210/starkit).
