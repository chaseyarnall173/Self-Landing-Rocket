# 5. Avionics (flight computer, sensors, actuators, power, pyro)

## 5.1 Block diagram

```
                      +----------------------------- 2S LiPo 450 mAh (7.4-8.4 V) ---------------------------+
                      |                                   |                                                 |
                 [MAIN SWITCH]                      [ARM KEY] (physical, remove-before-flight)               |
                      |                                   |                                                 |
        +-------------+--------------+              PYRO RAIL (fused 10 A)                                  |
        |                            |                    |                                                 |
   HV servos (direct 2S)       5 V 3 A buck          Igniter A    Igniter B   (both in the LANDING motor)    |
   - gimbal Y  (KST X08H)        |      |                |            |                                     |
   - gimbal Z  (KST X08H)        |   mechanism servos   drain        drain                                  |
   - paddles   (KST X08H)        |   (airbrake latch,    |            |                                     |
        | 1000 uF                |    leg pin, chute)    AO3400A     AO3400A  <- gates from Teensy 20 / 21  |
        |                        |    chute latch)       |            |         (100R series, 10k pulldown) |
        |                   Teensy 4.1 VIN               GND          GND                                   |
        |                        |                       |            |                                     |
        |       +----------------+-----------------------+------------+-------------------------------+   |
        |       |  Teensy 4.1 (600 MHz Cortex-M7, FPU)                                                 |   |
        |       |   Wire1 1 MHz : BMI088 IMU (accel +-24 g, gyro 2000 dps)                             |   |
        |       |   Wire 400kHz : BMP390 barometer, u-blox M9N/M10 GPS                                  |   |
        |       |   Serial1     : Benewake TFmini-S lidar (looks down from the aft end)                |   |
        |       |   SPI         : RFM95W 915 MHz LoRa telemetry                                         |   |
        |       |   SDIO        : microSD (flight log)                                                  |   |
        |       |   ADC         : continuity A/B, battery voltage;  D22: arm-key sense                  |   |
        |       +-------------------------------------------------------------------------------------+   |
        +-------------------------------------------------------------------------------------------------+
```

## 5.2 Parts

| Function | Part | Key spec | Why | Source |
|---|---|---|---|---|
| MCU | **Teensy 4.1** | 600 MHz M7, double-precision FPU, 1 MB RAM, built-in SDIO microSD | Plenty of headroom for 1 kHz estimation and the 50 Hz re-plan | [PJRC](https://www.pjrc.com/store/teensy41.html) |
| IMU | **Bosch BMI088** | Accel +-24 g, gyro +-2000 °/s, vibration-robust | BPS wiki's "best choice"; **avoid the MPU6050/9250** (end of life, drift) and the BNO055 (its fusion fails under flight loads) | [Bosch](https://www.bosch-sensortec.com/products/motion-sensors/imus/bmi088/), [BPS wiki](https://github.com/bps-space/bps-space.github.io/blob/master/electronics/imu-selection.md) |
| Barometer | **Bosch BMP390** | +-0.03 hPa relative (~25 cm), up to 200 Hz | Altitude through the whole flight | [Bosch](https://www.bosch-sensortec.com/products/environmental-sensors/pressure-sensors/pressure-sensors-bmp390.html) |
| Lidar | **Benewake TFmini-S** | 12 m (90% reflectivity) / **7 m on 10% reflectivity**, 70 klux sunlight, 100-1000 Hz | Precise height for the last ~7 m and touchdown detection. The ignition decision (~15 m) uses baro + IMU. **VL53L1X-type ToF sensors do not work outdoors.** | [Benewake](https://en.benewake.com/TFminiS/), [Pololu VL53L1X](https://www.pololu.com/product/3415) |
| Lidar (longer reach) | Benewake TF02-Pro | 40 m / 13.5 m on 10% reflectivity, 100 klux, 50 g | Use it if you want lidar at the ignition altitude | [datasheet](https://en.benewake.com/uploadfiles/2024/04/20240426135509321.pdf) |
| GPS | u-blox NEO-M9N (or M10) | up to 25 Hz | **Lateral velocity** for the landing burn. Scout E tipped over from horizontal drift traced to a bad GPS signal ([Hackaday](https://hackaday.com/2020/11/27/so-close-to-landing-a-model-rocket-on-its-tail/)) | [SparkFun](https://www.sparkfun.com/sparkfun-gps-breakout-neo-m9n-u-fl-qwiic.html) |
| Telemetry | HopeRF RFM95W 915 MHz LoRa | +20 dBm | 5 Hz summary; full-rate data stays in the onboard log | [HopeRF](https://hoperf.com/modules/lora/RFM95W.html) |
| Gimbal + paddle servos | 3 x **KST X08H** | 0.09 s/60°, 2.8 kg·cm, runs directly on 2S | Fast, HV, ~8 g. MG90S (0.10 s/60°, 4.8-6 V) is fine for bench prototypes | [X08H](https://servodatabase.com/servo/kst/x08h), [MG90S](https://servodatabase.com/servo/turnigy/mg90s) |
| Mechanism servos | 3 x 3.7-4.5 g micro servos | 5 V | Airbrake latch, leg pin, chute latch: no black powder needed | |
| Pyro FETs | 2 x AO3400A (or IRLML6344) | logic-level, Rds(on) specified at Vgs = 2.5 V | Switch ~4-6 A igniter pulses from 3.3 V logic | |
| Battery | 2S LiPo 450 mAh 45C | 7.4-8.4 V | Runs HV servos directly; ~15 min of pad time | |
| Regulator | Pololu D24V30F5 (5 V 3 A buck) | | Teensy VIN, lidar, micro servos | |

## 5.3 Pyro channel (x2) - schematic description

```
 PYRO RAIL (after ARM KEY + 10 A fuse) ──┬──────────── igniter ──────────┬── node D (drain)
                                          │                               │
                                          │                        AO3400A drain
                                          │                               │ source ── GND
                                          │        Teensy pin ──100Ω── gate
                                          │                               └─10kΩ── GND (gate pull-down)
                                          │
  continuity:  node D ──100kΩ──┬── ADC (A0 / A1)
                               └──10kΩ── GND
```

How the continuity sense works:
- **Key in, igniter present:** node D sits at the pyro rail (~8 V), so the ADC
  reads ~0.76 V. That is above the 0.4 V threshold, so continuity is OK.
- **No igniter:** the divider pulls node D to 0 V.
- **Test current:** ~76 µA, about 4000x below the 0.3-0.5 A no-fire current.

How the channel stays safe:
- **No key, no fire:** with the arm key out, the pyro rail is physically dead
  and firing is impossible. This is the primary safety, independent of
  software.
- **Pull-downs:** the 10 kΩ gate pull-downs keep the FETs off while the Teensy
  boots and while its pins float.
- **Software:**
  - the FSM only enables pyro in `PAD_ARMED`, which needs the key, verticality
    and continuity on both igniters;
  - the FIRE command also passes envelope checks;
  - the HAL fires one 1 s pulse per boot
    (`firmware/src/hal/actuators.cpp`).
- **Brownout protection:** a Schottky diode plus 470 µF on the buck input lets
  the Teensy ride through the current spike of a shorted igniter. An optional
  separate 2S 300 mAh pyro battery (+20 g) isolates it completely.

([Pyro-channel design discussion](https://www.rocketryforum.com/threads/pyro-channel-for-custom-flight-computer-inquiry.166899/),
[Altus Metrum pyro current](https://altusmetrum.org/FAQs/pyro_current.html))

## 5.4 Pin map (firmware/src/hal/board.h)

| Teensy pin | Function |
|---|---|
| 17 / 16 (Wire1) | BMI088 (accel 0x18, gyro 0x68), 1 MHz |
| 18 / 19 (Wire) | BMP390 (0x77), GPS (0x42), 400 kHz |
| 0 / 1 (Serial1) | TFmini-S RX / TX, 115200 |
| 10, 11, 12, 13, 30, 31 | RFM95W CS, MOSI, MISO, SCK, IRQ, RST |
| 2, 3 | Gimbal servos Y, Z (333 Hz) |
| 4 | Paddle servo (333 Hz) |
| 7 | Top-airbrake latch (50 Hz); pin 8 is spare |
| 28, 29 | Leg pin, chute latch (50 Hz) |
| 20, 21 | Pyro gates A, B |
| 14 (A0), 15 (A1) | Continuity sense A, B |
| 22 | Arm-key sense (LOW = armed) |
| 23 (A9) | Battery voltage (1/3 divider) |
| 37 | Buzzer |

Servo pins are grouped so that pins on the same FlexPWM sub-module share one
PWM frequency: 333 Hz for the digital servos, 50 Hz for the analogue micro
servos.

## 5.5 Construction notes

- **Board:** start on perfboard with breakouts. Move to a 2-layer PCB (KiCad;
  ~$20 for 5 from a fab) once the design is frozen. Mount the IMU **rigidly on
  the board**, as close to the vehicle axis as you can. Record its orientation
  in `IMU_MAP`.
- **Wiring:**
  - JST-GH or Molex PicoBlade locking connectors;
  - strain-relieve every wire to the gimbal, because it moves;
  - twist the servo signal and ground wires.
- **Barometer:** static ports are 3 x 2 mm holes around the avionics-bay
  circumference. Seal the bay so the motor bay can't pressurise it. Cover the
  sensor with open-cell foam (light and wind).
- **Lidar:** mount it at the aft ring looking straight down, offset from both
  nozzles and outside the paddle sweep. Shield the lens from soot: replaceable
  polycarbonate window, cleaned after each burn.
- **GPS antenna:** at the top of the avionics bay, clear of carbon fibre and of
  the 915 MHz antenna.
- **Bench mode:** the 5 Hz USB status line prints the phase, calibration, key,
  continuity A/B, battery, tilt, altitude and lidar. Use resistor dummy loads
  (2.2 Ω, 5 W) in place of igniters for every bench test.
