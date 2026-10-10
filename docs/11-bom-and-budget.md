# 11. Bill of materials and budget

Prices are approximate USD (2025-2026). Lines marked * were found in vendor
listings; the rest are typical-price estimates. Check current prices and
availability before ordering.

## 11.1 Vehicle

| Qty | Item | Example source | Each | Total |
|---:|---|---|---:|---:|
| 2 | LOC 3.9 in kraft body tube, 34 in | [LOC Precision](https://locprecision.com/products/cardboard-airframes) | $20 | $40 |
| 1 | LOC 3.9 in coupler (avionics bay) + extra coupler (aft reinforcement) | LOC | $12 | $24 |
| 1 | Nose cone: PETG filament (or LOC 3.9 in plastic nose, 5 oz) | [LOC nose cones](https://locprecision.com/product/nose-cones/) | $20 | $20 |
| 1 | 1/8 in aircraft birch plywood, 12 x 24 in (fins, bulkheads, rings) | hobby shop | $15 | $15 |
| 1 | CF-nylon filament, 500 g (gimbal, hinges) | | $40 | $40 |
| 1 | TPU-95A filament (feet) | | $25 | $25 |
| 2 | 29 mm aluminium thin-wall tube, 125 mm (motor sleeves) | McMaster / Online Metals | $10 | $20 |
| 2 | 29 mm motor retainers | Aero Pack-style screw retainers ([review](https://www.rocketreviews.com/review-180703144225.html)) | $25 | $50 |
| 1 | Pins, bushings, MR63 bearings, M2/M3 stainless hardware | | $25 | $25 |
| 1 | Paddle ceramic (alumina-silicate block) or graphite plate + 304 stainless strip | | $30 | $30 |
| 4 | 8 mm x 6 mm pultruded carbon tube, 0.6 m | | $6 | $24 |
| 4 | Torsion springs + Kevlar line + syringe dampers | | $4 | $16 |
| 1 | 30 in nylon parachute + Nomex blanket + Kevlar shock cord + swivel | | $45 | $45 |
| 1 | 9 in drogue + swivel | | $15 | $15 |
| 1 | Epoxy (30-min + high-temp), CA, fillers, paint | | $50 | $50 |
| | | | **Vehicle subtotal** | **≈ $440** |

## 11.2 Avionics

| Qty | Item | Each | Total |
|---:|---|---:|---:|
| 1 | Teensy 4.1 ([PJRC](https://www.pjrc.com/store/teensy41.html)) | $32* | $32 |
| 1 | BMI088 breakout (Seeed Grove) | $15* | $15 |
| 1 | BMP390 breakout (Adafruit) | $13* | $13 |
| 1 | Benewake TFmini-S lidar | $40 | $40 |
| 1 | u-blox NEO-M9N breakout (or a cheaper M10 module) | $70 | $70 |
| 2 | RFM95W 915 MHz breakouts (flight + ground station) | $20 | $40 |
| 3 | KST X08H HV digital servo (gimbal x2, paddles) | $35 | $105 |
| 4 | 4 g micro servos (latches, pins) | $8 | $32 |
| 1 | Pololu 5 V 3 A buck (D24V30F5) | $15 | $15 |
| 2 | 2S 450 mAh LiPo | $12 | $24 |
| 1 | LiPo balance charger + fire bag | $50 | $50 |
| 1 | Arm key switch, fuses, AO3400A FETs, resistors, capacitors, JST-GH kit, perfboard | $40 | $40 |
| 1 | microSD 16 GB | $8 | $8 |
| 1 | Ground station MCU (Feather / spare Teensy) | $25 | $25 |
| | **Avionics subtotal** | | **≈ $510** |

## 11.3 Motors and test equipment

| Qty | Item | Each | Total |
|---:|---|---:|---:|
| 12 | AeroTech G40W (5 static fires + 3 flights x 2 + spares) ([AeroTech](https://aerotech-rocketry.com/products/product_e54bba8f-a733-e5ba-b4f6-3db6a2d05b19)) | $44* | $528 |
| 1 | Hazmat shipping (one order) | $40 | $40 |
| 12 | Extra igniters (dual landing igniters) | $3 | $36 |
| 1 | Load cell 20 kg + ADS1220 ADC board + test-stand hardware | $60 | $60 |
| 1 | 2-axis swivel test rig (printed / aluminium) | $40 | $40 |
| 1 | Bench power supply, laser pointer, dummy-load resistors | $40 | $40 |
| 1 | NAR membership (insurance at club launches) + club fees | $60 | $60 |
| | **Motors and test subtotal** | | **≈ $800** |

## 11.4 Total

| | |
|---|---:|
| Vehicle | ≈ $440 |
| Avionics | ≈ $510 |
| Motors, test equipment, club | ≈ $800 |
| Contingency (20%: crashes happen) | ≈ $350 |
| **Total** | **≈ $2,100** |

**Ways to cut cost:**
- prototype with MG90S servos ($5) and skip the GPS for the first ascent-only
  flights;
- build a cheap EDF hopper ([docs/04](04-propulsion.md#47-truly-throttleable-alternative-edf-test-bed))
  to practise the control software without burning $44 motors.
