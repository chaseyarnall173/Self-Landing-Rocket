# 11. Bill of materials and budget

Prices are approximate USD (2025-2026). Lines marked * were found in vendor
listings; the rest are typical-price estimates. Check current prices and
availability before ordering.

This is the shopping list. The part-by-part list (all 129 CAD parts, with
quantity, material, how to make each one, mass and file names) is
[`cad/out/bom.csv`](../cad/out/bom.csv).

## 11.1 Vehicle

| Qty | Item | Example source | Each | Total |
|---:|---|---|---:|---:|
| 2 | LOC 3.9 in kraft body tube, 34 in | [LOC Precision](https://locprecision.com/products/cardboard-airframes) | $20 | $40 |
| 2 | LOC 3.9 in coupler, 8 in (avionics bay 160 mm; airbrake sleeve 189 mm) | LOC | $12 | $24 |
| 1 | Nose cone: PETG filament (a LOC plastic nose would add ~47 g and put rev C over 1500 g) | | $20 | $20 |
| 1 | Aircraft birch ply: 1/8 in 12 x 24 in (bulkheads, lids, thrust plate, doublers) + 3/32 in 6 x 12 in (fins); 1/8 in lite ply (sled) | hobby shop | $25 | $25 |
| 1 | CF-nylon filament, 500 g (gimbal, hinges) | | $40 | $40 |
| 1 | TPU-95A filament (feet) | | $25 | $25 |
| 1 | 29 mm kraft motor-mount tube (cut 2 x 113 mm sleeves) + 1 mm aluminium for the thrust caps | LOC / hobby shop | $12 | $12 |
| 2 | Spring-steel clip retainers (0.8 mm spring strip, or motor hooks); screw-cap retainers don't fit the 33 mm spacing | | $3 | $6 |
| 1 | 200 gsm carbon cloth, 0.25 m² (aft liner) | | $15 | $15 |
| 1 | M3 nylon threaded rod + nylon wing nuts (avionics bay) | | $5 | $5 |
| 1 | Pins, bushings, MR63 bearings, M2/M3 stainless hardware | | $25 | $25 |
| 1 | Paddle ceramic (alumina-silicate block) or graphite plate + 304 stainless strip | | $30 | $30 |
| 4 | 8 mm x 7 mm pultruded carbon tube, 0.6 m (legs) | | $6 | $24 |
| 4 | Torsion springs + Kevlar line + syringe dampers | | $4 | $16 |
| 1 | 30 in nylon parachute + Nomex blanket + Kevlar shock cord + swivel | | $45 | $45 |
| 1 | Airbrake petal hinges, torsion springs, stainless pins, latch ring | | $15 | $15 |
| 1 | Epoxy (30-min + high-temp), CA, fillers, paint | | $50 | $50 |
| | | | **Vehicle subtotal** | **≈ $420** |

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
| 3 | 4 g micro servos (airbrake latch, leg pin, chute latch) | $8 | $24 |
| 1 | Pololu 5 V 3 A buck (D24V30F5) | $15 | $15 |
| 2 | 2S 450 mAh LiPo | $12 | $24 |
| 1 | LiPo balance charger + fire bag | $50 | $50 |
| 1 | Arm key switch, fuses, AO3400A FETs, resistors, capacitors, JST-GH kit, perfboard | $40 | $40 |
| 1 | microSD 16 GB | $8 | $8 |
| 1 | Ground station MCU (Feather / spare Teensy) | $25 | $25 |
| | **Avionics subtotal** | | **≈ $500** |

## 11.3 Motors and test equipment

| Qty | Item | Each | Total |
|---:|---|---:|---:|
| 8 | AeroTech F25W landing motors (5 static fires + 3 flights) | $25 | $200 |
| 4 | AeroTech F52C ascent motors (flights F1-F3 + spare) | $25 | $100 |
| 1 | Hazmat shipping (one order) | $40 | $40 |
| 12 | Extra igniters (dual landing igniters) | $3 | $36 |
| 1 | Load cell 20 kg + ADS1220 ADC board + test-stand hardware | $60 | $60 |
| 1 | 2-axis swivel test rig (printed / aluminium) | $40 | $40 |
| 1 | Bench power supply, laser pointer, dummy-load resistors | $40 | $40 |
| 1 | NAR membership (insurance at club launches) + club fees | $60 | $60 |
| | **Motors and test subtotal** | | **≈ $580** |

## 11.4 Total

| | |
|---|---:|
| Vehicle | ≈ $420 |
| Avionics | ≈ $500 |
| Motors, test equipment, club | ≈ $580 |
| Contingency (20%: crashes happen) | ≈ $300 |
| **Total** | **≈ $1,800** |

**Ways to cut cost:**
- prototype with MG90S servos ($5) and skip the GPS for the first ascent-only
  flights;
- build a cheap EDF hopper ([docs/04](04-propulsion.md#47-truly-throttleable-alternative-edf-test-bed))
  to practise the control software without burning motors.
