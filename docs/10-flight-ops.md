# 10. Flight operations and checklists

Print this page. Two people: a **flyer** and a **safety/checker** who reads
each line aloud. Nothing gets skipped.

## 10.1 Go / no-go criteria

| Item | Go if |
|---|---|
| Surface wind (landing attempt) | **≤ 3 m/s (6.7 mph)** sustained. For an ascent-only flight, ≤ 20 mph per the NAR code |
| Clouds | Not launching into cloud; RSO approval |
| Field | ≥ 1000 ft minimum dimension (127 N·s total); landing area clear |
| Vehicle | Liftoff mass ≤ 1500 g (weighed today); all checks below passed |
| Battery | 2S at ≥ 8.0 V at the pad (the software refuses to arm below 7.4 V) |
| Parameters | `params.h` matches the measured ignition delay / k_min / mass; firmware version logged |

## 10.2 At home (the day before)

- [ ] Charge the 2S LiPo (balance) and the ground-station battery.
- [ ] Inspect the paddles: erosion < 1 mm; no cracks; pivots free.
- [ ] Inspect the gimbal: pins, backlash < 0.2°, servo horns tight.
- [ ] Leg deploy test x 3, then re-stow and check the release-pin servo.
- [ ] Airbrake petals: release and re-latch x 3; check all 4 snap to 90° and the stops are tight.
- [ ] Backup-chute ejection test with the servo (no pyro).
- [ ] Pack the main chute (30 in): Z-fold, Nomex blanket, shock cord flaked.
- [ ] Clean the lidar window.
- [ ] Flash the firmware; record the git commit hash in the log book.
- [ ] Bench check (USB status line), with no motors and no igniters:
  - `PAD_SAFE cal=1`;
  - tilt the airframe 10°: tilt reads 10° ± 1°;
  - gimbal sweep, paddle sweep, mechanism servos;
  - dummy-load continuity reads 1 with the key in, 0 with the key out.
- [ ] Put an empty SD card in.
- [ ] Pack: motors, igniters, arm key, spare paddles, tools, epoxy, fire
  extinguisher, water, wind meter, scale, laptop.

## 10.3 Prep table (field)

- [ ] Weigh both motors. Check the types (F42T ascent, F25W landing) and the plugged / no-ejection
  configuration.
- [ ] Install the **landing** motor in the LANDING sleeve (the one with the
  paddles), retainer locked.
- [ ] Install **two** landing igniters (A and B) in the landing motor. Leads
  shorted together until the pad.
- [ ] Install the **ascent** motor in the ASCENT sleeve, retainer locked. Its
  igniter is installed at the pad.
- [ ] Weigh the whole rocket: ______ g (≤ 1500). CG: ______ m (≈ 1.02).
- [ ] Legs stowed; release pin in. Airbrake petals latched flush. Main chute
  latched.
- [ ] Key **OUT**. Power on. Status: `PAD_SAFE`, GPS fix, telemetry received.

## 10.4 At the pad

1. [ ] Place the rocket on the pad and point **fin 1 north** (or record the
   heading). Plumb it vertical (< 2°).
2. [ ] Leave it untouched for 5 s → status `cal=1` (pad calibration done).
3. [ ] Connect the landing igniter leads (A, B) to the flight computer's pyro
   connector. **Key still OUT.**
4. [ ] Install the ascent igniter; connect the club launch-controller clips.
   The launch controller is safe, with its interlock key removed.
5. [ ] Clear the pad: only the flyer stays.
6. [ ] **Insert and turn the arm key.** Status → `PAD_ARMED`. Continuity A=1,
   B=1. Buzzer armed tone.
   - If it does not arm, pull the key and troubleshoot. Never bypass a check.
7. [ ] Walk back to the flight line. Ground station shows `PAD_ARMED`, tilt
   < 2°, battery ≥ 8.0 V, GPS fix.
8. [ ] RSO countdown → launch.

## 10.5 Misfire (ascent motor didn't light)

- [ ] Remove the launch-controller interlock. **Wait 60 s.**
- [ ] Approach and **pull the arm key first** (status → `PAD_SAFE`), then the
  ascent igniter.
- [ ] Replace the igniter and restart from 10.4 step 4.

## 10.6 After landing / recovery

1. [ ] Wait until the motor has burnt out and stopped smoking, and the
   telemetry shows `LANDED_SAFE` (or `ABORT_CHUTE` and stationary).
2. [ ] **If the landing motor never lit** (abort with an unfired landing
   motor), treat it as a live misfire:
   - wait 60 s;
   - approach from the side, away from the nozzle;
   - pull the arm key;
   - disconnect the igniters.
3. [ ] Pull the arm key. Power off.
4. [ ] Hot parts: let the gimbal and paddles cool for ≥ 5 min. Gloves.
5. [ ] Pull the SD card → `FLTnnn.CSV`. Back it up before anything else.
6. [ ] Post-flight: compare the logged descent speed, ignition delay (FIRE →
   IGNITION event), touchdown speed and throttle history with the simulation.
   Update the parameters. Re-run the Monte Carlo before the next flight.

## 10.7 Abort cases and what the vehicle does

| Situation | Vehicle response | Crew action |
|---|---|---|
| Tilt > 35° on descent | Main chute | Track it; recover like a misfire if the motor is unlit |
| Outside the fire envelope below 15 m (e.g. airbrakes failed, > 35 m/s) | Main chute; landing motor **not** lit | Same |
| No ignition 0.9 s after FIRE | Main chute | **Hang-fire risk**: keep clear for 60 s after landing |
| Late ignition under the chute | TVC holds the nozzle down | Keep clear; spectators upwind |
| Hard landing / tip-over | Paddles closed, attitude hold until burnout | Wait for burnout; inspect for damage |
