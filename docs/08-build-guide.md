# 8. Build guide (step by step)

Work in this order. Each stage ends with a check you can run before moving on.
**Weigh every part** and record it in `sim/rocketsim/designs.py`
(`SLR1_ITEMS`). Re-run `python sim/scripts/mass_budget.py` as you go.

## 8.0 Tools and shop setup

- **Tools:** digital scale (0.1 g), calipers, a 3D printer (PETG and CF-nylon
  capable, hardened nozzle), a fine-tooth saw, a sanding block and 120-400
  grit paper, Dremel, drill and bits.
- **Electronics:** soldering station, crimpers for JST-GH, multimeter, bench
  power supply.
- **Materials:** 30-minute epoxy, high-temperature epoxy, CA glue, microballoons
  or 406 colloidal silica, masking tape, nitrile gloves, respirator (for
  sanding fibreglass/carbon and for epoxy fumes).
- **Batteries:** LiPo balance charger and a fire bag.

## 8.1 Airframe

1. **Cut the tube.** Cut the 3.9 in LOC tube to 43.5 in. Square the ends with a
   tube-cutting guide or sanding block.
   - Mark the fin and leg lines with a printed wrap guide: 4 fin lines at 90°;
     4 leg lines at 45° between the fins; fin 1 marked **"N"** for the pad
     heading.
2. **Reinforce the aft section.** Bond a 6 in length of coupler (or 2 wraps of
   carbon sleeve plus epoxy) inside the aft end. It carries the fin roots, leg
   hinges and gimbal frame.
3. **Fin slots.** Cut through-the-wall slots, 4.7 in long, starting flush with
   the aft end. Leave 0.5 in at the very end uncut for strength if the gimbal
   frame doesn't cover it.
4. **Fins.** Cut 4 fins from 1/8 in birch ply and round the leading edges.
   - **Check:** the 4 fins match within 1 g, and their roots are straight.
5. **Avionics-bay coupler.** A 3.9 in coupler with two 1/8 in ply bulkheads.
   - Drill 3 x 2 mm static ports around the circumference, mid-bay.
   - Fit a removable sled and an M3 threaded-rod pair.
6. **Recovery bay.**
   - Forward bulkhead with a U-bolt.
   - Spring-plate ejector for the main chute (compression spring, ~40 N), held
     by the chute-latch servo pin.
   - Kevlar shock cord, 3x the body length.
7. **Nose cone.** Print it in PETG: 0.8 mm walls, 15% gyroid infill in the
   shoulder.
   - The tip holds a 40 mm drogue cup with a 20 N spring, a printed cap and a
     servo latch.
   - Hard-mount the drogue leash to a ply bulkhead in the nose shoulder, with
     a ball-bearing swivel.
   - The drogue-cut servo pin is at that bulkhead.
   - **Check:** pull the cup latch by hand. The cap and drogue must clear the
     tip within 0.3 s.

## 8.2 Motor bay and TVC gimbal

1. **Print the gimbal rings** in CF-nylon: outer, middle and inner. Dry the
   filament first.
   - Ream the pivot holes and press in 3 mm steel pins and bronze bushings (or
     MR63 bearings).
2. **Motor sleeves.** Two 29 mm thin-wall aluminium tubes, 125 mm long, bonded
   into the inner ring with high-temperature epoxy, 33 mm centre-to-centre.
   - Add a screw-cap motor retainer, or a clip, on each sleeve.
   - **Mark which sleeve is ASCENT and which is LANDING** (the landing sleeve
     is the one with the paddles under it).
3. **Servos.** Mount two KST X08H servos on the outer ring. Pushrods are
   1.5 mm steel with ball links; aim for about a 2:1 servo-to-gimbal ratio.
   - **Check:** move the gimbal by hand. No binding, < 0.2° of backlash.
4. **Thrust structure.** Bond the outer ring to a ply + 1 mm aluminium thrust
   plate, and that to the reinforced aft section. Motor thrust must flow
   sleeve → inner ring → pins → outer ring → thrust plate → airframe.
   - **Check:** press the sleeves with 100 N (10 kg) by hand. Nothing flexes
     visibly.
5. **Calibrate the gimbal.**
   - Clamp a laser pointer in a motor sleeve, 2 m from a wall.
   - Step each servo in 20 µs increments and record the dot position. 1° is
     34.9 mm at 2 m.
   - Fit pulse = centre + gain x angle and write the numbers into
     `firmware/src/hal/board.h`.
   - Approach each point from both sides to measure backlash.

## 8.3 Exhaust-paddle throttle

1. **Paddles.** Machine two 20 x 28 x 4 mm paddles from alumina-silicate
   ceramic, or cut them from graphite plate. Bolt them to 304 stainless arms
   (1.5 mm) with M2 stainless hardware.
2. **Pivots and link.** Pivot pins sit 10 mm below the landing nozzle exit, in
   a stainless bracket fixed to the inner gimbal ring, so the paddles move with
   the motor.
   - One KST X08H drives both arms through a scissor link, so they close
     symmetrically.
3. **Heat shield.** Wrap the servo and linkage in ceramic paper plus aluminium
   tape. Put **nothing printed** within 30 mm of the plume.
4. **Check:** the full stroke opens to clear the nozzle diameter and closes to
   the 30-40° deflector angle, with the servo under 50% load.

## 8.4 Landing legs

See [docs/07](07-landing-legs.md).

1. Print 4 hinge clevises and 4 feet (TPU-95A lattice).
2. Bond the clevises to internal ply doublers at station 1.40 m, on the leg
   lines.
3. Cut the struts: 0.60 m of 8 mm carbon tube, ends plugged with printed
   inserts, plus the dampers.
4. Fit torsion springs, the 55° stops and the detents.
5. Release line: Kevlar loop around all 4 legs, through the leg-pin servo's pin.
6. **Check:** 20 deploys in a row with full lock, each in under 0.3 s.

## 8.5 Avionics

1. **Perfboard prototype.** Teensy 4.1, BMI088, BMP390, buck regulator, 2
   pyro-FET channels per [docs/05](05-avionics.md#53-pyro-channel-x2---schematic-description),
   servo headers, arm-key connector, LoRa, GPS header.
   - Mount the BMI088 rigidly and note its orientation for `IMU_MAP`.
2. **Bench bring-up.**
   - Flash the firmware (`pio run -e teensy41 -t upload`). Watch the USB status
     line: `PAD_SAFE cal=1 key=0 ...`.
   - Tilt the board to check the attitude estimate and its axes.
   - Fit 2.2 Ω / 5 W dummy loads and confirm continuity reads 1 only with the
     key in.
   - Pull the key and confirm the phase returns to `PAD_SAFE`.
3. **Wiring harness.**
   - Servo extensions to the motor bay through a printed grommet in the aft
     avionics bulkhead.
   - The lidar cable runs along a leg line.
   - The landing-igniter leads come through the gimbal sleeve's forward end to
     a 2-pin locking connector, **unplugged until the pad**.
4. **Lidar.** Mount it at the aft ring looking down, with a replaceable window.
5. **Antennas.** GPS on top of the bay; the LoRa antenna (wire monopole) runs
   along the outside of the coupler.

## 8.6 Final assembly and checks

1. Fit the fins and fillet them with 30-minute epoxy plus silica (radius ~6 mm).
2. Install the gimbal assembly, legs, lidar and paddles.
3. **Spin test.** All servos sweep with the vehicle assembled, with no
   interference: gimbal ±7°, paddles through full travel, legs folded.
4. **Finish.** Fill the spirals, prime, paint light (< 15 g of paint). Keep the
   lidar window and static ports clear.
5. **Weigh and balance.**
   - Liftoff mass with 2 motors (or dummies of equal mass) must be **≤ 1500 g**,
     target ≤ 1400 g.
   - Find the CG by balancing on a knife edge; target 1.03 m from the nose.
6. **Inertia.** Hang it bifilar (two strings, 0.5 m apart, length L) and time
   the yaw period. I = m·g·d²·T²/(16π²·L), with d = string spacing and T = the
   period. Write it into `params.h` (`INERTIA_KGM2`) and `designs.py`.
7. **Update the simulation** with the measured mass, CG and inertia, then re-run
   `run_nominal.py` and `run_montecarlo.py`. Re-run the parameter generator
   (`firmware/tools/gen_tables.py`) and `make -C firmware test`.
