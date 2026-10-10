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

The CAD ([docs/13](13-cad.md)) has every part in 3D. Printed parts are in
`cad/out/stl/`, laser-cut ply parts in `cad/out/dxf/`, and `cad/out/bom.csv`
lists each part with its material and how to make it.

1. **Cut the tube.** From two 34 in LOC 3.9 in tubes, cut five pieces and
   square every end with a tube-cutting guide or sanding block:

   | Piece | Length | Stations from the nose tip |
   |---|---:|---|
   | Nose ring | 71 mm (2.80 in) | 0.324-0.395 m |
   | Petal band (becomes the 4 petals) | 150 mm (5.91 in) | 0.395-0.545 m |
   | Recovery section | 223 mm (8.78 in) | 0.545-0.768 m |
   | Switch band | 25 mm (0.98 in) | 0.768-0.793 m |
   | Aft tube | 731 mm (28.78 in) | 0.793-1.524 m |

   - Total 1200 mm (47.25 in). Tube A gives the aft tube and switch band;
     tube B gives the other three with ~400 mm spare.
   - Mark the fin and leg lines with a printed wrap guide: 4 fin lines at 90°;
     4 leg lines at 45° between the fins; fin 1 marked **"N"** for the pad
     heading.
2. **Reinforce the aft section.** Lay 2 wraps of 200 gsm carbon cloth with
   epoxy inside the last 6 in of the aft tube (0.5 mm liner). It carries the fin
   roots, leg hinges and thrust plate. A 6 in coupler works too but weighs
   ~32 g more, and the margin is thin.
3. **Fin and leg slots.** Through the tube and liner:
   - 4 fin slots, 107 mm long and 2.7 mm wide, ending 10 mm from the aft end;
   - 4 leg-clevis slots, 10 x 20 mm, centred 124 mm above the aft end on the
     leg lines.
4. **Fins.** Cut 4 fins from 3/32 in aircraft birch ply (`dxf/FIN-01.dxf`: root
   4.23 in, tip 2.16 in, span 2.70 in, sweep 2.07 in). Round the leading edges.
   - **Check:** the 4 fins match within 1 g, and their roots are straight.
5. **Avionics bay.** A 160 mm 3.9 in coupler, centred on the switch band and
   glued to it.
   - Drill through the band and coupler:
     - the arm-key hole (12 mm, at 180°);
     - the main-switch hole (6 mm, at 160°);
     - the leg-release pin hole (2 mm, at 67.5°);
     - 3 static ports (2 mm, at 90°, 210° and 330°).
   - Lids: one 1/8 in ply disc each (`dxf/AV-LID-F.dxf`, `AV-LID-A.dxf`) with a
     printed locating ring (`AV-LID-RING-*.stl`).
   - A removable 1/8 in lite-ply sled (`dxf/AV-SLED.dxf`) on 2 nylon M3 rods.
6. **Recovery bay.**
   - Cut the chute door from the recovery section: 105 mm long, starting
     40 mm from its forward end, 70 mm wide, centred on the 90° (fin 2) line.
     Hinge it at its aft edge (`DOOR-HINGE-*.stl`) and fit the latch lug.
   - Forward bulkhead (`dxf/RECOV-BH.dxf`) with an M4 U-bolt, glued 20 mm aft
     of the recovery section's forward end.
   - Ejector: a ply plate on 2 compression springs (~20 N each) against
     printed seats on the far wall, pushing the chute toward the door. The
     chute-latch servo under the bulkhead holds the door shut with a pin.
   - Kevlar shock cord, 3x the body length.
7. **Nose cone.** Print `NOSE-01.stl` in PETG: 0.8 mm walls, 1.2 mm shoulder.
   It is 375 mm tall with the shoulder, so most printers need it split at
   ~200 mm and glued over a printed sleeve. Drill the nose ring for the
   3 x M3 screws that hold it.
8. **Top airbrake petals** (see [docs/03](03-vehicle-design.md#34-top-airbrake-petals)).
   - Cut the 150 mm petal band into four 90° petals with a fine saw, with the
     cut lines on the leg lines so they don't line up with the fins.
   - Seal the cut edges with thin CA and stiffen each petal's inside face with
     one layer of 2 oz fibreglass.
   - Cut the 189 mm sleeve (a 3.9 in coupler). Cut a latch slot at the aft end
     and a 60 x 77 mm lightening window under each petal.
   - Assembly order:
     1. Glue the sleeve 19 mm into the nose ring.
     2. Glue the other end 20 mm into the recovery section, leaving exactly
        150 mm of sleeve exposed between them.
     3. Bond the fixed hinge halves (`PETAL-HINGE-F.stl`, with the 90° stop)
        to the nose ring.
     4. Bond the moving halves (`PETAL-HINGE-M.stl`) to the petals' forward
        edges.
     5. Fit 1.6 mm stainless pins and a torsion spring (~0.1 N·m) per petal.
   - Latch: bond a `PETAL-TAB` inside each petal's aft edge so it drops into
     the sleeve slot. Fit the latch ring (`LATCH-RING.stl`) inside the sleeve
     and link it to the latch micro servo.
   - **Check:** release the latch 20 times. All 4 petals must snap to 90°
     within 0.2 s, and each stop must hold 5 N pushed by hand.

## 8.2 Motor bay and TVC gimbal

1. **Print the gimbal** in CF-nylon: inner ring, middle ring and two yoke posts
   (`GIMBAL-*.stl`). Dry the filament first; use 4 perimeters and 40-60 %
   infill.
   - Ream the pivot holes and press in 3 mm steel pins (or MR63 bearings).
2. **Motor sleeves.** Two 29 mm kraft motor-mount tubes, 113 mm long, bonded
   into the inner ring with high-temperature epoxy, 33 mm centre-to-centre.
   The gimbal pivot must sit 25 mm above the sleeves' aft ends, at their
   middle, or the sleeves hit the wall at full deflection.
   - Aluminium thrust cap on the forward end of each sleeve.
   - A spring-steel clip retainer on the +Y side of each sleeve. Screw-cap
     retainers are wider than the 33 mm spacing.
   - Print the 13 mm `F25W-SPACER` so both nozzles exit together.
   - **Mark which sleeve is ASCENT (F52C) and which is LANDING (F25W).** The
     landing sleeve is the one with the paddles under it.
3. **Thrust structure.** Cut the thrust plate (`dxf/THRUST-PLATE.dxf`, 1/8 in
   ply, plus the 1 mm aluminium ring) and bond it inside the liner at station
   1.423 m (101 mm above the aft end). Bond the yoke posts under it. Motor thrust
   flows sleeve cap → inner ring → X pins → middle ring → Y pins → posts →
   thrust plate → airframe.
   - **Check:** press the sleeves with 100 N (10 kg) by hand. Nothing flexes
     visibly.
4. **Servos.** Mount two KST X08H servos on printed brackets (`SERVO-BRKT-*.stl`)
   on the plate, at ±Y. Pushrods are 1.5 mm steel with ball links, at about a
   2:1 servo-to-gimbal ratio: servo A drives the middle ring and servo B the
   inner ring.
   - **Check:** move the gimbal by hand through ±7° on both axes. No binding,
     < 0.2° of backlash.
5. **Calibrate the gimbal.**
   - Clamp a laser pointer in a motor sleeve, 2 m from a wall.
   - Step each servo in 20 µs increments and record the dot position. 1° is
     34.9 mm at 2 m.
   - Fit pulse = centre + gain x angle and write the numbers into
     `firmware/src/hal/board.h`.
   - Approach each point from both sides to measure backlash.

## 8.3 Exhaust-paddle throttle

1. **Paddles.** Machine two 20 x 28 x 4 mm paddles from alumina-silicate
   ceramic, or cut them from graphite plate. Bolt them to 1.5 mm 304 stainless
   arms with M2 stainless hardware.
2. **Pivots and link.**
   - The pivot pins sit 8 mm below the landing nozzle exit and 16 mm either
     side of its axis.
   - They run in a 1.5 mm aluminium bracket fixed under the inner gimbal ring,
     so the paddles move with the motor.
   - One KST X08H, strapped to the bracket outboard of the landing motor,
     drives both arms through a crosshead and scissor links, so they close
     symmetrically.
3. **Heat shield.** Put ceramic paper and aluminium tape under the servo and
   on the plume side of the bracket rails. Put **nothing printed** within
   30 mm of the plume.
4. **Check:** open, the paddles clear the nozzle diameter. Closed (~32°), the
   tips meet on the plume axis, with the servo under 50% load. Open, the
   paddles end 1 mm above the aft end of the airframe.

## 8.4 Landing legs

See [docs/07](07-landing-legs.md).

1. Print 4 hinge clevises (`LEG-CLEVIS.stl`, with the 55° stop), 4 knuckle
   inserts (`LEG-INSERT.stl`) and 4 feet (`LEG-FOOT.stl`, TPU-95A at 30 % gyroid).
2. Bond the clevises through their slots to 1/8 in ply doublers on the liner,
   124 mm above the aft end, on the leg lines.
3. Cut the struts: 532 mm of 8 x 7 mm carbon tube. Bond an insert in the root
   end and a syringe damper barrel over the tip end. Bond each foot to its
   damper plunger, set 13 mm off the strut axis so it clears the tube when
   stowed.
4. Fit the 3 mm pins, torsion springs (~0.15 N·m) and detents.
5. Release line: Kevlar loop around all 4 legs at the damper barrels, tied to
   the leg-release pin that pokes out of the switch band at 67.5°.
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
4. **Lidar.** Mount it on its printed bracket (`LIDAR-BRKT.stl`) on the liner at
   315°, 14-30 mm above the aft end, looking down through a replaceable 1 mm
   polycarbonate window. That spot is clear of both plumes and the paddles.
5. **Antennas.** GPS patch under the forward lid; the LoRa antenna (wire
   monopole) runs along the sled, away from the patch.

## 8.6 Final assembly and checks

1. Fit the fins and fillet them with 30-minute epoxy plus silica (radius ~6 mm).
2. Install the gimbal assembly, legs, lidar and paddles.
3. **Spin test.** All servos sweep with the vehicle assembled, with no
   interference: gimbal ±7°, paddles through full travel, legs folded.
4. **Finish.** Fill the spirals, prime, paint light (< 15 g of paint). Keep the
   lidar window and static ports clear.
5. **Weigh and balance.**
   - Liftoff mass with 2 motors (or dummies of equal mass) must be **≤ 1500 g**.
     The CAD predicts 1467 g, so aim for ≤ 1450 g and use the lightening
     options in [docs/03](03-vehicle-design.md#32-mass-budget-target--1500-g-at-liftoff)
     if you are over.
   - Find the CG by balancing on a knife edge; CAD value 0.97 m from the nose.
6. **Inertia.** Hang it bifilar (two strings, 0.5 m apart, length L) and time
   the yaw period. I = m·g·d²·T²/(16π²·L), with d = string spacing and T = the
   period. Write it into `params.h` (`INERTIA_KGM2`) and `designs.py`.
7. **Update the simulation** with the measured mass, CG and inertia, then re-run
   `run_nominal.py` and `run_montecarlo.py`. Re-run the parameter generator
   (`firmware/tools/gen_tables.py`) and `make -C firmware test`.
