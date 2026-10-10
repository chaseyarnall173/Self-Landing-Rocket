"""Backup-chute bay, avionics bay and the wiring runs. Modelled in place."""
import math

import cadquery as cq

from . import dims as D
from .catalog import Inst, Part
from .geom import (V, aim, bent_rod, box, cbox, cyl, frame, fuse, half_angle,
                   arc_tube, polar, rod, servo, spring, tube)

MICRO = dict(L=20.0, W=8.5, H=17.0, tab_L=26.0, tab_t=1.2, horn=8.5)   # 3.7-4.5 g micro servo


def hexnut(af, t, hole_r, z0, x=0.0, y=0.0):
    nut = cq.Workplane("XY").polygon(6, af / math.cos(math.pi / 6)).extrude(t).val()
    nut = nut.cut(cyl(hole_r, -1, t + 1))
    return nut.translate(V(x, y, z0))


def ball_link(p0, p1, r=0.75, ball=2.0):
    """Pushrod with a ball link at each end."""
    return fuse([rod(r, p0, p1), cq.Solid.makeSphere(ball, V(*p0)), cq.Solid.makeSphere(ball, V(*p1))])


def disc(r, z0, z1, holes=()):
    d = cyl(r, z0, z1)
    for (x, y, hr) in holes:
        d = d.cut(cyl(hr, z0 - 1, z1 + 1, x, y))
    return d


# ================================================================ chute bay
def recovery_bulkhead():
    z0, z1 = D.z(D.RECOV_BH_S[1]), D.z(D.RECOV_BH_S[0])
    c = polar(42.0, 190)
    return disc(D.R_IN, z0, z1, [(-10, 0, 2.1), (10, 0, 2.1), (c.x, c.y, 2.6)])


def ubolt():
    zb = D.z(D.RECOV_BH_S[1])           # aft face
    return bent_rod(2.0, [(-10, 0, zb + 7.0), (-10, 0, zb - 13), (10, 0, zb - 13), (10, 0, zb + 7.0)])


def chute_servo_frame():
    zb = D.z(D.RECOV_BH_S[1])
    return frame((0, 38.25, zb - 3.3 - MICRO["L"] / 2), (0, 0, 1), (1, 0, 0))


def door_pin_z():
    zf = D.z(D.DOOR_S[0])
    return zf - 18, zf + 1.5


# ================================================================ avionics bay
ZF = D.z(D.AVBAY_S[0])        # 824 forward end of the coupler
ZA = D.z(D.AVBAY_S[1])        # 664 aft end
ZB = D.z(sum(D.BAND_S) / 2)   # 743.5 band centre
SLED_X = D.PLY / 2


def av_lid(forward=True):
    t = D.PLY
    rods = [(0, D.AV_ROD_Y, 1.7), (0, -D.AV_ROD_Y, 1.7)]
    if forward:
        c = polar(42.0, 190)
        holes = rods + [(c.x, c.y, 2.6)]
        lid = disc(D.R_IN, ZF, ZF + t, holes)
    else:
        g = polar(39.0, 250)
        lc = polar(42.5, 315)
        holes = rods + [(g.x, g.y, 4.6), (lc.x, lc.y, 2.0)]
        lid = disc(D.R_IN, ZA - t, ZA, holes)
    return lid


def lid_ring(forward=True):
    """Printed locating ring that keys the lid into the coupler."""
    r0, r1 = D.R_CPL_IN - 0.1, D.R_CPL_IN - 3.0
    return tube(r0, r1, ZF - 3.0, ZF) if forward else tube(r0, r1, ZA, ZA + 3.0)


def legpin_servo_frame():
    az = math.radians(D.LEGPIN_AZ)
    t = (math.sin(az), -math.cos(az), 0)       # shaft on the side away from the sled tube
    c = polar(37.0, D.LEGPIN_AZ, ZB - 7.5)
    return frame(c, (0, 0, 1), t)


# ================================================================ parts list
def parts():
    out = []
    P = Part

    # ---------------- backup chute bay
    out.append(Inst(P("RECOV-BH", "Recovery bulkhead, 1/8 in ply, U-bolt + cable holes", recovery_bulkhead(),
                      "ply", "cut", "Backup chute bay", "couplers",
                      flat=cq.Workplane("XY").circle(D.R_IN))))
    out.append(Inst(P("UBOLT", "M4 U-bolt, shock-cord anchor", ubolt(), "steel", "buy", "Backup chute bay",
                      "chute", mass_g=8.0)))
    zt = D.z(D.RECOV_BH_S[0])
    nut = P("NUT-M4", "M4 nut", hexnut(7.0, 3.2, 2.0, 0.0), "steel", "buy", "Backup chute bay", "chute",
            mass_g=1.2)
    for i, x in enumerate((-10, 10)):
        out.append(Inst(nut, cq.Location(V(x, 0, zt)), n=i + 1))

    pack_c = (0.0, 18.0)
    blanket = cyl(28.0, 842, 918, *pack_c).cut(cyl(26.8, 843, 917, *pack_c))
    out.append(Inst(P("CHUTE-NOMEX", "Nomex chute protector (9 in), wrapped", blanket, "nomex", "buy",
                      "Backup chute bay", "chute", mass_g=14.0)))
    out.append(Inst(P("CHUTE-30", "Backup main parachute, 30 in nylon, Z-folded", cyl(26.8, 843, 917, *pack_c),
                      "nylon_chute", "buy", "Backup chute bay", "chute", mass_g=42.0,
                      note="Packed size about 54 x 74 mm.")))
    cord = cyl(9.0, 860, 930, -32, -22)
    out.append(Inst(P("SHOCK-CORD", "Kevlar shock cord, 4.5 m, flaked and bundled", cord, "kevlar", "buy",
                      "Backup chute bay", "chute", mass_g=14.0)))
    zb = D.z(D.RECOV_BH_S[1])
    leads = fuse([rod(1.4, (0, -2.5, zb - 16.5), (-30, -20, 931.5)),
                  bent_rod(1.4, [(-32, -22, 858.0), (-18.0, -5.0, 858.0), (-18.4, -5.0, 870.0)])])
    out.append(Inst(P("SHOCK-CORD-RUN", "Shock cord to U-bolt and riser", leads, "kevlar", "buy",
                      "Backup chute bay", "chute", mass_g=2.0)))
    out.append(Inst(P("EJECT-PLATE", "Ejector plate, 2 mm ply", box(-20, 20, -14.0, -12.0, 848, 912),
                      "ply", "cut", "Backup chute bay", "chute")))
    sp = P("EJECT-SPRING", "Ejector compression spring, ~20 N each", spring(5.0, 0.9, 4.2, 33.4,
           base=(0, -14.0, 0), axis=(0, -1, 0)), "steel", "buy", "Backup chute bay", "chute", mass_g=1.5)
    cup = box(-7, 7, -D.R_IN - 0.5, -47.4, -7, 7).intersect(cyl(D.R_IN, -8, 8))
    cupp = P("EJECT-CUP", "Ejector spring seat", cup.cut(cyl(5.6, -8, 8).rotate(V(), V(1, 0, 0), 90).translate(V(0, -48.5, 0))),
             "cf_nylon", "print", "Backup chute bay", "chute", stl=True)
    for i, zz in enumerate((862, 898)):
        out.append(Inst(sp, cq.Location(V(0, 0, zz)), n=i + 1))
        out.append(Inst(cupp, cq.Location(V(0, 0, zz)), n=i + 1))

    body, horn, tip = servo(**MICRO, horn_deg=90)
    fr = chute_servo_frame()
    out.append(Inst(P("SERVO-CHUTE", "Chute-latch micro servo (4.3 g)", body, "servo", "buy", "Backup chute bay",
                      "chute", mass_g=4.3), fr))
    out.append(Inst(P("HORN-CHUTE", "Servo horn", horn, "cf_nylon", "buy", "Backup chute bay", "chute",
                      mass_g=0.3), fr))
    z0p, z1p = door_pin_z()
    out.append(Inst(P("CHUTE-PIN", "Chute-latch pin, 1.8 mm steel", rod(0.9, (0, 47.3, z0p), (0, 47.3, z1p)),
                      "steel", "buy", "Backup chute bay", "chute")))
    out.append(Inst(P("CHUTE-LINK", "Servo-to-pin link", ball_link(tuple(_world(tip, fr)), (0.0, 47.3, z1p - 1.0), 0.6, 1.2),
                      "steel", "buy", "Backup chute bay", "chute", mass_g=0.4)))
    brk = box(-9.0, 3.3, 34.0, 42.5, zb - 3.3, zb)
    out.append(Inst(P("CHUTE-SERVO-BRKT", "Chute servo mount", brk, "cf_nylon", "print", "Backup chute bay",
                      "chute", stl=True)))

    # ---------------- avionics bay structure
    out.append(Inst(P("AV-LID-F", "Avionics bay forward lid, 1/8 in ply", av_lid(True), "ply", "cut",
                      "Avionics bay", "couplers", flat=cq.Workplane("XY").circle(D.R_IN))))
    out.append(Inst(P("AV-LID-A", "Avionics bay aft lid, 1/8 in ply, grommet + cable holes",
                      av_lid(False), "ply", "cut", "Avionics bay", "couplers", flat=cq.Workplane("XY").circle(D.R_IN))))
    for fwd in (True, False):
        out.append(Inst(P("AV-LID-RING-F" if fwd else "AV-LID-RING-A", "Lid locating ring, PETG", lid_ring(fwd),
                          "petg", "print", "Avionics bay", "couplers", stl=True, color=(0.85, 0.85, 0.82))))
    r3 = P("AV-ROD", "M3 nylon threaded rod, 175 mm", rod(1.5, (0, 0, ZA - D.PLY - 3.5), (0, 0, ZF + D.PLY + 3.5)),
           "cf_nylon", "buy", "Avionics bay", "couplers", color=(0.92, 0.92, 0.88))
    n3 = P("NUT-M3", "M3 nylon wing nut", hexnut(5.5, 2.4, 1.5, 0.0), "cf_nylon", "buy", "Avionics bay", "couplers",
           mass_g=0.2, color=(0.92, 0.92, 0.88))
    k = 1
    for i, y in enumerate((D.AV_ROD_Y, -D.AV_ROD_Y)):
        out.append(Inst(r3, cq.Location(V(0, y, 0)), n=i + 1))
        for zz in (ZF + D.PLY, ZA - D.PLY - 2.4):
            out.append(Inst(n3, cq.Location(V(0, y, zz)), n=k))
            k += 1
    s0, s1 = D.z(D.SLED_S[1]), D.z(D.SLED_S[0])
    sled = box(-SLED_X, SLED_X, -38.0, 38.0, s0, s1)
    out.append(Inst(P("AV-SLED", "Avionics sled, 1/8 in lite ply", sled, "liteply", "cut", "Avionics bay", "couplers",
                      flat=cq.Workplane("XY").rect(76.0, s1 - s0))))
    st = P("AV-SLED-TUBE", "Sled rod guide, printed", tube(2.0, 1.6, s0, s1), "cf_nylon", "print",
           "Avionics bay", "couplers", stl=True)
    for i, y in enumerate((D.AV_ROD_Y, -D.AV_ROD_Y)):
        out.append(Inst(st, cq.Location(V(0, y, 0)), n=i + 1))

    # ---------------- avionics, +X face (perfboard)
    pb_x0 = SLED_X + 3.0
    so = P("STANDOFF-3", "Nylon standoff 3 mm", rod(2.0, (SLED_X, 0, 0), (pb_x0, 0, 0)), "cf_nylon", "buy",
           "Avionics bay", "avionics", mass_g=0.1)
    for i, (y, zz) in enumerate(((-31, 688), (17, 688), (-31, 800), (17, 800))):
        out.append(Inst(so, cq.Location(V(0, y, zz)), n=i + 1))
    px1 = pb_x0 + 1.6
    out.append(Inst(P("PERFBOARD", "Perfboard 54 x 120 mm (flight computer carrier)", box(pb_x0, px1, -34, 20, 684, 804),
                      "perfboard", "buy", "Avionics bay", "avionics")))
    # Teensy 4.1 on two header strips
    ty0, ty1, tz0, tz1 = -2.0, 15.78, 742.0, 802.96
    hdr = fuse([box(px1, px1 + 8.5, ty0, ty0 + 2.54, tz0 + 1, tz1 - 1),
                box(px1, px1 + 8.5, ty1 - 2.54, ty1, tz0 + 1, tz1 - 1)])
    out.append(Inst(P("HEADERS", "Female header strips 2 x 24", hdr, "electronics", "buy", "Avionics bay",
                      "avionics", mass_g=2.0)))
    tx0 = px1 + 8.5
    teensy = fuse([box(tx0, tx0 + 1.6, ty0, ty1, tz0, tz1),
                   cbox(tx0 + 1.6 + 0.6, (ty0 + ty1) / 2, 775, 1.2, 10, 10),
                   box(tx0 + 1.6, tx0 + 3.6, ty0 + 3, ty1 - 3, tz0, tz0 + 14),
                   box(tx0 + 1.6, tx0 + 4.6, (ty0 + ty1) / 2 - 4, (ty0 + ty1) / 2 + 4, tz1 - 4, tz1 + 1.5)])
    out.append(Inst(P("TEENSY-41", "PJRC Teensy 4.1 (600 MHz, microSD log)", teensy, "pcb", "buy", "Avionics bay",
                      "avionics", mass_g=6.0)))
    imu = fuse([box(px1 + 5, px1 + 6.6, -10, 10, 690, 730), cbox(px1 + 7.1, 0, 710, 1.0, 3.0, 4.5),
                box(px1 + 6.6, px1 + 12.6, -4, 4, 724, 730)])
    out.append(Inst(P("IMU-BMI088", "Bosch BMI088 IMU breakout (Seeed Grove), on the axis", imu, "pcb", "buy",
                      "Avionics bay", "avionics", mass_g=3.0, color=(0.15, 0.35, 0.75))))
    out.append(Inst(P("STANDOFF-5", "IMU standoffs (4)", fuse([rod(1.5, (px1, y, zz), (px1 + 5, y, zz))
                      for y in (-7, 7) for zz in (693, 727)]), "cf_nylon", "buy", "Avionics bay", "avionics",
                      mass_g=0.3)))
    baro = fuse([box(px1 + 2.5, px1 + 4.1, -30, -4.6, 762, 779.8), cbox(px1 + 4.5, -17.3, 770.9, 0.8, 2.0, 2.0)])
    out.append(Inst(P("BARO-BMP390", "Bosch BMP390 barometer breakout (Adafruit)", baro, "pcb", "buy",
                      "Avionics bay", "avionics", mass_g=1.5, color=(0.05, 0.05, 0.08))))
    out.append(Inst(P("BARO-FOAM", "Open-cell foam over the barometer", cbox(px1 + 8.9, -17.3, 770.9, 8.0, 14.0, 14.0),
                      "foam", "buy", "Avionics bay", "avionics", mass_g=0.1)))
    buck = fuse([box(px1 + 2.5, px1 + 4.1, -30, -12.2, 740, 757.8), cbox(px1 + 6.1, -21.1, 748.9, 4.0, 7.0, 7.0)])
    out.append(Inst(P("BUCK-5V", "Pololu D24V30F5 5 V 3 A buck", buck, "pcb", "buy", "Avionics bay", "avionics",
                      mass_g=3.0, color=(0.1, 0.25, 0.6))))
    fet = P("FET-AO3400", "AO3400A pyro MOSFET (SOT-23)", cbox(px1 + 0.55, 0, 0, 1.1, 3.0, 3.0), "electronics",
            "buy", "Avionics bay", "avionics", mass_g=0.05)
    for i, zz in enumerate((735.5, 739.5)):
        out.append(Inst(fet, cq.Location(V(0, -6, zz)), n=i + 1))
    out.append(Inst(P("CAP-1000U", "1000 uF servo-rail capacitor", rod(5.0, (px1, -20, 700), (px1 + 16, -20, 700)),
                      "electronics", "buy", "Avionics bay", "avionics", mass_g=3.0, color=(0.1, 0.1, 0.3))))
    out.append(Inst(P("BUZZER", "11 mm piezo buzzer", rod(5.5, (px1, 16, 736.0), (px1 + 9.5, 16, 736.0)),
                      "electronics", "buy", "Avionics bay", "avionics", mass_g=2.0)))
    jst = P("JST-GH", "JST-GH locking header", cbox(px1 + 2.5, 0, 0, 5.0, 7.5, 4.5), "electronics", "buy",
            "Avionics bay", "avionics", mass_g=0.3, color=(0.95, 0.95, 0.92))
    for i, y in enumerate((-30, -21.5, -13, -4.5, 4, 12.5)):
        out.append(Inst(jst, cq.Location(V(0, y, 687.75)), n=i + 1))

    # leg-release servo and pin through the band
    lb, lh, ltip = servo(**MICRO, horn_deg=90)
    lf = legpin_servo_frame()
    out.append(Inst(P("SERVO-LEGPIN", "Leg-release micro servo (4.3 g)", lb, "servo", "buy", "Avionics bay", "legs",
                      mass_g=4.3), lf))
    out.append(Inst(P("HORN-LEGPIN", "Servo horn", lh, "cf_nylon", "buy", "Avionics bay", "legs", mass_g=0.3), lf))
    pin_in, pin_out = polar(44.5, D.LEGPIN_AZ, ZB), polar(58.5, D.LEGPIN_AZ, ZB)
    out.append(Inst(P("LEG-RELEASE-PIN", "Leg-release pin, 1.5 mm steel", rod(0.75, pin_in, pin_out), "steel", "buy",
                      "Avionics bay", "legs")))
    out.append(Inst(P("LEGPIN-LINK", "Servo-to-pin link", ball_link(tuple(_world(ltip, lf)), tuple(pin_in), 0.6, 1.2),
                      "steel", "buy", "Avionics bay", "legs", mass_g=0.3)))
    a = math.radians(D.LEGPIN_AZ)
    brk = cbox(0, 0, 0, 6.5, 16.0, 16.0)
    brk = brk.rotate(V(), V(0, 0, 1), D.LEGPIN_AZ).translate(polar(44.5, D.LEGPIN_AZ, ZB - 16))
    brk = brk.intersect(cyl(D.R_CPL_IN, 600, 900))
    out.append(Inst(P("LEGPIN-BRKT", "Leg-release servo mount", brk, "cf_nylon", "print", "Avionics bay", "legs",
                      stl=True)))

    # ---------------- avionics, -X face
    out.append(Inst(P("LIPO-2S", "2S 450 mAh LiPo", box(-SLED_X - 14.0, -SLED_X, -15.5, 15.5, 757, 812), "lipo",
                      "buy", "Avionics bay", "avionics", mass_g=30.0, color=(0.85, 0.75, 0.2))))
    so2 = P("STANDOFF-3", "Nylon standoff 3 mm", rod(2.0, (-SLED_X, 0, 0), (-SLED_X - 3.0, 0, 0)), "cf_nylon",
            "buy", "Avionics bay", "avionics", mass_g=0.1)
    k = 5
    for (y, zz) in ((8, 680), (27, 680), (8, 715), (27, 715), (-27, 694), (-8, 694), (-27, 715), (-8, 715)):
        out.append(Inst(so2, cq.Location(V(0, y, zz)), n=k))
        k += 1
    bx = -SLED_X - 3.0
    gps = fuse([box(bx - 1.6, bx, 5.0, 30.4, 676.0, 719.2), cbox(bx - 1.6 - 1.2, 17.7, 697.6, 2.4, 12.2, 16.0)])
    out.append(Inst(P("GPS-M9N", "u-blox NEO-M9N GPS breakout (SparkFun)", gps, "pcb", "buy", "Avionics bay",
                      "avionics", mass_g=6.0, color=(0.75, 0.1, 0.1))))
    patch = box(-33, -8, -12.5, 12.5, ZF - D.PLY - 4.0, ZF - D.PLY)
    out.append(Inst(P("GPS-PATCH", "25 mm ceramic GPS patch antenna, under the forward lid", patch, "ceramic", "buy",
                      "Avionics bay", "avionics", mass_g=9.0, color=(0.75, 0.70, 0.55))))
    lora = fuse([box(bx - 1.6, bx, -30.4, -5.0, 690.0, 719.2), cbox(bx - 2.6, -17.7, 704.6, 2.0, 16.0, 16.0)])
    out.append(Inst(P("LORA-RFM95W", "HopeRF RFM95W 915 MHz LoRa breakout", lora, "pcb", "buy", "Avionics bay",
                      "avionics", mass_g=3.0, color=(0.05, 0.05, 0.08))))
    out.append(Inst(P("LORA-ANT", "915 MHz quarter-wave wire antenna (82 mm)", rod(0.5, (bx - 1.0, -29.0, 719.2),
                      (bx - 1.0, -29.0, 801.2)), "brass", "buy", "Avionics bay", "avionics", mass_g=0.5)))

    # arm key and main switch through the band
    kx = lambda r: -r                                                     # az 180
    keysw = fuse([rod(7.0, (kx(46.0), 0, ZB), (kx(24.0), 0, ZB)),
                  rod(5.95, (kx(D.R_OUT + 1.0), 0, ZB), (kx(45.9), 0, ZB)),
                  hexnut(14.0, 2.0, 6.0, 0.0).rotate(V(), V(0, 1, 0), -90).translate(V(kx(D.R_OUT), 0, ZB))])
    out.append(Inst(P("ARM-KEY-SW", "Arm key switch (pyro rail), panel-mount through the band", keysw, "electronics",
                      "buy", "Avionics bay", "avionics", mass_g=12.0, color=(0.6, 0.6, 0.62))))
    key = fuse([rod(4.5, (kx(D.R_OUT + 2.0), 0, ZB), (kx(D.R_OUT + 10.0), 0, ZB)),
                box(kx(D.R_OUT + 19.0), kx(D.R_OUT + 10.0), -8.0, 8.0, ZB - 1.5, ZB + 1.5)])
    out.append(Inst(P("ARM-KEY", "Arm key (remove before flight on the bench)", key, "steel", "buy", "Avionics bay",
                      "avionics", mass_g=4.0, color=(0.85, 0.15, 0.1))))
    sw_a = D.SWITCH_AZ
    sw = cbox(0, 0, 0, 10.0, 8.0, 12.0).translate(V(41.0, 0, 0))
    sw = fuse([sw, rod(2.95, (46.0, 0, 0), (D.R_OUT + 1.0, 0, 0)), rod(1.0, (D.R_OUT + 1.0, 0, 0), (D.R_OUT + 8.0, 0, 3.0))])
    out.append(Inst(P("MAIN-SW", "Main power toggle switch", sw.rotate(V(), V(0, 0, 1), sw_a).translate(V(0, 0, ZB)),
                      "electronics", "buy", "Avionics bay", "avionics", mass_g=4.0)))

    # ---------------- wiring runs
    g = polar(39.0, 250)
    grom = fuse([tube(4.5, 3.6, ZA - D.PLY, ZA + D.PLY), tube(6.0, 3.6, ZA + D.PLY, ZA + D.PLY + 1.5)])
    out.append(Inst(P("GROMMET", "Harness grommet, aft lid", grom.translate(V(g.x, g.y, 0)), "tpu", "print",
                      "Avionics bay", "wiring", stl=True)))
    harness = fuse([rod(3.5, (g.x, g.y, ZA + 8), (g.x, g.y, 123.0)), cbox(g.x, g.y, 118.0, 9.0, 12.0, 10.0)])
    out.append(Inst(P("HARNESS-AFT", "Servo / pyro harness to the motor bay (bundle)", harness, "electronics", "buy",
                      "Aft airframe", "wiring", mass_g=18.0, color=(0.15, 0.15, 0.15))))
    lc = polar(42.5, 315)
    out.append(Inst(P("CABLE-LIDAR", "Lidar cable, along the 315 deg leg line", rod(1.5, (lc.x, lc.y, ZA + 6),
                      (lc.x, lc.y, D.LIDAR_Z[1])), "electronics", "buy", "Aft airframe", "wiring", mass_g=5.0,
                      color=(0.15, 0.15, 0.15))))
    fc = polar(42.0, 190)
    out.append(Inst(P("CABLE-FWD", "Airbrake / chute servo cable", rod(2.0, (fc.x, fc.y, ZF - D.PLY - 2),
                      (fc.x, fc.y, D.z(D.RECOV_BH_S[0]) + 10)), "electronics", "buy", "Backup chute bay", "wiring",
                      mass_g=4.0, color=(0.15, 0.15, 0.15))))
    return out


def _world(pt, loc):
    """Transform a point by a Location."""
    return cq.Vertex.makeVertex(pt.x, pt.y, pt.z).moved(loc).Center()
