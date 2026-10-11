"""Airframe: nose, tube pieces, couplers, fins, top airbrakes, chute door."""
import math

import cadquery as cq

from . import dims as D
from .catalog import Inst, Part
from .geom import (V, arc_tube, at_az, box, cyl, fuse, half_angle, loc_axis,
                   loc_z, polar, rod, spring, tube)

HK = math.degrees(D.KERF / 2 / D.R_OUT)      # half-kerf as an angle


# ---------------------------------------------------------------- nose cone
def _ogive_points(n=60):
    Ln, R = D.NOSE_LEN, D.R_OUT
    rho = (R * R + Ln * Ln) / (2 * R)
    pts = []
    for i in range(n + 1):
        x = Ln * (1 - math.cos(math.pi / 2 * i / n))    # cluster points at the tip
        y = math.sqrt(max(0.0, rho * rho - (Ln - x) ** 2)) + R - rho
        pts.append((x, y))
    return pts


def nose_cone():
    """Hollow PETG tangent ogive (0.8 mm wall) with a 2 in shoulder."""
    pts = _ogive_points()
    zt = D.L
    outer, inner = [], []
    for i, (x, y) in enumerate(pts):
        if y < D.NOSE_TIP_R * 0.35:
            continue
        outer.append((y, zt - x))
    # inward normal offset of the outer curve
    for i in range(1, len(outer) - 1):
        (r0, z0), (r1, z1), (r2, z2) = outer[i - 1], outer[i], outer[i + 1]
        tr, tz = r2 - r0, z2 - z0
        ln = math.hypot(tr, tz)
        nr, nz = -tz / ln, tr / ln         # rotate tangent; points inward (toward the axis)
        if nr > 0:
            nr, nz = -nr, -nz
        ri, zi = r1 + D.NOSE_WALL * nr, z1 + D.NOSE_WALL * nz
        if ri > 1.5:
            inner.append((ri, zi))
    zb = D.z(D.NOSE_LEN)
    inner.append((D.R_OUT - D.NOSE_WALL, zb))
    prof = [(0.0, zt)] + outer + [(D.R_OUT, zb)] + list(reversed(inner)) + \
        [(0.0, inner[0][1])]
    # build the revolve profile in the XZ plane (X = radius, Z = axial)
    wire = cq.Wire.makePolygon([V(r, 0, zz) for r, zz in prof], close=True)
    face = cq.Face.makeFromWires(wire)
    shell = cq.Solid.revolve(face, 360, V(0, 0, 0), V(0, 0, 1))
    r_sh = D.CPL_OD / 2 - 0.08
    z_sh0 = D.z(D.NOSE_LEN + D.NOSE_SHOULDER)
    shoulder = tube(r_sh, r_sh - D.NOSE_SHOULDER_WALL, z_sh0, zb)
    stiff = tube(r_sh, r_sh - 4.0, z_sh0, z_sh0 + 1.2)          # stiffening lip, no end disc
    flange = tube(D.R_OUT - 0.3, r_sh - D.NOSE_SHOULDER_WALL, zb, zb + 4.0)
    nose = fuse([shell, shoulder, stiff, flange])
    zs = D.z(D.NOSE_SCREW_S)
    for az in (0, 120, 240):
        nose = nose.cut(rod(1.25, polar(r_sh - 6, az, zs), polar(r_sh + 1, az, zs)))
        nose = nose.fuse(_screw_boss(az, zs, r_sh)).clean()
    return nose


def _screw_boss(az, zs, r_sh):
    """Printed boss inside the shoulder that takes an M3 screw."""
    boss = box(r_sh - D.NOSE_SHOULDER_WALL - 4.0, r_sh - D.NOSE_SHOULDER_WALL + 0.2,
               -4.0, 4.0, zs - 5.0, zs + 5.0)
    boss = boss.cut(rod(1.25, (r_sh - 8, 0, zs), (r_sh + 1, 0, zs)))
    return at_az(boss, az)


def m3_button(length=8.0):
    """M3 button-head screw pointing -X from a head seated at x = 0 (local)."""
    head = rod(2.85, (0, 0, 0), (1.65, 0, 0))
    shank = rod(1.25, (0, 0, 0), (-length, 0, 0))     # modelled at the thread minor diameter
    return fuse([head, shank])


# ---------------------------------------------------------------- tube pieces
def _tube(s0, s1):
    return tube(D.R_OUT, D.R_IN, D.z(s1), D.z(s0))


def nose_ring():
    t = _tube(*D.NOSE_RING_S)
    zs = D.z(D.NOSE_SCREW_S)
    for az in (0, 120, 240):
        t = t.cut(rod(1.6, polar(D.R_IN - 1, az, zs), polar(D.R_OUT + 1, az, zs)))
    return t


def petal():
    """One airbrake petal (modelled on +X): a 90 deg quadrant of the band."""
    s0, s1 = D.PETAL_S
    return arc_tube(D.R_OUT, D.R_IN, D.z(s1) + D.KERF / 2, D.z(s0) - D.KERF / 2,
                    0.0, 45.0 - HK)


def forward_tube():
    """Recovery section: petal band aft edge to the switch band, with the door."""
    t = _tube(D.PETAL_S[1], D.BAND_S[0])
    cut = arc_tube(D.R_OUT + 1, D.R_IN - 1, D.z(D.DOOR_S[1]), D.z(D.DOOR_S[0]),
                   D.DOOR_AZ, D.DOOR_HALF_DEG)
    return t.cut(cut)


def door_panel():
    """Chute door, modelled on +X (placed at DOOR_AZ)."""
    return arc_tube(D.R_OUT, D.R_IN, D.z(D.DOOR_S[1]) + D.KERF / 2,
                    D.z(D.DOOR_S[0]) - D.KERF / 2, 0.0, D.DOOR_HALF_DEG - HK)


def _band_holes(shape):
    zc = D.z(sum(D.BAND_S) / 2)
    holes = [(D.KEY_AZ, 6.1), (D.SWITCH_AZ, 3.1), (D.LEGPIN_AZ, 1.0)] + \
        [(az, 1.0) for az in D.STATIC_PORT_AZ]
    for az, r in holes:
        shape = shape.cut(rod(r, polar(D.R_CPL_IN - 2, az, zc), polar(D.R_OUT + 2, az, zc)))
    return shape


def switch_band():
    return _band_holes(_tube(*D.BAND_S))


def _fin_and_leg_slots(shape):
    zf0, zf1 = D.FIN_TE_Z, D.FIN_TE_Z + D.FIN_ROOT
    for az in D.FIN_AZ:
        slot = box(D.R_AFT_IN - 1, D.R_OUT + 1, -(D.FIN_T + 0.3) / 2, (D.FIN_T + 0.3) / 2, zf0, zf1)
        shape = shape.cut(at_az(slot, az))
    for az in D.LEG_AZ:
        slot = box(D.R_AFT_IN - 1, D.R_OUT + 1, -5.2, 5.2, D.LEG_HINGE_Z - 10.2, D.LEG_HINGE_Z + 10.2)
        shape = shape.cut(at_az(slot, az))
    return shape


def aft_tube():
    return _fin_and_leg_slots(_tube(*D.AFT_TUBE_S))


# ---------------------------------------------------------------- couplers
def _coupler(s0, s1):
    return tube(D.CPL_OD / 2, D.R_CPL_IN, D.z(s1), D.z(s0))


def airbrake_sleeve():
    t = _coupler(*D.SLEEVE_S)
    zl = D.z(D.LATCH_S)
    for az in D.PETAL_AZ:
        t = t.cut(arc_tube(D.CPL_OD / 2 + 1, D.R_CPL_IN - 1, zl - 8, zl + 8, az,
                           half_angle(16, D.R_CPL_IN)))
        # lightening window under each petal (petals cover it in flight)
        t = t.cut(arc_tube(D.CPL_OD / 2 + 1, D.R_CPL_IN - 1, zl + 14, D.z(D.PETAL_S[0]) - 16, az,
                           half_angle(60, D.R_CPL_IN)))
    return t


def avbay_coupler():
    return _band_holes(_coupler(*D.AVBAY_S))


def aft_liner():
    return _fin_and_leg_slots(tube(D.R_IN, D.R_AFT_IN, 0.0, D.AFT_CPL_LEN))


# ---------------------------------------------------------------- fins
def fin_profile():
    """Fin outline in local XY: x = radius from the axis, y = height above the
    aft end. Includes the through-the-wall tab down to the coupler bore."""
    zte, zle = D.FIN_TE_Z, D.FIN_TE_Z + D.FIN_ROOT
    rt = D.R_OUT + D.FIN_SPAN
    return [(D.R_AFT_IN, zte), (D.R_AFT_IN, zle), (D.R_OUT, zle),
            (rt, zle - D.FIN_SWEEP), (rt, zte)]


def fin():
    """Fin on +X in the XZ plane."""
    wp = cq.Workplane("XY").polyline(fin_profile()).close()
    flat = wp.extrude(D.FIN_T / 2, both=True).val()
    return flat.rotate(V(), V(1, 0, 0), 90), cq.Workplane("XY").polyline(fin_profile()).close()


def fin_fillet(side):
    """Epoxy fillet (6 mm) along one side of a fin root, on +X."""
    t2 = D.FIN_T / 2
    zf0, zf1 = D.FIN_TE_Z + 2, D.FIN_TE_Z + D.FIN_ROOT - 2
    r = 6.0
    sgn = 1 if side > 0 else -1
    y0, y1 = (t2, t2 + r) if sgn > 0 else (-t2 - r, -t2)
    blk = box(D.R_OUT - 1.0, D.R_OUT + r, y0, y1, zf0, zf1)
    yc = t2 + r if sgn > 0 else -t2 - r
    blk = blk.cut(cyl(r, zf0 - 1, zf1 + 1, D.R_OUT + r, yc))
    return blk.cut(cyl(D.R_OUT, zf0 - 1, zf1 + 1))


# ---------------------------------------------------------------- airbrake mechanism
PH = V(D.PETAL_HINGE_R, 0, D.z(D.PETAL_S[0]))     # petal hinge axis point (local +X)
KR = 2.0                                          # knuckle radius


def _knuckle(y0, y1):
    return rod(KR, (PH.x, y0, PH.z), (PH.x, y1, PH.z))


def petal_hinge_fixed():
    """Printed CF-nylon fixed half: leaf on the nose ring, 2 knuckles, 90 deg stop."""
    w = D.PETAL_HINGE_W / 2
    leaf = arc_tube(D.R_OUT + 1.2, D.R_OUT, PH.z + 2.5, PH.z + 14, 0, half_angle(2 * w, D.R_OUT))
    parts = [leaf]
    for y0, y1 in ((-w, -6.0), (6.0, w)):
        parts.append(_knuckle(y0, y1))
        web = box(D.R_OUT - 0.5, PH.x, y0, y1, PH.z, PH.z + 3.0).cut(cyl(D.R_OUT, PH.z - 1, PH.z + 4))
        parts.append(web)
    stop = box(D.R_OUT - 0.5, D.R_OUT + 9, -3.0, 3.0, PH.z + 3.7, PH.z + 11).cut(
        cyl(D.R_OUT, PH.z, PH.z + 12))
    parts.append(stop)
    out = fuse(parts)
    return out.cut(rod(0.85, (PH.x, -w - 1, PH.z), (PH.x, w + 1, PH.z)))


def petal_hinge_moving():
    """Printed CF-nylon moving half bonded to the petal's forward edge."""
    w = D.PETAL_HINGE_W / 2
    leaf = arc_tube(D.R_OUT + 1.2, D.R_OUT, PH.z - 14, PH.z - 2.5, 0, half_angle(2 * w, D.R_OUT))
    web = box(D.R_OUT - 0.5, PH.x, -5.5, 5.5, PH.z - 3.0, PH.z).cut(cyl(D.R_OUT, PH.z - 4, PH.z + 1))
    out = fuse([leaf, _knuckle(-5.5, 5.5), web])
    return out.cut(rod(0.85, (PH.x, -6, PH.z), (PH.x, 6, PH.z)))


def petal_pin():
    w = D.PETAL_HINGE_W / 2
    return rod(0.8, (PH.x, -w - 0.5, PH.z), (PH.x, w + 5.5, PH.z))


def petal_spring():
    w = D.PETAL_HINGE_W / 2
    return spring(1.55, 0.5, 0.75, 4.0, base=(PH.x, w + 0.6, PH.z), axis=(0, 1, 0))


def petal_tab():
    """Latch tab bonded inside the petal near its aft edge; pokes through the
    sleeve slot. The notch takes the latch-ring finger."""
    zl = D.z(D.LATCH_S)
    tab = box(45.0, D.R_IN + 0.5, -3.0, 3.0, zl - 4, zl + 4).intersect(cyl(D.R_IN, zl - 5, zl + 5))
    return tab.cut(box(44.5, 46.6, 0.0, 3.5, zl - 2, zl + 2))


def latch_ring():
    """Printed ring inside the sleeve; one finger per petal tab."""
    zl = D.z(D.LATCH_S)
    ring = tube(47.4, 44.0, zl - 12, zl - 8)
    parts = [ring]
    for az in D.PETAL_AZ:
        post = box(44.0, 46.4, 4.0, 8.0, zl - 8.5, zl + 1.7)
        finger = box(45.2, 46.4, 0.25, 8.0, zl - 1.7, zl + 1.7)
        parts += [at_az(post, az), at_az(finger, az)]
    lug = box(44.0, 47.3, -3.0, 3.0, zl - 8.5, zl - 4.0)
    parts.append(at_az(lug, 45.0))
    return fuse(parts)


LATCH_SERVO = dict(L=20.0, W=8.5, H=17.0, tab_L=26.0, tab_t=1.2, horn=8.5)   # 4.3 g micro servo


def latch_servo_parts():
    """Airbrake latch micro servo inside the sleeve on the 45 deg strip between
    two lightening windows (modelled on +X, placed at 45 deg). Returns body,
    horn, link and mount, all in the +X frame."""
    from .bays import ball_link, _world
    from .geom import frame, servo
    zl = D.z(D.LATCH_S)
    zc = zl + 20.0
    body, horn, tip = servo(**LATCH_SERVO, horn_deg=90)
    fr = frame((45.3 - LATCH_SERVO["H"] / 2, 0.0, zc), (0, 0, 1), (-1, 0, 0))
    lug = V(45.6, 0.0, zl - 6.0)
    link = ball_link(tuple(_world(tip, fr)), tuple(lug), 0.6, 1.2)
    mount = box(45.3, D.R_CPL_IN + 0.5, -7.0, 7.0, zc - 10, zc + 10).intersect(cyl(D.R_CPL_IN, zc - 20, zc + 20))
    return body.moved(fr), horn.moved(fr), link, mount


# ---------------------------------------------------------------- chute door hinge and latch
DH = V(D.R_OUT + 2.0, 0, D.z(D.DOOR_S[1]))       # door hinge axis (aft edge, local +X)


def door_hinge_fixed():
    w = 12.0
    leaf = arc_tube(D.R_OUT + 1.2, D.R_OUT, DH.z - 13, DH.z - 2.5, 0, half_angle(2 * w, D.R_OUT))
    parts = [leaf]
    for y0, y1 in ((-w, -4.5), (4.5, w)):
        parts.append(rod(KR, (DH.x, y0, DH.z), (DH.x, y1, DH.z)))
        parts.append(box(D.R_OUT - 0.5, DH.x, y0, y1, DH.z - 3.0, DH.z).cut(cyl(D.R_OUT, DH.z - 4, DH.z + 1)))
    return fuse(parts).cut(rod(0.85, (DH.x, -w - 1, DH.z), (DH.x, w + 1, DH.z)))


def door_hinge_moving():
    w = 12.0
    leaf = arc_tube(D.R_OUT + 1.2, D.R_OUT, DH.z + 2.5, DH.z + 13, 0, half_angle(2 * w, D.R_OUT))
    web = box(D.R_OUT - 0.5, DH.x, -4.0, 4.0, DH.z, DH.z + 3.0).cut(cyl(D.R_OUT, DH.z - 1, DH.z + 4))
    out = fuse([leaf, rod(KR, (DH.x, -4.0, DH.z), (DH.x, 4.0, DH.z)), web])
    return out.cut(rod(0.85, (DH.x, -5, DH.z), (DH.x, 5, DH.z)))


def door_pin():
    return rod(0.8, (DH.x, -12.5, DH.z), (DH.x, 12.5, DH.z))


def door_lug():
    """Latch lug inside the door's forward edge; the chute-latch pin passes through it."""
    zf = D.z(D.DOOR_S[0])
    lug = box(45.5, D.R_IN + 0.5, -4.0, 4.0, zf - 15, zf - 7).intersect(cyl(D.R_IN, zf - 16, zf - 6))
    return lug.cut(rod(0.9, (47.3, 0, zf - 17), (47.3, 0, zf - 5)))


# ---------------------------------------------------------------- assembly records
def parts():
    out = []
    P = Part
    nose = P("NOSE-01", "Nose cone, PETG tangent ogive 12.75 in + 2 in shoulder", nose_cone(),
             "petg", "print", "Nose", "nose", stl=True,
             note="0.8 mm walls; shoulder 1.6 mm. Held by 3 x M3 so the airbrake latch stays reachable.")
    out.append(Inst(nose))
    scr = P("NOSE-SCREW", "M3 x 8 button-head screw (nose retention)", m3_button(), "steel", "buy",
            "Nose", "wiring", mass_g=0.5)
    zs = D.z(D.NOSE_SCREW_S)
    for i, az in enumerate((0, 120, 240)):
        loc = cq.Location(polar(D.R_OUT, az, zs), V(0, 0, 1), az)
        out.append(Inst(scr, loc, n=i + 1))

    out.append(Inst(P("TUBE-01", "Nose ring, LOC 3.9 in kraft tube (71 mm)", nose_ring(), "kraft",
                      "tube", "Forward airframe", "tube")))
    pet = P("PETAL-01", "Airbrake petal, 90 deg quadrant cut from the tube (149 mm)", petal(),
            "kraft", "tube", "Top airbrakes", "tube", color=(0.91, 0.35, 0.05),
            note="Seal edges with thin CA; 2 oz fibreglass inside.")
    hf = P("PETAL-HINGE-F", "Petal hinge, fixed half with 90 deg stop", petal_hinge_fixed(),
           "cf_nylon", "print", "Top airbrakes", "airbrakes", stl=True)
    hm = P("PETAL-HINGE-M", "Petal hinge, moving half", petal_hinge_moving(), "cf_nylon", "print",
           "Top airbrakes", "airbrakes", stl=True)
    pin = P("PETAL-PIN", "Petal hinge pin, 1.6 mm stainless, 34 mm", petal_pin(), "steel", "buy",
            "Top airbrakes", "airbrakes")
    spr = P("PETAL-SPRING", "Petal torsion spring, ~0.1 N m", petal_spring(), "steel", "buy",
            "Top airbrakes", "airbrakes")
    tab = P("PETAL-TAB", "Petal latch tab", petal_tab(), "cf_nylon", "print", "Top airbrakes",
            "airbrakes", stl=True)
    for i, az in enumerate(D.PETAL_AZ):
        base = loc_z(az)
        opened = base * loc_axis(PH, (0, 1, 0), -D.PETAL_OPEN_DEG)
        out.append(Inst(hf, base, n=i + 1))
        for part in (pet, hm, pin, spr, tab):
            out.append(Inst(part, base, "stowed", n=i + 1))
            out.append(Inst(part, opened, "deployed", n=i + 1))
    out.append(Inst(P("CPL-01", "Airbrake sleeve, LOC 3.9 in coupler (189 mm), latch slots + 4 lightening windows",
                      airbrake_sleeve(), "kraft_cpl", "tube", "Top airbrakes", "couplers")))
    out.append(Inst(P("LATCH-RING", "Airbrake latch ring with 4 fingers", latch_ring(), "cf_nylon",
                      "print", "Top airbrakes", "airbrakes", stl=True)))
    lb, lh, ll, lm = latch_servo_parts()
    l45 = loc_z(45.0)
    out.append(Inst(P("SERVO-LATCH", "Airbrake-latch micro servo (4.3 g)", lb, "servo", "buy", "Top airbrakes",
                      "airbrakes", mass_g=4.3), l45))
    out.append(Inst(P("HORN-LATCH", "Servo horn", lh, "cf_nylon", "buy", "Top airbrakes", "airbrakes",
                      mass_g=0.3), l45))
    out.append(Inst(P("LATCH-LINK", "Servo-to-latch-ring link", ll, "steel", "buy", "Top airbrakes", "airbrakes",
                      mass_g=0.3), l45))
    out.append(Inst(P("LATCH-SERVO-BRKT", "Latch servo mount, bonded to the sleeve", lm, "cf_nylon", "print",
                      "Top airbrakes", "airbrakes", stl=True), l45))

    out.append(Inst(P("TUBE-02", "Recovery section, kraft tube (223 mm) with door cut-out",
                      forward_tube(), "kraft", "tube", "Forward airframe", "tube")))
    door = P("DOOR-01", "Chute door, cut from the tube", door_panel(), "kraft", "tube",
             "Backup chute bay", "tube", color=(0.91, 0.35, 0.05))
    dloc = loc_z(D.DOOR_AZ)
    out.append(Inst(door, dloc))
    out.append(Inst(P("DOOR-HINGE-F", "Door hinge, fixed half", door_hinge_fixed(), "cf_nylon",
                      "print", "Backup chute bay", "chute", stl=True), dloc))
    out.append(Inst(P("DOOR-HINGE-M", "Door hinge, moving half", door_hinge_moving(), "cf_nylon",
                      "print", "Backup chute bay", "chute", stl=True), dloc))
    out.append(Inst(P("DOOR-PIN", "Door hinge pin, 1.6 mm stainless", door_pin(), "steel", "buy",
                      "Backup chute bay", "chute"), dloc))
    out.append(Inst(P("DOOR-LUG", "Door latch lug", door_lug(), "cf_nylon", "print",
                      "Backup chute bay", "chute", stl=True), dloc))

    out.append(Inst(P("TUBE-03", "Switch band, kraft tube (25 mm): key, switch, static ports, leg pin",
                      switch_band(), "kraft", "tube", "Avionics bay", "tube", color=(0.15, 0.15, 0.16))))
    out.append(Inst(P("CPL-02", "Avionics-bay coupler, LOC 3.9 in (160 mm)", avbay_coupler(),
                      "kraft_cpl", "tube", "Avionics bay", "couplers")))
    out.append(Inst(P("TUBE-04", "Aft airframe, kraft tube (731 mm), fin and leg slots", aft_tube(),
                      "kraft", "tube", "Aft airframe", "tube")))
    out.append(Inst(P("LINER-AFT", "Aft carbon liner, 2 wraps 200 gsm + epoxy (6 in), slotted", aft_liner(),
                      "carbon", "layup", "Aft airframe", "couplers", color=(0.10, 0.10, 0.11))))

    fshape, fflat = fin()
    f = P("FIN-01", "Fin, 3/32 in aircraft birch ply, through-the-wall", fshape, "ply", "cut", "Aft airframe",
          "fins", flat=fflat, color=(0.13, 0.13, 0.14), note="Round the leading edge; 6 mm epoxy fillets.")
    fl = P("FIN-FILLET", "Epoxy root fillet, 6 mm", fin_fillet(+1), "epoxy", "layup", "Aft airframe",
           "wiring", color=(0.13, 0.13, 0.14))
    fr = P("FIN-FILLET", "Epoxy root fillet, 6 mm", fin_fillet(-1), "epoxy", "layup", "Aft airframe",
           "wiring", color=(0.13, 0.13, 0.14))
    for i, az in enumerate(D.FIN_AZ):
        out.append(Inst(f, loc_z(az), n=i + 1))
        out.append(Inst(fl, loc_z(az), n=2 * i + 1))
        out.append(Inst(fr, loc_z(az), n=2 * i + 2))
    return out
