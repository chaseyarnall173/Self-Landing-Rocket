"""Landing legs. One leg is modelled on +X (stowed, pointing forward along the
body) and instanced at 45/135/225/315 deg. Deployed = the moving parts turned
125 deg about the hinge pin, which leaves the strut 55 deg from the axis."""
import math

import cadquery as cq

from . import dims as D
from .catalog import Inst, Part
from .geom import (V, arc_tube, bent_rod, box, cyl, fuse, half_angle, loc_axis,
                   loc_z, polar, rod, spring, torus, tube)

HX, HZ = D.LEG_HINGE_R, D.LEG_HINGE_Z            # pin axis (local +X), along +Y
PIN = V(HX, 0, HZ)
A = math.radians(D.LEG_DEPLOY_DEG)
U = V(math.sin(A), 0, -math.cos(A))              # deployed strut direction
W = V(-math.cos(A), 0, -math.sin(A))             # deployed "down and in"
KNUCKLE_R = 4.0

# distances along the strut axis from the pin (mm), all keyed to the foot
D_TOP = D.LEG_LEN - D.FOOT_H * math.cos(A)       # where the axis meets the foot's top
D_BOSS = (D_TOP - 8.0, D_TOP)
D_PLUNGER = (D_TOP - 24.0, D_TOP - 8.0)
D_BARREL = (D_TOP - 54.8, D_TOP - 12.8)
D_STRUT = (5.0, D_TOP - 39.8)


def _ax(d):
    """Point on the stowed strut axis, d mm from the pin."""
    return V(HX, 0, HZ + d)


def clevis():
    """Printed CF-nylon clevis bonded to the tube; tongue through a slot to the doubler."""
    base = box(D.R_OUT - 0.6, D.R_OUT + 2.5, -9.0, 9.0, HZ - 14, HZ + 14).cut(cyl(D.R_OUT, HZ - 20, HZ + 20))
    tongue = box(D.R_AFT_IN, D.R_OUT + 0.1, -5.0, 5.0, HZ - 10, HZ + 10)
    tongue = tongue.intersect(cyl(D.R_OUT + 0.1, HZ - 11, HZ + 11)).cut(cyl(D.R_AFT_IN, HZ - 11, HZ + 11))
    cheeks = []
    for y0, y1 in ((4.5, 7.5), (-7.5, -4.5)):
        cheeks.append(box(D.R_OUT + 2.4, HX, y0, y1, HZ - 8, HZ + 8))
        cheeks.append(rod(5.0, (HX, y0, HZ), (HX, y1, HZ)))
    # 55 deg stop: its top face lies along the deployed strut's lower surface
    pts = []
    for d in (5.0, 12.0):
        q = PIN + U * d + W * 4.0
        pts.append((q.x, q.z))
    stop_poly = [(D.R_OUT + 2.4, HZ - 14), (pts[1][0], HZ - 14), pts[1], pts[0], (D.R_OUT + 2.4, pts[0][1])]
    stop = cq.Workplane("XZ").polyline(stop_poly).close().extrude(4.4, both=True).val()
    c = fuse([base, tongue, stop] + cheeks)
    return c.cut(rod(1.55, (HX, -12, HZ), (HX, 12, HZ)))


def doubler():
    return arc_tube(D.R_AFT_IN, D.R_AFT_IN - D.PLY, HZ - 18, HZ + 22, 0.0, half_angle(26.0, D.R_AFT_IN))


def leg_pin():
    return rod(1.5, (HX, -8.5, HZ), (HX, 12.5, HZ))


def leg_spring():
    return spring(3.1, 0.7, 1.0, 4.0, base=(HX, 7.6, HZ), axis=(0, 1, 0))


def insert():
    """Printed hinge knuckle + plug that bonds into the strut root."""
    knuckle = rod(KNUCKLE_R, (HX, -4.2, HZ), (HX, 4.2, HZ))
    neck = cyl(D.STRUT_OD / 2, HZ, HZ + D_STRUT[0], HX, 0)
    plug = cyl(D.STRUT_ID / 2 - 0.05, HZ + D_STRUT[0], HZ + D_STRUT[0] + 15, HX, 0)
    return fuse([knuckle, neck, plug]).cut(rod(1.55, (HX, -5, HZ), (HX, 5, HZ)))


def strut():
    return tube(D.STRUT_OD / 2, D.STRUT_ID / 2, HZ + D_STRUT[0], HZ + D_STRUT[1], HX, 0)


def damper():
    z0, z1 = HZ + D_BARREL[0], HZ + D_BARREL[1]
    barrel = tube(5.5, D.STRUT_OD / 2, z0, z1, HX, 0)
    cap = cyl(5.5, z1, z1 + 2.0, HX, 0).cut(cyl(2.1, z1 - 1, z1 + 3, HX, 0))
    return fuse([barrel, cap])


def plunger():
    z0, z1 = HZ + D_PLUNGER[0], HZ + D_PLUNGER[1] + 0.0
    rodp = cyl(2.0, z0, z1, HX, 0)
    piston = cyl(D.STRUT_OD / 2 - 0.1, z0, z0 + 2.0, HX, 0)
    return fuse([rodp, piston])


def foot():
    """TPU-95A lattice foot. Built where it sits when deployed (flat on the
    ground), then turned back to the stowed position."""
    F = PIN + U * D.LEG_LEN + W * D.LEG_FOOT_OFFSET              # bottom-centre, deployed
    disc = cq.Workplane().add(cyl(D.FOOT_D / 2, F.z, F.z + D.FOOT_H, F.x, 0)).edges("<Z").chamfer(1.5).val()
    for k in range(6):
        a = math.radians(30 + 60 * k)
        disc = disc.cut(cyl(2.3, F.z - 1, F.z + D.FOOT_H + 1, F.x + 9.5 * math.cos(a), 9.5 * math.sin(a)))
    post = cyl(5.5, F.z + D.FOOT_H - 0.5, F.z + D.FOOT_H + 6.0, F.x, 0)
    b0, b1 = PIN + U * D_BOSS[0], PIN + U * D_BOSS[1]
    boss = rod(4.0, b0, b1 + U * 2.0)
    deployed = fuse([disc, post, boss])
    return deployed.moved(loc_axis(PIN, (0, 1, 0), -D.LEG_ROTATE_DEG))


def boss_in_stowed():
    """The foot boss in the stowed frame: from D_BOSS along the stowed axis."""
    return rod(4.0, _ax(D_BOSS[0]), _ax(D_BOSS[1]))


def release_loop():
    z = HZ + 556.0
    R = HX + 5.5 + 0.6
    loop = torus(R, 0.6, z)
    t0 = polar(R, D.LEGPIN_AZ, z)
    t1 = polar(R - 2.0, D.LEGPIN_AZ, z + 40.0)
    t2 = polar(58.0, D.LEGPIN_AZ, D.z(sum(D.BAND_S) / 2) + 1.4)
    tail = bent_rod(0.6, [tuple(t0), tuple(t1), tuple(t2)])
    return fuse([loop, tail])


def parts():
    out = []
    P = Part
    fixed = [
        P("LEG-CLEVIS", "Leg hinge clevis with 55 deg stop, CF-nylon", clevis(), "cf_nylon", "print", "Landing legs",
          "legs", stl=True),
        P("LEG-DOUBLER", "Leg hinge doubler, 1/8 in ply (on the aft liner)", doubler(), "ply", "cut",
          "Landing legs", "couplers"),
        P("LEG-PIN", "Leg hinge pin, 3 mm steel", leg_pin(), "steel", "buy", "Landing legs", "legs"),
        P("LEG-SPRING", "Leg torsion spring, ~0.15 N m", leg_spring(), "steel", "buy", "Landing legs", "legs",
          mass_g=0.5),
    ]
    moving = [
        P("LEG-INSERT", "Leg hinge knuckle / strut plug, CF-nylon", insert(), "cf_nylon", "print", "Landing legs",
          "legs", stl=True),
        P("LEG-STRUT", f"Leg strut, 8 x 7 mm pultruded carbon tube, {D_STRUT[1] - D_STRUT[0]:.0f} mm", strut(), "carbon", "cut",
          "Landing legs", "legs"),
        P("LEG-DAMPER", "Syringe air damper barrel", damper(), "polycarb", "buy", "Landing legs", "legs",
          color=(0.85, 0.88, 0.92)),
        P("LEG-PLUNGER", "Damper plunger", plunger(), "steel", "buy", "Landing legs", "legs", mass_g=1.2),
        P("LEG-FOOT", "Foot, TPU-95A lattice crush pad (30 mm)", foot(), "tpu", "print", "Landing legs", "legs",
          fill=0.5, stl=True),
    ]
    for i, az in enumerate(D.LEG_AZ):
        base = loc_z(az)
        opened = base * loc_axis(PIN, (0, 1, 0), D.LEG_ROTATE_DEG)
        for p in fixed:
            out.append(Inst(p, base, n=i + 1))
        for p in moving:
            out.append(Inst(p, base, "stowed", n=i + 1))
            out.append(Inst(p, opened, "deployed", n=i + 1))
    out.append(Inst(P("LEG-LOOP", "Kevlar release loop around the 4 legs, to the release pin", release_loop(),
                      "kevlar", "buy", "Landing legs", "legs", mass_g=0.8), state="stowed"))
    return out
