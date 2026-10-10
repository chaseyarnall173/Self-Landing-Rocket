"""Motor bay: motors, sleeves, 2-axis gimbal, thrust plate, servos, exhaust-paddle
throttle and lidar. Modelled in place at zero gimbal deflection."""
import math

import cadquery as cq

from . import dims as D
from .bays import ball_link, _world
from .catalog import Inst, Part
from .geom import (V, box, cbox, cyl, frame, fuse, polar, rod, servo, stadium, tube)

X08H = dict(L=23.5, W=8.0, H=16.8, tab_L=32.0, tab_t=1.5, horn=10.0)
ZP = D.GIMBAL_PIVOT_Z
ZR0, ZR1 = ZP - D.GIMBAL_RING_H / 2, ZP + D.GIMBAL_RING_H / 2       # 58..70
XA, XL = D.MOTOR_DX, -D.MOTOR_DX                                    # ascent +X, landing -X
ZN = D.NOZZLE_EXIT_Z                                                # 39
ZPL = D.THRUST_PLATE_Z                                              # 101


def motor(length, color_key):
    """29 mm single-use motor: casing, aft closure with nozzle, forward closure."""
    r = D.MOTOR_D / 2
    body = cyl(r, ZN, ZN + length)
    nozzle = cq.Solid.makeCone(6.0, 2.6, 9.0, V(0, 0, ZN - 0.01))
    body = body.cut(nozzle)
    groove = tube(r + 0.1, r - 0.4, ZN + 6.0, ZN + 7.0)
    return body.cut(groove)


def sleeve():
    s = tube(D.SLEEVE_OD / 2, D.SLEEVE_ID / 2, D.SLEEVE_Z[0], D.SLEEVE_Z[1])
    return s.cut(rod(0.95, (0, D.SLEEVE_ID / 2 - 1, ZN + 11.0), (0, D.SLEEVE_OD / 2 + 1, ZN + 11.0)))  # clip screw


def sleeve_cap():
    z1 = D.SLEEVE_Z[1]
    cap = fuse([cyl(D.SLEEVE_ID / 2 - 0.05, z1 - 2.0, z1), cyl(D.SLEEVE_OD / 2, z1, z1 + 1.0)])
    return cap.cut(cyl(3.0, z1 - 3, z1 + 2))


def clip_retainer(x_motor):
    """Spring-steel clip on the +Y side of a sleeve; the hook catches the motor's aft face."""
    yo = D.SLEEVE_OD / 2
    strip = box(x_motor - 2.5, x_motor + 2.5, yo, yo + 0.8, ZN - 0.8, ZN + 16.0)
    hook = box(x_motor - 2.5, x_motor + 2.5, 12.0, yo + 0.8, ZN - 0.8, ZN)
    screw = rod(0.9, (x_motor, yo - 0.9, ZN + 11.0), (x_motor, yo + 2.0, ZN + 11.0))
    return fuse([strip, hook, screw])


def inner_ring():
    r = D.INNER_RING_R
    ring = stadium(D.MOTOR_DX, r, ZR0, ZR1)
    for x in (XA, XL):
        ring = ring.cut(cyl(D.SLEEVE_OD / 2, ZR0 - 1, ZR1 + 1, x, 0))
    lug = box(16.5, 20.5, -19.2, -16.0, ZR1, ZR1 + 8.0)              # servo B pushrod
    ring = fuse([ring, lug])
    ring = ring.cut(rod(1.55, (-40, 0, ZP), (40, 0, ZP)))             # X-axis pins
    flange = box(-35.5, -32.5, -10.0, 10.0, ZR0 - 1.5, ZR0)           # paddle bracket seat
    return fuse([ring, flange.cut(cyl(D.SLEEVE_OD / 2 + 0.3, ZR0 - 2, ZR0, XL, 0))])


def middle_ring():
    c = D.MID_RING_C
    ring = stadium(c, D.MID_RING_RO, ZR0, ZR1).cut(stadium(c, D.MID_RING_RI, ZR0 - 1, ZR1 + 1))
    lug = box(16.5, 20.5, 22.4, 27.0, ZR1, ZR1 + 6.0)                 # servo A pushrod
    ring = fuse([ring, lug])
    ring = ring.cut(rod(1.55, (-50, 0, ZP), (50, 0, ZP)))
    return ring.cut(rod(1.55, (0, -50, ZP), (0, 50, ZP)))


def post(sign):
    y0, y1 = D.POST_Y
    p = box(-D.POST_HALF_X, D.POST_HALF_X, y0, y1, ZP - 8.0, ZPL) if sign > 0 else \
        box(-D.POST_HALF_X, D.POST_HALF_X, -y1, -y0, ZP - 8.0, ZPL)
    p = p.intersect(cyl(D.R_AFT_IN - 0.3, 0, 200))
    return p.cut(rod(1.55, (0, -50, ZP), (0, 50, ZP)))


def thrust_plate(t, z0):
    ring = tube(D.R_AFT_IN, D.THRUST_PLATE_HOLE_R, z0, z0 + t)
    lc = polar(42.5, 315)
    return ring.cut(cyl(2.0, z0 - 1, z0 + t + 1, lc.x, lc.y))


def servo_bracket(sign):
    ys = (34.5, 46.5) if sign > 0 else (-46.5, -34.5)
    zb = ZPL + D.PLY + D.AL_PLATE_T
    base = box(-2.0, 18.8, ys[0], ys[1], zb, zb + 2.8)
    wall = box(-2.0, 0.0, ys[0], ys[1], zb + 2.8, zb + 12.0)
    return fuse([base, wall]).intersect(cyl(D.R_AFT_IN - 0.3, 0, 300))


def gimbal_servo_frame(sign):
    zb = ZPL + D.PLY + D.AL_PLATE_T + 2.8 + (X08H["tab_L"] - X08H["L"]) / 2   # lower tab sits on the bracket
    return frame((X08H["H"] / 2, sign * 40.0, zb + X08H["L"] / 2), (0, 0, 1), (1, 0, 0))


# ---------------------------------------------------------------- exhaust paddles
PY, PZ = D.PADDLE_PIVOT_Y, D.PADDLE_PIVOT_Z         # 16, 31
PX0, PX1 = -28.0, -8.0                              # paddle x extent (20 mm)


def paddle(sign):
    y0, y1 = (PY, PY + D.PADDLE_T) if sign > 0 else (-PY - D.PADDLE_T, -PY)
    return box(PX0, PX1, y0, y1, PZ - 2.0 - D.PADDLE_L, PZ - 2.0)


def paddle_arm(sign, outboard):
    y0, y1 = (PY - 1.0, PY + 4.0) if sign > 0 else (-PY - 4.0, -PY + 1.0)
    x0, x1 = (PX0 - 1.5, PX0) if outboard else (PX1, PX1 + 1.5)
    ztop = PZ + 3.0 if outboard else PZ + 2.0
    arm = box(x0, x1, y0, y1, PZ - 9.0, ztop)
    return arm.cut(rod(1.05, (x0 - 1, sign * PY, PZ), (x1 + 1, sign * PY, PZ)))


def paddle_bracket():
    out_pl = box(-34.0, -32.5, -22.0, 22.0, 24.0, ZR0 - 1.5)
    out_pl = out_pl.cut(box(-35, -32, -1.0, 13.0, 34.5, 43.5))       # servo link slot
    in_pl = box(PX1 + 3.0, PX1 + 4.5, -22.0, 22.0, 24.0, PZ + 6.0)
    rails = fuse([box(-32.5, PX1 + 3.0, 20.5, 22.0, 24.0, 26.0),
                  box(-32.5, PX1 + 3.0, -22.0, -20.5, 24.0, 26.0)])
    b = fuse([out_pl, in_pl, rails])
    for s in (1, -1):
        b = b.cut(rod(1.05, (-40, s * PY, PZ), (0, s * PY, PZ)))
    return b


def paddle_servo_frame():
    return frame((-38.0, 0.0, 44.75), (0, 0, 1), (0, 1, 0))


# ---------------------------------------------------------------- lidar
def lidar_body():
    r0 = D.LIDAR_R
    b = cbox(0, 0, sum(D.LIDAR_Z) / 2, 15.0, 42.0, D.LIDAR_Z[1] - D.LIDAR_Z[0])
    lens = fuse([cyl(5.0, D.LIDAR_Z[0] - 1.0, D.LIDAR_Z[0] + 0.1, 0, y) for y in (-10.0, 10.0)])
    b = fuse([b, lens])
    return b.rotate(V(), V(0, 0, 1), D.LIDAR_AZ).translate(polar(r0, D.LIDAR_AZ))


def lidar_window():
    w = cbox(0, 0, D.LIDAR_Z[0] - 1.5, 15.0, 42.0, 1.0)
    return w.rotate(V(), V(0, 0, 1), D.LIDAR_AZ).translate(polar(D.LIDAR_R, D.LIDAR_AZ))


def lidar_bracket():
    r_back = D.LIDAR_R + 7.5
    blk = box(r_back, D.R_AFT_IN + 1, -12.0, 12.0, D.LIDAR_Z[0] - 3.0, D.LIDAR_Z[1])
    lip = box(D.LIDAR_R - 7.5, r_back, -21.5, 21.5, D.LIDAR_Z[0] - 3.0, D.LIDAR_Z[0] - 2.0)
    lip = lip.cut(box(D.LIDAR_R - 6.0, r_back - 1.5, -19.0, 19.0, D.LIDAR_Z[0] - 4, D.LIDAR_Z[0]))
    b = fuse([blk, lip]).rotate(V(), V(0, 0, 1), D.LIDAR_AZ)
    return b.intersect(cyl(D.R_AFT_IN, 0, 100))


# ---------------------------------------------------------------- parts list
def parts():
    out = []
    P = Part
    out.append(Inst(P("MOTOR-F52C", "AeroTech F52C ascent motor (29 x 111 mm, 81 g)", motor(D.ASCENT_LEN, "a"),
                      "motor_f42t", "buy", "Motors", "motor_a", mass_g=81.4,
                      note="Plugged / no-ejection configuration only."), cq.Location(V(XA, 0, 0))))
    out.append(Inst(P("MOTOR-F25W", "AeroTech F25W landing motor (29 x 98 mm, 97 g)", motor(D.LANDING_LEN, "l"),
                      "motor_f25w", "buy", "Motors", "motor_l", mass_g=97.2,
                      note="Two igniters, fired by the flight computer."), cq.Location(V(XL, 0, 0))))
    sp = cyl(D.MOTOR_D / 2 - 0.1, ZN + D.LANDING_LEN, ZN + D.ASCENT_LEN).cut(cyl(3.0, 0, 300))
    out.append(Inst(P("F25W-SPACER", "F25W 13 mm spacer (both nozzles exit together)", sp, "petg", "print",
                      "Motor bay", "gimbal", fill=0.4, stl=True, color=(0.85, 0.85, 0.82)), cq.Location(V(XL, 0, 0))))
    sl = P("SLEEVE-29", "29 mm motor sleeve, kraft motor-mount tube, 113 mm (high-temp epoxy)", sleeve(),
           "kraft_cpl", "tube", "Motor bay", "gimbal")
    cap = P("SLEEVE-CAP", "Sleeve thrust cap, aluminium (igniter-lead hole)", sleeve_cap(), "aluminium",
            "machine", "Motor bay", "gimbal")
    for i, x in enumerate((XA, XL)):
        out.append(Inst(sl, cq.Location(V(x, 0, 0)), n=i + 1))
        out.append(Inst(cap, cq.Location(V(x, 0, 0)), n=i + 1))
    for i, x in enumerate((XA, XL)):
        out.append(Inst(P("CLIP-RET" if i == 0 else "CLIP-RET-L", "Spring-steel clip retainer + M2 screw",
                          clip_retainer(x), "steel", "bend", "Motor bay", "gimbal", mass_g=1.5)))
    out.append(Inst(P("IGN-CONN", "Landing-igniter 2-pin locking connector", cbox(XL, 0, D.SLEEVE_Z[1] + 4.0, 8.0, 6.0, 5.0),
                      "electronics", "buy", "Motor bay", "wiring", mass_g=0.5, color=(0.95, 0.95, 0.92))))

    out.append(Inst(P("GIMBAL-INNER", "Gimbal inner ring (holds both sleeves), CF-nylon", inner_ring(), "cf_nylon",
                      "print", "Motor bay", "gimbal", fill=0.7, stl=True)))
    out.append(Inst(P("GIMBAL-MIDDLE", "Gimbal middle ring, CF-nylon", middle_ring(), "cf_nylon", "print",
                      "Motor bay", "gimbal", fill=0.7, stl=True)))
    for i, s in enumerate((1, -1)):
        out.append(Inst(P("GIMBAL-POST" if s > 0 else "GIMBAL-POST-B", "Gimbal yoke post (outer frame), CF-nylon",
                          post(s), "cf_nylon", "print", "Motor bay", "gimbal", fill=0.7, stl=True)))
    xpin = P("PIN-X", "Gimbal pin, 3 mm steel (X axis)", rod(1.5, (32.3, 0, ZP), (44.5, 0, ZP)), "steel", "buy",
             "Motor bay", "gimbal")
    ypin = P("PIN-Y", "Gimbal pin, 3 mm steel (Y axis)", rod(1.5, (0, 22.3, ZP), (0, 41.0, ZP)), "steel", "buy",
             "Motor bay", "gimbal")
    for i, ang in enumerate((0, 180)):
        out.append(Inst(xpin, cq.Location(V(), V(0, 0, 1), ang), n=i + 1))
        out.append(Inst(ypin, cq.Location(V(), V(0, 0, 1), ang), n=i + 1))
    out.append(Inst(P("THRUST-PLATE", "Thrust plate, 1/8 in ply ring", thrust_plate(D.PLY, ZPL), "ply", "cut",
                      "Motor bay", "couplers", flat=cq.Workplane("XY").circle(D.R_AFT_IN).circle(D.THRUST_PLATE_HOLE_R))))
    out.append(Inst(P("THRUST-PLATE-AL", "Thrust plate, 1 mm aluminium ring", thrust_plate(D.AL_PLATE_T, ZPL + D.PLY),
                      "aluminium", "cut", "Motor bay", "couplers",
                      flat=cq.Workplane("XY").circle(D.R_AFT_IN).circle(D.THRUST_PLATE_HOLE_R))))

    lugA = V(18.5, 24.7, ZR1 + 6.0)
    lugB = V(18.5, -17.6, ZR1 + 8.0)
    for sign, name, lug in ((1, "A (pitch, drives middle ring)", lugA), (-1, "B (yaw, drives inner ring)", lugB)):
        body, horn, tip = servo(**X08H, horn_deg=-90 if sign > 0 else 90)
        fr = gimbal_servo_frame(sign)
        tag = "A" if sign > 0 else "B"
        out.append(Inst(P(f"SERVO-GIMBAL-{tag}", f"KST X08H gimbal servo {name}", body, "servo", "buy", "Motor bay",
                          "gimbal", mass_g=8.0), fr))
        out.append(Inst(P(f"HORN-GIMBAL-{tag}", "Servo horn (10 mm)", horn, "cf_nylon", "buy", "Motor bay", "gimbal",
                          mass_g=0.4), fr))
        out.append(Inst(P(f"PUSHROD-{tag}", "1.5 mm steel pushrod with ball links", ball_link(tuple(_world(tip, fr)),
                          tuple(lug)), "steel", "buy", "Motor bay", "gimbal", mass_g=1.2)))
        out.append(Inst(P(f"SERVO-BRKT-{tag}", "Gimbal servo bracket, CF-nylon", servo_bracket(sign), "cf_nylon",
                          "print", "Motor bay", "gimbal", stl=True)))

    # exhaust-paddle throttle
    for s in (1, -1):
        tag = "R" if s > 0 else "L"
        out.append(Inst(P(f"PADDLE-{tag}", "Throttle paddle, alumina-silicate ceramic 20 x 28 x 4", paddle(s),
                          "ceramic", "machine", "Throttle", "paddles")))
        out.append(Inst(P(f"PADDLE-ARM-O{tag}", "Paddle arm with lever, 1.5 mm 304 stainless", paddle_arm(s, True),
                          "steel", "machine", "Throttle", "paddles")))
        out.append(Inst(P(f"PADDLE-ARM-I{tag}", "Paddle arm, 1.5 mm 304 stainless", paddle_arm(s, False), "steel",
                          "machine", "Throttle", "paddles")))
        out.append(Inst(P(f"PADDLE-PIN-{tag}", "Paddle pivot pin, 2 mm steel", rod(1.0, (-34.0, s * PY, PZ),
                          (PX1 + 4.5, s * PY, PZ)), "steel", "buy", "Throttle", "paddles")))
        out.append(Inst(P(f"PADDLE-LINK-{tag}", "Scissor link", ball_link((-30.5, s * 2.5, 36.5),
                          (-28.75, s * PY, PZ + 3.0 - 0.5), 0.6, 1.0), "steel", "buy", "Throttle", "paddles", mass_g=0.3)))
    out.append(Inst(P("PADDLE-BRKT", "Paddle bracket on the inner gimbal ring, 1.5 mm 6061 aluminium", paddle_bracket(),
                      "aluminium", "machine", "Throttle", "paddles",
                      note="Wrap the plume-facing rails in stainless tape / ceramic paper.")))
    out.append(Inst(P("PADDLE-XHEAD", "Scissor crosshead", box(-31.5, -29.5, -3.0, 10.5, 35.5, 37.5), "steel",
                      "machine", "Throttle", "paddles")))
    body, horn, tip = servo(**dict(X08H, tab_L=X08H["L"]), horn_deg=0)    # strapped, no ears
    fr = paddle_servo_frame()
    out.append(Inst(P("SERVO-PADDLE", "KST X08H paddle (throttle) servo, strapped to the bracket", body, "servo", "buy", "Throttle", "paddles",
                      mass_g=8.0), fr))
    out.append(Inst(P("HORN-PADDLE", "Servo horn (10 mm)", horn, "cf_nylon", "buy", "Throttle", "paddles",
                      mass_g=0.4), fr))
    out.append(Inst(P("PADDLE-SERVO-LINK", "Servo-to-crosshead link", ball_link(tuple(_world(tip, fr)), (-30.5, 9.0, 38.5),
                      0.6, 1.0), "steel", "buy", "Throttle", "paddles", mass_g=0.3)))
    out.append(Inst(P("HEAT-SHIELD", "Servo heat shield, aluminium + ceramic paper", box(-42.5, -34.0, -9.0, 9.0, 31.5, 32.8),
                      "aluminium", "cut", "Throttle", "paddles")))

    out.append(Inst(P("LIDAR-TFMINI", "Benewake TFmini-S lidar, looking down", lidar_body(), "electronics", "buy",
                      "Motor bay", "lidar", mass_g=5.0, color=(0.12, 0.12, 0.14))))
    out.append(Inst(P("LIDAR-WINDOW", "Replaceable 1 mm polycarbonate lidar window", lidar_window(), "polycarb", "cut",
                      "Motor bay", "lidar", color=(0.75, 0.85, 0.95))))
    out.append(Inst(P("LIDAR-BRKT", "Lidar bracket, CF-nylon", lidar_bracket(), "cf_nylon", "print", "Motor bay",
                      "lidar", stl=True)))
    return out
