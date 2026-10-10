"""Small geometry helpers on top of CadQuery's OCC wrappers (all mm, degrees)."""
import math

import cadquery as cq
from cadquery import Location, Solid, Vector

ZAX = Vector(0, 0, 1)


def V(x=0.0, y=0.0, zz=0.0):
    return Vector(x, y, zz)


def cyl(r, z0, z1, x=0.0, y=0.0):
    return Solid.makeCylinder(r, z1 - z0, V(x, y, z0))


def tube(ro, ri, z0, z1, x=0.0, y=0.0):
    return cyl(ro, z0, z1, x, y).cut(cyl(ri, z0 - 1, z1 + 1, x, y))


def arc_tube(ro, ri, z0, z1, az_c, half_deg):
    """Cylindrical shell segment centred on azimuth az_c (deg)."""
    out = Solid.makeCylinder(ro, z1 - z0, V(0, 0, z0), ZAX, 2 * half_deg)
    inn = Solid.makeCylinder(ri, z1 - z0 + 2, V(0, 0, z0 - 1), ZAX, 2 * half_deg + 2)
    inn = inn.rotate(V(), ZAX, -1)
    return out.cut(inn).rotate(V(), ZAX, az_c - half_deg)


def box(x0, x1, y0, y1, z0, z1):
    return Solid.makeBox(x1 - x0, y1 - y0, z1 - z0, V(x0, y0, z0))


def cbox(cx, cy, cz, dx, dy, dz):
    """Box from centre and size."""
    return box(cx - dx / 2, cx + dx / 2, cy - dy / 2, cy + dy / 2, cz - dz / 2, cz + dz / 2)


def rod(r, p0, p1):
    p0, p1 = Vector(*p0), Vector(*p1)
    d = p1 - p0
    return Solid.makeCylinder(r, d.Length, p0, d.normalized())


def torus(R, r, zc, x=0.0, y=0.0):
    return Solid.makeTorus(R, r, V(x, y, zc), ZAX)


def rot_z(shape, deg):
    return shape.rotate(V(), ZAX, deg)


def polar(r, az_deg, zz=0.0):
    a = math.radians(az_deg)
    return V(r * math.cos(a), r * math.sin(a), zz)


def at_az(shape, az_deg):
    """Shapes are modelled on the +X azimuth; this turns them to az_deg."""
    return rot_z(shape, az_deg)


def loc_axis(point, axis, deg):
    """Location that rotates by deg about the line (point, axis)."""
    p = Vector(*point)
    return Location(p) * Location(V(), Vector(*axis), deg) * Location(-p)


def loc_z(deg, dz=0.0):
    return Location(V(0, 0, dz), ZAX, deg)


def fuse(shapes):
    shapes = [s for s in shapes if s is not None]
    out = shapes[0]
    if len(shapes) > 1:
        out = out.fuse(*shapes[1:]).clean()
    return out


def compound(shapes):
    return cq.Compound.makeCompound([s for s in shapes if s is not None])


def stadium(c, r, z0, z1):
    """Solid stadium: two circles radius r at x = +-c joined by tangents."""
    return fuse([cyl(r, z0, z1, -c), cyl(r, z0, z1, c), box(-c, c, -r, r, z0, z1)])


def spring(r_mean, wire_d, pitch, height, base=(0, 0, 0), axis=(0, 0, 1)):
    """Helical coil spring, built on +Z then aimed along axis."""
    helix = cq.Wire.makeHelix(pitch=pitch, height=height, radius=r_mean)
    prof = cq.Wire.makeCircle(wire_d / 2, V(r_mean, 0, 0), V(0, 1, 0))
    s = Solid.sweep(prof, [], helix, isFrenet=True)
    ax = Vector(*axis).normalized()
    if (ax - ZAX).Length > 1e-9:
        rot_axis = ZAX.cross(ax)
        if rot_axis.Length < 1e-9:
            s = s.rotate(V(), V(1, 0, 0), 180)
        else:
            ang = math.degrees(math.acos(max(-1.0, min(1.0, ZAX.dot(ax)))))
            s = s.rotate(V(), rot_axis, ang)
    return s.translate(Vector(*base))


def profile_xy(points):
    """Closed polygon in the XY plane as a Workplane (for DXF and extrusion)."""
    return cq.Workplane("XY").polyline(points).close()


def servo(L=23.5, W=8.0, H=16.8, tab_L=32.0, tab_t=1.5, horn=10.0, horn_deg=0.0):
    """Generic micro servo on its own frame: body centred at the origin with L
    along X, W along Y, H along Z; output shaft on +Z, 6 mm from the +X end.
    The horn arm points -X, turned by horn_deg about the shaft.
    Returns (body, horn, horn_tip_point) in the servo frame."""
    body = cbox(0, 0, 0, L, W, H)
    tabs = cbox(0, 0, H / 2 - 4.5, tab_L, W, tab_t)
    shaft_x = L / 2 - 6.0
    boss = cyl(3.0, H / 2, H / 2 + 2.0, shaft_x, 0)
    body = fuse([body, tabs, boss])
    horn_s = fuse([cyl(2.6, H / 2 + 2.0, H / 2 + 4.0, shaft_x, 0),
                   box(shaft_x - horn, shaft_x, -1.8, 1.8, H / 2 + 2.0, H / 2 + 3.4)])
    horn_s = horn_s.rotate(V(shaft_x, 0, 0), V(shaft_x, 0, 1), horn_deg)
    a = math.radians(horn_deg)
    tip = V(shaft_x - (horn - 1.5) * math.cos(a), -(horn - 1.5) * math.sin(a), H / 2 + 2.7)
    return body, horn_s, tip


def frame(origin, xdir, zdir):
    """Location mapping local X -> xdir, local Z -> zdir, placed at origin."""
    return Location(cq.Plane(origin=Vector(*origin), xDir=Vector(*xdir), normal=Vector(*zdir)))


def half_angle(width, r):
    """Half-angle (deg) of an arc of the given chord-ish width at radius r."""
    return math.degrees(width / 2.0 / r)


def bent_rod(r, pts):
    """Round rod swept through a polyline with rounded corners (U-bolts, cords)."""
    pts = [Vector(*p) for p in pts]
    path = cq.Wire.makePolygon(pts)
    if len(pts) > 2:
        path = path.fillet2D(min(6.0, 0.45 * min((pts[i + 1] - pts[i]).Length for i in range(len(pts) - 1))),
                             path.Vertices()[1:-1]) if hasattr(path, "fillet2D") else path
    d = (pts[1] - pts[0]).normalized()
    prof = cq.Wire.makeCircle(r, pts[0], d)
    return Solid.sweep(prof, [], path, transitionMode="round")


def pcb(w, l, t=1.6):
    """Board lying in XY, centred, components side +Z."""
    return cbox(0, 0, t / 2, w, l, t)


def aim(shape, frm, to):
    """Rotate shape about the origin so that unit vector frm points along to."""
    frm, to = Vector(*frm).normalized(), Vector(*to).normalized()
    ax = frm.cross(to)
    if ax.Length < 1e-9:
        if frm.dot(to) > 0:
            return shape
        perp = V(1, 0, 0) if abs(frm.x) < 0.9 else V(0, 1, 0)
        return shape.rotate(V(), frm.cross(perp), 180)
    ang = math.degrees(math.acos(max(-1.0, min(1.0, frm.dot(to)))))
    return shape.rotate(V(), ax, ang)
