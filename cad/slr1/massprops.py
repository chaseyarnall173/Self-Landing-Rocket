"""Mass properties from the CAD: per-instance mass, CG and pitch inertia,
rolled up by mass-budget group (feeds sim/rocketsim/designs.py)."""
from collections import defaultdict

from OCP.BRepGProp import BRepGProp
from OCP.GProp import GProp_GProps

from . import dims as D
from .catalog import BUDGET


def inst_props(inst):
    """(mass g, centre (x, y, z) mm, own inertia about its CG [Ixx, Iyy, Izz] g mm^2)."""
    shape = inst.world()
    props = GProp_GProps()
    BRepGProp.VolumeProperties_s(shape.wrapped, props)
    vol = props.Mass()
    m = inst.part.mass()
    c = props.CentreOfMass()
    mi = props.MatrixOfInertia()
    k = m / vol if vol > 0 else 0.0
    own = (mi.Value(1, 1) * k, mi.Value(2, 2) * k, mi.Value(3, 3) * k)
    return m, (c.X(), c.Y(), c.Z()), own


def rollup(insts):
    """Totals and per-group (mass, CG station, own pitch inertia about the group CG)."""
    rows = [(i, *inst_props(i)) for i in insts]
    M = sum(r[1] for r in rows)
    zc = sum(r[1] * r[2][2] for r in rows) / M
    xc = sum(r[1] * r[2][0] for r in rows) / M
    # pitch inertia about the Y axis through the CG
    Iyy = sum(r[3][1] + r[1] * ((r[2][0] - xc) ** 2 + (r[2][2] - zc) ** 2) for r in rows)
    groups = defaultdict(list)
    for r in rows:
        groups[r[0].part.budget].append(r)
    out = {}
    for g, rs in groups.items():
        m = sum(r[1] for r in rs)
        gz = sum(r[1] * r[2][2] for r in rs) / m
        gx = sum(r[1] * r[2][0] for r in rs) / m
        gi = sum(r[3][1] + r[1] * ((r[2][0] - gx) ** 2 + (r[2][2] - gz) ** 2) for r in rs)
        out[g] = dict(label=BUDGET[g], mass_g=m, station_mm=D.L - gz, x_mm=gx, own_Iyy_gmm2=gi, n=len(rs))
    return dict(mass_g=M, cg_station_mm=D.L - zc, cg_x_mm=xc, Iyy_kgm2=Iyy * 1e-9, groups=out, rows=rows)
