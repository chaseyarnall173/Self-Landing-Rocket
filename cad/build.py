"""Build the SLR-1 CAD outputs.

    python cad/build.py            # everything
    python cad/build.py --quick    # skip the interference check

Writes to cad/out/:
  SLR1_flight.step, SLR1_landing.step   full assemblies (open in SolidWorks / Fusion / Onshape)
  parts/*.step                          one file per unique part
  stl/*.stl                             every 3D-printed part, ready to slice
  dxf/*.dxf                             flat parts for laser / CNC (fins, bulkheads, plates, sled)
  viewer/*.glb + parts.json             meshes and metadata for the 3D viewer
  bom.csv, mass_report.md, mass_items.json, interference.md
"""
import argparse
import base64
import csv
import json
import math
import sys
import time
from collections import Counter, OrderedDict, defaultdict
from pathlib import Path

import cadquery as cq

ROOT = Path(__file__).resolve().parent
sys.path.insert(0, str(ROOT))
from slr1 import dims as D                          # noqa: E402
from slr1 import massprops as MP                    # noqa: E402
from slr1 import model                              # noqa: E402
from slr1.catalog import BUDGET, MATERIALS, PROCESS  # noqa: E402

OUT = ROOT / "out"

# paper budget the simulator used before the CAD existed (rev B), for comparison
PAPER = {"nose": 95, "airbrakes": 35, "chute": 85, "avionics": 105, "tube": 374, "couplers": 80,
         "gimbal": 130, "paddles": 50, "fins": 45, "legs": 85, "lidar": 12, "wiring": 45,
         "motor_a": 76.0, "motor_l": 97.2}


def color(part):
    r, g, b = part.rgb()
    return cq.Color(r, g, b, 1.0)


def unique_parts(insts):
    seen = OrderedDict()
    for i in insts:
        seen.setdefault(i.part.pid, i.part)
    return seen


def assembly(insts, name):
    top = cq.Assembly(name=name)
    groups = OrderedDict((g, cq.Assembly(name=g.replace(" ", "_"))) for g in model.GROUP_ORDER)
    count = Counter()
    for i in insts:
        p = i.part
        count[p.pid] += 1
        groups[p.group].add(p.shape, name=f"{p.pid}-{count[p.pid]}", loc=i.loc, color=color(p))
    for g, a in groups.items():
        if a.children:
            top.add(a, name=a.name)
    return top


def export_steps(flight, landing):
    t = time.time()
    assembly(flight, "SLR1_flight").export(str(OUT / "SLR1_flight.step"))
    assembly(landing, "SLR1_landing").export(str(OUT / "SLR1_landing.step"))
    print(f"  assemblies ({time.time() - t:.0f} s)")
    (OUT / "parts").mkdir(exist_ok=True)
    for pid, p in unique_parts(flight + landing).items():
        cq.exporters.export(p.shape, str(OUT / "parts" / f"{pid}.step"))


def export_stl_dxf(insts):
    (OUT / "stl").mkdir(exist_ok=True)
    (OUT / "dxf").mkdir(exist_ok=True)
    n_stl = n_dxf = 0
    for pid, p in unique_parts(insts).items():
        if p.stl:
            cq.exporters.export(p.shape, str(OUT / "stl" / f"{pid}.stl"), tolerance=0.02, angularTolerance=0.1)
            n_stl += 1
        if p.flat is not None:
            cq.exporters.export(p.flat, str(OUT / "dxf" / f"{pid}.dxf"))
            n_dxf += 1
    print(f"  {n_stl} STL, {n_dxf} DXF")


def export_viewer(flight, landing):
    """GLB per configuration plus per-part metadata for the web viewer."""
    vdir = OUT / "viewer"
    vdir.mkdir(exist_ok=True)
    meta = {}
    for tag, insts in (("flight", flight), ("landing", landing)):
        a = cq.Assembly(name=f"SLR1_{tag}")
        count = Counter()
        for i in insts:
            p = i.part
            count[p.pid] += 1
            node = f"{p.pid}-{count[p.pid]}"
            a.add(p.shape, name=node, loc=i.loc, color=color(p))
            meta.setdefault(p.pid, dict(name=p.name, group=p.group, material=p.material,
                                        process=PROCESS[p.process], mass_g=round(p.mass(), 2),
                                        budget=BUDGET[p.budget], note=p.note, stl=p.stl,
                                        dxf=p.flat is not None, qty=0))
        glb = vdir / f"slr1_{tag}.glb"
        a.export(str(glb), tolerance=0.25, angularTolerance=0.35)
        # web copy: artifact hosting serves only standard web types, so ship the GLB as base64 text
        (vdir / f"slr1_{tag}.glb.b64.txt").write_text(base64.b64encode(glb.read_bytes()).decode())
    for pid, n in Counter(i.part.pid for i in flight).items():
        meta[pid]["qty"] = n
    for pid, n in Counter(i.part.pid for i in landing).items():
        meta[pid]["qty"] = max(meta[pid]["qty"], n)
    json.dump(dict(groups=model.GROUP_ORDER, parts=meta), open(vdir / "parts.json", "w"), indent=1)
    page = (ROOT / "viewer" / "index.html").read_text()
    (vdir / "index.html").write_text('<!doctype html>\n<meta charset="utf-8">\n'
                                     '<meta name="viewport" content="width=device-width, initial-scale=1, viewport-fit=cover">\n'
                                     + page)
    print("  viewer GLBs:", ", ".join(f"{f.name} {f.stat().st_size / 1e6:.1f} MB" for f in vdir.glob("*.glb")))


def bom(flight, landing):
    rows = OrderedDict()
    allp = Counter(i.part.pid for i in flight)
    for pid, n in Counter(i.part.pid for i in landing).items():
        allp[pid] = max(allp[pid], n)
    for pid, p in unique_parts(flight + landing).items():
        rows[pid] = [pid, p.name, p.group, allp[pid], p.material, PROCESS[p.process],
                     round(p.mass(), 2), round(p.mass() * allp[pid], 1),
                     "stl/" + pid + ".stl" if p.stl else "", "dxf/" + pid + ".dxf" if p.flat is not None else "",
                     p.note]
    with open(OUT / "bom.csv", "w", newline="") as f:
        w = csv.writer(f)
        w.writerow(["part", "description", "subassembly", "qty", "material", "make", "mass_each_g",
                    "mass_total_g", "stl", "dxf", "note"])
        order = {g: k for k, g in enumerate(model.GROUP_ORDER)}
        for r in sorted(rows.values(), key=lambda r: (order[r[2]], r[0])):
            w.writerow(r)
    print(f"  bom.csv: {len(rows)} unique parts, {sum(allp.values())} pieces")


def mass_report(flight, landing):
    full = MP.rollup(flight)
    land = MP.rollup(landing)
    dry = MP.rollup([i for i in flight if i.part.budget not in ("motor_a", "motor_l")])
    mot = MP.rollup([i for i in flight if i.part.budget in ("motor_a", "motor_l")])
    items = []
    for g, v in sorted(dry["groups"].items(), key=lambda kv: kv[1]["station_mm"]):
        L = math.sqrt(12 * v["own_Iyy_gmm2"] / v["mass_g"]) / 1000
        items.append(dict(name=v["label"], mass_kg=round(v["mass_g"] / 1000, 4),
                          station_m=round(v["station_mm"] / 1000, 3), length_m=round(L, 3)))
    json.dump(dict(items=items, motor_station_m=round(mot["cg_station_mm"] / 1000, 3),
                   liftoff_mass_g=round(full["mass_g"], 1), liftoff_cg_m=round(full["cg_station_mm"] / 1000, 4),
                   liftoff_Iyy_kgm2=round(full["Iyy_kgm2"], 4)),
              open(OUT / "mass_items.json", "w"), indent=1)
    L = ["# SLR-1 mass report (from the CAD)", "",
         "Generated by `python cad/build.py`. Masses are CAD volume x material density for made parts and",
         "catalogue / estimated masses for bought parts (see `cad/slr1/catalog.py` and each part's",
         "`mass_g`). **Weigh every part as you build and replace these numbers.**", "",
         "| Configuration | Mass | CG station | Pitch inertia |", "|---|---:|---:|---:|",
         f"| Liftoff (flight, legs stowed) | {full['mass_g']:.0f} g | {full['cg_station_mm']:.0f} mm | "
         f"{full['Iyy_kgm2']:.3f} kg m^2 |",
         f"| Same, landing configuration (legs and airbrakes out) | {land['mass_g']:.0f} g | "
         f"{land['cg_station_mm']:.0f} mm | {land['Iyy_kgm2']:.3f} kg m^2 |",
         f"| Dry (no motors) | {dry['mass_g']:.0f} g | {dry['cg_station_mm']:.0f} mm | {dry['Iyy_kgm2']:.3f} kg m^2 |",
         "", f"Model-rocket limit 1500 g: margin **{1500 - full['mass_g']:.0f} g**.", "",
         "## By mass-budget line", "",
         "| Line | CAD (g) | CG station (mm) | Rev B paper budget (g) |", "|---|---:|---:|---:|"]
    for g, v in sorted(full["groups"].items(), key=lambda kv: kv[1]["station_mm"]):
        L.append(f"| {v['label']} | {v['mass_g']:.1f} | {v['station_mm']:.0f} | {PAPER.get(g, 0):.0f} |")
    L.append(f"| **Total** | **{full['mass_g']:.0f}** | **{full['cg_station_mm']:.0f}** | "
             f"**{sum(PAPER.values()):.0f}** |")
    L += ["", "## Heaviest parts", "", "| Part | Qty | Each (g) | Total (g) |", "|---|---:|---:|---:|"]
    per = defaultdict(lambda: [0, 0.0, ""])
    for i in flight:
        e = per[i.part.pid]
        e[0] += 1
        e[1] = i.part.mass()
        e[2] = i.part.name
    for pid, (n, m, name) in sorted(per.items(), key=lambda kv: -kv[1][0] * kv[1][1])[:25]:
        L.append(f"| {pid}: {name} | {n} | {m:.1f} | {n * m:.1f} |")
    L += ["", "## Densities used", "", "| Material | g/cm^3 |", "|---|---:|"]
    used = sorted({i.part.material for i in flight if i.part.mass_g is None})
    for m in used:
        L.append(f"| {m} | {MATERIALS[m][0]} |")
    (OUT / "mass_report.md").write_text("\n".join(L) + "\n")
    print(f"  liftoff {full['mass_g']:.0f} g, CG {full['cg_station_mm']:.0f} mm, Iyy {full['Iyy_kgm2']:.3f} kg m^2")
    return full


def interference(insts, label):
    """Pairwise solid overlaps (bounding-box prefilter, then boolean common)."""
    shapes = [(i, i.world()) for i in insts]
    boxes = [s.BoundingBox() for _, s in shapes]
    hits = []
    n_checked = 0
    for a in range(len(shapes)):
        ba = boxes[a]
        for b in range(a + 1, len(shapes)):
            bb = boxes[b]
            if (ba.xmax < bb.xmin or bb.xmax < ba.xmin or ba.ymax < bb.ymin or bb.ymax < ba.ymin
                    or ba.zmax < bb.zmin or bb.zmax < ba.zmin):
                continue
            n_checked += 1
            try:
                v = shapes[a][1].intersect(shapes[b][1]).Volume()
            except Exception:                       # noqa: BLE001
                v = float("nan")
            if not (v < 0.5):
                ia, ib = shapes[a][0], shapes[b][0]
                hits.append((v, f"{ia.part.pid}-{ia.n}", f"{ib.part.pid}-{ib.n}"))
    hits.sort(reverse=True)
    joints = [h for h in hits if _is_joint(h[1]) or _is_joint(h[2])]
    hits = [h for h in hits if h not in joints]
    print(f"  interference [{label}]: {n_checked} close pairs checked, {len(hits)} clashes, "
          f"{len(joints)} designed joints")
    return hits, joints, n_checked


def _is_joint(name):
    """Ball links, pushrods and cords are modelled seated in the part they attach to."""
    return any(k in name for k in ("LINK", "PUSHROD", "LOOP"))


INNER = ("MOTOR-", "F25W-SPACER", "SLEEVE-29", "SLEEVE-CAP", "CLIP-RET", "IGN-CONN", "GIMBAL-INNER")
MIDDLE = ("GIMBAL-MIDDLE", "PIN-X")


def gimbal_sweep(insts, deg=D.GIMBAL_MAX_DEG):
    """Tilt the gimbal to +-deg on each axis (and both at once) and check the moving
    parts against everything fixed. Pushrods are excluded (their ball links swivel)."""
    piv = cq.Vector(0, 0, D.GIMBAL_PIVOT_Z)

    def role(i):
        pid = i.part.pid
        if i.part.group == "Throttle" or pid.startswith(INNER):
            return "inner"
        if pid.startswith(MIDDLE):
            return "middle"
        return "fixed"

    fixed = [(i, i.world()) for i in insts if role(i) == "fixed" and "PUSHROD" not in i.part.pid]
    worst, rows = [], []
    for ax, ay in ((deg, 0), (-deg, 0), (0, deg), (0, -deg), (deg, deg), (-deg, -deg), (deg, -deg), (-deg, deg)):
        ry = cq.Location(piv) * cq.Location(cq.Vector(), cq.Vector(0, 1, 0), ay) * cq.Location(-piv)
        rx = cq.Location(piv) * cq.Location(cq.Vector(), cq.Vector(1, 0, 0), ax) * cq.Location(-piv)
        moving = []
        for i in insts:
            r = role(i)
            if r == "inner":
                moving.append((i, i.part.shape.moved(ry * rx * i.loc)))
            elif r == "middle":
                moving.append((i, i.part.shape.moved(ry * i.loc)))
        n = 0
        for im, sm in moving:
            bm = sm.BoundingBox()
            for jf, sf in fixed:
                bf = sf.BoundingBox()
                if (bm.xmax < bf.xmin or bf.xmax < bm.xmin or bm.ymax < bf.ymin or bf.ymax < bm.ymin
                        or bm.zmax < bf.zmin or bf.zmax < bm.zmin):
                    continue
                v = sm.intersect(sf).Volume()
                if v > 0.5:
                    n += 1
                    rows.append((ax, ay, v, f"{im.part.pid}-{im.n}", f"{jf.part.pid}-{jf.n}"))
        worst.append(((ax, ay), n))
    print("  gimbal sweep:", ", ".join(f"({a:+.0f},{b:+.0f}) {n} clashes" for (a, b), n in worst))
    return rows


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--quick", action="store_true")
    args = ap.parse_args()
    OUT.mkdir(exist_ok=True)
    t0 = time.time()
    allinst = model.instances()
    flight = model.config(allinst, "stowed")
    landing = model.config(allinst, "deployed")
    print(f"model: {len(unique_parts(allinst))} unique parts, {len(flight)} pieces ({time.time() - t0:.0f} s)")
    print(f"  overall length {D.L:.1f} mm, body tube {D.TUBE_TOTAL:.1f} mm ({D.TUBE_TOTAL / 25.4:.2f} in), "
          f"foot radius {D.FOOT_RADIUS:.0f} mm, feet {D.FEET_BELOW_AFT:.0f} mm below the aft end")
    mass_report(flight, landing)
    bom(flight, landing)
    if not args.quick:
        rep = ["# Interference check", "",
               "Every pair of solids whose bounding boxes touch is intersected; overlaps above 0.5 mm^3 are listed.", ""]
        for label, insts in (("flight", flight), ("landing", landing)):
            hits, joints, n = interference(insts, label)
            rep += [f"## {label} configuration", "",
                    f"{n} close pairs checked: **{len(hits)} clashes**, {len(joints)} designed joints "
                    "(ball links, pushrods and cords drawn seated in the horn, lug or pin they attach to).", ""]
            for title, rows in (("Clashes", hits), ("Designed joints", joints)):
                if rows:
                    rep += [f"### {title}", "", "| Overlap (mm^3) | Part A | Part B |", "|---:|---|---|"]
                    rep += [f"| {v:.1f} | {a} | {b} |" for v, a, b in rows]
                    rep.append("")
        sweep = gimbal_sweep(flight)
        rep += [f"## Gimbal sweep (+-{D.GIMBAL_MAX_DEG:.0f} deg on each axis and both together)", "",
                f"**{len(sweep)} clashes** between the parts the gimbal carries and the fixed structure.", ""]
        if sweep:
            rep += ["| X tilt | Y tilt | Overlap (mm^3) | Moving | Fixed |", "|---:|---:|---:|---|---|"]
            rep += [f"| {a:+.0f} | {b:+.0f} | {v:.1f} | {m} | {f} |" for a, b, v, m, f in sweep]
        (OUT / "interference.md").write_text("\n".join(rep) + "\n")
    export_viewer(flight, landing)       # first: coarse meshes for the web (OCC keeps the finest mesh made)
    export_stl_dxf(allinst)
    export_steps(flight, landing)
    print(f"done in {time.time() - t0:.0f} s -> {OUT}")


if __name__ == "__main__":
    main()
