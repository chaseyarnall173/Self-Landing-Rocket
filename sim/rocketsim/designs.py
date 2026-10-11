"""Baseline vehicle definition ("SLR-1") with an itemised mass budget.

The rev C budget is rolled up from the CAD model (``python cad/build.py``
writes cad/out/mass_items.json). It is still an estimate: WEIGH YOUR PARTS
and update the table. ``python sim/scripts/mass_budget.py`` prints the
budget, CG and static margins so you can see the effect of each change.
"""

from __future__ import annotations

import math
from dataclasses import dataclass

from pathlib import Path

from .motors import Motor, builtin
from .vehicle import AeroComponent, Vehicle

IN = 0.0254

# ---------------------------------------------------------------------------
# Motors: official RASP curves from ThrustCurve.org (sim/data/motors/*.eng)
# ---------------------------------------------------------------------------
DATA_DIR = Path(__file__).resolve().parent.parent / "data" / "motors"
PROPELLANT = {"Estes_F15": "black powder"}


# Extra hardware mass for reloadable motors (aluminium case + closures), which
# the RASP total weight does not include. ESTIMATES - weigh yours.
RELOAD_HARDWARE_KG = {"AeroTech_G25W": 0.075}


def motor(name: str) -> Motor:
    """Load sim/data/motors/<name>.eng; fall back to the approximate builtin."""
    path = DATA_DIR / f"{name}.eng"
    if path.exists():
        m = Motor.from_eng(path, propellant=PROPELLANT.get(name, "APCP"))
        m.name = name
        m.total_kg += RELOAD_HARDWARE_KG.get(name, 0.0)
        return m
    return builtin(name)


@dataclass
class MassItem:
    name: str
    mass_kg: float
    station_m: float          # CG of the item, from nose tip
    length_m: float = 0.0     # >0 = treat as uniform rod for inertia


# Rev C mass budget, rolled up from the CAD model (cad/out/mass_items.json, written by
# `python cad/build.py`): CAD volume x material density for made parts, catalogue
# estimates for bought parts. Stations are from the nose tip; the vehicle is 60 in long.
SLR1_ITEMS = [
    MassItem("Nose cone: printed PETG ogive (0.8 mm wall) + 2 in shoulder", 0.0991, 0.244, 0.344),
    MassItem("Top airbrakes: petal hinges, springs, tabs, latch ring + servo (petals are tube)", 0.0213, 0.471, 0.271),
    MassItem("Backup chute (30 in), Nomex, shock cord, ejector, door hinge/latch + servo", 0.0969, 0.630, 0.129),
    MassItem("Avionics: Teensy 4.1, BMI088, BMP390, GPS, LoRa, 2S 450 mAh, switches", 0.1117, 0.768, 0.148),
    MassItem("Couplers, bulkheads, sled, aft carbon liner, thrust plate, leg doublers", 0.2504, 0.861, 1.173),
    MassItem("Body tube: LOC 3.9 in kraft, 47.25 in in 5 pieces (incl. petals + door)", 0.4020, 0.923, 1.202),
    MassItem("4 landing legs (8x7 CF, dampers, TPU feet, clevises) + release servo", 0.1005, 1.090, 0.854),
    MassItem("Wiring harness, cables, screws, epoxy fillets", 0.0378, 1.132, 1.071),
    MassItem("Dual-motor TVC gimbal: rings, posts, kraft sleeves, 2 servos, pushrods", 0.0938, 1.427, 0.137),
    MassItem("4 fins, 3/32 in birch ply (90 % of the 4.7/2.4/3.0 in planform)", 0.0373, 1.472, 0.222),
    MassItem("Exhaust-paddle throttle: 2 ceramic paddles, arms, bracket, servo", 0.0326, 1.493, 0.063),
    MassItem("Lidar (TFmini-S) + bracket + window", 0.0103, 1.504, 0.036),
]
MOTOR_STATION_M = 1.433         # CG of the two motors (CAD)



def dry_mass_props(items: list[MassItem]) -> tuple[float, float, float]:
    m = sum(i.mass_kg for i in items)
    cg = sum(i.mass_kg * i.station_m for i in items) / m
    inertia = 0.0
    for i in items:
        inertia += i.mass_kg * (i.station_m - cg) ** 2 + i.mass_kg * i.length_m ** 2 / 12.0
    return m, cg, inertia


def barrowman_fins(n: int, root: float, tip: float, span: float, sweep: float,
                   body_d: float, root_le_station: float) -> tuple[float, float]:
    """Return (CN_alpha, CP station) for a fin set (Barrowman)."""
    r = body_d / 2.0
    mid = math.sqrt(span ** 2 + (sweep + tip / 2.0 - root / 2.0) ** 2)
    cn = (1.0 + r / (span + r)) * (4.0 * n * (span / body_d) ** 2) / \
        (1.0 + math.sqrt(1.0 + (2.0 * mid / (root + tip)) ** 2))
    xf = root_le_station + (sweep / 3.0) * (root + 2 * tip) / (root + tip) + \
        (1.0 / 6.0) * ((root + tip) - root * tip / (root + tip))
    return cn, xf


def slr1(drag_device: str = "petals", fins: bool = True,
         items: list[MassItem] | None = None,
         ascent: str = "AeroTech_F52C", landing: str = "AeroTech_F25W",
         gridfin_size_m: float = 0.055, gridfin_station_m: float = 0.42,
         fin_scale: float = 0.9, petal_len_m: float = 0.150) -> Vehicle:
    """Baseline SLR-1.

    drag_device: "petals" (baseline: four airbrake petals cut from the body
    tube just below the nose, sprung open at apogee - the rocket's Falcon-9
    grid fins; 150 mm long in rev C), "gridfins" (square lattice fins, sized by gridfin_size_m),
    "drogue" (9 in drogue on a nose leash - the earlier design), "flaps" or
    "none".
    """
    items = items or SLR1_ITEMS
    length = 60 * IN
    d = 4.0 * IN
    a_ref = math.pi * d * d / 4.0
    m, cg, inertia = dry_mass_props(items)

    comps = [
        # Ogive nose: CNa = 2 at 0.466 * exposed length (12.75 in).
        AeroComponent("nose", x_m=0.466 * 12.75 * IN, cna_leading=2.0, cna_trailing=0.0),
        # Blunt aft end + gimbal hardware when it is the leading face.
        AeroComponent("aft end", x_m=length - 0.03, cna_leading=1.0, cna_trailing=0.0),
    ]
    if fins:
        cnf, xf = barrowman_fins(4, 4.7 * IN * fin_scale, 2.4 * IN * fin_scale,
                                 3.0 * IN * fin_scale, 2.3 * IN * fin_scale, d,
                                 length - 4.7 * IN * fin_scale - 0.01)
        comps.append(AeroComponent("fins", x_m=xf, cna_leading=cnf, cna_trailing=cnf, ca=0.12))
    if drag_device == "drogue":
        cds = 0.80 * math.pi * (9 * IN / 2) ** 2          # 9 in drogue, Cd 0.8
        c = cds / a_ref
        comps.append(AeroComponent("drogue", x_m=0.0, cna_leading=c, cna_trailing=c,
                                   ca=c, deployable=True))
    elif drag_device == "petals":
        # Four curved airbrake petals cut from the body tube (one quadrant
        # each, ~80 mm wide x petal_len_m long), hinged at their forward edge
        # (station 0.395 m) and sprung open to 90 deg at apogee. Flat-plate Cd ~1.2.
        cds = 1.2 * 4 * (0.080 * petal_len_m)
        c = cds / a_ref
        comps.append(AeroComponent("airbrake petals", x_m=0.395 + 0.01, cna_leading=c,
                                   cna_trailing=c, ca=c, deployable=True))
    elif drag_device == "gridfins":
        # Falcon-9-style grid fins near the top, folded flat for ascent and
        # deployed at apogee. Lattice fins: Cd ~1.0 on their frontal area.
        cds = 1.0 * 4 * gridfin_size_m ** 2
        c = cds / a_ref
        comps.append(AeroComponent("grid fins", x_m=gridfin_station_m, cna_leading=c,
                                   cna_trailing=c, ca=c, deployable=True))
    elif drag_device == "flaps":
        cds = 1.2 * 4 * (0.10 * 0.08)                      # 4 petals 100x80 mm
        c = cds / a_ref
        comps.append(AeroComponent("flaps", x_m=0.45, cna_leading=c, cna_trailing=c,
                                   ca=c, deployable=True))

    return Vehicle(
        name=f"SLR-1 ({drag_device}{', fins' if fins else ', finless'})",
        length_m=length, diameter_m=d,
        dry_mass_kg=m, dry_cg_m=cg, dry_inertia_kgm2=inertia,
        ascent_motor=motor(ascent),
        landing_motor=motor(landing),
        motor_station_m=MOTOR_STATION_M, gimbal_station_m=1.46,
        ascent_offset_m=+0.0165, landing_offset_m=-0.0165,
        gimbal_limit_deg=7.0, servo_tau_s=0.030, servo_rate_dps=400.0,
        leg_drop_m=0.22, leg_ca_deployed=0.6,
        components=comps,
        notes="Side-by-side dual-motor gimbal; landing motor has exhaust paddles.",
    )
