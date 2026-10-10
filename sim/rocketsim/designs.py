"""Baseline vehicle definition ("SLR-1") with an itemised mass budget.

Every number here is a design estimate: WEIGH YOUR PARTS and update the
table. ``python sim/scripts/mass_budget.py`` prints the budget, CG and static
margins so you can see the effect of each change.
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


# Lean 4-inch build. Stations are from the nose tip; vehicle is 60 in long.
SLR1_ITEMS = [
    MassItem("Nose cone (3D-printed PETG ogive, 0.8 mm wall) + shoulder", 0.095, 0.24, 0.40),
    MassItem("Top airbrake petals: 4 cut from the tube, hinges, springs, latch servo", 0.035, 0.48),
    MassItem("Backup main chute (30 in) + shock cord + spring/servo door", 0.085, 0.55),
    MassItem("Avionics bay: Teensy 4.1 board, BMI088, BMP390, LoRa, 2S 450 mAh", 0.105, 0.80, 0.15),
    MassItem("Body tube: LOC 3.9 in kraft, 43.5 in", 0.374, 0.97, 1.10),
    MassItem("Couplers, bulkheads, motor-bay frame", 0.080, 0.95, 0.60),
    MassItem("Dual-motor 2-axis TVC gimbal + 2 servos", 0.130, 1.36),
    MassItem("Exhaust-paddle throttle (2 ceramic paddles, Al arms, servo)", 0.050, 1.49),
    MassItem("4 fins, 1/8 in birch ply, root 4.7 in tip 2.4 in span 3 in", 0.045, 1.45),
    MassItem("4 landing legs (CF tube, printed hinges, springs, TPU feet) + pin servo", 0.085, 1.30),
    MassItem("Lidar (TFmini-S) + bracket", 0.012, 1.46),
    MassItem("Wiring, hardware, epoxy, paint", 0.045, 0.95, 1.0),
]


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
         ascent: str = "AeroTech_F42T", landing: str = "AeroTech_F25W",
         gridfin_size_m: float = 0.055, gridfin_station_m: float = 0.42,
         fin_scale: float = 1.0) -> Vehicle:
    """Baseline SLR-1.

    drag_device: "petals" (baseline: four airbrake petals cut from the body
    tube just below the nose, sprung open at apogee - the rocket's Falcon-9
    grid fins), "gridfins" (square lattice fins, sized by gridfin_size_m),
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
        # each, ~80 mm wide x 125 mm long), hinged at their forward edge and
        # sprung open to 90 deg at apogee. Flat-plate Cd ~1.2.
        cds = 1.2 * 4 * (0.080 * 0.125)
        c = cds / a_ref
        comps.append(AeroComponent("airbrake petals", x_m=0.43, cna_leading=c,
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
        motor_station_m=1.40, gimbal_station_m=1.46,
        ascent_offset_m=+0.0165, landing_offset_m=-0.0165,
        gimbal_limit_deg=7.0, servo_tau_s=0.030, servo_rate_dps=400.0,
        leg_drop_m=0.22, leg_ca_deployed=0.6,
        components=comps,
        notes="Side-by-side dual-motor gimbal; landing motor has exhaust paddles.",
    )
