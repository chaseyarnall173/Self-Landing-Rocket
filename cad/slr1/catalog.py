"""Part and instance records, materials, and the mass-budget mapping."""
from dataclasses import dataclass, field

import cadquery as cq

# density g/cm^3, display colour (sRGB 0-1)
MATERIALS = {
    "kraft":      (0.844, (0.93, 0.93, 0.90)),  # LOC 3.9 in tube: 103 g/ft
    "kraft_cpl":  (0.844, (0.80, 0.68, 0.50)),  # coupler stock, unpainted
    "ply":        (0.68,  (0.85, 0.72, 0.52)),  # aircraft birch ply
    "liteply":    (0.45,  (0.90, 0.80, 0.62)),  # poplar lite ply
    "petg":       (1.27,  (0.91, 0.35, 0.05)),  # printed PETG (orange nose)
    "cf_nylon":   (1.17,  (0.20, 0.21, 0.23)),  # printed CF-nylon
    "tpu":        (1.21,  (0.12, 0.12, 0.12)),
    "aluminium":  (2.70,  (0.78, 0.79, 0.80)),
    "steel":      (7.90,  (0.62, 0.63, 0.65)),
    "carbon":     (1.55,  (0.08, 0.08, 0.09)),
    "ceramic":    (2.30,  (0.92, 0.90, 0.84)),  # machinable alumina-silicate
    "ceramic_paper": (0.25, (0.95, 0.95, 0.93)),
    "epoxy":      (1.15,  (0.85, 0.80, 0.65)),
    "fibreglass": (1.80,  (0.90, 0.90, 0.85)),
    "kevlar":     (1.44,  (0.92, 0.78, 0.20)),
    "nomex":      (0.35,  (0.93, 0.80, 0.15)),
    "nylon_chute": (0.45, (0.95, 0.35, 0.20)),  # packed canopy, effective
    "pcb":        (1.85,  (0.10, 0.42, 0.25)),
    "perfboard":  (1.85,  (0.76, 0.62, 0.36)),
    "electronics": (2.0,  (0.10, 0.10, 0.12)),
    "servo":      (1.5,   (0.10, 0.10, 0.11)),
    "lipo":       (2.0,   (0.70, 0.72, 0.75)),
    "foam":       (0.03,  (0.35, 0.35, 0.38)),
    "brass":      (8.5,   (0.80, 0.65, 0.30)),
    "motor_f42t": (1.0,   (0.16, 0.30, 0.62)),  # mass from the RASP file
    "motor_f25w": (1.0,   (0.62, 0.14, 0.14)),
    "polycarb":   (1.20,  (0.75, 0.85, 0.95)),
}

# simulator mass-budget lines (sim/rocketsim/designs.py SLR1_ITEMS)
BUDGET = {
    "nose": "Nose cone + shoulder",
    "airbrakes": "Top airbrake petals, hinges, springs, latch servo",
    "chute": "Backup chute + shock cord + spring/servo door",
    "avionics": "Avionics bay: boards, sensors, radio, battery",
    "tube": "Body tube",
    "couplers": "Couplers, bulkheads, motor-bay frame",
    "gimbal": "Dual-motor TVC gimbal + 2 servos",
    "paddles": "Exhaust-paddle throttle",
    "fins": "4 fins",
    "legs": "4 landing legs + release",
    "lidar": "Lidar + bracket",
    "wiring": "Wiring, hardware, epoxy, paint",
    "motor_a": "Ascent motor AeroTech F42T",
    "motor_l": "Landing motor AeroTech F25W",
}

PROCESS = {
    "print": "3D print",
    "cut": "cut / laser-cut",
    "tube": "cut from tube stock",
    "machine": "machine",
    "bend": "bend / form",
    "buy": "buy",
    "layup": "hand lay-up / pack",
}


@dataclass
class Part:
    pid: str                    # part number, e.g. "LEG-STRUT"
    name: str
    shape: cq.Shape             # modelled in place (or in its local frame for instanced parts)
    material: str
    process: str
    group: str                  # subassembly
    budget: str                 # key into BUDGET
    mass_g: float | None = None  # catalogue mass for bought parts
    fill: float = 1.0           # print infill / packing factor
    stl: bool = False           # export STL for printing
    flat: cq.Workplane | None = None  # 2D outline for DXF (laser / CNC)
    note: str = ""
    color: tuple | None = None

    def density(self):
        return MATERIALS[self.material][0]

    def rgb(self):
        return self.color or MATERIALS[self.material][1]

    def mass(self):
        if self.mass_g is not None:
            return self.mass_g
        return self.shape.Volume() / 1000.0 * self.density() * self.fill


@dataclass
class Inst:
    part: Part
    loc: cq.Location = field(default_factory=cq.Location)
    state: str = "both"         # "both" | "stowed" | "deployed"
    n: int = 1

    def world(self):
        return self.part.shape.moved(self.loc)
