"""The whole SLR-1 model as a flat list of part instances."""
from . import airframe, bays, legs, propulsion

GROUP_ORDER = ["Nose", "Forward airframe", "Top airbrakes", "Backup chute bay", "Avionics bay",
               "Aft airframe", "Landing legs", "Motor bay", "Throttle", "Motors"]


def instances():
    out = []
    for mod in (airframe, bays, propulsion, legs):
        out.extend(mod.parts())
    return out


def config(insts, state):
    """Instances present in a configuration: 'stowed' (pad / flight) or 'deployed' (landing)."""
    return [i for i in insts if i.state in ("both", state)]
