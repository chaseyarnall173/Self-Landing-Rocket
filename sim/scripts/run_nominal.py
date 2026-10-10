"""Fly the nominal mission once and make the time-history figures.

    python sim/scripts/run_nominal.py [wind_mps]
"""
import sys

import _path
from rocketsim import Dispersions, GuidanceConfig, simulate
from rocketsim.designs import slr1
from rocketsim import plots

wind = float(sys.argv[1]) if len(sys.argv) > 1 else 2.0
v = slr1()
r = simulate(v, GuidanceConfig(), Dispersions(wind_mps=wind))
print(f"{v.name}: outcome={r.outcome} apogee={r.apogee_m:.1f} m "
      f"({r.apogee_m * 3.281:.0f} ft) liftoff={r.liftoff_mass_kg * 1000:.0f} g")
for t, e in r.events:
    print(f"  t={t:6.2f}  {e}")
print("  touchdown:", {k: (round(float(x), 2) if isinstance(x, float) else x)
                       for k, x in r.touchdown.items() if k not in ("chute_open",)})
_path.IMG.mkdir(parents=True, exist_ok=True)
plots.mission(r, str(_path.IMG / "mission_nominal.png"),
              f"SLR-1 nominal mission (2x AeroTech G40W, {wind:g} m/s wind)")
plots.landing_zoom(r, str(_path.IMG / "landing_burn_nominal.png"),
                   "Landing burn: exhaust-paddle throttle + TVC")
print("figures written to", _path.IMG)
