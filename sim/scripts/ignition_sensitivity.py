"""Touchdown speed vs. actual ignition delay, for two paddle authorities.

    python sim/scripts/ignition_sensitivity.py
"""
import numpy as np

import _path
from rocketsim import Dispersions, GuidanceConfig, plots, simulate
from rocketsim.designs import slr1

delays = np.round(np.arange(0.10, 0.56, 0.05), 2)
series = {}
for kmin in (0.25, 0.40):
    ys = []
    for d in delays:
        r = simulate(slr1(), GuidanceConfig(throttle_min=kmin),
                     Dispersions(wind_mps=1.0, landing_ignition_delay_s=float(d),
                                 throttle_min_true=kmin), log_every=50)
        # Worst impact: first touchdown, or falling back after a post-touchdown hop.
        vz = abs(float(r.touchdown.get("vz", np.nan)))
        hop = float(r.touchdown.get("hop_max", 0.0))
        ys.append(max(vz, (2 * 9.80665 * hop) ** 0.5))
        print(f"k_min={kmin:.2f} delay={d:.2f}s -> {r.outcome:12s} vz={vz:.2f} "
              f"hop={hop:.2f} m worst impact={ys[-1]:.2f} m/s")
    series[f"paddles cut {100 * (1 - kmin):.0f}% (k_min {kmin:g})"] = ys
plots.delay_sensitivity(delays, series, str(_path.IMG / "ignition_delay_sensitivity.png"),
                        "Why paddle authority matters: tolerance to ignition-delay error", 2.5)
