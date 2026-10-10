"""Monte Carlo dispersion campaign + figures.

    python sim/scripts/run_montecarlo.py [n_runs]
"""
import sys
import time

import numpy as np

import _path
from rocketsim import GuidanceConfig, plots
from rocketsim.designs import slr1
from rocketsim.montecarlo import DispersionModel, run, summarize
from rocketsim.sim import TD_LIMITS

n = int(sys.argv[1]) if len(sys.argv) > 1 else 300
t0 = time.time()
model = DispersionModel()
res = run(slr1(), GuidanceConfig(), n=n, seed=7, model=model)
print(f"Dispersion model: {model}")
print(summarize(res))
print(f"({time.time() - t0:.0f} s)")
rows = res["rows"]
_path.IMG.mkdir(parents=True, exist_ok=True)
plots.mc_histograms(rows, str(_path.IMG / "mc_touchdown.png"),
                    f"Monte Carlo, {n} dispersed flights (wind 0-4 m/s, ignition delay 0.30 +/- 0.08 s)",
                    TD_LIMITS["vz"], TD_LIMITS["vx"])
bins = [(0, 1), (1, 2), (2, 3), (3, 4)]
labels, pct, cnt = [], [], []
for lo, hi in bins:
    sub = [x for x in rows if lo <= abs(x["wind"]) < hi]
    labels.append(f"{lo}-{hi}")
    cnt.append(len(sub))
    pct.append(100.0 * np.mean([x["outcome"] == "landed" for x in sub]) if sub else 0.0)
    print(f"  wind {lo}-{hi} m/s: {pct[-1]:.0f}% of {cnt[-1]}")
plots.success_vs_bins(labels, pct, cnt, str(_path.IMG / "mc_wind.png"),
                      "Clean-landing rate vs. surface wind", "Wind at 10 m (m/s)")
print("figures written to", _path.IMG)
