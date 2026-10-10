"""Monte Carlo dispersion analysis.

The dispersion ranges below are the *assumptions* that most need to be
replaced by your own test data (see docs/09-test-plan.md):
ignition delay, motor-to-motor impulse scatter and paddle throttle authority.
"""

from __future__ import annotations

import copy
import multiprocessing
import os
import random
from collections import Counter
from dataclasses import dataclass

import numpy as np

from .guidance import GuidanceConfig
from .sim import Dispersions, simulate
from .vehicle import Vehicle


@dataclass
class DispersionModel:
    delay_mean_s: float = 0.30
    delay_sigma_s: float = 0.08
    delay_min_s: float = 0.12
    delay_max_s: float = 0.70
    impulse_sigma: float = 0.03       # NFPA 1125 allows far more; lots matter
    burn_time_sigma: float = 0.03
    throttle_min_mean: float = 0.25
    throttle_min_sigma: float = 0.03
    throttle_gain_sigma: float = 0.03
    cd_sigma: float = 0.10
    mass_sigma: float = 0.02
    wind_max_mps: float = 4.0
    launch_tilt_sigma_deg: float = 1.0
    misalign_sigma_deg: float = 0.25
    alt_bias_sigma_m: float = 0.15
    gyro_bias_sigma_dps: float = 0.05
    no_ignition_prob: float = 0.0

    def sample(self, rng: random.Random, seed: int) -> Dispersions:
        def clip(x, lo, hi):
            return max(lo, min(hi, x))
        return Dispersions(
            ascent_impulse_scale=rng.gauss(1, self.impulse_sigma),
            ascent_time_scale=rng.gauss(1, self.burn_time_sigma),
            landing_impulse_scale=rng.gauss(1, self.impulse_sigma),
            landing_time_scale=rng.gauss(1, self.burn_time_sigma),
            landing_ignition_delay_s=clip(rng.gauss(self.delay_mean_s, self.delay_sigma_s),
                                          self.delay_min_s, self.delay_max_s),
            throttle_min_true=clip(rng.gauss(self.throttle_min_mean, self.throttle_min_sigma),
                                   0.15, 0.7),
            throttle_gain_true=rng.gauss(1, self.throttle_gain_sigma),
            cd_scale=rng.gauss(1, self.cd_sigma),
            dry_mass_scale=rng.gauss(1, self.mass_sigma),
            wind_mps=rng.uniform(-self.wind_max_mps, self.wind_max_mps),
            launch_tilt_deg=rng.gauss(0, self.launch_tilt_sigma_deg),
            thrust_misalign_deg=rng.gauss(0, self.misalign_sigma_deg),
            alt_bias_m=rng.gauss(0, self.alt_bias_sigma_m),
            gyro_bias_dps=rng.gauss(0, self.gyro_bias_sigma_dps),
            landing_fails_to_ignite=rng.random() < self.no_ignition_prob,
            seed=seed,
        )


def _one(args) -> dict:
    veh, gcfg, d = args
    r = simulate(veh, copy.deepcopy(gcfg), d, log_every=50)
    td = r.touchdown
    return dict(
        outcome=r.outcome, apogee=r.apogee_m,
        vz=td.get("vz", np.nan), vx=td.get("vx", np.nan),
        theta=td.get("theta_deg", np.nan), burn_clock=td.get("burn_clock", np.nan),
        hop=td.get("hop_max", np.nan), delay=d.landing_ignition_delay_s,
        wind=d.wind_mps, imp=d.landing_impulse_scale, kmin=d.throttle_min_true,
        drift=float(r.log["x"][-1]) if len(r.log["x"]) else np.nan,
    )


def run(veh: Vehicle, gcfg: GuidanceConfig, n: int = 200, seed: int = 1,
        model: DispersionModel | None = None, workers: int | None = None) -> dict:
    """Run ``n`` dispersed flights (in parallel processes when available)."""
    model = model or DispersionModel()
    rng = random.Random(seed)
    jobs = [(veh, gcfg, model.sample(rng, seed * 100000 + i)) for i in range(n)]
    workers = workers or os.cpu_count() or 1
    if workers > 1:
        with multiprocessing.get_context("fork").Pool(workers) as pool:
            rows = pool.map(_one, jobs, chunksize=4)
    else:
        rows = [_one(j) for j in jobs]
    counts = Counter(x["outcome"] for x in rows)
    return dict(rows=rows, counts=counts, n=n)


def summarize(res: dict) -> str:
    rows = res["rows"]
    n = res["n"]
    lines = [f"{n} runs: " + ", ".join(f"{k} {v} ({100 * v / n:.0f}%)"
                                       for k, v in res["counts"].most_common())]
    for key, label in [("apogee", "apogee m"), ("vz", "touchdown vz m/s"),
                       ("vx", "touchdown vx m/s"), ("theta", "touchdown tilt deg"),
                       ("burn_clock", "touchdown burn clock s"), ("hop", "hop m"),
                       ("drift", "landing distance from pad m")]:
        a = np.array([r[key] for r in rows], dtype=float)
        a = a[np.isfinite(a)]
        if len(a):
            lines.append(f"  {label:28s} mean {a.mean():7.2f}  p5 {np.percentile(a, 5):7.2f}"
                         f"  p95 {np.percentile(a, 95):7.2f}  min {a.min():7.2f}  max {a.max():7.2f}")
    return "\n".join(lines)
