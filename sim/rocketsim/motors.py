"""Solid rocket motor models.

Two ways to get a thrust curve:

1. ``Motor.from_eng(path)`` loads a RASP ``.eng`` file. This is what you should
   use for real design work. Download the file for your exact motor from
   https://www.thrustcurve.org (search the motor, then "Download" -> RASP).
2. ``builtin(name)`` returns an *approximate* curve generated from the
   certified headline numbers (total impulse, burn time, peak thrust,
   propellant mass). These are good enough for sizing and for exercising the
   flight software, but they are not certification data.

Mass during the burn is modelled as decreasing in proportion to the impulse
delivered so far (the standard RASP / OpenRocket assumption).
"""

from __future__ import annotations

from dataclasses import dataclass, field
from pathlib import Path

import numpy as np


@dataclass
class Motor:
    name: str
    diameter_mm: float
    length_mm: float
    propellant_kg: float
    total_kg: float
    t: np.ndarray          # s, starts at 0
    f: np.ndarray          # N
    propellant: str = "unknown"   # "black powder" or "APCP"
    notes: str = ""
    _cum_impulse: np.ndarray = field(init=False, repr=False)

    def __post_init__(self) -> None:
        self.t = np.asarray(self.t, dtype=float)
        self.f = np.asarray(self.f, dtype=float)
        if self.t[0] > 0.0:
            self.t = np.insert(self.t, 0, 0.0)
            self.f = np.insert(self.f, 0, 0.0)
        seg = 0.5 * (self.f[1:] + self.f[:-1]) * np.diff(self.t)
        self._cum_impulse = np.concatenate([[0.0], np.cumsum(seg)])

    # ------------------------------------------------------------------ props
    @property
    def burn_time(self) -> float:
        return float(self.t[-1])

    @property
    def total_impulse(self) -> float:
        return float(self._cum_impulse[-1])

    @property
    def average_thrust(self) -> float:
        return self.total_impulse / self.burn_time

    @property
    def peak_thrust(self) -> float:
        return float(self.f.max())

    @property
    def casing_kg(self) -> float:
        return self.total_kg - self.propellant_kg

    # --------------------------------------------------------------- queries
    def thrust(self, t_since_ignition: float) -> float:
        if t_since_ignition < 0.0 or t_since_ignition > self.burn_time:
            return 0.0
        return float(np.interp(t_since_ignition, self.t, self.f))

    def impulse_delivered(self, t_since_ignition: float) -> float:
        if t_since_ignition <= 0.0:
            return 0.0
        if t_since_ignition >= self.burn_time:
            return self.total_impulse
        # Integrate exactly up to t on the piecewise-linear curve.
        i = int(np.searchsorted(self.t, t_since_ignition)) - 1
        f_t = self.thrust(t_since_ignition)
        return float(self._cum_impulse[i]
                     + 0.5 * (self.f[i] + f_t) * (t_since_ignition - self.t[i]))

    def thrust_v(self, tb: np.ndarray) -> np.ndarray:
        """Vectorised thrust lookup (0 outside the burn)."""
        f = np.interp(tb, self.t, self.f, left=0.0, right=0.0)
        return np.where((tb < 0.0) | (tb > self.burn_time), 0.0, f)

    def mass_v(self, tb: np.ndarray) -> np.ndarray:
        """Vectorised motor mass (uses the cumulative-impulse table)."""
        imp = np.interp(np.clip(tb, 0.0, self.burn_time), self.t, self._cum_impulse)
        return self.total_kg - imp / self.total_impulse * self.propellant_kg

    def mass(self, t_since_ignition: float) -> float:
        """Motor mass (casing + remaining propellant) in kg."""
        burnt = self.impulse_delivered(t_since_ignition) / self.total_impulse
        return self.total_kg - burnt * self.propellant_kg

    def scaled(self, impulse_scale: float = 1.0, time_scale: float = 1.0,
               name: str | None = None) -> "Motor":
        """Return a copy with dispersed impulse/burn time (for Monte Carlo).

        Thrust is scaled so that total impulse changes by ``impulse_scale``
        while the burn time changes by ``time_scale``.
        """
        return Motor(
            name=name or self.name,
            diameter_mm=self.diameter_mm,
            length_mm=self.length_mm,
            propellant_kg=self.propellant_kg,
            total_kg=self.total_kg,
            t=self.t * time_scale,
            f=self.f * impulse_scale / time_scale,
            propellant=self.propellant,
            notes=self.notes,
        )

    def summary(self) -> str:
        return (f"{self.name}: I={self.total_impulse:.1f} N*s, "
                f"Favg={self.average_thrust:.1f} N, Fmax={self.peak_thrust:.1f} N, "
                f"tb={self.burn_time:.2f} s, prop={self.propellant_kg*1000:.1f} g, "
                f"total={self.total_kg*1000:.1f} g ({self.propellant})")

    # ----------------------------------------------------------------- input
    @classmethod
    def from_eng(cls, path: str | Path, propellant: str = "unknown") -> "Motor":
        """Parse a RASP .eng file (format: thrustcurve.org/info/raspformat.html)."""
        header = None
        pts: list[tuple[float, float]] = []
        for raw in Path(path).read_text().splitlines():
            line = raw.split(";", 1)[0].strip()
            if not line:
                continue
            if header is None:
                header = line.split()
                continue
            a, b = line.split()[:2]
            pts.append((float(a), float(b)))
        if header is None or len(header) < 7:
            raise ValueError(f"{path}: missing RASP header line")
        name, dia, length, _delays, prop_kg, total_kg = header[:6]
        t, f = zip(*pts)
        return cls(name=name, diameter_mm=float(dia), length_mm=float(length),
                   propellant_kg=float(prop_kg), total_kg=float(total_kg),
                   t=np.array(t), f=np.array(f), propellant=propellant,
                   notes=f"loaded from {Path(path).name}")


def _shaped_curve(total_impulse: float, burn_time: float, peak: float,
                  t_peak: float, shape: list[tuple[float, float]],
                  tail: float = 0.06) -> tuple[np.ndarray, np.ndarray]:
    """Build a curve: linear rise to ``peak`` at ``t_peak``, then a sustain
    whose *relative* shape is ``shape`` (list of (fraction_of_burn, level)),
    scaled so the integral equals ``total_impulse``, then a short tail-off."""
    t_end_sustain = burn_time * (1.0 - tail)
    ts = [0.0, 0.25 * t_peak, t_peak]
    fs = [0.0, 0.35 * peak, peak]
    rel_t = [max(t_peak + 1e-3, s * t_end_sustain) for s, _ in shape]
    rel_f = [lvl for _, lvl in shape]
    # Solve for the sustain scale k so that total impulse matches.
    t_all = np.array(ts + rel_t + [burn_time])

    def impulse(k: float) -> float:
        f_all = np.array(fs + [k * v for v in rel_f] + [0.0])
        return float(np.trapezoid(f_all, t_all))

    lo, hi = 0.0, 10.0 * peak
    for _ in range(80):
        mid = 0.5 * (lo + hi)
        if impulse(mid) < total_impulse:
            lo = mid
        else:
            hi = mid
    k = 0.5 * (lo + hi)
    f_all = np.array(fs + [k * v for v in rel_f] + [0.0])
    return t_all, f_all


# Headline numbers come from manufacturer / NAR certification data (see
# docs/04-propulsion.md for sources). Curve *shapes* are approximations.
_BUILTIN = {
    # Estes F15 (black powder, 29 mm). NAR cert: 49.61 N*s, 3.45 s, peak
    # 25.26 N, 60.0 g propellant, 101.8 g loaded. Classic BP curve: ignition
    # spike then a long, nearly flat sustain around 14-15 N.
    "Estes_F15": dict(
        total_impulse=49.61, burn_time=3.45, peak=25.26, t_peak=0.40,
        shape=[(0.17, 0.95), (0.25, 0.82), (0.40, 0.80), (0.70, 0.81),
               (0.90, 0.80), (1.00, 0.76)],
        diameter_mm=29, length_mm=114, propellant_kg=0.0600, total_kg=0.1018,
        propellant="black powder"),
    # AeroTech G40W (White Lightning APCP, 29 mm single use). NAR cert:
    # 97.1 N*s, 2.38 s, 53.8 g propellant. Regressive-ish White Lightning
    # curve. Loaded mass ~ 123 g (check your lot).
    "AeroTech_G40W": dict(
        total_impulse=97.1, burn_time=2.38, peak=58.0, t_peak=0.12,
        shape=[(0.10, 1.00), (0.30, 0.95), (0.55, 0.85), (0.80, 0.72),
               (1.00, 0.62)],
        diameter_mm=29, length_mm=124, propellant_kg=0.0538, total_kg=0.1230,
        propellant="APCP"),
}


def builtin(name: str) -> Motor:
    if name not in _BUILTIN:
        raise KeyError(f"unknown builtin motor {name!r}; have {sorted(_BUILTIN)}")
    d = dict(_BUILTIN[name])
    t, f = _shaped_curve(d.pop("total_impulse"), d.pop("burn_time"),
                         d.pop("peak"), d.pop("t_peak"), d.pop("shape"))
    return Motor(name=name, t=t, f=f,
                 notes="APPROXIMATE curve built from certified headline numbers",
                 **d)


def builtin_names() -> list[str]:
    return sorted(_BUILTIN)


def register_builtin(name: str, **spec) -> None:
    """Add/override a builtin approximate motor (see _BUILTIN for fields)."""
    _BUILTIN[name] = spec
