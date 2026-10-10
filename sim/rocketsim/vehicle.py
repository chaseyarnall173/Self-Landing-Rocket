"""Vehicle definition: geometry, mass properties, and a component-build-up
aerodynamic model that works for nose-first AND tail-first flow.

Coordinates along the body are measured from the nose tip, positive aft
(the usual OpenRocket / Barrowman convention).
"""

from __future__ import annotations

import math
from dataclasses import dataclass, field

from .motors import Motor

G0 = 9.80665


def air_density(alt_m: float, ground_temp_c: float = 15.0,
                ground_alt_m: float = 0.0) -> float:
    """ISA troposphere density (kg/m^3)."""
    h = ground_alt_m + max(alt_m, 0.0)
    t0 = ground_temp_c + 273.15
    t = t0 - 0.0065 * h
    p = 101325.0 * (t / t0) ** 5.2559
    return p / (287.05 * t)


@dataclass
class AeroComponent:
    """One contributor to normal force / axial drag.

    cna_leading / cna_trailing: normal-force slope (per radian, referenced to
    the body cross-section area) when this component is on the upstream
    (leading) or downstream (trailing) half of the vehicle for the current
    flow direction. For fins and plates the two are equal; for a nose cone
    or a blunt base they are not (a closing nose cone in the wake produces
    almost nothing, a leading one produces CNa ~= 2).
    """
    name: str
    x_m: float                 # station of its centre of pressure (from nose)
    cna_leading: float = 0.0
    cna_trailing: float = 0.0
    ca: float = 0.0            # axial drag coefficient contribution
    deployable: bool = False   # only active when deployed


@dataclass
class Vehicle:
    name: str
    length_m: float
    diameter_m: float
    dry_mass_kg: float           # everything except motors
    dry_cg_m: float              # from nose
    dry_inertia_kgm2: float      # pitch inertia of dry vehicle about its own CG
    ascent_motor: Motor
    landing_motor: Motor
    motor_station_m: float       # CG station of both motors (from nose)
    gimbal_station_m: float      # TVC pivot station (from nose)
    ascent_offset_m: float = 0.0   # lateral thrust-line offset (2-motor mount)
    landing_offset_m: float = 0.0
    gimbal_limit_deg: float = 7.0
    servo_tau_s: float = 0.030     # first-order servo+linkage lag
    servo_rate_dps: float = 400.0  # gimbal-angle rate limit
    leg_drop_m: float = 0.12       # feet below the aft end once deployed
    leg_ca_deployed: float = 0.6
    body_crossflow_cd: float = 1.2
    crossflow_eta: float = 0.65
    ca_nose_first: float = 0.55    # base + skin friction, legs stowed
    ca_tail_first: float = 0.95    # blunt aft end leading
    components: list[AeroComponent] = field(default_factory=list)
    notes: str = ""

    # ---------------------------------------------------------------- geometry
    @property
    def ref_area(self) -> float:
        return math.pi * self.diameter_m ** 2 / 4.0

    @property
    def planform_area(self) -> float:
        return self.length_m * self.diameter_m

    # ------------------------------------------------------------ mass props
    def mass_props(self, t_asc: float | None, t_land: float | None,
                   landing_jettisoned: bool = False) -> tuple[float, float, float]:
        """Return (mass, cg_station, pitch_inertia) for the given burn clocks
        (None = not ignited)."""
        m_a = self.ascent_motor.mass(t_asc) if t_asc is not None else self.ascent_motor.total_kg
        m_l = 0.0 if landing_jettisoned else (
            self.landing_motor.mass(t_land) if t_land is not None else self.landing_motor.total_kg)
        m_m = m_a + m_l
        m = self.dry_mass_kg + m_m
        cg = (self.dry_mass_kg * self.dry_cg_m + m_m * self.motor_station_m) / m
        inertia = (self.dry_inertia_kgm2
                   + self.dry_mass_kg * (self.dry_cg_m - cg) ** 2
                   + m_m * (self.motor_station_m - cg) ** 2)
        return m, cg, inertia

    def liftoff_mass(self) -> float:
        return self.mass_props(None, None)[0]

    def propellant_total_g(self) -> float:
        return 1000.0 * (self.ascent_motor.propellant_kg + self.landing_motor.propellant_kg)

    # ------------------------------------------------------------------ aero
    def aero(self, v_axial: float, v_normal: float, rho: float, cg_m: float,
             flaps: float = 0.0, legs: bool = False) -> tuple[float, float, float]:
        """Aerodynamic loads in the body frame.

        v_axial  : air-relative velocity along the body axis (+ = nose-first)
        v_normal : air-relative velocity normal to the body axis
        flaps    : deployable-component deployment fraction 0..1
        Returns (F_axial, F_normal, M_cg) where F_axial is along +body axis,
        F_normal along +normal axis, and M_cg is the pitching moment about the
        CG (positive = nose rotates toward +normal).
        """
        speed2 = v_axial ** 2 + v_normal ** 2
        if speed2 < 1e-6:
            return 0.0, 0.0, 0.0
        speed = math.sqrt(speed2)
        q = 0.5 * rho * speed2
        a = self.ref_area
        nose_first = v_axial >= 0.0
        sin_a = v_normal / speed            # signed sin(alpha)

        ca = self.ca_nose_first if nose_first else self.ca_tail_first
        if legs:
            ca += self.leg_ca_deployed
        normal = 0.0
        moment = 0.0
        mid = 0.5 * self.length_m
        for c in self.components:
            frac = flaps if c.deployable else 1.0
            if frac <= 0.0:
                continue
            leading = (c.x_m < mid) == nose_first
            cna = c.cna_leading if leading else c.cna_trailing
            ca += frac * c.ca
            # Linear normal force opposing the normal flow component.
            fn = -q * a * frac * cna * sin_a
            normal += fn
            moment += fn * (cg_m - c.x_m)   # lever: +ve when component is ahead of CG
        # Viscous crossflow (Jorgensen / Allen-Perkins), acts at planform centroid.
        fn_x = -q * a * self.crossflow_eta * self.body_crossflow_cd * \
            (self.planform_area / a) * sin_a * abs(sin_a)
        normal += fn_x
        moment += fn_x * (cg_m - mid)
        axial = -math.copysign(q * a * ca * abs(v_axial) / speed, v_axial)
        return axial, normal, moment

    def static_margin_cal(self, cg_m: float, flaps: float, nose_first: bool) -> float:
        """Small-angle static margin in calibers for the given flow direction
        (positive = stable). Uses only the linear components."""
        mid = 0.5 * self.length_m
        cna_sum = 0.0
        mom = 0.0
        for c in self.components:
            frac = flaps if c.deployable else 1.0
            if frac <= 0.0:
                continue
            leading = (c.x_m < mid) == nose_first
            cna = frac * (c.cna_leading if leading else c.cna_trailing)
            cna_sum += cna
            mom += cna * c.x_m
        if cna_sum <= 0.0:
            return float("nan")
        x_cp = mom / cna_sum
        # Nose-first: stable when CP is aft of CG. Tail-first: when CP is forward.
        margin = (x_cp - cg_m) if nose_first else (cg_m - x_cp)
        return margin / self.diameter_m
