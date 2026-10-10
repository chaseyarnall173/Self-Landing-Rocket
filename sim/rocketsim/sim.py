"""Planar (3-DOF: x, z, pitch) flight simulation of the whole mission:
pad -> TVC boost -> coast -> apogee -> stable tail-first descent ->
landing burn with TVC + exhaust-paddle throttle -> legs -> touchdown.

Equations of motion
-------------------
Body axis unit vector  b = (sin th, cos th)   (points at the nose)
Body normal unit vector n = (cos th, -sin th)
Thrust acts at the gimbal pivot, a distance L aft of the CG, along the body
axis rotated by the gimbal angle delta:  F_T = k*T*(sin(th+delta), cos(th+delta)).
With r = -L*b + e*n (e = lateral thrust-line offset) the pitching moment is
    M = -(L*T*sin(delta) + e*T*cos(delta))     [d(omega)/dt = M / I]
so a positive gimbal angle pitches the nose toward -x. Aerodynamic loads come
from ``Vehicle.aero`` (component build-up valid for nose-first and
tail-first flow).
"""

from __future__ import annotations

import math
import random
from dataclasses import dataclass, field

import numpy as np

from .guidance import Commands, FlightComputer, GuidanceConfig, Phase, Sensors
from .vehicle import G0, Vehicle, air_density


# Touchdown limits the landing gear is designed for (docs/07-landing-legs.md):
# 3-5 cm of damped leg stroke handles 2.5 m/s; a 1.0 m tip-to-tip footprint
# resists tip-over up to ~1.7 m/s lateral (we use 1.5 m/s).
TD_LIMITS = {"vz": 2.5, "vx": 1.5, "tilt_deg": 10.0}


@dataclass
class Dispersions:
    """Truth-model errors the flight computer does not know about."""
    ascent_impulse_scale: float = 1.0
    ascent_time_scale: float = 1.0
    landing_impulse_scale: float = 1.0
    landing_time_scale: float = 1.0
    landing_ignition_delay_s: float = 0.30
    throttle_min_true: float = 0.25       # real paddle authority
    throttle_gain_true: float = 1.0       # real thrust per commanded throttle
    cd_scale: float = 1.0
    dry_mass_scale: float = 1.0
    wind_mps: float = 2.0                 # +x wind at 10 m
    gust_mps: float = 0.8                 # random-walk gust intensity
    launch_tilt_deg: float = 0.0
    thrust_misalign_deg: float = 0.0
    alt_noise_m: float = 0.4              # baro-fused altitude error (1 sigma)
    alt_bias_m: float = 0.0
    vel_noise_mps: float = 0.25
    gyro_bias_dps: float = 0.0
    landing_fails_to_ignite: bool = False
    seed: int = 0


@dataclass
class Result:
    outcome: str
    t: np.ndarray
    log: dict
    events: list
    touchdown: dict = field(default_factory=dict)
    apogee_m: float = 0.0
    liftoff_mass_kg: float = 0.0


def simulate(veh: Vehicle, gcfg: GuidanceConfig, disp: Dispersions | None = None,
             dt: float = 0.002, ctrl_dt: float = 0.005, t_max: float = 40.0,
             log_every: int = 5) -> Result:
    disp = disp or Dispersions()
    rng = random.Random(disp.seed)

    asc_true = veh.ascent_motor.scaled(disp.ascent_impulse_scale, disp.ascent_time_scale)
    land_true = veh.landing_motor.scaled(disp.landing_impulse_scale, disp.landing_time_scale)

    # Truth vehicle (dispersed dry mass) – flight computer keeps nominal values.
    true_dry = veh.dry_mass_kg * disp.dry_mass_scale
    tv = Vehicle(**{**veh.__dict__, "dry_mass_kg": true_dry,
                    "ascent_motor": asc_true, "landing_motor": land_true})

    m0, cg0, i0 = veh.mass_props(None, None)
    lever_nom = veh.gimbal_station_m - veh.mass_props(veh.ascent_motor.burn_time, None)[1]
    ca_fixed = veh.ca_tail_first + sum(c.ca for c in veh.components if not c.deployable)
    ca_dev = sum(c.ca for c in veh.components if c.deployable)
    gcfg.cda_burn_m2 = ca_fixed * veh.ref_area
    gcfg.cda_pre_m2 = (ca_fixed + ca_dev) * veh.ref_area
    gcfg.cda_legs_m2 = veh.leg_ca_deployed * veh.ref_area
    fc = FlightComputer(cfg=gcfg, landing_motor=veh.landing_motor,
                        ascent_motor=veh.ascent_motor, dry_mass_kg=veh.dry_mass_kg,
                        inertia_kgm2=i0, gimbal_lever_m=lever_nom,
                        landing_offset_m=veh.landing_offset_m,
                        ascent_offset_m=veh.ascent_offset_m)

    # ---------------------------------------------------------------- state
    th = math.radians(disp.launch_tilt_deg)
    om = 0.0
    x = 0.0
    z = veh.length_m - cg0          # CG height with aft end on the ground
    vx = vz = 0.0
    delta = 0.0
    throttle_act = 1.0
    t = 0.0
    t_asc = 0.0                     # ascent ignites at t = 0
    t_land_fire: float | None = None
    flaps_act = 0.0
    legs = False
    chute = False
    chute_open_alt = None
    on_ground = True
    touched = False
    td: dict = {}
    hop_max = 0.0
    gust = 0.0
    apogee = 0.0
    cmd = Commands()
    misalign = math.radians(disp.thrust_misalign_deg)
    next_ctrl = 0.0
    keys = ["x", "z_feet", "vx", "vz", "theta_deg", "omega_dps", "gimbal_deg",
            "throttle", "thrust", "mass", "phase", "flaps", "aoa_deg", "q_pa"]
    log = {k: [] for k in keys}
    tlog = []
    step = 0
    outcome = "timeout"

    while t < t_max:
        # ------------------------------------------------ truth mass & thrust
        tb_land = None
        if t_land_fire is not None and not disp.landing_fails_to_ignite:
            tb_land = t - t_land_fire - disp.landing_ignition_delay_s
            if tb_land < 0:
                tb_land = None
        m, cg, inertia = tv.mass_props(t_asc if t >= 0 else None, tb_land)
        lever = veh.gimbal_station_m - cg
        t_asc_thrust = asc_true.thrust(t_asc)
        t_land_full = land_true.thrust(tb_land) if tb_land is not None else 0.0
        k_eff = max(disp.throttle_min_true, throttle_act) * disp.throttle_gain_true
        k_eff = min(1.0, k_eff)
        thrust_a = t_asc_thrust
        thrust_l = k_eff * t_land_full
        thrust = thrust_a + thrust_l
        offset = (veh.ascent_offset_m * thrust_a + veh.landing_offset_m * thrust_l) / thrust \
            if thrust > 0 else 0.0

        # ------------------------------------------------------- feet height
        tail_below_cg = (veh.length_m - cg) * math.cos(th)
        z_feet = z - tail_below_cg - (veh.leg_drop_m if legs else 0.0)

        # --------------------------------------------------------------- aero
        h = max(z_feet, 0.0)
        rho = air_density(h)
        wind = disp.wind_mps * (max(h, 1.0) / 10.0) ** (1 / 7) + gust
        vrx, vrz = vx - wind, vz
        bx, bz = math.sin(th), math.cos(th)
        nx, nz = math.cos(th), -math.sin(th)
        v_ax = vrx * bx + vrz * bz
        v_n = vrx * nx + vrz * nz
        fa, fn, ma = tv.aero(v_ax, v_n, rho, cg, flaps=flaps_act, legs=legs)
        fa *= disp.cd_scale
        fx_aero = fa * bx + fn * nx
        fz_aero = fa * bz + fn * nz
        if chute:
            # Backup parachute: 0.9 m dia, Cd 1.5, ~0.6 s to inflate.
            sp = math.hypot(vrx, vrz)
            if chute_open_alt is None:
                chute_open_alt = (t, z_feet)
            infl = min(1.0, (t - chute_open_alt[0]) / 0.6)
            dch = 0.5 * rho * sp * 1.5 * math.pi * 0.45 ** 2 * infl
            fx_aero -= dch * vrx
            fz_aero -= dch * vrz
            ma *= 0.2

        # ------------------------------------------------------ thrust forces
        ang = th + delta + misalign
        fx_t = thrust * math.sin(ang)
        fz_t = thrust * math.cos(ang)
        m_thrust = -(lever * thrust * math.sin(delta + misalign)
                     + offset * thrust * math.cos(delta + misalign))

        ax = (fx_t + fx_aero) / m
        az = (fz_t + fz_aero) / m - G0
        alpha = (m_thrust + ma) / inertia

        # ---------------------------------------------------- ground contact
        if on_ground:
            if az <= 0.0 and z_feet <= 0.001:
                ax = az = alpha = 0.0
                vx = vz = om = 0.0
            else:
                on_ground = False

        # ---------------------------------------------- flight computer (FC)
        if t >= next_ctrl:
            next_ctrl += ctrl_dt
            f_ax = ((fx_t + fx_aero) * bx + (fz_t + fz_aero) * bz) / m
            if on_ground and not touched:
                f_ax = max(f_ax, G0)  # on the pad the IMU reads +1 g
            sens = Sensors(
                t=t,
                h_feet=z_feet + disp.alt_bias_m + rng.gauss(0, disp.alt_noise_m
                                                         if z_feet > 10 else 0.05),
                vz=vz + rng.gauss(0, disp.vel_noise_mps),
                vx=vx + rng.gauss(0, disp.vel_noise_mps),
                theta=th + math.radians(disp.gyro_bias_dps) * t,
                omega=om + math.radians(disp.gyro_bias_dps),
                f_axial=f_ax + rng.gauss(0, 0.05 * G0),
                rho=rho,
            )
            asc_est = veh.ascent_motor.thrust(t)   # FC uses the nominal curve
            cmd = fc.step(sens, ctrl_dt, asc_est)
            if cmd.fire_landing and t_land_fire is None:
                t_land_fire = t
            chute = chute or cmd.deploy_chute
            legs = legs or cmd.deploy_legs
            gust += rng.gauss(0, disp.gust_mps) * math.sqrt(ctrl_dt) - 0.5 * gust * ctrl_dt

        # ------------------------------------------------------- actuators
        lim = math.radians(veh.gimbal_limit_deg)
        dcmd = max(-lim, min(lim, cmd.gimbal_rad))
        rate = (dcmd - delta) / veh.servo_tau_s
        rmax = math.radians(veh.servo_rate_dps)
        delta += max(-rmax, min(rmax, rate)) * dt
        throttle_act += max(-6.0, min(6.0, (cmd.throttle - throttle_act) / 0.04)) * dt
        flaps_act += max(-3.0, min(3.0, (cmd.flaps - flaps_act) / 0.1)) * dt

        # ------------------------------------------------------- integrate
        vx += ax * dt
        vz += az * dt
        om += alpha * dt
        x += vx * dt
        z += vz * dt
        th += om * dt
        t += dt
        t_asc = t
        apogee = max(apogee, z_feet)

        # ------------------------------------------------------- touchdown
        tail_below_cg = (veh.length_m - cg) * math.cos(th)
        z_feet = z - tail_below_cg - (veh.leg_drop_m if legs else 0.0)
        if not on_ground and t > 1.0 and z_feet <= 0.0:
            if not touched:
                touched = True
                td = dict(vz=vz, vx=vx, theta_deg=math.degrees(th),
                          omega_dps=math.degrees(om), legs=legs, t=t,
                          burn_clock=(tb_land if tb_land is not None else -1.0),
                          burn_left=(land_true.burn_time - tb_land)
                          if tb_land is not None else 0.0,
                          chute=chute, phase=fc.phase.name)
                fc.phase = Phase.TOUCHDOWN if fc.phase == Phase.LANDING_BURN else fc.phase
            z += -z_feet
            vz = max(vz, 0.0)
            vx *= 0.0
            om = 0.0
            on_ground = True
        if touched and not on_ground:
            hop_max = max(hop_max, z_feet)

        # --------------------------------------------------------------- log
        if step % log_every == 0:
            tlog.append(t)
            sp = math.hypot(vrx, vrz)
            log["x"].append(x)
            log["z_feet"].append(z_feet)
            log["vx"].append(vx)
            log["vz"].append(vz)
            log["theta_deg"].append(math.degrees(th))
            log["omega_dps"].append(math.degrees(om))
            log["gimbal_deg"].append(math.degrees(delta))
            log["throttle"].append(k_eff if thrust_l > 0 else float("nan"))
            log["thrust"].append(thrust)
            log["mass"].append(m)
            log["phase"].append(fc.phase.value)
            log["flaps"].append(flaps_act)
            log["aoa_deg"].append(math.degrees(math.atan2(abs(v_n), v_ax)) if sp > 1 else 0.0)
            log["q_pa"].append(0.5 * rho * sp * sp)
        step += 1

        # --------------------------------------------------------- end cond
        if touched:
            burn_done = (tb_land is None and t_land_fire is None) or \
                (tb_land is not None and tb_land > land_true.burn_time + 0.3) or \
                chute or disp.landing_fails_to_ignite
            if burn_done or t - td["t"] > 4.0:
                break

    # ----------------------------------------------------------- classify
    if not touched:
        outcome = "timeout"
    elif chute:
        outcome = "chute_abort"
    else:
        ok_v = abs(td["vz"]) <= TD_LIMITS["vz"]
        ok_h = abs(td["vx"]) <= TD_LIMITS["vx"]
        ok_t = abs(td["theta_deg"]) <= TD_LIMITS["tilt_deg"]
        ok_hop = hop_max <= 0.3
        if ok_v and ok_h and ok_t and ok_hop and td["legs"]:
            outcome = "landed"
        elif abs(td["vz"]) <= 5.0 and ok_t and td["legs"]:
            outcome = "hard_landing"
        else:
            outcome = "crash"
    td["hop_max"] = hop_max
    td["chute_open"] = chute_open_alt
    return Result(outcome=outcome, t=np.array(tlog),
                  log={k: np.array(v) for k, v in log.items()},
                  events=fc.events, touchdown=td, apogee_m=apogee,
                  liftoff_mass_kg=m0)
