"""Flight-computer logic: state machine, landing-burn ignition predictor,
throttle re-planning, and TVC attitude control.

This module mirrors ``firmware/src/core`` line-for-line in intent so the two
can be cross-checked: the simulator feeds it noisy "sensor" values and it
returns actuator commands, exactly like the real board would.

Landing-burn strategy (the important part)
------------------------------------------
A solid motor cannot be shut down, and our only thrust knob is a pair of
exhaust paddles that can *waste* some thrust (throttle k in [k_min, 1]).
So the flight computer:

1. Falls tail-first under a small drogue at a near-constant terminal speed.
2. Computes the touchdown window [t1, t2] on the burn clock: from the moment
   closed paddles can no longer lift the vehicle off its legs
   (k_min * T < W) to the moment thrust is too weak to arrest the descent.
3. Every cycle predicts the state at ignition (coasting the nominal ignition
   delay) and solves for the throttle ramp k(s) = k0 + k1*s that lands at
   mid-window. Far from the ground the plan is "fall, then brake hard"
   (k1 > 0); it FIRES when k1 <= 0, i.e. when a constant-throttle burn
   would land mid-window - the middle of the feasible ignition band.
4. After ignition it re-solves the ramp 50 times a second over a sweep of
   touchdown times inside the window and commands k0. Late ignition or a
   weak motor -> k rises toward 1; early ignition or a strong motor -> k
   falls toward k_min. The drogue stays attached (stability) and is cut at
   touchdown.
5. Below ``terminal_alt_m`` it switches to a constant-deceleration braking
   law, and on touchdown closes the paddles and holds attitude until burnout.

Conventions (planar): z up, x horizontal, theta = body tilt from vertical
(+ toward +x), omega = d(theta)/dt. A positive gimbal angle pitches the nose
toward -x (derivation in sim.py).
"""

from __future__ import annotations

import math
from dataclasses import dataclass, field
from enum import Enum

import numpy as np

from .motors import Motor

G0 = 9.80665


class Phase(Enum):
    PAD = 0
    BOOST = 1
    COAST = 2
    DESCENT = 3
    LANDING_BURN = 4
    TOUCHDOWN = 5
    ABORT_CHUTE = 6


@dataclass
class GuidanceConfig:
    # --- landing-burn ignition predictor
    ignition_delay_nominal_s: float = 0.30   # fire-command -> thrust (MEASURE IT)
    fire_k1: float = 0.0                     # fire when the plan's throttle ramp slope <= this
    fire_window_frac: float = 0.65          # where in the touchdown window the fire plan aims (0..1)
    throttle_min: float = 0.25               # paddles fully closed: DESIGN REQUIREMENT, measure it
    target_touchdown_speed: float = 0.5      # m/s, downward
    predictor_dt: float = 0.01
    replan_dt: float = 0.02
    predictor_gate_alt_m: float = 120.0      # only run predictor below this
    touchdown_window_s: tuple[float, float] | None = None  # burn-clock window; None = auto
    window_hop_margin: float = 1.0           # k_min*T must be < margin*W at window start
    window_end_thrust_ratio: float = 1.25    # window ends when T < ratio*W
    plan_samples: int = 40
    plan_candidates: int = 9
    cda_pre_m2: float = 0.0                  # descent CdA with drogue (set from vehicle)
    cda_burn_m2: float = 0.0                 # CdA after the drogue is released
    cda_legs_m2: float = 0.0                 # extra CdA once the legs are out
    release_drag_device_on_ignition: bool = False  # keep the top airbrakes out through the burn
    cut_drag_device_on_touchdown: bool = False     # True only for a drogue on a leash
    # --- burn shape: 'exp' = strong braking that decays to ~weight at touchdown,
    #                 'ramp' = linear throttle ramp (the original planner)
    burn_profile: str = 'exp'
    exp_tau_fracs: tuple[float, ...] = (0.25, 0.35, 0.5)   # decay time / time-to-go
    exp_end_weight: float = 2.0      # cost on (net accel at touchdown)/g, wants ~0 = hover
    exp_terminal_alt_m: float = 0.8
    # --- abort logic
    ignition_detect_timeout_s: float = 0.90  # no thrust by then -> chute
    abort_tilt_deg: float = 35.0
    # --- powered descent
    terminal_alt_m: float = 2.0              # below this: constant-decel braking law
    terminal_kv: float = 3.0
    lateral_kv: float = 1.5                  # thrust-tilt command per (m/s / g)
    lateral_tilt_max_deg: float = 12.0
    lateral_fade_alt_m: float = 2.5          # bring tilt command to 0 below this
    # --- attitude controller (dynamic inversion PID)
    att_wn: float = 12.0                     # rad/s
    att_zeta: float = 0.85
    att_ki: float = 4.0
    gimbal_limit_deg: float = 7.0
    ascent_lateral_kv: float = 0.8
    ascent_tilt_max_deg: float = 4.0
    # --- mechanisms
    leg_deploy_alt_m: float = 12.0
    drogue_deploy_on_apogee: bool = True     # deploys the top airbrakes (or a drogue)


@dataclass
class Sensors:
    """What the flight computer believes (estimator output + raw accel)."""
    t: float
    h_feet: float          # altitude of the feet/aft end above ground
    vz: float
    vx: float
    theta: float
    omega: float
    f_axial: float         # measured specific force along body axis (m/s^2)
    rho: float = 1.225


@dataclass
class Commands:
    gimbal_rad: float = 0.0
    throttle: float = 1.0
    fire_landing: bool = False
    deploy_legs: bool = False
    flaps: float = 0.0           # drogue / drag-device deployment (0 or 1)
    deploy_chute: bool = False


@dataclass
class FlightComputer:
    cfg: GuidanceConfig
    landing_motor: Motor
    ascent_motor: Motor
    dry_mass_kg: float
    inertia_kgm2: float
    gimbal_lever_m: float      # gimbal pivot -> CG
    landing_offset_m: float = 0.0
    ascent_offset_m: float = 0.0

    phase: Phase = Phase.PAD
    t_phase: float = 0.0
    t_fire_cmd: float | None = None
    t_ignition_detected: float | None = None
    f_axial_at_fire: float = 0.0
    thrust_scale: float = 1.0
    integ: float = 0.0
    flaps: float = 0.0
    legs: bool = False
    chute: bool = False
    throttle: float = 1.0
    max_h: float = 0.0
    _neg_vz_count: int = 0
    _low_acc_count: int = 0
    _next_replan: float = 0.0
    _t_land: float | None = None
    _terminal: bool = False
    _window: tuple[float, float] | None = None
    last_plan: tuple = ()
    events: list[tuple[float, str]] = field(default_factory=list)

    # ----------------------------------------------------------------- utils
    def _log(self, t: float, msg: str) -> None:
        self.events.append((t, msg))

    def _set_phase(self, t: float, p: Phase) -> None:
        self.phase = p
        self.t_phase = t
        self._log(t, p.name)

    def landing_mass(self, t_burn: float | None) -> float:
        m_l = self.landing_motor.mass(t_burn) if t_burn is not None else self.landing_motor.total_kg
        return self.dry_mass_kg + self.ascent_motor.casing_kg + m_l

    # ------------------------------------------- throttle-ramp landing plan
    def landing_window(self) -> tuple[float, float]:
        """Burn-clock window in which touchdown is allowed.

        Starts when closed paddles (k_min) can no longer lift the vehicle off
        its legs (no post-touchdown hop) and ends when the remaining thrust is
        too weak to arrest the last bit of descent."""
        if self.cfg.touchdown_window_s is not None:
            return self.cfg.touchdown_window_s
        if self._window is None:
            c = self.cfg
            tb = np.linspace(0.05, self.landing_motor.burn_time, 400)
            thrust = self.landing_motor.thrust_v(tb)
            w = (self.dry_mass_kg + self.ascent_motor.casing_kg
                 + self.landing_motor.mass_v(tb)) * G0
            ok_hop = c.throttle_min * thrust < c.window_hop_margin * w
            ok_end = thrust > c.window_end_thrust_ratio * w
            t1 = float(tb[np.argmax(ok_hop)]) if ok_hop.any() else float(tb[-1])
            t2 = float(tb[len(tb) - 1 - np.argmax(ok_end[::-1])]) if ok_end.any() else t1
            self._window = (t1, max(t1, t2))
        return self._window

    def plan(self, h: float, v: float, rho: float, tb_now: float, t_land: float,
             cos_tilt: float = 1.0, cda: float | None = None) -> tuple[float, float] | None:
        """Throttle ramp k(s) = k0 + k1*s that reaches (h=0, v=-v_td) at burn
        clock ``t_land``. The vertical dynamics are linear in k, so this is a
        2x2 linear solve over integrals of the known thrust curve."""
        c = self.cfg
        tau = t_land - tb_now
        if tau < 0.05:
            return None
        cda = c.cda_burn_m2 if cda is None else cda
        s_ = np.linspace(0.0, tau, c.plan_samples)
        tb = tb_now + s_
        m = self.dry_mass_kg + self.ascent_motor.casing_kg + self.landing_motor.mass_v(tb)
        a = self.thrust_scale * self.landing_motor.thrust_v(tb) * cos_tilt / m
        v_td = -c.target_touchdown_speed
        v_lin = v + (v_td - v) * s_ / tau              # drag along a nominal profile
        d = 0.5 * rho * cda * v_lin ** 2 / m
        trap = np.trapezoid
        a0, a1 = trap(a, s_), trap(s_ * a, s_)
        b0, b1 = trap((tau - s_) * a, s_), trap(s_ * (tau - s_) * a, s_)
        dv, dh = trap(d, s_), trap((tau - s_) * d, s_)
        rhs_v = v_td - v + G0 * tau - dv
        rhs_h = -h - v * tau + 0.5 * G0 * tau ** 2 - dh
        det = a0 * b1 - a1 * b0
        if abs(det) < 1e-9:
            return None
        k0 = (rhs_v * b1 - a1 * rhs_h) / det
        k1 = (a0 * rhs_h - b0 * rhs_v) / det
        return k0, k1

    def best_plan(self, h: float, v: float, rho: float, tb_now: float,
                  cos_tilt: float = 1.0, cda: float | None = None):
        """Search the touchdown window for the most feasible throttle ramp.
        Returns (t_land, k0, k1, violation)."""
        c = self.cfg
        t1, t2 = self.landing_window()
        lo = max(t1, tb_now + 0.15)
        hi = max(lo, t2)
        best = None
        pref = self._t_land if self._t_land is not None else 0.5 * (t1 + t2)
        for tl in np.linspace(lo, hi, c.plan_candidates):
            p = self.plan(h, v, rho, tb_now, float(tl), cos_tilt, cda)
            if p is None:
                continue
            k0, k1 = p
            ke = k0 + k1 * (tl - tb_now)
            viol = max(0.0, c.throttle_min - min(k0, ke)) + max(0.0, max(k0, ke) - 1.0)
            cost = 10.0 * viol + 0.05 * abs(tl - pref)
            if best is None or cost < best[0]:
                best = (cost, float(tl), k0, k1, viol)
        if best is None:
            return None
        return best[1:]

    def plan_exp(self, h: float, v: float, rho: float, tb_now: float, t_land: float,
                 tau_frac: float, cos_tilt: float = 1.0, cda: float | None = None):
        """'Strong then simmer' burn: net upward acceleration
        a(s) = A*exp(-s/tau) + C, solved so that h = 0 and v = -v_td at
        t_land. Returns (A, C, tau, k_profile, a_end) or None. The throttle
        that produces this acceleration is k = m*(a + g - drag)/thrust."""
        c = self.cfg
        T = t_land - tb_now
        if T < 0.05:
            return None
        cda = c.cda_burn_m2 if cda is None else cda
        tau = max(0.05, tau_frac * T)
        E = math.exp(-T / tau)
        v_td = -c.target_touchdown_speed
        a11, a12, r1 = tau * (1.0 - E), T, v_td - v
        a21, a22, r2 = tau * (T - tau * (1.0 - E)), 0.5 * T * T, -h - v * T
        det = a11 * a22 - a12 * a21
        if abs(det) < 1e-9:
            return None
        A = (r1 * a22 - a12 * r2) / det
        C = (a11 * r2 - a21 * r1) / det
        s_ = np.linspace(0.0, T, c.plan_samples)
        e = np.exp(-s_ / tau)
        a_net = A * e + C
        v_s = v + A * tau * (1.0 - e) + C * s_
        tb = tb_now + s_
        m = self.dry_mass_kg + self.ascent_motor.casing_kg + self.landing_motor.mass_v(tb)
        drag_up = -0.5 * rho * cda * v_s * np.abs(v_s) / m
        force = m * (a_net + G0 - drag_up)
        avail = self.thrust_scale * self.landing_motor.thrust_v(tb) * cos_tilt
        k = np.where(avail > 1.0, force / np.maximum(avail, 1e-6), 9.0)
        return A, C, tau, k, A * E + C

    def best_plan_exp(self, h: float, v: float, rho: float, tb_now: float,
                      cos_tilt: float = 1.0, cda: float | None = None):
        """Search touchdown time and decay rate for the most feasible
        'strong then simmer' burn. Returns (t_land, k_now, violation, a_end)."""
        c = self.cfg
        t1, t2 = self.landing_window()
        lo = max(t1, tb_now + 0.15)
        hi = max(lo, t2)
        pref = self._t_land if self._t_land is not None else 0.5 * (t1 + t2)
        best = None
        for tl in np.linspace(lo, hi, c.plan_candidates):
            for f in c.exp_tau_fracs:
                p = self.plan_exp(h, v, rho, tb_now, float(tl), f, cos_tilt, cda)
                if p is None:
                    continue
                _, _, _, k, a_end = p
                viol = max(0.0, c.throttle_min - float(k.min())) + max(0.0, float(k.max()) - 1.0)
                cost = 10.0 * viol + c.exp_end_weight * abs(a_end) / G0 + 0.05 * abs(tl - pref)
                if best is None or cost < best[0]:
                    best = (cost, float(tl), float(k[0]), viol, a_end)
        return None if best is None else best[1:]

    def fire_now(self, h: float, v: float, rho: float) -> bool:
        """Decide whether to send the ignition command now (see below)."""
        c = self.cfg
        dt = c.predictor_dt
        m = self.landing_mass(None)
        for _ in range(int(round(c.ignition_delay_nominal_s / dt))):
            a = -0.5 * rho * c.cda_pre_m2 * v * abs(v) / m - G0
            v += a * dt
            h += v * dt
        if h <= 0.0:
            return True
        t1, t2 = self.landing_window()
        tl = t1 + c.fire_window_frac * (t2 - t1)
        cda = c.cda_burn_m2 if c.release_drag_device_on_ignition else c.cda_pre_m2
        p = self.plan(h, v, rho, 0.0, tl, cda=cda)
        if p is None:
            return True
        k0, k1 = p
        self.last_plan = (h, v, tl, k0, k1)
        # Far from the ground the plan is "fall now, brake hard later"
        # (k1 >> 0). As we fall, k1 decreases monotonically; when it reaches
        # fire_k1 (0 = a constant-throttle burn lands mid-window) we fire.
        # That point sits in the middle of the feasible ignition band, so
        # ignition-delay scatter either way is absorbed by the paddles.
        return k1 <= c.fire_k1

    # ------------------------------------------------------ attitude control
    def _trim(self, offset_m: float) -> float:
        """Steady gimbal angle that puts an offset thrust line through the CG."""
        return math.asin(max(-0.5, min(0.5, -offset_m / self.gimbal_lever_m)))

    def _gimbal_for(self, theta_cmd: float, s: Sensors, thrust_est: float,
                    offset_m: float, dt: float) -> float:
        c = self.cfg
        if thrust_est < 1.0:
            return 0.0
        kp = c.att_wn ** 2
        kd = 2.0 * c.att_zeta * c.att_wn
        err = theta_cmd - s.theta
        alpha_des = kp * err - kd * s.omega + c.att_ki * self.integ
        # theta_ddot = -(L*T*sin(delta) + T*offset*cos(delta)) / I  (see sim.py)
        lever = self.gimbal_lever_m
        x = -(self.inertia_kgm2 * alpha_des) / (thrust_est * lever) - offset_m / lever
        lim = math.sin(math.radians(c.gimbal_limit_deg))
        sat = abs(x) >= lim
        x = max(-lim, min(lim, x))
        if not sat:                       # conditional-integration anti-windup
            self.integ += err * dt
        return math.asin(x)

    # ----------------------------------------------------------------- step
    def step(self, s: Sensors, dt: float, ascent_thrust_est: float) -> Commands:
        c = self.cfg
        cmd = Commands()
        self.max_h = max(self.max_h, s.h_feet)

        if self.phase == Phase.PAD:
            if s.f_axial > 2.0 * G0:
                self._set_phase(s.t, Phase.BOOST)

        if self.phase == Phase.BOOST:
            lim = math.radians(c.ascent_tilt_max_deg)
            theta_cmd = max(-lim, min(lim, -c.ascent_lateral_kv * s.vx / G0))
            theta_cmd -= self._trim(self.ascent_offset_m)
            cmd.gimbal_rad = self._gimbal_for(theta_cmd, s, ascent_thrust_est,
                                              self.ascent_offset_m, dt)
            if s.t - self.t_phase > 0.3 and s.f_axial < 0.0:
                self._low_acc_count += 1
                if self._low_acc_count >= 5:
                    self._set_phase(s.t, Phase.COAST)
                    self.integ = 0.0
            else:
                self._low_acc_count = 0

        elif self.phase == Phase.COAST:
            if s.vz < -0.5:
                self._neg_vz_count += 1
                if self._neg_vz_count >= 10:
                    self._set_phase(s.t, Phase.DESCENT)
                    if c.drogue_deploy_on_apogee:
                        self.flaps = 1.0
                        self._log(s.t, "AIRBRAKES")
            else:
                self._neg_vz_count = 0

        elif self.phase == Phase.DESCENT:
            if abs(s.theta) > math.radians(c.abort_tilt_deg) and s.h_feet < 0.7 * self.max_h:
                self._log(s.t, "ABORT: tilt")
                self.chute = True
                self._set_phase(s.t, Phase.ABORT_CHUTE)
            elif s.h_feet < c.predictor_gate_alt_m and self.fire_now(s.h_feet, s.vz, s.rho):
                self.t_fire_cmd = s.t
                self.f_axial_at_fire = s.f_axial
                cmd.fire_landing = True
                self._log(s.t, f"FIRE h={s.h_feet:.1f} vz={s.vz:.1f}")
                self._set_phase(s.t, Phase.LANDING_BURN)

        elif self.phase == Phase.LANDING_BURN:
            cmd.fire_landing = True
            if self.t_ignition_detected is None:
                self.throttle = 1.0
                if s.f_axial > self.f_axial_at_fire + 0.8 * G0:
                    self.t_ignition_detected = s.t
                    self.integ = 0.0
                    self._log(s.t, "IGNITION")
                    if c.release_drag_device_on_ignition and self.flaps > 0.0:
                        self.flaps = 0.0
                        self._log(s.t, "DROGUE RELEASE")
                elif s.t - self.t_fire_cmd > c.ignition_detect_timeout_s:
                    self._log(s.t, "ABORT: no ignition")
                    self.chute = True
                    self._set_phase(s.t, Phase.ABORT_CHUTE)
            if self.t_ignition_detected is not None:
                self._landing_guidance(s, dt, cmd)

        elif self.phase == Phase.TOUCHDOWN:
            # Paddles closed; keep holding attitude while the motor tails off
            # so a small post-touchdown hop cannot tip the vehicle over.
            self.throttle = c.throttle_min
            if self.t_ignition_detected is not None:
                tb = s.t - self.t_ignition_detected + 0.02
                t_est = c.throttle_min * self.thrust_scale * self.landing_motor.thrust(tb)
                cmd.gimbal_rad = self._gimbal_for(0.0, s, t_est, self.landing_offset_m, dt)

        if self.phase == Phase.LANDING_BURN and s.h_feet <= c.leg_deploy_alt_m and not self.legs:
            self.legs = True
            self._log(s.t, "LEGS")

        cmd.flaps = self.flaps
        cmd.deploy_legs = self.legs
        cmd.deploy_chute = self.chute
        cmd.throttle = self.throttle
        return cmd

    # ---------------------------------------------------- powered descent
    def _landing_guidance(self, s: Sensors, dt: float, cmd: Commands) -> None:
        c = self.cfg
        tb = s.t - self.t_ignition_detected + 0.02   # lit slightly before detection
        m = self.landing_mass(tb)
        cda = (c.cda_burn_m2 if self.flaps == 0.0 else c.cda_pre_m2) + \
            (c.cda_legs_m2 if self.legs else 0.0)
        nominal = self.landing_motor.thrust(tb)
        # Thrust actually delivered, from the accelerometer (drag removed).
        thrust_meas = max(0.0, s.f_axial * m - 0.5 * s.rho * cda * s.vz ** 2)
        if nominal > 5.0 and tb > 0.15:
            ratio = thrust_meas / (max(self.throttle, c.throttle_min) * nominal)
            ratio = max(0.7, min(1.3, ratio))
            self.thrust_scale += 0.05 * (ratio - self.thrust_scale)

        if s.h_feet <= 0.05:
            self._set_phase(s.t, Phase.TOUCHDOWN)
            self.throttle = c.throttle_min
            if self.flaps > 0.0 and c.cut_drag_device_on_touchdown:
                self.flaps = 0.0          # cut a drogue so ground wind can't drag us over
                self._log(s.t, "DROGUE RELEASE")
            return

        cos_t = max(0.7, math.cos(s.theta))
        exp_mode = c.burn_profile == "exp"
        term_alt = c.exp_terminal_alt_m if exp_mode else c.terminal_alt_m
        if s.h_feet > term_alt and not self._terminal:
            if s.t >= self._next_replan:
                self._next_replan = s.t + c.replan_dt
                if exp_mode:
                    bp = self.best_plan_exp(s.h_feet, s.vz, s.rho, tb, cos_t, cda)
                    if bp is None:
                        self._terminal = True
                    else:
                        self._t_land, k_now, viol, a_end = bp
                        self.last_plan = bp
                        self.throttle = max(c.throttle_min, min(1.0, k_now))
                else:
                    bp = self.best_plan(s.h_feet, s.vz, s.rho, tb, cos_t, cda)
                    if bp is None:
                        self._terminal = True
                    else:
                        self._t_land, k0, k1, viol = bp
                        self.last_plan = bp
                        self.throttle = max(c.throttle_min, min(1.0, k0))
        else:
            # Constant-deceleration braking to v_td exactly at h = 0, plus a
            # small rate term so we don't hang above the pad.
            v_td = c.target_touchdown_speed
            h_eff = max(s.h_feet, 0.05)
            brake = max(0.0, (s.vz * s.vz - v_td * v_td) / (2.0 * h_eff)) if s.vz < -v_td else 0.0
            a_req = G0 + brake + c.terminal_kv * (-v_td - s.vz) * (0.0 if brake > 0 else 1.0)
            a_req -= -0.5 * s.rho * cda * s.vz * abs(s.vz) / m
            avail = self.thrust_scale * nominal * cos_t
            k = a_req * m / avail if avail > 1.0 else 1.0
            self.throttle = max(c.throttle_min, min(1.0, k))

        # Lateral: tilt to null horizontal velocity; fade to upright at the end.
        # The lateral loop commands the THRUST direction; the body must lean the
        # other way by the gimbal trim that keeps the offset thrust line
        # through the CG, otherwise that trim pushes us sideways.
        tilt_lim = math.radians(c.lateral_tilt_max_deg)
        phi_cmd = max(-tilt_lim, min(tilt_lim, -c.lateral_kv * s.vx / G0))
        phi_cmd *= max(0.0, min(1.0, (s.h_feet - 0.3) / c.lateral_fade_alt_m))
        theta_cmd = phi_cmd - self._trim(self.landing_offset_m)
        thrust_now = self.throttle * self.thrust_scale * nominal
        cmd.gimbal_rad = self._gimbal_for(theta_cmd, s, thrust_now,
                                          self.landing_offset_m, dt)
