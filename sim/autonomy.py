"""Autonomy harness: mode state machine and an estimate-driven closed loop.

Conventions (assumed where the estimator spec is silent):
- est.heading is the aircraft heading psi in radians, same convention as the state vector
  (math angle from east, counter-clockwise).
- est.wind_dir_est_deg is the compass direction the wind blows FROM (as in sim.seawind).
- Energies are specific, in J/kg: E = g z + V^2 / 2.
"""
import json
import os
import time
from dataclasses import dataclass
from enum import Enum

import numpy as np

from sim.aircraft import BASELINE, G, clip_controls, derivatives, rk4_step

try:
    from sim.seawind import Environment, WaveTrain, Wind
except ImportError:
    from sim._seawind_stub import Environment, Wind
    WaveTrain = None

from sim.estimator import Estimate, Estimator
from sim.sensors import SensorSuite, truth_from_state


class Mode(Enum):
    LAUNCH = "launch"
    SOAR = "soar"
    LOITER = "loiter"
    TASK = "task"
    LAND = "land"
    FLOAT = "float"
    SLEEP = "sleep"
    COMMS = "comms"
    TAKEOFF = "takeoff"
    SAFE = "safe"


FLYING = (Mode.SOAR, Mode.LOITER, Mode.TASK, Mode.TAKEOFF)


@dataclass
class Thresholds:
    shear_min_ustar: float = 0.25   # m/s
    t_hold: float = 60.0            # s
    energy_reserve: float = 10.0    # J/kg above the minimum soaring energy
    takeoff_u10: float = 8.0        # m/s
    float_hs_max: float = 4.0       # m
    battery_low: float = 0.2
    link_window_s: float = 300.0    # s
    task_radius_m: float = 200.0    # m


def wrap(a):
    return (a + np.pi) % (2 * np.pi) - np.pi


def wind_to_psi(est):
    """Math angle (from east, ccw) of the direction the wind blows toward."""
    to_compass = np.radians(est.wind_dir_est_deg + 180.0)
    return float(np.pi / 2 - to_compass)


def u10_from_ustar(ustar, z0=2e-4):
    return float(ustar / 0.4 * np.log(10.0 / z0))


class ModeMachine:
    def __init__(self, aircraft=BASELINE, thresholds=None, mode=Mode.LAUNCH, h_min=1.0):
        self.ac = aircraft
        self.th = thresholds or Thresholds()
        self.mode = mode
        self.log = []  # (t, from, to, reason)
        # minimum soaring energy: clearance height at 1.1 v_stall
        self.e_min = G * h_min + 0.5 * (1.1 * aircraft.v_stall) ** 2
        self._since = {}
        self._entered = 0.0
        self._last_comms_exit = -1e9

    def energy(self, est):
        return G * float(est.z) + 0.5 * float(est.airspeed) ** 2

    def margin(self, est):
        return self.energy(est) - self.e_min

    def _held(self, key, cond, t):
        """Seconds that cond has been continuously true (0 when false)."""
        if not cond:
            self._since.pop(key, None)
            return 0.0
        return t - self._since.setdefault(key, t)

    def _go(self, new, t, reason):
        if new is Mode.FLOAT and self.mode is Mode.COMMS:
            self._last_comms_exit = t
        self.log.append((t, self.mode, new, reason))
        self.mode = new
        self._entered = t
        self._since.clear()
        return new

    def step(self, est, flight, t):
        th, m, fl = self.th, self.mode, flight
        usable = bool(est.valid) and est.ustar_est >= th.shear_min_ustar
        shear_low = self._held("shear", bool(est.valid) and est.ustar_est < th.shear_min_ustar, t)
        margin = self.margin(est)
        en_low = self._held("energy", margin < th.energy_reserve, t)
        dist = fl.get("distance_to_target_m", np.inf)

        if m in FLYING and fl.get("fault", False):
            return self._go(Mode.SAFE, t, "fault")

        if m is Mode.LAUNCH:
            if fl.get("on_water") and not fl.get("airborne"):
                return self._go(Mode.FLOAT, t, "launch failed on water")
            if fl.get("airborne") and usable:
                return self._go(Mode.SOAR, t, "airborne and shear usable")
        elif m in (Mode.SOAR, Mode.LOITER):
            if shear_low >= th.t_hold:
                return self._go(Mode.LAND, t, "shear below threshold for t_hold")
            if en_low >= th.t_hold:
                return self._go(Mode.LAND, t, "energy below reserve for t_hold")
            if m is Mode.SOAR and dist <= th.task_radius_m:
                return self._go(Mode.LOITER, t, "at target")
            if m is Mode.LOITER:
                if fl.get("task_done") or dist > 1.5 * th.task_radius_m:
                    return self._go(Mode.SOAR, t, "task done" if fl.get("task_done")
                                    else "drifted off station")
                if margin >= th.energy_reserve and dist <= th.task_radius_m:
                    return self._go(Mode.TASK, t, "on station and energy margin")
        elif m is Mode.TASK:
            if fl.get("task_done"):
                return self._go(Mode.LOITER, t, "data captured")
        elif m is Mode.LAND:
            if fl.get("on_water"):
                return self._go(Mode.FLOAT, t, "touchdown")
        elif m is Mode.FLOAT:
            batt = fl.get("battery", 1.0)
            if batt < th.battery_low or fl.get("night"):
                return self._go(Mode.SLEEP, t, "battery low or night")
            if fl.get("link_available") and t - self._last_comms_exit >= th.link_window_s:
                return self._go(Mode.COMMS, t, "link window")
            if fl.get("wind_u10_est", 0.0) > th.takeoff_u10 and \
                    fl.get("hs_est", np.inf) < th.float_hs_max:
                return self._go(Mode.TAKEOFF, t, "wind above takeoff threshold, sea state allowed")
        elif m is Mode.SLEEP:
            if fl.get("wake") or fl.get("battery", 0.0) >= th.battery_low + 0.1:
                return self._go(Mode.FLOAT, t, "wake timer or battery recovered")
        elif m is Mode.COMMS:
            if fl.get("comms_done"):
                return self._go(Mode.FLOAT, t, "sent")
            if not fl.get("link_available") or t - self._entered >= th.link_window_s:
                return self._go(Mode.FLOAT, t, "window closed")
        elif m is Mode.TAKEOFF:
            if fl.get("airborne"):
                return self._go(Mode.SOAR, t, "airborne")
            if fl.get("takeoff_failed"):
                return self._go(Mode.FLOAT, t, "take-off failed")
        elif m is Mode.SAFE:
            if fl.get("airborne"):
                return self._go(Mode.LAND, t, "still flying")
            if fl.get("on_water"):
                return self._go(Mode.FLOAT, t, "on water")
        return self.mode


class Phase(Enum):
    CLIMB = 0    # upwind climb, wings level
    TURN = 1     # turn from upwind to downwind
    DESC = 2     # downwind descent, wings level
    RETURN = 3   # S-turn (opposite bank) back to upwind


HAND_PICKED = dict(mu_turn=1.0, gamma_climb=0.35, gamma_desc=0.35, cl_turn=0.9,
                   h_top=12.0, h_bot=2.0, turn_sign=1)


class EstimatedDSController:
    """Four-phase Rayleigh dynamic-soaring cycle driven only by the estimate.

    In CLIMB and DESC the lift coefficient holds a flight path angle (+gamma_climb, -gamma_desc):
    cl = clip(cl_level + 2 (gamma_target - gamma_est)), gamma_est = asin(vz_ground / airspeed).

    Phases: CLIMB (upwind, bank 0) until h >= h_top or V <= 1.15 v_stall; TURN to downwind at
    bank turn_sign*mu_turn; DESC (downwind, bank 0) until h <= h_bot; RETURN (S-turn, opposite
    bank) back to upwind. Heading is judged against the estimated wind direction, h from
    est.h_surface, V from est.airspeed. In the straight phases a weak heading hold steers to the
    reference (wind direction plus the waypoint bias), which is how the bias acts.
    """

    def __init__(self, aircraft, cycle=None, clearance_min=1.0, noise_k=2.0,
                 bias_max=np.radians(30.0), k_hold=0.5, hold_max=0.15,
                 turn_done=np.radians(25.0)):
        self.ac = aircraft
        self.cycle = cycle if cycle is not None else {"p": dict(HAND_PICKED)}
        self.p = dict(self.cycle["p"])
        self.clearance_min = clearance_min
        self.noise_k = noise_k
        self.bias_max = bias_max
        self.k_hold = k_hold
        self.hold_max = hold_max
        self.turn_done = turn_done
        self.reset()

    def reset(self, phase=Phase.CLIMB):
        self.phase = phase
        self.cycle_starts = []   # times at which a CLIMB begins
        self.guard_count = 0
        self.guard_active = False
        self._h_hist = []
        self._d2 = None

    def noise_est(self):
        """Std of the h_surface estimate from its second differences (var of d2 = 6 sigma^2)."""
        return 0.0 if self._d2 is None else float(np.sqrt(self._d2 / 6.0))

    def _update_noise(self, h):
        self._h_hist = (self._h_hist + [h])[-3:]
        if len(self._h_hist) == 3:
            d2 = (self._h_hist[2] - 2 * self._h_hist[1] + self._h_hist[0]) ** 2
            self._d2 = d2 if self._d2 is None else 0.99 * self._d2 + 0.01 * d2

    def bias(self, est, target_bearing, energy_margin):
        """Rotation of the cycle's mean heading toward the target, at most bias_max."""
        if target_bearing is None or energy_margin <= 0:
            return 0.0
        return float(np.clip(wrap(target_bearing - wind_to_psi(est)),
                             -self.bias_max, self.bias_max))

    def command(self, est, t, target_bearing=None, energy_margin=0.0):
        p, ac = self.p, self.ac
        h, V, hd = float(est.h_surface), float(est.airspeed), float(est.heading)
        self._update_noise(h)
        guard = h < self.clearance_min + self.noise_k * self.noise_est()
        if guard and not self.guard_active:
            self.guard_count += 1
        self.guard_active = guard

        psi_w = wind_to_psi(est)
        b = self.bias(est, target_bearing, energy_margin)
        down, up = psi_w + b, psi_w + np.pi + b
        if not self.cycle_starts:
            self.cycle_starts.append(t)
        ph = self.phase
        if ph is Phase.CLIMB and (h >= p["h_top"] or V <= 1.15 * ac.v_stall):
            ph = Phase.TURN
        elif ph is Phase.TURN and abs(wrap(hd - down)) < self.turn_done:
            ph = Phase.DESC
        elif ph is Phase.DESC and h <= p["h_bot"]:
            ph = Phase.RETURN
        elif ph is Phase.RETURN and abs(wrap(hd - up)) < self.turn_done:
            ph = Phase.CLIMB
            self.cycle_starts.append(t)
        self.phase = ph

        if guard:
            return ac.cl_best_ld, 0.0
        sgn = p["turn_sign"]
        if ph in (Phase.CLIMB, Phase.DESC):
            gam = float(np.arcsin(np.clip(float(est.v_ground[2]) / max(V, 1e-3), -1.0, 1.0)))
            target = p["gamma_climb"] if ph is Phase.CLIMB else -p["gamma_desc"]
            cl_level = (2 * ac.mass * G * np.cos(gam)
                        / (ac.rho * max(V, 1.0) ** 2 * ac.wing_area))
            cl = float(np.clip(cl_level + 2.0 * (target - gam), ac.cl_min, ac.cl_max))
            return cl, self._hold(up if ph is Phase.CLIMB else down, hd)
        mu = sgn * p["mu_turn"] * (1 if ph is Phase.TURN else -1)
        return p["cl_turn"], mu

    def _hold(self, ref, hd):
        return float(np.clip(self.k_hold * wrap(ref - hd), -self.hold_max, self.hold_max))

    def land_command(self, est):
        """Straight glide into the estimated wind at 1.2 v_stall, wings held to the heading."""
        err = wrap(wind_to_psi(est) + np.pi - float(est.heading))
        return self.ac.cl_max / 1.44, float(np.clip(0.8 * err, -0.3, 0.3))


def _suite_names(suite):
    for a in ("names", "sensor_names"):
        if hasattr(suite, a):
            return list(getattr(suite, a))
    return [type(suite).__name__]


def initial_state(aircraft, env, cycle, h0=3.0):
    psi_w = np.arctan2(env.wind.ey, env.wind.ex)
    return [0.0, 0.0, h0 + float(env.surface(0.0, 0.0, 0.0)[0]), cycle["V0"], 0.0,
            psi_w + np.pi]  # start heading upwind, in the CLIMB phase


def fly(aircraft, env, suite, est, ctrl, s0, t_end, dt=0.02, seed=1, target=None,
        thresholds=None, initial_mode=Mode.SOAR):
    """Closed loop on estimated states. `est` is the Estimator; the dynamics are true.

    seed is accepted for the interface; the noise is seeded by the suite itself.
    The machine starts in `initial_mode` (SOAR) because LAUNCH is not flown.
    """
    ac = aircraft
    machine = ModeMachine(ac, thresholds, mode=initial_mode)
    s = np.asarray(s0, dtype=float)
    p0 = s[:2].copy()
    tgt = None if target is None else np.asarray(target, float)
    keys = ("t", "state", "est_pos", "est_wind", "est_h", "est_heading", "ustar_est", "mode",
            "cl", "mu", "n", "h", "e_air", "wind_err", "h_err", "heading_err")
    rec = {k: [] for k in keys}
    ended = "complete"
    prev = None
    nsteps = int(round(t_end / dt))
    for i in range(nsteps + 1):
        t = i * dt
        truth = truth_from_state(s, env, t, None if prev is None else prev[2], prev)
        e = est.update(suite.measure(truth, t), t)
        h_true = s[2] - float(env.surface(s[0], s[1], t)[0])
        epos = np.asarray(e.pos, float)
        dist = np.inf if tgt is None else float(np.linalg.norm(tgt - epos[:2]))
        u10 = u10_from_ustar(e.ustar_est)
        flight = {"airborne": h_true > 0, "on_water": h_true <= 0, "fault": False,
                  "battery": 1.0, "distance_to_target_m": dist, "link_available": False,
                  "wind_u10_est": u10, "hs_est": 0.21 * u10 ** 2 / G}
        mode = machine.step(e, flight, t)
        if mode is Mode.FLOAT and machine.log and machine.log[-1][1] is Mode.LAND:
            ended = "touchdown"
            break
        if mode not in (Mode.SOAR, Mode.LAND):
            ended = "mode " + mode.name
            break
        if h_true <= 0:
            ended = "crash"
            break
        margin = machine.margin(e)
        if mode is Mode.SOAR:
            bearing = None if tgt is None else float(np.arctan2(*(tgt - epos[:2])[::-1]))
            cl, mu = ctrl.command(e, t, bearing, margin)
        else:
            cl, mu = ctrl.land_command(e)
        cl, mu = clip_controls(ac, cl, mu)
        w_true = np.asarray(env.wind_at(s[0], s[1], s[2], t), float).reshape(3)
        q = 0.5 * ac.rho * s[3] ** 2 * ac.wing_area
        ew = np.asarray(e.wind, float)
        vals = dict(t=t, state=s.copy(), est_pos=epos, est_wind=ew, est_h=float(e.h_surface),
                    est_heading=float(e.heading), ustar_est=float(e.ustar_est), mode=mode.name,
                    cl=cl, mu=mu, n=q * cl / (ac.mass * G), h=h_true,
                    e_air=s[2] + s[3] ** 2 / (2 * G),
                    wind_err=float(np.linalg.norm(ew[:2] - w_true[:2])),
                    h_err=float(e.h_surface) - h_true,
                    heading_err=float(wrap(float(e.heading) - s[5])))
        for k, v in vals.items():
            rec[k].append(v)
        if i == nsteps:
            break
        prev = (s.copy(), t, mu)
        s = rk4_step(lambda tt, ss: derivatives(tt, ss, ac, env, cl, mu)[0], t, s, dt)
        if not np.all(np.isfinite(s)):
            ended = "non-finite state"
            break

    out = {k: (list(v) if k == "mode" else np.array(v)) for k, v in rec.items()}
    out["ended"] = ended
    out["transitions"] = [(a, b.name, c.name, d) for a, b, c, d in machine.log]
    out["metrics"] = _metrics(ctrl, out, p0, tgt)
    return out


def _metrics(ctrl, r, p0, tgt):
    m = {"ended": r["ended"], "n_steps": int(len(r["t"])), "transitions": r["transitions"],
         "guard_activations": int(ctrl.guard_count)}
    if len(r["t"]) < 2:
        return m
    t, st = r["t"], r["state"]
    starts = ctrl.cycle_starts  # completed cycles run between consecutive CLIMB starts
    ncyc = max(len(starts) - 1, 0)
    m["cycles"] = ncyc
    m["energy_gain_per_cycle"] = None
    if ncyc >= 1:
        i0 = min(int(np.searchsorted(t, starts[0] - 1e-9)), len(t) - 1)
        i1 = min(int(np.searchsorted(t, starts[-1] - 1e-9)), len(t) - 1)
        m["energy_gain_per_cycle"] = float(G * (r["e_air"][i1] - r["e_air"][i0]) / ncyc)
    m["min_h"] = float(np.min(r["h"]))
    m["max_n"] = float(np.max(r["n"]))
    span = t[-1] - t[0]
    m["ground_speed_toward_target"] = None
    if tgt is not None:
        u = (tgt - p0) / np.linalg.norm(tgt - p0)
        m["ground_speed_toward_target"] = float((st[-1, :2] - st[0, :2]) @ u / span)
    m["wind_rms"] = float(np.sqrt(np.mean(r["wind_err"] ** 2)))
    m["h_rms"] = float(np.sqrt(np.mean(r["h_err"] ** 2)))
    m["heading_rms"] = float(np.sqrt(np.mean(r["heading_err"] ** 2)))
    m["duration_s"] = float(span)
    return m


def default_cycle(aircraft, u10):
    """Cycle from sim/out/formstudy.json if it holds a four-phase p dict, else hand-picked."""
    path = os.path.join(os.path.dirname(__file__), "out", "formstudy.json")
    if os.path.exists(path):
        try:
            with open(path) as fh:
                d = json.load(fh)
            d = d.get(aircraft.name, d)
            if isinstance(d.get("p"), dict) and "mu_turn" in d["p"] and "V0" in d:
                return {"p": dict(d["p"]), "V0": float(d["V0"]), "source": "formstudy.json"}
        except (ValueError, AttributeError):
            pass
    return {"p": dict(HAND_PICKED), "V0": 2.0 * aircraft.v_stall, "source": "hand-picked"}


def compare_suites(aircraft, u10=10.0, t_end=120.0, cycle=None, seed=1, target_range=3000.0):
    cycle = cycle or default_cycle(aircraft, u10)
    wind = Wind(u10, 270.0)
    envs = {"flat": Environment.flat(wind)}
    if WaveTrain is not None:
        envs["waved"] = Environment(wind, [WaveTrain.fully_developed(wind)], seed=1)
    res = {}
    for ename, env in envs.items():
        psi_w = np.arctan2(env.wind.ey, env.wind.ex)
        target = target_range * np.array([np.cos(psi_w + np.radians(20)),
                                          np.sin(psi_w + np.radians(20))])
        for sname in ("minimal", "standard", "full"):
            suite = getattr(SensorSuite, sname)(seed)
            estimator = Estimator(_suite_names(suite))
            ctrl = EstimatedDSController(aircraft, cycle)
            r = fly(aircraft, env, suite, estimator, ctrl, initial_state(aircraft, env, cycle),
                    t_end, seed=seed, target=target)
            res["%s/%s" % (sname, ename)] = r["metrics"]
    return {"cycle": {"p": dict(cycle["p"]), "V0": cycle["V0"],
                      "source": cycle.get("source", "given")},
            "u10": u10, "t_end": t_end, "results": res}


def _f(v, fmt):
    return "n/a" if v is None else fmt % v


def main():
    t0 = time.time()
    out = compare_suites(BASELINE)
    print("cycle source: %s, p = %s, V0 = %.2f" % (
        out["cycle"]["source"], out["cycle"]["p"], out["cycle"]["V0"]))
    print("| suite | env | gain/cycle (J/kg) | min h (m) | max n | v toward target (m/s) | "
          "wind rms | h rms | guard | ended |")
    print("|---|---|---|---|---|---|---|---|---|---|")
    for key, m in out["results"].items():
        s, e = key.split("/")
        print("| %s | %s | %s | %s | %s | %s | %s | %s | %d | %s |" % (
            s, e, _f(m.get("energy_gain_per_cycle"), "%.1f"), _f(m.get("min_h"), "%.2f"),
            _f(m.get("max_n"), "%.2f"), _f(m.get("ground_speed_toward_target"), "%.2f"),
            _f(m.get("wind_rms"), "%.2f"), _f(m.get("h_rms"), "%.2f"),
            m["guard_activations"], m["ended"]))
    out["runtime_s"] = time.time() - t0
    path = os.path.join(os.path.dirname(__file__), "out")
    os.makedirs(path, exist_ok=True)
    with open(os.path.join(path, "autonomy.json"), "w", newline="\n") as fh:
        json.dump(out, fh, indent=1)
    print("runtime %.0f s" % out["runtime_s"])


if __name__ == "__main__":
    main()
