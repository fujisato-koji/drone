"""Flight over long-crested swell with the receding-horizon planner (Q-060).

Stage 1: the planner knows the sea exactly (oracle sea); the sea is one long-crested regular swell
component (amplitude Hs / 2, deep water, period Tp), the model of the cycle optimizer's wave mode
(sim.dsopt), with the wave-coherent wind of the P63 calibration or the P64 low bound
(sim.dsopt.wave_preset_params) and the COARE 3.5 log profile over the local surface.

Frames: the library frame of sim.dsopt and sim.mpc (the mean wind blows toward +x, angles counter-clockwise
from +x). A sea is named by the swell's direction of travel relative to the wind: "along" (0 deg), "across"
(90 deg, toward +y in the library frame) and "against" (180 deg). A course is named relative to the swell's
direction of travel (0 = with the swell).
"""
from __future__ import annotations

import numpy as np

from sim.aircraft import G
from sim.seawind import Environment, Wind

SWELL_DIRS = {"along": 0.0, "across": 90.0, "against": 180.0}   # library frame, deg from the wind's direction
PRESETS = ("p63", "p64_low")
CALM_U10 = 0.1      # m/s; "calm" is the wave mode's lower bound of u10 (sim.dsopt.U10_FLOOR_WAVE)


def wavelength(tp):
    """Deep-water wavelength (m) of period tp (s)."""
    return G * tp * tp / (2.0 * np.pi)


class RegularSwell:
    """A wave train with one long-crested deep-water component (WaveTrain interface for Environment):
    amplitude hs / 2, period tp, travelling toward the compass direction to_deg, phase at the origin."""

    def __init__(self, hs, tp, to_deg, phase=0.0, name="swell"):
        self.hs, self.tp, self.to_deg = float(hs), float(tp), float(to_deg)
        self.phase, self.name = float(phase), name
        self.gamma, self.hs_full = None, 0.0

    @property
    def growth_fraction(self):
        return 1.0

    def components(self, rng):
        om = 2.0 * np.pi / self.tp
        r = np.radians(self.to_deg)
        return {"a": np.array([0.5 * self.hs]), "k": np.array([om * om / G]), "omega": np.array([om]),
                "dx": np.array([np.sin(r)]), "dy": np.array([np.cos(r)]), "phase": np.array([self.phase])}


def preset_amp_decay(preset):
    """((u amplitude, w amplitude), (a_u, a_w)) of a wave-wind preset of the cycle optimizer."""
    from sim.dsopt import wave_preset_params
    p = wave_preset_params(preset)
    return (p["u_scale"], p["w_scale"]), tuple(p["decay"])


def swell_env(u10, hs, tp, direction, preset, from_deg=270.0, phase=0.0, surface_model="stokes2"):
    """The oracle sea of the experiments: COARE 3.5 log wind of u10 (at least CALM_U10) from from_deg over one
    regular swell component (Hs hs, period tp) travelling `direction` ("along", "across", "against" or an angle
    in deg, counter-clockwise from the wind's direction), with the wave-coherent wind of `preset`; the surface
    for contact and heights is second order (surface_model "stokes2", as docs/02-form.md#simulator-fitness)."""
    wind = Wind(max(float(u10), CALM_U10), from_deg, roughness="coare35")
    rel = SWELL_DIRS[direction] if isinstance(direction, str) else float(direction)
    amp, decay = preset_amp_decay(preset)
    trains = [] if hs <= 0.0 else [RegularSwell(hs, tp, (wind.to_deg - rel) % 360.0, phase)]
    return Environment(wind, trains, seed=1, decay=decay, wave_amp=amp, surface_model=surface_model)


# ---------------------------------------------------------------------------------------------
# step 1: why the earlier wave runs ended early (python -m sim.mpc --wave-diagnosis)
# ---------------------------------------------------------------------------------------------

DIAG_SWELL = (("albatross-3m", 6.0), ("albatross-3m", CALM_U10))   # the flat-library planner over swell
DIAG_SWELL_T_END = 300.0


class PlanLog:
    """plan_hook of fly_mpc: per plan the time, the dead share of the samples, the lowest predicted wingtip
    height of the nominal plan in the planner's world and against the true surface, and the reference height."""

    def __init__(self, ac, env_true):
        self.ac, self.env, self.rows = ac, env_true, []

    def __call__(self, t, plan, world, s, guide):
        ac = self.ac
        P = plan.path
        tt = t + plan.times
        mu = np.interp(plan.times, plan.knots_t, plan.mu_knots)
        tip_off = 0.5 * ac.span * np.sin(np.abs(mu)) * np.cos(P[:, 4])
        eta_w = np.asarray(world.surface(P[:, 0], P[:, 1], tt)[0])
        eta_t = np.asarray(self.env.surface(P[:, 0], P[:, 1], tt)[0])
        h_ref = None
        if guide is not None:
            h_ref = float(guide.cycle.state(guide.tau_s)[0])
        self.rows.append({"t": float(t), "n_dead": int(plan.n_dead), "tip_world_min": float(np.min(P[:, 2] - eta_w - tip_off)),
                          "tip_true_min": float(np.min(P[:, 2] - eta_t - tip_off)), "h_ref": h_ref})


def end_state(ac, env, out):
    """Where and how a kept run ended: the last state against the true surface and the sea there."""
    tr = out["traj"]
    t, st, mu, cl = tr["t"], tr["state"], tr["mu"], tr["cl"]
    i = len(t) - 1
    x, y, z, V, g, psi = st[i]
    eta, ex, ey = (float(v) for v in env.surface(x, y, t[i]))
    eta_next = float(env.surface(x, y, t[i] + 0.05)[0])
    tip = z - eta - 0.5 * ac.span * np.sin(abs(mu[i])) * np.cos(g)
    q = 0.5 * ac.rho * V * V * ac.wing_area
    th = tr.get("thrust")
    # sea statistics at the end point over one minute of time
    ts = t[i] - np.linspace(0.0, 60.0, 241)
    etas = np.asarray(env.surface(np.full(ts.shape, x), np.full(ts.shape, y), ts)[0])
    j = max(0, i - int(round(10.0 / (t[1] - t[0]))))
    hh = st[j:i + 1, 2] - np.asarray(env.surface(st[j:i + 1, 0], st[j:i + 1, 1], t[j:i + 1])[0])
    return {"t_s": float(t[i]), "h_m": float(z - eta), "tip_m": float(tip), "bank_deg": float(np.degrees(mu[i])),
            "v_over_stall": float(V / ac.v_stall), "gamma_deg": float(np.degrees(g)),
            "load_factor": float(q * cl[i] / (ac.mass * G)),
            "thrust_frac_weight": None if th is None else float(th[i] / (ac.mass * G)),
            "eta_m": eta, "eta_over_std": float(eta / max(np.std(etas), 1e-9)),
            "surface_rising": bool(eta_next > eta), "slope": float(np.hypot(ex, ey)),
            "h_min_last10_m": float(hh.min()), "h_mean_last10_m": float(hh.mean())}


def diag_job(job):
    """One diagnosis run: (kind, aircraft, u10, planner world, reflex, wave_aware, t_end). kind "wind_sea": the
    motor library experiment's wave case rerun as it was (sim.mpc.run_motor_job settings) with the plans logged;
    kind "swell": the same planner (flat-sea motor library, truth world) over the regular 3 m, 10 s swell along
    the wind with the P63 wave wind."""
    import time
    from sim.aircraft import Actuators
    from sim.aircraft_models import by_name
    from sim import mpc
    kind, name, u10, wm, reflex, wave_aware, t_end = job
    ac = by_name(name)
    env = mpc._make_env_coare("waves", u10) if kind == "wind_sea" else swell_env(u10, 3.0, 10.0, "along", "p63")
    log = PlanLog(ac, env)
    t0 = time.time()
    r = mpc.fly_mpc(ac, env, wm, [], t_end, env.wind.to_deg - 20.0, max(u10, CALM_U10), seed=1,
                    planner_kw=dict(mpc.MOTOR_EXP_PKW), library=mpc.CycleLibrary.from_keyword("motor"),
                    state_source="truth", wind_source="truth",
                    max_components=mpc.BURST_MAX_COMPONENTS if wm == "truth" else None,
                    actuators=Actuators(ac, rate_model="pb2v"), n_limit=True, stop_on_tip=True,
                    wave_aware=wave_aware, motor=True, w_energy=mpc.energy_weight(ac, "normal"), reflex=reflex,
                    keep=True, plan_hook=log)
    es = end_state(ac, env, r)
    rows = log.rows
    last = [p for p in rows if p["t"] >= es["t_s"] - 5.0]
    tr = r.pop("traj")
    eta = np.asarray(env.surface(tr["state"][:, 0], tr["state"][:, 1], tr["t"])[0])
    h = tr["state"][:, 2] - eta
    out = {"kind": kind, "aircraft": name, "u10": u10, "planner_world": wm, "reflex": "on" if reflex else "off",
           "wave_aware": bool(wave_aware), "t_end": t_end, "ended": r["ended"], "survival_s": r["survival_s"],
           "wall_s": time.time() - t0, "end": es, "sea_std_m": float(np.std(eta)),
           "h_median_m": float(np.median(h)), "h_min_m": float(h.min()),
           "motor": {k: r["motor"][k] for k in ("avg_power_w", "thrust_on_share", "mean_thrust_frac_weight")},
           "reflex_n": (r.get("reflex") or {}).get("n"),
           "plans_last5s": {"n": len(last),
                            "tip_world_min_m": min((p["tip_world_min"] for p in last), default=None),
                            "tip_true_min_m": min((p["tip_true_min"] for p in last), default=None),
                            "n_dead_mean": float(np.mean([p["n_dead"] for p in last])) if last else None},
           "plans_all": {"share_true_contact_predicted_world_clear": float(np.mean(
               [(p["tip_true_min"] < 0.0) and (p["tip_world_min"] >= 0.0) for p in rows])) if rows else None,
               "h_ref_median_m": float(np.median([p["h_ref"] for p in rows if p["h_ref"] is not None]))
               if rows else None}}
    return out


def diagnosis_jobs(results):
    """The wave runs of the motor library experiment that ended early, and the flat-library planner over swell."""
    jobs = [("wind_sea", r["aircraft"], r["u10"], r["planner_world"], r["reflex"] == "on", False, 180.0)
            for r in results if r["sea"] == "waves" and r["ended"] != "complete"]
    jobs += [("swell", n, u, "truth", False, wa, DIAG_SWELL_T_END) for n, u in DIAG_SWELL for wa in (False, True)]
    return jobs


def _dump(path, key, block):
    import json
    import os
    d = json.load(open(path)) if os.path.exists(path) else {}
    d[key] = block
    with open(path, "w", newline=chr(10)) as fh:
        json.dump(d, fh, indent=1)
        fh.write(chr(10))


def diagnosis_main(out, workers):
    """python -m sim.mpc --wave-diagnosis: key wave_diagnosis of sim/out/mpc.json."""
    import json
    import os
    import time
    from concurrent.futures import ProcessPoolExecutor
    from sim import record
    from sim.mpc import _no_nan
    here = os.path.dirname(__file__)
    res = json.load(open(out))["motor_library_exp"]["results"]
    jobs = diagnosis_jobs(res)
    jobs.sort(key=lambda j: -(j[3] == "truth"))
    t0 = time.time()
    with ProcessPoolExecutor(workers) as ex:
        rows = list(ex.map(diag_job, jobs, chunksize=1))
    for r in rows:
        if r["kind"] == "wind_sea":
            old = [x for x in res if (x["aircraft"], x["u10"], x["sea"], x["planner_world"], x["reflex"])
                   == (r["aircraft"], r["u10"], "waves", r["planner_world"], r["reflex"])][0]
            r["recorded_survival_s"] = old["survival_s"]
            r["recorded_ended"] = old["ended"]
    meta = record.meta("python -m sim.mpc --wave-diagnosis --workers %d" % workers,
                       [os.path.join(here, "mpc.py"), __file__, os.path.join(here, "out", "mpc_library_motor.json"),
                        os.path.join(here, "seawind.py")])
    meta.update({"runtime_s": time.time() - t0, "workers": workers,
                 "wind_sea": "the wave runs of motor_library_exp that ended early, rerun with its settings "
                             "(run_motor_job: fully developed COARE 3.5 sea of seed 1, motor library, wave_aware False, "
                             "180 s) with every plan logged (PlanLog) and the trajectory kept",
                 "swell": "the same planner (flat-sea motor library picked at 20 deg off downwind, truth world) "
                          "over one regular swell component, Hs 3 m, Tp 10 s, along the wind, P63 wave wind, "
                          "second-order surface, %g s, wave_aware False and True" % DIAG_SWELL_T_END,
                 "plans_last5s": "the plans of the last 5 s: lowest wingtip of the nominal plan in the planner's "
                                 "world and against the true surface, mean number of dead samples (of 96)",
                 "end": "state at the last step against the true surface; eta_over_std is the surface elevation "
                        "there in units of its standard deviation over the last minute at that point"})
    _dump(out, "wave_diagnosis", _no_nan({"meta": meta, "results": rows}))
    for r in rows:
        e = r["end"]
        print(r["kind"], r["aircraft"], r["u10"], r["planner_world"], r["reflex"], r["wave_aware"], r["ended"],
              "%.0f" % r["survival_s"], "rec", r.get("recorded_survival_s"), "h %.2f tip %.2f bank %.0f V/vs %.2f g %.0f n %.2f"
              % (e["h_m"], e["tip_m"], e["bank_deg"], e["v_over_stall"], e["gamma_deg"], e["load_factor"]),
              "eta/std %.2f rising %s" % (e["eta_over_std"], e["surface_rising"]), "plans", r["plans_last5s"],
              "P %.1f on %.2f hmed %.2f" % (r["motor"]["avg_power_w"], r["motor"]["thrust_on_share"], r["h_median_m"]))


# ---------------------------------------------------------------------------------------------
# step 2: the wave-cycle library (python -m sim.mpc --build-wave-library)
# ---------------------------------------------------------------------------------------------

import os as _os

WAVE_LIB_PATH = _os.path.join(_os.path.dirname(__file__), "out", "mpc_library_wave.json")
WAVE_AIRCRAFT = ("albatross-3m", "baseline-2m")
WAVE_HS = (3.0, 2.0, 1.5)
WAVE_TP = (10.0, 8.0, 12.0)
WAVE_U10 = (0.0, 3.0, 6.0, 9.0)          # 0 is flown and solved at CALM_U10
WAVE_COURSES = (0.0, 90.0, 180.0)        # relative to the swell's travel; across seas also -90
WAVE_MARGINS = (2.0, 0.8, 0.9, 1.2)      # h_min, n_frac, mu_frac, v_factor: the library's standard set (mpc.MARGIN_SETS)
WAVE_SEEDS = (0, 1, 2)
WAVE_N = 40
WAVE_MAXITER = 150
CREST_H_MAX = 10.0                       # m; highest crest ride: above it the ride moves toward the crest (phase-stable side)


def wave_dict(hs, tp, direction):
    """The sim.dsopt wave dict (amplitude, wavelength, direction in the library frame) of a sea."""
    rel = SWELL_DIRS[direction] if isinstance(direction, str) else float(direction)
    return {"a": 0.5 * float(hs), "wavelength": wavelength(tp), "to_deg": rel}


def courses_for(direction):
    return WAVE_COURSES + ((-90.0,) if direction == "across" else ())


def _r(a):
    return [float(f"{v:.6g}") for v in np.asarray(a, float)]


def _entry_arrays(col, x):
    """Node arrays of a solved wave-mode cycle: z is the height above the LOCAL surface (as the flat library's
    z, so that mpc.Cycle and the guide read it the same way), z_abs the height above mean sea level, theta
    the wave phase at the aircraft."""
    S, C, T, u = col.unpack(x)
    wv = col.wv
    phi = col.phi0(x)
    th = col.node_theta(S, T, phi)
    h = S[:, 2] - wv.a * np.cos(th)
    t = np.linspace(0.0, T, col.N)
    return {"t": _r(t), "x": _r(S[:, 0]), "y": _r(S[:, 1]), "z": _r(h), "z_abs": _r(S[:, 2]), "V": _r(S[:, 3]),
            "gamma": _r(S[:, 4]), "psi": _r(S[:, 5]), "cl": _r(C[:, 0]), "mu": _r(C[:, 1]),
            "thrust": _r(col.thrust(x)), "theta": _r(th)}


def _solve_wave_cycle(ac, preset, hs, tp, direction, u10, course, motor, seeds=WAVE_SEEDS, n=WAVE_N):
    """One wave-mode solve of sim.dsopt: max_speed along the course (soaring, folded propeller over the whole
    cycle) or min_power with the motor (mean speed along the course at least 0.5 v_best_ld), the standard
    margins, COARE 3.5 at max(u10, CALM_U10), geometric wingtip, Hermite-Simpson midpoints, pb2v rates, n >= 0,
    the phase pinned on the wave, m searched over -1, 0, 1 around each start, dense replay check and mesh
    refinement. Returns (info, x, col)."""
    from sim.dsopt import profile_for, solve
    u = max(float(u10), CALM_U10)
    rel = SWELL_DIRS[direction]
    h_min, n_frac, mu_frac, v_factor = WAVE_MARGINS
    kw = dict(mode="min_power" if motor else "max_speed", cycle="travel", n=n, u10=u,
              chi=np.radians(rel + course), tip_model="geometric", wave=wave_dict(hs, tp, direction),
              wave_preset=preset, rate_limits=True, rate_model="pb2v", n_min=0.0, h_min=h_min, n_frac=n_frac,
              mu_frac=mu_frac, v_factor=v_factor, phase_pin="theta", m_search=(-1, 0, 1), replay_check=True,
              refine=True, seeds=seeds, stop_after=1, maxiter=WAVE_MAXITER, return_x=True)
    if motor:
        kw.update(motor=True, v_travel_min=0.5 * ac.v_best_ld)
    else:
        kw.update(folded_prop=True)
    return solve(ac, profile_for(u, "log", roughness="coare35"), **kw)


def solve_wave_entry(job):
    """One library slot (aircraft, preset, hs, tp, direction, u10, course relative to the swell): a soaring cycle,
    else (motor allowed only where soaring fails) a motor cycle. The entry holds the sea, the wave phase at the
    start (theta0 = phi0), the mean ground velocity (library frame), the average electric power (0 when
    soaring), the dense replay figures and the node arrays (see _entry_arrays)."""
    import time
    from sim.aircraft_models import by_name
    from sim.dsopt import control_stats, motor_stats
    name, preset, hs, tp, direction, u10, course = job
    ac = by_name(name)
    rel = SWELL_DIRS[direction]
    base = {"aircraft": name, "preset": preset, "hs": float(hs), "tp": float(tp), "direction": direction,
            "rel_deg": rel, "u10": float(u10), "u10_flown": max(float(u10), CALM_U10), "course_rel_deg": float(course),
            "chi_deg": float(rel + course), "crest": False}
    tried = []
    for motor in (False, True):
        t0 = time.time()
        try:
            info, x, col = _solve_wave_cycle(ac, preset, hs, tp, direction, u10, course, motor)
        except Exception as exc:     # a failed solve is a missing slot, not a lost build
            tried.append({"kind": "motor" if motor else "soaring", "success": False, "error": repr(exc),
                          "runtime_s": time.time() - t0})
            continue
        tried.append({"kind": "motor" if motor else "soaring", "success": bool(info["success"]),
                      "converged": bool(info.get("converged", False)), "violation": float(info["violation"]),
                      "replay_pass": info.get("replay_pass"), "m": info.get("m"), "runtime_s": time.time() - t0})
        if not info["success"]:
            continue
        S, C, T, u = col.unpack(x)
        xe, ye = float(S[-1, 0] - S[0, 0]), float(S[-1, 1] - S[0, 1])
        e = dict(base, success=True, kind="motor" if motor else "soaring", period=float(T), m=int(col.m),
                 phi0=float(col.phi0(x)), theta0=float(col.phi0(x)), n_nodes=int(col.N),
                 mean_ground_velocity=[xe / T, ye / T],
                 speed_along_chi=float(info["speed_along_chi"]),
                 margins={"h_min": col.h_min, "n_frac": col.n_frac, "mu_frac": col.mu_frac,
                          "v_factor": WAVE_MARGINS[3], "n_min": 0.0, "h_tip_min": col.h_tip,
                          "k_roll": col.k_roll, "cl_rate_max_per_s": col.r_max},
                 h_min_position=info.get("h_min_position"),
                 replay={k: info.get(k) for k in ("replay_pass", "rc_closure_h", "rc_closure_V_rel", "rc_h_min",
                                                  "rc_tip_min", "rc_n_max", "rc_n_min")},
                 control=control_stats(col, x), tried=tried)
        if motor:
            ms = motor_stats(col, x)
            e.update(power_avg_w=ms["power_avg_w"], power_avg_dense_w=ms["power_avg_dense_w"],
                     thrust_on_share=ms["thrust_on_share"])
        else:
            e.update(power_avg_w=0.0)
        e.update(_entry_arrays(col, x))
        return e
    return dict(base, success=False, missing=True, kind=None, tried=tried)


# ---- crest riding: steady flight in the wave frame on the forward face ---------------------------------------

CREST_PERIOD_S = 8.0       # period of the synthetic cycle of a crest ride (nothing in it depends on the phase)
CREST_VC_STEP = 0.5        # m/s; grid of the along-crest ground speed


def _crest_setup(ac, preset, hs, tp, direction, u10):
    from sim.dsopt import WaveParams, _apply_preset, folded_aircraft, profile_for
    wv = WaveParams(**_apply_preset(wave_dict(hs, tp, direction), preset))
    u = max(float(u10), CALM_U10)
    return folded_aircraft(ac), wv, profile_for(u, "log", roughness="coare35"), u


def crest_state(acf, wv, prof, u, theta, h, v_c):
    """Steady flight in the wave frame at wave phase theta, height h above the local surface, along-crest ground
    speed v_c (positive to the left of the swell's travel), wings level, no thrust. The ground velocity is
    c d + v_c e (d the swell's travel, e = d turned 90 deg to the left), so the phase and the height stay
    constant and the wind there is constant (no wind acceleration along the path). Returns (residual dV/dt in
    m/s2, V, gamma, psi, cl): the ride exists where the residual is zero (the updraft balances the sink)."""
    from sim.dsopt import _drag, wave_wind_accel
    S = np.array([[0.0, 0.0, h + wv.a * np.cos(theta), 10.0, 0.0, 0.0]])
    (Wx, Wy, Wz), _ = wave_wind_accel(S, wv, prof, np.zeros(1), u, theta)
    Wx, Wy, Wz = float(Wx[0]), float(Wy[0]), float(Wz[0])
    vgx = wv.c * wv.cw - v_c * wv.sw
    vgy = wv.c * wv.sw + v_c * wv.cw
    ax, ay, az = vgx - Wx, vgy - Wy, -Wz
    V = float(np.sqrt(ax * ax + ay * ay + az * az))
    g = float(np.arcsin(az / V))
    psi = float(np.arctan2(ay, ax))
    q = 0.5 * acf.rho * V * V * acf.wing_area
    cl = acf.mass * G * np.cos(g) / q
    D = float(_drag(acf, q, cl, 0.0, h, False, None))
    return -D / acf.mass - G * np.sin(g), V, g, psi, float(cl)


CREST_FAMILY_DH = 0.5      # m; height step of the family of crest rides


def crest_ride(ac, preset, hs, tp, direction, u10, v_c, h_min=None, h_max=CREST_H_MAX, family=True):
    """The crest ride at along-crest speed v_c, or None where none soars within the margins (airspeed at least
    the margin set's factor times the stall speed, lift coefficient below cl_max, h_min <= h <= h_max).

    The rides of one along-crest speed form a family over the height: at each height the phase on the crest side
    of the forward face (between the crest and the phase of the largest surplus at h_min) where the updraft
    balances the sink; the crest side is the phase-stable one (an aircraft that falls back towards the crest
    loses updraft, sinks and speeds up). The family runs from h_min up to h_max or to the balance height at the
    phase of the largest surplus, whichever is lower, in CREST_FAMILY_DH steps; the ride is its highest member.
    Returns a dict (the ride's state, and "family": rows [h, theta, V, gamma, psi, cl])."""
    h_min = WAVE_MARGINS[0] if h_min is None else h_min
    acf, wv, prof, u = _crest_setup(ac, preset, hs, tp, direction, u10)
    vmin = WAVE_MARGINS[3] * ac.v_stall

    def res(th, h):
        return crest_state(acf, wv, prof, u, th, h, v_c)

    ths = np.linspace(0.02, np.pi - 0.02, 79)
    rs = np.array([res(th, h_min)[0] for th in ths])
    i = int(np.argmax(rs))
    th_s = float(ths[i])
    if rs[i] < 0.0 or res(th_s, h_min)[1] < vmin:
        return None
    heights = list(np.arange(h_min, h_max + 1e-9, CREST_FAMILY_DH)) if family else [h_max]
    if res(th_s, h_max)[0] < 0.0:      # the balance height at the best phase lies below h_max: it ends the family
        lo, hi = h_min, h_max
        for _ in range(40):
            mid = 0.5 * (lo + hi)
            if res(th_s, mid)[0] >= 0.0:
                lo = mid
            else:
                hi = mid
        heights = [h for h in heights if h < lo] + [lo]
    rows = []
    for h in heights:
        if res(th_s, h)[0] < 0.0:
            continue
        lo, hi = 0.0, th_s            # bisection on the crest side: no updraft at the crest
        if res(1e-6, h)[0] >= 0.0:
            continue
        for _ in range(40):
            mid = 0.5 * (lo + hi)
            if res(mid, h)[0] >= 0.0:
                hi = mid
            else:
                lo = mid
        r, V, g, psi, cl = res(hi, h)
        if V >= vmin and cl <= ac.cl_max:
            rows.append([float(h), float(hi), V, g, psi, cl])
    if not rows:
        return None
    h, th, V, g, psi, cl = rows[-1]
    r = res(th, h)[0]
    return {"theta": th, "h": h, "V": V, "gamma": g, "psi": psi, "cl": cl, "residual": float(r),
            "v_c": float(v_c), "c": float(wv.c),
            "vg": [float(wv.c * wv.cw - v_c * wv.sw), float(wv.c * wv.sw + v_c * wv.cw)],
            "a": float(wv.a), "k": float(wv.k), "omega": float(wv.om), "family": rows}


def crest_exists(ac, preset, hs, tp, direction, u10, v_c, h_min=None):
    """Quick test whether a crest ride soars at v_c: a positive surplus at h_min at some phase of the forward face,
    with the airspeed margin."""
    h_min = WAVE_MARGINS[0] if h_min is None else h_min
    acf, wv, prof, u = _crest_setup(ac, preset, hs, tp, direction, u10)
    vmin = WAVE_MARGINS[3] * ac.v_stall
    for th in np.linspace(0.02, np.pi - 0.02, 79):
        r, V, g, psi, cl = crest_state(acf, wv, prof, u, th, h_min, v_c)
        if r >= 0.0 and V >= vmin and cl <= ac.cl_max:
            return True
    return False


def crest_replay(ac, preset, hs, tp, direction, u10, ride, duration=CREST_PERIOD_S, dt=0.02):
    """RK4 replay of a crest ride in the optimizer's wave dynamics (folded propeller, fixed controls): the drift
    of the height above the local surface, the airspeed and the phase after `duration`. The ride is an
    equilibrium, so the drift shows its exactness and its open-loop growth."""
    from sim.dsopt import derivs_wave
    acf, wv, prof, u = _crest_setup(ac, preset, hs, tp, direction, u10)
    th0 = ride["theta"]
    s = np.array([[0.0, 0.0, ride["h"] + wv.a * np.cos(th0), ride["V"], ride["gamma"], ride["psi"]]])
    c = np.array([[ride["cl"], 0.0]])

    def f(s_, t_):
        return derivs_wave(s_, c, acf, prof, wv, np.array([t_]), u, th0)

    t = 0.0
    for _ in range(int(round(duration / dt))):
        k1 = f(s, t)
        k2 = f(s + 0.5 * dt * k1, t + 0.5 * dt)
        k3 = f(s + 0.5 * dt * k2, t + 0.5 * dt)
        k4 = f(s + dt * k3, t + dt)
        s = s + dt / 6.0 * (k1 + 2 * k2 + 2 * k3 + k4)
        t += dt
    th = wv.k * (wv.cw * s[0, 0] + wv.sw * s[0, 1]) - wv.om * t + th0
    h = s[0, 2] - wv.a * np.cos(th)
    return {"duration_s": duration, "h_drift_m": float(h - ride["h"]), "V_drift_ms": float(s[0, 3] - ride["V"]),
            "theta_drift": float((th - th0 + np.pi) % (2 * np.pi) - np.pi)}


def motor_crest_ride(ac, preset, hs, tp, direction, u10, h=None):
    """Where no crest ride soars: the steady ride in the wave frame with the motor (propeller running, so not
    folded) at height h (default h_min) of the least electric power over the phase on the forward face and the
    along-crest speed (airspeed at least the margin factor times the stall speed). Returns a ride dict with
    the thrust (N) and power (W), or None."""
    from sim.dsopt import prop_efficiency, thrust_max
    h = WAVE_MARGINS[0] if h is None else h
    _, wv, prof, u = _crest_setup(ac, preset, hs, tp, direction, u10)
    vmin = WAVE_MARGINS[3] * ac.v_stall
    eta = prop_efficiency()
    best = None
    for v_c in np.arange(-30.0, 30.0 + 1e-9, CREST_VC_STEP):
        for th in np.linspace(0.02, np.pi - 0.02, 79):
            r, V, g, psi, cl = crest_state(ac, wv, prof, u, th, h, v_c)
            if V < vmin or cl > ac.cl_max:
                continue
            T = max(0.0, -ac.mass * r)
            P = T * V / eta
            if T <= thrust_max(ac) and (best is None or P < best[0] - 1e-9):
                best = (P, float(v_c), float(th), T)
    if best is None:
        return None
    P, v_c, th, T = best
    r, V, g, psi, cl = crest_state(ac, wv, prof, u, th, h, v_c)
    return {"theta": th, "h": float(h), "V": V, "gamma": g, "psi": psi, "cl": cl, "residual": float(r + T / ac.mass),
            "v_c": v_c, "c": float(wv.c), "thrust_n": T, "power_w": P,
            "vg": [float(wv.c * wv.cw - v_c * wv.sw), float(wv.c * wv.sw + v_c * wv.cw)],
            "a": float(wv.a), "k": float(wv.k), "omega": float(wv.om)}


def _crest_entry(name, preset, hs, tp, direction, u10, rd, kind, replay):
    rel = SWELL_DIRS[direction]
    n, P = 9, CREST_PERIOD_S
    t = [P * i / (n - 1) for i in range(n)]
    vg = rd["vg"]
    course = float(np.degrees(np.arctan2(rd["v_c"], rd["c"])))
    th_n = rd.get("thrust_n", 0.0)
    e = {"aircraft": name, "preset": preset, "hs": float(hs), "tp": float(tp), "direction": direction,
         "rel_deg": rel, "u10": float(u10), "u10_flown": max(float(u10), CALM_U10), "course_rel_deg": course,
         "chi_deg": rel + course, "crest": True, "success": True, "kind": kind,
         "period": P, "m": 0, "phi0": rd["theta"], "theta0": rd["theta"], "n_nodes": n,
         "mean_ground_velocity": vg, "speed_along_chi": float(np.hypot(*vg)),
         "power_avg_w": float(rd.get("power_w", 0.0)), "thrust_on_share": 1.0 if th_n > 0.0 else 0.0,
         "margins": {"h_min": WAVE_MARGINS[0], "n_frac": WAVE_MARGINS[1], "mu_frac": WAVE_MARGINS[2],
                     "v_factor": WAVE_MARGINS[3], "n_min": 0.0, "h_tip_min": 0.3},
         "ride": rd, "replay": replay, "h_min_position": "forward face", "tried": []}
    e.update({"t": t, "x": [vg[0] * s for s in t], "y": [vg[1] * s for s in t], "z": [rd["h"]] * n,
              "z_abs": [rd["h"] + rd["a"] * np.cos(rd["theta"])] * n, "V": [rd["V"]] * n,
              "gamma": [rd["gamma"]] * n, "psi": [rd["psi"]] * n, "cl": [rd["cl"]] * n, "mu": [0.0] * n,
              "thrust": [th_n] * n, "theta": [rd["theta"]] * n})
    return e


def crest_entries(name, preset, hs, tp, direction, u10, motor=True):
    """The crest-riding entries of one sea and wind: the slowest along-crest speed that soars (the purest ride,
    course nearest the swell's travel) and the fastest either way along the crest, each a library entry (kind
    soaring, crest True, steady in the wave frame) with the ride's phase as its wave phase. Where no ride soars
    and motor is True: the motor ride of least power (kind motor, motor_crest_ride)."""
    from sim.aircraft_models import by_name
    ac = by_name(name)
    vcs = np.arange(-30.0, 30.0 + 1e-9, CREST_VC_STEP)
    ok = [float(v) for v in vcs if crest_exists(ac, preset, hs, tp, direction, u10, float(v))]
    rides = {v: crest_ride(ac, preset, hs, tp, direction, u10, v) for v in {min(ok, key=abs), max(ok), min(ok)}} if ok else {}
    ok = [v for v, r in rides.items() if r is not None]
    if not ok:
        if not motor:
            return []
        rd = motor_crest_ride(ac, preset, hs, tp, direction, u10)
        if rd is None:
            return []
        return [_crest_entry(name, preset, hs, tp, direction, u10, rd, "motor", None)]
    return [_crest_entry(name, preset, hs, tp, direction, u10, rides[v], "soaring",
                         crest_replay(ac, preset, hs, tp, direction, u10, rides[v]))
            for v in sorted({min(ok, key=abs), max(ok), min(ok)})]


# ---- the library in the closed loop ----------------------------------------------------------------------------

def sea_of(world, psi_w):
    """(hs, tp, rel_deg) of the world's dominant swell component (rel: its travel, deg counter-clockwise from the
    wind's direction psi_w), or None without waves."""
    from sim.mpc import world_swell
    sw = world_swell(world)
    if sw is None:
        return None
    k, om, dx, dy, ph, a = sw
    rel = float(np.degrees((np.arctan2(dy, dx) - psi_w + np.pi) % (2.0 * np.pi) - np.pi))
    return 2.0 * a, 2.0 * np.pi / om, rel


class WavePilot:
    """The wave library in the closed loop (fly_mpc(wave_library=...)): for the aircraft, the preset of the wave
    wind and the world's swell, it names the wave-synchronized cycle to fly, or None where the world has no
    swell (the flat-sea library then decides, unchanged).

    The sea is matched to the library's nearest Hs, Tp, direction class (along, across, against, by |rel|; a sea
    with the swell to the right of the wind flies the mirrored entries) and wind. policy "course": the entry whose
    course relative to the swell is nearest the one asked; "energy": the least average electric power, soaring
    first; among soaring entries the crest ride with the slowest along-crest speed first, then the wave cycle
    with the largest dense-replay height. entry_filter: an optional callable(entry) -> bool."""

    def __init__(self, entries, preset, policy="energy", sig=None, sig_theta=None, entry_filter=None, w_run=None,
                 w_d=None, flat_entries=None):
        from sim.mpc import WAVE_W_RUN
        self.entries = [e for e in entries if e.get("success")]
        self.preset, self.policy = preset, policy
        self.sig = sig                 # None: the guide's defaults (WAVE_GUIDE_SIG, WAVE_CREST_SIG for crest rides)
        self.sig_theta = None if sig_theta is None else float(sig_theta)
        self.w_run = WAVE_W_RUN if w_run is None else float(w_run)
        self.w_d = w_d
        self.entry_filter = entry_filter
        self.flat_entries = flat_entries   # the flat library: a motor wave entry that needs more power is not flown
        self._cache = {}
        self.last = None

    @classmethod
    def load(cls, preset, path=WAVE_LIB_PATH, **kw):
        import json
        with open(path) as fh:
            return cls(json.load(fh)["entries"], preset, **kw)

    def sea_entries(self, name, sea, u10):
        """(entries, mirror) of the library sea and wind nearest to sea = (hs, tp, rel_deg) and u10."""
        hs, tp, rel = sea
        es = [e for e in self.entries if e["aircraft"] == name and e["preset"] == self.preset
              and (self.entry_filter is None or self.entry_filter(e))]
        if not es:
            return [], False
        h = min({e["hs"] for e in es}, key=lambda v: abs(v - hs))
        p = min({e["tp"] for e in es if e["hs"] == h}, key=lambda v: abs(v - tp))
        es = [e for e in es if e["hs"] == h and e["tp"] == p]
        mirror = rel < -1e-6 and abs(abs(rel) - 180.0) > 1e-6
        # calm entries are solved once ("along"); in calm air the swell's direction does not matter
        u = min({e["u10_flown"] for e in es}, key=lambda v: abs(v - u10))
        es = [e for e in es if e["u10_flown"] == u]
        if u <= CALM_U10 + 1e-9:
            return es, False
        d = min({e["rel_deg"] for e in es}, key=lambda v: abs(v - abs(rel)))
        return [e for e in es if e["rel_deg"] == d], mirror

    def choose(self, es, course_rel):
        if not es:
            return None
        if self.policy == "course":
            return min(es, key=lambda e: (abs((e["course_rel_deg"] - course_rel + 180.0) % 360.0 - 180.0),
                                          e["power_avg_w"]))
        soar = [e for e in es if e["kind"] == "soaring"]
        if soar:
            crest = [e for e in soar if e.get("crest")]
            if crest:
                return min(crest, key=lambda e: abs(e["ride"]["v_c"]))
            return max(soar, key=lambda e: e["replay"].get("rc_h_min") or 0.0)
        return min(es, key=lambda e: e["power_avg_w"])

    def cycle(self, name, world, u10, chi, psi_w):
        """mpc.Cycle to fly (chi: the course asked, rad from the wind), or None without a swell."""
        from sim.mpc import Cycle
        sea = sea_of(world, psi_w)
        if sea is None or sea[0] <= 0.0:
            return None
        es, mirror = self.sea_entries(name, sea, u10)
        sgn = -1.0 if mirror else 1.0
        course_rel = float((sgn * (np.degrees(chi) - sea[2]) + 180.0) % 360.0 - 180.0)
        if es and es[0]["direction"] != "across" and course_rel < 0.0:
            # along, against and calm seas are symmetric about the swell's travel: a course to the right of it
            # flies the mirrored entry of the course to the left
            mirror, course_rel = not mirror, -course_rel
        e = self.choose(es, course_rel)
        if e is None:
            return None
        if e["kind"] == "motor" and self.flat_entries is not None:
            f = flat_reference(self.flat_entries, name, min({x["u10"] for x in self.flat_entries
                                                             if x["aircraft"] == name}, key=lambda v: abs(v - u10)))
            if f is not None and e["power_avg_w"] >= f["power_avg_w"]:
                self.last = None
                return None          # the swell does not pay here: fly the flat library's cycle
        key = (id(e), mirror)
        if key not in self._cache:
            self._cache[key] = Cycle(e, mirror=mirror)
        self.last = e
        return self._cache[key]


# ---- the build -------------------------------------------------------------------------------------------------

def wave_seas(hs, tp):
    """(direction, u10) of the grid at one swell: calm once (the swell's direction does not matter without
    wind), then each direction at each wind."""
    return [("along", 0.0)] + [(d, u) for d in SWELL_DIRS for u in WAVE_U10 if u > 0.0]


def wave_tiers():
    """The grid in the order it is solved: Tp 10 s at Hs 3 and 2 m first (kept complete), then Hs 1.5 m at
    Tp 10 s, then Tp 8 and 12 s at Hs 3 and 2 m, then Hs 1.5 m at those periods."""
    return [[(3.0, 10.0), (2.0, 10.0)], [(1.5, 10.0)], [(3.0, 8.0), (2.0, 8.0), (3.0, 12.0), (2.0, 12.0)],
            [(1.5, 8.0), (1.5, 12.0)]]


def wave_jobs(seas, names=WAVE_AIRCRAFT, presets=PRESETS):
    jobs = []
    for hs, tp in seas:
        for d, u in wave_seas(hs, tp):
            for c in courses_for(d):
                for p in presets:
                    for n in names:
                        jobs.append((n, p, hs, tp, d, u, c))
    return jobs


def _crest_job(job):
    name, preset, hs, tp, d, u = job
    return crest_entries(name, preset, hs, tp, d, u)


def build_wave_library(workers=3, path=WAVE_LIB_PATH, budget_h=10.0, tiers=None, crest=True, dsopt_tiers=1):
    """python -m sim.mpc --build-wave-library: the wave-cycle library (see the module docstring and
    solve_wave_entry, crest_entries), written to path after every finished slot. Crest rides over every sea of
    wave_tiers; the optimizer's cycles over the first dsopt_tiers tiers only (the others are listed as cut). No
    tier is started once budget_h hours are used; the slots not solved are listed in the meta block."""
    import json
    import time
    from concurrent.futures import FIRST_COMPLETED, ProcessPoolExecutor, wait
    from sim import record
    here = _os.path.dirname(__file__)
    t0 = time.time()
    tiers = wave_tiers() if tiers is None else tiers
    meta = record.meta("python -m sim.mpc --build-wave-library --workers %d" % workers,
                       [__file__, _os.path.join(here, "dsopt.py"), _os.path.join(here, "seawind.py")])
    settings = {"aircraft": list(WAVE_AIRCRAFT), "presets": list(PRESETS), "hs": list(WAVE_HS),
                "tp": list(WAVE_TP), "directions": SWELL_DIRS, "u10": list(WAVE_U10), "calm_u10_flown": CALM_U10,
                "courses_rel_swell_deg": list(WAVE_COURSES), "across_extra_course_deg": -90.0,
                "tiers": [[list(s) for s in tier] for tier in tiers], "budget_h": budget_h,
                "margins": dict(zip(("h_min", "n_frac", "mu_frac", "v_factor"), WAVE_MARGINS)),
                "dsopt": "solve(mode max_speed along the course (soaring, folded_prop True) else min_power with "
                         "motor and v_travel_min 0.5 v_best_ld, cycle travel, n %d, wave {a: Hs/2, wavelength: "
                         "g Tp^2/2pi, to_deg}, wave_preset, tip_model geometric, midpoints, rate_limits pb2v, "
                         "n_min 0, phase_pin theta, m_search (-1, 0, 1), replay_check, refine, seeds %s, "
                         "stop_after 1, maxiter %d, profile_for(max(u10, %g), log, coare35))"
                         % (WAVE_N, list(WAVE_SEEDS), WAVE_MAXITER, CALM_U10),
                "crest": "steady flight in the wave frame on the forward face (crest_ride): along-crest ground "
                         "speeds from -30 to 30 m/s in %g m/s steps, folded propeller, wings level, no thrust, "
                         "the phase of the largest surplus at h_min raised to the balance height (at most %g m); "
                         "kept: the slowest along-crest speed that soars and the fastest either way"
                         % (CREST_VC_STEP, CREST_H_MAX),
                "frame": "wind toward +x, angles ccw from +x; z is the height above the LOCAL surface, z_abs above "
                         "mean sea level, theta the wave phase at the aircraft; thrust in N per node"}
    out = {"meta": meta, "settings": settings, "entries": [], "complete": False}

    def dump():
        out["runtime_s"] = time.time() - t0
        out["counts"] = library_counts(out["entries"])
        with open(path, "w", newline=chr(10)) as fh:
            json.dump(out, fh, indent=1)
            fh.write(chr(10))

    not_run = []
    with ProcessPoolExecutor(workers) as ex:
        if crest:
            cjobs = [(n, p, hs, tp, d, u) for tier in tiers for hs, tp in tier for d, u in wave_seas(hs, tp)
                     for p in PRESETS for n in WAVE_AIRCRAFT]
            for es in ex.map(_crest_job, cjobs, chunksize=4):
                out["entries"] += es
            dump()
        out["cut"] = [[list(s_) for s_ in tier] for tier in tiers[dsopt_tiers:]]
        settings["cut"] = ("the optimizer's cycles are solved only for the seas of the first %d tier(s); the other "
                           "seas have crest rides only (a slot costs about 30 to 230 s of one core)" % dsopt_tiers)
        n_done = 0
        for k, tier in enumerate(tiers[:dsopt_tiers]):
            jobs = wave_jobs(tier)
            if time.time() - t0 > budget_h * 3600.0:
                not_run += [list(j) for j in jobs]
                continue
            pending = {ex.submit(solve_wave_entry, j): j for j in jobs}
            while pending:
                done, _ = wait(pending, return_when=FIRST_COMPLETED)
                for f in done:
                    j = pending.pop(f)
                    e = f.result()
                    out["entries"].append(e)
                    n_done += 1
                    print("%d/%d %s %s %s" % (n_done, len(jobs), j, e.get("kind"), e.get("success")), flush=True)
                dump()
    out["not_run"] = not_run
    out["complete"] = not not_run
    dump()
    return out


def library_counts(entries):
    """Counts of the successful entries by aircraft, preset, Hs, Tp, direction and kind (crest rides apart)."""
    c = {}
    for e in entries:
        key = "%s|%s|Hs%g|Tp%g|%s" % (e["aircraft"], e["preset"], e["hs"], e["tp"],
                                       "calm" if e["u10"] == 0.0 else e["direction"])
        d = c.setdefault(key, {"soaring": 0, "motor": 0, "crest": 0, "missing": 0})
        if not e.get("success"):
            d["missing"] += 1
        elif e.get("crest"):
            d["crest"] += 1
        else:
            d[e["kind"]] += 1
    return c


# ---------------------------------------------------------------------------------------------
# step 4: the closed loop over swell (python -m sim.mpc --wave-loop-exp)
# ---------------------------------------------------------------------------------------------

WAVE_EXP_T_END = 600.0
WAVE_EXP_HS = (3.0, 2.0)
WAVE_EXP_TP = 10.0
WAVE_EXP_DIRS = ("along", "against")
WAVE_EXP_U10 = (0.0, 3.0, 6.0, 9.0)
WAVE_EXP_KEY = "wave_loop_exp"


def wave_pkw():
    """Planner settings of the wave runs: those of the navigation runs (sim.mpc.NAV_PKW) plus smooth perturbations
    (3 cosine modes), without which the planner could not sample the coherent turn a crest ride's heading needs."""
    from sim.mpc import NAV_PKW
    return dict(NAV_PKW, smooth_modes=3)


def wave_exp_cases():
    """(aircraft, hs, direction, u10, preset): both aircraft, Hs 3 and 2 m at Tp 10 s, the swell along and against
    the wind, u10 0, 3, 6 and 9 m/s, P63 and the P64 low bound. In calm air the direction does not matter, so the
    calm case runs once ("along") and stands for both."""
    return [(n, hs, d, u, p) for n in WAVE_AIRCRAFT for hs in WAVE_EXP_HS for d in WAVE_EXP_DIRS for u in WAVE_EXP_U10
            for p in PRESETS if not (u == 0.0 and d != "along")]


def flat_reference(lib_entries, name, u10):
    """The flat-sea comparison at the same wind: the motor library's entry of least average power (any course),
    or a soaring one (power 0) where the flat sea soars."""
    es = [e for e in lib_entries if e["aircraft"] == name and e["u10"] == u10 and e.get("success")]
    if not es:
        return None
    e = min(es, key=lambda e: (e.get("power_avg_w", 0.0) if e["kind"] == "motor" else 0.0))
    return {"u10": e["u10"], "chi_deg": e["chi_deg"], "kind": e["kind"],
            "power_avg_w": float(e.get("power_avg_w", 0.0)) if e["kind"] == "motor" else 0.0,
            "speed_along_chi": e["speed_along_chi"]}


def flat_start_entry(ac, u10, course_deg):
    """The flat flight the runs start from: the navigation's steady motor leg (sim.navigator.SteadyModel, level at
    10 m above the local surface, the airspeed of least electric energy per metre) along the course (deg from the
    wind's direction), as a library entry (sim.mpc.steady_entry)."""
    from sim.mpc import steady_entry
    from sim.navigator import HullPoint, SteadyModel
    sm = SteadyModel.from_aircraft(ac)
    c = np.radians(course_deg)
    lg = sm.leg(max(float(u10), CALM_U10), (float(np.cos(c)), float(np.sin(c))))
    st = {"V": lg["V"], "cl": lg["cl"], "thrust": lg["thrust"], "heading": lg["heading"], "height": sm.height}
    pt = HullPoint(lg["ground"][0], lg["ground"][1], lg["power"], "motor", "start", None, 1, st)
    return steady_entry(pt)


def run_wave_job(job):
    """One closed-loop run over swell: oracle sea (the planner flies the true sea of swell_env), truth state and
    wind, motor on at the middle energy weight, actuators pb2v, load-factor limit, stop on a wingtip contact, reflex
    off, the flat motor library for the start (entry from flat flight) and the wave library (WavePilot, policy
    "energy") over the swell. Returns the metrics of fly_mpc plus the margins (sim.mpc.flight_margins), the power
    after the entry onto the wave library and the flat-sea comparison."""
    import time
    from sim.aircraft import Actuators
    from sim.aircraft_models import by_name
    from sim import mpc
    name, hs, d, u10, preset, t_end = job[:6]
    err = job[6] if len(job) > 6 else None      # the planner's sea with an error: ("phase", rad) or ("height", factor)
    ac = by_name(name)
    env = swell_env(u10, hs, WAVE_EXP_TP, d, preset)
    pw = None
    if err is not None:
        pw = (swell_env(u10, hs, WAVE_EXP_TP, d, preset, phase=err[1]) if err[0] == "phase"
              else swell_env(u10, hs * err[1], WAVE_EXP_TP, d, preset))
    lib = mpc.CycleLibrary.from_keyword("motor")
    pilot = WavePilot.load(preset, policy="energy", flat_entries=lib.entries)
    bearing = (env.wind.to_deg - SWELL_DIRS[d]) % 360.0      # start along the swell's travel
    start = flat_start_entry(ac, u10, SWELL_DIRS[d])
    t0 = time.time()
    r = mpc.fly_mpc(ac, env, "truth", [], t_end, bearing, max(u10, CALM_U10), seed=1, planner_kw=wave_pkw(),
                    library=lib, state_source="truth", wind_source="truth",
                    actuators=Actuators(ac, rate_model="pb2v"), n_limit=True, stop_on_tip=True, wave_aware=False,
                    motor=True, w_energy=mpc.energy_weight(ac, "normal"), keep=True, wave_library=pilot,
                    start_entry=start, planner_world=pw)
    tr = r.pop("traj")
    T, ST, MU = np.asarray(tr["t"]), np.asarray(tr["state"]), np.asarray(tr["mu"])
    marg = mpc.flight_margins(ac, env, T, ST, MU, r)
    marg["counts"] = bool(r["ended"] == "complete" and marg["valid"])
    sw = r.get("wave_switches") or []
    t_in = sw[0][0] if sw else None
    th = np.asarray(tr["thrust"], float)
    P = th * ST[:, 3] / r["motor"]["eta_total"]          # electric power per step, as sim.mpc.burst_stats
    after = None
    if t_in is not None and T[-1] > t_in + 1.0:
        k = T >= t_in
        after = {"t_entry_s": float(t_in), "duration_s": float(T[-1] - t_in), "avg_power_w": float(P[k].mean()),
                 "thrust_on_share": float(np.mean(th[k] > mpc.BURST_ON_FRAC * ac.mass * G)),
                 "height_median_m": float(np.median(ST[k, 2] - np.asarray(env.surface(ST[k, 0], ST[k, 1], T[k])[0])))}
    e = pilot.last
    entry = None if e is None else {"course_rel_deg": e["course_rel_deg"], "crest": bool(e.get("crest")),
                                    "kind": e["kind"], "power_avg_w": e["power_avg_w"],
                                    "h": e["ride"]["h"] if e.get("ride") else None}
    mot = r["motor"]
    mot["bursts"] = mot.get("bursts", [])[:10]
    out = {"aircraft": name, "hs": hs, "tp": WAVE_EXP_TP, "direction": d, "u10": u10, "preset": preset,
           "t_end": t_end, "planner_sea_error": None if err is None else list(err), "ended": r["ended"], "survival_s": r["survival_s"], "wall_s": time.time() - t0,
           "margins": marg, "motor": mot, "after_entry": after, "entry": entry, "wave_switches": sw,
           "flat": flat_reference(lib.entries, name, u10), "cpu_mean_s": r["cpu_mean_s"],
           "max_n_flown": r["max_n_flown"], "min_clear_wingtip": r.get("min_clear_wingtip")}
    return mpc._no_nan(out)


def wave_exp_table(rows):
    lines = ["| aircraft | Hs | swell | u10 | wave wind | end (s) | counts | power (W), after entry | motor on | "
             "h median (m) | entry (s), course, kind | flat sea (W) |", "|" + "---|" * 12]
    for r in rows:
        m, a, e, f = r["motor"], r.get("after_entry") or {}, r.get("entry") or {}, r.get("flat") or {}
        lines.append("| %s | %g | %s | %g | %s | %s %.0f | %s | %.1f, %s | %.2f | %.1f | %s, %s, %s | %s |" % (
            r["aircraft"], r["hs"], r["direction"] if r["u10"] > 0 else "calm", r["u10"], r["preset"], r["ended"],
            r["survival_s"], int(r["margins"]["counts"]), m["avg_power_w"],
            "n/a" if not a else "%.1f" % a["avg_power_w"], m["thrust_on_share"], r["margins"]["height_median_m"],
            "n/a" if not a else "%.0f" % a["t_entry_s"], "n/a" if not e else "%.0f" % e["course_rel_deg"],
            "n/a" if not e else ("crest " if e["crest"] else "") + e["kind"],
            "n/a" if not f else "%.1f" % f["power_avg_w"]))
    return chr(10).join(lines)


def wave_exp_main(a):
    """python -m sim.mpc --wave-loop-exp [--wave-only ...]: key wave_loop_exp of sim/out/mpc.json (merged by case
    when --wave-only runs a subset)."""
    import json
    import time
    from concurrent.futures import FIRST_COMPLETED, ProcessPoolExecutor, wait
    from sim import record
    from sim.mpc import BURST_ON_FRAC, W_ENERGY_LEVELS
    here = _os.path.dirname(__file__)
    t_end = getattr(a, "wave_t_end", None) or WAVE_EXP_T_END
    cases = wave_exp_cases()
    if a.wave_only:
        keys = set(a.wave_only.split(";"))
        cases = [c for c in cases if "%s,%g,%s,%g,%s" % c in keys]
    jobs = [c + (t_end,) for c in cases]
    t0 = time.time()
    d = json.load(open(a.out)) if _os.path.exists(a.out) else {}
    kept = d.get(WAVE_EXP_KEY, {}).get("results", []) if a.wave_only else []
    rows = [r for r in kept if (r["aircraft"], r["hs"], r["direction"], r["u10"], r["preset"]) not in set(cases)]
    order = {c: i for i, c in enumerate(wave_exp_cases())}

    def dump(done):
        rows.sort(key=lambda r: order.get((r["aircraft"], r["hs"], r["direction"], r["u10"], r["preset"]), 1e9))
        d[WAVE_EXP_KEY] = {"meta": dict(meta, runtime_s=time.time() - t0, complete=done), "results": rows}
        with open(a.out, "w", newline=chr(10)) as fh:
            json.dump(d, fh, indent=1)
            fh.write(chr(10))

    meta = record.meta("python -m sim.mpc --wave-loop-exp --workers %d" % a.workers
                       + (" --wave-only '%s'" % a.wave_only if a.wave_only else ""),
                       [_os.path.join(here, "mpc.py"), __file__, WAVE_LIB_PATH,
                        _os.path.join(here, "out", "mpc_library_motor.json"), _os.path.join(here, "seawind.py"),
                        _os.path.join(here, "aircraft.py")])
    meta.update({"runtime_s": time.time() - t0, "t_end": t_end, "seed": 1, "workers": a.workers,
                 "planner_kw": wave_pkw(), "energy_level": "normal (k = %g)" % W_ENERGY_LEVELS["normal"],
                 "sea": "swell_env: one regular long-crested swell component, Hs and Tp %g s, along or against the "
                        "wind (from 270 deg), COARE 3.5 log wind of u10 (calm flown at %g m/s), the wave wind of the "
                        "preset, second-order surface; the planner flies the same sea (oracle)" % (WAVE_EXP_TP, CALM_U10),
                 "start": "in flat flight: the navigation's steady motor leg (level at 10 m above the local surface, "
                          "airspeed of least energy per metre) along the swell's travel (flat_start_entry); the wave "
                          "library's entry is taken at a wave-phase match (at most 60 s of waiting)",
                 "wave_library": "WavePilot policy energy: soaring entries first (crest rides first, slowest "
                                 "along-crest speed), else the least average power, unless that motor entry needs "
                                 "at least the flat library's least power at the wind (then the start's steady "
                                 "flat leg is flown on); guide WaveCycleGuide",
                 "state_source": "truth", "wind_source": "truth", "reflex": False,
                 "actuators": "rate_model pb2v", "n_limit": True, "stop_on_tip": True,
                 "counts": "ended complete and flight_margins valid: heights relative to the local surface inside "
                           "the band of R-12, wingtip at least 0.3 m, no loop, no flight below the stall speed, "
                           "load factor within its limit",
                 "motor_on": "thrust above %g of the weight" % BURST_ON_FRAC,
                 "after_entry": "average power, motor-on share and median height from the first switch onto a wave "
                                "entry to the end",
                 "flat": "the flat motor library's entry of least average power at the same wind (any course)"})
    with ProcessPoolExecutor(a.workers) as ex:
        pending = {ex.submit(run_wave_job, j): j for j in jobs}
        while pending:
            done, _ = wait(pending, return_when=FIRST_COMPLETED)
            for f in done:
                pending.pop(f)
                r = f.result()
                rows.append(r)
                print("%d/%d %s %.0f counts %d P %.1f" % (len(rows), len(jobs), r["ended"], r["survival_s"],
                                                          r["margins"]["counts"], r["motor"]["avg_power_w"]),
                      flush=True)
            dump(False)
    print(wave_exp_table(rows))
    dump(True)


# ---------------------------------------------------------------------------------------------
# what sensing must deliver for stage 2 (python -m sim.mpc --wave-sense-exp)
# ---------------------------------------------------------------------------------------------

WAVE_SENSE_KEY = "wave_sense_exp"
WAVE_SENSE_CASE = ("albatross-3m", 3.0, "along", 6.0, "p63")
WAVE_SENSE_ERRORS = (("phase", 0.15), ("phase", 0.3), ("phase", 0.6), ("phase", 1.2), ("phase", -0.6),
                     ("height", 0.8), ("height", 1.2))
WAVE_SENSE_T_END = 300.0


def wave_sense_main(a):
    """The closed loop of one case of the wave experiment with the planner's sea known with an error: the swell's
    phase shifted by a constant (rad; the planner places the crests that far ahead or behind) or its height scaled.
    Key wave_sense_exp of sim/out/mpc.json."""
    import json
    import time
    from concurrent.futures import ProcessPoolExecutor
    from sim import record
    here = _os.path.dirname(__file__)
    jobs = [WAVE_SENSE_CASE + (WAVE_SENSE_T_END, e) for e in WAVE_SENSE_ERRORS]
    jobs.append(WAVE_SENSE_CASE + (WAVE_SENSE_T_END,))
    t0 = time.time()
    with ProcessPoolExecutor(a.workers) as ex:
        rows = list(ex.map(run_wave_job, jobs, chunksize=1))
    for r in rows:
        m, af = r["motor"], r.get("after_entry") or {}
        print(r["planner_sea_error"], r["ended"], r["survival_s"], int(r["margins"]["counts"]),
              "%.1f" % m["avg_power_w"], af.get("avg_power_w"), r["margins"]["height_median_m"])
    d = json.load(open(a.out)) if _os.path.exists(a.out) else {}
    meta = record.meta("python -m sim.mpc --wave-sense-exp --workers %d" % a.workers,
                       [_os.path.join(here, "mpc.py"), __file__, WAVE_LIB_PATH,
                        _os.path.join(here, "out", "mpc_library_motor.json"), _os.path.join(here, "seawind.py")])
    meta.update({"runtime_s": time.time() - t0, "case": list(WAVE_SENSE_CASE), "t_end": WAVE_SENSE_T_END,
                 "errors": [list(e) for e in WAVE_SENSE_ERRORS],
                 "how": "as wave_loop_exp, with fly_mpc(planner_world=...): the planner and the wave library's "
                        "choice see the swell with its phase shifted by the given angle (phase) or its height "
                        "scaled (height); the aircraft flies the true sea; the last row has no error"})
    d[WAVE_SENSE_KEY] = {"meta": meta, "results": rows}
    with open(a.out, "w", newline=chr(10)) as fh:
        json.dump(d, fh, indent=1)
        fh.write(chr(10))


# ---------------------------------------------------------------------------------------------
# step 5: a navigation run over swell (python -m sim.mpc --wave-nav-exp)
# ---------------------------------------------------------------------------------------------

WAVE_NAV_KEY = "wave_nav_exp"
# (aircraft, Hs, swell direction, u10, preset, distance, target): "across" is across the swell's travel (and, with
# the swell along the wind, across the wind too); "with" is along the swell's travel, the direction its soaring
# entries carry the aircraft
WAVE_NAV_CASES = (("albatross-3m", 3.0, "along", 6.0, "p63", 5000.0, "across"),
                  ("albatross-3m", 3.0, "along", 6.0, "p63", 5000.0, "with"))


def wave_hull_entries(entries, name, preset, hs, tp, direction, u10):
    """The wave library's entries of one sea and wind (for the velocity hull of the navigator)."""
    return [e for e in entries if e.get("success") and e["aircraft"] == name and e["preset"] == preset
            and e["hs"] == hs and e["tp"] == tp and e["direction"] == direction and e["u10"] == u10]


def run_wave_nav(job):
    """The navigated closed loop over swell: the velocity hull of the wave library's entries at the sea and wind
    (sim.navigator.HullSource; the mirror images are valid for a swell along or against the wind), the hull
    navigator with the steady motor legs, the planner flying what it names (wave entries with WaveCycleGuide),
    oracle sea, truth state and wind, start in flat flight towards the target. Returns the run with its kinematic
    prediction (sail_hull on the same hull)."""
    import json
    import time
    from sim.aircraft import Actuators
    from sim.aircraft_models import by_name
    from sim import mpc
    from sim.navigator import HullNavigator, HullSource, SteadyModel, sail_hull
    name, hs, d, u10, preset, dist, target = job
    ac = by_name(name)
    env = swell_env(u10, hs, WAVE_EXP_TP, d, preset)
    with open(WAVE_LIB_PATH) as fh:
        entries = json.load(fh)["entries"]
    es = wave_hull_entries(entries, name, preset, hs, WAVE_EXP_TP, d, u10)
    src = HullSource.from_entries(es, name)
    steady = SteadyModel.from_aircraft(ac)
    rel = np.radians(SWELL_DIRS[d] + (0.0 if target == "with" else 90.0))
    # the wind blows from the west (toward +x), so the swell's travel is at SWELL_DIRS[d] from +x
    tgt = (dist * float(np.cos(rel)), dist * float(np.sin(rel)))
    pred = sail_hull(src, lambda t: (u10, env.wind.from_deg), (0.0, 0.0), tgt,
                     nav=HullNavigator(src, arrival_radius=mpc.NAV_ARRIVAL_M, steady=steady), t_max=6 * 3600.0)
    first = HullNavigator(src, arrival_radius=mpc.NAV_ARRIVAL_M, steady=steady).step(
        0.0, (0.0, 0.0), u10, env.wind.from_deg, tgt)
    t_end = min(2.0 * pred["time"] + 600.0, 7200.0) if pred["arrived"] else 3600.0
    start = flat_start_entry(ac, u10, float(np.degrees(rel)))
    t0 = time.time()
    r = mpc.fly_mpc(ac, env, "truth", [], t_end, 0.0, max(u10, CALM_U10), seed=1, planner_kw=wave_pkw(),
                    library=mpc.CycleLibrary.from_keyword("motor"), state_source="truth", wind_source="truth",
                    actuators=Actuators(ac, rate_model="pb2v"), n_limit=True, stop_on_tip=True, wave_aware=False,
                    motor=True, w_energy=mpc.energy_weight(ac, "normal"), target=tgt, navigator=True,
                    nav_hull=src, guide_sig=mpc.NAV_GUIDE_SIG, nav_kw={"steady": steady}, start_entry=start)
    r["wall_s"] = time.time() - t0
    r = mpc._no_nan(r)
    r["motor"]["bursts"] = r["motor"].get("bursts", [])[:20]
    hull = src.at(u10)
    r["predicted"] = {"arrived": pred["arrived"], "time_s": pred["time"], "n_legs": pred["n_legs"],
                      "mean_vmg_ms": pred["mean_vmg"], "energy_wh": pred["energy_j"] / 3600.0,
                      "plan_vmg_ms": first.vmg, "plan_kind": first.kind, "plan_energy_per_m_j": first.per_m,
                      "track": pred["track"][::10].tolist()}
    r["hull"] = [{"vx": p.vx, "vy": p.vy, "power": p.power, "kind": p.kind, "key": p.key,
                  "crest": bool((p.entry or {}).get("crest"))} for p in hull.points]
    return {"aircraft": name, "hs": hs, "tp": WAVE_EXP_TP, "direction": d, "u10": u10, "preset": preset,
            "distance_m": dist, "target_rel_swell": target, "target": list(tgt),
            "hull_points": len(hull.points), **r}


def wave_nav_main(a):
    import json
    import time
    from concurrent.futures import ProcessPoolExecutor
    from sim import record
    here = _os.path.dirname(__file__)
    t0 = time.time()
    with ProcessPoolExecutor(min(a.workers, len(WAVE_NAV_CASES))) as ex:
        rows = list(ex.map(run_wave_nav, WAVE_NAV_CASES, chunksize=1))
    for r in rows:
        nv = r.get("nav") or {}
        print(r["target_rel_swell"], r["ended"], nv.get("arrival_time_s"), nv.get("counts_as_arrived"),
              round(nv.get("mean_vmg_ms") or 0, 2), round(r["motor"]["energy_wh"], 1),
              round(r["motor"]["thrust_on_share"], 2), nv.get("height_median_m"),
              {k: r["predicted"][k] for k in ("arrived", "time_s", "n_legs", "plan_kind", "plan_vmg_ms")},
              nv.get("kind_time_s"))
    d = json.load(open(a.out)) if _os.path.exists(a.out) else {}
    meta = record.meta("python -m sim.mpc --wave-nav-exp",
                       [_os.path.join(here, "mpc.py"), __file__, WAVE_LIB_PATH, _os.path.join(here, "navigator.py"),
                        _os.path.join(here, "out", "mpc_library_motor.json")])
    meta.update({"runtime_s": time.time() - t0, "cases": [list(c) for c in WAVE_NAV_CASES],
                 "planner_kw": wave_pkw(),
                 "sea": "swell_env (oracle), wind from 270 deg, swell running east (along the wind); the target "
                        "lies across the swell's travel (north) or with it (east)",
                 "hull": "the wave library's entries at the sea and wind with their mirror images "
                         "(sim.navigator.HullSource), the steady motor legs of the flat sea (SteadyModel)",
                 "start": "flat flight: the steady motor leg towards the target (flat_start_entry)",
                 "arrival": "within %g m; counts_as_arrived also needs the band and the margins" % 200.0})
    d[WAVE_NAV_KEY] = {"meta": meta, "results": rows}
    with open(a.out, "w", newline=chr(10)) as fh:
        json.dump(d, fh, indent=1)
        fh.write(chr(10))
