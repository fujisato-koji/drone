"""Wingtip skid contact and the reflex time budget (D-018, docs/02-form.md#wingtip-skim).

Hand calculations made reproducible, for the low turn of the skim study's cycles
(sim/out/skim_study.json, the skid-touch cells: wingtip clearance 0 at the lowest floor):

1. Water forces on a skid at the tip, against the immersion depth d, at the water speed of the
   tip in the low turn: a thin blade with its leading edge raked back (friction on both sides,
   spray drag of the piercing edge, side force of a slender foil if the flow meets it at an
   angle), a small planing ski (Savitsky's planing lift), the bare wingtip (the wing's own
   section at its angle of attack), and a wingtip float (a planing body with its air drag).
2. Their yaw and roll moments about the centre of mass and the impulse of contacts of 0.05 to
   0.2 s, against the momentum and the roll and yaw authority.
3. The time budget of the reflex: how deep the tip goes when the sea stands higher than
   planned, for a given delay from contact to the start of the roll-out (sensor, processing
   and the actuators of ctrl.actuator_delay_s and ctrl.actuator_tau_s), with the roll rate of
   ctrl.roll_pb2v.

Command: python -m sim.skid (writes sim/out/skid.json).
"""
import json
import os

import numpy as np

from sim.aircraft import G
from sim.record import value

OUT_PATH = os.path.join(os.path.dirname(__file__), "out", "skid.json")
SKIM_PATH = os.path.join(os.path.dirname(__file__), "out", "skim_study.json")
RHO_AIR = 1.225
NU_AIR = 1.46e-5


# assumed sizes of the candidate skids and the rudder (record skid.*, Q-070); the defaults are the
# record's values
DEFAULTS = {"blade_thickness_m": 1.0e-3, "blade_rake_deg": 60.0, "blade_depth_m": 0.04,
            "ski_beam_m": 0.02, "ski_length_m": 0.06, "ski_trim_deg": 4.0,
            "float_beam_m": 0.04, "float_length_m": 0.12, "rudder_cl_max": 0.5,
            "blade_mass_kg": 0.006, "ski_mass_kg": 0.010, "float_mass_kg": 0.030,
            "misalignment_deg": 5.0, "sense_process_s": 0.01}


def par(name):
    return value("skid." + name, DEFAULTS[name])


def rho_water():
    return value("skid.rho_seawater_kg_m3", 1026.0)


def nu_water():
    return value("skid.nu_seawater_m2_s", 1.19e-6)


def cf_ittc(re):
    """ITTC 1957 friction line, Cf = 0.075 / (log10 Re - 2)^2 (one side, turbulent)."""
    return 0.075 / (np.log10(max(re, 1e3)) - 2.0) ** 2


def q_water(v):
    return 0.5 * rho_water() * v * v


# ---- 1. water forces at immersion depth d (m), water speed v (m/s)
def blade(d, v, t=None, rake_deg=None, depth=None, beta_deg=0.0):
    """Thin flat blade below the tip, its plane along the flow, leading edge raked back by
    rake_deg from the vertical, trailing edge vertical, so that the wetted part at depth d is a
    triangle of area d^2 tan(rake)/2 per side (d capped at the blade's depth). Drag: friction on
    both sides (ITTC 1957 at the waterline length) and the spray drag of the piercing edge,
    skid.spray_cd_t2 q t^2. Side force if the flow meets the blade at beta_deg: slender-wing lift
    (pi/2) AR beta on the wetted triangle (AR = d^2 / area; a free surface at high Froude number
    does not double it), an upper bound since ventilation lowers it."""
    t = par("blade_thickness_m") if t is None else t
    rake_deg = par("blade_rake_deg") if rake_deg is None else rake_deg
    depth = par("blade_depth_m") if depth is None else depth
    d = min(max(d, 0.0), depth)
    tr = np.tan(np.radians(rake_deg))
    area = 0.5 * d * d * tr
    q = q_water(v)
    if d <= 0.0:
        return {"drag": 0.0, "side": 0.0, "normal": 0.0, "area": 0.0}
    ell = d * tr
    fric = 2.0 * cf_ittc(v * ell / nu_water()) * area * q
    spray = value("skid.spray_cd_t2", 0.24) * q * t * t
    ar = d * d / area
    side = 0.5 * np.pi * ar * np.radians(beta_deg) * q * area
    return {"drag": float(fric + spray), "friction": float(fric), "spray": float(spray),
            "side": float(side), "normal": 0.0, "area": float(area)}


def ski(d, v, beam=None, length=None, trim_deg=None):
    """Flat planing ski of the given beam and length at trim_deg to the water, immersed by d at
    its trailing edge: wetted length d / sin(trim) (capped at the length), Savitsky's lift
    C_L0 = tau^1.1 (0.0120 lambda^0.5 + 0.0055 lambda^2.5 / Cv^2) on beam^2 (tau in degrees,
    lambda = wetted length / beam, Cv = v / sqrt(g beam)); drag = lift tan(trim) + friction on
    the wetted bottom. The Froude number is far above Savitsky's range, where the Cv term is
    nil anyway."""
    beam = par("ski_beam_m") if beam is None else beam
    length = par("ski_length_m") if length is None else length
    trim_deg = par("ski_trim_deg") if trim_deg is None else trim_deg
    tau = np.radians(trim_deg)
    if d <= 0.0:
        return {"drag": 0.0, "side": 0.0, "normal": 0.0, "lam": 0.0}
    lw = min(d / np.sin(tau), length)
    lam = lw / beam
    cv = v / np.sqrt(G * beam)
    cl = trim_deg ** 1.1 * (0.0120 * lam ** 0.5 + 0.0055 * lam ** 2.5 / cv ** 2)
    q = q_water(v)
    lift = cl * q * beam * beam
    fric = cf_ittc(v * lw / nu_water()) * q * lw * beam / np.cos(tau)
    return {"drag": float(lift * np.tan(tau) + fric), "side": 0.0, "normal": float(lift),
            "lam": float(lam), "cv": float(cv), "friction": float(fric)}


def bare_tip(d, v, chord, tc, bank_deg, alpha_deg):
    """The wingtip itself immersed by d at a bank: a strip of span d / sin(bank) along the tip
    chord, at the wing's angle of attack. Normal force of a very low aspect ratio plate,
    (pi/2) AR alpha + 2 sin^2 alpha (cross-flow drag coefficient 2), drag from the spray of the
    section's leading edge (skid.spray_cd_t2 on the thickness squared), friction and the normal
    force's backward component."""
    if d <= 0.0:
        return {"drag": 0.0, "side": 0.0, "normal": 0.0}
    s_ = d / np.sin(np.radians(bank_deg))
    area = s_ * chord
    ar = s_ / chord
    a = np.radians(alpha_deg)
    q = q_water(v)
    cn = 0.5 * np.pi * ar * a + 2.0 * np.sin(a) ** 2
    fn = cn * q * area
    fric = 2.0 * cf_ittc(v * chord / nu_water()) * q * area
    spray = value("skid.spray_cd_t2", 0.24) * q * (tc * chord) ** 2
    return {"drag": float(fric + spray + fn * np.sin(a)), "side": 0.0, "normal": float(fn * np.cos(a)),
            "friction": float(fric), "spray": float(spray)}


def tip_float(d, v):
    """A small wingtip float: in the water it planes like a ski of its beam and length."""
    return ski(d, v, beam=par("float_beam_m"), length=par("float_length_m"), trim_deg=par("ski_trim_deg"))


def air_drag_area(kind):
    """Air drag area (m2) of one skid at 15 m/s: the blade (its depth square with the triangle
    below the tip and as much again at its root, laminar friction on both sides and a blunt
    edge of 0.5 on the thickness times the depth); the ski (its plate the same way, 1.5 mm
    thick, on a strut like the blade); the float (turbulent friction and the Hoerner form
    factor on a body of revolution, as wingopt.body_drag)."""
    from sim.wingopt import _cf, _form_factor
    v = 15.0
    if kind == "blade":
        h, t = par("blade_depth_m"), par("blade_thickness_m")
        c = h * np.tan(np.radians(par("blade_rake_deg")))
        area = 0.5 * c * h + 0.5 * c * h
        cf = 1.328 / np.sqrt(v * c / NU_AIR)
        return float(2.0 * cf * area + 0.5 * t * h)
    if kind == "ski":
        c, b, t = par("ski_length_m"), par("ski_beam_m"), 1.5e-3
        cf = 1.328 / np.sqrt(v * c / NU_AIR)
        return float(2.0 * cf * c * b + 0.5 * t * b) + air_drag_area("blade")
    if kind == "float":
        dd, ll = par("float_beam_m"), par("float_length_m")
        fr = ll / dd
        swet = np.pi * dd * ll * (1.0 - 2.0 / fr) ** (2.0 / 3.0) * (1.0 + 1.0 / fr ** 2)
        return float(_cf(v * ll / NU_AIR) * _form_factor(fr) * swet)
    raise ValueError(kind)




# ---- 2. inertia, authority, moments and impulses
def inertia(m_wing, span, taper=0.45, tail_arm=None):
    """Roll inertia of the wing, its mass spread with the chord of a linear taper (the mean of
    eta^2 weighted by the chord, eta = 2y/b), and the yaw inertia adding the tail (tail.mass_kg)
    at the tail arm and the boom (boom.mass_kg) spread from half the arm to the arm (the other
    masses at the centre of mass)."""
    e = np.linspace(0.0, 1.0, 2001)
    c = 1.0 - (1.0 - taper) * e
    m_e2 = float(np.trapezoid(e * e * c, e) / np.trapezoid(c, e))
    ixx = m_wing * (0.5 * span) ** 2 * m_e2
    arm = 0.33 * span if tail_arm is None else tail_arm
    mt, mb = value("tail.mass_kg", 0.05), value("boom.mass_kg", 0.06)
    izz = ixx + mt * arm ** 2 + mb * (arm ** 3 - (0.5 * arm) ** 3) / (3.0 * 0.5 * arm)
    return {"ixx": ixx, "izz": izz, "mean_eta2": m_e2}


def roll_damping(ar, taper=0.45):
    """Strip-theory roll damping C_lp = -(2 a / (S b^2)) int c y^2 dy over the span (a linear
    taper), with the lift slope a = 2 pi AR / (2 + sqrt(AR^2 + 4))."""
    a = 2.0 * np.pi * ar / (2.0 + np.sqrt(ar * ar + 4.0))
    e = np.linspace(0.0, 1.0, 2001)
    c = 1.0 - (1.0 - taper) * e
    # S b^2 = b^3 cbar (b = 2), int c y^2 dy over -b/2..b/2 = 2 (b/2)^3 int c eta^2 d eta
    ratio = 2.0 * 0.125 * np.trapezoid(c * e * e, e) / (np.trapezoid(c, e) * 1.0)
    return float(-2.0 * a * ratio), float(a)


def authority(mass, span, area, v, ixx, izz):
    """Roll and yaw authority at airspeed v: the roll rate of ctrl.roll_pb2v, the aileron moment
    that holds it against the strip-theory roll damping L_p = |C_lp| q S b^2 / (2 v), and a rudder
    moment of VV (wingopt) times skid.rudder_cl_max on q S b; the weathercock stiffness
    N_beta = q S b VV a_v of the fin alone (lift slope a_v at wingopt.AR_V)."""
    from sim.wingopt import AR_V, VV
    k = value("ctrl.roll_pb2v", 0.08)
    clp, a = roll_damping(span * span / area)
    q = 0.5 * RHO_AIR * v * v
    p_max = 2.0 * k * v / span
    lp = abs(clp) * q * area * span * span / (2.0 * v)
    a_v = 2.0 * np.pi * AR_V / (2.0 + np.sqrt(AR_V ** 2 + 4.0))
    return {"p_max_rad_s": p_max, "clp": clp, "lift_slope": a, "lp_nms": lp,
            "roll_moment_max_nm": abs(clp) * k * q * area * span,
            "yaw_moment_max_nm": VV * par("rudder_cl_max") * q * area * span,
            "n_beta_nm_per_rad": q * area * span * VV * a_v, "roll_time_const_s": ixx / lp}


def contact_effects(force, span, mass, v, ixx, izz, auth, durations=(0.05, 0.1, 0.2)):
    """Yaw moment D b/2 and roll moment (normal force, or the side force times sin bank for the
    blade, both on b/2) of a contact at the tip, against the rudder and the ailerons. For each
    duration: the linear impulse over the momentum; the yaw rate and angle the angular impulse
    leaves (no yaw damping, as the yaw mode is slow against the contact); the bank change with
    the roll damping, whose time constant is far shorter than the contact, so the roll rate
    settles at M / L_p, which is roll_over_aileron times the roll rate of ctrl.roll_pb2v; and the
    steady sideslip N / N_beta the yaw moment would hold."""
    s = 0.5 * span
    n_yaw = force["drag"] * s
    n_roll = force.get("normal", 0.0) * s + abs(force.get("side_roll", 0.0)) * s
    tr = auth["roll_time_const_s"]
    out = {"yaw_moment_nm": n_yaw, "roll_moment_nm": n_roll,
           "yaw_over_rudder": n_yaw / auth["yaw_moment_max_nm"],
           "roll_over_aileron": n_roll / auth["roll_moment_max_nm"],
           "sideslip_steady_deg": float(np.degrees(n_yaw / auth["n_beta_nm_per_rad"])), "pulses": []}
    for dt in durations:
        out["pulses"].append({"dt_s": dt, "impulse_over_momentum": force["drag"] * dt / (mass * v),
                              "yaw_rate_deg_s": float(np.degrees(n_yaw * dt / izz)),
                              "yaw_angle_deg": float(np.degrees(0.5 * n_yaw * dt * dt / izz)),
                              "bank_change_deg": float(np.degrees(n_roll / auth["lp_nms"]
                                                                  * (dt - tr * (1.0 - np.exp(-dt / tr)))))})
    return out


# ---- 3. the reflex time budget
def reflex_budget(bank_deg, v, span, n_load, delta, w, delay, dt=1e-3, t_end=1.5):
    """Deepest immersion of the tip and the time it is wet when the sea under a tip that runs
    level at its planned clearance rises above the plan at the rate w (m/s: the water speed of
    the tip times the slope of the sea, plus the wave's own rise) up to delta and stays there
    (the skid-touch cycles hold the tip at the clearance for over a second, so the planned rise
    of the path is left out). The tip meets the water at t = 0; the reflex starts rolling towards
    wings level at t = delay (negative: before contact, as a range sensor allows; None: never),
    when the gap is gap_at_start_m. The roll rate rises to 2 k v / b (ctrl.roll_pb2v) through a
    first-order lag of ctrl.actuator_tau_s; the load factor is held at n_load, so the roll-out
    also lifts the centre by n g (cos phi - cos phi0). The tip's rise is (b/2)(sin phi0 - sin
    phi) plus that lift (cos gamma = 1)."""
    k, tau = value("ctrl.roll_pb2v", 0.08), value("ctrl.actuator_tau_s", 0.05)
    p_max = 2.0 * k * v / span
    phi0 = np.radians(bank_deg)
    t_start = (0.0 if delay is None else min(0.0, delay)) - dt
    t = np.arange(t_start, t_end, dt)
    rise = np.minimum(w * t, delta)               # the sea above the planned tip height
    phi = np.full_like(t, phi0)
    zc = np.zeros_like(t)
    gap = None
    if delay is not None:
        gap = float(max(0.0, -w * delay))
        p, ph, vz, z = 0.0, phi0, 0.0, 0.0
        for i in range(1, len(t)):
            if t[i] >= delay:
                p += (p_max - p) * dt / tau
            ph = max(ph - p * dt, 0.0)
            vz += n_load * G * (np.cos(ph) - np.cos(phi0)) * dt
            z += vz * dt
            phi[i], zc[i] = ph, z
    h = 0.5 * span * (np.sin(phi0) - np.sin(phi)) + zc - rise
    wet = h < 0.0
    return {"max_depth_m": float(max(0.0, -h.min())), "wet_s": float(wet.sum() * dt),
            "bank_change_deg": float(np.degrees(phi0 - phi[np.argmin(h)])), "gap_at_start_m": gap,
            "tip_rise_rate_ms": float(k * v * np.cos(phi0))}


def latest_start(bank_deg, v, span, n_load, delta, w, depth_max, latency):
    """Latest start of the roll-out (seconds after the first contact, negative before it) that
    keeps the immersion within depth_max, by bisection; the gap at that start, and the gap at
    which a range sensor must trigger when latency seconds pass from its reading to the start."""
    lo, hi = -1.0, 1.0
    f = lambda dl: reflex_budget(bank_deg, v, span, n_load, delta, w, dl)["max_depth_m"] - depth_max
    if f(hi) <= 0.0:
        return {"start_s": hi, "gap_m": 0.0, "trigger_gap_m": 0.0}
    if f(lo) > 0.0:
        return {"start_s": None, "gap_m": None, "trigger_gap_m": None}
    for _ in range(40):
        mid = 0.5 * (lo + hi)
        lo, hi = (mid, hi) if f(mid) <= 0.0 else (lo, mid)
    return {"start_s": lo, "gap_m": max(0.0, -w * lo), "trigger_gap_m": max(0.0, -w * (lo - latency))}


SKIDS = ("blade", "ski", "bare tip", "float")
DEPTHS_M = (0.005, 0.01, 0.02, 0.03)
DELTAS_M = (0.05, 0.1, 0.2, 0.3)
RISE_RATES_MS = (0.5, 1.0, 2.0)
DELAYS_S = (0.01, 0.03, 0.06, 0.1, 0.2)   # contact to the start of the roll
DEPTH_LIMITS_M = (0.01, 0.02, 0.03)


def _cells(path=SKIM_PATH):
    """The skid-touch cell (wingtip clearance 0, flat wing) at the lowest floor that solved for
    each aircraft of the skim study."""
    with open(path) as fh:
        rows = json.load(fh)["rows"]
    out = {}
    for r in rows:
        if r.get("lever") or not r["success"] or r["h_tip_min"] != 0.0:
            continue
        if r["aircraft"] not in out or r["h_min"] < out[r["aircraft"]]["h_min"]:
            out[r["aircraft"]] = r
    return out


def aircraft_masses(name):
    """(total mass, wing mass, span, area, tip chord, thickness ratio) for a skim-study name."""
    from sim import wingopt as w
    if name == "albatross-3m":
        mm = w.mass_model(3.0, 25.0, w.RHO_A["base"])
        area, span, mass = 0.36, 3.0, 3.0
    else:
        from sim.formstudy import skim_aircraft
        d = skim_aircraft(name)[1]
        mm = w.basics(d)[0]
        area, span, mass = mm["area"], d.span, mm["mass"]
    croot = 2.0 * area / (span * 1.45)
    return mass, mm["m_wing"], span, area, 0.45 * croot, w.SPAR["tc"]


def study(path=OUT_PATH, skim_path=SKIM_PATH):
    from sim import record
    cells = _cells(skim_path)
    res = {}
    for name, c in cells.items():
        mass, m_wing, span, area, ctip, tc = aircraft_masses(name)
        inert = inertia(m_wing, span)
        v, vw, bank = c["V_at_tip_ms"], c["water_speed_at_tip_ms"], c["bank_at_tip_deg"]
        auth = authority(mass, span, area, v, inert["ixx"], inert["izz"])
        cl = mass * G * c["n_max"] / (0.5 * RHO_AIR * v * v * area)   # the low turn sits at n_max
        alpha = np.degrees(cl / auth["lift_slope"])
        forces = {}
        for d in DEPTHS_M:
            b0 = blade(d, vw)
            b5 = blade(d, vw, beta_deg=par("misalignment_deg"))
            fb = dict(b0, side=b5["side"], side_roll=b5["side"] * np.sin(np.radians(bank)))
            row = {"blade": fb, "ski": ski(d, vw), "bare tip": bare_tip(d, vw, ctip, tc, bank, alpha),
                   "float": tip_float(d, vw)}
            forces["%g" % d] = {k: dict(f, effects=contact_effects(f, span, mass, v, inert["ixx"],
                                                                    inert["izz"], auth))
                                for k, f in row.items()}
        budget, starts = [], []
        lat = par("sense_process_s") + value("ctrl.actuator_delay_s", 0.05)
        for w in RISE_RATES_MS:
            for delta in DELTAS_M:
                for delay in (None,) + DELAYS_S + (lat,):
                    r = reflex_budget(bank, v, span, c["n_max"], delta, w, delay)
                    budget.append(dict(r, rise_rate_ms=w, delta_m=delta, delay_s=delay,
                                       wet_sensor=delay == lat))
                for dmax in DEPTH_LIMITS_M:
                    ls = latest_start(bank, v, span, c["n_max"], delta, w, dmax, lat)
                    starts.append(dict(ls, rise_rate_ms=w, delta_m=delta, depth_max_m=dmax, latency_s=lat))
        res[name] = {"cell": {k: c[k] for k in ("h_min", "h_tip_min", "u10", "V_at_tip_ms", "water_speed_at_tip_ms",
                                                "bank_at_tip_deg", "tip_curvature_ms2", "time_within_0p1_s", "n_max")},
                     "mass": mass, "m_wing": m_wing, "span": span, "area": area, "tip_chord": ctip,
                     "alpha_low_turn_deg": float(alpha), "inertia": inert, "authority": auth,
                     "weight_n": mass * G, "forces": forces, "budget": budget, "latest_start": starts}
    out = {"meta": record.meta("python -m sim.skid", ["sim/skid.py", "sim/out/skim_study.json", "record/values.yaml"]),
           "air_drag_area_m2": {k: air_drag_area(k) for k in ("blade", "ski", "float")},
           "skid_mass_kg": {k: par(k + "_mass_kg") for k in ("blade", "ski", "float")}, "aircraft": res}
    with open(path, "w", newline="\n") as fh:
        json.dump(out, fh, indent=1)
        fh.write("\n")
    return out


if __name__ == "__main__":
    study()
