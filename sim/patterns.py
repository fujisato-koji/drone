"""Library of albatross-like flight patterns, with the models needed to pick one.

Sections: (1) the literature catalogue PATTERNS, (2) wave-slope soaring, (3) flap-glide
(motor-glide) energy, (4) the pattern selector, (5) the printed and JSON outputs.

Conventions follow sim.seawind (x east, y north, z up; compass degrees) and
sim.aircraft (state [x, y, z, V, gamma, psi], psi = atan2(y, x)).
"""
import json
from pathlib import Path

import numpy as np

from sim import record
from sim.aircraft import BASELINE, G, simulate
from sim.seawind import Environment, WaveTrain, Wind, compass_to_vector

ROOT = Path(__file__).resolve().parents[1]
OUT_PATH = ROOT / "sim" / "out" / "patterns.json"
_SIM = Path(__file__).resolve().parent
# files the output depends on: the stamp covers them, in this order
MODEL_FILES = [_SIM / "patterns.py", _SIM / "aircraft.py", _SIM / "aircraft_models.py",
               _SIM / "seawind.py", record.VALUES_PATH]


def tdr_value(key, default=None):
    """Value of a record key through sim.record; default (None if not given) when the key is absent."""
    try:
        return record.value(key, default)
    except KeyError:
        return None


# Minimum U10 for dynamic soaring: record value ds.min_u10_ms (read once at import), else 8.
U10_MIN_DS = record.value("ds.min_u10_ms", 8.0)

# ---------------------------------------------------------------- 1. catalogue

PATTERNS = [
    {
        "name": "ds_travel_crosswind",
        "energy_source": "wind shear",
        "when": "U10 above about 4 to 9 m/s",
        "geometry": "dynamic soaring S-pattern across the wind, shallow arcs, flown like a yacht",
        "parameters": {"u10_min_ms": (4, 9), "cycle_s": (10, 15), "height_m": (0, 15),
                       "turn_angle_median_deg": (54, 78), "preferred_heading_off_wind_deg": 83},
        "dsopt_mode": "max_speed, travel, chi 90 deg",
        "sources": ["P42", "P44", "P46", "P47", "P73"],  # P73 = Goto 2024
    },
    {
        "name": "ds_upwind_zigzag",
        "energy_source": "wind shear",
        "when": "bird needs at least about 3.6 m/s of wind; progress upwind by tacking",
        "geometry": "dynamic soaring tacking upwind (zigzag)",
        "parameters": {"min_wind_ms": 3.6,
                       "upwind_ground_speed_ms_at_wind": {"3.6": 8.4, "7": 12.1}},
        "dsopt_mode": "max_speed, travel, chi 0 deg",
        "sources": ["P43"],
    },
    {
        "name": "ds_turn_angle_by_wind",
        "energy_source": "wind shear",
        "when": "turn angle depends on the wind: falls as the wind rises",
        "geometry": "low shear gives serpentine paths, high shear flatter ones",
        "parameters": {"turn_angle_light_wind_deg": 137, "turn_angle_strong_wind_deg": 15},
        "dsopt_mode": "max_speed, travel, turn angle free",
        "sources": ["P74", "P75"],  # P74 Richardson and Wakefield 2022, P75 Chen 2025
    },
    {
        "name": "ds_loiter",
        "energy_source": "wind shear",
        "when": "search or station keeping over one area",
        "geometry": "closed O-loops over one area",
        "parameters": {},
        "dsopt_mode": "min_wind, loiter",
        "sources": ["P44"],
    },
    {
        "name": "wave_slope",
        "energy_source": "updraft ahead of a moving wave crest",
        "when": "dominant in light wind with large swell",
        "geometry": "slope soaring in the updraft ahead of the crest, flying along the crest",
        "parameters": {"shear_share_typical_pct": (80, 90),
                       "pelican_cost_of_transport_reduction_pct": (60, 70),
                       "pelican_model_height_m": 2, "pelican_model_wave_period_s": 15},
        "dsopt_mode": None,
        # P76 Pennycuick 1982 (abstract only); P42 shear 80 to 90% in typical conditions,
        # wave slope dominant in weak wind and big swell; P77 Stokes and Lucas 2021 (model).
        "sources": ["P76", "P42", "P77"],
    },
    {
        "name": "gust",
        "energy_source": "gusts and turbulence",
        "when": "proposed as a basis of petrel flight",
        "geometry": "uses gusts and turbulence (no geometry given)",
        "parameters": {},
        "dsopt_mode": None,
        "sources": ["P78"],  # Pennycuick 2002, not read
    },
    {
        "name": "flap_glide",
        "energy_source": "flapping bouts (motor for the drone) between glides",
        "when": "light or head winds",
        "geometry": "glides with short flapping bouts; for the drone the motor replaces flapping",
        "parameters": {"flapping_time_share_pct": 4.6,
                       "flapping_rate_fall_pct": (89, 93)},
        # black-browed albatross: energy mostly in the ~4.6% of time spent flapping;
        # flapping rate falls 89 to 93% with wind (Southern Ocean) and swell height (North Pacific)
        "dsopt_mode": None,
        "sources": ["P79", "P80", "P81"],  # Kempton 2022, Maywar 2025, Sakamoto 2013
    },
    {
        "name": "rest_on_water",
        "energy_source": "none (sits on the water)",
        "when": "wind too weak; take off when it returns",
        "geometry": "sit on the water",
        "parameters": {"takeoff_run_s_at_wind_2ms": {"calm_sea": 8, "high_waves": 4}},
        "dsopt_mode": None,
        "sources": ["P82"],  # Uesaka 2023: run and flapping fall with wind speed and wave height
    },
]


def patterns_table():
    """The catalogue as a markdown table."""
    rows = ["| pattern | energy source | when | dsopt mode | sources |",
            "|---|---|---|---|---|"]
    for p in PATTERNS:
        rows.append("| %s | %s | %s | %s | %s |" % (
            p["name"], p["energy_source"], p["when"], p["dsopt_mode"] or "-",
            ", ".join(p["sources"])))
    return "\n".join(rows)


# ---------------------------------------------------------------- 2. wave-slope soaring

class _SingleSwell(WaveTrain):
    """One component: a = hs/2, k = (2 pi/tp)^2/g, travelling to to_deg, phase 0."""

    def components(self, rng):
        om = 2.0 * np.pi / self.tp
        dx, dy = compass_to_vector(self.to_deg)
        return {"a": np.array([self.hs / 2.0]), "k": np.array([om * om / G]),
                "omega": np.array([om]), "dx": np.array([dx]), "dy": np.array([dy]),
                "phase": np.array([0.0])}


def swell_environment(hs, tp, u10=0.5, wind_from=270.0, swell_to=90.0, decay=(1.0, 1.0)):
    return Environment(Wind(u10, wind_from), [_SingleSwell(hs, tp, swell_to)], decay=decay)


def sink_rate(ac, v):
    """Sink rate (m/s) in level-flight lift at airspeed v on the parabolic polar (cl capped at cl_max)."""
    v = np.asarray(v, dtype=float)
    cl = np.minimum(2.0 * ac.mass * G / (ac.rho * ac.wing_area * v * v), ac.cl_max)
    return v * (ac.cd0 + ac.k_induced * cl * cl) / cl


def wave_slope_sustainable(aircraft, hs, tp, u10=0.5, wind_from=270, swell_to=90,
                           h=None, decay=(1.0, 1.0), sink_override=None):
    """Best point in the updraft ahead of a swell crest, and whether it sustains flight.

    Single long-crested swell (a = hs/2, k = (2 pi/tp)^2/g, to swell_to) in a light wind.
    In the frame moving with the crest the field is steady; w(phase, zloc) is read from
    env.wind_at at t = 0 over phases 0 to pi (the front face) and heights zloc 0.5 to 6 m
    (only h if given). The aircraft flies straight (mu = 0) along the crest while keeping
    pace with it, so its ground speed along the wave direction is the phase speed
    c = g tp/(2 pi) and its airspeed is V = max(speed of minimum sink, 1.1 stall speed,
    |c - wind along the wave|). Surplus = w - sink(V). Wingtip clearance
    zloc - 0.5 span sin(mu) >= 0.3 is enforced (mu = 0 here).

    sink_override replaces the polar sink rate (a number, m/s), e.g. 0.0 for an ideal glider.

    CAVEAT: the potential-flow perturbation of seawind.py stands in for the real flow over
    waves; it is to be verified (Q-008, Q-011).
    """
    env = swell_environment(hs, tp, u10, wind_from, swell_to, decay)
    k = float(env.c["k"][0])
    ewx, ewy = compass_to_vector(swell_to)
    c = G * tp / (2.0 * np.pi)
    ph = np.linspace(0.0, np.pi, 37)
    zl = np.linspace(0.5, 6.0, 12) if h is None else np.array([float(h)])
    PH, ZL = np.meshgrid(ph, zl, indexing="ij")
    eta = 0.5 * hs * np.cos(PH)
    wv = np.asarray(env.wind_at((PH / k) * ewx, (PH / k) * ewy, eta + ZL, np.zeros_like(PH)))
    w = wv[..., 2]
    uw = wv[..., 0] * ewx + wv[..., 1] * ewy
    v_ms = ac_v_min_sink(aircraft)
    v = np.maximum(np.maximum(v_ms, 1.1 * aircraft.v_stall), np.abs(c - uw))
    sink = sink_rate(aircraft, v) if sink_override is None else np.full_like(v, float(sink_override))
    mu = 0.0
    ok = (ZL - 0.5 * aircraft.span * np.sin(mu)) >= 0.3
    surplus = np.where(ok, w - sink, -np.inf)
    i, j = np.unravel_index(int(np.argmax(surplus)), surplus.shape)
    best = float(surplus[i, j])
    return {"surplus": best, "phase": float(ph[i]), "phase_deg": float(np.degrees(ph[i])),
            "zloc": float(zl[j]), "w_up": float(w[i, j]), "w_max": float(w.max()),
            "sink": float(sink[i, j]), "airspeed": float(v[i, j]),
            "sustainable": bool(best > 0.0), "c": c, "hs": hs, "tp": tp, "u10": u10}


def ac_v_min_sink(ac):
    return ac.min_sink[0]


class _SlopeController:
    """Holds the crest phase (heading across the crest) and the height above the surface (cl)."""

    KX = 0.3   # 1/s, phase position gain
    KH = 0.5   # 1/s, height gain
    KG = 2.0   # cl per rad of flight path angle error (as in sim.dsearch)

    def __init__(self, sw, phase, zloc, swell_to):
        self.k = float(sw["k"])
        self.c = sw["c"]
        self.xi_t = phase / self.k
        self.zloc = zloc
        self.ew = np.array(compass_to_vector(swell_to))
        self.ec = np.array([-self.ew[1], self.ew[0]])

    def __call__(self, t, s, env, ac):
        x, y, z, V, g, psi = s
        wind = np.asarray(env.wind_at(x, y, z, t))
        xi = x * self.ew[0] + y * self.ew[1] - self.c * t
        lam = 2.0 * np.pi / self.k
        err = (self.xi_t - xi + lam / 2.0) % lam - lam / 2.0
        uw = wind[0] * self.ew[0] + wind[1] * self.ew[1]
        va_w = float(np.clip(self.c - uw + self.KX * err, -0.97 * V, 0.97 * V))
        vc = float(np.sqrt(max(V * V - va_w * va_w, 0.0)))
        d = va_w * self.ew + vc * self.ec
        psi_des = np.arctan2(d[1], d[0])
        e_psi = (psi_des - psi + np.pi) % (2.0 * np.pi) - np.pi
        mu = float(np.arctan(V * e_psi / G))
        h = z - float(env.surface(x, y, t)[0])
        vz_des = float(np.clip(self.KH * (self.zloc - h), -1.0, 1.0))
        sg = float(np.clip((vz_des - wind[2]) / V, -0.3, 0.3))
        cl = 2.0 * ac.mass * G / (ac.rho * ac.wing_area * V * V) + self.KG * (np.arcsin(sg) - g)
        return cl, mu


def simulate_wave_slope(aircraft, hs, tp, u10=0.5, wind_from=270, swell_to=90,
                        h=None, decay=(1.0, 1.0), t_end=120.0, dt=0.05):
    """Fly the best wave-slope point for t_end s; return energy change per minute and min height.

    Energy is the aircraft's total specific energy e_air = z + V^2/2g (air-relative speed)
    from sim.aircraft; energy_j_per_min = m g d(e_air)/dt. Starts on the target phase and height
    at V = 1.1 * the required airspeed.
    """
    sw = wave_slope_sustainable(aircraft, hs, tp, u10, wind_from, swell_to, h, decay)
    env = swell_environment(hs, tp, u10, wind_from, swell_to, decay)
    sw["k"] = float(env.c["k"][0])
    ctrl = _SlopeController(sw, sw["phase"], sw["zloc"], swell_to)
    ew = ctrl.ew
    v0 = 1.1 * sw["airspeed"]
    psi0 = float(np.arctan2(ctrl.c * ew[1] + ctrl.ec[1] * np.sqrt(max(v0 ** 2 - ctrl.c ** 2, 0)),
                            ctrl.c * ew[0] + ctrl.ec[0] * np.sqrt(max(v0 ** 2 - ctrl.c ** 2, 0))))
    x0, y0 = ctrl.xi_t * ew[0], ctrl.xi_t * ew[1]
    z0 = float(env.surface(x0, y0, 0.0)[0]) + sw["zloc"]
    r = simulate(aircraft, env, ctrl, [x0, y0, z0, v0, 0.0, psi0], 0.0, t_end, dt=dt)
    dur = float(r["t"][-1] - r["t"][0])
    de = float(r["e_air"][-1] - r["e_air"][0])
    return {"energy_j_per_min": aircraft.mass * G * de / (dur / 60.0) if dur > 0 else 0.0,
            "height_equiv_m_per_min": de / (dur / 60.0) if dur > 0 else 0.0,
            "min_h": float(r["h"].min()), "duration_s": dur,
            "completed": bool(dur >= t_end - 1e-6), "surplus": sw["surplus"],
            "phase_deg": sw["phase_deg"], "zloc": sw["zloc"]}


HS_GRID = (1.0, 2.0, 3.0, 4.0)
TP_GRID = (8.0, 12.0, 15.0)
U10_GRID = (0.5, 3.0)


def wave_slope_table(candidates=None, run_sim=True):
    """Rows for every candidate x hs x tp x u10; sustainable cases are also flown."""
    if candidates is None:
        from sim.aircraft_models import candidates as _cands
        candidates = _cands()
    rows = []
    for ac in candidates:
        for u10 in U10_GRID:
            for hs in HS_GRID:
                for tp in TP_GRID:
                    r = wave_slope_sustainable(ac, hs, tp, u10=u10)
                    row = {"candidate": ac.name, "hs": hs, "tp": tp, "u10": u10,
                           "surplus": r["surplus"], "sustainable": r["sustainable"],
                           "zloc": r["zloc"], "phase_deg": r["phase_deg"], "c": r["c"],
                           "airspeed": r["airspeed"]}
                    if run_sim and r["sustainable"]:
                        s = simulate_wave_slope(ac, hs, tp, u10=u10)
                        row.update({"sim_energy_j_per_min": s["energy_j_per_min"],
                                    "sim_min_h": s["min_h"], "sim_completed": s["completed"]})
                    rows.append(row)
    return rows


# ---------------------------------------------------------------- 3. flap-glide

def flap_glide_cost(aircraft, u10, heading_rel_wind_deg, prop_eff=0.5, motor_eff=0.8,
                    burst_fraction=None):
    """Steady glide at best L/D speed, motor bursts make up the sink.

    Average electric power to hold height P = m g sink / (prop_eff motor_eff), sink = V/(L/D).
    heading_rel_wind_deg is the track angle from the direction the wind comes from
    (0 = headwind, 90 = crosswind, 180 = tailwind); the wind is the constant u10 and the
    airspeed stays at best L/D. Ground speed along the track = sqrt(V^2 - crosswind^2) -
    headwind (crab into the crosswind). If it is not positive, Wh/km is None (cannot make good).
    burst_fraction = share of time the motor runs (default 0.1); the burst power is P/fraction.
    """
    v = aircraft.v_best_ld
    sink = v / aircraft.best_ld
    p = aircraft.mass * G * sink / (prop_eff * motor_eff)
    th = np.radians(heading_rel_wind_deg)
    uh, uc = u10 * np.cos(th), abs(u10 * np.sin(th))
    gs = float(np.sqrt(v * v - uc * uc) - uh) if v > uc else 0.0
    bf = 0.1 if burst_fraction is None else burst_fraction
    avionics = tdr_value("ds.avionics_power_w", None)
    out = {"airspeed": v, "sink": sink, "power_w": p, "ground_speed": gs,
           "wh_per_km": (p / gs) * 1000.0 / 3600.0 if gs > 0.05 else None,
           "burst_fraction": bf, "burst_power_w": p / bf, "avionics_w": avionics}
    if avionics is not None and gs > 0.05:
        out["wh_per_km_with_avionics"] = ((p + avionics) / gs) * 1000.0 / 3600.0
    return out


# Assumptions for the battery and solar comparison (not in the record).
BATTERY_MASS_FRACTION = 0.15
SOLAR_CELL_EFF = 0.20
SOLAR_COVERAGE = 0.5  # fraction of the wing area covered by cells


def flap_glide_table(candidates=None):
    if candidates is None:
        from sim.aircraft_models import candidates as _cands
        candidates = _cands()
    wh_kg = tdr_value("lit.cell_p45b_wh_per_kg", None)
    ins = tdr_value("lit.insolation_50s_june_kwh_m2_day", None)
    rows = []
    for ac in candidates:
        batt = BATTERY_MASS_FRACTION * ac.mass * wh_kg if wh_kg else None
        solar_w = (ac.wing_area * SOLAR_COVERAGE * SOLAR_CELL_EFF * ins * 1000.0 / 24.0
                   if ins else None)
        for u10 in (0.0, 5.0, 10.0):
            for name, ang in (("headwind", 0.0), ("crosswind", 90.0)):
                if u10 == 0.0 and name == "crosswind":
                    continue
                r = flap_glide_cost(ac, u10, ang)
                rows.append({"candidate": ac.name, "u10": u10, "track": name,
                             "ground_speed": r["ground_speed"], "power_w": r["power_w"],
                             "wh_per_km": r["wh_per_km"],
                             "range_km": (batt / r["wh_per_km"]
                                          if batt and r["wh_per_km"] else None),
                             "battery_wh": batt, "solar_avg_w": solar_w,
                             "avionics_w": r["avionics_w"]})
    return rows


# ---------------------------------------------------------------- 4. selector

def select_pattern(u10, hs_swell, tp_swell, aircraft, u10_min_ds=None, battery_wh=0.0,
                   leg_km=10.0, hold_station=False):
    """Pattern the literature suggests; returns (name, reason).

    u10 >= u10_min_ds: ds_travel_crosswind (ds_loiter when hold_station);
    elif wave-slope sustainable for the swell: wave_slope;
    elif the battery covers a leg_km crosswind flap-glide leg: flap_glide;
    else rest_on_water.
    """
    umin = U10_MIN_DS if u10_min_ds is None else u10_min_ds
    if u10 >= umin:
        name = "ds_loiter" if hold_station else "ds_travel_crosswind"
        return name, "u10 %.1f >= dynamic soaring minimum %.1f m/s" % (u10, umin)
    ws = wave_slope_sustainable(aircraft, hs_swell, tp_swell, u10=max(u10, 0.5))
    if ws["sustainable"]:
        return "wave_slope", ("u10 %.1f < %.1f m/s; wave-slope surplus %.2f m/s (hs %.1f m, "
                              "tp %.0f s, %.1f m/s updraft at zloc %.1f m)" % (
                                  u10, umin, ws["surplus"], hs_swell, tp_swell,
                                  ws["w_up"], ws["zloc"]))
    fg = flap_glide_cost(aircraft, u10, 90.0)
    need = leg_km * fg["wh_per_km"] if fg["wh_per_km"] else float("inf")
    if battery_wh and battery_wh >= need:
        return "flap_glide", ("wave-slope surplus %.2f m/s <= 0; battery %.1f Wh >= %.1f Wh "
                              "for a %.0f km crosswind leg (%.2f Wh/km)" % (
                                  ws["surplus"], battery_wh, need, leg_km, fg["wh_per_km"]))
    return "rest_on_water", ("u10 %.1f < %.1f m/s; wave-slope surplus %.2f m/s <= 0; battery "
                             "%.1f Wh < %.1f Wh for %.0f km" % (
                                 u10, umin, ws["surplus"], battery_wh or 0.0, need, leg_km))


def selector_sweep(aircraft=BASELINE, tp_swell=10.0):
    wh_kg = tdr_value("lit.cell_p45b_wh_per_kg", None) or 231.0
    batt = BATTERY_MASS_FRACTION * aircraft.mass * wh_kg
    rows = []
    for u10 in (2, 4, 6, 8, 10, 12):
        for hs in (0.5, 2, 4):
            for label, b in (("battery", batt), ("no_battery", 0.0)):
                name, reason = select_pattern(u10, hs, tp_swell, aircraft, battery_wh=b)
                rows.append({"u10": u10, "hs_swell": hs, "tp_swell": tp_swell,
                             "battery": label, "pattern": name, "reason": reason})
    return rows


# ---------------------------------------------------------------- 5. outputs

def _clean(o):
    if isinstance(o, dict):
        return {str(k): _clean(v) for k, v in o.items()}
    if isinstance(o, (list, tuple)):
        return [_clean(v) for v in o]
    if isinstance(o, (np.floating, float)):
        return None if not np.isfinite(o) else float(f"{float(o):.6g}")
    if isinstance(o, (np.bool_,)):
        return bool(o)
    if isinstance(o, np.integer):
        return int(o)
    return o


def main():
    print(patterns_table())
    ws = wave_slope_table()
    print("\nwave slope (candidate, hs, tp, u10, surplus m/s, sustainable, sim J/min)")
    for r in ws:
        print("%-16s %.0f %4.0f %.1f %7.2f %-5s %s" % (
            r["candidate"], r["hs"], r["tp"], r["u10"], r["surplus"], r["sustainable"],
            "%.1f" % r["sim_energy_j_per_min"] if "sim_energy_j_per_min" in r else "-"))
    fg = flap_glide_table()
    print("\nflap glide (candidate, u10, track, ground m/s, W, Wh/km, range km)")
    for r in fg:
        f = lambda v, fmt: fmt % v if v is not None else "-"
        print("%-16s %4.1f %-9s %5.2f %6.1f %8s %8s" % (
            r["candidate"], r["u10"], r["track"], r["ground_speed"], r["power_w"],
            f(r["wh_per_km"], "%.2f"), f(r["range_km"], "%.1f")))
    sw = selector_sweep()
    print("\nselector sweep, baseline (u10, hs_swell, battery, pattern)")
    for r in sw:
        print("%4.0f %4.1f %-10s %s" % (r["u10"], r["hs_swell"], r["battery"], r["pattern"]))
    OUT_PATH.parent.mkdir(parents=True, exist_ok=True)
    with open(OUT_PATH, "w", newline="\n") as fh:
        json.dump(_clean({"meta": record.meta("python -m sim.patterns", MODEL_FILES),
                          "patterns": PATTERNS, "u10_min_ds": U10_MIN_DS, "wave_slope": ws,
                          "flap_glide": fg, "selector_sweep": sw,
                          "assumptions": {"battery_mass_fraction": BATTERY_MASS_FRACTION,
                                          "solar_cell_eff": SOLAR_CELL_EFF,
                                          "solar_coverage": SOLAR_COVERAGE}}),
                  fh, indent=1)
        fh.write("\n")


if __name__ == "__main__":
    main()
