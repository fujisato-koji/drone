"""Export the wave library (sim/out/mpc_library_wave.json) for the viewer's regular swell.

python -m sim.export_wave_library writes sim/viewer/wave/index.js (window.WAVE_INDEX, small: the grid, the presets of
the wave wind, the constants of the oracle sea, the planner settings of the wave runs, the flat-sea reference powers,
the margins of the runs and the Python closed-loop results to compare with) and one file per sea and aircraft,
sim/viewer/wave/<preset>_hs<Hs>_tp<Tp>_<aircraft>.js, which adds its entries to window.WAVE_SEAS[<file key>]. A page
loads the index and then only the file of the sea in use (a script tag works from file://).

An entry keeps what the viewer's pilot and guide use (sim/viewer/wave.js): the sea and course fields, kind, power,
the crest ride with its family (rows h, theta, V, gamma, psi, cl, 7 significant digits) or the optimizer cycle's
nodes (t and theta to 6 significant digits, the rest to 5), in the order of the library (the pilot's ties go to the
first entry). A crest ride is steady, so its nodes are written once ("steady": the node values) and expanded by
expand_entry (and by wave.js in the same way). Only converged entries are written.
"""
import argparse
import hashlib
import json
import sys
from pathlib import Path

from . import record
from .aircraft import BASELINE, Actuators

_HERE = Path(__file__).resolve().parent
OUT_DIR = _HERE / "viewer" / "wave"
SIG_CYCLE = {"t": 6, "theta": 6}
SIG_FAMILY = 7
SIG_STEADY = 10
CYCLE_KEYS = ("t", "z", "V", "gamma", "psi", "cl", "mu", "thrust", "theta")
STEADY_KEYS = ("z", "V", "gamma", "psi", "cl", "mu", "thrust", "theta")
COMMAND = "python -m sim.export_wave_library"


def _sig(v, n):
    return float(f"{v:.{n}g}")


def _sigs(a, n):
    return [_sig(v, n) for v in a]


def model_files():
    from .wavelib import WAVE_LIB_PATH
    return [_HERE / "export_wave_library.py", Path(WAVE_LIB_PATH), _HERE / "wavelib.py", _HERE / "mpc.py",
            _HERE / "seawind.py", _HERE / "navigator.py", record.VALUES_PATH]


def file_key(preset, hs, tp, aircraft):
    """File key of one sea and aircraft, e.g. p63_hs3_tp10_albatross-3m."""
    f = lambda v: ("%g" % v).replace(".", "p")
    return "%s_hs%s_tp%s_%s" % (preset, f(hs), f(tp), aircraft)


def slim_entry(e):
    """The fields of a library entry that the viewer uses (see the module docstring)."""
    d = {k: e[k] for k in ("aircraft", "preset", "hs", "tp", "direction", "rel_deg", "u10", "u10_flown", "kind")}
    d["course_rel_deg"] = _sig(e["course_rel_deg"], 10)
    d["crest"] = bool(e.get("crest"))
    d["success"] = True
    d["period"] = _sig(e["period"], 6)
    d["m"] = int(e.get("m", 0))
    d["power_avg_w"] = _sig(float(e.get("power_avg_w", 0.0)), 6)
    d["mean_ground_velocity"] = _sigs(e["mean_ground_velocity"], 6)
    rp = e.get("replay") or {}
    d["replay"] = {"rc_h_min": None if rp.get("rc_h_min") is None else _sig(rp["rc_h_min"], 6)}
    if d["crest"]:
        r = e["ride"]
        ride = {k: _sig(r[k], SIG_STEADY) for k in ("v_c", "h", "theta", "V", "gamma", "psi", "cl", "c") if k in r}
        for k in ("thrust_n", "power_w"):
            if k in r:
                ride[k] = _sig(r[k], SIG_STEADY)
        if r.get("family"):
            ride["family"] = [_sigs(row, SIG_FAMILY) for row in r["family"]]
        d["ride"] = ride
        n = len(e["t"])
        for k in STEADY_KEYS:            # a crest ride is steady: one value per array
            if any(v != e[k][0] for v in e[k]):
                raise ValueError("crest entry with a varying %s" % k)
        d["n_nodes"] = n
        d["steady"] = {k: _sig(e[k][0], SIG_STEADY) for k in STEADY_KEYS}
    else:
        for k in CYCLE_KEYS:
            d[k] = _sigs(e[k], SIG_CYCLE.get(k, 5))
    return d


def expand_entry(d):
    """A slim entry with its node arrays (a crest ride's steady values repeated over its nodes, at the times
    P i / (n - 1) of sim.wavelib._crest_entry). The same expansion as wave.js expandEntry."""
    if "steady" not in d:
        return d
    e = dict(d)
    n, P = d["n_nodes"], d["period"]
    e["t"] = [P * i / (n - 1) for i in range(n)]
    for k, v in d["steady"].items():
        e[k] = [v] * n
    return e


def flat_references(names):
    """Per aircraft, [u10, least average power (W), kind] of sim.wavelib.flat_reference at each wind of the flat
    motor library: the pilot does not fly a motor wave entry that needs at least this much."""
    from .mpc import CycleLibrary
    from .wavelib import flat_reference
    lib = CycleLibrary.from_keyword("motor")
    out = {}
    for name in names:
        us = sorted({e["u10"] for e in lib.entries if e["aircraft"] == name})
        rows = []
        for u in us:
            f = flat_reference(lib.entries, name, u)
            rows.append([float(u), float(f["power_avg_w"]), f["kind"]])
        out[name] = rows
    return out


def python_runs():
    """The closed-loop runs of --wave-loop-exp (sim/out/mpc.json, key wave_loop_exp) in the fields the viewer
    shows beside its own flight, or [] when the file has none."""
    p = _HERE / "out" / "mpc.json"
    try:
        with open(p) as fh:
            blk = json.load(fh).get("wave_loop_exp") or {}
    except (OSError, ValueError):
        return [], None
    rows = []
    for r in blk.get("results") or []:
        if r.get("planner_sea_error"):
            continue
        a = r.get("after_entry") or {}
        rows.append({"aircraft": r["aircraft"], "hs": r["hs"], "tp": r["tp"], "direction": r["direction"],
                     "u10": r["u10"], "preset": r["preset"], "ended": r["ended"], "aloft_s": r["survival_s"],
                     "counted": bool(r["margins"].get("counts")),
                     "power_w": round(r["motor"]["avg_power_w"], 2),
                     "power_after_w": None if not a else round(a["avg_power_w"], 2),
                     "motor_share": round(r["motor"]["thrust_on_share"], 3),
                     "motor_share_after": None if not a else round(a["thrust_on_share"], 3),
                     "height_median_m": round(r["margins"]["height_median_m"], 2),
                     "t_entry_s": None if not a else a["t_entry_s"],
                     "entry": r.get("entry")})
    meta = blk.get("meta") or {}
    return rows, {k: meta.get(k) for k in ("tdr_run", "commit", "stamp", "t_end")}


def build(path=None):
    """(index dict, {file key: [slim entries]})."""
    from .mpc import (CREST_CLIMB_M, NAV_TIP_MARGIN_M, WAVE_CREST_SIG, WAVE_CREST_SIG_THETA, WAVE_GUIDE_SIG,
                      WAVE_SIG_THETA, WAVE_SWITCH_MAX_WAIT_S, WAVE_SWITCH_THETA, WAVE_W_RUN, BURST_ON_FRAC,
                      energy_weight)
    from .seawind import (COARE35_INTERCEPT, COARE35_MAX, COARE35_SLOPE, G as G_SEA, KARMAN, NU_AIR)
    from .wavelib import (CALM_U10, PRESETS, SWELL_DIRS, WAVE_AIRCRAFT, WAVE_HS, WAVE_LIB_PATH, WAVE_TP,
                          preset_amp_decay, wave_pkw)
    from .aircraft_models import by_name
    path = WAVE_LIB_PATH if path is None else path
    with open(path) as fh:
        lib = json.load(fh)
    files = {}
    for e in lib["entries"]:
        if not e.get("success"):
            continue
        files.setdefault(file_key(e["preset"], e["hs"], e["tp"], e["aircraft"]), []).append(slim_entry(e))
    a = Actuators(BASELINE, rate_model="pb2v")
    names = {"n_samples": "nSamples", "dt": "dt", "n_iter": "nIter", "n_ctrl": "nCtrl", "lam": "lam",
             "sigma_cl": "sigmaCl", "sigma_mu": "sigmaMu", "clearance": "clearance", "h_clear": "hClear",
             "thrust_sampling": "thrustSampling", "fold_switch": "foldSwitch", "thrust_update": "thrustUpdate",
             "h_ceil": "hCeil", "smooth_modes": "smoothModes"}
    pkw = wave_pkw()
    planner = {names[k]: v for k, v in pkw.items()}
    planner.update({"motor": True, "energyLevel": "normal"})
    runs, runs_meta = python_runs()
    m = lib.get("meta") or {}
    cr = m.get("crest_rebuild") or {}
    seas = {}
    for k, es in files.items():
        e0 = es[0]
        seas[k] = {"preset": e0["preset"], "hs": e0["hs"], "tp": e0["tp"], "aircraft": e0["aircraft"],
                   "file": k + ".js", "entries": len(es),
                   "optimizer": sum(1 for x in es if not x["crest"]),
                   "sha": hashlib.sha256(json.dumps(es, separators=(",", ":")).encode()).hexdigest()[:12]}
    index = {
        "source": {"path": "sim/out/mpc_library_wave.json", "tdr_run": m.get("tdr_run"), "commit": m.get("commit"),
                   "stamp": m.get("stamp"), "crest_rebuild": {k: cr.get(k) for k in ("tdr_run", "commit", "stamp")}},
        "grid": {"aircraft": list(WAVE_AIRCRAFT), "hs": sorted(WAVE_HS), "tp": sorted(WAVE_TP),
                 "directions": SWELL_DIRS, "u10_flown": sorted({e["u10_flown"] for e in lib["entries"]}),
                 "calm_u10": CALM_U10},
        "presets": {p: {"amp": list(preset_amp_decay(p)[0]), "decay": list(preset_amp_decay(p)[1])}
                    for p in PRESETS},
        "sea": {"g": G_SEA, "karman": KARMAN, "nu_air": NU_AIR, "coare35": [COARE35_SLOPE, COARE35_INTERCEPT,
                                                                         COARE35_MAX],
                "from_deg": 270.0, "surface": "stokes2"},
        "guide": {"sig": list(WAVE_GUIDE_SIG), "sig_theta": WAVE_SIG_THETA, "crest_sig": list(WAVE_CREST_SIG),
                  "crest_sig_theta": WAVE_CREST_SIG_THETA, "w_run": WAVE_W_RUN, "crest_climb_m": CREST_CLIMB_M,
                  "switch_theta": WAVE_SWITCH_THETA, "switch_max_wait_s": WAVE_SWITCH_MAX_WAIT_S},
        "planner": planner,
        "w_energy": {n: energy_weight(by_name(n), "normal") for n in WAVE_AIRCRAFT},
        "actuators": {"rate_model": "pb2v", "tau_s": a.tau, "cl_rate_per_s": a.cl_rate, "k_pb2v": a.k_pb2v,
                      "delay_s": a.delay, "thrust_tau_s": a.thrust_tau},
        "flat": flat_references(WAVE_AIRCRAFT),
        "margins": {"band_m": [record.value("ds.clearance_min_m"), record.value("ds.altitude_band_max_m")],
                    "tip_m": NAV_TIP_MARGIN_M, "motor_on_frac": BURST_ON_FRAC},
        "runs": runs, "runs_meta": runs_meta,
        "seas": seas}
    return index, files


def write(out_dir=OUT_DIR, path=None):
    out_dir = Path(out_dir)
    out_dir.mkdir(parents=True, exist_ok=True)
    index, files = build(path)
    head = record.header(COMMAND, model_files())
    for k, es in files.items():
        with open(out_dir / (k + ".js"), "w", encoding="utf-8", newline="\n") as fh:
            fh.write(head + "\n")
            fh.write("(window.WAVE_SEAS = window.WAVE_SEAS || {})[" + json.dumps(k) + "] = "
                     + json.dumps(es, separators=(",", ":")) + ";\n")
    with open(out_dir / "index.js", "w", encoding="utf-8", newline="\n") as fh:
        fh.write(head + "\n")
        fh.write("window.WAVE_INDEX = " + json.dumps(index, separators=(",", ":")) + ";\n")
    keep = {k + ".js" for k in files} | {"index.js"}
    for p in out_dir.glob("*.js"):       # a sea that left the library
        if p.name not in keep:
            p.unlink()
    sizes = sorted((p.stat().st_size, p.name) for p in out_dir.glob("*.js"))
    return index, sizes


def load_js(path):
    """The JSON object of an exported file (index or sea)."""
    text = Path(path).read_text(encoding="utf-8")
    body = text.rsplit(" = ", 1)[1].rsplit(";", 1)[0]      # compact JSON has no " = "
    return json.loads(body)


def main(argv=None):
    ap = argparse.ArgumentParser()
    ap.add_argument("--out-dir", default=str(OUT_DIR))
    a = ap.parse_args(argv)
    index, sizes = write(a.out_dir)
    n = sum(s["entries"] for s in index["seas"].values())
    print("wrote %d entries in %d sea files; index %d bytes, largest sea file %d bytes (%s), total %d bytes"
          % (n, len(index["seas"]), next(s for s, nm in sizes if nm == "index.js"), sizes[-1][0], sizes[-1][1],
             sum(s for s, _ in sizes)))


if __name__ == "__main__":
    sys.exit(main())
