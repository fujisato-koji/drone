"""Studies of the near-surface options of the sea and wind model (Q-064, Q-061).

python -m sim.seawind_fitness [--out sim/out/seawind_fitness.json] [--only crest|shear|turb|breaking]

Three studies, all with the options of sim/seawind.py:
- crest: crest heights over 10 minutes on a 200 m grid, linear against stokes2 surface, for a
  fully developed sea at u10 10 and 15 m/s and for the swell of the storm scenario at hour 42,
  against a Rayleigh law with the sigma of the same series and the Forristall 3-D crest
  distribution at P = 1e-2 and 1e-3.
- breaking: fraction of the surface marked by the breaking model at u10 7, 10, 15 against the
  whitecap cover of Callaghan et al. 2008.
- shear: wind difference between 1 and 15 m and between 1.5 and 19 m for neutral and stable and
  unstable air, and for the log, swell_knee and swell_jet profiles.
- turb: sigma and the integral length scale of the vertical and the longitudinal turbulence at 1, 2, 5, 10 m,
  turbulence_profile "fixed" against "height".
"""
from __future__ import annotations

import argparse
import json
from pathlib import Path

import numpy as np

from . import record
from .export_field import STORM
from .sea_history import SeaHistory, WindHistory
from .seawind import (TURB_L, Environment, WaveTrain, Wind, _turb_amp, forristall_crest, obukhov_length,
                      turbulence_length_scales, whitecap_callaghan)

HERE = Path(__file__).resolve().parent
FILES = [HERE / "seawind_fitness.py", HERE / "seawind.py", HERE / "sea_history.py", HERE / "export_field.py",
         record.VALUES_PATH]
REGION = (0.0, 200.0, 0.0, 200.0)


def _r(v, n=4):
    return float(f"{v:.{n}g}")


def crest_study(duration=600.0, spacing=25.0, dt=0.4):
    seas = []
    for u10 in (10.0, 15.0):
        w = Wind(u10, 270.0)
        seas.append((f"fully developed u10 {u10:g}", w, [WaveTrain.fully_developed(w)]))
    env42 = SeaHistory(WindHistory(STORM)).environment(42.0, seed=1)
    swell = [t for t in env42.trains if t.name == "swell"]
    seas.append(("storm swell hour 42", env42.wind, swell))
    rows = []
    for name, w, trains in seas:
        row = {"sea": name, "hs_target": _r(sum(t.hs ** 2 for t in trains) ** 0.5), "tp": _r(trains[0].tp)}
        heights = {}
        for model in ("linear", "stokes2"):
            env = Environment(w, trains, seed=1, surface_model=model, interaction_n=96)
            cs = env.crest_stats(REGION, duration, spacing=spacing, dt=dt)
            ray = cs["rayleigh"]
            heights[model] = cs["heights"]
            c = env.c
            m0 = float(np.sum(c["a"] ** 2 / 2.0))
            t1 = 2.0 * np.pi * m0 / float(np.sum(c["a"] ** 2 / 2.0 * c["omega"]))
            row["t1_s"] = _r(t1)
            row[model] = {"sigma": _r(cs["sigma"]), "n_crests": cs["n"],
                          "h13": _r(cs["h13"]), "h110": _r(cs["h110"]), "hmax": _r(cs["hmax"]),
                          "rayleigh_h13": _r(ray["h13"]), "rayleigh_h110": _r(ray["h110"]),
                          "rayleigh_hmax": _r(ray["hmax"]),
                          "h13_over_rayleigh": _r(cs["h13"] / ray["h13"]),
                          "h110_over_rayleigh": _r(cs["h110"] / ray["h110"]),
                          "hmax_over_rayleigh": _r(cs["hmax"] / ray["hmax"])}
        hs = 4.0 * row["stokes2"]["sigma"]
        fr = {}
        for p in (1e-2, 1e-3):
            q = {m: float(h[min(h.size - 1, int(np.ceil((1.0 - p) * h.size)) - 1)]) for m, h in heights.items()}
            f = forristall_crest(hs, row["t1_s"], p)
            ray = row["stokes2"]["sigma"] * float(np.sqrt(-2.0 * np.log(p)))
            fr[f"P={p:g}"] = {"forristall": _r(f), "rayleigh": _r(ray), "linear": _r(q["linear"]),
                              "stokes2": _r(q["stokes2"]), "stokes2_over_forristall": _r(q["stokes2"] / f),
                              "linear_over_forristall": _r(q["linear"] / f)}
        row["forristall"] = {"hs_4sigma": _r(hs), "t1_s": row["t1_s"], "levels": fr,
                             "n_crests": row["stokes2"]["n_crests"]}
        for key in ("h13", "h110", "hmax"):
            row[f"stokes2_over_linear_{key}"] = _r(row["stokes2"][key] / row["linear"][key])
        rows.append(row)
        print(name, row["linear"]["h13_over_rayleigh"], row["stokes2"]["h13_over_rayleigh"],
              row["stokes2_over_linear_hmax"], flush=True)
    return {"grid": {"region": list(REGION), "spacing_m": spacing, "dt_s": dt, "duration_s": duration,
                     "components": 96, "interaction_n": 96}, "rows": rows}


def breaking_study(n=1_000_000, extent=3000.0, hours=1.0, seed=0):
    rows = []
    for u10 in (7.0, 10.0, 15.0):
        w = Wind(u10, 270.0)
        env = Environment(w, [WaveTrain.fully_developed(w)], seed=1, breaking="stats")
        st = env.breaking_stats()
        rng = np.random.default_rng(seed)
        x, y = rng.uniform(0, extent, n), rng.uniform(0, extent, n)
        t = rng.uniform(0, hours * 3600.0, n)
        frac = float(env.breaking_mask(x, y, t).mean())
        wc = float(whitecap_callaghan(u10))
        rows.append({"u10": u10, "eps_p": _r(st["eps_p"]), "pb": _r(st["pb"]), "kp": _r(st["kp"]),
                     "p_slot": _r(st["p_slot"]), "crest_region_m": _r(st["lc"]),
                     "marked_fraction": _r(frac), "expected_fraction": _r(st["p_slot"] * st["lc"] / st["lam"]),
                     "callaghan_W": _r(wc), "marked_over_callaghan": _r(frac / wc),
                     "stokes_corner_height_m": _r(0.443 / st["kp"]), "n_samples": n})
        print("breaking", u10, rows[-1]["marked_fraction"], wc, flush=True)
    return {"rows": rows, "note": "fully developed sea, 12 x 8 components; marked = crest region of a breaking "
                                  "event (env.breaking_crest_length_frac of the dominant wavelength), random points and times"}


def shear_study():
    def d(w, a, b):
        return float(w.speed(b) - w.speed(a))

    rows = []
    for u10 in (6.0, 9.0, 12.0):
        neutral = Wind(u10)
        n115, n19 = d(neutral, 1.0, 15.0), d(neutral, 1.5, 19.0)
        for label, L in (("neutral", None), ("L=+50", 50.0), ("L=+200", 200.0), ("L=-50", -50.0), ("L=-200", -200.0)):
            w = Wind(u10, stability_L=L)
            rows.append({"u10": u10, "case": label, "ustar": _r(w.ustar),
                         "shear_1_15_ms": _r(d(w, 1.0, 15.0)), "shear_1_15_over_u10": _r(d(w, 1.0, 15.0) / u10),
                         "shear_1_15_over_neutral": _r(d(w, 1.0, 15.0) / n115),
                         "shear_1p5_19_ms": _r(d(w, 1.5, 19.0)), "shear_1p5_19_over_neutral": _r(d(w, 1.5, 19.0) / n19)})
        profs = (("log", Wind(u10)), ("swell_knee 7.5 m", Wind(u10, profile="swell_knee")),
                 ("swell_jet", Wind(u10, profile="swell_jet")),
                 ("swell_jet cp/U=4 (maximum 1.1)", Wind(u10, profile="swell_jet", cp=4.0 * u10)))
        for label, w in profs:
            rows.append({"u10": u10, "case": label,
                         "shear_1_15_ms": _r(d(w, 1.0, 15.0)), "shear_1_15_over_u10": _r(d(w, 1.0, 15.0) / u10),
                         "shear_1_15_over_neutral": _r(d(w, 1.0, 15.0) / n115),
                         "shear_1p5_19_ms": _r(d(w, 1.5, 19.0)), "shear_1p5_19_over_neutral": _r(d(w, 1.5, 19.0) / n19),
                         "u8_minus_u2p5_over_u8": _r(d(w, 2.5, 8.0) / float(w.speed(8.0)))})
    # air-sea temperature difference to Obukhov length, and the shear factor it gives
    obu = []
    for u10 in (4.0, 8.0, 12.0):
        for dth in (2.0, -2.0):
            L = obukhov_length(u10, dth)
            w, n = Wind(u10, stability_L=L), Wind(u10)
            obu.append({"u10": u10, "dtheta_v_K": dth, "L_m": _r(L),
                        "shear_1_15_over_neutral": _r(d(w, 1.0, 15.0) / d(n, 1.0, 15.0))})
    return {"rows": rows, "obukhov": obu}


def turb_study(seeds=range(1, 9), dx=0.5, n=2**15):
    xs = np.arange(n) * dx
    rows = []
    for profile in ("fixed", "height"):
        for z in (1.0, 2.0, 5.0, 10.0):
            row = {"profile": profile, "z_m": z, "n_seeds": len(list(seeds))}
            w0 = Wind(10.0)
            iu = 0.06 * (1.0 + 0.043 * 10.0) * (z / 10.0) ** -0.22
            for comp, name, ratio in ((2, "w", 0.5), (0, "u", 1.0)):
                sig, scale, model = [], [], []
                for sd in seeds:
                    env = Environment(Wind(10.0, 270.0), [], seed=sd, turbulence="iso", turbulence_profile=profile)
                    g = env.gust(xs, 0 * xs, np.full_like(xs, z), 0 * xs)[:, comp]
                    sig.append(g.std())
                    f = np.fft.rfft(g - g.mean(), 2 * n)
                    r = np.fft.irfft(f * np.conj(f))[:n]
                    r /= r[0]
                    m = n // 20
                    zc = int(np.argmax(r[:m] <= 0.0)) or m
                    scale.append(r[:zc].sum() * dx - 0.5 * dx)
                    lens = turbulence_length_scales(np.array([z]))
                    lv = lens[2][0] if comp == 2 else lens[0][0]
                    lv = lv if profile == "height" else TURB_L
                    a2 = _turb_amp(env._turb["k"], np.array([lv]))[0] ** 2
                    model.append((a2 / env._turb["k"]).sum() / a2.sum())
                row[f"sigma_{name}_measured"] = _r(np.mean(sig))
                row[f"sigma_{name}_target"] = _r(iu * float(w0.speed(z)) * ratio)
                row[f"von_karman_L_{name}_m"] = _r(lv)
                row[f"integral_scale_{name}_measured_m"] = _r(np.mean(scale))
                row[f"integral_scale_{name}_modes_m"] = _r(np.mean(model))
            rows.append(row)
            print(profile, z, row["sigma_w_measured"], row["integral_scale_w_measured_m"],
                  row["integral_scale_u_measured_m"], flush=True)
    return {"u10": 10.0, "rows": rows,
            "note": "integral scale along the wind from the first zero crossing of the autocorrelation; "
                    "modes = energy-weighted mean of 1/k of the mode set (the value the mode set is built for)"}


def main(argv=None):
    ap = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    ap.add_argument("--out", default=str(HERE / "out" / "seawind_fitness.json"))
    ap.add_argument("--only", choices=["crest", "shear", "turb", "breaking"])
    a = ap.parse_args(argv)
    out = {"meta": record.meta("python -m sim.seawind_fitness", FILES)}
    if a.only in (None, "shear"):
        out["shear"] = shear_study()
    if a.only in (None, "turb"):
        out["turbulence"] = turb_study()
    if a.only in (None, "breaking"):
        out["breaking"] = breaking_study()
    if a.only in (None, "crest"):
        out["crest"] = crest_study()
    Path(a.out).parent.mkdir(parents=True, exist_ok=True)
    with open(a.out, "w", encoding="utf-8", newline="\n") as f:
        json.dump(out, f, indent=1)
        f.write("\n")
    print("wrote", a.out)


if __name__ == "__main__":
    main()
