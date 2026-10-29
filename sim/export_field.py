"""Export the sea/wind field to a JS file for the viewer."""

import argparse
import json

from .sea_history import SeaHistory, WindHistory
from .seawind import Environment, WaveTrain, Wind

# built-in scenario: (hour, u10 m/s, wind-from deg)
STORM = [(0, 4, 270), (6, 4, 270), (12, 15, 270), (30, 15, 270),
         (36, 15, 300), (42, 5, 300), (72, 5, 300)]
FRAME_STEP_H = 3


def export_scenario(a):
    roughness = a.roughness or "wave_age"
    stokes = True if a.stokes is None else a.stokes
    sh = SeaHistory(WindHistory(STORM))
    frames, rows = [], []
    for hour in range(0, 72 + 1, FRAME_STEP_H):
        env0 = sh.environment(float(hour), seed=a.seed, roughness=roughness)
        # keep each frame at most 128 components: swell 8 x 4 (SeaHistory uses 12 x 8)
        for tr in env0.trains:
            if tr.name == "swell":
                tr.n_omega, tr.n_theta = 8, 4
        env = Environment(env0.wind, env0.trains, seed=a.seed, stokes=stokes)
        d = env.to_dict()
        wc = env.whitecap_fraction(0.0)
        d["hour"] = float(hour)
        d["whitecap_fraction"] = float(f"{wc:.6g}")
        frames.append(d)
        tr = {t["name"]: t for t in d["trains"]}
        sea, sw = tr.get("wind sea"), tr.get("swell")
        rows.append((hour, env.wind.u10, sea["hs"] if sea else 0.0, sea["tp"] if sea else 0.0,
                     sw["hs"] if sw else 0.0, env.wind.ustar, env.wind.z0, wc))
    meta = dict(frames[0]["meta"])
    meta.update({"scenario": "storm", "roughness": roughness, "stokes": bool(stokes),
                 "n_components": max(f["meta"]["n_components"] for f in frames),
                 "n_frames": len(frames), "frame_step_h": FRAME_STEP_H})
    for f in frames:
        del f["meta"]
    out = {"meta": meta, "constants": frames[0].pop("constants"), "frames": frames}
    for f in frames[1:]:
        f.pop("constants")
    with open(a.out, "w", encoding="utf-8", newline="\n") as fh:
        fh.write("window.FIELD = " + json.dumps(out) + ";\n")
    print(f"scenario storm, roughness {roughness}, stokes {stokes}, "
          f"max components per frame {meta['n_components']}")
    print("hour   u10  hs_sea  tp_sea  hs_swell   ustar       z0      whitecap")
    for r in rows:
        print(f"{r[0]:4d} {r[1]:5.1f} {r[2]:7.2f} {r[3]:7.2f} {r[4]:9.2f} {r[5]:7.3f} {r[6]:9.2e} {r[7]:10.4f}")


def main(argv=None):
    p = argparse.ArgumentParser()
    p.add_argument("--u10", type=float, default=10.0)
    p.add_argument("--from", dest="from_deg", type=float, default=270.0)
    p.add_argument("--hs", type=float)
    p.add_argument("--tp", type=float)
    p.add_argument("--swell-hs", type=float)
    p.add_argument("--swell-tp", type=float)
    p.add_argument("--swell-to", type=float)
    p.add_argument("--seed", type=int, default=1)
    p.add_argument("--out", default="sim/viewer/field.js")
    p.add_argument("--scenario", choices=["storm"])
    p.add_argument("--roughness", choices=["charnock", "wave_age", "drennan"])
    p.add_argument("--stokes", action=argparse.BooleanOptionalAction, default=None)
    a = p.parse_args(argv)
    if a.scenario:
        return export_scenario(a)
    if a.roughness or a.stokes is not None:
        p.error("--roughness and --stokes need --scenario")

    wind = Wind(a.u10, a.from_deg)
    if a.hs is not None and a.tp is not None:
        trains = [WaveTrain(a.hs, a.tp, wind.to_deg, n_omega=12, n_theta=8)]
    else:
        trains = [WaveTrain.fully_developed(wind, n_omega=12, n_theta=8)]
    if a.swell_hs is not None:
        if a.swell_tp is None or a.swell_to is None:
            p.error("--swell-hs needs --swell-tp and --swell-to")
        trains.append(WaveTrain(a.swell_hs, a.swell_tp, a.swell_to, gamma=7.0,
                                n_omega=8, n_theta=4, name="swell"))
    env = Environment(wind, trains, seed=a.seed)
    env.to_js(a.out)
    print(f"ustar = {wind.ustar:.4f} m/s, z0 = {wind.z0:.3e} m")
    for tr, d in zip(trains, env.to_dict()["trains"]):
        print(f"{tr.name}: hs = {tr.hs:.3f} m, hs_discrete = {d['hs_discrete']:.3f} m, tp = {tr.tp:.3f} s")
    print(f"N = {env.n_components}")


if __name__ == "__main__":
    main()
