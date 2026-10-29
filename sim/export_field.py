"""Export the sea/wind field to a JS file for the viewer."""

import argparse

from .seawind import Environment, WaveTrain, Wind


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
    a = p.parse_args(argv)

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
