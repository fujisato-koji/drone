"""Flight over swell (sim.wavelib): the oracle sea against the optimizer's wave mode, the crest ride, the
wave-phase lock of the guide, the pilot's choice of entry, and the flat sea unchanged with a wave library."""
import unittest

import numpy as np

from sim.aircraft import Actuators
from sim.aircraft_models import by_name
from sim.dsopt import WaveParams, profile_for, wave_preset_params, wave_wind_accel
from sim.mpc import (NAV_PKW, Cycle, CycleLibrary, WaveCycleGuide, energy_weight, fly_mpc, world_swell, wrap)
from sim.seawind import Environment, Wind
from sim.wavelib import (WavePilot, crest_entries, crest_ride, crest_state, _crest_setup, sea_of, swell_env,
                         wavelength)


class TestSwellSea(unittest.TestCase):
    def test_wind_matches_the_optimizer_wave_mode(self):
        rng = np.random.default_rng(0)
        for preset in ("p63", "p64_low"):
            for d, rel in (("along", 0.0), ("across", 90.0), ("against", 180.0)):
                for u10 in (0.1, 6.0):
                    env = swell_env(u10, 3.0, 10.0, d, preset, phase=0.7, surface_model="linear")
                    p = wave_preset_params(preset)
                    wv = WaveParams(1.5, wavelength(10.0), rel, decay=p["decay"], u_scale=p["u_scale"],
                                    w_scale=p["w_scale"])
                    prof = profile_for(u10, "log", roughness="coare35")
                    x, y, t = rng.uniform(-100, 100, 6), rng.uniform(-100, 100, 6), rng.uniform(0, 30, 6)
                    th = wv.k * (wv.cw * x + wv.sw * y) - wv.om * t + 0.7
                    z = 1.5 * np.cos(th) + rng.uniform(1, 8, 6)
                    S = np.stack([x, y, z, np.full(6, 12.0), np.zeros(6), np.zeros(6)], axis=1)
                    (Wx, Wy, Wz), _ = wave_wind_accel(S, wv, prof, t, u10, 0.7)
                    W = env.wind_at(x, y, z, t)
                    self.assertLess(np.abs(np.stack([Wx, Wy, Wz], 1) - W).max(), 1e-9)
                    self.assertLess(np.abs(env.surface(x, y, t)[0] - 1.5 * np.cos(th)).max(), 1e-9)

    def test_sea_of_reads_the_swell(self):
        for d, rel in (("along", 0.0), ("across", 90.0), ("against", 180.0)):
            hs, tp, r = sea_of(swell_env(6.0, 2.0, 8.0, d, "p63"), 0.0)
            self.assertAlmostEqual(hs, 2.0)
            self.assertAlmostEqual(tp, 8.0)
            self.assertAlmostEqual(abs(r), rel, places=6)
        self.assertIsNone(sea_of(Environment.flat(Wind(6.0, 270.0)), 0.0))

    def test_second_order_surface_is_the_default(self):
        env = swell_env(6.0, 3.0, 10.0, "along", "p63")
        k = (2 * np.pi / 10.0) ** 2 / 9.81
        self.assertAlmostEqual(float(env.surface(0.0, 0.0, 0.0)[0]), 1.5 + 0.5 * 1.5 ** 2 * k, places=6)


class TestCrestRide(unittest.TestCase):
    def test_ride_is_an_equilibrium_on_the_forward_face(self):
        ac = by_name("albatross-3m")
        r = crest_ride(ac, "p63", 3.0, 10.0, "along", 0.0, 0.0)
        self.assertIsNotNone(r)
        self.assertLess(abs(r["residual"]), 1e-6)
        self.assertGreater(r["theta"], 0.0)              # forward face: 0 < theta < pi
        self.assertLess(r["theta"], np.pi)
        self.assertGreaterEqual(r["h"], 2.0 - 1e-9)
        self.assertGreaterEqual(r["V"], 1.2 * ac.v_stall)
        es = crest_entries("albatross-3m", "p63", 3.0, 10.0, "along", 0.0)
        self.assertTrue(es)
        for e in es:
            self.assertLess(abs(e["replay"]["h_drift_m"]), 0.05)   # an equilibrium holds over a crest period

    def test_no_ride_on_a_small_swell_with_the_low_bound(self):
        # 1.5 m swell, P64 low bound, calm: the baseline's sink at the wave speed exceeds the updraft
        self.assertEqual(crest_entries("baseline-2m", "p64_low", 1.5, 10.0, "along", 0.0), [])

    def test_ride_balances_updraft_and_sink(self):
        ac = by_name("albatross-3m")
        acf, wv, prof, u = _crest_setup(ac, "p63", 3.0, 10.0, "along", 0.0)
        r = crest_ride(ac, "p63", 3.0, 10.0, "along", 0.0, 0.0)
        res, V, g, psi, cl = crest_state(acf, wv, prof, u, r["theta"], r["h"], 0.0)
        self.assertAlmostEqual(V * np.cos(g) * np.cos(psi), wv.c - 0.0, delta=1.5)


def synthetic_wave_cycle(T=10.0, m=0):
    t = np.linspace(0.0, T, 41)
    w = 2 * np.pi / T
    th = 0.8 + 0.4 * np.sin(w * t) - 2 * np.pi * m * t / T
    d = {"period": T, "t": t, "x": 15.0 * t, "y": 2.0 * np.sin(w * t), "z": 6.0 + 3.0 * np.cos(w * t),
         "V": 14.0 + 2.0 * np.sin(w * t), "gamma": 0.2 * np.sin(w * t), "psi": 0.3 * np.sin(w * t),
         "cl": 0.6 + 0.2 * np.cos(w * t), "mu": 0.5 * np.sin(w * t), "thrust": np.zeros(41), "theta": th,
         "m": m, "kind": "soaring"}
    return Cycle(d)


class TestWaveGuide(unittest.TestCase):
    def test_phase_lock_picks_the_matching_wave_phase(self):
        cyc = synthetic_wave_cycle()
        env = swell_env(6.0, 3.0, 10.0, "along", "p63", surface_model="linear")
        k, om, dx, dy, ph, a = world_swell(env)
        tau = 3.7
        h, V, g, pr = cyc.state(tau)
        x = (cyc.theta_at(tau) - ph) / k          # the aircraft where the sea's phase equals the cycle's at tau
        eta = float(env.surface(x, 0.0, 0.0)[0])
        s = [x, 0.0, h + eta, V, g, pr]
        gd = WaveCycleGuide(cyc, env, 0.0, s, 0.0, 6.0)
        self.assertLess(abs((gd.tau0 - tau + 5.0) % 10.0 - 5.0), 0.15)
        self.assertLess(abs(gd.theta_err0), 0.05)
        # the same state half a wavelength away: the phase term makes the match worse
        s2 = [x + np.pi / k, 0.0, h + float(env.surface(x + np.pi / k, 0.0, 0.0)[0]), V, g, pr]
        gd2 = WaveCycleGuide(cyc, env, 0.0, s2, 0.0, 6.0)
        self.assertGreater(gd2.match_dist, gd.match_dist + 10.0)

    def test_terminal_cost_sees_the_wave_phase(self):
        cyc = synthetic_wave_cycle()
        env = swell_env(6.0, 3.0, 10.0, "along", "p63", surface_model="linear")
        k, om, dx, dy, ph, a = world_swell(env)
        s = [(cyc.theta_at(0.0) - ph) / k, 0.0, 0.0, 14.0, 0.0, 0.0]
        s[2] = cyc.state(0.0)[0] + float(env.surface(s[0], 0.0, 0.0)[0])
        gd = WaveCycleGuide(cyc, env, 0.0, s, 0.0, 6.0)
        ht, Vt, gt, pt = gd.target
        xe = (gd.theta_t - ph + om * 6.0) / k
        end = np.array([[xe, xe + 0.5 * np.pi / k], [0.0, 0.0], [0.0, 0.0], [Vt, Vt], [gt, gt], [pt, pt]])
        end[2] = ht + np.asarray(env.surface(end[0], end[1], np.full(2, 6.0))[0])
        P = type("P", (), {"w_e": 1.0})()
        c = gd.terminal_cost(env, 6.0, end, P)
        self.assertLess(float(c[0]), 1.0)
        self.assertGreater(float(c[1]), float(c[0]) + 100.0)

    def test_theta_continues_over_periods(self):
        cyc = synthetic_wave_cycle(m=1)
        self.assertAlmostEqual(float(cyc.theta_at(10.0) - cyc.theta_at(0.0)), -2 * np.pi, places=6)
        self.assertAlmostEqual(float(cyc.theta_at(23.0) - cyc.theta_at(13.0)), -2 * np.pi, places=6)


class TestPilot(unittest.TestCase):
    def test_mirror_and_course(self):
        es = crest_entries("albatross-3m", "p63", 3.0, 10.0, "along", 6.0)
        pilot = WavePilot(es, "p63", policy="course")
        env = swell_env(6.0, 3.0, 10.0, "along", "p63")
        left = pilot.cycle("albatross-3m", env, 6.0, np.radians(40.0), 0.0)
        right = pilot.cycle("albatross-3m", env, 6.0, np.radians(-40.0), 0.0)
        self.assertGreater(left.disp[1], 0.0)
        self.assertLess(right.disp[1], 0.0)
        self.assertTrue(right.mirror)
        self.assertIsNone(pilot.cycle("albatross-3m", Environment.flat(Wind(6.0, 270.0)), 6.0, 0.0, 0.0))

    def test_flat_sea_unchanged_with_a_wave_library(self):
        ac = by_name("baseline-2m")
        lib = CycleLibrary.from_keyword("motor")
        pilot = WavePilot(crest_entries("baseline-2m", "p63", 3.0, 10.0, "along", 9.0), "p63")
        runs = []
        for wl in (None, pilot):
            env = Environment.flat(Wind(9.0, 270.0, roughness="coare35"))
            r = fly_mpc(ac, env, "flat", [], 4.0, env.wind.to_deg - 20.0, 9.0, seed=1, planner_kw=dict(NAV_PKW),
                        library=lib, actuators=Actuators(ac, rate_model="pb2v"), n_limit=True, stop_on_tip=True,
                        wave_aware=False, motor=True, w_energy=energy_weight(ac, "normal"), keep=True,
                        wave_library=wl)
            runs.append(r)
        self.assertTrue(np.array_equal(runs[0]["traj"]["state"], runs[1]["traj"]["state"]))
        self.assertEqual(runs[1]["wave_switches"], [])


if __name__ == "__main__":
    unittest.main()
