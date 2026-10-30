import unittest

import numpy as np

from sim.aircraft import BASELINE, derivatives
from sim.aircraft_models import by_name
from sim.dsopt import (Collocation, Profile, WaveParams, _apply_preset, _resample, dense_replay,
                       derivs, profile_for, sachs_albatross, solve, solve_min_wind,
                       wave_preset_params, wave_wind_accel)
from sim.record import value
from sim.seawind import Environment, Wind

BOWER_U10 = 8.56  # m/s, Bower 2011 recomputation of Sachs 2005 (record P45)


class TestWindTerms(unittest.TestCase):
    def test_matches_aircraft_derivatives(self):
        wind = Wind(10.0, from_deg=270.0)
        env = Environment.flat(wind)
        h = 0.05  # the finite-difference step of sim.aircraft.wind_derivs
        rng = np.random.default_rng(3)
        for _ in range(12):
            z = rng.uniform(1.0, 15.0)
            s = np.array([rng.uniform(-50, 50), rng.uniform(-50, 50), z, rng.uniform(8, 25),
                          rng.uniform(-0.6, 0.6), rng.uniform(-np.pi, np.pi)])
            cl, mu = rng.uniform(0.1, 1.1), rng.uniform(-1.0, 1.0)
            ref, _ = derivatives(0.0, s, BASELINE, env, cl, mu)
            U = float(wind.speed(z))
            Up = float(wind.speed(z + h) - wind.speed(z - h)) / (2 * h)
            mine = derivs(s, np.array([cl, mu]), BASELINE, U, Up)
            np.testing.assert_allclose(mine, ref, rtol=1e-6, atol=1e-9)

    def test_profile_matches_wind(self):
        z = np.array([0.5, 1.0, 2.5, 4.0, 9.0, 20.0])
        w = Wind(9.0)
        np.testing.assert_allclose(Profile.from_wind(w).f(z), w.speed(z) / 9.0, rtol=1e-10)
        wk = Wind(9.0, profile="swell_knee", knee_m=7.5)
        zk = np.array([0.5, 1.0, 2.5, 4.0])  # away from the (smoothed) knee
        np.testing.assert_allclose(Profile.from_wind(wk).f(zk), wk.speed(zk) / 9.0, rtol=1e-3)
        self.assertAlmostEqual(float(Profile.from_wind(wk).f(30.0)), 1.0, delta=0.01)


class TestJacobian(unittest.TestCase):
    def test_constraint_jacobians_match_finite_differences(self):
        prof = Profile.from_wind(Wind(8.0, profile="swell_knee", knee_m=7.5))
        col = Collocation(by_name("baseline-2m"), prof, "min_wind", "loiter", 9)
        rng = np.random.default_rng(1)
        x = col.guess(rng) * (1 + 0.01 * rng.standard_normal(col.nv))
        for fun, jac in ((col.eq_fun, col.eq_jac), (col.ineq_fun, col.ineq_jac)):
            J = jac(x)
            Jn = np.zeros_like(J)
            for i in range(x.size):
                xp, xm = x.copy(), x.copy()
                xp[i] += 1e-6
                xm[i] -= 1e-6
                Jn[:, i] = (fun(xp) - fun(xm)) / 2e-6
            self.assertLess(np.abs(J - Jn).max(), 1e-6 * max(1.0, np.abs(Jn).max()))


class TestSolve(unittest.TestCase):
    def test_validation_sachs_albatross(self):
        # Slow (about 15 to 30 s). The Sachs model has no speed floor, so v_factor = 0.
        ac, prof = sachs_albatross()
        self.assertAlmostEqual(ac.best_ld, 20.0, places=6)
        info = solve(ac, prof, "min_wind", "travel", 40, h_min=1.5, v_factor=0.0,
                     seeds=(1, 0, 2), stop_after=1)
        self.assertTrue(info["success"], info["message"])
        self.assertLess(abs(info["u10"] - BOWER_U10) / BOWER_U10, 0.15)
        self.assertTrue(5.0 < info["period"] < 9.0)
        self.assertLessEqual(info["n_max"], 3.0 + 1e-6)
        self.assertGreaterEqual(info["z_min"], 1.5 - 1e-6)
        self.assertAlmostEqual(info["net_turn_deg"], 0.0, places=4)

    def test_max_speed_across_wind(self):
        info = solve(by_name("baseline-2m"), Profile.from_wind(Wind(12.0)), "max_speed",
                     "travel", 30, u10=12.0, chi=np.pi / 2, seeds=(0, 1, 2), stop_after=1)
        self.assertTrue(info["success"], info["message"])
        self.assertAlmostEqual(info["u10"], 12.0, places=6)
        self.assertGreater(info["speed_along_chi"], 5.0)


class TestWaveMode(unittest.TestCase):
    def test_wind_acceleration_matches_finite_differences(self):
        # a_W = dW/dt + (v_g . grad) W against a central difference along the path, 20 states
        prof = profile_for(8.0)
        rng = np.random.default_rng(5)
        d = 1e-5
        for _ in range(20):
            sh = bool(rng.integers(2))
            wv = WaveParams(rng.uniform(0.3, 1.5) * (2.0 if sh else 1.0),
                            rng.uniform(40, 150) * (0.4 if sh else 1.0), rng.uniform(-60, 60),
                            decay=(0.77, 0.62), shelter=sh)
            S = np.array([rng.uniform(-50, 50), rng.uniform(-50, 50), rng.uniform(3, 12),
                          rng.uniform(8, 22), rng.uniform(-0.6, 0.6), rng.uniform(-3, 3)])
            t, u, phi = rng.uniform(0, 10), rng.uniform(5, 12), rng.uniform(0, 6)
            W, aW = wave_wind_accel(S, wv, prof, t, u, phi)
            vg = np.array([S[3] * np.cos(S[4]) * np.cos(S[5]) + W[0],
                           S[3] * np.cos(S[4]) * np.sin(S[5]) + W[1], S[3] * np.sin(S[4]) + W[2]])

            def wind(dt):
                s = S.copy()
                s[:3] += vg * dt
                return np.array(wave_wind_accel(s, wv, prof, t + dt, u, phi)[0])

            fd = (wind(d) - wind(-d)) / (2 * d)
            np.testing.assert_allclose(np.array(aW), fd, rtol=1e-6, atol=1e-7)

    def test_wave_constraint_jacobians_match_finite_differences(self):
        prof = profile_for(8.0)
        for tip in ("abs", "geometric"):
            wave = {"a": 1.0, "wavelength": 60.0, "to_deg": 37.0, "decay": (0.77, 0.62),
                    "shelter": tip == "geometric"}
            col = Collocation(by_name("baseline-2m"), prof, "min_wind", "travel", 9,
                              tip_model=tip, wave=wave)
            rng = np.random.default_rng(1)
            x = col.guess(rng) * (1 + 0.01 * rng.standard_normal(col.nv))
            col.set_m(x)
            for fun, jac in ((col.eq_fun, col.eq_jac), (col.ineq_fun, col.ineq_jac)):
                J = jac(x)
                Jn = np.zeros_like(J)
                for i in range(x.size):
                    xp, xm = x.copy(), x.copy()
                    xp[i] += 1e-6
                    xm[i] -= 1e-6
                    Jn[:, i] = (fun(xp) - fun(xm)) / 2e-6
                self.assertLess(np.abs(J - Jn).max(), 1e-6 * max(1.0, np.abs(Jn).max()))

    def test_zero_amplitude_reproduces_flat(self):
        # slow (two solves); a = 0 drops the phase condition, phi0 is then inert
        ac, prof = by_name("baseline-2m"), profile_for(8.0)
        kw = dict(mode="min_wind", cycle="travel", n=30, h_min=1.0, tip_model="geometric",
                  seeds=(0, 1, 2), stop_after=1)
        flat = solve(ac, prof, **kw)
        # midpoints False: the flat mode has no midpoint clearance constraint (with it the wave
        # mode is stricter than flat, by 0.05 m/s for this aircraft)
        wave = solve(ac, prof, wave={"a": 0.0, "wavelength": 80.0, "to_deg": 0.0,
                                     "midpoints": False}, phase_pin="gamma", **kw)
        self.assertTrue(flat["success"] and wave["success"])
        self.assertAlmostEqual(wave["u10"], flat["u10"], delta=0.02)
        # the theta pin (default): with a = 0 the wave phase is inert, so the time shift is still
        # fixed by gamma(0) = 0 and the result stays with the flat one
        wave_t = solve(ac, prof, wave={"a": 0.0, "wavelength": 80.0, "to_deg": 0.0,
                                       "midpoints": False}, phase_pin="theta", **kw)
        self.assertTrue(wave_t["success"])
        self.assertAlmostEqual(wave_t["u10"], flat["u10"], delta=0.05)


class TestRoughness(unittest.TestCase):
    def test_self_consistent_z0(self):
        info = solve_min_wind(by_name("baseline-2m"), "self_consistent", cycle="travel", n=30,
                              h_min=1.0, tip_model="geometric", seeds=(0, 1, 2), stop_after=1)
        self.assertTrue(info["success"] and info["outer_converged"])
        self.assertAlmostEqual(info["z0"], profile_for(info["u10"]).z0, delta=0.02 * info["z0"])
        self.assertLess(info["u10"], 11.5)  # z0 at u10 = 8 overstates the minimum

    def test_fixed_z0_profile(self):
        self.assertEqual(profile_for(8.0, roughness="fixed_z0", z0=0.03).z0, 0.03)

    def test_coare35_profile(self):
        self.assertEqual(profile_for(10.0, roughness="coare35").z0,
                         Wind(10.0, roughness="coare35").z0)
        self.assertNotEqual(profile_for(10.0, roughness="coare35").z0, profile_for(10.0).z0)


def _fd_check(test, col, x, tol=1e-6):
    for fun, jac in ((col.eq_fun, col.eq_jac), (col.ineq_fun, col.ineq_jac)):
        J = jac(x)
        Jn = np.zeros_like(J)
        for i in range(x.size):
            xp, xm = x.copy(), x.copy()
            xp[i] += 1e-6
            xm[i] -= 1e-6
            Jn[:, i] = (fun(xp) - fun(xm)) / 2e-6
        test.assertLess(np.abs(J - Jn).max(), tol * max(1.0, np.abs(Jn).max()))


class TestConstraints(unittest.TestCase):
    """Midpoints in the flat mode, rate limits, n_min, margins, the theta pin, m_search."""

    NEW = dict(midpoints=True, rate_limits=True, n_min=0.2, n_frac=0.8, mu_frac=0.9)

    def test_jacobians_flat(self):
        prof = profile_for(10.0)
        for tip in ("abs", "geometric"):
            col = Collocation(by_name("baseline-2m"), prof, "min_wind", "travel", 9,
                              tip_model=tip, bank_rate_max_deg_s=100.0, **self.NEW)
            rng = np.random.default_rng(2)
            x = col.guess(rng) * (1 + 0.01 * rng.standard_normal(col.nv))
            f, J = col.ineq_fun(x), col.ineq_jac(x)
            self.assertEqual(f.size, 9 + 9 + 9 + 8 + 8 + 8 + 4 * 8)  # n, n_min, tip, h mid, tip mid, n mid, rates
            _fd_check(self, col, x)

    def test_jacobians_pb2v(self):
        prof = profile_for(10.0)
        col = Collocation(by_name("baseline-2m"), prof, "min_wind", "travel", 9,
                          tip_model="geometric", midpoints=True, rate_limits=True, n_min=0.0,
                          rate_model="pb2v", roll_pb2v=0.08)
        self.assertEqual(col.r_max, 1.0)  # CL rate default of the pb2v model
        rng = np.random.default_rng(4)
        x = col.guess(rng) * (1 + 0.02 * rng.standard_normal(col.nv))
        _fd_check(self, col, x)
        # the limit is on the slower node: halve V at node 3, the rows of intervals 2 and 3 follow
        S, C, T, u = col.unpack(x)
        f0 = col._rates(x)[0]
        S[3, 3] *= 0.5
        f1 = col._rates(col.pack(S, C, T, u))[0]
        M = col.N - 1
        for off in (0, M):  # the two sign rows of the bank rate
            self.assertTrue(np.all(f1[off + 2:off + 4] < f0[off + 2:off + 4]))
            self.assertAlmostEqual(f1[off], f0[off], places=12)

    def test_pb2v_solution(self):
        ac = by_name("baseline-2m")
        info = solve(ac, profile_for(11.0), "min_wind", "travel", 30, seeds=(0, 1, 2),
                     stop_after=1, tip_model="geometric", midpoints=True, rate_limits=True,
                     n_min=0.0, rate_model="pb2v", roll_pb2v=0.08)
        self.assertTrue(info["success"], info["message"])
        self.assertLessEqual(info["pb2v_used_max"], 0.08 + 1e-5)
        self.assertLessEqual(info["cl_rate_max_per_s"], 1.0 + 1e-5)
        self.assertLessEqual(info["bank_rate_over_limit_max"], 1.0 + 1e-5)

    def test_jacobians_wave(self):
        prof = profile_for(8.0)
        wave = {"a": 1.0, "wavelength": 60.0, "to_deg": 37.0, "decay": (0.77, 0.62),
                "u_scale": 0.9, "w_scale": 0.6}
        col = Collocation(by_name("albatross-3m"), prof, "min_wind", "travel", 9,
                          tip_model="geometric", wave=wave, **self.NEW)
        rng = np.random.default_rng(1)
        x = col.guess(rng) * (1 + 0.01 * rng.standard_normal(col.nv))
        col.set_m(x)
        _fd_check(self, col, x)
        rows = col._periodic(x)[0].size
        self.assertEqual(rows, 4 + 1 + 2 + 1)  # z V gamma psi, theta pin, seam controls, m

    def test_wave_scales(self):
        prof = profile_for(8.0)
        S = np.array([3.0, 4.0, 5.0, 12.0, 0.1, 0.5])
        W1, _ = wave_wind_accel(S, WaveParams(1.0, 60.0, 20.0), prof, 1.0, 8.0, 0.3)
        W2, _ = wave_wind_accel(S, WaveParams(1.0, 60.0, 20.0, u_scale=0.5, w_scale=0.6),
                                prof, 1.0, 8.0, 0.3)
        base = float(prof.f(5.0 - 1.0 * np.cos(WaveParams(1.0, 60.0, 20.0).k * (
            np.cos(np.radians(20.0)) * 3.0 + np.sin(np.radians(20.0)) * 4.0)
            - WaveParams(1.0, 60.0, 20.0).om + 0.3))) * 8.0
        self.assertAlmostEqual(W2[2], 0.6 * W1[2], places=12)
        self.assertAlmostEqual(W2[0] - base, 0.5 * (W1[0] - base), places=10)

    def test_solution_satisfies_new_constraints(self):
        ac = by_name("baseline-2m")
        info = solve(ac, profile_for(11.0), "min_wind", "travel", 30, seeds=(0, 1, 2),
                     stop_after=1, tip_model="geometric", bank_rate_max_deg_s=100.0, **self.NEW)
        self.assertTrue(info["success"], info["message"])
        tol = 1e-5
        self.assertLessEqual(info["bank_rate_max_deg_s"], 100.0 + tol)
        self.assertLessEqual(info["cl_rate_max_per_s"], value("ctrl.const_cl_rate_max_per_s", 2.0) + tol)
        self.assertLessEqual(info["n_max_node"], 0.8 * ac.n_max + tol)
        self.assertGreaterEqual(info["n_min_node"], 0.2 - tol)
        self.assertLessEqual(info["mu_max_deg"], 0.9 * np.degrees(ac.mu_max) + tol)
        self.assertLess(max(info["control_seam_jump"]), tol)
        self.assertGreaterEqual(info["z_min"], 1.0 - tol)
        self.assertGreaterEqual(info["replay_tip_min"], 0.3 - 0.03)  # dense replay, see below

    def test_flat_midpoints_hold_the_tip(self):
        ac, prof = by_name("baseline-2m"), profile_for(11.0)
        kw = dict(cycle="travel", n=30, tip_model="geometric", seeds=(0, 1, 2), stop_after=1)
        nodes = solve(ac, prof, "min_wind", **kw)
        mid = solve(ac, prof, "min_wind", midpoints=True, **kw)
        self.assertTrue(nodes["success"] and mid["success"])
        self.assertLess(nodes["replay_tip_min"], 0.27)  # K-009: the tip dips between nodes
        self.assertGreaterEqual(mid["replay_tip_min"], 0.3 - 0.03)
        self.assertGreaterEqual(mid["replay_h_min"], 1.0 - 0.03)
        self.assertGreater(mid["u10"], nodes["u10"] - 1e-9)

    def test_rate_limits_off_reproduce_baseline(self):
        info = solve(by_name("baseline-2m"), profile_for(8.0), "min_wind", "travel", 30,
                     seeds=(0, 1, 2), stop_after=1, rate_limits=False)
        self.assertAlmostEqual(info["u10"], 11.864159641948673, places=10)

    def test_m_search_is_no_worse(self):
        ac = by_name("baseline-2m")
        lam = 9.81 * 8.0 ** 2 / (2.0 * np.pi)
        wave = {"a": 1.0, "wavelength": lam, "to_deg": 0.0}
        kw = dict(cycle="travel", n=30, tip_model="geometric", seeds=(0,), stop_after=1,
                  wave=wave, midpoints=True)
        prof = profile_for(10.7)
        one = solve(ac, prof, "min_wind", **kw)
        many = solve(ac, prof, "min_wind", m_search=(-1, 0, 1), **kw)
        self.assertTrue(one["success"] and many["success"])
        self.assertLessEqual(many["u10"], one["u10"] + 1e-9)


class _OneComponent:
    """Single-component train for sim.seawind.Environment (a, wavelength, phase; along the wind)."""

    hs, tp, gamma = 1.0, 8.0, 1.0

    def __init__(self, a, wavelength, phase, to_deg=90.0):
        self.a, self.wavelength, self.phase, self.to_deg = a, wavelength, phase, to_deg

    def components(self, rng):
        k = 2.0 * np.pi / self.wavelength
        om = np.sqrt(9.81 * k)
        return {"a": np.array([self.a]), "k": np.array([k]), "omega": np.array([om]),
                "dx": np.array([1.0]), "dy": np.array([0.0]), "phase": np.array([self.phase])}


class TestAgainstEnvironment(unittest.TestCase):
    def test_wave_wind_matches_environment_with_shelter(self):
        wl, u10, phase = 20.0, 10.0, 0.7
        a = 0.3 / (2.0 * np.pi / wl)
        wind = Wind(u10, from_deg=270.0)  # blows toward +x (east)
        env = Environment(wind, [_OneComponent(a, wl, phase)], shelter=True, decay=(1.0, 1.0))
        prof = Profile.from_wind(wind)
        wv = WaveParams(a, wl, 0.0, decay=(1.0, 1.0), shelter=True)
        rng = np.random.default_rng(11)
        worst, where = 0.0, None
        for t in (0.0, 1.3, 2.9):
            for _ in range(17):
                x, y, h = rng.uniform(0.0, wl), rng.uniform(-5, 5), rng.uniform(1.0, 5.0)
                eta = float(env.surface(x, y, t)[0])
                S = np.array([x, y, h + eta, 12.0, 0.0, 0.0])
                W, _ = wave_wind_accel(S, wv, prof, t, u10, phase)
                ref = env.wind_at(x, y, h + eta, t)
                err = float(np.abs(np.array(W) - ref).max())
                if err > worst:
                    worst, where = err, (t, x, h, float(env.shelter_r(x, y, t)))
        self.assertLess(worst, 0.1, where)
        print("env comparison: max |dW| = %.4f m/s at (t, x, h, r) = %s" % (worst, where))

class TestReplayRefinePreset(unittest.TestCase):
    def test_presets(self):
        self.assertIsNone(wave_preset_params("model"))
        p63 = wave_preset_params("p63")
        self.assertEqual(p63["w_scale"], value("lit.wave_induced_amp_w", 1.07))
        self.assertEqual(p63["decay"], (value("lit.wave_induced_decay_u", 0.77),
                                        value("lit.wave_induced_decay_w", 0.62)))
        self.assertEqual(p63["u_scale"], 1.0)
        low = wave_preset_params("p64_low")
        self.assertEqual(low["w_scale"], 0.6)
        self.assertEqual(low["decay"], p63["decay"])
        w = _apply_preset({"a": 1.0, "wavelength": 60.0, "to_deg": 0.0, "w_scale": 0.3}, "p63")
        self.assertEqual(w["w_scale"], 0.3)       # explicit values win
        self.assertEqual(w["decay"], p63["decay"])
        self.assertNotIn("w_scale", _apply_preset({"a": 1.0}, "model"))
        with self.assertRaises(ValueError):
            _apply_preset({"a": 1.0}, "nope")

    def test_resample_roundtrip(self):
        col = Collocation(by_name("baseline-2m"), profile_for(8.0), "min_wind", "travel", 30,
                          wave={"a": 1.0, "wavelength": 60.0, "to_deg": 0.0})
        x = col.guess(np.random.default_rng(0))
        col2 = Collocation(by_name("baseline-2m"), profile_for(8.0), "min_wind", "travel", 59,
                           wave={"a": 1.0, "wavelength": 60.0, "to_deg": 0.0})
        x2 = _resample(col, x, col2)
        self.assertEqual(x2.size, col2.nv)
        S, C, T, u = col.unpack(x)
        S2, C2, T2, u2 = col2.unpack(x2)
        self.assertAlmostEqual(T, T2)
        self.assertAlmostEqual(u, u2)
        np.testing.assert_allclose(S2[::2], S, atol=1e-6)
        self.assertAlmostEqual(col.phi0(x), col2.phi0(x2))

    def test_dense_replay_flat_cycle_and_a_spoiled_one(self):
        info, x, col = solve(by_name("baseline-2m"), profile_for(12.0), "min_wind", "travel", 30,
                             h_min=1.0, tip_model="geometric", midpoints=True, seeds=(0, 1, 2),
                             stop_after=1, replay_check=True, return_x=True)
        self.assertTrue(info["success"], info["message"])
        self.assertTrue(info["replay_pass"])
        self.assertLess(info["rc_closure_h"], 0.05)
        self.assertLess(info["rc_closure_V_rel"], 0.01)
        self.assertGreater(info["rc_n_max"], 1.0)
        # one node pushed 0.5 m: the interval before it no longer closes
        bad = x.copy()
        bad[col.N * 0 + 15 * 6 + 2] += 0.5 / col.sc[15 * 6 + 2]
        rc = dense_replay(col, bad)
        self.assertFalse(rc["replay_pass"])
        self.assertGreater(rc["rc_closure_h"], 0.05)

    def test_refine_adds_nodes_to_reach_dt_max(self):
        info = solve(by_name("baseline-2m"), profile_for(12.0), "min_wind", "travel", 16,
                     h_min=1.0, tip_model="geometric", midpoints=True, seeds=(0, 1, 2),
                     stop_after=1, refine=True, dt_max=0.25)
        if info["success"]:
            self.assertGreaterEqual(info["n_nodes"], int(np.ceil(info["period"] / 0.25)) + 1)
            self.assertTrue(info["refined"])
            self.assertEqual(info["n_nodes_initial"], 16)
            self.assertTrue(info["replay_pass"])


if __name__ == "__main__":
    unittest.main()
