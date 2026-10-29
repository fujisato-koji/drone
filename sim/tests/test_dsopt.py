import unittest

import numpy as np

from sim.aircraft import BASELINE, derivatives
from sim.aircraft_models import by_name
from sim.dsopt import (Collocation, Profile, WaveParams, derivs, profile_for, sachs_albatross,
                       solve, solve_min_wind, wave_wind_accel)
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
                                     "midpoints": False}, **kw)
        self.assertTrue(flat["success"] and wave["success"])
        self.assertAlmostEqual(wave["u10"], flat["u10"], delta=0.02)


class TestRoughness(unittest.TestCase):
    def test_self_consistent_z0(self):
        info = solve_min_wind(by_name("baseline-2m"), "self_consistent", cycle="travel", n=30,
                              h_min=1.0, tip_model="geometric", seeds=(0, 1, 2), stop_after=1)
        self.assertTrue(info["success"] and info["outer_converged"])
        self.assertAlmostEqual(info["z0"], profile_for(info["u10"]).z0, delta=0.02 * info["z0"])
        self.assertLess(info["u10"], 11.5)  # z0 at u10 = 8 overstates the minimum

    def test_fixed_z0_profile(self):
        self.assertEqual(profile_for(8.0, roughness="fixed_z0", z0=0.03).z0, 0.03)


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


if __name__ == "__main__":
    unittest.main()
