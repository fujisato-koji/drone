import unittest

import numpy as np

from sim.aircraft import BASELINE, derivatives
from sim.aircraft_models import by_name
from sim.dsopt import Collocation, Profile, derivs, sachs_albatross, solve
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


if __name__ == "__main__":
    unittest.main()
