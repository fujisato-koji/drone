"""The swell estimate from the aircraft's own sensors (sim.swellest): the navigation error, the radar's footprint
inversion, the estimated world against the true one, the estimator with ideal sensors, and the sensing pilot."""
import math
import unittest

import numpy as np

from sim import swellest as se
from sim.seawind import Environment, Wind
from sim.wavelib import swell_env


class TestNavError(unittest.TestCase):
    def test_rtk_is_tighter_than_standalone(self):
        rtk = se.NavError("rtk", 200.0, seed=3)
        alone = se.NavError("standalone", 200.0, seed=3)
        self.assertLess(np.std(rtk.dz), 0.05)
        self.assertLess(rtk.q_drift, alone.q_drift)
        self.assertEqual(len(rtk.at(10.0)), 4)

    def test_error_does_not_depend_on_the_seed_layout(self):
        a = se.NavError("baro", 50.0, seed=1)
        b = se.NavError("baro", 50.0, seed=1)
        self.assertTrue(np.array_equal(a.dz, b.dz))


class TestSensors(unittest.TestCase):
    def setUp(self):
        self.flat = Environment.flat(Wind(6.0, 270.0))
        self.nav = se.NavError("rtk", 20.0, seed=0)
        for v in (self.nav.dx, self.nav.dy, self.nav.dz, self.nav.dmu):
            v[:] = 0.0

    def test_radar_inverts_the_footprint_over_a_flat_sea(self):
        sn = se.radar_nadir()
        sn.noise = 0.0
        rng = np.random.default_rng(0)
        for bank in (0.0, 10.0, 25.0):
            S = np.array([[0.0, 0.0, 8.0, 12.0, 0.0, 0.3]])
            m = se.measure(sn, self.flat, S, np.radians([bank]), np.array([1.0]), self.nav, rng, 6.0, 3.0)
            self.assertEqual(len(m), 1)
            self.assertAlmostEqual(m[0, 3], 0.0, places=6)      # the surface at the mean level

    def test_tip_sensors_carry_the_lever_of_the_bank(self):
        sn = se.radar_tip("r")
        sn.noise = 0.0
        rng = np.random.default_rng(0)
        S = np.array([[0.0, 0.0, 8.0, 12.0, 0.0, 0.0]])
        m = se.measure(sn, self.flat, S, np.radians([5.0]), np.array([1.0]), self.nav, rng, 6.0, 3.0)
        self.assertAlmostEqual(m[0, 3], 0.0, places=6)
        self.assertAlmostEqual(m[0, 6], 1.5 * math.cos(math.radians(5.0)), places=6)

    def test_lidar_loses_returns_off_nadir(self):
        sn = se.lidar_nadir()
        rng = np.random.default_rng(0)
        n = 400
        S = np.tile([0.0, 0.0, 8.0, 12.0, 0.0, 0.0], (n, 1))
        t = np.linspace(0.0, 10.0, n)
        level = se.measure(sn, self.flat, S, np.zeros(n), t, self.nav, rng, 6.0, 3.0)
        banked = se.measure(sn, self.flat, S, np.full(n, math.radians(25.0)), t, self.nav, rng, 6.0, 3.0)
        self.assertGreater(len(level), 0.3 * n)
        self.assertLess(len(banked), 0.05 * n)

    def test_rig_returns_a_list_for_fly_mpc(self):
        rig = se.SensorRig("radar_tips", self.nav, 6.0, 3.0, seed=0)
        got = rig.measure(np.array([0.0, 0.0, 8.0, 12.0, 0.0, 0.0]), 0.0, self.flat, 0.0)
        self.assertIsInstance(got, list)
        self.assertEqual(len(got), 3)


class TestWorld(unittest.TestCase):
    def test_exact_state_gives_the_true_sea_in_the_true_frame(self):
        env = swell_env(6.0, 3.0, 10.0, "along", "p63", phase=0.4)
        comps = se.swell_components(env)
        tp, beta = se.swell_peak(comps)
        om = 2.0 * np.pi / tp
        k = om * om / se.G
        dx, dy, dz = 1.3, -0.7, 0.9                   # the navigation error at the plan
        xr, yr, tr = 120.0, -40.0, 33.0              # a reference point in the navigation frame
        # the true phase at the true point (xr - dx, yr - dy), seen at the navigation point (xr, yr)
        psi = k * (math.cos(beta) * (xr - dx) + math.sin(beta) * (yr - dy)) - om * tr + 0.4
        state = {"x": [1.5, psi, om, beta, dz, 0.0], "ref": [xr, yr, tr]}
        w = se.estimated_world(state, 6.0, 270.0, "p63", (dx, dy, dz))
        x, y, t = np.linspace(-50, 300, 7), np.linspace(-20, 40, 7), np.linspace(0, 60, 7)
        self.assertTrue(np.allclose(w.surface(x, y, t)[0], env.surface(x, y, t)[0], atol=1e-9))
        z = np.asarray(env.surface(x, y, t)[0]) + 5.0
        self.assertTrue(np.allclose(w.wind_at(x, y, z, t), env.wind_at(x, y, z, t), atol=1e-9))


class TestEstimator(unittest.TestCase):
    def test_ideal_sensors_find_the_swell(self):
        env, names = se.error_sea("regular", 3.0, 10.0, "along", 0)
        comps = se.swell_components(env, names)
        T, S, MU = se.path_states("survey_ride", env, comps, 90.0, 0)
        nav = se.NavError("rtk", 90.0, 0)
        for v in (nav.dx, nav.dy, nav.dz, nav.dmu):
            v[:] = 0.0
        nav.q_drift, nav.r_fast = 1e-8, 1e-6
        d = se.sample_path("radar_tips", env, T, S, MU, nav, 0)
        est = se.run_estimator(se.SwellEKF(nav), d, 90.0)
        self.assertTrue(est.ready)
        rows = np.array(se.evaluate(est, env, comps, T, S, nav))
        late = rows[rows[:, 0] > 50.0]
        self.assertLess(np.max(np.abs(late[:, 1])), 0.05)
        self.assertLess(np.max(np.abs(late[:, 2])), 0.05)


class TestSensingPilot(unittest.TestCase):
    def test_start_then_survey_until_ready(self):
        from sim.aircraft_models import by_name
        from sim.wavelib import flat_start_entry
        ac = by_name("albatross-3m")
        start = flat_start_entry(ac, 6.0, 0.0)

        class Inner:
            last = None
            sig = "inner"

            def cycle(self, *a):
                return "wave"

        class Est:
            ready, t_last = False, 0.0

        est = Est()
        sp = se.SensingPilot(Inner(), est, ac, 6.0, start, 0.0)
        self.assertIs(sp.cycle("albatross-3m", None, 6.0, 0.0, 0.0).d, start)
        est.t_last = se.SURVEY_START_S + 0.1
        self.assertIsNot(sp.cycle("albatross-3m", None, 6.0, 0.0, 0.0).d, start)
        est.ready = True
        self.assertEqual(sp.cycle("albatross-3m", None, 6.0, 0.0, 0.0), "wave")
        self.assertEqual(sp.sig, "inner")


if __name__ == "__main__":
    unittest.main()
