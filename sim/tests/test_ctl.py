"""Actuators, the load-factor limit, the two-parameter profile fit and the planner on estimates."""
import unittest

import numpy as np

from sim.aircraft import (BASELINE, Actuators, G, clip_controls, derivatives, n_limited_cl,
                          simulate)
from sim.estimator import Estimator
from sim.seawind import Environment, Wind
from sim.sensors import SensorSuite, truth_from_state


def log_env(z0, u10=13.0):
    """Flat sea with a log profile of the given z0 and u10 (u* from U(10) = u10)."""
    env = Environment.flat(Wind(u10, 270.0))
    env.wind.z0 = z0
    env.wind.ustar = 0.4 * u10 / np.log(10.0 / z0)
    return env


def profile_fit_case(z0, mode="two_param", u10=13.0, t_end=60.0, suite="standard", seed=3):
    """Standard suite on a prescribed loop (height 1 to 15 m every 12 s, heading turning once per
    24 s, 15 m/s) in a flat log wind; returns the Estimate at t_end."""
    env = log_env(z0, u10)
    su = getattr(SensorSuite, suite)(seed)
    est = Estimator(su.names, profile_fit=mode)
    dt, x, y, out = 0.02, 0.0, 0.0, None
    for i in range(int(t_end / dt) + 1):
        t = i * dt
        s = np.array([x, y, 8.0 + 7.0 * np.sin(2 * np.pi * t / 12.0), 15.0, 0.0,
                      2 * np.pi * t / 24.0])
        tr = truth_from_state(s, env, t)
        out = est.update(su.measure(tr, t), t)
        x += tr["v_ground"][0] * dt
        y += tr["v_ground"][1] * dt
    return out


class TestActuators(unittest.TestCase):
    def test_lag_and_rate_limit(self):
        ac = BASELINE
        a = Actuators(ac, tau=0.1, mu_rate_deg_s=150.0, cl_rate=2.0)
        dt = 0.01
        self.assertEqual(a.step(0.5, 0.0, dt), (0.5, 0.0))       # starts at the command
        # small cl step (inside the rate limit): first-order, 63 % after one time constant
        for _ in range(10):
            cl, mu = a.step(0.6, 0.0, dt)
        self.assertAlmostEqual((cl - 0.5) / 0.1, 1 - np.exp(-1.0), delta=0.02)
        self.assertEqual(a.n_cl_rate, 0)
        # large bank step: every move is at most the rate limit
        prev = mu
        for _ in range(20):
            cl, mu = a.step(0.6, 1.0, dt)
            self.assertLessEqual(abs(mu - prev), np.radians(150.0) * dt + 1e-12)
            prev = mu
        self.assertGreater(a.n_mu_rate, 15)
        self.assertAlmostEqual(prev, 20 * np.radians(150.0) * dt, places=6)
        # commands are clipped to the aircraft limits
        b = Actuators(ac)
        b.step(5.0, 9.0, dt)
        self.assertEqual((b.cl, b.mu), clip_controls(ac, 5.0, 9.0))

    def test_zero_tau_is_rate_only(self):
        a = Actuators(BASELINE, tau=0.0, mu_rate_deg_s=100.0, cl_rate=2.0)
        a.step(0.5, 0.0, 0.01)
        _, mu = a.step(0.5, 0.01, 0.01)
        self.assertAlmostEqual(mu, 0.01)


class TestPb2v(unittest.TestCase):
    def test_delay(self):
        ac, dt = BASELINE, 0.01
        a = Actuators(ac, rate_model="pb2v", tau=1e-9, delay_s=0.05, cl_rate=1e3, k_pb2v=1e3)
        a.step(0.5, 0.0, dt, 15.0)
        out = [a.step(0.5, 0.3, dt, 15.0)[1] for _ in range(8)]
        self.assertEqual(out[:5], [0.0] * 5)                 # 5 steps of delay = 0.05 s
        self.assertAlmostEqual(out[5], 0.3, places=6)

    def test_speed_dependent_limit(self):
        ac, dt = BASELINE, 0.01
        for V in (10.0, 20.0):
            a = Actuators(ac, rate_model="pb2v", tau=1e-9, delay_s=0.0)
            a.step(0.5, 0.0, dt, V)
            _, mu = a.step(0.5, 1.0, dt, V)
            self.assertAlmostEqual(mu, 0.08 * 2 * V / ac.span * dt)
        self.assertEqual(Actuators(ac, rate_model="pb2v").cl_rate, 1.0)
        with self.assertRaises(ValueError):
            a.step(0.5, 0.0, dt)

    def test_planner_uses_model(self):
        from sim.mpc import MPPIPlanner
        env = log_env(2e-4)
        pl = MPPIPlanner(BASELINE, n_samples=4, n_ctrl=6, horizon_s=2.0, dt=0.1)
        U = np.stack([np.full((4, 6), 0.6), np.zeros((4, 6))], axis=1)
        U[:, 1, 2:] = 1.0                                   # step to 1 rad of bank
        s0 = np.array([0.0, 0.0, 30.0, 15.0, 0.0, 0.0])
        _, p0, _ = pl.rollout(env, 0.0, s0, U, np.array([1.0, 0.0]))
        pl.act = Actuators(BASELINE, rate_model="pb2v")
        _, p1, _ = pl.rollout(env, 0.0, s0, U, np.array([1.0, 0.0]))
        self.assertGreater(np.abs(p0[-1, 5, 0] - p1[-1, 5, 0]), 0.01)   # heading lags


class TestNLimit(unittest.TestCase):
    def test_cap(self):
        ac = BASELINE
        V = 30.0
        cl = n_limited_cl(ac, V, 1.2)
        n = 0.5 * ac.rho * V ** 2 * ac.wing_area * cl / (ac.mass * G)
        self.assertAlmostEqual(n, ac.n_max)
        self.assertEqual(n_limited_cl(ac, 12.0, 0.3), 0.3)       # below the limit: untouched

    def test_derivatives_option(self):
        env = log_env(2e-4)
        s = [0.0, 0.0, 10.0, 30.0, 0.1, 0.3]
        d0 = derivatives(0.0, s, BASELINE, env, 1.2, 0.5)[0]
        d1 = derivatives(0.0, s, BASELINE, env, 1.2, 0.5, n_limit=True)[0]
        d2 = derivatives(0.0, s, BASELINE, env, n_limited_cl(BASELINE, 30.0, 1.2), 0.5)[0]
        self.assertFalse(np.allclose(d0, d1))
        np.testing.assert_allclose(d1, d2)
        np.testing.assert_allclose(derivatives(0.0, s, BASELINE, env, 0.4, 0.5, n_limit=True)[0],
                                   derivatives(0.0, s, BASELINE, env, 0.4, 0.5)[0])

    def test_simulate_pullup(self):
        env = log_env(2e-4)
        ctrl = lambda t, s, e, a: (1.2, 0.0)
        s0 = [0.0, 0.0, 100.0, 30.0, 0.0, 0.0]
        r0 = simulate(BASELINE, env, ctrl, s0, 0.0, 1.0)
        r1 = simulate(BASELINE, env, ctrl, s0, 0.0, 1.0, n_limit=True)
        self.assertGreater(r0["n"].max(), BASELINE.n_max + 1.0)
        self.assertLessEqual(r1["n"].max(), BASELINE.n_max + 1e-9)

    def test_fly_records(self):
        from sim.autonomy import (EstimatedDSController, PerfectEstimator, default_cycle, fly,
                                  initial_state, make_env)
        ac, env = BASELINE, make_env("flat", 13.0)
        cyc = default_cycle(ac, 13.0)

        def run(**kw):
            return fly(ac, env, None, PerfectEstimator(), EstimatedDSController(ac, cyc),
                       initial_state(ac, env, cyc), 8.0, **kw)
        r0 = run()
        np.testing.assert_allclose(r0["mu_act"], r0["mu"])
        np.testing.assert_allclose(r0["n_flown"], r0["n"])
        r1 = run(actuators=Actuators(ac), n_limit=True)
        self.assertLessEqual(r1["metrics"]["max_n_flown"], ac.n_max + 1e-9)
        self.assertIn("actuator_bind", r1["metrics"])
        self.assertFalse(np.allclose(r1["mu_act"], r1["mu"]))


class TestProfileFit(unittest.TestCase):
    def test_two_param(self):
        for z0 in (1e-4, 3e-2):
            e = profile_fit_case(z0)
            self.assertLess(abs(np.log(e.z0_est / z0)), np.log(3.0), (z0, e.z0_est))
            self.assertLess(abs(e.u10_est / 13.0 - 1.0), 0.05, (z0, e.u10_est))
            self.assertAlmostEqual(e.ustar_est / 0.4 * np.log(10.0 / e.z0_est), e.u10_est, 6)

    def test_fixed_exposes_u10(self):
        e = profile_fit_case(1e-4, mode="fixed_z0")
        self.assertEqual(e.z0_est, 2e-4)
        self.assertLess(abs(e.u10_est / 13.0 - 1.0), 0.08)

    def test_rejections(self):
        est = Estimator(SensorSuite.standard().names, profile_fit="two_param")
        # span under 3 m
        est._buf.extend((i * 0.1, 5.0 + (i % 2), 8.0, 0.0) for i in range(100))
        est._fit_profile()
        self.assertEqual(est._fit, (0.0, 0.0))
        # wind decreasing with height (a <= 0)
        est._buf.clear()
        est._buf.extend((i * 0.1, 1.0 + 0.1 * i, 12.0 - 0.3 * i, 0.0) for i in range(60))
        est._fit_profile()
        self.assertEqual(est._fit, (0.0, 0.0))
        self.assertTrue(np.isnan(est._z0_fit))

    def test_z0_clip(self):
        est = Estimator(SensorSuite.standard().names, profile_fit="two_param")
        h = np.linspace(2.0, 14.0, 60)
        est._buf.extend((i * 0.1, hh, 0.2 * np.log(hh) + 4.0, 0.0) for i, hh in enumerate(h))
        est._fit_profile()                     # c > 0 means z0 = exp(-c / a) tiny: clipped
        self.assertEqual(est._z0_fit, 1e-6)


class TestFlyMpcEstimates(unittest.TestCase):
    def test_short_flight(self):
        from sim.aircraft_models import by_name
        from sim.autonomy import make_env
        from sim.mpc import LIB_PKW, CycleLibrary, fly_mpc
        ac, env = by_name("baseline-2m"), make_env("flat", 13.0)
        lib = CycleLibrary.load()
        args = (ac, env, "flat", [], 12.0, env.wind.to_deg - 20.0, 13.0)
        kw = dict(seed=1, planner_kw=dict(LIB_PKW), library=lib)
        a = fly_mpc(*args, **kw)
        b = fly_mpc(*args, state_source="truth", wind_source="truth", **kw)
        for k in ("survival_s", "min_clear_surface", "max_n", "cycles"):
            self.assertEqual(a[k], b[k])
        e = fly_mpc(*args, state_source="estimate", wind_source="estimate",
                    actuators=Actuators(ac), n_limit=True, **kw)
        self.assertEqual(e["ended"], "complete")
        self.assertGreater(e["prelude_s"], 0.0)
        self.assertLess(e["t_ready_s"], 10.0)
        self.assertTrue(e["est_log"])
        self.assertLess(e["est_summary"]["profile_dir_err_rms_deg"], 5.0)
        self.assertLessEqual(e["max_n_flown"], ac.n_max + 1e-9)
        with self.assertRaises(ValueError):
            fly_mpc(ac, env, "truth", [], 1.0, 0.0, 13.0, wind_source="estimate")

    def test_tip_stop_and_planner_nlimit(self):
        from sim.aircraft import wind_derivs
        from sim.aircraft_models import by_name
        from sim.autonomy import make_env
        from sim.mpc import LIB_PKW, CycleLibrary, derivatives_vec, fly_mpc
        ac, env = by_name("baseline-2m"), make_env("flat", 13.0)
        # vectorized derivatives with n_limit equal the scalar ones
        s = np.array([[0.0], [0.0], [10.0], [30.0], [0.1], [0.3]])
        dv = derivatives_vec(0.0, s, ac, env, np.array([1.2]), np.array([0.5]), True)[:, 0]
        ds = derivatives(0.0, s[:, 0], ac, env, 1.2, 0.5, n_limit=True)[0]
        np.testing.assert_allclose(dv, ds, rtol=1e-9, atol=1e-9)
        # a bank low over the water ends the run on the tip before the centre
        kw = dict(seed=1, planner_kw=dict(LIB_PKW), library=CycleLibrary.load())
        args = (ac, env, "flat", [], 60.0, env.wind.to_deg - 20.0, 13.0)
        r = fly_mpc(*args, n_limit=True, actuators=Actuators(ac), stop_on_tip=True, **kw)
        self.assertIn(r["ended"], ("complete", "contact", "tip contact"))
        if r["ended"] == "complete":
            self.assertGreaterEqual(r["min_clear_wingtip"], 0.0)
        self.assertLessEqual(r["max_n_cmd"], ac.n_max + 1e-9)


if __name__ == "__main__":
    unittest.main()
