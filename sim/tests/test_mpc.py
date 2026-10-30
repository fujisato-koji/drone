import unittest

import numpy as np

from sim.aircraft import BASELINE, G, Actuators, derivatives
from sim.aircraft import clip_controls, n_limited_cl, rk4_step
from sim.dsopt import folded_dcd, thrust_max
from sim.mpc import (Cycle, CycleGuide, CycleLibrary, MPPIPlanner, Reflex, burst_stats,
                     derivatives_vec, energy_weight, fold_effective, fly_mpc, wrap)
from sim.seawind import Environment, WaveTrain, Wind


def wave_env(u10=13.0):
    w = Wind(u10, 270.0)
    return Environment(w, [WaveTrain.fully_developed(w, n_omega=4, n_theta=3)], seed=1)


class TestMPC(unittest.TestCase):
    def test_vectorized_dynamics_match(self):
        env = wave_env()
        rng = np.random.default_rng(0)
        S = 20
        s = np.stack([rng.uniform(-50, 50, S), rng.uniform(-50, 50, S), rng.uniform(0.2, 12, S),
                      rng.uniform(8, 20, S), rng.uniform(-0.4, 0.4, S), rng.uniform(-3, 3, S)])
        cl, mu = rng.uniform(0.0, 1.2, S), rng.uniform(-1.0, 1.0, S)
        dv = derivatives_vec(3.3, s, BASELINE, env, cl, mu)
        for i in range(S):
            d = derivatives(3.3, s[:, i], BASELINE, env, cl[i], mu[i])[0]
            self.assertLess(np.max(np.abs(dv[:, i] - d)) / np.max(np.abs(d)), 1e-9)

    def test_contact_costs_more(self):
        env = Environment.flat(Wind(13.0, 270.0))
        pl = MPPIPlanner(BASELINE, n_samples=4, dt=0.1, horizon_s=4.0, n_ctrl=5)
        s0 = np.array([0.0, 0.0, 8.0, 10.0, 0.0, 0.0])
        level = np.array([[BASELINE.cl_best_ld] * 5, [0.0] * 5])
        dive = np.array([[-0.3] * 5, [0.0] * 5])     # pushes the nose down into the sea
        cost, _, dead = pl.rollout(env, 0.0, s0, np.stack([level, dive]), (1.0, 0.0))
        self.assertFalse(dead[0])
        self.assertTrue(dead[1])
        self.assertGreater(cost[1], cost[0] + 1000.0)

    def test_flat_sea_truth_stays_aloft_60s(self):
        env = Environment.flat(Wind(13.0, 270.0))
        r = fly_mpc(BASELINE, env, "truth", [], 60.0, env.wind.to_deg - 20.0, 13.0, seed=1,
                    planner_kw={"n_samples": 96, "dt": 0.1, "n_iter": 1})
        self.assertEqual(r["ended"], "complete")
        self.assertGreater(r["min_clear_surface"], 0.0)


class TestMotor(unittest.TestCase):
    def test_vectorized_dynamics_with_thrust_and_folded_drag_match(self):
        env = wave_env()
        rng = np.random.default_rng(1)
        S = 12
        s = np.stack([rng.uniform(-50, 50, S), rng.uniform(-50, 50, S), rng.uniform(0.5, 12, S),
                      rng.uniform(8, 20, S), rng.uniform(-0.4, 0.4, S), rng.uniform(-3, 3, S)])
        cl, mu = rng.uniform(0.0, 1.2, S), rng.uniform(-1.0, 1.0, S)
        th = rng.uniform(0.0, 5.0, S)
        th[0] = 0.0
        dcd = 3e-4 / BASELINE.wing_area
        dv = derivatives_vec(3.3, s, BASELINE, env, cl, mu, thrust=th, fold_dcd=dcd)
        for i in range(S):
            d = derivatives(3.3, s[:, i], BASELINE, env, cl[i], mu[i], thrust=th[i], fold_dcd=dcd)[0]
            self.assertLess(np.max(np.abs(dv[:, i] - d)) / np.max(np.abs(d)), 1e-9)
        # the folded propeller costs speed with the motor off
        d0 = derivatives(3.3, s[:, 1], BASELINE, env, cl[1], mu[1])[0]
        d1 = derivatives(3.3, s[:, 1], BASELINE, env, cl[1], mu[1], fold_dcd=dcd)[0]
        self.assertLess(d1[3], d0[3])

    def test_thrust_lag(self):
        a = Actuators(BASELINE, rate_model="pb2v", thrust_tau=0.1)
        a.step(0.5, 0.0, 0.01, 10.0, thrust=0.0)
        self.assertEqual(a.T, 0.0)
        for _ in range(10):
            a.step(0.5, 0.0, 0.01, 10.0, thrust=5.0)
        self.assertAlmostEqual(a.T, 5.0 * (1.0 - np.exp(-1.0)), places=9)   # one time constant

    def test_motor_planner_shapes_and_nonnegative_thrust(self):
        pl = MPPIPlanner(BASELINE, n_samples=16, dt=0.1, horizon_s=3.0, n_ctrl=6, motor=True)
        self.assertEqual(pl._initial_plan().shape, (3, 6))
        self.assertEqual(pl._candidates().shape, (6, 3, 6))
        eps = np.random.default_rng(0).standard_normal((16, 3, 6)) * pl.sigma_t
        Us = pl._clip(np.zeros((16, 3, 6)) + eps)
        self.assertGreaterEqual(Us[:, 2].min(), 0.0)
        self.assertLessEqual(Us[:, 2].max(), pl.t_max)
        self.assertAlmostEqual(pl.sigma_t, 0.15 * pl.t_max)
        off = MPPIPlanner(BASELINE, n_samples=16, dt=0.1, horizon_s=3.0, n_ctrl=6)
        self.assertEqual(off._initial_plan().shape, (2, 6))
        self.assertFalse(hasattr(off, "t_max"))

    def test_thrust_gains_energy_and_costs_energy_weight(self):
        env = Environment.flat(Wind(6.0, 270.0))
        s0 = np.array([0.0, 0.0, 8.0, 10.0, 0.0, 0.0])
        for w in (0.0, 1.0):
            pl = MPPIPlanner(BASELINE, n_samples=2, dt=0.1, horizon_s=4.0, n_ctrl=5, motor=True,
                             w_energy=w, w_e=1.0)
            glide = np.array([[BASELINE.cl_best_ld] * 5, [0.0] * 5, [0.0] * 5])
            burn = glide.copy()
            burn[2] = 0.3 * pl.t_max
            cost, path, dead = pl.rollout(env, 0.0, s0, np.stack([glide, burn]), (1.0, 0.0))
            self.assertFalse(dead.any())
            e = G * path[-1, 2] + 0.5 * path[-1, 3] ** 2
            self.assertGreater(e[1], e[0])                     # the thrust adds energy
            if w == 0.0:
                self.assertLess(cost[1], cost[0])              # free electricity: worth it
            else:
                self.assertGreater(cost[1], cost[0])           # costly electricity: not here

    def test_energy_weight_levels_increase(self):
        w = [energy_weight(BASELINE, k) for k in ("free", "normal", "costly")]
        self.assertTrue(w[0] < w[1] < w[2])

    def test_burst_stats(self):
        dt = 0.01
        T = np.zeros(1000)
        T[100:300] = 0.5 * BASELINE.mass * G      # 2 s
        T[600:650] = 0.3 * BASELINE.mass * G      # 0.5 s
        V = np.full(1000, 10.0)
        pl = MPPIPlanner(BASELINE, motor=True)
        m = burst_stats(BASELINE, pl, dt, T, V, np.full(1000, 5.0), np.linspace(0, 0.999, 1000),
                        np.linspace(0, 0.999, 1000))
        self.assertEqual(m["n_bursts"], 2)
        self.assertAlmostEqual(m["thrust_on_share"], 0.25, places=9)
        self.assertAlmostEqual(m["burst_len_median_s"], 1.25, places=9)
        e = (200 * 0.5 + 50 * 0.3) * BASELINE.mass * G * 10.0 * dt / pl.eta
        self.assertAlmostEqual(m["energy_j"], e, places=6)
        self.assertAlmostEqual(m["avg_power_w"], e / 10.0, places=6)
        self.assertEqual(m["burst_phase_hist8"], [1, 0, 0, 0, 1, 0, 0, 0])

    def test_fly_mpc_motor_logs_and_off_has_no_motor_key(self):
        env = Environment.flat(Wind(9.0, 270.0))
        kw = {"n_samples": 32, "dt": 0.1, "n_iter": 1}
        r = fly_mpc(BASELINE, env, "flat", [], 8.0, env.wind.to_deg - 20.0, 9.0, seed=1, planner_kw=kw,
                    actuators=Actuators(BASELINE, rate_model="pb2v"), n_limit=True, motor=True,
                    w_energy=0.01)
        m = r["motor"]
        self.assertAlmostEqual(m["energy_j"], m["avg_power_w"] * m["duration_s"], places=6)
        self.assertGreater(m["energy_j"], 0.0)
        r0 = fly_mpc(BASELINE, env, "flat", [], 3.0, env.wind.to_deg - 20.0, 9.0, seed=1, planner_kw=kw)
        self.assertNotIn("motor", r0)


def synthetic_cycle(mirror=False):
    n, T = 40, 6.0
    t = np.linspace(0.0, T, n)
    w = 2 * np.pi / T
    d = {"period": T, "t": t, "x": 12.0 * t, "y": 3.0 * t + np.sin(w * t),
         "z": 6.0 + 5.0 * np.cos(w * t), "V": 14.0 + 3.0 * np.sin(w * t),
         "gamma": 0.3 * np.sin(w * t), "psi": 1.0 * np.sin(w * t),
         "cl": 0.8 + 0.3 * np.cos(w * t), "mu": 0.6 * np.sin(w * t)}
    return Cycle(d, mirror)


class TestLibrary(unittest.TestCase):
    def test_phase_match_recovers_phase(self):
        cyc = synthetic_cycle()
        env = Environment.flat(Wind(13.0, 270.0))
        psi_w = float(np.arctan2(env.wind.ey, env.wind.ex))
        tau = 2.1
        h, V, g, pr = cyc.state(tau)
        s = [5.0, 7.0, h, V, g, psi_w + pr]
        gd = CycleGuide(cyc, env, 0.0, s, psi_w, 6.0)
        dphase = (gd.tau0 - tau + 3.0) % 6.0 - 3.0
        self.assertLess(abs(dphase), 0.1)
        self.assertEqual(gd.delta, 0.0)            # flat sea: no wave, no shift
        self.assertEqual(gd.nominal(np.linspace(0, 6, 12)).shape, (2, 12))
        # the terminal cost is small at the cycle's own end state, larger away from it
        ht, Vt, gt, pt = gd.target
        end = np.array([[0.0], [0.0], [ht], [Vt], [gt], [psi_w + pt]])
        near = gd.terminal_cost(env, 6.0, end, type("P", (), {"w_e": 1.0})())
        far = gd.terminal_cost(env, 6.0, end + np.array([[0], [0], [5.0], [0], [0], [0]]),
                               type("P", (), {"w_e": 1.0})())
        self.assertLess(float(near[0]), 1.0)
        self.assertGreater(float(far[0]), float(near[0]) + 5.0)

    def test_mirror_flips_turn(self):
        a, b = synthetic_cycle(), synthetic_cycle(True)
        self.assertTrue(np.allclose(a.mu, -b.mu))
        self.assertTrue(np.allclose(a.x, b.x) and np.allclose(a.y, -b.y))

    def test_wave_aware_shift_moves_low_point_toward_forward_face(self):
        cyc = synthetic_cycle()
        w = Wind(13.0, 270.0)
        env = Environment(w, [WaveTrain.fully_developed(w, n_omega=2, n_theta=1)], seed=3)
        psi_w = float(np.arctan2(w.ey, w.ex))
        h, V, g, pr = cyc.state(1.0)
        eta0 = float(env.surface(0.0, 0.0, 0.0)[0])
        s = [0.0, 0.0, h + eta0, V, g, psi_w + pr]
        g0 = CycleGuide(cyc, env, 0.0, s, psi_w, 6.0, wave_aware=False)
        g1 = CycleGuide(cyc, env, 0.0, s, psi_w, 6.0, wave_aware=True)
        self.assertEqual(g0.delta, 0.0)
        self.assertLessEqual(abs(g1.delta), 0.15 * cyc.T + 1e-9)
        err = lambda th: abs(float(wrap(th - np.pi / 3)))
        self.assertIsNotNone(g1.theta_low)
        # the scan includes delta = 0, so the chosen shift cannot be worse than no shift
        self.assertLessEqual(err(g1.theta_low), err(g1._theta0) + 1e-9)

    def test_planner_with_guide_follows_nominal_in_flat_sea(self):
        cyc = synthetic_cycle()
        env = Environment.flat(Wind(13.0, 270.0))
        psi_w = float(np.arctan2(env.wind.ey, env.wind.ex))
        h, V, g, pr = cyc.state(0.0)
        s = np.array([0.0, 0.0, h, V, g, psi_w + pr])
        pl = MPPIPlanner(BASELINE, n_samples=32, dt=0.1, n_iter=1, sigma_cl=0.05, sigma_mu=0.05,
                         clearance="wingtip", h_clear=0.3)
        gd = CycleGuide(cyc, env, 0.0, s, psi_w, pl.horizon_s)
        plan = pl.plan(env, 0.0, s, (1.0, 0.0), guide=gd)
        nom = gd.nominal(pl.knots_t)
        self.assertLess(np.max(np.abs(plan.cl_knots - nom[0])), 0.5)
        self.assertEqual(plan.cl.shape, plan.t.shape)


class TestReflexAndSampling(unittest.TestCase):
    def setUp(self):
        self.world = Environment.flat(Wind(9.0, 270.0))
        self.ac = BASELINE
        self.rf = Reflex(self.ac, thrust_max(self.ac), folded_dcd(self.ac), True)

    def dive_plan(self, cl=0.05):
        kt = np.linspace(0.0, 6.0, 24)
        return kt, np.full(24, cl), np.full(24, 0.0), np.zeros(24)

    def test_fires_on_a_dive_and_clears_when_overridden(self):
        ac, rf, world = self.ac, self.rf, self.world
        kt, cl, mu, th = self.dive_plan()
        s = np.array([0.0, 0.0, 3.0, 12.0, -0.35, 0.0])
        hit = rf.predict(world, 0.0, s, kt, cl, mu, th, 0.0)
        self.assertIsNotNone(hit)
        self.assertEqual(hit[0], "centre")
        self.assertLess(hit[1], rf.horizon_s)
        # fly the override until the prediction (of the same dive plan) clears
        t, cleared, hmin, n_on = 0.0, False, 1e9, 0
        for _ in range(400):
            hit = rf.predict(world, t, s, kt, cl, mu, th, 0.0)
            if hit is None:
                cleared = True
                break
            n_on += 1
            c, m, tt = rf.override(s)
            c = n_limited_cl(ac, s[3], c)
            s = rk4_step(lambda a, b: derivatives(a, b, ac, world, c, m, True, thrust=tt,
                                                  fold_dcd=folded_dcd(ac))[0], t, s, 0.01)
            t += 0.01
            hmin = min(hmin, s[2])
        self.assertTrue(cleared, "the override never cleared the prediction")
        self.assertGreater(hmin, 0.0)
        self.assertGreater(n_on, 0)

    def test_stays_quiet_in_level_flight_and_fires_on_stall(self):
        rf, world = self.rf, self.world
        kt, cl, mu, th = self.dive_plan(cl=self.ac.cl_best_ld)
        self.assertIsNone(rf.predict(world, 0.0, np.array([0.0, 0.0, 12.0, 11.0, -0.05, 0.0]),
                                     kt, cl, mu, th, 0.0))
        hit = rf.predict(world, 0.0, np.array([0.0, 0.0, 30.0, 1.02 * self.ac.v_stall, 0.0, 0.0]),
                         kt, cl, mu, th, 0.0)
        self.assertEqual(hit[0], "stall")

    def test_fold_effective_gives_the_state_switch(self):
        ac, dcd = self.ac, folded_dcd(self.ac)
        mg = ac.mass * G
        f = lambda T: 1.0 - T / (T + 0.01 * mg)
        # off (command below 0.5 % of the weight): the full folded drag whatever the lagging thrust is
        for t_act in (0.0, 0.002 * mg):
            self.assertAlmostEqual(float(fold_effective(ac, dcd, t_act, 0.001 * mg)) * f(t_act), dcd, places=15)
        self.assertEqual(float(fold_effective(ac, dcd, 0.1 * mg, 0.1 * mg)), 0.0)

    def test_sustained_sampling_and_candidates(self):
        kw = dict(n_samples=64, dt=0.1, horizon_s=3.0, n_ctrl=6, motor=True)
        pl = MPPIPlanner(BASELINE, thrust_sampling="sustained", **kw)
        eT = pl._thrust_noise(4000)
        self.assertEqual(eT.shape, (4000, 1, 6))
        const = [(np.ptp(e[0][e[0] != 0]) == 0 and (e[0] != 0).any()) for e in eT[:400]]
        self.assertGreater(np.mean(const), 0.3)       # about half hold one level
        U = pl._initial_plan()
        c = pl._thrust_candidates(U)
        self.assertEqual(c.shape, (6, 3, 6))
        mg = BASELINE.mass * G
        np.testing.assert_allclose(c[:, 2, 0], np.repeat([0.05, 0.10, 0.20], 2) * mg)
        # basic sampling leaves the earlier draw sequence alone
        a = MPPIPlanner(BASELINE, seed=3, **kw)._thrust_noise(8)
        b = 0.15 * thrust_max(BASELINE) * np.random.default_rng(3).standard_normal((8, 1, 6))
        np.testing.assert_allclose(a, b)

    def test_fly_mpc_reflex_logs_and_off_has_no_key(self):
        env = Environment.flat(Wind(9.0, 270.0))
        kw = {"n_samples": 32, "dt": 0.1, "n_iter": 1}
        args = (BASELINE, env, "flat", [], 6.0, env.wind.to_deg - 20.0, 9.0)
        r = fly_mpc(*args, seed=1, planner_kw=kw, actuators=Actuators(BASELINE, rate_model="pb2v"),
                    n_limit=True, motor=True, w_energy=0.01, reflex=True)
        self.assertIn("reflex", r)
        for k in ("n", "total_s", "energy_j", "causes", "events"):
            self.assertIn(k, r["reflex"])
        r0 = fly_mpc(*args, seed=1, planner_kw=kw, motor=True, w_energy=0.01)
        self.assertNotIn("reflex", r0)
        with self.assertRaises(ValueError):
            fly_mpc(*args, seed=1, planner_kw=kw, reflex=True)

    def test_motor_library_cycle_carries_its_thrust(self):
        d = synthetic_cycle().d
        d = dict(d, thrust=3.0 + 2.0 * np.sin(2 * np.pi * np.asarray(d["t"]) / d["period"]))
        cyc = Cycle(d)
        self.assertTrue(cyc.has_thrust)
        self.assertFalse(synthetic_cycle().has_thrust)
        pl = MPPIPlanner(BASELINE, n_samples=16, dt=0.1, horizon_s=3.0, n_ctrl=6, motor=True)
        world = Environment.flat(Wind(9.0, 270.0))
        s0 = np.array([0.0, 0.0, 8.0, 12.0, 0.0, 0.0])
        g = CycleGuide(cyc, world, 0.0, s0, 0.0, 3.0, wave_aware=False)
        plan = pl.plan(world, 0.0, s0, np.array([1.0, 0.0]), guide=g)
        self.assertEqual(plan.thrust_knots.shape, (6,))
        np.testing.assert_allclose(g.nominal_thrust(pl.knots_t),
                                   np.interp((g.tau_s + pl.knots_t) % cyc.T, cyc.t, d["thrust"]))


if __name__ == "__main__":
    unittest.main()
