import unittest

import numpy as np

from sim.aircraft import (BASELINE, G, derivatives, ground_effect_factor, ground_effect_phi)
from sim.aircraft_models import by_name
from sim.dsopt import (Collocation, Profile, WaveParams, derivs, derivs_wave, dyn_jac,
                       dyn_jac_wave, folded_dcd, profile_for, prop_efficiency, solve, thrust_max)
from sim.mpc import derivatives_vec
from sim.seawind import Environment, Wind


def fd_jacobian(fun, x, eps=1e-6):
    cols = []
    for i in range(x.size):
        xp, xm = x.copy(), x.copy()
        xp[i] += eps
        xm[i] -= eps
        cols.append((fun(xp) - fun(xm)) / (2 * eps))
    return np.stack(cols, axis=-1)


class TestGroundEffectFactor(unittest.TestCase):
    def test_wieselsberger_bank_zero(self):
        for x in np.linspace(1 / 30, 0.25, 12):
            ref = 1 - (1 - 1.32 * x) / (1.05 + 7.4 * x)
            self.assertAlmostEqual(float(ground_effect_factor(x, 0.0)) / ref, 1.0, delta=0.01)

    def test_image_method_reference_numbers(self):
        def k(xc, deg):
            return float(ground_effect_factor(xc, np.radians(deg)))

        for deg, ref in zip((0, 15, 30), (0.811, 0.796, 0.730)):
            self.assertAlmostEqual(k(0.3, deg) / ref, 1.0, delta=0.02)
        for deg, ref in zip((0, 15, 30, 45, 60), (0.906, 0.903, 0.891, 0.869, 0.832)):
            self.assertAlmostEqual(k(0.5, deg) / ref, 1.0, delta=0.02)
        for deg, ref in zip((0, 15, 30, 45, 60), (0.342, 0.625, 0.730, 0.784, 0.813)):
            xc = 0.05 + 0.5 * np.sin(np.radians(deg))  # lowest tip at 0.05 b
            self.assertAlmostEqual(k(xc, deg) / ref, 1.0, delta=0.02)

    def test_direct_integral_matches_table(self):
        from sim.groundeffect import image_ratio
        for xc, deg in ((0.12, 0.0), (0.4, 20.0), (0.7, 50.0), (0.9, 12.0)):
            bk = np.radians(deg)
            self.assertAlmostEqual(float(ground_effect_factor(xc, bk)), image_ratio(xc, bk),
                                   delta=2e-3)

    def test_small_bank_matches_integral(self):
        from sim.groundeffect import image_ratio
        for deg in (0.5, 0.75, 1.0, 1.5, 2.5, 4.0, 5.0):
            for xt in (0.02, 0.03, 0.05, 0.08, 0.15, 0.3):
                bk = np.radians(deg)
                xc = xt + 0.5 * np.sin(bk)
                self.assertAlmostEqual(float(ground_effect_factor(xc, bk)) / image_ratio(xc, bk),
                                       1.0, delta=0.01, msg=(deg, xt))

    def test_limits_and_symmetry(self):
        self.assertEqual(float(ground_effect_factor(10.0, 0.7)), 1.0)
        self.assertGreater(float(ground_effect_factor(2.0, 0.3)), 0.99)
        self.assertAlmostEqual(float(ground_effect_factor(0.4, 0.5)),
                               float(ground_effect_factor(0.4, -0.5)), places=12)
        # tip below the surface is clipped, not extrapolated
        k = float(ground_effect_factor(0.1, 1.3))
        self.assertTrue(0.0 < k < 1.0)
        self.assertEqual(k, float(ground_effect_factor(0.02 + 0.5 * np.sin(1.3), 1.3)))

    def test_surface_normal_and_relative_bank(self):
        b, h, mu = 2.0, 0.8, 0.3
        self.assertAlmostEqual(float(ground_effect_phi(h, b, mu)),
                               float(ground_effect_factor(h / b, mu)), places=12)
        # heading +x: the left wing direction is +y; a surface rising toward +y by slope s adds
        # atan(s) to the bank and shortens the normal height
        s = 0.2
        ref = ground_effect_factor(h / np.sqrt(1 + s * s) / b, mu + np.arctan(s))
        self.assertAlmostEqual(float(ground_effect_phi(h, b, mu, 0.0, 0.0, s)), float(ref),
                               places=12)


class TestDefaultsUnchanged(unittest.TestCase):
    """The default-off paths reproduce the original expressions bit for bit."""

    def test_flat_derivs_bitwise(self):
        ac = BASELINE
        rng = np.random.default_rng(5)
        S = np.column_stack([rng.uniform(-9, 9, 7), rng.uniform(-9, 9, 7), rng.uniform(1, 15, 7),
                             rng.uniform(8, 20, 7), rng.uniform(-0.5, 0.5, 7),
                             rng.uniform(-3, 3, 7)])
        C = np.column_stack([rng.uniform(0.1, 1.1, 7), rng.uniform(-1, 1, 7)])
        U, Up = rng.uniform(0, 9, 7), rng.uniform(0, 1, 7)
        V, g, p = S[:, 3], S[:, 4], S[:, 5]
        cl, mu = C[:, 0], C[:, 1]
        cg, sg, cp, sp = np.cos(g), np.sin(g), np.cos(p), np.sin(p)
        q = 0.5 * ac.rho * V * V * ac.wing_area
        D = q * (ac.cd0 + ac.k_induced * cl * cl)
        aw = Up * (V * sg)
        ref = np.empty_like(S)
        ref[:, 0] = V * cg * cp + U
        ref[:, 1] = V * cg * sp
        ref[:, 2] = V * sg
        ref[:, 3] = -D / ac.mass - G * sg - aw * cg * cp
        ref[:, 4] = (q * cl * np.cos(mu) / ac.mass - G * cg + aw * sg * cp) / V
        ref[:, 5] = (q * cl * np.sin(mu) / ac.mass + aw * sp) / (V * cg)
        self.assertTrue(np.array_equal(derivs(S, C, ac, U, Up), ref))
        self.assertTrue(np.array_equal(derivs(S, C, ac, U, Up, False, None), ref))

    def test_aircraft_and_vec_defaults(self):
        env = Environment.flat(Wind(9.0, from_deg=270.0))
        s = np.array([3.0, 4.0, 5.0, 12.0, 0.1, 0.7])
        d0, _ = derivatives(0.0, s, BASELINE, env, 0.5, 0.4)
        d1, _ = derivatives(0.0, s, BASELINE, env, 0.5, 0.4, False, 0.0, False)
        self.assertTrue(np.array_equal(d0, d1))
        S = np.stack([s, s * 1.1], axis=1)
        a = derivatives_vec(0.0, S, BASELINE, env, np.array([0.5, 0.6]), np.array([0.4, 0.2]))
        b = derivatives_vec(0.0, S, BASELINE, env, np.array([0.5, 0.6]), np.array([0.4, 0.2]),
                            False, 0.0, False)
        self.assertTrue(np.array_equal(a, b))

    def test_collocation_defaults_bitwise(self):
        prof = Profile(0.003, 10.0)
        a = Collocation(by_name("baseline-2m"), prof, "min_wind", "loiter", 9, midpoints=True)
        b = Collocation(by_name("baseline-2m"), prof, "min_wind", "loiter", 9, midpoints=True,
                        ground_effect=False, motor=False)
        x = a.guess(np.random.default_rng(2))
        self.assertEqual(a.nv, b.nv)
        self.assertTrue(np.array_equal(a.eq_fun(x), b.eq_fun(x)))
        self.assertTrue(np.array_equal(a.eq_jac(x), b.eq_jac(x)))
        self.assertTrue(np.array_equal(a.ineq_jac(x), b.ineq_jac(x)))


class TestGroundEffectDynamics(unittest.TestCase):
    def test_aircraft_drag_changes_only_near_surface(self):
        env = Environment.flat(Wind(0.01, from_deg=270.0))
        lo = np.array([0.0, 0.0, 0.3, 12.0, 0.0, 0.0])
        hi = np.array([0.0, 0.0, 80.0, 12.0, 0.0, 0.0])
        for s, near in ((lo, True), (hi, False)):
            off, _ = derivatives(0.0, s, BASELINE, env, 0.8, 0.0)
            on, _ = derivatives(0.0, s, BASELINE, env, 0.8, 0.0, ground_effect=True)
            if near:
                self.assertGreater(on[3], off[3] + 1e-3)  # less drag, less deceleration
                self.assertEqual(on[4], off[4])  # lift unchanged
            else:
                self.assertAlmostEqual(on[3], off[3], places=4)

    def test_vec_matches_scalar(self):
        env = Environment.flat(Wind(9.0, from_deg=270.0))
        rng = np.random.default_rng(8)
        for _ in range(4):
            s = np.array([1.0, 2.0, rng.uniform(0.5, 6), rng.uniform(9, 20), rng.uniform(-.4, .4),
                          rng.uniform(-3, 3)])
            cl, mu, th = rng.uniform(0.2, 1.0), rng.uniform(-1, 1), rng.uniform(0, 3)
            ref, _ = derivatives(0.0, s, BASELINE, env, cl, mu, False, th, True)
            v = derivatives_vec(0.0, s[:, None], BASELINE, env, np.array([cl]), np.array([mu]),
                                False, th, True)
            np.testing.assert_allclose(v[:, 0], ref, rtol=1e-9, atol=1e-10)

    def test_flat_derivs_matches_aircraft(self):
        wind = Wind(8.0, from_deg=270.0)
        env = Environment.flat(wind)
        h = 0.05
        rng = np.random.default_rng(4)
        for _ in range(8):
            z = rng.uniform(0.5, 8.0)
            s = np.array([0.0, 0.0, z, rng.uniform(9, 20), rng.uniform(-.5, .5),
                          rng.uniform(-3, 3)])
            cl, mu, th = rng.uniform(0.1, 1.1), rng.uniform(-1, 1), rng.uniform(0, 4)
            ref, _ = derivatives(0.0, s, BASELINE, env, cl, mu, False, th, True)
            U = float(wind.speed(z))
            Up = float(wind.speed(z + h) - wind.speed(z - h)) / (2 * h)
            mine = derivs(s, np.array([cl, mu]), BASELINE, U, Up, True, th)
            # the optimizer's model also has the smooth folded propeller drag; aircraft does not
            q = 0.5 * BASELINE.rho * s[3] ** 2 * BASELINE.wing_area
            fold = folded_dcd(BASELINE) * (1 - th / (th + 0.01 * BASELINE.mass * G))
            mine[3] += q * fold / BASELINE.mass
            np.testing.assert_allclose(mine, ref, rtol=1e-6, atol=1e-9)


class TestJacobians(unittest.TestCase):
    def test_dyn_jac_vs_fd_flat(self):
        prof = Profile(0.003, 10.0)
        ac = by_name("baseline-2m")
        rng = np.random.default_rng(1)
        n = 5
        A = np.column_stack([rng.uniform(-9, 9, n), rng.uniform(-9, 9, n), rng.uniform(0.8, 6, n),
                             rng.uniform(9, 20, n), rng.uniform(-.4, .4, n), rng.uniform(-3, 3, n),
                             rng.uniform(0.2, 1.1, n), rng.uniform(-1, 1, n)])
        th = rng.uniform(0.0, 5.0, n)
        F, J = dyn_jac(A, ac, prof, 9.0, True, th)
        self.assertEqual(J.shape, (n, 6, 10))
        for i in range(n):
            x0 = np.concatenate([A[i], [9.0], [th[i]]])

            def f(x):
                S, C = x[None, :6], x[None, 6:8]
                return dyn_jac(np.concatenate([S, C], axis=1), ac, prof, x[8], True, x[9:10])[0][0]

            Jn = fd_jacobian(f, x0)
            self.assertLess(np.abs(J[i] - Jn).max(), 1e-6 * max(1.0, np.abs(Jn).max()))

    def test_dyn_jac_vs_fd_wave(self):
        prof = Profile(0.003, 10.0)
        ac = by_name("baseline-2m")
        wv = WaveParams(0.8, 30.0, 20.0)
        rng = np.random.default_rng(2)
        n = 4
        A = np.column_stack([rng.uniform(-9, 9, n), rng.uniform(-9, 9, n), rng.uniform(1, 6, n),
                             rng.uniform(9, 20, n), rng.uniform(-.4, .4, n), rng.uniform(-3, 3, n),
                             rng.uniform(0.2, 1.1, n), rng.uniform(-1, 1, n)])
        tt = rng.uniform(0, 5, n)
        th = rng.uniform(0.0, 5.0, n)
        F, J = dyn_jac_wave(A, tt, 9.0, 0.4, ac, prof, wv, True, th)
        self.assertEqual(J.shape, (n, 6, 11))
        for i in range(n):
            x0 = np.concatenate([A[i], [9.0, 0.4], [th[i]]])

            def f(x):
                return dyn_jac_wave(x[None, :8], tt[i:i + 1], x[8], x[9], ac, prof, wv, True,
                                    x[10:11])[0][0]

            Jn = fd_jacobian(f, x0)
            # the t column is not an input of f here (J has no t column)
            self.assertLess(np.abs(J[i] - Jn).max(), 1e-6 * max(1.0, np.abs(Jn).max()))

    def _check_constraints(self, col, seed):
        x = col.guess(np.random.default_rng(seed)) * (1 + 0.01 * np.random.default_rng(seed + 9)
                                                      .standard_normal(col.nv))
        for fun, jac in ((col.eq_fun, col.eq_jac), (col.ineq_fun, col.ineq_jac)):
            J = jac(x)
            Jn = fd_jacobian(fun, x)
            self.assertLess(np.abs(J - Jn).max(), 1e-5 * max(1.0, np.abs(Jn).max()))
        g = col.objective(x)[1]
        gn = fd_jacobian(lambda y: np.array([col.objective(y)[0]]), x)[0]
        self.assertLess(np.abs(g - gn).max(), 1e-6 * max(1.0, np.abs(gn).max()))

    def test_collocation_flat_ground_and_motor(self):
        prof = profile_for(8.0)
        col = Collocation(by_name("baseline-2m"), prof, "min_power", "travel", 9, u10=6.0,
                          midpoints=True, rate_limits=True, rate_model="pb2v", n_min=0.0,
                          tip_model="geometric", ground_effect=True, motor=True, v_travel_min=2.0)
        self._check_constraints(col, 1)

    def test_collocation_wave_ground_and_motor(self):
        prof = profile_for(8.0)
        col = Collocation(by_name("baseline-2m"), prof, "min_power", "loiter", 9, u10=6.0,
                          wave=dict(a=0.6, wavelength=25.0, to_deg=10.0), n_min=0.0,
                          tip_model="geometric", ground_effect=True, motor=True)
        self._check_constraints(col, 3)

    def test_collocation_motor_only_flat_nonmidpoint(self):
        prof = profile_for(8.0)
        col = Collocation(by_name("baseline-2m"), prof, "max_speed", "travel", 9, u10=8.0,
                          motor=True)
        self._check_constraints(col, 2)


class TestMotorSolve(unittest.TestCase):
    KW = dict(n=24, h_min=1.0, tip_model="geometric", midpoints=True, rate_limits=True,
              rate_model="pb2v", roll_pb2v=0.08, cl_rate_max_per_s=1.0, n_min=0.0, motor=True,
              seeds=(0, 1, 2, 3), stop_after=1, maxiter=1000)

    def test_bounds_and_power_formula(self):
        ac = by_name("baseline-2m")
        col = Collocation(ac, profile_for(8.0), "min_power", "loiter", 9, u10=5.0, motor=True)
        b = np.asarray(col.bounds())
        sl = slice(col.iTh, col.iTh + col.N)
        np.testing.assert_allclose(b[sl, 0] * col.sc[sl], 0.0)
        np.testing.assert_allclose(b[sl, 1] * col.sc[sl], 0.5 * ac.mass * G)
        self.assertAlmostEqual(thrust_max(ac), 0.5 * ac.mass * G)
        x = col.guess(np.random.default_rng(0))
        S = col.unpack(x)[0]
        manual = np.trapezoid(col.thrust(x) * S[:, 3], dx=1.0 / (col.N - 1)) / prop_efficiency()
        self.assertAlmostEqual(col.avg_power(x), float(manual), places=9)

    def test_power_zero_above_and_positive_below_soaring_minimum(self):
        ac = by_name("baseline-2m")
        res = {}
        for u10 in (14.0, 4.0):
            info = solve(ac, profile_for(u10, roughness="coare35"), "min_power", "loiter",
                         u10=u10, **self.KW)
            self.assertTrue(info["success"], (u10, info["message"], info["violation"]))
            res[u10] = info["power_avg_w"]
        self.assertLess(res[14.0], 0.5)
        self.assertGreater(res[4.0], 5.0)


class TestReplayWithMotorAndGroundEffect(unittest.TestCase):
    """The dense replay and the mesh resampling of the wave work carry the motor and ground effect."""

    def test_replay_closes_and_resample_keeps_thrust(self):
        from sim.dsopt import _resample, dense_replay, profile_for, solve
        ac = by_name("baseline-2m")
        info, x, col = solve(ac, profile_for(6.0, roughness="coare35"), "min_power", "travel", 24,
                             u10=6.0, chi=0.349, motor=True, ground_effect=True,
                             tip_model="geometric", midpoints=True, seeds=(0,), stop_after=1,
                             maxiter=400, return_x=True)
        rc = dense_replay(col, x)
        self.assertLess(rc["rc_closure_h"], 0.05)
        self.assertLess(rc["rc_closure_V_rel"], 0.01)
        c2 = Collocation(ac, col.prof, "min_power", "travel", 48, 6.0, 0.349,
                         tip_model="geometric", midpoints=True, motor=True, ground_effect=True)
        x2 = _resample(col, x, c2)
        self.assertEqual(len(x2), c2.nv)
        self.assertAlmostEqual(float(c2.thrust(x2).mean()), float(col.thrust(x).mean()), delta=0.05)


if __name__ == "__main__":
    unittest.main()
