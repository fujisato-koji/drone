import unittest
from dataclasses import replace

import numpy as np

from sim import wingopt as w
from sim.aircraft import BASELINE, G, derivatives
from sim.dsopt import Profile, _drag, dyn_jac, derivs
from sim.record import value
from sim.seawind import Environment, Wind


def polar_ac(**kw):
    return replace(BASELINE, cdp1=-0.008, cdp2=0.011, cdp8=0.002, **kw)


class TestPolarExtension(unittest.TestCase):
    def test_default_is_bit_identical(self):
        rng = np.random.default_rng(1)
        ac = BASELINE
        for _ in range(20):
            V, cl, mu = rng.uniform(5, 25), rng.uniform(-0.3, 1.2), rng.uniform(-1, 1)
            q = 0.5 * ac.rho * V * V * ac.wing_area
            old = q * (ac.cd0 + ac.k_induced * cl * cl)
            self.assertEqual(float(_drag(ac, q, cl, mu, 3.0, False, None)), float(old))
            s = np.array([0.0, 0.0, 5.0, V, 0.1, 0.3])
            env = Environment.flat(Wind(10.0, from_deg=270.0))
            d0, _ = derivatives(0.0, s, ac, env, cl, mu)
            d1, _ = derivatives(0.0, s, replace(ac, cdp1=0.0, cdp2=0.0, cdp8=0.0), env, cl, mu)
            np.testing.assert_array_equal(d0, d1)
        self.assertFalse(ac.has_polar)

    def test_polar_drag_matches_formula(self):
        ac = polar_ac()
        cl, V = 0.7, 14.0
        q = 0.5 * ac.rho * V * V * ac.wing_area
        want = q * (ac.cd0 + ac.cdp1 * cl + ac.cdp2 * cl ** 2 + ac.cdp8 * cl ** 8 + ac.k_induced * cl ** 2)
        self.assertAlmostEqual(float(_drag(ac, q, cl, 0.0, 3.0, False, None)), want, places=12)
        self.assertAlmostEqual(float(ac.cd(cl)), want / q, places=12)
        self.assertGreater(ac.best_ld, 5.0)

    def test_jacobian_matches_finite_differences(self):
        ac = polar_ac()
        prof = Profile(0.001)
        rng = np.random.default_rng(2)
        A = np.column_stack([rng.uniform(-5, 5, 6), rng.uniform(-5, 5, 6), rng.uniform(2, 9, 6),
                             rng.uniform(9, 20, 6), rng.uniform(-.4, .4, 6), rng.uniform(-3, 3, 6),
                             rng.uniform(0.2, 1.1, 6), rng.uniform(-.8, .8, 6)])
        u10 = 9.0
        f, J = dyn_jac(A, ac, prof, u10)
        h = 1e-6
        for j in range(8):
            Ap, Am = A.copy(), A.copy()
            Ap[:, j] += h
            Am[:, j] -= h
            fd = (dyn_jac(Ap, ac, prof, u10)[0] - dyn_jac(Am, ac, prof, u10)[0]) / (2 * h)
            np.testing.assert_allclose(J[:, :, j], fd, rtol=1e-5, atol=1e-6)


class TestDesignModel(unittest.TestCase):
    def test_airfoil_reproduces_fit(self):
        a0, a1, a2, n, a8, _ = w.AIRFOILS["SD7037"]
        cl = 0.6
        self.assertAlmostEqual(w.airfoil_cd("SD7037", cl, 2e5), a0 + a1 * cl + a2 * cl ** 2 + a8 * cl ** 8, 12)
        low = (a0 + a1 * cl + a2 * cl ** 2) * (1e5 / 2e5) ** -n + a8 * cl ** 8
        self.assertAlmostEqual(w.airfoil_cd("SD7037", cl, 1e5), low, 12)
        d = w.Design(2.5, 16.0)
        ac = w.build_aircraft(d, 1.5e5)
        want = w.airfoil_cd("SD7037", 0.6, 1.5e5) + d.cda / ac.wing_area
        self.assertAlmostEqual(float(ac.cd(0.6) - ac.k_induced * 0.36), want, 10)

    def test_span_efficiency(self):
        f = 0.0524 * .45 ** 4 - .15 * .45 ** 3 + .1659 * .45 ** 2 - .0706 * .45 + .0119
        self.assertAlmostEqual(w.span_efficiency(20, 0.45, 3.0), (1 - 2 * (0.06 / 3) ** 2) / (1 + f * 20), 12)

    def test_mass_consistency(self):
        mm = w.mass_model(3.0, 25.0, 1.5)
        n_ult = value("ds.n_limit", 4.0) * value("ds.safety_factor", 1.5)
        spar = (2 * 1400.0 * n_ult * mm["mass"] * G * 3.0 * 25.0) / (9 * np.pi * 300e6 * 0.85 * 0.09)
        self.assertAlmostEqual(mm["m_spar"], spar, 12)
        self.assertAlmostEqual(mm["m_wing"], 1.5 * 0.36 + spar + 0.05, 12)
        self.assertAlmostEqual(mm["mass"], w.other_mass() + mm["m_wing"], 12)
        self.assertAlmostEqual(w.other_mass(), 0.68, 9)

    def test_feasibility(self):
        self.assertFalse(w.feasibility(w.Design(3.0, 25.0))[0])   # Re 69k at 1.2 Vs
        self.assertTrue(w.feasibility(w.Design(3.0, 16.0))[0])
        self.assertFalse(w.feasibility(w.Design(3.5, 12.0))[0])   # span limit

    def test_quick_evaluation(self):
        r = w.evaluate(("A", w.Design(3.0, 20.0), None))
        self.assertTrue(r["success"])
        self.assertTrue(5.0 < r["u10"] < 20.0)
        self.assertTrue(r["replay_pass"])


if __name__ == "__main__":
    unittest.main()
