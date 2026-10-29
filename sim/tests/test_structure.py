import unittest

import numpy as np

from sim import structure as st


class TestStructure(unittest.TestCase):
    def test_lift_and_root_moment(self):
        span, L = 2.0, 58.86
        y = np.linspace(-span / 2, span / 2, 2001)
        total = np.trapezoid(st.elliptical_lift(y, L, span), y)
        self.assertAlmostEqual(total / L, 1.0, delta=0.005)
        yh = np.linspace(0, span / 2, 2001)
        V, M = st.shear_and_moment(yh, L, span)
        self.assertAlmostEqual(M[0] / (L * span / (3 * np.pi)), 1.0, delta=0.01)

    def test_tube_closed_form(self):
        t = st.TubeSpar(0.012, 0.001, st.MATERIALS["carbon_tube"])
        self.assertAlmostEqual(t.I, np.pi / 64 * (0.012 ** 4 - 0.010 ** 4), places=15)
        self.assertAlmostEqual(t.stress(10.0), 10.0 * 0.006 / t.I, places=3)

    def test_cantilever_deflection(self):
        L, P, EI = 1.0, 5.0, 120e9 * 5e-10
        y = np.linspace(0, L, 2001)
        w = st.tip_deflection(y, P * (L - y), EI)
        self.assertAlmostEqual(w[-1] / (P * L ** 3 / (3 * EI)), 1.0, delta=0.01)

    def test_baseline_spar(self):
        geo = st.WingGeometry(2.0, 0.16, 0.6)
        r = st.size_tube_spar(geo, 1.5, 4.0, 1.5, st.MATERIALS["carbon_tube"])
        self.assertLessEqual(r["od"], geo.max_spar_od(0.0))
        self.assertGreaterEqual(r["margin"], 0.0)
        self.assertLess(r["deflection_ratio"], 0.15)
        self.assertTrue(0.02 < r["mass"] < 0.3)
        with self.assertRaises(ValueError):
            st.size_tube_spar(geo, 50.0, 4.0, 1.5, st.MATERIALS["carbon_tube"])

    def test_mass_model(self):
        model, _ = st.build_baseline()
        self.assertAlmostEqual(model.total_mass(), sum(c.mass for c in model.components))
        I = model.inertia()
        np.testing.assert_allclose(I, I.T, atol=1e-12)
        self.assertTrue(np.all(np.diag(I) > 0))
        cg0 = model.cg()
        c = model.components[-1]
        c.position = (c.position[0] - 0.1, c.position[1], c.position[2])
        self.assertAlmostEqual(model.cg()[0] - cg0[0], -0.1 * c.mass / model.total_mass())


if __name__ == "__main__":
    unittest.main()
