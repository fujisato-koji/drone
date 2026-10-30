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
        d = w.Design(2.5, 16.0, cda=w.CDA_BASE)
        ac = w.build_aircraft(d, 1.5e5)
        want = w.airfoil_cd("SD7037", 0.6, 1.5e5) + d.cda / ac.wing_area
        self.assertAlmostEqual(float(ac.cd(0.6) - ac.k_induced * 0.36), want, 10)

    def test_span_efficiency(self):
        lam = 0.45 - (-0.357 + 0.45)   # Nita and Scholz eq. 37 at zero sweep: offset 0.093
        self.assertAlmostEqual(lam, 0.357, 12)
        f = 0.0524 * lam ** 4 - .15 * lam ** 3 + .1659 * lam ** 2 - .0706 * lam + .0119
        self.assertAlmostEqual(w.span_efficiency(20, 0.45, 3.0), (1 - 2 * (0.06 / 3) ** 2) / (1 + f * 20), 12)
        self.assertAlmostEqual(w.span_efficiency(12, 0.45, 3.0), 0.977, places=3)   # K-014: 0.977 against 0.973 without the offset

    def test_body_drag_buildup(self):
        bd = w.body_drag(3.0, 9.0 / 16.0, 12.0)
        parts = bd["pod"] + bd["boom"] + bd["tail"]
        self.assertAlmostEqual(bd["total"], 1.10 * parts, 12)
        self.assertAlmostEqual(bd["gaps"], 0.10 * parts, 12)
        fr = 1.0 / 0.07   # pod: Cf 0.455/log10(Re)^2.58, Hoerner form factor, body-of-revolution wetted area
        swet = np.pi * 0.07 * (1 - 2 / fr) ** (2 / 3) * (1 + 1 / fr ** 2)
        cf = 0.455 / np.log10(12.0 / w.NU_AIR) ** 2.58
        self.assertAlmostEqual(bd["pod"], cf * (1 + 1.5 / fr ** 1.5 + 7 / fr ** 3) * swet, 12)
        # tail areas from the volume coefficients: S_h = 0.4 S c / arm, S_v = 0.02 S b / arm
        arm = w.TAIL_ARM_PER_SPAN * 3.0
        self.assertAlmostEqual(bd["s_h"], 0.4 * (9.0 / 16.0) * (3.0 / 16.0) / arm, 12)
        self.assertAlmostEqual(bd["s_v"], 0.02 * (9.0 / 16.0) * 3.0 / arm, 12)
        # the design uses it: a fixed area overrides
        d = w.Design(3.0, 16.0)
        ac = w.build_aircraft(d, 2e5)
        v = 2e5 * w.NU_AIR / (ac.wing_area / 3.0)
        cda = w.body_drag(3.0, ac.wing_area, v)["total"]
        self.assertAlmostEqual(float(ac.cd(0.0)), 0.01061 + cda / ac.wing_area, 10)
        ac2 = w.build_aircraft(w.Design(3.0, 16.0, cda=10e-4), 2e5)
        self.assertAlmostEqual(float(ac2.cd(0.0)), 0.01061 + 10e-4 / ac2.wing_area, 10)

    def test_ballast(self):
        m0 = w.mass_model(3.0, 16.0, 1.5)["mass"]
        d = w.Design(3.0, 16.0, ballast=0.5)
        mm = w.basics(d)[0]
        self.assertGreater(mm["mass"], m0 + 0.5)   # the spar carries the ballast too
        self.assertAlmostEqual(mm["mass"], mm["m_other"] + mm["m_wing"], 12)
        bal = w.ballast_to_mass(3.0, 16.0, 1.5, 3.0)
        self.assertAlmostEqual(w.basics(w.Design(3.0, 16.0, ballast=bal))[0]["mass"], 3.0, 9)
        g = w.ballast_grid(3.0, 16.0)
        self.assertEqual(g[:3], [0.0, 0.5, 1.0])
        self.assertLess(w.basics(w.Design(3.0, 16.0, ballast=round(g[-1], 4)))[0]["mass"], value("ds.mass_max_kg") + 1e-9)
        self.assertFalse(w.feasibility(w.Design(3.0, 16.0, ballast=2.0))[0])   # above the mass limit

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
        self.assertTrue(w.feasibility(w.Design(3.0, 25.0, ballast=0.5))[0])   # ballast raises the stall speed
        self.assertTrue(w.feasibility(w.Design(3.0, 16.0))[0])
        self.assertFalse(w.feasibility(w.Design(3.5, 12.0))[0])   # span limit

    def test_quick_evaluation(self):
        r = w.solve_a(w.Design(3.0, 20.0, ballast=0.5), seeds=(0, 1, 2))
        self.assertTrue(r["success"])
        self.assertTrue(5.0 < r["u10"] < 20.0)
        self.assertTrue(r["replay_pass"])
        self.assertAlmostEqual(r["mass"], w.basics(w.Design(3.0, 20.0, ballast=0.5))[0]["mass"], 12)
        self.assertLessEqual(abs(r["re_used"] - r["history"][-1]["re"]), 1e-6)   # the aircraft solved is the one reported


class TestSteadyGuess(unittest.TestCase):
    def test_steady_level_flight_is_feasible(self):
        from sim.dsopt import profile_for
        d = w.Design(3.0, 16.0, ballast=0.5)
        ac = w.build_aircraft(d, 1.5e5)
        prof = profile_for(9.0, "log", roughness="coare35")
        for chi in (20.0, 180.0):
            x, info, col = w.steady_guess(ac, prof, 9.0, chi, w.MARGINS)
            self.assertLess(col.violation(x), 1e-9)
            self.assertTrue(info["thrust_ok"] and info["cl_ok"])
            self.assertGreaterEqual(info["progress"], info["v_travel_min"] - 1e-9)
            self.assertAlmostEqual(info["power_w"], col.avg_power(x), 6)


class TestHeavy(unittest.TestCase):
    """The heavier-than-limit study (Q-067, Q-069): ds.mass_max_kg is waived, not changed."""

    def test_heavy_design_mass_and_feasibility(self):
        d = w.heavy_design(20.0, 8.0)
        self.assertAlmostEqual(w.basics(d)[0]["mass"], 8.0, 9)
        self.assertTrue(w.feasibility(d)[0])
        self.assertFalse(w.feasibility(w.Design(3.0, 20.0, ballast=d.ballast))[0])   # the limit still holds
        self.assertTrue(d.key.endswith("|heavy"))
        self.assertEqual(value("ds.mass_max_kg"), 3.0)

    def test_stiff_caps_factor_is_mass_independent(self):
        a, b = w.cap_stiffness(w.heavy_design(16.0, 4.0)), w.cap_stiffness(w.heavy_design(16.0, 10.0))
        self.assertAlmostEqual(a["stiffness_factor"], b["stiffness_factor"], 9)
        self.assertGreater(a["stiffness_factor"], 1.0)
        self.assertAlmostEqual(b["caps_stiff_kg"] / a["caps_stiff_kg"], b["caps_stress_kg"] / a["caps_stress_kg"], 9)

    def test_steady_flight_at_its_own_reynolds_number(self):
        d = w.heavy_design(20.0, 3.0)
        st = w.steady_flight(d, 9.0, 20.0, 1.6e5)
        self.assertLess(abs(w.re_of(st["V"], w.basics(d)[2]) / st["re"] - 1.0), 2e-3)
        hi = w.steady_flight(d, 9.0, 20.0, 1.6e5, iters=0)   # the polar at the start's Reynolds number
        self.assertLess(hi["power_w"], st["power_w"])   # a higher Reynolds number flatters the steady flight
        self.assertAlmostEqual(st["wh_per_km"], st["power_w"] / (3.6 * st["progress"]), 12)

    def test_spar_tube_check(self):
        light, heavy = w.spar_tube_check(w.heavy_design(16.0, 3.0)), w.spar_tube_check(w.heavy_design(16.0, 10.0))
        self.assertTrue(light["fits"])
        self.assertLessEqual(light["deflection_ratio"], value("spar.deflection_limit") + 1e-12)
        self.assertFalse(heavy["fits"])


if __name__ == "__main__":
    unittest.main()
