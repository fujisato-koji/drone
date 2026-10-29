import unittest

import numpy as np

from sim.aircraft import BASELINE, Aircraft, dumb_glide_ctrl, simulate

from sim.seawind import Environment, Wind

NOSTOP = lambda t, s: False


class UniformEnv:
    def __init__(self, w):
        self.w = np.array(w, dtype=float)

    def wind_at(self, x, y, z, t):
        x = np.asarray(x, dtype=float)
        return np.broadcast_to(self.w, x.shape + (3,)).copy()

    def surface(self, x, y, t):
        z = np.zeros_like(np.asarray(x, dtype=float))
        return z, z, z


def still():
    return Environment.flat(Wind(0.1, 270))


class TestAircraft(unittest.TestCase):
    def test_still_air_glide(self):
        ac = BASELINE
        s0 = [0, 0, 500, ac.v_best_ld, -np.arctan(1 / ac.best_ld), 0.0]  # start on the steady glide
        r = simulate(ac, still(), dumb_glide_ctrl, s0, 0, 10, stop=NOSTOP)
        self.assertAlmostEqual(np.tan(-r["gamma"][-1]) * ac.best_ld, 1.0, delta=0.05)

    def test_energy_conserved_without_drag(self):
        ac = Aircraft("nodrag", 1.5, 2.0, 0.25, 0.0, 1e9, 1.2)
        s0 = [0, 0, 500, 12.0, 0.0, 0.0]
        r = simulate(ac, still(), lambda t, s, e, a: (0.8, 0.5), s0, 0, 10, stop=NOSTOP)
        self.assertLess(np.max(np.abs(r["e_air"] - r["e_air"][0])), 1e-3)

    def test_uniform_wind(self):
        ac = BASELINE
        s0 = [0, 0, 500, 12.0, -0.05, 0.3]
        ctrl = lambda t, s, e, a: (0.6, 0.3)
        r0 = simulate(ac, UniformEnv([0, 0, 0]), ctrl, s0, 0, 10, stop=NOSTOP)
        r1 = simulate(ac, UniformEnv([5, 0, 0]), ctrl, s0, 0, 10, stop=NOSTOP)
        self.assertAlmostEqual(r1["V"][-1], r0["V"][-1], places=6)
        self.assertAlmostEqual(r1["x"][-1] - r0["x"][-1], 50.0, places=4)
        self.assertAlmostEqual(r1["y"][-1], r0["y"][-1], places=6)

    def test_shear_gain(self):
        ac = BASELINE
        env = Environment.flat(Wind(10, 270))  # wind blows towards +x
        s0 = [0, 0, 10, 12.0, 0.2, np.pi]  # heading -x, into the wind
        ctrl = lambda t, s, e, a: (0.6, 0.0)
        rs = simulate(ac, env, ctrl, s0, 0, 1, stop=NOSTOP)
        r0 = simulate(ac, still(), ctrl, s0, 0, 1, stop=NOSTOP)
        self.assertGreater(rs["e_air"][-1] - rs["e_air"][0],
                           r0["e_air"][-1] - r0["e_air"][0])

    def test_dsearch_smoke(self):
        from sim.dsearch import evaluate
        env = Environment.flat(Wind(10, 270))
        p = (8.0, 0.8, 0.0, 0.7, 0.3, 1.0, 0.5)
        e = evaluate(BASELINE, env, p, 3.0, 2 * BASELINE.v_stall)
        for k in ("gain", "min_h", "max_n", "min_V"):
            self.assertTrue(np.isfinite(e[k]))
        self.assertTrue(np.all(np.isfinite(e["mean_ground_velocity"])))

    def test_rayleigh_smoke(self):
        from sim.dsearch import RayleighCycleController, evaluate_controller
        env = Environment.flat(Wind(12, 270))
        psi_wind = np.arctan2(env.wind.ey, env.wind.ex)
        ctrl = RayleighCycleController((1.0, 0.4, 0.9, 0.3, 10.0, 2.0, 1.0), psi_wind)
        e = evaluate_controller(BASELINE, env, ctrl, 3.0, 2 * BASELINE.v_stall, 40.0)
        self.assertGreaterEqual(len(ctrl.changes), 3)
        self.assertTrue(np.all(np.isfinite(e["result"]["e_air"])))


if __name__ == "__main__":
    unittest.main()
