import unittest

import numpy as np

from sim.aircraft import BASELINE, G
from sim import patterns as P


class TestPatterns(unittest.TestCase):
    def test_catalogue(self):
        self.assertEqual(len(P.PATTERNS), 8)
        for p in P.PATTERNS:
            for k in ("name", "energy_source", "when", "geometry", "parameters",
                      "dsopt_mode", "sources"):
                self.assertIn(k, p)
            self.assertTrue(p["sources"])
        self.assertIn("ds_loiter", P.patterns_table())

    def test_wave_slope(self):
        a = P.wave_slope_sustainable(BASELINE, 1.0, 10.0)
        b = P.wave_slope_sustainable(BASELINE, 3.0, 10.0)
        self.assertGreater(a["w_up"], 0.0)
        self.assertGreater(b["w_up"], a["w_up"])
        z = P.wave_slope_sustainable(BASELINE, 2.0, 10.0, sink_override=0.0)
        self.assertTrue(z["sustainable"])
        self.assertAlmostEqual(z["c"], G * 10.0 / (2 * np.pi), delta=1e-9)

    def test_flap_glide_calm(self):
        r = P.flap_glide_cost(BASELINE, 0.0, 0.0)
        want = (BASELINE.mass * G * BASELINE.v_best_ld / BASELINE.best_ld) / (0.5 * 0.8)
        self.assertAlmostEqual(r["power_w"] / want, 1.0, delta=0.01)

    def test_selector(self):
        self.assertEqual(P.select_pattern(12.0, 1.0, 10.0, BASELINE)[0], "ds_travel_crosswind")
        self.assertEqual(P.select_pattern(1.0, 0.3, 10.0, BASELINE, battery_wh=0.0)[0],
                         "rest_on_water")


if __name__ == "__main__":
    unittest.main()
