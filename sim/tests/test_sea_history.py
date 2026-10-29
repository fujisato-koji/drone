import unittest

import numpy as np

from sim.sea_history import SeaHistory, WindHistory
from sim.seawind import WaveTrain, Wind


class TestSeaHistory(unittest.TestCase):
    def test_step_up(self):
        h = WindHistory([(0, 5, 270), (10, 5, 270), (10.01, 15, 270), (100, 15, 270)])
        sh = SeaHistory(h)
        hmax = WaveTrain.duration_limited(Wind(15, 270), 1e4).hs  # CEM limit
        hs = [r[2] for r in sh.table([10, 12, 14, 18, 30, 60])]
        self.assertTrue(all(b > a for a, b in zip(hs, hs[1:4])))
        self.assertTrue(all(v <= 1.01 * hmax for v in hs))
        self.assertGreater(hs[-1], hs[-2])
        self.assertGreater(hs[-1], 0.5 * hmax)

    def test_constant_wind_follows_duration_law(self):
        sh = SeaHistory(WindHistory([(0, 15, 270), (100, 15, 270)]))
        for hr in (6, 12, 24):
            r = sh.table([hr])[0]
            ref = WaveTrain.duration_limited(Wind(15, 270), hr)
            self.assertAlmostEqual(r[2] / ref.hs, 1.0, delta=0.02)
            self.assertAlmostEqual(r[3] / ref.tp, 1.0, delta=0.02)

    def test_cold_start_tp_from_duration_law(self):
        sh = SeaHistory(WindHistory([(0, 15, 270), (10, 15, 270)]))
        r = sh.table([0])[0]
        self.assertEqual(r[2], 0.0)
        self.assertLess(r[3], WaveTrain.fully_developed(Wind(15, 270)).tp)

    def test_swell_energy_decay(self):
        h = WindHistory([(0, 15, 270), (40, 15, 270), (40.01, 3, 270), (200, 3, 270)])
        sh = SeaHistory(h)
        a = sh.table([41])[0][4]
        b = sh.table([41 + 12])[0][4]
        self.assertAlmostEqual((b / a) ** 2, np.exp(-1.0), delta=0.03)

    def test_drop_leaves_decaying_swell(self):
        h = WindHistory([(0, 15, 270), (40, 15, 270), (40.01, 3, 270), (200, 3, 270)])
        sh = SeaHistory(h)
        r41 = sh.table([41])[0]
        self.assertGreater(r41[4], 3.0)
        self.assertGreater(r41[5], 5.0)
        self.assertLess(sh.table([40 + 96])[0][4], 0.2)
        self.assertEqual(len(sh.environment(41).trains), 2)
        self.assertEqual(len(sh.environment(40 + 96).trains), 1)

    def test_environment_wind(self):
        h = WindHistory([(0, 5, 270), (10, 15, 180)])
        sh = SeaHistory(h)
        env = sh.environment(5.0, seed=2)
        self.assertAlmostEqual(env.wind.u10, 10.0, places=9)
        self.assertAlmostEqual(env.wind.from_deg, 225.0, places=6)

    def test_environment_roughness(self):
        h = WindHistory([(0, 10, 270), (50, 10, 270)])
        sh = SeaHistory(h)
        a = sh.environment(30.0)
        b = sh.environment(30.0, roughness="wave_age")
        self.assertEqual(a.wind.roughness, "charnock")
        self.assertEqual(b.wind.roughness, "wave_age")
        self.assertGreater(b.wind.hs, 1.0)


if __name__ == "__main__":
    unittest.main()
