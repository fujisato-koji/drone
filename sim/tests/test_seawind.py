import json
import os
import tempfile
import unittest

import numpy as np

from sim.seawind import G, Environment, WaveTrain, Wind


class Mono(WaveTrain):
    def components(self, rng):
        k = 0.1
        return {"a": np.array([1.0]), "k": np.array([k]),
                "omega": np.array([np.sqrt(G * k)]), "dx": np.array([1.0]),
                "dy": np.array([0.0]), "phase": np.array([0.0])}


class TestSeaWind(unittest.TestCase):
    def test_wind(self):
        w = Wind(10.0)
        self.assertTrue(0.30 <= w.ustar <= 0.42)
        self.assertTrue(1e-4 <= w.z0 <= 4e-4)
        self.assertAlmostEqual(float(w.speed(10.0)), 10.0, delta=1e-9)
        self.assertEqual(float(w.speed(w.z0 / 2)), 0.0)

    def test_waves(self):
        w = Wind(10.0)
        tr = WaveTrain.fully_developed(w)
        self.assertTrue(1.8 <= tr.hs <= 2.8)
        self.assertTrue(6.5 <= tr.tp <= 8.5)
        c = tr.components(np.random.default_rng(1))
        hsd = 4 * np.sqrt(np.sum(c["a"] ** 2 / 2))
        self.assertLess(abs(hsd / tr.hs - 1), 0.03)
        # equal-energy bins are narrowest where the spectrum is densest, so
        # the narrowest bin sits at the peak
        om = np.unique(np.round(c["omega"], 9))
        d = np.diff(om)
        i = int(np.argmin(d))
        om_peak = 0.5 * (om[i] + om[i + 1])
        self.assertLess(abs(om_peak / (2 * np.pi / tr.tp) - 1), 0.15)

    def test_surface_stats(self):
        w = Wind(10.0)
        tr = WaveTrain.fully_developed(w)
        env = Environment(w, [tr], seed=2)
        r = np.random.default_rng(5)
        n = 20000
        eta, _, _ = env.surface(r.uniform(0, 1000, n), r.uniform(0, 1000, n),
                                r.uniform(0, 3600, n))
        self.assertLess(abs(4 * eta.std() / tr.hs - 1), 0.10)

    def _mono(self):
        w = Wind(15.0, 270.0)
        return w, Environment(w, [Mono(1.0, 10.0, 90.0)])

    def test_monochromatic(self):
        w, env = self._mono()
        self.assertGreater(env.urel[0], 0)
        x = -np.pi / 2 / 0.1
        eta, _, _ = env.surface(x, 0.0, 0.0)
        v = env.wind_at(x, 0.0, eta + 1.0, 0.0)
        self.assertGreater(v[2], 0)
        eta, _, _ = env.surface(0.0, 0.0, 0.0)
        v = env.wind_at(0.0, 0.0, eta + 1.0, 0.0)
        self.assertGreater(v[0], float(w.speed(1.0)))

    def test_noslip(self):
        w, env = self._mono()
        x, t = 12.3, 4.5
        eta, _, _ = env.surface(x, 0.0, t)
        v = env.wind_at(x, 0.0, eta + w.z0 / 2, t)
        orb = np.array(env.orbital(x, 0.0, t))
        np.testing.assert_allclose(v, orb, atol=1e-12)

    def test_broadcasting(self):
        w = Wind(10.0)
        env = Environment(w, [WaveTrain.fully_developed(w)])
        a = np.linspace(0, 50, 5)
        self.assertEqual(env.wind_at(a, a, a + 1, 0.0).shape, (5, 3))
        z = np.linspace(1, 10, 7)[:, None]
        self.assertEqual(env.wind_at(a[None, :], 0.0, z, 0.0).shape, (7, 5, 3))

    def test_to_js(self):
        w = Wind(10.0)
        env = Environment(w, [WaveTrain.fully_developed(w)])
        with tempfile.TemporaryDirectory() as d:
            p = os.path.join(d, "f.js")
            env.to_js(p)
            with open(p, encoding="utf-8", newline="") as f:
                s = f.read()
        self.assertTrue(s.startswith("window.FIELD = "))
        self.assertNotIn("\r", s)
        j = json.loads(s[len("window.FIELD = "):].rstrip().rstrip(";"))
        self.assertEqual(len(j["components"]["a"]), j["meta"]["n_components"])


if __name__ == "__main__":
    unittest.main()
