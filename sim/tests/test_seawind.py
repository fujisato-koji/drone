import json
import os
import tempfile
import unittest

import numpy as np

from sim.seawind import G, Environment, WaveTrain, Wind, gust_factor


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


class TestSeaExtensions(unittest.TestCase):
    def test_duration_limited(self):
        w = Wind(10.0)
        t1 = WaveTrain.duration_limited(w, 1.0)
        t24 = WaveTrain.duration_limited(w, 24.0)
        tlong = WaveTrain.duration_limited(w, 500.0)
        # CEM limits (changed on purpose from the PM cap): g Hm0/u*^2 = 211.5, g Tp/u* = 239.8
        ust2 = 0.001 * (1.1 + 0.035 * 10.0) * 100.0
        self.assertAlmostEqual(tlong.hs, 211.5 * ust2 / 9.81, delta=1e-9)
        self.assertAlmostEqual(tlong.tp, 239.8 * np.sqrt(ust2) / 9.81, delta=1e-9)
        self.assertAlmostEqual(tlong.growth_fraction, 1.0, delta=1e-9)
        self.assertEqual(tlong.gamma, 1.0)
        self.assertLess(t1.hs, 0.3 * tlong.hs)
        # with II-2-38 a 10 m/s sea reaches the CEM height limit after about 32 h
        self.assertLess(t24.growth_fraction, 1.0)
        prev = None
        for h in (1, 2, 4, 8, 16, 32, 48):
            t = WaveTrain.duration_limited(w, h)
            if prev:
                self.assertGreaterEqual(t.hs, prev.hs)
                self.assertGreaterEqual(t.tp, prev.tp)
            prev = t
        self.assertGreater(prev.hs, t1.hs)
        self.assertGreater(prev.tp, t1.tp)

    def test_duration_limited_cem_15(self):
        t = WaveTrain.duration_limited(Wind(15.0), 12.0)
        ust2 = 0.001 * (1.1 + 0.035 * 15.0) * 225.0
        xstar = 5.23e-3 * (9.81 * 12 * 3600.0 / np.sqrt(ust2)) ** 1.5  # CEM II-2-38
        self.assertAlmostEqual(t.hs, 4.13e-2 * np.sqrt(xstar) * ust2 / 9.81, delta=1e-9)
        self.assertAlmostEqual(WaveTrain.equivalent_hours(15.0, t.hs), 12.0, delta=1e-9)
        # value computed from the printed equations by check K-006: 2.70 m at 15 m/s after 12 h
        self.assertAlmostEqual(t.hs, 2.70, delta=0.01)

    def test_fully_developed_profile_independent(self):
        a = WaveTrain.fully_developed(Wind(10.0, profile="log"))
        b = WaveTrain.fully_developed(Wind(10.0, profile="swell_knee"))
        self.assertAlmostEqual(a.hs, b.hs, delta=1e-12)
        self.assertAlmostEqual(a.tp, b.tp, delta=1e-12)

    def test_stokes_mono(self):
        w = Wind(10.0)
        tr = Mono(1.0, 10.0, 90.0)
        env = Environment(w, [tr], stokes=True)
        lam = 2 * np.pi / 0.1
        x = np.linspace(0, lam, 4000, endpoint=False)
        eta, _, _ = env.surface(x, 0.0, 0.0)
        self.assertAlmostEqual(eta.max(), 1.05, places=4)
        self.assertAlmostEqual(-eta.min(), 0.95, places=4)
        self.assertAlmostEqual(float(eta.mean()), 0.0, delta=1e-6)
        lin = Environment(w, [tr]).surface(x, 0.0, 0.0)[0]
        self.assertAlmostEqual(lin.max(), 1.0, places=4)

    def test_breaking(self):
        w = Wind(10.0)
        calm = Environment(w, [WaveTrain(0.5, 4.0, 90.0)], seed=1)
        self.assertEqual(calm.whitecap_fraction(0.0), 0.0)
        steep = Environment(w, [WaveTrain(6.0, 5.0, 90.0)], seed=1)
        self.assertGreater(steep.whitecap_fraction(0.0), 0.0)

    def test_roughness(self):
        base = Wind(10.0)
        self.assertEqual(Wind(10.0, roughness="charnock").z0, base.z0)
        self.assertAlmostEqual(base.ustar, 0.370717, places=5)
        self.assertAlmostEqual(base.z0, 2.06066e-4, delta=1e-8)
        full = WaveTrain.fully_developed(base)
        for mode in ("wave_age", "drennan"):
            young = Wind(10.0, roughness=mode, sea=(1.5, 4.5))
            old = Wind(10.0, roughness=mode, hs=full.hs, tp=full.tp)
            self.assertGreater(young.z0, old.z0)
            print(mode, "young", young.z0, young.ustar, "full", old.z0, old.ustar)
        print("charnock", base.z0, base.ustar)
        with self.assertRaises(ValueError):
            Wind(10.0, roughness="wave_age")


class TestSeaWindExtras(unittest.TestCase):
    def test_decay(self):
        w = Wind(10.0)
        tr = Mono(hs=1.0, tp=8.0, to_deg=90.0)
        base = Environment(w, [tr])
        meas = Environment(w, [tr], decay=(0.77, 0.62))
        k, zl = 0.1, 5.0
        x, y, t = np.array([3.0]), np.array([0.0]), np.array([0.0])
        eta, _, _ = base.surface(x, y, t)
        z = zl + eta
        mean = w.vector(np.array([zl]))
        d0 = base.wind_at(x, y, z, t) - mean
        d1 = meas.wind_at(x, y, z, t) - mean
        self.assertGreater(abs(d0[0, 0]), 1e-6)
        self.assertGreater(abs(d0[0, 2]), 1e-6)
        self.assertAlmostEqual(d1[0, 0] / d0[0, 0], np.exp((1 - 0.77) * k * zl), delta=1e-9)
        self.assertAlmostEqual(d1[0, 2] / d0[0, 2], np.exp((1 - 0.62) * k * zl), delta=1e-9)

    def test_swell_knee(self):
        a = Wind(8.0)
        b = Wind(8.0, profile="swell_knee")
        self.assertEqual(b.knee_m, 7.5)
        for w in (a, b):
            self.assertAlmostEqual(float(w.speed(10.0)), 8.0, delta=1e-9)
        self.assertAlmostEqual(float(b.speed(20.0)), float(b.speed(10.0)), delta=1e-12)
        sh = lambda w: float(w.speed(4.0) - w.speed(2.0))
        self.assertGreater(sh(b), sh(a))
        c = Wind(8.0, profile="swell_knee", knee_m=12.0)
        self.assertAlmostEqual(float(c.speed(10.0)), 8.0, delta=1e-9)
        self.assertAlmostEqual(float(c.speed(30.0)), float(c.speed(12.0)), delta=1e-12)

    def test_turbulence(self):
        w = Wind(10.0, from_deg=270.0)
        flat = Environment(w, [])
        self.assertTrue((flat.gust(1.0, 2.0, 10.0, 0.0) == 0).all())
        env = Environment(w, [], turbulence="iso")
        rng = np.random.default_rng(5)
        n = 20000
        x = rng.uniform(0, 5000, n)
        y = rng.uniform(0, 5000, n)
        t = rng.uniform(0, 1000, n)
        g = env.gust(x, y, np.full(n, 10.0), t)
        self.assertEqual(g.shape, (n, 3))
        u = g[:, 0] * w.ex + g[:, 1] * w.ey
        target = 0.06 * (1 + 0.043 * 10.0) * 10.0
        print("turb std u", u.std(), "target", target, "mean", u.mean())
        self.assertLess(abs(u.std() / target - 1.0), 0.15)
        self.assertLess(abs(u.mean()), 0.1)
        self.assertAlmostEqual(g[:, 2].std() / u.std(), 0.5, delta=0.1)
        v = -g[:, 0] * w.ey + g[:, 1] * w.ex
        self.assertAlmostEqual(v.std() / u.std(), 0.8, delta=0.1)
        # 3 s gust factor along a 600 s series at a fixed point
        gf = gust_factor()
        print("3 s gust factor", gf)
        self.assertTrue(1.05 <= gf <= 1.5)

    def test_to_dict_model(self):
        env = Environment(Wind(10.0, profile="swell_knee"), [], decay=(0.77, 0.62),
                          turbulence="iso")
        m = json.loads(json.dumps(env.to_dict()))["model"]
        self.assertEqual(m["decay"], [0.77, 0.62])
        self.assertEqual(m["profile"], "swell_knee")
        self.assertEqual(m["knee_m"], 7.5)
        self.assertEqual(m["turbulence"], "iso")
        self.assertNotIn("kx", json.dumps(env.to_dict()))


if __name__ == "__main__":
    unittest.main()
