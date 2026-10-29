import json
import os
import tempfile
import unittest

import numpy as np

from sim.seawind import G, Environment, WaveTrain, Wind, gust_factor, separation_steepness


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


class SteepMono(WaveTrain):
    """One east-going component, k = 0.1, amplitude set by `ak`."""

    ak = 0.35

    def components(self, rng):
        k = 0.1
        return {"a": np.array([self.ak / k]), "k": np.array([k]),
                "omega": np.array([np.sqrt(G * k)]), "dx": np.array([1.0]),
                "dy": np.array([0.0]), "phase": np.array([0.0])}


class TestWaveWindInteraction(unittest.TestCase):
    def test_auto_wind_sea_is_log(self):
        w = Wind(10.0)
        tr = WaveTrain.fully_developed(w)
        env = Environment(w, [tr], profile_mode="auto")
        cp, a, f = env.wave_age()
        self.assertAlmostEqual(cp, G * tr.tp / (2 * np.pi), places=9)
        self.assertAlmostEqual(a, 10.0 / cp, places=9)
        # A is 0.826 for a fully developed sea, just inside the 0.83 threshold,
        # so f is about 1e-4 rather than exactly 0.
        self.assertLess(f, 5e-4)
        z = np.array([0.5, 2.0, 7.5, 10.0, 30.0])
        np.testing.assert_allclose(env._mean_speed(z), w.speed(z), rtol=1e-3)
        flat = Environment(w, [], profile_mode="auto")
        self.assertEqual(flat.wave_age()[2], 0.0)
        np.testing.assert_array_equal(flat._mean_speed(z), w.speed(z))

    def test_auto_swell_is_knee(self):
        w = Wind(10.0)
        # cp = 7 U10 (A = 0.143 < 0.15). The spec's cp = 3 U10 gives A = 0.333, f = 0.82, not 1.
        tp = 2 * np.pi * 70.0 / G
        env = Environment(w, [WaveTrain(hs=2.0, tp=tp, to_deg=90.0)], profile_mode="auto")
        cp, a, f = env.wave_age()
        self.assertAlmostEqual(cp, 70.0, places=9)
        self.assertAlmostEqual(f, 1.0, places=9)
        knee = Wind(10.0, profile="swell_knee")
        z = np.array([0.5, 2.0, 7.5, 10.0, 30.0])
        np.testing.assert_allclose(env._mean_speed(z), knee.speed(z), rtol=1e-12)
        self.assertAlmostEqual(float(env._mean_speed(10.0)), 10.0, places=9)

    def test_f_monotonic_in_wave_age(self):
        w = Wind(10.0)
        fs = []
        for cpv in (8.0, 10.0, 12.0, 20.0, 40.0, 80.0):
            env = Environment(w, [WaveTrain(hs=1.0, tp=2 * np.pi * cpv / G, to_deg=90.0)],
                              profile_mode="auto")
            fs.append((env.wave_age()[1], env.wave_age()[2]))
        fs.sort()
        self.assertTrue(all(b[1] <= a[1] + 1e-12 for a, b in zip(fs, fs[1:])))
        self.assertAlmostEqual(fs[0][1], 1.0)

    def test_fixed_default_unchanged(self):
        w = Wind(10.0)
        tr = WaveTrain.fully_developed(w)
        a = Environment(w, [tr])
        b = Environment(w, [tr], profile_mode="fixed", shelter=False)
        x = np.linspace(0, 300, 40)
        np.testing.assert_array_equal(a.wind_at(x, 0 * x, 5 + 0 * x, 0 * x),
                                      b.wind_at(x, 0 * x, 5 + 0 * x, 0 * x))
        with self.assertRaises(ValueError):
            Environment(w, [], profile_mode="x")

    def _lee_mean(self, env, zloc, ak=0.35):
        k = 0.1
        th = np.linspace(0.0, 2 * np.pi, 4001)[:-1]
        x = th / k  # phase = k x at t = 0 (phase 0)
        eta, _, _ = env.surface(x, 0 * x, 0 * x)
        u = env.wind_at(x, 0 * x, eta + zloc, 0 * x)[:, 0]  # wind blows east
        return u[th < np.pi].mean(), u[th >= np.pi].mean()

    def test_shelter(self):
        w = Wind(30.0, 270.0)
        tr = SteepMono(hs=1.0, tp=5.0, to_deg=90.0)
        off = Environment(w, [tr])
        on = Environment(w, [tr], shelter=True)
        self.assertGreater(on.urel[0], 0.0)
        lee_off, win_off = self._lee_mean(off, 1.0)
        lee_on, win_on = self._lee_mean(on, 1.0)
        self.assertAlmostEqual(win_on, win_off, places=9)  # windward half untouched
        self.assertLess(lee_on, win_on)
        self.assertLess(lee_on, lee_off - 0.5)
        # gone by 3/k
        lee3, _ = self._lee_mean(on, 30.0)
        lee3o, _ = self._lee_mean(off, 30.0)
        self.assertLess(abs(lee3 - lee3o) / lee3o, 5e-3)
        # vertical perturbation scaled by (1 - r)
        x = np.array([0.5 * np.pi / 0.1])
        z = np.array([5.0])
        r = float(on.shelter_r(x, 0 * x, 0 * x)[0])
        sc = float(separation_steepness(w.ustar, np.sqrt(G / 0.1)))
        self.assertAlmostEqual(r, min(1.0, (0.35 - sc) / sc), places=6)
        self.assertGreater(r, 0.0)
        wo = off.wind_at(x, 0 * x, z, 0 * x)[0, 2]
        self.assertAlmostEqual(on.wind_at(x, 0 * x, z, 0 * x)[0, 2] / wo, 1.0 - r, places=9)

    def test_shelter_gentle_and_slow(self):
        w = Wind(30.0, 270.0)
        gentle = SteepMono(hs=1.0, tp=5.0, to_deg=90.0)
        gentle.ak = 0.05
        x = np.linspace(0, 60, 50)
        a = Environment(w, [gentle]).wind_at(x, 0 * x, 2 + 0 * x, 0 * x)
        b = Environment(w, [gentle], shelter=True).wind_at(x, 0 * x, 2 + 0 * x, 0 * x)
        np.testing.assert_array_equal(a, b)
        # wave faster than the wind: no separation
        slow_wind = Wind(5.0, 270.0)
        steep = SteepMono(hs=1.0, tp=5.0, to_deg=90.0)
        env = Environment(slow_wind, [steep], shelter=True)
        self.assertLess(env.urel[0], 0.0)
        ref = Environment(slow_wind, [steep])
        np.testing.assert_array_equal(env.wind_at(x, 0 * x, 2 + 0 * x, 0 * x),
                                      ref.wind_at(x, 0 * x, 2 + 0 * x, 0 * x))

    def test_wave_age_direction(self):
        w = Wind(10.0, 270.0)  # blows toward 90
        tp = 2 * np.pi * 70.0 / G

        def wa(to_deg):
            return Environment(w, [WaveTrain(hs=2.0, tp=tp, to_deg=to_deg)],
                               profile_mode="auto").wave_age()
        _, a0, f0 = wa(90.0)
        self.assertAlmostEqual(f0, 1.0, places=9)
        _, a60, _ = wa(30.0)
        self.assertAlmostEqual(a60, 0.5 * a0, places=9)
        _, ac, fc = wa(270.0)
        self.assertLess(ac, 0.0)
        self.assertEqual(fc, 0.0)

    def test_shelter_two_components(self):
        w = Wind(30.0, 270.0)
        lam = (40.0, 44.0)

        class Two(WaveTrain):
            def components(self, rng):
                k = 2 * np.pi / np.array(lam)
                return {"a": 0.12 / k, "k": k, "omega": np.sqrt(G * k),
                        "dx": np.ones(2), "dy": np.zeros(2), "phase": np.zeros(2)}
        tr = Two(hs=1.0, tp=5.0, to_deg=90.0)
        off = Environment(w, [tr])
        on = Environment(w, [tr], shelter=True)
        x = np.linspace(0.0, 440.0, 8801)
        r = on.shelter_r(x, 0 * x, 0 * x)
        self.assertGreater(r.max(), 0.0)
        i = int(np.argmax(r))
        pt = (x[i:i + 1], np.zeros(1), 1.0 + on.surface(x[i:i + 1], 0 * x[i:i + 1], 0 * x[i:i + 1])[0], np.zeros(1))
        self.assertLess(on.wind_at(*pt)[0, 0], off.wind_at(*pt)[0, 0])

    def test_shelter_fully_developed(self):
        w = Wind(15.0, 270.0)
        env = Environment(w, [WaveTrain.fully_developed(w)], shelter=True)
        g = np.linspace(0.0, 200.0, 101)
        xx, yy = np.meshgrid(g, g)
        r = env.shelter_r(xx, yy, 0 * xx)
        env.wind_at(xx.ravel()[:50], yy.ravel()[:50], 5 + 0 * xx.ravel()[:50], 0 * xx.ravel()[:50])
        print("fully developed u10 15: fraction of surface with r > 0 =", float((r > 0).mean()))

    def test_separation_steepness(self):
        self.assertAlmostEqual(float(separation_steepness(0.05, 1.0)), 0.396 / 1.5, delta=0.003)
        self.assertAlmostEqual(float(separation_steepness(2.0, 1.0)), 0.1026 / 1.5, delta=0.001)
        q = np.logspace(-2, 1, 50)
        v = separation_steepness(q, 1.0)
        self.assertEqual(v.shape, q.shape)
        self.assertTrue(np.all(np.diff(v) < 0.0))

    def test_to_dict_wave_wind(self):
        w = Wind(10.0)
        env = Environment(w, [WaveTrain.fully_developed(w)], profile_mode="auto", shelter=True)
        m = json.loads(json.dumps(env.to_dict()))["model"]
        self.assertEqual(m["profile_mode"], "auto")
        self.assertEqual(set(m["wave_age"]), {"cp", "A", "f"})
        self.assertTrue(m["shelter"]["on"])
        self.assertGreater(m["shelter"]["steepness"], 0.0)
        flat = json.loads(json.dumps(Environment(w, []).to_dict()))["model"]
        self.assertIsNone(flat["wave_age"]["cp"])


if __name__ == "__main__":
    unittest.main()
