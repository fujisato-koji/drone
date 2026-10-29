import unittest

import numpy as np

from sim.seasense import (ForwardScanner, LidarAltimeter, LocalWaveEstimator, RadarAltimeter)
from sim.seawind import G, Environment, WaveTrain, Wind


class Mono(WaveTrain):
    def __init__(self, a=1.0, lam=60.0, th=0.0):
        super().__init__(hs=1.0, tp=8.0, to_deg=0.0)
        self._c = (a, lam, th)

    def components(self, rng):
        a, lam, th = self._c
        k = 2 * np.pi / lam
        return {"a": np.array([a]), "k": np.array([k]), "omega": np.array([np.sqrt(G * k)]),
                "dx": np.array([np.cos(th)]), "dy": np.array([np.sin(th)]),
                "phase": np.array([0.7])}


def mono_env(u10=8.0, **kw):
    return Environment(Wind(u10), [Mono(**kw)])


class TestSensors(unittest.TestCase):
    def test_radar_level_returns_true_height(self):
        env = mono_env(lam=60.0)
        rng = np.random.default_rng(1)
        radar = RadarAltimeter()
        err = []
        for i in range(200):
            t = i * 0.0125
            s = [15.0 * t, 3.0, 8.0, 15.0, 0.0, 0.0]
            m = radar.measure(s, 0.0, env, t, rng)
            self.assertEqual(len(m), 1)
            x, y, tt, h = m[0]
            truth = s[2] - float(env.surface(x, y, tt)[0])
            err.append(h - truth)
        self.assertLess(np.max(np.abs(err)), 3 * radar.noise)
        self.assertLess(abs(np.mean(err)), 0.02)

    def test_surface_point_is_on_surface(self):
        env = mono_env(lam=30.0, a=1.0)
        radar = RadarAltimeter(noise=0.0)
        s = [0.0, 0.0, 6.0, 15.0, 0.1, 0.4]
        x, y, t, h = radar.measure(s, 0.2, env, 0.0, np.random.default_rng(0))[0]
        self.assertAlmostEqual(s[2] - h, float(env.surface(x, y, t)[0]), places=5)

    def test_lidar_nothing_when_banked(self):
        env = mono_env(u10=8.0)
        lidar = LidarAltimeter()
        rng = np.random.default_rng(1)
        got = 0
        for i in range(400):
            got += len(lidar.measure([0.0, 0.0, 8.0, 15.0, 0.0, 0.0], np.radians(20), env,
                                     i * 0.05, rng))
        self.assertEqual(got, 0)

    def test_lidar_dropout_half_in_wind(self):
        env = mono_env(u10=8.0)
        lidar = LidarAltimeter()
        rng = np.random.default_rng(2)
        got = sum(len(lidar.measure([0.0, 0.0, 8.0, 15.0, 0.0, 0.0], 0.0, env, i * 0.05, rng))
                  for i in range(2000))
        self.assertTrue(0.4 < got / 2000 < 0.6)

    def test_forward_scanner(self):
        env = mono_env()
        fs = ForwardScanner()
        m = fs.measure([0.0, 0.0, 8.0, 15.0, 0.0, 0.0], 0.0, env, 0.0, np.random.default_rng(3))
        self.assertTrue(1 <= len(m) <= 8)
        self.assertTrue(all(10.0 <= p[0] <= 40.0 for p in m))


class TestEstimator(unittest.TestCase):
    def feed(self, x, y, t, truth_fn, est):
        for i in range(len(t)):
            est.add([(x[i], y[i], t[i], -truth_fn(x[i], y[i], t[i]))], 0.0)

    def setUp(self):
        lam, self.a, self.th = 60.0, 1.0, np.pi / 2     # direction 90 deg (toward +y)
        k = 2 * np.pi / lam
        self.k, self.om = k, np.sqrt(G * k)
        self.eta = lambda x, y, t: self.a * np.cos(
            k * (np.cos(self.th) * x + np.sin(self.th) * y) - self.om * t + 0.7)
        self.t = np.arange(0.0, 20.0, 0.0125)

    def test_straight_path_predicts_ahead(self):
        # A straight path at constant speed samples one frequency only, so (k, theta) are not
        # separable from it; amplitude and the prediction along the path are.
        t = self.t
        x, y = 15.0 * t, 0.0 * t
        est = LocalWaveEstimator()
        self.feed(x, y, t, self.eta, est)
        f = est.fit(20.0)
        self.assertAlmostEqual(float(np.sum(f.a)), self.a, delta=0.05)
        tt = np.array([21.0, 22.0, 23.0])
        pred = f.surface(15.0 * tt, 0.0 * tt, tt)[0]
        self.assertLess(np.max(np.abs(pred - self.eta(15.0 * tt, 0.0 * tt, tt))), 0.1)
        self.assertLess(f.quality, 0.01)

    def test_dogleg_path_recovers_wave(self):
        t = self.t
        h = np.where(t < 10.0, 0.0, np.radians(70.0))
        x, y = np.cumsum(15.0 * np.cos(h)) * 0.0125, np.cumsum(15.0 * np.sin(h)) * 0.0125
        est = LocalWaveEstimator()
        self.feed(x, y, t, self.eta, est)
        f = est.fit(20.0)
        i = int(np.argmax(f.a))
        self.assertAlmostEqual(f.a[i], self.a, delta=0.05 * self.a)
        self.assertAlmostEqual(f.wavelength[i], 60.0, delta=3.0)
        dth = (np.degrees(f.th[i]) - 90.0 + 180.0) % 360.0 - 180.0
        self.assertLess(abs(dth), 10.0)
        # 3 s ahead along the last heading
        tt = 23.0
        xx, yy = x[-1] + 15.0 * np.cos(h[-1]) * 3.0, y[-1] + 15.0 * np.sin(h[-1]) * 3.0
        self.assertLess(abs(float(f.surface(xx, yy, tt)[0]) - self.eta(xx, yy, tt)), 0.1)

    def test_environment_object(self):
        t = self.t
        h = np.where(t < 10.0, 0.0, np.radians(70.0))
        x, y = np.cumsum(15.0 * np.cos(h)) * 0.0125, np.cumsum(15.0 * np.sin(h)) * 0.0125
        est = LocalWaveEstimator()
        self.feed(x, y, t, self.eta, est)
        f = est.fit(20.0)
        env = f.environment(8.0, 270.0)
        e1 = float(env.surface(10.0, 20.0, 21.0)[0])
        e2 = float(f.surface(10.0, 20.0, 21.0)[0])
        self.assertAlmostEqual(e1, e2, places=9)
        w = env.wind_at(10.0, 20.0, 5.0, 21.0)
        self.assertEqual(np.shape(w), (3,))
        self.assertGreater(f.n_samples, 100)

    def test_too_few_samples(self):
        est = LocalWaveEstimator()
        est.add([(0.0, 0.0, 0.0, 5.0)], 5.0)
        self.assertIsNone(est.fit(1.0))


if __name__ == "__main__":
    unittest.main()
