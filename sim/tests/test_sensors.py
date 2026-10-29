import unittest

import numpy as np

from sim.estimator import Estimator, run_estimator
from sim.seawind import Environment, Wind
from sim.sensors import SensorSuite, truth_from_state


def zeroed(suite):
    for sp in suite.specs.values():
        sp.noise = 0.0
        sp.bias = 0.0
        sp.bias_walk = 0.0
    for n, sp in suite.specs.items():
        suite._bias[n] = suite._vec(0.0, n)
    return suite


def level_flight(env, duration, dt=0.01, V=15.0, z=5.0):
    times = np.arange(0.0, duration, dt)
    w = env.wind_at(0.0, 0.0, z, 0.0)
    states = np.zeros((len(times), 6))
    states[:, 0] = (-V + w[0]) * times
    states[:, 2] = z
    states[:, 3] = V
    states[:, 5] = np.pi  # into the wind (wind blows toward +x)
    return states, times


class UniformEnv:
    """Uniform horizontal wind, flat surface."""

    def __init__(self, speed=6.0, from_deg=270.0):
        r = np.radians(from_deg + 180.0)
        self.w = np.array([speed * np.sin(r), speed * np.cos(r), 0.0])

    def wind_at(self, x, y, z, t):
        return self.w.copy()

    def surface(self, x, y, t):
        return 0.0, 0.0, 0.0


def circles(env, duration=60.0, period=20.0, V=15.0, z=30.0, dt=0.01):
    times = np.arange(0.0, duration, dt)
    om = 2 * np.pi / period
    psi = om * times
    states = np.zeros((len(times), 6))
    states[:, 0] = np.cumsum(V * np.cos(psi) * dt + env.w[0] * dt)
    states[:, 1] = np.cumsum(V * np.sin(psi) * dt + env.w[1] * dt)
    states[:, 2] = z
    states[:, 3] = V
    states[:, 5] = psi
    return states, times


class ScaledPitot:
    """Wraps a suite and scales the pitot reading (injected scale error)."""

    def __init__(self, suite, k):
        self.suite, self.k = suite, k

    def measure(self, truth, t):
        m = self.suite.measure(truth, t)
        if "pitot" in m:
            m["pitot"]["airspeed"] *= self.k
        return m


class LogEnv:
    """Pure log profile with fixed ustar and z0, flat surface, wind toward +x."""

    def __init__(self, ustar=0.37, z0=2e-4):
        self.ustar, self.z0 = ustar, z0

    def wind_at(self, x, y, z, t):
        return np.array([self.ustar / 0.4 * np.log(max(z, 1e-3) / self.z0), 0.0, 0.0])

    def surface(self, x, y, t):
        return 0.0, 0.0, 0.0


class TestSensors(unittest.TestCase):
    def test_zero_noise_matches_truth(self):
        env = Environment.flat(Wind(10.0))
        states, times = level_flight(env, 5.0)
        suite = zeroed(SensorSuite.standard(1))
        est = run_estimator(suite, Estimator(suite.names), states, env, times)
        e = est[-1]
        truth = truth_from_state(states[-1], env, times[-1])
        self.assertTrue(e.valid)
        self.assertAlmostEqual(e.airspeed, 15.0, delta=1e-6)
        self.assertLess(np.abs(e.wind - truth["wind"]).max(), 0.05)
        self.assertAlmostEqual(e.h_surface, 5.0, delta=1e-6)

    def test_noisy_wind_error(self):
        env = Environment.flat(Wind(10.0))
        states, times = level_flight(env, 60.0)
        suite = SensorSuite.standard(3)
        est = run_estimator(suite, Estimator(suite.names), states, env, times)
        wt = env.wind_at(0.0, 0.0, 5.0, 0.0)
        err = np.array([e.wind - wt for e in est if e.t > 10.0])
        print("wind err mean", err.mean(axis=0), "std", err.std(axis=0))
        self.assertLess(err.std(axis=0).max(), 0.5)
        # The mean error is the unobserved pitot bias (0.3 m/s plus its walk): a straight
        # glide cannot observe it. Observing it from GNSS during turns is future work.
        self.assertLess(np.abs(err.mean(axis=0)).max(), 0.45)

    def test_circles_no_mag(self):
        env = UniformEnv(6.0, 270.0)
        states, times = circles(env)
        suite = SensorSuite.minimal(4)
        est = run_estimator(suite, Estimator(suite.names), states, env, times)
        early = [e for e in est if e.t < 2.0]
        self.assertFalse(any(e.wind_valid for e in early))
        late = [e for e in est if e.t >= 40.0]
        self.assertTrue(all(e.wind_valid for e in late))
        err = np.array([np.hypot(*(e.wind[:2] - env.w[:2])) for e in late])
        print("circles no-mag wind err max", err.max(), "mean", err.mean(),
              "scale", est[-1].pitot_scale_est)
        self.assertLess(err.max(), 0.5)
        self.assertEqual(est[-1].wind[2], 0.0)

    def test_pitot_scale_no_mag(self):
        # Pitot reads 0.97 of the true airspeed, so the fitted s is near 1/0.97.
        # The default pitot additive bias (0.3 m/s) is zeroed here: at constant airspeed
        # an additive bias and a scale error cannot be told apart.
        env = UniformEnv(6.0, 270.0)
        states, times = circles(env)
        suite = SensorSuite.minimal(4)
        suite.specs["pitot"].bias = 0.0
        suite.specs["pitot"].bias_walk = 0.0
        suite._bias["pitot"] = suite._vec(0.0, "pitot")
        est = run_estimator(ScaledPitot(suite, 0.97), Estimator(suite.names), states, env, times)
        e = est[-1]
        print("pitot_scale_est", e.pitot_scale_est, "1/0.97 =", 1 / 0.97)
        self.assertAlmostEqual(e.pitot_scale_est / (1 / 0.97), 1.0, delta=0.01)

    def test_scale_exposed_with_mag(self):
        env = UniformEnv(6.0, 270.0)
        states, times = circles(env)
        suite = SensorSuite.standard(4)
        est = run_estimator(suite, Estimator(suite.names), states, env, times)
        self.assertTrue(est[-1].wind_valid)
        self.assertGreater(est[-1].pitot_scale_est, 0.9)
        self.assertLess(est[-1].pitot_scale_est, 1.1)

    def test_straight_no_mag_not_valid(self):
        env = Environment.flat(Wind(10.0))
        states, times = level_flight(env, 20.0)
        suite = SensorSuite.minimal(2)
        est = run_estimator(suite, Estimator(suite.names), states, env, times)
        self.assertFalse(est[-1].wind_valid)

    def test_profile_fit(self):
        env = LogEnv()
        times = np.arange(0.0, 60.0, 0.01)
        z = 1.0 + 14.0 * times / 60.0
        V, climb = 15.0, 14.0 / 60.0
        states = np.zeros((len(times), 6))
        states[:, 2] = z
        states[:, 3] = V
        states[:, 4] = np.arcsin(climb / V)
        states[:, 5] = np.pi
        suite = SensorSuite.standard(5)
        est = run_estimator(suite, Estimator(suite.names), states, env, times)
        e = est[-1]
        print("ustar_est", e.ustar_est, "dir", e.wind_dir_est_deg, "q", e.shear_quality)
        self.assertLess(abs(e.ustar_est - 0.37) / 0.37, 0.25)
        self.assertLess(abs(e.wind_dir_est_deg - 270.0), 10.0)

    def test_rates(self):
        env = Environment.flat(Wind(10.0))
        suite = SensorSuite.standard(2)
        s = np.array([0.0, 0.0, 5.0, 15.0, 0.0, np.pi])
        cnt = {n: 0 for n in suite.names}
        for k in range(200):
            t = k * 0.005
            m = suite.measure(truth_from_state(s, env, t), t)
            for n in m:
                cnt[n] += 1
        self.assertAlmostEqual(cnt["gnss"], 5, delta=1)
        self.assertAlmostEqual(cnt["imu"], 200, delta=2)

    def test_power_mass(self):
        self.assertAlmostEqual(SensorSuite.minimal().mass_g(), 36.0)
        self.assertAlmostEqual(SensorSuite.minimal().power_w(), 0.46)


if __name__ == "__main__":
    unittest.main()
