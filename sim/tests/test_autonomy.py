import unittest

import numpy as np

from sim.aircraft import BASELINE
from sim.autonomy import (HAND_PICKED, EstimatedDSController, Estimate, Estimator, Mode,
                          ModeMachine, Phase, SensorSuite, Thresholds, default_cycle, fly,
                          initial_state)
from sim.autonomy import (Environment, PerfectEstimator, Reference, ReferenceTracker, Wind,
                          initial_state_ref, reference_from_dsopt, run_case)


def make_est(h=5.0, hd=np.pi, V=15.0, z=None, ustar=0.5, wind_from=270.0, valid=True):
    return Estimate(t=0.0, pos=np.array([0.0, 0.0, h if z is None else z]),
                    v_ground=np.zeros(3), v_air=np.zeros(3), airspeed=V, heading=hd,
                    wind=np.zeros(3), h_surface=h, z=h if z is None else z, ustar_est=ustar,
                    wind_dir_est_deg=wind_from, shear_quality=1.0, valid=valid)


FLIGHT = dict(airborne=True, on_water=False, fault=False, battery=1.0,
              distance_to_target_m=5000.0, link_available=False, wind_u10_est=10.0, hs_est=1.0)


class TestModeMachine(unittest.TestCase):
    def test_soar_to_land_after_t_hold(self):
        m = ModeMachine(BASELINE, Thresholds(), mode=Mode.SOAR)
        e = make_est(h=20.0, ustar=0.1)
        for t in range(0, 60):
            self.assertIs(m.step(e, FLIGHT, float(t)), Mode.SOAR)
        self.assertIs(m.step(e, FLIGHT, 61.0), Mode.LAND)
        t, a, b, reason = m.log[-1]
        self.assertEqual((a, b), (Mode.SOAR, Mode.LAND))
        self.assertIn("shear", reason)

    def test_shear_recovery_resets_timer(self):
        m = ModeMachine(BASELINE, Thresholds(), mode=Mode.SOAR)
        for t in range(0, 50):
            m.step(make_est(h=20.0, ustar=0.1), FLIGHT, float(t))
        m.step(make_est(h=20.0, ustar=0.5), FLIGHT, 50.0)
        self.assertIs(m.step(make_est(h=20.0, ustar=0.1), FLIGHT, 70.0), Mode.SOAR)

    def test_float_to_takeoff(self):
        m = ModeMachine(BASELINE, Thresholds(), mode=Mode.FLOAT)
        fl = dict(FLIGHT, airborne=False, on_water=True, wind_u10_est=9.0, hs_est=2.0)
        self.assertIs(m.step(make_est(), fl, 5.0), Mode.TAKEOFF)
        self.assertEqual(m.log[-1][3][:4], "wind")
        m2 = ModeMachine(BASELINE, Thresholds(), mode=Mode.FLOAT)
        fl2 = dict(fl, hs_est=5.0)
        self.assertIs(m2.step(make_est(), fl2, 5.0), Mode.FLOAT)

    def test_fault_leads_to_safe(self):
        for mode in (Mode.SOAR, Mode.LOITER, Mode.TASK, Mode.TAKEOFF):
            m = ModeMachine(BASELINE, Thresholds(), mode=mode)
            self.assertIs(m.step(make_est(), dict(FLIGHT, fault=True), 3.0), Mode.SAFE)
            self.assertEqual(m.log[-1], (3.0, mode, Mode.SAFE, "fault"))

    def test_safe_exits_to_water(self):
        m = ModeMachine(BASELINE, Thresholds(), mode=Mode.SAFE)
        self.assertIs(m.step(make_est(), dict(FLIGHT, fault=True), 1.0), Mode.LAND)
        m = ModeMachine(BASELINE, Thresholds(), mode=Mode.SAFE)
        fl = dict(FLIGHT, airborne=False, on_water=True)
        self.assertIs(m.step(make_est(), fl, 1.0), Mode.FLOAT)


class TestController(unittest.TestCase):
    def ctrl(self, **kw):
        return EstimatedDSController(BASELINE, {"p": dict(HAND_PICKED)}, noise_k=0.0, **kw)

    def expected_cl(self, V, gam, target):
        ac = BASELINE
        lvl = 2 * ac.mass * 9.81 * np.cos(gam) / (ac.rho * V ** 2 * ac.wing_area)
        return float(np.clip(lvl + 2.0 * (target - gam), ac.cl_min, ac.cl_max))

    def test_flight_path_hold(self):
        c = self.ctrl()
        e = make_est(h=5.0, hd=np.pi, V=15.0)
        e.v_ground = np.array([0.0, 0.0, 15.0 * np.sin(0.2)])
        cl, _ = c.command(e, 0.0)
        self.assertAlmostEqual(cl, self.expected_cl(15.0, 0.2, 0.35), places=6)

    def test_four_phases_with_perfect_estimate(self):
        p, c = HAND_PICKED, self.ctrl()
        # wind from 270 blows toward east: downwind psi = 0, upwind psi = pi
        cl, mu = c.command(make_est(h=5.0, hd=np.pi), 0.0)
        self.assertAlmostEqual(cl, self.expected_cl(15.0, 0.0, p["gamma_climb"]), places=6)
        self.assertAlmostEqual(mu, 0.0, places=6)
        cl, mu = c.command(make_est(h=12.5, hd=np.pi), 1.0)  # above h_top: turn
        self.assertIs(c.phase, Phase.TURN)
        self.assertAlmostEqual(cl, p["cl_turn"], places=6)
        self.assertAlmostEqual(mu, p["turn_sign"] * p["mu_turn"], places=6)
        cl, mu = c.command(make_est(h=8.0, hd=0.1), 2.0)  # pointing downwind: descend
        self.assertIs(c.phase, Phase.DESC)
        self.assertAlmostEqual(cl, self.expected_cl(15.0, 0.0, -p["gamma_desc"]), places=6)
        self.assertAlmostEqual(mu, 0.0, places=1)
        cl, mu = c.command(make_est(h=1.5, hd=0.0), 3.0)  # below h_bot: S-turn back
        self.assertIs(c.phase, Phase.RETURN)
        self.assertAlmostEqual(mu, -p["turn_sign"] * p["mu_turn"], places=6)
        c.command(make_est(h=4.0, hd=np.pi - 0.1), 4.0)
        self.assertIs(c.phase, Phase.CLIMB)
        self.assertEqual(len(c.cycle_starts), 2)

    def test_low_airspeed_ends_climb(self):
        c = self.ctrl()
        c.command(make_est(h=5.0, V=0.9 * 1.15 * BASELINE.v_stall), 0.0)
        self.assertIs(c.phase, Phase.TURN)

    def test_clearance_guard(self):
        c = EstimatedDSController(BASELINE, {"p": dict(HAND_PICKED)})
        cl, mu = c.command(make_est(h=0.5), 0.0)
        self.assertEqual(mu, 0.0)
        self.assertEqual(cl, BASELINE.cl_best_ld)
        self.assertEqual(c.guard_count, 1)

    def test_guard_margin_grows_with_noise(self):
        c = EstimatedDSController(BASELINE, {"p": dict(HAND_PICKED)})
        rng = np.random.default_rng(0)
        for k in range(600):
            c.command(make_est(h=6.0 + rng.normal(0, 0.8)), 0.02 * k)
        self.assertAlmostEqual(c.noise_est(), 0.8 * (1 - 0.99 ** 599) ** 0.5, delta=0.25)

    def test_waypoint_bias_limited_to_30_deg(self):
        c = self.ctrl()
        e = make_est()
        self.assertAlmostEqual(c.bias(e, np.radians(80.0), 5.0), np.radians(30.0))
        self.assertAlmostEqual(c.bias(e, np.radians(10.0), 5.0), np.radians(10.0))
        self.assertEqual(c.bias(e, np.radians(80.0), -1.0), 0.0)


class TestFly(unittest.TestCase):
    def test_fly_10s_standard(self):
        env = Environment.flat(Wind(10.0, 270.0))
        cyc = default_cycle(BASELINE, 10.0)
        suite = SensorSuite.standard(1)
        r = fly(BASELINE, env, suite, Estimator(suite.names), EstimatedDSController(BASELINE, cyc),
                initial_state(BASELINE, env, cyc), 10.0, target=[2500.0, 900.0])
        m = r["metrics"]
        self.assertGreater(len(r["t"]), 10)
        for k in ("min_h", "max_n", "ground_speed_toward_target", "wind_rms", "h_rms",
                  "heading_rms"):
            self.assertTrue(np.isfinite(m[k]), k)
        self.assertTrue(np.all(np.isfinite(r["state"])))


class TestReferenceTracker(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.ref = reference_from_dsopt(BASELINE, 13.0, "log")

    def test_phase_matching_returns_the_node(self):
        ref = self.ref
        for k in (0, 7, 15, 23, 31, ref.n - 1):
            ph = ref.match(ref.z[k], ref.psi[k])
            d = abs((ph - k + ref.n / 2) % ref.n - ref.n / 2)
            self.assertLess(d, 1e-6, k)
        # with the forward window from a nearby previous phase
        self.assertAlmostEqual(ref.match(ref.z[20], ref.psi[20], prev=19.3), 20.0, places=6)

    def test_tracker_phase_follows_rotated_wind_frame(self):
        c = ReferenceTracker(BASELINE, self.ref)
        k = 12
        psi_w = np.radians(30.0)  # wind blows toward 30 deg (math angle)
        e = make_est(h=float(self.ref.z[k]), hd=float(self.ref.psi[k]) + psi_w, V=15.0,
                     wind_from=np.degrees(np.pi / 2 - psi_w) + 180.0)
        e.wind_valid = True
        c.command(e, 0.0)
        self.assertAlmostEqual(c.phase_idx, k, places=4)

    def test_perfect_estimate_flat_log_13(self):
        m = run_case(BASELINE, "flat", 13.0, "truth", 120.0)
        self.assertFalse(m["crashed"])
        self.assertGreaterEqual(m["cycles"], 10)
        self.assertGreaterEqual(m["energy_gain_per_cycle"], 0.0)


if __name__ == "__main__":
    unittest.main()
