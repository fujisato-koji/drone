"""The pilot's choice, the forward-face guide, the powered leg over swell (datum and heading hold) and the redesigned
reflex (sim.wavelib, sim.mpc.Reflex2, sim.reflex)."""
import unittest

import numpy as np

from sim.aircraft import G
from sim.aircraft_models import by_name
from sim.dsopt import folded_dcd, thrust_max
from sim.mpc import Cycle, CycleGuide, Reflex2, WaveCycleGuide, steady_entry, tip_trigger_gap
from sim.navigator import HullPoint, SteadyModel
from sim.seawind import Environment, Wind
from sim.wavelib import (PILOT_FACE, WavePilot, crest_elevation, crest_entries, entry_key, flat_start_entry,
                         pilot_candidates, pilot_choose, steady_model, swell_env)


class TestFaceGuide(unittest.TestCase):
    def setUp(self):
        e = [x for x in crest_entries("albatross-3m", "p63", 3.0, 10.0, "along", 3.0) if x["course_rel_deg"] == 0.0][0]
        self.cyc = Cycle(e)
        self.env = swell_env(3.0, 3.0, 10.0, "along", "p63")
        self.F = np.asarray(e["ride"]["family"])

    def guide(self, theta, h, face):
        from sim.mpc import world_swell
        k, om, dx, dy, ph, a = world_swell(self.env)
        x = (theta - ph) / k
        s = [x, 0.0, h + float(self.env.surface(x, 0.0, 0.0)[0]), 13.0, 0.0, 0.0]
        return WaveCycleGuide(self.cyc, self.env, 0.0, s, 0.0, 6.0, face=face)

    def test_barrier_shape(self):
        gd = self.guide(0.5, 4.0, PILOT_FACE)
        b = gd.face_barrier(np.array([0.5, 0.3, 0.15, 0.0, -0.3, -np.pi / 2, -2.5, -np.pi + 1e-9]))
        self.assertEqual(b[0], 0.0)
        self.assertAlmostEqual(b[1], 0.0)
        self.assertAlmostEqual(b[2], 0.25)
        self.assertAlmostEqual(b[3], 1.0)
        self.assertAlmostEqual(b[4], 1.0)
        self.assertAlmostEqual(b[5], 1.0)
        self.assertLess(b[6], 1.0)
        self.assertLess(b[7], 1e-6)
        self.assertLessEqual(b.max(), 1.0)

    def test_no_climb_behind_the_ride_and_asymmetric_phase(self):
        from sim.mpc import CREST_CLIMB_M
        th_own = float(np.interp(4.0, self.F[:, 0], self.F[:, 1]))
        behind = self.guide(th_own - 0.2, 4.0, PILOT_FACE)
        ahead = self.guide(th_own + 0.2, 4.0, PILOT_FACE)
        self.assertAlmostEqual(behind.target[0], 4.0, places=6)
        self.assertAlmostEqual(ahead.target[0], 4.0 + CREST_CLIMB_M, places=6)
        # a phase 0.2 rad ahead of the reference costs less than 0.2 rad behind it
        self.assertLess(float(behind._phase2(np.array(0.2))), float(behind._phase2(np.array(-0.2))))
        off = self.guide(th_own - 0.2, 4.0, None)
        self.assertAlmostEqual(float(off._phase2(np.array(0.2))), float(off._phase2(np.array(-0.2))))


class TestPowered(unittest.TestCase):
    def test_crest_elevation_is_second_order(self):
        env = swell_env(6.0, 3.0, 10.0, "along", "p63")
        k = (2 * np.pi / 10.0) ** 2 / G
        self.assertAlmostEqual(crest_elevation(env), 1.5 + 0.5 * k * 1.5 ** 2, places=3)
        self.assertEqual(crest_elevation(Environment.flat(Wind(6.0, 270.0))), 0.0)

    def test_msl_leg_holds_its_height_above_the_mean_level(self):
        ac = by_name("albatross-3m")
        env = swell_env(6.0, 3.0, 10.0, "along", "p63")
        sm = steady_model(ac, env, "msl", 2.0)
        self.assertAlmostEqual(sm.ref_height(), 10.0 + crest_elevation(env))
        e = flat_start_entry(ac, 6.0, 90.0, model=sm)
        self.assertEqual(e["datum"], "msl")
        cyc = Cycle(e)
        # a state at the reference height above the mean level over a trough: no height error for the guide
        s = [0.0, 0.0, sm.ref_height(), e["V"][0], 0.0, e["psi"][0]]
        gd = CycleGuide(cyc, env, 5.0, s, 0.0, 6.0)
        self.assertTrue(gd.msl)
        self.assertAlmostEqual(gd.h0, sm.ref_height())
        # the default leg is unchanged: above the local surface, no hold
        e0 = flat_start_entry(ac, 6.0, 90.0)
        self.assertEqual(e0["datum"], "surface")
        self.assertEqual(e0["heading_tau_s"], 0.0)

    def test_heading_hold_banks_towards_the_reference(self):
        ac = by_name("albatross-3m")
        env = Environment.flat(Wind(6.0, 270.0))
        e = flat_start_entry(ac, 6.0, 90.0, model=steady_model(ac, None, "surface", 2.0))
        cyc = Cycle(e)
        psi_ref = e["psi"][0]
        kt = np.linspace(0.0, 6.0, 24)
        for err in (0.2, -0.2):
            s = [0.0, 0.0, 10.0, e["V"][0], 0.0, psi_ref - err]
            mu = CycleGuide(cyc, env, 0.0, s, 0.0, 6.0).nominal(kt)[1]
            self.assertGreater(np.sign(err) * mu[0], 0.0)          # turns towards the reference
            self.assertLess(abs(mu[-1]), abs(mu[0]))               # and fades over the horizon
        s = [0.0, 0.0, 10.0, e["V"][0], 0.0, psi_ref]
        self.assertTrue(np.all(np.abs(CycleGuide(cyc, env, 0.0, s, 0.0, 6.0).nominal(kt)[1]) < 1e-12))

    def test_navigator_leg_carries_the_datum(self):
        from sim.navigator import HullNavigator, HullSource, VelocityHull
        ac = by_name("albatross-3m")
        sm = SteadyModel.from_aircraft(ac)
        sm.datum, sm.crest_m, sm.heading_tau_s = "msl", 1.5, 2.0
        src = HullSource([VelocityHull([HullPoint(5.0, 0.0, 0.0, "soaring", "s1")], 6.0)])
        cmd = HullNavigator(src, steady=sm).step(0.0, (0.0, 0.0), 6.0, 270.0, (0.0, 5000.0))
        self.assertEqual(cmd.point.steady["datum"], "msl")
        self.assertAlmostEqual(cmd.point.steady["height"], 11.5)
        self.assertEqual(steady_entry(cmd.point)["heading_tau_s"], 2.0)


class TestPilotChoice(unittest.TestCase):
    def test_candidates_keep_one_of_each_mirror_pair_and_the_flat_leg(self):
        keys = pilot_candidates("albatross-3m", 3.0, "along", 6.0, "p63")
        self.assertIsNone(keys[-1])
        crest = [k for k in keys[:-1] if "|crest|" in k]
        speeds = [abs(float(k.split("|")[3])) for k in crest]
        self.assertEqual(len(speeds), len(set(speeds)))

    def test_choose_the_least_cost_and_fly_it(self):
        ts = [{"key": "a", "cost_w": 5.0}, {"key": None, "cost_w": 4.0}, {"key": "b", "cost_w": 3.0}]
        self.assertEqual(pilot_choose(ts), "b")
        self.assertIsNone(pilot_choose([{"key": "a", "cost_w": 4.0}, {"key": None, "cost_w": 4.0}]))
        es = crest_entries("albatross-3m", "p63", 3.0, 10.0, "along", 6.0)
        env = swell_env(6.0, 3.0, 10.0, "along", "p63")
        k = entry_key(es[-1])
        cyc = WavePilot(es, "p63", policy="trial", choice=k).cycle("albatross-3m", env, 6.0, 0.0, 0.0)
        self.assertEqual(entry_key(cyc.d), k)
        self.assertIsNone(WavePilot(es, "p63", policy="trial", choice=None).cycle("albatross-3m", env, 6.0, 0.0, 0.0))


class TestReflex2(unittest.TestCase):
    def setUp(self):
        self.ac = by_name("baseline-2m")
        self.world = Environment.flat(Wind(9.0, 270.0))
        self.rf = Reflex2(self.ac, thrust_max(self.ac), folded_dcd(self.ac), True, seed=0)

    def plan(self, cl, mu=0.0):
        kt = np.linspace(0.0, 6.0, 24)
        return kt, np.full(24, cl), np.full(24, mu), np.zeros(24)

    def test_fires_on_a_dive_quiet_in_level_flight_and_on_the_wing_s_limit(self):
        ac, rf, w = self.ac, self.rf, self.world
        hit = rf.predict(w, 0.0, np.array([0.0, 0.0, 3.0, 12.0, -0.35, 0.0]), *self.plan(0.05), 0.0)
        self.assertEqual(hit[0], "centre")
        self.assertIsNone(rf.predict(w, 0.0, np.array([0.0, 0.0, 12.0, 11.0, -0.05, 0.0]),
                                     *self.plan(ac.cl_best_ld), 0.0))
        # the normal top turn of a soaring cycle: 62 deg of bank at 1.2 stall speeds with the lift coefficient at 0.86
        # of cl_max (the best cycles fly 1.0 of 1.16): the path curves down, but the wing is not at its limit
        top = np.array([0.0, 0.0, 30.0, 1.2 * ac.v_stall, 0.1, 0.0])
        self.assertIsNone(rf.predict(w, 0.0, top, *self.plan(0.86 * ac.cl_max, np.radians(62.0)), 0.0))
        # a 60 deg turn at 1.2 stall speeds with the wing at cl_max needs 2 g and gets 1.44: a stall (the earlier test
        # on the wings-level load factor missed it, K-022)
        hit = rf.predict(w, 0.0, np.array([0.0, 0.0, 30.0, 1.2 * ac.v_stall, 0.0, 0.0]),
                         *self.plan(ac.cl_max, np.radians(60.0)), 0.0)
        self.assertEqual(hit[0], "stall")
        # slow, wings level, the wing at cl_max: a stall; the same airspeed at a low lift coefficient (a zoom's top,
        # unloaded) is not
        hit = rf.predict(w, 0.0, np.array([0.0, 0.0, 30.0, 0.9 * ac.v_stall, 0.0, 0.0]), *self.plan(ac.cl_max), 0.0)
        self.assertEqual(hit[0], "stall")
        self.assertTrue(rf.stalls(0.9 * ac.v_stall, ac.cl_max, -0.1))
        self.assertFalse(rf.stalls(0.9 * ac.v_stall, 0.3 * ac.cl_max, -0.1))
        self.assertFalse(rf.stalls(1.2 * ac.v_stall, ac.cl_max, 0.05))      # at cl_max but the path still rises
        self.assertTrue(rf.stalls(0.4 * ac.v_stall, 0.1, 0.1))               # the planner's departure

    def test_prediction_fires_only_within_the_lead(self):
        ac, rf, w = self.ac, self.rf, self.world
        kt, cl, mu, th = self.plan(0.05)
        far = np.array([0.0, 0.0, 8.0, 12.0, -0.1, 0.0])         # a slow descent that meets the water late
        hit = rf.predict(w, 0.0, far, kt, cl, mu, th, 0.0)
        self.assertIsNotNone(hit)
        self.assertGreater(hit[1], rf.lead_s)
        rf.reset()
        self.assertIsNone(rf.check(w, 0.0, far, kt, cl, mu, th, 0.0))
        near = np.array([0.0, 0.0, 3.0, 12.0, -0.35, 0.0])
        rf.reset()
        self.assertEqual(rf.check(w, 0.0, near, kt, cl, mu, th, 0.0)[0], "centre")

    def test_override_is_scaled_with_the_attitude(self):
        ac, rf = self.ac, self.rf
        V = 1.5 * ac.v_stall
        cl_dive, mu, T_dive = rf.override(np.array([0, 0, 2.0, V, -0.3, 0.0]), "centre", 1.0)
        cl_up, _, T_up = rf.override(np.array([0, 0, 2.0, V, 0.8, 0.0]), "centre", 1.0)
        self.assertEqual(mu, 0.0)
        self.assertAlmostEqual(T_dive, rf.t_max)
        self.assertEqual(T_up, 0.0)                       # no thrust in a steep climb
        self.assertGreater(cl_dive, cl_up)                # pulls out of the dive, eases off in the climb
        q = 0.5 * ac.rho * V * V * ac.wing_area
        self.assertLessEqual(cl_dive * q / (ac.mass * G), 0.8 * ac.n_max + 1e-9)

    def test_range_trigger_grows_with_the_closing_rate(self):
        ac, rf = self.ac, self.rf
        V = 1.5 * ac.v_stall
        g0 = tip_trigger_gap(ac, rf.table, 0.8, V, 0.0)
        g1 = tip_trigger_gap(ac, rf.table, 0.8, V, 1.0)
        g2 = tip_trigger_gap(ac, rf.table, 0.8, V, 2.0)
        self.assertEqual(g0, 0.0)
        self.assertLess(g1, g2)
        # a tip closing on the water at 1.5 m/s from 0.4 m: the range trigger fires before contact
        rf.reset()
        t, gap, fired = 0.0, 0.4, None
        while gap > 0.0 and fired is None:
            rf.sense(t, gap, 0.8, V)
            hit = rf.tip_check(t)
            if hit is not None:
                fired = (hit[0], gap)
            t += 0.01
            gap -= 0.015
        self.assertIsNotNone(fired)
        self.assertEqual(fired[0], "range")
        self.assertGreater(fired[1], 0.0)
        # the same closing where the planner's sea has the water (the plan's own approach): quiet
        rf.reset()
        t, gap = 0.0, 0.4
        while gap > 0.02:
            rf.sense(t, gap, 0.8, V, gap)
            self.assertIsNone(rf.tip_check(t))
            t += 0.01
            gap -= 0.015
        # a tip holding 0.35 m: quiet
        rf.reset()
        for i in range(100):
            rf.sense(i * 0.01, 0.35, 0.8, V)
            self.assertIsNone(rf.tip_check(i * 0.01))

    def test_wet_sensor_is_the_last_resort(self):
        rf = self.rf
        rf.sensor = True
        rf.reset()
        rf.sense(0.0, -0.002, 0.0, 12.0)
        rf.reads = [(0.0, 0.5, np.nan)]            # a range reading that does not fire
        self.assertIsNone(rf.tip_check(0.0))
        rf.sense(0.02, -0.004, 0.0, 12.0)
        rf.reads = [(0.02, 0.5, np.nan)]
        self.assertEqual(rf.tip_check(0.02)[0], "wet")


class TestReflexHarness(unittest.TestCase):
    def test_a_constructed_contact_is_rescued_by_the_range_trigger(self):
        from sim.reflex import run_reflex_case
        r = run_reflex_case(("contact", "albatross-3m", "flat", (35.0, 0.3, 1.3, 1.2)))
        self.assertNotEqual(r["results"]["none"]["ended"], "complete")
        self.assertEqual(r["results"]["v2"]["ended"], "complete")
        self.assertIn("range", r["results"]["v2"]["causes"])


if __name__ == "__main__":
    unittest.main()
