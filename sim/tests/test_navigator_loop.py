"""Closed loops of fly_mpc with the hull navigator (flat COARE 3.5 sea, truth state and wind, motor on at the
middle weight, motor library). Each flight takes a few tens of seconds of wall time."""
import os
import unittest

from sim.aircraft import Actuators
from sim.aircraft_models import by_name
from sim.mpc import (LIB_MOTOR_PATH, NAV_GUIDE_SIG, NAV_PKW, CycleLibrary, energy_weight, fly_mpc, nav_target,
                     _make_env_coare)
from sim.navigator import HullSource, SteadyModel


@unittest.skipUnless(os.path.exists(LIB_MOTOR_PATH), "no motor library")
class TestNavigatorLoop(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.lib = CycleLibrary.from_keyword("motor")

    def fly(self, direction, dist, t_end, u10=16.0, planner_kw=None, name="albatross-3m"):
        ac = by_name(name)
        env = _make_env_coare("flat", u10)
        src = HullSource.from_entries(self.lib.entries, name)
        return fly_mpc(ac, env, "flat", [], t_end, 0.0, u10, seed=1, planner_kw=dict(planner_kw or NAV_PKW),
                       library=self.lib, actuators=Actuators(ac, rate_model="pb2v"), n_limit=True,
                       stop_on_tip=True, wave_aware=False, motor=True, w_energy=energy_weight(ac, "normal"),
                       target=nav_target(direction, dist), navigator=True, nav_hull=src,
                       guide_sig=NAV_GUIDE_SIG, nav_kw={"steady": SteadyModel.from_aircraft(ac)})

    def test_upwind_on_a_steady_motor_leg(self):
        r = self.fly("upwind", 1000.0, 200.0)
        nv = r["nav"]
        self.assertTrue(nv["counts_as_arrived"], nv["end_distance_m"])
        self.assertGreater(nv["mean_vmg_ms"], 6.5)                      # the hull: 8.9 m/s at 16 m/s wind
        self.assertEqual(set(nv["kind_time_s"]), {"motor"})
        self.assertLess(abs(nv["height_median_m"] - 10.0), 2.0)         # the powered leg flies at its reference height
        self.assertLess(nv["height_max_m"], 30.0)

    def test_across_the_wind_on_a_steady_motor_leg(self):
        r = self.fly("across", 1500.0, 200.0)
        self.assertTrue(r["nav"]["counts_as_arrived"])
        self.assertGreater(r["nav"]["mean_vmg_ms"], 9.0)
        self.assertLess(abs(r["nav"]["height_median_m"] - 10.0), 2.0)

    def test_soaring_leg_leaves_the_propeller_folded_most_of_the_time(self):
        """The thrust update uses the unclipped perturbations; with the clipped ones the motor ran all the
        time on a cycle that sustains itself (share 0.99 of the time over the 1 % of the weight)."""
        r = self.fly("downwind", 1500.0, 100.0)
        self.assertTrue(r["nav"]["counts_as_arrived"])
        self.assertLess(r["motor"]["thrust_on_share"], 0.5)

    def test_downwind_arrives_without_a_loop_a_stall_or_a_margin_violation(self):
        """The downwind final approach used to change from a soaring cycle to a steady motor leg: two loops,
        3.9 m/s against a stall speed of 10.1 m/s for 10.5 s, a wingtip at 0.18 m, and it was logged as arrived."""
        r = self.fly("downwind", 5000.0, 400.0)
        nv = r["nav"]
        self.assertTrue(nv["arrived"])
        self.assertEqual(nv["loop_s"], 0.0)
        self.assertEqual(nv["below_stall_s"], 0.0)
        self.assertGreaterEqual(nv["airspeed_min_ms"], nv["stall_speed_ms"])
        self.assertGreaterEqual(nv["wingtip_min_m"], 0.3)
        self.assertGreaterEqual(nv["height_min_m"], 1.0)           # ds.clearance_min_m
        self.assertLessEqual(nv["height_max_m"], 30.0)             # ds.altitude_band_max_m
        self.assertTrue(nv["counts_as_arrived"])
        self.assertEqual(set(nv["kind_time_s"]), {"soaring"})       # no change of kind


if __name__ == "__main__":
    unittest.main()
