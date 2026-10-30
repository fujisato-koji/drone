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
        self.assertEqual(r["ended"], "arrived", r["nav"]["end_distance_m"])
        self.assertGreater(r["nav"]["mean_vmg_ms"], 6.0)               # the hull: 8.9 m/s at 16 m/s wind
        self.assertEqual(set(r["nav"]["kind_time_s"]), {"motor"})
        self.assertGreater(r["min_clear_surface"], 5.0)                 # flown at 10 m

    def test_across_the_wind_on_a_steady_motor_leg(self):
        r = self.fly("across", 1500.0, 200.0)
        self.assertEqual(r["ended"], "arrived")
        self.assertGreater(r["nav"]["mean_vmg_ms"], 9.0)

    def test_soaring_leg_leaves_the_propeller_folded_most_of_the_time(self):
        """The thrust update uses the unclipped perturbations; with the clipped ones the motor ran all the
        time on a cycle that sustains itself (share 0.99 of the time over the 1 % of the weight)."""
        r = self.fly("downwind", 1500.0, 100.0)
        self.assertEqual(r["ended"], "arrived")
        self.assertLess(r["motor"]["thrust_on_share"], 0.5)


if __name__ == "__main__":
    unittest.main()
