"""The velocity-hull navigator (sim.navigator.VelocityHull, HullNavigator, sail_hull) on a synthetic hull
and, when the motor library file has them, on the library's own hull."""
import json
import math
import unittest

import numpy as np

from sim.mpc import LIB_MOTOR_PATH
from sim.aircraft_models import by_name
from sim.navigator import HullNavigator, HullPoint, HullSource, SteadyModel, VelocityHull, sail_hull


def soaring_points():
    """Mean ground velocities of soaring cycles (vx always downwind, like the albatross at 16 m/s)."""
    pts = []
    for k, (vx, vy) in enumerate([(25.0, 8.0), (22.0, 3.0), (14.0, 18.0), (9.0, 20.0)]):
        for side in (1, -1):
            pts.append(HullPoint(vx, side * vy, 0.0, "soaring", "soaring:16:%d:%d" % (k, side), None, side))
    return pts


def motor_points(power=50.0, vx=-6.0):
    return [HullPoint(vx, 0.0, power, "motor", "motor:16:180:%d" % s, None, s) for s in (1, -1)]


def source(points, u10=16.0):
    return HullSource([VelocityHull(points, u10)])


class TestHull(unittest.TestCase):
    def test_support_upwind_is_negative(self):
        h = VelocityHull(soaring_points())
        self.assertLess(h.support((-1.0, 0.0)), 0.0)
        self.assertGreater(h.support((1.0, 0.0)), 20.0)

    def test_pair_on_the_line_has_zero_cross_velocity(self):
        h = VelocityHull(soaring_points())
        d = (math.cos(math.radians(40)), math.sin(math.radians(40)))
        pl = h.plan_pair(d)
        self.assertEqual(pl["kind"], "soaring")
        pts = h.points
        cross = pl["wi"] * (d[0] * pts[pl["i"]].vy - d[1] * pts[pl["i"]].vx) \
            + pl["wj"] * (d[0] * pts[pl["j"]].vy - d[1] * pts[pl["j"]].vx)
        self.assertAlmostEqual(cross, 0.0, places=9)
        self.assertAlmostEqual(pl["wi"] + pl["wj"], 1.0, places=12)
        self.assertGreater(pl["r"], 10.0)

    def test_downwind_is_the_hull_boundary(self):
        h = VelocityHull(soaring_points())
        pl = h.plan_pair((1.0, 0.0))
        self.assertAlmostEqual(pl["r"], 25.0, places=6)      # a cycle and its mirror image

    def test_upwind_and_across_need_the_motor(self):
        h = VelocityHull(soaring_points() + motor_points())
        for d in ((-1.0, 0.0), (0.0, 1.0)):
            self.assertIsNone(h.plan_pair(d, motor_ok=False))
            pl = h.plan_pair(d)
            self.assertEqual(pl["kind"], "motor")
            self.assertGreater(pl["per_m"], 0.0)

    def test_no_progress_without_motor_points(self):
        nav = HullNavigator(source(soaring_points()), arrival_radius=100.0)
        c = nav.step(0.0, (0.0, 0.0), 16.0, 270.0, (-5000.0, 0.0))     # upwind of a west wind
        self.assertEqual(c.mode, "no progress")

    def test_least_energy_per_metre_picks_the_cheaper_motor_cycle(self):
        cheap = [HullPoint(-6.0, 0.0, 30.0, "motor", "motor:16:150:%d" % s, None, s) for s in (1, -1)]
        dear = [HullPoint(-8.0, 0.0, 90.0, "motor", "motor:16:165:%d" % s, None, s) for s in (1, -1)]
        h = VelocityHull(soaring_points() + dear + cheap)
        pl = h.plan_pair((-1.0, 0.0))
        self.assertAlmostEqual(pl["per_m"], 30.0 / 6.0, places=9)
        used = [h.points[k].key[:12] for k, w in ((pl["i"], pl["wi"]), (pl["j"], pl["wj"])) if w > 0]
        self.assertIn("motor:16:150", used)
        self.assertNotIn("motor:16:165", used)


class TestSail(unittest.TestCase):
    def run_case(self, target, wind_from=270.0, extra=(), **kw):
        src = source(soaring_points() + list(extra))
        nav = HullNavigator(src, arrival_radius=100.0, **kw)
        return sail_hull(src, lambda t: (16.0, wind_from), (0.0, 0.0), target, nav=nav, dt=1.0,
                         t_max=6000.0)

    def test_downwind_arrives_near_the_hull_speed(self):
        r = self.run_case((6000.0, 0.0))
        self.assertTrue(r["arrived"])
        self.assertGreater(r["mean_vmg"], 0.6 * 25.0)

    def test_reach_with_soaring_alone_tacks_inside_the_corridor(self):
        # a target 60 deg off the downwind axis (wind from the west, so downwind is east)
        tgt = (6000.0 * math.cos(math.radians(60)), 6000.0 * math.sin(math.radians(60)))
        r = self.run_case(tgt)
        self.assertTrue(r["arrived"])
        self.assertGreaterEqual(r["n_legs"], 1)
        self.assertEqual(r["energy_j"], 0.0)

    def test_min_leg_before_the_funnel(self):
        tgt = (8000.0 * math.cos(math.radians(70)), 8000.0 * math.sin(math.radians(70)))
        r = self.run_case(tgt)
        gaps = np.diff(r["switch_times"])
        far = [g for g, t in zip(gaps, r["switch_times"][1:]) if t < 0.5 * r["time"]]
        self.assertTrue(all(g >= 90.0 for g in far))

    def test_upwind_with_the_motor_arrives_and_uses_energy(self):
        r = self.run_case((-4000.0, 0.0), extra=motor_points())
        self.assertTrue(r["arrived"])
        self.assertGreater(r["energy_j"], 0.0)
        self.assertLess(r["time"], 4000.0 / 6.0 * 1.3)

    def test_across_wind_with_the_motor(self):
        r = self.run_case((0.0, 4000.0), extra=motor_points())
        self.assertTrue(r["arrived"])

    def test_wind_direction_is_respected(self):
        # wind from the south (180): downwind is north
        r = self.run_case((0.0, 5000.0), wind_from=180.0)
        self.assertTrue(r["arrived"])
        self.assertGreater(r["mean_vmg"], 15.0)

    def test_pair_is_kept_inside_the_hysteresis(self):
        src = source(soaring_points())
        nav = HullNavigator(src, arrival_radius=100.0)
        c1 = nav.step(0.0, (0.0, 0.0), 16.0, 270.0, (6000.0, 3000.0))
        pair = nav.pair
        nav.step(5.0, (50.0, 10.0), 16.0, 270.0, (6000.0, 3000.0))
        self.assertEqual(nav.pair, pair)
        self.assertFalse(c1.tacked)

    def test_arrived(self):
        nav = HullNavigator(source(soaring_points()), arrival_radius=100.0)
        c = nav.step(0.0, (10.0, 10.0), 16.0, 270.0, (50.0, 10.0))
        self.assertEqual(c.mode, "arrived")


class TestLibraryHull(unittest.TestCase):
    def setUp(self):
        try:
            with open(LIB_MOTOR_PATH) as f:
                self.entries = json.load(f)["entries"]
        except OSError:
            self.skipTest("no motor library")

    def test_soaring_cannot_go_upwind_at_the_soaring_winds(self):
        for name in ("albatross-3m", "baseline-2m", "sealedhull-2p5m"):
            src = HullSource.from_entries(self.entries, name)
            for u in (13.0, 16.0):
                h = src.at(u)
                self.assertEqual(h.u10, u)
                self.assertLess(h.support((-1.0, 0.0)), 0.0, (name, u))

    def test_entries_have_a_velocity(self):
        src = HullSource.from_entries(self.entries, "albatross-3m")
        self.assertTrue(all(len(h.points) % 2 == 0 for h in src.hulls))


class TestExport(unittest.TestCase):
    def test_viewer_library_has_the_hull_velocities(self):
        """library_motor.js carries vx and vy of every cycle, equal to the library's (to rounding)."""
        from pathlib import Path
        js = Path(__file__).resolve().parents[1] / "viewer" / "library_motor.js"
        if not js.exists():
            self.skipTest("no library_motor.js")
        lib = json.loads(js.read_text(encoding="utf-8").split("window.LIBRARY_MOTOR = ", 1)[1].rsplit(";", 1)[0])
        with open(LIB_MOTOR_PATH) as f:
            ents = {(e["aircraft"], e["u10"], e["chi_deg"], e["kind"]): e for e in json.load(f)["entries"] if e["success"]}
        n = 0
        for name, es in lib["aircraft"].items():
            for e in es:
                p = ents[(name, e["u10"], e["chi_deg"], e["kind"])]
                vx = (p["x"][-1] - p["x"][0]) / p["period"]
                self.assertAlmostEqual(e["vx"], vx, delta=1e-4 * max(1.0, abs(vx)))
                n += 1
        self.assertEqual(n, len(ents))


class TestCalmCourse(unittest.TestCase):
    def test_calm_entries_fly_along_their_course(self):
        from sim.mpc import calm_course_errors
        with open(LIB_MOTOR_PATH) as f:
            ents = json.load(f)["entries"]
        self.assertEqual(calm_course_errors(ents, 5.0), [])
        self.assertTrue(any(e["u10"] == 0.0 and e["success"] and e["chi_deg"] >= 120 for e in ents))

    def test_rotation_keeps_the_cycle(self):
        from sim.mpc import course_error_deg, rotate_calm_entry
        e = dict(x=[0.0, 3.0, 6.0], y=[0.0, 4.0, 8.0], psi=[0.0, 0.1, 0.0], period=2.0, chi_deg=0.0,
                 u10=0.0, success=True)
        r = rotate_calm_entry(e, 120.0)
        self.assertLess(abs(course_error_deg(r)), 1e-4)
        self.assertAlmostEqual(math.hypot(r["x"][-1], r["y"][-1]), 10.0, places=3)


class TestSteadyLeg(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.ac = by_name("albatross-3m")
        cls.model = SteadyModel.from_aircraft(cls.ac)

    def test_level_flight_balances_weight_and_drag(self):
        cl, T, P = self.model.level(20.0)
        q = 0.5 * self.ac.rho * 400.0 * self.ac.wing_area
        self.assertAlmostEqual(q * cl, self.ac.mass * self.model.g, places=6)
        self.assertAlmostEqual(T, q * float(self.ac.cd(cl)), places=9)
        self.assertAlmostEqual(P, T * 20.0 / self.model.eta, places=9)

    def test_airspeed_is_the_cheapest_energy_per_metre(self):
        lg = self.model.leg(16.0, (-1.0, 0.0))                    # straight into a 16 m/s wind
        for V in np.arange(14.5, 40.0, 0.25):
            cl, T, P = self.model.level(V)
            along = V - 16.0
            if T <= 0.8 * self.model.t_max and along >= 0.3 and V >= 1.2 * self.model.v_stall:
                self.assertGreaterEqual(P / along, lg["per_m"] - 1e-9)
        self.assertGreater(lg["along"], 5.0)

    def test_no_leg_when_the_wind_beats_the_aircraft(self):
        self.assertIsNone(self.model.leg(60.0, (-1.0, 0.0)))

    def test_cross_track_correction_turns_towards_the_line_and_is_limited(self):
        dl = (-1.0, 0.0)
        t0 = self.model.track(dl, 0.0)
        self.assertAlmostEqual(t0[1], 0.0, places=12)
        tl = self.model.track(dl, 200.0)          # left of the line: turn right (towards +y when going -x)
        self.assertGreater(tl[1], 0.0)
        far = self.model.track(dl, 1e6)
        self.assertAlmostEqual(math.degrees(math.atan2(far[1], -far[0])), 35.0, places=6)

    def test_navigator_uses_the_motor_only_when_soaring_cannot(self):
        src = source(soaring_points())
        nav = HullNavigator(src, arrival_radius=200.0, steady=self.model)
        up = nav.step(0.0, (0.0, 0.0), 16.0, 270.0, (-5000.0, 0.0))
        self.assertEqual((up.kind, up.point.key), ("motor", "steady"))
        down = nav.step(0.0, (0.0, 0.0), 16.0, 270.0, (5000.0, 0.0))
        self.assertEqual(down.kind, "soaring")

    def test_kinematic_upwind_and_across_with_the_steady_leg(self):
        src = source(soaring_points())
        for tgt, lo in (((-5000.0, 0.0), 6.0), ((0.0, 5000.0), 9.0)):
            nav = HullNavigator(src, arrival_radius=200.0, steady=self.model)
            r = sail_hull(src, lambda t: (16.0, 270.0), (0.0, 0.0), tgt, nav=nav, dt=1.0, t_max=4000.0)
            self.assertTrue(r["arrived"], tgt)
            self.assertGreater(r["mean_vmg"], lo)
            self.assertEqual(r["n_legs"], 0)

    def test_steady_leg_follows_a_cross_wind_line(self):
        """Pushed off the line, the leg turns back to it (kinematic run with a drifting start)."""
        src = source(soaring_points())
        nav = HullNavigator(src, arrival_radius=200.0, steady=self.model)
        c0 = nav.step(0.0, (0.0, 0.0), 16.0, 270.0, (-5000.0, 0.0))
        nav.step(0.0, (0.0, 0.0), 16.0, 270.0, (-5000.0, 0.0))
        left = nav.step(5.0, (-50.0, 300.0), 16.0, 270.0, (-5000.0, 0.0))       # north of the line (wind from west)
        self.assertLess(left.point.steady["heading"][1], c0.point.steady["heading"][1] - 0.05)      # turns south, back to the line


if __name__ == "__main__":
    unittest.main()
