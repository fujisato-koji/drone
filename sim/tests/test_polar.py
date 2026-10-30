import unittest

import numpy as np

from sim import polar


class TestPolar(unittest.TestCase):
    def test_one_case_runs(self):
        info, x = polar.solve_case("baseline-2m", 20.0, 90.0, polar.MS_STANDARD)
        self.assertTrue(info["success"])
        self.assertGreater(info["speed_along_chi"], 0.0)
        e = polar._entry(info, "standard", polar.MS_STANDARD, 90.0, 1)
        self.assertAlmostEqual(e["vmg_upwind"], 0.0, places=6)
        self.assertGreater(e["period"], 0.0)

    def test_vmg_tacking_angle(self):
        # synthetic polar: progress = 10 sin(chi/2) type shape peaking at a known angle
        def case(chi, p):
            return {"chi_deg": float(chi), "converged": True, "progress": p}
        cases = [case(c, 10.0 * np.sin(np.radians(c)) ** 0.5) for c in range(0, 181, 15)]
        cases[0] = case(0, 3.0)          # downwind speed 3 m/s -> VMG 3
        cases[-1] = {"chi_deg": 180.0, "converged": False}
        cases.insert(5, case(75, 1.0))   # does not beat the 75 deg entry already there
        b = polar.tacking(cases)
        # upwind VMG = p cos(180 - chi) is positive only for chi > 90; best near 105 or 120
        ups = {c["chi_deg"]: c["progress"] * np.cos(np.radians(180 - c["chi_deg"]))
               for c in cases if c["converged"] and c["chi_deg"] > 90}
        self.assertEqual(b["upwind"]["course_deg"], max(ups, key=ups.get))
        self.assertAlmostEqual(b["upwind"]["vmg"], max(ups.values()))
        downs = {c["chi_deg"]: c["progress"] * np.cos(np.radians(c["chi_deg"]))
                 for c in cases if c["converged"] and c["chi_deg"] < 90}
        self.assertEqual(b["downwind"]["course_deg"], max(downs, key=downs.get))

    def test_no_upwind_progress(self):
        cases = [{"chi_deg": 0.0, "converged": True, "progress": 5.0},
                 {"chi_deg": 180.0, "converged": False}]
        self.assertIsNone(polar.tacking(cases)["upwind"])


if __name__ == "__main__":
    unittest.main()
