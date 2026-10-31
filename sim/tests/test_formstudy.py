import unittest

import numpy as np

from sim import formstudy as f
from sim.record import value


class TestSkimStudy(unittest.TestCase):
    """formstudy.skim_study pieces (D-018, docs/02-form.md#wingtip-skim)."""

    def test_aircraft(self):
        from sim import wingopt as w
        mk, d, re = f.skim_aircraft("ar20-sd7037-limit")
        self.assertAlmostEqual(w.basics(d)[0]["mass"], value("ds.mass_max_kg") - 1e-3, 9)
        self.assertGreater(re, 5e4)
        self.assertAlmostEqual(mk(1.5e5).mass, w.basics(d)[0]["mass"], 9)
        _, d0, _ = f.skim_aircraft("ar20-sd7037-none")
        self.assertEqual(d0.ballast, 0.0)
        mk, d, re = f.skim_aircraft("albatross-3m")
        self.assertIsNone(d)
        self.assertEqual(mk(None).name, "albatross-3m")
        with self.assertRaises(ValueError):
            f.skim_aircraft("nothing")

    def test_one_cell_holds_its_clearance(self):
        mk, d, re = f.skim_aircraft("albatross-3m")
        row, x = f._skim_solve(mk, d, re, 2.0, 0.3, None, [], seeds=(0, 1))
        self.assertTrue(row["success"])
        self.assertGreaterEqual(row["tip_min_m"], 0.3 - 1e-6)
        self.assertGreaterEqual(row["rc_tip_min"], 0.3 - 0.02)
        self.assertGreaterEqual(row["z_min"], 2.0 - 1e-6)
        self.assertLessEqual(row["bank_max_deg"], 0.9 * np.degrees(mk(None).mu_max) + 1e-6)
        self.assertGreater(row["water_speed_at_tip_ms"], 0.0)
        # a warm start from the solution reproduces it
        row2, _ = f._skim_solve(mk, d, re, 2.0, 0.3, None, [("same", x)], seeds=())
        self.assertTrue(row2["success"])
        self.assertAlmostEqual(row2["u10"], row["u10"], 2)


class TestSkimUpwind(unittest.TestCase):
    def test_start_reynolds_number(self):
        from sim import wingopt as w
        mk, d, re = f.skim_aircraft("ar20-sd7037-limit")
        self.assertAlmostEqual(f._skim_re("ar20-sd7037-limit", path="no such file"), re, 6)
        self.assertGreater(re, 0.0)
        self.assertGreater(w.re_of(1.0, 0.1), 0.0)

    def test_one_course_holds_its_margins(self):
        from sim.dsopt import profile_for
        ac = f.skim_aircraft("albatross-3m")[0](None)
        best, tried = f._skim_course(ac, profile_for(16.0, "log", roughness="coare35"), 16.0, 90.0, 2.0, 0.3,
                                     None, [], seeds=(0, 1))
        self.assertIsNotNone(best)
        info, x, col = best[1]
        self.assertGreater(info["speed_along_chi"], 0.0)   # across the wind soaring makes progress
        self.assertGreaterEqual(info["z_min"], 2.0 - 1e-6)
        self.assertGreaterEqual(info["rc_tip_min"], 0.3 - 0.02)
        self.assertEqual(set(tried), {"cold"})


if __name__ == "__main__":
    unittest.main()
