import unittest

import numpy as np

from sim import polar


class TestPolar(unittest.TestCase):
    def test_one_case_runs(self):
        info, x = polar.solve_case("baseline-2m", 20.0, 90.0, polar.MS_STANDARD)
        self.assertTrue(info["success"])
        self.assertGreater(info["speed_along_chi"], 0.0)
        e = polar._entry(info, "standard", polar.MS_STANDARD, 90.0, 1)
        self.assertGreater(e["period"], 0.0)

    def test_progress_is_not_scored_as_a_vmg(self):
        """The progress is a projection (the cross velocity is free), so no speed made good towards a direction is
        derived from it: the entry has no vmg fields and the module has no tacking()."""
        self.assertFalse(hasattr(polar, "tacking"))
        info, x = polar.solve_case("baseline-2m", 20.0, 90.0, polar.MS_STANDARD)
        e = polar._entry(info, "standard", polar.MS_STANDARD, 90.0, 1)
        self.assertNotIn("vmg_upwind", e)
        self.assertNotIn("vmg_downwind", e)


if __name__ == "__main__":
    unittest.main()
