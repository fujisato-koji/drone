import math
import unittest

from sim.aircraft_models import by_name, candidates, parametric
from sim.formstudy import static_numbers


class TestModels(unittest.TestCase):
    def test_candidates(self):
        cs = candidates()
        self.assertEqual(len(cs), 7)
        self.assertEqual(len({c.name for c in cs}), 7)
        for c in cs:
            self.assertTrue(6 <= c.aspect_ratio <= 30, c.name)
            self.assertTrue(8 <= c.best_ld <= 40, c.name)
            self.assertTrue(5 <= c.v_stall <= 20, c.name)
            self.assertIs(by_name(c.name), c)

    def test_parametric(self):
        self.assertAlmostEqual(parametric(2.0, 16, 1.5).wing_area, 0.25, delta=1e-9)

    def test_static_numbers_finite(self):
        for c in candidates():
            for k, v in static_numbers(c).items():
                self.assertTrue(math.isfinite(v), (c.name, k))


if __name__ == "__main__":
    unittest.main()
