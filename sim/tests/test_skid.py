import unittest

import numpy as np

from sim import skid as s


class TestSkidForces(unittest.TestCase):
    def test_blade_grows_with_depth_and_aligns(self):
        a, b = s.blade(0.01, 20.0), s.blade(0.02, 20.0)
        self.assertGreater(b["drag"], a["drag"])
        self.assertEqual(a["side"], 0.0)                      # aligned with the flow
        self.assertGreater(s.blade(0.01, 20.0, beta_deg=5.0)["side"], 0.0)
        self.assertEqual(s.blade(0.0, 20.0)["drag"], 0.0)
        self.assertEqual(s.blade(0.5, 20.0)["area"], s.blade(0.04, 20.0)["area"])   # capped at the blade's depth

    def test_ski_matches_savitsky_at_high_speed(self):
        f = s.ski(0.01, 25.0, beam=0.02, length=0.06, trim_deg=4.0)
        lam = 3.0
        cl = 4.0 ** 1.1 * 0.0120 * lam ** 0.5   # the Cv term is nil at this Froude number
        self.assertAlmostEqual(f["normal"], cl * s.q_water(25.0) * 0.02 ** 2, 1)
        self.assertAlmostEqual(f["lam"], lam, 9)

    def test_cf_ittc(self):
        self.assertAlmostEqual(s.cf_ittc(1e6), 0.075 / 16.0, 12)


class TestDynamics(unittest.TestCase):
    def test_roll_damping_rectangle(self):
        clp, a = s.roll_damping(10.0, taper=1.0)
        self.assertAlmostEqual(clp, -a / 6.0, 4)

    def test_inertia_uniform_bar(self):
        i = s.inertia(1.2, 3.0, taper=1.0)
        self.assertAlmostEqual(i["ixx"], 1.2 * 3.0 ** 2 / 12.0, 4)
        self.assertGreater(i["izz"], i["ixx"])

    def test_no_reflex_goes_as_deep_as_the_rise(self):
        r = s.reflex_budget(60.0, 20.0, 3.0, 3.0, 0.1, 1.0, None)
        self.assertAlmostEqual(r["max_depth_m"], 0.1, 6)
        self.assertGreater(r["wet_s"], 1.0)   # the sea stays up and nothing lifts the tip

    def test_earlier_reflex_is_shallower(self):
        d = [s.reflex_budget(60.0, 20.0, 3.0, 3.0, 0.1, 1.0, dl)["max_depth_m"] for dl in (-0.05, 0.0, 0.05, 0.1)]
        self.assertTrue(all(x <= y + 1e-12 for x, y in zip(d, d[1:])))
        ls = s.latest_start(60.0, 20.0, 3.0, 3.0, 0.1, 1.0, 0.02, 0.06)
        r = s.reflex_budget(60.0, 20.0, 3.0, 3.0, 0.1, 1.0, ls["start_s"])
        self.assertLessEqual(r["max_depth_m"], 0.02 + 1e-6)
        self.assertAlmostEqual(ls["trigger_gap_m"], max(0.0, -1.0 * (ls["start_s"] - 0.06)), 12)


class TestArcAndChecks(unittest.TestCase):
    """K-018: the skim arc, the dihedral roll and the skimmer check."""

    def test_dihedral_clbeta_strip_theory(self):
        cb = s.dihedral_clbeta(20.0, 10.0, taper=0.45)
        a = s.roll_damping(20.0, 0.45)[1]
        self.assertAlmostEqual(cb, -a * np.radians(10.0) / 6.0 * 1.9 / 1.45, 12)
        self.assertEqual(s.dihedral_clbeta(20.0, 0.0), 0.0)

    def test_skimmer_check_uses_the_record(self):
        from sim.record import value
        d = value("lit.skimmer_bill_immersion_m")
        f = s.skimmer_check()
        self.assertAlmostEqual(f["drag"], s.blade(d, 10.0, t=value("lit.skimmer_lower_beak_thickness_max_mm") * 1e-3,
                                                  rake_deg=value("skid.blade_rake_deg"), depth=d)["drag"], 12)
        self.assertAlmostEqual(s.depth_froude(6.8, 0.04), 6.8 / np.sqrt(9.80665 * 0.04), 2)

    def test_arc_of_a_skid_touch_cycle(self):
        import json
        rows = json.load(open(s.SKIM_PATH))["rows"]
        r = next(r for r in rows if r["aircraft"] == "albatross-3m" and not r.get("lever") and r["h_tip_min"] == 0.0
                 and r["h_min"] == 1.0)
        a = s.arc_stats(r)
        self.assertGreater(a["arc_s"], 0.3)
        self.assertLess(a["bank_deg"][1], 62.0 + 0.5)            # the bank limit of the margins
        self.assertLess(a["bank_deg"][0], a["bank_deg"][1])
        self.assertGreater(a["crab_deg"][0], 5.0)                # the tip does not move along the heading
        self.assertGreater(a["water_speed_ms"][1], a["water_speed_ms"][0])
        self.assertGreater(a["bend_lift_m"][0], 0.0)


if __name__ == "__main__":
    unittest.main()
