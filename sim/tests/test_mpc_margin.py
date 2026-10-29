import json
import os
import unittest

import numpy as np

from sim.aircraft_models import by_name
from sim.dsopt import dyn, profile_for
from sim.mpc import LIB_MARGIN_PATH, MARGIN_SETS, Cycle, CycleLibrary

G = 9.80665


def replay(e, ac, sub=20):
    """Dense replay: integrate (RK4) each interval from its node state with the piecewise-linear
    node controls; return min height, min geometric wingtip height, and the node arrays."""
    a = lambda k: np.asarray(e[k], float)
    S = np.stack([a("x"), a("y"), a("z"), a("V"), a("gamma"), a("psi")], axis=1)
    C = np.stack([a("cl"), a("mu")], axis=1)
    T = e["period"]
    prof = profile_for(e["u10"], "log", roughness="coare35")
    dt = T / (len(S) - 1)
    f = lambda s, c: dyn(s, c, ac, prof, e["u10"])
    s = S[:-1].copy()
    hmin, tipmin = np.inf, np.inf
    for j in range(sub + 1):
        fr = j / sub
        c = C[:-1] + (C[1:] - C[:-1]) * fr
        hmin = min(hmin, s[:, 2].min())
        tipmin = min(tipmin, (s[:, 2] - 0.5 * ac.span * np.sin(np.abs(c[:, 1]))
                              * np.cos(s[:, 4])).min())
        if j == sub:
            break
        h = dt / sub
        cm = C[:-1] + (C[1:] - C[:-1]) * (fr + 0.5 / sub)
        cn = C[:-1] + (C[1:] - C[:-1]) * (fr + 1.0 / sub)
        k1 = f(s, c)
        k2 = f(s + 0.5 * h * k1, cm)
        k3 = f(s + 0.5 * h * k2, cm)
        k4 = f(s + h * k3, cn)
        s = s + h / 6.0 * (k1 + 2 * k2 + 2 * k3 + k4)
    return hmin, tipmin, S, C, dt


class TestMarginLibrary(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        with open(LIB_MARGIN_PATH) as fh:
            cls.data = json.load(fh)

    def test_loads_and_covers_both_aircraft(self):
        lib = CycleLibrary.load(LIB_MARGIN_PATH)
        self.assertEqual(len(lib.entries), sum(1 for e in self.data["entries"] if e["success"]))
        self.assertIn("commit", self.data["meta"])
        for name in ("baseline-2m", "albatross-3m"):
            for u10 in (13.0, 16.0):
                self.assertIsInstance(lib.pick(name, u10, np.radians(20.0)), Cycle)
        same = CycleLibrary.from_keyword("margin")
        self.assertEqual(len(same.entries), len(lib.entries))

    def test_every_entry_has_its_margin_set(self):
        sets = dict(MARGIN_SETS)
        for e in self.data["entries"]:
            if not e["success"]:
                self.assertTrue(e.get("missing"))
                continue
            m = e["margins"]
            self.assertEqual((m["h_min"], m["n_frac"], m["mu_frac"], m["v_factor"]),
                             sets[e["margin_set"]])

    def test_entries_respect_their_margins_on_a_dense_replay(self):
        n_ok = 0
        for e in self.data["entries"]:
            if not e["success"]:
                continue
            ac = by_name(e["aircraft"])
            m = e["margins"]
            hmin, tipmin, S, C, dt = replay(e, ac)
            tag = "%s u10 %g chi %g (%s)" % (e["aircraft"], e["u10"], e["chi_deg"],
                                             e["margin_set"])
            self.assertGreaterEqual(hmin, m["h_min"] - 0.05, tag)
            self.assertGreaterEqual(tipmin, m["h_tip_min"] - 0.05, tag)
            K = 0.5 * ac.rho * ac.wing_area / (ac.mass * G)
            n = K * S[:, 3] ** 2 * C[:, 0]
            self.assertLessEqual(n.max(), m["n_frac"] * ac.n_max * 1.01, tag)
            self.assertGreaterEqual(n.min(), m["n_min"] - 0.02, tag)
            self.assertLessEqual(np.abs(C[:, 1]).max(), m["mu_frac"] * ac.mu_max * 1.01, tag)
            self.assertGreaterEqual(S[:, 3].min(), m["v_factor"] * ac.v_stall * 0.99, tag)
            Vm = np.minimum(S[1:, 3], S[:-1, 3])
            lim = 2.0 * m["k_roll"] * Vm / ac.span
            self.assertLessEqual((np.abs(np.diff(C[:, 1])) / dt / lim).max(), 1.02, tag)
            self.assertLessEqual(np.abs(np.diff(C[:, 0])).max() / dt,
                                 m["cl_rate_max_per_s"] * 1.02, tag)
            n_ok += 1
        self.assertGreater(n_ok, 0)


if __name__ == "__main__":
    unittest.main()
