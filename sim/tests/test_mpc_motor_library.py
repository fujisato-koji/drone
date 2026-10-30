"""The motor library (sim/out/mpc_library_motor.json): every entry replays within its margins, with
the node thrust in the dynamics, as test_mpc_margin does for the margin library."""
import json
import unittest

import numpy as np

from sim.aircraft_models import by_name
from sim.dsopt import dyn, profile_for, thrust_max
from sim.mpc import KEPT_AIRCRAFT, LIB_CHI_DEG, LIB_MOTOR_PATH, MARGIN_SETS, NAV_CHI_DEG, Cycle, CycleLibrary

G = 9.80665


def replay(e, ac, sub=20):
    """Dense replay (RK4 from each node with the piecewise-linear controls and thrust): lowest
    height, lowest geometric wingtip height, node arrays."""
    a = lambda k: np.asarray(e[k], float)
    S = np.stack([a("x"), a("y"), a("z"), a("V"), a("gamma"), a("psi")], axis=1)
    C = np.stack([a("cl"), a("mu")], axis=1)
    Th = a("thrust")
    T = e["period"]
    prof = profile_for(max(e["u10"], 1.0), "log", roughness="coare35")
    dt = T / (len(S) - 1)
    f = lambda s, c, th: dyn(s, c, ac, prof, e["u10"], False, th)
    s = S[:-1].copy()
    hmin, tipmin = np.inf, np.inf
    for j in range(sub + 1):
        fr = j / sub
        c = C[:-1] + (C[1:] - C[:-1]) * fr
        hmin = min(hmin, s[:, 2].min())
        tipmin = min(tipmin, (s[:, 2] - 0.5 * ac.span * np.sin(np.abs(c[:, 1])) * np.cos(s[:, 4])).min())
        if j == sub:
            break
        h = dt / sub
        cm = C[:-1] + (C[1:] - C[:-1]) * (fr + 0.5 / sub)
        cn = C[:-1] + (C[1:] - C[:-1]) * (fr + 1.0 / sub)
        t0, tm, tn = (Th[:-1] + (Th[1:] - Th[:-1]) * q for q in (fr, fr + 0.5 / sub, fr + 1.0 / sub))
        k1 = f(s, c, t0)
        k2 = f(s + 0.5 * h * k1, cm, tm)
        k3 = f(s + 0.5 * h * k2, cm, tm)
        k4 = f(s + h * k3, cn, tn)
        s = s + h / 6.0 * (k1 + 2 * k2 + 2 * k3 + k4)
    return hmin, tipmin, S, C, dt


class TestMotorLibrary(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        with open(LIB_MOTOR_PATH) as fh:
            cls.data = json.load(fh)
        cls.ok = [e for e in cls.data["entries"] if e["success"]]

    def test_covers_every_kept_aircraft_with_both_kinds_of_cycle(self):
        lib = CycleLibrary.load(LIB_MOTOR_PATH)
        self.assertEqual(len(lib.entries), len(self.ok))
        self.assertIn("commit", self.data["meta"])
        for name in KEPT_AIRCRAFT:
            es = [e for e in self.ok if e["aircraft"] == name]
            self.assertTrue(any(e["kind"] == "soaring" for e in es), name)
            self.assertTrue(any(e["kind"] == "motor" for e in es), name)
            self.assertIsInstance(lib.pick(name, 9.0, np.radians(20.0)), Cycle)
        self.assertNotIn("flyingwing-2m", {e["aircraft"] for e in self.ok})
        self.assertEqual(len(CycleLibrary.from_keyword("motor").entries), len(self.ok))

    def test_thrust_schedules(self):
        for e in self.ok:
            th = np.asarray(e["thrust"])
            ac = by_name(e["aircraft"])
            self.assertEqual(len(th), len(e["t"]))
            self.assertGreaterEqual(th.min(), -1e-6)
            self.assertLessEqual(th.max(), thrust_max(ac) * 1.001)
            if e["kind"] == "soaring":
                self.assertEqual(float(th.max()), 0.0)
                self.assertEqual(e["power_avg_w"], 0.0)
            else:
                self.assertGreater(e["power_avg_w"], 0.0)
                self.assertGreater(e["speed_along_chi"], 0.5 * ac.v_best_ld * 0.99)
        chis = {c for e in self.ok for c in [e["chi_deg"]]}
        # the build's courses, the upwind-side courses of extend_motor_library and straight upwind (180)
        self.assertTrue(chis <= set(LIB_CHI_DEG) | set(NAV_CHI_DEG) | {180.0})

    def test_entries_respect_their_margins_on_a_dense_replay(self):
        sets = dict(MARGIN_SETS)
        n_motor = 0
        for e in self.ok:
            ac = by_name(e["aircraft"])
            m = e["margins"]
            want = sets[e["margin_set"]]
            # the straight upwind motor cycles are solved with a raised minimum height (extend_motor_upwind)
            self.assertEqual((m["n_frac"], m["mu_frac"], m["v_factor"]), want[1:])
            self.assertGreaterEqual(m["h_min"], want[0])
            if m["h_min"] > want[0]:
                self.assertEqual(e["chi_deg"], 180.0)
            hmin, tipmin, S, C, dt = replay(e, ac)
            tag = "%s %s u10 %g chi %g (%s)" % (e["aircraft"], e["kind"], e["u10"], e["chi_deg"], e["margin_set"])
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
            self.assertLessEqual(np.abs(np.diff(C[:, 0])).max() / dt, m["cl_rate_max_per_s"] * 1.02, tag)
            # the cycle closes on the dense replay (the optimizer's own check, K-011): the end of each
            # interval within 0.05 m in height and 1 % in airspeed of the next node
            if e["kind"] == "motor":
                n_motor += 1
                self.assertLess(e["replay"]["replay_closure"], 0.2, tag)
        self.assertGreater(n_motor, 0)


if __name__ == "__main__":
    unittest.main()
