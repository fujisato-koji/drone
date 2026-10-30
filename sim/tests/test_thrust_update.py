"""The MPPI thrust update (MPPIPlanner thrust_update): with costs that do not depend on the thrust, the planner
must not ask for thrust. Synthetic test through MPPIPlanner.plan with a rollout that returns random costs."""
import unittest

import numpy as np

from sim.aircraft import G
from sim.aircraft_models import by_name
from sim.mpc import LIB_PKW, MPPIPlanner

AC = by_name("albatross-3m")


class FakeGuide:
    def __init__(self, n):
        self.n = n

    def nominal(self, knots_t):
        return np.stack([np.full(self.n, 0.4), np.zeros(self.n)])

    def nominal_thrust(self, knots_t):
        return np.zeros(self.n)


def run(thrust_update, sampling, trials=600, seed=5):
    pl = MPPIPlanner(AC, motor=True, thrust_sampling=sampling, thrust_update=thrust_update, seed=seed,
                     **{k: v for k, v in LIB_PKW.items()})
    rng = np.random.default_rng(seed + 1)
    N = pl.nsteps

    def fake(world, t0, s0, Us, target_dir):
        S = Us.shape[0]
        return rng.standard_normal(S), np.zeros((N + 1, 6, S)), np.zeros(S, bool)

    pl.rollout = fake
    guide = FakeGuide(pl.n_ctrl)
    signed, applied = [], []
    for _ in range(trials):
        pl.reset()
        plan = pl.plan(None, 0.0, np.zeros(6), np.array([1.0, 0.0]), guide=guide)
        signed.append(float(np.mean(pl.U[2])))
        applied.append(float(np.mean(plan.thrust_knots)))
    w = AC.mass * G
    return np.mean(signed) / w, np.mean(applied) / w


class TestThrustUpdate(unittest.TestCase):
    def test_irrelevant_thrust_averages_zero_with_the_raw_update(self):
        for sampling in ("basic", "sustained"):
            clipped_signed, clipped_applied = run("clipped", sampling)
            raw_signed, raw_applied = run("raw", sampling)
            print("\n%s: nominal thrust mean, clipped %+.2f%% of the weight (applied %.2f%%), raw %+.2f%% (applied %.2f%%)"
                  % (sampling, 100 * clipped_signed, 100 * clipped_applied, 100 * raw_signed, 100 * raw_applied))
            self.assertGreater(clipped_applied, 0.02)               # the old update: a thrust nobody asked for
            # the new nominal averages zero; with sustained sampling the six constant-thrust candidates (5 to 20 %
            # of the weight, 6 of 96 samples) are real samples and add about 0.6 %
            self.assertLess(abs(raw_signed), 0.01)
            # what is applied is the nominal clipped at zero: the spread of the weighted mean about zero still
            # gives a positive mean (about 0.4 sigma), so it is reported, not asserted to vanish
            self.assertLess(raw_applied, 0.04)


if __name__ == "__main__":
    unittest.main()
