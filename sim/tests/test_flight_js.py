"""The viewer's flight.js against sim.aircraft, and its tracker on a flat log wind (needs node)."""
import json
import shutil
import subprocess
import unittest
from pathlib import Path

import numpy as np

from sim.aircraft import BASELINE, clip_controls, derivatives, rk4_step
from sim.seawind import Environment, Wind

FLIGHT_JS = Path(__file__).resolve().parents[1] / "viewer" / "flight.js"
NODE = shutil.which("node")

# Node snippet: reads one JSON object on stdin, writes one JSON object on stdout.
SNIPPET = r"""
const Flight = require(process.argv[1]);
const inp = JSON.parse(require('fs').readFileSync(0, 'utf8'));
const ac = inp.ac, K = 0.4;
// flat log wind with the same constants as sim.seawind.Wind
const windFn = (x, y, z, t) => {
  const sp = z > inp.z0 ? (inp.ustar / K) * Math.log(z / inp.z0) : 0;
  return [sp * inp.ex, sp * inp.ey, 0];
};
windFn.z0 = inp.z0;
const surfaceFn = () => 0;
if (inp.run === 'integrate') {
  let s = inp.s0.slice();
  const dt = 0.01, n = 1000;
  for (let i = 0; i < n; i++) s = Flight.rk4(s, 0.8, i < 500 ? 0.4 : -0.4, ac, windFn, i * dt, dt);
  console.log(JSON.stringify({ s: s }));
} else {
  const tr = new Flight.Tracker(inp.ref, ac);
  let s = inp.s0.slice(), t = 0, hmin = Infinity, guardSteps = 0, cl = 0;
  const dt = 0.01, n = Math.round(inp.T / dt);
  let nan = false;
  for (let i = 0; i < n; i++) {
    const c = tr.command(s, windFn, surfaceFn, t);
    guardSteps += c.guard ? 1 : 0;
    const clp = Math.min(Math.max(c.cl, ac.cl_min), ac.cl_max);
    s = Flight.rk4(s, clp, c.mu, ac, windFn, t, dt);
    t += dt;
    const m = Flight.metrics(s, ac, windFn, surfaceFn, t, clp);
    hmin = Math.min(hmin, m.height);
    if (!s.every(Number.isFinite)) { nan = true; break; }
  }
  console.log(JSON.stringify({ hmin: hmin, guardSteps: guardSteps, nan: nan, passes: tr.passes.length,
    s: s }));
}
"""


def run_node(payload):
    r = subprocess.run([NODE, "-e", SNIPPET, str(FLIGHT_JS)], input=json.dumps(payload),
                       capture_output=True, text=True, timeout=300)
    if r.returncode != 0:
        raise RuntimeError(r.stderr)
    return json.loads(r.stdout)


def ac_dict(ac):
    return {"mass": ac.mass, "span": ac.span, "wing_area": ac.wing_area, "cd0": ac.cd0,
            "oswald": ac.oswald, "cl_max": ac.cl_max, "cl_min": ac.cl_min, "mu_max": ac.mu_max,
            "n_max": ac.n_max, "rho": ac.rho, "k_induced": ac.k_induced, "v_stall": ac.v_stall}


@unittest.skipUnless(NODE, "node is not on PATH: flight.js tests skipped")
class TestFlightJs(unittest.TestCase):
    def setUp(self):
        self.wind = Wind(10.0)
        self.base = {"ac": ac_dict(BASELINE), "ustar": self.wind.ustar, "z0": self.wind.z0,
                     "ex": self.wind.ex, "ey": self.wind.ey}

    def test_rk4_matches_python(self):
        env = Environment.flat(self.wind)
        s0 = np.array([0.0, 0.0, 12.0, 14.0, -0.05, 0.3])
        s = s0.copy()
        dt = 0.01
        for i in range(1000):
            cl, mu = clip_controls(BASELINE, 0.8, 0.4 if i < 500 else -0.4)
            s = rk4_step(lambda t, y: derivatives(t, y, BASELINE, env, cl, mu)[0], i * dt, s, dt)
        out = np.array(run_node(dict(self.base, run="integrate", s0=list(s0)))["s"])
        scale = np.maximum(np.abs(s), 1.0)
        rel = float(np.max(np.abs(out - s) / scale))
        print("\nJS vs Python after 10 s: max relative difference %.2e" % rel)
        self.assertLess(rel, 1e-6)

    def test_tracker_keeps_height(self):
        from sim.export_refs import solve_one
        job, ref, _, why = solve_one(("baseline-2m", 13, "travel"))
        self.assertIsNotNone(ref, "reference did not converge: " + why)
        k = int(np.argmax(ref["z"][:-1]))
        s0 = [0.0, 0.0, ref["z"][k], ref["V"][k], ref["gamma"][k], ref["psi"][k]]  # wind along +x
        w13 = Wind(13.0)   # the wind the reference was solved for
        base = dict(self.base, ustar=w13.ustar, z0=w13.z0, ex=w13.ex, ey=w13.ey)
        res = run_node(dict(base, run="track", s0=s0, ref=ref, T=60.0))
        print("\ntracker 60 s at u10 13: min height %.2f m, guard steps %d, phase wraps %d"
              % (res["hmin"], res["guardSteps"], res["passes"]))
        self.assertFalse(res["nan"])
        self.assertGreater(res["hmin"], 0.0)


if __name__ == "__main__":
    unittest.main()
