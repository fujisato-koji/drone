"""The viewer's flight.js against sim.aircraft (needs node)."""
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
let s = inp.s0.slice();
const dt = 0.01, n = 1000;
for (let i = 0; i < n; i++) s = Flight.rk4(s, 0.8, i < 500 ? 0.4 : -0.4, ac, windFn, i * dt, dt);
console.log(JSON.stringify({ s: s }));
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


class TestViewerAircraft(unittest.TestCase):
    """aircraft.js carries the parameters of the library aircraft (no node needed)."""

    def test_parameters_match_python(self):
        from sim.aircraft_models import by_name
        from sim.export_aircraft import build_params, params
        text = (FLIGHT_JS.parent / "aircraft.js").read_text(encoding="utf-8")
        data = json.loads(text[text.index("window.AIRCRAFT = ") + len("window.AIRCRAFT = "):].rstrip().rstrip(";"))
        self.assertEqual(data, build_params())
        for name, p in data.items():
            self.assertEqual(p, params(by_name(name)))
        lib = (FLIGHT_JS.parent / "library.js").read_text(encoding="utf-8")
        for name in data:
            self.assertIn('"%s"' % name, lib)

    def test_tracker_is_gone_from_the_page(self):
        page = (FLIGHT_JS.parent / "index.html").read_text(encoding="utf-8")
        for word in ("refs.js", "REFS", "Tracker", "flCtl"):
            self.assertNotIn(word, page)
        self.assertNotIn("Tracker", FLIGHT_JS.read_text(encoding="utf-8"))


if __name__ == "__main__":
    unittest.main()
