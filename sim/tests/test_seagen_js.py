"""The viewer's seagen.js against sim.seawind: wind, growth and the deterministic wave components (needs node)."""
import json
import shutil
import subprocess
import unittest
from pathlib import Path

import numpy as np

from sim.seawind import G, Wind, WaveTrain, jonswap

SEAGEN_JS = Path(__file__).resolve().parents[1] / "viewer" / "seagen.js"
NODE = shutil.which("node")
WINDS = [4.0, 8.0, 12.0, 16.0, 20.0]
HOURS = [2.0, 6.0, 12.0]
FROM = 250.0

SNIPPET = r"""
const S = require(process.argv[1]);
const inp = JSON.parse(require('fs').readFileSync(0, 'utf8'));
const out = {};
const arr = (c) => { const r = {}; for (const k of ['a','k','omega','dx','dy','phase']) r[k] = Array.from(c[k]); return r; };
if (inp.run === 'growth') {
  out.wind = []; out.fd = []; out.dl = [];
  for (const u of inp.winds) for (const rough of ['charnock', 'coare35']) {
    const w = S.wind(u, inp.from, rough);
    out.wind.push({ u: u, rough: rough, ustar: w.ustar, z0: w.z0, to: w.to_deg, ex: w.ex, ey: w.ey });
    if (rough === 'charnock') {
      out.fd.push(S.fullyDeveloped(w));
      for (const h of inp.hours) out.dl.push(Object.assign({ u: u, h: h }, S.durationLimited(w, h)));
    }
  }
} else if (inp.run === 'components') {
  const w = S.wind(inp.u, inp.from);
  const tr = S.durationLimited(w, inp.hours);
  const c = S.components(tr, 96, inp.seed);
  out.tr = tr; out.c = arr(c); out.bins = S.binning(tr, 12);
  const sw = S.swell(0.8, 11, 200, 32, inp.seed);
  out.sw = arr(sw);
  const comb = S.combine(tr, { hs: 0.8, tp: 11, to_deg: 200 }, inp.seed);
  out.comb = arr(comb); out.slices = comb.slices;
  out.comb_noswell = arr(S.combine(tr, null, inp.seed));
  // phases: different seeds differ, same seed repeats
  out.ph2 = Array.from(S.components(tr, 96, inp.seed + 1).phase);
  out.ph1 = Array.from(S.components(tr, 96, inp.seed).phase);
} else if (inp.run === 'timing') {
  const w = S.wind(12, 270);
  const t0 = process.hrtime.bigint();
  const reps = 200;
  for (let i = 0; i < reps; i++) {
    const w2 = S.wind(8 + (i % 8), 270, 'coare35');
    S.combine(S.durationLimited(w2, 6), { hs: 0.8, tp: 11, to_deg: 200 }, i);
  }
  out.ms = Number(process.hrtime.bigint() - t0) / 1e6 / reps;
}
console.log(JSON.stringify(out));
"""


def run_node(**inp):
    res = subprocess.run([NODE, "-e", SNIPPET, str(SEAGEN_JS)], input=json.dumps(inp),
                         capture_output=True, text=True, timeout=60)
    if res.returncode != 0:
        raise RuntimeError(res.stderr)
    return json.loads(res.stdout)


def rel(a, b):
    a, b = np.asarray(a, float), np.asarray(b, float)
    return float(np.max(np.abs(a - b) / np.maximum(np.abs(b), 1e-300)))


@unittest.skipUnless(NODE, "node not on PATH")
class SeaGenTest(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.g = run_node(run="growth", winds=WINDS, hours=HOURS, **{"from": FROM})

    def test_wind(self):
        for r in self.g["wind"]:
            w = Wind(r["u"], FROM, roughness=r["rough"])
            with self.subTest(u=r["u"], rough=r["rough"]):
                self.assertLess(rel(r["ustar"], w.ustar), 1e-9)
                self.assertLess(rel(r["z0"], w.z0), 1e-9)
                self.assertAlmostEqual(r["to"], w.to_deg, places=9)
                self.assertAlmostEqual(r["ex"], w.ex, places=12)
                self.assertAlmostEqual(r["ey"], w.ey, places=12)

    def test_fully_developed(self):
        for u, r in zip(WINDS, self.g["fd"]):
            t = WaveTrain.fully_developed(Wind(u, FROM))
            with self.subTest(u=u):
                self.assertLess(rel(r["hs"], t.hs), 1e-9)
                self.assertLess(rel(r["tp"], t.tp), 1e-9)
                self.assertEqual(r["gamma"], t.gamma)

    def test_duration_limited(self):
        self.assertEqual(len(self.g["dl"]), len(WINDS) * len(HOURS))
        for r in self.g["dl"]:
            t = WaveTrain.duration_limited(Wind(r["u"], FROM), r["h"])
            with self.subTest(u=r["u"], h=r["h"]):
                self.assertLess(rel(r["hs"], t.hs), 1e-9)
                self.assertLess(rel(r["tp"], t.tp), 1e-9)
                self.assertLess(rel(r["hs_full"], t.hs_full), 1e-9)
                self.assertEqual(r["gamma"], t.gamma)


@unittest.skipUnless(NODE, "node not on PATH")
class SeaGenComponentsTest(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.r = run_node(run="components", u=14.0, hours=5.0, seed=7, **{"from": 250.0})
        cls.t = WaveTrain.duration_limited(Wind(14.0, 250.0), 5.0)
        cls.py = cls.t.components(np.random.default_rng(7))

    def test_train_matches(self):
        self.assertLess(rel(self.r["tr"]["hs"], self.t.hs), 1e-9)
        self.assertEqual((self.r["tr"]["n_omega"], self.r["tr"]["n_theta"]), (12, 8))

    def test_deterministic_arrays(self):
        c = self.r["c"]
        self.assertEqual(len(c["a"]), 96)
        for key in ("a", "k", "omega"):
            self.assertLess(rel(c[key], self.py[key]), 1e-9, key)
        for key in ("dx", "dy"):
            self.assertLess(float(np.max(np.abs(np.array(c[key]) - self.py[key]))), 1e-12, key)

    def test_bins(self):
        t = self.t
        wp = 2 * np.pi / t.tp
        om = np.linspace(0.5 * wp, 4 * wp, 2000)
        s = jonswap(om, t.hs, t.tp, t.gamma)
        cum = np.concatenate([[0.0], np.cumsum(0.5 * (s[1:] + s[:-1]) * np.diff(om))])
        frac = cum / cum[-1]
        b = self.r["bins"]
        edges = [om[np.argmax(np.floor(frac * 12) >= i)] for i in range(12)] + [om[-1]]
        self.assertLess(rel(b["edges"], edges), 1e-9)
        self.assertLess(rel(b["centres"], self.py["omega"].reshape(12, 8)[:, 0]), 1e-9)
        self.assertLess(rel(b["m0"], cum[-1]), 1e-9)
        self.assertTrue(np.all(np.diff(b["edges"]) > 0))
        # equal energy: every bin carries the same amplitude sum a^2/2
        a2 = (np.array(self.r["c"]["a"]) ** 2 / 2).reshape(12, 8).sum(axis=1)
        self.assertLess(rel(a2, np.full(12, cum[-1] / 12)), 1e-9)

    def test_dispersion(self):
        c = self.r["c"]
        self.assertLess(rel(c["k"], np.array(c["omega"]) ** 2 / G), 1e-12)

    def test_variance_and_hs(self):
        c = self.r["c"]
        m0 = float(np.sum(np.array(c["a"]) ** 2 / 2))
        self.assertLess(rel(m0, self.r["bins"]["m0"]), 1e-9)
        hs_c = 4 * np.sqrt(m0)
        # the spectrum is cut to 0.5 to 4 wp, so Hs of the components is a little below the train's;
        # the Python components have the same cut
        self.assertLess(rel(hs_c, 4 * np.sqrt(np.sum(self.py["a"] ** 2 / 2))), 1e-9)
        # 1e-6 is not reachable: the 0.5 to 4 wp cut and the approximate JONSWAP normalisation leave
        # Hs 0.04% low at gamma 3.3 (0.24% at gamma 1, 1% at gamma 7), the same in Python
        self.assertLess(abs(hs_c - self.t.hs) / self.t.hs, 1e-3)

    def test_direction_moments(self):
        c = self.r["c"]
        w = np.array(c["a"]) ** 2
        for src, name in ((c, "js"), (self.py, "py")):
            ang = np.degrees(np.arctan2(src["dx"], src["dy"]))
            vec = np.array([np.sum(w * np.sin(np.radians(ang))), np.sum(w * np.cos(np.radians(ang)))]) / w.sum()
            mean = np.degrees(np.arctan2(vec[0], vec[1])) % 360
            spread = np.degrees(np.sqrt(2 * (1 - np.hypot(*vec))))
            if name == "js":
                js = (mean, spread)
            else:
                py = (mean, spread)
        self.assertLess(abs(js[0] - py[0]), 1.0)
        self.assertLess(abs(js[1] - py[1]), 1.0)
        self.assertLess(abs(js[0] - self.t.to_deg), 1.0)

    def test_phases(self):
        ph = np.array(self.r["c"]["phase"])
        self.assertTrue(np.all((ph >= 0) & (ph < 2 * np.pi)))
        self.assertAlmostEqual(float(ph.mean()), np.pi, delta=0.5)
        self.assertEqual(self.r["ph1"], list(ph))
        self.assertNotEqual(self.r["ph1"], self.r["ph2"])

    def test_swell_and_combination(self):
        r = self.r
        self.assertEqual(r["slices"], [[0, 96], [96, 128]])
        sw = WaveTrain(0.8, 11.0, 200.0, gamma=7.0, n_omega=8, n_theta=4, name="swell")
        py_sw = sw.components(np.random.default_rng(1))
        for key in ("a", "k", "omega", "dx", "dy"):
            self.assertLess(float(np.max(np.abs(np.array(r["sw"][key]) - py_sw[key]) / np.maximum(np.abs(py_sw[key]), 1e-12))), 1e-9, key)
        a = np.array(r["comb"]["a"])
        self.assertEqual(a.size, 128)
        var_sea = np.sum(np.array(r["c"]["a"]) ** 2 / 2)
        var_sw = np.sum(np.array(r["sw"]["a"]) ** 2 / 2)
        self.assertLess(rel(np.sum(a ** 2 / 2), var_sea + var_sw), 1e-9)
        hs_tot = 4 * np.sqrt(np.sum(a ** 2 / 2))
        self.assertLess(abs(hs_tot - np.hypot(self.t.hs, 0.8)) / hs_tot, 0.02)
        for key in ("a", "omega"):
            self.assertEqual(r["comb"][key][:96], r["c"][key])
        self.assertEqual(len(r["comb_noswell"]["a"]), 96)

    def test_speed(self):
        ms = run_node(run="timing")["ms"]
        print(f"seagen.js: wind + duration-limited + combine of 128 components, {ms:.2f} ms each")
        self.assertLess(ms, 10.0)


if __name__ == "__main__":
    unittest.main()
