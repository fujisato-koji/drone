"""The viewer's time-varying wind (gust.js) and free-setting sea (envsea.js) against Python (needs node)."""
import json
import shutil
import subprocess
import unittest
from pathlib import Path

import numpy as np

from sim import gustmodes
from sim.seawind import (TURB_K_RANGE, TURB_L, TURB_LEN_U_PER_Z, TURB_LEN_W_PER_Z, TURB_N_MODES,
                         TURB_V_RATIO, TURB_W_RATIO, Environment, Wind, WaveTrain)

VIEWER = Path(__file__).resolve().parents[1] / "viewer"
NODE = shutil.which("node")

SNIPPET = r"""
const path = require('path');
const dir = process.argv[1];
const Gust = require(path.join(dir, 'gust.js'));
const EnvSea = require(path.join(dir, 'envsea.js'));
const inp = JSON.parse(require('fs').readFileSync(0, 'utf8'));
const out = {};
if (inp.run === 'defaults') {
  out.d = Gust.DEFAULTS;
} else if (inp.run === 'rng') {
  const r = Gust.rng(inp.seed); out.u = []; for (let i = 0; i < 20; i++) out.u.push(r());
} else if (inp.run === 'gust') {
  const g = Gust.create(inp.seed);
  out.modes = { k: Array.from(g.modes.k), kx: Array.from(g.modes.kx), ky: Array.from(g.modes.ky),
                phase: g.modes.phase.map((r) => Array.from(r)) };
  out.v = inp.pts.map((p) => Array.from(g.at(p.x, p.y, p.z, p.zloc, p.t, inp.w, [0, 0, 0])));
} else if (inp.run === 'slow') {
  out.res = [];
  for (const seed of inp.seeds) for (const kind of ['calm', 'cell']) {
    const s = Gust.slowWind(seed, kind);
    const n = Math.round(inp.hours * 3600 / inp.dt);
    let sum = 0, sum2 = 0, mx = 0;
    for (let i = 0; i < n; i++) { const v = s.at(i * inp.dt); sum += v; sum2 += v * v; mx = Math.max(mx, Math.abs(v)); }
    const mean = sum / n;
    out.res.push({ seed: seed, kind: kind, mean: mean, std: Math.sqrt(sum2 / n - mean * mean), max: mx,
      sigma: s.sigma, f: s.f, a: s.a, again: s.at(1234.5), same: Gust.slowWind(seed, kind).at(1234.5) });
  }
} else if (inp.run === 'response') {
  out.tau = inp.T.map((T) => Gust.responseTime(T));
  out.scale = [Gust.shortScale(10, 10), Gust.shortScale(5, 10), Gust.shortScale(20, 10), Gust.shortScale(11, 10)];
} else if (inp.run === 'sea') {
  const arr = (c) => { const r = {}; for (const k of ['a', 'k', 'omega', 'dx', 'dy', 'phase', 'urel']) r[k] = Array.from(c[k]); return r; };
  out.res = inp.sets.map((s) => {
    const b = EnvSea.build(s);
    const b2 = EnvSea.build(s);
    return { c: arr(b.comps), c2: arr(b2.comps), ustar: b.wind.ustar, z0: b.wind.z0, hsTotal: b.hsTotal,
      trains: b.trains.map((t) => ({ hs: t.hs, tp: t.tp, to_deg: t.to_deg, gamma: t.gamma })) };
  });
}
console.log(JSON.stringify(out));
"""


def run_node(**inp):
    r = subprocess.run([NODE, "-e", SNIPPET, str(VIEWER)], input=json.dumps(inp),
                       capture_output=True, text=True, timeout=300)
    if r.returncode != 0:
        raise RuntimeError(r.stderr)
    return json.loads(r.stdout)


def rel(a, b):
    a, b = np.asarray(a, float), np.asarray(b, float)
    return float(np.max(np.abs(a - b) / np.maximum(np.abs(b), 1e-300)))


@unittest.skipUnless(NODE, "node is not on PATH: gust.js tests skipped")
class TestGustJs(unittest.TestCase):
    def test_defaults_are_the_python_constants(self):
        d = run_node(run="defaults")["d"]
        self.assertEqual(d["nModes"], TURB_N_MODES)
        self.assertAlmostEqual(d["kLo"], TURB_K_RANGE[0], places=12)
        self.assertAlmostEqual(d["kHi"], TURB_K_RANGE[1], places=12)
        self.assertEqual(d["length"], TURB_L)
        self.assertEqual(d["wRatio"], TURB_W_RATIO)
        self.assertEqual(d["vRatio"], TURB_V_RATIO)
        self.assertEqual(d["lwPerZ"], TURB_LEN_W_PER_Z)
        self.assertEqual(d["luPerZ"], TURB_LEN_U_PER_Z)

    def test_generator_matches_python(self):
        for seed in (1, 7, 123456):
            u = run_node(run="rng", seed=seed)["u"]
            r = gustmodes.mulberry32(seed)
            self.assertEqual(u, [r() for _ in range(20)], seed)

    def test_gust_matches_python(self):
        rng = np.random.default_rng(3)
        pts = []
        for _ in range(40):
            zloc = float(rng.uniform(0.3, 40.0))
            pts.append({"x": float(rng.uniform(-200, 200)), "y": float(rng.uniform(-200, 200)),
                        "z": zloc + float(rng.uniform(-1.5, 1.5)), "zloc": zloc,
                        "t": float(rng.uniform(0, 900))})
        worst = 0.0
        for u10, frm, seed in ((12.0, 250.0, 5), (6.0, 90.0, 2), (20.0, 10.0, 9)):
            wind = Wind(u10, frm)
            env = Environment(wind, [], seed=1, turbulence="iso", turbulence_profile="height")
            env._turb = gustmodes.modes(seed)
            w = {"u10": u10, "ex": wind.ex, "ey": wind.ey, "z0": wind.z0, "ustar": wind.ustar, "karman": 0.4}
            res = run_node(run="gust", seed=seed, pts=pts, w=w)
            for key in ("k", "kx", "ky"):
                self.assertLess(rel(res["modes"][key], env._turb[key]), 1e-12, key)
            self.assertLess(rel(res["modes"]["phase"], env._turb["phase"]), 1e-12)
            for p, v in zip(pts, res["v"]):
                ref = env.gust(p["x"], p["y"], p["z"], p["t"], zloc=p["zloc"])
                worst = max(worst, float(np.max(np.abs(np.array(v) - ref))))
            self.assertGreater(float(np.abs(res["v"]).max()), 0.05)   # a real field, not zeros
        print("\ngust.js vs Python: largest difference %.2e m/s at 120 points" % worst)
        self.assertLess(worst, 1e-9)

    def test_gust_is_frozen_and_advected(self):
        wind = Wind(10.0, 270.0)   # blows toward +x
        w = {"u10": 10.0, "ex": wind.ex, "ey": wind.ey, "z0": wind.z0, "ustar": wind.ustar}
        pts = [{"x": 0.0, "y": 0.0, "z": 5.0, "zloc": 5.0, "t": 0.0},
               {"x": 100.0, "y": 0.0, "z": 5.0, "zloc": 5.0, "t": 10.0}]   # 10 s at 10 m/s
        v = run_node(run="gust", seed=4, pts=pts, w=w)["v"]
        self.assertLess(float(np.max(np.abs(np.array(v[0]) - v[1]))), 1e-9)

    def test_slow_variation(self):
        res = run_node(run="slow", seeds=[1, 2, 3, 4], hours=300, dt=15.0)["res"]
        for r in res:
            with self.subTest(seed=r["seed"], kind=r["kind"]):
                self.assertEqual(r["again"], r["same"])   # seeded
                if r["kind"] == "calm":
                    f = np.array(r["f"])
                    self.assertTrue(np.all((f >= 1 / 21600) & (f <= 1 / 600)))
                    self.assertAlmostEqual(r["sigma"], 0.3, places=12)
                    print("\ncalm seed %d: std over 300 h %.3f m/s (set 0.3)" % (r["seed"], r["std"]))
                    self.assertLess(abs(r["std"] - 0.3) / 0.3, 0.10)
                else:
                    periods = 1 / np.array(r["f"]) / 60.0
                    self.assertTrue(np.all((periods >= 30) & (periods <= 60)), periods)
                    amp = sum(r["a"])
                    self.assertTrue(2.0 <= amp <= 4.0, amp)
                    self.assertLessEqual(r["max"], amp + 1e-9)
                    self.assertGreater(r["max"], 0.7 * amp)   # it does swing about as much as asked

    def test_seeds_differ(self):
        res = run_node(run="slow", seeds=[1, 2], hours=1, dt=60.0)["res"]
        by = {(r["seed"], r["kind"]): r["again"] for r in res}
        self.assertNotEqual(by[(1, "calm")], by[(2, "calm")])
        self.assertNotEqual(by[(1, "cell")], by[(2, "cell")])

    def test_response_time_of_short_waves(self):
        T = [0.5, 1.0, 2.0, 3.0]
        r = run_node(run="response", T=T)
        tau = r["tau"]
        self.assertEqual(tau, sorted(tau))
        self.assertTrue(all(60 <= v <= 600 for v in tau))
        self.assertAlmostEqual(tau[0], 60.0)          # 21 s at 0.5 s in the source, raised to 1 min
        self.assertAlmostEqual(tau[2], 21 * 4 ** (np.log(3000 / 21) / np.log(8)), delta=1.0)
        self.assertEqual(r["scale"][0], 1.0)
        self.assertEqual(r["scale"][1], 0.25)   # half the wind, Hs follows U^2, limited at a quarter
        self.assertEqual(r["scale"][2], 4.0)
        self.assertAlmostEqual(r["scale"][3], 1.21)


@unittest.skipUnless(NODE, "node is not on PATH: envsea.js tests skipped")
class TestEnvSeaJs(unittest.TestCase):
    SETS = [
        {"u10": 8.0, "fromDeg": 270.0, "hours": 0, "swell": None},
        {"u10": 14.0, "fromDeg": 200.0, "hours": 6, "swell": None},
        {"u10": 20.0, "fromDeg": 45.0, "hours": 1, "swell": {"hs": 1.2, "tp": 12.0, "fromDeg": 300.0}},
        {"u10": 5.0, "fromDeg": 10.0, "hours": 24, "swell": {"hs": 0.5, "tp": 8.0, "fromDeg": 10.0}},
        {"u10": 14.0, "fromDeg": 200.0, "hours": 6, "swell": None},   # the second setting repeated
    ]

    @classmethod
    def setUpClass(cls):
        cls.res = run_node(run="sea", sets=[dict(s, seed=1) for s in cls.SETS])["res"]

    def test_every_rebuild_matches_python(self):
        for s, r in zip(self.SETS, self.res):
            with self.subTest(s=s):
                w = Wind(s["u10"], s["fromDeg"])
                tr = WaveTrain.fully_developed(w) if s["hours"] == 0 else WaveTrain.duration_limited(w, s["hours"])
                py = tr.components(np.random.default_rng(1))
                n = len(py["a"])
                c = r["c"]
                for key in ("a", "k", "omega"):
                    self.assertLess(rel(c[key][:n], py[key]), 1e-9, key)
                for key in ("dx", "dy"):
                    self.assertLess(float(np.max(np.abs(np.array(c[key][:n]) - py[key]))), 1e-12, key)
                if s["swell"]:
                    sw = WaveTrain(s["swell"]["hs"], s["swell"]["tp"], (s["swell"]["fromDeg"] + 180) % 360,
                                   gamma=7.0, n_omega=8, n_theta=4, name="swell")
                    ps = sw.components(np.random.default_rng(1))
                    self.assertEqual(len(c["a"]), n + 32)
                    for key in ("a", "k", "omega"):
                        self.assertLess(rel(c[key][n:], ps[key]), 1e-9, key)
                else:
                    self.assertEqual(len(c["a"]), n)
                self.assertLess(rel(r["ustar"], w.ustar), 1e-9)
                self.assertLess(rel(r["z0"], w.z0), 1e-9)
                # urel as the viewer's setWind has it
                zref = np.maximum(1 / np.array(c["k"]), 2 * w.z0)
                urel = (w.speed(zref) * (w.ex * np.array(c["dx"]) + w.ey * np.array(c["dy"]))
                        - np.array(c["omega"]) / np.array(c["k"]))
                self.assertLess(float(np.max(np.abs(urel - c["urel"]))), 1e-9)
                self.assertEqual(r["c"], r["c2"])   # a rebuild with the same setting is identical
                self.assertLessEqual(len(c["a"]), 128)

    def test_rebuild_back_gives_the_same_sea(self):
        self.assertEqual(self.res[1]["c"], self.res[4]["c"])   # same setting after other changes

    def test_phases_stay_when_a_slider_moves(self):
        a = run_node(run="sea", sets=[{"u10": 10.0, "fromDeg": 270.0, "hours": 0, "swell": None, "seed": 3},
                                      {"u10": 11.0, "fromDeg": 270.0, "hours": 0, "swell": None, "seed": 3}])["res"]
        self.assertEqual(a[0]["c"]["phase"], a[1]["c"]["phase"])
        self.assertNotEqual(a[0]["c"]["a"], a[1]["c"]["a"])


if __name__ == "__main__":
    unittest.main()
