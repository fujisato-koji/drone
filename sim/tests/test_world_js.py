"""The viewer's worlddata.js against sim.world: decoding, geometry, interpolation, the forecast's corridor and request
URLs, and the development hours of the 3D sea (needs node)."""
import json
import shutil
import subprocess
import unittest
from pathlib import Path

import numpy as np

from sim import world as W
from sim.seawind import WaveTrain, Wind
from sim.tests.test_world import FakeService, linear_corridor

WORLD_JS = Path(__file__).resolve().parents[1] / "viewer" / "worlddata.js"
NODE = shutil.which("node")

SNIPPET = r"""
const WD = require(process.argv[1]);
const inp = JSON.parse(require('fs').readFileSync(0, 'utf8'));
const out = {};
const arr = (a) => Array.from(a, (x) => (Number.isFinite(x) ? x : null));
const cor = WD.unpack(inp.corridor), glob = WD.unpack(inp.global);
out.cor = {}; for (const k of Object.keys(cor.vals)) out.cor[k] = arr(cor.vals[k]);
out.glob = {}; for (const k of Object.keys(glob.vals)) out.glob[k] = arr(glob.vals[k]);
out.nOcean = glob.nOcean;
out.fwd = inp.pts.map(([la0, lo0, la, lo]) => WD.aeqdFwd(la0, lo0, la, lo));
out.inv = inp.xy.map(([la0, lo0, x, y]) => WD.aeqdInv(la0, lo0, x, y));
const cp = WD.corridorPoints(inp.spec); out.cpts = [Array.from(cp.lat), Array.from(cp.lon)];
out.samples = inp.samples.map(([la, lo, t]) => { const s = WD.sampleCorridor(cor, la, lo, t); const o = {}; for (const k of Object.keys(s)) o[k] = (typeof s[k] === 'number' && !Number.isFinite(s[k])) ? null : s[k]; return o; });
out.gsamples = inp.gsamples.map(([la, lo]) => { const s = WD.sampleGlobal(glob, la, lo); const o = {}; for (const k of Object.keys(s)) o[k] = Number.isFinite(s[k]) ? s[k] : null; return o; });
const built = WD.buildCorridor(inp.nowSpec, inp.nowT0, inp.wind, inp.waves);
out.built = {}; for (const k of Object.keys(built.vals)) out.built[k] = arr(built.vals[k]);
out.urls = inp.urlCases.map(([key, la, lo, d0, d1]) => WD.requestUrl(key, la, lo, d0, d1));
out.dev = inp.dev.map(([u, hs]) => WD.developHours(u, 250, hs));
out.q = {}; for (const nm of Object.keys(inp.raw)) out.q[nm] = inp.raw[nm].map((x) => WD.quantize(nm, x === null ? NaN : x));
out.packed = WD.pack(built);
{   // the phases stay continuous at a point when the sea is rebuilt from a nearby sample
  const ES = require(require('path').join(require('path').dirname(process.argv[1]), 'envsea.js'));
  const b1 = ES.build({ u10: 9, fromDeg: 250, hours: 6, swell: { hs: 1.5, tp: 11, fromDeg: 200 }, seed: 1 });
  const b2 = ES.build({ u10: 9.4, fromDeg: 253, hours: 6.5, swell: { hs: 1.6, tp: 11.2, fromDeg: 202 }, seed: 1 });
  const n = b1.comps.a.length, x = 412.5, y = -233.1, t = 1234.5;
  const th = (c, i) => c.k[i] * (c.dx[i] * x + c.dy[i] * y) - c.omega[i] * t + c.phase[i];
  const before = Math.max(...Array.from({ length: n }, (_, i) => Math.abs(Math.sin((th(b1.comps, i) - th(b2.comps, i)) / 2))));
  out.matched = WD.matchPhases(b1.comps, b2.comps, n, x, y, t);
  out.phaseErr = Math.max(...Array.from({ length: n }, (_, i) => Math.abs(Math.sin((th(b1.comps, i) - th(b2.comps, i)) / 2))));
  out.phaseBefore = before;
  out.mismatch = WD.matchPhases(b1.comps, ES.build({ u10: 9, fromDeg: 250, hours: 6, swell: null, seed: 1 }).comps, n, x, y, t);
}
out.set = WD.seaSet({ u10: -6, v10: -8, hs: 3.0, hsw: 2.0, tsw: 12.5, dsw: 200 }, null, 3);
console.log(JSON.stringify(out));
"""


def run_node(inp):
    res = subprocess.run([NODE, "-e", SNIPPET, str(WORLD_JS)], input=json.dumps(inp), capture_output=True,
                         text=True, timeout=60)
    if res.returncode != 0:
        raise RuntimeError(res.stderr)
    return json.loads(res.stdout)


def nan_equal(a, b, tol):
    a = np.array([np.nan if x is None else x for x in np.ravel(a)], float)
    b = np.asarray(b, float).ravel()
    same_nan = np.isnan(a) == np.isnan(b)
    ok = np.isnan(a) | (np.abs(np.nan_to_num(a) - np.nan_to_num(b)) <= tol)
    return bool(same_nan.all() and ok.all())


@unittest.skipUnless(NODE, "node not on PATH")
class WorldJsTest(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        rng = np.random.default_rng(11)
        c = linear_corridor()
        c["vals"]["v10"] = -0.5 * c["vals"]["u10"]
        c["vals"]["hsw"] = 0.6 * c["vals"]["hs"]
        c["vals"]["tsw"] = np.full_like(c["vals"]["hs"], 12.0)
        c["vals"]["hs"][2, 5] = np.nan
        c["vals"]["dsw"] = rng.uniform(0, 360, c["vals"]["hs"].shape)
        c["lat"], c["lon"], c["bearing_deg"] = -48.0, 178.0, 70.0     # across 180 deg
        cls.cpack = json.loads(json.dumps(W.pack_field(c)))
        cls.cor = W.unpack_field(cls.cpack)
        g = dict(W.GLOBAL, kind="global", t0=0, time="x")
        la, lo = W.global_grid(g)
        g["mask"] = W.ocean_mask(W.load_land())
        g["vals"] = {"u10": (5 + 0.1 * la)[g["mask"]], "v10": np.cos(np.radians(lo))[g["mask"]],
                     "hs": (2 + 0.02 * np.abs(la))[g["mask"]]}
        cls.gpack = json.loads(json.dumps(W.pack_field(g)))
        cls.glob = W.unpack_field(cls.gpack)
        cls.pts = [[-50.0, 58.0, -47.3, 66.1], [10.0, 179.0, 12.0, -178.5], [0.0, 0.0, 0.0, 0.0], [65.0, -20.0, 60.0, -30.0]]
        cls.xy = [[-50.0, 58.0, 412.0, -233.0], [10.0, 179.0, 300.0, 50.0], [0.0, 0.0, 0.0, 0.0], [65.0, -20.0, -800.0, 900.0]]
        lat, lon = W.corridor_points(cls.cor)
        cls.samples = []
        for _ in range(40):
            i = rng.integers(0, len(lat))
            cls.samples.append([float(lat[i] + rng.normal(0, 0.3)), float(lon[i] + rng.normal(0, 0.4)),
                                float(c["t0"] + rng.uniform(-3600, 14 * 3600))])
        cls.gsamples = [[float(rng.uniform(-75, 75)), float(rng.uniform(-180, 180))] for _ in range(60)] + [[12.5, 179.9]]
        # the forecast's corridor from the fake service's responses (the same lists in Python and node)
        cls.now_spec = dict(W.CORRIDOR, na=3, nc=2, lat=-20.0, lon=60.0)
        fake = FakeService()
        plat, plon = W.corridor_points(cls.now_spec)
        cls.wind = json.loads(fake(W.request_url("ifs_wind", plat, plon, "2023-07-07", "2023-07-10"))[0])
        cls.waves = json.loads(fake(W.request_url("mfwam_waves", plat, plon, "2023-07-07", "2023-07-10"))[0])
        cls.now_t0 = W.parse_utc("2023-07-07T03:00Z")
        cls.url_cases = [["ifs_wind", [-15.0, 12.3456], [-10.0, 179.99991], "2026-11-01", "2026-11-04"],
                         ["mfwam_waves", [-15.25], [3.0], "2026-11-01", "2026-11-04"],
                         ["mfwam_waves_global", [-70.0, 65.0], [-180.0, 175.0], "2026-11-01", "2026-11-01"]]
        cls.raw = {nm: [float(x) for x in rng.uniform(-40 if W.QUANT[nm].get("signed") else -1, 400 if nm == "dsw" else 40, 300)]
                   + [0.03, 0.09, 0.15, -0.125, 0.375, 359.99] for nm in W.QUANT}
        for nm in cls.raw:
            cls.raw[nm][5] = None
        cls.dev = [[10.0, 0.5], [10.0, 1.5], [10.0, 50.0], [3.0, 0.05], [15.0, float("nan")]]
        cls.r = run_node({"corridor": cls.cpack, "global": cls.gpack, "pts": cls.pts, "xy": cls.xy, "spec": {k: cls.cor[k] for k in ("lat", "lon", "bearing_deg", "na", "nc", "step_km")},
                          "samples": cls.samples, "gsamples": cls.gsamples, "nowSpec": cls.now_spec, "nowT0": cls.now_t0,
                          "wind": cls.wind, "waves": cls.waves, "urlCases": cls.url_cases,
                          "dev": [[u, None if np.isnan(h) else h] for u, h in cls.dev], "raw": cls.raw})

    def test_decode_is_exact(self):
        for k, v in self.cor["vals"].items():
            self.assertTrue(nan_equal(self.r["cor"][k], v, 0.0), k)
        for k, v in self.glob["vals"].items():
            self.assertTrue(nan_equal(self.r["glob"][k], v, 0.0), k)
        self.assertEqual(self.r["nOcean"], int(self.glob["mask"].sum()))

    def test_geometry(self):
        for (la0, lo0, la, lo), js in zip(self.pts, self.r["fwd"]):
            x, y = W.aeqd_forward(la0, lo0, la, lo)
            self.assertAlmostEqual(js[0], float(x), places=8)
            self.assertAlmostEqual(js[1], float(y), places=8)
        for (la0, lo0, x, y), js in zip(self.xy, self.r["inv"]):
            la, lo = W.aeqd_inverse(la0, lo0, x, y)
            self.assertAlmostEqual(js[0], float(la), places=9)
            self.assertAlmostEqual(js[1], float(lo), places=9)
        lat, lon = W.corridor_points(self.cor)
        self.assertLess(np.abs(np.array(self.r["cpts"][0]) - lat).max(), 1e-9)
        self.assertLess(np.abs(np.array(self.r["cpts"][1]) - lon).max(), 1e-9)

    def test_corridor_samples(self):
        for (la, lo, t), js in zip(self.samples, self.r["samples"]):
            py = W.sample_corridor(self.cor, la, lo, t)
            self.assertEqual(js["inside"], py["inside"])
            for k in ("fi", "fj", "u10", "v10", "hs", "hsw", "tsw"):
                self.assertTrue(nan_equal([js[k]], [py[k]], 1e-9), (k, js[k], py[k]))
            d = abs((js["dsw"] - py["dsw"] + 180) % 360 - 180)
            self.assertLess(d, 1e-7)

    def test_global_samples(self):
        for (la, lo), js in zip(self.gsamples, self.r["gsamples"]):
            py = W.sample_global(self.glob, la, lo)
            for k in ("u10", "v10", "hs"):
                self.assertTrue(nan_equal([js[k]], [py[k]], 1e-9), (k, la, lo, js[k], py[k]))

    def test_forecast_corridor(self):
        py = W.build_corridor(self.now_spec, self.now_t0, self.wind, self.waves, "mfwam_waves")
        for k in W.CORRIDOR_VARS:
            self.assertTrue(nan_equal(self.r["built"][k], py["vals"][k], 1e-12), k)

    def test_quantize_and_pack_agree(self):
        for nm, xs in self.raw.items():
            py, _ = W.quantize(nm, [np.nan if x is None else x for x in xs])
            self.assertEqual(self.r["q"][nm], [int(c) for c in py], nm)
        py = W.pack_field(W.build_corridor(self.now_spec, self.now_t0, self.wind, self.waves, "mfwam_waves"))
        js = self.r["packed"]
        self.assertEqual(js["data"], py["data"])
        for k in ("kind", "lat", "lon", "bearing_deg", "na", "nc", "step_km", "t0", "dt_h", "nt", "swell", "vars", "layout"):
            self.assertEqual(js[k], py[k], k)
        back = W.unpack_field(json.loads(json.dumps(js)))
        self.assertEqual(back["t0"], self.now_t0)

    def test_phases_stay_continuous(self):
        self.assertTrue(self.r["matched"])
        self.assertGreater(self.r["phaseBefore"], 0.1)
        self.assertLess(self.r["phaseErr"], 1e-9)
        self.assertFalse(self.r["mismatch"])      # a different set of trains is not matched

    def test_request_urls(self):
        for case, js in zip(self.url_cases, self.r["urls"]):
            self.assertEqual(js, W.request_url(*case))

    def test_development_hours(self):
        for (u, hs), h in zip(self.dev, self.r["dev"]):
            w = Wind(u, 250.0)
            full = WaveTrain.fully_developed(w).hs
            if not np.isfinite(hs) or hs >= full:
                self.assertEqual(h, 0)
            else:
                self.assertGreater(h, 0)
                self.assertAlmostEqual(WaveTrain.duration_limited(w, h).hs / hs, 1.0, places=6)
        s = self.r["set"]
        self.assertAlmostEqual(s["u10"], 10.0)
        self.assertAlmostEqual(s["fromDeg"], 36.86989764584402)     # a wind towards the south-west comes from the north-east
        self.assertAlmostEqual(s["hsWind"], 5 ** 0.5)
        self.assertEqual(s["swell"], {"hs": 2.0, "tp": 12.5, "fromDeg": 200})


if __name__ == "__main__":
    unittest.main()
