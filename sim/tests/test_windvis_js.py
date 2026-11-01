"""The wind arrows around the aircraft (sim/viewer/windvis.js), needs node.

Each layer's arrows must be the wind function's values at their points: here the wind function is the regular swell's
oracle world (sim/viewer/mpc.js SeaWorld from wave.js swellSea, which is the viewer's own wind function over the regular
swell) and a free-setting sea of envsea.js in the same world. The headless check compares the drawn arrows with the
page's own fWind and fSurface in the three settings. Also checked: the grid's lattice, the arrows' size, that over the
regular swell the 2 m layer shows the air rising over the forward faces, the colour scale, the record's keys and the
panel.
"""
import json
import re
import shutil
import subprocess
import unittest
from pathlib import Path

from sim import record

VIEWER = Path(__file__).resolve().parents[1] / "viewer"
NODE = shutil.which("node")

SNIPPET = r"""
global.window = global;
const dir = process.argv[1];
require(dir + '/wave/index.js');
const MPC = require(dir + '/mpc.js'), WAVE = require(dir + '/wave.js'), V = require(dir + '/windvis.js');
const E = require(dir + '/envsea.js');
const inp = JSON.parse(require('fs').readFileSync(0, 'utf8'));
const worlds = {
  regular: MPC.makeWorld(WAVE.swellSea({ u10: 6, fromDeg: 270, hs: 3, tp: 10, direction: 'along', preset: 'p63' }, window.WAVE_INDEX)),
};
{
  const b = E.build({ u10: 10, fromDeg: 270, hours: 0, swell: { hs: 2, tp: 11, fromDeg: 200 }, seed: 1 });
  worlds.free = MPC.makeWorld({ kind: 'truth', wind: Object.assign({ karman: 0.4 }, b.wind), comps: b.comps, stokes: false });
}
const o3 = [0, 0, 0];
const out = { defaults: V.DEFAULTS, cases: [] };
for (const c of inp.cases) {
  const w = worlds[c.world];
  const wind = (x, y, z, t) => { w.windAt(x, y, z, t, o3); return [o3[0], o3[1], o3[2]]; };
  const surf = (x, y, t) => w.eta(x, y, t);
  const col = V.columnOf(wind, surf);
  const pts = V.gridPoints(c.cx, c.cy);
  const layers = c.hs.map((h) => {
    const g = V.sampleGrid(pts, h, col, c.t), rows = [];
    for (let p = 0; p < g.n; p++) {
      const x = pts.x[p], y = pts.y[p];
      // the surface's slope along +x (the regular swell's travel) at the point
      const slope = (surf(x + 0.5, y, c.t) - surf(x - 0.5, y, c.t)) / 1.0;
      rows.push([x, y, g.z[p], g.u[p], g.v[p], g.w[p], surf(x, y, c.t), slope].concat(wind(x, y, g.z[p], c.t)));
    }
    return { h: h, rows: rows };
  });
  out.cases.push({ name: c.name, d: pts.d, n: pts.n, layers: layers });
}
out.colours = [-1, -0.5, -0.25, 0, 0.25, 0.5, 1].map((w) => V.divColor(w, 0.5, [0, 0, 0]).slice());
out.poles = V.POLES;
out.sizes = [0, 1, 4, 10, 25].map((s) => V.arrowSize(s));
console.log(JSON.stringify(out));
"""

CASES = [
    # the regular swell along a 6 m/s wind at the mast and around an aircraft; the free setting's sea with an irregular
    # swell from 200 deg
    {"name": "regular mast", "world": "regular", "cx": 0.0, "cy": 0.0, "t": 3.7, "hs": [2.0, 5.0, 10.0]},
    {"name": "regular aircraft", "world": "regular", "cx": 137.3, "cy": -41.9, "t": 61.2, "hs": [2.0, 5.0, 1.7]},
    {"name": "free aircraft", "world": "free", "cx": -63.1, "cy": 18.4, "t": 12.3, "hs": [2.0, 5.0, 7.3]},
]


def run_node():
    res = subprocess.run([NODE, "-e", SNIPPET, str(VIEWER)], input=json.dumps({"cases": CASES}),
                         capture_output=True, text=True, timeout=80)
    if res.returncode != 0:
        raise RuntimeError(res.stderr)
    return json.loads(res.stdout)


@unittest.skipUnless(NODE, "node not on PATH")
class WindVisTest(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.out = run_node()

    def test_arrows_are_the_wind_at_their_points(self):
        for c, case in zip(self.out["cases"], CASES):
            d, n = c["d"], c["n"]
            for layer in c["layers"]:
                rows = layer["rows"]
                self.assertEqual(n * n, len(rows))
                xs = sorted({round(r[0], 9) for r in rows})
                ys = sorted({round(r[1], 9) for r in rows})
                self.assertEqual((len(xs), len(ys)), (n, n))
                for a, b in zip(xs, xs[1:]):
                    self.assertAlmostEqual(b - a, d, places=9)
                # centred on the lattice point nearest the aircraft
                self.assertLessEqual(abs(0.5 * (xs[0] + xs[-1]) - case["cx"]), d / 2)
                self.assertLessEqual(abs(0.5 * (ys[0] + ys[-1]) - case["cy"]), d / 2)
                for x, y, z, u, v, w, eta, slope, qu, qv, qw in rows:
                    self.assertAlmostEqual(z, eta + layer["h"], places=12)   # the same height above the local surface
                    self.assertEqual((u, v, w), (qu, qv, qw))

    def test_two_metre_layer_over_the_regular_swell(self):
        """Over the regular swell along a light wind (the swell faster than the wind) the 2 m layer shows the air rising
        over the forward faces, which slope down ahead of a crest (the swell travels towards +x), and sinking over the
        backs: the arrows there are warm and cool."""
        for c in self.out["cases"][:2]:
            layer = next(lay for lay in c["layers"] if lay["h"] == 2.0)
            agree = disagree = 0
            for x, y, z, u, v, w, eta, slope, *_ in layer["rows"]:
                if abs(slope) > 0.02:
                    if (slope < 0) == (w > 0):
                        agree += 1
                    else:
                        disagree += 1
            self.assertGreater(agree, 60, c["name"])
            self.assertEqual(disagree, 0, c["name"])

    def test_arrow_size(self):
        d = self.out["defaults"]
        for s, (length, width) in zip([0, 1, 4, 10, 25], self.out["sizes"]):
            self.assertAlmostEqual(length, s * d["arrowScale"], places=12)
            self.assertAlmostEqual(width, min(d["widthMax"], d["widthPerLength"] * length), places=12)

    def test_colour_scale(self):
        cols, poles = self.out["colours"], self.out["poles"]
        self.assertEqual(cols[3], poles["neutral"])
        self.assertEqual(cols[0], poles["down"])
        self.assertEqual(cols[1], poles["down"])   # saturated beyond wMax
        self.assertEqual(cols[5], poles["up"])
        self.assertEqual(cols[6], poles["up"])
        self.assertGreater(cols[5][0], cols[5][2])   # up is warm (red over blue)
        self.assertGreater(cols[0][2], cols[0][0])   # down is cool

    def test_defaults_are_the_record_values(self):
        d = self.out["defaults"]
        pairs = {"viewer.wind_grid_n": d["gridN"], "viewer.wind_grid_half_m": d["gridHalf"],
                 "viewer.wind_grid_mast_h_m": d["mastH"], "viewer.wind_layer_low_h_m": d["layerHeights"][0],
                 "viewer.wind_layer_mid_h_m": d["layerHeights"][1], "viewer.wind_arrow_s": d["arrowScale"],
                 "viewer.wind_arrow_width_ratio": d["widthPerLength"], "viewer.wind_arrow_width_max_m": d["widthMax"],
                 "viewer.wind_w_max_ms": d["wMax"], "viewer.wind_refresh_s": d["refreshS"]}
        for key, v in pairs.items():
            self.assertEqual(record.value(key), v, key)
        self.assertEqual(len(d["layerHeights"]), 2)


class PanelTest(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.page = (VIEWER / "index.html").read_text(encoding="utf-8")

    def test_switches_and_defaults(self):
        self.assertIn('<script src="windvis.js"></script>', self.page)
        for sid, on in (("wdAc", True), ("wdL2", True), ("wdL5", False), ("ptc", False)):
            tag = re.search(r'<input id="%s"[^>]*>' % sid, self.page).group(0)
            self.assertEqual(" checked" in tag, on, sid)
        self.assertEqual(re.findall(r'<option value="([^"]+)"', re.search(r'<select id="flCam".*?</select>', self.page).group(0)),
                         ["free", "chase", "fwd", "tip"])
        for gone in ('id="flChase"', 'id="wdSec"', 'id="wdStream"', 'id="wdFrame"', 'id="wdDir"', "streamlines"):
            self.assertNotIn(gone, self.page)

    def test_every_new_text_in_both_languages(self):
        ja = self.page[self.page.index("  ja: {"):self.page.index("  en: {")]
        en = self.page[self.page.index("  en: {"):]
        for key in ("wdHead", "wdHeadTip", "wdAc", "wdAcTip", "wdL2", "wdL2Tip", "wdL5", "wdL5Tip", "wdLegTip",
                    "wdNoteGrid", "wdNoteAc", "wdNoteOnboard", "cam", "camTip", "camFwd", "camTipV",
                    "hudH", "hudV", "hudTip", "hudPre"):
            for block in (ja, en):
                self.assertRegex(block, r"[\s,]" + key + r": ", key)


if __name__ == "__main__":
    unittest.main()
