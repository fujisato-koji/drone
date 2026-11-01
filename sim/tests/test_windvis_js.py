"""The wind display around the aircraft (sim/viewer/windvis.js), needs node.

The arrows of the grid and of the section must be the wind function's values at their points: here the wind function
is the regular swell's oracle world (sim/viewer/mpc.js SeaWorld from wave.js swellSea, which is the viewer's own wind
function over the regular swell) and a free-setting sea of envsea.js in the same world. The headless check compares the
drawn arrows with the page's own fWind and fSurface in the three settings. Also checked: the grid's lattice, the
section's line and deviation, the streamlines against the field, the colour scale, the record's keys and the panel.
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
  const mean = (h) => (h > w.z0 ? (w.wind.ustar / w.kar) * Math.log(h / w.z0) : 0);
  const pts = V.gridPoints(c.cx, c.cy);
  const g = V.sampleGrid(pts, c.h, col, c.t);
  const grid = [];
  for (let p = 0; p < g.n; p++) grid.push([pts.x[p], pts.y[p], g.z[p], g.u[p], g.v[p], g.w[p], surf(pts.x[p], pts.y[p], c.t)].concat(wind(pts.x[p], pts.y[p], g.z[p], c.t)));
  const ex = c.dir[0], ey = c.dir[1];
  const sp = { cx: c.cx, cy: c.cy, ex: ex, ey: ey, mean: mean, wx: w.wind.ex, wy: w.wind.ey };
  const r = V.sampleSection(sp, col, c.t);
  // the same section in three pieces, as the viewer samples it
  const r3 = V.sectionLayout(sp);
  for (let i0 = 0; i0 < r3.nc; i0 += 50) V.sampleColumns(r3, col, c.t, i0, i0 + 50);
  const sec = [];
  let pieces = 0;
  for (let i = 0; i < r.nc; i++) for (let j = 0; j < r.nl; j++) {
    const k = i * r.nl + j, z = r.eta[i] + r.levels[j], q = wind(r.x[i], r.y[i], z, c.t);
    sec.push([r.s[i], r.x[i], r.y[i], r.levels[j], r.eta[i], surf(r.x[i], r.y[i], c.t), r.u[k], r.v[k], r.w[k], r.dev[k], mean(r.levels[j])].concat(q));
    pieces = Math.max(pieces, Math.abs(r3.u[k] - r.u[k]), Math.abs(r3.w[k] - r.w[k]), Math.abs(r3.dev[k] - r.dev[k]));
  }
  // streamlines: every step against the interpolated velocity at its midpoint
  const lines = V.streamlines(r, c.vs), qv = [0, 0, 0];
  let worstAngle = 0, outside = 0, pts2 = 0;
  for (const ln of lines) {
    for (let n = 1; n < ln.s.length; n++) {
      const ds = ln.s[n] - ln.s[n - 1], dz = ln.z[n] - ln.z[n - 1];
      if (!V.velocityAt(r, c.vs, 0.5 * (ln.s[n] + ln.s[n - 1]), 0.5 * (ln.z[n] + ln.z[n - 1]), qv)) { outside++; continue; }
      const cr = Math.abs(ds * qv[1] - dz * qv[0]) / (Math.hypot(ds, dz) * Math.hypot(qv[0], qv[1]));
      const dot = ds * qv[0] + dz * qv[1];
      worstAngle = Math.max(worstAngle, dot > 0 ? Math.asin(Math.min(1, cr)) : Math.PI);
      pts2++;
    }
  }
  out.cases.push({ name: c.name, grid: grid, d: pts.d, n: pts.n, sec: sec, ex: ex, ey: ey, wex: w.wind.ex, wey: w.wind.ey,
    pieces: pieces, nLines: lines.length, worstAngle: worstAngle, outside: outside, segs: pts2,
    firstS: lines.map((l) => [l.s[0], l.s[l.s.length - 1], l.restart, l.z[0] - V.etaAt(r, l.s[0]) - l.h0]),
    sMin: r.s[0], sMax: r.s[r.nc - 1] });
}
out.colours = [-1, -0.5, -0.25, 0, 0.25, 0.5, 1].map((w) => V.divColor(w, 0.5, [0, 0, 0]).slice());
out.poles = V.POLES;
console.log(JSON.stringify(out));
"""

CASES = [
    # the regular swell along a 6 m/s wind, at the mast and away from it, in the wave frame (15.6 m/s along it)
    {"name": "regular mast", "world": "regular", "cx": 0.0, "cy": 0.0, "h": 10.0, "t": 3.7, "dir": [1.0, 0.0],
     "vs": 9.81 * 10 / (2 * 3.141592653589793)},
    {"name": "regular aircraft", "world": "regular", "cx": 137.3, "cy": -41.9, "h": 1.7, "t": 61.2, "dir": [1.0, 0.0],
     "vs": 0.0},
    # the free setting's sea with an irregular swell from 200 deg, the section along the wind and along the swell
    {"name": "free wind", "world": "free", "cx": -63.1, "cy": 18.4, "h": 7.3, "t": 12.3, "dir": [1.0, 0.0], "vs": 0.0},
    {"name": "free swell", "world": "free", "cx": 12.0, "cy": 5.0, "h": 4.0, "t": 30.0,
     "dir": [0.3420201433256687, 0.9396926207859084], "vs": 17.17},
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

    def test_grid_arrows_are_the_wind_at_their_points(self):
        for c, case in zip(self.out["cases"], CASES):
            d, n = c["d"], c["n"]
            self.assertEqual(n * n, len(c["grid"]))
            xs = sorted({round(g[0], 9) for g in c["grid"]})
            ys = sorted({round(g[1], 9) for g in c["grid"]})
            self.assertEqual(len(xs), n)
            self.assertEqual(len(ys), n)
            for a, b in zip(xs, xs[1:]):
                self.assertAlmostEqual(b - a, d, places=9)
            # centred on the lattice point nearest the aircraft
            self.assertLessEqual(abs(0.5 * (xs[0] + xs[-1]) - case["cx"]), d / 2)
            self.assertLessEqual(abs(0.5 * (ys[0] + ys[-1]) - case["cy"]), d / 2)
            for x, y, z, u, v, w, eta, qu, qv, qw in c["grid"]:
                self.assertAlmostEqual(z, eta + case["h"], places=12)   # the same height above the local surface
                self.assertEqual((u, v, w), (qu, qv, qw))

    def test_section_arrows_are_the_wind_less_the_mean_profile(self):
        for c, case in zip(self.out["cases"], CASES):
            ex, ey = c["ex"], c["ey"]
            p0 = -case["cx"] * ey + case["cy"] * ex
            s0 = case["cx"] * ex + case["cy"] * ey
            ss = sorted({row[0] for row in c["sec"]})
            self.assertTrue(min(abs(s - s0) for s in ss) <= 0.5 * (ss[1] - ss[0]) + 1e-9)   # through the aircraft
            for s, x, y, h, eta, surf, u, v, w, dev, mean, qu, qv, qw in c["sec"]:
                self.assertAlmostEqual(-x * ey + y * ex, p0, places=9)   # on the line through the aircraft
                self.assertEqual(eta, surf)
                self.assertEqual((u, v, w), (qu, qv, qw))
                along = qu * ex + qv * ey - mean * (c["wex"] * ex + c["wey"] * ey)
                self.assertAlmostEqual(dev, along, places=12)
            self.assertEqual(c["pieces"], 0.0)   # sampled in pieces as the viewer does: the same values

    def test_streamlines_follow_the_relative_flow(self):
        for c in self.out["cases"]:
            self.assertGreater(c["nLines"], 0, c["name"])
            self.assertEqual(c["outside"], 0, c["name"])
            self.assertLess(c["worstAngle"], 0.05, c["name"])
            ends = [s0 for s0, s1, restart, dh in c["firstS"] if not restart]
            self.assertEqual(len(ends), 12, c["name"])   # every seed starts at an end of the section
            for s0, s1, restart, dh in c["firstS"]:
                self.assertTrue(restart or abs(s0 - c["sMin"]) < 1e-9 or abs(s0 - c["sMax"]) < 1e-9, c["name"])
                self.assertAlmostEqual(dh, 0.0, places=9)   # at its seed's height above the surface
            # the lines cover the section: together they reach both of its ends
            reach = [x for s0, s1, restart, dh in c["firstS"] for x in (s0, s1)]
            self.assertLess(min(reach), c["sMin"] + 3.0, c["name"])
            self.assertGreater(max(reach), c["sMax"] - 3.0, c["name"])

    def test_wave_frame_over_the_regular_swell(self):
        """In the frame of the swell (faster than the wind) the air flows back over the crests; the forward face's air
        rises: the colour of the arrows there is warm."""
        c = self.out["cases"][0]
        firsts = c["firstS"]
        self.assertTrue(all(not restart and abs(a - c["sMax"]) < 1e-9 and b < a for a, b, restart, dh in firsts))
        # 1 m above the surface the air rises over the forward faces (which slope down ahead of a crest, the swell
        # travelling towards +s) and sinks over the backs
        rows = [r for r in c["sec"] if r[3] == 1.0]
        agree = disagree = 0
        for prev, nxt, row in zip(rows, rows[2:], rows[1:]):
            slope = (nxt[4] - prev[4]) / (nxt[0] - prev[0])
            if abs(slope) > 0.02:
                if (slope < 0) == (row[8] > 0):
                    agree += 1
                else:
                    disagree += 1
        self.assertGreater(agree, 20)
        self.assertEqual(disagree, 0)

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
                 "viewer.wind_grid_mast_h_m": d["mastH"], "viewer.wind_grid_low_h_m": d["lowH"],
                 "viewer.wind_arrow_s": d["arrowScale"], "viewer.wind_section_half_m": d["secHalf"],
                 "viewer.wind_section_top_m": d["secLevels"][-1], "viewer.wind_section_exag": d["secExag"],
                 "viewer.wind_section_cap_m": d["secCap"], "viewer.wind_w_max_ms": d["wMax"],
                 "viewer.wind_refresh_s": d["refreshS"]}
        for key, v in pairs.items():
            self.assertEqual(record.value(key), v, key)


class PanelTest(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.page = (VIEWER / "index.html").read_text(encoding="utf-8")

    def test_switches_and_defaults(self):
        self.assertIn('<script src="windvis.js"></script>', self.page)
        for sid, on in (("wdGrid", True), ("wdSec", True), ("wdLow", False), ("wdStream", False), ("ptc", False)):
            tag = re.search(r'<input id="%s"[^>]*>' % sid, self.page).group(0)
            self.assertEqual(" checked" in tag, on, sid)
        self.assertEqual(re.findall(r'<option value="([^"]+)"', re.search(r'<select id="wdDir".*?</select>', self.page).group(0)),
                         ["wind", "travel"])
        self.assertEqual(re.findall(r'<option value="([^"]+)"', re.search(r'<select id="wdFrame".*?</select>', self.page).group(0)),
                         ["sea", "wave"])
        self.assertEqual(re.findall(r'<option value="([^"]+)"', re.search(r'<select id="flCam".*?</select>', self.page).group(0)),
                         ["free", "chase", "fwd", "tip"])
        self.assertNotIn('id="flChase"', self.page)

    def test_every_new_text_in_both_languages(self):
        ja = self.page[self.page.index("  ja: {"):self.page.index("  en: {")]
        en = self.page[self.page.index("  en: {"):]
        for key in ("wdHead", "wdGridTip", "wdLowTip", "wdSecTip", "wdDirTip", "wdStreamTip", "wdFrameTip", "wdLegTip",
                    "wdNoteGrid", "wdNoteSec", "wdNoteWave", "wdNoteOnboard", "cam", "camTip", "camFwd", "camTipV",
                    "hudH", "hudV", "hudTip", "hudPre"):
            for block in (ja, en):
                self.assertRegex(block, r"[\s,]" + key + r": ", key)


if __name__ == "__main__":
    unittest.main()
