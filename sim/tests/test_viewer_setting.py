"""The viewer's sea and wind setting (section 1 of sim/viewer/index.html): two settings, the swell types, the launch at
the mast, and the snap between the irregular swell and the regular swell of the wave library (envsea.js, needs node)."""
import json
import re
import shutil
import subprocess
import unittest
from pathlib import Path

from sim.wavelib import SWELL_DIRS, swell_env

VIEWER = Path(__file__).resolve().parents[1] / "viewer"
NODE = shutil.which("node")

SNIPPET = r"""
global.window = global;
require(process.argv[1] + '/wave/index.js');
const E = require(process.argv[1] + '/envsea.js');
const inp = JSON.parse(require('fs').readFileSync(0, 'utf8'));
const g = window.WAVE_INDEX.grid;
const out = { grid: g, snap: [], back: [] };
for (const c of inp.snap) out.snap.push(E.snapRegular(c.windFrom, { hs: c.hs, tp: c.tp, fromDeg: c.swFrom }, g));
for (const c of inp.back) out.back.push(E.irregularOf(c.windFrom, { hs: c.hs, tp: c.tp, direction: c.direction }, g));
console.log(JSON.stringify(out));
"""


def run_node(inp):
    res = subprocess.run([NODE, "-e", SNIPPET, str(VIEWER)], input=json.dumps(inp),
                         capture_output=True, text=True, timeout=60)
    if res.returncode != 0:
        raise RuntimeError(res.stderr)
    return json.loads(res.stdout)


def options(page, select_id):
    m = re.search(r'<select id="%s"[^>]*>(.*?)</select>' % select_id, page, re.S)
    return re.findall(r'<option value="([^"]+)"', m.group(1)) if m else None


class PanelTest(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.page = (VIEWER / "index.html").read_text(encoding="utf-8")

    def test_two_settings_and_three_swells(self):
        self.assertEqual(options(self.page, "envMode"), ["free", "world"])
        self.assertEqual(options(self.page, "swType"), ["none", "irregular", "regular"])
        self.assertIn('id="vary"', self.page)

    def test_removed_controls(self):
        """The scenario's hour and playback, the separate varying and swell modes, and the launch-point choice are gone."""
        for word in ('id="hour"', 'id="play"', 'id="rate"', 'id="swOn"', 'id="flPos"', 'id="flPlace"',
                     'value="scenario"', 'value="varying"', 'value="swell"', "setHour", "FL.pt", "needClick"):
            self.assertNotIn(word, self.page)

    def test_japanese_texts_have_no_middle_dots_or_dashes(self):
        ja = self.page[self.page.index("  ja: {"):self.page.index("  en: {")]
        for ch in "・—–―":
            self.assertNotIn(ch, ja)


@unittest.skipUnless(NODE, "node not on PATH")
class SnapTest(unittest.TestCase):
    def test_snap_and_back(self):
        cases = [  # wind from, irregular swell (hs, tp, from), the expected regular swell
            (270, 1.0, 11.0, 200, (1.5, 10, "across")),   # the sliders' defaults: ties go to the first grid point
            (270, 2.0, 9.0, 270, (2, 8, "along")),
            (270, 3.6, 18.0, 90, (3, 12, "against")),
            (10, 2.6, 12.5, 280, (3, 12, "across")),
            (0, 2.0, 10.0, 90, (2, 10, "along")),         # a swell to the right: along and against tie, along first
        ]
        back = [{"windFrom": w, "hs": hs, "tp": tp, "direction": d}
                for w in (0, 135, 270) for hs in (1.5, 2, 3) for tp in (8, 10, 12) for d in SWELL_DIRS]
        r = run_node({"snap": [{"windFrom": c[0], "hs": c[1], "tp": c[2], "swFrom": c[3]} for c in cases], "back": back})
        for c, got in zip(cases, r["snap"]):
            self.assertEqual((got["hs"], got["tp"], got["direction"]), c[4], c)
        for c, got in zip(back, r["back"]):
            # the compass direction the swell comes from is the one sim.wavelib.swell_env makes it travel to, less 180
            env = swell_env(6.0, c["hs"], c["tp"], c["direction"], "p63", from_deg=c["windFrom"])
            self.assertAlmostEqual(got["fromDeg"], (env.trains[0].to_deg + 180.0) % 360.0, places=9)
            self.assertEqual((got["hs"], got["tp"]), (c["hs"], c["tp"]))
        # and a grid point comes back to itself
        again = run_node({"snap": [{"windFrom": c["windFrom"], "hs": g["hs"], "tp": g["tp"], "swFrom": g["fromDeg"]}
                                   for c, g in zip(back, r["back"])], "back": []})
        for c, got in zip(back, again["snap"]):
            self.assertEqual((got["hs"], got["tp"], got["direction"]), (c["hs"], c["tp"], c["direction"]))


if __name__ == "__main__":
    unittest.main()
