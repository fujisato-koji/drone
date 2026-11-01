"""The viewer's route.js against sim.route (needs node): the same routes, times, charges and energies, on the six world
cases. Python and JavaScript plan the experiment pairs of each case on a coarser graph (refine 2, one ring) with the
relaxation ladder; and Python replans a sample of the experiments of sim/out/route.json, which route.js planned on the
default graph (python -m sim.route exp), and must give their stored results."""
import json
import shutil
import subprocess
import unittest
from pathlib import Path

from sim import route as R
from sim import world as W

VIEWER = Path(__file__).resolve().parents[1] / "viewer"
NODE = shutil.which("node")
COARSE_POPS = 20000     # labels taken at most in the coarse-graph comparison
REPLAN = 10             # experiments replanned in Python
REPLAN_POPS = 30000     # ... among those whose searches took at most this many labels

SNIPPET = r"""
const path = require('path'), fs = require('fs');
const V = path.resolve(process.argv[1]);
const WD = require(path.join(V, 'worlddata.js'));
const ROUTE = require(path.join(V, 'route.js'));
const inp = JSON.parse(fs.readFileSync(0, 'utf8'));
const land = WD.decodeLand(WD.parseFile(fs.readFileSync(path.join(V, 'world', 'land.js'), 'utf8')));
let params = null;
if (inp.useParams) params = WD.parseFile(fs.readFileSync(path.join(V, 'route_params.js'), 'utf8'));
const out = [];
const cache = {};
for (const c of inp.cases) {
  const file = WD.parseFile(fs.readFileSync(path.join(V, 'world', c.id + '.js'), 'utf8'));
  const field = new ROUTE.CorridorField(WD.unpack(file.corridor));
  const lp = path.join(V, 'route', 'solar_' + c.id + '.js');
  const layer = fs.existsSync(lp) ? new ROUTE.SolarLayer(WD.parseFile(fs.readFileSync(lp, 'utf8'))) : null;
  const graph = ROUTE.corridorGraph(field, c.settings, land);
  for (const job of c.jobs) {
    const hulls = params ? params.aircraft[job.aircraft].hulls : job.hulls;
    const steady = params ? params.aircraft[job.aircraft].steady : job.steady;
    const en = params ? ROUTE.energyOf(params, job.aircraft, job.cap) : job.energy;
    const key = job.aircraft;
    const legs = cache[key] || (cache[key] = new ROUTE.LegModel(hulls, steady, c.settings));
    const tab = new ROUTE.Tables(graph, field, en, layer, { settings: c.settings });
    for (const p of job.pairs) {
      const o = { layer: layer, tables: tab, settings: c.settings };
      if (p.length > 2) o.t0 = p[2];
      if (c.settings && c.settings.max_pops) { o.maxPops = c.settings.max_pops; o.relaxPops = c.settings.relax_pops; }
      const r = ROUTE.plan(field, graph, legs, en, p[0], p[1], o);
      const rt = r.route || r.closest;
      out.push({ ok: r.ok, reason: r.reason, relaxation: r.closest ? r.closest.relaxation : null,
        nodes: rt ? rt.waypoints.map(w => w.node) : null, times: rt ? rt.waypoints.map(w => w.t) : null,
        socs: rt ? rt.waypoints.map(w => w.soc_wh) : null, summary: rt ? rt.summary : null,
        pops: r.stats ? r.stats.pops : null, startNeed: r.start_need_wh === undefined ? null : r.start_need_wh });
    }
  }
}
process.stdout.write(JSON.stringify(out, (k, v) => (typeof v === 'number' && !Number.isFinite(v)) ? String(v) : v));
"""

SUMMARY = ("time_h", "motor_wh", "takeoff_wh", "draw_wh", "solar_wh", "soaring_h", "motor_h", "wait_h", "waits",
           "min_soc_wh", "soc_end_wh")


def run_node(inp):
    r = subprocess.run([NODE, "-e", SNIPPET, str(VIEWER)], input=json.dumps(inp), capture_output=True, text=True,
                       timeout=1800)
    if r.returncode:
        raise AssertionError(r.stderr[-3000:])
    return json.loads(r.stdout)


def py_row(r):
    rt = r["route"] or r.get("closest")
    return {"ok": r["ok"], "reason": r["reason"], "relaxation": r["closest"]["relaxation"] if r.get("closest") else None,
            "nodes": [w["node"] for w in rt["waypoints"]] if rt else None,
            "times": [w["t"] for w in rt["waypoints"]] if rt else None,
            "socs": [w["soc_wh"] for w in rt["waypoints"]] if rt else None, "summary": rt["summary"] if rt else None,
            "pops": r["stats"]["pops"] if r.get("stats") else None}


@unittest.skipUnless(NODE, "node not available")
class TestRouteJs(unittest.TestCase):
    def check(self, py, js, label):
        self.assertEqual(len(py), len(js))
        for k, (p, j) in enumerate(zip(py, js)):
            msg = f"{label} row {k}"
            self.assertEqual(p["ok"], j["ok"], msg)
            self.assertEqual(p["reason"], j["reason"], msg)
            self.assertEqual(p["relaxation"], j["relaxation"], msg)
            self.assertEqual(p["nodes"], j["nodes"], msg)
            if p["pops"] is not None:
                self.assertEqual(p["pops"], j["pops"], msg)
            if p["times"]:
                for a, b in zip(p["times"], j["times"]):
                    self.assertAlmostEqual(a, b, delta=1e-6, msg=msg)
                for a, b in zip(p["socs"], j["socs"]):
                    self.assertAlmostEqual(a, b, delta=1e-8, msg=msg)
                for f in SUMMARY:
                    self.assertAlmostEqual(p["summary"][f], j["summary"][f], delta=1e-8, msg=f"{msg} {f}")

    def test_coarse_graph_python_and_js(self):
        """The experiment pairs of the six cases on a coarser graph, both languages with the same caps on the labels
        taken (so that a capped search stops at the same label)."""
        st = {"refine": 2, "rings": 1, "max_pops": COARSE_POPS, "relax_pops": COARSE_POPS}
        cases, py = [], []
        for s in W.SCENARIOS:
            field = R.CorridorField.from_case(s["id"])
            layer = R.SolarLayer.load(s["id"])
            graph = R.corridor_graph(field, refine=2, rings=1)
            pairs, _ = R.exp_pairs(field.f)
            jobs = []
            for a, cap in (("albatross-3m", 1.0), ("baseline-2m", 2.0)):
                en = R.energy_defaults(a, cap)
                lm = R.leg_model(a)
                tab = R.Tables(graph, field, en, layer)
                for _, p0, p1 in pairs:
                    py.append(py_row(R.plan(field, graph, lm, en, p0, p1, layer=layer, tables=tab,
                                            max_pops=COARSE_POPS, relax_pops=COARSE_POPS)))
                hulls = [{"u10": u, "points": [dict(vx=vx, vy=vy, power=pw, kind=kd, key=key, side=sd)
                                               for vx, vy, pw, kd, key, sd in pts]} for u, pts in R.hull_points(a)]
                jobs.append({"aircraft": a, "cap": cap, "energy": en, "hulls": hulls, "steady": R.steady_params(a),
                             "pairs": [[list(p0), list(p1)] for _, p0, p1 in pairs]})
            cases.append({"id": s["id"], "settings": st, "jobs": jobs})
        js = run_node({"useParams": False, "cases": cases})
        self.check(py, js, "coarse")
        self.assertTrue(any(p["ok"] for p in py))
        self.assertTrue(any(not p["ok"] for p in py))

    def test_experiments_replanned_in_python(self):
        """sim/out/route.json is planned in node (python -m sim.route exp): Python replans the experiments whose
        searches took the fewest labels (up to REPLAN of them, from every case that has such) and must give the
        stored routes, label counts, times and charges."""
        if not R.OUT_JSON.exists():
            self.skipTest("no sim/out/route.json")
        d = json.loads(R.OUT_JSON.read_text(encoding="utf-8"))

        def pops(r):
            return (r.get("stats") or {}).get("pops", 0) + ((r.get("closest") or {}).get("stats") or {}).get("pops", 0)

        rows = sorted((r for r in d["rows"] if r["reason"] != 1), key=lambda r: (pops(r), r["case"], r["pair"]))
        pick, seen = [], set()
        for r in rows:
            if len(pick) >= REPLAN or pops(r) > REPLAN_POPS:
                break
            if (r["case"], r["ok"]) in seen and len(seen) < 6:
                continue
            seen.add((r["case"], r["ok"]))
            pick.append(r)
        self.assertGreater(len(pick), 3)
        self.assertTrue(any(r["ok"] for r in pick))
        graphs = {}
        stored, mine = [], []
        for r in pick:
            c = r["case"]
            if c not in graphs:
                field = R.CorridorField.from_case(c)
                graphs[c] = (field, R.SolarLayer.load(c), R.corridor_graph(field))
            field, layer, graph = graphs[c]
            en = R.energy_defaults(r["aircraft"], r["capacity_factor"])
            p = R.plan(field, graph, R.leg_model(r["aircraft"]), en, tuple(r["start"]), tuple(r["dest"]), t0=r["t0"],
                       layer=layer)
            stored.append(py_row(r))
            mine.append(py_row(p))
        self.check(stored, mine, "replanned")

    def test_params_file_matches_python(self):
        p = W.load_js(VIEWER / "route_params.js")
        for a in R.AIRCRAFT:
            en = R.energy_defaults(a)
            for k, v in en.items():
                self.assertAlmostEqual(p["aircraft"][a]["energy"][k], v, places=12, msg=k)
            self.assertEqual(p["aircraft"][a]["steady"], R.steady_params(a))
            self.assertEqual([h["u10"] for h in p["aircraft"][a]["hulls"]], [u for u, _ in R.hull_points(a)])


if __name__ == "__main__":
    unittest.main()
