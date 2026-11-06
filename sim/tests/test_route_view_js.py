"""The route planner in the viewer's world seas (sim/viewer/routeview.js, needs node): the viewer's planning path gives
the stored results of sim/out/route.json (planned by route.js through sim/route_exp.js), in the page's code and in the
worker made from the modules' source texts; the default energy limit is the record's; a plan's steps and segments
keep its times; the 3D flight's bookkeeping; the coverage of a click; and the panel's controls and texts."""
import json
import re
import shutil
import subprocess
import unittest
from pathlib import Path

from sim import record
from sim import route as R

VIEWER = Path(__file__).resolve().parents[1] / "viewer"
NODE = shutil.which("node")

# stored experiments replanned through the viewer's path: two that keep the rules (one with a wait first, one that
# soars) and two that do not (a closest route below the reserve, and a start in too high a sea)
ROWS = [("so_summer_quiet", "upwind", "albatross-3m", 1.0), ("so_winter_low", "across", "albatross-3m", 2.0),
        ("so_summer_quiet", "upwind", "albatross-3m", 0.5), ("np_winter", "downwind", "albatross-3m", 1.0)]

SNIPPET = r"""
const path = require('path'), fs = require('fs'), vm = require('vm');
const V = path.resolve(process.argv[1]);
const WD = require(path.join(V, 'worlddata.js'));
const RV = require(path.join(V, 'routeview.js'));
const inp = JSON.parse(fs.readFileSync(0, 'utf8'));
const params = WD.parseFile(fs.readFileSync(path.join(V, 'route_params.js'), 'utf8'));
const land = WD.decodeLand(WD.parseFile(fs.readFileSync(path.join(V, 'world', 'land.js'), 'utf8')));
const planner = new RV.Planner(params, land);
const jobOf = (c) => {
  const file = WD.parseFile(fs.readFileSync(path.join(V, 'world', c.case + '.js'), 'utf8'));
  const lp = path.join(V, 'route', 'solar_' + c.case + '.js');
  const d = RV.defaults(params, c.aircraft);
  return { key: c.case, corridor: WD.unpack(file.corridor), solar: fs.existsSync(lp) ? WD.parseFile(fs.readFileSync(lp, 'utf8')) : null,
    aircraft: c.aircraft, capWh: d.capacityWh * c.factor, share: d.motorShare, start: c.start, dest: c.dest, t0: c.t0 };
};
const fin = (k, v) => (typeof v === 'number' && !Number.isFinite(v)) ? String(v) : v;
const out = { defaults: {}, plans: [], worker: null, flight: null, coverage: null };
for (const a of Object.keys(params.aircraft)) out.defaults[a] = RV.defaults(params, a);
for (const c of inp.rows) out.plans.push(planner.plan(jobOf(c)));
// the worker: the modules' source texts in a bare context, as a Blob worker runs them
const ctx = vm.createContext({ atob: atob });
const msgs = [];
ctx.self = ctx; ctx.postMessage = (m) => msgs.push(JSON.parse(JSON.stringify(m, fin)));
vm.runInContext(RV.workerSource(), ctx);
ctx.onmessage({ data: { type: 'init', params: params, land: land } });
ctx.onmessage({ data: { type: 'plan', id: 7, job: jobOf(inp.rows[inp.worker]) } });
out.worker = msgs;
// the 3D flight's bookkeeping along the first plan's segments, arriving on time
const segs = out.plans[0].route.segments, fp = new RV.FlightPlan(segs), steps = [];
let guard = 0;
while (!fp.done && guard++ < 1000) {
  if (fp.wait) { const lv = fp.leave(fp.wait.t0 + 60); steps.push(['leave', lv.t, fp.i]); continue; }
  const s = fp.seg();
  steps.push(['half', fp.planTime(500, 1000), (s.t0 + s.t1) / 2]);
  steps.push(['arrive', fp.arrive(s.t1), fp.i]);
}
out.flight = { steps: steps, late: new RV.FlightPlan(segs).leave(segs[0].t1 + 900).t };
// coverage of a click: inside, outside, on land (a square of land laid around the corridor's centre, since no case's
// grid point lies on land), and where a forecast has no values (a corridor whose values around a cell are removed)
const corOf = (id) => WD.unpack(WD.parseFile(fs.readFileSync(path.join(V, 'world', id + '.js'), 'utf8')).corridor);
const cor = corOf(inp.cov), cov = new RV.Coverage(cor, land), t0 = cor.t0;
const gap = corOf(inp.cov), i0 = (gap.na >> 1), j0 = (gap.nc >> 1), pts = WD.corridorPoints(gap), cell = [];
for (let a = -1; a <= 2; a++) for (let b = -1; b <= 2; b++) cell.push((i0 + a) * gap.nc + j0 + b);
for (const nm of Object.keys(gap.vals)) for (let k = 0; k < gap.nt; k++) for (const n of cell) gap.vals[nm][k * gap.na * gap.nc + n] = NaN;
const mid = [0, 1, gap.nc, gap.nc + 1].map((d) => i0 * gap.nc + j0 + d);
const at = [mid.reduce((s, n) => s + pts.lat[n], 0) / 4, mid.reduce((s, n) => s + pts.lon[n], 0) / 4];
const sq = [[[cor.lon - 0.5, cor.lat - 0.5], [cor.lon + 0.5, cor.lat - 0.5], [cor.lon + 0.5, cor.lat + 0.5], [cor.lon - 0.5, cor.lat + 0.5]]];
out.coverage = { inside: cov.check(cor.lat, cor.lon, t0), outside: cov.check(cor.lat + 30, cor.lon, t0),
  land: new RV.Coverage(cor, land.concat(sq)).check(cor.lat, cor.lon, t0),
  nodata: new RV.Coverage(gap, land).check(at[0], at[1], t0), gapInsideBefore: cov.check(at[0], at[1], t0) };
process.stdout.write(JSON.stringify(out, fin));
"""

SUMMARY = ("time_h", "motor_wh", "takeoff_wh", "draw_wh", "solar_wh", "soaring_h", "motor_h", "wait_h", "waits",
           "min_soc_wh", "soc_end_wh")


def run_node(inp):
    r = subprocess.run([NODE, "-e", SNIPPET, str(VIEWER)], input=json.dumps(inp), capture_output=True, text=True,
                       timeout=900)
    if r.returncode:
        raise AssertionError(r.stderr[-3000:])
    return json.loads(r.stdout)


def num(x):
    return float(x) if isinstance(x, str) else x


@unittest.skipUnless(NODE, "node not available")
class TestViewerPlanningPath(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        d = json.loads(R.OUT_JSON.read_text(encoding="utf-8"))
        cls.stored = [next(r for r in d["rows"] if (r["case"], r["pair"], r["aircraft"], r["capacity_factor"]) == key)
                      for key in ROWS]
        rows = [{"case": r["case"], "aircraft": r["aircraft"], "factor": r["capacity_factor"], "start": r["start"],
                 "dest": r["dest"], "t0": r["t0"]} for r in cls.stored]
        cls.out = run_node({"rows": rows, "worker": 1, "cov": "so_winter_low"})

    def same_route(self, a, b, msg):
        self.assertEqual([w["node"] for w in a["waypoints"]], [w["node"] for w in b["waypoints"]], msg)
        for x, y in zip(a["waypoints"], b["waypoints"]):
            self.assertAlmostEqual(x["t"], y["t"], delta=1e-6, msg=msg)
            self.assertAlmostEqual(x["soc_wh"], y["soc_wh"], delta=1e-8, msg=msg)
        for k in SUMMARY:
            self.assertAlmostEqual(a["summary"][k], b["summary"][k], delta=1e-8, msg=f"{msg} {k}")

    def test_stored_rows(self):
        """The viewer's inputs (the capacity in Wh, the share, the clicked points, the map's time) give the stored
        plans: the same result, reason, route or closest route, times and charges."""
        oks = set()
        for r, p in zip(self.stored, self.out["plans"]):
            msg = f"{r['case']} {r['pair']} {r['aircraft']} {r['capacity_factor']}"
            self.assertEqual(p["ok"], r["ok"], msg)
            self.assertEqual(p["reason"], r["reason"], msg)
            oks.add(p["ok"])
            self.assertEqual(p["route"] is None, r["route"] is None, msg)
            self.assertEqual(p["closest"] is None, r["closest"] is None, msg)
            if r["route"]:
                self.same_route(p["route"], r["route"], msg)
            if r["closest"]:
                self.assertEqual(p["closest"]["relaxation"], r["closest"]["relaxation"], msg)
                self.same_route(p["closest"], r["closest"], msg)
                for k in ("reserve_margin_wh", "night_margin_wh", "allowance_margin_wh"):
                    self.assertAlmostEqual(num(p["closest"][k]), num(r["closest"][k]), delta=1e-6, msg=f"{msg} {k}")
            self.assertAlmostEqual(p["energy"]["capacity_wh"], r["capacity_wh"], places=12, msg=msg)
            self.assertAlmostEqual(p["energy"]["reserve_wh"], r["reserve_wh"], places=12, msg=msg)
        self.assertEqual(oks, {True, False})

    def test_worker_gives_the_same_plan(self):
        """The worker made from the modules' source texts plans as the page's code does."""
        msgs = self.out["worker"]
        self.assertEqual([m["type"] for m in msgs], ["ready", "plan"])
        self.assertEqual(msgs[1]["id"], 7)
        w, p = msgs[1]["result"], self.out["plans"][1]
        self.assertEqual((w["ok"], w["reason"]), (p["ok"], p["reason"]))
        self.same_route(w["route"], p["route"], "worker")
        self.assertGreaterEqual(w["plan_ms"], 0)

    def test_steps_and_segments_keep_the_plan_times(self):
        """Each leg's steps, timed again from the field and the leg model, end at the leg's arrival; the segments
        follow one another without a gap from the start to the arrival, one kind each, and the waits are whole."""
        for r, p in zip(self.stored, self.out["plans"]):
            for rt in (p["route"], p["closest"]):
                if not rt:
                    continue
                msg = r["case"] + " " + r["pair"]
                self.assertLess(rt["step_error_s"], 1e-6, msg)
                segs = rt["segments"]
                self.assertAlmostEqual(segs[0]["t0"], p["t0"], delta=1e-6, msg=msg)
                self.assertAlmostEqual(segs[-1]["t1"], rt["waypoints"][-1]["t"], delta=1e-6, msg=msg)
                for a, b in zip(segs, segs[1:]):
                    self.assertAlmostEqual(a["t1"], b["t0"], delta=1e-6, msg=msg)
                    self.assertFalse(a["kind"] == "wait" and b["kind"] == "wait", msg)
                for s in segs:
                    self.assertIn(s["kind"], ("soaring", "motor", "wait"), msg)
                    if s["kind"] != "wait":
                        self.assertGreaterEqual(len(s["pts"]), 2, msg)
                wait_h = sum(s["t1"] - s["t0"] for s in segs if s["kind"] == "wait") / 3600.0
                self.assertAlmostEqual(wait_h, rt["summary"]["wait_h"], delta=1e-9, msg=msg)
                low = min([w["soc_wh"] for w in rt["waypoints"]] + [lg["soc_min_wh"] for lg in rt["legs"]])
                self.assertAlmostEqual(rt["lowest_soc_wh"], low, delta=1e-12, msg=msg)
        kinds = {s["kind"] for p in self.out["plans"] if p["route"] for s in p["route"]["segments"]}
        self.assertEqual(kinds, {"soaring", "motor", "wait"})
        ok = self.out["plans"][0]["route"]
        self.assertGreaterEqual(ok["margins"]["night_wh"], -1e-6)
        self.assertGreaterEqual(ok["margins"]["reserve_wh"], -1e-6)

    def test_flight_bookkeeping(self):
        """The 3D flight's segments in turn: a wait first, the launch at its end (or later when 3D is late), the plan's
        time halfway along a segment, and the end at the destination."""
        segs = self.out["plans"][0]["route"]["segments"]
        steps = self.out["flight"]["steps"]
        self.assertEqual(segs[0]["kind"], "wait")
        self.assertEqual(steps[0][0], "leave")
        self.assertEqual(steps[0][1], segs[0]["t1"])
        self.assertEqual(self.out["flight"]["late"], segs[0]["t1"] + 900)
        arrivals = [s for s in steps if s[0] == "arrive"]
        self.assertEqual(arrivals[-1][1], "end")
        self.assertEqual(len(arrivals), sum(1 for s in segs if s["kind"] != "wait"))
        for s in steps:
            if s[0] == "half":
                self.assertAlmostEqual(s[1], s[2], delta=1e-6)

    def test_coverage(self):
        c = self.out["coverage"]
        self.assertEqual(c["inside"], "ok")
        self.assertEqual(c["outside"], "outside")
        self.assertEqual(c["gapInsideBefore"], "ok")
        self.assertEqual(c["nodata"], "nodata")
        self.assertEqual(c["land"], "land")


class TestDefaults(unittest.TestCase):
    def test_default_energy_limit_is_the_record(self):
        """The fields' defaults come from route_params.js, which holds sim.route's defaults: the usable capacity of
        D-023's battery (route.capacity_wh) and the motor's share of D-021 (route.motor_share)."""
        from sim.world import load_js
        p = load_js(VIEWER / "route_params.js")
        for a in R.AIRCRAFT:
            e = p["aircraft"][a]["energy"]
            self.assertAlmostEqual(e["capacity_wh"], R.energy_defaults(a)["capacity_wh"], places=12)
            self.assertAlmostEqual(e["capacity_wh"], record.value("route.capacity_wh"), delta=0.05)
            self.assertEqual(e["motor_share"], record.value("route.motor_share"))
            self.assertEqual(e["reserve_fraction"], record.value("route.reserve_fraction"))

    @unittest.skipUnless(NODE, "node not available")
    def test_viewer_defaults(self):
        snippet = ("const path=require('path'),fs=require('fs');const V=process.argv[1];"
                   "const WD=require(path.join(V,'worlddata.js')),RV=require(path.join(V,'routeview.js'));"
                   "const p=WD.parseFile(fs.readFileSync(path.join(V,'route_params.js'),'utf8'));"
                   "const o={};for(const a of Object.keys(p.aircraft)){o[a]=[RV.defaults(p,a),RV.energyFor(p,a,RV.defaults(p,a).capacityWh,null)];}"
                   "console.log(JSON.stringify(o));")
        r = subprocess.run([NODE, "-e", snippet, str(VIEWER)], capture_output=True, text=True, timeout=60)
        self.assertEqual(r.returncode, 0, r.stderr)
        for a, (d, en) in json.loads(r.stdout).items():
            ref = R.energy_defaults(a)
            self.assertAlmostEqual(d["capacityWh"], ref["capacity_wh"], places=12)
            self.assertEqual(d["motorShare"], record.value("route.motor_share"))
            for k in ("capacity_wh", "reserve_wh", "soc0_wh", "motor_share"):
                self.assertAlmostEqual(en[k], ref[k], places=12, msg=k)


class TestPanel(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.page = (VIEWER / "index.html").read_text(encoding="utf-8")

    def test_controls(self):
        for sid in ("rtPick", "rtCap", "rtShare", "rtReset", "rtPlan", "rtCancel", "rtGo", "rtSkip", "rtro", "rtfl",
                    "rtWait", "rtBusy", "rtMsg"):
            self.assertIn('id="%s"' % sid, self.page, sid)
        m = re.search(r'<select id="rtPick".*?</select>', self.page)
        self.assertEqual(re.findall(r'<option value="([^"]+)"', m.group(0)), ["place", "start", "dest"])
        for src in ("route_params.js", "route.js", "routeview.js"):
            self.assertIn('<script src="%s"></script>' % src, self.page)
        # the energy fields carry no value of their own: the page fills them from route_params.js
        for sid in ("rtCap", "rtShare"):
            tag = re.search(r'<input id="%s"[^>]*>' % sid, self.page).group(0)
            self.assertNotIn("value=", tag)

    def test_every_new_text_in_both_languages(self):
        ja = self.page[self.page.index("  ja: {"):self.page.index("  en: {")]
        en = self.page[self.page.index("  en: {"):]
        keys = re.findall(r"data-[it]=\"(rt[A-Za-z_]*)\"", self.page)
        keys += ["help7", "rtPtStart", "rtPtDest", "rtNone", "rtOut_outside", "rtOut_land", "rtOut_nodata", "rtOut_none",
                 "rtGoNo", "rtBusy", "rtTook", "rtNeedPts", "rtAcMissing", "rtBadEnergy", "rtNoWorker", "rtErr",
                 "rtCancelled", "rtStale", "rtNoParams", "rtLegSoar", "rtLegMotor", "rtLegWait", "rtLegEnds",
                 "rtLegClosest", "rsOk", "rsFail", "rsIn", "rsArr", "rsDist", "rsShare", "rsSoc", "rsNightOk",
                 "rsNightNo", "rsEnergy", "rsSnap", "rsSolarLayer", "rsSolarModel", "rsNeed", "rsRelax1", "rsRelax2",
                 "rsShort", "rfSeg", "rfClock", "rfWaitPlan", "rfLate", "rfEarly", "rfOn", "rfDone", "rfEnded", "endW",
                 "endX", "rtWaitTxt", "rtWaitWhy", "rtWaitSoc", "rtWaitSlack", "rtWaitMid", "rtWaitNight",
                 "rtWaitSea", "rtWaitWind"]
        keys += ["rs_" + k for k in ("res", "in", "arr", "dist", "share", "soc", "night", "energy", "snap", "solar",
                                     "need", "close", "short")]
        keys += ["rf_" + k for k in ("seg", "clock", "plan", "diff")]
        for key in set(keys):
            for block in (ja, en):
                self.assertRegex(block, r"[\s,]" + key + r": ", key)
        for r in range(1, 9):   # route.js's reasons in Japanese (English shows route.js's own text)
            self.assertRegex(ja, r"[\s,]rtR%d: " % r)
        for ch in "・—–―":
            self.assertNotIn(ch, ja)


if __name__ == "__main__":
    unittest.main()
