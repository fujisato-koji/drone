"""The viewer's navigator.js (velocity hull and hull navigator) against sim.navigator, and a closed loop
in node. Needs node (skipped without it). Both sides get the same hull points (full precision, from
JSON), so decisions must agree exactly (strings) and numbers to rounding."""
import json
import math
import shutil
import subprocess
import unittest
from pathlib import Path

import numpy as np

from sim.aircraft_models import by_name
from sim.navigator import (HullNavigator, HullPoint, HullSource, SteadyModel, VelocityHull, sail_hull)
from sim.tests.test_navigator_hull import motor_points, soaring_points

VIEWER = Path(__file__).resolve().parents[1] / "viewer"
NAV_JS = VIEWER / "navigator.js"
NODE = shutil.which("node")

SNIPPET = r"""
const NAV = require(process.argv[1]);
const inp = JSON.parse(require('fs').readFileSync(0, 'utf8'));
const src = NAV.HullSource.fromPoints(inp.hulls);
const out = {steps: [], sails: [], plans: [], legs: []};
const mkOpt = (c) => { const o = Object.assign({}, c.opt); if (c.steady) o.steady = new NAV.SteadyModel(c.steady); return o; };
inp.legs.forEach(l => {
  const m = new NAV.SteadyModel(l.model);
  const r = m.leg(l.uz, l.tau, 0.3);
  out.legs.push(r);
});
inp.plans.forEach(p => {
  const h = src.at(p.u10);
  const r = h.planPair(p.d, 0.3, p.motorOk);
  out.plans.push(r ? {i: r.i, j: r.j, r: r.r, perM: r.perM, kind: r.kind} : null);
});
inp.steps.forEach(c => {
  const nav = new NAV.HullNavigator(src, mkOpt(c));
  const res = [];
  c.calls.forEach(k => {
    const cmd = nav.step(k.t, k.pos, k.u10, k.wf, k.target);
    res.push({key: cmd.point.key, V: cmd.point.steady ? cmd.point.steady.V : 0, hd: cmd.point.steady ? cmd.point.steady.heading : null, mode: cmd.mode, tacked: cmd.tacked, bearing: cmd.bearing, vmg: cmd.vmg,
      cross: cmd.cross, corridor: cmd.corridor, dist: cmd.distance});
  });
  out.steps.push(res);
});
inp.sails.forEach(c => {
  const nav = new NAV.HullNavigator(src, mkOpt(c));
  const r = NAV.sailHull(src, () => [c.u10, c.wf], c.start, c.target, nav, {dt: 1, tMax: c.tMax});
  out.sails.push({arrived: r.arrived, time: r.time, nLegs: r.nLegs, switchTimes: r.switchTimes,
    energyJ: r.energyJ, modes: r.modes.map(m => m[1]), end: r.track[r.track.length - 1]});
});
process.stdout.write(JSON.stringify(out));
"""


def model_json(m):
    return dict(mass=m.mass, wingArea=m.wing_area, cd0=m.cd0, kInduced=m.k_induced, rho=m.rho, clMax=m.cl_max,
                vStall=m.v_stall, vBestLd=m.v_best_ld, tMax=m.t_max, eta=m.eta, g=m.g)


def points_json(pts):
    return [dict(vx=p.vx, vy=p.vy, power=p.power, kind=p.kind, key=p.key, side=p.side) for p in pts]


def lib_points():
    """Hull points of the motor library at its winds (albatross-3m), if the file exists."""
    from sim.mpc import LIB_MOTOR_PATH
    try:
        with open(LIB_MOTOR_PATH) as f:
            ents = json.load(f)["entries"]
    except OSError:
        return []
    src = HullSource.from_entries(ents, "albatross-3m")
    return [(h.u10, h.points) for h in src.hulls]


@unittest.skipUnless(NODE, "node not available")
class TestNavigatorJs(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        hulls = [(16.0, soaring_points() + motor_points()), (13.0, soaring_points()[:6])]
        hulls += lib_points()
        cls.hulls = hulls
        cls.src = HullSource([VelocityHull(p, u) for u, p in hulls])
        rng = np.random.default_rng(3)
        cls.model = SteadyModel.from_aircraft(by_name("albatross-3m"))
        mj = model_json(cls.model)
        plans, steps, sails, legs = [], [], [], []
        for _ in range(40):
            a = rng.uniform(0, 2 * math.pi)
            legs.append(dict(model=mj, uz=float(rng.uniform(3, 24)), tau=[math.cos(a), math.sin(a)]))
        for _ in range(60):
            a = rng.uniform(0, 2 * math.pi)
            plans.append(dict(u10=float(rng.choice([u for u, _ in hulls])), d=[math.cos(a), math.sin(a)],
                              motorOk=bool(rng.integers(0, 2))))
        for k in range(12):
            u = float(hulls[k % len(hulls)][0])
            wf = float(rng.uniform(0, 360))
            tgt = [float(rng.uniform(-6000, 6000)), float(rng.uniform(-6000, 6000))]
            pos = [0.0, 0.0]
            calls, t = [], 0.0
            use = bool(k % 2)
            nav = HullNavigator(cls.src, arrival_radius=100.0, steady=cls.model if use else None)
            for _ in range(80):
                calls.append(dict(t=t, pos=list(pos), u10=u, wf=wf, target=tgt))
                c = nav.step(t, pos, u, wf, tgt)
                w = HullNavigator.to_world(HullNavigator._psi(wf), (c.point.vx, c.point.vy))
                pos = [pos[0] + 5 * w[0], pos[1] + 5 * w[1]]
                t += 5.0
            steps.append(dict(opt=dict(arrivalRadius=100.0), calls=calls, steady=mj if use else None))
        for k in range(8):
            u = float(hulls[k % len(hulls)][0])
            wf = float(rng.uniform(0, 360))
            tgt = [float(rng.uniform(-7000, 7000)), float(rng.uniform(-7000, 7000))]
            sails.append(dict(opt=dict(arrivalRadius=100.0), u10=u, wf=wf, start=[0.0, 0.0], target=tgt,
                              tMax=3000, steady=mj if k % 2 else None))
        cls.cases = dict(plans=plans, steps=steps, sails=sails, legs=legs)
        inp = dict(hulls=[dict(u10=u, points=points_json(p)) for u, p in hulls], **cls.cases)
        r = subprocess.run([NODE, "-e", SNIPPET, str(NAV_JS)], input=json.dumps(inp), capture_output=True,
                           text=True, timeout=300)
        if r.returncode != 0:
            raise RuntimeError(r.stderr)
        cls.js = json.loads(r.stdout)

    def test_plans_agree(self):
        for p, j in zip(self.cases["plans"], self.js["plans"]):
            h = self.src.at(p["u10"])
            py = h.plan_pair(tuple(p["d"]), 0.3, p["motorOk"])
            if py is None:
                self.assertIsNone(j)
                continue
            self.assertEqual((py["i"], py["j"], py["kind"]), (j["i"], j["j"], j["kind"]))
            self.assertAlmostEqual(py["r"], j["r"], places=9)
            self.assertAlmostEqual(py["per_m"], j["perM"], places=9)

    def test_step_decisions_agree_including_tacks(self):
        n_tack = n_mode = 0
        for case, res in zip(self.cases["steps"], self.js["steps"]):
            nav = HullNavigator(self.src, arrival_radius=100.0, steady=self.model if case["steady"] else None)
            for k, r in zip(case["calls"], res):
                c = nav.step(k["t"], k["pos"], k["u10"], k["wf"], k["target"])
                self.assertEqual(c.point.key, r["key"])
                self.assertEqual(c.mode, r["mode"])
                self.assertEqual(c.tacked, r["tacked"])
                self.assertAlmostEqual(c.bearing, r["bearing"], places=6)
                self.assertAlmostEqual(c.vmg, r["vmg"], places=6)
                self.assertAlmostEqual(c.cross, r["cross"], places=5)
                self.assertAlmostEqual(c.corridor, r["corridor"], places=5)
                n_tack += int(c.tacked)
                n_mode += 1
        self.assertGreater(n_mode, 500)
        self.assertGreater(n_tack, 0)             # the set includes decisions with a leg switch

    def test_steady_legs_agree(self):
        n = 0
        for l, j in zip(self.cases["legs"], self.js["legs"]):
            py = self.model.leg(l["uz"], tuple(l["tau"]), 0.3)
            if py is None:
                self.assertIsNone(j)
                continue
            n += 1
            for k in ("V", "cl", "thrust", "power", "along", "per_m"):
                self.assertAlmostEqual(py[k], j["perM" if k == "per_m" else k], places=9)
            self.assertAlmostEqual(py["heading"][0], j["heading"][0], places=9)
            self.assertAlmostEqual(py["heading"][1], j["heading"][1], places=9)
        self.assertGreater(n, 20)

    def test_steady_legs_aim_by_the_wind_triangle(self):
        """The ground velocity of a steady leg lies on the track and the energy is the least over the airspeeds."""
        for uz in (6.0, 13.0, 16.0):
            for a in np.radians(np.arange(0, 360, 30)):
                tau = (math.cos(a), math.sin(a))
                lg = self.model.leg(uz, tau)
                if lg is None:
                    continue
                h = lg["heading"]
                gx, gy = lg["V"] * h[0] + uz, lg["V"] * h[1]
                self.assertAlmostEqual(gx * tau[1] - gy * tau[0], 0.0, places=9)       # no cross component
                self.assertAlmostEqual(gx * tau[0] + gy * tau[1], lg["along"], places=9)
                self.assertLessEqual(lg["thrust"], 0.8 * self.model.t_max + 1e-9)

    def test_kinematic_runs_agree(self):
        for c, j in zip(self.cases["sails"], self.js["sails"]):
            nav = HullNavigator(self.src, arrival_radius=100.0, steady=self.model if c["steady"] else None)
            r = sail_hull(self.src, lambda t: (c["u10"], c["wf"]), c["start"], c["target"], nav=nav, dt=1.0,
                          t_max=c["tMax"])
            self.assertEqual(r["arrived"], j["arrived"])
            self.assertEqual(r["n_legs"], j["nLegs"])
            self.assertEqual([m for _, m in r["modes"]], j["modes"])
            self.assertAlmostEqual(r["energy_j"], j["energyJ"], places=4)
            self.assertLess(float(np.hypot(*(r["track"][-1] - np.array(j["end"])))), 1e-5)


if __name__ == "__main__":
    unittest.main()


LOOP = r"""
const MPC = require(process.argv[1]);
const Flight = require(process.argv[2]);
const NAV = require(process.argv[3]);
const inp = JSON.parse(require('fs').readFileSync(0, 'utf8'));
const ac = MPC.makeAircraft(inp.ac), w = inp.wind, K = 0.4;
const trueWorld = new MPC.SeaWorld({ustar: w.ustar, z0: w.z0, ex: w.ex, ey: w.ey, karman: K}, null, false);
const windFn = (x, y, z, t) => { const o = [0, 0, 0]; trueWorld.windAt(x, y, z, t, o); return o; };
windFn.z0 = w.z0;
const entries = inp.entries.filter(e => e.vx !== undefined && (inp.motor || e.kind === 'soaring'));
const src = new NAV.HullSource(entries);
const nav = new NAV.HullNavigator(src, {arrivalRadius: 200,
  steady: inp.motor ? new NAV.SteadyModel(NAV.steadyParams(ac, MPC.MOTOR)) : null});
const psiW = Math.atan2(w.ey, w.ex), windFrom = -90 - psiW * 180 / Math.PI;
const ctl = new MPC.Controller(inp.ac, entries, {actuators: inp.actuators, settings: inp.settings});
const act = new MPC.Actuators(ac, {rateModel: inp.actuators.rate_model, tau: inp.actuators.tau_s,
  clRate: inp.actuators.cl_rate_per_s, kPb2v: inp.actuators.k_pb2v, delay: inp.actuators.delay_s});
const tgt = inp.target;
let cmd = nav.step(0, [0, 0], inp.u10, windFrom, tgt);
const c0 = new MPC.Cycle(cmd.point.steady ? NAV.steadyEntry(cmd.point) : cmd.point.entry, cmd.point.side < 0);
let k0 = 0;
for (let i = 1; i < c0.z.length; i++) if (c0.z[i] > c0.z[k0]) k0 = i;
let s = [0, 0, c0.z[k0], c0.V[k0], c0.g[k0], psiW + c0.psi[k0]];
const dt = 0.01, n = Math.round(inp.T / dt), every = Math.round(0.5 / dt), navEvery = Math.round(5 / dt);
const motor = !!inp.settings.motor;
const foldDcd = MPC.MOTOR.foldedCdaM2 / ac.wing_area, tMax = MPC.MOTOR.thrustMaxPerWeight * ac.mass * 9.81;
let plan = null, tPlan = 0, ended = 'complete', tEnd = inp.T, hmin = Infinity, eJ = 0, tA = 0, legs = 0;
const modes = {};
for (let i = 0; i <= n; i++) {
  const t = i * dt;
  const h = s[2] - trueWorld.eta(s[0], s[1], t);
  hmin = Math.min(hmin, h);
  if (h < 0) { ended = 'contact'; tEnd = t; break; }
  if (!s.every(Number.isFinite)) { ended = 'non-finite'; tEnd = t; break; }
  if (Math.hypot(tgt[0] - s[0], tgt[1] - s[1]) <= 200) { ended = 'arrived'; tEnd = t; break; }
  if (i % navEvery === 0 && i > 0) {
    cmd = nav.step(t, [s[0], s[1]], inp.u10, windFrom, tgt);
    if (cmd.tacked) legs++;
    modes[cmd.mode] = (modes[cmd.mode] || 0) + 5;
  }
  if (plan === null || i % every === 0) {
    const ck = cmd.point.steady ? null : {kind: cmd.point.kind, u10: cmd.point.entry.u10, chi_deg: cmd.point.entry.chi_deg, mirror: cmd.point.side < 0};
    plan = ctl.plan({t: t, s: s.slice(), act: act.cl === null ? null :
      {cl: act.cl, mu: act.mu, clCmd: act.cmdLast[0], muCmd: act.cmdLast[1], T: act.T},
      world: {kind: 'flat', wind: {ustar: w.ustar, z0: w.z0, ex: w.ex, ey: w.ey, karman: K}, comps: null, stokes: false, maxComponents: 48},
      u10: inp.u10, chiDeg: 20, cycle: ck, entry: cmd.point.steady ? NAV.steadyEntry(cmd.point) : undefined, guideSig: [3.0, 3.0, 0.15, 0.1]});
    tPlan = t;
  }
  let [cl, mu] = plan.controls(t - tPlan);
  cl = Math.min(Math.max(cl, ac.cl_min), ac.cl_max); mu = Math.min(Math.max(mu, -ac.mu_max), ac.mu_max);
  cl = MPC.nLimitedCl(ac, s[3], cl);
  let clA, muA;
  if (motor) {
    const tCmd = Math.min(Math.max(MPC.planThrust(plan, t - tPlan), 0), tMax);
    [clA, muA] = act.step(cl, mu, dt, s[3], tCmd);
    tA = act.T;
    eJ += tA * s[3] * dt / (MPC.MOTOR.propEff * MPC.MOTOR.motorEff);
  } else [clA, muA] = act.step(cl, mu, dt, s[3]);
  if (i === n) break;
  s = motor ? Flight.rk4(s, clA, muA, ac, windFn, t, dt, true, tA, foldDcd) : Flight.rk4(s, clA, muA, ac, windFn, t, dt, true);
}
console.log(JSON.stringify({ended: ended, tEnd: tEnd, hmin: hmin, legs: legs, eWh: eJ / 3600, modes: modes,
  end: [s[0], s[1]], dist: Math.hypot(tgt[0] - s[0], tgt[1] - s[1])}));
"""


@unittest.skipUnless(NODE, "node not available")
class TestNavigatorLoopJs(unittest.TestCase):
    """A closed loop in node: albatross-3m at 16 m/s on a flat sea flies to a destination 1 km upwind
    (it needs the motor: soaring cycles never fly upwind) through the hull navigator, the worker's
    Controller (cycle named by the navigator) and the true dynamics."""

    def run_loop(self, target, motor, T):
        from sim.aircraft_models import by_name
        from sim.seawind import Wind
        from sim.tests.test_mpc_js import ac_params, wind_dict
        lib_js = (VIEWER / "library_motor.js").read_text(encoding="utf-8")
        lib = json.loads(lib_js.split("window.LIBRARY_MOTOR = ", 1)[1].rsplit(";", 1)[0])
        name = "albatross-3m"
        settings = {"motor": True, "energyLevel": "normal", "thrustSampling": "sustained",
                    "foldSwitch": "state", "thrustUpdate": "raw"} if motor else {}
        w = Wind(16.0, 270.0, roughness="coare35")
        payload = dict(ac=ac_params(by_name(name)), settings=settings, actuators=lib["actuators"],
                       entries=lib["aircraft"][name], u10=16.0, wind=wind_dict(w), target=target, T=T,
                       motor=motor)
        r = subprocess.run([NODE, "-e", LOOP, str(VIEWER / "mpc.js"), str(VIEWER / "flight.js"), str(NAV_JS)],
                           input=json.dumps(payload), capture_output=True, text=True, timeout=1500)
        if r.returncode != 0:
            raise RuntimeError(r.stderr)
        return json.loads(r.stdout)

    def test_one_km_upwind_with_the_motor(self):
        """Needs the motor: the navigator flies a steady level leg at 10 m aimed by the wind triangle and the
        planner holds it (about 8 m/s made good at 16 m/s wind)."""
        r = self.run_loop([-1000.0, 0.0], True, 400.0)
        print(chr(10) + "node closed loop, albatross-3m 16 m/s, 1 km upwind, motor: %s at %.0f s, min height %.2f m, "
              "%d leg changes, %.2f Wh, end distance %.0f m, modes %s"
              % (r["ended"], r["tEnd"], r["hmin"], r["legs"], r["eWh"], r["dist"], r["modes"]))
        self.assertEqual(r["ended"], "arrived")
        self.assertGreater(r["hmin"], 0.0)

    def test_one_km_downwind_soaring(self):
        r = self.run_loop([1000.0, 0.0], False, 200.0)
        self.assertEqual(r["ended"], "arrived")
        self.assertLess(r["tEnd"], 100.0)

    def test_upwind_without_the_motor_makes_no_progress(self):
        r = self.run_loop([-1000.0, 0.0], False, 30.0)
        self.assertIn("no progress", r["modes"])
        self.assertGreater(r["dist"], 900.0)


LOOP_FREE = r"""
const MPC = require(process.argv[1]);
const Flight = require(process.argv[2]);
const NAV = require(process.argv[3]);
const inp = JSON.parse(require('fs').readFileSync(0, 'utf8'));
const ac = MPC.makeAircraft(inp.ac), w = inp.wind, K = 0.4;
const trueWorld = new MPC.SeaWorld({ustar: w.ustar, z0: w.z0, ex: w.ex, ey: w.ey, karman: K}, null, false);
const windFn = (x, y, z, t) => { const o = [0, 0, 0]; trueWorld.windAt(x, y, z, t, o); return o; };
windFn.z0 = w.z0;
// the viewer's table of a flight without a destination (freeTable) and its course chooser (freeStart)
const entries = inp.entries.filter(e => e.vx !== undefined && (inp.motor || e.kind === 'soaring'));
const SIG = [3.0, 3.0, 0.15, 0.1], cycs = {};   // the navigator's guide tolerances (sim.mpc.NAV_GUIDE_SIG)
const cycOf = (p) => cycs[p.key] || (cycs[p.key] = new MPC.Cycle(p.entry, p.side < 0));
const fc = new NAV.FreeCourse(new NAV.HullSource(entries),
  {steady: inp.motor ? new NAV.SteadyModel(NAV.steadyParams(ac, MPC.MOTOR)) : null,
   link: (a, b) => MPC.cycleLink(cycOf(a), cycOf(b), SIG), linkMax: MPC.SWITCH_MATCH_MAX / 2});   // viewer.course_link_frac
const flownOf = (f) => !f ? null : f.marginSet === 'steady' ? 'steady' : {kind: f.kind, u10: f.u10, chi_deg: f.chiDeg, mirror: f.mirror};
const DIR = {up: [-1, 0], down: [1, 0], left: [0, 1], right: [0, -1], ref: [Math.cos(Math.PI / 9), Math.sin(Math.PI / 9)]};
const HEAD = {up: Math.PI, down: 0, left: Math.PI / 2, right: -Math.PI / 2};   // 'ref': the first cycle's own heading
const psiW = Math.atan2(w.ey, w.ex), windFrom = -90 - psiW * 180 / Math.PI;
const ctl = new MPC.Controller(inp.ac, entries, {actuators: inp.actuators, settings: inp.settings});
const act = new MPC.Actuators(ac, {rateModel: inp.actuators.rate_model, tau: inp.actuators.tau_s,
  clRate: inp.actuators.cl_rate_per_s, kPb2v: inp.actuators.k_pb2v, delay: inp.actuators.delay_s});
const keyOf = (pt) => pt.steady ? 'steady' : pt.kind + ':' + pt.entry.u10 + ':' + pt.entry.chi_deg + ':' + (pt.side < 0 ? 'm' : 'n');
const infoKey = (f) => f.marginSet === 'steady' ? 'steady' : f.kind + ':' + f.u10 + ':' + f.chiDeg + ':' + (f.mirror ? 'm' : 'n');
let head = inp.courses[0][1], ci = 1;
let cmd = fc.step(0, inp.u10, windFrom, DIR[head]);
const c0 = new MPC.Cycle(cmd.point.steady ? NAV.steadyEntry(cmd.point) : cmd.point.entry, cmd.point.side < 0);
let k0 = 0;
for (let i = 1; i < c0.z.length; i++) if (c0.z[i] > c0.z[k0]) k0 = i;
let s = [0, 0, c0.z[k0], c0.V[k0], c0.g[k0], psiW + (head === 'ref' ? c0.psi[k0] : HEAD[head])];   // as the viewer launches: at the cycle's top, on the heading
const dt = 0.01, n = Math.round(inp.T / dt), every = Math.round(0.5 / dt), navEvery = Math.round(5 / dt);
const steadyEvery = Math.round(1 / dt);
const motor = !!inp.settings.motor;
const foldDcd = MPC.MOTOR.foldedCdaM2 / ac.wing_area, tMax = MPC.MOTOR.thrustMaxPerWeight * ac.mass * 9.81;
let plan = null, tPlan = 0, ended = 'complete', tEnd = inp.T, tA = 0;
let vMin = Infinity, tvMin = 0, cosMin = Infinity, hMin = Infinity, tipMin = Infinity;
const changes = [], plans = [], track = [];
for (let i = 0; i <= n; i++) {
  const t = i * dt;
  const h = s[2] - trueWorld.eta(s[0], s[1], t);
  const tip = h - 0.5 * ac.span * Math.sin(Math.abs(act.mu || 0)) * Math.cos(s[4]);
  hMin = Math.min(hMin, h); tipMin = Math.min(tipMin, tip); if (s[3] < vMin) { vMin = s[3]; tvMin = t; } cosMin = Math.min(cosMin, Math.cos(s[4]));
  if (h < 0 || tip < 0) { ended = 'contact'; tEnd = t; break; }
  if (!s.every(Number.isFinite)) { ended = 'non-finite'; tEnd = t; break; }
  if (i % 100 === 0) track.push([t, s[0], s[1]]);
  // as the viewer: a change of the heading at once, else every 5 s (1 s on a steady leg) and every plan while a
  // change goes through intermediate cycles
  const flown = flownOf(plan && plan.info);
  if (ci < inp.courses.length && t >= inp.courses[ci][0] - 1e-9) {
    head = inp.courses[ci][1]; ci++;
    cmd = fc.step(t, inp.u10, windFrom, DIR[head], flown);
    changes.push({t: t, head: head, key: keyOf(cmd.goal), mode: cmd.mode, held: cmd.held});
  } else if (i > 0 && (i % (cmd.point.steady ? steadyEvery : navEvery) === 0 || (cmd.point !== cmd.goal && i % every === 0))) {
    cmd = fc.step(t, inp.u10, windFrom, DIR[head], flown);
  }
  if (plan === null || i % every === 0) {
    const ck = cmd.point.steady ? null : {kind: cmd.point.kind, u10: cmd.point.entry.u10, chi_deg: cmd.point.entry.chi_deg, mirror: cmd.point.side < 0};
    plan = ctl.plan({t: t, s: s.slice(), act: act.cl === null ? null :
      {cl: act.cl, mu: act.mu, clCmd: act.cmdLast[0], muCmd: act.cmdLast[1], T: act.T},
      world: {kind: 'flat', wind: {ustar: w.ustar, z0: w.z0, ex: w.ex, ey: w.ey, karman: K}, comps: null, stokes: false, maxComponents: 48},
      u10: inp.u10, chiDeg: 20, cycle: ck, entry: cmd.point.steady ? NAV.steadyEntry(cmd.point) : undefined, guideSig: [3.0, 3.0, 0.15, 0.1]});
    tPlan = t;
    plans.push({t: t, want: keyOf(cmd.point), goal: keyOf(cmd.goal), flown: infoKey(plan.info), match: plan.info.matchDist});
  }
  let [cl, mu] = plan.controls(t - tPlan);
  cl = Math.min(Math.max(cl, ac.cl_min), ac.cl_max); mu = Math.min(Math.max(mu, -ac.mu_max), ac.mu_max);
  cl = MPC.nLimitedCl(ac, s[3], cl);
  let clA, muA;
  if (motor) {
    const tCmd = Math.min(Math.max(MPC.planThrust(plan, t - tPlan), 0), tMax);
    [clA, muA] = act.step(cl, mu, dt, s[3], tCmd);
    tA = act.T;
  } else [clA, muA] = act.step(cl, mu, dt, s[3]);
  if (i === n) break;
  s = motor ? Flight.rk4(s, clA, muA, ac, windFn, t, dt, true, tA, foldDcd) : Flight.rk4(s, clA, muA, ac, windFn, t, dt, true);
}
const hull = NAV.pointsFromEntries(entries, inp.hullU10).map(p => ({key: p.kind + ':' + p.entry.u10 + ':' + p.entry.chi_deg + ':' + (p.side < 0 ? 'm' : 'n'), vx: p.vx, vy: p.vy}));
console.log(JSON.stringify({ended: ended, tEnd: tEnd, vMin: vMin, tvMin: tvMin, vStall: ac.v_stall, cosMin: cosMin, hMin: hMin,
  tipMin: tipMin, changes: changes, plans: plans, track: track, psiW: psiW, hull: hull}));
"""


@unittest.skipUnless(NODE, "node not available")
class TestFreeCourseLoopJs(unittest.TestCase):
    """The viewer's free course (no destination): a closed loop in node with the albatross-like glider at 13 m/s
    on a flat sea, NAV.FreeCourse choosing the leg of the heading, the worker's Controller flying it and the true
    dynamics. A change of the heading in flight must lead to the new course's cycle after a phase match (a change
    between soaring cycles waits for the match, at most SWITCH_MAX_WAIT_S) and the aircraft must then make good
    along the new course, with no contact, no loop and the airspeed above the stall speed throughout."""

    SWITCH_MATCH_MAX, SWITCH_MAX_WAIT_S, REPLAN_S = 2.0, 15.0, 0.5   # sim.mpc and mpc.js

    def run_loop(self, courses, motor, T, thrust_update="raw", h_ceil=28.5):
        from sim.seawind import Wind
        from sim.tests.test_mpc_js import ac_params, wind_dict
        lib_js = (VIEWER / "library_motor.js").read_text(encoding="utf-8")
        lib = json.loads(lib_js.split("window.LIBRARY_MOTOR = ", 1)[1].rsplit(";", 1)[0])
        name = "albatross-3m"
        # the viewer's settings of a flight on named legs (with a destination or on a free course: sim.mpc.NAV_PKW)
        settings = {"motor": True, "energyLevel": "normal", "thrustSampling": "sustained", "foldSwitch": "state",
                    "thrustUpdate": thrust_update, "hCeil": h_ceil} if motor else {}
        w = Wind(13.0, 270.0, roughness="coare35")
        payload = dict(ac=ac_params(by_name(name)), settings=settings, actuators=lib["actuators"],
                       entries=lib["aircraft"][name], u10=13.0, hullU10=13.0, wind=wind_dict(w), courses=courses,
                       T=T, motor=motor)
        r = subprocess.run([NODE, "-e", LOOP_FREE, str(VIEWER / "mpc.js"), str(VIEWER / "flight.js"), str(NAV_JS)],
                           input=json.dumps(payload), capture_output=True, text=True, timeout=600)
        if r.returncode != 0:
            raise RuntimeError(r.stderr)
        return json.loads(r.stdout)

    @staticmethod
    def made_good(r, t0, t1):
        """Mean ground velocity in the wind frame (x downwind, y to its left) between t0 and t1."""
        tr = [p for p in r["track"] if t0 - 1e-9 <= p[0] <= t1 + 1e-9]
        (ta, xa, ya), (tb, xb, yb) = tr[0], tr[-1]
        c, s = math.cos(r["psiW"]), math.sin(r["psiW"])
        dx, dy = (xb - xa) / (tb - ta), (yb - ya) / (tb - ta)
        return c * dx + s * dy, -s * dx + c * dy

    @staticmethod
    def switch(r, k):
        """(time of the change k, time the plan first flew its goal, the changes of the cycle flown from the change to
        then as (time, from, to, match distance))"""
        ch = r["changes"][k]
        after = [p for p in r["plans"] if p["t"] >= ch["t"] - 1e-9]
        before = [p for p in r["plans"] if p["t"] < ch["t"] - 1e-9]
        first = next(p for p in after if p["flown"] == ch["key"])
        hops, last = [], before[-1]["flown"]
        for p in after:
            if p["t"] > first["t"]:
                break
            if p["flown"] != last:
                hops.append((p["t"], last, p["flown"], p["match"]))
                last = p["flown"]
        return ch["t"], first["t"], hops

    def check_margins(self, r):
        self.assertEqual(r["ended"], "complete")
        self.assertGreater(r["vMin"], r["vStall"])       # no stall
        self.assertGreater(r["cosMin"], 0.0)             # no loop
        self.assertGreater(r["tipMin"], 0.0)

    def report(self, label, r, k, t0, t1):
        t_ch, t_goal, hops = self.switch(r, k)
        print(chr(10) + "node free course, albatross-3m 13 m/s, %s: %s, %s %s%s flown %.1f s after the change over %s, "
              "min airspeed %.1f m/s (stall %.1f), min cos gamma %.2f, min height %.2f m, min wingtip %.2f m, "
              "made good (downwind, left) %s m/s before and %s m/s after"
              % (label, r["ended"], r["changes"][k]["mode"], r["changes"][k]["key"],
                 " (held: the kind of leg does not change in flight)" if r["changes"][k]["held"] else "", t_goal - t_ch,
                 ["%s at %.1f s (match %.2f)" % (h[2], h[0] - t_ch, h[3]) for h in hops],
                 r["vMin"], r["vStall"], r["cosMin"], r["hMin"], r["tipMin"],
                 tuple(round(v, 1) for v in self.made_good(r, 5.0, t_ch)),
                 tuple(round(v, 1) for v in self.made_good(r, t0, t1))))
        return t_ch, t_goal, hops

    def check_switches(self, hops):
        """Every change between soaring cycles at a phase match (none forced at the end of the wait)."""
        self.assertTrue(hops)
        for t, a, b, m in hops:
            if a != "steady" and b != "steady":
                self.assertLessEqual(m, self.SWITCH_MATCH_MAX)

    def test_downwind_then_reference_crosses_the_wind_at_matches(self):
        """Downwind (a cycle on the right of the wind), then the reference course 20 deg off downwind from 20 s, whose
        cycle lies on the left and matches the downwind cycle at no phase: the change goes through the downwind
        cycle's mirror image, each step at a phase match, and the track then lies left of downwind."""
        r = self.run_loop([[0.0, "down"], [20.0, "ref"]], False, 60.0)
        t_ch, t_goal, hops = self.report("down, then ref at 20 s", r, 0, 35.0, 60.0)
        self.check_margins(r)
        self.assertEqual(r["changes"][0]["mode"], "course")
        self.assertGreaterEqual(len(hops), 2)
        self.check_switches(hops)
        self.assertLess(t_goal - t_ch, 2 * (self.SWITCH_MAX_WAIT_S + self.REPLAN_S))
        self.assertLess(self.made_good(r, 5.0, 20.0)[1], 0.0)    # right of downwind before the change
        vx, vy = self.made_good(r, 35.0, 60.0)
        self.assertGreater(vx, 0.0)
        self.assertGreater(vy, 0.0)                              # left of downwind after it

    def test_steady_flight_stays_steady_and_turns_gradually(self):
        """Launched upwind with the motor (a steady leg: no soaring cycle flies upwind), then downwind from 20 s: the
        flight stays on a steady leg (held), its track turning by at most turnDeg a second, and makes good downwind
        without a stall."""
        r = self.run_loop([[0.0, "up"], [20.0, "down"]], True, 70.0)
        t_ch, t_goal, hops = self.report("motor, up, then down at 20 s", r, 0, 45.0, 70.0)
        self.assertEqual(r["changes"][0]["key"], "steady")
        self.assertTrue(r["changes"][0]["held"])
        self.check_margins(r)
        self.assertLess(self.made_good(r, 5.0, 20.0)[0], 0.0)
        self.assertGreater(self.made_good(r, 45.0, 70.0)[0], 0.0)

    def test_no_powered_leg_from_a_soaring_cycle_in_flight(self):
        """With the motor, downwind and then upwind: from a soaring cycle the free course does not take a steady leg
        in flight (the transient of Q-066 lost the aircraft in this loop), it flies the soaring cycle that gets
        furthest upwind, reached at a phase match."""
        r = self.run_loop([[0.0, "down"], [20.0, "up"]], True, 60.0)
        t_ch, t_goal, hops = self.report("motor, down, then up at 20 s", r, 0, 40.0, 60.0)
        self.assertEqual(r["changes"][0]["mode"], "no progress")
        self.assertTrue(r["changes"][0]["held"])
        self.check_switches(hops)
        self.check_margins(r)

    def test_upwind_without_the_motor_takes_the_cycle_that_gets_furthest(self):
        r = self.run_loop([[0.0, "down"], [20.0, "up"]], False, 60.0)
        t_ch, t_goal, hops = self.report("down, then up at 20 s without the motor", r, 0, 40.0, 60.0)
        self.assertEqual(r["changes"][0]["mode"], "no progress")
        self.check_switches(hops)
        self.check_margins(r)


CHAIN = r"""
const NAV = require(process.argv[1]);
const MPC = require(process.argv[2]);
const inp = JSON.parse(require('fs').readFileSync(0, 'utf8'));
const entries = inp.entries.filter(e => e.kind === 'soaring');
const SIG = [3.0, 3.0, 0.15, 0.1], cycs = {};
const cycOf = (p) => cycs[p.key] || (cycs[p.key] = new MPC.Cycle(p.entry, p.side < 0));
const link = (a, b) => MPC.cycleLink(cycOf(a), cycOf(b), SIG);
const DIR = {down: [1, 0], left: [0, 1], right: [0, -1]};
const out = [];
inp.cases.forEach(c => {
  const fc = new NAV.FreeCourse(new NAV.HullSource(entries), {link: link, linkMax: MPC.SWITCH_MATCH_MAX / 2});   // viewer.course_link_frac
  let cmd = fc.step(0, c.u10, 270, DIR[c.from], null);
  const start = cmd.goal;
  let flown = {kind: start.kind, u10: start.entry.u10, chi_deg: start.entry.chi_deg, mirror: start.side < 0};
  const path = [start.key], links = [];
  for (let k = 0; k < 10; k++) {   // each commanded cycle taken as flown, as after its phase match
    cmd = fc.step(5 * (k + 1), c.u10, 270, DIR[c.to], flown);
    links.push(link(fc._find(flown), cmd.point));
    path.push(cmd.point.key);
    flown = {kind: cmd.point.kind, u10: cmd.point.entry.u10, chi_deg: cmd.point.entry.chi_deg, mirror: cmd.point.side < 0};
    if (cmd.point === cmd.goal) break;
  }
  out.push({path: path, links: links, goal: cmd.goal.key, direct: link(start, cmd.goal)});
});
process.stdout.write(JSON.stringify(out));
"""


@unittest.skipUnless(NODE, "node not available")
class TestFreeCourseChainJs(unittest.TestCase):
    """FreeCourse's chain of changes (no dynamics): from a soaring cycle to one on the other side of the wind that it
    matches at no phase, each commanded cycle must match the one before within half SWITCH_MATCH_MAX; where no such
    chain exists (the two sides do not connect at that library wind) the change is commanded at once."""

    def run_cases(self, cases):
        lib_js = (VIEWER / "library_motor.js").read_text(encoding="utf-8")
        lib = json.loads(lib_js.split("window.LIBRARY_MOTOR = ", 1)[1].rsplit(";", 1)[0])
        r = subprocess.run([NODE, "-e", CHAIN, str(NAV_JS), str(VIEWER / "mpc.js")],
                           input=json.dumps(dict(entries=lib["aircraft"]["albatross-3m"], cases=cases)),
                           capture_output=True, text=True, timeout=120)
        if r.returncode != 0:
            raise RuntimeError(r.stderr)
        return json.loads(r.stdout)

    def test_left_and_right_chain_through_matching_cycles(self):
        cases = [dict(u10=13.0, **{"from": "left", "to": "right"}), dict(u10=13.0, **{"from": "right", "to": "left"}),
                 dict(u10=16.0, **{"from": "left", "to": "right"}), dict(u10=13.0, **{"from": "down", "to": "left"})]
        for c, r in zip(cases, self.run_cases(cases)):
            print(chr(10) + "free course chain, albatross-3m %s m/s, %s to %s: %s, match distances %s (direct %.1f)"
                  % (c["u10"], c["from"], c["to"], " > ".join(r["path"]), [round(x, 2) for x in r["links"]], r["direct"]))
            self.assertGreater(r["direct"], 1.0)             # the goal matches the start at no phase
            self.assertGreater(len(r["path"]), 2)            # so there are intermediate cycles
            self.assertEqual(r["path"][-1], r["goal"])
            for x in r["links"]:
                self.assertLessEqual(x, 1.0)

    def test_no_chain_commands_the_goal_at_once(self):
        """At 11 m/s the cycles on either side of the wind match at no phase: the change is commanded at once and the
        planner's own wait applies, as for the navigator's tacks."""
        r = self.run_cases([dict(u10=11.0, **{"from": "left", "to": "right"})])[0]
        self.assertEqual(len(r["path"]), 2)
        self.assertEqual(r["path"][-1], r["goal"])
