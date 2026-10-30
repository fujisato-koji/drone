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
      u10: inp.u10, chiDeg: 20, cycle: ck, entry: cmd.point.steady ? NAV.steadyEntry(cmd.point) : undefined, guideSig: [3.0, null, 0.25, 0.1]});
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
