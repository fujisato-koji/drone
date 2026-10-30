"""The viewer's mpc.js (receding-horizon planner) against sim.mpc, and a closed loop in node.

Needs node (skipped without it). The library cycles are read from sim/viewer/library.js, so
Python and JS plan with the same (rounded) numbers.
"""
import json
import shutil
import subprocess
import unittest
from pathlib import Path

import numpy as np

from sim.aircraft import Actuators
from sim.aircraft_models import by_name
from sim.dsopt import folded_dcd, prop_efficiency, thrust_max
from sim.record import value
from sim.autonomy import make_env
from sim.mpc import (LIB_PKW, P_HOLD, THRUST_CAND_LEVELS, W_ENERGY_LEVELS, CycleGuide, CycleLibrary,
                     MPPIPlanner, Reflex, energy_weight, wrap)
from sim.seawind import Environment, Wind

VIEWER = Path(__file__).resolve().parents[1] / "viewer"
MPC_JS, FLIGHT_JS, LIBRARY_JS = VIEWER / "mpc.js", VIEWER / "flight.js", VIEWER / "library.js"
NODE = shutil.which("node")
NAME = "albatross-3m"
U10 = 13.0
CHI_DEG = 20.0

# Node snippet: reads one JSON object on stdin, writes one JSON object on stdout.
SNIPPET = r"""
const MPC = require(process.argv[1]);
const Flight = require(process.argv[2]);
const inp = JSON.parse(require('fs').readFileSync(0, 'utf8'));
const out = {};
const ac = MPC.makeAircraft(inp.ac);
const mkPlanner = () => {
  const pl = new MPC.Planner(ac, inp.settings);
  pl.act = MPC.actuatorParams(ac, {rateModel: inp.actuators.rate_model, tau: inp.actuators.tau_s,
    clRate: inp.actuators.cl_rate_per_s, kPb2v: inp.actuators.k_pb2v, delay: inp.actuators.delay_s});
  return pl;
};
const flat = (a) => Array.from(a);
if (inp.run === 'rollout') {
  const world = MPC.makeWorld(inp.world);
  const pl = mkPlanner();
  pl.actState = inp.actState;
  const psiW = Math.atan2(inp.world.wind.ey, inp.world.wind.ex);
  const cyc = MPC.pickCycle(inp.entries, inp.u10, inp.chi);
  const guide = new MPC.Guide(cyc, world, inp.t, inp.s, psiW, pl.horizonS, pl.wD, null);
  const S = inp.Us.length;
  const Us = Float64Array.from([].concat(...inp.Us.map(u => [].concat(...u))));
  const r = pl.rollout(world, inp.t, inp.s, Us, S, guide, true);
  out.cost = flat(r.cost); out.dead = flat(r.dead); out.paths = r.paths.map(flat);
  out.tau0 = guide.tau0; out.target = guide.target; out.dEc = guide.dEc;
  out.nominal = flat(guide.nominal(pl.knotsT)); out.mirror = cyc.mirror;
  out.nominalThrust = flat(guide.nominalThrust(pl.knotsT));
} else if (inp.run === 'update') {
  const pl = mkPlanner();
  const S = inp.Us.length;
  const Us = Float64Array.from([].concat(...inp.Us.map(u => [].concat(...u))));
  const r = pl.update(Float64Array.from([].concat(...inp.U)), Us, S, inp.cost, inp.dead);
  out.U = flat(r.U); out.ess = r.ess; out.nDead = r.nDead;
} else if (inp.run === 'plan') {
  const world = MPC.makeWorld(inp.world);
  const pl = mkPlanner();
  const psiW = Math.atan2(inp.world.wind.ey, inp.world.wind.ex);
  const cyc = MPC.pickCycle(inp.entries, inp.u10, inp.chi);
  const guide = new MPC.Guide(cyc, world, inp.t, inp.s, psiW, pl.horizonS, pl.wD, null);
  out.U = [];
  inp.eps.forEach((eps) => {
    pl.epsHook = () => Float64Array.from([].concat(...[].concat(...eps)));
    pl.actState = inp.actState;
    pl.plan(world, inp.t, inp.s, guide);
    out.U.push(flat(pl.U));
  });
} else if (inp.run === 'windcheck') {
  // the one-pass nine-point wind of SeaWorld.windDerivs against nine windAt calls, incl. points
  // inside the roughness layer and the second-order surface
  const world = MPC.makeWorld(inp.world);
  const H = 0.05, DT = 0.01, o = [0, 0, 0];
  let worst = 0, nOrb = 0;
  for (const p of inp.points) {
    const [x, y, z, t] = p;
    const low = z - H <= world.z0, zlo = low ? z : z - H;
    const pts = [[x, y, z, t], [x + H, y, z, t], [x - H, y, z, t], [x, y + H, z, t], [x, y - H, z, t],
      [x, y, z + H, t], [x, y, zlo, t], [x, y, z, t + DT], [x, y, z, t - DT]];
    const P = new Float64Array(27);
    world.windDerivs(x, y, z, t, P);
    pts.forEach((q, i) => {
      world.windAt(q[0], q[1], q[2], q[3], o);
      for (let j = 0; j < 3; j++) worst = Math.max(worst, Math.abs(P[3 * i + j] - o[j]) / Math.max(1, Math.abs(o[j])));
      if (q[2] - world.eta(q[0], q[1], q[3]) <= world.z0) nOrb++;
    });
  }
  out.worst = worst; out.nOrb = nOrb;
} else if (inp.run === 'reflex') {
  const world = MPC.makeWorld(inp.world);
  const rf = new MPC.Reflex(ac, {foldSwitch: inp.foldSwitch});
  const plan = {knotsT: inp.knotsT, cl: inp.cl, mu: inp.mu, th: inp.th};
  out.hits = inp.states.map((st) => rf.predict(world, inp.t, st, plan, inp.tau));
  out.override = inp.states.map((st) => rf.override(st));
  out.tMax = rf.tMax; out.params = [rf.horizonS, rf.clearanceM, rf.vFrac, rf.dt, rf.nFrac, rf.vFactor];
  const pl = new MPC.Planner(ac, {motor: true, thrustSampling: 'sustained'});
  out.cands = pl.thrustCandidates(Float64Array.from(inp.U)).map(flat);
  out.sustain = MPC.SUSTAIN;
} else if (inp.run === 'defaults') {
  out.defaults = MPC.DEFAULTS; out.motor = MPC.MOTOR; out.energyK = MPC.ENERGY_K;
} else if (inp.run === 'loop') {
  // closed loop on the true dynamics: the planner every 0.5 s, actuators pb2v, load-factor limit
  const K = 0.4, w = inp.wind;
  const comps = inp.comps;   // null: flat sea
  const trueWorld = new MPC.SeaWorld({ustar: w.ustar, z0: w.z0, ex: w.ex, ey: w.ey, karman: K}, comps, false);
  const windFn = (x, y, z, t) => { const o = [0, 0, 0]; trueWorld.windAt(x, y, z, t, o); return o; };
  windFn.z0 = w.z0;
  const ctl = new MPC.Controller(inp.ac, inp.entries, {actuators: inp.actuators, settings: inp.settings});
  const act = new MPC.Actuators(ac, {rateModel: inp.actuators.rate_model, tau: inp.actuators.tau_s,
    clRate: inp.actuators.cl_rate_per_s, kPb2v: inp.actuators.k_pb2v, delay: inp.actuators.delay_s});
  const psiW = Math.atan2(w.ey, w.ex);
  const c0 = MPC.pickCycle(inp.entries, inp.u10, inp.chiDeg * Math.PI / 180);
  let k0 = 0;
  for (let i = 1; i < c0.z.length; i++) if (c0.z[i] > c0.z[k0]) k0 = i;
  let s = [0, 0, trueWorld.eta(0, 0, 0) + c0.z[k0], c0.V[k0], c0.g[k0], psiW + c0.psi[k0]];
  const dt = 0.01, n = Math.round(inp.T / dt), every = Math.round(0.5 / dt);
  const motor = !!(inp.settings && inp.settings.motor);
  const foldDcd = MPC.MOTOR.foldedCdaM2 / ac.wing_area, tMax = MPC.MOTOR.thrustMaxPerWeight * ac.mass * 9.81;
  let eJ = 0, nOn = 0, tA = 0, nBurst = 0, wasOn = false;
  const ms = [], tau0 = [];
  let plan = null, tPlan = 0, ended = 'complete', hmin = Infinity, tipmin = Infinity, tEnd = inp.T, nmax = 0;
  const worldSpec = {kind: inp.model, wind: {ustar: w.ustar, z0: w.z0, ex: w.ex, ey: w.ey, karman: K},
    comps: comps, stokes: false, maxComponents: inp.maxComponents};
  for (let i = 0; i <= n; i++) {
    const t = i * dt;
    const h = s[2] - trueWorld.eta(s[0], s[1], t);
    const tip = h - 0.5 * ac.span * Math.sin(Math.abs(act.mu || 0)) * Math.cos(s[4]);
    hmin = Math.min(hmin, h); tipmin = Math.min(tipmin, tip);
    if (h < 0) { ended = 'contact'; tEnd = t; break; }
    if (tip < 0) { ended = 'tip contact'; tEnd = t; break; }
    if (!s.every(Number.isFinite)) { ended = 'non-finite'; tEnd = t; break; }
    if (plan === null || i % every === 0) {
      const t0 = process.hrtime.bigint();
      plan = ctl.plan({t: t, s: s.slice(), act: act.cl === null ? null :
        {cl: act.cl, mu: act.mu, clCmd: act.cmdLast[0], muCmd: act.cmdLast[1], T: act.T},
        world: worldSpec, u10: inp.u10, chiDeg: inp.chiDeg});
      ms.push(Number(process.hrtime.bigint() - t0) / 1e6);
      tPlan = t; tau0.push(plan.info.tau0);
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
      const on = tA > 0.01 * ac.mass * 9.81;
      if (on) nOn++;
      if (on && !wasOn) nBurst++;
      wasOn = on;
    } else [clA, muA] = act.step(cl, mu, dt, s[3]);
    nmax = Math.max(nmax, 0.5 * ac.rho * s[3] * s[3] * ac.wing_area * Math.min(clA, MPC.nLimitedCl(ac, s[3], clA)) / (ac.mass * 9.81));
    if (i === n) break;
    s = motor ? Flight.rk4(s, clA, muA, ac, windFn, t, dt, true, tA, foldDcd) : Flight.rk4(s, clA, muA, ac, windFn, t, dt, true);
  }
  out.eJ = eJ; out.onShare = nOn / (n + 1); out.nBurst = nBurst; out.avgPowerW = eJ / tEnd;
  out.ended = ended; out.tEnd = tEnd; out.hmin = hmin; out.tipmin = tipmin; out.nmax = nmax;
  out.ms = ms; out.tau0 = tau0;
}
console.log(JSON.stringify(out));
"""


def run_node(payload):
    r = subprocess.run([NODE, "-e", SNIPPET, str(MPC_JS), str(FLIGHT_JS)], input=json.dumps(payload),
                       capture_output=True, text=True, timeout=900)
    if r.returncode != 0:
        raise RuntimeError(r.stderr)
    return json.loads(r.stdout)


def load_library_js():
    text = LIBRARY_JS.read_text(encoding="utf-8")
    body = text.split("window.LIBRARY = ", 1)[1].rsplit(";", 1)[0]
    return json.loads(body)


def ac_params(ac):
    return {"name": ac.name, "mass": ac.mass, "span": ac.span, "wing_area": ac.wing_area,
            "cd0": ac.cd0, "oswald": ac.oswald, "cl_max": ac.cl_max, "cl_min": ac.cl_min,
            "mu_max": ac.mu_max, "n_max": ac.n_max, "rho": ac.rho}


def js_settings(pkw):
    names = {"n_samples": "nSamples", "dt": "dt", "n_iter": "nIter", "n_ctrl": "nCtrl", "lam": "lam",
             "sigma_cl": "sigmaCl", "sigma_mu": "sigmaMu", "clearance": "clearance",
             "h_clear": "hClear", "motor": "motor", "w_energy": "wEnergy",
             "thrust_sampling": "thrustSampling", "fold_switch": "foldSwitch"}
    return {names[k]: v for k, v in pkw.items()}


def wind_dict(w):
    return {"ustar": float(w.ustar), "z0": float(w.z0), "ex": float(w.ex), "ey": float(w.ey),
            "karman": 0.4}


# two wave components travelling roughly with the wind (a = 0.9 m and 0.5 m)
COMPS = {"a": [0.9, 0.5], "k": [0.12, 0.31], "omega": [1.08, 1.74], "dx": [1.0, 0.8],
         "dy": [0.0, 0.6], "phase": [0.7, 2.1]}
_NORM = np.hypot(COMPS["dx"], COMPS["dy"])
COMPS["dx"], COMPS["dy"] = list(np.array(COMPS["dx"]) / _NORM), list(np.array(COMPS["dy"]) / _NORM)


def python_world(kind, wind, stokes=False):
    if kind == "flat":
        return Environment.flat(wind)
    e = Environment(wind, [], stokes=stokes)
    c = {k: np.array(v, float) for k, v in COMPS.items()}
    e.c = c
    zref = np.maximum(1.0 / c["k"], 2.0 * wind.z0)
    e.urel = wind.speed(zref) * (wind.ex * c["dx"] + wind.ey * c["dy"]) - c["omega"] / c["k"]
    return e


def js_world(kind, wind, env):
    d = {"kind": kind, "wind": wind_dict(wind), "stokes": bool(getattr(env, "stokes", False))}
    if kind == "truth":
        d["comps"] = {k: [float(v) for v in env.c[k]] for k in ("a", "k", "omega", "phase", "dx", "dy")}
        d["comps"]["urel"] = [float(v) for v in env.urel]
    return d


def close(a, b, rtol, atol):
    a, b = np.asarray(a, float), np.asarray(b, float)
    return bool(np.all(np.abs(a - b) <= atol + rtol * np.abs(b))), float(np.max(np.abs(a - b) / (atol + np.abs(b))))


@unittest.skipUnless(NODE, "node is not on PATH: mpc.js tests skipped")
class TestMpcJs(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.lib = load_library_js()
        cls.entries = cls.lib["aircraft"][NAME]
        cls.ac = by_name(NAME)
        # x and y are not exported (the library mode does not use them); zeros satisfy sim.mpc.Cycle
        zeros = [0.0] * len(cls.entries[0]["t"])
        cls.pylib = CycleLibrary([dict(e, aircraft=NAME, success=True, x=zeros, y=zeros)
                                  for e in cls.entries])
        # the same cycles with a nonzero thrust schedule (a motor-assisted library entry)
        cls.entries_th = [dict(e, thrust=[float(f"{1.5 + 1.2 * np.sin(2 * np.pi * t / e['period']):.4g}")
                                          for t in e["t"]]) for e in cls.entries]
        cls.pylib_th = CycleLibrary([dict(e, aircraft=NAME, success=True, x=zeros, y=zeros)
                                     for e in cls.entries_th])
        cls.wind = Wind(U10, 270.0)
        cls.psi_w = float(np.arctan2(cls.wind.ey, cls.wind.ex))
        cls.chi = np.radians(CHI_DEG)

    def setup_case(self, kind, height_frac=0.45, stokes=False, motor=False, extra=None):
        """Python planner, guide, state and the control sequences for one world. With motor the
        planner carries the thrust (w_energy of the normal level) and the plans a third row."""
        ac = self.ac
        world = python_world(kind, self.wind, stokes)
        self.pkw = dict(LIB_PKW, motor=True, w_energy=energy_weight(ac, "normal")) if motor else dict(LIB_PKW)
        self.pkw.update(extra or {})
        self.cur_entries = self.entries_th if motor else self.entries
        pl = MPPIPlanner(ac, **self.pkw)
        pl.n_limit = True
        pl.act = Actuators(ac, rate_model="pb2v")
        cyc = (self.pylib_th if motor else self.pylib).pick(NAME, U10, self.chi)
        t = 3.7
        k = int(np.argmax(cyc.z))
        eta = float(world.surface(0.0, 0.0, t)[0])
        # a state a little below the top of the cycle, so that the dive below touches the water
        s0 = np.array([5.0, -3.0, eta + height_frac * cyc.z[k], cyc.V[k], cyc.g[k] - 0.05, self.psi_w + cyc.psi[k]])
        pl.act_state = (0.6, 0.15, 0.7, 0.2) + ((0.4,) if motor else ())
        guide = CycleGuide(cyc, world, t, s0, self.psi_w, pl.horizon_s, wave_aware=False, w_d=pl.w_d)
        pl.guide = guide
        nc = pl.n_ctrl
        nominal = guide.nominal(pl.knots_t)
        ramp = np.stack([np.full(nc, ac.cl_best_ld), np.linspace(0.0, 0.8, nc)])
        harsh = np.stack([np.full(nc, 0.0), np.where(np.arange(nc) % 2 == 0, 1.0, -1.0)])
        harsh[0] = 1.4 * ac.cl_max * np.linspace(0.5, 1.0, nc)       # above the limits: clipped
        dive = np.stack([np.full(nc, 0.05), np.full(nc, 0.3)])
        if motor:
            tm = pl.t_max
            nominal, ramp = (np.vstack([nominal, guide.nominal_thrust(pl.knots_t)]),
                             np.vstack([ramp, np.linspace(0.0, 0.8 * tm, nc)]))
            harsh = np.vstack([harsh, np.where(np.arange(nc) % 3 == 0, 1.5 * tm, -0.3 * tm)])    # clipped to [0, tm]
            dive = np.vstack([dive, np.full(nc, 0.3 * tm)])
        Us = np.stack([nominal, ramp, harsh, dive])
        return world, pl, guide, s0, t, Us

    def payload(self, kind, pl, world, s0, t, run, **kw):
        act = pl.act
        d = {"run": run, "ac": ac_params(self.ac), "settings": js_settings(self.pkw),
             "actuators": self.lib["actuators"], "entries": self.cur_entries, "u10": U10,
             "chi": float(self.chi), "world": js_world(kind, self.wind, world), "s": [float(v) for v in s0],
             "t": t, "actState": list(pl.act_state)}
        d.update(kw)
        return d

    def rollout_parity(self, kind, stokes=False, motor=False, extra=None):
        world, pl, guide, s0, t, Us = self.setup_case(kind, stokes=stokes, motor=motor, extra=extra)
        cost, path, dead = pl.rollout(world, t, s0, Us, np.array([1.0, 0.0]))
        js = run_node(self.payload(kind, pl, world, s0, t, "rollout", Us=Us.tolist()))
        self.assertEqual(js["dead"], [int(v) for v in dead])
        ok_c, rel_c = close(js["cost"], cost, 1e-6, 1e-9)
        pyp = np.transpose(path, (2, 0, 1))                 # (S, N + 1, 6)
        jsp = np.array(js["paths"]).reshape(pyp.shape)
        ok_p, rel_p = close(jsp, pyp, 1e-6, 1e-9)
        ok_t, _ = close(js["tau0"], guide.tau0, 1e-12, 1e-12)
        ok_n, _ = close(js["nominal"], guide.nominal(pl.knots_t).ravel(), 1e-9, 1e-12)
        if motor:
            okt, _ = close(js["nominalThrust"], guide.nominal_thrust(pl.knots_t), 1e-9, 1e-12)
            self.assertTrue(okt, "nominal thrust differs")
            self.assertGreater(float(np.max(js["nominalThrust"])), 0.5)
        print("\nrollout %s%s%s: dead %s, cost max rel diff %.2e, state max rel diff %.2e"
              % (kind, " with the second-order surface" if stokes else "", " with the motor" if motor else "",
                 [int(v) for v in dead], rel_c, rel_p))
        self.assertTrue(dead.any(), "the dive should touch the water")
        self.assertTrue(ok_t and ok_n, "cycle phase match / nominal controls differ")
        self.assertTrue(ok_c, "costs differ: %.3e" % rel_c)
        self.assertTrue(ok_p, "states differ: %.3e" % rel_p)

    def test_rollout_flat_world(self):
        self.rollout_parity("flat")

    def test_rollout_truth_world(self):
        self.rollout_parity("truth")

    def test_rollout_truth_world_second_order(self):
        self.rollout_parity("truth", stokes=True)

    def test_rollout_flat_world_motor(self):
        self.rollout_parity("flat", motor=True)

    def test_rollout_truth_world_motor(self):
        self.rollout_parity("truth", motor=True)

    def test_rollout_truth_world_motor_state_fold(self):
        self.rollout_parity("truth", motor=True, extra={"fold_switch": "state"})

    def test_reflex_parity(self):
        ac, wind = self.ac, self.wind
        kt = np.linspace(0.0, 6.0, 24)
        cl, mu = np.full(24, 0.05), np.full(24, 0.1)
        th = np.linspace(0.0, 4.0, 24)
        states = [[0.0, 0.0, 3.0, 12.0, -0.35, 0.0], [0.0, 0.0, 12.0, 11.0, -0.05, 0.0],
                  [0.0, 0.0, 30.0, 1.02 * ac.v_stall, 0.0, 0.0], [3.0, 1.0, 1.4, 10.0, -0.05, 0.4]]
        for kind in ("flat", "truth"):
            for fold in ("smooth", "state"):
                world = python_world(kind, wind)
                rf = Reflex(ac, thrust_max(ac), folded_dcd(ac), True, fold)
                py = [rf.predict(world, 2.0, np.array(st), kt, cl, mu, th, 0.3) for st in states]
                js = run_node({"run": "reflex", "ac": ac_params(ac), "settings": {}, "actuators": self.lib["actuators"],
                               "world": js_world(kind, wind, world), "t": 2.0, "tau": 0.3, "states": states,
                               "knotsT": kt.tolist(), "cl": cl.tolist(), "mu": mu.tolist(), "th": th.tolist(),
                               "foldSwitch": fold, "U": cl.tolist() + mu.tolist() + th.tolist()})
                fired = [h is not None for h in py]
                self.assertEqual(fired, [h is not None for h in js["hits"]], (kind, fold))
                self.assertTrue(fired[0] and not fired[1] and fired[2], fired)
                for a, b in zip(py, js["hits"]):
                    if a is None:
                        continue
                    self.assertEqual(a[0], b["cause"])
                    ok, rel = close([a[1], a[2], a[3], a[4]], [b["ahead"], b["h"], b["tip"], b["V"]], 1e-9, 1e-9)
                    self.assertTrue(ok, "%s %s: %.2e" % (kind, fold, rel))
                for st, o in zip(states, js["override"]):
                    ok, _ = close(o, rf.override(np.array(st)), 1e-12, 1e-12)
                    self.assertTrue(ok)
        # the constants
        rf = Reflex(ac, thrust_max(ac), folded_dcd(ac), True)
        ok, _ = close(js["params"], [rf.horizon_s, rf.clearance_m, rf.v_frac, rf.dt, rf.n_frac, rf.v_factor], 1e-14, 1e-14)
        self.assertTrue(ok)
        self.assertEqual(js["sustain"]["pHold"], P_HOLD)
        self.assertEqual(js["sustain"]["candLevels"], list(THRUST_CAND_LEVELS))
        pl = MPPIPlanner(ac, motor=True, thrust_sampling="sustained", n_ctrl=24)
        U = np.array([cl, mu, th])
        ok, rel = close(np.array(js["cands"]).reshape(6, 3, 24), pl._thrust_candidates(U), 1e-12, 1e-12)
        self.assertTrue(ok, "thrust candidates differ")

    def test_wind_derivs_one_pass(self):
        rng = np.random.default_rng(3)
        w = self.wind
        pts = [[float(rng.uniform(-50, 50)), float(rng.uniform(-50, 50)), float(h), float(rng.uniform(0, 30))]
               for h in np.concatenate([rng.uniform(-0.5, 3.0, 40), w.z0 + np.array([0.0, 0.02, 0.05, 0.06, 0.1])])
               for _ in range(2)]
        for stokes in (False, True):
            env = python_world("truth", w, stokes)
            r = run_node({"run": "windcheck", "ac": ac_params(self.ac), "settings": {}, "actuators": self.lib["actuators"],
                          "world": js_world("truth", w, env), "points": pts})
            print("one pass wind vs nine calls (stokes %s): worst %.2e, %d samples in the roughness layer"
                  % (stokes, r["worst"], r["nOrb"]))
            self.assertLess(r["worst"], 1e-11)
            self.assertGreater(r["nOrb"], 0)

    def test_syntax(self):
        """node --check on mpc.js, flight.js and the inline module script of index.html."""
        import re
        import tempfile
        html = (VIEWER / "index.html").read_text(encoding="utf-8")
        body = re.search(r'<script type="module">(.*)</script>\s*</body>', html, re.S).group(1)
        with tempfile.TemporaryDirectory() as d:
            mod = Path(d) / "viewer_module.mjs"
            mod.write_text(body, encoding="utf-8")
            for path in (MPC_JS, FLIGHT_JS, mod):
                r = subprocess.run([NODE, "--check", str(path)], capture_output=True, text=True)
                self.assertEqual(r.returncode, 0, path.name + ": " + r.stderr)

    def test_defaults_match_python(self):
        d = run_node({"run": "defaults", "ac": ac_params(self.ac), "settings": {},
                      "actuators": self.lib["actuators"]})["defaults"]
        py = MPPIPlanner(self.ac, **LIB_PKW)
        for jk, pk in (("horizonS", "horizon_s"), ("dt", "dt"), ("nSamples", "n_samples"),
                       ("nCtrl", "n_ctrl"), ("lam", "lam"), ("sigmaCl", "sigma_cl"),
                       ("sigmaMu", "sigma_mu"), ("replanS", "replan_s"), ("nIter", "n_iter"),
                       ("wE", "w_e"), ("hClear", "h_clear"), ("clearance", "clearance"),
                       ("wClear", "w_clear"), ("wN", "w_n"), ("wStall", "w_stall"),
                       ("cContact", "c_contact"), ("wD", "w_d")):
            self.assertEqual(d[jk], getattr(py, pk), jk)
        a = Actuators(self.ac, rate_model="pb2v")
        lib = self.lib["actuators"]
        js = run_node({"run": "defaults", "ac": ac_params(self.ac), "settings": {},
                       "actuators": self.lib["actuators"]})
        m = js["motor"]
        self.assertEqual(m["thrustMaxPerWeight"] * self.ac.mass * 9.81, thrust_max(self.ac))
        self.assertAlmostEqual(m["propEff"] * m["motorEff"], prop_efficiency(), places=14)
        self.assertEqual(m["foldedCdaM2"] / self.ac.wing_area, folded_dcd(self.ac))
        self.assertEqual(m["thrustTauS"], a.thrust_tau)
        self.assertEqual(m["sigmaFrac"], value("ctrl.thrust_sigma_frac", 0.15))
        self.assertEqual(js["energyK"], W_ENERGY_LEVELS)
        self.assertEqual((lib["tau_s"], lib["cl_rate_per_s"], lib["k_pb2v"], lib["delay_s"]),
                         (a.tau, a.cl_rate, a.k_pb2v, a.delay))

    def test_mppi_update_parity(self):
        self.update_parity(False)

    def test_mppi_update_parity_motor(self):
        self.update_parity(True)

    def update_parity(self, motor):
        world, pl, guide, s0, t, Us0 = self.setup_case("truth", 1.0, motor=motor)
        rng = np.random.default_rng(5)
        S, nc = pl.n_samples, pl.n_ctrl
        U = guide.nominal(pl.knots_t)
        sig = np.array([[pl.sigma_cl], [pl.sigma_mu]])
        eps = rng.standard_normal((S, 2, nc)) * sig * 3.0     # wide, so that some samples die and some do not
        if motor:
            U = np.vstack([U, np.zeros(nc)])
            eps = np.concatenate([eps, rng.standard_normal((S, 1, nc)) * pl.sigma_t * 3.0], axis=1)
        eps[0] = 0.0
        Us = pl._clip(U[None] + eps)
        cost, _, dead = pl.rollout(world, t, s0, Us, np.array([1.0, 0.0]))
        U_py, ess_py, nd_py = pl.update(U, Us, cost, dead)
        js = run_node(self.payload("truth", pl, world, s0, t, "update", U=U.tolist(), Us=Us.tolist(),
                                   cost=[float(c) for c in cost], dead=[int(d) for d in dead]))
        ok, rel = close(js["U"], U_py.ravel(), 1e-9, 1e-12)
        print("\nMPPI update%s: dead %d of %d, ess %.2f, max rel diff %.2e"
              % (" with the motor" if motor else "", nd_py, S, ess_py, rel))
        self.assertTrue(0 < nd_py < S, "the test needs both dead and live samples")
        self.assertEqual(js["nDead"], nd_py)
        self.assertAlmostEqual(js["ess"], ess_py, delta=1e-9 * ess_py)
        self.assertTrue(ok, "updates differ: %.3e" % rel)

    def test_plan_with_injected_perturbations(self):
        self.plan_parity(False)

    def test_plan_with_injected_perturbations_motor(self):
        self.plan_parity(True)

    def test_plan_with_injected_perturbations_motor_sustained_state_fold(self):
        self.plan_parity(True, {"thrust_sampling": "sustained", "fold_switch": "state"})

    def plan_parity(self, motor, extra=None):
        world, pl, guide, s0, t, Us0 = self.setup_case("truth", 1.0, motor=motor, extra=extra)
        pl.guide = None
        rng = np.random.default_rng(7)
        S, nc = pl.n_samples, pl.n_ctrl
        sig = np.array([[pl.sigma_cl], [pl.sigma_mu]])
        eps = [rng.standard_normal((S, 2, nc)) * sig * 3.0 for _ in range(2)]   # the second call has a previous plan
        if motor:
            eps = [np.concatenate([e, rng.standard_normal((S, 1, nc)) * pl.sigma_t * 3.0], axis=1) for e in eps]
        out = []
        for e in eps:
            pl.eps_hook = lambda it, e=e: e
            pl.plan(world, t, s0, np.array([1.0, 0.0]), guide=guide)
            out.append(pl.U.copy())
        js = run_node(self.payload("truth", pl, world, s0, t, "plan", eps=[e.tolist() for e in eps]))
        for k in range(2):
            ok, rel = close(js["U"][k], out[k].ravel(), 1e-9, 1e-12)
            print("\nplan %d with injected perturbations%s: max rel diff %.2e"
                  % (k, " (motor)" if motor else "", rel))
            self.assertTrue(ok, "plan %d differs: %.3e" % (k, rel))

    def loop(self, model, comps, T, max_components=48, name=NAME, u10=U10, settings=None):
        w = Wind(u10, 270.0)
        payload = {"run": "loop", "ac": ac_params(by_name(name)), "settings": settings or {},
                   "actuators": self.lib["actuators"], "entries": self.lib["aircraft"][name], "u10": u10,
                   "chiDeg": CHI_DEG, "wind": wind_dict(w), "comps": comps, "model": model, "T": T,
                   "maxComponents": max_components}
        return run_node(payload)

    def test_closed_loop_flat_sea(self):
        for model in ("truth", "flat"):
            r = self.loop(model, None, 60.0)
            ms = np.array(r["ms"])
            print("\nclosed loop, albatross-3m u10 13, flat sea, %s world, 60 s: %s at %.1f s, min height %.2f m, "
                  "min tip %.2f m, max n %.2f; planning %d replans, mean %.1f ms, max %.1f ms"
                  % (model, r["ended"], r["tEnd"], r["hmin"], r["tipmin"], r["nmax"], len(ms), ms.mean(), ms.max()))
            self.assertEqual(r["ended"], "complete")
            self.assertGreater(r["hmin"], 0.0)

    def test_closed_loop_motor_flat_sea(self):
        """baseline-2m at u10 9 (below the soaring minimum of the library) on a flat sea with the motor
        (normal level): flies 60 s, and the motor supplies the energy the sea does not."""
        r = self.loop("flat", None, 60.0, name="baseline-2m", u10=9.0,
                      settings={"motor": True, "energyLevel": "normal"})
        print("\nclosed loop with the motor, baseline-2m u10 9, flat sea, 60 s: %s at %.1f s, min height %.2f m, "
              "average power %.1f W, thrust on %.2f of the time, %d bursts"
              % (r["ended"], r["tEnd"], r["hmin"], r["avgPowerW"], r["onShare"], r["nBurst"]))
        self.assertEqual(r["ended"], "complete")
        self.assertGreater(r["hmin"], 0.0)
        self.assertGreater(r["avgPowerW"], 1.0)
        self.assertLess(r["avgPowerW"], 40.0)

    def test_closed_loop_wave_sea_timing(self):
        """Informational: the 96 component sea of the Python margin experiment, truth world with
        the 48 strongest components. Only checks that it runs and does not leave the loop broken."""
        env = make_env("waves", U10)
        comps = {k: [float(v) for v in env.c[k]] for k in ("a", "k", "omega", "phase", "dx", "dy")}
        comps["urel"] = [float(v) for v in env.urel]
        r = self.loop("truth", comps, 20.0)
        ms = np.array(r["ms"])
        print("\nclosed loop, wave sea (96 components), truth world (48 used), 20 s: %s at %.1f s, min height "
              "%.2f m; planning %d replans, mean %.1f ms, max %.1f ms"
              % (r["ended"], r["tEnd"], r["hmin"], len(ms), ms.mean(), ms.max()))
        self.assertTrue(np.isfinite(ms).all())


if __name__ == "__main__":
    unittest.main()
