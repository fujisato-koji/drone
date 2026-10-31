"""The viewer's swell mode (sim/viewer/wave.js and the wave parts of mpc.js) against sim.wavelib and sim.mpc.

Needs node (skipped without it). The wave library is read from the viewer's export (sim/viewer/wave, python -m
sim.export_wave_library), so Python and JS plan with the same (rounded) numbers. Checked: the oracle sea (wind,
components, surface and wind field), the guide (phase lock, crest-ride family member, costs), the pilot (entry,
mirror and refusal over the whole grid), the flat start, the rollouts and the planner update with the running cost,
and a short closed loop with the same perturbations in both.
"""
import json
import shutil
import subprocess
import unittest
from pathlib import Path

import numpy as np

from sim.aircraft import Actuators
from sim.aircraft_models import by_name
from sim.export_wave_library import OUT_DIR, expand_entry, load_js
from sim.mpc import (CREST_CLIMB_M, WAVE_CREST_SIG, WAVE_CREST_SIG_THETA, WAVE_GUIDE_SIG, WAVE_SIG_THETA,
                     WAVE_SWITCH_MAX_WAIT_S, WAVE_SWITCH_THETA, WAVE_W_RUN, Cycle, CycleLibrary, MPPIPlanner,
                     WaveCycleGuide, derivatives_vec, energy_weight, world_swell, wrap)
from sim.wavelib import (CALM_U10, SWELL_DIRS, WavePilot, flat_start_entry, swell_env, wave_pkw)

VIEWER = Path(__file__).resolve().parents[1] / "viewer"
MPC_JS, WAVE_JS, FLIGHT_JS, NAV_JS = (VIEWER / "mpc.js", VIEWER / "wave.js", VIEWER / "flight.js",
                                      VIEWER / "navigator.js")
NODE = shutil.which("node")
NAME = "albatross-3m"

SNIPPET = r"""
const MPC = require(process.argv[1]);
const WAVE = require(process.argv[2]);
const Flight = require(process.argv[3]);
const NAV = require(process.argv[4]);
const inp = JSON.parse(require('fs').readFileSync(0, 'utf8'));
const out = {};
const flat = (a) => Array.from(a);
const ex = (es) => es.map(WAVE.expandEntry);
if (inp.run === 'consts') {
  out.guide = WAVE.GUIDE; out.dirs = WAVE.SWELL_DIRS; out.calm = WAVE.CALM_U10;
} else if (inp.run === 'sea') {
  out.cases = inp.cases.map((c) => {
    const sp = WAVE.swellSea(c.set, inp.index);
    const w = MPC.makeWorld(sp), o = [0, 0, 0];
    const pts = c.points.map((p) => { w.windAt(p[0], p[1], p[2], p[3], o); return [w.eta(p[0], p[1], p[3]), o[0], o[1], o[2]]; });
    return { wind: sp.wind, comps: sp.comps, pts: pts, z0: w.z0 };
  });
} else if (inp.run === 'guide') {
  const es = ex(inp.entries);
  out.cases = inp.cases.map((c) => {
    const world = MPC.makeWorld(c.world);
    const cyc = WAVE.waveCycle(es[c.i], c.mirror);
    const g = new WAVE.WaveCycleGuide(cyc, world, c.t, c.s, c.psiW, 6.0, { tauHint: c.hint, wRun: 5.0 });
    const kt = MPC.linspace(0.0, 6.0, 24);
    return { tau0: g.tau0, md: g.matchDist, te: g.thetaErr0, target: g.target, thT: g.thetaT, dEc: g.dEc,
      E0: g.E0, member: g.member, nominal: flat(g.nominal(kt)), thrust: flat(g.nominalThrust(kt)),
      run: c.probe.map((p) => g.runningCost(world, p[0], p[1])),
      term: c.probe.map((p) => g.terminalCost(world, p[0], p[1], { wE: 1.0 })) };
  });
} else if (inp.run === 'pilot') {
  const es = ex(inp.entries);
  const pilots = {};
  out.picks = inp.cases.map((c) => {
    const key = c.name + '|' + c.preset;
    if (!pilots[key]) pilots[key] = new WAVE.WavePilot(es, c.preset, { flat: inp.flat[c.name] });
    const p = pilots[key];
    const world = c.world ? MPC.makeWorld(c.world) : MPC.makeWorld(WAVE.swellSea(c.set, inp.index));
    const cy = p.cycle(c.name, world, c.u10, c.chi, c.psiW);
    return cy === null ? null : [es.indexOf(p.last), cy.mirror];
  });
} else if (inp.run === 'start') {
  out.entries = inp.cases.map((c) => WAVE.flatStartEntry(NAV, MPC.makeAircraft(inp.acs[c.name]), c.u10, c.course));
} else if (inp.run === 'rollout' || inp.run === 'plan') {
  const ac = MPC.makeAircraft(inp.ac);
  const world = MPC.makeWorld(inp.world);
  const pl = new MPC.Planner(ac, inp.settings);
  const A = inp.actuators;
  pl.act = MPC.actuatorParams(ac, {rateModel: A.rate_model, tau: A.tau_s, clRate: A.cl_rate_per_s, kPb2v: A.k_pb2v,
    delay: A.delay_s, thrustTau: A.thrust_tau_s});
  const es = ex(inp.entries);
  const cyc = WAVE.waveCycle(es[inp.i], inp.mirror);
  const g = new WAVE.WaveCycleGuide(cyc, world, inp.t, inp.s, inp.psiW, pl.horizonS, { wRun: 5.0, wD: pl.wD });
  out.basis = pl.basis.map(flat);
  if (inp.run === 'rollout') {
    pl.actState = inp.actState;
    const S = inp.Us.length;
    const Us = Float64Array.from([].concat(...inp.Us.map(u => [].concat(...u))));
    const r = pl.rollout(world, inp.t, inp.s, Us, S, g, true);
    out.cost = flat(r.cost); out.dead = flat(r.dead); out.paths = r.paths.map(flat);
  } else {
    out.U = [];
    inp.eps.forEach((eps) => {
      pl.epsHook = () => Float64Array.from([].concat(...[].concat(...eps)));
      pl.actState = inp.actState;
      pl.plan(world, inp.t, inp.s, g);
      out.U.push(flat(pl.U));
    });
  }
} else if (inp.run === 'loop') {
  // the closed loop of sim.mpc.fly_mpc with a wave library and a start entry (sim.wavelib.run_wave_job), with the
  // planner's perturbations given (one array per plan)
  const ac = MPC.makeAircraft(inp.ac), G = 9.81;
  const spec = inp.world, world = MPC.makeWorld(spec);
  const windFn = (x, y, z, t) => { const o = [0, 0, 0]; world.windAt(x, y, z, t, o); return o; };
  windFn.z0 = world.z0;
  const ctl = new WAVE.WaveController(inp.ac, ex(inp.entries), { settings: inp.settings, actuators: inp.actuators,
    preset: inp.preset, flat: inp.flat, startEntry: inp.start });
  const A = inp.actuators;
  const act = new MPC.Actuators(ac, {rateModel: A.rate_model, tau: A.tau_s, clRate: A.cl_rate_per_s, kPb2v: A.k_pb2v,
    delay: A.delay_s, thrustTau: A.thrust_tau_s});
  let s = inp.s0.slice();
  const dt = 0.01, n = Math.round(inp.T / dt), every = Math.round(ctl.pl.replanS / dt);
  const tMax = ctl.pl.tMax, foldDcd = ctl.pl.foldDcd;
  let plan = null, tPlan = 0, ended = 'complete', np_ = 0, eJ = 0, nOn = 0;
  const states = [], ms = [], infos = [];
  for (let i = 0; i <= n; i++) {
    const t = i * dt;
    const eta = world.eta(s[0], s[1], t);
    if (s[2] - eta < 0) { ended = 'contact'; break; }
    if (s[2] - eta - 0.5 * ac.span * Math.sin(Math.abs(act.mu === null ? 0 : act.mu)) * Math.cos(s[4]) < 0) { ended = 'tip contact'; break; }
    if (!s.every(Number.isFinite)) { ended = 'non-finite'; break; }
    if (plan === null || i % every === 0) {
      const e = inp.eps[np_++];
      ctl.pl.epsHook = () => Float64Array.from([].concat(...[].concat(...e)));
      const t0 = process.hrtime.bigint();
      plan = ctl.plan({ t: t, s: s.slice(), act: act.cl === null ? null :
        { cl: act.cl, mu: act.mu, clCmd: act.cmdLast[0], muCmd: act.cmdLast[1], T: act.T },
        world: spec, u10: inp.u10, chi: inp.chi });
      ms.push(Number(process.hrtime.bigint() - t0) / 1e6);
      tPlan = t; states.push([t].concat(s)); infos.push(plan.info);
    }
    let [cl, mu] = plan.controls(t - tPlan);
    cl = Math.min(Math.max(cl, ac.cl_min), ac.cl_max); mu = Math.min(Math.max(mu, -ac.mu_max), ac.mu_max);
    cl = MPC.nLimitedCl(ac, s[3], cl);
    const tc = Math.min(Math.max(MPC.planThrust(plan, t - tPlan), 0), tMax);
    const a = act.step(cl, mu, dt, s[3], tc);
    eJ += act.T * s[3] * dt / ctl.pl.eta;
    if (act.T > 0.01 * ac.mass * G) nOn++;
    if (i === n) break;
    s = Flight.rk4(s, a[0], a[1], ac, windFn, t, dt, true, act.T, MPC.foldEffective(ac, foldDcd, act.T, tc));
  }
  out.ended = ended; out.states = states; out.ms = ms; out.switches = ctl.switches; out.eJ = eJ; out.nOn = nOn;
  out.infos = infos.map((f) => ({ ref: f.ref, waiting: f.waiting, thetaErr: f.thetaErr, courseRelDeg: f.courseRelDeg }));
}
console.log(JSON.stringify(out));
"""


def run_node(payload):
    r = subprocess.run([NODE, "-e", SNIPPET, str(MPC_JS), str(WAVE_JS), str(FLIGHT_JS), str(NAV_JS)],
                       input=json.dumps(payload), capture_output=True, text=True, timeout=900)
    if r.returncode != 0:
        raise RuntimeError(r.stderr[-4000:])
    return json.loads(r.stdout)


def ac_params(ac):
    return {"name": ac.name, "mass": ac.mass, "span": ac.span, "wing_area": ac.wing_area,
            "cd0": ac.cd0, "oswald": ac.oswald, "cl_max": ac.cl_max, "cl_min": ac.cl_min,
            "mu_max": ac.mu_max, "n_max": ac.n_max, "rho": ac.rho}


def py_entry(d):
    """A slim exported entry as sim.mpc.Cycle and sim.wavelib.WavePilot take it (x and y are not used)."""
    e = expand_entry(d)
    z = [0.0] * len(e["t"])
    return dict(e, x=z, y=z)


def js_world(env):
    """The world spec of mpc.js for a swell_env (its own components and wind, so the JS planner sees the same sea)."""
    w = env.wind
    return {"kind": "truth", "wind": {"ustar": float(w.ustar), "z0": float(w.z0), "ex": float(w.ex), "ey": float(w.ey),
                                      "karman": 0.4},
            "comps": dict({k: [float(v) for v in env.c[k]] for k in ("a", "k", "omega", "phase", "dx", "dy")},
                          urel=[float(v) for v in env.urel]),
            "stokes": True, "waveWind": {"amp": list(env._amp), "decay": list(env.decay)}, "maxComponents": 0}


def close(a, b, rtol, atol):
    a, b = np.asarray(a, float), np.asarray(b, float)
    return bool(np.all(np.abs(a - b) <= atol + rtol * np.abs(b))), float(np.max(np.abs(a - b) / (atol + np.abs(b))))


def loop_eps_recorder():
    """A subclass of MPPIPlanner that draws its perturbations as plan() does and keeps them (for the JS loop)."""
    import sim.mpc as M
    made = []

    class Rec(M.MPPIPlanner):
        def __init__(self, *a, **k):
            super().__init__(*a, **k)
            self.rec = []
            self.eps_hook = self._draw
            made.append(self)

        def _draw(self, it):
            sig = np.array([[self.sigma_cl], [self.sigma_mu]])
            eps = self.rng.standard_normal((self.n_samples, 2, self.n_ctrl)) * sig
            if self.smooth_modes:
                z = self.rng.standard_normal((self.n_samples, 2, self.smooth_modes))
                eps += self.smooth_gain * sig[None, :, :] / np.sqrt(self.smooth_modes) * (z @ self._basis)
            if self.motor:
                eps = np.concatenate([eps, self._thrust_noise(self.n_samples)], axis=1)
            eps[0] = 0.0
            self.rec.append(eps.copy())
            return eps

    return Rec, made


@unittest.skipUnless(NODE, "node is not on PATH: wave.js tests skipped")
class TestWaveJs(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.index = load_js(OUT_DIR / "index.js")
        cls.files = {k: load_js(OUT_DIR / s["file"]) for k, s in cls.index["seas"].items()}
        cls.key = "p63_hs3_tp10_" + NAME
        cls.entries = cls.files[cls.key]
        cls.ac = by_name(NAME)

    # ---- constants and the export
    def test_constants_match_python(self):
        js = run_node({"run": "consts"})
        g = js["guide"]
        self.assertEqual(g["sig"], list(WAVE_GUIDE_SIG))
        self.assertEqual(g["crestSig"], list(WAVE_CREST_SIG))
        self.assertEqual((g["sigTheta"], g["crestSigTheta"], g["wRun"], g["crestClimbM"], g["switchTheta"],
                          g["switchMaxWaitS"]),
                         (WAVE_SIG_THETA, WAVE_CREST_SIG_THETA, WAVE_W_RUN, CREST_CLIMB_M, WAVE_SWITCH_THETA,
                          WAVE_SWITCH_MAX_WAIT_S))
        self.assertEqual(js["dirs"], SWELL_DIRS)
        self.assertEqual(js["calm"], CALM_U10)
        ix = self.index
        self.assertEqual(ix["guide"]["sig"], list(WAVE_GUIDE_SIG))
        names = {"n_samples": "nSamples", "dt": "dt", "n_iter": "nIter", "n_ctrl": "nCtrl", "lam": "lam",
                 "sigma_cl": "sigmaCl", "sigma_mu": "sigmaMu", "clearance": "clearance", "h_clear": "hClear",
                 "thrust_sampling": "thrustSampling", "fold_switch": "foldSwitch", "thrust_update": "thrustUpdate",
                 "h_ceil": "hCeil", "smooth_modes": "smoothModes"}
        for k, v in wave_pkw().items():
            self.assertEqual(ix["planner"][names[k]], v, k)
        a = Actuators(self.ac, rate_model="pb2v")
        A = ix["actuators"]
        self.assertEqual((A["tau_s"], A["cl_rate_per_s"], A["k_pb2v"], A["delay_s"], A["thrust_tau_s"]),
                         (a.tau, a.cl_rate, a.k_pb2v, a.delay, a.thrust_tau))
        for n in ix["grid"]["aircraft"]:
            self.assertAlmostEqual(ix["w_energy"][n], energy_weight(by_name(n), "normal"), places=14)

    def test_export_covers_the_library(self):
        """Every converged library entry is in its sea file, in the library's order, with the same choice fields;
        each file is stamped and small."""
        from sim.wavelib import WAVE_LIB_PATH
        with open(WAVE_LIB_PATH) as fh:
            lib = [e for e in json.load(fh)["entries"] if e.get("success")]
        n = sum(len(v) for v in self.files.values())
        self.assertEqual(n, len(lib))
        pos = {k: 0 for k in self.files}
        for e in lib:
            k = "%s_hs%s_tp%s_%s" % (e["preset"], ("%g" % e["hs"]).replace(".", "p"), ("%g" % e["tp"]).replace(".", "p"),
                                     e["aircraft"])
            d = self.files[k][pos[k]]
            pos[k] += 1
            for f in ("direction", "u10_flown", "rel_deg", "kind", "crest"):
                self.assertEqual(d[f], bool(e.get(f)) if f == "crest" else e[f], f)
            if e.get("crest"):
                self.assertAlmostEqual(d["ride"]["v_c"], e["ride"]["v_c"], places=9)
            x = expand_entry(d)
            for f in ("z", "V", "theta", "cl"):
                ok, rel = close(x[f], e[f], 1e-4, 1e-6)
                self.assertTrue(ok, (k, f, rel))
        for s in self.index["seas"].values():
            p = OUT_DIR / s["file"]
            self.assertTrue(p.read_text(encoding="utf-8").startswith("// tdr-run: python -m sim.export_wave_library"))
            self.assertLess(p.stat().st_size, 250_000, s["file"])
        self.assertLess((OUT_DIR / "index.js").stat().st_size, 60_000)

    # ---- the oracle sea
    def test_sea_matches_swell_env(self):
        rng = np.random.default_rng(11)
        cases, py = [], []
        for u10 in (0.0, 0.05, 3.0, 6.0, 9.0, 12.5):
            for direction, from_deg, preset, hs, tp in (("along", 270.0, "p63", 3.0, 10.0),
                                                        ("across", 0.0, "p64_low", 2.0, 8.0),
                                                        ("against", 123.4, "p63", 1.5, 12.0),
                                                        (-90.0, 300.0, "p64_low", 3.0, 12.0)):
                env = swell_env(u10, hs, tp, direction, preset, from_deg=from_deg)
                pts = []
                for _ in range(30):
                    x, y, t = rng.uniform(-200, 200), rng.uniform(-200, 200), rng.uniform(0, 120)
                    eta = float(env.surface(x, y, t)[0])
                    h = float(rng.choice([rng.uniform(-0.2, 0.3), rng.uniform(0.0, 3.0), rng.uniform(2, 40)]))
                    pts.append([float(x), float(y), eta + h, float(t)])
                pts.append([5.0, 7.0, float(env.surface(5.0, 7.0, 1.0)[0]) + 1e-6, 1.0])      # inside the roughness layer
                P = np.array(pts)
                eta = np.asarray(env.surface(P[:, 0], P[:, 1], P[:, 3])[0])
                wv = np.asarray(env.wind_at(P[:, 0], P[:, 1], P[:, 2], P[:, 3]))
                py.append((env, np.column_stack([eta, wv])))
                cases.append({"set": {"u10": u10, "fromDeg": from_deg, "hs": hs, "tp": tp, "direction": direction,
                                      "preset": preset}, "points": pts})
        js = run_node({"run": "sea", "index": self.index, "cases": cases})
        worst_w, worst_p, n_orb = 0.0, 0.0, 0
        for (env, ref), c, cs in zip(py, js["cases"], cases):
            w = env.wind
            got = [c["wind"][k] for k in ("ustar", "z0", "ex", "ey")]
            ok, rel = close(got, [w.ustar, w.z0, w.ex, w.ey], 1e-13, 1e-15)
            self.assertTrue(ok, (cs["set"], rel))
            worst_w = max(worst_w, rel)
            for k in ("a", "k", "omega", "phase", "dx", "dy"):
                ok, rel = close(c["comps"][k], env.c[k], 1e-14, 1e-15)
                self.assertTrue(ok, (k, rel))
            ok, rel = close(c["comps"]["urel"], env.urel, 1e-12, 1e-14)
            self.assertTrue(ok, ("urel", rel))
            ok, rel = close(c["pts"], ref, 1e-11, 1e-13)
            self.assertTrue(ok, (cs["set"], rel))
            worst_p = max(worst_p, rel)
            P = np.array(cs["points"])
            n_orb += int(np.sum(P[:, 2] - ref[:, 0] <= env.wind.z0))
        print("\noracle sea: wind and components max rel diff %.1e, surface and wind field %.1e (%d points in the "
              "roughness layer)" % (worst_w, worst_p, n_orb))
        self.assertGreater(n_orb, 0)

    # ---- the guide
    def guide_cases(self):
        rng = np.random.default_rng(3)
        es = [py_entry(d) for d in self.entries]
        crest = [i for i, e in enumerate(es) if e["crest"] and e["ride"].get("family") and e["u10_flown"] == 6.0][:2]
        motor = [i for i, e in enumerate(es) if e["crest"] and e["kind"] == "motor"][:1]
        cyc = [i for i, e in enumerate(es) if not e["crest"] and e["u10_flown"] == 6.0][:2]
        self.assertTrue(crest and cyc, "the test needs crest rides with families and optimizer cycles")
        env = swell_env(6.0, 3.0, 10.0, "along", "p63")
        psi_w = float(np.arctan2(env.wind.ey, env.wind.ex))
        cases, py = [], []
        for i in crest + motor + cyc:
            for mirror in (False, True):
                for hint in (None, 1.7):
                    t = float(rng.uniform(0, 50))
                    x, y = float(rng.uniform(-100, 100)), float(rng.uniform(-100, 100))
                    eta = float(env.surface(x, y, t)[0])
                    s = [x, y, eta + float(rng.uniform(1.5, 12.0)), float(rng.uniform(12, 20)),
                         float(rng.uniform(-0.2, 0.2)), psi_w + float(rng.uniform(-1.0, 1.0))]
                    c = Cycle(es[i], mirror=mirror)
                    g = WaveCycleGuide(c, env, t, np.array(s), psi_w, 6.0, tau_hint=hint, w_run=5.0)
                    probe = []
                    for _ in range(4):
                        tt = t + float(rng.uniform(0.1, 6.0))
                        ss = [x + float(rng.uniform(-80, 80)), y + float(rng.uniform(-30, 30)), 0.0,
                              float(rng.uniform(11, 21)), float(rng.uniform(-0.3, 0.3)), psi_w + float(rng.uniform(-3, 3))]
                        ss[2] = float(env.surface(ss[0], ss[1], tt)[0]) + float(rng.uniform(0.5, 15))
                        probe.append([tt, ss])
                    kt = np.linspace(0.0, 6.0, 24)
                    py.append({"tau0": g.tau0, "md": g.match_dist, "te": g.theta_err0, "target": list(g.target),
                               "thT": g.theta_t, "dEc": g.dE_c, "E0": g.E0,
                               "member": None if g.member is None else list(g.member),
                               "nominal": g.nominal(kt).ravel(), "thrust": g.nominal_thrust(kt),
                               "run": [float(g.running_cost(env, p[0], np.array(p[1]))) for p in probe],
                               "term": [float(g.terminal_cost(env, p[0], np.array(p[1]),
                                                              type("P", (), {"w_e": 1.0})())) for p in probe]})
                    cases.append({"i": i, "mirror": mirror, "hint": hint, "t": t, "s": s, "psiW": psi_w,
                                  "world": js_world(env), "probe": probe})
        return cases, py

    def test_guide_matches_python(self):
        cases, py = self.guide_cases()
        js = run_node({"run": "guide", "entries": self.entries, "cases": cases})["cases"]
        worst = 0.0
        for a, b, c in zip(py, js, cases):
            self.assertAlmostEqual(b["tau0"], a["tau0"], delta=1e-12, msg=c["i"])
            self.assertEqual(b["member"] is None, a["member"] is None)
            for k in ("md", "te", "target", "thT", "dEc", "E0", "nominal", "thrust", "run", "term") + (
                    ("member",) if a["member"] is not None else ()):
                ok, rel = close(b[k], a[k], 1e-10, 1e-12)
                self.assertTrue(ok, (k, c["i"], c["mirror"], rel))
                worst = max(worst, rel)
        print("\nwave guide: %d cases (crest families, a motor ride, optimizer cycles, mirrored, with a hint), max "
              "rel diff %.1e" % (len(cases), worst))

    # ---- the pilot
    def test_pilot_matches_python_over_the_grid(self):
        """Every sea of the export, every aircraft and preset, the swell along, across (left and right) and against,
        winds of 0 to 12 m/s and three courses: the same entry, mirror and refusal as sim.wavelib.WavePilot."""
        lib = CycleLibrary.from_keyword("motor")
        all_js = [d for k in sorted(self.files) for d in self.files[k]]
        all_py = [py_entry(d) for d in all_js]
        pilots = {}
        cases, py = [], []
        for name in self.index["grid"]["aircraft"]:
            for preset in ("p63", "p64_low"):
                pilots[(name, preset)] = WavePilot(all_py, preset, policy="energy", flat_entries=lib.entries)
                for hs in (1.5, 2.0, 3.0):
                    for tp in (8.0, 10.0, 12.0):
                        for direction in ("along", "across", "against", -90.0):
                            for u10 in range(0, 13):
                                env = swell_env(float(u10), hs, tp, direction, preset)
                                psi_w = float(np.arctan2(env.wind.ey, env.wind.ex))
                                rel = SWELL_DIRS[direction] if isinstance(direction, str) else direction
                                for dc in (0.0, 35.0, -35.0):
                                    chi = float(wrap(np.radians(rel + dc)))
                                    p = pilots[(name, preset)]
                                    c = p.cycle(name, env, max(float(u10), CALM_U10), chi, psi_w)
                                    py.append(None if c is None else [next(i for i, e in enumerate(all_py)
                                                                           if e is p.last), c.mirror])
                                    cases.append({"name": name, "preset": preset, "u10": max(float(u10), CALM_U10),
                                                  "chi": chi, "psiW": psi_w, "world": js_world(env)})
        js = run_node({"run": "pilot", "entries": all_js, "flat": self.index["flat"], "cases": cases})["picks"]
        diff = [(c["name"], c["preset"], c["u10"], a, b) for a, b, c in zip(py, js, cases) if a != b]
        n_none = sum(1 for a in py if a is None)
        n_mir = sum(1 for a in py if a and a[1])
        print("\nwave pilot: %d cases, %d refused (fly flat), %d mirrored, %d differ" % (len(cases), n_none, n_mir,
                                                                                       len(diff)))
        self.assertEqual(diff[:5], [])
        self.assertGreater(n_none, 0)
        self.assertGreater(n_mir, 0)

    def test_pilot_on_the_js_sea(self):
        """The same choice when JS builds the sea itself (swellSea) for the viewer's settings."""
        lib = CycleLibrary.from_keyword("motor")
        es = [py_entry(d) for d in self.entries]
        p = WavePilot(es, "p63", policy="energy", flat_entries=lib.entries)
        cases, py = [], []
        for direction in ("along", "across", "against"):
            for u10 in range(0, 13):
                env = swell_env(float(u10), 3.0, 10.0, direction, "p63")
                psi_w = float(np.arctan2(env.wind.ey, env.wind.ex))
                chi = float(np.radians(SWELL_DIRS[direction]))
                c = p.cycle(NAME, env, max(float(u10), CALM_U10), chi, psi_w)
                py.append(None if c is None else [next(i for i, e in enumerate(es) if e is p.last), c.mirror])
                cases.append({"name": NAME, "preset": "p63", "u10": max(float(u10), CALM_U10), "chi": chi,
                              "psiW": psi_w, "set": {"u10": float(u10), "hs": 3.0, "tp": 10.0, "direction": direction,
                                                     "preset": "p63"}})
        js = run_node({"run": "pilot", "entries": self.entries, "flat": self.index["flat"], "index": self.index,
                       "cases": cases})["picks"]
        self.assertEqual(js, py)

    def test_flat_start_matches_python(self):
        cases, py = [], []
        acs = {n: ac_params(by_name(n)) for n in self.index["grid"]["aircraft"]}
        for n in acs:
            for u10 in (0.0, 3.0, 6.0, 9.0, 12.0):
                for course in (0.0, 90.0, 180.0):
                    py.append(flat_start_entry(by_name(n), u10, course))
                    cases.append({"name": n, "u10": u10, "course": course})
        js = run_node({"run": "start", "acs": acs, "cases": cases})["entries"]
        for a, b in zip(py, js):
            for k in ("z", "V", "psi", "cl", "thrust", "gamma", "mu", "t"):
                ok, rel = close(b[k], a[k], 1e-12, 1e-12)
                self.assertTrue(ok, (k, rel))
            self.assertEqual(b["period"], a["period"])

    # ---- the planner with the wave guide
    def planner_case(self):
        ac = self.ac
        env = swell_env(6.0, 3.0, 10.0, "along", "p63")
        psi_w = float(np.arctan2(env.wind.ey, env.wind.ex))
        es = [py_entry(d) for d in self.entries]
        i = next(i for i, e in enumerate(es) if e["crest"] and e["ride"].get("family") and e["u10_flown"] == 6.0)
        pkw = dict(wave_pkw(), motor=True, w_energy=energy_weight(ac, "normal"))
        pl = MPPIPlanner(ac, **pkw)
        pl.n_limit = True
        pl.act = Actuators(ac, rate_model="pb2v")
        t = 12.3
        x, y = 30.0, -4.0
        s0 = np.array([x, y, float(env.surface(x, y, t)[0]) + 4.0, 16.0, -0.02, psi_w - 0.4])
        pl.act_state = (0.5, 0.05, 0.55, 0.08, 1.2)
        g = WaveCycleGuide(Cycle(es[i], mirror=False), env, t, s0, psi_w, pl.horizon_s, w_d=pl.w_d, w_run=5.0)
        pl.guide = g
        settings = dict(self.index["planner"])
        payload = {"ac": ac_params(ac), "world": js_world(env), "settings": settings,
                   "actuators": self.index["actuators"], "entries": self.entries, "i": i, "mirror": False,
                   "t": t, "s": [float(v) for v in s0], "psiW": psi_w, "actState": list(pl.act_state)}
        return pl, g, env, s0, t, payload

    def test_rollout_with_the_running_cost(self):
        pl, g, env, s0, t, payload = self.planner_case()
        ac, nc, tm = self.ac, pl.n_ctrl, pl.t_max
        nominal = np.vstack([g.nominal(pl.knots_t), np.clip(g.nominal_thrust(pl.knots_t), 0, tm)])
        turn = np.stack([np.full(nc, ac.cl_best_ld), np.linspace(0.0, 0.7, nc), np.full(nc, 0.1 * tm)])
        dive = np.stack([np.full(nc, 0.05), np.full(nc, 0.3), np.zeros(nc)])
        climb = np.stack([np.full(nc, 0.9 * ac.cl_max), np.zeros(nc), np.full(nc, 0.4 * tm)])
        Us = np.stack([nominal, turn, dive, climb])
        cost, path, dead = pl.rollout(env, t, s0, Us, np.array([1.0, 0.0]))
        js = run_node(dict(payload, run="rollout", Us=Us.tolist()))
        self.assertEqual(js["dead"], [int(v) for v in dead])
        ok_c, rel_c = close(js["cost"], cost, 1e-11, 1e-12)
        jsp = np.array(js["paths"]).reshape(np.transpose(path, (2, 0, 1)).shape)
        ok_p, rel_p = close(jsp, np.transpose(path, (2, 0, 1)), 1e-11, 1e-12)
        # the running cost is part of it: without it the costs differ
        g.w_run = 0.0
        cost0, _, _ = pl.rollout(env, t, s0, Us, np.array([1.0, 0.0]))
        print("\nrollout over the swell with the wave guide: dead %s, cost max rel diff %.1e, states %.1e; running "
              "cost share of the nominal %.2f" % ([int(v) for v in dead], rel_c, rel_p, 1 - cost0[0] / cost[0]))
        self.assertTrue(dead.any() and not dead.all())
        self.assertTrue(ok_c and ok_p, (rel_c, rel_p))
        self.assertGreater(abs(cost[0] - cost0[0]), 1e-3 * abs(cost[0]))
        ok_b, _ = close(js["basis"], pl._basis, 1e-14, 1e-15)
        self.assertTrue(ok_b and len(js["basis"]) == 3, "smooth basis")

    def test_plan_with_injected_perturbations(self):
        pl, g, env, s0, t, payload = self.planner_case()
        rng = np.random.default_rng(7)
        S, nc = pl.n_samples, pl.n_ctrl
        sig = np.array([[pl.sigma_cl], [pl.sigma_mu]])
        eps = [np.concatenate([rng.standard_normal((S, 2, nc)) * sig * 2.0,
                               rng.standard_normal((S, 1, nc)) * pl.sigma_t * 2.0], axis=1) for _ in range(2)]
        out = []
        for e in eps:
            pl.eps_hook = lambda it, e=e: e.copy()
            pl.plan(env, t, s0, np.array([1.0, 0.0]), guide=g)
            out.append(pl.U.copy())
        js = run_node(dict(payload, run="plan", eps=[e.tolist() for e in eps]))
        for k in range(2):
            ok, rel = close(js["U"][k], out[k].ravel(), 1e-9, 1e-12)
            print("\nwave plan %d with injected perturbations: max rel diff %.1e" % (k, rel))
            self.assertTrue(ok, rel)

    # ---- the closed loop
    def test_short_closed_loop_matches_python(self):
        """sim.wavelib.run_wave_job's loop (albatross-3m, 3 m swell of 10 s along a 6 m/s wind, P63) for 20 s with
        the same perturbations in Python and in node: the same entry onto the crest ride at the same plan, and states
        that agree at every plan."""
        import sim.mpc as M
        from sim.wavelib import WAVE_EXP_TP
        Rec, made = loop_eps_recorder()
        ac, u10, hs, d, preset, T = self.ac, 6.0, 3.0, "along", "p63", 20.0
        env = swell_env(u10, hs, WAVE_EXP_TP, d, preset)
        lib = CycleLibrary.from_keyword("motor")
        es = [py_entry(x) for x in self.entries]
        pilot = WavePilot(es, preset, policy="energy", flat_entries=lib.entries)
        bearing = (env.wind.to_deg - SWELL_DIRS[d]) % 360.0
        start = flat_start_entry(ac, u10, SWELL_DIRS[d])
        orig = M.MPPIPlanner
        M.MPPIPlanner = Rec
        try:
            r = M.fly_mpc(ac, env, "truth", [], T, bearing, max(u10, CALM_U10), seed=1, planner_kw=wave_pkw(),
                          library=lib, state_source="truth", wind_source="truth",
                          actuators=Actuators(ac, rate_model="pb2v"), n_limit=True, stop_on_tip=True,
                          wave_aware=False, motor=True, w_energy=M.energy_weight(ac, "normal"), keep=True,
                          wave_library=pilot, start_entry=start)
        finally:
            M.MPPIPlanner = orig
        pl = made[-1]
        tr = r["traj"]
        Tt, ST = np.asarray(tr["t"]), np.asarray(tr["state"])
        u = np.array([np.sin(np.radians(bearing)), np.cos(np.radians(bearing))])
        psi_w = float(np.arctan2(env.wind.ey, env.wind.ex))
        chi = float(wrap(np.arctan2(u[1], u[0]) - psi_w))
        c0 = Cycle(start)
        s0 = [0.0, 0.0, float(env.surface(0.0, 0.0, 0.0)[0]) + float(c0.z[0]), float(c0.V[0]), float(c0.g[0]),
              psi_w + float(c0.psi[0])]
        self.assertTrue(np.allclose(ST[0], s0, rtol=0, atol=1e-12))
        js = run_node({"run": "loop", "ac": ac_params(ac), "world": js_world(env), "settings": self.index["planner"],
                       "actuators": self.index["actuators"], "entries": self.entries, "preset": preset,
                       "flat": self.index["flat"][NAME], "start": start, "s0": s0, "u10": max(u10, CALM_U10),
                       "chi": chi, "T": T, "eps": [e.tolist() for e in pl.rec]})
        self.assertEqual(js["ended"], r["ended"])
        sw_py, sw_js = r["wave_switches"], js["switches"]
        self.assertEqual(len(sw_js), len(sw_py))
        self.assertTrue(sw_py, "the loop should enter the wave library within 20 s")
        for a, b in zip(sw_py, sw_js):
            self.assertAlmostEqual(a[0], b[0], places=9)
            self.assertAlmostEqual(a[1], b[1], places=6)
            self.assertEqual((a[2], a[3]), (b[2], b[3]))
        worst, worst_t = 0.0, 0.0
        for row in js["states"]:
            i = int(round(row[0] / 0.01))
            dlt = float(np.max(np.abs(np.array(row[1:]) - ST[i])))
            if dlt > worst:
                worst, worst_t = dlt, row[0]
        P = np.asarray(tr["thrust"]) * ST[:, 3] / r["motor"]["eta_total"]
        ej_py = float(P.sum() * 0.01)
        ms = np.array(js["ms"])
        print("\nshort closed loop over the swell, 20 s: entry at %.1f s onto course %.1f deg (crest %s) in both; "
              "states at the %d plans agree to %.1e (largest at %.1f s); energy %.2f J in node, %.2f J in Python; "
              "node plans mean %.1f ms" % (sw_py[0][0], sw_py[0][1], sw_py[0][2], len(js["states"]), worst, worst_t,
                                          js["eJ"], ej_py, ms.mean()))
        self.assertLess(worst, 1e-6)
        self.assertAlmostEqual(js["eJ"], ej_py, delta=1e-6 * max(1.0, ej_py))

    def test_syntax(self):
        r = subprocess.run([NODE, "--check", str(WAVE_JS)], capture_output=True, text=True)
        self.assertEqual(r.returncode, 0, r.stderr)


if __name__ == "__main__":
    unittest.main()
