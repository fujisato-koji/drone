// Receding-horizon planner for the viewer: a JavaScript mirror of sim/mpc.py in library mode
// (MPPIPlanner + CycleLibrary + CycleGuide, margin library, pb2v actuators, load-factor limit).
// Classic script. Everything lives in mpcFactory(), so that the same source text can be turned
// into a Web Worker through a Blob URL (MPC.workerSource()); that works from file:// where a
// worker script cannot be loaded from a file URL. Attaches MPC to globalThis and sets
// module.exports under node.
//
// Sim coordinates: x east, y north, z up. State s = [x, y, z, V, gamma, psi] as in flight.js.
// Planner settings are the ones of sim.mpc.LIB_PKW. Motor option (settings motor: true): the thrust
// T is a third control per knot, as sim.mpc.MPPIPlanner(motor=True); see the notes there. Not mirrored (the Python code has them, the
// library mode of the margin experiment does not use them): the wave-aware phase shift of the
// guide (wave_aware False there), smooth perturbation modes (smooth_modes 0), the non-library
// cost terms (progress, smoothness), and the estimators.
function mpcFactory() {
  'use strict';
  var G = 9.81, H = 0.05, DT = 0.01, TWO_PI = 2 * Math.PI;

  function wrap(a) { var x = a + Math.PI; return x - TWO_PI * Math.floor(x / TWO_PI) - Math.PI; }
  function clamp(v, lo, hi) { return Math.min(hi, Math.max(lo, v)); }
  // numpy.interp: xs increasing, clamped outside
  function interp(x, xs, ys) {
    var n = xs.length;
    if (x <= xs[0]) return ys[0];
    if (x >= xs[n - 1]) return ys[n - 1];
    var lo = 0, hi = n - 1;
    while (hi - lo > 1) { var mid = (lo + hi) >> 1; if (xs[mid] <= x) lo = mid; else hi = mid; }
    return (ys[hi] - ys[lo]) / (xs[hi] - xs[lo]) * (x - xs[lo]) + ys[lo];
  }
  function linspace(a, b, n, endpoint) {
    var out = new Float64Array(n), step = (b - a) / (endpoint === false ? n : n - 1);
    for (var i = 0; i < n; i++) out[i] = a + i * step;
    if (endpoint !== false && n > 1) out[n - 1] = b;
    return out;
  }
  function unwrap(a) {   // numpy.unwrap with period 2 pi
    var out = new Float64Array(a.length), corr = 0;
    out[0] = a[0];
    for (var i = 1; i < a.length; i++) {
      var dd = a[i] - a[i - 1];
      var dm = ((dd + Math.PI) - TWO_PI * Math.floor((dd + Math.PI) / TWO_PI)) - Math.PI;
      if (dm === -Math.PI && dd > 0) dm = Math.PI;
      var pc = dm - dd;
      if (Math.abs(dd) < Math.PI) pc = 0;
      corr += pc;
      out[i] = a[i] + corr;
    }
    return out;
  }

  // ---------------------------------------------------------------------------- aircraft
  // p: mass, span, wing_area, cd0, oswald, cl_max (and optionally cl_min, mu_max, n_max, rho).
  // Derived values are recomputed here (not taken from rounded exports), as sim.aircraft does.
  function makeAircraft(p) {
    var ac = {
      name: p.name, mass: p.mass, span: p.span, wing_area: p.wing_area, cd0: p.cd0,
      oswald: p.oswald, cl_max: p.cl_max,
      cl_min: p.cl_min !== undefined ? p.cl_min : -0.3,
      mu_max: p.mu_max !== undefined ? p.mu_max : 1.2,
      n_max: p.n_max !== undefined ? p.n_max : 4.0,
      rho: p.rho !== undefined ? p.rho : 1.225
    };
    var ar = ac.span * ac.span / ac.wing_area;
    ac.k_induced = 1.0 / (Math.PI * ac.oswald * ar);
    ac.cl_best_ld = Math.sqrt(ac.cd0 / ac.k_induced);
    ac.v_stall = Math.sqrt(2 * ac.mass * G / (ac.rho * ac.wing_area * ac.cl_max));
    return ac;
  }

  // ---------------------------------------------------------------------------- random numbers
  function mulberry32(a) {
    return function () {
      a = (a + 0x6D2B79F5) | 0;
      var t = Math.imul(a ^ (a >>> 15), 1 | a);
      t = (t + Math.imul(t ^ (t >>> 7), 61 | t)) ^ t;
      return ((t ^ (t >>> 14)) >>> 0) / 4294967296;
    };
  }
  function Gauss(seed) {
    var u = mulberry32(seed), spare = null;
    this.uniform = function () { return u(); };
    this.next = function () {
      if (spare !== null) { var r = spare; spare = null; return r; }
      var a;
      do { a = u(); } while (a <= 1e-12);
      var b = u(), rad = Math.sqrt(-2 * Math.log(a));
      spare = rad * Math.sin(TWO_PI * b);
      return rad * Math.cos(TWO_PI * b);
    };
  }

  // ---------------------------------------------------------------------------- worlds
  // A world has z0 (roughness length, picks the one-sided z difference near the surface),
  // eta(x, y, t) (surface height) and windAt(x, y, z, t, out) (out = [u, v, w], z absolute).
  // wind: {ustar, z0, ex, ey, karman}. comps: {a, k, omega, phase, dx, dy, urel} (arrays).
  // The formulas are those of the viewer (surface() and windAt() in index.html) and of
  // sim.seawind.Environment with the linear surface (stokes adds the second-order term to eta).
  function SeaWorld(wind, comps, stokes) {
    this.wind = wind; this.z0 = wind.z0;
    this.kar = wind.karman !== undefined ? wind.karman : 0.4;
    this.stokes = !!stokes;
    this.c = comps || { a: [], k: [], omega: [], phase: [], dx: [], dy: [], urel: [] };
    this.n = this.c.a.length;
    this.C = new Float64Array(this.n); this.S = new Float64Array(this.n);
  }
  SeaWorld.prototype.eta = function (x, y, t) {
    var c = this.c, n = this.n, eta = 0;
    for (var i = 0; i < n; i++) {
      var th = c.k[i] * (c.dx[i] * x + c.dy[i] * y) - c.omega[i] * t + c.phase[i];
      var cs = Math.cos(th);
      eta += c.a[i] * cs;
      if (this.stokes) eta += 0.5 * c.a[i] * c.a[i] * c.k[i] * (2 * cs * cs - 1);
    }
    return eta;
  };
  SeaWorld.prototype.windAt = function (x, y, z, t, o) {
    var c = this.c, n = this.n, C = this.C, S = this.S, eta = 0, i;
    for (i = 0; i < n; i++) {
      var th = c.k[i] * (c.dx[i] * x + c.dy[i] * y) - c.omega[i] * t + c.phase[i];
      var cs = Math.cos(th);
      C[i] = cs; S[i] = Math.sin(th);
      eta += c.a[i] * cs;
      if (this.stokes) eta += 0.5 * c.a[i] * c.a[i] * c.k[i] * (2 * cs * cs - 1);
    }
    var h = z - eta, w = this.wind, u = 0, v = 0, ww = 0;
    if (h <= w.z0) {   // inside the roughness layer: the orbital velocity of the surface
      for (i = 0; i < n; i++) {
        var ao = c.a[i] * c.omega[i];
        u += ao * C[i] * c.dx[i]; v += ao * C[i] * c.dy[i]; ww += ao * S[i];
      }
    } else {
      var sp = (w.ustar / this.kar) * Math.log(h / w.z0);
      u = sp * w.ex; v = sp * w.ey;
      for (i = 0; i < n; i++) {
        var k = c.k[i], f = c.urel[i] * c.a[i] * k * Math.exp(-k * h);
        var up = f * C[i];
        u += up * c.dx[i]; v += up * c.dy[i]; ww -= f * S[i];
      }
    }
    o[0] = u; o[1] = v; o[2] = ww;
  };
  // The nine wind samples of the finite differences of sim.aircraft.wind_derivs (point 0 at the
  // state, x +- H, y +- H, z + H, z - H (or z when z - H <= z0), t +- DT) written to P[0..26] in
  // one pass over the components. The seven distinct (x, y, t) positions share one cos and sin
  // per component (the others follow by angle addition) and the three z levels share one exp,
  // which makes it about three times faster than nine windAt calls with the same results.
  var CLS = [0, 1, 2, 3, 4, 0, 0, 5, 6];   // position class of each of the nine points
  SeaWorld.prototype._prep = function () {
    var c = this.c, n = this.n, K = {}, names = ['cdx', 'sdx', 'cdy', 'sdy', 'cw', 'sw', 'ekH', 'ekHi'], i;
    names.forEach(function (nm) { K[nm] = new Float64Array(n); });
    for (i = 0; i < n; i++) {
      var kx = c.k[i] * c.dx[i] * H, ky = c.k[i] * c.dy[i] * H, w = c.omega[i] * DT;
      K.cdx[i] = Math.cos(kx); K.sdx[i] = Math.sin(kx); K.cdy[i] = Math.cos(ky); K.sdy[i] = Math.sin(ky);
      K.cw[i] = Math.cos(w); K.sw[i] = Math.sin(w); K.ekH[i] = Math.exp(-c.k[i] * H); K.ekHi[i] = Math.exp(c.k[i] * H);
    }
    this._K = K;
    this._cs = new Float64Array(7 * n); this._sn = new Float64Array(7 * n);
    this._eta = new Float64Array(7); this._h = new Float64Array(9); this._ab = new Uint8Array(9);
    this._U = new Float64Array(9); this._V = new Float64Array(9); this._W = new Float64Array(9);
    this._zp = new Float64Array(9);
  };
  SeaWorld.prototype.windDerivs = function (x, y, z, t, P) {
    if (!this._K) this._prep();
    var c = this.c, n = this.n, K = this._K, cs = this._cs, sn = this._sn, eta = this._eta, h = this._h;
    var ab = this._ab, U = this._U, V = this._V, W = this._W, zp = this._zp, w = this.wind, z0 = w.z0;
    var i, j, p, low = z - H <= z0;
    for (j = 0; j < 7; j++) eta[j] = 0;
    for (i = 0; i < n; i++) {
      var k = c.k[i], a = c.a[i];
      var th = k * (c.dx[i] * x + c.dy[i] * y) - c.omega[i] * t + c.phase[i];
      var c0 = Math.cos(th), s0 = Math.sin(th);
      var cx = K.cdx[i], sx = K.sdx[i], cy = K.cdy[i], sy = K.sdy[i], cw = K.cw[i], sw = K.sw[i];
      cs[i] = c0; sn[i] = s0;
      cs[n + i] = c0 * cx - s0 * sx; sn[n + i] = s0 * cx + c0 * sx;
      cs[2 * n + i] = c0 * cx + s0 * sx; sn[2 * n + i] = s0 * cx - c0 * sx;
      cs[3 * n + i] = c0 * cy - s0 * sy; sn[3 * n + i] = s0 * cy + c0 * sy;
      cs[4 * n + i] = c0 * cy + s0 * sy; sn[4 * n + i] = s0 * cy - c0 * sy;
      cs[5 * n + i] = c0 * cw + s0 * sw; sn[5 * n + i] = s0 * cw - c0 * sw;
      cs[6 * n + i] = c0 * cw - s0 * sw; sn[6 * n + i] = s0 * cw + c0 * sw;
      for (j = 0; j < 7; j++) {
        var cj = cs[j * n + i];
        eta[j] += a * cj;
        if (this.stokes) eta[j] += 0.5 * a * a * k * (2 * cj * cj - 1);
      }
    }
    for (p = 0; p < 9; p++) {
      zp[p] = p === 5 ? z + H : p === 6 ? (low ? z : z - H) : z;
      h[p] = zp[p] - eta[CLS[p]];
      ab[p] = h[p] > z0 ? 1 : 0;
      U[p] = 0; V[p] = 0; W[p] = 0;
    }
    for (i = 0; i < n; i++) {
      var kk = c.k[i], aa = c.a[i], ddx = c.dx[i], ddy = c.dy[i], om = c.omega[i];
      var f0 = c.urel[i] * aa * kk, ao = aa * om;
      var e0 = Math.exp(-kk * h[0]);
      for (p = 0; p < 9; p++) {
        var cl = CLS[p], cp = cs[cl * n + i], sp = sn[cl * n + i];
        if (ab[p]) {
          var e = p === 0 ? e0 : p === 5 ? e0 * K.ekH[i] : p === 6 ? (low ? e0 : e0 * K.ekHi[i]) : Math.exp(-kk * h[p]);
          var f = f0 * e, up = f * cp;
          U[p] += up * ddx; V[p] += up * ddy; W[p] -= f * sp;
        } else {
          var oc = ao * cp;
          U[p] += oc * ddx; V[p] += oc * ddy; W[p] += ao * sp;
        }
      }
    }
    for (p = 0; p < 9; p++) {
      var spd = ab[p] ? (w.ustar / this.kar) * Math.log(h[p] / z0) : 0;
      P[3 * p] = spd * w.ex + U[p]; P[3 * p + 1] = spd * w.ey + V[p]; P[3 * p + 2] = W[p];
    }
  };
  // flat sea at z = 0, log wind (no wave components)
  function flatWorld(wind) { return new SeaWorld(wind, null, false); }
  // the nMax strongest components by a * k, in their original order (sim.mpc._Limited)
  function limitComponents(c, nMax) {
    var n = c.a.length;
    if (!(nMax > 0) || n <= nMax) return c;
    var idx = [], i;
    for (i = 0; i < n; i++) idx.push(i);
    idx.sort(function (p, q) { return c.a[q] * c.k[q] - c.a[p] * c.k[p]; });
    idx = idx.slice(0, nMax).sort(function (p, q) { return p - q; });
    var out = {};
    ['a', 'k', 'omega', 'phase', 'dx', 'dy', 'urel'].forEach(function (key) {
      out[key] = idx.map(function (j) { return c[key][j]; });
    });
    return out;
  }
  // spec: {kind: 'flat' | 'truth', wind, comps, stokes, maxComponents}
  function makeWorld(spec) {
    if (spec.kind === 'truth' && spec.comps) {
      return new SeaWorld(spec.wind, limitComponents(spec.comps, spec.maxComponents || 0), spec.stokes);
    }
    return flatWorld(spec.wind);
  }

  // ---------------------------------------------------------------------------- actuators
  // Mirror of sim.aircraft.Actuators. opt: rateModel 'pb2v' (default) or 'constant', tau,
  // clRate, kPb2v, delay, muRate (rad/s, constant model). Values come from LIBRARY.actuators.
  // The motor values of the record (sim.dsopt.thrust_max, prop_efficiency, folded_dcd; ctrl.thrust_sigma_frac,
  // prop.thrust_tau_s); sim/tests/test_mpc_js.py checks them against Python.
  var MOTOR = { thrustMaxPerWeight: 0.5, propEff: 0.60, motorEff: 0.75, foldedCdaM2: 3.0e-4,
    thrustTauS: 0.1, sigmaFrac: 0.15 };
  // named energy weights, in multiples of the break-even weight wE eta / m (sim.mpc.W_ENERGY_LEVELS)
  var ENERGY_K = { free: 0.03, normal: 2.0, costly: 4.0 };
  function actuatorParams(ac, opt) {
    opt = opt || {};
    var pb = (opt.rateModel || 'pb2v') === 'pb2v';
    return { rateModel: pb ? 'pb2v' : 'constant', span: ac.span,
      tau: opt.tau !== undefined ? opt.tau : (pb ? 0.05 : 0.1),
      clRate: opt.clRate !== undefined ? opt.clRate : (pb ? 1.0 : 2.0),
      kPb2v: opt.kPb2v !== undefined ? opt.kPb2v : 0.08,
      delay: opt.delay !== undefined ? opt.delay : (pb ? 0.05 : 0.0),
      muRate: opt.muRate !== undefined ? opt.muRate : 150 * Math.PI / 180,
      thrustTau: opt.thrustTau !== undefined ? opt.thrustTau : MOTOR.thrustTauS };
  }
  function muRateAt(p, V) { return p.rateModel === 'pb2v' ? p.kPb2v * 2.0 * V / p.span : p.muRate; }
  function Actuators(ac, opt) {
    this.ac = ac; this.p = actuatorParams(ac, opt);
    this.reset();
  }
  Actuators.prototype.reset = function () {
    this.cl = null; this.mu = null; this.T = null; this.cmdLast = null; this.q = null;
    this.nSteps = 0; this.nMuRate = 0; this.nClRate = 0;
  };
  // thrust (N, optional): the motor command; the actual thrust (this.T) follows it by a first-order lag
  // (no delay, no rate limit), starting at the command
  Actuators.prototype.step = function (clCmd, muCmd, dt, V, thrust) {
    var ac = this.ac, p = this.p;
    if (thrust !== undefined) {
      if (this.T === null) this.T = thrust;
      else this.T += (p.thrustTau > 0 ? 1.0 - Math.exp(-dt / p.thrustTau) : 1.0) * (thrust - this.T);
    }
    clCmd = clamp(clCmd, ac.cl_min, ac.cl_max); muCmd = clamp(muCmd, -ac.mu_max, ac.mu_max);
    this.nSteps++;
    this.cmdLast = [clCmd, muCmd];
    if (this.q === null) {
      var nd = Math.round(p.delay / dt);
      this.q = [];
      for (var i = 0; i < nd; i++) this.q.push(this.cmdLast);
    }
    this.q.push(this.cmdLast);
    var c = this.q.shift();
    clCmd = c[0]; muCmd = c[1];
    if (this.cl === null) { this.cl = clCmd; this.mu = muCmd; return [this.cl, this.mu]; }
    var a = p.tau > 0 ? 1.0 - Math.exp(-dt / p.tau) : 1.0;
    var dMu = a * (muCmd - this.mu), dCl = a * (clCmd - this.cl);
    var limMu = muRateAt(p, V) * dt, limCl = p.clRate * dt;
    if (Math.abs(dMu) > limMu) { dMu = (dMu < 0 ? -1 : 1) * limMu; this.nMuRate++; }
    if (Math.abs(dCl) > limCl) { dCl = (dCl < 0 ? -1 : 1) * limCl; this.nClRate++; }
    this.mu += dMu; this.cl += dCl;
    return [this.cl, this.mu];
  };
  // load-factor limit of sim.aircraft.n_limited_cl (positive side only)
  function nLimitedCl(ac, V, cl) {
    var q = 0.5 * ac.rho * V * V * ac.wing_area;
    if (!(q > 0)) return cl;
    return Math.min(cl, ac.n_max * ac.mass * G / q);
  }

  // ---------------------------------------------------------------------------- cycle library
  // One periodic cycle in the wind frame (wind toward +x), optionally mirrored (sim.mpc.Cycle).
  function Cycle(d, mirror) {
    var sg = mirror ? -1 : 1, i;
    this.mirror = !!mirror;
    this.d = d;
    this.T = d.period;
    this.t = Float64Array.from(d.t);
    this.z = Float64Array.from(d.z); this.V = Float64Array.from(d.V); this.g = Float64Array.from(d.gamma);
    this.psi = unwrap(Float64Array.from(d.psi, function (v) { return sg * v; }));
    this.cl = Float64Array.from(d.cl); this.mu = Float64Array.from(d.mu, function (v) { return sg * v; });
    // node thrust (N) of a motor-assisted cycle; zeros for a soaring cycle or an old library entry
    this.thrust = d.thrust ? Float64Array.from(d.thrust) : new Float64Array(this.t.length);
    this.hasThrust = false;
    for (i = 0; i < this.thrust.length; i++) if (this.thrust[i] > 0) this.hasThrust = true;
    var s = 0;
    for (i = 0; i < this.V.length; i++) s += this.V[i];
    this.vMean = s / this.V.length;
  }
  Cycle.prototype._p = function (tau, arr) {
    var T = this.T;
    return interp(tau - T * Math.floor(tau / T), this.t, arr);
  };
  // [h, V, gamma, psi relative to the wind] at phase tau
  Cycle.prototype.state = function (tau) {
    return [this._p(tau, this.z), this._p(tau, this.V), this._p(tau, this.g), wrap(this._p(tau, this.psi))];
  };
  Cycle.prototype.controls = function (tau) { return [this._p(tau, this.cl), this._p(tau, this.mu)]; };
  Cycle.prototype.thrustAt = function (tau) { return this._p(tau, this.thrust); };
  Cycle.prototype.energy = function (tau) {
    var h = this._p(tau, this.z), V = this._p(tau, this.V);
    return G * h + 0.5 * V * V;
  };
  // cycle of the aircraft nearest to u10Est and |chi| (mirrored for negative chi);
  // entries: array of {u10, chi_deg, ...} for one aircraft
  function pickCycle(entries, u10Est, chiRad) {
    if (!entries || !entries.length) throw new Error('no cycles');
    var chiTol = 20, i, j, winds = [];
    for (i = 0; i < entries.length; i++) if (winds.indexOf(entries[i].u10) < 0) winds.push(entries[i].u10);
    winds.sort(function (p, q) { return Math.abs(p - u10Est) - Math.abs(q - u10Est); });   // stable: ties keep the library order
    var chi = Math.abs(chiRad * 180 / Math.PI), gap = {}, u = null;
    winds.forEach(function (w) {
      var g = Infinity;
      for (j = 0; j < entries.length; j++) if (entries[j].u10 === w) g = Math.min(g, Math.abs(entries[j].chi_deg - chi));
      gap[w] = g;
    });
    for (i = 0; i < winds.length; i++) if (gap[winds[i]] <= chiTol) { u = winds[i]; break; }
    if (u === null) {   // none within the tolerance: the smallest gap, ties to the nearer wind
      u = winds[0];
      for (i = 1; i < winds.length; i++) {
        if (gap[winds[i]] < gap[u] || (gap[winds[i]] === gap[u] && Math.abs(winds[i] - u10Est) < Math.abs(u - u10Est))) u = winds[i];
      }
    }
    var best = null;
    for (i = 0; i < entries.length; i++) {
      if (entries[i].u10 !== u) continue;
      if (best === null || Math.abs(entries[i].chi_deg - chi) < Math.abs(best.chi_deg - chi)) best = entries[i];
    }
    return new Cycle(best, chiRad < 0);
  }

  // Library cycle matched to the current state (sim.mpc.CycleGuide without the wave shift).
  var GUIDE_SIG = [3.0, null, 0.25, 0.6];
  var SWITCH_MATCH_MAX = 2.0, SWITCH_MAX_WAIT_S = 15.0;
  var STEADY_SIG_H = 0.5, STEADY_SIG_PSI = 0.04, STEADY_W_D = 200.0;   // match to a steady reference: tight height, heavier terminal weight
  function Guide(cycle, world, t, s, psiW, horizon, wD, tauHint, hintWindow, sig) {
    this.cycle = cycle; this.psiW = psiW; this.H = horizon; this.wD = wD;
    var sg = sig || GUIDE_SIG;   // sig: [h, V or null, gamma, heading] tolerances of the match
    this.sig = [sg[0], sg[1] ? sg[1] : 0.25 * cycle.vMean, sg[2], sg[3]];
    var x = s[0], y = s[1], z = s[2], V = s[3], g = s[4], psi = s[5];
    this.h0 = z - world.eta(x, y, t);
    var f0 = this.h0, f1 = V, f2 = g, f3 = wrap(psi - psiW);
    var T = cycle.T, n = 80, step = T / n, best = Infinity, bi = 0, hw = hintWindow === undefined ? 0.2 : hintWindow;
    for (var i = 0; i < n; i++) {
      var tau = i * step;
      if (tauHint !== null && tauHint !== undefined &&
          Math.abs(wrap((tau - tauHint) * TWO_PI / T)) > hw * TWO_PI) continue;
      var st = cycle.state(tau);
      var d = Math.pow((st[0] - f0) / this.sig[0], 2) + Math.pow((st[1] - f1) / this.sig[1], 2) +
        Math.pow((st[2] - f2) / this.sig[2], 2) + Math.pow(wrap(st[3] - f3) / this.sig[3], 2);
      if (d < best) { best = d; bi = i; }
    }
    this.tau0 = bi * step;
    this.matchDist = best;
    this.delta = 0;
    this.tauS = this.tau0;
    var te = this.tauS + horizon;
    this.target = cycle.state(te);
    this.dEc = cycle.energy(te) - cycle.energy(this.tauS);
    this.E0 = G * this.h0 + 0.5 * V * V;
    if (cycle.d.steady) {
      this.sig[0] = Math.min(this.sig[0], STEADY_SIG_H); this.sig[3] = Math.min(this.sig[3], STEADY_SIG_PSI); this.wD = Math.max(this.wD, STEADY_W_D);   // see sim.mpc.CycleGuide
      this.dEc = cycle.energy(te) - this.E0;
    }   // a steady reference holds an energy level (see sim.mpc.CycleGuide)
  }
  // nominal controls at the knot times: Float64Array [cl knots..., mu knots...]
  Guide.prototype.nominal = function (knotsT) {
    var n = knotsT.length, out = new Float64Array(2 * n);
    for (var j = 0; j < n; j++) {
      var c = this.cycle.controls(this.tauS + knotsT[j]);
      out[j] = c[0]; out[n + j] = c[1];
    }
    return out;
  };
  // thrust (N) of the cycle at the knot times, phase-matched like the controls (zeros for a soaring cycle)
  Guide.prototype.nominalThrust = function (knotsT) {
    var n = knotsT.length, out = new Float64Array(n);
    for (var j = 0; j < n; j++) out[j] = this.cycle.thrustAt(this.tauS + knotsT[j]);
    return out;
  };
  Guide.prototype.terminalCost = function (world, tEnd, s, planner) {
    var h = s[2] - world.eta(s[0], s[1], tEnd), tg = this.target, sg = this.sig;
    var d2 = Math.pow((h - tg[0]) / sg[0], 2) + Math.pow((s[3] - tg[1]) / sg[1], 2) +
      Math.pow((s[4] - tg[2]) / sg[2], 2) + Math.pow(wrap(s[5] - this.psiW - tg[3]) / sg[3], 2);
    var deficit = Math.max(0, this.dEc - (G * h + 0.5 * s[3] * s[3] - this.E0));
    return this.wD * d2 + planner.wE * deficit;
  };

  // ---------------------------------------------------------------------------- planner
  var DEFAULTS = { horizonS: 6.0, dt: 0.1, nSamples: 96, nCtrl: 24, lam: 0.2, sigmaCl: 0.08,
    sigmaMu: 0.12, replanS: 0.5, nIter: 1, wE: 1.0, hClear: 0.3, clearance: 'wingtip',
    wClear: 200.0, wN: 50.0, wStall: 300.0, cContact: 1e4, wD: 10.0, nLimit: true, seed: 1,
    thrustSampling: 'basic', foldSwitch: 'smooth' };
  // thrustSampling 'sustained' and foldSwitch 'state': see sim.mpc (P_HOLD, HOLD_SIGMA_FRAC,
  // THRUST_CAND_LEVELS, FOLD_CMD_FRAC)
  var SUSTAIN = { pHold: 0.5, holdSigmaFrac: 0.3, candLevels: [0.05, 0.10, 0.20], foldCmdFrac: 0.005 };
  // folded-propeller drag to hand to the smooth-switch formula so that it gives the state switch
  function foldEffective(ac, dcd, tAct, tCmd) {
    var f = 1.0 - tAct / (tAct + 0.01 * ac.mass * G);
    return tCmd < SUSTAIN.foldCmdFrac * ac.mass * G ? dcd / f : 0.0;
  }

  function Planner(ac, opt) {
    opt = opt || {};
    var k;
    for (k in DEFAULTS) this[k] = opt[k] !== undefined ? opt[k] : DEFAULTS[k];
    this.ac = ac;
    this.motor = !!opt.motor;
    this.hCeil = opt.hCeil !== undefined && opt.hCeil !== null ? opt.hCeil : null;   // soft ceiling (m), null: off
    this.wCeil = opt.wCeil !== undefined ? opt.wCeil : 200.0;
    this.thrustUpdate = opt.thrustUpdate || 'clipped';   // 'raw': the unbiased thrust update (sim.mpc MPPIPlanner)
    if (this.thrustUpdate !== 'clipped' && this.thrustUpdate !== 'raw') throw new Error('thrustUpdate');
    if ((this.thrustSampling !== 'basic' && this.thrustSampling !== 'sustained') ||
        (this.foldSwitch !== 'smooth' && this.foldSwitch !== 'state')) throw new Error('thrustSampling / foldSwitch');
    this.nu = this.motor ? 3 : 2;      // controls per knot: cl, mu (and the thrust T, N)
    if (this.motor) {
      var mv = Object.assign({}, MOTOR, opt.motorValues || {});
      this.tMax = mv.thrustMaxPerWeight * ac.mass * G;
      this.eta = mv.propEff * mv.motorEff;
      this.foldDcd = mv.foldedCdaM2 / ac.wing_area;
      this.sigmaT = mv.sigmaFrac * this.tMax;
      this.sigmaHold = SUSTAIN.holdSigmaFrac * this.tMax;
      this.wEnergy = opt.wEnergy !== undefined ? opt.wEnergy
        : ENERGY_K[opt.energyLevel || 'normal'] * this.wE * this.eta / ac.mass;
    }
    this.act = null;        // actuator parameters (actuatorParams) or null: rollouts fly the commands
    this.actState = null;   // [actual cl, actual mu, last cl command, last mu command] or null
    this.epsHook = null;    // test hook: function (iteration) -> Float64Array (nSamples, 2, nCtrl)
    this.U = null;          // Float64Array (2, nCtrl): nominal plan
    this.rng = new Gauss(this.seed);
    this.nsteps = Math.round(this.horizonS / this.dt);
    var nc = this.nCtrl, nh = 2 * this.nsteps + 1;
    this.knotsT = linspace(0.0, this.horizonS, nc);
    this.idx = new Int32Array(nh); this.frac = new Float64Array(nh);
    for (var j = 0; j < nh; j++) {
      var tau = j * this.dt / 2.0, c = 0;
      while (c < nc && this.knotsT[c] <= tau) c++;   // searchsorted(side='right')
      var i0 = Math.min(Math.max(c - 1, 0), nc - 2);
      this.idx[j] = i0;
      this.frac[j] = (tau - this.knotsT[i0]) / (this.knotsT[i0 + 1] - this.knotsT[i0]);
    }
    this._sc = { s: new Float64Array(6), tmp: new Float64Array(6), k1: new Float64Array(6),
      k2: new Float64Array(6), k3: new Float64Array(6), k4: new Float64Array(6),
      P: new Float64Array(27), o: new Float64Array(3), clAll: new Float64Array(nh),
      muAll: new Float64Array(nh), tAll: new Float64Array(nh) };
  }

  // derivatives of the point mass (sim.mpc.derivatives_vec for one state); out: Float64Array(6)
  // thrust (N, optional): with the motor, adds thrust / m and the folded-propeller drag switch
  Planner.prototype._deriv = function (world, t, s, cl, mu, out, thrust, fold) {
    var ac = this.ac, P = this._sc.P, o = this._sc.o;
    var x = s[0], y = s[1], z = s[2], V = s[3], g = s[4], p = s[5];
    if (this.nLimit) cl = Math.min(cl, ac.n_max * ac.mass * G / (0.5 * ac.rho * V * V * ac.wing_area));
    var cg = Math.cos(g), sg = Math.sin(g), cp = Math.cos(p), sp = Math.sin(p);
    var low = z - H <= world.z0, zlo = low ? z : z - H;
    if (world.windDerivs) world.windDerivs(x, y, z, t, P);
    else {
      world.windAt(x, y, z, t, o); P[0] = o[0]; P[1] = o[1]; P[2] = o[2];
      world.windAt(x + H, y, z, t, o); P[3] = o[0]; P[4] = o[1]; P[5] = o[2];
      world.windAt(x - H, y, z, t, o); P[6] = o[0]; P[7] = o[1]; P[8] = o[2];
      world.windAt(x, y + H, z, t, o); P[9] = o[0]; P[10] = o[1]; P[11] = o[2];
      world.windAt(x, y - H, z, t, o); P[12] = o[0]; P[13] = o[1]; P[14] = o[2];
      world.windAt(x, y, z + H, t, o); P[15] = o[0]; P[16] = o[1]; P[17] = o[2];
      world.windAt(x, y, zlo, t, o); P[18] = o[0]; P[19] = o[1]; P[20] = o[2];
      world.windAt(x, y, z, t + DT, o); P[21] = o[0]; P[22] = o[1]; P[23] = o[2];
      world.windAt(x, y, z, t - DT, o); P[24] = o[0]; P[25] = o[1]; P[26] = o[2];
    }
    var vg0 = V * cg * cp + P[0], vg1 = V * cg * sp + P[1], vg2 = V * sg + P[2];
    var dz = low ? H : 2 * H;
    var a0 = (P[21] - P[24]) / (2 * DT) + (P[3] - P[6]) / (2 * H) * vg0 + (P[9] - P[12]) / (2 * H) * vg1 + (P[15] - P[18]) / dz * vg2;
    var a1 = (P[22] - P[25]) / (2 * DT) + (P[4] - P[7]) / (2 * H) * vg0 + (P[10] - P[13]) / (2 * H) * vg1 + (P[16] - P[19]) / dz * vg2;
    var a2 = (P[23] - P[26]) / (2 * DT) + (P[5] - P[8]) / (2 * H) * vg0 + (P[11] - P[14]) / (2 * H) * vg1 + (P[17] - P[20]) / dz * vg2;
    var q = 0.5 * ac.rho * V * V * ac.wing_area, L = q * cl, m = ac.mass;
    var cd0 = ac.cd0;
    if (thrust !== undefined) cd0 = cd0 + (fold !== undefined ? fold : this.foldDcd) * (1.0 - thrust / (thrust + 0.01 * ac.mass * G));
    var D = q * (cd0 + ac.k_induced * cl * cl);
    var aV = a0 * cg * cp + a1 * cg * sp + a2 * sg;
    var aG = -a0 * sg * cp - a1 * sg * sp + a2 * cg;
    var aP = -a0 * sp + a1 * cp;
    var cgf = Math.max(Math.abs(cg), 0.05);
    out[0] = vg0; out[1] = vg1; out[2] = vg2;
    out[3] = -D / m - G * sg - aV;
    if (thrust !== undefined) out[3] = out[3] + thrust / m;
    out[4] = (L * Math.cos(mu) / m - G * cg - aG) / V;
    out[5] = (L * Math.sin(mu) / m - aP) / (V * cgf);
  };

  // Cost of each control plan in Us (Float64Array (S, 2, nCtrl)) from state s0 at time t0 (the
  // body of sim.mpc.MPPIPlanner.rollout with a guide). Returns {cost, dead, paths}; paths (only
  // with wantPath) is an array of Float64Array((N + 1) * 6), one per plan.
  Planner.prototype.rollout = function (world, t0, s0, Us, S, guide, wantPath) {
    var ac = this.ac, dt = this.dt, N = this.nsteps, nc = this.nCtrl, nh = 2 * N + 1, hd = dt / 2.0;
    var act = this.act, sc = this._sc, clAll = sc.clAll, muAll = sc.muAll, tAll = sc.tAll;
    var motor = this.motor, nu = this.nu, aTLag = 0;
    var s = sc.s, tmp = sc.tmp, k1 = sc.k1, k2 = sc.k2, k3 = sc.k3, k4 = sc.k4;
    var cost = new Float64Array(S), dead = new Uint8Array(S), paths = wantPath ? [] : null;
    var vmin = 1.1 * ac.v_stall, aLag = 0, m = 0, j, i, d;
    if (act) {
      m = Math.round(act.delay / hd);
      aLag = act.tau > 0 ? 1.0 - Math.exp(-hd / act.tau) : 1.0;
      if (motor) aTLag = act.thrustTau > 0 ? 1.0 - Math.exp(-hd / act.thrustTau) : 1.0;
    }
    for (var si = 0; si < S; si++) {
      var oc = (si * nu) * nc, om = (si * nu + 1) * nc, ot = (si * nu + 2) * nc;
      for (j = 0; j < nh; j++) {
        var i0 = this.idx[j], fr = this.frac[j];
        clAll[j] = clamp(Us[oc + i0] * (1 - fr) + Us[oc + i0 + 1] * fr, ac.cl_min, ac.cl_max);
        muAll[j] = clamp(Us[om + i0] * (1 - fr) + Us[om + i0 + 1] * fr, -ac.mu_max, ac.mu_max);
        if (motor) tAll[j] = clamp(Us[ot + i0] * (1 - fr) + Us[ot + i0 + 1] * fr, 0.0, this.tMax);
      }
      var xcl = 0, xmu = 0, xT = 0;
      if (act) {
        var st = this.actState || [clAll[0], muAll[0], clAll[0], muAll[0]];
        if (m > 0) {
          for (j = nh - 1; j >= m; j--) { clAll[j] = clAll[j - m]; muAll[j] = muAll[j - m]; }
          for (j = 0; j < Math.min(m, nh); j++) { clAll[j] = st[2]; muAll[j] = st[3]; }
        }
        xcl = st[0]; xmu = st[1];
        if (motor) xT = st.length > 4 ? st[4] : 0.0;
      }
      for (j = 0; j < 6; j++) s[j] = s0[j];
      var path = wantPath ? new Float64Array((N + 1) * 6) : null;
      if (path) path.set(s, 0);
      var c = 0, isDead = false, tDead = this.horizonS;
      for (i = 0; i < N; i++) {
        var t = t0 + i * dt;
        var c1 = clAll[2 * i], c2 = clAll[2 * i + 1], c3 = clAll[2 * i + 2];
        var m1 = muAll[2 * i], m2 = muAll[2 * i + 1], m3 = muAll[2 * i + 2];
        var c3cmd = c3, t1 = 0, t2 = 0, t3 = 0, v0 = s[3], f1, f2, f3, tc1 = 0, tc2 = 0, tc3 = 0;
        if (motor) {
          t1 = tAll[2 * i]; t2 = tAll[2 * i + 1]; t3 = tAll[2 * i + 2];
          tc1 = t1; tc2 = t2; tc3 = t3;   // the commands (before the motor lag)
          if (act) {   // the motor lags its command (no delay)
            t1 = xT;
            xT = xT + aTLag * (t2 - xT); t2 = xT;
            xT = xT + aTLag * (t3 - xT); t3 = xT;
          }
        }
        if (act) {
          var limMu = muRateAt(act, s[3]) * hd, limCl = act.clRate * hd;
          c1 = xcl; m1 = xmu;
          xcl = xcl + clamp(aLag * (c2 - xcl), -limCl, limCl);
          xmu = xmu + clamp(aLag * (m2 - xmu), -limMu, limMu);
          c2 = xcl; m2 = xmu;
          xcl = xcl + clamp(aLag * (c3 - xcl), -limCl, limCl);
          xmu = xmu + clamp(aLag * (m3 - xmu), -limMu, limMu);
          c3 = xcl; m3 = xmu;
        }
        if (motor && this.foldSwitch === 'state') {
          f1 = foldEffective(ac, this.foldDcd, t1, tc1); f2 = foldEffective(ac, this.foldDcd, t2, tc2);
          f3 = foldEffective(ac, this.foldDcd, t3, tc3);
        }
        this._deriv(world, t, s, c1, m1, k1, motor ? t1 : undefined, f1);
        for (d = 0; d < 6; d++) tmp[d] = s[d] + dt / 2 * k1[d];
        this._deriv(world, t + dt / 2, tmp, c2, m2, k2, motor ? t2 : undefined, f2);
        for (d = 0; d < 6; d++) tmp[d] = s[d] + dt / 2 * k2[d];
        this._deriv(world, t + dt / 2, tmp, c2, m2, k3, motor ? t2 : undefined, f2);
        for (d = 0; d < 6; d++) tmp[d] = s[d] + dt * k3[d];
        this._deriv(world, t + dt, tmp, c3, m3, k4, motor ? t3 : undefined, f3);
        if (isDead) {   // frozen once dead
          for (d = 0; d < 6; d++) tmp[d] = s[d];
        } else {
          for (d = 0; d < 6; d++) tmp[d] = s[d] + dt / 6 * (k1[d] + 2 * k2[d] + 2 * k3[d] + k4[d]);
        }
        var bad = !(isFinite(tmp[0]) && isFinite(tmp[1]) && isFinite(tmp[2]) && isFinite(tmp[3]) &&
          isFinite(tmp[4]) && isFinite(tmp[5])) || tmp[3] < 0.5 * ac.v_stall;
        if (bad) { for (d = 0; d < 6; d++) tmp[d] = s[d]; }
        for (d = 0; d < 6; d++) s[d] = tmp[d];
        var hc = s[2] - world.eta(s[0], s[1], t + dt);
        if (this.clearance === 'wingtip') hc = hc - 0.5 * ac.span * Math.sin(Math.abs(m3)) * Math.cos(s[4]);
        var live = !isDead && !bad;
        var V = s[3], q = 0.5 * ac.rho * V * V * ac.wing_area, n = q * c3cmd / (ac.mass * G);
        var pen = this.wClear * Math.pow(Math.max(0.0, this.hClear - hc) / this.hClear, 2) +
          this.wN * Math.pow(Math.max(0.0, n - ac.n_max), 2) +
          this.wStall * Math.pow(Math.max(0.0, vmin - V) / ac.v_stall, 2) +
          (this.hCeil !== null ? this.wCeil * Math.pow(Math.max(0.0, hc - this.hCeil), 2) : 0.0);
        if (live) c += dt * pen;
        if (motor && live) c += dt * this.wEnergy * t2 * v0 / this.eta;   // electric energy of the step
        var hit = live && hc < 0.0;
        var newly = hit || (bad && !isDead);
        if (newly) { tDead = (i + 1) * dt; isDead = true; }
        if (path) path.set(s, (i + 1) * 6);
      }
      c += guide.terminalCost(world, t0 + N * dt, s, this);
      if (isDead) c += this.cContact * (1.0 + (this.horizonS - tDead) / this.horizonS);
      cost[si] = c; dead[si] = isDead ? 1 : 0;
      if (wantPath) paths.push(path);
    }
    return { cost: cost, dead: dead, paths: paths };
  };

  // The weighted MPPI update (MPPIPlanner.update). U: (2, nCtrl); Us: (S, 2, nCtrl).
  // Returns {U, ess, nDead}.
  Planner.prototype.update = function (U, Us, S, cost, dead, clipThrust) {
    var nc = this.nCtrl, nu = this.nu, i, j;
    var cmin = Infinity, nOk = 0, sum = 0, sum2 = 0, sumAll = 0, sum2All = 0;
    for (i = 0; i < S; i++) {
      cmin = Math.min(cmin, cost[i]);
      sumAll += cost[i];
      if (!dead[i]) { nOk++; sum += cost[i]; }
    }
    var scale;
    if (nOk >= 2) {
      var mean = sum / nOk;
      for (i = 0; i < S; i++) if (!dead[i]) sum2 += (cost[i] - mean) * (cost[i] - mean);
      scale = Math.sqrt(sum2 / nOk);
    } else {
      var meanA = sumAll / S;
      for (i = 0; i < S; i++) sum2All += (cost[i] - meanA) * (cost[i] - meanA);
      scale = Math.sqrt(sum2All / S);
    }
    scale = Math.max(scale, 1e-6);
    var w = new Float64Array(S), wsum = 0, nDead = 0;
    for (i = 0; i < S; i++) { w[i] = Math.exp(-(cost[i] - cmin) / (this.lam * scale)); wsum += w[i]; nDead += dead[i]; }
    var w2 = 0;
    for (i = 0; i < S; i++) { w[i] /= wsum; w2 += w[i] * w[i]; }
    var out = new Float64Array(nu * nc);
    for (j = 0; j < nu * nc; j++) {
      var acc = 0;
      for (i = 0; i < S; i++) acc += w[i] * (Us[i * nu * nc + j] - U[j]);
      out[j] = U[j] + acc;
    }
    this._clip(out, clipThrust === false);
    return { U: out, ess: 1.0 / w2, nDead: nDead };
  };

  Planner.prototype._clip = function (U, keepThrust) {   // U: (nu, nCtrl); keepThrust: leave the thrust row unclipped
    var ac = this.ac, nc = this.nCtrl;
    for (var j = 0; j < nc; j++) {
      U[j] = clamp(U[j], ac.cl_min, ac.cl_max);
      U[nc + j] = clamp(U[nc + j], -ac.mu_max, ac.mu_max);
      if (this.motor && !keepThrust) U[2 * nc + j] = clamp(U[2 * nc + j], 0.0, this.tMax);
    }
    return U;
  };
  Planner.prototype._clipAt = function (Us, o) {
    var ac = this.ac, nc = this.nCtrl;
    for (var j = 0; j < nc; j++) {
      Us[o + j] = clamp(Us[o + j], ac.cl_min, ac.cl_max);
      Us[o + nc + j] = clamp(Us[o + nc + j], -ac.mu_max, ac.mu_max);
      if (this.motor) Us[o + 2 * nc + j] = clamp(Us[o + 2 * nc + j], 0.0, this.tMax);
    }
  };
  Planner.prototype.reset = function () { this.U = null; };
  Planner.prototype.shift = function (U) {
    var nc = this.nCtrl, out = new Float64Array(this.nu * nc), kt = this.knotsT;
    var cl = U.subarray(0, nc), mu = U.subarray(nc, 2 * nc);
    for (var j = 0; j < nc; j++) {
      var tau = Math.min(kt[j] + this.replanS, this.horizonS);
      out[j] = interp(tau, kt, cl); out[nc + j] = interp(tau, kt, mu);
      if (this.motor) out[2 * nc + j] = interp(tau, kt, U.subarray(2 * nc, 3 * nc));
    }
    return out;
  };
  // constant-control plans always in the sample set (sim.mpc.MPPIPlanner._candidates)
  Planner.prototype.candidates = function () {
    var ac = this.ac, nc = this.nCtrl, nu = this.nu, out = [];
    [ac.cl_best_ld, 0.8 * ac.cl_max].forEach(function (cl) {
      [0.0, -0.6, 0.6].forEach(function (mu) {
        var u = new Float64Array(nu * nc);   // the thrust row stays 0
        u.fill(cl, 0, nc); u.fill(mu, nc, 2 * nc);
        out.push(u);
      });
    });
    return out;
  };

  // constant-thrust candidates (thrust_sampling 'sustained'): each level of SUSTAIN.candLevels of the
  // weight over the whole horizon, with the controls of the nominal U and wings level at the best L/D
  Planner.prototype.thrustCandidates = function (U) {
    var ac = this.ac, nc = this.nCtrl, out = [];
    SUSTAIN.candLevels.forEach(function (lv) {
      var th = Math.min(lv * ac.mass * G, this.tMax), a = new Float64Array(3 * nc), b = new Float64Array(3 * nc);
      a.set(U.subarray(0, 2 * nc)); a.fill(th, 2 * nc, 3 * nc);
      b.fill(ac.cl_best_ld, 0, nc); b.fill(th, 2 * nc, 3 * nc);
      out.push(a); out.push(b);
    }, this);
    return out;
  };
  // thrust perturbations of one sample into eps[o .. o + nc): independent Gaussian per knot; with
  // 'sustained' a share pHold of the samples hold one half-normal level over a random run of knots
  Planner.prototype.thrustNoise = function (eps, o) {
    var nc = this.nCtrl, j;
    if (this.thrustSampling === 'sustained' && this.rng.uniform() < SUSTAIN.pHold) {
      var lvl = this.rng.next() * this.sigmaHold;
      if (this.thrustUpdate !== 'raw') lvl = Math.abs(lvl);   // one-sided; with 'raw' symmetric about the nominal
      var a = Math.floor(this.rng.uniform() * nc), b = Math.floor(this.rng.uniform() * nc);
      var lo = Math.min(a, b), hi = Math.max(a, b);
      for (j = 0; j < nc; j++) eps[o + j] = (j >= lo && j <= hi) ? lvl : 0.0;
    } else {
      for (j = 0; j < nc; j++) eps[o + j] = this.rng.next() * this.sigmaT;
    }
  };

  // One planning call from state s at time t in `world`, centred on the guide's cycle
  // (MPPIPlanner.plan with a guide). Returns a plan.
  Planner.prototype.plan = function (world, t, s, guide) {
    var nc = this.nCtrl, nu = this.nu, S = this.nSamples, i, j, it;
    var prev = this.U === null ? null : this.shift(this.U);
    var U = guide.nominal(this.knotsT);
    if (this.motor) {   // the thrust schedule of the cycle (zero for a soaring cycle)
      var U3 = new Float64Array(3 * nc), nt = guide.nominalThrust(this.knotsT);
      U3.set(U);
      for (j = 0; j < nc; j++) U3[2 * nc + j] = clamp(nt[j], 0.0, this.tMax);
      U = U3;
    }
    var cand = this.candidates();
    if (this.motor && this.thrustSampling === 'sustained') cand = cand.concat(this.thrustCandidates(U));
    var nCand = cand.length;
    var Us = new Float64Array(S * nu * nc), ess = 1, nDead = 0;
    for (it = 0; it < this.nIter; it++) {
      var eps;
      if (this.epsHook) eps = this.epsHook(it);
      else {
        eps = new Float64Array(S * nu * nc);
        for (i = 0; i < S; i++) {
          for (j = 0; j < nc; j++) eps[i * nu * nc + j] = this.rng.next() * this.sigmaCl;
          for (j = 0; j < nc; j++) eps[i * nu * nc + nc + j] = this.rng.next() * this.sigmaMu;
          if (this.motor) this.thrustNoise(eps, i * nu * nc + 2 * nc);
        }
      }
      for (i = 0; i < S; i++) {
        for (j = 0; j < nu * nc; j++) Us[i * nu * nc + j] = U[j] + (i === 0 ? 0 : eps[i * nu * nc + j]);
        this._clipAt(Us, i * nu * nc);   // the clip keeps U_T + eps in [0, tMax]
      }
      for (i = 0; i < nCand; i++) Us.set(cand[i], (1 + i) * nu * nc);
      if (prev !== null) {
        var pv = Float64Array.from(prev);
        this._clip(pv);
        Us.set(pv, (1 + nCand) * nu * nc);
      }
      var r = this.rollout(world, t, s, Us, S, guide, false);
      var UsUpd = Us;
      if (this.motor && this.thrustUpdate === 'raw') {
        // the thrust row of the update uses the perturbed values before the clip at zero (the clipped
        // ones are all >= 0, so their weighted mean exceeds the nominal even when nothing asks for thrust)
        UsUpd = Float64Array.from(Us);
        for (i = 0; i < S; i++) {
          if (i >= 1 && i <= nCand) continue;
          if (prev !== null && i === 1 + nCand) continue;
          for (j = 0; j < nc; j++) UsUpd[i * nu * nc + 2 * nc + j] = U[2 * nc + j] + (i === 0 ? 0 : eps[i * nu * nc + 2 * nc + j]);
        }
      }
      var u = this.update(U, UsUpd, S, r.cost, r.dead, !(this.motor && this.thrustUpdate === 'raw'));
      U = u.U; ess = u.ess; nDead = u.nDead;
    }
    this.U = U;
    var fin = this.rollout(world, t, s, U, 1, guide, true);
    var times = new Float64Array(this.nsteps + 1);
    for (i = 0; i <= this.nsteps; i++) times[i] = i * this.dt;
    var tMaxApplied = this.tMax;   // the applied thrust command is clipped (the nominal U may be unclipped)
    var plan = { knotsT: this.knotsT, cl: Float64Array.from(U.subarray(0, nc)),
      mu: Float64Array.from(U.subarray(nc, 2 * nc)), path: fin.paths[0], times: times,
      cost: fin.cost[0], ess: ess, nDead: nDead,
      th: this.motor ? Float64Array.from(U.subarray(2 * nc, 3 * nc), function (v) { return clamp(v, 0.0, tMaxApplied); }) : null };
    plan.controls = function (tau) { return planControls(plan, tau); };
    return plan;
  };
  // (cl, mu) at tau seconds from the plan time (piecewise linear, held after the end)
  function planControls(plan, tau) {
    return [interp(tau, plan.knotsT, plan.cl), interp(tau, plan.knotsT, plan.mu)];
  }
  // commanded thrust (N) at tau seconds from the plan time (0 without the motor)
  function planThrust(plan, tau) { return plan.th ? interp(tau, plan.knotsT, plan.th) : 0.0; }

  // ---------------------------------------------------------------------------- reflex
  // Mirror of sim.mpc.Reflex: predicts REFLEX.horizonS of flight with the plan's own controls and
  // reports the first time the centre or a wingtip is predicted below REFLEX.clearanceM above the
  // surface of the world, or the airspeed below REFLEX.vFrac stall speeds. Runs on the main thread
  // at every control step (the viewer builds the world with makeWorld from the request it sends).
  // The values are those of the record (ctrl.reflex_*); test_mpc_js checks them against Python.
  var REFLEX = { horizonS: 1.5, clearanceM: 0.3, vFrac: 1.05, dt: 0.1, nFrac: 0.8, vFactor: 1.2 };
  function Reflex(ac, opt) {
    opt = opt || {};
    var mv = Object.assign({}, MOTOR, opt.motorValues || {}), k;
    this.ac = ac;
    for (k in REFLEX) this[k] = opt[k] !== undefined ? opt[k] : REFLEX[k];
    this.tMax = mv.thrustMaxPerWeight * ac.mass * G;
    this.foldSwitch = opt.foldSwitch || 'smooth';
    this.n = Math.round(this.horizonS / this.dt);
    this.pl = new Planner(ac, { motor: true, nLimit: opt.nLimit !== false, motorValues: opt.motorValues });
    this._o = new Float64Array(6);
  }
  // plan: {knotsT, cl, mu, th (or null)}; tau: seconds after the plan time. Returns null when the
  // prediction clears, else {cause: 'centre' | 'tip' | 'stall', ahead, h, tip, V}.
  Reflex.prototype.predict = function (world, t, s, plan, tau) {
    var ac = this.ac, dt = this.dt, n = this.n, pl = this.pl, kt = plan.knotsT, i, d;
    var nh = 2 * n + 1, cl = new Float64Array(nh), mu = new Float64Array(nh), T = new Float64Array(nh);
    for (i = 0; i < nh; i++) {
      var ta = tau + i * dt / 2.0;
      cl[i] = clamp(interp(ta, kt, plan.cl), ac.cl_min, ac.cl_max);
      mu[i] = clamp(interp(ta, kt, plan.mu), -ac.mu_max, ac.mu_max);
      T[i] = plan.th ? clamp(interp(ta, kt, plan.th), 0.0, this.tMax) : 0.0;
    }
    var st = [s[0], s[1], s[2], s[3], s[4], s[5]], tmp = [0, 0, 0, 0, 0, 0], k1 = this._o, km = [0, 0, 0, 0, 0, 0];
    var vLim = this.vFrac * ac.v_stall;
    for (i = 0; i <= n; i++) {
      var tt = t + i * dt;
      var h = st[2] - world.eta(st[0], st[1], tt);
      var tip = h - 0.5 * ac.span * Math.sin(Math.abs(mu[2 * i])) * Math.cos(st[4]);
      var cause = h < this.clearanceM ? 'centre' : tip < this.clearanceM ? 'tip' : st[3] < vLim ? 'stall' : null;
      if (cause !== null) return { cause: cause, ahead: i * dt, h: h, tip: tip, V: st[3] };
      if (i === n) break;
      var fa = this.foldSwitch === 'state' ? foldEffective(ac, pl.foldDcd, T[2 * i], T[2 * i]) : undefined;
      var fb = this.foldSwitch === 'state' ? foldEffective(ac, pl.foldDcd, T[2 * i + 1], T[2 * i + 1]) : undefined;
      pl._deriv(world, tt, st, cl[2 * i], mu[2 * i], k1, T[2 * i], fa);
      for (d = 0; d < 6; d++) tmp[d] = st[d] + dt / 2 * k1[d];
      pl._deriv(world, tt + dt / 2, tmp, cl[2 * i + 1], mu[2 * i + 1], km, T[2 * i + 1], fb);
      var fin = true;
      for (d = 0; d < 6; d++) { st[d] = st[d] + dt * km[d]; if (!isFinite(st[d])) fin = false; }
      if (!fin) return { cause: 'stall', ahead: (i + 1) * dt, h: NaN, tip: NaN, V: NaN };
    }
    return null;
  };
  // [cl, mu, thrust] of the override at state s: full thrust, cl at its margin limit, wings level
  Reflex.prototype.override = function (s) {
    var ac = this.ac, q = 0.5 * ac.rho * s[3] * s[3] * ac.wing_area;
    var cl = Math.min(ac.cl_max / (this.vFactor * this.vFactor), this.nFrac * ac.n_max * ac.mass * G / q);
    return [clamp(cl, ac.cl_min, ac.cl_max), 0.0, this.tMax];
  };

  // ---------------------------------------------------------------------------- controller
  // The library mode of fly_mpc: pick the cycle, match its phase to the state, plan.
  // entries: the library entries of the aircraft; opt: planner settings, actuators (library
  // block), seed.
  function Controller(acParams, entries, opt) {
    opt = opt || {};
    this.ac = makeAircraft(acParams);
    this.entries = entries;
    this.pl = new Planner(this.ac, opt.settings);
    this.pl.act = opt.actuators ? actuatorParams(this.ac, {
      rateModel: opt.actuators.rate_model, tau: opt.actuators.tau_s, clRate: opt.actuators.cl_rate_per_s,
      kPb2v: opt.actuators.k_pb2v, delay: opt.actuators.delay_s,
      thrustTau: (opt.settings && opt.settings.motorValues || {}).thrustTauS }) : null;
    this.tauPrev = null;
  }
  Controller.prototype.reset = function () { this.pl.reset(); this.tauPrev = null; this.prevCycle = null; this.switchWaitT = null; };
  // req: {t, s, act: {cl, mu, clCmd, muCmd} | null, world: spec, u10, chiDeg, cycle?: {kind, u10, chi_deg, mirror}}
  Controller.prototype.plan = function (req) {
    var pl = this.pl, world = makeWorld(req.world), w = req.world.wind;
    var psiW = Math.atan2(w.ey, w.ex), chi = req.chiDeg * Math.PI / 180;
    var cyc;
    if (req.entry) {   // a steady motor leg: its own entry (see navigator.js, steadyEntry)
      cyc = new Cycle(req.entry, false);
    } else if (req.cycle) {   // the navigator names the cycle: {kind, u10, chi_deg, mirror}
      var ck = req.cycle, ent = null, q;
      for (q = 0; q < this.entries.length; q++) {
        var en = this.entries[q];
        if (en.u10 === ck.u10 && en.chi_deg === ck.chi_deg && (en.kind || 'soaring') === ck.kind) { ent = en; break; }
      }
      if (ent === null) throw new Error('no such cycle ' + JSON.stringify(ck));
      cyc = new Cycle(ent, !!ck.mirror);
    } else cyc = pickCycle(this.entries, req.u10, chi);
    var pc = this.prevCycle;
    if (pc && (pc.d.kind || 'soaring') === 'soaring' && (cyc.d.kind || 'soaring') === 'soaring' &&
        (pc.d !== cyc.d || pc.mirror !== cyc.mirror) && req.cycle) {
      // a change between soaring cycles waits for the phase where the new cycle matches the present state
      // (sim.mpc SWITCH_MATCH_MAX, SWITCH_MAX_WAIT_S): at any other phase the mirrored heading and bank are a jump
      var probe = new Guide(cyc, world, req.t, req.s, psiW, pl.horizonS, pl.wD, null, undefined, req.guideSig);
      if (this.switchWaitT === null || this.switchWaitT === undefined) this.switchWaitT = req.t;
      if (probe.matchDist > SWITCH_MATCH_MAX && req.t - this.switchWaitT < SWITCH_MAX_WAIT_S) cyc = pc;
      else this.switchWaitT = null;
    } else this.switchWaitT = null;
    if (this.prevCycle && (this.prevCycle.d !== cyc.d || this.prevCycle.mirror !== cyc.mirror)) this.tauPrev = null;   // another cycle: match its phase afresh
    this.prevCycle = cyc;
    var guide = new Guide(cyc, world, req.t, req.s, psiW, pl.horizonS, pl.wD,
      this.tauPrev === null ? null : this.tauPrev + pl.replanS, undefined, req.guideSig);
    this.tauPrev = guide.tau0;
    var a = req.act;
    pl.actState = a ? [a.cl, a.mu, a.clCmd, a.muCmd].concat(pl.motor && a.T !== undefined && a.T !== null ? [a.T] : []) : null;
    if (!a && pl.motor) {   // the very first plan: the actuators start at the cycle's own controls and thrust
      var n0 = guide.nominal(pl.knotsT), th0 = guide.nominalThrust(pl.knotsT);
      pl.actState = [n0[0], n0[pl.nCtrl], n0[0], n0[pl.nCtrl], th0[0]];
    }
    var plan = pl.plan(world, req.t, req.s, guide);
    plan.t0 = req.t;
    plan.info = { tau0: guide.tau0, period: cyc.T, u10: cyc.d.u10, chiDeg: cyc.d.chi_deg,
      mirror: cyc.mirror, marginSet: cyc.d.margin_set, matchDist: guide.matchDist,
      kind: cyc.d.kind || 'soaring', powerW: cyc.d.power_avg_w || 0 };
    return plan;
  };

  // ---------------------------------------------------------------------------- worker
  // The worker body: one Controller per 'init' message, a plan per 'plan' message.
  function workerMain(MPC) {
    var ctl = null;
    self.onmessage = function (e) {
      var m = e.data;
      try {
        if (m.type === 'init') {
          ctl = new MPC.Controller(m.ac, m.entries, { settings: m.settings, actuators: m.actuators });
          self.postMessage({ type: 'ready' });
        } else if (m.type === 'reset') {
          if (ctl) ctl.reset();
        } else if (m.type === 'plan') {
          var t0 = performance.now();
          var p = ctl.plan(m.req);
          var ms = performance.now() - t0;
          self.postMessage({ type: 'plan', id: m.id, t0: p.t0, ms: ms, knotsT: p.knotsT, cl: p.cl,
            mu: p.mu, th: p.th, path: p.path, dt: ctl.pl.dt, info: p.info, ess: p.ess, nDead: p.nDead });
        }
      } catch (err) {
        self.postMessage({ type: 'error', message: String(err && err.stack || err) });
      }
    };
  }

  return { G: G, wrap: wrap, interp: interp, DEFAULTS: DEFAULTS, makeAircraft: makeAircraft,
    SeaWorld: SeaWorld, flatWorld: flatWorld, makeWorld: makeWorld, limitComponents: limitComponents,
    Actuators: Actuators, actuatorParams: actuatorParams, nLimitedCl: nLimitedCl,
    Cycle: Cycle, pickCycle: pickCycle, Guide: Guide, Planner: Planner, Controller: Controller,
    planControls: planControls, planThrust: planThrust, Reflex: Reflex, REFLEX: REFLEX, SUSTAIN: SUSTAIN, foldEffective: foldEffective, MOTOR: MOTOR, ENERGY_K: ENERGY_K, workerMain: workerMain, Gauss: Gauss };
}

(function () {
  'use strict';
  var MPC = mpcFactory();
  // Source text of a worker that holds the planner (for a Blob URL; works from file://).
  MPC.workerSource = function () {
    return mpcFactory.toString() + '\n(' + MPC.workerMain.toString() + ')(mpcFactory());\n';
  };
  globalThis.MPC = MPC;
  if (typeof module !== 'undefined' && module.exports) module.exports = MPC;
})();
