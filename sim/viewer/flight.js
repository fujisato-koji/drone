// Point-mass glider flight in a wind field: a JavaScript port of sim/aircraft.py (derivatives,
// rk4) and of the reference tracker in sim/autonomy.py (ReferenceTracker), driven by the true
// state. Classic script: attaches Flight to globalThis, and sets module.exports under node.
// Sim coordinates: x east, y north, z up. State s = [x, y, z, V, gamma, psi] (V = airspeed,
// gamma = air-relative climb angle, psi = heading, math angle from east, counter-clockwise).
(function () {
  'use strict';
  var G = 9.81;
  var H = 0.05, DT = 0.01;   // finite-difference steps of sim.aircraft.wind_derivs

  function wrap(a) { return a - 2 * Math.PI * Math.floor((a + Math.PI) / (2 * Math.PI)); }
  function clamp(v, lo, hi) { return Math.min(hi, Math.max(lo, v)); }

  // windFn(x, y, z, t) -> [u, v, w]. Optional windFn.z0 is the roughness length used only to
  // pick the one-sided z derivative near the surface, as aircraft.wind_derivs does.
  // Optional nLimit (default false): reduce cl to the load-factor limit at the current airspeed
  // (sim.aircraft.n_limited_cl), as the MPC mode flies.
  // Optional thrust (N, default 0): along the air-relative velocity, adds thrust / m to dV/dt.
  // Optional foldDcd (default undefined, off): the folded-propeller drag, cd0 + foldDcd (1 - T / (T +
  // 0.01 m g)), as sim.aircraft.derivatives (fold_dcd).
  function derivatives(s, cl, mu, ac, windFn, t, nLimit, thrust, foldDcd) {
    var x = s[0], y = s[1], z = s[2], V = s[3], g = s[4], p = s[5];
    if (nLimit) cl = Math.min(cl, ac.n_max * ac.mass * G / (0.5 * ac.rho * V * V * ac.wing_area));
    var cg = Math.cos(g), sg = Math.sin(g), cp = Math.cos(p), sp = Math.sin(p);
    var eV = [cg * cp, cg * sp, sg], eg = [-sg * cp, -sg * sp, cg], ep = [-sp, cp, 0];
    var z0 = windFn.z0 || 0;
    var low = z - H <= z0;
    var zlo = low ? z : z - H;
    var W = windFn(x, y, z, t);
    var wxp = windFn(x + H, y, z, t), wxm = windFn(x - H, y, z, t);
    var wyp = windFn(x, y + H, z, t), wym = windFn(x, y - H, z, t);
    var wzp = windFn(x, y, z + H, t), wzm = windFn(x, y, zlo, t);
    var wtp = windFn(x, y, z, t + DT), wtm = windFn(x, y, z, t - DT);
    var vg = [V * eV[0] + W[0], V * eV[1] + W[1], V * eV[2] + W[2]];
    var dz = low ? H : 2 * H;
    var aW = [0, 0, 0];
    for (var i = 0; i < 3; i++) {
      var gx = (wxp[i] - wxm[i]) / (2 * H), gy = (wyp[i] - wym[i]) / (2 * H);
      var gz = (wzp[i] - wzm[i]) / dz;
      aW[i] = (wtp[i] - wtm[i]) / (2 * DT) + gx * vg[0] + gy * vg[1] + gz * vg[2];
    }
    var dot = function (a, b) { return a[0] * b[0] + a[1] * b[1] + a[2] * b[2]; };
    var k = ac.k_induced;
    var q = 0.5 * ac.rho * V * V * ac.wing_area;
    var cd0 = ac.cd0;
    if (foldDcd !== undefined) cd0 = cd0 + foldDcd * (1.0 - thrust / (thrust + 0.01 * ac.mass * G));
    var L = q * cl, D = q * (cd0 + k * cl * cl), m = ac.mass;
    var dV = -D / m - G * sg - dot(aW, eV);
    if (thrust) dV = dV + thrust / m;
    var dg = (L * Math.cos(mu) / m - G * cg - dot(aW, eg)) / V;
    var cgf = Math.max(Math.abs(cg), 0.05);   // floor near vertical flight
    var dp = (L * Math.sin(mu) / m - dot(aW, ep)) / (V * cgf);
    return [vg[0], vg[1], vg[2], dV, dg, dp];
  }

  // classical RK4 with the controls held over the step
  function rk4(s, cl, mu, ac, windFn, t, dt, nLimit, thrust, foldDcd) {
    var add = function (a, b, c) { return a.map(function (v, i) { return v + c * b[i]; }); };
    var k1 = derivatives(s, cl, mu, ac, windFn, t, nLimit, thrust, foldDcd);
    var k2 = derivatives(add(s, k1, dt / 2), cl, mu, ac, windFn, t + dt / 2, nLimit, thrust, foldDcd);
    var k3 = derivatives(add(s, k2, dt / 2), cl, mu, ac, windFn, t + dt / 2, nLimit, thrust, foldDcd);
    var k4 = derivatives(add(s, k3, dt), cl, mu, ac, windFn, t + dt, nLimit, thrust, foldDcd);
    return s.map(function (v, i) { return v + dt / 6 * (k1[i] + 2 * k2[i] + 2 * k3[i] + k4[i]); });
  }

  // Heading the wind blows toward (math angle), from the wind 10 m above the local surface.
  function windPsi(windFn, surfaceFn, x, y, t, fallback) {
    var w = windFn(x, y, surfaceFn(x, y, t) + 10, t);
    return Math.hypot(w[0], w[1]) > 1e-3 ? Math.atan2(w[1], w[0]) : fallback;
  }

  // ref: {t, z, V, gamma, psi, cl, mu, ...} node arrays of one dsopt period (wind along +x; the
  // closing node, equal to the first, is dropped), or null for guard-only flight.
  function Tracker(ref, ac, opt) {
    opt = opt || {};
    this.ac = ac;
    this.kPsi = opt.kPsi !== undefined ? opt.kPsi : 1.0;
    this.kH = opt.kH !== undefined ? opt.kH : 0.05;
    this.kG = opt.kG !== undefined ? opt.kG : 1.0;
    this.fbMu = opt.fbMu !== undefined ? opt.fbMu : 0.5;
    this.fbCl = opt.fbCl !== undefined ? opt.fbCl : 0.5;
    this.clearance = opt.clearance !== undefined ? opt.clearance : 0.5;
    this.hs = 3.0; this.ps = 0.5;   // phase-matching weights (Reference h_scale, psi_scale)
    this.phase = null;              // fractional node index, null before the first match
    this.psiW = 0;
    this.guard = false;
    this.passes = [];               // {t, energy} at each wrap of the phase through node 0
    this.ref = null;
    if (ref && ref.z && ref.z.length > 2) {
      var n = ref.z.length - 1, r = {};
      ['t', 'z', 'V', 'gamma', 'psi', 'cl', 'mu'].forEach(function (k) { r[k] = ref[k].slice(0, n); });
      r.n = n; r.dh = []; r.dp = [];
      for (var i = 0; i < n; i++) {
        var j = (i + 1) % n;
        r.dh.push((r.z[j] - r.z[i]) / this.hs);
        r.dp.push(wrap(r.psi[j] - r.psi[i]) / this.ps);
      }
      this.ref = r;
    }
  }
  // phase of the nearest point of the (height, heading) loop; with prev the search covers
  // segments from prev - 2 to prev + n / 4 only, as Reference.match does
  Tracker.prototype.match = function (h, psiRel, prev) {
    var r = this.ref, n = r.n, best = Infinity, bk = 0, bs = 0;
    var back = 2.0, fwd = n / 4.0;
    for (var k = 0; k < n; k++) {
      if (prev !== null) {
        var off = ((k - Math.floor(prev) + n / 2) % n + n) % n - n / 2;
        if (off < -back || off > fwd) continue;
      }
      var qh = (h - r.z[k]) / this.hs, qp = wrap(psiRel - r.psi[k]) / this.ps;
      var d2 = r.dh[k] * r.dh[k] + r.dp[k] * r.dp[k];
      var sgm = clamp((qh * r.dh[k] + qp * r.dp[k]) / Math.max(d2, 1e-12), 0, 1);
      var dist = Math.pow(qh - sgm * r.dh[k], 2) + Math.pow(qp - sgm * r.dp[k], 2);
      if (dist < best) { best = dist; bk = k; bs = sgm; }
    }
    return (bk + bs) % n;
  };
  Tracker.prototype.at = function (phase) {
    var r = this.ref, n = r.n, k = Math.floor(phase) % n, f = phase - Math.floor(phase), j = (k + 1) % n;
    var lin = function (a) { return a[k] + f * (a[j] - a[k]); };
    return { z: lin(r.z), V: lin(r.V), gamma: lin(r.gamma),
      psi: r.psi[k] + f * wrap(r.psi[j] - r.psi[k]), cl: lin(r.cl), mu: lin(r.mu) };
  };
  // s: true state; returns {cl, mu, phase, guard}
  Tracker.prototype.command = function (s, windFn, surfaceFn, t) {
    var ac = this.ac, V = s[3];
    var h = s[2] - surfaceFn(s[0], s[1], t);
    var clBest = Math.sqrt(ac.cd0 / ac.k_induced);
    this.guard = h < this.clearance;
    if (this.guard || !this.ref) {
      this.guard = this.guard || !this.ref;   // no reference: guard mode only
      return { cl: this.limit(clBest, V), mu: 0, phase: this.phase, guard: this.guard };
    }
    this.psiW = windPsi(windFn, surfaceFn, s[0], s[1], t, this.psiW);
    var psiRel = wrap(s[5] - this.psiW);
    var ph = this.match(h, psiRel, this.phase);
    if (this.phase !== null && ph < this.phase - this.ref.n / 2) {
      this.passes.push({ t: t, energy: s[2] + V * V / (2 * G) });
    }
    this.phase = ph;
    var r = this.at(ph);
    var mu = r.mu + clamp(this.kPsi * wrap(r.psi - psiRel), -this.fbMu, this.fbMu);
    var cl = r.cl + clamp(this.kH * (r.z - h) + this.kG * (r.gamma - s[4]), -this.fbCl, this.fbCl);
    cl = clamp(cl, ac.cl_min, ac.cl_max);
    mu = clamp(mu, -ac.mu_max, ac.mu_max);
    return { cl: this.limit(cl, V), mu: mu, phase: ph, guard: false };
  };
  // load-factor limit: reduce |cl| so that |n| <= n_max
  Tracker.prototype.limit = function (cl, V) {
    var ac = this.ac, q = 0.5 * ac.rho * V * V * ac.wing_area;
    var clN = ac.n_max * ac.mass * G / Math.max(q, 1e-9);
    return clamp(cl, -clN, clN);
  };

  // The optional cl (the control in use) is needed for the load factor.
  function metrics(s, ac, windFn, surfaceFn, t, cl) {
    var V = s[3], g = s[4], p = s[5];
    var W = windFn(s[0], s[1], s[2], t);
    var vx = V * Math.cos(g) * Math.cos(p) + W[0], vy = V * Math.cos(g) * Math.sin(p) + W[1];
    var q = 0.5 * ac.rho * V * V * ac.wing_area;
    return { airspeed: V, height: s[2] - surfaceFn(s[0], s[1], t),
      energy: s[2] + V * V / (2 * G),
      loadFactor: cl === undefined ? NaN : q * cl / (ac.mass * G),
      groundSpeed: Math.hypot(vx, vy) };
  }

  var Flight = { G: G, derivatives: derivatives, rk4: rk4, Tracker: Tracker, metrics: metrics,
    windPsi: windPsi, wrap: wrap };
  globalThis.Flight = Flight;
  if (typeof module !== 'undefined' && module.exports) module.exports = Flight;
})();
