// Time-varying wind for the viewer: gusts from a frozen random Fourier turbulence field and a
// slow mean-wind variation. Classic script: attaches Gust to globalThis, and sets module.exports
// under node.
//
// Gusts: a port of Environment._make_turbulence and Environment.gust of sim/seawind.py for
// turbulence="iso", turbulence_profile="height" (von Karman length scales that follow the height
// above the surface, L_u = 2.4 z, L_w = L_v = 0.58 z, capped at 50 m; ISO intensity with the
// lateral and vertical ratios 0.8 and 0.5). The modes come from a seeded mulberry32 generator,
// not numpy's, so the same seed gives the same field in JavaScript and in Python only through
// sim/gustmodes.py, which makes the same modes (see test_gust_js.py).
//
// Slow variation: sums of cosines with a seeded random phase. "calm" has the shape of Larsen's
// mesoscale spectrum S(f) = a1 f^(-5/3) + a2 f^(-3) between 1/(6 h) and 1/(10 min), scaled to a
// standard deviation (record P147 to P154). "cell" is open-cell convection: three components with
// periods of 30 to 60 min and a total amplitude of 2 to 4 m/s.
(function (root) {
  'use strict';
  var TWO_PI = 2 * Math.PI;
  var DEFAULTS = {
    nModes: 64, kLo: TWO_PI / 500, kHi: TWO_PI / 2,
    length: 50, wRatio: 0.5, vRatio: 0.8, lwPerZ: 0.58, luPerZ: 2.4,
    // Larsen's mesoscale spectrum (record P147 to P154), the band and the calm-mode deviation
    a1: 3e-4, a2: 3e-11, fSlowLo: 1 / 21600, fSlowHi: 1 / 600, calmSigma: 0.3, nSlow: 48,
    // WAM wind input e-folding times of the short waves (record P153): 21 s at 0.5 s period and
    // 50 min at 4 s, at 10 m/s, joined by a power law and limited to 1 to 10 min
    tauT1: 0.5, tau1: 21, tauT2: 4, tau2: 3000, tauMin: 60, tauMax: 600, shortPeriod: 2
  };

  // seeded generator (mulberry32), uniform in [0, 1); the same as seagen.js rng
  function rng(seed) {
    var t = (seed >>> 0) || 0;
    return function () {
      t = (t + 0x6D2B79F5) >>> 0;
      var r = Math.imul(t ^ (t >>> 15), 1 | t);
      r = (r + Math.imul(r ^ (r >>> 7), 61 | r)) ^ r;
      return ((r ^ (r >>> 14)) >>> 0) / 4294967296;
    };
  }

  // Modes of the frozen field: n log-uniform wavenumbers, n angles, then three rows of n phases.
  function modes(seed, o) {
    o = Object.assign({}, DEFAULTS, o || {});
    var n = o.nModes, rand = rng(seed), lo = Math.log(o.kLo), hi = Math.log(o.kHi), i;
    var k = new Float64Array(n), ang = new Float64Array(n), ph = [];
    for (i = 0; i < n; i++) k[i] = Math.exp(lo + (hi - lo) * rand());
    for (i = 0; i < n; i++) ang[i] = TWO_PI * rand();
    for (var r = 0; r < 3; r++) {
      var row = new Float64Array(n);
      for (i = 0; i < n; i++) row[i] = TWO_PI * rand();
      ph.push(row);
    }
    var kx = new Float64Array(n), ky = new Float64Array(n);
    for (i = 0; i < n; i++) { kx[i] = k[i] * Math.sin(ang[i]); ky[i] = k[i] * Math.cos(ang[i]); }
    return { k: k, kx: kx, ky: ky, phase: ph, n: n };
  }

  // Mode amplitudes of unit variance for a length scale (sim.seawind._turb_amp)
  function amps(k, length, out) {
    var n = k.length, s = 0, i;
    for (i = 0; i < n; i++) {
      var kl = k[i] * length, kl2 = kl * kl;
      out[i] = kl2 * kl2 / Math.exp(17 / 6 * Math.log(1 + kl2)) * k[i];
      s += out[i];
    }
    for (i = 0; i < n; i++) out[i] = Math.sqrt(2 * out[i] / s);
    return out;
  }

  // (L_u, L_w) at the height zloc above the surface (sim.seawind.turbulence_length_scales)
  function lengthScales(zloc, o) {
    var floor = 1 / o.kHi;
    return [Math.max(Math.min(o.length, o.luPerZ * zloc), floor), Math.max(Math.min(o.length, o.lwPerZ * zloc), floor)];
  }

  // A gust field. at(x, y, z, zloc, t, w, out): turbulent velocity (east, north, up) into out.
  // z: height above the mean sea level (the intensity), zloc: height above the local surface (the
  // length scales). w: {u10, ex, ey, z0, ustar, karman, adv}: the mean wind that sets the
  // intensity and its direction; adv is the speed that carries the frozen field (default u10).
  function create(seed, options) {
    var o = Object.assign({}, DEFAULTS, options || {});
    var m = modes(seed, o), n = m.n;
    var au = new Float64Array(n), aw = new Float64Array(n);
    function at(x, y, z, zloc, t, w, out) {
      var adv = w.adv !== undefined ? w.adv : w.u10;
      var xs = x - adv * w.ex * t, ys = y - adv * w.ey * t;
      var zc = Math.max(z, 2 * w.z0);
      var iu = 0.06 * (1 + 0.043 * w.u10) * Math.pow(zc / 10, -0.22);
      var sp = zc > w.z0 ? (w.ustar / (w.karman || 0.4)) * Math.log(zc / w.z0) : 0;
      var sig = iu * sp;
      var L = lengthScales(Math.max(zloc, 2 * w.z0), o);
      amps(m.k, L[0], au); amps(m.k, L[1], aw);
      var lon = 0, lat = 0, ver = 0, p0 = m.phase[0], p1 = m.phase[1], p2 = m.phase[2];
      for (var i = 0; i < n; i++) {
        var ph = m.kx[i] * xs + m.ky[i] * ys;
        lon += au[i] * Math.cos(ph + p0[i]);
        lat += aw[i] * Math.cos(ph + p1[i]);
        ver += aw[i] * Math.cos(ph + p2[i]);
      }
      lon *= sig; lat *= sig * o.vRatio; ver *= sig * o.wRatio;
      out[0] = lon * w.ex - lat * w.ey;
      out[1] = lon * w.ey + lat * w.ex;
      out[2] = ver;
      return out;
    }
    return { modes: m, at: at, options: o };
  }

  // Slow variation of the mean 10 m wind speed, m/s about the set mean. kind 'calm' (standard
  // deviation sigma, default 0.3) or 'cell' (amplitude 2 to 4 m/s, periods 30 to 60 min).
  // Returns {kind, sigma, at(t)}; sigma is the standard deviation over a long time.
  function slowWind(seed, kind, options) {
    var o = Object.assign({}, DEFAULTS, options || {}), rand = rng(seed), i;
    var f = [], a = [], p = [];
    if (kind === 'calm') {
      var n = o.nSlow, lo = Math.log(o.fSlowLo), hi = Math.log(o.fSlowHi), w = [], ws = 0;
      for (i = 0; i < n; i++) {   // one frequency drawn in each of n equal bins of ln f
        var fi = Math.exp(lo + (hi - lo) * (i + rand()) / n);
        f.push(fi); p.push(TWO_PI * rand());
        w.push((o.a1 * Math.pow(fi, -5 / 3) + o.a2 * Math.pow(fi, -3)) * fi);   // S(f) f, power per ln f
        ws += w[i];
      }
      var sg = o.calmSigma;
      for (i = 0; i < n; i++) a.push(sg * Math.sqrt(2 * w[i] / ws));
    } else if (kind === 'cell') {
      var amp = 2 + 2 * rand(), share = [0.6, 0.3, 0.1];
      for (i = 0; i < 3; i++) {
        f.push(1 / (60 * (30 + 30 * rand()))); p.push(TWO_PI * rand()); a.push(amp * share[i]);
      }
    } else throw new Error('unknown slow kind ' + kind);
    var s2 = 0;
    for (i = 0; i < a.length; i++) s2 += a[i] * a[i] / 2;
    return {
      kind: kind, sigma: Math.sqrt(s2), f: f, a: a, phase: p,
      at: function (t) {
        var s = 0;
        for (var j = 0; j < a.length; j++) s += a[j] * Math.cos(TWO_PI * f[j] * t + p[j]);
        return s;
      }
    };
  }

  // e-folding time (s) with which short waves of period T follow the wind (WAM wind input)
  function responseTime(T, o) {
    o = Object.assign({}, DEFAULTS, o || {});
    var q = Math.log(o.tau2 / o.tau1) / Math.log(o.tauT2 / o.tauT1);
    return Math.min(o.tauMax, Math.max(o.tauMin, o.tau1 * Math.pow(T / o.tauT1, q)));
  }
  // amplitude factor of the short waves for a lagged mean wind: Hs follows U^2
  function shortScale(uLag, uSet) {
    var r = uLag / uSet;
    return Math.min(4, Math.max(0.25, r * r));
  }

  var Gust = { DEFAULTS: DEFAULTS, rng: rng, modes: modes, amps: amps, lengthScales: lengthScales,
    create: create, slowWind: slowWind, responseTime: responseTime, shortScale: shortScale };
  root.Gust = Gust;
  if (typeof module !== 'undefined' && module.exports) module.exports = Gust;
})(typeof window !== 'undefined' ? window : globalThis);
