// Wave components from a wind (and an optional swell), ported from sim/seawind.py
// (Wind, cem_limits, jonswap, WaveTrain) and sim/sea_history.py (swell gamma 7).
// Deterministic parts match Python; only the phases come from a seeded mulberry32
// generator and so differ from numpy's. Directions are deterministic in Python too
// (fixed cos^2 bins), so they match exactly.
// Defaults are the values in record/values.yaml at the time of writing; pass an
// options object to override any of them.
(function (root) {
  'use strict';
  const G = 9.81;
  const DEFAULTS = {
    karman: 0.40, charnock: 0.0144, nuAir: 1.46e-5,
    coare35Slope: 0.0017, coare35Intercept: -0.005, coare35Max: 0.028,
    jonswapGamma: 3.3, pmHs: 0.21, pmPeak: 0.877,
    cemHm0Full: 211.5, cemTpFull: 239.8, cemHm0Coeff: 4.13e-2, cemTpCoeff: 0.651,
    cemDurationCoeff: 5.23e-3, cdLawA: 1.1, cdLawB: 0.035,
    swellGamma: 7.0
  };
  const opt = (o) => Object.assign({}, DEFAULTS, o || {});

  function compassToVector(deg) {
    const r = deg * Math.PI / 180;
    return [Math.sin(r), Math.cos(r)];
  }

  // Mean wind of sim.seawind.Wind for roughness "charnock" or "coare35" (neutral log profile).
  function wind(u10, fromDeg, roughness, options) {
    const o = opt(options);
    roughness = roughness || 'charnock';
    if (!(u10 > 0)) throw new Error('u10 must be > 0');
    if (roughness !== 'charnock' && roughness !== 'coare35') throw new Error('unknown roughness ' + roughness);
    if (fromDeg === undefined) fromDeg = 270;
    const toDeg = (((fromDeg + 180) % 360) + 360) % 360;
    const e = compassToVector(toDeg);
    let alpha = o.charnock;
    if (roughness === 'coare35') alpha = Math.min(Math.max(o.coare35Slope * u10 + o.coare35Intercept, 0), o.coare35Max);
    let ustar = 0.03 * u10 + 1e-3, z0 = 0;
    for (let i = 0; i < 50; i++) {
      z0 = alpha * ustar * ustar / G + 0.11 * o.nuAir / ustar;
      ustar = o.karman * u10 / Math.log(10 / z0);
    }
    const w = { u10: u10, from_deg: fromDeg, to_deg: toDeg, ex: e[0], ey: e[1], ustar: ustar, z0: z0,
                alpha: alpha, karman: o.karman, roughness: roughness };
    w.speed = (z) => (z > z0 ? (ustar / o.karman) * Math.log(z / z0) : 0);
    w.opts = o;
    return w;
  }

  function cemLimits(u10, options) {
    const o = opt(options);
    const cd = 0.001 * (o.cdLawA + o.cdLawB * u10);
    const ust = Math.sqrt(cd) * u10;
    return { ustar: ust, hs: o.cemHm0Full * ust * ust / G, tp: o.cemTpFull * ust / G };
  }

  function train(hs, tp, toDeg, gamma, nOmega, nTheta, name, hsFull) {
    return { hs: hs, tp: tp, to_deg: toDeg, gamma: gamma, n_omega: nOmega, n_theta: nTheta,
             name: name, hs_full: hsFull || 0 };
  }

  // Pierson-Moskowitz fully developed sea; u19.5 from the log profile of this wind.
  function fullyDeveloped(w, options) {
    const o = opt(options || w.opts);
    const u = w.speed(19.5);
    const hs = o.pmHs * u * u / G;
    return train(hs, 2 * Math.PI * u / (o.pmPeak * G), w.to_deg, 1.0, 12, 8, 'wind sea', hs);
  }

  // CEM II-2-36 to II-2-38, each of Hs and Tp capped at its fully developed limit.
  function durationLimited(w, hours, options) {
    const o = opt(options || w.opts);
    if (!(hours > 0)) throw new Error('hours must be > 0');
    const c = cemLimits(w.u10, o);
    const xstar = o.cemDurationCoeff * Math.pow(G * hours * 3600 / c.ustar, 1.5);
    let hs = o.cemHm0Coeff * Math.sqrt(xstar) * c.ustar * c.ustar / G;
    let tp = o.cemTpCoeff * Math.cbrt(xstar) * c.ustar / G;
    const capped = hs >= c.hs && tp >= c.tp;
    hs = Math.min(hs, c.hs); tp = Math.min(tp, c.tp);
    return train(hs, tp, w.to_deg, capped ? 1.0 : o.jonswapGamma, 12, 8, 'wind sea', c.hs);
  }

  function jonswap(om, hs, tp, gamma) {
    const wp = 2 * Math.PI / tp;
    const sigma = om <= wp ? 0.07 : 0.09;
    const r = Math.exp(-((om - wp) * (om - wp)) / (2 * sigma * sigma * wp * wp));
    const ag = 1 - 0.287 * Math.log(gamma);
    return ag * (5 / 16) * hs * hs * Math.pow(wp, 4) * Math.pow(om, -5)
      * Math.exp(-1.25 * Math.pow(wp / om, 4)) * Math.pow(gamma, r);
  }

  const NGRID = 2000;

  // Equal-energy binning of the spectrum on 2000 points from 0.5 wp to 4 wp (the high-frequency cut).
  // Returns the bin centres (energy-weighted mean omega), edges (grid omega at the first point of each
  // bin, and the last grid point) and m0, the trapezoid integral of the spectrum on that grid.
  function binning(tr, nOmega) {
    nOmega = nOmega || tr.n_omega;
    const wp = 2 * Math.PI / tr.tp, lo = 0.5 * wp, hi = 4 * wp;
    const om = new Float64Array(NGRID), s = new Float64Array(NGRID), cum = new Float64Array(NGRID);
    for (let i = 0; i < NGRID; i++) {
      om[i] = lo + (hi - lo) * i / (NGRID - 1);
      s[i] = jonswap(om[i], tr.hs, tr.tp, tr.gamma);
    }
    for (let i = 1; i < NGRID; i++) cum[i] = cum[i - 1] + 0.5 * (s[i] + s[i - 1]) * (om[i] - om[i - 1]);
    const m0 = cum[NGRID - 1];
    const wo = new Float64Array(nOmega), ws = new Float64Array(nOmega);
    const edges = new Array(nOmega + 1).fill(NaN);
    for (let i = 0; i < NGRID; i++) {
      const j = Math.min(Math.floor(cum[i] / m0 * nOmega), nOmega - 1);
      wo[j] += s[i] * om[i]; ws[j] += s[i];
      if (isNaN(edges[j])) edges[j] = om[i];
    }
    edges[nOmega] = om[NGRID - 1];
    const centres = Array.from(wo, (v, j) => v / ws[j]);
    return { centres: centres, edges: edges, m0: m0 };
  }

  // Seeded generator (mulberry32); uniform in [0, 1).
  function rng(seed) {
    let t = (seed >>> 0) || 0;
    return function () {
      t = (t + 0x6D2B79F5) >>> 0;
      let r = Math.imul(t ^ (t >>> 15), 1 | t);
      r = (r + Math.imul(r ^ (r >>> 7), 61 | r)) ^ r;
      return ((r ^ (r >>> 14)) >>> 0) / 4294967296;
    };
  }

  // Components of one train, in the order of WaveTrain.components (omega-major, then direction).
  // n is the total count; the number of directions is options.nTheta (default tr.n_theta), the
  // number of frequencies round(n / nTheta). options.rng reuses a generator (for combine).
  function components(tr, n, seed, options) {
    const o = options || {};
    const nTheta = o.nTheta || tr.n_theta;
    const nOmega = n ? Math.max(1, Math.round(n / nTheta)) : tr.n_omega;
    const b = binning(tr, nOmega);
    const rand = o.rng || rng(seed);
    const th = new Float64Array(nTheta), d = new Float64Array(nTheta);
    let dsum = 0;
    for (let j = 0; j < nTheta; j++) {
      th[j] = tr.to_deg - 90 + (j + 0.5) * 180 / nTheta;
      d[j] = Math.pow(Math.cos((th[j] - tr.to_deg) * Math.PI / 180), 2);
      dsum += d[j];
    }
    const N = nOmega * nTheta;
    const out = { a: new Float64Array(N), k: new Float64Array(N), omega: new Float64Array(N),
                  dx: new Float64Array(N), dy: new Float64Array(N), phase: new Float64Array(N) };
    for (let i = 0; i < nOmega; i++) {
      for (let j = 0; j < nTheta; j++) {
        const q = i * nTheta + j, v = compassToVector(th[j]);
        out.a[q] = Math.sqrt(2 * (b.m0 / nOmega) * (d[j] / dsum));
        out.omega[q] = b.centres[i];
        out.k[q] = b.centres[i] * b.centres[i] / G;
        out.dx[q] = v[0]; out.dy[q] = v[1];
      }
    }
    for (let q = 0; q < N; q++) out.phase[q] = rand() * 2 * Math.PI;
    return out;
  }

  // Narrow swell: JONSWAP gamma 7 (sea_history), 8 x 4 components as in the exported field.
  function swellTrain(hs, tp, toDeg, options) {
    return train(hs, tp, toDeg, opt(options).swellGamma, 8, 4, 'swell', 0);
  }
  function swell(hs, tp, toDeg, n, seed, options) {
    return components(swellTrain(hs, tp, toDeg, options), n || 32, seed);
  }

  // One component set: wind sea (default 96 = 12 x 8) plus optional swell {hs, tp, to_deg}
  // (default 32 = 8 x 4), one generator over both. Returns the arrays plus slices of each train.
  function combine(sea, sw, seed, options) {
    const o = options || {};
    const rand = rng(seed);
    const parts = [];
    if (sea) parts.push(components(sea, o.nSea || sea.n_omega * sea.n_theta, seed, { rng: rand }));
    if (sw) parts.push(components(swellTrain(sw.hs, sw.tp, sw.to_deg, o.consts), o.nSwell || 32, seed, { rng: rand }));
    const keys = ['a', 'k', 'omega', 'dx', 'dy', 'phase'];
    const N = parts.reduce((s, p) => s + p.a.length, 0);
    const out = { slices: [] };
    for (const key of keys) out[key] = new Float64Array(N);
    let at = 0;
    for (const p of parts) {
      for (const key of keys) out[key].set(p[key], at);
      out.slices.push([at, at + p.a.length]);
      at += p.a.length;
    }
    return out;
  }

  const SeaGen = { G: G, DEFAULTS: DEFAULTS, wind: wind, cemLimits: cemLimits, fullyDeveloped: fullyDeveloped,
                   durationLimited: durationLimited, jonswap: jonswap, binning: binning, rng: rng,
                   components: components, swellTrain: swellTrain, swell: swell, combine: combine };
  root.SeaGen = SeaGen;
  if (typeof module !== 'undefined' && module.exports) module.exports = SeaGen;
})(typeof window !== 'undefined' ? window : globalThis);
