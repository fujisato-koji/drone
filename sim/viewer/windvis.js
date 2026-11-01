// The wind display of the viewer: a horizontal grid of arrows that follows the aircraft, and a vertical section
// through it with the deviation from the mean log profile and optional streamlines. Classic script: attaches
// WindVis to globalThis, and sets module.exports under node.
//
// Every value comes from a column sampler column(x, y, t, hs, n, U, V, W) that returns the surface elevation at
// (x, y) and writes the wind at the heights hs[0..n-1] above that surface into U, V, W. The viewer builds it on its
// own surface() and windAt() (or on the regular swell's oracle world); columnOf() builds one from any pair of a
// wind function wind(x, y, z, t) -> [u, v, w] and a surface function surface(x, y, t) -> eta, as the tests do.
(function (root) {
  'use strict';
  var DEFAULTS = {
    gridN: 13, gridHalf: 150,    // arrows per side, half width (m); the grid snaps to a lattice of its own spacing
    mastH: 10, lowH: 2, minH: 0.5,   // layer height without a flight, the near-surface layer, the lowest layer
    arrowScale: 1,               // grid arrow length (m) per m/s of horizontal wind
    secHalf: 150, secDs: 2.5,    // section half length and column spacing (m)
    secLevels: [0.5, 1, 2, 3, 4, 5, 7, 9, 11, 13, 15, 17, 19, 21, 23, 25],   // heights above the local surface (m)
    secArrowEvery: 2,            // an arrow on every second column, at these of the levels above
    secArrowLevels: [1, 3, 5, 7, 9, 11, 13, 15, 17, 19, 21, 23, 25],
    secExag: 5,                  // section arrows: the deviation drawn secExag times the grid's scale
    secCap: 8,                   // ... and no longer than secCap (m), so that gusts do not cover the section
    secChunkMs: 4,               // the viewer samples a section in pieces of about this much time per frame
    wMax: 0.5,                   // the colour scale of the vertical wind saturates at +- wMax (m/s)
    refreshS: 0.25,              // the displays are recomputed every refreshS of real time
    streamSeeds: [1, 2, 3, 4.5, 6, 8, 10, 12.5, 15, 18, 21, 24],   // streamline seeds above the surface (m)
    streamStep: 1, streamMaxSteps: 800, streamRestarts: 40, chevronEvery: 30, chevronLen: 1.6
  };

  // a column sampler from a wind function and a surface function (z is the height above the mean sea level)
  function columnOf(wind, surface) {
    return function (x, y, t, hs, n, U, V, W) {
      var e = surface(x, y, t);
      for (var j = 0; j < n; j++) {
        var q = wind(x, y, e + hs[j], t);
        U[j] = q[0]; V[j] = q[1]; W[j] = q[2];
      }
      return e;
    };
  }

  // the grid's points: n x n at the spacing 2 half / (n - 1), on the lattice of that spacing nearest (cx, cy)
  function gridPoints(cx, cy, o) {
    o = Object.assign({}, DEFAULTS, o || {});
    var n = o.gridN, d = 2 * o.gridHalf / (n - 1), x0 = Math.round(cx / d) * d, y0 = Math.round(cy / d) * d;
    var x = new Float64Array(n * n), y = new Float64Array(n * n), m = (n - 1) / 2;
    for (var i = 0; i < n; i++) for (var j = 0; j < n; j++) { x[i * n + j] = x0 + (j - m) * d; y[i * n + j] = y0 + (i - m) * d; }
    return { n: n, d: d, x: x, y: y };
  }

  // the wind on the grid at the height h above the local surface: eta, z = eta + h, u, v, w at every point
  function sampleGrid(pts, h, column, t, out) {
    var np = pts.x.length, U = [0], V = [0], W = [0], hs = [h];
    if (!out || out.u.length < np) out = { eta: new Float64Array(np), z: new Float64Array(np), u: new Float64Array(np),
      v: new Float64Array(np), w: new Float64Array(np) };
    for (var p = 0; p < np; p++) {
      var e = column(pts.x[p], pts.y[p], t, hs, 1, U, V, W);
      out.eta[p] = e; out.z[p] = e + h; out.u[p] = U[0]; out.v[p] = V[0]; out.w[p] = W[0];
    }
    out.n = np; out.h = h; out.t = t;
    return out;
  }

  // The section through (cx, cy) along the unit vector (ex, ey): columns every secDs on the lattice of that step along
  // the line, at the heights secLevels above the local surface. mean(h) is the speed of the mean log profile and
  // (wx, wy) its direction; the deviation is the wind less that profile, in the section's plane (along e and up).
  // sectionLayout places the columns; sampleColumns fills the columns i0 to i1 - 1 at the time t (the viewer spreads
  // a section over a few frames this way); sampleSection does both at once.
  function sectionLayout(sp, o) {
    o = Object.assign({}, DEFAULTS, o || {});
    var ex = sp.ex, ey = sp.ey, ds = o.secDs, m = Math.round(o.secHalf / ds), nc = 2 * m + 1;
    var L = o.secLevels, nl = L.length;
    var s0 = sp.cx * ex + sp.cy * ey, p = -sp.cx * ey + sp.cy * ex, sc = Math.round(s0 / ds) * ds;
    var r = { nc: nc, nl: nl, levels: L.slice(), ex: ex, ey: ey, p: p, sAc: s0, ds: ds, proj: sp.wx * ex + sp.wy * ey,
      s: new Float64Array(nc), x: new Float64Array(nc), y: new Float64Array(nc), eta: new Float64Array(nc),
      u: new Float64Array(nc * nl), v: new Float64Array(nc * nl), w: new Float64Array(nc * nl),
      us: new Float64Array(nc * nl), dev: new Float64Array(nc * nl), mean: new Float64Array(nl), done: 0 };
    for (var j = 0; j < nl; j++) r.mean[j] = sp.mean(L[j]);
    for (var i = 0; i < nc; i++) {
      var s = sc + (i - m) * ds;
      r.s[i] = s; r.x[i] = s * ex - p * ey; r.y[i] = s * ey + p * ex;
    }
    return r;
  }
  function sampleColumns(r, column, t, i0, i1) {
    var nl = r.nl, L = r.levels, U = new Float64Array(nl), V = new Float64Array(nl), W = new Float64Array(nl);
    r.t = t;
    for (var i = i0; i < Math.min(i1, r.nc); i++) {
      r.eta[i] = column(r.x[i], r.y[i], t, L, nl, U, V, W);
      for (var j = 0; j < nl; j++) {
        var k = i * nl + j, us = U[j] * r.ex + V[j] * r.ey;
        r.u[k] = U[j]; r.v[k] = V[j]; r.w[k] = W[j]; r.us[k] = us;
        r.dev[k] = us - r.mean[j] * r.proj;
      }
    }
    r.done = Math.max(r.done, Math.min(i1, r.nc));
    return r;
  }
  function sampleSection(sp, column, t, o) {
    var r = sectionLayout(sp, o);
    return sampleColumns(r, column, t, 0, r.nc);
  }

  // the in-plane velocity relative to a frame moving at vs along the section, interpolated at (s, z):
  // linear in s between columns and linear in the height above the interpolated surface between levels
  // (clamped to the lowest and highest level). Writes [qs, qz, h] into q; returns false outside the columns.
  function velocityAt(r, vs, s, z, q) {
    var fi = (s - r.s[0]) / r.ds;
    if (fi < -1e-9 || fi > r.nc - 1 + 1e-9) return false;
    fi = Math.min(r.nc - 1, Math.max(0, fi));
    var i = Math.min(r.nc - 2, Math.floor(fi)), a = fi - i;
    var e = r.eta[i] * (1 - a) + r.eta[i + 1] * a, h = z - e, L = r.levels, nl = r.nl;
    var hc = Math.min(L[nl - 1], Math.max(L[0], h)), j = 0;
    while (j < nl - 2 && L[j + 1] < hc) j++;
    var b = (hc - L[j]) / (L[j + 1] - L[j]);
    var k0 = i * nl + j, k1 = (i + 1) * nl + j;
    var q0 = (r.us[k0] * (1 - b) + r.us[k0 + 1] * b), q1 = (r.us[k1] * (1 - b) + r.us[k1 + 1] * b);
    var w0 = (r.w[k0] * (1 - b) + r.w[k0 + 1] * b), w1 = (r.w[k1] * (1 - b) + r.w[k1 + 1] * b);
    q[0] = q0 * (1 - a) + q1 * a - vs; q[1] = w0 * (1 - a) + w1 * a; q[2] = h;
    return true;
  }

  // Streamlines of the in-plane velocity relative to the frame moving at vs along the section (vs = 0: fixed to the
  // sea). Each seed starts at the end of the section the relative flow enters at that height and is integrated by
  // the midpoint rule in steps of streamStep of arc length until it leaves the section, touches the surface or
  // rises above the highest level. In the frame of the sea the surface moves, so the streamlines of the moment cross
  // it; a line that ends at the surface or at the top starts again at its seed's height one column further on (at most
  // streamRestarts times), so that the section stays covered. Returns a list of {s: [], z: [], h0, restart} polylines
  // in the section's coordinates.
  function etaAt(r, s) {
    var fi = Math.min(r.nc - 1, Math.max(0, (s - r.s[0]) / r.ds)), i = Math.min(r.nc - 2, Math.floor(fi)), a = fi - i;
    return r.eta[i] * (1 - a) + r.eta[i + 1] * a;
  }
  function streamlines(r, vs, o) {
    o = Object.assign({}, DEFAULTS, o || {});
    var q = [0, 0, 0], q2 = [0, 0, 0], out = [], L = r.levels, top = L[L.length - 1];
    var sMin = r.s[0], sMax = r.s[r.nc - 1], st = o.streamStep;
    for (var k = 0; k < o.streamSeeds.length; k++) {
      var hSeed = o.streamSeeds[k], s = null, z, sg;
      if (velocityAt(r, vs, sMin, r.eta[0] + hSeed, q) && q[0] > 0) { s = sMin; sg = 1; }
      else if (velocityAt(r, vs, sMax, r.eta[r.nc - 1] + hSeed, q) && q[0] < 0) { s = sMax; sg = -1; }
      if (s === null) continue;
      z = etaAt(r, s) + hSeed;
      for (var restart = 0; restart <= o.streamRestarts; restart++) {
        var line = { s: [s], z: [z], h0: hSeed, restart: restart > 0 }, ends = false;
        for (var n = 0; n < o.streamMaxSteps; n++) {
          if (!velocityAt(r, vs, s, z, q)) break;
          var m1 = Math.hypot(q[0], q[1]);
          if (m1 < 1e-6) break;
          var sm = s + 0.5 * st * q[0] / m1, zm = z + 0.5 * st * q[1] / m1;
          if (!velocityAt(r, vs, sm, zm, q2)) break;
          var m2 = Math.hypot(q2[0], q2[1]);
          if (m2 < 1e-6) break;
          s += st * q2[0] / m2; z += st * q2[1] / m2;
          if (s < sMin || s > sMax) break;
          if (!velocityAt(r, vs, s, z, q)) break;
          line.s.push(s); line.z.push(z);
          if (q[2] < 0.05 || q[2] > top) { ends = true; break; }
        }
        if (line.s.length > 1) out.push(line);
        if (!ends) break;
        s += sg * r.ds;   // start again one column further on, at the seed's height
        if (s < sMin || s > sMax) break;
        z = etaAt(r, s) + hSeed;
        if (!velocityAt(r, vs, s, z, q) || q[0] * sg <= 0) break;
      }
    }
    return out;
  }

  // The diverging colour of the vertical wind: the cool pole for downward, a neutral grey at zero, the warm pole for
  // upward, linear in sRGB on each arm and saturated at +- wMax. Writes sRGB in 0..1 into out.
  var POLE_DOWN = [0x2a / 255, 0x78 / 255, 0xd6 / 255], NEUTRAL = [0xd6 / 255, 0xd4 / 255, 0xcd / 255], POLE_UP = [0xe3 / 255, 0x49 / 255, 0x48 / 255];
  function divColor(w, wMax, out) {
    var f = Math.max(-1, Math.min(1, w / (wMax || DEFAULTS.wMax))), pole = f < 0 ? POLE_DOWN : POLE_UP, a = Math.abs(f);
    for (var c = 0; c < 3; c++) out[c] = NEUTRAL[c] * (1 - a) + pole[c] * a;
    return out;
  }

  var WindVis = { DEFAULTS: DEFAULTS, columnOf: columnOf, gridPoints: gridPoints, sampleGrid: sampleGrid,
    sectionLayout: sectionLayout, sampleColumns: sampleColumns, sampleSection: sampleSection, velocityAt: velocityAt, etaAt: etaAt, streamlines: streamlines, divColor: divColor,
    POLES: { down: POLE_DOWN, neutral: NEUTRAL, up: POLE_UP } };
  root.WindVis = WindVis;
  if (typeof module !== 'undefined' && module.exports) module.exports = WindVis;
})(typeof window !== 'undefined' ? window : globalThis);
