// The wind display of the viewer: layers of arrows on a horizontal grid that follows the aircraft, each at a fixed
// height above the local surface (or at the aircraft's height). Classic script: attaches WindVis to globalThis, and
// sets module.exports under node.
//
// Every value comes from a column sampler column(x, y, t, hs, n, U, V, W) that returns the surface elevation at
// (x, y) and writes the wind at the heights hs[0..n-1] above that surface into U, V, W. The viewer builds it on its
// own surface() and windAt() (or on the regular swell's oracle world); columnOf() builds one from any pair of a
// wind function wind(x, y, z, t) -> [u, v, w] and a surface function surface(x, y, t) -> eta, as the tests do.
(function (root) {
  'use strict';
  var DEFAULTS = {
    gridN: 13, gridHalf: 150,    // arrows per side, half width (m); the grid snaps to a lattice of its own spacing
    layerHeights: [2, 5],        // the fixed layers' heights above the local surface (m)
    mastH: 10, minH: 0.5,        // the aircraft's layer without a flight, and its lowest height
    arrowScale: 0.5,             // arrow length (m) per m/s of horizontal wind
    widthPerLength: 0.08, widthMax: 0.35,   // the arrow's shaft width (m): this share of its length, at most widthMax
    wMax: 0.5,                   // the colour scale of the vertical wind saturates at +- wMax (m/s)
    refreshS: 0.25               // the layers are recomputed every refreshS of real time
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

  // the length and the shaft width (m) of the arrow for a horizontal speed (m/s)
  function arrowSize(speed, o) {
    o = o || DEFAULTS;
    var L = speed * o.arrowScale;
    return [L, Math.min(o.widthMax, o.widthPerLength * L)];
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
    arrowSize: arrowSize, divColor: divColor, POLES: { down: POLE_DOWN, neutral: NEUTRAL, up: POLE_UP } };
  root.WindVis = WindVis;
  if (typeof module !== 'undefined' && module.exports) module.exports = WindVis;
})(typeof window !== 'undefined' ? window : globalThis);
