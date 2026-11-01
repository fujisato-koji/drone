// The sea of the free wind setting: wind speed, direction, growth time and an optional swell go in,
// wave components of the viewer (a, k, omega, phase, dx, dy, urel) come out, built with seagen.js;
// and the snap between the irregular swell and the regular swell of the wave library.
// Classic script: attaches EnvSea to globalThis, and sets module.exports under node (which needs
// seagen.js next to it).
(function (root) {
  'use strict';
  var SG = root.SeaGen || (typeof require !== 'undefined' ? require('./seagen.js') : null);

  // set: {u10 (m/s), fromDeg, hours (0: fully developed, else the time since the wind began),
  //       swell: null | {hs, tp, fromDeg}, seed}
  // consts: {karman, charnock, nu_air} of the field (optional; seagen defaults otherwise)
  function build(set, consts) {
    var so = consts ? { karman: consts.karman, charnock: consts.charnock, nuAir: consts.nu_air } : undefined;
    var w = SG.wind(set.u10, set.fromDeg, 'charnock', so);
    var sea = set.hours > 0 ? SG.durationLimited(w, set.hours, so) : SG.fullyDeveloped(w, so);
    var sw = set.swell ? { hs: set.swell.hs, tp: set.swell.tp, to_deg: (set.swell.fromDeg + 180) % 360 } : null;
    var c = SG.combine(sea, sw, set.seed === undefined ? 1 : set.seed, { consts: so });
    var n = c.a.length, urel = new Float64Array(n);
    for (var i = 0; i < n; i++) {   // the wind of the page's setWind: speed at 1/k (not below 2 z0) along the wave, less its phase speed
      var zref = Math.max(1 / c.k[i], 2 * w.z0);
      urel[i] = w.speed(zref) * (w.ex * c.dx[i] + w.ey * c.dy[i]) - c.omega[i] / c.k[i];
    }
    var trains = [sea].concat(sw ? [SG.swellTrain(sw.hs, sw.tp, sw.to_deg, so)] : []);
    var hsTotal = Math.sqrt(trains.reduce(function (s, t) { return s + t.hs * t.hs; }, 0));
    return { wind: w, sea: sea, swell: sw, trains: trains, hsTotal: hsTotal,
      comps: { a: c.a, k: c.k, omega: c.omega, phase: c.phase, dx: c.dx, dy: c.dy, urel: urel }, slices: c.slices };
  }

  // Hours since the wind began for the slider index 0..5: 0 is fully developed
  var AGE_HOURS = [0, 1, 3, 6, 12, 24];

  // The swell of the free setting is none, irregular (built above) or regular (the oracle swell of the wave library,
  // sim/viewer/wave.js). The regular swell's direction is relative to the wind, as wave.js swellSea takes it: the swell
  // travels to the wind's compass direction less rel (0 along, 90 across to the left facing downwind, 180 against),
  // so rel = windFrom - swellFrom.
  var pymod = function (a, n) { return ((a % n) + n) % n; };
  var angDist = function (a, b) { var d = pymod(a - b, 360); return Math.min(d, 360 - d); };
  function nearest(list, v, dist) {   // the first of the nearest on a tie
    var best = list[0];
    for (var i = 1; i < list.length; i++) if (dist(list[i], v) < dist(best, v)) best = list[i];
    return best;
  }
  // The regular swell nearest to an irregular one ({hs, tp, fromDeg} under a wind from windFromDeg) on the grid of
  // the wave library (WAVE_INDEX.grid: hs, tp, directions {name: rel deg}): {hs, tp, direction}
  function snapRegular(windFromDeg, sw, grid) {
    var lin = function (a, b) { return Math.abs(a - b); };
    var names = Object.keys(grid.directions), rel = pymod(windFromDeg - sw.fromDeg, 360);
    var dir = nearest(names, rel, function (n, r) { return angDist(grid.directions[n], r); });
    return { hs: nearest(grid.hs, sw.hs, lin), tp: nearest(grid.tp, sw.tp, lin), direction: dir };
  }
  // and back: the irregular swell's height, period and compass direction of a regular swell {hs, tp, direction}
  function irregularOf(windFromDeg, reg, grid) {
    return { hs: reg.hs, tp: reg.tp, fromDeg: pymod(windFromDeg - grid.directions[reg.direction], 360) };
  }

  var EnvSea = { build: build, AGE_HOURS: AGE_HOURS, snapRegular: snapRegular, irregularOf: irregularOf };
  root.EnvSea = EnvSea;
  if (typeof module !== 'undefined' && module.exports) module.exports = EnvSea;
})(typeof window !== 'undefined' ? window : globalThis);
