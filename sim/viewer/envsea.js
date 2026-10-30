// The sea of the free wind setting: wind speed, direction, growth time and an optional swell go in,
// wave components of the viewer (a, k, omega, phase, dx, dy, urel) come out, built with seagen.js.
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

  var EnvSea = { build: build, AGE_HOURS: AGE_HOURS };
  root.EnvSea = EnvSea;
  if (typeof module !== 'undefined' && module.exports) module.exports = EnvSea;
})(typeof window !== 'undefined' ? window : globalThis);
