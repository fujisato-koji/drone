// Point-mass glider flight in a wind field: a JavaScript port of sim/aircraft.py (derivatives,
// rk4). The controls come from the MPC (mpc.js). Classic script: attaches Flight to globalThis,
// and sets module.exports under node.
// Sim coordinates: x east, y north, z up. State s = [x, y, z, V, gamma, psi] (V = airspeed,
// gamma = air-relative climb angle, psi = heading, math angle from east, counter-clockwise).
(function () {
  'use strict';
  var G = 9.81;
  var H = 0.05, DT = 0.01;   // finite-difference steps of sim.aircraft.wind_derivs

  function wrap(a) { return a - 2 * Math.PI * Math.floor((a + Math.PI) / (2 * Math.PI)); }

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

  var Flight = { G: G, derivatives: derivatives, rk4: rk4, metrics: metrics, wrap: wrap };
  globalThis.Flight = Flight;
  if (typeof module !== 'undefined' && module.exports) module.exports = Flight;
})();
