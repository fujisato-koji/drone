// Tactical layer for the viewer: a JavaScript mirror of the velocity-hull navigator in
// sim/navigator.py (VelocityHull, HullSource, HullNavigator, sailHull). Read the notes above the hull
// section of sim/navigator.py first: the cycles' mean ground velocities (with their mirror images)
// form the hull; for a target line the navigator picks a pair of cycles on either side of it
// (soaring first, motor cycles by least energy per metre only when no soaring pair makes progress)
// and flies them as legs inside a corridor that narrows towards the target.
//
// Frame: wind toward +x, y to the left of the downwind direction. A world vector (east, north)
// turns into it through psiW = -90 deg - windFrom (radians). Compass bearings in degrees.
// Attaches NAV to globalThis; module.exports under node. Parity with Python is tested in
// sim/tests/test_navigator_js.py.
(function () {
  'use strict';
  var MIN_VMG = 0.3;
  var LIMIT_FRAC = 0.5;
  var CROSS_ON_LINE = 0.2;

  function hypot(a, b) { return Math.sqrt(a * a + b * b); }

  // entry: library entry {u10, chi_deg, kind, power_avg_w, vx, vy}; points for both sides of the wind
  function pointsFromEntries(entries, u10) {
    var pts = [];
    entries.forEach(function (e) {
      if (e.u10 !== u10) return;
      var kind = e.kind || 'soaring';
      var pw = kind === 'motor' ? (e.power_avg_w || 0) : 0;
      [1, -1].forEach(function (side) {
        pts.push({ vx: e.vx, vy: side * e.vy, power: pw, kind: kind, side: side, entry: e,
          key: kind + ':' + e.u10 + ':' + e.chi_deg + ':' + side });
      });
    });
    return pts;
  }

  function VelocityHull(points, u10) { this.points = points; this.u10 = u10; }

  // lexicographic compare of two pairs
  function lessKey(a, b) { return a[0] < b[0] || (a[0] === b[0] && a[1] < b[1]); }

  VelocityHull.prototype.planPair = function (d, minVmg, motorOk) {
    if (minVmg === undefined) minVmg = MIN_VMG;
    if (motorOk === undefined) motorOk = true;
    var pts = this.points, n = pts.length, a = [], c = [], k;
    for (k = 0; k < n; k++) {
      a.push(pts[k].vx * d[0] + pts[k].vy * d[1]);
      c.push(d[0] * pts[k].vy - d[1] * pts[k].vx);
    }
    function best(idx, byEnergy) {
      var out = null, ii, jj, i, j;
      for (ii = 0; ii < idx.length; ii++) {
        i = idx[ii];
        if (c[i] > 0) continue;
        for (jj = 0; jj < idx.length; jj++) {
          j = idx[jj];
          if (c[j] < 0) continue;
          var den = c[j] - c[i];
          var wi = den <= 1e-12 ? 1 : c[j] / den, wj = den <= 1e-12 ? 0 : -c[i] / den;
          var r = wi * a[i] + wj * a[j];
          if (r < minVmg) continue;
          var perM = (wi * pts[i].power + wj * pts[j].power) / r;
          var key = byEnergy ? [perM, -r] : [-r, perM];
          if (out === null || lessKey(key, out.key)) {
            out = { key: key, i: i, j: j, wi: wi, wj: wj, r: r, perM: perM, ci: c[i], cj: c[j] };
          }
        }
      }
      return out;
    }
    var soar = [], all = [];
    for (k = 0; k < n; k++) { all.push(k); if (pts[k].kind === 'soaring') soar.push(k); }
    var res = best(soar, false);
    if (res) { res.kind = 'soaring'; return res; }
    if (!motorOk) return null;
    res = best(all, true);
    if (res) res.kind = 'motor';
    return res;
  };

  VelocityHull.prototype.pairValue = function (d, i, j, minVmg) {
    if (minVmg === undefined) minVmg = MIN_VMG;
    var pts = this.points;
    var ai = pts[i].vx * d[0] + pts[i].vy * d[1], aj = pts[j].vx * d[0] + pts[j].vy * d[1];
    var ci = d[0] * pts[i].vy - d[1] * pts[i].vx, cj = d[0] * pts[j].vy - d[1] * pts[j].vx;
    if (ci > 0 || cj < 0) return null;
    var den = cj - ci;
    var wi = den <= 1e-12 ? 1 : cj / den, wj = den <= 1e-12 ? 0 : -ci / den;
    var r = wi * ai + wj * aj;
    if (r < minVmg) return null;
    return { r: r, perM: (wi * pts[i].power + wj * pts[j].power) / r, wi: wi, wj: wj };
  };

  VelocityHull.prototype.bestSingle = function (d, minVmg, dist, radius, prefer) {
    if (minVmg === undefined) minVmg = MIN_VMG;
    var pts = this.points, k, a = [], ok = [], miss = {}, i;
    for (k = 0; k < pts.length; k++) {
      a.push(pts[k].vx * d[0] + pts[k].vy * d[1]);
      if (a[k] >= minVmg) ok.push(k);
    }
    if (!ok.length) return null;
    function argmax(list, f) {
      var b = list[0];
      for (var q = 1; q < list.length; q++) if (f(list[q]) > f(b)) b = list[q];
      return b;
    }
    function argminKey(list, f) {
      var b = list[0], kb = f(b);
      for (var q = 1; q < list.length; q++) { var kq = f(list[q]); if (lessKey(kq, kb)) { b = list[q]; kb = kq; } }
      return b;
    }
    var pool = ok, soar, kk;
    if (dist !== undefined && dist !== null && radius !== undefined && radius !== null) {
      for (i = 0; i < ok.length; i++) {
        k = ok[i];
        var sp = Math.max(hypot(pts[k].vx, pts[k].vy), 1e-9);
        miss[k] = dist * Math.abs(d[0] * pts[k].vy - d[1] * pts[k].vx) / sp;
      }
      if (prefer !== undefined && prefer !== null && miss[prefer] !== undefined && miss[prefer] <= radius) return { index: prefer, a: a[prefer] };
      var on = ok.filter(function (q) { return miss[q] <= 0.7 * radius; });
      if (!on.length) {
        kk = argminKey(ok, function (q) { return [miss[q], pts[q].kind !== 'soaring' ? 1 : 0]; });
        return { index: kk, a: a[kk] };
      }
      pool = on;
    }
    var hasPrefer = prefer !== undefined && prefer !== null && pool.indexOf(prefer) >= 0;
    soar = pool.filter(function (q) { return pts[q].kind === 'soaring'; });
    if (soar.length) {
      kk = argmax(soar, function (q) { return a[q]; });
      if (hasPrefer && soar.indexOf(prefer) >= 0 && a[prefer] >= 0.98 * a[kk]) kk = prefer;
      return { index: kk, a: a[kk] };
    }
    kk = argminKey(pool, function (q) { return [pts[q].power / a[q], -a[q]]; });
    if (hasPrefer && pts[prefer].power / a[prefer] <= 1.02 * pts[kk].power / a[kk]) kk = prefer;
    return { index: kk, a: a[kk] };
  };

  VelocityHull.prototype.rayPoint = function (d) {   // soaring boundary point along d, or null
    var pl = this.planPair(d, MIN_VMG, false);
    return pl ? pl.r : null;
  };

  VelocityHull.prototype.support = function (d) {
    var m = NaN;
    this.points.forEach(function (p) {
      if (p.kind !== 'soaring') return;
      var v = p.vx * d[0] + p.vy * d[1];
      if (!(v <= m)) m = v;
    });
    return m;
  };

  // hulls of one aircraft at each library wind
  function HullSource(entries) {
    var winds = [], i;
    entries.forEach(function (e) { if (winds.indexOf(e.u10) < 0) winds.push(e.u10); });
    winds.sort(function (p, q) { return p - q; });
    this.hulls = [];
    for (i = 0; i < winds.length; i++) {
      var pts = pointsFromEntries(entries, winds[i]);
      if (pts.length) this.hulls.push(new VelocityHull(pts, winds[i]));
    }
  }
  HullSource.prototype.at = function (u10) {
    var best = null;
    this.hulls.forEach(function (h) { if (best === null || Math.abs(h.u10 - u10) < Math.abs(best.u10 - u10)) best = h; });
    return best;
  };
  // build a source straight from points (parity tests): hulls = [{u10, points: [...]}]
  HullSource.fromPoints = function (hulls) {
    var s = Object.create(HullSource.prototype);
    s.hulls = hulls.map(function (h) { return new VelocityHull(h.points, h.u10); });
    s.hulls.sort(function (p, q) { return p.u10 - q.u10; });
    return s;
  };

  function toWind(psi, v) {
    var c = Math.cos(psi), s = Math.sin(psi);
    return [c * v[0] + s * v[1], -s * v[0] + c * v[1]];
  }
  function toWorld(psi, v) {
    var c = Math.cos(psi), s = Math.sin(psi);
    return [c * v[0] - s * v[1], s * v[0] + c * v[1]];
  }
  function psiOf(windFrom) { return (-90 - windFrom) * Math.PI / 180; }

  // Steady motor leg (see STEADY_NOTES in sim/navigator.py): level flight at 10 m aimed by the wind triangle
  // at the airspeed of least electric energy per metre made good. p: plain numbers, camelCase.
  function SteadyModel(p) {
    this.mass = p.mass; this.wingArea = p.wingArea; this.cd0 = p.cd0; this.kInduced = p.kInduced;
    this.rho = p.rho; this.clMax = p.clMax; this.vStall = p.vStall; this.vBestLd = p.vBestLd;
    this.tMax = p.tMax; this.eta = p.eta;
    this.g = p.g !== undefined ? p.g : 9.81;
    this.vFactor = p.vFactor !== undefined ? p.vFactor : 1.2;
    this.height = p.height !== undefined ? p.height : 10;
    this.cap = p.cap !== undefined ? p.cap : 3;
    this.thrustFrac = p.thrustFrac !== undefined ? p.thrustFrac : 0.8;
    this.lookahead = p.lookahead !== undefined ? p.lookahead : 400;
    this.maxTurnDeg = p.maxTurnDeg !== undefined ? p.maxTurnDeg : 35;
    this.vStep = p.vStep !== undefined ? p.vStep : 0.25;
  }
  SteadyModel.prototype.level = function (V) {
    var q = 0.5 * this.rho * V * V * this.wingArea, cl = this.mass * this.g / q;
    var T = q * (this.cd0 + this.kInduced * cl * cl);
    return { cl: cl, thrust: T, power: T * V / this.eta };
  };
  SteadyModel.prototype.leg = function (uz, tau, minVmg) {
    if (minVmg === undefined) minVmg = MIN_VMG;
    var n = [-tau[1], tau[0]], wn = uz * n[0], wt = uz * tau[0];
    var v0 = this.vFactor * this.vStall, v1 = this.cap * this.vBestLd, best = null, k = 0;
    for (;;) {
      var V = v0 + k * this.vStep;
      k++;
      if (V > v1 + 1e-9) break;
      if (Math.abs(wn) >= V) continue;
      var lv = this.level(V);
      if (lv.thrust > this.thrustFrac * this.tMax) continue;
      var sb = -wn / V, cb = Math.sqrt(1 - sb * sb), along = V * cb + wt;
      if (along < minVmg) continue;
      var perM = lv.power / along;
      if (best === null || perM < best.perM - 1e-12) {
        best = { V: V, cl: lv.cl, thrust: lv.thrust, power: lv.power, along: along, perM: perM,
          heading: [tau[0] * cb + n[0] * sb, tau[1] * cb + n[1] * sb], ground: [along * tau[0], along * tau[1]] };
      }
    }
    return best;
  };
  SteadyModel.prototype.track = function (dl, e) {
    var lim = this.maxTurnDeg * Math.PI / 180;
    var phi = -Math.max(-lim, Math.min(lim, Math.atan(e / this.lookahead)));
    var c = Math.cos(phi), s = Math.sin(phi);
    return [dl[0] * c - dl[1] * s, dl[0] * s + dl[1] * c];
  };

  function HullNavigator(source, opt) {
    opt = opt || {};
    this.source = source;
    this.tackCostS = opt.tackCostS !== undefined ? opt.tackCostS : 30;
    this.hysteresis = opt.hysteresis !== undefined ? opt.hysteresis : 0.05;
    this.minLegS = opt.minLegS !== undefined ? opt.minLegS : 90;
    this.arrivalRadius = opt.arrivalRadius !== undefined ? opt.arrivalRadius : 50;
    this.approachFactor = opt.approachFactor !== undefined ? opt.approachFactor : 5;
    this.minVmg = opt.minVmg !== undefined ? opt.minVmg : MIN_VMG;
    this.maxTackLoss = opt.maxTackLoss !== undefined ? opt.maxTackLoss : 0.1;
    this.maxCorridorFrac = opt.maxCorridorFrac !== undefined ? opt.maxCorridorFrac : 0.2;
    this.funnelFrac = opt.funnelFrac !== undefined ? opt.funnelFrac : 0.3;
    this.steady = opt.steady || null;   // a SteadyModel: motor legs are steady flights
    this.reset();
  }
  HullNavigator.prototype.reset = function () {
    this.mode = null; this.line0 = null; this.target = null; this.pair = null; this.leg = null;
    this.lastSwitchT = -Infinity; this.corridor = 0; this.corridorFull = 0;
  };
  HullNavigator.prototype._steadyCommand = function (psi, tau, dist, e, E, t) {
    var lg = this.steady.leg(this._uz, tau, this.minVmg);
    if (lg === null) return null;
    var pt = { vx: lg.ground[0], vy: lg.ground[1], power: lg.power, kind: 'motor', key: 'steady', entry: null, side: 1,
      steady: { V: lg.V, cl: lg.cl, thrust: lg.thrust, heading: lg.heading, height: this.steady.height } };
    var tacked = this.leg !== null && this.leg !== 'steady';
    if (tacked) this.lastSwitchT = t;
    this.leg = 'steady'; this.mode = 'direct'; this.pair = null;
    return this._command(psi, pt, 'direct', lg.along, tacked, dist, lg.perM, e, E);
  };
  HullNavigator.prototype._approach = function (t, psi, hull, pos, target, dist, e, E) {
    var pts = hull.points, k;
    var dn = toWind(psi, [target[0] - pos[0], target[1] - pos[1]]);
    dn = [dn[0] / dist, dn[1] / dist];
    var pk = -1;
    for (k = 0; k < pts.length; k++) if (pts[k].key === this.leg) pk = k;
    var sg = hull.bestSingle(dn, this.minVmg, dist, this.arrivalRadius, pk >= 0 ? pk : null);
    this.line0 = [pos[0], pos[1]]; this.pair = null;
    if (this.steady !== null) {
      var off = sg !== null && dist * Math.abs(dn[0] * pts[sg.index].vy - dn[1] * pts[sg.index].vx) /
        Math.max(hypot(pts[sg.index].vx, pts[sg.index].vy), 1e-9) > 0.7 * this.arrivalRadius;
      var soaringLeg = this.leg !== null && this.leg !== 'steady';
      if (sg === null || pts[sg.index].kind === 'motor' || this.leg === 'steady' || (off && !soaringLeg)) {
        var rs = this._steadyCommand(psi, dn, dist, e, E, t);
        if (rs !== null) return rs;
      }
    }
    if (sg === null) {
      this.mode = 'no progress'; this.leg = null;
      var bp = pts[0], bv = -Infinity;
      for (k = 0; k < pts.length; k++) {
        var v = pts[k].vx * dn[0] + pts[k].vy * dn[1];
        if (v > bv) { bv = v; bp = pts[k]; }
      }
      return this._command(psi, bp, 'no progress', 0, false, dist, 0, e, E);
    }
    var pt = pts[sg.index];
    var tk = this.leg !== null && pt.key !== this.leg;
    if (tk) this.lastSwitchT = t;
    this.leg = pt.key; this.mode = 'direct';
    return this._command(psi, pt, 'direct', sg.a, tk, dist, pt.power / sg.a, e, E);
  };
  HullNavigator.prototype._limits = function (hull, dl, sRem, E, plan) {
    if (plan.kind !== 'soaring' || E <= 0) return [-E, E];
    var nl = [-dl[1], dl[0]], self = this;
    function ok(ep) {
      var vx = sRem * dl[0] - ep * nl[0], vy = sRem * dl[1] - ep * nl[1];
      var n = hypot(vx, vy);
      var pl = hull.planPair([vx / n, vy / n], self.minVmg, false);
      return pl !== null && pl.r >= LIMIT_FRAC * plan.r;
    }
    var out = [], sgs = [-1, 1], q, k;
    for (q = 0; q < 2; q++) {
      var lim = 0;
      for (k = 1; k <= 10; k++) {
        var ep = E * k / 10;
        if (!ok(sgs[q] * ep)) break;
        lim = ep;
      }
      out.push(sgs[q] * lim);
    }
    return out;
  };
  HullNavigator.prototype._command = function (psi, pt, mode, vmg, tacked, dist, perM, cross, corridor) {
    var ang = psi + Math.atan2(pt.vy, pt.vx);
    var bearing = (((90 - ang * 180 / Math.PI) % 360) + 360) % 360;
    return { bearing: bearing, chi: Math.abs(Math.atan2(pt.vy, pt.vx) * 180 / Math.PI),
      side: pt.vy > 0 ? 'port' : 'starboard', mode: mode, vmg: vmg, tacked: tacked, distance: dist,
      point: pt, kind: pt.kind, perM: perM, cross: cross, corridor: corridor };
  };
  HullNavigator.prototype.step = function (t, pos, u10, windFrom, target) {
    var psi = psiOf(windFrom), hull = this.source.at(u10), pts = hull.points, k;
    this._uz = u10;   // the wind at the steady height (10 m, the reference height)
    var byk = {};
    for (k = 0; k < pts.length; k++) byk[pts[k].key] = k;
    var dist = hypot(target[0] - pos[0], target[1] - pos[1]);
    if (dist <= this.arrivalRadius) {
      this.mode = 'arrived';
      var p0 = this.leg !== null && byk[this.leg] !== undefined ? pts[byk[this.leg]] : pts[0];
      return this._command(psi, p0, 'arrived', 0, false, dist, 0, 0, this.corridor);
    }
    if (this.target === null || this.line0 === null ||
        hypot(target[0] - this.target[0], target[1] - this.target[1]) > 1) {
      this.line0 = [pos[0], pos[1]]; this.target = [target[0], target[1]];
      this.pair = null; this.leg = null;
    }
    var dl = toWind(psi, [target[0] - this.line0[0], target[1] - this.line0[1]]);
    var nl = hypot(dl[0], dl[1]);
    dl = [dl[0] / nl, dl[1] / nl];
    var pw = toWind(psi, [pos[0] - this.line0[0], pos[1] - this.line0[1]]);
    var e = dl[0] * pw[1] - dl[1] * pw[0];
    if (this.corridorFull > 0 && Math.abs(e) > 2 * Math.min(this.corridorFull, this.funnelFrac * dist)) {
      this.line0 = [pos[0], pos[1]]; this.pair = null; this.leg = null;
      dl = toWind(psi, [target[0] - pos[0], target[1] - pos[1]]);
      dl = [dl[0] / dist, dl[1] / dist];
      e = 0;
    }
    if (dist <= this.approachFactor * this.arrivalRadius) {
      return this._approach(t, psi, hull, pos, target, dist, e, this.corridor);
    }
    var plan = hull.planPair(dl, this.minVmg);
    if (this.steady !== null && (plan === null || plan.kind === 'motor')) {
      var rs2 = this._steadyCommand(psi, this.steady.track(dl, e), dist, e, 0, t);
      if (rs2 !== null) { this.corridor = 0; this.corridorFull = 0; return rs2; }
    }
    if (plan === null) {
      this.mode = 'no progress'; this.pair = null; this.leg = null;
      var bp = pts[0], bv = -Infinity;
      for (k = 0; k < pts.length; k++) {
        var v = pts[k].vx * dl[0] + pts[k].vy * dl[1];
        if (v > bv) { bv = v; bp = pts[k]; }
      }
      return this._command(psi, bp, 'no progress', 0, false, dist, 0, e, 0);
    }
    if (this.pair !== null && byk[this.pair[0]] !== undefined && byk[this.pair[1]] !== undefined) {
      var io = byk[this.pair[0]], jo = byk[this.pair[1]];
      var old = hull.pairValue(dl, io, jo, this.minVmg);
      if (old !== null) {
        var same = pts[io].kind === 'soaring' && pts[jo].kind === 'soaring';
        if ((plan.kind === 'soaring' && same && old.r >= (1 - this.hysteresis) * plan.r) ||
            (plan.kind === 'motor' && old.perM <= (1 + this.hysteresis) * plan.perM)) {
          plan.i = io; plan.j = jo; plan.r = old.r; plan.perM = old.perM; plan.wi = old.wi; plan.wj = old.wj;
          plan.ci = dl[0] * pts[io].vy - dl[1] * pts[io].vx;
          plan.cj = dl[0] * pts[jo].vy - dl[1] * pts[jo].vx;
        }
      }
    }
    var A = pts[plan.i], B = pts[plan.j];
    var cmax = Math.max(Math.abs(plan.ci), Math.abs(plan.cj));
    var legS = Math.max(this.minLegS, this.tackCostS / this.maxTackLoss);
    var Efull = Math.min(legS * cmax / 2, this.maxCorridorFrac * nl);
    var E = Math.min(Efull, this.funnelFrac * dist);
    this.corridor = E; this.corridorFull = Efull;
    var tw = toWind(psi, [target[0] - pos[0], target[1] - pos[1]]);
    var lim = this._limits(hull, dl, tw[0] * dl[0] + tw[1] * dl[1], E, plan);
    if (cmax < CROSS_ON_LINE) return this._approach(t, psi, hull, pos, target, dist, e, E);
    this.pair = [A.key, B.key];
    var prev = this.leg, cur;
    if (prev !== A.key && prev !== B.key) {
      cur = (A.vx * dl[0] + A.vy * dl[1]) >= (B.vx * dl[0] + B.vy * dl[1]) ? A : B;
    } else {
      cur = prev === A.key ? A : B;
      if (t - this.lastSwitchT >= this.minLegS || E < 0.999 * Efull) {
        if (cur === B && e >= lim[1]) cur = A;
        else if (cur === A && e <= lim[0]) cur = B;
      }
    }
    var tacked = prev !== null && cur.key !== prev;
    if (tacked) this.lastSwitchT = t;
    this.leg = cur.key;
    var mode = dl[0] < 0 ? 'upwind tacking' : 'downwind gybing';
    this.mode = mode;
    return this._command(psi, cur, mode, plan.r, tacked, dist, plan.perM, e, E);
  };

  // SteadyModel parameters from an aircraft made by MPC.makeAircraft and MPC.MOTOR (viewer use)
  function steadyParams(ac, motor, g) {
    if (g === undefined) g = 9.81;   // sim.aircraft.G
    return { mass: ac.mass, wingArea: ac.wing_area, cd0: ac.cd0, kInduced: ac.k_induced, rho: ac.rho,
      clMax: ac.cl_max, vStall: ac.v_stall, vBestLd: Math.sqrt(2 * ac.mass * g / (ac.rho * ac.wing_area * ac.cl_best_ld)),
      tMax: motor.thrustMaxPerWeight * ac.mass * g, eta: motor.propEff * motor.motorEff, g: g };
  }
  // library-entry dict of a steady leg (sim.mpc.steady_entry): the planner's reference
  var STEADY_PERIOD_S = 8;
  function steadyEntry(pt) {
    var st = pt.steady, n = 9, t = [], x = [], y = [], z = [], V = [], g = [], psi = [], cl = [], mu = [], th = [], i;
    var ps = Math.atan2(st.heading[1], st.heading[0]);
    for (i = 0; i < n; i++) {
      var tt = STEADY_PERIOD_S * i / (n - 1);
      t.push(tt); x.push(pt.vx * tt); y.push(pt.vy * tt); z.push(st.height); V.push(st.V); g.push(0);
      psi.push(ps); cl.push(st.cl); mu.push(0); th.push(st.thrust);
    }
    return { aircraft: 'steady', u10: 0, chi_deg: Math.round(Math.abs(ps * 180 / Math.PI) * 10) / 10, margin_set: 'steady', steady: true, success: true, kind: 'motor', period: STEADY_PERIOD_S,
      t: t, x: x, y: y, z: z, V: V, gamma: g, psi: psi, cl: cl, mu: mu, thrust: th, power_avg_w: pt.power };
  }

  // kinematic run, as sail_hull
  function sailHull(source, windFn, start, target, nav, o) {
    o = o || {};
    var dt = o.dt || 1, navEvery = o.navEveryS || 5, tMax = o.tMax || 12 * 3600;
    var pc = o.physicalTackCostS !== undefined ? o.physicalTackCostS : nav.tackCostS;
    nav.reset();
    var pos = [start[0], start[1]], d0 = hypot(target[0] - pos[0], target[1] - pos[1]);
    var track = [[pos[0], pos[1]]], sw = [], modes = [];
    var t = 0, pauseUntil = -1, nextCall = 0, cmd = null, energy = 0, arrived = false;
    while (t < tMax) {
      var w = windFn(t);
      if (t >= nextCall) {
        cmd = nav.step(t, pos, w[0], w[1], target);
        nextCall = t + (cmd.mode === 'direct' ? 1 : navEvery);
        modes.push([t, cmd.mode]);
        if (cmd.mode === 'arrived') { arrived = true; break; }
        if (cmd.tacked) { sw.push(t); pauseUntil = t + pc; }
      }
      if (hypot(target[0] - pos[0], target[1] - pos[1]) <= nav.arrivalRadius) { arrived = true; break; }
      if (t >= pauseUntil && cmd.mode !== 'no progress') {
        var v = toWorld(psiOf(w[1]), [cmd.point.vx, cmd.point.vy]);
        pos = [pos[0] + dt * v[0], pos[1] + dt * v[1]];
        energy += cmd.point.power * dt;
      }
      t += dt;
      track.push([pos[0], pos[1]]);
    }
    var d1 = hypot(target[0] - pos[0], target[1] - pos[1]);
    return { track: track, modes: modes, nLegs: sw.length, switchTimes: sw, time: arrived ? t : NaN,
      arrived: arrived, meanVmg: t > 0 ? (d0 - d1) / t : 0, energyJ: energy, distance: d0 };
  }

  var NAV = { MIN_VMG: MIN_VMG, VelocityHull: VelocityHull, HullSource: HullSource,
    HullNavigator: HullNavigator, SteadyModel: SteadyModel, steadyParams: steadyParams, steadyEntry: steadyEntry, sailHull: sailHull, pointsFromEntries: pointsFromEntries,
    toWind: toWind, toWorld: toWorld, psiOf: psiOf };
  globalThis.NAV = NAV;
  if (typeof module !== 'undefined' && module.exports) module.exports = NAV;
})();
