// Strategic layer for the viewer: a JavaScript mirror of the route planner in sim/route.py (read its module notes
// first). The earliest arrival from a start to a destination over a world sea, under the owner's energy rules (D-021):
// an hourly battery (solar yield less the avionics with the sensor set, the motor and the take-offs) kept within the
// usable capacity and above the reserve, the next night carried from any moment, and the motor's daily allowance
// (a share of the solar energy the route collects that day). Nodes have latitude and longitude, legs are
// great-circle arcs flown in sub-steps, the field is pluggable (CorridorField for the corridor files), legs that
// touch land are left out, waiting on the water is an edge.
//
// Needs navigator.js (NAV: the velocity hull and the steady leg) and worlddata.js (WorldData: decoding and the
// corridor's projection). Parameters come from route_params.js (python -m sim.route export); solar layers from
// sim/viewer/route/solar_<id>.js. Attaches ROUTE to globalThis; module.exports under node. Parity with Python is
// tested in sim/tests/test_route_js.py.
(function (root) {
  'use strict';
  var NAV = root.NAV || (typeof require !== 'undefined' ? require('./navigator.js') : null);
  var WD = root.WorldData || (typeof require !== 'undefined' ? require('./worlddata.js') : null);
  var R_EARTH_KM = 6371.0088;
  var D2R = Math.PI / 180;

  var DEFAULTS = { refine: 3, rings: 2, sub_km: 10, wait_s: 3600, lookahead_h: 36, u_q: 0.1, a_q: 1,
    eps_soc: 0.25, eps_slack: 0.25, min_vmg: 0.3 };
  var RELAX = [['reserve_minus_1.5_capacity', { below: 1.5 }], ['reserve_minus_3_capacity_no_allowance', { below: 3, share: Infinity }]];
  var REASONS = { 0: 'ok', 1: 'the start lies outside the field, on land or in seas above the avoidance height',
    2: 'the battery cannot carry the coming night from the start',
    3: 'the night drains the battery below the reserve on every route found',
    4: "the route's draw takes the battery below the reserve",
    5: "the motor allowance (its share of the day's solar generation) does not cover the motor legs needed",
    6: "no route reaches the destination within the field's time (seas above the avoidance height, or too slow)",
    7: "only the relaxed search found a route, and it keeps the rules (the strict search's approximations missed it)" };

  function settingsOf(s) { var o = {}, k; for (k in DEFAULTS) o[k] = (s && s[k] !== undefined) ? s[k] : DEFAULTS[k]; return o; }

  // ------------------------------------------------------------ sun and solar yield (NOAA formulas, as sim.route)
  function yearDay(t) {
    var days = Math.floor(t / 86400), hour = (t - days * 86400) / 3600;
    var y = new Date(days * 86400000).getUTCFullYear();
    var n = (y % 4 === 0 && (y % 100 !== 0 || y % 400 === 0)) ? 366 : 365;
    var doy = days - Date.UTC(y, 0, 1) / 86400000 + 1;
    return [n, doy, hour];
  }
  function sunSinElev(lat, lon, t) {
    var yd = yearDay(t), n = yd[0], doy = yd[1], hour = yd[2];
    var g = 2.0 * Math.PI / n * (doy - 1 + (hour - 12.0) / 24.0);
    var eqt = 229.18 * (0.000075 + 0.001868 * Math.cos(g) - 0.032077 * Math.sin(g) - 0.014615 * Math.cos(2 * g)
      - 0.040849 * Math.sin(2 * g));
    var dec = (0.006918 - 0.399912 * Math.cos(g) + 0.070257 * Math.sin(g) - 0.006758 * Math.cos(2 * g)
      + 0.000907 * Math.sin(2 * g) - 0.002697 * Math.cos(3 * g) + 0.00148 * Math.sin(3 * g));
    var tst = hour * 60.0 + eqt + 4.0 * lon;
    var ha = (tst / 4.0 - 180.0) * D2R, p = lat * D2R;
    return Math.sin(p) * Math.sin(dec) + Math.cos(p) * Math.cos(dec) * Math.cos(ha);
  }
  function modelHourMean(lat, lon, tHour, fit) {
    var s = 0, q = [0.125, 0.375, 0.625, 0.875], i;
    for (i = 0; i < 4; i++) s += Math.max(0.0, sunSinElev(lat, lon, tHour + q[i] * 3600.0));
    return fit * s / 4.0;
  }

  // ------------------------------------------------------------ interpolation and the fields
  function bilinear(grid, off, n0, n1, fi, fj) {
    fi = Math.min(Math.max(fi, 0.0), n0 - 1.0); fj = Math.min(Math.max(fj, 0.0), n1 - 1.0);
    var i = n0 > 1 ? Math.min(Math.floor(fi), n0 - 2) : 0, j = n1 > 1 ? Math.min(Math.floor(fj), n1 - 2) : 0;
    var a = fi - i, b = fj - j, tot = 0.0, ws = 0.0;
    var cs = [[0, 0, (1 - a) * (1 - b)], [1, 0, a * (1 - b)], [0, 1, (1 - a) * b], [1, 1, a * b]], k, v, w;
    for (k = 0; k < 4; k++) {
      v = grid[off + Math.min(i + cs[k][0], n0 - 1) * n1 + Math.min(j + cs[k][1], n1 - 1)]; w = cs[k][2];
      if (w > 0 && Number.isFinite(v)) { tot += w * v; ws += w; }
    }
    if (ws > 1e-12) return tot / ws;
    var s = 0, m = 0;
    for (k = 0; k < 4; k++) {
      v = grid[off + Math.min(i + cs[k][0], n0 - 1) * n1 + Math.min(j + cs[k][1], n1 - 1)];
      if (Number.isFinite(v)) { s += v; m++; }
    }
    return m ? s / m : NaN;
  }

  // the corridor of a world file: f = WorldData.unpack(file.corridor)
  function CorridorField(f) {
    this.f = f; this.na = f.na; this.nc = f.nc; this.nt = f.nt; this.np = f.na * f.nc;
    this.t0 = f.t0; this.dt = 3600.0 * f.dt_h; this.tEnd = this.t0 + this.dt * (this.nt - 1);
    var u = f.vals.u10, v = f.vals.v10, mx = -Infinity, i;
    for (i = 0; i < u.length; i++) { var s = Math.sqrt(u[i] * u[i] + v[i] * v[i]); if (s > mx) mx = s; }
    this.u10Max = mx;
  }
  // the corners of bilinear() at (fi, fj) with a positive weight, [[flat index, weight], ...] (sim.route.corners)
  function corners(n0, n1, fi, fj) {
    fi = Math.min(Math.max(fi, 0.0), n0 - 1.0); fj = Math.min(Math.max(fj, 0.0), n1 - 1.0);
    var i = n0 > 1 ? Math.min(Math.floor(fi), n0 - 2) : 0, j = n1 > 1 ? Math.min(Math.floor(fj), n1 - 2) : 0;
    var a = fi - i, b = fj - j, out = [], k;
    var cs = [[0, 0, (1 - a) * (1 - b)], [1, 0, a * (1 - b)], [0, 1, (1 - a) * b], [1, 1, a * b]];
    for (k = 0; k < 4; k++) if (cs[k][2] > 0) out.push([Math.min(i + cs[k][0], n0 - 1) * n1 + Math.min(j + cs[k][1], n1 - 1), cs[k][2]]);
    return out;
  }
  CorridorField.prototype.locate = function (lat, lon) {
    var ij = WD.corridorIndex(this.f, lat, lon), fi = ij[0], fj = ij[1];
    if (fi >= -1e-9 && fi <= this.na - 1 + 1e-9 && fj >= -1e-9 && fj <= this.nc - 1 + 1e-9) return [fi, fj, corners(this.na, this.nc, fi, fj)];
    return null;
  };
  CorridorField.prototype.at = function (g, off, loc) {
    var tot = 0.0, ws = 0.0, c = loc[2], k;
    for (k = 0; k < c.length; k++) { var v = g[off + c[k][0]]; if (Number.isFinite(v)) { tot += c[k][1] * v; ws += c[k][1]; } }
    return ws > 1e-12 ? tot / ws : bilinear(g, off, this.na, this.nc, loc[0], loc[1]);
  };
  CorridorField.prototype.sample = function (loc, t) {
    var x = Math.min(Math.max((t - this.t0) / this.dt, 0.0), this.nt - 1.0);
    var k = this.nt > 1 ? Math.min(Math.floor(x), this.nt - 2) : 0, ft = x - k, out = [], names = ['u10', 'v10', 'hs'], q;
    for (q = 0; q < 3; q++) {
      var g = this.f.vals[names[q]], a = this.at(g, k * this.np, loc);
      if (this.nt === 1 || ft === 0) { out.push(a); continue; }
      var b = this.at(g, (k + 1) * this.np, loc);
      out.push(!Number.isFinite(a) ? b : !Number.isFinite(b) ? a : a + (b - a) * ft);
    }
    return out;
  };

  // a solar layer of python -m sim.route solar (the JSON object of its file)
  function SolarLayer(d) {
    this.d = d; this.na = d.na; this.nc = d.nc; this.nt = d.nt; this.t0 = d.t0;
    var raw = WD.unb64(d.data), i;
    this.v = new Float64Array(raw.length);
    for (i = 0; i < raw.length; i++) this.v[i] = raw[i] === 255 ? NaN : raw[i] * d.q.step;
  }
  SolarLayer.prototype.locate = function (lat, lon) { return WD.corridorIndex(this.d, lat, lon); };
  SolarLayer.prototype.hourMean = function (loc, tHour) {
    var k = Math.round((tHour - this.t0) / 3600.0);
    if (k < 0 || k >= this.nt) return null;
    var v = bilinear(this.v, k * this.na * this.nc, this.na, this.nc, loc[0], loc[1]);
    return Number.isFinite(v) ? v : null;
  };

  // ------------------------------------------------------------ great-circle geometry
  function gcKm(lat1, lon1, lat2, lon2) {
    var p1 = lat1 * D2R, p2 = lat2 * D2R;
    var a = Math.pow(Math.sin((p2 - p1) / 2), 2) + Math.cos(p1) * Math.cos(p2) * Math.pow(Math.sin((lon2 - lon1) * D2R / 2), 2);
    return 2 * R_EARTH_KM * Math.asin(Math.min(1.0, Math.sqrt(a)));
  }
  function gcPoint(lat1, lon1, lat2, lon2, f) {
    var p1 = lat1 * D2R, l1 = lon1 * D2R, p2 = lat2 * D2R, l2 = lon2 * D2R;
    var a = [Math.cos(p1) * Math.cos(l1), Math.cos(p1) * Math.sin(l1), Math.sin(p1)];
    var b = [Math.cos(p2) * Math.cos(l2), Math.cos(p2) * Math.sin(l2), Math.sin(p2)];
    var dot = Math.min(1.0, Math.max(-1.0, a[0] * b[0] + a[1] * b[1] + a[2] * b[2]));
    var om = Math.acos(dot);
    if (om < 1e-12) return [lat1, lon1];
    var sa = Math.sin((1 - f) * om) / Math.sin(om), sb = Math.sin(f * om) / Math.sin(om);
    var x = sa * a[0] + sb * b[0], y = sa * a[1] + sb * b[1], z = sa * a[2] + sb * b[2];
    return [Math.atan2(z, Math.sqrt(x * x + y * y)) / D2R, Math.atan2(y, x) / D2R];
  }
  function wrap360(x) { var r = x % 360.0; return r < 0 ? r + 360.0 : r; }
  function wrap180(x) { return wrap360(x + 180.0) - 180.0; }
  function gcBearing(lat1, lon1, lat2, lon2) {
    var p1 = lat1 * D2R, p2 = lat2 * D2R, dl = (lon2 - lon1) * D2R;
    var y = Math.sin(dl) * Math.cos(p2), x = Math.cos(p1) * Math.sin(p2) - Math.sin(p1) * Math.cos(p2) * Math.cos(dl);
    return wrap360(Math.atan2(y, x) / D2R);
  }
  function ringOffsets(rings) {
    function gcd(a, b) { while (b) { var t = a % b; a = b; b = t; } return a; }
    var out = [], di, dj;
    for (di = -rings; di <= rings; di++) for (dj = -rings; dj <= rings; dj++) {
      if ((di !== 0 || dj !== 0) && gcd(Math.abs(di), Math.abs(dj)) === 1) out.push([di, dj]);
    }
    return out;
  }
  // even-odd test against the land rings ([[lon, lat], ...], WorldData.decodeLand), as sim.world.on_land
  function landTester(rings) {
    var box = rings.map(function (r) {
      var x0 = Infinity, x1 = -Infinity, y0 = Infinity, y1 = -Infinity;
      r.forEach(function (p) { if (p[0] < x0) x0 = p[0]; if (p[0] > x1) x1 = p[0]; if (p[1] < y0) y0 = p[1]; if (p[1] > y1) y1 = p[1]; });
      return [x0, x1, y0, y1];
    });
    return function (lat, lon) {
      var inside = false, k, i;
      for (k = 0; k < rings.length; k++) {
        var bx = box[k];
        if (!(lon >= bx[0] && lon <= bx[1] && lat >= bx[2] && lat <= bx[3])) continue;
        var r = rings[k], n = r.length, c = 0;
        for (i = 0; i < n; i++) {
          var x = r[i][0], y = r[i][1], x2 = r[(i + 1) % n][0], y2 = r[(i + 1) % n][1];
          if (((y > lat) !== (y2 > lat)) && (lon < (x2 - x) * (lat - y) / (y2 === y ? 1e-30 : y2 - y) + x)) c++;
        }
        if (c % 2) inside = !inside;
      }
      return inside;
    };
  }

  // the graph of a corridor (sim.route.corridor_graph); land: rings or null for none
  function corridorGraph(field, st, land) {
    st = settingsOf(st);
    var f = field.f, na = f.na, nc = f.nc, step = f.step_km, refine = st.refine;
    var NA = (na - 1) * refine + 1, NC = (nc - 1) * refine + 1, h = step / refine;
    var r = f.bearing_deg * D2R, ax = Math.sin(r), ay = Math.cos(r), cx = Math.cos(r), cy = -Math.sin(r);
    var lat = [], lon = [], loc = [], i, j, n;
    var onLand = land && land.length ? landTester(land) : null;
    for (i = 0; i < NA; i++) for (j = 0; j < NC; j++) {
      var s = (i - (NA - 1) / 2.0) * h, q = (j - (NC - 1) / 2.0) * h;
      var p = WD.aeqdInv(f.lat, f.lon, s * ax + q * cx, s * ay + q * cy);
      lat.push(p[0]); lon.push(p[1]);
    }
    for (n = 0; n < lat.length; n++) loc.push(onLand && onLand(lat[n], lon[n]) ? null : field.locate(lat[n], lon[n]));
    var offs = ringOffsets(st.rings), edges = lat.map(function () { return []; }), o, k;
    for (i = 0; i < NA; i++) for (j = 0; j < NC; j++) {
      n = i * NC + j;
      if (loc[n] === null) continue;
      for (o = 0; o < offs.length; o++) {
        var ii = i + offs[o][0], jj = j + offs[o][1];
        if (ii < 0 || ii >= NA || jj < 0 || jj >= NC) continue;
        var m = ii * NC + jj;
        if (loc[m] === null) continue;
        var L = gcKm(lat[n], lon[n], lat[m], lon[m]), ns = Math.max(1, Math.ceil(L / st.sub_km - 1e-9));
        var pts = [], bad = false;
        for (k = 0; k < ns; k++) pts.push(gcPoint(lat[n], lon[n], lat[m], lon[m], k / ns));
        if (onLand) {
          for (k = 1; k < ns && !bad; k++) if (onLand(pts[k][0], pts[k][1])) bad = true;
          for (k = 0; k < ns && !bad; k++) { var c = gcPoint(lat[n], lon[n], lat[m], lon[m], (k + 0.5) / ns); if (onLand(c[0], c[1])) bad = true; }
        }
        if (bad) continue;
        var steps = [];
        for (k = 0; k < ns; k++) {
          var lc = field.locate(pts[k][0], pts[k][1]);
          if (lc === null) { bad = true; break; }
          steps.push([lc, gcBearing(pts[k][0], pts[k][1], lat[m], lon[m])]);
        }
        if (!bad) edges[n].push([m, L * 1000.0 / ns, steps]);
      }
    }
    return { lat: lat, lon: lon, loc: loc, edges: edges, stepKm: h };
  }
  function nearest(g, lat, lon) {
    var best = null, bd = Infinity, n;
    for (n = 0; n < g.lat.length; n++) {
      if (g.loc[n] === null) continue;
      var d = gcKm(lat, lon, g.lat[n], g.lon[n]);
      if (d < bd) { best = n; bd = d; }
    }
    return [best, bd];
  }

  // ------------------------------------------------------------ leg model
  function LegModel(hulls, steady, st) {
    st = settingsOf(st);
    this.st = st;
    this.source = NAV.HullSource.fromPoints(hulls);
    this.steady = new NAV.SteadyModel({ mass: steady.mass, wingArea: steady.wing_area, cd0: steady.cd0, kInduced: steady.k_induced,
      rho: steady.rho, clMax: steady.cl_max, vStall: steady.v_stall, vBestLd: steady.v_best_ld, tMax: steady.t_max, eta: steady.eta, g: steady.g });
    this.cache = new Map();
  }
  LegModel.prototype.leg = function (u10, relDeg) {
    var iu = Math.floor(u10 / this.st.u_q + 0.5), ia = Math.floor(Math.abs(wrap180(relDeg)) / this.st.a_q + 0.5);
    var key = iu * 1000 + ia, r = this.cache.get(key);
    if (r === undefined) {
      var uq = iu * this.st.u_q, aq = ia * this.st.a_q * D2R, d = [Math.cos(aq), Math.sin(aq)];
      var pl = this.source.at(uq).planPair(d, this.st.min_vmg, false);
      if (pl !== null) r = ['soaring', pl.r, 0.0];
      else {
        var lg = this.steady.leg(uq, d, this.st.min_vmg);
        r = lg !== null ? ['motor', lg.along, lg.power] : ['none', 0.0, 0.0];
      }
      this.cache.set(key, r);
    }
    return r;
  };
  LegModel.prototype.vMax = function (u10Max) {
    var v = 0;
    this.source.hulls.forEach(function (h) { h.points.forEach(function (p) { var s = Math.sqrt(p.vx * p.vx + p.vy * p.vy); if (s > v) v = s; }); });
    for (var u = 0.0; u <= u10Max + 1.0; u += 0.5) {
      var lg = this.steady.leg(u, [1.0, 0.0], this.st.min_vmg);
      if (lg !== null && lg.along > v) v = lg.along;
    }
    return 1.05 * v + 1.0;
  };

  // ------------------------------------------------------------ battery tables (sim.route.Tables)
  function localDay(t, lon) { return Math.floor((t + lon * 240.0) / 86400.0); }

  function Tables(graph, field, en, layer, opt) {
    opt = opt || {};
    var st = settingsOf(opt.settings);
    this.T0 = opt.tStart === undefined ? Math.floor(field.t0 / 3600.0) * 3600.0 - 86400.0 : opt.tStart;
    this.K = opt.hours === undefined ? Math.ceil((field.tEnd - this.T0) / 3600.0) + st.lookahead_h : opt.hours;
    this.en = en;
    var fit = en.solar_fit_w_m2, R = opt.reserve === undefined ? en.reserve_wh : opt.reserve, C = en.capacity_wh, D = en.flight_draw_w;
    this.P = []; this.need = []; this.needRaw = []; this.rest = []; this.dayStart = [];
    var n, k;
    for (n = 0; n < graph.lat.length; n++) {
      if (graph.loc[n] === null) { this.P.push(null); this.need.push(null); this.needRaw.push(null); this.rest.push(null); this.dayStart.push(null); continue; }
      var la = graph.lat[n], lo = graph.lon[n], ll = layer ? layer.locate(la, lo) : null, P = new Float64Array(this.K);
      for (k = 0; k < this.K; k++) {
        var th = this.T0 + 3600.0 * k, g = layer ? layer.hourMean(ll, th) : null;
        if (g === null) g = modelHourMean(la, lo, th, fit);
        P[k] = g * en.panel_m2;
      }
      this.P.push(P);
      var need = new Float64Array(this.K), cap = new Float64Array(this.K), short = new Float64Array(this.K), req = R, rc = R, sh = 0.0, net;
      for (k = this.K - 1; k >= 0; k--) {
        net = P[k] - D;
        if (net < 0 && (k + 1 === this.K || P[k + 1] - D >= 0)) { req = R; rc = R; sh = 0.0; }
        if (net >= 0 && k + 1 < this.K && P[k + 1] - D < 0) { sh = Math.max(0.0, rc - C); rc = Math.min(rc, C); }
        need[k] = req; cap[k] = rc; short[k] = sh;
        var rs = Math.max(R, req - net);
        req = rs > C + 1e-9 ? Infinity : rs;
        rc = Math.max(R, rc - net);
      }
      this.need.push(need);
      this.needRaw.push([cap, short]);
      var rest = new Float64Array(this.K), acc = 0.0, day = null, dk;
      for (k = this.K - 1; k >= 0; k--) {
        dk = Math.floor((this.T0 + 3600.0 * k + 1800.0 + lo * 240.0) / 86400.0);
        if (dk !== day) { acc = 0.0; day = dk; }
        acc += P[k];
        rest[k] = acc;
      }
      this.rest.push(rest);
      var ds = new Int32Array(this.K), first = 0; day = null;
      for (k = 0; k < this.K; k++) {
        dk = Math.floor((this.T0 + 3600.0 * k + 1800.0 + lo * 240.0) / 86400.0);
        if (dk !== day) { first = k; day = dk; }
        ds[k] = first;
      }
      this.dayStart.push(ds);
    }
  }
  Tables.prototype.hour = function (t) { return Math.floor((t - this.T0) / 3600.0); };
  Tables.prototype.solarWh = function (n, ta, tb) {
    var P = this.P[n], s = 0.0, t = ta;
    while (t < tb - 1e-9) {
      var k = this.hour(t), te = Math.min(tb, this.T0 + 3600.0 * (k + 1));
      if (k >= 0 && k < this.K) s += P[k] * (te - t) / 3600.0;
      t = te;
    }
    return s;
  };
  Tables.prototype.needAt = function (n, t, reserve, raw) {
    var k = this.hour(t);
    if (k < 0 || k >= this.K) return reserve;
    var D = this.en.flight_draw_w, frac = (this.T0 + 3600.0 * (k + 1) - t) / 3600.0;
    if (!raw) return Math.max(reserve, this.need[n][k] - (this.P[n][k] - D) * frac);
    var cr = this.needRaw[n];
    return Math.max(reserve, cr[0][k] - (this.P[n][k] - D) * frac) + cr[1][k];
  };
  Tables.prototype.dayTotal = function (n, t) {
    var k = this.hour(t);
    if (k < 0 || k >= this.K) return 0.0;
    return this.rest[n][this.dayStart[n][k]];
  };
  Tables.prototype.restAt = function (n, t) {
    var k = this.hour(t);
    if (k < 0 || k >= this.K) return 0.0;
    return Math.max(0.0, this.rest[n][k] - this.P[n][k] * (t - this.T0 - 3600.0 * k) / 3600.0);
  };

  // ------------------------------------------------------------ search (sim.route.search)
  function Heap() { this.a = []; }
  function less(x, y) { return x[0] < y[0] || (x[0] === y[0] && x[1] < y[1]); }
  Heap.prototype.push = function (x) {
    var a = this.a, i = a.length;
    a.push(x);
    while (i > 0) { var p = (i - 1) >> 1; if (!less(a[i], a[p])) break; var t = a[i]; a[i] = a[p]; a[p] = t; i = p; }
  };
  Heap.prototype.pop = function () {
    var a = this.a, top = a[0], last = a.pop();
    if (a.length) {
      a[0] = last;
      var i = 0, n = a.length;
      for (;;) {
        var l = 2 * i + 1, r = l + 1, m = i;
        if (l < n && less(a[l], a[m])) m = l;
        if (r < n && less(a[r], a[m])) m = r;
        if (m === i) break;
        var t = a[i]; a[i] = a[m]; a[m] = t; i = m;
      }
    }
    return top;
  };

  function search(graph, field, legs, tab, en, start, dest, t0, o) {
    o = o || {};
    var st = settingsOf(o.settings);
    var R = o.reserve === undefined ? en.reserve_wh : o.reserve, C = en.capacity_wh;
    var night = o.night === undefined ? true : o.night;
    var sh = o.share === undefined || o.share === null ? en.motor_share : o.share, useSlack = Number.isFinite(sh);
    var avoid = en.avoid_hs_m, H = o.horizon === undefined ? field.tEnd : o.horizon;
    var fd = en.flight_draw_w, wd = en.water_draw_w, WAIT = st.wait_s, EPS_SOC = st.eps_soc, EPS_SLACK = st.eps_slack;
    var vm = o.vMax === undefined ? legs.vMax(field.u10Max) : o.vMax, maxPops = o.maxPops || 2000000;
    var dl = graph.lat[dest], dn = graph.lon[dest], hc = new Map();
    function heur(n) { var v = hc.get(n); if (v === undefined) { v = gcKm(graph.lat[n], graph.lon[n], dl, dn) * 1000.0 / vm; hc.set(n, v); } return v; }
    var LT = [], LS = [], LK = [], LD = [], LW = [], LN = [], LP = [], LE = [], LA = [], pareto = new Map();
    var why = { seas: 0, no_leg: 0, horizon: 0, reserve: 0, allowance: 0, night: 0, dominated: 0 };
    var stats = { pops: 0, labels: 0, capped: false, pruned: why };
    var T0 = tab.T0, toWh = en.takeoff_wh;
    // label a at least as good as label b at node n (sim.route.search: dom): the same hour of the tables, a not later,
    // and a's charge and slack after holding on until b's time at least b's
    function dom(n, ta, sa, ka, da, wa, tb, sb, kb, db, wb) {
      if (ta > tb || Math.floor((ta - T0) / 3600.0) !== Math.floor((tb - T0) / 3600.0)) return false;
      var x = sa, sol = 0.0;
      if (tb > ta) {
        sol = tab.solarWh(n, ta, tb);
        var drain = (wa || wb) ? wd : fd;
        x = Math.min(C, sa + sol - drain * (tb - ta) / 3600.0);
      }
      if (wa && !wb) x -= toWh;
      if (x < sb - EPS_SOC) return false;
      if (!useSlack) return true;
      return (da === db && ka + sh * sol >= kb - EPS_SLACK) || (da < db && ka >= 0.0);
    }
    function pkey(n, t) { return n * 100000 + Math.floor((t - T0) / 3600.0); }
    function dominated(n, t, soc, slack, day, water) {
      var lst = pareto.get(pkey(n, t)) || [], q, li;
      for (q = 0; q < lst.length; q++) {
        li = lst[q];
        if (LA[li] && dom(n, LT[li], LS[li], LK[li], LD[li], LW[li], t, soc, slack, day, water)) return true;
      }
      return false;
    }
    function add(n, t, soc, slack, day, water, parent, edge) {
      if (dominated(n, t, soc, slack, day, water)) { why.dominated++; return null; }
      var lst = pareto.get(pkey(n, t)), q, li;
      if (!lst) { lst = []; pareto.set(pkey(n, t), lst); }
      for (q = 0; q < lst.length; q++) {
        li = lst[q];
        if (LA[li] && dom(n, t, soc, slack, day, water, LT[li], LS[li], LK[li], LD[li], LW[li])) LA[li] = false;
      }
      var kept = lst.filter(function (x) { return LA[x]; });
      lst.length = 0; Array.prototype.push.apply(lst, kept);
      var i = LT.length;
      LT.push(t); LS.push(soc); LK.push(slack); LD.push(day); LW.push(water); LN.push(n); LP.push(parent); LE.push(edge); LA.push(true);
      lst.push(i);
      return i;
    }
    function okAfter(n, t, soc, slack, day) {
      if (night && soc < tab.needAt(n, t, R) - 1e-9) { why.night++; return false; }
      if (useSlack) {
        var rest = localDay(t, graph.lon[n]) === day ? tab.restAt(n, t) : 0.0;
        if (slack + sh * rest < -EPS_SLACK) { why.allowance++; return false; }
      }
      return true;
    }
    var seq = 0, soc0 = Math.min(C, en.soc0_wh);
    var k0 = useSlack ? sh * (tab.dayTotal(start, t0) - tab.restAt(start, t0)) : 0.0;
    var i0 = add(start, t0, soc0, k0, localDay(t0, graph.lon[start]), false, null, null);
    var heap = new Heap();
    heap.push([t0 + heur(start), seq, i0]);
    var labels = { LT: LT, LS: LS, LK: LK, LD: LD, LW: LW, LN: LN, LP: LP, LE: LE };
    while (heap.a.length) {
      var li = heap.pop()[2];
      if (!LA[li]) continue;
      stats.pops++;
      if (stats.pops > maxPops) { stats.capped = true; break; }
      var n = LN[li];
      if (n === dest) { stats.labels = LT.length; return { labels: labels, li: li, stats: stats }; }
      var t = LT[li], soc = LS[li], slack = LK[li], day = LD[li], water = LW[li];
      if (t >= H) continue;
      var lonN = graph.lon[n], te = Math.min(t + WAIT, T0 + 3600.0 * (Math.floor((t - T0) / 3600.0) + 1)), d2, k2, s2, sol, dd, good, j, e;
      // wait on the water
      if (te <= H + 1e-9) {
        var hsA = field.sample(graph.loc[n], t)[2], hsB = field.sample(graph.loc[n], te)[2];
        if (Number.isFinite(hsA) && Number.isFinite(hsB) && hsA <= avoid && hsB <= avoid) {
          d2 = day; k2 = slack; dd = localDay(t, lonN); good = true;
          if (dd > d2) { if (useSlack && k2 < -EPS_SLACK) good = false; d2 = dd; k2 = 0.0; }
          if (good) {
            sol = tab.solarWh(n, t, te);
            s2 = Math.min(C, soc + sol - wd * (te - t) / 3600.0);
            k2 = k2 + (useSlack ? sh * sol : 0.0);
            if (s2 >= R - 1e-9 && okAfter(n, te, s2, k2, d2) && !(n === dest)) {
              e = ['wait', n, n, t, te, ['waiting'], 0.0, wd * (te - t) / 3600.0, sol, 0.0];
              j = add(n, te, s2, k2, d2, true, li, e);
              if (j !== null) { seq++; heap.push([te + heur(n), seq, j]); }
            }
          }
        }
      }
      // legs
      var es = graph.edges[n], q;
      for (q = 0; q < es.length; q++) {
        var m = es[q][0], subM = es[q][1], steps = es[q][2];
        var tt = t, motor = 0.0, draw = 0.0, solTot = 0.0, to = 0.0, modes = [];
        s2 = soc; k2 = slack; d2 = day; good = true;
        if (water) {
          dd = localDay(tt, lonN);
          if (dd > d2) { if (useSlack && k2 < -EPS_SLACK) continue; d2 = dd; k2 = 0.0; }
          to = en.takeoff_wh;
          s2 -= to;
          k2 -= useSlack ? to : 0.0;
          motor += to;
          tt += en.takeoff_s;
          if (s2 < R - 1e-9) continue;
        }
        for (var z = 0; z < steps.length; z++) {
          var lc = steps[z][0], brg = steps[z][1];
          if (tt > H) { why.horizon++; good = false; break; }
          var smp = field.sample(lc, tt), u = smp[0], v = smp[1], hs = smp[2];
          if (!(hs <= avoid)) { why.seas++; good = false; break; }
          var sp = Math.sqrt(u * u + v * v), wf = wrap360(Math.atan2(-u, -v) / D2R);
          var lg = legs.leg(sp, brg - wf - 180.0);
          if (lg[0] === 'none') { why.no_leg++; good = false; break; }
          var dt = subM / lg[1];
          dd = localDay(tt, lonN);
          if (dd > d2) {
            if (useSlack && k2 < -EPS_SLACK) { why.allowance++; good = false; break; }
            d2 = dd; k2 = 0.0;
          }
          sol = tab.solarWh(n, tt, tt + dt);
          var mo = lg[2] * dt / 3600.0, dr = fd * dt / 3600.0;
          s2 = Math.min(C, s2 + sol - dr - mo);
          if (useSlack) k2 += sh * sol - mo;
          motor += mo; draw += dr; solTot += sol; tt += dt;
          modes.push(lg[0]);
          if (s2 < R - 1e-9) { why.reserve++; good = false; break; }
          if (useSlack) {
            var rest = localDay(tt, lonN) === d2 ? tab.restAt(n, tt) : 0.0;
            if (k2 + sh * rest < -EPS_SLACK) { why.allowance++; good = false; break; }
          }
        }
        if (!good || !okAfter(m, tt, s2, k2, d2)) continue;
        e = ['leg', n, m, t, tt, modes, motor, draw, solTot, to];
        j = add(m, tt, s2, k2, d2, false, li, e);
        if (j !== null) { seq++; heap.push([tt + heur(m), seq, j]); }
      }
    }
    stats.labels = LT.length;
    return { labels: labels, li: null, stats: stats };
  }

  function routeOf(graph, labels, li) {
    var chain = [];
    while (li !== null) { chain.push(li); li = labels.LP[li]; }
    chain.reverse();
    var wps = [], legs = [];
    chain.forEach(function (x) {
      var n = labels.LN[x];
      wps.push({ node: n, lat: graph.lat[n], lon: graph.lon[n], t: labels.LT[x], soc_wh: labels.LS[x], slack_wh: labels.LK[x], water: labels.LW[x] });
      var e = labels.LE[x];
      if (e === null) return;
      var modes = e[5], uniq = modes.filter(function (m, i) { return modes.indexOf(m) === i; });
      legs.push({ from: e[1], to: e[2], t0: e[3], t1: e[4], mode: e[0] === 'wait' ? 'waiting' : (uniq.length === 1 ? modes[0] : 'mixed'),
        modes: modes.slice(), motor_wh: e[6], draw_wh: e[7], solar_wh: e[8], takeoff_wh: e[9], soc_end_wh: labels.LS[x] });
    });
    return { waypoints: wps, legs: legs };
  }
  function summarize(legs, wps, t0) {
    var out = { time_h: (wps[wps.length - 1].t - t0) / 3600.0, motor_wh: 0.0, takeoff_wh: 0.0, draw_wh: 0.0, solar_wh: 0.0,
      soaring_h: 0.0, motor_h: 0.0, wait_h: 0.0, waits: 0, min_soc_wh: Math.min.apply(null, wps.map(function (w) { return w.soc_wh; })),
      soc_end_wh: wps[wps.length - 1].soc_wh };
    var prevWait = false;
    legs.forEach(function (lg) {
      var dt = (lg.t1 - lg.t0) / 3600.0;
      out.motor_wh += lg.motor_wh; out.takeoff_wh += lg.takeoff_wh; out.draw_wh += lg.draw_wh; out.solar_wh += lg.solar_wh;
      if (lg.mode === 'waiting') { out.wait_h += dt; out.waits += prevWait ? 0 : 1; prevWait = true; } else {
        prevWait = false;
        var ns = lg.modes.length;
        out.soaring_h += dt * lg.modes.filter(function (m) { return m === 'soaring'; }).length / ns;
        out.motor_h += dt * lg.modes.filter(function (m) { return m === 'motor'; }).length / ns;
      }
    });
    out.net_wh = out.draw_wh + out.motor_wh - out.solar_wh;
    return out;
  }
  function strictMargin(graph, tab, en, wps, legs) {
    var R = en.reserve_wh, socM = Infinity, nightM = Infinity;
    wps.forEach(function (w) { socM = Math.min(socM, w.soc_wh - R); nightM = Math.min(nightM, w.soc_wh - tab.needAt(w.node, w.t, R, true)); });
    var s0 = wps[0], days = new Map(), motor = new Map();
    days.set(localDay(s0.t, s0.lon), tab.dayTotal(s0.node, s0.t) - tab.restAt(s0.node, s0.t));
    legs.forEach(function (lg) {
      var d = localDay(lg.t0, graph.lon[lg.from]);
      days.set(d, (days.has(d) ? days.get(d) : 0.0) + lg.solar_wh);
      motor.set(d, (motor.has(d) ? motor.get(d) : 0.0) + lg.motor_wh);
    });
    var am = Infinity;
    days.forEach(function (v, d) { am = Math.min(am, en.motor_share * v - (motor.has(d) ? motor.get(d) : 0.0)); });
    return [socM, nightM, am];
  }

  // plan a route (sim.route.plan): o = {t0, layer, tables, relax, maxPops, relaxPops, settings}
  function plan(field, graph, legs, en, startLL, destLL, o) {
    o = o || {};
    var t0 = o.t0 === undefined ? field.t0 : o.t0, layer = o.layer || null;
    var tab = o.tables || new Tables(graph, field, en, layer, { settings: o.settings });
    var sn = nearest(graph, startLL[0], startLL[1]), dn = nearest(graph, destLL[0], destLL[1]), s = sn[0], d = dn[0];
    var out = { start_node: s, dest_node: d, t0: t0, start_snap_km: sn[1], dest_snap_km: dn[1],
      dist_km: s !== null && d !== null ? gcKm(graph.lat[s], graph.lon[s], graph.lat[d], graph.lon[d]) : null };
    if (s === null || d === null || field.sample(graph.loc[s], t0)[2] > en.avoid_hs_m) {
      out.ok = false; out.reason = 1; out.reason_text = REASONS[1]; out.route = null; out.closest = null; return out;
    }
    var vm = legs.vMax(field.u10Max), so = { vMax: vm, maxPops: o.maxPops || 300000, settings: o.settings };
    var r = search(graph, field, legs, tab, en, s, d, t0, so);
    out.stats = r.stats;
    if (r.li !== null) {
      var rt = routeOf(graph, r.labels, r.li);
      out.ok = true; out.reason = 0; out.reason_text = REASONS[0];
      out.route = { waypoints: rt.waypoints, legs: rt.legs, summary: summarize(rt.legs, rt.waypoints, t0) }; out.closest = null;
      return out;
    }
    out.ok = false;
    var R = en.reserve_wh, C = en.capacity_wh, startNeed = tab.needAt(s, t0, R, true);
    var reason = Math.min(C, en.soc0_wh) < startNeed - 1e-9 ? 2 : null;
    out.start_need_wh = startNeed; out.route = null; out.closest = null;
    if (o.relax === false) { out.reason = reason || 6; out.reason_text = REASONS[out.reason]; return out; }
    var skip = false, q;
    for (q = 0; q < RELAX.length; q++) {
      var name = RELAX[q][0], kw = RELAX[q][1];
      if (skip && kw.share === undefined) continue;
      var rv = R - kw.below * C;
      var tab2 = new Tables(graph, field, en, layer, { reserve: rv, tStart: tab.T0, hours: tab.K, settings: o.settings });
      var r2 = search(graph, field, legs, tab2, en, s, d, t0, { vMax: vm, maxPops: o.relaxPops || 150000, reserve: rv, share: kw.share, settings: o.settings });
      if (r2.li === null) { skip = r2.stats.pruned.reserve === 0 && r2.stats.pruned.night === 0; continue; }
      var rt2 = routeOf(graph, r2.labels, r2.li), mg = strictMargin(graph, tab, en, rt2.waypoints, rt2.legs);
      out.closest = { relaxation: name, waypoints: rt2.waypoints, legs: rt2.legs, summary: summarize(rt2.legs, rt2.waypoints, t0),
        reserve_margin_wh: mg[0], night_margin_wh: mg[1], allowance_margin_wh: mg[2], stats: r2.stats };
      if (reason === null) reason = mg[1] < -1e-6 ? 3 : mg[0] < -1e-6 ? 4 : mg[2] < -1e-6 ? 5 : 7;
      break;
    }
    if (reason === null) reason = 6;
    out.reason = reason; out.reason_text = REASONS[reason];
    return out;
  }

  // energy parameters of an aircraft from route_params.js with a capacity factor (sim.route.energy_defaults)
  function energyOf(params, aircraft, capacityFactor) {
    var e = Object.assign({}, params.aircraft[aircraft].energy), f = capacityFactor === undefined ? 1 : capacityFactor;
    e.capacity_wh *= f; e.reserve_wh = e.reserve_fraction * e.capacity_wh; e.soc0_wh = e.capacity_wh;
    return e;
  }

  var ROUTE = { DEFAULTS: DEFAULTS, RELAX: RELAX, REASONS: REASONS, sunSinElev: sunSinElev, modelHourMean: modelHourMean,
    bilinear: bilinear, CorridorField: CorridorField, SolarLayer: SolarLayer, gcKm: gcKm, gcPoint: gcPoint, gcBearing: gcBearing,
    wrap360: wrap360, wrap180: wrap180, ringOffsets: ringOffsets, landTester: landTester, corridorGraph: corridorGraph, nearest: nearest,
    LegModel: LegModel, Tables: Tables, localDay: localDay, search: search, routeOf: routeOf, summarize: summarize,
    strictMargin: strictMargin, plan: plan, energyOf: energyOf };
  root.ROUTE = ROUTE;
  if (typeof module !== 'undefined' && module.exports) module.exports = ROUTE;
})(typeof window !== 'undefined' ? window : globalThis);
