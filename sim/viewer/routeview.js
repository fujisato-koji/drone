// The route planner in the viewer's world seas (docs/02-form.md#viewer). Read the notes of sim/viewer/route.js first:
// this file does not plan; it gives route.js (the port of sim/route.py) the viewer's inputs, runs it in a worker made
// from a Blob URL (which works from a page opened as a file), and turns a plan into what the map and the 3D view draw
// and fly.
//
// Planner(params, land).plan(job) is the viewer's planning path, in the worker and in sim/tests/test_route_view_js.py.
// job = {key, corridor (WorldData.unpack of a world file's corridor), solar (a solar layer file's object, or null for
// the fitted model), aircraft, capWh (usable capacity, Wh), share (the motor's share of the day's solar), start
// [lat, lon], dest [lat, lon], t0 (s)}. It plans as sim/route_exp.js does (the same graph, leg model, tables and
// settings of route_params.js) and adds, for the route and the closest route: the steps of each leg with their
// times (the field and the leg model as the search flew them), the runs of one kind (segments: soaring, motor, wait),
// the lowest charge and the margins against the strict rules.
// FlightPlan(segments) is the bookkeeping of the 3D flight: the segment flown, the arrival at its end, a wait and
// the launch after it. Coverage(corridor, land) says whether a clicked point lies in the data.
// Classic script: attaches RouteView to globalThis; module.exports under node.
(function routeViewModule(root) {
  'use strict';
  var ROUTE = root.ROUTE || (typeof require !== 'undefined' ? require('./route.js') : null);
  var NAV = root.NAV || (typeof require !== 'undefined' ? require('./navigator.js') : null);
  var WD = root.WorldData || (typeof require !== 'undefined' ? require('./worlddata.js') : null);
  var R2D = 180 / Math.PI;

  // the defaults of the energy limit of an aircraft, from route_params.js (sim.route.energy_defaults: the usable
  // capacity of D-023's battery, the motor's share of D-021)
  function defaults(params, aircraft) {
    var e = params.aircraft[aircraft].energy;
    return { capacityWh: e.capacity_wh, motorShare: e.motor_share, reserveFraction: e.reserve_fraction };
  }
  // the energy of a plan: route.js energyOf with the capacity factor capWh over the default (the reserve is its share
  // of the capacity, the battery starts full), and the motor's share
  function energyFor(params, aircraft, capWh, share) {
    var d = params.aircraft[aircraft].energy.capacity_wh;
    var en = ROUTE.energyOf(params, aircraft, capWh === undefined || capWh === null ? 1 : capWh / d);
    if (share !== undefined && share !== null) en.motor_share = share;
    return en;
  }

  // whether a point lies in the data a plan can use at time t: 'ok', 'outside' (the corridor's grid), 'land' or
  // 'nodata' (no finite wind or wave height there and then, as where a forecast's fetch has no values)
  function Coverage(corridor, land) {
    this.field = new ROUTE.CorridorField(corridor);
    this.onLand = land && land.length ? ROUTE.landTester(land) : null;
  }
  Coverage.prototype.check = function (lat, lon, t) {
    var loc = this.field.locate(lat, lon);
    if (loc === null) return 'outside';
    if (this.onLand && this.onLand(lat, lon)) return 'land';
    var s = this.field.sample(loc, t);
    if (!Number.isFinite(s[0]) || !Number.isFinite(s[1]) || !Number.isFinite(s[2])) return 'nodata';
    return 'ok';
  };

  // the steps of a leg as the search flew them (route.js search: fly): each step's mode, ends and times, from the same
  // samples of the field and the same leg model; water: the leg starts on the water (a take-off first)
  function legSteps(c, legs, en, lg, water) {
    var g = c.graph, es = g.edges[lg.from], e = null, q;
    for (q = 0; q < es.length; q++) if (es[q][0] === lg.to) { e = es[q]; break; }
    if (e === null) return null;
    var subM = e[1], steps = e[2], ns = steps.length, la0 = g.lat[lg.from], lo0 = g.lon[lg.from], la1 = g.lat[lg.to], lo1 = g.lon[lg.to];
    var tt = lg.t0 + (water ? en.takeoff_s : 0), out = [], z;
    for (z = 0; z < ns; z++) {
      var smp = c.field.sample(steps[z][0], tt), u = smp[0], v = smp[1];
      var r = legs.leg(Math.sqrt(u * u + v * v), steps[z][1] - ROUTE.wrap360(Math.atan2(-u, -v) * R2D) - 180.0);
      var dt = subM / (lg.modes[z] === 'soaring' ? r[0] : r[1]);
      out.push({ mode: lg.modes[z], a: ROUTE.gcPoint(la0, lo0, la1, lo1, z / ns),
        b: z + 1 === ns ? [la1, lo1] : ROUTE.gcPoint(la0, lo0, la1, lo1, (z + 1) / ns), t0: tt, t1: tt + dt });
      tt += dt;
    }
    return out;
  }

  // the facts of a wait (the plan does not record why it waits): the charge, the allowance left, the sea and the wind
  // at both ends, whether a local midnight renews the allowance inside it, and the share of it the sun is down
  function waitFacts(c, en, s) {
    var loc = c.graph.loc[s.node], a = c.field.sample(loc, s.t0), b = c.field.sample(loc, s.t1);
    var lat = c.graph.lat[s.node], lon = c.graph.lon[s.node], n = 0, d = 0, t;
    s.hs = [a[2], b[2]]; s.wind = [Math.sqrt(a[0] * a[0] + a[1] * a[1]), Math.sqrt(b[0] * b[0] + b[1] * b[1])];
    s.avoid_hs_m = en.avoid_hs_m;
    s.midnight = ROUTE.localDay(s.t0, lon) !== ROUTE.localDay(s.t1, lon);
    for (t = s.t0 + 450; t < s.t1; t += 900) { n++; if (ROUTE.sunSinElev(lat, lon, t) < 0) d++; }
    s.night_share = n ? d / n : 0;
  }

  // a route (or the closest route) of a plan, with its steps, segments, length, lowest charge and strict margins
  function describe(c, legs, en, tab, rt) {
    var wps = rt.waypoints, segs = [], err = 0, len = 0, lo = Infinity, i, k, q;
    for (i = 0; i < wps.length; i++) lo = Math.min(lo, wps[i].soc_wh);
    for (i = 0; i < rt.legs.length; i++) {
      var lg = rt.legs[i], w0 = wps[i], w1 = wps[i + 1];
      lo = Math.min(lo, lg.soc_min_wh);
      if (lg.mode === 'waiting') {
        var last = segs.length ? segs[segs.length - 1] : null;
        if (last && last.kind === 'wait' && last.node === lg.from) {
          last.t1 = lg.t1; last.soc1 = w1.soc_wh; last.slack1 = w1.slack_wh; last.solar_wh += lg.solar_wh; last.draw_wh += lg.draw_wh;
        } else {
          segs.push({ kind: 'wait', node: lg.from, leg: i, a: [w0.lat, w0.lon], b: [w0.lat, w0.lon], t0: lg.t0, t1: lg.t1,
            soc0: w0.soc_wh, soc1: w1.soc_wh, slack0: w0.slack_wh, slack1: w1.slack_wh, solar_wh: lg.solar_wh, draw_wh: lg.draw_wh });
        }
        continue;
      }
      var st = legSteps(c, legs, en, lg, w0.water);
      lg.steps = st;
      len += ROUTE.gcKm(w0.lat, w0.lon, w1.lat, w1.lon);
      err = Math.max(err, Math.abs(st[st.length - 1].t1 - lg.t1));
      k = 0;
      while (k < st.length) {   // the runs of one mode inside the leg
        var j = k;
        while (j + 1 < st.length && st[j + 1].mode === st[k].mode) j++;
        var pts = [st[k].a];
        for (q = k; q <= j; q++) pts.push(st[q].b);
        segs.push({ kind: st[k].mode === 'soaring' ? 'soaring' : 'motor', leg: i, a: st[k].a, b: st[j].b, pts: pts,
          t0: k === 0 ? lg.t0 : st[k].t0, t1: j + 1 === st.length ? lg.t1 : st[j].t1, takeoff: k === 0 && !!w0.water });
        k = j + 1;
      }
    }
    segs.forEach(function (s) { if (s.kind === 'wait') waitFacts(c, en, s); });
    rt.segments = segs;
    rt.length_km = len;
    rt.lowest_soc_wh = lo;
    rt.step_error_s = err;   // the steps' times against the search's (a test requires it to be rounding)
    var m = ROUTE.strictMargin(c.graph, tab, en, wps, rt.legs);
    rt.margins = { reserve_wh: m[0], night_wh: m[1], allowance_wh: m[2] };
  }

  function Planner(params, land) {
    this.params = params; this.land = land || null; this.ctx = new Map(); this.legs = {};
  }
  // the field, the graph and the solar layer of a world file, kept by its key
  Planner.prototype.context = function (job) {
    var c = this.ctx.get(job.key);
    if (!c) {
      var field = new ROUTE.CorridorField(job.corridor);
      c = { field: field, graph: ROUTE.corridorGraph(field, this.params.settings, this.land),
        layer: job.solar ? new ROUTE.SolarLayer(job.solar) : null, tabs: new Map() };
      if (this.ctx.size >= 4) this.ctx.clear();
      this.ctx.set(job.key, c);
    }
    return c;
  };
  Planner.prototype.legModel = function (aircraft) {
    var p = this.params.aircraft[aircraft];
    return this.legs[aircraft] || (this.legs[aircraft] = new ROUTE.LegModel(p.hulls, p.steady, this.params.settings));
  };
  Planner.prototype.plan = function (job) {
    var st = this.params.settings, c = this.context(job), en = energyFor(this.params, job.aircraft, job.capWh, job.share);
    var tk = job.aircraft + '|' + en.capacity_wh, tab = c.tabs.get(tk);
    if (!tab) {
      tab = new ROUTE.Tables(c.graph, c.field, en, c.layer, { settings: st });
      if (c.tabs.size >= 4) c.tabs.clear();
      c.tabs.set(tk, tab);
    }
    var legs = this.legModel(job.aircraft);
    var r = ROUTE.plan(c.field, c.graph, legs, en, job.start, job.dest, { t0: job.t0, layer: c.layer, tables: tab, settings: st });
    r.key = job.key; r.aircraft = job.aircraft; r.solar = c.layer ? 'layer' : 'model';
    r.energy = { capacity_wh: en.capacity_wh, reserve_wh: en.reserve_wh, motor_share: en.motor_share, avoid_hs_m: en.avoid_hs_m };
    r.relax_below = st.relax_below.slice();
    if (r.route) describe(c, legs, en, tab, r.route);
    if (r.closest) describe(c, legs, en, tab, r.closest);
    return r;
  };

  // The 3D flight of a route's segments. i is the segment flown (or the wait shown); arrive(tw) at the end of a
  // flying segment at weather time tw goes on to the next ('fly'), stops at a wait ('wait') or at the destination
  // ('end'); leave(tw) ends a wait and gives the weather time and the place of the next launch: the wait's end, or tw
  // when the flight is later than the plan.
  function FlightPlan(segments) {
    this.segs = segments; this.i = 0; this.done = false; this.log = [];
    this.wait = segments.length && segments[0].kind === 'wait' ? segments[0] : null;
  }
  FlightPlan.prototype.seg = function () { return this.segs[this.i] || null; };
  FlightPlan.prototype.arrive = function (tw) {
    var s = this.seg();
    this.log.push({ i: this.i, t: tw, plan: s ? s.t1 : null });
    this.i++;
    var n = this.seg();
    if (!n) { this.done = true; return 'end'; }
    if (n.kind === 'wait') { this.wait = n; return 'wait'; }
    return 'fly';
  };
  FlightPlan.prototype.leave = function (tw) {
    var w = this.wait;
    if (!w) return null;
    this.wait = null; this.i++;
    if (!this.seg()) this.done = true;
    return { t: Math.max(w.t1, tw), at: w.a };
  };
  // the plan's time at the aircraft on the flying segment: by the share of the segment's length flown
  FlightPlan.prototype.planTime = function (left, length) {
    var s = this.seg();
    if (!s || s.kind === 'wait') return null;
    var f = length > 0 ? Math.min(1, Math.max(0, 1 - left / length)) : 1;
    return s.t0 + f * (s.t1 - s.t0);
  };

  // the worker: 'init' with route_params.js and the land rings, then 'plan' jobs; replies 'plan' with the result and
  // the time the plan took in the worker, or 'error'
  function workerMain(scope) {
    var planner = null;
    scope.onmessage = function (e) {
      var m = e.data;
      try {
        if (m.type === 'init') { planner = new Planner(m.params, m.land); scope.postMessage({ type: 'ready' }); return; }
        if (m.type === 'plan') {
          var t1 = Date.now(), r = planner.plan(m.job);
          r.plan_ms = Date.now() - t1;
          scope.postMessage({ type: 'plan', id: m.id, result: r });
        }
      } catch (err) {
        scope.postMessage({ type: 'error', id: m.id, message: String((err && err.message) || err) });
      }
    };
  }
  // the worker's code: navigator.js, worlddata.js, route.js and this file, from their own source texts
  function workerSource() {
    return NAV.moduleSource() + WD.moduleSource() + ROUTE.moduleSource()
      + '(' + routeViewModule.toString() + ')(globalThis);\nRouteView.workerMain(self);\n';
  }

  var RouteView = { defaults: defaults, energyFor: energyFor, Coverage: Coverage, legSteps: legSteps, Planner: Planner,
    FlightPlan: FlightPlan, workerMain: workerMain, workerSource: workerSource };
  root.RouteView = RouteView;
  if (typeof module !== 'undefined' && module.exports) module.exports = RouteView;
})(typeof window !== 'undefined' ? window : globalThis);
