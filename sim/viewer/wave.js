// Flight over swell for the viewer: a JavaScript mirror of the wave loop of sim/wavelib.py and sim/mpc.py
// (docs/02-form.md#wave-loop): the oracle sea of sim.wavelib.swell_env (one regular long-crested component with
// the second-order surface, the COARE 3.5 log wind and the wave wind of a preset), WaveCycleGuide (a wave cycle
// locked to the wave phase at the aircraft, with the crest-ride families), WavePilot (the wave library's entry for
// the sea, the wind and the course) and the closed loop's choice of reference (sim.mpc.fly_mpc with a wave library:
// start in flat flight, enter a wave cycle and switch between wave cycles at a wave-phase match).
//
// Classic script. Everything lives in waveFactory(MPC), so that the same source text runs in the planner's Web
// Worker (WAVE.workerSource(), which also carries mpc.js); attaches WAVE to globalThis and sets module.exports
// under node (which needs mpc.js next to it). The library comes from python -m sim.export_wave_library
// (sim/viewer/wave/index.js and one file per sea and aircraft); sim/tests/test_wave_js.py checks it all against
// Python.
//
// Frames: the library frame of sim.wavelib (the mean wind blows toward +x, angles counter-clockwise from +x) for
// cycles, courses and the swell's direction; the world is sim coordinates (x east, y north, z up).
function waveFactory(MPC) {
  'use strict';
  var G = 9.81, PI = Math.PI, TWO_PI = 2.0 * Math.PI, wrap = MPC.wrap, interp = MPC.interp;
  // sim.mpc WAVE_GUIDE_SIG, WAVE_SIG_THETA, WAVE_CREST_SIG, WAVE_CREST_SIG_THETA, WAVE_W_RUN, CREST_CLIMB_M,
  // WAVE_SWITCH_THETA, WAVE_SWITCH_MAX_WAIT_S (sim/tests/test_wave_js.py checks them)
  var GUIDE = { sig: [2.0, 2.0, 0.15, 0.15], sigTheta: 0.3, crestSig: [2.0, 5.0, 0.15, 0.15], crestSigTheta: 0.1,
    wRun: 5.0, crestClimbM: 2.0, switchTheta: 0.3, switchMaxWaitS: 60.0 };
  var SWELL_DIRS = { along: 0.0, across: 90.0, against: 180.0 };   // sim.wavelib.SWELL_DIRS
  var CALM_U10 = 0.1;                                               // sim.wavelib.CALM_U10

  // Python's float modulo (numpy.mod): the sign of the divisor, from fmod
  function pymod(a, b) { var m = a % b; if (m !== 0 && (m < 0) !== (b < 0)) m += b; return m; }
  function radians(d) { return d * (PI / 180.0); }   // numpy.radians
  function degrees(r) { return r * (180.0 / PI); }   // numpy.degrees
  function rint(x) {                                 // round half to even (numpy.rint)
    var f = Math.floor(x), d = x - f;
    if (d < 0.5) return f;
    if (d > 0.5) return f + 1;
    return f % 2 === 0 ? f : f + 1;
  }
  function round3(x) { return rint(x * 1000.0) / 1000.0; }   // numpy.round(x, 3)
  function sq(x) { return x * x; }

  // ---------------------------------------------------------------------------- the oracle sea
  // sim.seawind.Wind(u10, from_deg, roughness="coare35"): COARE 3.5 Charnock parameter and the smooth-flow term,
  // the same 50 fixed-point steps. K: the sea block of the index (g, karman, nu_air, coare35 [slope, intercept, max]).
  function coareWind(u10, fromDeg, K) {
    var g = K.g, kar = K.karman, nu = K.nu_air, c = K.coare35;
    var w = { u10: u10, from_deg: fromDeg, karman: kar };
    w.to_deg = pymod(fromDeg + 180.0, 360.0);
    var r = radians(w.to_deg);
    w.ex = Math.sin(r); w.ey = Math.cos(r);
    var ustar = 0.03 * u10 + 1e-3, z0 = 0.0;
    var alpha = Math.min(Math.max(c[0] * u10 + c[1], 0.0), c[2]);
    for (var i = 0; i < 50; i++) {
      z0 = alpha * (ustar * ustar) / g + 0.11 * nu / ustar;
      ustar = kar * u10 / Math.log(10.0 / z0);
    }
    w.ustar = ustar; w.z0 = z0; w.alpha = alpha;
    return w;
  }
  function windSpeed(w, z) { return z > w.z0 ? (w.ustar / w.karman) * Math.log(z / w.z0) : 0.0; }

  // sim.wavelib.swell_env: set {u10, fromDeg (default 270), hs, tp, direction ("along", "across", "against" or deg
  // counter-clockwise from the wind's direction), preset ("p63" or "p64_low"), phase}; index: window.WAVE_INDEX.
  // Returns the world spec of mpc.js (kind "truth", wind, comps with urel, stokes, waveWind) and the swell.
  function swellSea(set, index) {
    var K = index.sea, pr = index.presets[set.preset];
    if (!pr) throw new Error('unknown preset ' + set.preset);
    var fromDeg = set.fromDeg === undefined ? K.from_deg : set.fromDeg;
    var w = coareWind(Math.max(+set.u10, CALM_U10), fromDeg, K);
    var rel = typeof set.direction === 'string' ? SWELL_DIRS[set.direction] : +set.direction;
    var comps = { a: [], k: [], omega: [], phase: [], dx: [], dy: [], urel: [] }, toDeg = null;
    if (set.hs > 0) {
      toDeg = pymod(w.to_deg - rel, 360.0);
      var om = 2.0 * PI / set.tp, rr = radians(toDeg), k = om * om / K.g;
      var dx = Math.sin(rr), dy = Math.cos(rr);
      var zref = Math.max(1.0 / k, 2.0 * w.z0), along = w.ex * dx + w.ey * dy;
      comps = { a: [0.5 * set.hs], k: [k], omega: [om], phase: [set.phase || 0.0], dx: [dx], dy: [dy],
        urel: [windSpeed(w, zref) * along - om / k] };
    }
    return { kind: 'truth', wind: { u10: w.u10, from_deg: w.from_deg, to_deg: w.to_deg, ex: w.ex, ey: w.ey,
      ustar: w.ustar, z0: w.z0, karman: w.karman }, comps: comps, stokes: true,
      waveWind: { amp: pr.amp.slice(), decay: pr.decay.slice() }, maxComponents: 0,
      swell: { hs: +set.hs, tp: +set.tp, relDeg: rel, toDeg: toDeg, preset: set.preset } };
  }

  // sim.mpc.dominant_wave: (a, k, dx, dy, phase, omega) of the largest component (amplitude rounded to 1 mm,
  // the longest wavelength on ties), or null without waves
  function dominantWave(world) {
    var c = world && world.c;
    if (!c || !c.a || c.a.length === 0) return null;
    var bi = 0;
    for (var i = 1; i < c.a.length; i++) {
      var ra = -round3(c.a[i]), rb = -round3(c.a[bi]);
      if (ra < rb || (ra === rb && c.k[i] < c.k[bi])) bi = i;
    }
    return [c.a[bi], c.k[bi], c.dx[bi], c.dy[bi], c.phase[bi], c.omega[bi]];
  }
  // sim.mpc.world_swell: [k, omega, dx, dy, phase, a] or null
  function worldSwell(world) {
    var w = dominantWave(world);
    return w === null ? null : [w[1], w[5], w[2], w[3], w[4], w[0]];
  }
  // sim.wavelib.sea_of: [hs, tp, rel deg (the swell's travel counter-clockwise from the wind's direction psiW)]
  function seaOf(world, psiW) {
    var sw = worldSwell(world);
    if (sw === null) return null;
    var rel = degrees(pymod(Math.atan2(sw[3], sw[2]) - psiW + PI, 2.0 * PI) - PI);
    return [2.0 * sw[5], 2.0 * PI / sw[1], rel];
  }
  // the wave phase at (x, y, t) of a world's dominant component, in [0, 2 pi): 0 at the crest, pi in the trough
  function phaseAt(world, x, y, t) {
    var sw = worldSwell(world);
    if (sw === null) return null;
    return pymod(sw[0] * (sw[2] * x + sw[3] * y) - sw[1] * t + sw[4], TWO_PI);
  }

  // ---------------------------------------------------------------------------- library entries
  // the node arrays of an exported entry (a crest ride is written once, sim.export_wave_library.expand_entry)
  function expandEntry(d) {
    if (!d.steady) return d;
    var e = Object.assign({}, d), n = d.n_nodes, P = d.period, i;
    e.t = [];
    for (i = 0; i < n; i++) e.t.push(P * i / (n - 1));
    Object.keys(d.steady).forEach(function (k) {
      var a = [];
      for (i = 0; i < n; i++) a.push(d.steady[k]);
      e[k] = a;
    });
    return e;
  }
  // sim.mpc.Cycle with the wave phase at the aircraft per node (unwrapped; over a period it advances by its last
  // minus its first value); a mirrored cycle flies the mirrored sea, so the phase is unchanged
  function waveCycle(d, mirror) {
    var c = new MPC.Cycle(d, mirror);
    c.wave = d.theta !== undefined && d.theta !== null;
    if (c.wave) {
      c.theta = MPC.unwrap(Float64Array.from(d.theta));
      c.dTheta = c.theta[c.theta.length - 1] - c.theta[0];
    }
    return c;
  }
  // Cycle.theta_at: the wave phase at cycle phase tau, continued over the periods
  function thetaAt(c, tau) {
    var T = c.T, k = Math.floor(tau / T);
    return interp(pymod(tau, T), c.t, c.theta) + k * c.dTheta;
  }

  // ---------------------------------------------------------------------------- the guide
  // sim.mpc.WaveCycleGuide. o: wD (10), sig, sigTheta (null: the defaults, those of a crest ride for a crest ride),
  // tauHint, hintWindow (0.2), wRun (0). Same interface as MPC.Guide (nominal, nominalThrust, terminalCost, tau0,
  // tauS, matchDist, delta), plus runningCost (with wRun > 0 the rollouts add it at every step) and thetaErr0.
  function WaveCycleGuide(cycle, world, t, s, psiW, horizon, o) {
    o = o || {};
    this.cycle = cycle; this.psiW = psiW; this.H = horizon;
    this.wD = o.wD !== undefined && o.wD !== null ? o.wD : 10.0;
    this.wRun = o.wRun !== undefined && o.wRun !== null ? +o.wRun : 0.0;
    this.tPlan = t;
    var crest = !!cycle.d.crest;
    var sg = o.sig ? o.sig : (crest ? GUIDE.crestSig : GUIDE.sig);
    this.sig = [sg[0], sg[1] ? sg[1] : 0.25 * cycle.vMean, sg[2], sg[3]];
    this.sigTheta = o.sigTheta !== undefined && o.sigTheta !== null ? +o.sigTheta
      : (crest ? GUIDE.crestSigTheta : GUIDE.sigTheta);
    var sw = worldSwell(world);
    if (sw === null || !cycle.wave) throw new Error('WaveCycleGuide needs a wave cycle and a world with waves');
    this.k = sw[0]; this.om = sw[1]; this.dx = sw[2]; this.dy = sw[3]; this.ph = sw[4];
    var x = s[0], y = s[1], z = s[2], V = s[3], g = s[4], psi = s[5], i, j;
    this.h0 = z - world.eta(x, y, t);
    // a crest ride with its family over the height: the member at the aircraft's height plus crestClimbM (clipped
    // to the family); it is steady, so it replaces the cycle
    this.member = null;
    var fam = crest && cycle.d.ride ? cycle.d.ride.family : null;
    if (fam && fam.length) {
      var cols = [];
      for (j = 0; j < 6; j++) cols.push(Float64Array.from(fam, function (row) { return row[j]; }));
      var hm = Math.min(Math.max(this.h0 + GUIDE.crestClimbM, cols[0][0]), cols[0][fam.length - 1]);
      var r = [];
      for (j = 0; j < 6; j++) r.push(interp(hm, cols[0], cols[j]));
      var sgn = cycle.mirror ? -1.0 : 1.0;
      this.member = [r[0], r[2], r[3], wrap(sgn * r[4]), r[1], r[5]];   // h, V, gamma, psi, theta, cl
    }
    var thA = this.thetaWorld(x, y, t);
    var T = cycle.T, n = 160, step = T / n, best = Infinity, bi = 0, hw = o.hintWindow === undefined ? 0.2 : o.hintWindow;
    var hint = o.tauHint === undefined ? null : o.tauHint, psiRel = wrap(psi - psiW);
    for (i = 0; i < n; i++) {
      var tau = 0 + i * step;
      if (hint !== null && Math.abs(wrap((tau - hint) * 2 * PI / T)) > hw * 2 * PI) continue;
      var st = this._state(tau);
      var d = sq((st[0] - this.h0) / this.sig[0]) + sq((st[1] - V) / this.sig[1]) + sq((st[2] - g) / this.sig[2]) +
        sq(wrap(st[3] - psiRel) / this.sig[3]) + sq(wrap(this._theta(tau) - thA) / this.sigTheta);
      if (d < best) { best = d; bi = i; }
    }
    this.tau0 = 0 + bi * step;
    this.matchDist = best;
    this.thetaErr0 = wrap(thA - this._theta(this.tau0));     // aircraft minus cycle
    this.matchState = this.matchDist - sq(this.thetaErr0 / this.sigTheta);
    this.delta = 0.0; this.thetaLow = null;
    this.tauS = this.tau0;
    var te = this.tauS + horizon;
    this.target = this._state(te);
    this.thetaT = this._theta(te);
    this.dEc = this.member ? 0.0 : cycle.energy(te) - cycle.energy(this.tauS);
    this.E0 = G * this.h0 + 0.5 * V * V;
  }
  WaveCycleGuide.prototype._state = function (tau) {
    return this.member ? this.member.slice(0, 4) : this.cycle.state(tau);
  };
  WaveCycleGuide.prototype._theta = function (tau) {
    return this.member ? this.member[4] : thetaAt(this.cycle, tau);
  };
  WaveCycleGuide.prototype.thetaWorld = function (x, y, t) {
    return this.k * (this.dx * x + this.dy * y) - this.om * t + this.ph;
  };
  WaveCycleGuide.prototype.nominal = function (knotsT) {
    var n = knotsT.length, out = new Float64Array(2 * n), j;
    if (this.member) { for (j = 0; j < n; j++) out[j] = this.member[5]; return out; }
    for (j = 0; j < n; j++) {
      var c = this.cycle.controls(this.tauS + knotsT[j]);
      out[j] = c[0]; out[n + j] = c[1];
    }
    return out;
  };
  WaveCycleGuide.prototype.nominalThrust = function (knotsT) {
    var n = knotsT.length, out = new Float64Array(n);
    if (this.member) return out;
    for (var j = 0; j < n; j++) out[j] = this.cycle.thrustAt(this.tauS + knotsT[j]);
    return out;
  };
  WaveCycleGuide.prototype._d2 = function (world, t, s, ref, thetaRef) {
    var h = s[2] - world.eta(s[0], s[1], t), sg = this.sig;
    var d2 = sq((h - ref[0]) / sg[0]) + sq((s[3] - ref[1]) / sg[1]) + sq((s[4] - ref[2]) / sg[2]) +
      sq(wrap(s[5] - this.psiW - ref[3]) / sg[3]) + sq(wrap(this.thetaWorld(s[0], s[1], t) - thetaRef) / this.sigTheta);
    this._h = h;
    return d2;
  };
  // wRun times the squared distance from the cycle at the same time (a cost per second)
  WaveCycleGuide.prototype.runningCost = function (world, t, s) {
    var tau = this.tauS + (t - this.tPlan);
    return this.wRun * this._d2(world, t, s, this._state(tau), this._theta(tau));
  };
  WaveCycleGuide.prototype.terminalCost = function (world, tEnd, s, planner) {
    var d2 = this._d2(world, tEnd, s, this.target, this.thetaT), h = this._h, V = s[3];
    var deficit = Math.max(0.0, this.dEc - (G * h + 0.5 * V * V - this.E0));
    return this.wD * d2 + planner.wE * deficit;
  };

  // ---------------------------------------------------------------------------- the pilot
  // sim.wavelib.WavePilot: the wave library's entry for the aircraft, the preset, the world's swell, the wind and
  // the course; null where the world has no swell or the swell does not pay. entries: expanded entries (any seas);
  // o: policy ("energy" or "course"), flat ([[u10, least power W, kind], ...] of the aircraft's flat motor
  // library, sim.export_wave_library.flat_references), sig, sigTheta, wRun, wD.
  // Ties of the nearest sea or wind go to the lower value (Python's set order decides them; the viewer's choices
  // lie on the grid and its winds are whole m/s, where there are none).
  function nearest(vals, x) {
    var u = vals.filter(function (v, i) { return vals.indexOf(v) === i; }).sort(function (p, q) { return p - q; });
    var best = u[0];
    for (var i = 1; i < u.length; i++) if (Math.abs(u[i] - x) < Math.abs(best - x)) best = u[i];
    return best;
  }
  function WavePilot(entries, preset, o) {
    o = o || {};
    this.entries = entries.filter(function (e) { return e.success; });
    this.preset = preset; this.policy = o.policy || 'energy';
    this.sig = o.sig || null;
    this.sigTheta = o.sigTheta === undefined || o.sigTheta === null ? null : +o.sigTheta;
    this.wRun = o.wRun === undefined || o.wRun === null ? GUIDE.wRun : +o.wRun;
    this.wD = o.wD || null;
    this.flat = o.flat || null;
    this.cache = [];
    this.last = null; this.lastMirror = false; this.refused = false;
  }
  // [entries, mirror] of the library sea and wind nearest to sea = [hs, tp, rel deg] and u10
  WavePilot.prototype.seaEntries = function (name, sea, u10) {
    var hs = sea[0], tp = sea[1], rel = sea[2], pre = this.preset;
    var es = this.entries.filter(function (e) { return e.aircraft === name && e.preset === pre; });
    if (!es.length) return [[], false];
    var h = nearest(es.map(function (e) { return e.hs; }), hs);
    var p = nearest(es.filter(function (e) { return e.hs === h; }).map(function (e) { return e.tp; }), tp);
    es = es.filter(function (e) { return e.hs === h && e.tp === p; });
    var mirror = rel < -1e-6 && Math.abs(Math.abs(rel) - 180.0) > 1e-6;
    var u = nearest(es.map(function (e) { return e.u10_flown; }), u10);
    es = es.filter(function (e) { return e.u10_flown === u; });
    if (u <= CALM_U10 + 1e-9) return [es, false];   // calm is solved once; the direction does not matter
    var d = nearest(es.map(function (e) { return e.rel_deg; }), Math.abs(rel));
    return [es.filter(function (e) { return e.rel_deg === d; }), mirror];
  };
  WavePilot.prototype.choose = function (es, courseRel) {
    if (!es.length) return null;
    var i, best = null;
    if (this.policy === 'course') {
      var key = function (e) { return Math.abs(pymod(e.course_rel_deg - courseRel + 180.0, 360.0) - 180.0); };
      for (i = 0; i < es.length; i++) {
        if (best === null || key(es[i]) < key(best) || (key(es[i]) === key(best) && es[i].power_avg_w < best.power_avg_w)) best = es[i];
      }
      return best;
    }
    var soar = es.filter(function (e) { return e.kind === 'soaring'; });
    if (soar.length) {
      var crest = soar.filter(function (e) { return e.crest; });
      if (crest.length) {
        for (i = 0; i < crest.length; i++) if (best === null || Math.abs(crest[i].ride.v_c) < Math.abs(best.ride.v_c)) best = crest[i];
        return best;
      }
      var hmin = function (e) { return (e.replay && e.replay.rc_h_min) || 0.0; };
      for (i = 0; i < soar.length; i++) if (best === null || hmin(soar[i]) > hmin(best)) best = soar[i];
      return best;
    }
    for (i = 0; i < es.length; i++) if (best === null || es[i].power_avg_w < best.power_avg_w) best = es[i];
    return best;
  };
  // the flat-sea reference power of the aircraft at the flat library's wind nearest u10
  WavePilot.prototype.flatPower = function (u10) {
    if (!this.flat || !this.flat.length) return null;
    var u = nearest(this.flat.map(function (r) { return r[0]; }), u10);
    for (var i = 0; i < this.flat.length; i++) if (this.flat[i][0] === u) return this.flat[i][1];
    return null;
  };
  // the cycle to fly (a waveCycle), or null without a swell or where the swell does not pay (chi: the course asked,
  // rad from the wind's direction psiW)
  WavePilot.prototype.cycle = function (name, world, u10, chi, psiW) {
    this.refused = false;
    var sea = seaOf(world, psiW);
    if (sea === null || sea[0] <= 0.0) return null;
    var r = this.seaEntries(name, sea, u10), es = r[0], mirror = r[1];
    var sgn = mirror ? -1.0 : 1.0;
    var courseRel = pymod(sgn * (degrees(chi) - sea[2]) + 180.0, 360.0) - 180.0;
    if (es.length && es[0].direction !== 'across' && courseRel < 0.0) {
      // along, against and calm seas are symmetric about the swell's travel: a course to the right of it flies
      // the mirrored entry of the course to the left
      mirror = !mirror; courseRel = -courseRel;
    }
    var e = this.choose(es, courseRel);
    if (e === null) return null;
    if (e.kind === 'motor' && this.flat) {
      var fp = this.flatPower(u10);
      if (fp !== null && e.power_avg_w >= fp) { this.last = null; this.refused = true; return null; }   // fly the flat flight
    }
    var c = null;
    for (var i = 0; i < this.cache.length; i++) if (this.cache[i][0] === e && this.cache[i][1] === mirror) c = this.cache[i][2];
    if (c === null) { c = waveCycle(e, mirror); this.cache.push([e, mirror, c]); }
    this.last = e; this.lastMirror = mirror;
    return c;
  };

  // ---------------------------------------------------------------------------- the flat start
  // sim.wavelib.flat_start_entry: the navigation's steady motor leg (level at 10 m above the local surface, the
  // airspeed of least electric energy per metre) along the course (deg from the wind's direction), as an entry.
  // NAV: navigator.js; ac: MPC.makeAircraft.
  function flatStartEntry(NAV, ac, u10, courseDeg) {
    var sm = new NAV.SteadyModel(NAV.steadyParams(ac, MPC.MOTOR));
    var c = radians(courseDeg);
    var lg = sm.leg(Math.max(+u10, CALM_U10), [Math.cos(c), Math.sin(c)]);
    if (!lg) return null;
    return NAV.steadyEntry({ steady: { V: lg.V, cl: lg.cl, thrust: lg.thrust, heading: lg.heading, height: sm.height },
      vx: lg.ground[0], vy: lg.ground[1], power: lg.power });
  }

  // ---------------------------------------------------------------------------- the closed loop's reference
  // The wave library in the loop (sim.mpc.fly_mpc with wave_library and start_entry): the first plan flies the
  // start's flat flight; later plans fly the pilot's wave cycle, or the start where the swell does not pay; a change
  // onto a wave cycle (from the flat flight or another wave cycle) waits until the wave phase at the aircraft is
  // within GUIDE.switchTheta of the cycle's, at most GUIDE.switchMaxWaitS. o: settings (planner, as mpc.js),
  // actuators (library block), preset, flat (the pilot's), startEntry, policy.
  function WaveController(acParams, entries, o) {
    o = o || {};
    this.ac = MPC.makeAircraft(acParams);
    this.name = acParams.name;
    this.pl = new MPC.Planner(this.ac, o.settings);
    var A = o.actuators;
    this.pl.act = A ? MPC.actuatorParams(this.ac, { rateModel: A.rate_model, tau: A.tau_s, clRate: A.cl_rate_per_s,
      kPb2v: A.k_pb2v, delay: A.delay_s, thrustTau: A.thrust_tau_s }) : null;
    this.pilot = new WavePilot(entries, o.preset, { flat: o.flat, policy: o.policy });
    this.start = waveCycle(o.startEntry, false);
    this.reset();
  }
  WaveController.prototype.reset = function () {
    this.pl.reset(); this.prev = null; this.tauPrev = null; this.switchWaitT = null; this.switches = [];
  };
  WaveController.prototype.guideFor = function (cyc, world, t, s, psiW, hint) {
    var pl = this.pl, pi = this.pilot;
    if (cyc.wave) {
      return new WaveCycleGuide(cyc, world, t, s, psiW, pl.horizonS, { wD: pi.wD || pl.wD, sig: pi.sig,
        sigTheta: pi.sigTheta, tauHint: hint, wRun: pi.wRun });
    }
    return new MPC.Guide(cyc, world, t, s, psiW, pl.horizonS, pl.wD, hint, undefined, null);
  };
  // req: {t, s, act: {cl, mu, clCmd, muCmd, T} | null, world: spec, u10 (as flown), chi (rad, course asked)}
  WaveController.prototype.plan = function (req) {
    var pl = this.pl, world = MPC.makeWorld(req.world), w = req.world.wind;
    var psiW = Math.atan2(w.ey, w.ex), cyc, wc = null, swell = worldSwell(world) !== null;
    if (this.prev === null) cyc = this.start;          // the first plan flies the start: entry from flat flight
    else {
      wc = swell ? this.pilot.cycle(this.name, world, req.u10, req.chi, psiW) : null;
      cyc = wc !== null ? wc : this.start;             // the swell does not pay: stay on the start's flat flight
    }
    var prev = this.prev, changed = prev !== null && (cyc.d !== prev.d || cyc.mirror !== prev.mirror);
    var waiting = false, probeErr = null;
    if (changed && cyc.wave) {
      // onto a wave cycle: the wave phase at the aircraft must match the cycle's, the rest the planner can fix
      var probe = this.guideFor(cyc, world, req.t, req.s, psiW, null);
      probeErr = probe.thetaErr0;
      if (this.switchWaitT === null) this.switchWaitT = req.t;
      var far = Math.abs(probe.thetaErr0) > GUIDE.switchTheta;
      if (far && req.t - this.switchWaitT < GUIDE.switchMaxWaitS) { cyc = prev; waiting = true; }
      else this.switchWaitT = null;
    } else this.switchWaitT = null;
    if (prev !== null && (cyc.d !== prev.d || cyc.mirror !== prev.mirror)) {
      this.tauPrev = null;                               // another cycle: its phase is matched afresh
      if (cyc.wave) this.switches.push([req.t, cyc.d.course_rel_deg, !!cyc.d.crest, cyc.d.kind]);
    }
    this.prev = cyc;
    var guide = this.guideFor(cyc, world, req.t, req.s, psiW, this.tauPrev === null ? null : this.tauPrev + pl.replanS);
    this.tauPrev = guide.tau0;
    var a = req.act;
    pl.actState = a ? [a.cl, a.mu, a.clCmd, a.muCmd].concat(pl.motor && a.T !== undefined && a.T !== null ? [a.T] : []) : null;
    if (!a && pl.motor) {   // the very first plan: the actuators start at the reference's own controls and thrust
      var n0 = guide.nominal(pl.knotsT), th0 = guide.nominalThrust(pl.knotsT);
      pl.actState = [n0[0], n0[pl.nCtrl], n0[0], n0[pl.nCtrl], th0[0]];
    }
    var plan = pl.plan(world, req.t, req.s, guide);
    plan.t0 = req.t;
    var d = cyc.d, sg = cyc.mirror ? -1 : 1;
    plan.info = { tau0: guide.tau0, period: cyc.T, kind: d.kind || 'motor', powerW: d.power_avg_w || 0,
      ref: cyc === this.start ? 'start' : d.crest ? 'crest' : 'cycle', mirror: cyc.mirror,
      courseRelDeg: d.course_rel_deg === undefined ? null : sg * d.course_rel_deg,
      vc: d.crest && d.ride ? sg * d.ride.v_c : null, rideH: guide.member ? guide.member[0] : null,
      u10Lib: d.u10_flown === undefined ? null : d.u10_flown, hsLib: d.hs === undefined ? null : d.hs,
      tpLib: d.tp === undefined ? null : d.tp, dirLib: d.direction || null,
      thetaErr: guide.thetaErr0 === undefined ? null : guide.thetaErr0, matchDist: guide.matchDist,
      waiting: waiting, waitS: waiting ? req.t - this.switchWaitT : 0, probeErr: probeErr,
      refused: wc === null && this.pilot.refused, switches: this.switches.length,
      tEntry: this.switches.length ? this.switches[0][0] : null,
      phase: phaseAt(world, req.s[0], req.s[1], req.t) };
    return plan;
  };

  // ---------------------------------------------------------------------------- worker
  // The worker body: a WaveController (init with wave: true) or the flat mpc.js Controller per 'init', a plan per
  // 'plan' message.
  function workerMain(MPC_, WAVE_) {
    var ctl = null;
    self.onmessage = function (e) {
      var m = e.data;
      try {
        if (m.type === 'init') {
          ctl = m.wave ? new WAVE_.WaveController(m.ac, m.entries, { settings: m.settings, actuators: m.actuators,
            preset: m.preset, flat: m.flat, startEntry: m.startEntry })
            : new MPC_.Controller(m.ac, m.entries, { settings: m.settings, actuators: m.actuators });
          self.postMessage({ type: 'ready' });
        } else if (m.type === 'reset') {
          if (ctl) ctl.reset();
        } else if (m.type === 'plan') {
          var t0 = performance.now();
          var p = ctl.plan(m.req);
          var ms = performance.now() - t0;
          self.postMessage({ type: 'plan', id: m.id, t0: p.t0, ms: ms, knotsT: p.knotsT, cl: p.cl,
            mu: p.mu, th: p.th, path: p.path, dt: ctl.pl.dt, info: p.info, ess: p.ess, nDead: p.nDead });
        }
      } catch (err) {
        self.postMessage({ type: 'error', message: String(err && err.stack || err) });
      }
    };
  }

  return { GUIDE: GUIDE, SWELL_DIRS: SWELL_DIRS, CALM_U10: CALM_U10, pymod: pymod, coareWind: coareWind,
    swellSea: swellSea, dominantWave: dominantWave, worldSwell: worldSwell, seaOf: seaOf, phaseAt: phaseAt,
    expandEntry: expandEntry, waveCycle: waveCycle, thetaAt: thetaAt, WaveCycleGuide: WaveCycleGuide,
    WavePilot: WavePilot, flatStartEntry: flatStartEntry, WaveController: WaveController, workerMain: workerMain };
}

(function () {
  'use strict';
  var M = globalThis.MPC || (typeof require !== 'undefined' ? require('./mpc.js') : null);
  var WAVE = waveFactory(M);
  // Source text of a worker that holds the planner and the wave pilot (for a Blob URL; works from file://). It
  // also serves the flat-sea Controller of mpc.js, so the page needs only this one worker.
  WAVE.workerSource = function () {
    return mpcFactory.toString() + '\n' + waveFactory.toString() + '\n(function () { var M = mpcFactory(); (' +
      WAVE.workerMain.toString() + ')(M, waveFactory(M)); })();\n';
  };
  // the file key of one sea and aircraft (sim.export_wave_library.file_key)
  WAVE.fileKey = function (preset, hs, tp, aircraft) {
    var f = function (v) { return String(v).replace('.', 'p'); };
    return preset + '_hs' + f(hs) + '_tp' + f(tp) + '_' + aircraft;
  };
  globalThis.WAVE = WAVE;
  if (typeof module !== 'undefined' && module.exports) module.exports = WAVE;
})();
