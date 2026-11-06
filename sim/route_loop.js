// The laps of sim.route in node: `python -m sim.route loop` (engine "node") sends one job per direction and group of
// start times on stdin, {fields, solar, points, dry, no_wait, sub_km, land, energy, hulls, steady, settings, starts,
// relax, max_pops, relax_pops} (the boxes with null for NaN), and this plans each start from node 0 to the last node
// of the route's path graph with sim/viewer/route.js (the port of sim/route.py, sim/tests/test_route_loop.py) and
// writes the plans as JSON on stdout (non-finite numbers as strings). It reads land.js when the job asks for the land
// test, and writes no file.
'use strict';
const path = require('path');
const fs = require('fs');
const V = path.join(__dirname, 'viewer');
const WD = require(path.join(V, 'worlddata.js'));
const ROUTE = require(path.join(V, 'route.js'));

const job = JSON.parse(fs.readFileSync(0, 'utf8'));
const st = job.settings;
const field = ROUTE.BoxField.join(job.fields);
const layer = job.solar && job.solar.length ? ROUTE.BoxSolar.join(job.solar) : null;
const land = job.land ? WD.decodeLand(WD.parseFile(fs.readFileSync(path.join(V, 'world', 'land.js'), 'utf8'))) : null;
const graph = ROUTE.pathGraph(field, job.points, job.sub_km, job.dry, land);
for (const e of job.no_wait) ROUTE.setNoWait(graph, e.months, e.vertices);
const legs = new ROUTE.LegModel(job.hulls, job.steady, st);
const en = job.energy;
const tab = new ROUTE.Tables(graph, field, en, layer, { settings: st });
const last = graph.lat.length - 1;

// the strict margins of the route found and the waypoint of the lowest night margin (sim.route.loop_lows)
function lows(r) {
  const R = en.reserve_wh;
  for (const k of ['route', 'closest']) {
    const rt = r[k];
    if (!rt) continue;
    if (k === 'route') {
      const m = ROUTE.strictMargin(graph, tab, en, rt.waypoints, rt.legs);
      rt.margins = { reserve_margin_wh: m[0], night_margin_wh: m[1], allowance_margin_wh: m[2] };
    }
    let best = null;
    for (const w of rt.waypoints) {
      const x = tab.flyOnMin(w.node, w.t, w.soc_wh) - R;
      if (best === null || x < best.night_margin_wh) best = { node: w.node, t: w.t, night_margin_wh: x };
    }
    rt.night_low = best;
  }
}

const out = [];
for (const t0 of job.starts) {
  const t1 = Date.now();
  const r = ROUTE.plan(field, graph, legs, en, null, null, { t0: t0, layer: layer, tables: tab, settings: st,
    relax: job.relax, maxPops: job.max_pops, relaxPops: job.relax_pops, startNode: 0, destNode: last });
  r.wall_s = (Date.now() - t1) / 1000;
  lows(r);
  out.push(r);
}
process.stdout.write(JSON.stringify(out, (k, v) => (typeof v === 'number' && !Number.isFinite(v)) ? String(v) : v));
