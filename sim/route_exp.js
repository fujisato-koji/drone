// The experiments of sim.route in node: `python -m sim.route exp` (engine "node") sends one job per case, aircraft
// and battery on stdin, {case, aircraft, cap, pairs: [[start [lat, lon], destination [lat, lon], t0], ...]}, and this
// plans each pair with sim/viewer/route.js (the port of sim/route.py, sim/tests/test_route_js.py) from route_params.js,
// the case's world file, its solar layer and the land, and writes the plans as JSON on stdout.
'use strict';
const path = require('path');
const fs = require('fs');
const V = path.join(__dirname, 'viewer');
const WD = require(path.join(V, 'worlddata.js'));
const ROUTE = require(path.join(V, 'route.js'));

const job = JSON.parse(fs.readFileSync(0, 'utf8'));
const params = WD.parseFile(fs.readFileSync(path.join(V, 'route_params.js'), 'utf8'));
const land = WD.decodeLand(WD.parseFile(fs.readFileSync(path.join(V, 'world', 'land.js'), 'utf8')));
const file = WD.parseFile(fs.readFileSync(path.join(V, 'world', job.case + '.js'), 'utf8'));
const field = new ROUTE.CorridorField(WD.unpack(file.corridor));
const lp = path.join(V, 'route', 'solar_' + job.case + '.js');
const layer = fs.existsSync(lp) ? new ROUTE.SolarLayer(WD.parseFile(fs.readFileSync(lp, 'utf8'))) : null;
const st = params.settings;
const graph = ROUTE.corridorGraph(field, st, land);
const legs = new ROUTE.LegModel(params.aircraft[job.aircraft].hulls, params.aircraft[job.aircraft].steady, st);
const en = ROUTE.energyOf(params, job.aircraft, job.cap);
const tab = new ROUTE.Tables(graph, field, en, layer, { settings: st });
const out = [];
for (const p of job.pairs) {
  const t1 = Date.now();
  const r = ROUTE.plan(field, graph, legs, en, p[0], p[1], { t0: p[2], layer: layer, tables: tab, settings: st });
  r.wall_s = (Date.now() - t1) / 1000;
  out.push(r);
}
process.stdout.write(JSON.stringify(out));
