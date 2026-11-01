// World seas and winds for the viewer's map, ported from sim/world.py: decode the packed files, interpolate the
// corridor at a point and time (space bilinear, time linear, directions by unit vectors), map latitude and
// longitude to the local plane (azimuthal equidistant about a centre), turn a sample into the free setting's
// sea (EnvSea), and fetch the forecast "now" from Open-Meteo in batches of locations.
// Classic script: attaches WorldData to globalThis, and sets module.exports under node (which needs seagen.js
// next to it for seaSet).
(function (root) {
  'use strict';
  const R = 6371.0088;   // km, as sim.world.R_EARTH_KM
  const D2R = Math.PI / 180;
  const SG = root.SeaGen || (typeof require !== 'undefined' ? require('./seagen.js') : null);

  // ------------------------------------------------------------ decoding
  function unb64(s) {
    if (typeof atob === 'function') {
      const b = atob(s), out = new Uint8Array(b.length);
      for (let i = 0; i < b.length; i++) out[i] = b.charCodeAt(i);
      return out;
    }
    return new Uint8Array(Buffer.from(s, 'base64'));
  }
  function dequant(sc, c) {
    if (sc.signed) { const s = c > 127 ? c - 256 : c; return s === -128 ? NaN : s * sc.step; }
    return c === 255 ? NaN : c * sc.step;
  }
  // A packed field of sim.world.pack_field: values as Float64Arrays (corridor: frame-major, nt * np; global:
  // the ocean points) and, for the global field, the mask of the grid's ocean points.
  function unpack(d) {
    const f = {};
    for (const k of Object.keys(d)) if (!['data', 'mask', 'scales', 'layout', 'vars', 'clipped'].includes(k)) f[k] = d[k];
    const raw = unb64(d.data), names = d.vars;
    f.vals = {};
    if (d.kind === 'global') {
      const n = d.nlat * d.nlon, mb = unb64(d.mask), mask = new Uint8Array(n);
      let m = 0;
      for (let i = 0; i < n; i++) { mask[i] = (mb[i >> 3] >> (i & 7)) & 1; m += mask[i]; }
      f.mask = mask; f.nOcean = m;
      names.forEach((nm, v) => {
        const a = new Float64Array(m), sc = d.scales[nm];
        for (let i = 0; i < m; i++) a[i] = dequant(sc, raw[v * m + i]);
        f.vals[nm] = a;
      });
      f.grid = oceanGrid(f);
    } else {
      const np = d.na * d.nc, nv = names.length;
      names.forEach((nm, v) => {
        const a = new Float64Array(d.nt * np), sc = d.scales[nm];
        for (let k = 0; k < d.nt; k++) for (let i = 0; i < np; i++) a[k * np + i] = dequant(sc, raw[(k * nv + v) * np + i]);
        f.vals[nm] = a;
      });
    }
    return f;
  }
  // the global values on the whole grid (NaN on land), for sampling and drawing
  function oceanGrid(g) {
    const n = g.nlat * g.nlon, out = {};
    for (const nm of Object.keys(g.vals)) {
      const a = new Float64Array(n).fill(NaN);
      let m = 0;
      for (let i = 0; i < n; i++) if (g.mask[i]) a[i] = g.vals[nm][m++];
      out[nm] = a;
    }
    return out;
  }
  // the land rings of land.js ([x0, y0, dx1, dy1, ...] integers of `scale` deg) as [[lon, lat], ...] arrays
  function decodeLand(L) {
    const s = L.scale;
    return L.rings.map((a) => {
      let x = a[0], y = a[1];
      const out = [[x * s, y * s]];
      for (let i = 2; i < a.length; i += 2) { x += a[i]; y += a[i + 1]; out.push([x * s, y * s]); }
      return out;
    });
  }
  // ------------------------------------------------------------ packing (as sim.world.pack_field), to save a forecast
  const SCALES = {
    u10: { step: 0.25, signed: true, unit: 'm/s' }, v10: { step: 0.25, signed: true, unit: 'm/s' },
    hs: { step: 0.06, unit: 'm' }, hsw: { step: 0.06, unit: 'm' }, tsw: { step: 0.1, unit: 's' },
    dsw: { step: 360 / 255, wrap: 360, unit: 'deg' }
  };
  function quantize(nm, x) {
    const q = SCALES[nm];
    if (!Number.isFinite(x)) return q.signed ? 128 : 255;
    if (q.wrap) return ((Math.floor((((x % q.wrap) + q.wrap) % q.wrap) / q.step + 0.5) % 255) + 255) % 255;
    const r = Math.floor(x / q.step + 0.5);
    return q.signed ? Math.min(127, Math.max(-127, r)) & 0xFF : Math.min(254, Math.max(0, r));
  }
  function b64(bytes) {
    if (typeof btoa === 'function') { let s = ''; for (let i = 0; i < bytes.length; i++) s += String.fromCharCode(bytes[i]); return btoa(s); }
    return Buffer.from(bytes).toString('base64');
  }
  function pack(f) {
    const names = Object.keys(f.vals), d = {};
    for (const k of Object.keys(f)) if (!['vals', 'mask', 'grid', 'nOcean', '_pts', '_outline'].includes(k)) d[k] = f[k];
    d.vars = names;
    d.scales = {};
    for (const nm of names) d.scales[nm] = Object.assign({}, SCALES[nm]);
    if (f.kind === 'global') {
      const n = f.nlat * f.nlon, mb = new Uint8Array(Math.ceil(n / 8)), m = f.vals[names[0]].length, out = new Uint8Array(m * names.length);
      for (let i = 0; i < n; i++) if (f.mask[i]) mb[i >> 3] |= 1 << (i & 7);
      names.forEach((nm, v) => { for (let i = 0; i < m; i++) out[v * m + i] = quantize(nm, f.vals[nm][i]); });
      d.layout = 'var, point (ocean points in row-major grid order)'; d.mask = b64(mb); d.data = b64(out);
    } else {
      const np = f.na * f.nc, nv = names.length, out = new Uint8Array(f.nt * nv * np);
      for (let k = 0; k < f.nt; k++) names.forEach((nm, v) => { for (let i = 0; i < np; i++) out[(k * nv + v) * np + i] = quantize(nm, f.vals[nm][k * np + i]); });
      d.layout = 'frame, var, point (i * nc + j)'; d.data = b64(out);
    }
    d.clipped = {};
    return d;
  }
  // the JSON object of a world file's text (the lines after the one that ends with " =")
  function parseFile(text) {
    const lines = text.split('\n');
    const k = lines.findIndex((ln) => /\s=\s*$/.test(ln));
    if (k < 0) return JSON.parse(text);
    return JSON.parse(lines.slice(k + 1).join('\n').trim().replace(/;$/, ''));
  }

  // ------------------------------------------------------------ geometry
  function aeqdFwd(lat0, lon0, lat, lon) {
    const p0 = lat0 * D2R, p = lat * D2R, dl = (lon - lon0) * D2R;
    const cc = Math.min(1, Math.max(-1, Math.sin(p0) * Math.sin(p) + Math.cos(p0) * Math.cos(p) * Math.cos(dl)));
    const c = Math.acos(cc);
    const k = c < 1e-12 ? 1 : c / Math.sin(c);
    return [R * k * Math.cos(p) * Math.sin(dl), R * k * (Math.cos(p0) * Math.sin(p) - Math.sin(p0) * Math.cos(p) * Math.cos(dl))];
  }
  function aeqdInv(lat0, lon0, x, y) {
    const p0 = lat0 * D2R, rho = Math.hypot(x, y);
    if (rho < 1e-12) return [lat0, ((lon0 + 180) % 360 + 360) % 360 - 180];
    const c = rho / R;
    const lat = Math.asin(Math.min(1, Math.max(-1, Math.cos(c) * Math.sin(p0) + y * Math.sin(c) * Math.cos(p0) / rho)));
    const lon = lon0 * D2R + Math.atan2(x * Math.sin(c), rho * Math.cos(p0) * Math.cos(c) - y * Math.sin(p0) * Math.sin(c));
    return [lat / D2R, (((lon / D2R + 180) % 360) + 360) % 360 - 180];
  }
  function axes(b) { const r = b * D2R; return [[Math.sin(r), Math.cos(r)], [Math.cos(r), -Math.sin(r)]]; }
  function corridorPoints(c) {
    const [[ax, ay], [cx, cy]] = axes(c.bearing_deg), n = c.na * c.nc;
    const lat = new Float64Array(n), lon = new Float64Array(n);
    for (let i = 0; i < c.na; i++) for (let j = 0; j < c.nc; j++) {
      const s = (i - (c.na - 1) / 2) * c.step_km, q = (j - (c.nc - 1) / 2) * c.step_km;
      const p = aeqdInv(c.lat, c.lon, s * ax + q * cx, s * ay + q * cy);
      lat[i * c.nc + j] = p[0]; lon[i * c.nc + j] = p[1];
    }
    return { lat, lon };
  }
  function corridorIndex(c, lat, lon) {
    const [[ax, ay], [cx, cy]] = axes(c.bearing_deg), xy = aeqdFwd(c.lat, c.lon, lat, lon);
    return [(xy[0] * ax + xy[1] * ay) / c.step_km + (c.na - 1) / 2, (xy[0] * cx + xy[1] * cy) / c.step_km + (c.nc - 1) / 2];
  }
  // the corridor's outline (points along its edge, padKm outside the outer points), for drawing
  function corridorOutline(c, per, padKm) {
    per = per || 8; padKm = padKm || 0;
    const [[ax, ay], [cx, cy]] = axes(c.bearing_deg), hs = (c.na - 1) / 2 * c.step_km + padKm, hq = (c.nc - 1) / 2 * c.step_km + padKm, out = [];
    const edge = (s0, q0, s1, q1) => { for (let k = 0; k < per; k++) { const f = k / per, s = s0 + (s1 - s0) * f, q = q0 + (q1 - q0) * f; out.push(aeqdInv(c.lat, c.lon, s * ax + q * cx, s * ay + q * cy)); } };
    edge(-hs, -hq, hs, -hq); edge(hs, -hq, hs, hq); edge(hs, hq, -hs, hq); edge(-hs, hq, -hs, -hq);
    out.push(out[0]);
    return out;
  }

  // ------------------------------------------------------------ interpolation (as sim.world)
  function bilinear(get, n0, n1, fi, fj) {
    fi = Math.min(Math.max(fi, 0), n0 - 1); fj = Math.min(Math.max(fj, 0), n1 - 1);
    const i = n0 > 1 ? Math.min(Math.floor(fi), n0 - 2) : 0, j = n1 > 1 ? Math.min(Math.floor(fj), n1 - 2) : 0;
    const a = fi - i, b = fj - j;
    let tot = 0, ws = 0;
    const cs = [[0, 0, (1 - a) * (1 - b)], [1, 0, a * (1 - b)], [0, 1, (1 - a) * b], [1, 1, a * b]];
    for (const [di, dj, w] of cs) {
      const v = get(Math.min(i + di, n0 - 1), Math.min(j + dj, n1 - 1));
      if (w > 0 && Number.isFinite(v)) { tot += w * v; ws += w; }
    }
    if (ws > 1e-12) return tot / ws;
    let s = 0, n = 0;   // on a missing node: the mean of the cell's valid corners
    for (const [di, dj] of cs) { const v = get(Math.min(i + di, n0 - 1), Math.min(j + dj, n1 - 1)); if (Number.isFinite(v)) { s += v; n++; } }
    return n ? s / n : NaN;
  }
  function timeWeights(f, t) {
    const x = Math.min(Math.max((t - f.t0) / (3600 * f.dt_h), 0), f.nt - 1);
    const k = f.nt > 1 ? Math.min(Math.floor(x), f.nt - 2) : 0;
    return [k, x - k];
  }
  function sampleCorridor(f, lat, lon, t) {
    const [fi, fj] = corridorIndex(f, lat, lon), [k, ft] = timeWeights(f, t);
    const na = f.na, nc = f.nc, np = na * nc;
    const out = { inside: fi >= -1e-9 && fi <= na - 1 + 1e-9 && fj >= -1e-9 && fj <= nc - 1 + 1e-9, fi, fj };
    const at = (a, kk, fn) => bilinear((i, j) => (fn ? fn(a[kk * np + i * nc + j]) : a[kk * np + i * nc + j]), na, nc, fi, fj);
    for (const nm of Object.keys(f.vals)) {
      if (nm === 'dsw') continue;
      const a = f.vals[nm], va = at(a, k);
      if (f.nt === 1 || ft === 0) { out[nm] = va; continue; }
      const vb = at(a, k + 1);
      out[nm] = !Number.isFinite(va) ? vb : !Number.isFinite(vb) ? va : va + (vb - va) * ft;
    }
    if (f.vals.dsw) {
      const a = f.vals.dsw, sn = (d) => Math.sin(d * D2R), cs = (d) => Math.cos(d * D2R);
      let sx = 0, sy = 0;
      for (const [kk, w] of [[k, 1 - ft], [Math.min(k + 1, f.nt - 1), ft]]) {
        if (w <= 0) continue;
        const gx = at(a, kk, sn), gy = at(a, kk, cs);
        if (Number.isFinite(gx)) { sx += w * gx; sy += w * gy; }
      }
      out.dsw = (sx || sy) ? ((Math.atan2(sx, sy) / D2R) + 360) % 360 : NaN;
    }
    return out;
  }
  function sampleGlobal(g, lat, lon) {
    const fi = (lat - g.lat0) / g.step_deg, fj = (((lon - g.lon0) % 360) + 360) % 360 / g.step_deg, out = {};
    for (const nm of Object.keys(g.grid)) {
      const a = g.grid[nm];
      out[nm] = bilinear((i, j) => a[i * g.nlon + (j % g.nlon)], g.nlat, g.nlon + 1, fi, fj);
    }
    return out;
  }
  // the corridor's values at frame time t (no space interpolation): {name: Float64Array(np)}, for drawing
  function corridorAt(f, t) {
    const [k, ft] = timeWeights(f, t), np = f.na * f.nc, out = {};
    for (const nm of Object.keys(f.vals)) {
      const a = f.vals[nm], o = new Float64Array(np);
      for (let i = 0; i < np; i++) {
        const va = a[k * np + i], vb = f.nt > 1 ? a[(k + 1) * np + i] : va;
        if (nm === 'dsw') {
          const x = (1 - ft) * Math.sin(va * D2R) + ft * Math.sin(vb * D2R), y = (1 - ft) * Math.cos(va * D2R) + ft * Math.cos(vb * D2R);
          o[i] = Number.isFinite(va) && Number.isFinite(vb) ? ((Math.atan2(x, y) / D2R) + 360) % 360 : (Number.isFinite(va) ? va : vb);
        } else o[i] = !Number.isFinite(va) ? vb : !Number.isFinite(vb) ? va : va + (vb - va) * ft;
      }
      out[nm] = o;
    }
    return out;
  }

  // ------------------------------------------------------------ wind and the 3D sea
  function windFrom(u, v) { return [Math.hypot(u, v), ((Math.atan2(-u, -v) / D2R) + 360) % 360]; }
  function windUV(speed, fromDeg) { const r = fromDeg * D2R; return [-speed * Math.sin(r), -speed * Math.cos(r)]; }
  const U_MIN = 0.5;          // m/s: the sea generator needs some wind
  const SWELL_MIN = 0.05;     // m: below this the swell is left out
  const H_MIN = 0.1, H_MAX = 72;   // h: the range of the development searched
  // Hours since the wind began whose duration-limited wind sea (seagen.durationLimited) has the height hsWind;
  // 0 (fully developed, as EnvSea reads it) when hsWind reaches the fully developed sea or is unknown.
  function developHours(u10, fromDeg, hsWind, so) {
    if (!Number.isFinite(hsWind)) return 0;
    const w = SG.wind(u10, fromDeg, 'charnock', so);
    if (hsWind >= SG.fullyDeveloped(w, so).hs) return 0;
    if (SG.durationLimited(w, H_MAX, so).hs < hsWind) return 0;
    let lo = H_MIN, hi = H_MAX;
    if (SG.durationLimited(w, lo, so).hs >= hsWind) return lo;
    for (let n = 0; n < 60; n++) { const m = Math.sqrt(lo * hi); if (SG.durationLimited(w, m, so).hs < hsWind) lo = m; else hi = m; }
    return Math.sqrt(lo * hi);
  }
  // The free setting's set (sim/viewer/envsea.js) from a corridor sample: the 10 m wind, the wind sea grown for
  // the hours that give the height left after the swell (sqrt(Hs^2 - Hswell^2)), and the swell.
  function seaSet(s, consts, seed) {
    const so = consts ? { karman: consts.karman, charnock: consts.charnock, nuAir: consts.nu_air } : undefined;
    const wf = windFrom(s.u10, s.v10), u10 = Math.max(U_MIN, wf[0]);
    const hsw = Number.isFinite(s.hsw) ? s.hsw : 0;
    const hsWind = Number.isFinite(s.hs) ? Math.sqrt(Math.max(0, s.hs * s.hs - hsw * hsw)) : NaN;
    const swell = hsw >= SWELL_MIN && Number.isFinite(s.tsw) && Number.isFinite(s.dsw)
      ? { hs: hsw, tp: Math.min(25, Math.max(4, s.tsw)), fromDeg: s.dsw } : null;
    return { u10, fromDeg: wf[1], hours: developHours(u10, wf[1], hsWind, so), swell, seed: seed === undefined ? 1 : seed,
      hsWind, hsTotal: s.hs, u10Data: wf[0] };
  }

  // Keep every wave component's phase continuous at the point (x, y) (m) and sea time t when the sea is rebuilt
  // from a slightly different sample: phase' += theta(x, y, t) - theta'(x, y, t). Only when both sets have n
  // components (the same trains, so the same order); returns whether it matched.
  function matchPhases(old, nw, n, x, y, t) {
    if (nw.a.length !== n) return false;
    for (let i = 0; i < n; i++) {
      const th0 = old.k[i] * (old.dx[i] * x + old.dy[i] * y) - old.omega[i] * t + old.phase[i];
      const th1 = nw.k[i] * (nw.dx[i] * x + nw.dy[i] * y) - nw.omega[i] * t + nw.phase[i];
      const d = th0 - th1;
      nw.phase[i] += d - 2 * Math.PI * Math.round(d / (2 * Math.PI));
    }
    return true;
  }

  // ------------------------------------------------------------ now: the forecast from Open-Meteo
  const FORECAST_URL = 'https://api.open-meteo.com/v1/forecast', MARINE_URL = 'https://marine-api.open-meteo.com/v1/marine';
  const DATASETS = {
    ifs_wind: { url: FORECAST_URL, model: 'ecmwf_ifs025', vars: ['wind_speed_10m', 'wind_direction_10m'], wind: true },
    mfwam_waves: { url: MARINE_URL, model: 'meteofrance_wave', vars: ['wave_height', 'swell_wave_height', 'swell_wave_period', 'swell_wave_direction'] },
    mfwam_waves_global: { url: MARINE_URL, model: 'meteofrance_wave', vars: ['wave_height'] }
  };
  const BATCH = 100, PACE_PER_MIN = 400;
  const fmt = (x) => (+x.toFixed(4)).toString();
  function dateUTC(t) { return new Date(t * 1000).toISOString().slice(0, 10); }
  function requestUrl(key, lats, lons, d0, d1) {
    const ds = DATASETS[key];
    let q = 'latitude=' + Array.from(lats, fmt).join(',') + '&longitude=' + Array.from(lons, fmt).join(',') +
      '&hourly=' + ds.vars.join(',') + '&models=' + ds.model + '&start_date=' + d0 + '&end_date=' + d1 + '&timezone=GMT&timeformat=unixtime';
    if (ds.wind) q += '&wind_speed_unit=ms&cell_selection=sea';
    return ds.url + '?' + q;
  }
  function series(loc, v, model) {
    const h = loc.hourly, a = h[v] !== undefined ? h[v] : h[v + '_' + model];
    return a ? a.map((x) => (x === null ? NaN : x)) : h.time.map(() => NaN);
  }
  function atTimes(loc, v, model, times) {
    const a = series(loc, v, model), idx = new Map(loc.hourly.time.map((t, i) => [t, i]));
    return times.map((t) => (idx.has(t) ? a[idx.get(t)] : NaN));
  }
  // Fetch a dataset for many locations, BATCH per request, paced under PACE_PER_MIN locations a minute
  // (each location is at least one call of the free API). opts: {fetch, sleep, onProgress(done, total), log}
  async function fetchLocations(key, lats, lons, d0, d1, opts) {
    opts = opts || {};
    const get = opts.fetch || root.fetch.bind(root), sleep = opts.sleep || ((ms) => new Promise((r) => setTimeout(r, ms)));
    const out = [], urls = [], n = lats.length;
    for (let b = 0; b < n; b += BATCH) {
      const la = lats.slice(b, b + BATCH), lo = lons.slice(b, b + BATCH), url = requestUrl(key, la, lo, d0, d1);
      let body = null;
      for (let attempt = 0; attempt < 4 && body === null; attempt++) {
        const r = await get(url);
        if (r.status === 429 || r.status >= 500) { await sleep(65000); continue; }
        if (!r.ok) throw new Error(key + ': HTTP ' + r.status + ' ' + (await r.text()).slice(0, 200));
        body = await r.json();
      }
      if (body === null) throw new Error(key + ': no answer after retries');
      if (body.error) throw new Error(key + ': ' + body.reason);
      const locs = Array.isArray(body) ? body : [body];
      if (locs.length !== la.length) throw new Error(key + ': ' + locs.length + ' locations for ' + la.length);
      out.push(...locs); urls.push({ url, retrieved_utc: new Date().toISOString().slice(0, 19) + 'Z' });
      if (opts.onProgress) opts.onProgress(Math.min(n, b + BATCH), n);
      if (b + BATCH < n) await sleep(60000 * la.length / ((root.WorldData && root.WorldData.PACE_PER_MIN) || PACE_PER_MIN));
    }
    return { locs: out, urls };
  }
  function buildCorridor(spec, t0, wl, ml, frames) {
    frames = frames || { dt_h: 3, nt: 25 };
    const np = spec.na * spec.nc, nt = frames.nt, times = [];
    for (let k = 0; k < nt; k++) times.push(t0 + 3600 * frames.dt_h * k);
    const vals = {};
    for (const nm of ['u10', 'v10', 'hs', 'hsw', 'tsw', 'dsw']) vals[nm] = new Float64Array(nt * np).fill(NaN);
    const wm = DATASETS.ifs_wind.model, mm = DATASETS.mfwam_waves.model;
    for (let i = 0; i < np; i++) {
      const sp = atTimes(wl[i], 'wind_speed_10m', wm, times), dr = atTimes(wl[i], 'wind_direction_10m', wm, times);
      const hs = atTimes(ml[i], 'wave_height', mm, times), hw = atTimes(ml[i], 'swell_wave_height', mm, times);
      const tw = atTimes(ml[i], 'swell_wave_period', mm, times), dw = atTimes(ml[i], 'swell_wave_direction', mm, times);
      for (let k = 0; k < nt; k++) {
        const uv = windUV(sp[k], dr[k]), o = k * np + i;
        vals.u10[o] = uv[0]; vals.v10[o] = uv[1]; vals.hs[o] = hs[k]; vals.hsw[o] = hw[k];
        vals.tsw[o] = Number.isFinite(hw[k]) ? tw[k] : NaN; vals.dsw[o] = Number.isFinite(hw[k]) ? dw[k] : NaN;
      }
    }
    return Object.assign({ kind: 'corridor', t0, time: new Date(t0 * 1000).toISOString().slice(0, 16) + 'Z', dt_h: frames.dt_h, nt,
      swell: 'model', vals }, spec);
  }
  // the global grid's ocean points from a mask (Uint8Array over the grid, row-major from the south-west)
  function oceanPoints(g, mask) {
    const lats = [], lons = [];
    for (let i = 0; i < g.nlat; i++) for (let j = 0; j < g.nlon; j++) if (mask[i * g.nlon + j]) { lats.push(g.lat0 + g.step_deg * i); lons.push(g.lon0 + g.step_deg * j); }
    return { lats, lons };
  }
  function buildGlobal(g, mask, t, wl, ml) {
    const m = wl.length, u = new Float64Array(m), v = new Float64Array(m), hs = new Float64Array(m);
    for (let i = 0; i < m; i++) {
      const sp = atTimes(wl[i], 'wind_speed_10m', DATASETS.ifs_wind.model, [t])[0], dr = atTimes(wl[i], 'wind_direction_10m', DATASETS.ifs_wind.model, [t])[0];
      const uv = windUV(sp, dr); u[i] = uv[0]; v[i] = uv[1];
      hs[i] = atTimes(ml[i], 'wave_height', DATASETS.mfwam_waves_global.model, [t])[0];
    }
    const f = Object.assign({}, g, { kind: 'global', t0: t, time: new Date(t * 1000).toISOString().slice(0, 16) + 'Z', mask, nOcean: m,
      vals: { u10: u, v10: v, hs } });
    f.grid = oceanGrid(f);
    return f;
  }
  // The forecast now: a corridor around (lat, lon) and, with opts.global = {grid, mask}, the global snapshot.
  // The coordinates go to Open-Meteo. Returns {corridor, global, sources}.
  async function fetchNow(lat, lon, opts) {
    opts = opts || {};
    const now = opts.now !== undefined ? opts.now : Date.now() / 1000;
    const t0 = Math.floor(now / 10800) * 10800, frames = opts.frames || { dt_h: 3, nt: 25 };
    const spec = Object.assign({ na: 19, nc: 11, step_km: 55.6, bearing_deg: 90 }, opts.spec || {}, { lat, lon });
    const pts = corridorPoints(spec), d0 = dateUTC(t0), d1 = dateUTC(t0 + 3600 * frames.dt_h * (frames.nt - 1));
    const prog = (stage) => (d, n) => { if (opts.onProgress) opts.onProgress(stage, d, n); };
    const lats = Array.from(pts.lat), lons = Array.from(pts.lon);
    const w = await fetchLocations('ifs_wind', lats, lons, d0, d1, Object.assign({}, opts, { onProgress: prog('corridorWind') }));
    const m = await fetchLocations('mfwam_waves', lats, lons, d0, d1, Object.assign({}, opts, { onProgress: prog('corridorWaves') }));
    const res = { corridor: buildCorridor(spec, t0, w.locs, m.locs, frames), sources: { corridor: w.urls.concat(m.urls) } };
    if (opts.global) {
      const gp = oceanPoints(opts.global.grid, opts.global.mask);
      const gw = await fetchLocations('ifs_wind', gp.lats, gp.lons, d0, d0, Object.assign({}, opts, { onProgress: prog('globalWind') }));
      const gm = await fetchLocations('mfwam_waves_global', gp.lats, gp.lons, d0, d0, Object.assign({}, opts, { onProgress: prog('globalWaves') }));
      res.global = buildGlobal(opts.global.grid, opts.global.mask, t0, gw.locs, gm.locs);
      res.sources.global = gw.urls.concat(gm.urls);
    }
    return res;
  }
  // calls of the free API that fetchNow will cost (each location at least one call)
  function nowCost(spec, nOcean) { const n = (spec ? spec.na * spec.nc : 19 * 11) * 2; return { corridor: n, global: (nOcean || 0) * 2 }; }

  const WorldData = { R, unb64, unpack, oceanGrid, parseFile, decodeLand, SCALES, quantize, pack, aeqdFwd, aeqdInv, corridorPoints, corridorIndex, corridorOutline,
    bilinear, sampleCorridor, sampleGlobal, corridorAt, windFrom, windUV, developHours, seaSet, matchPhases, U_MIN, SWELL_MIN,
    DATASETS, BATCH, PACE_PER_MIN, requestUrl, fetchLocations, buildCorridor, buildGlobal, oceanPoints, fetchNow, nowCost, dateUTC };
  root.WorldData = WorldData;
  if (typeof module !== 'undefined' && module.exports) module.exports = WorldData;
})(typeof window !== 'undefined' ? window : globalThis);
