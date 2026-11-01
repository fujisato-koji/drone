// The viewer's small world map: an equirectangular canvas with the land of sim/viewer/world/land.js, the wave
// height and the wind arrows of the global snapshot, the finer field of the corridor at a chosen time, the chosen
// place, and the aircraft with its trail. Wheel to zoom, drag to pan, click to choose a place.
// Classic script for the browser: attaches WorldMap to window (uses WorldData from worlddata.js).
(function (root) {
  'use strict';
  const WD = root.WorldData;
  // wave height colour: the reference sequential blue, light (calm) to dark (high), on a square-root scale
  const RAMP = ['#cde2fb', '#b7d3f6', '#9ec5f4', '#86b6ef', '#6da7ec', '#5598e7', '#3987e5', '#2a78d6', '#256abf',
    '#1c5cab', '#184f95', '#104281', '#0d366b'];
  const HS_MAX = 12;
  const RGB = RAMP.map((h) => [parseInt(h.slice(1, 3), 16), parseInt(h.slice(3, 5), 16), parseInt(h.slice(5, 7), 16)]);
  function hsColor(hs) {
    if (!Number.isFinite(hs)) return null;
    const s = Math.sqrt(Math.min(Math.max(hs, 0), HS_MAX) / HS_MAX) * (RGB.length - 1);
    const i = Math.min(RGB.length - 2, Math.floor(s)), f = s - i, a = RGB[i], b = RGB[i + 1];
    return [Math.round(a[0] + (b[0] - a[0]) * f), Math.round(a[1] + (b[1] - a[1]) * f), Math.round(a[2] + (b[2] - a[2]) * f)];
  }
  const css = (c, al) => (al === undefined ? 'rgb(' + c.join(',') + ')' : 'rgba(' + c.join(',') + ',' + al + ')');
  const COL = { sea: '#eef1f4', land: '#dcd8cd', coast: '#9c978b', grat: 'rgba(82,81,78,0.18)', ink: '#0b0b0b', halo: 'rgba(252,252,251,0.95)',
    corridor: '#eb6834', pick: '#e87ba4', aircraft: '#d95926', trail: '#d95926', text: '#52514e' };

  function create(cv, opts) {
    opts = opts || {};
    const g = cv.getContext('2d');
    const st = { W: 0, H: 0, dpr: 1, lon0: 0, lat0: 0, k: 1, land: null, global: null, cor: null, t: 0, pick: null,
      aircraft: null, trail: [], drag: null, img: null, imgKey: null };
    function size() {
      const r = Math.min(root.devicePixelRatio || 1, 2), w = cv.clientWidth || cv.width, h = cv.clientHeight || cv.height;
      if (w !== st.W || h !== st.H || r !== st.dpr) { st.W = w; st.H = h; st.dpr = r; cv.width = w * r; cv.height = h * r; }
      g.setTransform(st.dpr, 0, 0, st.dpr, 0, 0);
    }
    // projection: x = (lon - lon0) k + W/2, y = (lat0 - lat) k + H/2, longitudes taken to the copy nearest the centre
    const wrapNear = (lon) => lon - 360 * Math.round((lon - st.lon0) / 360);
    const px = (lon) => (wrapNear(lon) - st.lon0) * st.k + st.W / 2;
    const py = (lat) => (st.lat0 - lat) * st.k + st.H / 2;
    const lonAt = (x) => ((((x - st.W / 2) / st.k + st.lon0) + 180) % 360 + 360) % 360 - 180;
    const latAt = (y) => st.lat0 - (y - st.H / 2) / st.k;
    function clampView() {
      const kmin = st.W / 360;
      st.k = Math.min(Math.max(st.k, kmin), 2e5);   // up to 2e5 px per degree: about half a metre per pixel
      const half = st.H / 2 / st.k;
      st.lat0 = Math.min(Math.max(st.lat0, -90 + half), 90 - half);
      if (half >= 90) st.lat0 = 0;
    }
    function fitBox(lat1, lat2, lon1, lon2, pad) {
      pad = pad === undefined ? 1.15 : pad;
      const dlon = Math.max(1e-4, lon2 - lon1), dlat = Math.max(1e-4, lat2 - lat1);
      st.lon0 = (lon1 + lon2) / 2; st.lat0 = (lat1 + lat2) / 2;
      st.k = Math.min(st.W / (dlon * pad), st.H / (dlat * pad));
      clampView();
    }
    function fit(what) {
      size();
      if (what === 'corridor' && st.cor) {
        const o = WD.corridorOutline(st.cor, 4), lons = o.map((p) => p[1] - 360 * Math.round((p[1] - st.cor.lon) / 360)), lats = o.map((p) => p[0]);
        fitBox(Math.min(...lats), Math.max(...lats), Math.min(...lons), Math.max(...lons));
      } else if (what === 'aircraft' && (st.aircraft || st.pick)) {
        const p = st.aircraft || st.pick, d = 3 / 111.2;   // a box of about 3 km
        fitBox(p.lat - d, p.lat + d, p.lon - d / Math.max(0.1, Math.cos(p.lat * Math.PI / 180)), p.lon + d / Math.max(0.1, Math.cos(p.lat * Math.PI / 180)), 1);
      } else { st.lon0 = 0; st.lat0 = 0; st.k = st.W / 360; clampView(); }
      draw();
    }
    // the global snapshot as a small image, one pixel per grid point, drawn smoothed
    function globalImage() {
      const G = st.global;
      if (!G) return null;
      if (st.imgKey === G) return st.img;
      const c = document.createElement('canvas'); c.width = G.nlon + 1; c.height = G.nlat;
      const cg = c.getContext('2d'), im = cg.createImageData(c.width, c.height);
      for (let i = 0; i < G.nlat; i++) for (let j = 0; j <= G.nlon; j++) {
        const col = hsColor(G.grid.hs[i * G.nlon + (j % G.nlon)]), o = 4 * ((G.nlat - 1 - i) * c.width + j);
        if (col) { im.data[o] = col[0]; im.data[o + 1] = col[1]; im.data[o + 2] = col[2]; im.data[o + 3] = 255; }
      }
      cg.putImageData(im, 0, 0);
      st.img = c; st.imgKey = G;
      return c;
    }
    function ring(r) {
      // a land ring in the copy nearest the view centre (rings never cross 180 deg in Natural Earth)
      let mn = 1e9, mx = -1e9;
      for (const p of r) { mn = Math.min(mn, p[0]); mx = Math.max(mx, p[0]); }
      const shifts = [-360, 0, 360].filter((s) => {
        const a = (mn + s - st.lon0) * st.k + st.W / 2, b = (mx + s - st.lon0) * st.k + st.W / 2;
        return b >= -2 && a <= st.W + 2;
      });
      for (const s of shifts) {
        g.moveTo((r[0][0] + s - st.lon0) * st.k + st.W / 2, py(r[0][1]));
        for (let i = 1; i < r.length; i++) g.lineTo((r[i][0] + s - st.lon0) * st.k + st.W / 2, py(r[i][1]));
        g.closePath();
      }
    }
    function arrow(x, y, u, v, len) {
      // an arrow towards where the wind blows, length len px; u east, v north
      const sp = Math.hypot(u, v);
      if (!(sp > 0.05)) return;
      const ex = u / sp, ey = -v / sp, x2 = x + ex * len, y2 = y + ey * len, hl = Math.min(4, 0.4 * len);
      for (const [w, c] of [[3, COL.halo], [1.2, COL.ink]]) {
        g.strokeStyle = c; g.lineWidth = w; g.beginPath();
        g.moveTo(x - ex * len * 0.5, y - ey * len * 0.5); g.lineTo(x2 - ex * len * 0.5, y2 - ey * len * 0.5);
        const hx = x2 - ex * len * 0.5, hy = y2 - ey * len * 0.5;
        g.moveTo(hx, hy); g.lineTo(hx - ex * hl - ey * hl * 0.6, hy - ey * hl + ex * hl * 0.6);
        g.moveTo(hx, hy); g.lineTo(hx - ex * hl + ey * hl * 0.6, hy - ey * hl - ex * hl * 0.6);
        g.stroke();
      }
    }
    function draw() {
      size();
      g.clearRect(0, 0, st.W, st.H);
      g.fillStyle = COL.sea; g.fillRect(0, 0, st.W, st.H);
      const G = st.global;
      // the global wave height
      const img = globalImage();
      if (img) {
        g.imageSmoothingEnabled = true;
        const lonL = G.lon0 - G.step_deg / 2, latT = G.lat0 + (G.nlat - 0.5) * G.step_deg;
        for (const s of [-360, 0, 360]) {
          const x0 = (lonL + s - st.lon0) * st.k + st.W / 2, w = (G.nlon + 1) * G.step_deg * st.k;
          if (x0 > st.W || x0 + w < 0) continue;
          g.drawImage(img, x0, py(latT), w, G.nlat * G.step_deg * st.k);
        }
      }
      // the corridor's cells at time t
      const C = st.cor;
      let cv0 = null, cPts = null;
      if (C) {
        cv0 = WD.corridorAt(C, st.t); cPts = C._pts || (C._pts = WD.corridorPoints(C));
        const half = C.step_km / 2;
        const ax = Math.sin(C.bearing_deg * Math.PI / 180), ay = Math.cos(C.bearing_deg * Math.PI / 180);
        for (let i = 0; i < C.na; i++) for (let j = 0; j < C.nc; j++) {
          const n = i * C.nc + j, col = hsColor(cv0.hs[n]);
          if (!col) continue;
          const s = (i - (C.na - 1) / 2) * C.step_km, q = (j - (C.nc - 1) / 2) * C.step_km;
          g.fillStyle = css(col);
          g.beginPath();
          [[-1, -1], [1, -1], [1, 1], [-1, 1]].forEach(([a, b], m) => {
            const ss = s + a * half, qq = q + b * half;
            const p = WD.aeqdInv(C.lat, C.lon, ss * ax + qq * ay, ss * ay - qq * ax);
            const X = px(p[1]), Y = py(p[0]);
            if (m === 0) g.moveTo(X, Y); else g.lineTo(X, Y);
          });
          g.closePath(); g.fill();
        }
      }
      // graticule
      const stepG = st.k > 40 ? 1 : st.k > 8 ? 5 : st.k > 3 ? 10 : 30;
      g.strokeStyle = COL.grat; g.lineWidth = 1; g.beginPath();
      const lonA = Math.floor((st.lon0 - st.W / 2 / st.k) / stepG) * stepG, lonB = st.lon0 + st.W / 2 / st.k;
      if (st.k < 2000) for (let lo = lonA; lo <= lonB; lo += stepG) { const X = (lo - st.lon0) * st.k + st.W / 2; g.moveTo(X, 0); g.lineTo(X, st.H); }
      const latA = Math.floor(latAt(st.H) / stepG) * stepG;
      if (st.k < 2000) for (let la = latA; la <= latAt(0); la += stepG) { const Y = py(la); g.moveTo(0, Y); g.lineTo(st.W, Y); }
      g.stroke();
      // land
      if (st.land) {
        g.fillStyle = COL.land; g.strokeStyle = COL.coast; g.lineWidth = 0.8;
        g.beginPath(); for (const r of st.land) ring(r); g.fill('evenodd'); g.stroke();
      }
      // wind arrows: the corridor's where it is large enough on the screen, else the global ones
      const cellPx = G ? G.step_deg * st.k : 0;
      const corPx = C ? C.step_km / 111.2 * st.k : 0;
      if (G && cellPx >= 5) {
        const every = cellPx < 9 ? 2 : 1, len = Math.min(cellPx * 0.9, 22);
        for (let i = 0; i < G.nlat; i += every) for (let j = 0; j < G.nlon; j += every) {
          const n = i * G.nlon + j;
          if (!G.mask[n]) continue;
          const la = G.lat0 + G.step_deg * i, lo = G.lon0 + G.step_deg * j, X = px(lo), Y = py(la);
          if (X < -20 || X > st.W + 20 || Y < -20 || Y > st.H + 20) continue;
          if (C && corPx >= 6) { const ij = WD.corridorIndex(C, la, lo); if (ij[0] >= -0.5 && ij[0] <= C.na - 0.5 && ij[1] >= -0.5 && ij[1] <= C.nc - 0.5) continue; }
          const u = G.grid.u10[n], v = G.grid.v10[n];
          arrow(X, Y, u, v, len * Math.min(1.4, Math.hypot(u, v) / 10));
        }
      }
      if (C) {
        if (corPx >= 6) {
          const every = corPx < 11 ? 2 : 1, len = Math.min(corPx * every * 0.9, 26);
          for (let i = 0; i < C.na; i += every) for (let j = 0; j < C.nc; j += every) {
            const n = i * C.nc + j, X = px(cPts.lon[n]), Y = py(cPts.lat[n]);
            arrow(X, Y, cv0.u10[n], cv0.v10[n], len * Math.min(1.4, Math.hypot(cv0.u10[n], cv0.v10[n]) / 10));
          }
        }
        g.strokeStyle = COL.corridor; g.lineWidth = 2; g.beginPath();
        const o = C._outline || (C._outline = WD.corridorOutline(C, 12, C.step_km / 2));   // around the cells
        o.forEach((p, m) => { const X = px(p[1]), Y = py(p[0]); if (m === 0) g.moveTo(X, Y); else g.lineTo(X, Y); });
        g.stroke();
      }
      // trail and aircraft
      if (st.trail.length > 1) {
        g.strokeStyle = COL.trail; g.lineWidth = 2; g.beginPath();
        st.trail.forEach((p, m) => { const X = px(p[1]), Y = py(p[0]); if (m === 0) g.moveTo(X, Y); else g.lineTo(X, Y); });
        g.stroke();
      }
      if (st.pick) {
        const X = px(st.pick.lon), Y = py(st.pick.lat);
        g.strokeStyle = COL.halo; g.lineWidth = 4; g.beginPath(); g.arc(X, Y, 6, 0, 2 * Math.PI); g.stroke();
        g.strokeStyle = COL.pick; g.lineWidth = 2; g.beginPath(); g.arc(X, Y, 6, 0, 2 * Math.PI);
        g.moveTo(X - 10, Y); g.lineTo(X - 3, Y); g.moveTo(X + 3, Y); g.lineTo(X + 10, Y);
        g.moveTo(X, Y - 10); g.lineTo(X, Y - 3); g.moveTo(X, Y + 3); g.lineTo(X, Y + 10); g.stroke();
      }
      if (st.aircraft) {
        const X = px(st.aircraft.lon), Y = py(st.aircraft.lat), h = st.aircraft.heading || 0;
        const ex = Math.sin(h), ey = -Math.cos(h);
        g.fillStyle = COL.aircraft; g.strokeStyle = COL.halo; g.lineWidth = 2; g.beginPath();
        g.moveTo(X + ex * 9, Y + ey * 9); g.lineTo(X - ex * 6 - ey * 5, Y - ey * 6 + ex * 5); g.lineTo(X - ex * 6 + ey * 5, Y - ey * 6 - ex * 5);
        g.closePath(); g.stroke(); g.fill();
      }
      // scale bar
      {
        const kmPerPx = 111.2 / st.k * Math.cos(Math.min(85, Math.abs(st.lat0)) * Math.PI / 180);
        const target = kmPerPx * 80, p10 = Math.pow(10, Math.floor(Math.log10(target))), nice = [1, 2, 5, 10].map((m) => m * p10).find((m) => m >= target * 0.5) || p10;
        const L = nice / kmPerPx, x = st.W - 12 - L, y = st.H - 10;
        g.strokeStyle = COL.ink; g.lineWidth = 1.5; g.beginPath(); g.moveTo(x, y - 4); g.lineTo(x, y); g.lineTo(x + L, y); g.lineTo(x + L, y - 4); g.stroke();
        g.fillStyle = COL.ink; g.font = '11px system-ui, sans-serif'; g.textAlign = 'center';
        g.fillText(nice >= 1 ? nice + ' km' : (nice * 1000) + ' m', x + L / 2, y - 6);
      }
    }
    // input: wheel zoom about the cursor, drag to pan, click to pick
    cv.addEventListener('wheel', (e) => {
      e.preventDefault();
      const r = cv.getBoundingClientRect(), x = e.clientX - r.left, y = e.clientY - r.top;
      const lo = (x - st.W / 2) / st.k + st.lon0, la = latAt(y);
      st.k *= Math.exp(-e.deltaY * 0.0015);
      clampView();
      st.lon0 = lo - (x - st.W / 2) / st.k; st.lat0 = la + (y - st.H / 2) / st.k;
      clampView(); draw();
    }, { passive: false });
    cv.addEventListener('pointerdown', (e) => { st.drag = { x: e.clientX, y: e.clientY, lon0: st.lon0, lat0: st.lat0, moved: false, t: performance.now() }; cv.setPointerCapture(e.pointerId); });
    cv.addEventListener('pointermove', (e) => {
      const r = cv.getBoundingClientRect(), x = e.clientX - r.left, y = e.clientY - r.top;
      if (st.drag) {
        const dx = e.clientX - st.drag.x, dy = e.clientY - st.drag.y;
        if (Math.hypot(dx, dy) > 3) st.drag.moved = true;
        if (st.drag.moved) { st.lon0 = st.drag.lon0 - dx / st.k; st.lat0 = st.drag.lat0 + dy / st.k; clampView(); draw(); }
      }
      if (opts.onHover) opts.onHover(latAt(y), lonAt(x), e.clientX, e.clientY);
    });
    cv.addEventListener('pointerleave', () => { if (opts.onHover) opts.onHover(null); });
    cv.addEventListener('pointerup', (e) => {
      const d = st.drag; st.drag = null;
      if (!d || d.moved) return;
      const r = cv.getBoundingClientRect(), la = latAt(e.clientY - r.top), lo = lonAt(e.clientX - r.left);
      if (la < -90 || la > 90) return;
      if (opts.onPick) opts.onPick(la, lo);
    });
    const api = {
      setLand(L) { st.land = L ? WD.decodeLand(L) : null; },
      setGlobal(G) { st.global = G; st.imgKey = null; },
      setCorridor(C) { st.cor = C; },
      setTime(t) { st.t = t; },
      setPick(p) { st.pick = p; },
      setAircraft(p) { st.aircraft = p; },
      setTrail(tr) { st.trail = tr || []; },
      fit, draw, hsColor, HS_MAX, RAMP,
      view() { return { lon0: st.lon0, lat0: st.lat0, k: st.k }; },
      project(lat, lon) { return [px(lon), py(lat)]; }
    };
    size(); st.k = st.W / 360;
    return api;
  }
  root.WorldMap = { create, hsColor, HS_MAX, RAMP };
})(window);
