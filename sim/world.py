"""World seas and winds for the viewer's map: fetch from Open-Meteo, quantize to one byte, pack small files.

    python -m sim.world land --src FILE
        simplify the Natural Earth 1:110m land polygons (GeoJSON) into sim/viewer/world/land.js.
    python -m sim.world scenarios [--only ID ...]
        fetch (cached) the ERA5 corridors and global snapshots of the historical scenarios and write
        sim/viewer/world/<id>.js and sim/viewer/world/index.js.
    python -m sim.world now --lat LAT --lon LON [--bearing DEG] [--no-global] [--out FILE]
        fetch the current forecast (ECMWF IFS 0.25 deg wind, Meteo-France MFWAM waves with the swell partition)
        for the global snapshot and a corridor around the point, and write one world file (default
        sim/viewer/world/now.js, which git ignores). A corridor near the owner's private sites is refused
        inside the repository; write it under the private store with --out.
    python -m sim.world sizes
        list the files in sim/viewer/world and their sizes.

Two fields, both one byte per value with fixed scales (QUANT):
- the global snapshot: GLOBAL grid, ocean points only (the land polygons decide), 10 m wind (u, v) and the
  significant wave height, one time;
- the corridor: a band of CORRIDOR points along a bearing around a centre, on a grid of the azimuthal
  equidistant projection about the centre, 10 m wind (u, v), total significant wave height, swell height,
  period and direction, every FRAMES["dt_h"] hours for FRAMES["nt"] frames.
The viewer interpolates the corridor at the aircraft's point and time (space bilinear, time linear,
directions by their unit vectors), as sample_corridor does here (sim/viewer/worlddata.js is the port).

ERA5 through Open-Meteo has no swell partition (sim.climate), so a historical corridor takes the swell height
from sim.climate.swell_lower_bound, the swell period from the peak period and the swell direction from the
mean wave direction of the whole spectrum ("swell": "lower_bound" in the file). The forecast's MFWAM gives a
swell partition ("swell": "model").

Open-Meteo's free API is for non-commercial use and counts each location of a request as at least one call
(600 a minute, 5,000 an hour, 10,000 a day per address); requests go in batches of locations, are paced
under LIMITS by a shared log in the cache, and every response is cached, so a run that stops resumes.
"""
from __future__ import annotations

import argparse
import base64
import datetime as _dt
import gzip
import hashlib
import json
import math
import sys
import time
import urllib.error
import urllib.parse
import urllib.request
from pathlib import Path

import numpy as np

from sim import record

ROOT = Path(__file__).resolve().parents[1]
OUT_DIR = ROOT / "sim" / "viewer" / "world"
CACHE_DIR = ROOT / "sim" / "cache" / "world"
DEFAULT_PRIVATE = ROOT.parent / "drone-private"
LAND_JS = OUT_DIR / "land.js"
MODEL_FILES = [Path(__file__).resolve()]

R_EARTH_KM = 6371.0088          # mean Earth radius (IUGG)
GLOBAL = {"lat0": -70.0, "lon0": -180.0, "step_deg": 5.0, "nlat": 29, "nlon": 72}
CORRIDOR = {"na": 19, "nc": 11, "step_km": 55.6, "bearing_deg": 90.0}   # 0.5 deg of latitude is 55.6 km
FRAMES = {"dt_h": 3, "nt": 25}
# step, signed (int8 with -128 missing) or unsigned (uint8 with 255 missing), wrap (directions: codes 0..254)
QUANT = {
    "u10": {"step": 0.25, "signed": True, "unit": "m/s"},
    "v10": {"step": 0.25, "signed": True, "unit": "m/s"},
    "hs": {"step": 0.06, "unit": "m"},
    "hsw": {"step": 0.06, "unit": "m"},
    "tsw": {"step": 0.1, "unit": "s"},
    "dsw": {"step": 360.0 / 255.0, "wrap": 360.0, "unit": "deg"},
}
GLOBAL_VARS = ("u10", "v10", "hs")
CORRIDOR_VARS = ("u10", "v10", "hs", "hsw", "tsw", "dsw")
PRIVATE_GUARD_KM = 1500.0       # a corridor centre this close to a private site is never written into the repository

# ---------------------------------------------------------------- services and datasets

FORECAST_URL = "https://api.open-meteo.com/v1/forecast"
ARCHIVE_URL = "https://archive-api.open-meteo.com/v1/archive"
MARINE_URL = "https://marine-api.open-meteo.com/v1/marine"
ATTRIBUTION = "Weather data by Open-Meteo.com (CC BY 4.0)"
ERA5_ATTRIBUTION = "Generated using Copernicus Climate Change Service information (ERA5, CC BY 4.0)"
LICENCE = "Open-Meteo API data CC BY 4.0 (https://open-meteo.com/en/licence), free API for non-commercial use only"
DATASETS = {
    "era5_wind": {"url": ARCHIVE_URL, "model": "era5", "vars": ("wind_speed_10m", "wind_direction_10m"),
                  "dataset": "ERA5 hourly 10 m wind (0.25 deg), Open-Meteo historical weather API",
                  "licence": LICENCE + "; ERA5 CC BY 4.0"},
    "era5_waves": {"url": MARINE_URL, "model": "era5_ocean",
                   "vars": ("wave_height", "wave_peak_period", "wave_direction"),
                   "dataset": "ERA5-Ocean hourly significant wave height, peak period and mean direction (0.5 deg), "
                              "Open-Meteo marine API", "licence": LICENCE + "; ERA5 CC BY 4.0"},
    "era5_waves_global": {"url": MARINE_URL, "model": "era5_ocean", "vars": ("wave_height",),
                          "dataset": "ERA5-Ocean hourly significant wave height (0.5 deg), Open-Meteo marine API",
                          "licence": LICENCE + "; ERA5 CC BY 4.0"},
    "ifs_wind": {"url": FORECAST_URL, "model": "ecmwf_ifs025", "vars": ("wind_speed_10m", "wind_direction_10m"),
                 "dataset": "ECMWF IFS 0.25 deg 10 m wind forecast, Open-Meteo forecast API",
                 "licence": LICENCE + "; ECMWF open data CC BY 4.0"},
    "mfwam_waves": {"url": MARINE_URL, "model": "meteofrance_wave",
                    "vars": ("wave_height", "swell_wave_height", "swell_wave_period", "swell_wave_direction"),
                    "dataset": "Meteo-France MFWAM wave forecast with the swell partition (0.08 deg, 3-hourly), "
                               "Open-Meteo marine API", "licence": LICENCE},
    "mfwam_waves_global": {"url": MARINE_URL, "model": "meteofrance_wave", "vars": ("wave_height",),
                           "dataset": "Meteo-France MFWAM significant wave height forecast (0.08 deg), "
                                      "Open-Meteo marine API", "licence": LICENCE},
}
BATCH = 100                     # locations per request (a URL stays under about 2.5 kB)
# the pace kept (the free limits are 600 a minute, 5,000 an hour, 10,000 a UTC day)
LIMITS = {"minute": 400.0, "hour": 4500.0, "day": 9500.0}
USER_AGENT = "drone-world/1 (personal research, non-commercial)"

# ---------------------------------------------------------------- historical scenarios
# Each corridor is FRAMES long from its start (UTC); the global snapshot is at the start. The global snapshots of
# a window come from one request per location (a location costs one call for up to 70 days of two variables),
# so the scenarios share two windows. Why each was picked: docs/02-form.md#world-map.
SCENARIOS = (
    {"id": "so_winter_low", "lat": -50.0, "lon": 58.0, "start": "2023-07-12T03:00Z", "window": "jul2023",
     "title": {"ja": "南大洋 冬 低気圧の通過", "en": "Southern Ocean, winter, a low passing"}},
    {"id": "so_summer_quiet", "lat": -45.0, "lon": 52.0, "start": "2022-01-02T18:00Z", "window": "jan2022",
     "title": {"ja": "南大洋 夏 風の弱い日々", "en": "Southern Ocean, summer, a quiet spell"}},
    {"id": "np_winter", "lat": 40.0, "lon": -170.0, "start": "2022-01-05T21:00Z", "window": "jan2022",
     "title": {"ja": "北太平洋 遷移帯 冬", "en": "North Pacific transition zone, winter"}},
    {"id": "np_summer_calm", "lat": 35.0, "lon": -150.0, "start": "2023-07-10T21:00Z", "window": "jul2023",
     "title": {"ja": "北太平洋 夏 亜熱帯高気圧の凪", "en": "North Pacific, summer, calm under the subtropical high"}},
    {"id": "trade_wind", "lat": -15.0, "lon": -10.0, "start": "2023-07-07T15:00Z", "window": "jul2023",
     "title": {"ja": "南大西洋 貿易風帯", "en": "South Atlantic trade-wind belt"}},
    {"id": "swell_no_wind", "lat": 8.0, "lon": -110.0, "start": "2023-07-13T06:00Z", "window": "jul2023",
     "title": {"ja": "東太平洋 赤道無風帯 風のないうねり", "en": "Eastern Pacific doldrums, swell without wind"}},
)
WINDOWS = {"jul2023": ("2023-07-07", "2023-07-13"), "jan2022": ("2022-01-02", "2022-01-05")}

# ---------------------------------------------------------------- time


def parse_utc(s) -> int:
    """Unix seconds of an ISO time such as 2023-07-12T03:00Z."""
    return int(_dt.datetime.strptime(s.replace("Z", ""), "%Y-%m-%dT%H:%M").replace(tzinfo=_dt.timezone.utc).timestamp())


def iso_utc(t) -> str:
    return _dt.datetime.fromtimestamp(int(t), _dt.timezone.utc).strftime("%Y-%m-%dT%H:%MZ")


def date_utc(t) -> str:
    return _dt.datetime.fromtimestamp(int(t), _dt.timezone.utc).strftime("%Y-%m-%d")


def now_utc() -> str:
    return _dt.datetime.now(_dt.timezone.utc).strftime("%Y-%m-%dT%H:%M:%SZ")


# ---------------------------------------------------------------- quantization


def quantize(name, x):
    """uint8 codes of the values x of variable name, and the number clipped to the range."""
    q = QUANT[name]
    x = np.asarray(x, dtype=float)
    miss = ~np.isfinite(x)
    xs = np.where(miss, 0.0, x)
    # round half up (floor(x / step + 0.5)), as worlddata.js does, so that both give the same codes
    if "wrap" in q:
        c = np.mod(np.floor(np.mod(xs, q["wrap"]) / q["step"] + 0.5), 255).astype(np.int64)
        clipped = 0
    elif q.get("signed"):
        r = np.floor(xs / q["step"] + 0.5)
        clipped = int(((r < -127) | (r > 127))[~miss].sum())
        c = np.clip(r, -127, 127).astype(np.int64) & 0xFF
    else:
        r = np.floor(xs / q["step"] + 0.5)
        clipped = int(((r < 0) | (r > 254))[~miss].sum())
        c = np.clip(r, 0, 254).astype(np.int64)
    c = np.where(miss, 128 if q.get("signed") else 255, c)
    return c.astype(np.uint8), clipped


def dequantize(name, codes):
    """Float values of the uint8 codes, nan where missing."""
    q = QUANT[name]
    c = np.asarray(codes, dtype=np.int64)
    if q.get("signed"):
        s = np.where(c > 127, c - 256, c)
        return np.where(s == -128, np.nan, s * q["step"])
    return np.where(c == 255, np.nan, c * q["step"])


# ---------------------------------------------------------------- geometry (sphere of R_EARTH_KM)


def aeqd_forward(lat0, lon0, lat, lon):
    """Azimuthal equidistant x (east) and y (north) in km of (lat, lon) about (lat0, lon0)."""
    p0, l0 = math.radians(lat0), math.radians(lon0)
    p, dl = np.radians(np.asarray(lat, float)), np.radians(np.asarray(lon, float)) - l0
    cc = np.clip(math.sin(p0) * np.sin(p) + math.cos(p0) * np.cos(p) * np.cos(dl), -1.0, 1.0)
    c = np.arccos(cc)
    k = np.where(c < 1e-12, 1.0, c / np.where(np.sin(c) < 1e-15, 1.0, np.sin(c)))
    x = R_EARTH_KM * k * np.cos(p) * np.sin(dl)
    y = R_EARTH_KM * k * (math.cos(p0) * np.sin(p) - math.sin(p0) * np.cos(p) * np.cos(dl))
    return x, y


def aeqd_inverse(lat0, lon0, x, y):
    """Latitude and longitude (deg, longitude in [-180, 180)) of the point x, y (km) about (lat0, lon0)."""
    p0, l0 = math.radians(lat0), math.radians(lon0)
    x, y = np.asarray(x, float), np.asarray(y, float)
    rho = np.hypot(x, y)
    c = rho / R_EARTH_KM
    safe = np.where(rho < 1e-12, 1.0, rho)
    lat = np.arcsin(np.clip(np.cos(c) * math.sin(p0) + y * np.sin(c) * math.cos(p0) / safe, -1.0, 1.0))
    lon = l0 + np.arctan2(x * np.sin(c), safe * math.cos(p0) * np.cos(c) - y * math.sin(p0) * np.sin(c))
    lat = np.where(rho < 1e-12, p0, lat)
    lon = np.where(rho < 1e-12, l0, lon)
    lon = np.mod(np.degrees(lon) + 180.0, 360.0) - 180.0
    return np.degrees(lat), lon


def _axes(bearing_deg):
    b = math.radians(bearing_deg)
    return (math.sin(b), math.cos(b)), (math.cos(b), -math.sin(b))   # along, and across to its right


def corridor_points(c):
    """Latitudes and longitudes of the corridor points, index i * nc + j (i along, j across to the right)."""
    (ax, ay), (cx, cy) = _axes(c["bearing_deg"])
    s = (np.arange(c["na"]) - (c["na"] - 1) / 2.0) * c["step_km"]
    q = (np.arange(c["nc"]) - (c["nc"] - 1) / 2.0) * c["step_km"]
    S, Q = np.meshgrid(s, q, indexing="ij")
    x, y = S * ax + Q * cx, S * ay + Q * cy
    lat, lon = aeqd_inverse(c["lat"], c["lon"], x.ravel(), y.ravel())
    return lat, lon


def corridor_index(c, lat, lon):
    """Fractional grid indices (along, across) of (lat, lon) in the corridor c."""
    (ax, ay), (cx, cy) = _axes(c["bearing_deg"])
    x, y = aeqd_forward(c["lat"], c["lon"], lat, lon)
    s, q = x * ax + y * ay, x * cx + y * cy
    return s / c["step_km"] + (c["na"] - 1) / 2.0, q / c["step_km"] + (c["nc"] - 1) / 2.0


def great_circle_km(lat1, lon1, lat2, lon2):
    p1, p2 = math.radians(lat1), math.radians(lat2)
    a = math.sin((p2 - p1) / 2) ** 2 + math.cos(p1) * math.cos(p2) * math.sin(math.radians(lon2 - lon1) / 2) ** 2
    return 2 * R_EARTH_KM * math.asin(min(1.0, math.sqrt(a)))


# ---------------------------------------------------------------- wind components


def wind_uv(speed, from_deg):
    """Eastward and northward 10 m wind from the speed and the direction it blows from."""
    r = np.radians(np.asarray(from_deg, float))
    s = np.asarray(speed, float)
    return -s * np.sin(r), -s * np.cos(r)


def wind_speed_from(u, v):
    return float(math.hypot(u, v)), float((math.degrees(math.atan2(-u, -v)) + 360.0) % 360.0)


# ---------------------------------------------------------------- interpolation


def _bilinear(grid, fi, fj):
    """Bilinear value of grid[i, j] at fractional (fi, fj), clamped to the grid; missing corners are left out
    and the weights of the others renormalized (nan when all four are missing)."""
    n0, n1 = grid.shape
    fi = min(max(fi, 0.0), n0 - 1.0)
    fj = min(max(fj, 0.0), n1 - 1.0)
    i, j = min(int(fi), n0 - 2) if n0 > 1 else 0, min(int(fj), n1 - 2) if n1 > 1 else 0
    a, b = fi - i, fj - j
    tot = wsum = 0.0
    for di, dj, w in ((0, 0, (1 - a) * (1 - b)), (1, 0, a * (1 - b)), (0, 1, (1 - a) * b), (1, 1, a * b)):
        ii, jj = min(i + di, n0 - 1), min(j + dj, n1 - 1)
        v = grid[ii, jj]
        if w > 0 and np.isfinite(v):
            tot += w * v
            wsum += w
    if wsum > 1e-12:
        return tot / wsum
    # on a missing node or edge: the mean of the cell's valid corners, as the nearest values
    vals = [grid[min(i + di, n0 - 1), min(j + dj, n1 - 1)] for di in (0, 1) for dj in (0, 1)]
    vals = [v for v in vals if np.isfinite(v)]
    return float(sum(vals) / len(vals)) if vals else float("nan")


def _time_weights(f, t):
    x = (t - f["t0"]) / (3600.0 * f["dt_h"])
    x = min(max(x, 0.0), f["nt"] - 1.0)
    k = min(int(x), f["nt"] - 2) if f["nt"] > 1 else 0
    return k, x - k


def sample_corridor(f, lat, lon, t):
    """The corridor's values at (lat, lon) and unix time t: space bilinear, time linear, directions by unit
    vectors. Adds "inside" (the point lies within the corridor's grid) and the indices fi, fj."""
    fi, fj = corridor_index(f, lat, lon)
    fi, fj = float(fi), float(fj)
    k, ft = _time_weights(f, t)
    na, nc = f["na"], f["nc"]
    out = {"inside": bool(-1e-9 <= fi <= na - 1 + 1e-9 and -1e-9 <= fj <= nc - 1 + 1e-9), "fi": fi, "fj": fj}

    def at(name, kk):
        return _bilinear(np.asarray(f["vals"][name][kk], float).reshape(na, nc), fi, fj)

    def lin(name):
        a = at(name, k)
        if f["nt"] == 1 or ft == 0:
            return a
        b = at(name, k + 1)
        if not np.isfinite(a):
            return b
        if not np.isfinite(b):
            return a
        return a + (b - a) * ft

    for name in f["vals"]:
        if name == "dsw":
            continue
        out[name] = lin(name)
    if "dsw" in f["vals"]:
        sx = sy = 0.0
        for kk, w in ((k, 1 - ft), (min(k + 1, f["nt"] - 1), ft)):
            if w <= 0:
                continue
            d = np.radians(np.asarray(f["vals"]["dsw"][kk], float)).reshape(na, nc)
            gx, gy = _bilinear(np.sin(d), fi, fj), _bilinear(np.cos(d), fi, fj)
            if np.isfinite(gx):
                sx += w * gx
                sy += w * gy
        out["dsw"] = float((math.degrees(math.atan2(sx, sy)) + 360.0) % 360.0) if (sx or sy) else float("nan")
    return out


def global_grid(g):
    """Latitudes and longitudes of the global grid, row-major from the south-west corner."""
    lat = g["lat0"] + g["step_deg"] * np.arange(g["nlat"])
    lon = g["lon0"] + g["step_deg"] * np.arange(g["nlon"])
    LA, LO = np.meshgrid(lat, lon, indexing="ij")
    return LA.ravel(), LO.ravel()


def sample_global(g, lat, lon):
    """The global snapshot's values at (lat, lon): bilinear over the ocean points, longitude wrapping."""
    fi = (lat - g["lat0"]) / g["step_deg"]
    fj = ((lon - g["lon0"]) % 360.0) / g["step_deg"]
    out = {}
    for name, v in g["vals"].items():
        full = np.full(g["nlat"] * g["nlon"], np.nan)
        full[g["mask"]] = v
        grid = full.reshape(g["nlat"], g["nlon"])
        grid = np.concatenate([grid, grid[:, :1]], axis=1)   # the wrap column
        out[name] = _bilinear(grid, fi, fj)
    return out


# ---------------------------------------------------------------- land


def _rdp(pts, tol):
    """Ramer-Douglas-Peucker simplification of an open polyline (list of (x, y))."""
    if len(pts) < 3:
        return pts
    keep = [False] * len(pts)
    keep[0] = keep[-1] = True
    stack = [(0, len(pts) - 1)]
    while stack:
        a, b = stack.pop()
        (x1, y1), (x2, y2) = pts[a], pts[b]
        dx, dy = x2 - x1, y2 - y1
        n = math.hypot(dx, dy)
        best, bi = -1.0, -1
        for i in range(a + 1, b):
            px, py = pts[i]
            d = abs(dy * (px - x1) - dx * (py - y1)) / n if n > 0 else math.hypot(px - x1, py - y1)
            if d > best:
                best, bi = d, i
        if best > tol:
            keep[bi] = True
            stack += [(a, bi), (bi, b)]
    return [p for p, k in zip(pts, keep) if k]


def simplify_land(geojson, tol_deg=0.05, scale=0.01, min_points=4):
    """Rings of the land polygons as lists of integer (lon, lat) / scale after RDP at tol_deg, closed."""
    rings = []
    for f in geojson["features"]:
        g = f["geometry"]
        polys = g["coordinates"] if g["type"] == "MultiPolygon" else [g["coordinates"]]
        for poly in polys:
            for ring in poly:
                pts = [(float(x), float(y)) for x, y in ring]
                if pts[0] == pts[-1]:
                    pts = pts[:-1]
                if len(pts) < 3:
                    continue
                # split the closed ring at its farthest pair so that RDP keeps both ends
                far = max(range(len(pts)), key=lambda i: (pts[i][0] - pts[0][0]) ** 2 + (pts[i][1] - pts[0][1]) ** 2)
                s = _rdp(pts[:far + 1], tol_deg)[:-1] + _rdp(pts[far:] + [pts[0]], tol_deg)[:-1]
                q = []
                for x, y in s:
                    p = (int(round(x / scale)), int(round(y / scale)))
                    if not q or q[-1] != p:
                        q.append(p)
                if len(q) >= min_points - 1:
                    rings.append(q)
    return rings


def encode_rings(rings):
    """Each ring as [x0, y0, dx1, dy1, ...] (integers)."""
    out = []
    for r in rings:
        a = [r[0][0], r[0][1]]
        for (x0, y0), (x1, y1) in zip(r, r[1:]):
            a += [x1 - x0, y1 - y0]
        out.append(a)
    return out


def decode_rings(enc, scale):
    rings = []
    for a in enc:
        x, y = a[0], a[1]
        pts = [(x * scale, y * scale)]
        for i in range(2, len(a), 2):
            x += a[i]
            y += a[i + 1]
            pts.append((x * scale, y * scale))
        rings.append(np.asarray(pts))
    return rings


def load_land(path=LAND_JS):
    d = load_js(path)
    return decode_rings(d["rings"], d["scale"])


def on_land(rings, lat, lon):
    """Boolean array: the points lie inside an odd number of land rings (even-odd rule, holes included)."""
    lat, lon = np.atleast_1d(np.asarray(lat, float)), np.atleast_1d(np.asarray(lon, float))
    inside = np.zeros(lat.shape, bool)
    for r in rings:
        x, y = r[:, 0], r[:, 1]
        x2, y2 = np.roll(x, -1), np.roll(y, -1)
        bx = (x.min(), x.max(), y.min(), y.max())
        cand = (lon >= bx[0]) & (lon <= bx[1]) & (lat >= bx[2]) & (lat <= bx[3])
        if not cand.any():
            continue
        px, py = lon[cand][:, None], lat[cand][:, None]
        cross = ((y > py) != (y2 > py)) & (px < (x2 - x) * (py - y) / np.where(y2 == y, 1e-30, y2 - y) + x)
        inside[cand] ^= (cross.sum(axis=1) % 2).astype(bool)
    return inside


def ocean_mask(rings, g=GLOBAL):
    lat, lon = global_grid(g)
    return ~on_land(rings, lat, lon)


def write_land(src, out=LAND_JS):
    raw = Path(src).read_bytes()
    gj = json.loads(raw)
    rings = simplify_land(gj)
    obj = {"source": {"dataset": "Natural Earth 1:110m physical vectors, land (ne_110m_land), version 5.1.2",
                      "url": "https://raw.githubusercontent.com/nvkelso/natural-earth-vector/v5.1.2/geojson/ne_110m_land.geojson",
                      "sha256": hashlib.sha256(raw).hexdigest(), "licence": "public domain (Natural Earth)",
                      "retrieved": "2026-11-01", "simplified": "Ramer-Douglas-Peucker 0.05 deg, 0.01 deg integers, delta coded"},
           "scale": 0.01, "rings": encode_rings(rings)}
    write_js(out, "window.WORLD_LAND", obj, "python -m sim.world land")
    return obj, sum(len(r) for r in rings), len(rings)


# ---------------------------------------------------------------- files


def b64(codes) -> str:
    return base64.b64encode(np.asarray(codes, np.uint8).tobytes()).decode("ascii")


def unb64(s):
    return np.frombuffer(base64.b64decode(s), dtype=np.uint8)


def pack_field(f):
    """The JSON-able form of a global or corridor field (values quantized, base64)."""
    names = list(f["vals"])
    codes, clipped = [], {}
    for n in names:
        c, k = quantize(n, np.asarray(f["vals"][n], float).ravel())
        codes.append(c)
        if k:
            clipped[n] = k
    meta = {k: v for k, v in f.items() if k not in ("vals", "mask")}
    meta["vars"] = names
    meta["scales"] = {n: {kk: vv for kk, vv in QUANT[n].items() if kk != "unit"} | {"unit": QUANT[n]["unit"]}
                      for n in names}
    meta["layout"] = ("var, point (ocean points in row-major grid order)" if f["kind"] == "global"
                      else "frame, var, point (i * nc + j)")
    if f["kind"] == "global":
        meta["mask"] = b64(np.packbits(np.asarray(f["mask"], bool), bitorder="little"))
        meta["data"] = b64(np.concatenate(codes))
    else:
        nt, npnt = f["nt"], f["na"] * f["nc"]
        arr = np.stack([c.reshape(nt, npnt) for c in codes], axis=1)    # frame, var, point
        meta["data"] = b64(arr.ravel())
    meta["clipped"] = clipped
    return meta


def unpack_field(d):
    f = {k: v for k, v in d.items() if k not in ("data", "mask", "scales", "layout", "vars", "clipped")}
    raw = unb64(d["data"])
    names = d["vars"]
    if d["kind"] == "global":
        n = d["nlat"] * d["nlon"]
        mask = np.unpackbits(unb64(d["mask"]), bitorder="little")[:n].astype(bool)
        m = int(mask.sum())
        f["mask"] = mask
        f["vals"] = {nm: dequantize(nm, raw[i * m:(i + 1) * m]) for i, nm in enumerate(names)}
    else:
        nt, npnt = d["nt"], d["na"] * d["nc"]
        arr = raw.reshape(nt, len(names), npnt)
        f["vals"] = {nm: dequantize(nm, arr[:, i, :]) for i, nm in enumerate(names)}
    return f


def write_js(path, target, obj, command):
    path = Path(path)
    path.parent.mkdir(parents=True, exist_ok=True)
    with open(path, "w", encoding="utf-8", newline="\n") as fh:
        fh.write(record.header(command, MODEL_FILES) + "\n")
        fh.write(target + " =\n" + json.dumps(obj, separators=(",", ":"), ensure_ascii=False) + ";\n")


def load_js(path):
    """The JSON object of a world file: the lines after the one that ends with " =" (the viewer reads a
    file the user picks in the same way)."""
    lines = Path(path).read_text(encoding="utf-8").split("\n")
    k = next(i for i, ln in enumerate(lines) if ln.rstrip().endswith(" ="))
    return json.loads("\n".join(lines[k + 1:]).strip().rstrip(";"))


def world_file_target(file_id):
    return "(window.WORLD_FILES = window.WORLD_FILES || {})[" + json.dumps(file_id) + "]"


# ---------------------------------------------------------------- fetching


class BudgetExceeded(RuntimeError):
    pass


def location_weight(n_vars, n_days):
    """Open-Meteo's weight of one location: max(1, V / 10 * max(1, D / 14))."""
    return max(1.0, n_vars / 10.0 * max(1.0, n_days / 14.0))


class Pacer:
    """Keeps the calls under LIMITS, with a log in the cache that every run shares."""

    def __init__(self, cache=CACHE_DIR, limits=None, clock=time.time, sleep=time.sleep):
        self.path = Path(cache) / "calls.jsonl"
        self.limits = dict(LIMITS, **(limits or {}))
        self.clock, self.sleep = clock, sleep

    def _log(self):
        if not self.path.exists():
            return []
        out = []
        for line in self.path.read_text(encoding="utf-8").splitlines():
            try:
                d = json.loads(line)
                out.append((float(d["t"]), float(d["cost"])))
            except (ValueError, KeyError):
                continue
        return out

    def used(self, now=None):
        now = self.clock() if now is None else now
        day0 = (now // 86400) * 86400
        log = self._log()
        return {"minute": sum(c for t, c in log if t > now - 60), "hour": sum(c for t, c in log if t > now - 3600),
                "day": sum(c for t, c in log if t >= day0)}

    def wait(self, cost):
        """Sleep until cost fits the minute and hour pace; raise BudgetExceeded when the UTC day would not hold it."""
        while True:
            u = self.used()
            if u["day"] + cost > self.limits["day"]:
                raise BudgetExceeded(f"the UTC day's pace of {self.limits['day']:.0f} calls would be passed "
                                     f"({u['day']:.0f} used, {cost:.0f} more); run again after 00:00 UTC (cached)")
            if u["minute"] + cost <= self.limits["minute"] and u["hour"] + cost <= self.limits["hour"]:
                return
            self.sleep(20.0 if u["hour"] + cost <= self.limits["hour"] else 120.0)

    def note(self, cost, url):
        self.path.parent.mkdir(parents=True, exist_ok=True)
        with open(self.path, "a", encoding="utf-8") as fh:
            fh.write(json.dumps({"t": round(self.clock(), 1), "cost": cost, "url": url[:120]}) + "\n")


def http_get(url, timeout=120):
    """Body and headers of a GET; raises urllib.error.HTTPError."""
    req = urllib.request.Request(url, headers={"User-Agent": USER_AGENT})
    with urllib.request.urlopen(req, timeout=timeout) as r:
        return r.read(), r.headers       # case-insensitive get


def request_url(ds_key, lats, lons, start_date, end_date):
    ds = DATASETS[ds_key]
    q = {"latitude": ",".join(f"{x:.4f}".rstrip("0").rstrip(".") for x in lats),
         "longitude": ",".join(f"{x:.4f}".rstrip("0").rstrip(".") for x in lons),
         "hourly": ",".join(ds["vars"]), "models": ds["model"], "start_date": start_date, "end_date": end_date,
         "timezone": "GMT", "timeformat": "unixtime"}
    if "wind" in ds_key:      # the wind of the sea cell, not of a nearby land cell
        q["wind_speed_unit"] = "ms"
        q["cell_selection"] = "sea"
    return ds["url"] + "?" + urllib.parse.urlencode(q, safe=",")


def fetch_locations(ds_key, lats, lons, start_date, end_date, cache=CACHE_DIR, pacer=None, get=None,
                    batch=BATCH, retries=5, log=print):
    """Per-location responses (dicts with latitude, longitude, hourly) in the order of lats, and the
    list of request records (url, retrieved, sha256, cached)."""
    get = get or http_get
    pacer = pacer or Pacer(cache)
    cache = Path(cache)
    ds = DATASETS[ds_key]
    days = (_dt.date.fromisoformat(end_date) - _dt.date.fromisoformat(start_date)).days + 1
    w = location_weight(len(ds["vars"]), days)
    out, reqs = [], []
    for b0 in range(0, len(lats), batch):
        la, lo = list(lats[b0:b0 + batch]), list(lons[b0:b0 + batch])
        url = request_url(ds_key, la, lo, start_date, end_date)
        key = hashlib.sha256(url.encode()).hexdigest()[:20]
        path = cache / "raw" / f"{ds_key}_{key}.json.gz"
        metap = path.with_name(path.name.replace(".json.gz", ".meta.json"))
        if path.exists():
            body = gzip.decompress(path.read_bytes())
            meta = json.loads(metap.read_text(encoding="utf-8"))
            meta["cached"] = True
        else:
            cost = w * len(la)
            for attempt in range(retries):
                pacer.wait(cost)
                try:
                    body, headers = get(url)
                    pacer.note(float(headers.get("X-RateLimit-Cost", cost)) if headers else cost, url)
                    d = json.loads(body)
                    if isinstance(d, dict) and d.get("error"):
                        raise RuntimeError(f"{ds_key}: {d.get('reason')}")
                    break
                except urllib.error.HTTPError as e:
                    msg = e.read()[:300].decode("utf-8", "replace")
                    pacer.note(0.0 if e.code == 429 else 1.0, url)
                    log(f"{ds_key}: HTTP {e.code} {msg}")
                    if e.code in (429, 502, 503, 504) and attempt + 1 < retries:
                        pacer.sleep(float(e.headers.get("Retry-After", 65)) if e.headers else 65.0)
                        continue
                    raise
            path.parent.mkdir(parents=True, exist_ok=True)
            path.write_bytes(gzip.compress(body, mtime=0))
            meta = {"url": url, "retrieved_utc": now_utc(), "dataset": ds["dataset"], "licence": ds["licence"],
                    "sha256": hashlib.sha256(body).hexdigest(), "bytes": len(body), "locations": len(la),
                    "cost_est": cost}
            metap.write_text(json.dumps(meta, indent=1), encoding="utf-8")
            meta["cached"] = False
            log(f"{ds_key}: {len(la)} locations, {len(body)} bytes")
        d = json.loads(body)
        locs = d if isinstance(d, list) else [d]
        if len(locs) != len(la):
            raise RuntimeError(f"{ds_key}: {len(locs)} locations returned for {len(la)}")
        out += locs
        meta["first"] = b0
        reqs.append(meta)
    return out, reqs


def hourly(loc, var, model):
    """The hourly series of var (nan for missing), with or without the model suffix."""
    h = loc["hourly"]
    v = h.get(var, h.get(f"{var}_{model}"))
    if v is None:
        return np.full(len(h["time"]), np.nan)
    return np.array([np.nan if x is None else x for x in v], dtype=float)


def at_times(loc, var, model, times):
    t = np.asarray(loc["hourly"]["time"], np.int64)
    v = hourly(loc, var, model)
    idx = {int(x): i for i, x in enumerate(t)}
    return np.array([v[idx[int(s)]] if int(s) in idx else np.nan for s in times])


# ---------------------------------------------------------------- building fields


def build_corridor(spec, t0, wind_locs, wave_locs, wave_ds, frames=FRAMES):
    """A corridor field from per-point wind and wave responses (lists in point order)."""
    nt = frames["nt"]
    times = [t0 + 3600 * frames["dt_h"] * k for k in range(nt)]
    npnt = spec["na"] * spec["nc"]
    vals = {n: np.full((nt, npnt), np.nan) for n in CORRIDOR_VARS}
    wm, mm = DATASETS["era5_wind" if wave_ds.startswith("era5") else "ifs_wind"]["model"], DATASETS[wave_ds]["model"]
    for i in range(npnt):
        sp = at_times(wind_locs[i], "wind_speed_10m", wm, times)
        dr = at_times(wind_locs[i], "wind_direction_10m", wm, times)
        u, v = wind_uv(sp, dr)
        vals["u10"][:, i], vals["v10"][:, i] = u, v
        hs = at_times(wave_locs[i], "wave_height", mm, times)
        vals["hs"][:, i] = hs
        if wave_ds.startswith("era5"):
            from sim.climate import swell_lower_bound
            vals["hsw"][:, i] = np.where(np.isfinite(hs), swell_lower_bound(np.nan_to_num(hs), sp), np.nan)
            vals["tsw"][:, i] = at_times(wave_locs[i], "wave_peak_period", mm, times)
            vals["dsw"][:, i] = at_times(wave_locs[i], "wave_direction", mm, times)
        else:
            vals["hsw"][:, i] = at_times(wave_locs[i], "swell_wave_height", mm, times)
            vals["tsw"][:, i] = at_times(wave_locs[i], "swell_wave_period", mm, times)
            vals["dsw"][:, i] = at_times(wave_locs[i], "swell_wave_direction", mm, times)
    # a swell direction or period without a swell height says nothing
    gone = ~np.isfinite(vals["hsw"])
    vals["tsw"][gone] = np.nan
    vals["dsw"][gone] = np.nan
    return {"kind": "corridor", "lat": spec["lat"], "lon": spec["lon"], "bearing_deg": spec["bearing_deg"],
            "na": spec["na"], "nc": spec["nc"], "step_km": spec["step_km"], "t0": int(t0), "time": iso_utc(t0),
            "dt_h": frames["dt_h"], "nt": nt, "swell": "lower_bound" if wave_ds.startswith("era5") else "model",
            "vals": vals}


def build_global(t, mask, wind_locs, wave_locs, wind_ds, wave_ds, g=GLOBAL):
    wm, mm = DATASETS[wind_ds]["model"], DATASETS[wave_ds]["model"]
    sp = np.array([at_times(x, "wind_speed_10m", wm, [t])[0] for x in wind_locs])
    dr = np.array([at_times(x, "wind_direction_10m", wm, [t])[0] for x in wind_locs])
    u, v = wind_uv(sp, dr)
    hs = np.array([at_times(x, "wave_height", mm, [t])[0] for x in wave_locs])
    return dict(g, kind="global", t0=int(t), time=iso_utc(t), mask=np.asarray(mask, bool),
                vals={"u10": u, "v10": v, "hs": hs})


def sources_of(reqs, points):
    """The requests behind a field, grouped by dataset: the URL without its coordinate lists (they are the field's
    points, `points` says which, in order, as request_url writes them), the licence, and for each request the first
    point and the number of points, the retrieval time and the sha256 of the response."""
    out = {}
    for r in reqs:
        u = urllib.parse.urlsplit(r["url"])
        q = [(k, v) for k, v in urllib.parse.parse_qsl(u.query) if k not in ("latitude", "longitude")]
        base = urllib.parse.urlunsplit((u.scheme, u.netloc, u.path, urllib.parse.urlencode(q, safe=","), ""))
        g = out.setdefault((r["dataset"], base), {"dataset": r["dataset"], "licence": r["licence"], "url": base,
                                                   "points": points, "requests": []})
        g["requests"].append([r.get("first", 0), r["locations"], r["retrieved_utc"], r["sha256"]])
    return list(out.values())


def source_urls(src, lats, lons):
    """The full request URLs of a sources_of group for the points (lats, lons) of its field."""
    out = []
    for first, n, _, _ in src["requests"]:
        la = ",".join(f"{x:.4f}".rstrip("0").rstrip(".") for x in lats[first:first + n])
        lo = ",".join(f"{x:.4f}".rstrip("0").rstrip(".") for x in lons[first:first + n])
        u = urllib.parse.urlsplit(src["url"])
        q = urllib.parse.urlencode([("latitude", la), ("longitude", lo)] + urllib.parse.parse_qsl(u.query), safe=",")
        out.append(urllib.parse.urlunsplit((u.scheme, u.netloc, u.path, q, "")))
    return out


CORRIDOR_POINTS = "the corridor's points in the order i * nc + j (sim.world.corridor_points)"
GLOBAL_POINTS = "the ocean points of the global grid in row-major order from the south-west (sim.world.global_grid)"


def scenario_spec(s):
    return dict(CORRIDOR, lat=s["lat"], lon=s["lon"])


def make_scenarios(only=None, cache=CACHE_DIR, out_dir=OUT_DIR, pacer=None, get=None, log=print):
    """Fetch (cached) and write the historical scenarios; returns the index object."""
    rings = load_land()
    mask = ocean_mask(rings)
    glat, glon = global_grid(GLOBAL)
    glat, glon = glat[mask], glon[mask]
    todo = [s for s in SCENARIOS if not only or s["id"] in only]
    cors = {}
    for s in todo:          # the corridors first (the smaller requests), then the global windows
        t0 = parse_utc(s["start"])
        spec = scenario_spec(s)
        plat, plon = corridor_points(spec)
        d0, d1 = date_utc(t0), date_utc(t0 + 3600 * FRAMES["dt_h"] * (FRAMES["nt"] - 1))
        cw, cwr = fetch_locations("era5_wind", plat, plon, d0, d1, cache, pacer, get, log=log)
        cm, cmr = fetch_locations("era5_waves", plat, plon, d0, d1, cache, pacer, get, log=log)
        cors[s["id"]] = (build_corridor(spec, t0, cw, cm, "era5_waves"), cwr + cmr)
    win_data = {}
    for wkey in sorted({s["window"] for s in todo}):
        d0, d1 = WINDOWS[wkey]
        wl, wr = fetch_locations("era5_wind", glat, glon, d0, d1, cache, pacer, get, log=log)
        ml, mr = fetch_locations("era5_waves_global", glat, glon, d0, d1, cache, pacer, get, log=log)
        win_data[wkey] = (wl, ml, wr + mr)
    for s in todo:
        cor, creqs = cors[s["id"]]
        wl, ml, greqs = win_data[s["window"]]
        glob = build_global(cor["t0"], mask, wl, ml, "era5_wind", "era5_waves_global")
        obj = {"id": s["id"], "kind": "scenario", "title": s["title"], "corridor": pack_field(cor),
               "global": pack_field(glob),
               "attribution": [ATTRIBUTION, ERA5_ATTRIBUTION],
               "sources": {"corridor": sources_of(creqs, CORRIDOR_POINTS), "global": sources_of(greqs, GLOBAL_POINTS)}}
        write_js(Path(out_dir) / f"{s['id']}.js", world_file_target(s["id"]), obj,
                 f"python -m sim.world scenarios --only {s['id']}")
        log(f"wrote {s['id']}.js")
    return write_index(out_dir)


def write_index(out_dir=OUT_DIR):
    """sim/viewer/world/index.js: the scenarios present, with their file's sha256 and a short summary."""
    out_dir = Path(out_dir)
    items = []
    for s in SCENARIOS:
        p = out_dir / f"{s['id']}.js"
        if not p.exists():
            continue
        d = load_js(p)
        c = d["corridor"]
        items.append({"id": s["id"], "file": p.name, "sha": hashlib.sha256(p.read_bytes()).hexdigest()[:12],
                      "title": s["title"], "lat": s["lat"], "lon": s["lon"], "time": c["time"],
                      "bytes": p.stat().st_size})
    mask = ocean_mask(load_land())
    obj = {"scenarios": items, "grid": {"global": GLOBAL, "corridor": CORRIDOR, "frames": FRAMES},
           "ocean_mask": b64(np.packbits(mask, bitorder="little")), "ocean_points": int(mask.sum()),
           "attribution": [ATTRIBUTION, ERA5_ATTRIBUTION]}
    write_js(out_dir / "index.js", "window.WORLD_INDEX", obj, "python -m sim.world scenarios")
    return obj


# ---------------------------------------------------------------- now (forecast)


def private_sites(private_dir=DEFAULT_PRIVATE):
    p = Path(private_dir) / "sites.json"
    if not p.exists():
        return []
    d = json.loads(p.read_text(encoding="utf-8"))
    return [(float(q["lat"]), float(q["lon"])) for q in d.get("points", [])]


def near_private(lat, lon, private_dir=DEFAULT_PRIVATE, km=PRIVATE_GUARD_KM) -> bool:
    """True when (lat, lon) lies within km of a private site (the sites are never printed)."""
    return any(great_circle_km(lat, lon, a, b) < km for a, b in private_sites(private_dir))


def inside_repo(path) -> bool:
    try:
        Path(path).resolve().relative_to(ROOT)
        return True
    except ValueError:
        return False


def make_now(lat, lon, bearing=CORRIDOR["bearing_deg"], with_global=True, out=None, cache=CACHE_DIR,
             private_dir=DEFAULT_PRIVATE, pacer=None, get=None, now=None, log=print):
    out = Path(out) if out else OUT_DIR / "now.js"
    if inside_repo(out) and near_private(lat, lon, private_dir):
        raise SystemExit("this corridor lies near a private site: write it under the private store with --out")
    now = time.time() if now is None else now
    t0 = int(now // (3 * 3600) * (3 * 3600))
    spec = dict(CORRIDOR, lat=lat, lon=lon, bearing_deg=bearing)
    plat, plon = corridor_points(spec)
    d0, d1 = date_utc(t0), date_utc(t0 + 3600 * FRAMES["dt_h"] * (FRAMES["nt"] - 1))
    cw, cwr = fetch_locations("ifs_wind", plat, plon, d0, d1, cache, pacer, get, log=log)
    cm, cmr = fetch_locations("mfwam_waves", plat, plon, d0, d1, cache, pacer, get, log=log)
    obj = {"id": "now", "kind": "now", "title": {"ja": "今の予報", "en": "Forecast now"},
           "corridor": pack_field(build_corridor(spec, t0, cw, cm, "mfwam_waves")),
           "attribution": [ATTRIBUTION], "sources": {"corridor": sources_of(cwr + cmr, CORRIDOR_POINTS)}}
    if with_global:
        mask = ocean_mask(load_land())
        glat, glon = global_grid(GLOBAL)
        wl, wr = fetch_locations("ifs_wind", glat[mask], glon[mask], d0, d0, cache, pacer, get, log=log)
        ml, mr = fetch_locations("mfwam_waves_global", glat[mask], glon[mask], d0, d0, cache, pacer, get, log=log)
        obj["global"] = pack_field(build_global(t0, mask, wl, ml, "ifs_wind", "mfwam_waves_global"))
        obj["sources"]["global"] = sources_of(wr + mr, GLOBAL_POINTS)
    write_js(out, world_file_target("now"), obj, "python -m sim.world now")
    log(f"wrote {out}")
    return obj


def sizes(out_dir=OUT_DIR):
    return sorted((p.name, p.stat().st_size) for p in Path(out_dir).glob("*.js"))


# ---------------------------------------------------------------- command


def main(argv=None):
    ap = argparse.ArgumentParser(prog="python -m sim.world", description=__doc__.split("\n")[0])
    sub = ap.add_subparsers(dest="cmd", required=True)
    a_land = sub.add_parser("land")
    a_land.add_argument("--src", required=True)
    a_sc = sub.add_parser("scenarios")
    a_sc.add_argument("--only", nargs="*")
    a_sc.add_argument("--cache", default=str(CACHE_DIR))
    a_now = sub.add_parser("now")
    a_now.add_argument("--lat", type=float, required=True)
    a_now.add_argument("--lon", type=float, required=True)
    a_now.add_argument("--bearing", type=float, default=CORRIDOR["bearing_deg"])
    a_now.add_argument("--no-global", action="store_true")
    a_now.add_argument("--out")
    a_now.add_argument("--cache", default=str(CACHE_DIR))
    a_now.add_argument("--private", default=str(DEFAULT_PRIVATE))
    sub.add_parser("sizes")
    a = ap.parse_args(argv)
    if a.cmd == "land":
        _, n, r = write_land(a.src)
        print(f"wrote {LAND_JS.relative_to(ROOT).as_posix()}: {r} rings, {n} points, {LAND_JS.stat().st_size} bytes")
    elif a.cmd == "scenarios":
        idx = make_scenarios(a.only, a.cache)
        print(f"index: {len(idx['scenarios'])} scenarios")
    elif a.cmd == "now":
        make_now(a.lat, a.lon, a.bearing, not a.no_global, a.out, a.cache, a.private)
    elif a.cmd == "sizes":
        for n, b in sizes():
            print(f"{n:28s} {b:8d}")


if __name__ == "__main__":
    main(sys.argv[1:])
