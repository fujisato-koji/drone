"""Strategic layer: the route from a start to a destination over the world sea (D-021, docs/02-form.md#route-planning).

    python -m sim.route solar [--only ID ...] [--pace-log DIR]
        fetch (cached) the ERA5 surface shortwave radiation at every second point of each case's corridor, hourly over
        the case's days and the look-ahead after them, and write sim/viewer/route/solar_<id>.js (the solar layer).
    python -m sim.route export
        write sim/viewer/route_params.js: the energy parameters, the hull points, the steady-leg parameters and the
        solar fit of each aircraft, for sim/viewer/route.js.
    python -m sim.route exp [--workers 4]
        the experiments of docs/02-form.md#route-planning into sim/out/route.json.
    python -m sim.route plan --case ID --start LAT,LON --dest LAT,LON [--aircraft albatross-3m] [--capacity-factor 1]

The planner finds the earliest arrival at the destination subject to the owner's energy rules (D-021):
- an hourly battery: state of charge = previous + solar yield - avionics - motor - take-off, at most the usable
  capacity, never below the reserve;
- the night: from any moment, flying on (avionics and the sensor set, no motor) must carry the battery to the end of
  the next night (the next run of hours whose solar yield is below that draw) without passing below the reserve;
- the motor's allowance: in each local solar day the motor (with the take-offs) may use at most a share of the solar
  energy the route collects that day.

Geometry: nodes have latitude and longitude, a leg between two nodes is a great-circle arc flown in sub-steps of at
most SUB_KM, each at the course of the arc at its start and with the wind and waves sampled there and then (no flat
plane beyond a leg). The field (wind and wave height) is pluggable: CorridorField samples a corridor file of
sim.world; a coarse global or band time series can take its place with the same locate() and sample(). The graph of
a corridor is the corridor grid refined REFINE times, each node linked to the nodes of RINGS rings (one link per
direction); links that touch land (sim/viewer/world/land.js) are left out.

Leg model (sim.navigator): for a course, the hull of the library wind nearest to the sampled wind (HullSource.at)
makes way by soaring when its best soaring pair reaches MIN_VMG along the course (plan_pair, motor_ok=False), at its
speed made good and no motor power; else the steady powered leg (SteadyModel.leg) at its speed and electric power;
else the leg is not flyable. The wind is taken to U_Q and the course off downwind to A_Q (a table, so that the
Python and the JavaScript planner take the same decisions). No wave-ride benefit (the world seas' swell is irregular,
Q-081). Every sub-step and every wait must keep the total wave height within the avoidance height (ds.survival_hs_m).

Search: a label-setting search with labels (time, state of charge, the day's motor slack, on the water or not),
the labels of a node that no other label of the same node and hour dominates, a priority queue on time plus a lower
bound of the time to go (distance over the fastest speed of the leg model); the first label popped at the
destination is the earliest arrival. Waiting on the water is an edge to the end of the hour of the tables (at most
WAIT_S; avionics only, solar charging); the first leg after a wait pays the take-off. Labels of different hours are
not compared, because waiting costs or gains charge (see dom() in search()). When no route keeps the rules, the
planner names the reason and searches again with the rules relaxed in the steps of RELAX; the first route found is
the closest route. sim/viewer/route.js is the same planner, operation for operation (sim/tests/test_route_js.py).
"""
from __future__ import annotations

import argparse
import datetime as _dt
import gzip
import hashlib
import heapq
import json
import math
import os
import sys
import time
import urllib.parse
from pathlib import Path

import numpy as np

from sim import record
from sim import world as W

HERE = Path(__file__).resolve().parent
ROOT = HERE.parent
SOLAR_DIR = HERE / "viewer" / "route"
PARAMS_JS = HERE / "viewer" / "route_params.js"
OUT_JSON = HERE / "out" / "route.json"
MODEL_FILES = [Path(__file__).resolve(), HERE / "navigator.py"]

# ---------------------------------------------------------------- planner settings (record keys route.*, assumed: Q-082)
REFINE = 3            # corridor step / REFINE between nodes (route.node_step_km)
RINGS = 2             # links to the nodes within this many rings (route.rings)
SUB_KM = 10.0         # longest sub-step of a leg, km (route.sub_km)
WAIT_S = 3600.0       # one wait on the water, s (route.wait_h)
LOOKAHEAD_H = 36      # hours of the battery tables after the field's last time (the night after the data)
U_Q = 0.1             # wind step of the leg table, m/s
A_Q = 1.0             # course step of the leg table, deg off downwind
EPS_SOC = 0.25        # Wh: a label with this much less charge but earlier still counts as dominating
EPS_SLACK = 0.25      # Wh, the same for the motor slack
MIN_VMG = 0.3         # m/s, as sim.navigator.MIN_VMG

# ---------------------------------------------------------------- energy (record keys, see energy_defaults)
PACK_FRACTION = 0.84       # route.pack_fraction: the cells' share of the battery mass (three P45B cells of P50's mass in battery.mass_kg)
USABLE_DEPTH = 0.8         # route.usable_depth: the share of the cells' energy used in daily cycles
RESERVE_FRACTION = 0.2     # route.reserve_fraction: share of the usable capacity kept as a reserve
MOTOR_SHARE = 0.5          # route.motor_share: the motor's share of the day's solar generation (D-021)
TAKEOFF_RUN_S = 10.0       # route.takeoff_run_s: the take-off at full thrust (P82 for the bird's run)
SENSOR_KEY = "est.set_tips_rtk_power_w"   # the sensor set flown (tip radars, RTK GNSS, IMU, barometer)
SOLAR_YEAR_KEY = "lit.insolation_50s_year_kwh_m2_day"
SOLAR_FIT_POINT = (-50.0, 70.0, 2023)     # P51's point and a year without a leap day
AIRCRAFT = ("albatross-3m", "baseline-2m")
SHORT = {"albatross-3m": "albatross", "baseline-2m": "baseline"}

# ---------------------------------------------------------------- the solar layer (one byte per hourly value)
SOLAR_Q = 5.0              # W/m2 per step (route.q_ghi_w_m2), 255 = missing
SOLAR_EVERY = 2            # every second corridor point along and across
SOLAR_DS = {"url": W.ARCHIVE_URL, "model": "era5", "vars": ("shortwave_radiation",),
            "dataset": "ERA5 hourly surface shortwave radiation, mean of the preceding hour (0.25 deg), "
                       "Open-Meteo historical weather API", "licence": W.LICENCE + "; ERA5 CC BY 4.0"}

REASONS = {0: "ok", 1: "the start lies outside the field, on land or in seas above the avoidance height",
           2: "the battery cannot carry the coming night from the start",
           3: "the night drains the battery below the reserve on every route found",
           4: "the route's draw takes the battery below the reserve",
           5: "the motor allowance (its share of the day's solar generation) does not cover the motor legs needed",
           6: "no route reaches the destination within the field's time (seas above the avoidance height, or too slow)",
           7: "only the relaxed search found a route, and it keeps the rules (the strict search's approximations missed it)"}


# ================================================================ sun and solar yield


def _year_day(t):
    """(days in the year, day of the year from 1, hour of the day) of the unix time t (UTC)."""
    days = math.floor(t / 86400.0)
    hour = (t - days * 86400.0) / 3600.0
    d = _dt.date(1970, 1, 1) + _dt.timedelta(days=int(days))
    n = 366 if (d.year % 4 == 0 and (d.year % 100 != 0 or d.year % 400 == 0)) else 365
    return n, d.timetuple().tm_yday, hour


def sun_sin_elev(lat, lon, t):
    """Sine of the sun's elevation at (lat, lon) and unix time t, by the NOAA formulas (P174)."""
    n, doy, hour = _year_day(t)
    g = 2.0 * math.pi / n * (doy - 1 + (hour - 12.0) / 24.0)
    eqt = 229.18 * (0.000075 + 0.001868 * math.cos(g) - 0.032077 * math.sin(g) - 0.014615 * math.cos(2 * g)
                    - 0.040849 * math.sin(2 * g))
    dec = (0.006918 - 0.399912 * math.cos(g) + 0.070257 * math.sin(g) - 0.006758 * math.cos(2 * g)
           + 0.000907 * math.sin(2 * g) - 0.002697 * math.cos(3 * g) + 0.00148 * math.sin(3 * g))
    tst = hour * 60.0 + eqt + 4.0 * lon
    ha = math.radians(tst / 4.0 - 180.0)
    p = math.radians(lat)
    return math.sin(p) * math.sin(dec) + math.cos(p) * math.cos(dec) * math.cos(ha)


def night_hours(lat, lon, t):
    """Hours of the night (the sun's centre below the NOAA sunrise zenith of 90.833 deg) on the day of t at noon."""
    n, doy, _ = _year_day(t)
    g = 2.0 * math.pi / n * (doy - 1)
    dec = (0.006918 - 0.399912 * math.cos(g) + 0.070257 * math.sin(g) - 0.006758 * math.cos(2 * g)
           + 0.000907 * math.sin(2 * g) - 0.002697 * math.cos(3 * g) + 0.00148 * math.sin(3 * g))
    p = math.radians(lat)
    c = math.cos(math.radians(90.833)) / (math.cos(p) * math.cos(dec)) - math.tan(p) * math.tan(dec)
    if c >= 1.0:
        return 24.0
    if c <= -1.0:
        return 0.0
    return 24.0 - 2.0 * math.degrees(math.acos(c)) / 15.0


_FIT = {}


def solar_fit():
    """All-sky horizontal irradiance per unit sine of the sun's elevation, W/m2: P51's yearly mean insolation at
    SOLAR_FIT_POINT over the yearly mean of the daily integral of the positive sine (5 min steps)."""
    if "w" not in _FIT:
        lat, lon, year = SOLAR_FIT_POINT
        t0 = (_dt.date(year, 1, 1) - _dt.date(1970, 1, 1)).days * 86400.0
        days = []
        for d in range(365):
            s = 0.0
            for k in range(288):
                s += max(0.0, sun_sin_elev(lat, lon, t0 + d * 86400.0 + (k + 0.5) * 300.0))
            days.append(s / 12.0)        # hours of sine
        _FIT["w"] = record.value(SOLAR_YEAR_KEY) * 1000.0 / (sum(days) / 365.0)
        june = [days[d] for d in range(151, 181)]
        _FIT["june_kwh"] = _FIT["w"] * sum(june) / len(june) / 1000.0
    return _FIT["w"]


def model_hour_mean(lat, lon, t_hour, fit):
    """Mean irradiance (W/m2) of the fitted model over the hour from t_hour (four quarter-hour midpoints)."""
    s = 0.0
    for q in (0.125, 0.375, 0.625, 0.875):
        s += max(0.0, sun_sin_elev(lat, lon, t_hour + q * 3600.0))
    return fit * s / 4.0


class SolarLayer:
    """Hourly mean surface shortwave radiation on a coarse corridor grid (a file of `python -m sim.route solar`):
    bilinear in space as sim.world's corridor, constant over each hour [t0 + k h, t0 + (k + 1) h)."""

    def __init__(self, d):
        self.d = d
        self.na, self.nc, self.nt = d["na"], d["nc"], d["nt"]
        self.t0 = d["t0"]
        raw = W.unb64(d["data"])
        self.v = [[(float("nan") if c == 255 else float(c) * d["q"]["step"]) for c in raw[k * self.na * self.nc:(k + 1) * self.na * self.nc]]
                  for k in range(self.nt)]

    @classmethod
    def load(cls, case_id, directory=SOLAR_DIR):
        p = Path(directory) / f"solar_{case_id}.js"
        return cls(W.load_js(p)) if p.exists() else None

    def locate(self, lat, lon):
        fi, fj = W.corridor_index(self.d, lat, lon)
        return float(fi), float(fj)

    def hour_mean(self, loc, t_hour):
        """W/m2 over the hour starting at t_hour, or None outside the layer's hours."""
        k = int(round((t_hour - self.t0) / 3600.0))
        if k < 0 or k >= self.nt:
            return None
        v = bilinear(self.v[k], self.na, self.nc, loc[0], loc[1])
        return None if not math.isfinite(v) else v


# ================================================================ the field


def bilinear(grid, n0, n1, fi, fj):
    """sim.world._bilinear on a flat list grid[i * n1 + j]."""
    fi = min(max(fi, 0.0), n0 - 1.0)
    fj = min(max(fj, 0.0), n1 - 1.0)
    i = min(int(math.floor(fi)), n0 - 2) if n0 > 1 else 0
    j = min(int(math.floor(fj)), n1 - 2) if n1 > 1 else 0
    a, b = fi - i, fj - j
    tot = ws = 0.0
    for di, dj, w in ((0, 0, (1 - a) * (1 - b)), (1, 0, a * (1 - b)), (0, 1, (1 - a) * b), (1, 1, a * b)):
        v = grid[min(i + di, n0 - 1) * n1 + min(j + dj, n1 - 1)]
        if w > 0 and math.isfinite(v):
            tot += w * v
            ws += w
    if ws > 1e-12:
        return tot / ws
    vals = [grid[min(i + di, n0 - 1) * n1 + min(j + dj, n1 - 1)] for di in (0, 1) for dj in (0, 1)]
    vals = [v for v in vals if math.isfinite(v)]
    return sum(vals) / len(vals) if vals else float("nan")


def corners(n0, n1, fi, fj):
    """The corners of bilinear() at (fi, fj) with a positive weight: ((flat index, weight), ...) in its order."""
    fi = min(max(fi, 0.0), n0 - 1.0)
    fj = min(max(fj, 0.0), n1 - 1.0)
    i = min(int(math.floor(fi)), n0 - 2) if n0 > 1 else 0
    j = min(int(math.floor(fj)), n1 - 2) if n1 > 1 else 0
    a, b = fi - i, fj - j
    out = []
    for di, dj, w in ((0, 0, (1 - a) * (1 - b)), (1, 0, a * (1 - b)), (0, 1, (1 - a) * b), (1, 1, a * b)):
        if w > 0:
            out.append((min(i + di, n0 - 1) * n1 + min(j + dj, n1 - 1), w))
    return tuple(out)


class CorridorField:
    """The 10 m wind and the total wave height of a corridor (sim.world.unpack_field), sampled as
    sim.world.sample_corridor does (space bilinear, time linear)."""

    def __init__(self, f):
        self.f = f
        self.na, self.nc, self.nt = f["na"], f["nc"], f["nt"]
        self.t0, self.dt = float(f["t0"]), 3600.0 * f["dt_h"]
        self.t_end = self.t0 + self.dt * (self.nt - 1)
        self.vals = {n: [list(map(float, np.asarray(f["vals"][n][k], float))) for k in range(self.nt)]
                     for n in ("u10", "v10", "hs")}
        u, v = np.asarray(f["vals"]["u10"], float), np.asarray(f["vals"]["v10"], float)
        self.u10_max = float(np.nanmax(np.sqrt(u * u + v * v)))

    @classmethod
    def from_case(cls, case_id, directory=W.OUT_DIR):
        d = W.load_js(Path(directory) / f"{case_id}.js")
        return cls(W.unpack_field(d["corridor"]))

    def locate(self, lat, lon):
        """The location of (lat, lon) for sample(): its fractional indices and the bilinear corners (flat index and
        weight, as bilinear() takes them), or None outside the corridor's grid."""
        fi, fj = W.corridor_index(self.f, lat, lon)
        fi, fj = float(fi), float(fj)
        if -1e-9 <= fi <= self.na - 1 + 1e-9 and -1e-9 <= fj <= self.nc - 1 + 1e-9:
            return fi, fj, corners(self.na, self.nc, fi, fj)
        return None

    def _at(self, g, loc):
        tot = ws = 0.0
        for idx, w in loc[2]:
            v = g[idx]
            if math.isfinite(v):
                tot += w * v
                ws += w
        return tot / ws if ws > 1e-12 else bilinear(g, self.na, self.nc, loc[0], loc[1])

    def sample(self, loc, t):
        """(u10 east, v10 north, total wave height) at loc and unix time t."""
        x = min(max((t - self.t0) / self.dt, 0.0), self.nt - 1.0)
        k = min(int(math.floor(x)), self.nt - 2) if self.nt > 1 else 0
        ft = x - k
        out = []
        for n in ("u10", "v10", "hs"):
            a = self._at(self.vals[n][k], loc)
            if self.nt == 1 or ft == 0:
                out.append(a)
                continue
            b = self._at(self.vals[n][k + 1], loc)
            out.append(b if not math.isfinite(a) else a if not math.isfinite(b) else a + (b - a) * ft)
        return out[0], out[1], out[2]


# ================================================================ great-circle geometry


def gc_km(lat1, lon1, lat2, lon2):
    return W.great_circle_km(lat1, lon1, lat2, lon2)


def gc_point(lat1, lon1, lat2, lon2, f):
    """The point at fraction f of the great-circle arc (slerp on the unit sphere)."""
    p1, l1, p2, l2 = map(math.radians, (lat1, lon1, lat2, lon2))
    a = (math.cos(p1) * math.cos(l1), math.cos(p1) * math.sin(l1), math.sin(p1))
    b = (math.cos(p2) * math.cos(l2), math.cos(p2) * math.sin(l2), math.sin(p2))
    dot = min(1.0, max(-1.0, a[0] * b[0] + a[1] * b[1] + a[2] * b[2]))
    om = math.acos(dot)
    if om < 1e-12:
        return lat1, lon1
    sa, sb = math.sin((1 - f) * om) / math.sin(om), math.sin(f * om) / math.sin(om)
    x, y, z = (sa * a[0] + sb * b[0], sa * a[1] + sb * b[1], sa * a[2] + sb * b[2])
    return math.degrees(math.atan2(z, math.sqrt(x * x + y * y))), math.degrees(math.atan2(y, x))


def gc_bearing(lat1, lon1, lat2, lon2):
    """Initial course (deg clockwise from north) from point 1 to point 2."""
    p1, p2 = math.radians(lat1), math.radians(lat2)
    dl = math.radians(lon2 - lon1)
    y = math.sin(dl) * math.cos(p2)
    x = math.cos(p1) * math.sin(p2) - math.sin(p1) * math.cos(p2) * math.cos(dl)
    return wrap360(math.degrees(math.atan2(y, x)))


def wrap360(x):
    r = math.fmod(x, 360.0)
    return r + 360.0 if r < 0 else r


def wrap180(x):
    return wrap360(x + 180.0) - 180.0


def ring_offsets(rings):
    """Grid offsets (di, dj) within `rings` rings, one per direction (gcd 1)."""
    out = []
    for di in range(-rings, rings + 1):
        for dj in range(-rings, rings + 1):
            if (di, dj) != (0, 0) and math.gcd(abs(di), abs(dj)) == 1:
                out.append((di, dj))
    return out


class Graph:
    """Nodes (lat, lon, field location) and their legs: for each node a list of (to, sub-step metres, steps) with
    steps a list of (field location, course deg) at the start of each sub-step."""

    def __init__(self, lat, lon, loc, edges, step_km):
        self.lat, self.lon, self.loc, self.edges, self.step_km = lat, lon, loc, edges, step_km

    def nearest(self, lat, lon):
        best, bd = None, math.inf
        for n in range(len(self.lat)):
            if self.loc[n] is None:
                continue
            d = gc_km(lat, lon, self.lat[n], self.lon[n])
            if d < bd:
                best, bd = n, d
        return best, bd


def corridor_graph(field, refine=REFINE, rings=RINGS, sub_km=SUB_KM, land=None):
    """The graph of a corridor: its grid refined `refine` times, legs to the nodes of `rings` rings, no node or
    sub-step point on land (land: rings of sim.world.load_land, None to load them, False for none)."""
    f = field.f
    na, nc, step = f["na"], f["nc"], f["step_km"]
    NA, NC = (na - 1) * refine + 1, (nc - 1) * refine + 1
    h = step / refine
    (ax, ay), (cx, cy) = W._axes(f["bearing_deg"])
    lat, lon = [], []
    for i in range(NA):
        for j in range(NC):
            s, q = (i - (NA - 1) / 2.0) * h, (j - (NC - 1) / 2.0) * h
            la, lo = W.aeqd_inverse(f["lat"], f["lon"], s * ax + q * cx, s * ay + q * cy)
            lat.append(float(la))
            lon.append(float(lo))
    rings_land = W.load_land() if land is None else (land or [])
    wet = ~W.on_land(rings_land, lat, lon) if rings_land else np.ones(len(lat), bool)
    loc = [field.locate(lat[n], lon[n]) if wet[n] else None for n in range(len(lat))]
    offs = ring_offsets(rings)
    cand = []           # (n, m, nsub, sub_m, points of the steps, check points)
    for i in range(NA):
        for j in range(NC):
            n = i * NC + j
            if loc[n] is None:
                continue
            for di, dj in offs:
                ii, jj = i + di, j + dj
                if not (0 <= ii < NA and 0 <= jj < NC):
                    continue
                m = ii * NC + jj
                if loc[m] is None:
                    continue
                L = gc_km(lat[n], lon[n], lat[m], lon[m])
                ns = max(1, int(math.ceil(L / sub_km - 1e-9)))
                pts = [gc_point(lat[n], lon[n], lat[m], lon[m], k / ns) for k in range(ns)]
                chk = [gc_point(lat[n], lon[n], lat[m], lon[m], (k + 0.5) / ns) for k in range(ns)]
                cand.append((n, m, ns, L * 1000.0 / ns, pts, chk))
    if rings_land:
        allp = [p for c in cand for p in c[4][1:] + c[5]]
        dry = W.on_land(rings_land, [p[0] for p in allp], [p[1] for p in allp]) if allp else []
    edges = [[] for _ in lat]
    k = 0
    for n, m, ns, sub_m, pts, chk in cand:
        npts = len(pts) - 1 + len(chk)
        if rings_land:
            bad = bool(np.any(dry[k:k + npts]))
            k += npts
            if bad:
                continue
        steps = []
        ok = True
        for p in pts:
            lc = field.locate(p[0], p[1])
            if lc is None:
                ok = False
                break
            steps.append((lc, gc_bearing(p[0], p[1], lat[m], lon[m])))
        if ok:
            edges[n].append((m, sub_m, steps))
    return Graph(lat, lon, loc, edges, h)


# ================================================================ leg model


class LegModel:
    """Mode, speed made good along a course and motor power from the wind and the course off downwind (module notes)."""

    def __init__(self, source, steady):
        self.source, self.steady = source, steady
        self.cache = {}

    def leg(self, u10, rel_deg):
        iu = int(math.floor(u10 / U_Q + 0.5))
        ia = int(math.floor(abs(wrap180(rel_deg)) / A_Q + 0.5))
        key = iu * 1000 + ia
        r = self.cache.get(key)
        if r is None:
            uq, aq = iu * U_Q, math.radians(ia * A_Q)
            d = (math.cos(aq), math.sin(aq))
            pl = self.source.at(uq).plan_pair(d, MIN_VMG, motor_ok=False)
            if pl is not None:
                r = ("soaring", pl["r"], 0.0)
            else:
                lg = self.steady.leg(uq, d, MIN_VMG)
                r = ("motor", lg["along"], lg["power"]) if lg is not None else ("none", 0.0, 0.0)
            self.cache[key] = r
        return r

    def v_max(self, u10_max):
        """An upper bound of the speed made good: the fastest hull point and the steady leg downwind up to u10_max."""
        v = max(math.sqrt(p.vx * p.vx + p.vy * p.vy) for h in self.source.hulls for p in h.points)
        u = 0.0
        while u <= u10_max + 1.0:
            lg = self.steady.leg(u, (1.0, 0.0), MIN_VMG)
            if lg is not None:
                v = max(v, lg["along"])
            u += 0.5
        return 1.05 * v + 1.0


_LEGS = {}


def hull_points(aircraft):
    """[(u10, [(vx, vy, power, kind, key, side)])] of the motor library (sim.navigator.HullSource)."""
    from sim.mpc import CycleLibrary
    from sim.navigator import HullSource
    src = HullSource.from_entries(CycleLibrary.from_keyword("motor").entries, aircraft)
    return [(h.u10, [(p.vx, p.vy, p.power, p.kind, p.key, p.side) for p in h.points]) for h in src.hulls]


def steady_params(aircraft):
    from sim.aircraft_models import by_name
    from sim.navigator import SteadyModel
    m = SteadyModel.from_aircraft(by_name(aircraft))
    return dict(mass=m.mass, wing_area=m.wing_area, cd0=m.cd0, k_induced=m.k_induced, rho=m.rho, cl_max=m.cl_max,
                v_stall=m.v_stall, v_best_ld=m.v_best_ld, t_max=m.t_max, eta=m.eta, g=m.g)


def leg_model(aircraft, hulls=None, steady=None):
    """The leg model of an aircraft from the motor library (cached), or from given hull points and steady params."""
    from sim.navigator import HullPoint, HullSource, SteadyModel, VelocityHull
    if hulls is None and steady is None and aircraft in _LEGS:
        return _LEGS[aircraft]
    hulls = hull_points(aircraft) if hulls is None else hulls
    steady = steady_params(aircraft) if steady is None else steady
    src = HullSource([VelocityHull([HullPoint(vx, vy, pw, kd, key, None, sd) for vx, vy, pw, kd, key, sd in pts], u)
                      for u, pts in hulls])
    lm = LegModel(src, SteadyModel(**steady))
    if aircraft is not None:
        _LEGS.setdefault(aircraft, lm)
    return lm


# ================================================================ energy


def energy_defaults(aircraft, capacity_factor=1.0):
    """The planner's energy parameters from the record (docs/02-form.md#route-planning)."""
    from sim.aircraft_models import by_name
    from sim.dsopt import prop_efficiency, thrust_max
    from sim.patterns import SOLAR_CELL_EFF, SOLAR_COVERAGE
    from sim.navigator import STEADY_HEIGHT_M, SteadyModel
    ac = by_name(aircraft)
    cap = record.value("battery.mass_kg") * record.value("lit.cell_p45b_wh_per_kg") * PACK_FRACTION * USABLE_DEPTH
    cap *= capacity_factor
    eta = prop_efficiency()
    v_to = SteadyModel.from_aircraft(ac).v_factor * ac.v_stall
    to_j = thrust_max(ac) * v_to / eta * TAKEOFF_RUN_S + ac.mass * 9.81 * STEADY_HEIGHT_M / eta
    av = record.value("ds.avionics_power_w")
    return {"capacity_wh": cap, "reserve_wh": RESERVE_FRACTION * cap, "soc0_wh": cap, "reserve_fraction": RESERVE_FRACTION,
            "flight_draw_w": av + record.value(SENSOR_KEY), "water_draw_w": av,
            "panel_m2": float(ac.wing_area) * SOLAR_COVERAGE * SOLAR_CELL_EFF,
            "takeoff_wh": to_j / 3600.0, "takeoff_s": TAKEOFF_RUN_S, "motor_share": MOTOR_SHARE,
            "avoid_hs_m": record.value("ds.survival_hs_m"), "solar_fit_w_m2": solar_fit()}


class Tables:
    """Per node and hour from T0: the panel's mean power (W), the requirement at the end of the hour for the night
    check (need_end, Wh) and the solar energy left in the node's local day from the start of the hour (rest, Wh)."""

    def __init__(self, graph, field, en, layer=None, reserve=None, t_start=None, hours=None):
        # from a day before the field's start (the start's day counts its solar energy before the start)
        self.T0 = math.floor((field.t0 if t_start is None else t_start) / 3600.0) * 3600.0 - (86400.0 if t_start is None else 0.0)
        self.K = int(math.ceil((field.t_end - self.T0) / 3600.0)) + LOOKAHEAD_H if hours is None else hours
        self.en = en
        fit = en["solar_fit_w_m2"]
        R = en["reserve_wh"] if reserve is None else reserve
        C, D = en["capacity_wh"], en["flight_draw_w"]
        self.P, self.need, self.need_raw, self.rest, self.day_start = [], [], [], [], []
        self.layer_hours = 0         # hours of the tables that the layer covers (the model the others)
        for n in range(len(graph.lat)):
            if graph.loc[n] is None:
                self.P.append(None)
                self.need.append(None)
                self.need_raw.append(None)
                self.rest.append(None)
                self.day_start.append(None)
                continue
            la, lo = graph.lat[n], graph.lon[n]
            ll = layer.locate(la, lo) if layer is not None else None
            P = []
            for k in range(self.K):
                th = self.T0 + 3600.0 * k
                g = layer.hour_mean(ll, th) if layer is not None else None
                if g is None:
                    g = model_hour_mean(la, lo, th, fit)
                P.append(g * en["panel_m2"])
            self.P.append(P)
            net = [p - D for p in P]
            # need: the charge at the end of hour k that carries the night (inf when the night needs more than the
            # capacity); for the closest route's margin, cap is the same without the limit during the night and held
            # at the capacity before it, and short is what a full battery at the night's start still lacks
            need, cap, short = [0.0] * self.K, [0.0] * self.K, [0.0] * self.K
            req, rc, sh_ = R, R, 0.0
            for k in range(self.K - 1, -1, -1):
                if net[k] < 0 and (k + 1 == self.K or net[k + 1] >= 0):
                    req, rc, sh_ = R, R, 0.0
                if net[k] >= 0 and k + 1 < self.K and net[k + 1] < 0:
                    sh_, rc = max(0.0, rc - C), min(rc, C)
                need[k], cap[k], short[k] = req, rc, sh_
                rs = max(R, req - net[k])
                req = math.inf if rs > C + 1e-9 else rs
                rc = max(R, rc - net[k])
            self.need.append(need)
            self.need_raw.append((cap, short))
            rest = [0.0] * self.K
            acc, day = 0.0, None
            for k in range(self.K - 1, -1, -1):
                dk = math.floor((self.T0 + 3600.0 * k + 1800.0 + lo * 240.0) / 86400.0)
                if dk != day:
                    acc, day = 0.0, dk
                acc += P[k]
                rest[k] = acc
            self.rest.append(rest)
            ds, first, day = [0] * self.K, 0, None
            for k in range(self.K):
                dk = math.floor((self.T0 + 3600.0 * k + 1800.0 + lo * 240.0) / 86400.0)
                if dk != day:
                    first, day = k, dk
                ds[k] = first
            self.day_start.append(ds)
        if layer is not None:
            self.layer_hours = sum(1 for k in range(self.K) if 0 <= round((self.T0 + 3600.0 * k - layer.t0) / 3600.0) < layer.nt)

    def _hour(self, t):
        return int(math.floor((t - self.T0) / 3600.0))

    def solar_wh(self, n, ta, tb):
        P, s, t = self.P[n], 0.0, ta
        while t < tb - 1e-9:
            k = self._hour(t)
            te = min(tb, self.T0 + 3600.0 * (k + 1))
            if 0 <= k < self.K:
                s += P[k] * (te - t) / 3600.0
            t = te
        return s

    def need_at(self, n, t, reserve, raw=False):
        """The charge that carries the next night from node n at t (inf when it needs more than the capacity; raw:
        the charge that would carry it, the capacity's limit counted as a shortfall, for a margin)."""
        k = self._hour(t)
        if k < 0 or k >= self.K:
            return reserve
        D = self.en["flight_draw_w"]
        frac = (self.T0 + 3600.0 * (k + 1) - t) / 3600.0
        if not raw:
            return max(reserve, self.need[n][k] - (self.P[n][k] - D) * frac)
        cap, short = self.need_raw[n]
        return max(reserve, cap[k] - (self.P[n][k] - D) * frac) + short[k]

    def day_total(self, n, t):
        """The solar energy of the node's local day that contains t (Wh, hours of the tables only)."""
        k = self._hour(t)
        if k < 0 or k >= self.K:
            return 0.0
        return self.rest[n][self.day_start[n][k]]

    def rest_at(self, n, t):
        k = self._hour(t)
        if k < 0 or k >= self.K:
            return 0.0
        return max(0.0, self.rest[n][k] - self.P[n][k] * (t - self.T0 - 3600.0 * k) / 3600.0)


def local_day(t, lon):
    return math.floor((t + lon * 240.0) / 86400.0)


# ================================================================ search


def search(graph, field, legs, tab, en, start, dest, t0, *, reserve=None, night=True, share=None, horizon=None,
           v_max=None, max_pops=2_000_000):
    """Earliest arrival at `dest` from `start` (node indices) at unix time t0 (module notes). share None: the
    allowance of D-021 (en["motor_share"]); share math.inf: no allowance. Returns (labels, the arrival label or
    None, stats)."""
    R = en["reserve_wh"] if reserve is None else reserve
    C = en["capacity_wh"]
    sh = en["motor_share"] if share is None else share
    use_slack = math.isfinite(sh)
    avoid = en["avoid_hs_m"]
    H = field.t_end if horizon is None else horizon
    fd, wd = en["flight_draw_w"], en["water_draw_w"]
    vm = legs.v_max(field.u10_max) if v_max is None else v_max
    dl, dn = graph.lat[dest], graph.lon[dest]
    hcache = {}

    def heur(n):
        v = hcache.get(n)
        if v is None:
            v = gc_km(graph.lat[n], graph.lon[n], dl, dn) * 1000.0 / vm
            hcache[n] = v
        return v

    # label arrays
    LT, LS, LK, LD, LW, LN, LP, LE, LA = [], [], [], [], [], [], [], [], []
    pareto = {}                 # (node, hour of the tables) -> labels

    T0 = tab.T0
    to_wh = en["takeoff_wh"]

    def dom(n, ta, sa, ka, da, wa, tb, sb, kb, db, wb):
        """Label a is at least as good as label b at node n: a is not later and in the same hour of the tables (its
        next wait ends at the hour's end), and after holding on until b's time (on the water, or in the air if both
        fly, with the node's solar energy, at most the capacity, and a take-off if a must leave the water) it has b's
        charge (to EPS_SOC) and b's motor slack (to EPS_SLACK; a label of an earlier day must have a slack of at least
        zero). Labels in different hours are not compared: waiting costs or gains charge, so being earlier is not
        better by itself, and a wait's continuation exists only at the hour's end."""
        if ta > tb or math.floor((ta - T0) / 3600.0) != math.floor((tb - T0) / 3600.0):
            return False
        x, sol = sa, 0.0
        if tb > ta:
            sol = tab.solar_wh(n, ta, tb)
            drain = wd if (wa or wb) else fd
            x = min(C, sa + sol - drain * (tb - ta) / 3600.0)
        if wa and not wb:
            x -= to_wh
        if x < sb - EPS_SOC:
            return False
        if not use_slack:
            return True
        return (da == db and ka + sh * sol >= kb - EPS_SLACK) or (da < db and ka >= 0.0)

    def dominated(n, t, soc, slack, day, water):
        for li in pareto.get((n, math.floor((t - T0) / 3600.0)), ()):
            if LA[li] and dom(n, LT[li], LS[li], LK[li], LD[li], LW[li], t, soc, slack, day, water):
                return True
        return False

    def add(n, t, soc, slack, day, water, parent, edge):
        """A new label, or None when one at the node dominates it; it removes the labels it dominates."""
        if dominated(n, t, soc, slack, day, water):
            why["dominated"] += 1
            return None
        lst = pareto.setdefault((n, math.floor((t - T0) / 3600.0)), [])
        for li in lst:
            if LA[li] and dom(n, t, soc, slack, day, water, LT[li], LS[li], LK[li], LD[li], LW[li]):
                LA[li] = False
        lst[:] = [li for li in lst if LA[li]]
        i = len(LT)
        LT.append(t), LS.append(soc), LK.append(slack), LD.append(day), LW.append(water), LN.append(n)
        LP.append(parent), LE.append(edge), LA.append(True)
        lst.append(i)
        return i

    stats = {"pops": 0, "labels": 0, "capped": False}
    why = {"seas": 0, "no_leg": 0, "horizon": 0, "reserve": 0, "allowance": 0, "night": 0, "dominated": 0}
    stats["pruned"] = why

    def ok_after(n, t, soc, slack, day):
        """Night check and the day's allowance after reaching node n at t; terminal checks at the destination."""
        if night and soc < tab.need_at(n, t, R) - 1e-9:
            why["night"] += 1
            return False
        if use_slack:
            rest = tab.rest_at(n, t) if local_day(t, graph.lon[n]) == day else 0.0
            if slack + sh * rest < -EPS_SLACK:
                why["allowance"] += 1
                return False
        return True

    seq = 0
    soc0 = min(C, en["soc0_wh"])
    # the start's day counts the solar energy of that day before the start (at the start node) as generated that day
    k0 = (sh * (tab.day_total(start, t0) - tab.rest_at(start, t0))) if use_slack else 0.0
    i0 = add(start, t0, soc0, k0, local_day(t0, graph.lon[start]), False, None, None)
    heap = [(t0 + heur(start), seq, i0)]
    while heap:
        _, _, li = heapq.heappop(heap)
        if not LA[li]:
            continue
        stats["pops"] += 1
        if stats["pops"] > max_pops:
            stats["capped"] = True
            break
        n = LN[li]
        if n == dest:
            stats["labels"] = len(LT)
            return (LT, LS, LK, LD, LW, LN, LP, LE), li, stats
        t, soc, slack, day, water = LT[li], LS[li], LK[li], LD[li], LW[li]
        if t >= H:
            continue
        lon_n = graph.lon[n]
        # ---- wait on the water until the end of the hour of the tables (WAIT_S at most)
        te = min(t + WAIT_S, T0 + 3600.0 * (math.floor((t - T0) / 3600.0) + 1))
        if te <= H + 1e-9:
            hs_a = field.sample(graph.loc[n], t)[2]
            hs_b = field.sample(graph.loc[n], te)[2]
            if math.isfinite(hs_a) and math.isfinite(hs_b) and hs_a <= avoid and hs_b <= avoid:
                d2, k2 = day, slack
                dd = local_day(t, lon_n)
                good = True
                if dd > d2:
                    if use_slack and k2 < -EPS_SLACK:
                        good = False
                    d2, k2 = dd, 0.0
                if good:
                    sol = tab.solar_wh(n, t, te)
                    s2 = min(C, soc + sol - wd * (te - t) / 3600.0)
                    k2 = k2 + (sh * sol if use_slack else 0.0)
                    if s2 >= R - 1e-9 and ok_after(n, te, s2, k2, d2) and not (n == dest):
                        e = ("wait", n, n, t, te, ("waiting",), 0.0, wd * (te - t) / 3600.0, sol, 0.0)
                        j = add(n, te, s2, k2, d2, True, li, e)
                        if j is not None:
                            seq += 1
                            heapq.heappush(heap, (te + heur(n), seq, j))
        # ---- legs
        for m, sub_m, steps in graph.edges[n]:
            tt, s2, k2, d2 = t, soc, slack, day
            motor = draw = sol_tot = to = 0.0
            modes = []
            good = True
            if water:
                dd = local_day(tt, lon_n)
                if dd > d2:
                    if use_slack and k2 < -EPS_SLACK:
                        continue
                    d2, k2 = dd, 0.0
                to = en["takeoff_wh"]
                s2 -= to
                k2 -= to if use_slack else 0.0
                motor += to
                tt += en["takeoff_s"]
                if s2 < R - 1e-9:
                    continue
            for lc, brg in steps:
                if tt > H:
                    why["horizon"] += 1
                    good = False
                    break
                u, v, hs = field.sample(lc, tt)
                if not (hs <= avoid):
                    why["seas"] += 1
                    good = False
                    break
                sp = math.sqrt(u * u + v * v)
                wf = wrap360(math.degrees(math.atan2(-u, -v)))
                mode, vs, pw = legs.leg(sp, brg - wf - 180.0)
                if mode == "none":
                    why["no_leg"] += 1
                    good = False
                    break
                dt = sub_m / vs
                dd = local_day(tt, lon_n)
                if dd > d2:
                    if use_slack and k2 < -EPS_SLACK:
                        why["allowance"] += 1
                        good = False
                        break
                    d2, k2 = dd, 0.0
                sol = tab.solar_wh(n, tt, tt + dt)
                mo = pw * dt / 3600.0
                dr = fd * dt / 3600.0
                s2 = min(C, s2 + sol - dr - mo)
                if use_slack:
                    k2 += sh * sol - mo
                motor += mo
                draw += dr
                sol_tot += sol
                tt += dt
                modes.append(mode)
                if s2 < R - 1e-9:
                    why["reserve"] += 1
                    good = False
                    break
                if use_slack:
                    rest = tab.rest_at(n, tt) if local_day(tt, lon_n) == d2 else 0.0
                    if k2 + sh * rest < -EPS_SLACK:
                        why["allowance"] += 1
                        good = False
                        break
            if not good or not ok_after(m, tt, s2, k2, d2):
                continue
            e = ("leg", n, m, t, tt, tuple(modes), motor, draw, sol_tot, to)
            j = add(m, tt, s2, k2, d2, False, li, e)
            if j is not None:
                seq += 1
                heapq.heappush(heap, (tt + heur(m), seq, j))
    stats["labels"] = len(LT)
    return (LT, LS, LK, LD, LW, LN, LP, LE), None, stats


def route_of(graph, labels, li):
    """The route of label li: its waypoints (node, lat, lon, t, soc, slack, on the water) and its legs and waits."""
    LT, LS, LK, LD, LW, LN, LP, LE = labels
    chain = []
    while li is not None:
        chain.append(li)
        li = LP[li]
    chain.reverse()
    wps, legs = [], []
    for li in chain:
        n = LN[li]
        wps.append({"node": n, "lat": graph.lat[n], "lon": graph.lon[n], "t": LT[li], "soc_wh": LS[li],
                    "slack_wh": LK[li], "water": LW[li]})
        e = LE[li]
        if e is None:
            continue
        kind, a, b, ta, tb, modes, motor, draw, sol, to = e
        mode = "waiting" if kind == "wait" else (modes[0] if len(set(modes)) == 1 else "mixed")
        legs.append({"from": a, "to": b, "t0": ta, "t1": tb, "mode": mode, "modes": list(modes), "motor_wh": motor,
                     "draw_wh": draw, "solar_wh": sol, "takeoff_wh": to, "soc_end_wh": LS[li]})
    return wps, legs


def summarize(legs, wps, t0):
    out = {"time_h": (wps[-1]["t"] - t0) / 3600.0, "motor_wh": 0.0, "takeoff_wh": 0.0, "draw_wh": 0.0, "solar_wh": 0.0,
           "soaring_h": 0.0, "motor_h": 0.0, "wait_h": 0.0, "waits": 0, "min_soc_wh": min(w["soc_wh"] for w in wps),
           "soc_end_wh": wps[-1]["soc_wh"]}
    prev_wait = False           # waits: the number of separate waits (a wait of several hours counts once)
    for lg in legs:
        dt = (lg["t1"] - lg["t0"]) / 3600.0
        out["motor_wh"] += lg["motor_wh"]
        out["takeoff_wh"] += lg["takeoff_wh"]
        out["draw_wh"] += lg["draw_wh"]
        out["solar_wh"] += lg["solar_wh"]
        if lg["mode"] == "waiting":
            out["wait_h"] += dt
            out["waits"] += 1 if not prev_wait else 0
            prev_wait = True
        else:
            prev_wait = False
            ns = len(lg["modes"])
            out["soaring_h"] += dt * sum(1 for m in lg["modes"] if m == "soaring") / ns
            out["motor_h"] += dt * sum(1 for m in lg["modes"] if m == "motor") / ns
    out["net_wh"] = out["draw_wh"] + out["motor_wh"] - out["solar_wh"]
    return out


def strict_margin(graph, tab, en, wps, legs):
    """Lowest margins of a route against the strict rules (negative: the shortfall), Wh: the charge above the
    reserve and above the night's requirement at its waypoints, and the day's allowance less the motor energy of
    the day (each leg in the local day of its start; the start's day with its solar energy before the start)."""
    R = en["reserve_wh"]
    soc_m = min(w["soc_wh"] - R for w in wps)
    night_m = min(w["soc_wh"] - tab.need_at(w["node"], w["t"], R, raw=True) for w in wps)
    s0 = wps[0]
    days = {local_day(s0["t"], s0["lon"]): tab.day_total(s0["node"], s0["t"]) - tab.rest_at(s0["node"], s0["t"])}
    motor = {}
    for lg in legs:
        d = local_day(lg["t0"], graph.lon[lg["from"]])
        days[d] = days.get(d, 0.0) + lg["solar_wh"]
        motor[d] = motor.get(d, 0.0) + lg["motor_wh"]
    allow_m = min(en["motor_share"] * days[d] - motor.get(d, 0.0) for d in days)
    return soc_m, night_m, allow_m


# the relaxation ladder: the reserve (and with it the night's requirement) lowered by these shares of the capacity,
# then also no motor allowance; the first rung that finds a route gives the closest route
RELAX = (("reserve_minus_1.5_capacity", {"below": 1.5}),
         ("reserve_minus_3_capacity_no_allowance", {"below": 3.0, "share": math.inf}))


def plan(field, graph, legs, en, start_ll, dest_ll, t0=None, layer=None, tables=None, relax=True, max_pops=300_000,
         relax_pops=150_000):
    """Plan a route (module notes). Returns a dict: ok, reason (code, text), route (waypoints, legs, summary),
    closest (the closest route when not ok, with the relaxation that found it), stats."""
    t0 = field.t0 if t0 is None else float(t0)
    tab = tables or Tables(graph, field, en, layer)
    s, ds_ = graph.nearest(*start_ll)
    d, dd_ = graph.nearest(*dest_ll)
    out = {"start_node": s, "dest_node": d, "t0": t0, "start_snap_km": ds_, "dest_snap_km": dd_,
           "dist_km": gc_km(graph.lat[s], graph.lon[s], graph.lat[d], graph.lon[d]) if s is not None and d is not None else None}
    if s is None or d is None or field.sample(graph.loc[s], t0)[2] > en["avoid_hs_m"]:
        out.update(ok=False, reason=1, reason_text=REASONS[1], route=None, closest=None)
        return out
    vm = legs.v_max(field.u10_max)
    labels, li, st = search(graph, field, legs, tab, en, s, d, t0, v_max=vm, max_pops=max_pops)
    out["stats"] = st
    if li is not None:
        wps, lg = route_of(graph, labels, li)
        out.update(ok=True, reason=0, reason_text=REASONS[0], route={"waypoints": wps, "legs": lg,
                                                                      "summary": summarize(lg, wps, t0)}, closest=None)
        return out
    out["ok"] = False
    R, C = en["reserve_wh"], en["capacity_wh"]
    start_need = tab.need_at(s, t0, R, raw=True)
    reason = 2 if min(C, en["soc0_wh"]) < start_need - 1e-9 else None
    out["start_need_wh"] = start_need
    out["route"] = None
    out["closest"] = None
    if not relax:
        out.update(reason=reason or 6, reason_text=REASONS[reason or 6])
        return out
    # the relaxation ladder (RELAX); a rung that pruned nothing for the reserve or the night skips to the last rung
    skip = False
    for name, kw in RELAX:
        if skip and "share" not in kw:
            continue
        rv = R - kw["below"] * C
        tab2 = Tables(graph, field, en, layer, reserve=rv, t_start=tab.T0, hours=tab.K)
        labels, li, st2 = search(graph, field, legs, tab2, en, s, d, t0, v_max=vm, max_pops=relax_pops,
                                 reserve=rv, share=kw.get("share"))
        if li is None:
            skip = st2["pruned"]["reserve"] == 0 and st2["pruned"]["night"] == 0
            continue
        wps, lg = route_of(graph, labels, li)
        sm, nm, am = strict_margin(graph, tab, en, wps, lg)
        out["closest"] = {"relaxation": name, "waypoints": wps, "legs": lg, "summary": summarize(lg, wps, t0),
                          "reserve_margin_wh": sm, "night_margin_wh": nm, "allowance_margin_wh": am, "stats": st2}
        if reason is None:
            reason = 3 if nm < -1e-6 else 4 if sm < -1e-6 else 5 if am < -1e-6 else 7
        break
    if reason is None:
        reason = 6
    out.update(reason=reason, reason_text=REASONS[reason])
    return out


# ================================================================ the solar layer: fetch and write


def solar_points(f):
    """Latitudes, longitudes and the grid spec of the solar layer: every SOLAR_EVERY-th corridor point."""
    na = (f["na"] - 1) // SOLAR_EVERY + 1
    nc = (f["nc"] - 1) // SOLAR_EVERY + 1
    spec = {"lat": f["lat"], "lon": f["lon"], "bearing_deg": f["bearing_deg"], "na": na, "nc": nc,
            "step_km": f["step_km"] * SOLAR_EVERY}
    la, lo = W.corridor_points(spec)
    return [float(x) for x in la], [float(x) for x in lo], spec


def solar_request_url(lats, lons, start_date, end_date):
    q = {"latitude": ",".join(f"{x:.4f}".rstrip("0").rstrip(".") for x in lats),
         "longitude": ",".join(f"{x:.4f}".rstrip("0").rstrip(".") for x in lons),
         "hourly": ",".join(SOLAR_DS["vars"]), "models": SOLAR_DS["model"], "start_date": start_date,
         "end_date": end_date, "timezone": "GMT", "timeformat": "unixtime", "cell_selection": "sea"}
    return SOLAR_DS["url"] + "?" + urllib.parse.urlencode(q, safe=",")


def fetch_solar(lats, lons, start_date, end_date, cache=W.CACHE_DIR, pacer=None, get=None, log=print):
    """Per-location responses and request records, as sim.world.fetch_locations (cached, paced)."""
    get = get or W.http_get
    pacer = pacer or W.Pacer(cache)
    days = (_dt.date.fromisoformat(end_date) - _dt.date.fromisoformat(start_date)).days + 1
    w = W.location_weight(len(SOLAR_DS["vars"]), days)
    out, reqs = [], []
    for b0 in range(0, len(lats), W.BATCH):
        la, lo = lats[b0:b0 + W.BATCH], lons[b0:b0 + W.BATCH]
        url = solar_request_url(la, lo, start_date, end_date)
        key = hashlib.sha256(url.encode()).hexdigest()[:20]
        path = Path(cache) / "raw" / f"era5_solar_{key}.json.gz"
        metap = path.with_name(path.name.replace(".json.gz", ".meta.json"))
        if path.exists():
            body = gzip.decompress(path.read_bytes())
            meta = json.loads(metap.read_text(encoding="utf-8"))
            meta["cached"] = True
        else:
            cost = w * len(la)
            pacer.wait(cost)
            body, headers = get(url)
            pacer.note(float(headers.get("X-RateLimit-Cost", cost)) if headers else cost, url)
            d = json.loads(body)
            if isinstance(d, dict) and d.get("error"):
                raise RuntimeError(f"solar: {d.get('reason')}")
            path.parent.mkdir(parents=True, exist_ok=True)
            path.write_bytes(gzip.compress(body, mtime=0))
            meta = {"url": url, "retrieved_utc": W.now_utc(), "dataset": SOLAR_DS["dataset"], "licence": SOLAR_DS["licence"],
                    "sha256": hashlib.sha256(body).hexdigest(), "bytes": len(body), "locations": len(la), "cost_est": cost}
            metap.write_text(json.dumps(meta, indent=1), encoding="utf-8")
            meta["cached"] = False
            log(f"solar: {len(la)} locations, {len(body)} bytes")
        d = json.loads(body)
        locs = d if isinstance(d, list) else [d]
        if len(locs) != len(la):
            raise RuntimeError(f"solar: {len(locs)} locations returned for {len(la)}")
        out += locs
        meta["first"] = b0
        reqs.append(meta)
    return out, reqs


def build_solar_layer(case_id, f, locs, reqs):
    lats, lons, spec = solar_points(f)
    nt = 24 + int(f["dt_h"]) * (f["nt"] - 1) + LOOKAHEAD_H      # from a day before the corridor's start (as Tables)
    t0 = int(f["t0"]) - 86400
    vals = np.full((nt, len(lats)), np.nan)
    for i, loc in enumerate(locs):
        # the value stamped T is the mean of [T - 1 h, T]: hour k of the layer is the value stamped t0 + (k + 1) h
        vals[:, i] = W.at_times(loc, "shortwave_radiation", SOLAR_DS["model"], [t0 + 3600 * (k + 1) for k in range(nt)])
    # round half up (floor(x / step + 0.5)), as sim.world.quantize does
    codes = np.where(np.isfinite(vals), np.clip(np.floor(np.nan_to_num(vals) / SOLAR_Q + 0.5), 0, 254), 255).astype(np.uint8)
    return {"id": case_id, "kind": "solar", **spec, "t0": t0, "dt_h": 1, "nt": nt,
            "q": {"step": SOLAR_Q, "unit": "W/m2", "missing": 255},
            "layout": "hour, point (i * nc + j); hour k is the mean over [t0 + k h, t0 + (k + 1) h)",
            "data": W.b64(codes.ravel()),
            "attribution": W.ATTRIBUTION + "; generated using Copernicus Climate Change Service information",
            "sources": [{k: r[k] for k in ("url", "retrieved_utc", "sha256", "dataset", "licence") if k in r} for r in reqs]}


def make_solar(only=None, pace_dir=None, log=print):
    pacer = W.Pacer(Path(pace_dir)) if pace_dir else None
    SOLAR_DIR.mkdir(parents=True, exist_ok=True)
    for s in W.SCENARIOS:
        if only and s["id"] not in only:
            continue
        f = W.unpack_field(W.load_js(W.OUT_DIR / f"{s['id']}.js")["corridor"])
        lats, lons, _ = solar_points(f)
        t0 = int(f["t0"]) - 86400
        t1 = int(f["t0"]) + 3600 * (int(f["dt_h"]) * (f["nt"] - 1) + LOOKAHEAD_H + 1)
        locs, reqs = fetch_solar(lats, lons, W.date_utc(t0), W.date_utc(t1), pacer=pacer, log=log)
        obj = build_solar_layer(s["id"], f, locs, reqs)
        path = SOLAR_DIR / f"solar_{s['id']}.js"
        W.write_js(path, "(window.ROUTE_SOLAR = window.ROUTE_SOLAR || {})[" + json.dumps(s["id"]) + "]", obj,
                   f"python -m sim.route solar --only {s['id']}")
        log(f"wrote {path.name} ({path.stat().st_size} bytes)")


# ================================================================ export for route.js


def export_params(path=PARAMS_JS):
    obj = {"settings": {"refine": REFINE, "rings": RINGS, "sub_km": SUB_KM, "wait_s": WAIT_S, "lookahead_h": LOOKAHEAD_H,
                        "u_q": U_Q, "a_q": A_Q, "eps_soc": EPS_SOC, "eps_slack": EPS_SLACK, "min_vmg": MIN_VMG},
           "aircraft": {}}
    for a in AIRCRAFT:
        obj["aircraft"][a] = {"energy": energy_defaults(a), "steady": steady_params(a),
                              "hulls": [{"u10": u, "points": [dict(vx=vx, vy=vy, power=pw, kind=kd, key=key, side=sd)
                                                               for vx, vy, pw, kd, key, sd in pts]}
                                        for u, pts in hull_points(a)]}
    from sim.mpc import LIB_MOTOR_PATH
    path = Path(path)
    with open(path, "w", encoding="utf-8", newline="\n") as fh:
        fh.write(record.header("python -m sim.route export", MODEL_FILES + [Path(LIB_MOTOR_PATH), ROOT / "record" / "values.yaml"]) + "\n")
        fh.write("window.ROUTE_PARAMS =\n" + json.dumps(obj, separators=(",", ":")) + ";\n")
    return path


# ================================================================ experiments

EXP_PAIRS = (("downwind", 0.0, 400.0), ("upwind", 180.0, 200.0), ("across", 90.0, 300.0))
EXP_CAPACITY = (0.5, 1.0, 2.0)
EDGE_MARGIN_KM = 30.0


def mean_wind_from(f):
    u = np.nanmean(np.asarray(f["vals"]["u10"], float))
    v = np.nanmean(np.asarray(f["vals"]["v10"], float))
    return W.wind_speed_from(u, v)[1]


def exp_pairs(f):
    """Start and destination of each experiment pair: centred on the corridor's centre along the mean wind
    (downwind, upwind) or across it, shortened to keep EDGE_MARGIN_KM inside the corridor (at least 100 km)."""
    wf = mean_wind_from(f)
    (ax, ay), (cx, cy) = W._axes(f["bearing_deg"])
    hs = (f["na"] - 1) / 2.0 * f["step_km"] - EDGE_MARGIN_KM
    hq = (f["nc"] - 1) / 2.0 * f["step_km"] - EDGE_MARGIN_KM
    out = []
    for name, off, D in EXP_PAIRS:
        b = math.radians(wf + 180.0 + off)          # the course from the start to the destination
        ex, ey = math.sin(b), math.cos(b)
        es, eq = ex * ax + ey * ay, ex * cx + ey * cy
        half = D / 2.0
        lim = min(hs / abs(es) if abs(es) > 1e-9 else math.inf, hq / abs(eq) if abs(eq) > 1e-9 else math.inf)
        half = max(50.0, min(half, lim))
        pts = []
        for sg in (-1.0, 1.0):
            la, lo = W.aeqd_inverse(f["lat"], f["lon"], sg * half * ex, sg * half * ey)
            pts.append((float(la), float(lo)))
        out.append((name, pts[0], pts[1]))
    return out, wf


def exp_start(field, graph, en, a):
    """The start of an experiment: the first of the field's times in its first two days at which the sea at the
    start node is within the avoidance height (the field's first time when there is none)."""
    s, _ = graph.nearest(*a)
    for k in range(int(48 * 3600 / field.dt) + 1):
        t = field.t0 + k * field.dt
        if s is not None and field.sample(graph.loc[s], t)[2] <= en["avoid_hs_m"]:
            return t
    return field.t0


def run_exp_job(job):
    case_id, aircraft, cap = job
    field = CorridorField.from_case(case_id)
    layer = SolarLayer.load(case_id)
    graph = corridor_graph(field)
    legs = leg_model(aircraft)
    en = energy_defaults(aircraft, cap)
    tab = Tables(graph, field, en, layer)
    pairs, wf = exp_pairs(field.f)
    rows = []
    for name, a, b in pairs:
        t1 = time.time()
        t0 = exp_start(field, graph, en, a)
        r = plan(field, graph, legs, en, a, b, t0=t0, layer=layer, tables=tab)
        r["wall_s"] = time.time() - t1
        r.update(case=case_id, aircraft=aircraft, capacity_factor=cap, pair=name, start=a, dest=b,
                 mean_wind_from_deg=wf, capacity_wh=en["capacity_wh"], reserve_wh=en["reserve_wh"])
        rows.append(r)
    return rows


def _log(msg):
    print(msg, flush=True)


def run_exp(workers=4, cases=None, out=OUT_JSON, log=_log):
    from concurrent.futures import ProcessPoolExecutor
    cases = cases or [s["id"] for s in W.SCENARIOS]
    jobs = [(c, a, k) for c in cases for a in AIRCRAFT for k in EXP_CAPACITY]
    rows = []
    t0 = time.time()
    with ProcessPoolExecutor(max_workers=workers) as ex:
        for res in ex.map(run_exp_job, jobs):
            rows += res
            r = res[0]
            log(f"{r['case']} {r['aircraft']} x{r['capacity_factor']}: " + ", ".join(
                f"{x['pair']} {'ok' if x['ok'] else 'fail ' + str(x['reason'])}" for x in res)
                + f"  ({time.time() - t0:.0f} s)")
    layers = {c: (SolarLayer.load(c) is not None) for c in cases}
    obj = {"meta": record.meta("python -m sim.route exp", MODEL_FILES + [W.OUT_DIR / f"{c}.js" for c in cases]),
           "settings": {"refine": REFINE, "rings": RINGS, "sub_km": SUB_KM, "wait_s": WAIT_S, "eps_soc": EPS_SOC,
                        "eps_slack": EPS_SLACK, "solar_layer": layers},
           "energy": {a: energy_defaults(a) for a in AIRCRAFT}, "rows": rows}
    Path(out).parent.mkdir(parents=True, exist_ok=True)
    with open(out, "w", encoding="utf-8", newline="\n") as fh:
        json.dump(obj, fh, indent=None, separators=(",", ":"), default=float)
    return obj


def main(argv=None):
    ap = argparse.ArgumentParser(prog="python -m sim.route")
    sub = ap.add_subparsers(dest="cmd", required=True)
    p = sub.add_parser("solar")
    p.add_argument("--only", nargs="*")
    p.add_argument("--pace-log", default=None, help="directory of the pace log shared with other fetches")
    sub.add_parser("export")
    p = sub.add_parser("exp")
    p.add_argument("--workers", type=int, default=4)
    p.add_argument("--cases", nargs="*")
    p = sub.add_parser("plan")
    p.add_argument("--case", required=True)
    p.add_argument("--start", required=True)
    p.add_argument("--dest", required=True)
    p.add_argument("--aircraft", default="albatross-3m")
    p.add_argument("--capacity-factor", type=float, default=1.0)
    a = ap.parse_args(argv)
    if a.cmd == "solar":
        make_solar(a.only, a.pace_log)
    elif a.cmd == "export":
        print(export_params())
    elif a.cmd == "exp":
        run_exp(a.workers, a.cases)
    else:
        field = CorridorField.from_case(a.case)
        layer = SolarLayer.load(a.case)
        graph = corridor_graph(field)
        en = energy_defaults(a.aircraft, a.capacity_factor)
        r = plan(field, graph, leg_model(a.aircraft), en, tuple(map(float, a.start.split(","))),
                 tuple(map(float, a.dest.split(","))), layer=layer)
        rt = r["route"] or (r["closest"] or {})
        print(json.dumps({k: r.get(k) for k in ("ok", "reason", "reason_text", "dist_km", "stats", "start_need_wh")},
                         default=float))
        if rt:
            print(json.dumps(rt.get("summary"), default=float))


if __name__ == "__main__":
    main()
