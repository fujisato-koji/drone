"""Strategic layer: the route from a start to a destination over the world sea (D-021, docs/02-form.md#route-planning).

    python -m sim.route solar [--only ID ...] [--pace-log DIR]
        fetch (cached) the ERA5 surface shortwave radiation at every second point of each case's corridor, hourly over
        the case's days and the look-ahead after them, and write sim/viewer/route/solar_<id>.js (the solar layer).
    python -m sim.route export
        write sim/viewer/route_params.js: the energy parameters, the hull points, the steady-leg parameters and the
        solar fit of each aircraft, for sim/viewer/route.js.
    python -m sim.route exp [--workers 4] [--engine node|python]
        the experiments of docs/02-form.md#route-planning into sim/out/route.json (by default planned in node with
        sim/viewer/route.js through sim/route_exp.js, which plans as this module does, sim/tests/test_route_js.py).
    python -m sim.route plan --case ID --start LAT,LON --dest LAT,LON [--aircraft albatross-3m] [--capacity-factor 1]
    python -m sim.route loop INPUT --out OUT [--workers 4] [--engine node|python]
        the laps of a fixed route (the benchmark lap of D-022) from a private input file (the notes above run_loop):
        each start time in both directions on a path graph over a box of weather, the plans and their summaries into
        OUT (node by default, through sim/route_loop.js); nothing else is written and no coordinate is printed.

The planner finds the earliest arrival at the destination subject to the owner's energy rules (D-021):
- an hourly battery: state of charge = previous + solar yield - avionics - motor - take-off, at most the usable
  capacity, never below the reserve;
- the night: from any moment, flying on (the flight draw, no motor) must carry the battery to the end of the next
  night (the next run of hours whose solar yield is below that draw) without passing below the reserve;
- the motor's allowance: in each local solar day the motor (with the take-offs) may use at most a share of the solar
  energy the route collects that day.

Geometry: nodes have latitude and longitude, a leg between two nodes is a great-circle arc flown in sub-steps of at
most SUB_KM, each at the course of the arc at its start and with the wind and waves sampled there and then (no flat
plane beyond a leg). The field (wind and wave height) is pluggable: CorridorField samples a corridor file of
sim.world, BoxField a box of latitude and longitude (months joined in time), with the same locate(), crossings() and
sample(); BoxSolar is a solar layer of 3-hour means on a box. The graph of a corridor is the corridor grid refined
REFINE times, each node linked to the nodes of RINGS rings (one link per direction); links that touch land
(sim/viewer/world/land.js) are left out. The graph of a fixed route (path_graph) is its polyline in nodes at most
SUB_KM apart, each linked to the next, with dry stretches over land, the distance left along the path and months
without waits on the water per node, which the search's hooks read.

Leg model (sim.navigator): for a course, the hull of the highest library wind at or below the sampled wind makes way
by soaring when its best soaring pair reaches MIN_VMG along the course (plan_pair, motor_ok=False), at its speed made
good and no motor power; the steady powered leg (SteadyModel.leg) makes way at its speed and electric power; where
both do, the search tries both (two edges), else the one that does; else the leg is not flyable. The wind is taken
to U_Q and the course off downwind to A_Q (a table, so that the Python and the JavaScript planner take the same
decisions). No wave-ride benefit (the world seas' swell is irregular, Q-081). The total wave height is kept within the
avoidance height (ds.survival_hs_m) at the start of every sub-step, where a sub-step crosses a line of the field's grid,
at the destination on arrival, and at both ends of every wait.

Energy: by day the flight draw is ds.avionics_power_w plus the sensor set less what the avionics hold already, afloat
ds.avionics_power_w; with the sun below the horizon the low-power night draw of D-023, flying or afloat.

Search: a label-setting search with labels (time, state of charge, the day's motor slack, on the water or not),
the labels of a node that no other label of the same node dominates, a priority queue on time plus a lower bound of
the time to go (distance over the fastest speed of the leg model). A label dominates another only if it is at most
DOM_WINDOW_S earlier, with the same day of the allowance and both times in the same local day at the node, in the
same state, and holding on until the other's time keeps the other's charge and slack. The search cannot hold on, so
this is an approximation that the window does not bound: where the sea is still too high, the wind still below a
soaring threshold or the allowance not yet renewed at the earlier time, or where the later label's charge or slack
within the tolerances is needed, the earlier label's best route can be much later, or there is none (K-023). Waiting on the water is
an edge to the end of the hour of the tables (at most WAIT_S); the first leg after a wait pays the take-off. When no
route keeps the rules, the planner names the reason and searches again with the rules relaxed in the steps of RELAX;
the first route found is the closest route. sim/viewer/route.js is the same planner, operation for operation
(sim/tests/test_route_js.py).
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
REFINE = 3              # route.refine: corridor step / REFINE between nodes (route.node_step_km)
RINGS = 2               # route.rings: links to the nodes within this many rings
SUB_KM = 10.0           # route.sub_km: longest sub-step of a leg, km
WAIT_S = 3600.0         # route.wait_h: the longest wait on the water (a wait ends at the end of the hour of the tables)
LOOKAHEAD_H = 36        # route.lookahead_h: hours of the battery tables after the field's last time
U_Q = 0.1               # route.u_q_ms: wind step of the leg table, m/s
A_Q = 1.0               # route.a_q_deg: course step of the leg table, deg off downwind
DOM_WINDOW_S = 600.0    # route.dom_window_s: a label dominates another only this much earlier at most
EPS_SOC = 0.25          # route.eps_soc_wh: the charge a dominating label may lack
EPS_SLACK = 0.25        # route.eps_slack_wh: the same for the motor slack
MIN_VMG = 0.3           # route.min_vmg_ms: least speed made good of a way, as sim.navigator.MIN_VMG
MAX_POPS = 300_000      # route.max_pops: labels the strict search takes at most
RELAX_POPS = 150_000    # route.relax_pops: labels each relaxed search takes at most
RELAX_BELOW = (1.5, 3.0)  # route.relax_below_1, route.relax_below_2: capacities by which the reserve is lowered

# ---------------------------------------------------------------- energy (record keys, see energy_defaults)
BATTERY_MASS_KG = 0.35     # route.battery_mass_kg: D-023 (assumed, Q-082)
PACK_OVERHEAD_KG = 0.04    # route.pack_overhead_kg: wiring, protection and case, the first pack's (250 g less three cells)
USABLE_DEPTH = 0.8         # route.usable_depth: the share of the cells' energy used in daily cycles
RESERVE_FRACTION = 0.2     # route.reserve_fraction: share of the usable capacity kept as a reserve
MOTOR_SHARE = 0.5          # route.motor_share: the motor's share of the day's solar generation (D-021)
TAKEOFF_RUN_S = 10.0       # route.takeoff_run_s: the take-off at full thrust (P82 for the bird's run)
NIGHT_DRAW_W = 1.5         # route.night_draw_w: the low-power night of D-023, flying or afloat (assumed, Q-082)
# the sensor sets flown by day (their keys hold the GNSS receiver and the IMU, which ds.avionics_power_w is taken to
# hold already, COUNTED_KEYS): over the open sea the forward radar without RTK (no base station at sea), near a coast
# with a base the tip radars with RTK
SENSOR_SETS = {"radar_tips_fwd_baro": "est.set_radar_tips_fwd_baro_power_w", "tips_rtk": "est.set_tips_rtk_power_w"}
SENSOR_SET = "radar_tips_fwd_baro"
COUNTED_KEYS = ("sense.gnss_power_w", "sense.imu_power_w")
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
           3: "the closest route keeps the reserve on its way but cannot carry the next night",
           4: "the closest route's draw takes the battery below the reserve",
           5: "the closest route keeps the battery's rules but its motor needs more than the day's allowance",
           6: "no route reaches the destination within the field's time even with the rules relaxed (the seas, the field's end, more energy than the relaxed rules allow, or a relaxed search's limit of labels)",
           7: "only the relaxed search found a route, and it keeps the rules (the strict search's approximations missed it)",
           8: "the strict search stopped at its limit of labels (route.max_pops) without a route, so none is ruled out"}


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

    def crossings(self, a, b):
        """The points where the straight segment from location a to location b (in the grid's indices) crosses a line
        of the grid, ((fraction along, location), ...) in order: with the ends they bound the sea along a step."""
        fi0, fj0, fi1, fj1 = a[0], a[1], b[0], b[1]
        fs = []
        for x0, x1 in ((fi0, fi1), (fj0, fj1)):
            if x1 != x0:
                g = math.floor(min(x0, x1)) + 1
                while g < max(x0, x1):
                    fs.append((g - x0) / (x1 - x0))
                    g += 1
        out = []
        prev = None
        for f in sorted(fs):
            if 0.0 < f < 1.0 and f != prev:
                fi, fj = fi0 + (fi1 - fi0) * f, fj0 + (fj1 - fj0) * f
                out.append((f, (fi, fj, corners(self.na, self.nc, fi, fj))))
                prev = f
        return tuple(out)

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


# ================================================================ a box of latitude and longitude (fixed routes)


def _floats(xs, n, what):
    """A flat array of n floats (None, as JSON's null, for NaN)."""
    if len(xs) != n:
        raise ValueError(f"{what}: {len(xs)} values for {n} points")
    return [float("nan") if x is None else float(x) for x in xs]


BOX_GRID = ("na", "nc", "lat0", "lon0", "step_deg", "dt_h")


def box_join(ds, names):
    """One box dict from several joined in time (months of a horizon of weeks): the same grid and time step, each on
    the time grid of the first and with no gap after the frames before it; a frame at a time an earlier dict holds
    already is left out (the earlier dict's is kept)."""
    ds = sorted(ds, key=lambda d: float(d["t0"]))
    a = ds[0]
    dt = 3600.0 * float(a["dt_h"])
    t_first, t_last = float(a["t0"]), None
    vals = {n: [] for n in names}
    for d in ds:
        for k in BOX_GRID:
            if float(d[k]) != float(a[k]):
                raise ValueError(f"box_join: {k} differs between the dicts")
        for q in range(int(d["nt"])):
            t = float(d["t0"]) + q * dt
            x = (t - t_first) / dt
            if abs(x - round(x)) > 1e-6:
                raise ValueError("box_join: a frame lies off the time grid of the first dict")
            if t_last is not None:
                if t <= t_last + 1e-6:
                    continue
                if t > t_last + dt + 1e-6:
                    raise ValueError("box_join: a gap in time between the dicts")
            for n in names:
                vals[n].append(d["vals"][n][q])
            t_last = t
    return dict({k: a[k] for k in BOX_GRID}, t0=a["t0"], nt=len(vals[names[0]]), vals=vals)


def box_index(box, lat, lon):
    """Fractional grid indices (along the latitudes, along the longitudes) of (lat, lon) in a box (lat0, lon0,
    step), the longitude taken within 180 deg of lon0."""
    d = lon - box.lon0
    if d >= 180.0:
        d -= 360.0
    elif d < -180.0:
        d += 360.0
    return (lat - box.lat0) / box.step, d / box.step


class BoxField(CorridorField):
    """The 10 m wind and the total wave height on a regular latitude-longitude grid (a box), with the corridor's
    interface (locate, crossings, sample, t0, t_end, u10_max, nt, dt) and its sampling (space bilinear over the finite
    corners, time linear). Input {na, nc, lat0, lon0, step_deg, t0, dt_h, nt, vals: {u10, v10, hs}}: each variable a list
    of nt flat arrays of na * nc values, row-major from the south-west (index i * nc + j at lat0 + i step_deg,
    lon0 + j step_deg), frame k at t0 + k dt_h hours, NaN (or None) for missing; hs is NaN over land, as the
    corridor's is. BoxField.join takes several such dicts in time (box_join)."""

    NAMES = ("u10", "v10", "hs")

    def __init__(self, d):
        self.f = {k: d[k] for k in BOX_GRID + ("t0", "nt")}
        self.na, self.nc, self.nt = int(d["na"]), int(d["nc"]), int(d["nt"])
        self.lat0, self.lon0, self.step = float(d["lat0"]), float(d["lon0"]), float(d["step_deg"])
        self.t0, self.dt = float(d["t0"]), 3600.0 * float(d["dt_h"])
        self.t_end = self.t0 + self.dt * (self.nt - 1)
        n = self.na * self.nc
        self.vals = {nm: [_floats(d["vals"][nm][k], n, f"BoxField {nm} frame {k}") for k in range(self.nt)]
                     for nm in self.NAMES}
        u, v = np.asarray(self.vals["u10"], float), np.asarray(self.vals["v10"], float)
        self.u10_max = float(np.nanmax(np.sqrt(u * u + v * v)))

    @classmethod
    def join(cls, ds):
        return cls(box_join([ds] if isinstance(ds, dict) else list(ds), cls.NAMES))

    def locate(self, lat, lon):
        """As CorridorField.locate: (fi, fj, corners), or None outside the box."""
        fi, fj = box_index(self, lat, lon)
        if -1e-9 <= fi <= self.na - 1 + 1e-9 and -1e-9 <= fj <= self.nc - 1 + 1e-9:
            return fi, fj, corners(self.na, self.nc, fi, fj)
        return None


class BoxSolar:
    """The surface shortwave radiation on a box (BoxField's grid) as means over intervals of dt_h whole hours (3-hour
    means): the value stamped s is the mean over [s - dt_h, s). SolarLayer's interface (locate, hour_mean, t0 the start
    of its first hour, nt its hours). hour_mean is the interval's mean, bilinear in space over the finite corners, split
    into its hours in proportion to the fitted model's hourly means at the place (model_hour_mean, whose factor
    route.solar_fit_w_m2 cancels), or held flat where the model's mean over the interval is 0 (assumed,
    docs/02-form.md#route-planning). Input {na, nc, lat0, lon0, step_deg, t0, dt_h, nt, vals: {shortwave_radiation}}
    with t0 the first stamp; BoxSolar.join takes several in time."""

    NAMES = ("shortwave_radiation",)

    def __init__(self, d):
        self.na, self.nc = int(d["na"]), int(d["nc"])
        self.lat0, self.lon0, self.step = float(d["lat0"]), float(d["lon0"]), float(d["step_deg"])
        self.s0, self.ds, self.ns = float(d["t0"]), 3600.0 * float(d["dt_h"]), int(d["nt"])
        self.hours = int(round(self.ds / 3600.0))
        if self.hours < 1 or abs(self.ds - 3600.0 * self.hours) > 1e-6 or abs(self.s0 / 3600.0 - round(self.s0 / 3600.0)) > 1e-9:
            raise ValueError("BoxSolar: the stamps and the interval must be whole hours")
        n = self.na * self.nc
        self.v = [_floats(d["vals"]["shortwave_radiation"][k], n, f"BoxSolar frame {k}") for k in range(self.ns)]
        self.t0 = self.s0 - self.ds
        self.nt = self.ns * self.hours
        self._shape = {}

    @classmethod
    def join(cls, ds):
        return cls(box_join([ds] if isinstance(ds, dict) else list(ds), cls.NAMES))

    def locate(self, lat, lon):
        """(fi, fj, lat, lon), or None outside the box (the tables then take the fitted model)."""
        fi, fj = box_index(self, lat, lon)
        if -1e-9 <= fi <= self.na - 1 + 1e-9 and -1e-9 <= fj <= self.nc - 1 + 1e-9:
            return fi, fj, lat, lon
        return None

    def shape(self, loc, q):
        """The model's hourly means over interval q at loc and their sum."""
        key = (loc[2], loc[3], q)
        r = self._shape.get(key)
        if r is None:
            ta = self.t0 + q * self.ds
            s = [model_hour_mean(loc[2], loc[3], ta + 3600.0 * h, 1.0) for h in range(self.hours)]
            tot = 0.0
            for x in s:
                tot += x
            r = (s, tot)
            self._shape[key] = r
        return r

    def hour_mean(self, loc, t_hour):
        """W/m2 over the hour starting at t_hour, or None outside the box, its hours or its finite values."""
        if loc is None:
            return None
        k = int(round((t_hour - self.t0) / 3600.0))
        if k < 0 or k >= self.nt:
            return None
        q, h = k // self.hours, k % self.hours
        g = bilinear(self.v[q], self.na, self.nc, loc[0], loc[1])
        if not math.isfinite(g):
            return None
        s, tot = self.shape(loc, q)
        m = tot / self.hours
        return g * s[h] / m if m > 0.0 else g


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
    return math.atan2(z, math.sqrt(x * x + y * y)) * R2D, math.atan2(y, x) * R2D


def gc_bearing(lat1, lon1, lat2, lon2):
    """Initial course (deg clockwise from north) from point 1 to point 2."""
    p1, p2 = math.radians(lat1), math.radians(lat2)
    dl = math.radians(lon2 - lon1)
    y = math.sin(dl) * math.cos(p2)
    x = math.cos(p1) * math.sin(p2) - math.sin(p1) * math.cos(p2) * math.cos(dl)
    return wrap360(math.atan2(y, x) * R2D)


R2D = 180.0 / math.pi      # degrees per radian, as route.js computes it (math.degrees rounds differently)


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
    steps a list of (field location, course deg) at the start of each sub-step. A graph along a fixed route
    (path_graph) also carries rem_m, dry_node, dry_edge and no_wait_months, which the search reads (None on a
    corridor's graph)."""

    def __init__(self, lat, lon, loc, edges, step_km):
        self.lat, self.lon, self.loc, self.edges, self.step_km = lat, lon, loc, edges, step_km
        self.rem_m = self.dry_node = self.dry_edge = self.no_wait_months = None

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
    sub-step point on land (land: rings of sim.world.load_land, None to load them, False for none). A leg's steps are
    (location, course, checks) with checks the points where the step crosses a line of the field's grid, where the
    sea is tested as well (field.crossings)."""
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
        locs = []
        for p in pts:
            lc = field.locate(p[0], p[1])
            if lc is None:
                break
            locs.append(lc)
        if len(locs) < len(pts):
            continue
        steps = []
        for q, p in enumerate(pts):
            end = locs[q + 1] if q + 1 < len(locs) else loc[m]
            steps.append((locs[q], gc_bearing(p[0], p[1], lat[m], lon[m]), field.crossings(locs[q], end)))
        edges[n].append((m, sub_m, steps))
    return Graph(lat, lon, loc, edges, h)


def path_graph(field, points, sub_km=SUB_KM, dry=None, land=None):
    """The graph of a fixed route (the benchmark lap of D-022, or any polyline of (lat, lon) points): nodes along the
    polyline at most sub_km apart (each segment in equal parts), every vertex a node, and one leg from each node to the
    next, in steps as corridor_graph builds them (location, course, grid crossings). A closed loop is a polyline
    whose last point equals its first; its end is a node of its own at the start's place, so a lap runs from node 0
    to the last node. dry: ranges (a, b) of vertex indices, a < b, flown over land: the legs from vertex a to vertex b
    are dry and the nodes strictly between them are dry nodes (no wait, no sea test on arrival). land: rings of
    sim.world.load_land (None to load them, False for none) against which the other nodes and the steps of the other
    legs are tested as corridor_graph tests them; a fixed route keeps its legs, so the hits are listed (land_hits: the
    nodes, and the legs by their start node), not removed. The graph carries rem_m (each node's distance along the
    path to the last node, m), vertex_node (each vertex's node), seg (the vertex each node follows), dry_node,
    dry_edge (per node, per leg) and no_wait_months (per node a set of UTC months 1..12 in which the search does not
    wait on the water there; empty here, set_no_wait sets them)."""
    pts = [(float(p[0]), float(p[1])) for p in points]
    if len(pts) < 2:
        raise ValueError("path_graph: a route needs two points at least")
    lat, lon, seg, vnode = [], [], [], []
    for i in range(len(pts) - 1):
        (la1, lo1), (la2, lo2) = pts[i], pts[i + 1]
        L = gc_km(la1, lo1, la2, lo2)
        if L < 1e-9:
            raise ValueError(f"path_graph: vertices {i} and {i + 1} coincide")
        ns = max(1, int(math.ceil(L / sub_km - 1e-9)))
        vnode.append(len(lat))
        for k in range(ns):
            p = (la1, lo1) if k == 0 else gc_point(la1, lo1, la2, lo2, k / ns)
            lat.append(p[0])
            lon.append(p[1])
            seg.append(i)
    vnode.append(len(lat))
    lat.append(pts[-1][0])
    lon.append(pts[-1][1])
    seg.append(len(pts) - 1)
    N = len(lat)
    loc = [field.locate(lat[n], lon[n]) for n in range(N)]
    if any(lc is None for lc in loc):
        raise ValueError(f"path_graph: {sum(1 for lc in loc if lc is None)} nodes lie outside the field")
    dry_node, dry_edge = [False] * N, [[False] for _ in range(N - 1)] + [[]]
    for a, b in (dry or ()):
        a, b = int(a), int(b)
        if not 0 <= a < b <= len(pts) - 1:
            raise ValueError(f"path_graph: dry range ({a}, {b}) is not a < b within the vertices")
        for n in range(vnode[a], vnode[b]):
            dry_edge[n][0] = True
        for n in range(vnode[a] + 1, vnode[b]):
            dry_node[n] = True
    edges, length, pts_wet = [], [], []
    for n in range(N - 1):
        m = n + 1
        L = gc_km(lat[n], lon[n], lat[m], lon[m])
        ns = max(1, int(math.ceil(L / sub_km - 1e-9)))
        pp = [gc_point(lat[n], lon[n], lat[m], lon[m], k / ns) for k in range(ns)]
        locs = [field.locate(p[0], p[1]) for p in pp]
        if any(lc is None for lc in locs):
            raise ValueError(f"path_graph: a step of the leg from node {n} lies outside the field")
        steps = [(locs[q], gc_bearing(pp[q][0], pp[q][1], lat[m], lon[m]),
                  field.crossings(locs[q], locs[q + 1] if q + 1 < ns else loc[m])) for q in range(ns)]
        edges.append([(m, L * 1000.0 / ns, steps)])
        length.append(L * 1000.0)
        if not dry_edge[n][0]:
            chk = [gc_point(lat[n], lon[n], lat[m], lon[m], (k + 0.5) / ns) for k in range(ns)]
            pts_wet.append((n, pp[1:] + chk))
    edges.append([])
    rem = [0.0] * N
    for n in range(N - 2, -1, -1):
        rem[n] = rem[n + 1] + length[n]
    rings = W.load_land() if land is None else (land or [])
    hit_nodes, hit_legs = [], []
    if rings:
        wet = [n for n in range(N) if not dry_node[n]]
        on = W.on_land(rings, [lat[n] for n in wet], [lon[n] for n in wet])
        hit_nodes = [n for n, x in zip(wet, on) if x]
        allp = [p for _, ps in pts_wet for p in ps]
        if allp:
            on = W.on_land(rings, [p[0] for p in allp], [p[1] for p in allp])
            k = 0
            for n, ps in pts_wet:
                if bool(np.any(on[k:k + len(ps)])):
                    hit_legs.append(n)
                k += len(ps)
    g = Graph(lat, lon, loc, edges, sub_km)
    g.rem_m, g.dry_node, g.dry_edge = rem, dry_node, dry_edge
    g.no_wait_months = [set() for _ in range(N)]
    g.vertex_node, g.seg = vnode, seg
    g.land_hits = {"nodes": hit_nodes, "legs": hit_legs}
    return g


def set_no_wait(graph, months, vertices=None):
    """Forbid waiting on the water in the UTC months `months` (1..12) at the nodes of a path graph from vertex a to
    vertex b (vertices (a, b), both included; None: every node)."""
    if vertices is None:
        a, b = 0, len(graph.lat) - 1
    else:
        a, b = graph.vertex_node[int(vertices[0])], graph.vertex_node[int(vertices[1])]
        if a > b:
            raise ValueError("set_no_wait: vertices (a, b) need a <= b")
    ms = {int(x) for x in months}
    if not ms <= set(range(1, 13)):
        raise ValueError("set_no_wait: months are 1..12")
    for n in range(a, b + 1):
        graph.no_wait_months[n] |= ms


def utc_month(t):
    """The UTC month (1..12) of the unix time t."""
    return (_dt.date(1970, 1, 1) + _dt.timedelta(days=int(math.floor(t / 86400.0)))).month


# ================================================================ leg model


class LegModel:
    """The speed made good along a course by soaring and by the steady powered leg, and the latter's electric power,
    from the wind and the course off downwind (module notes)."""

    def __init__(self, source, steady):
        self.source, self.steady = source, steady
        self.winds = [h.u10 for h in source.hulls]          # ascending (HullSource sorts)
        self.cache = {}

    def hull_below(self, u):
        """The hull of the highest library wind at or below u (the lowest when u is below all): conservative."""
        k = 0
        for q, w in enumerate(self.winds):
            if w <= u + 1e-9:
                k = q
        return self.source.hulls[k]

    def hull_for(self, uq):
        """The hull for the winds rounded to uq: at or below every wind of the step (from uq - U_Q / 2)."""
        return self.hull_below(uq - 0.5 * U_Q)

    def leg(self, u10, rel_deg):
        """(soaring speed made good, steady speed made good, steady electric power) along the course; a speed is 0
        where that way makes no way (below MIN_VMG)."""
        iu = int(math.floor(u10 / U_Q + 0.5))
        ia = int(math.floor(abs(wrap180(rel_deg)) / A_Q + 0.5))
        key = iu * 1000 + ia
        r = self.cache.get(key)
        if r is None:
            uq, aq = iu * U_Q, math.radians(ia * A_Q)
            d = (math.cos(aq), math.sin(aq))
            pl = self.hull_for(uq).plan_pair(d, MIN_VMG, motor_ok=False)
            lg = self.steady.leg(uq, d, MIN_VMG)
            r = (pl["r"] if pl is not None else 0.0, lg["along"] if lg is not None else 0.0,
                 lg["power"] if lg is not None else 0.0)
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


def pack():
    """(cells, the cells' mass kg, the cells' share of the battery mass) of route.battery_mass_kg: the whole P45B
    cells (lit.cell_p45b_mass_g) that fit beside route.pack_overhead_kg."""
    cell = record.value("lit.cell_p45b_mass_g") / 1000.0
    n = int(math.floor((BATTERY_MASS_KG - PACK_OVERHEAD_KG) / cell + 1e-9))
    return n, n * cell, n * cell / BATTERY_MASS_KG


def energy_defaults(aircraft, capacity_factor=1.0, sensor_set=SENSOR_SET):
    """The planner's energy parameters from the record (docs/02-form.md#route-planning). sensor_set: the set flown by
    day, "radar_tips_fwd_baro" over the open sea (no RTK base station), "tips_rtk" near a coast with a base."""
    from sim.aircraft_models import by_name
    from sim.dsopt import prop_efficiency, thrust_max
    from sim.patterns import SOLAR_CELL_EFF, SOLAR_COVERAGE
    from sim.navigator import STEADY_HEIGHT_M, SteadyModel
    ac = by_name(aircraft)
    _, _, frac = pack()
    cap = BATTERY_MASS_KG * record.value("lit.cell_p45b_wh_per_kg") * frac * USABLE_DEPTH
    cap *= capacity_factor
    eta = prop_efficiency()
    v_to = SteadyModel.from_aircraft(ac).v_factor * ac.v_stall
    to_j = thrust_max(ac) * v_to / eta * TAKEOFF_RUN_S + ac.mass * 9.81 * STEADY_HEIGHT_M / eta
    av = record.value("ds.avionics_power_w")
    sensors = record.value(SENSOR_SETS[sensor_set]) - sum(record.value(k) for k in COUNTED_KEYS)
    return {"capacity_wh": cap, "reserve_wh": RESERVE_FRACTION * cap, "soc0_wh": cap, "reserve_fraction": RESERVE_FRACTION,
            "flight_draw_w": av + sensors, "water_draw_w": av, "night_draw_w": NIGHT_DRAW_W, "sensor_set": sensor_set,
            "panel_m2": float(ac.wing_area) * SOLAR_COVERAGE * SOLAR_CELL_EFF,
            "takeoff_wh": to_j / 3600.0, "takeoff_s": TAKEOFF_RUN_S, "motor_share": MOTOR_SHARE,
            "avoid_hs_m": record.value("ds.survival_hs_m"), "solar_fit_w_m2": solar_fit()}


QUARTERS = (0.125, 0.375, 0.625, 0.875)


class Tables:
    """Per node and hour from T0: the panel's mean power P, the mean draw flying (F) and afloat (Wd), by day the
    flight or the water draw and by night (the sun below the horizon, in quarter hours) the night draw; the
    requirement at the end of the hour for the night check (need, inf when the night needs more than the capacity;
    and for margins the same without the capacity's limit during the night and the shortfall a full battery leaves);
    and the solar energy left in the node's local day from the start of the hour (rest)."""

    def __init__(self, graph, field, en, layer=None, reserve=None, t_start=None, hours=None):
        # from a day before the field's start (the start's day counts its solar energy before the start)
        self.T0 = math.floor((field.t0 if t_start is None else t_start) / 3600.0) * 3600.0 - (86400.0 if t_start is None else 0.0)
        self.K = int(math.ceil((field.t_end - self.T0) / 3600.0)) + LOOKAHEAD_H if hours is None else hours
        self.en = en
        fit = en["solar_fit_w_m2"]
        R = en["reserve_wh"] if reserve is None else reserve
        C = en["capacity_wh"]
        fd, wd, nd = en["flight_draw_w"], en["water_draw_w"], en["night_draw_w"]
        self.P, self.F, self.Wd, self.need, self.need_raw, self.rest, self.day_start = [], [], [], [], [], [], []
        self.layer_hours = 0         # hours of the tables that the layer covers (the model the others)
        for n in range(len(graph.lat)):
            if graph.loc[n] is None:
                for a in (self.P, self.F, self.Wd, self.need, self.need_raw, self.rest, self.day_start):
                    a.append(None)
                continue
            la, lo = graph.lat[n], graph.lon[n]
            ll = layer.locate(la, lo) if layer is not None else None
            P, F, Wd = [], [], []
            for k in range(self.K):
                th = self.T0 + 3600.0 * k
                s4 = [sun_sin_elev(la, lo, th + q * 3600.0) for q in QUARTERS]
                nf = sum(1 for x in s4 if x < 0.0) / 4.0
                g = layer.hour_mean(ll, th) if layer is not None else None
                if g is None:
                    s = 0.0
                    for x in s4:
                        s += max(0.0, x)
                    g = fit * s / 4.0
                P.append(g * en["panel_m2"])
                F.append(nf * nd + (1.0 - nf) * fd)
                Wd.append(nf * nd + (1.0 - nf) * wd)
            self.P.append(P)
            self.F.append(F)
            self.Wd.append(Wd)
            net = [P[k] - F[k] for k in range(self.K)]
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

    def energy(self, n, ta, tb, water):
        """(solar energy, draw) in Wh at node n from ta to tb, afloat or flying."""
        P, D = self.P[n], (self.Wd[n] if water else self.F[n])
        s = d = 0.0
        t = ta
        while t < tb - 1e-9:
            k = self._hour(t)
            te = min(tb, self.T0 + 3600.0 * (k + 1))
            if 0 <= k < self.K:
                s += P[k] * (te - t) / 3600.0
                d += D[k] * (te - t) / 3600.0
            else:
                d += (self.en["water_draw_w"] if water else self.en["flight_draw_w"]) * (te - t) / 3600.0
            t = te
        return s, d

    def need_at(self, n, t, reserve, raw=False):
        """The charge that carries the next night from node n at t (inf when it needs more than the capacity; raw:
        the charge that would carry it, the capacity's limit counted as a shortfall)."""
        k = self._hour(t)
        if k < 0 or k >= self.K:
            return reserve
        frac = (self.T0 + 3600.0 * (k + 1) - t) / 3600.0
        net = self.P[n][k] - self.F[n][k]
        if not raw:
            return max(reserve, self.need[n][k] - net * frac)
        cap, short = self.need_raw[n]
        return max(reserve, cap[k] - net * frac) + short[k]

    def fly_on_min(self, n, t, soc):
        """The lowest charge flying on from node n at t without the motor, with the node's solar energy and the
        capacity's limit hour by hour, to the end of the next run of hours whose solar energy is below the flight
        draw (the run of need_at)."""
        k = self._hour(t)
        if k < 0 or k >= self.K:
            return soc
        C = self.en["capacity_wh"]
        P, F = self.P[n], self.F[n]
        net = P[k] - F[k]
        x = min(C, soc + net * (self.T0 + 3600.0 * (k + 1) - t) / 3600.0)
        lo = min(soc, x)
        neg = net < 0
        k += 1
        while k < self.K:
            net = P[k] - F[k]
            if neg and net >= 0:
                break
            if net < 0:
                neg = True
            x = min(C, x + net)
            lo = min(lo, x)
            k += 1
        return lo

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
           v_max=None, max_pops=MAX_POPS):
    """Earliest arrival at `dest` from `start` (node indices) at unix time t0 (module notes). share None: the
    allowance of D-021 (en["motor_share"]); share math.inf: no allowance. Returns (labels, the arrival label or
    None, stats). On a graph of a fixed route (path_graph) the lower bound of the time to go is the distance left along
    the path (rem_m) over the fastest speed, the search does not wait at a dry node or at a node whose no_wait_months
    hold the label's UTC month, a dry leg flies the steady powered leg only and tests no sea, and the arrival at a dry
    node tests no sea; stats["reached"] is the farthest node taken."""
    R = en["reserve_wh"] if reserve is None else reserve
    C = en["capacity_wh"]
    sh = en["motor_share"] if share is None else share
    use_slack = math.isfinite(sh)
    avoid = en["avoid_hs_m"]
    H = field.t_end if horizon is None else horizon
    vm = legs.v_max(field.u10_max) if v_max is None else v_max
    dl, dn = graph.lat[dest], graph.lon[dest]
    hcache = {}
    rem = getattr(graph, "rem_m", None)
    dry_node = getattr(graph, "dry_node", None)
    dry_edge = getattr(graph, "dry_edge", None)
    no_wait = getattr(graph, "no_wait_months", None)
    path = rem is not None or dry_node is not None or no_wait is not None
    months = {}

    def heur(n):
        v = hcache.get(n)
        if v is None:
            if rem is not None:
                v = max(0.0, rem[n] - rem[dest]) / vm
            else:
                v = gc_km(graph.lat[n], graph.lon[n], dl, dn) * 1000.0 / vm
            hcache[n] = v
        return v

    def may_wait(n, t):
        """No wait at a dry node, nor at a node whose no_wait_months hold the UTC month of t."""
        if dry_node is not None and dry_node[n]:
            return False
        if no_wait is not None and no_wait[n]:
            d = math.floor(t / 86400.0)
            mo = months.get(d)
            if mo is None:
                mo = utc_month(t)
                months[d] = mo
            return mo not in no_wait[n]
        return True

    # label arrays
    LT, LS, LK, LD, LW, LN, LP, LE, LA = [], [], [], [], [], [], [], [], []
    pareto = {}                 # (node, time bucket of DOM_WINDOW_S) -> labels
    T0 = tab.T0
    WIN = DOM_WINDOW_S

    def dom(n, ta, sa, ka, da, wa, tb, sb, kb, db, wb):
        """Label a is at least as good as label b at node n: not later and at most DOM_WINDOW_S earlier, with the same
        day of the allowance and both times in the same local day at the node (no renewal of the allowance between
        them), in the same state (afloat or flying), and holding on until b's time (the node's solar energy and draw,
        at most the capacity) it has b's charge to EPS_SOC and b's motor slack to EPS_SLACK. The search cannot hold
        on, so this is an approximation that the window does not bound (docs/02-form.md#route-planning, K-023)."""
        if ta > tb or tb - ta > WIN or da != db or wa != wb:
            return False
        lo = graph.lon[n]
        if local_day(ta, lo) != local_day(tb, lo):
            return False
        x, sol = sa, 0.0
        if tb > ta:
            sol, dr = tab.energy(n, ta, tb, wa)
            x = min(C, sa + sol - dr)
        if x < sb - EPS_SOC:
            return False
        return not use_slack or ka + sh * sol >= kb - EPS_SLACK

    def bucket(t):
        return math.floor((t - T0) / WIN)

    def dominated(n, t, soc, slack, day, water):
        b = bucket(t)
        for bb in (b - 1, b):
            for li in pareto.get((n, bb), ()):
                if LA[li] and dom(n, LT[li], LS[li], LK[li], LD[li], LW[li], t, soc, slack, day, water):
                    return True
        return False

    def add(n, t, soc, slack, day, water, parent, edge):
        """A new label, or None when one at the node dominates it; it removes the labels it dominates."""
        if dominated(n, t, soc, slack, day, water):
            why["dominated"] += 1
            return None
        b = bucket(t)
        for bb in (b, b + 1):
            lst = pareto.get((n, bb))
            if not lst:
                continue
            for li in lst:
                if LA[li] and dom(n, t, soc, slack, day, water, LT[li], LS[li], LK[li], LD[li], LW[li]):
                    LA[li] = False
            lst[:] = [li for li in lst if LA[li]]
        i = len(LT)
        LT.append(t), LS.append(soc), LK.append(slack), LD.append(day), LW.append(water), LN.append(n)
        LP.append(parent), LE.append(edge), LA.append(True)
        pareto.setdefault((n, b), []).append(i)
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

    def fly(n, m, sub_m, steps, t, soc, slack, day, water, steady_first, dry=False):
        """One leg from node n (soaring where the hull makes way, else the steady leg; or the steady leg where it
        makes way, else soaring; a dry leg the steady leg only, with no sea test): (arrival time, charge, slack, day,
        edge, soared) or (None, soared)."""
        tt, s2, k2, d2 = t, soc, slack, day
        lon_n = graph.lon[n]
        motor = draw = sol_tot = to = 0.0
        modes, days = [], {}
        soared = False
        s_min = soc
        if water:
            dd = local_day(tt, lon_n)
            if dd > d2:
                if use_slack and k2 < -EPS_SLACK:
                    why["allowance"] += 1
                    return None, soared
                d2, k2 = dd, 0.0
            to = en["takeoff_wh"]
            s2 -= to
            k2 -= to if use_slack else 0.0
            motor += to
            days[d2] = [0.0, to]
            tt += en["takeoff_s"]
            s_min = min(s_min, s2)
            if s2 < R - 1e-9:
                why["reserve"] += 1
                return None, soared
        for lc, brg, checks in steps:
            if tt > H:
                why["horizon"] += 1
                return None, soared
            u, v, hs = field.sample(lc, tt)
            if dry:
                if not (math.isfinite(u) and math.isfinite(v)):
                    why["no_leg"] += 1
                    return None, soared
            elif not (hs <= avoid):
                why["seas"] += 1
                return None, soared
            sp = math.sqrt(u * u + v * v)
            wf = wrap360(math.atan2(-u, -v) * R2D)
            vs_soar, vs_mot, pw_mot = legs.leg(sp, brg - wf - 180.0)
            if dry:
                if vs_mot > 0.0:
                    mode, vs, pw = "motor", vs_mot, pw_mot
                else:
                    why["no_leg"] += 1
                    return None, soared
            elif not steady_first and vs_soar > 0.0:
                mode, vs, pw = "soaring", vs_soar, 0.0
            elif vs_mot > 0.0:
                mode, vs, pw = "motor", vs_mot, pw_mot
            elif vs_soar > 0.0:
                mode, vs, pw = "soaring", vs_soar, 0.0
            else:
                why["no_leg"] += 1
                return None, soared
            soared = soared or mode == "soaring"
            dt = sub_m / vs
            for f, lc2 in (() if dry else checks):
                if not (field.sample(lc2, tt + f * dt)[2] <= avoid):
                    why["seas"] += 1
                    return None, soared
            dd = local_day(tt, lon_n)
            if dd > d2:
                if use_slack and k2 < -EPS_SLACK:
                    why["allowance"] += 1
                    return None, soared
                d2, k2 = dd, 0.0
            sol, dr = tab.energy(n, tt, tt + dt, False)
            mo = pw * dt / 3600.0
            s2 = min(C, s2 + sol - dr - mo)
            if use_slack:
                k2 += sh * sol - mo
            e = days.setdefault(d2, [0.0, 0.0])
            e[0] += sol
            e[1] += mo
            motor += mo
            draw += dr
            sol_tot += sol
            tt += dt
            modes.append(mode)
            s_min = min(s_min, s2)
            if s2 < R - 1e-9:
                why["reserve"] += 1
                return None, soared
            if use_slack:
                rest = tab.rest_at(n, tt) if local_day(tt, lon_n) == d2 else 0.0
                if k2 + sh * rest < -EPS_SLACK:
                    why["allowance"] += 1
                    return None, soared
        if not (dry_node is not None and dry_node[m]) and not (field.sample(graph.loc[m], tt)[2] <= avoid):
            why["seas"] += 1
            return None, soared
        if not ok_after(m, tt, s2, k2, d2):
            return None, soared
        e = ("leg", n, m, t, tt, tuple(modes), motor, draw, sol_tot, to,
             tuple((d, v[0], v[1]) for d, v in sorted(days.items())), s_min)
        return (tt, s2, k2, d2, e), soared

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
        if path and n > stats.get("reached", -1):
            stats["reached"] = n
        if n == dest:
            stats["labels"] = len(LT)
            return (LT, LS, LK, LD, LW, LN, LP, LE), li, stats
        t, soc, slack, day, water = LT[li], LS[li], LK[li], LD[li], LW[li]
        if t >= H:
            continue
        lon_n = graph.lon[n]
        # ---- wait on the water until the end of the hour of the tables (WAIT_S at most)
        te = min(t + WAIT_S, T0 + 3600.0 * (math.floor((t - T0) / 3600.0) + 1))
        if te <= H + 1e-9 and (not path or may_wait(n, t)):
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
                    sol, dr = tab.energy(n, t, te, True)
                    s2 = min(C, soc + sol - dr)
                    k2 = k2 + (sh * sol if use_slack else 0.0)
                    if s2 >= R - 1e-9 and ok_after(n, te, s2, k2, d2) and not (n == dest):
                        e = ("wait", n, n, t, te, ("waiting",), 0.0, dr, sol, 0.0, ((d2, sol, 0.0),), min(soc, s2))
                        j = add(n, te, s2, k2, d2, True, li, e)
                        if j is not None:
                            seq += 1
                            heapq.heappush(heap, (te + heur(n), seq, j))
        # ---- legs: soaring where the hull makes way, and where a step soared also the steady leg (two edges)
        de = dry_edge[n] if dry_edge is not None else None
        for q, (m, sub_m, steps) in enumerate(graph.edges[n]):
            dry = de is not None and de[q]
            for steady_first in (False, True):
                r, soared = fly(n, m, sub_m, steps, t, soc, slack, day, water, steady_first, dry)
                if r is not None:
                    tt, s2, k2, d2, e = r
                    j = add(m, tt, s2, k2, d2, False, li, e)
                    if j is not None:
                        seq += 1
                        heapq.heappush(heap, (tt + heur(m), seq, j))
                if not soared:
                    break
    stats["labels"] = len(LT)
    return (LT, LS, LK, LD, LW, LN, LP, LE), None, stats


def route_of(graph, labels, li):
    """The route of label li: its waypoints (node, lat, lon, t, soc, slack, day, on the water) and its legs and
    waits (with the solar and motor energy of each local day they touch)."""
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
                    "slack_wh": LK[li], "day": LD[li], "water": LW[li]})
        e = LE[li]
        if e is None:
            continue
        kind, a, b, ta, tb, modes, motor, draw, sol, to, days, smin = e
        mode = "waiting" if kind == "wait" else (modes[0] if len(set(modes)) == 1 else "mixed")
        legs.append({"from": a, "to": b, "t0": ta, "t1": tb, "mode": mode, "modes": list(modes), "motor_wh": motor,
                     "draw_wh": draw, "solar_wh": sol, "takeoff_wh": to, "soc_end_wh": LS[li],
                     "soc_min_wh": smin, "days": [list(x) for x in days]})
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
    """Lowest margins of a route against the strict rules (negative: the shortfall), Wh, by the same bookkeeping as
    the search: the charge above the reserve at its waypoints and at every sub-step of its legs; the lowest charge above the reserve flying on from each
    waypoint without the motor to the end of the next night, the capacity's limit applied (Tables.fly_on_min); and
    the worst local day's allowance less its motor and take-off energy (the start's day with its solar energy before
    the start, the arrival day with the rest of that day's solar energy at the destination, as the search counts)."""
    R = en["reserve_wh"]
    soc_m = min([w["soc_wh"] - R for w in wps] + [lg["soc_min_wh"] - R for lg in legs])
    night_m = min(tab.fly_on_min(w["node"], w["t"], w["soc_wh"]) - R for w in wps)
    s0, s1 = wps[0], wps[-1]
    sol = {s0["day"]: tab.day_total(s0["node"], s0["t"]) - tab.rest_at(s0["node"], s0["t"])}
    mot = {}
    for lg in legs:
        for d, s, m in lg["days"]:
            sol[d] = sol.get(d, 0.0) + s
            mot[d] = mot.get(d, 0.0) + m
    if local_day(s1["t"], s1["lon"]) == s1["day"]:
        sol[s1["day"]] = sol.get(s1["day"], 0.0) + tab.rest_at(s1["node"], s1["t"])
    allow_m = min(en["motor_share"] * sol.get(d, 0.0) - mot.get(d, 0.0) for d in set(sol) | set(mot))
    return soc_m, night_m, allow_m


# the relaxation ladder: the reserve (and with it the night's requirement) lowered by RELAX_BELOW capacities, the
# second also without the motor's allowance; the first rung that finds a route gives the closest route
RELAX = (("reserve_lowered_1", {"below": RELAX_BELOW[0]}),
         ("reserve_lowered_2_no_allowance", {"below": RELAX_BELOW[1], "share": math.inf}))


def plan(field, graph, legs, en, start_ll, dest_ll, t0=None, layer=None, tables=None, relax=True, max_pops=MAX_POPS,
         relax_pops=RELAX_POPS, start_node=None, dest_node=None):
    """Plan a route (module notes). Returns a dict: ok, reason (code, text), route (waypoints, legs, summary),
    closest (the closest route when not ok, with the relaxation that found it and its margins), stats, and relaxed
    (each relaxed search tried: its name, whether it found a route, its labels taken and whether it stopped at its
    limit). start_node, dest_node: node indices in place of the nearest nodes to start_ll and dest_ll (a lap's start
    and end coincide: node 0 and the last node of its path graph); a path graph's plan also gives path_km."""
    t0 = field.t0 if t0 is None else float(t0)
    tab = tables or Tables(graph, field, en, layer)
    s, ds_ = graph.nearest(*start_ll) if start_node is None else (int(start_node), 0.0)
    d, dd_ = graph.nearest(*dest_ll) if dest_node is None else (int(dest_node), 0.0)
    out = {"start_node": s, "dest_node": d, "t0": t0, "start_snap_km": ds_, "dest_snap_km": dd_,
           "dist_km": gc_km(graph.lat[s], graph.lon[s], graph.lat[d], graph.lon[d]) if s is not None and d is not None else None}
    rem = getattr(graph, "rem_m", None)
    if rem is not None and s is not None and d is not None:
        out["path_km"] = (rem[s] - rem[d]) / 1000.0
    if s is None or d is None or graph.loc[s] is None or field.sample(graph.loc[s], t0)[2] > en["avoid_hs_m"]:
        out.update(ok=False, reason=1, reason_text=REASONS[1], route=None, closest=None)
        return out
    vm = legs.v_max(field.u10_max)
    R, C = en["reserve_wh"], en["capacity_wh"]
    # reason 2: the coming night cannot be carried even from the start (the strict requirement); the raw one is kept
    out["start_need_wh"] = tab.need_at(s, t0, R, raw=True)
    reason = 2 if min(C, en["soc0_wh"]) < tab.need_at(s, t0, R) - 1e-9 else None
    out["route"] = None
    out["closest"] = None
    capped = False
    if reason is None:
        labels, li, st = search(graph, field, legs, tab, en, s, d, t0, v_max=vm, max_pops=max_pops)
        out["stats"] = st
        capped = st["capped"]
        if li is not None:
            wps, lg = route_of(graph, labels, li)
            out.update(ok=True, reason=0, reason_text=REASONS[0], route={"waypoints": wps, "legs": lg,
                                                                          "summary": summarize(lg, wps, t0)})
            return out
    out["ok"] = False
    if not relax:
        out.update(reason=reason or 6, reason_text=REASONS[reason or 6])
        return out
    # the relaxation ladder (RELAX); a rung that pruned nothing for the reserve or the night skips to the last rung
    skip = False
    strict_reason = reason
    out["relaxed"] = []
    for name, kw in RELAX:
        if skip and "share" not in kw:
            continue
        rv = R - kw["below"] * C
        tab2 = Tables(graph, field, en, layer, reserve=rv, t_start=tab.T0, hours=tab.K)
        labels, li, st2 = search(graph, field, legs, tab2, en, s, d, t0, v_max=vm, max_pops=relax_pops,
                                 reserve=rv, share=kw.get("share"))
        out["relaxed"].append({"relaxation": name, "found": li is not None, "pops": st2["pops"],
                               "capped": st2["capped"]})
        if li is None:
            skip = st2["pruned"]["reserve"] == 0 and st2["pruned"]["night"] == 0
            continue
        wps, lg = route_of(graph, labels, li)
        sm, nm, am = strict_margin(graph, tab, en, wps, lg)
        out["closest"] = {"relaxation": name, "waypoints": wps, "legs": lg, "summary": summarize(lg, wps, t0),
                          "reserve_margin_wh": sm, "night_margin_wh": nm, "allowance_margin_wh": am, "stats": st2}
        if reason is None:
            reason = 4 if sm < -1e-6 else 3 if nm < -1e-6 else 5 if am < -1e-6 else 7
        break
    if strict_reason is None and capped:
        reason = 8                  # a limit of labels, not the rules, ended the search (the closest route is kept)
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


def js_settings():
    """The planner's settings as route.js takes them (its DEFAULTS)."""
    return {"refine": REFINE, "rings": RINGS, "sub_km": SUB_KM, "wait_s": WAIT_S, "lookahead_h": LOOKAHEAD_H,
            "u_q": U_Q, "a_q": A_Q, "eps_soc": EPS_SOC, "eps_slack": EPS_SLACK, "min_vmg": MIN_VMG,
            "dom_window_s": DOM_WINDOW_S, "max_pops": MAX_POPS, "relax_pops": RELAX_POPS,
            "relax_below": list(RELAX_BELOW)}


def js_hulls(aircraft):
    """The hull points of an aircraft as route.js's LegModel takes them."""
    return [{"u10": u, "points": [dict(vx=vx, vy=vy, power=pw, kind=kd, key=key, side=sd) for vx, vy, pw, kd, key, sd in pts]}
            for u, pts in hull_points(aircraft)]


def export_params(path=PARAMS_JS):
    obj = {"settings": js_settings(), "aircraft": {}}
    for a in AIRCRAFT:
        obj["aircraft"][a] = {"energy": energy_defaults(a), "steady": steady_params(a), "hulls": js_hulls(a)}
    from sim.mpc import LIB_MOTOR_PATH
    path = Path(path)
    with open(path, "w", encoding="utf-8", newline="\n") as fh:
        fh.write(record.header("python -m sim.route export", MODEL_FILES + [Path(LIB_MOTOR_PATH), ROOT / "record" / "values.yaml"]) + "\n")
        fh.write("window.ROUTE_PARAMS =\n" + json.dumps(obj, separators=(",", ":")) + ";\n")
    return path


# ================================================================ experiments

EXP_PAIRS = (("downwind", 0.0, 400.0), ("upwind", 180.0, 200.0), ("across", 90.0, 300.0))
EXP_CAPACITY = (0.5, 1.0, 2.0)
EDGE_MARGIN_KM = 30.0      # route.edge_margin_km


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


START_WINDOW_H = 48       # route.start_window_h
EDGE_MARGIN_KM_KEY = "route.edge_margin_km"


def exp_start(field, graph, en, a):
    """The start of an experiment: the first of the field's times within START_WINDOW_H of its first at which the sea
    at the start node is within the avoidance height (the field's first time when there is none)."""
    s, _ = graph.nearest(*a)
    for k in range(int(START_WINDOW_H * 3600 / field.dt) + 1):
        t = field.t0 + k * field.dt
        if s is not None and field.sample(graph.loc[s], t)[2] <= en["avoid_hs_m"]:
            return t
    return field.t0


def exp_jobs(cases):
    """The experiments' plans: (case, aircraft, capacity factor, [(pair, start, destination, start time)]) with the
    pairs of exp_pairs and the starts of exp_start, and the cases' mean wind directions."""
    jobs, winds = [], {}
    for c in cases:
        field = CorridorField.from_case(c)
        graph = corridor_graph(field)
        pairs, wf = exp_pairs(field.f)
        winds[c] = wf
        for a in AIRCRAFT:
            en = energy_defaults(a)
            pt = [(name, p0, p1, exp_start(field, graph, en, p0)) for name, p0, p1 in pairs]
            for k in EXP_CAPACITY:
                jobs.append((c, a, k, pt))
    return jobs, winds


def run_exp_job(job):
    """One job in Python (engine "python")."""
    case_id, aircraft, cap, pairs = job
    field = CorridorField.from_case(case_id)
    layer = SolarLayer.load(case_id)
    graph = corridor_graph(field)
    legs = leg_model(aircraft)
    en = energy_defaults(aircraft, cap)
    tab = Tables(graph, field, en, layer)
    rows = []
    for name, a, b, t0 in pairs:
        t1 = time.time()
        r = plan(field, graph, legs, en, a, b, t0=t0, layer=layer, tables=tab)
        r["wall_s"] = time.time() - t1
        rows.append(r)
    return rows


def run_exp_job_node(job):
    """One job in node with sim/viewer/route.js (engine "node", sim/route_exp.js)."""
    import shutil
    import subprocess
    case_id, aircraft, cap, pairs = job
    inp = {"case": case_id, "aircraft": aircraft, "cap": cap, "pairs": [[list(a), list(b), t0] for _, a, b, t0 in pairs]}
    r = subprocess.run([shutil.which("node"), str(HERE / "route_exp.js")], input=json.dumps(inp), capture_output=True,
                       text=True, timeout=4 * 3600)
    if r.returncode:
        raise RuntimeError(r.stderr[-3000:])
    return json.loads(r.stdout)


def _log(msg):
    print(msg, flush=True)


def run_exp(workers=4, cases=None, out=OUT_JSON, log=_log, engine="node"):
    """The experiments into sim/out/route.json. engine "node" plans with sim/viewer/route.js, which gives the same
    plans as sim/route.py (sim/tests/test_route_js.py) in a fraction of the time; "python" plans with sim/route.py."""
    from concurrent.futures import ProcessPoolExecutor, ThreadPoolExecutor
    cases = cases or [s["id"] for s in W.SCENARIOS]
    if engine == "node":
        export_params()
    jobs, winds = exp_jobs(cases)
    rows = []
    t0 = time.time()
    pool = ThreadPoolExecutor if engine == "node" else ProcessPoolExecutor
    fn = run_exp_job_node if engine == "node" else run_exp_job
    with pool(max_workers=workers) as ex:
        for job, res in zip(jobs, ex.map(fn, jobs)):
            c, a, k, pairs = job
            en = energy_defaults(a, k)
            for (name, p0, p1, ts), r in zip(pairs, res):
                r.update(case=c, aircraft=a, capacity_factor=k, pair=name, start=list(p0), dest=list(p1),
                         mean_wind_from_deg=winds[c], capacity_wh=en["capacity_wh"], reserve_wh=en["reserve_wh"])
                rows.append(r)
            log(f"{c} {a} x{k}: " + ", ".join(f"{x['pair']} {'ok' if x['ok'] else 'fail ' + str(x['reason'])}"
                                                for x in rows[-len(pairs):]) + f"  ({time.time() - t0:.0f} s)")
    layers = {c: (SolarLayer.load(c) is not None) for c in cases}
    files = MODEL_FILES + [W.OUT_DIR / f"{c}.js" for c in cases]
    if engine == "node":
        files += [HERE / "viewer" / "route.js", HERE / "route_exp.js", PARAMS_JS]
    obj = {"meta": dict(record.meta("python -m sim.route exp", files), engine=engine),
           "settings": {"refine": REFINE, "rings": RINGS, "sub_km": SUB_KM, "wait_s": WAIT_S, "eps_soc": EPS_SOC,
                        "eps_slack": EPS_SLACK, "dom_window_s": DOM_WINDOW_S, "solar_layer": layers},
           "energy": {a: energy_defaults(a) for a in AIRCRAFT}, "rows": rows}
    Path(out).parent.mkdir(parents=True, exist_ok=True)
    with open(out, "w", encoding="utf-8", newline="\n") as fh:
        json.dump(obj, fh, indent=None, separators=(",", ":"), default=float)
    return obj


# ================================================================ fixed routes: laps (python -m sim.route loop)
#
# The input of `python -m sim.route loop INPUT --out OUT` is one JSON file (NaN or null for a missing value):
#   fields           a list of BoxField dicts (one or more months, joined in time; BoxField)
#   solar            a list of BoxSolar dicts (3-hour means, joined in time), or null for the fitted model alone
#   points           the polyline [[lat, lon], ...]; a closed loop ends at its first point
#   dry              optional, [[a, b], ...]: vertex ranges flown over land (path_graph)
#   no_wait          optional, [{"months": [1..12, ...], "vertices": [a, b]}, ...]: no wait on the water in those UTC
#                    months at the nodes from vertex a to vertex b ("vertices" left out or null: every node)
#   starts           the start times (unix seconds, UTC)
#   aircraft         optional, one of AIRCRAFT (default the first); capacity_factor optional (default 1);
#   sensor_set       optional, a key of SENSOR_SETS (default SENSOR_SET)
#   directions       optional, a subset of ["forward", "reverse"] (default both; reverse flies the reversed polyline)
#   sub_km           optional, the longest distance between nodes and of a step (default route.sub_km)
#   max_pops, relax_pops   optional, the labels the strict and each relaxed search take at most (default route.max_pops,
#                    route.relax_pops); relax optional (default true); land optional (default true: test the
#                    route's wet nodes and legs against sim/viewer/world/land.js and list the hits)
# The output holds no coordinates: the waypoints and the summaries name nodes (path_graph of the same input gives
# their places) and the vertices of the input's polyline they lie between.

LOOP_DIRECTIONS = ("forward", "reverse")


def loop_directions(inp):
    """[(name, points, dry ranges, no-wait entries)] of a loop input, the reverse direction's polyline reversed and its
    vertex ranges renumbered to it."""
    pts = [[float(p[0]), float(p[1])] for p in inp["points"]]
    nv = len(pts)
    dry = [[int(r[0]), int(r[1])] for r in inp.get("dry") or []]
    nw = [{"months": [int(m) for m in e["months"]],
           "vertices": None if e.get("vertices") is None else [int(e["vertices"][0]), int(e["vertices"][1])]}
          for e in inp.get("no_wait") or []]

    def rv(r):
        return [nv - 1 - r[1], nv - 1 - r[0]]

    out = []
    for name in inp.get("directions") or LOOP_DIRECTIONS:
        if name == "forward":
            out.append((name, pts, dry, nw))
        elif name == "reverse":
            out.append((name, pts[::-1], [rv(r) for r in dry],
                        [{"months": e["months"], "vertices": None if e["vertices"] is None else rv(e["vertices"])} for e in nw]))
        else:
            raise ValueError(f"loop: unknown direction {name!r}")
    return out


def loop_graph(field, points, dry, no_wait, sub_km=SUB_KM, land=None):
    g = path_graph(field, points, sub_km, dry, land)
    for e in no_wait:
        set_no_wait(g, e["months"], e["vertices"])
    return g


def loop_lows(graph, tab, en, r):
    """The strict margins of the route found (as plan gives a closest route's) and, for the route or the closest
    route, the waypoint with the lowest night margin (Tables.fly_on_min less the reserve)."""
    R = en["reserve_wh"]
    for k in ("route", "closest"):
        rt = r.get(k)
        if not rt:
            continue
        if k == "route":
            sm, nm, am = strict_margin(graph, tab, en, rt["waypoints"], rt["legs"])
            rt["margins"] = {"reserve_margin_wh": sm, "night_margin_wh": nm, "allowance_margin_wh": am}
        best = None
        for w in rt["waypoints"]:
            x = tab.fly_on_min(w["node"], w["t"], w["soc_wh"]) - R
            if best is None or x < best["night_margin_wh"]:
                best = {"node": w["node"], "t": w["t"], "night_margin_wh": x}
        rt["night_low"] = best


def loop_job(job):
    """One direction's plans in Python (engine "python"): job (input, direction, start times)."""
    inp, name, starts = job
    field = BoxField.join(inp["fields"])
    layer = BoxSolar.join(inp["solar"]) if inp.get("solar") else None
    _, pts, dry, nw = next(d for d in loop_directions(inp) if d[0] == name)
    g = loop_graph(field, pts, dry, nw, inp.get("sub_km", SUB_KM), None if inp.get("land", True) else False)
    aircraft = inp.get("aircraft", AIRCRAFT[0])
    lm = leg_model(aircraft)
    en = energy_defaults(aircraft, inp.get("capacity_factor", 1.0), inp.get("sensor_set", SENSOR_SET))
    tab = Tables(g, field, en, layer)
    rows = []
    for t0 in starts:
        t1 = time.time()
        r = plan(field, g, lm, en, None, None, t0=t0, layer=layer, tables=tab, relax=inp.get("relax", True),
                 max_pops=inp.get("max_pops", MAX_POPS), relax_pops=inp.get("relax_pops", RELAX_POPS),
                 start_node=0, dest_node=len(g.lat) - 1)
        r["wall_s"] = time.time() - t1
        loop_lows(g, tab, en, r)
        rows.append(r)
    return rows


def _nulls(frames):
    return [[None if (x is None or not math.isfinite(float(x))) else x for x in fr] for fr in frames]


def _box_json(ds):
    """Box dicts as JSON for node (null for NaN)."""
    return json.dumps([dict(d, vals={k: _nulls(v) for k, v in d["vals"].items()}) for d in ds], allow_nan=False,
                      separators=(",", ":"))


def loop_node_boxes(inp):
    """The input's boxes as JSON for node, once for all its jobs (loop_job_node)."""
    inp["_fields_json"] = _box_json(inp["fields"])
    inp["_solar_json"] = _box_json(inp["solar"]) if inp.get("solar") else "null"
    return inp


def _numbers_back(x):
    """node's JSON with the non-finite numbers as strings (route_loop.js) back to floats."""
    if isinstance(x, dict):
        return {k: _numbers_back(v) for k, v in x.items()}
    if isinstance(x, list):
        return [_numbers_back(v) for v in x]
    if x in ("Infinity", "-Infinity", "NaN"):
        return float(x)
    return x


def loop_job_node(job):
    """One direction's plans in node with sim/viewer/route.js (engine "node", sim/route_loop.js): job (input with
    its boxes as JSON by loop_node_boxes, direction, start times)."""
    import shutil
    import subprocess
    inp, name, starts = job
    node = shutil.which("node")
    if not node:
        raise RuntimeError("loop: node not found (use --engine python)")
    _, pts, dry, nw = next(d for d in loop_directions(inp) if d[0] == name)
    aircraft = inp.get("aircraft", AIRCRAFT[0])
    rest = {"points": pts, "dry": dry, "no_wait": nw, "sub_km": inp.get("sub_km", SUB_KM),
            "land": bool(inp.get("land", True)),
            "energy": energy_defaults(aircraft, inp.get("capacity_factor", 1.0), inp.get("sensor_set", SENSOR_SET)),
            "hulls": js_hulls(aircraft), "steady": steady_params(aircraft), "settings": js_settings(),
            "starts": list(starts), "relax": bool(inp.get("relax", True)),
            "max_pops": inp.get("max_pops", MAX_POPS), "relax_pops": inp.get("relax_pops", RELAX_POPS)}
    msg = ('{"fields":' + inp["_fields_json"] + ',"solar":' + inp["_solar_json"] + ","
           + json.dumps(rest, allow_nan=False, separators=(",", ":"))[1:])
    r = subprocess.run([node, str(HERE / "route_loop.js")], input=msg, capture_output=True, text=True)
    if r.returncode:
        raise RuntimeError(r.stderr[-3000:])
    return _numbers_back(json.loads(r.stdout))


def loop_where(g, n, nv, reverse):
    """The vertices of the input's polyline that node n of a direction's graph lies between ([i, i] at vertex i)."""
    i = g.seg[n]
    a, b = (i, i) if g.vertex_node[i] == n else (i, i + 1)
    return [nv - 1 - a, nv - 1 - b] if reverse else [a, b]


def loop_summary(g, r, en, nv, reverse):
    """A lap's plan in short: the result and reason, the days, the shares of soaring, motor and waiting by time and
    of soaring and motor by distance (and the dry distance), the lowest charge, the strict margins with the night's
    lowest, the waits (merged, longest first) and the legs with motor energy (most first), and the time, waits and
    motor per segment of the input's polyline; of the route found, else of the closest route."""
    st = r.get("stats") or {}
    out = {"ok": r["ok"], "reason": r["reason"], "reason_text": r["reason_text"], "t0": r["t0"],
           "path_km": r.get("path_km"), "nodes": len(g.lat), "strict_pops": st.get("pops"),
           "strict_capped": st.get("capped"), "reached_node": st.get("reached"),
           "reached_between": loop_where(g, st["reached"], nv, reverse) if st.get("reached") is not None else None,
           "relaxed": r.get("relaxed"), "of": None}
    rt = r.get("route") or r.get("closest")
    if not rt:
        return out
    out["of"] = "route" if r.get("route") else "closest (" + rt["relaxation"] + ")"
    wps, legs, sm = rt["waypoints"], rt["legs"], rt["summary"]
    T = sm["time_h"]
    mg = rt.get("margins") or {k: rt[k] for k in ("reserve_margin_wh", "night_margin_wh", "allowance_margin_wh")}
    km = {"soaring": 0.0, "motor": 0.0}
    dry_km = 0.0
    segs = {}
    waits, motor = [], []

    def segment(n):
        """The input's polyline segment (its two vertices) that the leg or wait from node n belongs to."""
        i = min(g.seg[n], nv - 2)
        return (nv - 2 - i, nv - 1 - i) if reverse else (i, i + 1)

    for lg in legs:
        n = lg["from"]
        sg = segs.setdefault(segment(n), {"hours": 0.0, "wait_h": 0.0, "motor_wh": 0.0, "soaring_km": 0.0, "motor_km": 0.0})
        h = (lg["t1"] - lg["t0"]) / 3600.0
        sg["hours"] += h
        if lg["mode"] == "waiting":
            sg["wait_h"] += h
            if waits and waits[-1]["node"] == n and abs(waits[-1]["t1"] - lg["t0"]) < 1e-6:
                waits[-1]["t1"] = lg["t1"]
                waits[-1]["hours"] += h
            else:
                waits.append({"node": n, "between": loop_where(g, n, nv, reverse), "t0": lg["t0"], "t1": lg["t1"], "hours": h})
            continue
        m, sub_m, _ = g.edges[n][0]
        if m != lg["to"]:
            raise ValueError("loop_summary: a leg that is not the path's")
        dry = g.dry_edge[n][0]
        for md in lg["modes"]:
            km[md] += sub_m / 1000.0
            sg[md + "_km"] += sub_m / 1000.0
        if dry:
            dry_km += sub_m * len(lg["modes"]) / 1000.0
        sg["motor_wh"] += lg["motor_wh"]
        if lg["motor_wh"] > 0.0:
            motor.append({"from": n, "to": m, "between": loop_where(g, n, nv, reverse), "t0": lg["t0"],
                          "motor_wh": lg["motor_wh"], "takeoff_wh": lg["takeoff_wh"],
                          "motor_h": h * sum(1 for x in lg["modes"] if x == "motor") / len(lg["modes"]), "dry": dry})
    low = None
    for w in wps:
        if low is None or w["soc_wh"] < low["soc_wh"]:
            low = {"soc_wh": w["soc_wh"], "node": w["node"], "t": w["t"], "within_leg": False}
    for lg in legs:
        if lg["soc_min_wh"] < low["soc_wh"]:
            low = {"soc_wh": lg["soc_min_wh"], "node": lg["from"], "t": lg["t0"], "within_leg": True}
    low["between"] = loop_where(g, low["node"], nv, reverse)
    nl = rt.get("night_low")
    dist = km["soaring"] + km["motor"]
    out.update({
        "days": T / 24.0, "time_h": T,
        "share_time": {k: (sm[k + "_h"] / T if T > 0 else 0.0) for k in ("soaring", "motor", "wait")},
        "share_distance": {k: (km[k] / dist if dist > 0 else 0.0) for k in ("soaring", "motor")},
        "km": dict(km, dry=dry_km), "waits": sm["waits"], "takeoffs": sum(1 for lg in legs if lg["takeoff_wh"] > 0.0),
        "motor_wh": sm["motor_wh"], "takeoff_wh": sm["takeoff_wh"], "solar_wh": sm["solar_wh"],
        "lowest_charge": low, "margins": mg,
        "night": {"carried": mg["night_margin_wh"] >= -1e-6, "night_margin_wh": mg["night_margin_wh"],
                  "lowest_at": dict(nl, between=loop_where(g, nl["node"], nv, reverse)) if nl else None},
        "longest_waits": sorted(waits, key=lambda w: -w["hours"]),
        "most_motor": sorted(motor, key=lambda x: -x["motor_wh"]),
        "segments": [dict(v, between=list(k)) for k, v in sorted(segs.items())]})
    return out


def _strip_coords(r):
    for k in ("route", "closest"):
        if r.get(k):
            for w in r[k]["waypoints"]:
                w.pop("lat", None)
                w.pop("lon", None)
    return r


def run_loop(path, out, workers=4, engine="node", log=_log):
    """Plan the laps of a loop input (the notes above) and write them, with each plan's summary (loop_summary), to
    `out`. Prints no coordinates."""
    from concurrent.futures import ProcessPoolExecutor, ThreadPoolExecutor
    raw = Path(path).read_bytes()
    inp = json.loads(raw)
    for k in ("fields", "points", "starts"):
        if not inp.get(k):
            raise SystemExit(f"loop: the input has no {k}")
    if not isinstance(inp["fields"], list):
        inp["fields"] = [inp["fields"]]
    if inp.get("solar") and not isinstance(inp["solar"], list):
        inp["solar"] = [inp["solar"]]
    starts = [float(t) for t in inp["starts"]]
    field = BoxField.join(inp["fields"])
    dirs = loop_directions(inp)
    land = None if inp.get("land", True) else False
    if land is None:
        land = W.load_land()
    nv = len(inp["points"])
    graphs, info = {}, {}
    for name, pts, dry, nw in dirs:
        g = loop_graph(field, pts, dry, nw, inp.get("sub_km", SUB_KM), land)
        graphs[name] = g
        times = [field.t0 + k * field.dt for k in range(field.nt)]
        nosea = [n for n in range(len(g.lat)) if not g.dry_node[n]
                 and any(not math.isfinite(field.sample(g.loc[n], t)[2]) for t in times)]
        info[name] = {"nodes": len(g.lat), "path_km": g.rem_m[0] / 1000.0, "dry_nodes": sum(g.dry_node),
                      "vertex_node": g.vertex_node, "land_hits": g.land_hits, "wet_nodes_without_sea": nosea}
        log(f"{name}: {len(g.lat)} nodes, {g.rem_m[0] / 1000.0:.1f} km, {sum(g.dry_node)} dry nodes, "
            f"{len(g.land_hits['nodes'])} wet nodes and {len(g.land_hits['legs'])} wet legs on land, "
            f"{len(nosea)} wet nodes without a sea value")
    if engine == "node":
        loop_node_boxes(inp)
    k = max(1, min(len(starts), -(-workers // len(dirs))))
    jobs = [(inp, name, starts[q::k]) for name, *_ in dirs for q in range(k) if starts[q::k]]
    pool = ThreadPoolExecutor if engine == "node" else ProcessPoolExecutor
    fn = loop_job_node if engine == "node" else loop_job
    aircraft = inp.get("aircraft", AIRCRAFT[0])
    en = energy_defaults(aircraft, inp.get("capacity_factor", 1.0), inp.get("sensor_set", SENSOR_SET))
    plans = []
    t_wall = time.time()
    with pool(max_workers=workers) as ex:
        for (_, name, ts), res in zip(jobs, ex.map(fn, jobs)):
            for t0, r in zip(ts, res):
                s = loop_summary(graphs[name], r, en, nv, name == "reverse")
                plans.append({"direction": name, "start": t0, "summary": s, "plan": _strip_coords(r)})
                day = _dt.datetime.fromtimestamp(t0, _dt.timezone.utc).strftime("%Y-%m-%d %H:%M")
                log(f"{name} {day}Z: " + (f"ok in {s['days']:.2f} d" if r["ok"] else f"fail {r['reason']}")
                    + (f" (strict search capped at {s['strict_pops']} labels)" if s["strict_capped"] else "")
                    + f"  ({r['wall_s']:.1f} s, {time.time() - t_wall:.0f} s in all)")
    plans.sort(key=lambda p: (LOOP_DIRECTIONS.index(p["direction"]), p["start"]))
    files = MODEL_FILES + ([HERE / "viewer" / "route.js", HERE / "route_loop.js"] if engine == "node" else [])
    obj = {"meta": dict(record.meta("python -m sim.route loop", files), engine=engine,
                        input_sha256=hashlib.sha256(raw).hexdigest()),
           "settings": dict(js_settings(), sub_km=inp.get("sub_km", SUB_KM), max_pops=inp.get("max_pops", MAX_POPS),
                            relax_pops=inp.get("relax_pops", RELAX_POPS), relax=bool(inp.get("relax", True)),
                            solar_box=bool(inp.get("solar"))),
           "aircraft": aircraft, "capacity_factor": inp.get("capacity_factor", 1.0), "energy": en,
           "graphs": info, "plans": plans}
    Path(out).parent.mkdir(parents=True, exist_ok=True)
    with open(out, "w", encoding="utf-8", newline="\n") as fh:
        json.dump(obj, fh, indent=None, separators=(",", ":"), default=float)
    log(f"wrote {len(plans)} plans")
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
    p.add_argument("--engine", choices=("node", "python"), default="node")
    p = sub.add_parser("loop", help="plan the laps of a fixed route from a private input file (notes above run_loop)")
    p.add_argument("input")
    p.add_argument("--out", required=True)
    p.add_argument("--workers", type=int, default=4)
    p.add_argument("--engine", choices=("node", "python"), default="node")
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
        run_exp(a.workers, a.cases, engine=a.engine)
    elif a.cmd == "loop":
        run_loop(a.input, a.out, a.workers, a.engine)
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
