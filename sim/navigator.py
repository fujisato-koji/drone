"""Tactical layer: course relative to the wind, tack and gybe decisions, like a sailing yacht.

Conventions
-----------
* Positions are metres, x east and y north. Bearings are degrees clockwise from north.
* ``wind_from`` is the direction the wind blows FROM (meteorological). The downwind bearing is
  wind_from + 180.
* A course relative to the wind is chi in [0, 180]: 0 downwind, 180 upwind. A signed course
  c = bearing - downwind in (-180, 180]; c > 0 is "starboard" (the wind comes from the right of
  the heading), c < 0 is "port".
* The speed polar V(chi, u10) is the progress speed of the cycle-averaged flight along the
  course (a table). It is mirrored to both sides of the wind.

Polar file format (loadable from sim/out/ds_polar.json when it exists)::

    {"winds": [u10 ...], "chi_deg": [0 ... 180], "V": [[per chi ...] per wind]}   null = no cycle

Choices the spec left open: the VMG of a course is V(chi) cos(course - bearing to target); the
best course on a side is searched on a 0.5 deg grid plus the direct course; outside the table
(wind speed, or any infeasible corner) the polar is NaN; a tack or gybe costs a pause (no
motion) in the simulator; the tack cost is amortised over min(remaining time, amort_s), the assumed life of a tack's benefit; a
direct course is entered when its VMG is within the hysteresis of the best tacking VMG (this is
how the layline is detected: the target is then fetchable on one tack).
"""
from __future__ import annotations

import json
import math
from dataclasses import dataclass

import numpy as np


def wrap180(a):
    """Wrap an angle (deg) to (-180, 180]."""
    r = (np.asarray(a, dtype=float) + 180.0) % 360.0 - 180.0
    r = np.where(r == -180.0, 180.0, r)
    return float(r) if r.ndim == 0 else r


def bearing_to(src, dst):
    d = np.asarray(dst, float) - np.asarray(src, float)
    return math.degrees(math.atan2(d[0], d[1])) % 360.0


class Polar:
    """Progress speed V(chi, u10), linear in both, NaN outside the feasible region."""

    def __init__(self, winds, chis, table):
        self.winds = np.asarray(winds, float)
        self.chis = np.asarray(chis, float)
        self.table = np.asarray(table, float)
        if self.table.shape != (self.winds.size, self.chis.size):
            raise ValueError("table must be (n_winds, n_courses)")
        if np.any(np.diff(self.winds) <= 0) or np.any(np.diff(self.chis) <= 0):
            raise ValueError("winds and courses must increase")

    @classmethod
    def from_json(cls, path):
        with open(path, encoding="utf-8") as f:
            d = json.load(f)
        v = np.array([[np.nan if x is None else x for x in row] for row in d["V"]], float)
        return cls(d["winds"], d["chi_deg"], v)

    @staticmethod
    def _interp_axis(grid, x):
        """Lower node index and weight of the upper node; NaN weight outside the grid."""
        x = np.asarray(x, float)
        i = np.clip(np.searchsorted(grid, x, side="right") - 1, 0, grid.size - 2)
        w = (x - grid[i]) / (grid[i + 1] - grid[i])
        w = np.where((x < grid[0]) | (x > grid[-1]), np.nan, w)
        return i, w

    def speed(self, chi, u10):
        """V at course(s) chi (deg, mirrored) and wind speed u10; NaN if infeasible."""
        chi = np.abs(np.asarray(wrap180(chi), float))
        scalar = chi.ndim == 0
        chi = np.atleast_1d(chi)
        iw, ww = self._interp_axis(self.winds, np.full(chi.shape, float(u10)))
        ic, wc = self._interp_axis(self.chis, chi)

        def lerp(a, b, w):   # weight exactly 0 or 1 ignores the other node (NaN-safe)
            return np.where(w == 0, a, np.where(w == 1, b, a * (1 - w) + b * w))

        t = self.table
        v0 = lerp(t[iw, ic], t[iw, ic + 1], wc)
        v1 = lerp(t[iw + 1, ic], t[iw + 1, ic + 1], wc)
        v = lerp(v0, v1, ww)
        v = np.where(np.isnan(ww) | np.isnan(wc), np.nan, v)
        return float(v[0]) if scalar else v

    # ---- VMG helpers -------------------------------------------------------------------
    def vmg_course(self, tau, u10, side=None, step_deg=0.5):
        """Best VMG towards a target at signed angle tau (deg, relative to downwind).

        side: +1 starboard, -1 port, None for either. Returns (vmg, signed course, chi); the
        direct course c = tau is a candidate. (0.0, nan, nan) if no course makes progress.
        """
        chi = np.arange(0.0, 180.0 + 1e-9, step_deg)
        best = (0.0, math.nan, math.nan)
        for s in ((+1, -1) if side is None else (side,)):
            c = s * chi
            if np.sign(tau) == s or tau in (0.0, 180.0):
                c = np.append(c, tau)
            v = self.speed(np.abs(c), u10)
            vmg = v * np.cos(np.radians(c - tau))
            vmg = np.where(np.isnan(vmg), -np.inf, vmg)
            k = int(np.argmax(vmg))
            if vmg[k] > best[0] + 1e-12:
                best = (float(vmg[k]), float(c[k]), abs(float(c[k])))
        return best

    def direct_vmg(self, tau, u10):
        v = self.speed(abs(tau), u10)
        return float(v) if np.isfinite(v) else 0.0

    def upwind_angle(self, u10):
        """Best upwind VMG course, degrees off the wind (0 = into the wind), and its VMG."""
        vmg, _, chi = self.vmg_course(180.0, u10)
        return (180.0 - chi, vmg) if np.isfinite(chi) else (math.nan, 0.0)

    def downwind_angle(self, u10):
        """Best downwind VMG course, degrees off the downwind axis, and its VMG."""
        vmg, _, chi = self.vmg_course(0.0, u10)
        return (chi, vmg) if np.isfinite(chi) else (math.nan, 0.0)


@dataclass
class Command:
    bearing: float          # absolute commanded course, deg
    chi: float              # course relative to the wind, 0 downwind .. 180 upwind
    side: str               # "port" | "starboard"
    mode: str               # direct | upwind tacking | downwind gybing | arrived | no progress
    vmg: float = 0.0        # expected VMG towards the target on this course
    tacked: bool = False    # this call switched side (a tack or a gybe)
    distance: float = 0.0


class Navigator:
    """Yacht-like state machine. Call step() every few seconds.

    hysteresis: fraction of the best VMG by which the other tack must beat the current one.
    tack_cost_s: seconds of progress lost per tack/gybe, amortised over min(remaining time,
    amort_s). min_leg_s: no voluntary tack sooner than this after the last one.
    shift_tacking: False = tack only at the layline (baseline for the wind-shift tests).
    """

    def __init__(self, polar, tack_cost_s=30.0, hysteresis=0.05, min_leg_s=90.0,
                 arrival_radius=50.0, amort_s=60.0, shift_tacking=True):
        self.polar = polar
        self.tack_cost_s = float(tack_cost_s)
        self.hysteresis = float(hysteresis)
        self.min_leg_s = float(min_leg_s)
        self.arrival_radius = float(arrival_radius)
        self.amort_s = float(amort_s)
        self.shift_tacking = bool(shift_tacking)
        self.reset()

    def reset(self):
        self.side = None            # +1 starboard, -1 port
        self.mode = None
        self.last_tack_t = -math.inf

    @staticmethod
    def _name(s):
        return "starboard" if s > 0 else "port"

    def step(self, t, pos, u10, wind_from, target, course=None):
        """Return a Command. course (current absolute bearing) only seeds the side at start."""
        pos = np.asarray(pos, float)
        dist = float(np.hypot(*(np.asarray(target, float) - pos)))
        beta = bearing_to(pos, target)
        down = wind_from + 180.0
        tau = wrap180(beta - down)
        tside = 1 if tau >= 0 else -1
        if dist <= self.arrival_radius:
            self.mode = "arrived"
            return Command(beta, abs(tau), self._name(tside), "arrived", 0.0, False, dist)
        P = self.polar
        vm = {s: P.vmg_course(tau, u10, side=s) for s in (+1, -1)}
        best = max(vm[+1][0], vm[-1][0])
        dvmg = P.direct_vmg(tau, u10)
        if best <= 1e-9:
            self.mode = "no progress"
            return Command(beta, abs(tau), self._name(tside), "no progress", 0.0, False, dist)
        if self.side is None and course is not None:
            self.side = 1 if wrap180(course - down) >= 0 else -1
        prev_side, prev_mode = self.side, self.mode

        # direct course, with a band so it does not flip against tacking
        if prev_mode == "direct":
            go_direct = dvmg > 0 and best <= dvmg * (1 + self.hysteresis)
        else:
            go_direct = dvmg > 0 and dvmg >= best * (1 - self.hysteresis)
        if go_direct:
            side = tside if tau != 0 else (prev_side or 1)
            c, vmg, mode = tau, dvmg, "direct"
        else:
            if prev_side is None or prev_mode == "direct":   # start / leaving direct: better side
                cur = 1 if vm[+1][0] >= vm[-1][0] else -1
            else:
                cur = prev_side
            oth = -cur
            t_h = max(min(dist / best, self.amort_s), self.tack_cost_s, 1e-9)
            thresh = self.hysteresis * best + best * self.tack_cost_s / t_h
            gain = vm[oth][0] - vm[cur][0]
            forced = vm[cur][0] <= 1e-9 and vm[oth][0] > 1e-9
            voluntary = (self.shift_tacking and gain > thresh
                         and t - self.last_tack_t >= self.min_leg_s)
            side = oth if (forced or voluntary) else cur
            vmg, c, _ = vm[side]
            mode = "upwind tacking" if abs(tau) > 90.0 else "downwind gybing"
        tacked = (prev_side is not None and side != prev_side
                  and not (prev_mode == "direct" and mode == "direct"))
        if tacked:
            self.last_tack_t = t
        self.side, self.mode = side, mode
        return Command((down + c) % 360.0, abs(c), self._name(side), mode, float(vmg), tacked,
                       dist)


def sail(polar, wind_fn, start, target, dt=1.0, nav=None, nav_every_s=5.0, t_max=48 * 3600.0,
         physical_tack_cost_s=None, **nav_kw):
    """Kinematic run: a point moves at the polar speed along the commanded course.

    wind_fn(t) -> (u10, wind_from_deg). The commanded bearing is held between navigator calls;
    if the wind turns it into an infeasible course the speed is 0 until the next call. A tack or
    gybe pauses the motion for the tack cost. Returns a dict: track (N,2), t, modes, n_tacks,
    tack_times, time (to arrive, nan if not), arrived, mean_vmg (net progress towards the target
    over elapsed time), distance.
    """
    nav = nav or Navigator(polar, **nav_kw)
    nav.reset()
    pc = nav.tack_cost_s if physical_tack_cost_s is None else float(physical_tack_cost_s)
    pos = np.asarray(start, float).copy()
    target = np.asarray(target, float)
    d0 = float(np.hypot(*(target - pos)))
    track, times, modes, tack_times = [pos.copy()], [0.0], [], []
    t, pause_until, next_call, cmd = 0.0, -1.0, 0.0, None
    arrived = False
    while t < t_max:
        u10, wf = wind_fn(t)
        if t >= next_call:
            cmd = nav.step(t, pos, u10, wf, target, None if cmd is None else cmd.bearing)
            next_call = t + nav_every_s
            modes.append((t, cmd.mode))
            if cmd.mode == "arrived":
                arrived = True
                break
            if cmd.tacked:
                tack_times.append(t)
                pause_until = t + pc
        v = 0.0
        if t >= pause_until and cmd.mode != "no progress":
            v = polar.speed(wrap180(cmd.bearing - (wf + 180.0)), u10)
            v = 0.0 if not np.isfinite(v) else v
        b = math.radians(cmd.bearing)
        pos = pos + v * dt * np.array([math.sin(b), math.cos(b)])
        t += dt
        track.append(pos.copy())
        times.append(t)
    d1 = float(np.hypot(*(target - pos)))
    return dict(track=np.array(track), t=np.array(times), modes=modes, n_tacks=len(tack_times),
                tack_times=tack_times, time=t if arrived else math.nan, arrived=arrived,
                mean_vmg=(d0 - d1) / t if t > 0 else 0.0, distance=d0)
