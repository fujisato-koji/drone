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


DS_POLAR_PATH = None      # set lazily: sim/out/ds_polar.json next to this file
MIN_PROGRESS = 0.1        # m/s; progress speeds at or below this are "no cycle" in a table built from files


def polar_from_ds(name, path=None):
    """Polar of one aircraft from sim/out/ds_polar.json (python -m sim.polar), or None if the file has
    no converged case for it. Winds whose row has no progress above MIN_PROGRESS are dropped from the
    table (the polar is NaN below the lowest kept wind), and a course that makes no progress along
    itself is NaN (the table holds the progress speed along each course)."""
    import os
    path = path or os.path.join(os.path.dirname(__file__), "out", "ds_polar.json")
    with open(path, encoding="utf-8") as f:
        rows = [r for r in json.load(f)["rows"] if r["aircraft"] == name]
    chis = sorted({c["chi_deg"] for r in rows for c in r["cases"]})
    winds, table = [], []
    for r in sorted(rows, key=lambda r: r["u10"]):
        v = {c["chi_deg"]: c["progress"] for c in r["cases"] if c.get("converged")}
        line = [v[c] if v.get(c, 0.0) > MIN_PROGRESS else np.nan for c in chis]
        if np.any(np.isfinite(line)):
            winds.append(r["u10"])
            table.append(line)
    if len(winds) < 2:
        return None
    return Polar(winds, chis, table)


# =============================================================================================
# Navigation on the velocity hull
# =============================================================================================
"""
The progress speed of a cycle along its nominal course (the polar above) is a projection: the cycle
optimizer maximises the speed along the course and leaves the cross-track velocity free, so the
cycle's mean ground track lies far off its course and the speed made good towards a target is NOT
progress * cos(angle). What a cycle really has is a mean ground velocity vector v (net displacement
over one period / period). Those vectors, with their mirror images for the other side of the wind,
are the points of the VELOCITY HULL; any mean velocity inside their convex hull can be had by flying
the cycles in turn (time shares), which is tacking.

Frame of the hull: wind toward +x, y to the left of the downwind direction (the library frame; angles
ccw from +x). A world vector (east, north) turns into it through psi_w, the math angle of the
downwind direction (psi_w = -90 deg - wind_from).

Choices the spec left open (simplest thing taken):
* Wind: the hull of the library wind nearest to the wind estimate (no scaling between winds).
* For a target line d (unit vector in the wind frame) every point has a speed along the line a = v.d
  and a cross velocity c = v x d (positive to the left of the line). A pair with c_i <= 0 <= c_j
  flown with the time shares w_i = c_j / (c_j - c_i), w_j = -c_i / (c_j - c_i) has zero mean cross
  velocity and the speed made good r = w_i a_i + w_j a_j (the hull's boundary along the line).
* Soaring points first: the pair of soaring points with the largest r, if r >= min_vmg. Otherwise all
  points: among the pairs with r >= min_vmg the one with the least electric energy per metre made good
  (w_i P_i + w_j P_j) / r. Nothing reaches min_vmg: "no progress".
* A pair is kept while its r is within the hysteresis of the best (soaring) or its energy per metre
  within the hysteresis of the least (motor).
* Legs: the line runs from where it was set (first call, a new target, or the aircraft more than twice
  the (funnel) corridor off) to the target. A leg flies the pair's point that pushes the aircraft towards the
  corridor edge; at the edge the other point takes over, never sooner than min_leg_s after the last
  switch. Corridor half-width E = leg_s * max |c| / 2 with leg_s = max(min_leg_s, tack_cost_s /
  max_tack_loss) (a leg is about that long at the faster cross speed, so the tack pause costs at most
  max_tack_loss = 10 % of the time), at most max_corridor_frac (0.2) of the line length.
* A soaring corridor is clipped on each side where the target would no longer be reachable (the soaring
  speed made good towards it from there below LIMIT_FRAC = 0.5 of the plan's, scanned in 10 steps): the
  fixed-velocity cycles drift downwind, so near the edge of the hull the aircraft must stay on the
  upwind side of the line.
* Funnel: the corridor narrows as the target nears, E = min(E_full, funnel_frac * distance) with
  funnel_frac 0.3, and while it is narrower than E_full the minimum leg does not hold (a cycle flown
  at fixed velocity cannot otherwise be brought to within the arrival radius).
* Final approach: nearer than approach_factor (5) arrival radii to the target the navigator steers straight at it:
  every call the point that closes at min_vmg or more and passes within 0.7 arrival radius of the target
  (soaring first, fastest; else the least energy per metre), or failing that the smallest closest
  approach; the line re-set at the aircraft. Mode "direct".
* Mode names as before: "upwind tacking" (target in the upwind half) or "downwind gybing" for a pair,
  "direct", "no progress", "arrived".
"""

LIMIT_FRAC = 0.5           # corridor limit: the target must stay reachable at this share of the plan's speed
CROSS_ON_LINE = 0.2       # m/s of cross velocity below which a pair is one cycle on the line (no legs)
MIN_VMG = 0.3             # m/s of speed made good below which a combination does not count as progress


@dataclass
class HullPoint:
    vx: float
    vy: float
    power: float = 0.0          # average electric power, W (0 for soaring)
    kind: str = "soaring"       # soaring | motor
    key: str = ""               # identity ("kind:u10:chi:side")
    entry: object = None        # the library entry (Python side), not used by the navigator
    side: int = 1               # +1 the entry as stored, -1 its mirror image
    steady: object = None       # a steady motor leg: dict(V, cl, thrust, heading (wind frame), height)

    @property
    def speed(self):
        return math.hypot(self.vx, self.vy)


class VelocityHull:
    """The achievable mean ground velocities at one wind (see the module notes above)."""

    def __init__(self, points, u10=None):
        self.points = list(points)
        self.u10 = u10

    @classmethod
    def from_entries(cls, entries, u10):
        """Points of the library entries (successful, with x, y, period) at wind u10, each with its
        mirror image."""
        pts = []
        for e in entries:
            if not e.get("success") or e["u10"] != u10:
                continue
            T = e["period"]
            vx = (e["x"][-1] - e["x"][0]) / T
            vy = (e["y"][-1] - e["y"][0]) / T
            kind = e.get("kind", "soaring")
            pw = float(e.get("power_avg_w", 0.0)) if kind == "motor" else 0.0
            for side in (1, -1):
                pts.append(HullPoint(vx, side * vy, pw, kind,
                                     "%s:%g:%g:%d" % (kind, u10, e["chi_deg"], side), e, side))
        return cls(pts, u10)

    def plan_pair(self, d, min_vmg=MIN_VMG, motor_ok=True):
        """Best way to make good along the unit line d (wind frame): dict(i, j, wi, wj, r, per_m, ci,
        cj, kind) or None. kind "soaring" or "motor" (see the module notes)."""
        pts = self.points
        a = [p.vx * d[0] + p.vy * d[1] for p in pts]
        c = [d[0] * p.vy - d[1] * p.vx for p in pts]

        def best(idx, by_energy):
            out = None
            for i in idx:
                if c[i] > 0.0:
                    continue
                for j in idx:
                    if c[j] < 0.0:
                        continue
                    den = c[j] - c[i]
                    wi, wj = (1.0, 0.0) if den <= 1e-12 else (c[j] / den, -c[i] / den)
                    r = wi * a[i] + wj * a[j]
                    if r < min_vmg:
                        continue
                    per_m = (wi * pts[i].power + wj * pts[j].power) / r
                    key = (per_m, -r) if by_energy else (-r, per_m)
                    if out is None or key < out[0]:
                        out = (key, dict(i=i, j=j, wi=wi, wj=wj, r=r, per_m=per_m, ci=c[i], cj=c[j]))
            return out[1] if out else None

        soar = [k for k, p in enumerate(pts) if p.kind == "soaring"]
        res = best(soar, False)
        if res is not None:
            res["kind"] = "soaring"
            return res
        if not motor_ok:
            return None
        res = best(list(range(len(pts))), True)
        if res is not None:
            res["kind"] = "motor"
        return res

    def pair_value(self, d, i, j, min_vmg=MIN_VMG):
        """(r, per_m, wi, wj) of the pair (i, j) on the line d, or None when it does not straddle the
        line or does not reach min_vmg."""
        pts = self.points
        ai, aj = (pts[k].vx * d[0] + pts[k].vy * d[1] for k in (i, j))
        ci, cj = (d[0] * pts[k].vy - d[1] * pts[k].vx for k in (i, j))
        if ci > 0.0 or cj < 0.0:
            return None
        den = cj - ci
        wi, wj = (1.0, 0.0) if den <= 1e-12 else (cj / den, -ci / den)
        r = wi * ai + wj * aj
        if r < min_vmg:
            return None
        return r, (wi * pts[i].power + wj * pts[j].power) / r, wi, wj

    def best_single(self, d, min_vmg=MIN_VMG, dist=None, radius=None, prefer=None):
        """(index, closing speed) of the point to fly straight at a target in direction d, or None.

        Points closing at min_vmg or more only. With dist and radius (the final approach) a point
        counts as on target when its closest approach dist * |c| / |v| is within 0.7 radius: among
        the soaring ones on target the fastest closing, else among all on target the least energy per
        metre; if none is on target, the one with the smallest closest approach (soaring first). Without
        them: soaring with the largest closing speed, else the least energy per metre. prefer: the index
        of the point flown now; it is kept when it is in the same set, of the same kind, and within 2 % of
        the choice (a mirror image of the same cycle must not take over for a tie: a change of cycle
        costs a transient)."""
        pts = self.points
        a = [p.vx * d[0] + p.vy * d[1] for p in pts]
        ok = [k for k in range(len(pts)) if a[k] >= min_vmg]
        if not ok:
            return None
        pool, miss = ok, None
        if dist is not None and radius is not None:
            miss = {k: dist * abs(d[0] * pts[k].vy - d[1] * pts[k].vx) / max(pts[k].speed, 1e-9)
                    for k in ok}
            on = [k for k in ok if miss[k] <= 0.7 * radius]
            if not on:
                k = min(ok, key=lambda k: (miss[k], pts[k].kind != "soaring"))
                return k, a[k]
            pool = on
        soar = [k for k in pool if pts[k].kind == "soaring"]
        if soar:
            k = max(soar, key=lambda k: a[k])
            if prefer is not None and prefer in soar and a[prefer] >= 0.98 * a[k]:
                k = prefer
            return k, a[k]
        k = min(pool, key=lambda k: (pts[k].power / a[k], -a[k]))
        if prefer is not None and prefer in pool and pts[prefer].power / a[prefer] <= 1.02 * pts[k].power / a[k]:
            k = prefer
        return k, a[k]

    def ray_speed(self, d, min_vmg=MIN_VMG):
        """Speed made good along d by soaring points alone (the hull boundary along d), or None."""
        pl = self.plan_pair(d, min_vmg, motor_ok=False)
        return None if pl is None else pl["r"]

    def support(self, d):
        """Largest component of any soaring point's velocity along d (the support function)."""
        return max((p.vx * d[0] + p.vy * d[1] for p in self.points if p.kind == "soaring"),
                   default=math.nan)


class HullSource:
    """Hulls of one aircraft at each library wind; at() gives the nearest wind that has points."""

    def __init__(self, hulls):
        self.hulls = sorted(hulls, key=lambda h: h.u10)

    @classmethod
    def from_entries(cls, entries, aircraft):
        es = [e for e in entries if e["aircraft"] == aircraft and e.get("success")]
        hs = [VelocityHull.from_entries(es, u) for u in sorted({e["u10"] for e in es})]
        return cls([h for h in hs if h.points])

    def at(self, u10):
        return min(self.hulls, key=lambda h: abs(h.u10 - u10))


@dataclass
class HullCommand:
    bearing: float          # compass bearing of the commanded cycle's mean ground velocity
    chi: float              # angle of that velocity from downwind, 0..180
    side: str
    mode: str               # direct | upwind tacking | downwind gybing | arrived | no progress
    vmg: float = 0.0        # speed made good towards the target expected from the plan
    tacked: bool = False    # this call switched leg
    distance: float = 0.0
    point: object = None    # the HullPoint to fly
    kind: str = ""          # soaring | motor
    per_m: float = 0.0      # expected electric energy per metre made good, J/m
    cross: float = 0.0      # cross-track error, m (positive to the left of the line, wind frame)
    corridor: float = 0.0   # half-width of the corridor, m


STEADY_NOTES = """
Steady motor leg (choices the spec left open; the flat sea only). A powered leg does not need a periodic
cycle: steady straight level flight at constant height is feasible at any heading. The leg is flown at
STEADY_HEIGHT_M = 10 m, the reference height of the wind profile, so the wind there is u10 (a log
profile would give u(z) = u10 ln(z / z0) / ln(10 / z0)); the library's 2 m upwind motor cycle is a
knife edge for the planner (the wind gradient makes level flight unstable and any slip below it is a
contact), and at 10 m the gradient is a fifth of that. In the wind frame the wind is (u10, 0). For a
track direction tau and an airspeed V the heading is the wind triangle: the cross component of the
ground velocity is zero, sin(beta) = -(w . n) / V with n = tau turned 90 deg left, and the speed along
the track is V cos(beta) + w . tau. Level flight needs cl = 2 m g / (rho S V^2), thrust = drag, and an
electric power thrust V / eta. The airspeed is the one of least electric energy per metre made good
between the speed floor (v_factor x stall speed, v_factor 1.2 like the margins) and a cap
(STEADY_CAP x the best glide speed), with the thrust at most STEADY_THRUST_FRAC of the maximum and a
speed made good of at least min_vmg. Cross track error is corrected by turning the track towards the
line by atan(e / lookahead) up to max_turn (a proportional guidance with a limit).
"""

STEADY_HEIGHT_M = 10.0
STEADY_CAP = 3.0           # airspeed cap, in best glide speeds
STEADY_THRUST_FRAC = 0.8   # share of the maximum thrust a steady leg may ask for
STEADY_LOOKAHEAD_M = 400.0
STEADY_MAX_TURN_DEG = 35.0
STEADY_V_STEP = 0.25


@dataclass
class SteadyModel:
    """Steady level flight of one aircraft with the motor (see STEADY_NOTES). Plain numbers, so that
    the same model runs in navigator.js."""
    mass: float
    wing_area: float
    cd0: float
    k_induced: float
    rho: float
    cl_max: float
    v_stall: float
    v_best_ld: float
    t_max: float
    eta: float
    g: float = 9.81
    v_factor: float = 1.2
    height: float = STEADY_HEIGHT_M
    cap: float = STEADY_CAP
    thrust_frac: float = STEADY_THRUST_FRAC
    lookahead: float = STEADY_LOOKAHEAD_M
    max_turn_deg: float = STEADY_MAX_TURN_DEG
    v_step: float = STEADY_V_STEP

    @classmethod
    def from_aircraft(cls, ac):
        from sim.dsopt import prop_efficiency, thrust_max
        from sim.aircraft import G
        return cls(float(ac.mass), float(ac.wing_area), float(ac.cd0), float(ac.k_induced), float(ac.rho),
                   float(ac.cl_max), float(ac.v_stall), float(ac.v_best_ld), float(thrust_max(ac)),
                   float(prop_efficiency()), g=float(G))

    def level(self, V):
        """(cl, thrust N, electric power W) of level flight at airspeed V."""
        q = 0.5 * self.rho * V * V * self.wing_area
        cl = self.mass * self.g / q
        T = q * (self.cd0 + self.k_induced * cl * cl)
        return cl, T, T * V / self.eta

    def leg(self, uz, tau, min_vmg=MIN_VMG):
        """The steady leg that makes good along the unit track tau (wind frame, wind (uz, 0)) with the
        least electric energy per metre: dict(V, cl, thrust, power, heading (unit vector), ground (vx, vy),
        along, per_m) or None when no airspeed gets min_vmg along tau."""
        n = (-tau[1], tau[0])
        wn = uz * n[0]
        wt = uz * tau[0]
        v0 = self.v_factor * self.v_stall
        v1 = self.cap * self.v_best_ld
        best = None
        k = 0
        while True:
            V = v0 + k * self.v_step
            k += 1
            if V > v1 + 1e-9:
                break
            if abs(wn) >= V:
                continue
            cl, T, P = self.level(V)
            if T > self.thrust_frac * self.t_max:
                continue
            sb = -wn / V
            cb = math.sqrt(1.0 - sb * sb)
            along = V * cb + wt
            if along < min_vmg:
                continue
            per_m = P / along
            if best is None or per_m < best["per_m"] - 1e-12:
                h = (tau[0] * cb + n[0] * sb, tau[1] * cb + n[1] * sb)
                best = dict(V=V, cl=cl, thrust=T, power=P, heading=h, along=along, per_m=per_m,
                            ground=(along * tau[0], along * tau[1]))
        return best

    def track(self, dl, e):
        """Track direction (wind frame) that turns the line direction dl towards the line by
        atan(e / lookahead), limited to max_turn; e > 0 is left of the line."""
        phi = -max(-math.radians(self.max_turn_deg),
                   min(math.radians(self.max_turn_deg), math.atan(e / self.lookahead)))
        c, s_ = math.cos(phi), math.sin(phi)
        return (dl[0] * c - dl[1] * s_, dl[0] * s_ + dl[1] * c)


class HullNavigator:
    """Tactical layer on the velocity hull (module notes above). Call step() every few seconds."""

    def __init__(self, source, tack_cost_s=30.0, hysteresis=0.05, min_leg_s=90.0, arrival_radius=50.0,
                 approach_factor=5.0, min_vmg=MIN_VMG, max_tack_loss=0.1, max_corridor_frac=0.2,
                 funnel_frac=0.3, steady=None):
        self.source = source
        self.steady = steady        # a SteadyModel: motor legs are steady flights aimed by the wind triangle
        self.funnel_frac = float(funnel_frac)
        self.max_tack_loss = float(max_tack_loss)
        self.max_corridor_frac = float(max_corridor_frac)
        self.tack_cost_s = float(tack_cost_s)
        self.hysteresis = float(hysteresis)
        self.min_leg_s = float(min_leg_s)
        self.arrival_radius = float(arrival_radius)
        self.approach_factor = float(approach_factor)
        self.min_vmg = float(min_vmg)
        self.reset()

    def reset(self):
        self.mode = None
        self.line0 = None           # world position where the line starts
        self.target = None
        self.pair = None            # (key_i, key_j)
        self.leg = None             # key of the point being flown
        self.last_switch_t = -math.inf
        self.corridor = 0.0
        self.corridor_full = 0.0

    def _steady_command(self, psi, uz, tau, vmg_dir, mode, dist, e, E, tacked_from):
        """Command for a steady motor leg along the unit track tau (wind frame), or None when the model
        finds no airspeed that makes good."""
        lg = self.steady.leg(uz, tau, self.min_vmg)
        if lg is None:
            return None
        pt = HullPoint(lg["ground"][0], lg["ground"][1], lg["power"], "motor", "steady", None, 1,
                       dict(V=lg["V"], cl=lg["cl"], thrust=lg["thrust"], heading=lg["heading"],
                            height=self.steady.height))
        return pt, lg

    def _steady_finish(self, psi, r, dist, e, E, t):
        pt, lg = r
        tacked = self.leg is not None and self.leg != "steady"
        if tacked:
            self.last_switch_t = t
        self.leg, self.mode, self.pair = "steady", "direct", None
        return self._command(psi, pt, "direct", lg["along"], tacked, dist, lg["per_m"], e, E)

    def _approach(self, t, psi, hull, pos, target, dist, e, E):
        """Final approach: steer straight at the target (see the module notes)."""
        pts = hull.points
        byk_prefer = next((k for k, p in enumerate(pts) if p.key == self.leg), None)
        dn = self.to_wind(psi, (target[0] - pos[0], target[1] - pos[1]))
        dn = (dn[0] / dist, dn[1] / dist)
        sg = hull.best_single(dn, self.min_vmg, dist, self.arrival_radius, byk_prefer)
        self.line0, self.pair = pos, None
        off = sg is not None and dist * abs(dn[0] * pts[sg[0]].vy - dn[1] * pts[sg[0]].vx)             / max(pts[sg[0]].speed, 1e-9) > 0.7 * self.arrival_radius      # best point still misses
        if self.steady is not None and (sg is None or pts[sg[0]].kind == "motor" or off
                                        or self.leg == "steady"):      # a motor leg stays one to the end
            r = self._steady_command(psi, self._uz, dn, 0.0, "direct", dist, e, E, None)
            if r is not None:
                return self._steady_finish(psi, r, dist, e, E, t)
        if sg is None:
            self.mode = "no progress"
            self.leg = None
            pt = max(pts, key=lambda p: p.vx * dn[0] + p.vy * dn[1])
            return self._command(psi, pt, "no progress", 0.0, False, dist, 0.0, e, E)
        pt = pts[sg[0]]
        tacked = bool(self.leg is not None and pt.key != self.leg)
        if tacked:
            self.last_switch_t = t
        self.leg, self.mode = pt.key, "direct"
        return self._command(psi, pt, "direct", sg[1], tacked, dist, pt.power / sg[1], e, E)

    def _limits(self, hull, dl, s_rem, E, plan):
        """Lateral limits (lo <= 0 <= hi, at most E) of the corridor: a soaring plan must stay where the
        target is still reachable, i.e. where the soaring speed made good towards the target seen from
        the aircraft is at least LIMIT_FRAC of the plan's. Scanned in 10 steps outward from the line."""
        if plan["kind"] != "soaring" or E <= 0.0:
            return -E, E
        nl = (-dl[1], dl[0])

        def ok(ep):
            vx = s_rem * dl[0] - ep * nl[0]
            vy = s_rem * dl[1] - ep * nl[1]
            n = math.hypot(vx, vy)
            pl = hull.plan_pair((vx / n, vy / n), self.min_vmg, motor_ok=False)
            return pl is not None and pl["r"] >= LIMIT_FRAC * plan["r"]

        out = []
        for sg in (-1.0, 1.0):
            lim = 0.0
            for k in range(1, 11):
                ep = E * k / 10.0
                if not ok(sg * ep):
                    break
                lim = ep
            out.append(sg * lim)
        return out[0], out[1]

    @staticmethod
    def _psi(wind_from):
        return math.radians(-90.0 - wind_from)

    @staticmethod
    def to_wind(psi, v):
        c, s = math.cos(psi), math.sin(psi)
        return (c * v[0] + s * v[1], -s * v[0] + c * v[1])

    @staticmethod
    def to_world(psi, v):
        c, s = math.cos(psi), math.sin(psi)
        return (c * v[0] - s * v[1], s * v[0] + c * v[1])

    def _command(self, psi, pt, mode, vmg, tacked, dist, per_m, cross, corridor):
        ang = psi + math.atan2(pt.vy, pt.vx)
        bearing = (90.0 - math.degrees(ang)) % 360.0
        chi = abs(math.degrees(math.atan2(pt.vy, pt.vx)))
        side = "port" if pt.vy > 0 else "starboard"
        return HullCommand(bearing, chi, side, mode, vmg, tacked, dist, pt, pt.kind, per_m, cross,
                           corridor)

    def step(self, t, pos, u10, wind_from, target, course=None):
        pos = (float(pos[0]), float(pos[1]))
        target = (float(target[0]), float(target[1]))
        dist = math.hypot(target[0] - pos[0], target[1] - pos[1])
        psi = self._psi(wind_from)
        self._uz = float(u10)       # the wind at the steady height (10 m, the reference height)
        hull = self.source.at(u10)
        pts = hull.points
        byk = {p.key: k for k, p in enumerate(pts)}
        if dist <= self.arrival_radius:
            self.mode = "arrived"
            pt = pts[byk[self.leg]] if self.leg in byk else pts[0]
            return self._command(psi, pt, "arrived", 0.0, False, dist, 0.0, 0.0, self.corridor)
        if (self.target is None or self.line0 is None
                or math.hypot(target[0] - self.target[0], target[1] - self.target[1]) > 1.0):
            self.line0, self.target = pos, target
            self.pair = self.leg = None
        dl = self.to_wind(psi, (target[0] - self.line0[0], target[1] - self.line0[1]))
        nl = math.hypot(*dl)
        dl = (dl[0] / nl, dl[1] / nl)
        pw = self.to_wind(psi, (pos[0] - self.line0[0], pos[1] - self.line0[1]))
        e = dl[0] * pw[1] - dl[1] * pw[0]
        if self.corridor_full > 0.0 and abs(e) > 2.0 * min(self.corridor_full, self.funnel_frac * dist):        # far off the line: new line
            self.line0 = pos
            self.pair = self.leg = None
            dl = self.to_wind(psi, (target[0] - pos[0], target[1] - pos[1]))
            dl = (dl[0] / dist, dl[1] / dist)
            e = 0.0
        if dist <= self.approach_factor * self.arrival_radius:
            return self._approach(t, psi, hull, pos, target, dist, e, self.corridor)
        plan = hull.plan_pair(dl, self.min_vmg)
        if self.steady is not None and (plan is None or plan["kind"] == "motor"):
            r = self._steady_command(psi, self._uz, self.steady.track(dl, e), 0.0, "direct", dist, e, 0.0, None)
            if r is not None:
                self.corridor = self.corridor_full = 0.0
                return self._steady_finish(psi, r, dist, e, 0.0, t)
        if plan is None:
            self.mode = "no progress"
            self.pair = self.leg = None
            pt = max(pts, key=lambda p: p.vx * dl[0] + p.vy * dl[1])
            return self._command(psi, pt, "no progress", 0.0, False, dist, 0.0, e, 0.0)
        # keep the previous pair while it is within the hysteresis of the best
        if self.pair is not None and self.pair[0] in byk and self.pair[1] in byk:
            io, jo = byk[self.pair[0]], byk[self.pair[1]]
            old = hull.pair_value(dl, io, jo, self.min_vmg)
            if old is not None:
                same = pts[io].kind == pts[jo].kind == "soaring"
                if ((plan["kind"] == "soaring" and same and old[0] >= (1 - self.hysteresis) * plan["r"])
                        or (plan["kind"] == "motor"
                            and old[1] <= (1 + self.hysteresis) * plan["per_m"])):
                    ci = dl[0] * pts[io].vy - dl[1] * pts[io].vx
                    cj = dl[0] * pts[jo].vy - dl[1] * pts[jo].vx
                    plan.update(i=io, j=jo, r=old[0], per_m=old[1], wi=old[2], wj=old[3], ci=ci, cj=cj)
        A, B = pts[plan["i"]], pts[plan["j"]]          # A pushes to the right (c <= 0), B to the left
        cmax = max(abs(plan["ci"]), abs(plan["cj"]))
        leg_s = max(self.min_leg_s, self.tack_cost_s / self.max_tack_loss)
        E_full = min(leg_s * cmax / 2.0, self.max_corridor_frac * nl)
        E = min(E_full, self.funnel_frac * dist)
        self.corridor, self.corridor_full = E, E_full
        tw = self.to_wind(psi, (target[0] - pos[0], target[1] - pos[1]))
        e_lo, e_hi = self._limits(hull, dl, tw[0] * dl[0] + tw[1] * dl[1], E, plan)
        if cmax < CROSS_ON_LINE:
            return self._approach(t, psi, hull, pos, target, dist, e, E)
        self.pair = (A.key, B.key)
        prev = self.leg
        if prev not in (A.key, B.key):
            cur = A if (A.vx * dl[0] + A.vy * dl[1]) >= (B.vx * dl[0] + B.vy * dl[1]) else B
        else:
            cur = A if prev == A.key else B
            if t - self.last_switch_t >= self.min_leg_s or E < 0.999 * E_full:
                if cur is B and e >= e_hi:
                    cur = A
                elif cur is A and e <= e_lo:
                    cur = B
        tacked = prev is not None and cur.key != prev
        if tacked:
            self.last_switch_t = t
        self.leg = cur.key
        mode = "upwind tacking" if dl[0] < 0 else "downwind gybing"
        self.mode = mode
        return self._command(psi, cur, mode, plan["r"], tacked, dist, plan["per_m"], e, E)


def sail_hull(source, wind_fn, start, target, nav=None, dt=1.0, nav_every_s=5.0, t_max=12 * 3600.0,
              physical_tack_cost_s=None, **nav_kw):
    """Kinematic run on the hull: a point moves with the mean ground velocity of the commanded cycle.

    wind_fn(t) -> (u10, wind_from_deg). A leg switch pauses the motion for the tack cost. Returns a
    dict: track (N, 2), t, n_legs (leg switches), switch_times, time (to arrive, nan if not), arrived,
    mean_vmg (net distance closed over elapsed time), energy_j (electric, from the points' power),
    modes (t, mode), distance."""
    nav = nav or HullNavigator(source, **nav_kw)
    nav.reset()
    pc = nav.tack_cost_s if physical_tack_cost_s is None else float(physical_tack_cost_s)
    pos = np.asarray(start, float).copy()
    target = np.asarray(target, float)
    d0 = float(np.hypot(*(target - pos)))
    track, times, modes, sw = [pos.copy()], [0.0], [], []
    t, pause_until, next_call, cmd, energy, arrived = 0.0, -1.0, 0.0, None, 0.0, False
    while t < t_max:
        u10, wf = wind_fn(t)
        if t >= next_call:
            cmd = nav.step(t, pos, u10, wf, target)
            next_call = t + (1.0 if cmd.mode == "direct" else nav_every_s)   # every second in the final approach
            modes.append((t, cmd.mode))
            if cmd.mode == "arrived":
                arrived = True
                break
            if cmd.tacked:
                sw.append(t)
                pause_until = t + pc
        if float(np.hypot(*(target - pos))) <= nav.arrival_radius:
            arrived = True
            break
        if t >= pause_until and cmd.mode != "no progress":
            w = nav.to_world(nav._psi(wf), (cmd.point.vx, cmd.point.vy))
            pos = pos + dt * np.array(w)
            energy += cmd.point.power * dt
        t += dt
        track.append(pos.copy())
        times.append(t)
    d1 = float(np.hypot(*(target - pos)))
    return dict(track=np.array(track), t=np.array(times), modes=modes, n_legs=len(sw),
                switch_times=sw, time=t if arrived else math.nan, arrived=arrived,
                mean_vmg=(d0 - d1) / t if t > 0 else 0.0, energy_j=energy, distance=d0)


def hull_summary(hull, step_deg=1.0):
    """Numbers of one hull for the record: the smallest downwind velocity component of a soaring
    cycle (min_downwind_ms; positive means no soaring cycle can hold position), the support function
    straight upwind (best soaring velocity component towards the wind's eye), the half angle of the
    sector of directions off the downwind axis that soaring alone can make good along
    (soaring_sector_deg), the speed made good straight downwind, and the motor plan straight upwind
    (speed, electric energy per metre)."""
    soar = [p for p in hull.points if p.kind == "soaring"]
    sector = 0.0
    if soar:
        a = 0.0
        while a <= 180.0:
            d = (math.cos(math.radians(a)), math.sin(math.radians(a)))
            if hull.ray_speed(d) is None:
                break
            sector = a
            a += step_deg
    up = hull.plan_pair((-1.0, 0.0))
    return {"u10": hull.u10, "n_soaring": len(soar), "n_motor": len(hull.points) - len(soar),
            "min_downwind_ms": min((p.vx for p in soar), default=math.nan),
            "support_upwind_ms": hull.support((-1.0, 0.0)),
            "soaring_sector_deg": sector,
            "downwind_ms": hull.ray_speed((1.0, 0.0)),
            "motor_upwind_ms": None if up is None or up["kind"] != "motor" else up["r"],
            "motor_upwind_j_per_m": None if up is None or up["kind"] != "motor" else up["per_m"]}
