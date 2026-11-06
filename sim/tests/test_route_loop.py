"""sim.route on fixed routes (docs/02-form.md#route-planning): BoxField and its months joined in time, BoxSolar's
hourly split of 3-hour means, the path graph of a lap, the search's hooks (the distance left along the path, no wait
in the no-wait months or at a dry node, the steady leg alone over land), and Python against route.js on a lap in both
directions, through the runner of `python -m sim.route loop`.

Everything here is synthetic: an invented round island with an invented cape, in the open South Pacific far from any
land, with simple invented winds, waves and sunshine over a few days. It stands for no real place."""
import json
import math
import os
import shutil
import tempfile
import unittest

import numpy as np

from sim import route as R
from sim import world as W

NODE = shutil.which("node")

# the invented island: its centre, the box around it (0.25 deg, as a reanalysis grid) and the lap around it
C_LAT, C_LON = -47.0, -128.0
STEP = 0.25
NA, NC = 7, 9
LAT0, LON0 = C_LAT - 3 * STEP, C_LON - 4 * STEP
LAP_KM = 35.0
LAP_N = 12                       # vertices of the lap, one every 30 deg of bearing from the centre, from the south
# land (hs NaN) at these grid points (i, j): the island and its cape to the north, two columns wide, which the lap
# crosses between bearings 0 and about 33 deg
LAND = [(i, j) for i in (2, 3, 4, 5) for j in (4, 5)]
T_START = 1738195200             # 2025-01-30T00:00Z
T_FEB = 1738368000               # 2025-02-01T00:00Z
HOURS = 96


def lap_points(n=LAP_N, r_km=LAP_KM):
    """The lap's polyline: n vertices around the centre at r_km, clockwise from the south, closed (last = first)."""
    pts = []
    for k in range(n):
        b = math.radians(180.0 + 360.0 * k / n)
        la, lo = W.aeqd_inverse(C_LAT, C_LON, r_km * math.sin(b), r_km * math.cos(b))
        pts.append([float(la), float(lo)])
    return pts + [list(pts[0])]


def cape_range(n=LAP_N):
    """The vertex range over the cape: from the vertex before bearing 0 to the first after 33 deg."""
    a = next(k for k in range(n) if (180.0 + 360.0 * k / n) % 360.0 == 330.0)
    return [a, a + 3]


def wind_at(t, i, j):
    """An invented wind: about 14 m/s from the west that eases to a calm in the second night and backs slowly."""
    h = (t - T_START) / 3600.0
    calm = 1.0 - 0.85 * math.exp(-((h - 52.0) / 7.0) ** 2)
    speed = (13.0 + 2.0 * math.sin(h / 9.0) + 0.1 * i - 0.05 * j) * calm
    frm = 270.0 - 0.4 * h + 2.0 * math.sin(h / 5.0)
    u, v = W.wind_uv(speed, frm)
    return float(u), float(v)


def box_dict(t0, nt, dt_h=1.0, land=True):
    """A BoxField dict over nt frames from t0 (the invented winds, waves of 1.2 to 2.2 m, NaN on the land)."""
    vals = {"u10": [], "v10": [], "hs": []}
    for k in range(nt):
        t = t0 + 3600.0 * dt_h * k
        u, v, hs = [], [], []
        for i in range(NA):
            for j in range(NC):
                a, b = wind_at(t, i, j)
                u.append(a)
                v.append(b)
                hs.append(float("nan") if (land and (i, j) in LAND)
                          else 1.7 + 0.5 * math.sin((t - T_START) / 3600.0 / 11.0 + 0.3 * i))
        vals["u10"].append(u)
        vals["v10"].append(v)
        vals["hs"].append(hs)
    return {"na": NA, "nc": NC, "lat0": LAT0, "lon0": LON0, "step_deg": STEP, "t0": t0, "dt_h": dt_h, "nt": nt,
            "vals": vals}


def months():
    """The box over HOURS hours as two months: January to its last hour and February from its first."""
    nj = int((T_FEB - T_START) / 3600)
    return [box_dict(T_START, nj), box_dict(T_FEB, HOURS + 1 - nj)]


def solar_dict(t0=T_START + 3 * 3600, nt=(HOURS + 48) // 3):
    """Invented 3-hour means stamped at the end of each interval: 0.7 of the fitted model's mean there and then, a
    little patchy, over a diffuse floor of 2 W/m2 (so the night's intervals have sun in the data, none in the
    model)."""
    fit = R.solar_fit()
    out = []
    for k in range(nt):
        s = t0 + 3 * 3600.0 * k
        fr = []
        for i in range(NA):
            for j in range(NC):
                la, lo = LAT0 + i * STEP, LON0 + j * STEP
                m = sum(R.model_hour_mean(la, lo, s - 3 * 3600.0 + 3600.0 * h, fit) for h in range(3)) / 3.0
                fr.append(0.7 * m * (1.0 + 0.1 * math.sin(i + 2 * j + k)) + 2.0)
        out.append(fr)
    return {"na": NA, "nc": NC, "lat0": LAT0, "lon0": LON0, "step_deg": STEP, "t0": t0, "dt_h": 3, "nt": nt,
            "vals": {"shortwave_radiation": out}}


def loop_input(**kw):
    inp = {"fields": months(), "solar": [solar_dict()], "points": lap_points(), "dry": [cape_range()],
           "starts": [T_START + 18 * 3600.0], "aircraft": "albatross-3m", "capacity_factor": 2.0, "land": False}
    inp.update(kw)
    return inp


def walk(x):
    """Every key of a nested JSON object."""
    if isinstance(x, dict):
        for k, v in x.items():
            yield k
            yield from walk(v)
    elif isinstance(x, list):
        for v in x:
            yield from walk(v)


class TestBoxField(unittest.TestCase):
    def test_grid_points_frames_and_bounds(self):
        f = R.BoxField(box_dict(T_START, 5))
        d = box_dict(T_START, 5)
        for i, j, k in ((0, 0, 0), (6, 8, 4), (1, 3, 2)):
            loc = f.locate(LAT0 + i * STEP, LON0 + j * STEP)
            u, v, hs = f.sample(loc, T_START + 3600.0 * k)
            self.assertAlmostEqual(u, d["vals"]["u10"][k][i * NC + j], places=9)
            self.assertAlmostEqual(v, d["vals"]["v10"][k][i * NC + j], places=9)
        # linear in time between frames, bilinear in space within a cell
        loc = f.locate(LAT0 + 0.5 * STEP, LON0 + 0.25 * STEP)
        a, b = f.sample(loc, T_START)[0], f.sample(loc, T_START + 3600.0)[0]
        self.assertAlmostEqual(f.sample(loc, T_START + 900.0)[0], a + 0.25 * (b - a), places=12)
        g = d["vals"]["u10"][0]
        want = 0.375 * g[0] + 0.125 * g[1] + 0.375 * g[NC] + 0.125 * g[NC + 1]
        self.assertAlmostEqual(a, want, places=12)
        # on the land (all four corners NaN) there is no sea; outside the box no location; the longitude wraps
        self.assertTrue(math.isnan(f.sample(f.locate(LAT0 + 4.5 * STEP, LON0 + 4.5 * STEP), T_START)[2]))
        self.assertIsNone(f.locate(LAT0 - 0.01, LON0))
        self.assertIsNone(f.locate(LAT0, LON0 + NC * STEP))
        self.assertEqual(f.locate(LAT0, LON0 + 360.0)[:2], f.locate(LAT0, LON0)[:2])
        self.assertEqual((f.t0, f.t_end, f.nt, f.dt), (T_START, T_START + 4 * 3600.0, 5, 3600.0))
        self.assertGreater(f.u10_max, 10.0)

    def test_months_join_continuously(self):
        """The two months joined sample as one dict over the whole time, across the join as well; a month that
        repeats the other's last frame joins the same; a gap or a frame off the time grid is refused."""
        one = R.BoxField(box_dict(T_START, HOURS + 1))
        jan, feb = months()
        two = R.BoxField.join([feb, jan])
        self.assertEqual((two.t0, two.nt, two.t_end), (one.t0, one.nt, one.t_end))
        rng = np.random.default_rng(7)
        times = [T_FEB - 3600.0 + x for x in (0.0, 1.0, 1800.0, 3599.0, 3600.0)] + list(rng.uniform(T_START, one.t_end, 30))
        for t in times:
            la, lo = LAT0 + rng.uniform(0, NA - 1) * STEP, LON0 + rng.uniform(0, NC - 1) * STEP
            a, b = one.sample(one.locate(la, lo), t), two.sample(two.locate(la, lo), t)
            for x, y in zip(a, b):
                self.assertTrue(x == y or (math.isnan(x) and math.isnan(y)), (t, x, y))
        over = box_dict(T_FEB - 3600.0, HOURS + 2 - int((T_FEB - T_START) / 3600))
        three = R.BoxField.join([jan, over])
        for n in R.BoxField.NAMES:
            self.assertTrue(np.array_equal(np.asarray(three.vals[n]), np.asarray(two.vals[n]), equal_nan=True), n)
        with self.assertRaises(ValueError):
            R.BoxField.join([jan, box_dict(T_FEB + 3600.0, 5)])
        with self.assertRaises(ValueError):
            R.BoxField.join([jan, box_dict(T_FEB + 1800.0, 5)])


class TestBoxSolar(unittest.TestCase):
    def test_hours_sum_back_to_the_means(self):
        d = solar_dict()
        lay = R.BoxSolar(d)
        self.assertEqual(lay.t0, d["t0"] - 3 * 3600.0)
        self.assertEqual(lay.nt, 3 * d["nt"])
        rng = np.random.default_rng(3)
        flat = sunny = 0
        for _ in range(60):
            la, lo = LAT0 + rng.uniform(0, NA - 1) * STEP, LON0 + rng.uniform(0, NC - 1) * STEP
            loc = lay.locate(la, lo)
            q = int(rng.integers(0, d["nt"]))
            ta = lay.t0 + 3 * 3600.0 * q
            hours = [lay.hour_mean(loc, ta + 3600.0 * h) for h in range(3)]
            mean = R.bilinear(lay.v[q], NA, NC, loc[0], loc[1])
            self.assertAlmostEqual(sum(hours), 3.0 * mean, delta=1e-9 * max(1.0, mean))
            model = [R.model_hour_mean(la, lo, ta + 3600.0 * h, 1.0) for h in range(3)]
            if sum(model) == 0.0:
                flat += 1
                self.assertEqual(hours, [mean] * 3)          # no sun in the model: the mean held flat
            else:
                sunny += 1
                for h in range(3):
                    self.assertAlmostEqual(hours[h], mean * 3.0 * model[h] / sum(model), delta=1e-9 * max(1.0, mean))
        self.assertGreater(flat, 5)
        self.assertGreater(sunny, 5)
        self.assertIsNone(lay.hour_mean(loc, lay.t0 - 3600.0))
        self.assertIsNone(lay.hour_mean(loc, lay.t0 + 3600.0 * lay.nt))
        self.assertIsNone(lay.locate(LAT0 - 1.0, LON0))
        with self.assertRaises(ValueError):
            R.BoxSolar(dict(d, t0=d["t0"] + 1800.0))

    def test_tables_take_the_box_solar(self):
        field = R.BoxField.join(months())
        lay = R.BoxSolar(solar_dict())
        g = R.path_graph(field, lap_points(), R.SUB_KM, [cape_range()], False)
        en = R.energy_defaults("albatross-3m")
        tab = R.Tables(g, field, en, lay)
        n = 3
        ll = lay.locate(g.lat[n], g.lon[n])
        for k in (30, 40, 50, 60):
            th = tab.T0 + 3600.0 * k
            want = lay.hour_mean(ll, th)
            self.assertIsNotNone(want)
            self.assertAlmostEqual(tab.P[n][k], want * en["panel_m2"], places=12)
        self.assertGreater(tab.layer_hours, 0)


class TestPathGraph(unittest.TestCase):
    def test_nodes_legs_and_distance_left(self):
        field = R.BoxField.join(months())
        pts = lap_points()
        a, b = cape_range()
        g = R.path_graph(field, pts, R.SUB_KM, [[a, b]], False)
        N = len(g.lat)
        self.assertEqual(len(g.vertex_node), len(pts))
        for i, n in enumerate(g.vertex_node):
            self.assertEqual((g.lat[n], g.lon[n]), tuple(pts[i]))
        # a closed loop: the end is a node of its own at the start's place
        self.assertEqual(g.vertex_node[-1], N - 1)
        self.assertNotEqual(g.vertex_node[0], g.vertex_node[-1])
        self.assertEqual((g.lat[0], g.lon[0]), (g.lat[-1], g.lon[-1]))
        total = 0.0
        for n in range(N - 1):
            self.assertEqual(len(g.edges[n]), 1)
            m, sub_m, steps = g.edges[n][0]
            self.assertEqual(m, n + 1)
            L = R.gc_km(g.lat[n], g.lon[n], g.lat[m], g.lon[m]) * 1000.0
            self.assertLessEqual(L, R.SUB_KM * 1000.0 + 1e-6)
            self.assertAlmostEqual(sub_m * len(steps), L, places=6)
            self.assertAlmostEqual(g.rem_m[n] - g.rem_m[m], L, places=6)
            total += L
        self.assertEqual(g.edges[N - 1], [])
        self.assertEqual(g.rem_m[-1], 0.0)
        self.assertAlmostEqual(g.rem_m[0], total, places=3)
        poly = sum(R.gc_km(*pts[i], *pts[i + 1]) for i in range(len(pts) - 1)) * 1000.0
        self.assertAlmostEqual(g.rem_m[0], poly, delta=1e-6 * poly)
        # the cape: its legs dry, the nodes strictly inside dry, its two end vertices wet
        va, vb = g.vertex_node[a], g.vertex_node[b]
        self.assertEqual([n for n in range(N) if g.dry_node[n]], list(range(va + 1, vb)))
        self.assertEqual([n for n in range(N - 1) if g.dry_edge[n][0]], list(range(va, vb)))
        for n in range(N):
            hs = field.sample(g.loc[n], T_START)[2]
            self.assertEqual(math.isfinite(hs), not g.dry_node[n] or n in (va + 1, vb - 1), n)
        with self.assertRaises(ValueError):
            R.path_graph(field, pts, R.SUB_KM, [[b, a]], False)
        with self.assertRaises(ValueError):
            R.path_graph(field, [pts[0], [C_LAT + 5.0, C_LON]], R.SUB_KM, None, False)

    def test_land_hits_are_listed_not_removed(self):
        """Land rings around the invented cape: with the dry range no wet node or leg touches them; without it the
        cape's nodes and legs are listed and kept."""
        field = R.BoxField.join(months())
        lo0, lo1 = C_LON - 0.05, C_LON + 0.3
        la0, la1 = C_LAT + 0.25, C_LAT + 0.45
        ring = np.array([[lo0, la0], [lo1, la0], [lo1, la1], [lo0, la1]])
        g = R.path_graph(field, lap_points(), R.SUB_KM, [cape_range()], [ring])
        self.assertEqual(g.land_hits, {"nodes": [], "legs": []})
        g2 = R.path_graph(field, lap_points(), R.SUB_KM, None, [ring])
        self.assertTrue(g2.land_hits["nodes"])
        self.assertTrue(g2.land_hits["legs"])
        self.assertEqual(len(g2.lat), len(g.lat))
        self.assertTrue(all(len(g2.edges[n]) == 1 for n in range(len(g2.lat) - 1)))


class TestLoopSearch(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.field = R.BoxField.join(months())
        cls.layer = R.BoxSolar(solar_dict())
        cls.lm = R.leg_model("albatross-3m")
        cls.en = R.energy_defaults("albatross-3m", 2.0)

    def plan(self, g, t0, tab=None):
        tab = tab or R.Tables(g, self.field, self.en, self.layer)
        return R.plan(self.field, g, self.lm, self.en, None, None, t0=t0, layer=self.layer, tables=tab,
                      start_node=0, dest_node=len(g.lat) - 1)

    def graph(self, dry=True):
        return R.path_graph(self.field, lap_points(), R.SUB_KM, [cape_range()] if dry else None, False)

    def test_lap_from_node_0_to_the_last(self):
        g = self.graph()
        r = self.plan(g, T_START + 18 * 3600.0)
        self.assertTrue(r["ok"], r["reason_text"])
        wps = r["route"]["waypoints"]
        self.assertEqual((wps[0]["node"], wps[-1]["node"]), (0, len(g.lat) - 1))
        self.assertEqual(r["dist_km"], 0.0)
        self.assertAlmostEqual(r["path_km"], g.rem_m[0] / 1000.0, places=9)
        self.assertEqual(r["stats"]["reached"], len(g.lat) - 1)
        # the distance left along the path bounds the time to go no worse than the great circle to the end, which
        # is the start's place: the same arrival with no more labels taken
        rem, g.rem_m = g.rem_m, None
        r2 = self.plan(g, T_START + 18 * 3600.0)
        g.rem_m = rem
        self.assertTrue(r2["ok"])
        self.assertEqual(wps[-1]["t"], r2["route"]["waypoints"][-1]["t"])
        self.assertLessEqual(r["stats"]["pops"], r2["stats"]["pops"])

    def test_no_wait_months(self):
        """No wait on the water in January at the nodes from vertex 2 to vertex 5: the plan without the rule waits
        there in January, the plan with it never does, and still waits elsewhere or in February."""
        t0 = T_START + 6 * 3600.0
        g = self.graph()
        lo, hi = g.vertex_node[2], g.vertex_node[5]

        def jan_waits(r):
            return [lg for lg in r["route"]["legs"] if lg["mode"] == "waiting" and R.utc_month(lg["t0"]) == 1
                    and lo <= lg["from"] <= hi]

        free = self.plan(g, t0)
        self.assertTrue(free["ok"])
        self.assertTrue(jan_waits(free))
        R.set_no_wait(g, [1], (2, 5))
        self.assertEqual([n for n in range(len(g.lat)) if g.no_wait_months[n]], list(range(lo, hi + 1)))
        kept = self.plan(g, t0)
        self.assertTrue(kept["ok"], kept["reason_text"])
        self.assertEqual(jan_waits(kept), [])
        self.assertTrue(any(lg["mode"] == "waiting" for lg in kept["route"]["legs"]))
        # every node in January: no wait at all before February
        R.set_no_wait(g, [1])
        r = self.plan(g, t0)
        rt = r["route"] or r["closest"]
        self.assertFalse(any(lg["mode"] == "waiting" and R.utc_month(lg["t0"]) == 1 for lg in rt["legs"]))
        self.assertEqual(R.utc_month(T_FEB - 1.0), 1)
        self.assertEqual(R.utc_month(T_FEB), 2)

    def test_dry_stretch_on_the_steady_leg(self):
        """Over the cape the plan flies the steady leg on every step, never waits at a dry node, and pays no
        take-off there; without the dry range the cape's missing sea stops every route there."""
        g = self.graph()
        dry_from = [n for n in range(len(g.lat) - 1) if g.dry_edge[n][0]]
        for h in (6, 18, 30):
            r = self.plan(g, T_START + h * 3600.0)
            rt = r["route"] or r["closest"]
            seen = 0
            for lg in rt["legs"]:
                if lg["mode"] == "waiting":
                    self.assertFalse(g.dry_node[lg["from"]])
                elif lg["from"] in dry_from:
                    seen += 1
                    self.assertTrue(all(m == "motor" for m in lg["modes"]))
                    self.assertGreater(lg["motor_wh"], 0.0)
                    if g.dry_node[lg["from"]]:
                        self.assertEqual(lg["takeoff_wh"], 0.0)
            self.assertEqual(seen, len(dry_from))
        wet = self.graph(dry=False)
        r = self.plan(wet, T_START + 18 * 3600.0)
        self.assertFalse(r["ok"])
        self.assertIsNone(r["closest"])
        self.assertLessEqual(r["stats"]["reached"], g.vertex_node[cape_range()[0]] + 1)
        self.assertGreater(r["stats"]["pruned"]["seas"], 0)


def counts_close(a, b):
    from sim.tests.test_route_js import counts_close as cc
    return cc(a, b)


@unittest.skipUnless(NODE, "node not available")
class TestLoopJs(unittest.TestCase):
    STARTS = [T_START + h * 3600.0 for h in (6, 18, 30, 42)]

    def test_python_and_js_same_plans_both_directions(self):
        """The runner's jobs in Python and in node on the lap with the cape and a no-wait rule: the same plans in
        both directions (routes, times, charges, summaries, margins, label counts)."""
        inp = loop_input(starts=self.STARTS, no_wait=[{"months": [2], "vertices": [2, 5]}])
        R.loop_node_boxes(inp)
        seen = set()
        for name in R.LOOP_DIRECTIONS:
            py = R.loop_job((inp, name, self.STARTS))
            js = R.loop_job_node((inp, name, self.STARTS))
            self.assertEqual(len(py), len(js))
            for k, (p, j) in enumerate(zip(py, js)):
                msg = f"{name} start {k}"
                seen.add((name, p["ok"]))
                for f in ("ok", "reason", "start_node", "dest_node"):
                    self.assertEqual(p[f], j[f], msg)
                self.assertAlmostEqual(p["path_km"], j["path_km"], delta=1e-9)
                for f in ("pops", "labels", "reached"):
                    self.assertTrue(counts_close(p["stats"][f], j["stats"][f]), f"{msg} {f}")
                self.assertEqual(p.get("relaxed"), j.get("relaxed"), msg)
                for key in ("route", "closest"):
                    a, b = p.get(key), j.get(key)
                    self.assertEqual(a is None, b is None, msg)
                    if a is None:
                        continue
                    self.assertEqual([w["node"] for w in a["waypoints"]], [w["node"] for w in b["waypoints"]], msg)
                    for x, y in zip(a["waypoints"], b["waypoints"]):
                        self.assertAlmostEqual(x["t"], y["t"], delta=1e-6, msg=msg)
                        self.assertAlmostEqual(x["soc_wh"], y["soc_wh"], delta=1e-8, msg=msg)
                        self.assertEqual(x["water"], y["water"], msg)
                    self.assertEqual([lg["modes"] for lg in a["legs"]], [lg["modes"] for lg in b["legs"]], msg)
                    for f, v in a["summary"].items():
                        self.assertAlmostEqual(v, b["summary"][f], delta=1e-8, msg=f"{msg} {f}")
                    ma = a.get("margins") or {f: a[f] for f in ("reserve_margin_wh", "night_margin_wh", "allowance_margin_wh")}
                    mb = b.get("margins") or {f: b[f] for f in ("reserve_margin_wh", "night_margin_wh", "allowance_margin_wh")}
                    for f in ma:
                        self.assertAlmostEqual(ma[f], mb[f], delta=1e-6, msg=f"{msg} {f}")
                    self.assertEqual(a["night_low"]["node"], b["night_low"]["node"], msg)
                    self.assertAlmostEqual(a["night_low"]["night_margin_wh"], b["night_low"]["night_margin_wh"], delta=1e-6)
        for x in (("forward", True), ("forward", False), ("reverse", False)):
            self.assertIn(x, seen)          # routes found and closest routes are both compared

    def test_runner_writes_the_plans_and_no_coordinates(self):
        """python -m sim.route loop on a private-style input file: the plans of both directions with their
        summaries in the output file, which holds no coordinates, and a log that prints none."""
        inp = loop_input(starts=self.STARTS[:2], land=True)
        lines = []
        with tempfile.TemporaryDirectory() as d:
            src, dst = os.path.join(d, "lap.json"), os.path.join(d, "out", "plans.json")
            with open(src, "w", encoding="utf-8") as fh:
                json.dump(inp, fh)              # NaN as Python writes it; the runner reads it
            R.run_loop(src, dst, workers=2, engine="node", log=lines.append)
            with open(dst, encoding="utf-8") as fh:
                out = json.load(fh)
        self.assertEqual(len(out["plans"]), 4)
        self.assertEqual([p["direction"] for p in out["plans"]], ["forward", "forward", "reverse", "reverse"])
        keys = set(walk(out))
        self.assertNotIn("lat", keys)
        self.assertNotIn("lon", keys)
        text = "\n".join(lines)
        for la, lo in inp["points"]:
            self.assertNotIn(f"{la:.2f}", text)
            self.assertNotIn(f"{lo:.2f}", text)
        g = out["graphs"]["forward"]
        self.assertEqual(g["land_hits"], {"nodes": [], "legs": []})      # no land of land.js in the open sea here
        self.assertEqual(g["wet_nodes_without_sea"], [])
        ok = [p for p in out["plans"] if p["summary"]["ok"]]
        self.assertTrue(ok)
        for p in out["plans"]:
            s = p["summary"]
            if s["of"] is None:
                continue
            self.assertAlmostEqual(sum(s["share_time"].values()), 1.0, places=9)
            self.assertAlmostEqual(sum(s["share_distance"].values()), 1.0, places=9)
            self.assertAlmostEqual(s["km"]["soaring"] + s["km"]["motor"], s["path_km"], places=6)
            self.assertGreater(s["km"]["dry"], 0.0)
            self.assertAlmostEqual(s["days"] * 24.0, s["time_h"], places=12)
            self.assertAlmostEqual(sum(x["hours"] for x in s["segments"]), s["time_h"], places=6)
            self.assertEqual(len(s["segments"]), LAP_N)
            self.assertEqual([x["between"] for x in s["segments"]], [[i, i + 1] for i in range(LAP_N)])
            hours = [w["hours"] for w in s["longest_waits"]]
            self.assertEqual(hours, sorted(hours, reverse=True))
            self.assertAlmostEqual(sum(hours), s["share_time"]["wait"] * s["time_h"], places=6)
            mw = [x["motor_wh"] for x in s["most_motor"]]
            self.assertEqual(mw, sorted(mw, reverse=True))
            self.assertAlmostEqual(sum(mw), s["motor_wh"], places=6)
            self.assertIn("night_margin_wh", s["night"])
        for p in ok:
            self.assertTrue(p["summary"]["night"]["carried"])
            self.assertGreaterEqual(p["summary"]["lowest_charge"]["soc_wh"], out["energy"]["reserve_wh"] - 1e-9)
        # the reverse direction's summaries name the input's vertices
        rv = [p for p in out["plans"] if p["direction"] == "reverse" and p["summary"]["of"]]
        for p in rv:
            self.assertEqual(sorted(x["between"][0] for x in p["summary"]["segments"]), list(range(LAP_N)))


if __name__ == "__main__":
    unittest.main()
