"""sim.route: the sun, the solar fit, the field sampler against sim.world, the great-circle geometry, the battery
tables, the leg model against sim.navigator, the energy defaults against the record, the solar layer's packing, and
searches on small synthetic corridors (soaring downwind, the motor allowance, waiting through a night)."""
import math
import unittest

import numpy as np

from sim import record
from sim import route as R
from sim import world as W

T_BASE = 1688169600      # 2023-07-01T00:00Z


def small_corridor(u10=16.0, wind_from=270.0, hs=1.0, lat=-15.0, lon=-10.0, na=7, nc=5, nt=25, t0=T_BASE):
    """A corridor of uniform wind and waves (the layout of sim.world.unpack_field)."""
    u, v = W.wind_uv(u10, wind_from)
    n = na * nc
    vals = {"u10": np.full((nt, n), float(u)), "v10": np.full((nt, n), float(v)), "hs": np.full((nt, n), hs),
            "hsw": np.full((nt, n), 0.5), "tsw": np.full((nt, n), 10.0), "dsw": np.full((nt, n), 200.0)}
    return {"kind": "corridor", "lat": lat, "lon": lon, "bearing_deg": 90.0, "na": na, "nc": nc, "step_km": 55.6,
            "t0": t0, "dt_h": 3, "nt": nt, "vals": vals}


class TestSun(unittest.TestCase):
    def test_noon_and_night(self):
        # 2023-03-21 near 12:07 UTC the sun stands almost overhead at 0 N 0 E; at midnight it is far below
        t = 1679400000 + 7 * 60   # 2023-03-21T12:07Z
        self.assertGreater(R.sun_sin_elev(0.0, 0.0, t), 0.999)
        self.assertLess(R.sun_sin_elev(0.0, 0.0, t - 12 * 3600), -0.99)

    def test_night_length_at_50s_in_june(self):
        n = R.night_hours(-50.0, 70.0, 1687348800)     # 2023-06-21T12:00Z
        self.assertGreater(n, 15.5)
        self.assertLess(n, 16.5)
        self.assertAlmostEqual(R.night_hours(0.0, 0.0, 1687348800), 12.0, delta=0.3)

    def test_fit_reproduces_the_yearly_insolation(self):
        fit = R.solar_fit()
        lat, lon, year = R.SOLAR_FIT_POINT
        t0 = 1672531200.0       # 2023-01-01
        s = 0.0
        for d in range(365):
            for k in range(288):
                s += max(0.0, R.sun_sin_elev(lat, lon, t0 + d * 86400.0 + (k + 0.5) * 300.0))
        mean_kwh = fit * s / 12.0 / 365.0 / 1000.0
        self.assertAlmostEqual(mean_kwh, record.value("lit.insolation_50s_year_kwh_m2_day"), places=9)


class TestField(unittest.TestCase):
    def test_sampler_matches_sim_world(self):
        d = W.load_js(W.OUT_DIR / "np_winter.js")
        f = W.unpack_field(d["corridor"])
        cf = R.CorridorField(f)
        rng = np.random.default_rng(4)
        for _ in range(40):
            fi, fj = rng.uniform(0, f["na"] - 1), rng.uniform(0, f["nc"] - 1)
            (ax, ay), (cx, cy) = W._axes(f["bearing_deg"])
            s, q = (fi - (f["na"] - 1) / 2) * f["step_km"], (fj - (f["nc"] - 1) / 2) * f["step_km"]
            la, lo = W.aeqd_inverse(f["lat"], f["lon"], s * ax + q * cx, s * ay + q * cy)
            t = f["t0"] + rng.uniform(-3600, 80 * 3600)
            ref = W.sample_corridor(f, float(la), float(lo), t)
            loc = cf.locate(float(la), float(lo))
            u, v, hs = cf.sample(loc, t)
            self.assertAlmostEqual(u, ref["u10"], places=10)
            self.assertAlmostEqual(v, ref["v10"], places=10)
            self.assertAlmostEqual(hs, ref["hs"], places=10)
        self.assertIsNone(cf.locate(f["lat"] + 20.0, f["lon"]))


class TestGeometry(unittest.TestCase):
    def test_great_circle(self):
        a, b = (-50.0, 58.0), (-48.0, 64.0)
        L = R.gc_km(*a, *b)
        p0, p1 = R.gc_point(*a, *b, 0.0), R.gc_point(*a, *b, 1.0)
        self.assertAlmostEqual(p0[0], a[0], places=9)
        self.assertAlmostEqual(p1[1], b[1], places=9)
        m = R.gc_point(*a, *b, 0.5)
        self.assertAlmostEqual(R.gc_km(*a, *m) + R.gc_km(*m, *b), L, places=6)
        self.assertAlmostEqual(R.gc_bearing(0.0, 0.0, 1.0, 0.0), 0.0, places=9)
        self.assertAlmostEqual(R.gc_bearing(0.0, 0.0, 0.0, 1.0), 90.0, places=9)
        self.assertEqual(len(R.ring_offsets(1)), 8)
        self.assertEqual(len(R.ring_offsets(2)), 16)

    def test_graph_of_a_corridor(self):
        cf = R.CorridorField(small_corridor())
        g = R.corridor_graph(cf, land=False)
        self.assertEqual(len(g.lat), 19 * 13)
        self.assertAlmostEqual(g.step_km, 55.6 / 3)
        n = 6 * 13 + 6
        self.assertEqual(len(g.edges[n]), 16)
        for m, sub_m, steps in g.edges[n]:
            self.assertLessEqual(sub_m, R.SUB_KM * 1000.0 + 1e-6)
            self.assertAlmostEqual(sub_m * len(steps), R.gc_km(g.lat[n], g.lon[n], g.lat[m], g.lon[m]) * 1000.0, places=3)

    def test_land_is_left_out(self):
        # a corridor around a part of Africa's coast: no node or leg on land
        f = small_corridor(lat=-34.0, lon=19.0)
        cf = R.CorridorField(f)
        rings = W.load_land()
        g = R.corridor_graph(cf, land=rings)
        wet = [n for n in range(len(g.lat)) if g.loc[n] is not None]
        self.assertLess(len(wet), len(g.lat))
        self.assertFalse(W.on_land(rings, [g.lat[n] for n in wet], [g.lon[n] for n in wet]).any())


class TestLegModel(unittest.TestCase):
    def test_matches_the_navigator(self):
        lm = R.leg_model("albatross-3m")
        for u10, rel in ((16.0, 0.0), (16.0, 60.0), (16.0, 180.0), (3.0, 90.0), (13.0, 120.0)):
            mode, v, pw = lm.leg(u10, rel)
            d = (math.cos(math.radians(rel)), math.sin(math.radians(rel)))
            pl = lm.source.at(u10).plan_pair(d, R.MIN_VMG, motor_ok=False)
            if pl is not None:
                self.assertEqual(mode, "soaring")
                self.assertAlmostEqual(v, pl["r"], places=12)
                self.assertEqual(pw, 0.0)
            else:
                lg = lm.steady.leg(u10, d, R.MIN_VMG)
                self.assertEqual(mode, "motor")
                self.assertAlmostEqual(v, lg["along"], places=12)
                self.assertAlmostEqual(pw, lg["power"], places=12)
        self.assertEqual(lm.leg(16.0, -60.0), lm.leg(16.0, 60.0))     # the hull's mirror images
        self.assertGreater(lm.v_max(25.0), 20.0)


class TestEnergy(unittest.TestCase):
    def test_defaults_from_the_record(self):
        from sim.patterns import SOLAR_CELL_EFF, SOLAR_COVERAGE
        en = R.energy_defaults("albatross-3m")
        cap = record.value("battery.mass_kg") * record.value("lit.cell_p45b_wh_per_kg") * R.PACK_FRACTION * R.USABLE_DEPTH
        self.assertAlmostEqual(en["capacity_wh"], cap, places=12)
        self.assertAlmostEqual(en["reserve_wh"], R.RESERVE_FRACTION * cap, places=12)
        self.assertAlmostEqual(en["flight_draw_w"], record.value("ds.avionics_power_w") + record.value(R.SENSOR_KEY))
        self.assertEqual(en["water_draw_w"], record.value("ds.avionics_power_w"))
        self.assertEqual(en["avoid_hs_m"], record.value("ds.survival_hs_m"))
        self.assertAlmostEqual(en["panel_m2"], record.value("form.albatross_3m.wing_area_m2") * SOLAR_COVERAGE * SOLAR_CELL_EFF)
        self.assertEqual(record.value("patterns.solar_cell_eff"), SOLAR_CELL_EFF)
        self.assertEqual(record.value("patterns.solar_coverage"), SOLAR_COVERAGE)
        self.assertAlmostEqual(R.energy_defaults("albatross-3m", 2.0)["capacity_wh"], 2.0 * cap, places=12)

    def test_night_requirement_by_hand(self):
        cf = R.CorridorField(small_corridor())
        g = R.corridor_graph(cf, refine=1, rings=1, land=False)
        en = dict(R.energy_defaults("albatross-3m"), capacity_wh=100.0, reserve_wh=10.0, flight_draw_w=4.0)
        tab = R.Tables(g, cf, en, None)
        n = 0
        P = tab.P[n]
        # the first hour of the first night after the field's start, and its end
        k0 = next(k for k in range(24, tab.K) if P[k] - 4.0 < 0 <= P[k - 1] - 4.0)
        k1 = next(k for k in range(k0, tab.K) if P[k] - 4.0 >= 0) - 1
        night = sum(4.0 - P[k] for k in range(k0, k1 + 1))
        t = tab.T0 + 3600.0 * k0
        self.assertAlmostEqual(tab.need_at(n, t, 10.0), 10.0 + night, places=9)
        self.assertAlmostEqual(tab.need_at(n, tab.T0 + 3600.0 * k1 + 1800.0, 10.0), 10.0 + (4.0 - P[k1]) / 2.0, places=9)
        # the solar energy of an interval is the hourly powers over its overlap
        self.assertAlmostEqual(tab.solar_wh(n, t - 5400.0, t), P[k0 - 2] / 2.0 + P[k0 - 1], places=9)
        # a battery too small for the night: the strict requirement is infinite, the margin's requirement is the
        # night's whole draw (what a full battery at the night's start lacks counts as a shortfall)
        small = dict(en, capacity_wh=20.0)
        tab2 = R.Tables(g, cf, small, None)
        self.assertEqual(tab2.need_at(n, t, 10.0), math.inf)
        self.assertAlmostEqual(tab2.need_at(n, t, 10.0, raw=True), 10.0 + night, places=9)
        day = tab2.T0 + 3600.0 * (k0 - 3)          # three hours before the night: held at the capacity plus the shortfall
        lack = 10.0 + night - 20.0
        want = max(10.0, 20.0 - sum(P[k] - 4.0 for k in range(k0 - 3, k0))) + lack
        self.assertAlmostEqual(tab2.need_at(n, day, 10.0, raw=True), want, places=9)


class TestSolarLayer(unittest.TestCase):
    def test_packing_and_hours(self):
        f = small_corridor(na=19, nc=11)
        lats, lons, spec = R.solar_points(f)
        self.assertEqual((spec["na"], spec["nc"]), (10, 6))
        nt = 24 + 3 * 24 + R.LOOKAHEAD_H
        t0 = f["t0"] - 86400
        times = [t0 + 3600 * k for k in range(nt + 2)]
        locs = [{"hourly": {"time": times, "shortwave_radiation": [float((k * 37 + i) % 900) for k in range(nt + 2)]}}
                for i in range(len(lats))]
        obj = R.build_solar_layer("test", f, locs, [{"url": "u", "retrieved_utc": "r", "sha256": "s"}])
        lay = R.SolarLayer(obj)
        self.assertEqual(lay.nt, nt)
        for i in (0, 7, 59):
            for k in (0, 5, nt - 1):
                v = lay.v[k][i]
                want = float(((k + 1) * 37 + i) % 900)     # hour k is the value stamped at its end
                self.assertLessEqual(abs(v - want), R.SOLAR_Q / 2.0 + 1e-9)
        self.assertIsNone(lay.hour_mean(lay.locate(lats[0], lons[0]), t0 - 3600.0))


class TestSearch(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.lm = R.leg_model("albatross-3m")

    def plan(self, f, a, b, **en_kw):
        cf = R.CorridorField(f)
        g = R.corridor_graph(cf, land=False)
        en = dict(R.energy_defaults("albatross-3m", 2.0), **en_kw)
        return R.plan(cf, g, self.lm, en, a, b), g

    def test_downwind_soaring(self):
        f = small_corridor(u10=16.0, wind_from=270.0, t0=T_BASE + 10 * 3600)     # wind from the west, local morning
        a, b = W.aeqd_inverse(f["lat"], f["lon"], -120.0, 0.0), W.aeqd_inverse(f["lat"], f["lon"], 120.0, 0.0)
        r, g = self.plan(f, (float(a[0]), float(a[1])), (float(b[0]), float(b[1])))
        self.assertTrue(r["ok"], r.get("reason_text"))
        sm = r["route"]["summary"]
        self.assertEqual(sm["motor_h"], 0.0)
        self.assertEqual(sm["wait_h"], 0.0)
        v = r["dist_km"] * 1000.0 / (sm["time_h"] * 3600.0)
        best = self.lm.leg(16.0, 0.0)[1]
        self.assertLessEqual(v, best + 1e-6)
        self.assertGreater(v, 0.8 * best)
        # the battery follows the balance of each leg: solar less draw less motor, at most the capacity
        en = R.energy_defaults("albatross-3m", 2.0)
        prev = r["route"]["waypoints"][0]["soc_wh"]
        for lg in r["route"]["legs"]:
            want = prev + lg["solar_wh"] - lg["draw_wh"] - lg["motor_wh"]
            self.assertLessEqual(lg["soc_end_wh"], min(en["capacity_wh"], want) + 1e-9)
            if want < en["capacity_wh"] - 1e-6 and prev < en["capacity_wh"] - 20.0:
                self.assertAlmostEqual(lg["soc_end_wh"], want, places=9)
            prev = lg["soc_end_wh"]

    def test_no_allowance_no_motor_route(self):
        f = small_corridor(u10=2.0, wind_from=270.0, t0=T_BASE + 10 * 3600)       # calm: only the motor makes way
        a, b = W.aeqd_inverse(f["lat"], f["lon"], -60.0, 0.0), W.aeqd_inverse(f["lat"], f["lon"], 60.0, 0.0)
        r, _ = self.plan(f, (float(a[0]), float(a[1])), (float(b[0]), float(b[1])), motor_share=0.0)
        self.assertFalse(r["ok"])
        self.assertEqual(r["reason"], 5)
        self.assertIsNotNone(r["closest"])
        self.assertLess(r["closest"]["allowance_margin_wh"], 0.0)
        r2, _ = self.plan(f, (float(a[0]), float(a[1])), (float(b[0]), float(b[1])))
        self.assertTrue(r2["ok"])
        self.assertGreater(r2["route"]["summary"]["motor_h"], 0.0)

    def test_small_battery_fails_on_the_night(self):
        f = small_corridor(u10=16.0, wind_from=270.0, t0=T_BASE + 10 * 3600)
        a, b = W.aeqd_inverse(f["lat"], f["lon"], -120.0, 0.0), W.aeqd_inverse(f["lat"], f["lon"], 120.0, 0.0)
        en0 = R.energy_defaults("albatross-3m", 2.0)
        r, _ = self.plan(f, (float(a[0]), float(a[1])), (float(b[0]), float(b[1])), capacity_wh=20.0, soc0_wh=20.0,
                         reserve_wh=4.0)
        self.assertFalse(r["ok"])
        self.assertIn(r["reason"], (2, 3))
        self.assertGreater(r["start_need_wh"], 0.0)
        self.assertLess(en0["flight_draw_w"] * 9.0, 2 * en0["capacity_wh"])

    def test_waits_through_the_night(self):
        # evening start in a calm sea: the motor's allowance is spent, the drone waits on the water for the morning
        f = small_corridor(u10=1.0, wind_from=270.0, t0=T_BASE + 15 * 3600, nt=25)
        a, b = W.aeqd_inverse(f["lat"], f["lon"], -150.0, 0.0), W.aeqd_inverse(f["lat"], f["lon"], 150.0, 0.0)
        r, _ = self.plan(f, (float(a[0]), float(a[1])), (float(b[0]), float(b[1])))
        self.assertTrue(r["ok"], r.get("reason_text"))
        sm = r["route"]["summary"]
        self.assertGreater(sm["wait_h"], 4.0)
        self.assertGreater(sm["takeoff_wh"], 0.0)
        legs = r["route"]["legs"]
        waits = [lg for lg in legs if lg["mode"] == "waiting"]
        self.assertTrue(all(lg["motor_wh"] == 0.0 for lg in waits))
        # never below the reserve, and the night carried at every waypoint
        en = R.energy_defaults("albatross-3m", 2.0)
        self.assertGreaterEqual(sm["min_soc_wh"], en["reserve_wh"] - 1e-9)


if __name__ == "__main__":
    unittest.main()
