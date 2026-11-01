"""sim.world: quantization, geometry, interpolation, packing, fetching with a fake service, and the privacy guard."""
import json
import math
import tempfile
import unittest
import urllib.parse
from pathlib import Path

import numpy as np

from sim import world as W

FIXTURE = Path(__file__).resolve().parent / "fixtures" / "world_era5_two_points.json"


# ---------------------------------------------------------------- a fake Open-Meteo


def truth(var, lat, lon, t):
    """The fake service's smooth fields (t in hours since 2023-01-01)."""
    if var == "wind_speed_10m":
        return 8.0 + 0.05 * lat + 0.02 * lon + 0.1 * t % 5.0
    if var == "wind_direction_10m":
        return (250.0 + 0.5 * lon + 2.0 * t) % 360.0
    if var == "wave_height":
        return 2.0 + 0.02 * abs(lat) + 0.01 * (t % 24)
    if var in ("wave_peak_period", "swell_wave_period"):
        return 11.0 + 0.01 * lon % 3
    if var in ("wave_direction", "swell_wave_direction"):
        return (200.0 + lat + t) % 360.0
    if var == "swell_wave_height":
        return 1.0 + 0.01 * abs(lat)
    raise KeyError(var)


T_BASE = 1672531200  # 2023-01-01T00:00Z


class FakeService:
    def __init__(self, land_lat=None):
        self.calls = []
        self.land_lat = land_lat     # marine values are null north of this latitude (a land stand-in)

    def __call__(self, url):
        self.calls.append(url)
        q = urllib.parse.parse_qs(urllib.parse.urlsplit(url).query)
        lats = [float(x) for x in q["latitude"][0].split(",")]
        lons = [float(x) for x in q["longitude"][0].split(",")]
        d0 = W.parse_utc(q["start_date"][0] + "T00:00Z")
        d1 = W.parse_utc(q["end_date"][0] + "T23:00Z")
        times = list(range(d0, d1 + 1, 3600))
        marine = "marine" in url
        out = []
        for la, lo in zip(lats, lons):
            h = {"time": times}
            for v in q["hourly"][0].split(","):
                vals = [round(truth(v, la, lo, (t - T_BASE) / 3600.0), 2) for t in times]
                if marine and self.land_lat is not None and la > self.land_lat:
                    vals = [None] * len(times)
                h[v] = vals
            out.append({"latitude": la, "longitude": lo, "hourly": h})
        return json.dumps(out if len(out) > 1 else out[0]).encode(), {"X-RateLimit-Cost": str(float(len(out)))}


class FakeClock:
    def __init__(self, t=1_700_000_000.0):
        self.t = t
        self.slept = 0.0

    def __call__(self):
        return self.t

    def sleep(self, s):
        self.t += s
        self.slept += s


def pacer(tmp, **limits):
    c = FakeClock()
    return W.Pacer(tmp, limits=limits or None, clock=c, sleep=c.sleep), c


# ---------------------------------------------------------------- tests


class TestQuantize(unittest.TestCase):
    def test_round_trip_within_half_a_step(self):
        rng = np.random.default_rng(3)
        ranges = {"u10": (-31.7, 31.7), "v10": (-31.7, 31.7), "hs": (0.0, 15.2), "hsw": (0.0, 15.2),
                  "tsw": (0.0, 25.4), "dsw": (0.0, 360.0)}
        for name, (a, b) in ranges.items():
            x = rng.uniform(a, b, 5000)
            y = W.dequantize(name, W.quantize(name, x)[0])
            step = W.QUANT[name]["step"]
            if name == "dsw":
                err = np.abs((y - x + 180.0) % 360.0 - 180.0)
            else:
                err = np.abs(y - x)
            self.assertLessEqual(err.max(), step / 2 + 1e-9, name)

    def test_missing_and_clipping(self):
        for name in W.QUANT:
            c, _ = W.quantize(name, [np.nan, 1.0])
            y = W.dequantize(name, c)
            self.assertTrue(np.isnan(y[0]), name)
            self.assertTrue(np.isfinite(y[1]), name)
        c, k = W.quantize("u10", [40.0, -40.0, 3.0])
        self.assertEqual(k, 2)
        self.assertAlmostEqual(W.dequantize("u10", c)[0], 127 * 0.25)
        c, k = W.quantize("hs", [20.0, -1.0])
        self.assertEqual(k, 2)
        self.assertAlmostEqual(W.dequantize("hs", c)[1], 0.0)
        # a direction of 360 wraps to 0
        self.assertAlmostEqual(W.dequantize("dsw", W.quantize("dsw", [360.0])[0])[0], 0.0)


class TestGeometry(unittest.TestCase):
    def test_aeqd_round_trip(self):
        rng = np.random.default_rng(1)
        for lat0, lon0 in ((-50.0, 58.0), (0.0, 179.5), (65.0, -20.0)):
            x, y = rng.uniform(-900, 900, 200), rng.uniform(-900, 900, 200)
            la, lo = W.aeqd_inverse(lat0, lon0, x, y)
            x2, y2 = W.aeqd_forward(lat0, lon0, la, lo)
            self.assertLess(np.abs(x2 - x).max(), 1e-6)
            self.assertLess(np.abs(y2 - y).max(), 1e-6)

    def test_distance_from_centre_is_exact(self):
        la, lo = W.aeqd_inverse(-50.0, 58.0, np.array([300.0]), np.array([400.0]))
        self.assertAlmostEqual(W.great_circle_km(-50.0, 58.0, la[0], lo[0]), 500.0, places=6)

    def test_corridor(self):
        for bearing in (90.0, 30.0):
            c = dict(W.CORRIDOR, lat=-45.0, lon=179.0, bearing_deg=bearing)
            lat, lon = W.corridor_points(c)
            na, nc, d = c["na"], c["nc"], c["step_km"]
            self.assertEqual(len(lat), na * nc)
            LA, LO = lat.reshape(na, nc), lon.reshape(na, nc)
            mid = (nc - 1) // 2
            # the centre line is a great circle through the centre: its length is exact
            self.assertAlmostEqual(W.great_circle_km(LA[0, mid], LO[0, mid], LA[-1, mid], LO[-1, mid]),
                                   (na - 1) * d, places=5)
            # the initial bearing from the centre along the corridor
            i0 = (na - 1) // 2
            p1, p2 = math.radians(LA[i0, mid]), math.radians(LA[i0 + 1, mid])
            dl = math.radians(LO[i0 + 1, mid] - LO[i0, mid])
            b = math.degrees(math.atan2(math.sin(dl) * math.cos(p2),
                                        math.cos(p1) * math.sin(p2) - math.sin(p1) * math.cos(p2) * math.cos(dl)))
            self.assertAlmostEqual(b % 360.0, bearing, places=6)
            # neighbours are a step apart within 0.3 % over the whole band
            dd = [W.great_circle_km(LA[i, j], LO[i, j], LA[i + 1, j], LO[i + 1, j])
                  for i in range(na - 1) for j in range(nc)]
            dd += [W.great_circle_km(LA[i, j], LO[i, j], LA[i, j + 1], LO[i, j + 1])
                   for i in range(na) for j in range(nc - 1)]
            self.assertLess(max(abs(x / d - 1) for x in dd), 0.003)
            # the index of each point is its grid position
            fi, fj = W.corridor_index(c, lat, lon)
            I, J = np.meshgrid(np.arange(na), np.arange(nc), indexing="ij")
            self.assertLess(np.abs(fi - I.ravel()).max(), 1e-6)
            self.assertLess(np.abs(fj - J.ravel()).max(), 1e-6)
            # the band wraps across 180 deg of longitude without a jump
            self.assertTrue(np.all((lon >= -180.0) & (lon < 180.0)))


def linear_corridor():
    c = dict(W.CORRIDOR, lat=10.0, lon=-30.0, kind="corridor", t0=T_BASE, dt_h=3, nt=5)
    na, nc, nt = c["na"], c["nc"], c["nt"]
    I, J = np.meshgrid(np.arange(na), np.arange(nc), indexing="ij")
    vals = {}
    for name, f in (("u10", lambda k: 1.0 + 0.3 * I + 0.2 * J + 0.5 * k), ("hs", lambda k: 2.0 + 0.1 * I - 0.05 * J + 0.1 * k)):
        vals[name] = np.stack([f(k).ravel() for k in range(nt)])
    vals["dsw"] = np.full((nt, na * nc), 350.0)
    vals["dsw"][:, ::2] = 10.0
    c["vals"] = vals
    return c


class TestInterpolation(unittest.TestCase):
    def test_bilinear_in_space_linear_in_time(self):
        c = linear_corridor()
        lat, lon = W.corridor_points(c)
        rng = np.random.default_rng(5)
        for _ in range(50):
            fi, fj = rng.uniform(0, c["na"] - 1), rng.uniform(0, c["nc"] - 1)
            k = rng.uniform(0, c["nt"] - 1)
            (ax, ay), (cx, cy) = W._axes(c["bearing_deg"])
            s, q = (fi - (c["na"] - 1) / 2) * c["step_km"], (fj - (c["nc"] - 1) / 2) * c["step_km"]
            la, lo = W.aeqd_inverse(c["lat"], c["lon"], s * ax + q * cx, s * ay + q * cy)
            r = W.sample_corridor(c, float(la), float(lo), T_BASE + k * 3 * 3600)
            self.assertTrue(r["inside"])
            self.assertAlmostEqual(r["u10"], 1.0 + 0.3 * fi + 0.2 * fj + 0.5 * k, places=6)
            self.assertAlmostEqual(r["hs"], 2.0 + 0.1 * fi - 0.05 * fj + 0.1 * k, places=6)
            # 350 and 10 deg average to north through the unit vectors, never to 180
            self.assertLess(abs((r["dsw"] + 180) % 360 - 180), 10.0 + 1e-9)

    def test_outside_and_missing(self):
        c = linear_corridor()
        r = W.sample_corridor(c, c["lat"] + 20.0, c["lon"], T_BASE)
        self.assertFalse(r["inside"])
        # a missing corner is left out and the others renormalized
        c["vals"]["hs"][0, 0] = np.nan
        lat, lon = W.corridor_points(c)
        r = W.sample_corridor(c, float(lat[0]), float(lon[0]), T_BASE)
        self.assertAlmostEqual(r["hs"], 2.0 + 0.1 * r["fi"] - 0.05 * r["fj"], places=1)
        self.assertTrue(np.isfinite(r["hs"]))

    def test_global_wraps(self):
        g = dict(W.GLOBAL, kind="global")
        lat, lon = W.global_grid(g)
        g["mask"] = np.ones(lat.size, bool)
        g["vals"] = {"hs": 1.0 + 0.1 * (lat + 70.0), "u10": np.cos(np.radians(lon))}
        r = W.sample_global(g, 12.5, 177.5)
        self.assertAlmostEqual(r["hs"], 1.0 + 0.1 * 82.5, places=9)
        self.assertAlmostEqual(r["u10"], (math.cos(math.radians(175.0)) + math.cos(math.radians(180.0))) / 2, places=9)
        g["mask"][(lat == 10.0) & (lon == 175.0)] = False
        g["vals"] = {k: v[g["mask"]] for k, v in g["vals"].items()}
        self.assertTrue(np.isfinite(W.sample_global(g, 10.0, 175.0)["hs"]))


class TestPack(unittest.TestCase):
    def test_round_trip(self):
        c = linear_corridor()
        c["vals"]["v10"] = -c["vals"]["u10"]
        d = json.loads(json.dumps(W.pack_field(c)))
        f = W.unpack_field(d)
        for name in c["vals"]:
            a, b = np.asarray(c["vals"][name]), f["vals"][name]
            err = np.abs((b - a + 180) % 360 - 180) if name == "dsw" else np.abs(b - a)
            self.assertLessEqual(err.max(), W.QUANT[name]["step"] / 2 + 1e-9, name)
        self.assertEqual(len(W.unb64(d["data"])), c["nt"] * len(c["vals"]) * c["na"] * c["nc"])

    def test_land_and_mask(self):
        rings = W.load_land()
        self.assertGreater(len(rings), 50)
        # a point in each ocean and on each continent
        sea = [(0.0, -150.0), (-50.0, 58.0), (30.0, -40.0), (-20.0, 80.0)]
        land = [(45.0, 100.0), (-25.0, 135.0), (40.0, -100.0), (0.0, 20.0), (-80.0, 0.0), (-10.0, -55.0)]
        self.assertFalse(W.on_land(rings, [p[0] for p in sea], [p[1] for p in sea]).any())
        self.assertTrue(W.on_land(rings, [p[0] for p in land], [p[1] for p in land]).all())
        m = W.ocean_mask(rings)
        self.assertGreater(m.mean(), 0.6)
        self.assertLess(m.mean(), 0.8)


class TestFetch(unittest.TestCase):
    def test_real_response_format(self):
        fx = json.loads(FIXTURE.read_text(encoding="utf-8"))
        loc = fx["waves"][1]
        hs = W.hourly(loc, "wave_height", "era5_ocean")
        self.assertTrue(np.isnan(hs[5]))
        self.assertEqual(np.isnan(hs).sum(), 1)
        t = loc["hourly"]["time"]
        v = W.at_times(loc, "wave_peak_period", "era5_ocean", [t[3], t[3] + 7 * 86400])
        self.assertAlmostEqual(v[0], loc["hourly"]["wave_peak_period"][3])
        self.assertTrue(np.isnan(v[1]))
        spec = dict(W.CORRIDOR, na=2, nc=1, lat=0.0, lon=0.0)
        cor = W.build_corridor(spec, t[0], fx["wind"], fx["waves"], "era5_waves", frames={"dt_h": 3, "nt": 8})
        self.assertTrue(np.all(cor["vals"]["hsw"][np.isfinite(cor["vals"]["hsw"])]
                               <= cor["vals"]["hs"][np.isfinite(cor["vals"]["hsw"])] + 1e-12))
        u = cor["vals"]["u10"][0, 0]
        sp, dr = fx["wind"][0]["hourly"]["wind_speed_10m"][0], fx["wind"][0]["hourly"]["wind_direction_10m"][0]
        self.assertAlmostEqual(u, -sp * math.sin(math.radians(dr)), places=9)
        self.assertEqual(W.wind_speed_from(*W.wind_uv(sp, dr))[1], float(dr % 360))

    def test_batches_cache_and_cost(self):
        with tempfile.TemporaryDirectory() as tmp:
            fake = FakeService()
            p, clock = pacer(tmp)
            lat, lon = np.linspace(-40, 40, 250), np.linspace(-170, 170, 250)
            locs, reqs = W.fetch_locations("era5_wind", lat, lon, "2023-07-07", "2023-07-09", tmp, p, fake, log=lambda m: None)
            self.assertEqual(len(fake.calls), 3)
            self.assertEqual(len(locs), 250)
            self.assertAlmostEqual(locs[17]["latitude"], lat[17], places=3)
            self.assertAlmostEqual(p.used()["day"], 250.0)
            locs2, reqs2 = W.fetch_locations("era5_wind", lat, lon, "2023-07-07", "2023-07-09", tmp, p, fake, log=lambda m: None)
            self.assertEqual(len(fake.calls), 3)          # all from the cache
            self.assertTrue(all(r["cached"] for r in reqs2))
            self.assertEqual(json.dumps(locs2), json.dumps(locs))
            self.assertIn("cell_selection=sea", fake.calls[0])
            self.assertIn("models=era5", fake.calls[0])

    def test_pacer(self):
        with tempfile.TemporaryDirectory() as tmp:
            p, clock = pacer(tmp, minute=100.0, hour=250.0, day=400.0)
            p.wait(100.0)
            self.assertEqual(clock.slept, 0.0)
            p.note(100.0, "u")
            p.wait(50.0)                          # the minute is full: waits until it has passed
            self.assertGreaterEqual(clock.slept, 60.0)
            p.note(50.0, "u")
            clock.t += 120.0
            p.note(90.0, "u")
            clock.t += 61.0
            t_before = clock.t
            p.wait(90.0)                          # 240 used in the hour: waits for the first note to leave it
            self.assertGreater(clock.t - t_before, 3000.0)
            with self.assertRaises(W.BudgetExceeded):
                p.wait(400.0)

    def test_weight(self):
        self.assertEqual(W.location_weight(2, 7), 1.0)
        self.assertEqual(W.location_weight(2, 70), 1.0)
        self.assertAlmostEqual(W.location_weight(15, 7), 1.5)
        self.assertAlmostEqual(W.location_weight(10, 28), 2.0)


class TestScenarios(unittest.TestCase):
    def test_one_scenario_end_to_end(self):
        with tempfile.TemporaryDirectory() as tmp:
            fake = FakeService()
            p, _ = pacer(tmp)
            out = Path(tmp) / "out"
            idx = W.make_scenarios(["trade_wind"], cache=tmp, out_dir=out, pacer=p, get=fake, log=lambda m: None)
            self.assertEqual([s["id"] for s in idx["scenarios"]], ["trade_wind"])
            d = W.load_js(out / "trade_wind.js")
            cor, glob = W.unpack_field(d["corridor"]), W.unpack_field(d["global"])
            s = next(x for x in W.SCENARIOS if x["id"] == "trade_wind")
            t0 = W.parse_utc(s["start"])
            self.assertEqual(cor["t0"], t0)
            self.assertEqual(cor["swell"], "lower_bound")
            # the centre point at a frame time carries the service's value, within the step
            r = W.sample_corridor(cor, s["lat"], s["lon"], t0 + 6 * 3600)
            th = (t0 + 6 * 3600 - T_BASE) / 3600.0
            sp, dr = truth("wind_speed_10m", s["lat"], s["lon"], th), truth("wind_direction_10m", s["lat"], s["lon"], th)
            u, v = W.wind_uv(round(sp, 2), round(dr, 2))
            self.assertAlmostEqual(r["u10"], float(u), delta=0.125 + 0.02)
            self.assertAlmostEqual(r["v10"], float(v), delta=0.125 + 0.02)
            self.assertAlmostEqual(r["hs"], truth("wave_height", s["lat"], s["lon"], th), delta=0.03 + 0.01)
            self.assertEqual(int(glob["mask"].sum()), int(W.ocean_mask(W.load_land()).sum()))
            # sizes: one byte per value
            n_ocean = int(glob["mask"].sum())
            self.assertEqual(len(W.unb64(d["global"]["data"])), 3 * n_ocean)
            self.assertEqual(len(W.unb64(d["corridor"]["data"])), W.FRAMES["nt"] * 6 * W.CORRIDOR["na"] * W.CORRIDOR["nc"])
            # the sources rebuild the very URLs that were fetched
            self.assertEqual(len(d["sources"]["corridor"]), 2)
            plat, plon = W.corridor_points(W.scenario_spec(s))
            urls = [u for g in d["sources"]["corridor"] for u in W.source_urls(g, plat, plon)]
            self.assertEqual(sorted(urls), sorted(x for x in fake.calls if x in urls))
            self.assertEqual(len(urls), 2 * math.ceil(len(plat) / W.BATCH))
            self.assertTrue(set(urls) <= set(fake.calls))
            glat, glon = W.global_grid(W.GLOBAL)
            gm = glob["mask"]
            gurls = [u for g in d["sources"]["global"] for u in W.source_urls(g, glat[gm], glon[gm])]
            self.assertTrue(set(gurls) <= set(fake.calls))
            self.assertEqual(len(gurls), 2 * math.ceil(int(gm.sum()) / W.BATCH))
            self.assertTrue(all(r[2] and len(r[3]) == 64 for g in d["sources"]["global"] for r in g["requests"]))

    def test_marine_nulls_become_missing(self):
        spec = dict(W.CORRIDOR, na=3, nc=2, lat=0.0, lon=0.0)
        fake = FakeService(land_lat=-1.0)
        lat, lon = W.corridor_points(spec)
        wind = json.loads(fake(W.request_url("era5_wind", lat, lon, "2023-07-07", "2023-07-10"))[0])
        waves = json.loads(fake(W.request_url("era5_waves", lat, lon, "2023-07-07", "2023-07-10"))[0])
        cor = W.build_corridor(spec, W.parse_utc("2023-07-07T00:00Z"), wind, waves, "era5_waves")
        self.assertTrue(np.isnan(cor["vals"]["hs"]).all())
        self.assertTrue(np.isnan(cor["vals"]["dsw"]).all())
        self.assertTrue(np.isfinite(cor["vals"]["u10"]).all())
        d = W.unpack_field(json.loads(json.dumps(W.pack_field(cor))))
        self.assertTrue(np.isnan(d["vals"]["tsw"]).all())

    def test_windows_hold_their_scenarios(self):
        for s in W.SCENARIOS:
            d0, d1 = W.WINDOWS[s["window"]]
            t0 = W.parse_utc(s["start"])
            self.assertTrue(W.parse_utc(d0 + "T00:00Z") <= t0 <= W.parse_utc(d1 + "T23:00Z"), s["id"])
            self.assertEqual(t0 % (3 * 3600), 0, s["id"])
            days = (W.parse_utc(d1 + "T00:00Z") - W.parse_utc(d0 + "T00:00Z")) / 86400 + 1
            self.assertEqual(W.location_weight(2, days), 1.0)


class TestPrivacy(unittest.TestCase):
    def test_guard_refuses_a_corridor_near_a_private_site_inside_the_repo(self):
        with tempfile.TemporaryDirectory() as tmp:
            priv = Path(tmp) / "priv"
            priv.mkdir()
            (priv / "sites.json").write_text(json.dumps({"points": [{"lat": 0.0, "lon": 0.0}]}), encoding="utf-8")
            with self.assertRaises(SystemExit):
                W.make_now(0.5, 0.5, out=W.OUT_DIR / "now_test.js", private_dir=priv, get=FakeService(),
                           cache=tmp, pacer=pacer(tmp)[0], with_global=False, log=lambda m: None)
            self.assertFalse((W.OUT_DIR / "now_test.js").exists())
            out = Path(tmp) / "elsewhere" / "now.js"
            W.make_now(0.5, 0.5, out=out, private_dir=priv, get=FakeService(), cache=tmp, pacer=pacer(tmp)[0],
                       with_global=False, now=W.parse_utc("2023-07-07T05:00Z"), log=lambda m: None)
            d = W.load_js(out)
            self.assertEqual(d["corridor"]["t0"], W.parse_utc("2023-07-07T03:00Z"))
            self.assertEqual(d["corridor"]["swell"], "model")

    def test_no_scenario_near_the_owners_sites(self):
        sites = W.private_sites()
        if not sites:
            self.skipTest("no private store here")
        for s in W.SCENARIOS:      # never print the sites
            self.assertFalse(W.near_private(s["lat"], s["lon"], km=W.PRIVATE_GUARD_KM), s["id"])


if __name__ == "__main__":
    unittest.main()
