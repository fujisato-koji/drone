import gzip
import json
import tempfile
import unittest
from pathlib import Path

import numpy as np

from sim import climate as C
from sim import climate_keys as CK
from sim import record

T0 = 1104537600  # 2005-01-01T00:00Z


def _write(path, lat, lon, hourly):
    path.parent.mkdir(parents=True, exist_ok=True)
    body = json.dumps({"latitude": lat, "longitude": lon, "hourly": hourly}).encode("utf-8")
    path.write_bytes(gzip.compress(body))


def _series(n, u, hs):
    t = (T0 + 3600 * np.arange(n)).tolist()
    wind = {"time": t, "wind_speed_10m": list(u), "wind_direction_10m": [270.0] * n}
    waves = {"time": t, "wave_height": list(hs), "wave_period": [8.0] * n, "wave_peak_period": [12.0] * n,
             "wave_direction": [250.0] * n}
    return wind, waves


class TestSpells(unittest.TestCase):
    def test_runs(self):
        w = [False, True, True, False, True, False, False, True, True, True]
        self.assertEqual(C.spell_runs(w), [(1, 2), (4, 1), (7, 3)])
        self.assertEqual(C.spell_runs([True, True]), [(0, 2)])
        self.assertEqual(C.spell_runs([False]), [])

    def test_merge(self):
        runs = [(1, 2), (4, 1), (7, 3)]
        # break of 1 h between the first two, 2 h before the third
        self.assertEqual(C.merge_runs(runs, 2), [(1, 4), (7, 3)])
        self.assertEqual(C.merge_runs(runs, 3), [(1, 9)])

    def test_months(self):
        m = C.months_of([T0, T0 + 31 * 86400, T0 + 365 * 86400 - 3600])
        self.assertEqual(m.tolist(), [1, 2, 12])

    def test_swell_lower_bound(self):
        u = np.array([0.0, 8.0, 20.0])
        full = C.wind_sea_full_hs(u)
        self.assertLess(full[0], 0.01)
        self.assertGreater(full[2], full[1])
        sw = C.swell_lower_bound(np.array([2.0, full[1], full[2] * 0.5]), u)
        self.assertAlmostEqual(sw[0], 2.0, delta=0.01)
        self.assertAlmostEqual(sw[1], 0.0, delta=1e-9)
        self.assertEqual(sw[2], 0.0)


class TestAnalysis(unittest.TestCase):
    def setUp(self):
        self.tmp = tempfile.TemporaryDirectory()
        base = Path(self.tmp.name)
        self.cache, self.private = base / "cache", base / "private"
        # one public point (so_1) and one private point, 30 days of hours
        n = 24 * 30
        u = np.full(n, 15.0)
        u[10:34] = 3.0      # a 24 h spell below 6
        u[100:105] = 7.0    # a 5 h spell below 8 only
        u[200:202] = 12.0   # a 2 h spell below wing.best_u10_min_ms only
        hs = np.full(n, 1.0)
        hs[10:22] = 3.5
        wind, waves = _series(n, u, hs)
        _write(self.cache / "so_1_wind.json.gz", -45.0, 52.0, wind)
        _write(self.cache / "so_1_waves.json.gz", -45.0, 52.0, waves)
        self.secret = (12.345, 98.765)
        (self.private).mkdir(parents=True)
        (self.private / "sites.json").write_text(json.dumps(
            {"points": [{"id": "home_1", "lat": self.secret[0], "lon": self.secret[1]}]}), encoding="utf-8")
        raw = self.private / "climate" / "raw"
        _write(raw / "home_1_wind.json.gz", self.secret[0], self.secret[1], wind)
        _write(raw / "home_1_waves.json.gz", self.secret[0], self.secret[1], waves)
        self.res = C.analyse(self.cache, self.private)

    def tearDown(self):
        self.tmp.cleanup()

    def test_shares_and_spells(self):
        g = self.res["regions"]["so"]
        n = 24 * 30
        self.assertAlmostEqual(g["below"]["u6"]["year"], round(24 / n, 4))
        self.assertAlmostEqual(g["below"]["dsmin"]["year"], round(29 / n, 4))
        self.assertAlmostEqual(g["below"]["best"]["year"], round(31 / n, 4))
        sp = g["spells"]["best"]["year"]
        self.assertEqual(sp["count"], 3)
        self.assertEqual(sp["max_h"], 24.0)
        self.assertEqual(sp["p50_h"], 5.0)
        self.assertEqual(g["spells"]["u6"]["djf"]["count"], 1)
        self.assertEqual(g["spells"]["u6"]["jja"]["count"], 0)

    def test_weak_wind_waves(self):
        w = self.res["regions"]["so"]["weak_wind_waves"]["u6"]["year"]
        self.assertEqual(w["weak_hours"], 24)
        self.assertAlmostEqual(w["hs_ge3_share"], 0.5)
        self.assertAlmostEqual(w["swell_ge3_share"], 0.5)
        self.assertEqual(w["tp_p50_s_swell_ge2"], 12.0)

    def test_energy(self):
        av = record.value("ds.avionics_power_w")
        pw = record.value("wing.front1_c9_w")
        e = self.res["regions"]["so"]["energy"]["best"]["best"]["year"]["max"]
        self.assertAlmostEqual(e["float_wh"], av * 24.0, places=3)
        self.assertAlmostEqual(e["motor_wh"], (pw + av) * 24.0, places=1)

    def test_private_points_stay_private(self):
        text = json.dumps(self.res)
        for x in self.secret:
            self.assertNotIn(str(x), text)
        self.assertNotIn("lat", self.res["points"]["home_1"])
        self.assertNotIn("home_1", self.res["sources"])
        self.assertIn("home", self.res["regions"])

    def test_keys_and_tables(self):
        ks = CK.keys(self.res)
        names = [k for k, _, _ in ks]
        self.assertEqual(len(names), len(set(names)))
        self.assertIn("climate.so_year_below_dsmin_share", names)
        self.assertIn("climate.home_best_float_p99_wh", names)
        self.assertEqual(CK.basis_of("climate.home_year_u10_mean_ms"), [CK.BASIS, "private:loc-01"])
        self.assertEqual(CK.basis_of("climate.so_year_u10_mean_ms"), [CK.BASIS])
        block = CK.values_block(self.res)
        for x in self.secret:
            self.assertNotIn(str(x), block)
        doc = "".join(f"<!-- climate_keys {n} begin -->\n<!-- climate_keys {n} end -->\n" for n in CK.TABLES)
        out = CK.replace_tables(doc, self.res)
        cells = {c.strip() for line in out.splitlines() if line.startswith("|") for c in line.split("|")}
        for c in cells:
            if c.startswith("climate."):
                self.assertIn(c, names)


if __name__ == "__main__":
    unittest.main()
