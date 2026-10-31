"""Wind and swell climate from ERA5: weak-wind spells, swell during them, and the battery to ride them out.

    python -m sim.climate fetch [--private DIR] [--cache DIR] [--pause S]
        download the hourly ERA5 series of every point (Open-Meteo archive and marine APIs) into the
        cache, one request at a time with a pause between them (the free API counts a 20-year request as
        about 520 calls against 5,000 an hour and 10,000 a day); cached files are not fetched again.
    python -m sim.climate [--private DIR] [--cache DIR]
        analyse the cached series and write sim/out/climate.json.
    python -m sim.climate_keys
        write the record keys and the key tables of docs/02-form.md#wind-climate from that file.

Points: three public points in the Southern Ocean (wandering albatross range) and three in the North
Pacific (Laysan and black-footed albatross range) are in PUBLIC_POINTS below. The points of the home
operating area (private:loc-01) are read from <private>/sites.json and are never written anywhere in the
repository: their raw downloads go to <private>/climate/raw and the output names them home_1, home_2 only.

Data: ERA5 (Hersbach et al. 2020) hourly 10 m wind speed and direction at 0.25 deg through the Open-Meteo
archive API (models=era5), and ERA5 wave height, mean and peak period and direction at 0.5 deg through
the Open-Meteo marine API (models=era5_ocean). ERA5 through Open-Meteo has no swell and wind-sea
partition, so the swell is estimated from the total wave height (see swell_lower_bound).

Standard library and numpy only; light on CPU (a few seconds per point).
"""
from __future__ import annotations

import argparse
import datetime as _dt
import gzip
import hashlib
import json
import sys
import time
import urllib.error
import urllib.parse
import urllib.request
from pathlib import Path

import numpy as np

from sim import record

ROOT = Path(__file__).resolve().parents[1]
OUT_PATH = ROOT / "sim" / "out" / "climate.json"
DEFAULT_CACHE = ROOT / "sim" / "cache" / "climate"
DEFAULT_PRIVATE = ROOT.parent / "drone-private"
_SIM = Path(__file__).resolve().parent
# files the output depends on: the stamp covers them, in this order. The record values it reads are listed
# in the output's inputs block instead (sim.climate_keys rewrites record/values.yaml after each run), and the
# data files with their sha256 in its sources block (public points only).
MODEL_FILES = [_SIM / "climate.py", _SIM / "seawind.py", _SIM / "patterns.py"]

START_DATE = "2005-01-01"
END_DATE = "2024-12-31"

ARCHIVE_URL = "https://archive-api.open-meteo.com/v1/archive"
MARINE_URL = "https://marine-api.open-meteo.com/v1/marine"
WIND_VARS = ("wind_speed_10m", "wind_direction_10m")
WAVE_VARS = ("wave_height", "wave_period", "wave_peak_period", "wave_direction")
LICENCE = ("ERA5: CC BY 4.0 in the Copernicus Climate Data Store since 2025-07-02 (the Licence to use Copernicus "
           "Products before); Open-Meteo API data: CC BY 4.0, free API for non-commercial use")
DATASETS = {
    "wind": {"url": ARCHIVE_URL, "vars": WIND_VARS, "model": "era5",
             "dataset": "ERA5 hourly single levels, 10 m wind (0.25 deg), via the Open-Meteo historical weather API",
             "licence": LICENCE},
    "waves": {"url": MARINE_URL, "vars": WAVE_VARS, "model": "era5_ocean",
              "dataset": "ERA5 ocean waves, significant height, mean and peak period (0.5 deg), via the Open-Meteo marine API",
              "licence": LICENCE},
}

# Public points (lat, lon in degrees; on the 0.5 deg grid, which both ERA5 grids share), and why:
# so_1 45S 52E: open water about 160 km north of the Crozet Islands, inside the foraging range of the
#   wandering albatrosses breeding there, away from the islands' wake.
# so_2 50S 65E: the latitude of the record's Southern Ocean insolation point (P51, 50S 70E), moved 5 deg
#   west so that it lies upwind of the Kerguelen Islands in the westerlies rather than in their wake.
# so_3 55S 120W: the open South Pacific sector of the circumpolar range, far from any land.
# np_1 35N 150W: north-east of the Hawaiian colonies, where breeding Laysan and black-footed albatrosses
#   forage in the subtropical gyre (light winds in summer).
# np_2 40N 170W: the North Pacific transition zone, the main foraging belt of both species.
# np_3 45N 160E: the north-west Pacific south-east of the Kuril Islands, at the edge of the subarctic gyre
#   north of the Kuroshio extension, a winter foraging area.
PUBLIC_POINTS = (
    {"id": "so_1", "region": "so", "lat": -45.0, "lon": 52.0},
    {"id": "so_2", "region": "so", "lat": -50.0, "lon": 65.0},
    {"id": "so_3", "region": "so", "lat": -55.0, "lon": -120.0},
    {"id": "np_1", "region": "np", "lat": 35.0, "lon": -150.0},
    {"id": "np_2", "region": "np", "lat": 40.0, "lon": -170.0},
    {"id": "np_3", "region": "np", "lat": 45.0, "lon": 160.0},
)
REGIONS = ("so", "np", "home")
SEASONS = {"djf": (12, 1, 2), "mam": (3, 4, 5), "jja": (6, 7, 8), "son": (9, 10, 11),
           "year": tuple(range(1, 13))}

# The skim study's best soaring minimum: the best wing (AR 20, SD7037, ballast to the mass limit) with
# 10 deg dihedral, the 0.3 m floor and the skid touching the water (docs/02-form.md#wingtip-skim).
SKIM_KEY = "skim.ar20l_dih10_h03_c000_u10_ms"
CALM_U10_MS = 6.0
SPELL_PCTS = (50, 90, 99)
HOURS_PER_YEAR = 8766.0
HS_LEVELS_M = (2.0, 3.0)


def thresholds() -> dict:
    """Name -> u10 threshold (m/s): u6, dsmin (ds.min_u10_ms), skim (SKIM_KEY), best (wing.best_u10_min_ms)."""
    return {"u6": CALM_U10_MS, "dsmin": record.value("ds.min_u10_ms"), "skim": record.value(SKIM_KEY),
            "best": record.value("wing.best_u10_min_ms")}


# ---------------------------------------------------------------- points and cache

def private_points(private_dir) -> list:
    """The home-area points from <private_dir>/sites.json, as dicts with id, region "home", lat, lon."""
    p = Path(private_dir) / "sites.json"
    if not p.exists():
        return []
    d = json.loads(p.read_text(encoding="utf-8"))
    # the ids are made here, never taken from the file, so no name from the private store reaches an output
    return [{"id": f"home_{i + 1}", "region": "home", "lat": float(q["lat"]), "lon": float(q["lon"]),
             "private": True} for i, q in enumerate(d["points"])]


def all_points(private_dir) -> list:
    return [dict(p, private=False) for p in PUBLIC_POINTS] + private_points(private_dir)


def cache_dir_for(point, cache, private_dir) -> Path:
    """Raw downloads of a private point go only under the private store."""
    return (Path(private_dir) / "climate" / "raw") if point["private"] else Path(cache)


def request_url(point, kind) -> str:
    ds = DATASETS[kind]
    q = {"latitude": point["lat"], "longitude": point["lon"], "start_date": START_DATE, "end_date": END_DATE,
         "hourly": ",".join(ds["vars"]), "models": ds["model"], "timezone": "GMT", "timeformat": "unixtime"}
    if kind == "wind":
        q["wind_speed_unit"] = "ms"
    return ds["url"] + "?" + urllib.parse.urlencode(q)


def raw_path(point, kind, cache, private_dir) -> Path:
    return cache_dir_for(point, cache, private_dir) / f"{point['id']}_{kind}.json.gz"


def fetch_one(point, kind, cache, private_dir, retries=6, wait_s=900.0) -> bool:
    """Download one series unless cached; True when a request was made. Waits and retries on a limit."""
    path = raw_path(point, kind, cache, private_dir)
    if path.exists():
        return False
    path.parent.mkdir(parents=True, exist_ok=True)
    url = request_url(point, kind)
    for attempt in range(retries):
        try:
            req = urllib.request.Request(url, headers={"User-Agent": "drone-climate/1 (personal research)"})
            with urllib.request.urlopen(req, timeout=300) as r:
                body = r.read()
            d = json.loads(body)
            if d.get("error"):
                raise RuntimeError(d.get("reason"))
            break
        except urllib.error.HTTPError as e:
            msg = e.read()[:300].decode("utf-8", "replace")
            print(f"{point['id']} {kind}: HTTP {e.code} {msg}", flush=True)
            if e.code in (429, 503) and attempt + 1 < retries:
                time.sleep(wait_s)
                continue
            raise
    path.write_bytes(gzip.compress(body, mtime=0))
    meta = {"url": url, "retrieved_utc": _dt.datetime.now(_dt.timezone.utc).strftime("%Y-%m-%dT%H:%M:%SZ"),
            "dataset": DATASETS[kind]["dataset"], "licence": DATASETS[kind]["licence"],
            "sha256_json": hashlib.sha256(body).hexdigest(), "bytes": len(body)}
    path.with_name(path.name.replace(".json.gz", ".meta.json")).write_text(json.dumps(meta, indent=1),
                                                                          encoding="utf-8")
    print(f"{point['id']} {kind}: {len(body)} bytes", flush=True)
    return True


def fetch(cache, private_dir, pause_s) -> None:
    pts = all_points(private_dir)
    # whole points, regions interleaved, so that a stop part way leaves every region some points
    by_region = [[p for p in pts if p["region"] == r] for r in ("home", "so", "np")]
    order = [g[i] for i in range(max(len(g) for g in by_region)) for g in by_region if i < len(g)]
    jobs = [(p, k) for p in order for k in ("wind", "waves")]
    made = False
    for p, k in jobs:
        if made:
            time.sleep(pause_s)
        made = fetch_one(p, k, cache, private_dir)


def load_series(point, cache, private_dir):
    """(time in unix seconds, {var: float array with nan for missing}) for wind and waves merged on time."""
    out = {}
    times = None
    for kind in ("wind", "waves"):
        path = raw_path(point, kind, cache, private_dir)
        if not path.exists():
            out[kind] = None
            continue
        d = json.loads(gzip.decompress(path.read_bytes()))
        out[kind + "_grid"] = (d.get("latitude"), d.get("longitude"))
        h = d["hourly"]
        t = np.asarray(h["time"], dtype=np.int64)
        if times is None:
            times = t
        elif len(t) != len(times) or np.any(t != times):
            raise ValueError(f"{point['id']}: wind and wave hours differ")
        out[kind] = {v: np.array([np.nan if x is None else x for x in h[v]], dtype=float)
                     for v in DATASETS[kind]["vars"]}
    return times, out


def source_meta(point, cache, private_dir) -> dict:
    """URL, retrieval date, dataset and sha256 of the public downloads (private points: none)."""
    if point["private"]:
        return {}
    m = {}
    for kind in ("wind", "waves"):
        p = raw_path(point, kind, cache, private_dir)
        mp = p.with_name(p.name.replace(".json.gz", ".meta.json"))
        if mp.exists():
            m[kind] = json.loads(mp.read_text(encoding="utf-8"))
            m[kind]["licence"] = DATASETS[kind]["licence"]  # the current wording, not the one at download
    return m


# ---------------------------------------------------------------- analysis

def months_of(t_unix) -> np.ndarray:
    """Calendar month 1..12 (UTC) of each unix time."""
    return np.asarray(t_unix, dtype="datetime64[s]").astype("datetime64[M]").astype(np.int64) % 12 + 1


def spell_runs(weak) -> list:
    """(start index, length) of each run of True in a boolean array (a missing hour is not weak)."""
    w = np.concatenate(([False], np.asarray(weak, dtype=bool), [False]))
    d = np.diff(w.astype(np.int8))
    starts = np.flatnonzero(d == 1)
    ends = np.flatnonzero(d == -1)
    return list(zip(starts.tolist(), (ends - starts).tolist()))


def merge_runs(runs, max_break) -> list:
    """Join runs separated by fewer than max_break hours; a joined run spans its breaks."""
    out = []
    for s, n in runs:
        if out and s - (out[-1][0] + out[-1][1]) < max_break:
            out[-1] = (out[-1][0], s + n - out[-1][0])
        else:
            out.append((s, n))
    return out


_FD = None


def wind_sea_full_hs(u10) -> np.ndarray:
    """Fully developed (Pierson-Moskowitz) wind-sea Hs for each u10, from sim.seawind, interpolated."""
    global _FD
    if _FD is None:
        from sim.seawind import WaveTrain, Wind
        grid = np.arange(0.0, 45.01, 0.25)
        hs = np.array([WaveTrain.fully_developed(Wind(u10=max(u, 0.05))).hs for u in grid])
        _FD = (grid, hs)
    return np.interp(np.nan_to_num(u10, nan=0.0), _FD[0], _FD[1])


def swell_lower_bound(hs, u10) -> np.ndarray:
    """Swell height not explained by the largest wind sea the hour's wind can hold.

    ERA5 through Open-Meteo gives only the total significant height. A wind sea is at most fully
    developed, so sqrt(Hs^2 - Hs_full(u10)^2) (energies add) is a lower bound on the swell height; it is
    near Hs in weak wind, where the fully developed sea is small.
    """
    full = wind_sea_full_hs(u10)
    return np.sqrt(np.clip(hs * hs - full * full, 0.0, None))


def prepare(point, cache, private_dir):
    """Hourly arrays of one point, or None when its wind is not cached."""
    t, d = load_series(point, cache, private_dir)
    if d.get("wind") is None:
        return None
    u = d["wind"]["wind_speed_10m"]
    s = {"id": point["id"], "region": point["region"], "u10": u, "month": months_of(t), "hours": len(t),
         "missing_hours": {"u10": int(np.isnan(u).sum())}}
    if d.get("waves") is not None:
        hs = d["waves"]["wave_height"]
        s.update(hs=hs, tp=d["waves"]["wave_peak_period"], tm=d["waves"]["wave_period"],
                 swell=swell_lower_bound(hs, u))
        # hours without a value are left out of each statistic (the peak period ends 2024-11-18, K-019)
        s["missing_hours"].update(hs=int(np.isnan(hs).sum()), tp=int(np.isnan(s["tp"]).sum()))
    if not point["private"]:
        s["grid"] = {"wind": d.get("wind_grid"), "waves": d.get("waves_grid")}
    return s


def _pct(x, p):
    x = np.asarray(x, dtype=float)
    x = x[np.isfinite(x)]
    return float(np.percentile(x, p)) if x.size else None


def _r(x):
    """A finite float or None; the analysis keeps full precision and sim.climate_keys rounds once (K-019)."""
    return None if x is None or not np.isfinite(x) else float(x)


def _mean(x):
    return float(np.mean(x)) if np.size(x) else None


def _std(x):
    return float(np.std(x)) if np.size(x) else None


def _share(num, den):
    return None if den == 0 else num / den


def hour_weighted_pct(lengths, p):
    """Length L such that p percent of the hours in the spells lie in spells no longer than L."""
    x = np.sort(np.asarray(lengths, dtype=float))
    if not x.size:
        return None
    c = np.cumsum(x)
    return float(x[np.searchsorted(c, p / 100.0 * c[-1])])


def spell_table(series, thr, merge_break=None) -> dict:
    """Spells below thr pooled over the series, by the season of their first hour."""
    lens, months = [], []
    point_years = sum(len(s["u10"]) for s in series) / HOURS_PER_YEAR
    for s in series:
        weak = np.isfinite(s["u10"]) & (s["u10"] < thr)
        runs = spell_runs(weak)
        if merge_break:
            runs = merge_runs(runs, merge_break)
        lens += [n for _, n in runs]
        months += [int(s["month"][i]) for i, _ in runs]
    lens, months = np.array(lens, dtype=float), np.array(months, dtype=int)
    out = {}
    for name, ms in SEASONS.items():
        m = np.isin(months, ms)
        x = lens[m]
        row = {"count": int(x.size)}
        for p in SPELL_PCTS:
            row[f"p{p}_h"] = _r(_pct(x, p))
        row["max_h"] = float(x.max()) if x.size else None
        tot = float(x.sum())
        for h in (24, 72):
            row[f"hours_share_in_ge{h}h"] = _share(float(x[x >= h].sum()), tot) if tot else None
        # how often a spell longer than the percentile spell comes, per point and year of the series
        # (the season's spells over all the years), and the spell a random weak hour lies in (K-019)
        for p in (90, 99):
            lim = row[f"p{p}_h"]
            row[f"per_year_longer_than_p{p}"] = (float((x > lim).sum()) / point_years
                                                 if lim is not None and point_years else None)
            row[f"hour_weighted_p{p}_h"] = hour_weighted_pct(x, p)
        out[name] = row
    return out


def stats(series, P_avionics, motor_w, solar_day_wh) -> dict:
    """Climate statistics pooled over a list of prepared series (one point or a region)."""
    thr = thresholds()
    survival = record.value("ds.survival_hs_m")
    u = np.concatenate([s["u10"] for s in series])
    mon = np.concatenate([s["month"] for s in series])
    have_waves = all("hs" in s for s in series)
    if have_waves:
        hs = np.concatenate([s["hs"] for s in series])
        tp = np.concatenate([s["tp"] for s in series])
        sw = np.concatenate([s["swell"] for s in series])
    res = {"points": [s["id"] for s in series], "hours": int(u.size), "u10": {}, "below": {}, "spells": {},
           "spells_merged_3h": {}, "waves": {}, "weak_wind_waves": {}, "energy": {}}
    for name, ms in SEASONS.items():
        m = np.isin(mon, ms) & np.isfinite(u)
        x = u[m]
        res["u10"][name] = {"n": int(x.size), "mean": _r(_mean(x)), "std": _r(_std(x)),
                            **{f"p{p}": _r(_pct(x, p)) for p in (10, 50, 90)}}
        if have_waves:
            mh = m & np.isfinite(hs)
            res["waves"][name] = {"hs_mean": _r(_mean(hs[mh])), "hs_p50": _r(_pct(hs[mh], 50)),
                                  "hs_p90": _r(_pct(hs[mh], 90)),
                                  "hs_ge_survival_share": _share(int((hs[mh] >= survival).sum()), int(mh.sum())),
                                  "tp_p50": _r(_pct(tp[mh], 50))}
    for tn, tv in thr.items():
        res["below"][tn] = {}
        res["weak_wind_waves"][tn] = {}
        for name, ms in SEASONS.items():
            m = np.isin(mon, ms) & np.isfinite(u)
            weak = m & (u < tv)
            res["below"][tn][name] = _share(int(weak.sum()), int(m.sum()))
            if have_waves:
                wk = weak & np.isfinite(hs)
                n = int(wk.sum())
                row = {"weak_hours": n}
                for lv in HS_LEVELS_M:
                    row[f"hs_ge{lv:g}_share"] = _share(int((hs[wk] >= lv).sum()), n)
                    row[f"swell_ge{lv:g}_share"] = _share(int((sw[wk] >= lv).sum()), n)
                row["hs_ge_survival_share"] = _share(int((hs[wk] >= survival).sum()), n)
                big = wk & (sw >= HS_LEVELS_M[0])
                row["tp_p50_s_swell_ge2"] = _r(_pct(tp[big], 50))
                row["tp_p10_s_swell_ge2"] = _r(_pct(tp[big], 10))
                row["tp_p90_s_swell_ge2"] = _r(_pct(tp[big], 90))
                res["weak_wind_waves"][tn][name] = row
        res["spells"][tn] = spell_table(series, tv)
        res["spells_merged_3h"][tn] = spell_table(series, tv, merge_break=3)
    # energy to ride out spells of the given percentiles, on the motor or floating (no solar credit)
    for ac, pw in motor_w.items():
        res["energy"][ac] = {}
        for tn in thr:
            res["energy"][ac][tn] = {}
            for name in SEASONS:
                row = {}
                for p in SPELL_PCTS + ("max",):
                    h = res["spells"][tn][name][f"p{p}_h" if p != "max" else "max_h"]
                    if h is None:
                        continue
                    key = f"p{p}" if p != "max" else "max"
                    row[key] = energy_row(h, pw, P_avionics, solar_day_wh.get(ac))
                res["energy"][ac][tn][name] = row
    return res


def energy_row(hours, motor_w, avionics_w, solar_day_wh=None) -> dict:
    """Energy (Wh) and cell mass (kg) to motor through, or float through, a spell of the given hours."""
    wh_kg = record.value("lit.cell_p45b_wh_per_kg")
    motor = (motor_w + avionics_w) * hours
    flt = avionics_w * hours
    row = {"spell_h": _r(hours), "motor_wh": _r(motor), "float_wh": _r(flt),
           "motor_cell_kg": _r(motor / wh_kg), "float_cell_kg": _r(flt / wh_kg)}
    if solar_day_wh is not None:
        # darkest-month daily solar yield spread over the day; the night still has to be bridged
        row["float_net_of_solar_wh"] = _r(max(avionics_w - solar_day_wh / 24.0, 0.0) * hours)
    return row


def aircraft_inputs() -> dict:
    """Motor power below the soaring minimum, wing area, and the threshold each is paired with."""
    best_area = record.value("wing.best_span_m") ** 2 / record.value("wing.best_ar")
    return {
        "albatross": {"motor_w": record.value("motor.albatross_calm_w"),
                      "motor_key": "motor.albatross_calm_w",
                      "wing_area_m2": record.value("form.albatross_3m.wing_area_m2"),
                      "paired_threshold": "dsmin"},
        "best": {"motor_w": record.value("wing.front1_c9_w"), "motor_key": "wing.front1_c9_w",
                 "wing_area_m2": best_area, "paired_threshold": "best"},
    }


def solar_day_wh(area_m2, insolation_kwh_m2_day) -> float:
    """Daily solar yield (Wh) of the wing with the pattern study's cell assumptions (sim.patterns)."""
    from sim.patterns import SOLAR_CELL_EFF, SOLAR_COVERAGE
    return area_m2 * SOLAR_COVERAGE * SOLAR_CELL_EFF * insolation_kwh_m2_day * 1000.0


def analyse(cache, private_dir) -> dict:
    from sim.patterns import SOLAR_CELL_EFF, SOLAR_COVERAGE
    av = record.value("ds.avionics_power_w")
    acs = aircraft_inputs()
    motor_w = {k: v["motor_w"] for k, v in acs.items()}
    insol = {"so": record.value("lit.insolation_50s_june_kwh_m2_day"),
             "home": record.value("lit.insolation_home_december_kwh_m2_day")}
    solar = {r: {k: solar_day_wh(v["wing_area_m2"], ins) for k, v in acs.items()} for r, ins in insol.items()}
    pts = all_points(private_dir)
    prepared = {p["id"]: prepare(p, cache, private_dir) for p in pts}
    out = {"meta": record.meta("python -m sim.climate", MODEL_FILES),
           "period": {"start": START_DATE, "end": END_DATE, "step": "1 h, UTC"},
           "inputs": {"thresholds_ms": thresholds(), "threshold_keys": {
               "u6": "plain 6 m/s", "dsmin": "ds.min_u10_ms", "best": "wing.best_u10_min_ms",
               "skim": SKIM_KEY},
               "avionics_w": av, "aircraft": acs, "insolation_kwh_m2_day": insol,
               "solar_cell_eff": SOLAR_CELL_EFF, "solar_coverage": SOLAR_COVERAGE,
               "survival_hs_m": record.value("ds.survival_hs_m"),
               "cell_wh_per_kg": record.value("lit.cell_p45b_wh_per_kg")},
           "solar_day_wh": {r: {k: _r(v) for k, v in d.items()} for r, d in solar.items()},
           "avionics_day_wh": _r(av * 24.0),
           "sources": {}, "points": {}, "regions": {}}
    for p in pts:
        s = prepared[p["id"]]
        if s is None:
            continue
        st = stats([s], av, motor_w, solar.get(p["region"], {}))
        info = {"region": p["region"], "missing_hours": s["missing_hours"]}
        if not p["private"]:
            info.update(lat=p["lat"], lon=p["lon"], grid=s.get("grid"))
            out["sources"][p["id"]] = source_meta(p, cache, private_dir)
        out["points"][p["id"]] = dict(info, **st)
    for r in REGIONS:
        ser = [prepared[p["id"]] for p in pts if p["region"] == r and prepared[p["id"]] is not None]
        if ser:
            out["regions"][r] = stats(ser, av, motor_w, solar.get(r, {}))
    return out


# ---------------------------------------------------------------- command

def main(argv=None):
    ap = argparse.ArgumentParser(prog="python -m sim.climate", description=__doc__.split("\n")[0])
    ap.add_argument("command", nargs="?", default="analyse", choices=("analyse", "fetch"))
    ap.add_argument("--private", default=str(DEFAULT_PRIVATE), help="the owner's private store")
    ap.add_argument("--cache", default=str(DEFAULT_CACHE), help="cache of the public downloads")
    ap.add_argument("--pause", type=float, default=450.0, help="seconds between requests (fetch)")
    a = ap.parse_args(argv)
    if a.command == "fetch":
        fetch(a.cache, a.private, a.pause)
        return
    res = analyse(a.cache, a.private)
    OUT_PATH.parent.mkdir(parents=True, exist_ok=True)
    with open(OUT_PATH, "w", encoding="utf-8", newline="\n") as fh:
        json.dump(res, fh, indent=1)
        fh.write("\n")
    print(f"wrote {OUT_PATH.relative_to(ROOT).as_posix()}")


if __name__ == "__main__":
    main(sys.argv[1:])
