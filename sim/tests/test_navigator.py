import math

import numpy as np

from sim.navigator import Navigator, Polar, sail, wrap180

K = 0.5   # V = K * u10 * shape


def make_polar(downwind="gybe", nogo=40.0):
    """Synthetic yacht polar. chi 0 downwind .. 180 upwind. No progress within nogo of the wind.
    Upwind and reaching: V = K u sin(a), a = 180 - chi (best VMG at 45 deg off the wind).
    Downwind (chi < 90): "gybe" = K u (1 + 0.5 sin 2chi), best VMG off the axis;
    "direct" = K u, best straight downwind."""
    winds = [4.0, 8.0, 12.0]
    chis = np.arange(0.0, 180.01, 1.0)
    tab = np.full((len(winds), len(chis)), np.nan)
    for i, u in enumerate(winds):
        for j, chi in enumerate(chis):
            a = 180.0 - chi
            if a < nogo:
                continue
            if chi >= 90:
                tab[i, j] = K * u * math.sin(math.radians(a))
            elif downwind == "gybe":
                tab[i, j] = K * u * (1 + 0.5 * math.sin(math.radians(2 * chi)))
            else:
                tab[i, j] = K * u
    return Polar(winds, chis, tab)


def steady(u=8.0, wf=0.0):
    return lambda t: (u, wf)


def run(polar, target, wind=None, **kw):
    kw.setdefault("tack_cost_s", 30.0)
    return sail(polar, wind or steady(), (0.0, 0.0), target, dt=1.0, **kw)


def test_polar_interpolation_and_mirror():
    p = make_polar()
    assert np.isnan(p.speed(150.0, 8.0))          # inside 40 deg of the wind
    assert p.speed(135.0, 8.0) == p.speed(-135.0, 8.0)
    assert abs(p.speed(135.0, 6.0) - K * 6 * math.sin(math.radians(45))) < 1e-9
    assert np.isnan(p.speed(90.0, 20.0))          # outside the wind range
    ang, vmg = p.upwind_angle(8.0)
    assert 43 <= ang <= 47 and abs(vmg - K * 8 * 0.5) < 0.02
    ang, _ = make_polar("gybe").downwind_angle(8.0)
    assert 10 < ang < 40
    assert make_polar("direct").downwind_angle(8.0)[0] == 0.0


def test_upwind_zigzag_and_tack_cost():
    p = make_polar()
    D = 10000.0
    ideal = D / (K * 8 * 0.5)               # distance / best VMG = 5000 s
    res = {}
    for cost in (0.0, 30.0, 120.0):
        r = run(p, (0.0, D), tack_cost_s=cost)
        assert r["arrived"]
        res[cost] = r
        # legs sit at the tacking angle: 45 deg off the wind (wind from north)
        tr = r["track"]
        seg = tr[10::10] - tr[:-10:10]
        moving = np.hypot(seg[:, 0], seg[:, 1]) > 1.0
        brg = np.degrees(np.arctan2(seg[:, 0], seg[:, 1]))[moving]
        off = np.abs(wrap180(brg))[: int(0.6 * len(brg))]   # before the layline/end game
        assert np.mean((off > 39.0) & (off < 52.0)) > 0.75   # end game bends legs
        assert r["time"] < 1.08 * ideal + r["n_tacks"] * cost
        assert r["time"] > ideal - 50.0 / 2.0 + r["n_tacks"] * cost * 0.99   # arrival radius 50 m
    assert res[0.0]["n_tacks"] > res[30.0]["n_tacks"] > res[120.0]["n_tacks"] >= 1
    print("upwind", {c: (r["n_tacks"], round(r["time"]), round(r["mean_vmg"], 3))
                     for c, r in res.items()}, "ideal", ideal)


def test_downwind_gybe_vs_direct():
    D = 10000.0
    r = run(make_polar("gybe"), (0.0, -D))
    assert r["arrived"] and r["n_tacks"] >= 1
    assert any(m == "downwind gybing" for _, m in r["modes"])
    r2 = run(make_polar("direct"), (0.0, -D))
    assert r2["arrived"] and r2["n_tacks"] == 0
    assert {m for _, m in r2["modes"]} <= {"direct", "arrived"}
    assert abs(r2["time"] - D / (K * 8)) < 30
    assert make_polar("gybe").downwind_angle(8.0)[1] > K * 8


def test_reach_goes_direct():
    # wind from north, target east: beam reach (the polar with a flat downwind V; with the gybing
    # polar a broad-reach zigzag legitimately beats the beam reach), and a broad reach 60 deg off downwind
    p = make_polar("direct")
    r = run(p, (10000.0, 0.0))
    assert r["n_tacks"] == 0
    assert {m for _, m in r["modes"]} <= {"direct", "arrived"}
    assert abs(r["time"] - 10000.0 / (K * 8)) < 30
    tgt = (10000.0 * math.sin(math.radians(120)), 10000.0 * math.cos(math.radians(120)))
    r = run(p, tgt)    # broad reach, 60 deg off the downwind axis, flat V: straight is best
    assert r["n_tacks"] == 0 and {m for _, m in r["modes"]} <= {"direct", "arrived"}


def test_oscillating_wind_header_tacking():
    p = make_polar()

    def wind(t):
        return 8.0, 15.0 * math.sin(2 * math.pi * t / 1200.0)

    D = 10000.0
    a = run(p, (0.0, D), wind=wind, tack_cost_s=20.0, shift_tacking=True, hysteresis=0.03)
    b = run(p, (0.0, D), wind=wind, tack_cost_s=20.0, shift_tacking=False)
    assert a["arrived"] and b["arrived"]
    print("oscillating: shifts", a["n_tacks"], round(a["time"]), "layline-only", b["n_tacks"],
          round(b["time"]))
    assert a["time"] < b["time"]


def test_no_progress():
    # no cycle beyond 90 deg off the downwind axis: no way to go upwind
    winds, chis = [4.0, 12.0], np.arange(0.0, 180.1, 5.0)
    tab = np.where(chis < 90, 3.0, np.nan)[None, :].repeat(2, 0)
    p = Polar(winds, chis, tab)
    r = run(p, (0.0, 5000.0), t_max=60.0)
    assert r["modes"][0][1] == "no progress" and not r["arrived"]
    nav = Navigator(p)
    assert nav.step(0.0, (0, 0), 8.0, 0.0, (0, 5000.0)).mode == "no progress"
    assert nav.step(0.0, (0, 0), 8.0, 0.0, (0, -5000.0)).mode == "direct"


def test_determinism_and_no_chattering():
    p = make_polar()
    for tgt in ((0.0, 10000.0), (1500.0, 9000.0), (-3000.0, 6000.0)):
        r1, r2 = run(p, tgt), run(p, tgt)
        assert np.array_equal(r1["track"], r2["track"]) and r1["n_tacks"] == r2["n_tacks"]
        tt = np.diff(r1["tack_times"])
        assert r1["arrived"]
        # voluntary tacks respect the min leg (90 s); a forced tack near the target may come
        # sooner, but never call by call
        assert len(tt) == 0 or tt.min() >= 45.0
    # target slightly off the axis: near the layline it must settle, not flip each call
    r = run(p, (4000.0, 6000.0), tack_cost_s=30.0)
    assert r["arrived"] and r["n_tacks"] <= 8
    assert np.all(np.diff(r["tack_times"]) >= 45.0)
