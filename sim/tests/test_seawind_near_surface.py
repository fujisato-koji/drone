"""Tests of the near-surface options of sim/seawind.py (Q-064, Q-061): all opt-in."""
import unittest

import numpy as np

from sim.seawind import (G, KARMAN, TURB_L, Environment, WaveTrain, Wind, _turb_amp, forristall_crest,
                         obukhov_length, psi_m, rayleigh_crest_stats, turbulence_length_scales,
                         whitecap_callaghan)


class Mono(WaveTrain):
    def components(self, rng):
        return {"a": np.array([1.0]), "k": np.array([0.1]), "omega": np.array([np.sqrt(G * 0.1)]),
                "dx": np.array([1.0]), "dy": np.array([0.0]), "phase": np.array([0.0])}


class Two(WaveTrain):
    """Two collinear components of different wavenumber."""

    def components(self, rng):
        k = np.array([0.08, 0.15])
        return {"a": np.array([0.8, 0.5]), "k": k, "omega": np.sqrt(G * k),
                "dx": np.array([1.0, 1.0]), "dy": np.array([0.0, 0.0]),
                "phase": np.array([0.3, 1.1])}


class Oblique(WaveTrain):
    def components(self, rng):
        k = np.array([0.08, 0.1])
        return {"a": np.array([0.8, 0.6]), "k": k, "omega": np.sqrt(G * k),
                "dx": np.array([1.0, np.cos(1.0)]), "dy": np.array([0.0, np.sin(1.0)]),
                "phase": np.array([0.3, 1.1])}


def sea(u10=12.0, **kw):
    w = Wind(u10, 270.0)
    return w, Environment(w, [WaveTrain.fully_developed(w)], seed=2, **kw)


class TestSurfaceModel(unittest.TestCase):
    def test_defaults_are_off(self):
        w, env = sea()
        self.assertEqual(env.surface_model, "linear")
        self.assertEqual(env.breaking_mode, "none")
        self.assertEqual(env.turbulence_profile, "fixed")
        self.assertIsNone(w.stability_L)
        self.assertNotIn("options", env.to_dict()["model"])

    def test_options_in_dict_when_on(self):
        _, env = sea(surface_model="stokes2", interaction_n=8, turbulence_profile="height")
        o = env.to_dict()["model"]["options"]
        self.assertEqual(o["surface_model"], "stokes2")
        self.assertEqual(o["turbulence_profile"], "height")
        w = Wind(9.0, profile="swell_jet", cp=40.0)
        o = Environment(w, []).to_dict()["model"]["options"]
        self.assertAlmostEqual(o["swell_jet"]["max_ratio"], 1.1)
        o = Environment(Wind(9.0, stability_L=50.0), []).to_dict()["model"]["options"]
        self.assertEqual(o["stability_L"], 50.0)

    def test_bad_options(self):
        w = Wind(9.0)
        for kw in ({"surface_model": "x"}, {"breaking": "x"}, {"turbulence_profile": "x"}, {"swell_profile": "x"}):
            with self.assertRaises(ValueError):
                Environment(w, [], **kw)

    def test_single_component_equals_stokes_flag(self):
        class One(Two):
            def components(self, rng):
                c = Two.components(self, rng)
                return {k: v[:1] for k, v in c.items()}
        w = Wind(10.0)
        a = Environment(w, [One(1.0, 8.0, 90.0)], stokes=True).surface(np.array([3.0, 40.0]), 0.0, 1.0)
        b = Environment(w, [One(1.0, 8.0, 90.0)], surface_model="stokes2").surface(np.array([3.0, 40.0]), 0.0, 1.0)
        for u, v in zip(a, b):
            np.testing.assert_allclose(u, v, atol=1e-12)

    def test_collinear_pair_coefficient(self):
        w = Wind(10.0)
        tr = Two(1.0, 8.0, 90.0)
        lin = Environment(w, [tr])
        s2 = Environment(w, [tr], surface_model="stokes2")
        x = np.linspace(0.0, 300.0, 50)
        c = lin.c
        d = s2.surface(x, 0.0 * x, 0.0 * x)[0] - lin.surface(x, 0.0 * x, 0.0 * x)[0]
        th = c["k"][:, None] * x[None, :] + c["phase"][:, None]
        expect = (0.5 * c["a"][:, None] ** 2 * c["k"][:, None] * np.cos(2 * th)).sum(0)
        expect += 0.5 * c["a"][0] * c["a"][1] * (c["k"][0] + c["k"][1]) * np.cos(th[0] + th[1])
        np.testing.assert_allclose(d, expect, atol=1e-12)

    def test_slopes_match_finite_difference(self):
        w = Wind(10.0)
        for tr in (Oblique(1.0, 8.0, 90.0), WaveTrain(2.0, 8.0, 80.0, n_omega=4, n_theta=4)):
            env = Environment(w, [tr], surface_model="stokes2", seed=4)
            x = np.array([13.0, 40.0, -7.0])
            y = np.array([7.0, -3.0, 21.0])
            t = np.array([5.0, 5.0, 9.0])
            _, sx, sy = env.surface(x, y, t)
            h = 1e-5
            fx = (env.surface(x + h, y, t)[0] - env.surface(x - h, y, t)[0]) / (2 * h)
            fy = (env.surface(x, y + h, t)[0] - env.surface(x, y - h, t)[0]) / (2 * h)
            np.testing.assert_allclose(sx, fx, atol=1e-7)
            np.testing.assert_allclose(sy, fy, atol=1e-7)

    def test_height_above_surface_uses_second_order(self):
        w = Wind(10.0)
        lin = Environment(w, [Two(1.0, 8.0, 90.0)])
        s2 = Environment(w, [Two(1.0, 8.0, 90.0)], surface_model="stokes2")
        x = np.linspace(0.0, 300.0, 40)
        e1 = lin.surface(x, 0 * x, 0 * x)[0]
        e2 = s2.surface(x, 0 * x, 0 * x)[0]
        self.assertGreater(np.abs(e2 - e1).max(), 0.01)
        # the wind at 2 m above the linear crest differs because zloc = z - eta changes
        a = lin.wind_at(x, 0 * x, np.full_like(x, 3.0), 0 * x)
        b = s2.wind_at(x, 0 * x, np.full_like(x, 3.0), 0 * x)
        self.assertGreater(np.abs(a - b).max(), 1e-3)

    def test_crest_stats_rayleigh_and_sharpening(self):
        w = Wind(12.0, 270.0)
        tr = WaveTrain.fully_developed(w, n_omega=10, n_theta=4)
        lin = Environment(w, [tr], seed=2).crest_stats((0, 100, 0, 100), 400.0, spacing=50.0, dt=0.5)
        s2 = Environment(w, [tr], seed=2, surface_model="stokes2", interaction_n=40).crest_stats(
            (0, 100, 0, 100), 400.0, spacing=50.0, dt=0.5)
        self.assertGreater(lin["n"], 500)
        self.assertLess(abs(lin["h13"] / lin["rayleigh"]["h13"] - 1.0), 0.12)
        self.assertGreater(s2["h13"], lin["h13"])
        self.assertGreater(s2["hmax"], lin["hmax"] * 0.98)
        self.assertEqual(lin["heights"].size, lin["n"])
        self.assertTrue(np.all(np.diff(lin["heights"]) >= 0))

    def test_rayleigh_values(self):
        r = rayleigh_crest_stats(1.0, 1000)
        self.assertAlmostEqual(r["h13"], 2.0, delta=0.05)   # Hs/2 = 2 sigma
        self.assertAlmostEqual(r["h110"], 2.54, delta=0.05)
        self.assertGreater(r["hmax"], r["h110"])


class TestBreaking(unittest.TestCase):
    def test_off_by_default(self):
        _, a = sea()
        self.assertFalse(a.breaking_mask(np.arange(5.0), 0.0, 0.0).any())
        self.assertIsNone(a.breaking_stats())

    def test_banner_probability(self):
        _, e = sea(10.0, breaking="stats")
        st = e.breaking_stats()
        self.assertAlmostEqual(st["eps_p"], st["hp"] * st["kp"] / 2.0)
        self.assertAlmostEqual(st["pb"], 22.0 * (st["eps_p"] - 0.055) ** 2.01)
        self.assertGreater(st["pb"], 0.0)
        self.assertAlmostEqual(st["p_slot"], st["pb"] * 2.0 / st["tp"])
        self.assertAlmostEqual(st["lc"], 0.25 * st["lam"])

    def test_mask_is_stateless_and_rare(self):
        _, e = sea(10.0, breaking="stats")
        rng = np.random.default_rng(1)
        x, y, t = rng.uniform(0, 2000, 40000), rng.uniform(0, 2000, 40000), rng.uniform(0, 3600, 40000)
        m1 = e.breaking_mask(x, y, t)
        np.testing.assert_array_equal(m1, e.breaking_mask(x, y, t))
        self.assertLess(m1.mean(), 0.01)

    def forced(self, p=0.5, **kw):
        _, e = sea(10.0, breaking="stats", **kw)
        e._brk["p_slot"] = p
        return e

    def test_forced_events_lift_crests_only_up(self):
        e = self.forced()
        _, ref = sea(10.0)
        x = np.linspace(0, 400, 2000)
        t = np.full_like(x, 3.0)
        m = e.breaking_mask(x, 0 * x, t)
        self.assertTrue(0.02 < m.mean() < 0.25)
        d = e.surface(x, 0 * x, t)[0] - ref.surface(x, 0 * x, t)[0]
        self.assertTrue(np.all(d >= 0.0))
        self.assertTrue(np.all(d[~m] == 0.0))
        self.assertGreater(d.max(), 0.0)
        self.assertLess(d.max(), 0.443 / e._brk["kp"] * 0.3)
        self.assertLess(d.max(), 0.1 * 0.443 / e._brk["kp"] * 3)

    def test_lift_peaks_at_crest_top(self):
        e = self.forced(p=1.0)
        _, ref = sea(10.0)
        x = np.linspace(0, 600, 6001)
        t = np.full_like(x, 3.0)
        m = e.breaking_mask(x, 0 * x, t)
        d = e.surface(x, 0 * x, t)[0] - ref.surface(x, 0 * x, t)[0]
        self.assertTrue(m.any())
        lab = np.cumsum(np.r_[1, np.diff(m.astype(int)) == 1])
        checked = 0
        for i in np.unique(lab[m]):
            idx = np.flatnonzero(m & (lab == i))
            if idx.size < 20 or idx[0] == 0 or idx[-1] == x.size - 1:
                continue
            seg = slice(idx[0], idx[-1] + 1)
            xmax = x[seg][np.argmax(d[seg])]
            ev = e._breaking_events(x[idx], 0 * x[idx], t[idx])
            xcrest = np.median(x[idx] + ev["sc"])
            self.assertLess(abs(xmax - xcrest), 0.1 * e._brk["lc"])
            etot = e.surface(x[seg], 0 * x[seg], t[seg])[0]
            self.assertLess(abs(x[seg][np.argmax(etot)] - xcrest), 0.15 * e._brk["lc"])
            checked += 1
        self.assertGreater(checked, 0)

    def test_lee_shelter_removes_wind(self):
        e = self.forced(shelter=True)
        w = e.wind
        x = np.linspace(0, 400, 2000)
        t = np.full_like(x, 3.0)
        lee = e._breaking_lee(x, 0 * x, t) > 0
        self.assertTrue(lee.any())
        eta = e.surface(x, 0 * x, t)[0]
        z = eta + 0.5
        a = e.wind_at(x, 0 * x, z, t)
        _, plain = sea(10.0, shelter=True)
        b = plain.wind_at(x, 0 * x, z, t)
        al = lambda v: v[..., 0] * w.ex + v[..., 1] * w.ey  # noqa: E731
        self.assertTrue(np.all(al(a)[lee] <= al(b)[lee] + 1e-9))
        self.assertLess(al(a)[lee].mean(), 0.5 * al(b)[lee].mean())
        far = ~lee & ~e.breaking_mask(x, 0 * x, t)
        np.testing.assert_allclose(a[far], b[far], atol=1e-12)

    def test_whitecap_and_forristall(self):
        self.assertAlmostEqual(float(whitecap_callaghan(10.0)), 3.18e-3 * 6.3**3 / 100.0)
        self.assertAlmostEqual(float(whitecap_callaghan(10.18)), float(whitecap_callaghan(10.1800001)), delta=5e-5)
        self.assertEqual(float(whitecap_callaghan(3.0)), 0.0)
        # small steepness: alpha 0.3536, beta 2 (Hs / (2 sqrt 2) sqrt(-ln p))
        self.assertAlmostEqual(forristall_crest(1.0, 1e3, 1e-3), 0.3536 * np.sqrt(-np.log(1e-3)), places=3)
        self.assertGreater(forristall_crest(4.0, 8.0, 1e-3), forristall_crest(4.0, 8.0, 1e-2))
        self.assertGreater(forristall_crest(4.0, 8.0, 1e-3), 0.3536 * 4.0 * np.sqrt(-np.log(1e-3)))


class TestWaveWind(unittest.TestCase):
    def mono(self, **kw):
        w = Wind(12.0, 270.0)
        return w, Environment(w, [Mono(1.0, 8.0, 90.0)], **kw)

    def test_off_is_default_and_floor_on_changes(self):
        w, a = self.mono()
        _, b = self.mono(critical_floor=None)
        z = np.array([1.0, 3.0])
        np.testing.assert_array_equal(a.wind_at(0.0, 0.0, z, 0.0), b.wind_at(0.0, 0.0, z, 0.0))
        self.assertIsNone(a.critical_floor)
        _, c = self.mono(critical_floor=True)
        self.assertEqual(c.critical_floor, 0.1)

    def test_sign_change_and_floor_at_critical_height(self):
        w, e = self.mono(critical_floor=0.1)
        cph = np.sqrt(G / 0.1)
        zs = np.linspace(0.1, 40, 4000)
        zc = zs[np.argmin(np.abs(w.speed(zs) - cph))]

        def u_pert(z):
            zl = np.array([z])
            full = e.wind_at(0.0, 0.0, zl + 1.0, 0.0)[0, 0]   # eta = a = 1 at x = 0, t = 0
            return full - float(w.speed(zl)[0])
        self.assertLess(u_pert(0.5 * zc), 0.0)
        self.assertGreater(u_pert(2.0 * zc), 0.0)
        # at the critical height the amplitude sits at the floor: A a k c f exp(-k z)
        self.assertAlmostEqual(abs(u_pert(zc)), cph * 0.1 * 0.1 * np.exp(-0.1 * zc), delta=0.02)

    def test_p63_preset(self):
        _, e = self.mono(critical_floor=True, wave_wind="p63")
        self.assertEqual(e._amp, (0.95, 1.07))
        self.assertEqual(e.decay, (0.77, 0.62))
        o = e.to_dict()["model"]["options"]
        self.assertEqual(o["wave_wind"]["preset"], "p63")
        with self.assertRaises(ValueError):
            self.mono(wave_wind="x")


class TestTurbulenceProfile(unittest.TestCase):
    def test_length_scale_laws(self):
        lu, lv, lw = turbulence_length_scales(np.array([1.0, 10.0, 200.0]))
        np.testing.assert_allclose(lw, [0.58, 5.8, TURB_L])
        np.testing.assert_allclose(lv, lw)
        np.testing.assert_allclose(lu, [2.4, 24.0, TURB_L])

    def test_fixed_unchanged_and_height_equal_aloft(self):
        w = Wind(10.0, 270.0)
        xs = np.linspace(0, 3000, 500)
        z = np.full_like(xs, 120.0)
        f = Environment(w, [], seed=1, turbulence="iso").gust(xs, 0 * xs, z, 0 * xs)
        h = Environment(w, [], seed=1, turbulence="iso", turbulence_profile="height").gust(xs, 0 * xs, z, 0 * xs)
        np.testing.assert_allclose(f, h, rtol=1e-9, atol=1e-12)

    def test_variance_kept_and_scale_shrinks(self):
        xs = np.arange(2**15) * 0.5
        sd_f, sd_h = [], []
        for z in (1.0, 5.0):
            a, b = [], []
            for seed in range(1, 7):
                w = Wind(10.0, 270.0)
                g = Environment(w, [], seed=seed, turbulence="iso", turbulence_profile="height").gust(
                    xs, 0 * xs, np.full_like(xs, z), 0 * xs)
                f = Environment(w, [], seed=seed, turbulence="iso").gust(
                    xs, 0 * xs, np.full_like(xs, z), 0 * xs)
                a.append(g[:, 2].std())
                b.append(f[:, 2].std())
            sd_h.append(np.mean(a))
            sd_f.append(np.mean(b))
        np.testing.assert_allclose(sd_h, sd_f, rtol=0.15)
        # integral scale of the mode set: <1/k> weighted by the mode energy
        e = Environment(Wind(10.0, 270.0), [], seed=1, turbulence="iso")
        k = e._turb["k"]

        def lam(length):
            a2 = _turb_amp(k, np.array([length]))[0] ** 2
            return (a2 / k).sum() / a2.sum()
        self.assertLess(lam(1.0), 0.2 * lam(TURB_L))
        self.assertLess(lam(1.0), lam(5.0))
        self.assertAlmostEqual(lam(TURB_L), float(_turb_amp(k, np.array([TURB_L]))[0].dot(
            _turb_amp(k, np.array([TURB_L]))[0] / k) / (_turb_amp(k, np.array([TURB_L]))[0] ** 2).sum()))


class TestStability(unittest.TestCase):
    @staticmethod
    def shear(L, u10=9.0):
        w = Wind(u10, stability_L=L)
        return float(w.speed(15.0) - w.speed(1.0)) / u10, w

    def test_neutral_unchanged(self):
        a, w = self.shear(None)
        b = Wind(9.0)
        self.assertEqual(float(w.speed(3.0)), float(b.speed(3.0)))
        self.assertEqual(w.ustar, b.ustar)
        # a very long Obukhov length tends to neutral
        c, _ = self.shear(1e7)
        self.assertAlmostEqual(a, c, places=4)

    def test_u10_kept(self):
        for L in (50.0, 200.0, -50.0, -200.0):
            _, w = self.shear(L)
            self.assertAlmostEqual(float(w.speed(10.0)), 9.0, places=9)

    def test_stable_adds_shear_unstable_reduces(self):
        n, _ = self.shear(None)
        s50, _ = self.shear(50.0)
        s200, _ = self.shear(200.0)
        u50, _ = self.shear(-50.0)
        u200, _ = self.shear(-200.0)
        self.assertTrue(1.3 < s50 / n < 1.5, s50 / n)    # a reader found 0.33 against 0.235 of U10
        self.assertTrue(1.0 < s200 / n < s50 / n)
        self.assertTrue(u200 < n and u50 < u200)

    def test_functions(self):
        self.assertAlmostEqual(float(psi_m(0.2)), -1.0)
        self.assertAlmostEqual(float(psi_m(-1e-9)), 0.0, places=6)
        self.assertGreater(float(psi_m(-0.5)), 0.0)

    def test_knee_with_stability_is_flat_above(self):
        w = Wind(9.0, profile="swell_knee", stability_L=100.0)
        self.assertEqual(float(w.speed(8.0)), float(w.speed(20.0)))
        self.assertAlmostEqual(float(w.speed(10.0)), 9.0, places=9)

    def test_obukhov_length(self):
        ls = obukhov_length(8.0, 2.0)
        lu = obukhov_length(8.0, -2.0)
        self.assertTrue(10.0 < ls < 500.0, ls)
        self.assertTrue(-500.0 < lu < -10.0, lu)
        self.assertGreater(obukhov_length(12.0, 2.0), ls)      # more wind, less stable
        self.assertGreater(ls, obukhov_length(8.0, 4.0))        # colder sea, more stable
        self.assertEqual(obukhov_length(8.0, 0.0), float("inf"))
        self.assertGreater(obukhov_length(8.0, 2.0, z0t=1e-3), 0.0)

    def test_stability_with_jet_rejected(self):
        with self.assertRaises(ValueError):
            Wind(9.0, profile="swell_jet", stability_L=50.0)
        with self.assertRaises(ValueError):
            Environment(Wind(9.0, stability_L=50.0), [], swell_profile="jet")


class TestSwellJet(unittest.TestCase):
    def test_calibration(self):
        w = Wind(9.0, profile="swell_jet")
        u = lambda z: float(w.speed(z))  # noqa: E731
        self.assertAlmostEqual((u(8.0) - u(2.5)) / u(8.0), 0.25, places=9)
        self.assertAlmostEqual(u(10.0), 9.0, places=9)
        self.assertAlmostEqual(u(8.0), u(15.0), places=9)   # no cp: nearly constant above
        # far more shear than the neutral log between 2.5 and 8 m (0.09 of U8)
        log = Wind(9.0)
        self.assertGreater((u(8.0) - u(2.5)) / u(8.0), 2.0 * float(log.speed(8.0) - log.speed(2.5)) / float(log.speed(8.0)))
        # dimensionless shear well above 1 at 2.5 and 5 m with the neutral u*
        for z in (2.5, 5.0):
            dz = 1e-4
            phi = KARMAN * z / w.ustar * (u(z + dz) - u(z - dz)) / (2 * dz)
            self.assertGreater(phi, 1.5)

    def test_wind_maximum_by_wave_age(self):
        for cp, ratio in ((9.0 * 1.7, 1.0), (9.0 * 4.0, 1.1), (9.0 * 8.0, 1.1), (9.0 * 2.85, 1.05)):
            w = Wind(9.0, profile="swell_jet", cp=cp)
            self.assertAlmostEqual(w.jet_ratio, ratio, places=9)
            self.assertAlmostEqual(float(w.speed(9.0 * 0 + 8.0) / w.speed(16.0)), ratio, places=9)
            self.assertAlmostEqual(float(w.speed(10.0)), 9.0, places=9)

    def test_record_keys_and_validation(self):
        w = Wind(9.0, profile="swell_jet", jet_shear_frac=0.4, jet_height_m=6.0)
        self.assertAlmostEqual(float(w.speed(6.0) - w.speed(2.5)) / float(w.speed(6.0)), 0.4, places=9)
        with self.assertRaises(ValueError):
            Wind(9.0, profile="swell_jet", jet_shear_frac=1.2)

    def test_auto_blend_with_jet(self):
        w = Wind(8.0, 270.0)
        swell = WaveTrain(1.5, 16.0, 90.0, gamma=7.0, name="swell")   # cp about 25 m/s, A about 0.32
        env = Environment(w, [swell], profile_mode="auto", swell_profile="jet")
        f = env.wave_age()[2]
        self.assertGreater(f, 0.5)
        z = np.array([1.5, 2.5, 5.0, 8.0])
        j = env._w_swell.speed(z)
        lg = Wind(8.0).speed(z)
        np.testing.assert_allclose(env._mean_speed(z), (1 - f) * lg + f * j)
        self.assertGreater(env._mean_speed(np.array([8.0]))[0] - env._mean_speed(np.array([2.5]))[0],
                           float(Wind(8.0, profile="swell_knee").speed(8.0) - Wind(8.0, profile="swell_knee").speed(2.5)))
        # the knee is the default
        k = Environment(w, [swell], profile_mode="auto")
        self.assertEqual(k._w_swell.profile, "swell_knee")


if __name__ == "__main__":
    unittest.main()
