"""Frozen turbulence modes from the seeded generator of the viewer (sim/viewer/gust.js).

sim.seawind.Environment draws its modes from numpy's generator, which JavaScript does not have.
The viewer draws them from mulberry32 instead. This module makes the same modes in Python, so one
seed gives one turbulence field in both: put them into an environment with

    env = Environment(wind, [], turbulence="iso", turbulence_profile="height")
    env._turb = gustmodes.modes(seed)

and env.gust(...) then equals Gust.create(seed).at(...) of gust.js (test_gust_js.py).
"""
import numpy as np

from .seawind import TURB_K_RANGE, TURB_L, TURB_N_MODES

_M = 0xFFFFFFFF


def _imul(a, b):
    return (a * b) & _M


def mulberry32(seed):
    """Uniform [0, 1) numbers, the same sequence as rng() of gust.js and seagen.js."""
    t = int(seed) & _M

    def nxt():
        nonlocal t
        t = (t + 0x6D2B79F5) & _M
        r = _imul(t ^ (t >> 15), 1 | t)
        r = ((r + _imul(r ^ (r >> 7), 61 | r)) & _M) ^ r
        return ((r ^ (r >> 14)) & _M) / 4294967296.0

    return nxt


def modes(seed, n=TURB_N_MODES, k_range=TURB_K_RANGE):
    """The dictionary Environment._make_turbulence returns (kx, ky, amp, k, phase), from mulberry32.

    Order of the draws: n log-uniform wavenumbers, n angles, then three rows of n phases.
    """
    rand = mulberry32(seed)
    lo, hi = np.log(k_range[0]), np.log(k_range[1])
    k = np.exp(np.array([lo + (hi - lo) * rand() for _ in range(n)]))
    ang = np.array([2.0 * np.pi * rand() for _ in range(n)])
    phase = np.array([[2.0 * np.pi * rand() for _ in range(n)] for _ in range(3)])
    kl = k * TURB_L
    amp2 = kl**4 / (1.0 + kl**2) ** (17.0 / 6.0) * k
    amp = np.sqrt(2.0 * amp2 / amp2.sum())
    return {"kx": k * np.sin(ang), "ky": k * np.cos(ang), "amp": amp, "k": k, "phase": phase}
