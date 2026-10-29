"""Sea state that evolves with the wind history.

The scheme is a stand-in for a spectral wave model.

Changes:
- 2026-10-29 (check K-005): wind sea grows by the equivalent-duration method on
  WaveTrain.duration_limited (CEM 2002 forms), replacing the 3 h relaxation to
  the fully developed state.
- 2026-10-29 (check K-005): swell decays on its energy (Hs^2) with e-folding time
  env.sea_decay_tau_h = 12 h, and not in the step in which it is created.
- 2026-10-29 (check K-005): cold start is a calm sea with Tp from the duration law.

Scheme, in steps of dt = 0.25 h from t = 0 (cold start: no sea at t = 0):
- Wind sea: keeps an equivalent duration t_eq. At each step, under the U10 at
  the end of the step, t_eq is set to the duration that gives the current Hs
  under that U10 (inverse of the Hs law), dt is added, and Hs and Tp are taken
  from duration_limited(U10, t_eq). The direction is the current wind direction.
- Sea above what the new wind can sustain (Hs above the CEM limit for the new
  U10, i.e. the wind dropped): the sea is set to the limit state and the energy
  excess (Hs^2 difference) moves to the swell. The swell keeps the Tp and the
  direction the sea had (the last growing direction); when merging with an
  existing swell, Hs adds in quadrature and Tp and direction come from the
  component with more energy. Swell energy (Hs^2) decays as exp(-t / tau) with
  tau = env.sea_decay_tau_h = 12 h; the decay is applied to the existing swell
  before the new energy is added, so new swell does not decay in its own step.
- Wave state at an hour is taken from the last step node at or before it;
  the wind itself is interpolated exactly.
"""

from __future__ import annotations

import numpy as np

from sim.seawind import Environment, WaveTrain, Wind, cem_limits

DT = 0.25
TAU_DECAY = 12.0  # env.sea_decay_tau_h, on Hs^2
SWELL_MIN_HS = 0.2


class WindHistory:
    """Series of (t_hours, u10, from_deg) with linear interpolation.

    Direction is interpolated along the unwrapped angle; values are held
    constant outside the time range.
    """

    def __init__(self, points):
        pts = sorted((float(t), float(u), float(d)) for t, u, d in points)
        if not pts:
            raise ValueError("empty history")
        self.t = np.array([p[0] for p in pts])
        self.u = np.array([p[1] for p in pts])
        self._rad = np.unwrap(np.radians([p[2] for p in pts]))

    def at(self, hour):
        u = float(np.interp(hour, self.t, self.u))
        d = float(np.degrees(np.interp(hour, self.t, self._rad)) % 360.0)
        return u, d


class SeaHistory:
    def __init__(self, history: WindHistory, tau_decay=TAU_DECAY, dt=DT):
        self.h = history
        self.tau_decay = tau_decay
        self.dt = dt
        # node state: (hs_sea, tp_sea, dir_sea, hs_swell, tp_swell, dir_swell)
        u, d = history.at(0.0)
        ws = Wind(u, d)
        tp0 = WaveTrain.duration_limited(ws, dt).tp  # Tp of the duration law after one step
        self._nodes = [(0.0, tp0, ws.to_deg, 0.0, tp0, ws.to_deg)]
        self.t_eq = [0.0]  # equivalent duration (h) of the wind sea at each node

    def _step(self, state, hour):
        hs, tp, dr, ss, st, sd = state
        u, d = self.h.at(hour + self.dt)
        w = Wind(u, d)
        _, hs_cap, _ = cem_limits(u)
        # Existing swell decays first, so swell created in this step is not decayed.
        ss2 = ss**2 * float(np.exp(-self.dt / self.tau_decay))
        if hs <= hs_cap:
            teq = WaveTrain.equivalent_hours(u, hs) + self.dt
            new = WaveTrain.duration_limited(w, teq)
            hs, tp, dr = new.hs, new.tp, new.to_deg
        else:
            e_new = hs**2 - hs_cap**2
            if e_new > ss2:
                st, sd = tp, dr
            ss2 += e_new
            new = WaveTrain.duration_limited(w, 1e6)  # at the limit
            hs, tp, dr = new.hs, new.tp, new.to_deg
            teq = 1e6
        self._teq_last = teq
        return (float(hs), float(tp), float(dr), float(np.sqrt(ss2)), st, sd)

    def _state(self, hour):
        i = max(0, int(np.floor(hour / self.dt + 1e-9)))
        while len(self._nodes) <= i:
            n = len(self._nodes) - 1
            self._nodes.append(self._step(self._nodes[-1], n * self.dt))
            self.t_eq.append(self._teq_last)
        return self._nodes[i]

    def environment(self, hour, seed=1, roughness="charnock") -> Environment:
        """Environment at `hour`. With roughness other than "charnock" the
        wind-sea Hs and Tp at that hour are passed to Wind (needs Hs > 0.01)."""
        u, d = self.h.at(hour)
        hs, tp, dr, ss, st, sd = self._state(hour)
        if roughness == "charnock":
            wind = Wind(u, d)
        else:
            wind = Wind(u, d, roughness=roughness, sea=(max(hs, 0.01), tp))
        trains = []
        if hs > 0.01:
            trains.append(WaveTrain(hs=hs, tp=tp, to_deg=wind.to_deg))
        if ss > SWELL_MIN_HS:
            trains.append(WaveTrain(hs=ss, tp=st, to_deg=sd, gamma=7.0, name="swell"))
        return Environment(wind, trains, seed=seed)

    def table(self, hours):
        rows = []
        for hr in hours:
            u, _ = self.h.at(hr)
            hs, tp, _, ss, st, _ = self._state(hr)
            rows.append((float(hr), u, hs, tp, ss, st))
        return rows
