"""Minimal stand-in for sim.seawind: log wind profile, no waves."""
import numpy as np

RHO_AIR = 1.225
KAPPA = 0.4


class Wind:
    def __init__(self, u10, from_deg, z0=2e-4):
        self.u10 = u10
        self.from_deg = from_deg
        self.z0 = z0
        self.ustar = KAPPA * u10 / np.log(10.0 / z0)
        self.to_deg = (from_deg + 180.0) % 360.0
        a = np.radians(90.0 - self.to_deg)  # compass bearing -> math angle
        self.ex, self.ey = float(np.cos(a)), float(np.sin(a))

    def speed(self, z):
        z = np.maximum(np.asarray(z, dtype=float), self.z0)
        return self.ustar / KAPPA * np.log(z / self.z0)


class Environment:
    def __init__(self, wind):
        self.wind = wind

    @classmethod
    def flat(cls, wind):
        return cls(wind)

    def wind_at(self, x, y, z, t):
        x, y, z, t = np.broadcast_arrays(*[np.asarray(a, dtype=float) for a in (x, y, z, t)])
        s = self.wind.speed(z)
        return np.stack([s * self.wind.ex, s * self.wind.ey, np.zeros_like(s)], axis=-1)

    def surface(self, x, y, t):
        x, y, t = np.broadcast_arrays(*[np.asarray(a, dtype=float) for a in (x, y, t)])
        z = np.zeros_like(x)
        return z, z.copy(), z.copy()
