"""Direct collocation of periodic dynamic soaring cycles (point mass, height-only wind).

States s = [x, y, z, V, gamma, psi], controls (cl, mu), wind W = (u10 f(z), 0, 0).
Hermite-Simpson collocation (compressed form, no midpoint variables), SLSQP with an
analytic constraint Jacobian (the node Jacobians of the dynamics come from complex step).
"""
import time

import numpy as np
from scipy.optimize import minimize

from sim.aircraft import G

NS, NC = 6, 2


class Profile:
    """f(z) = U(z) / u10 for a log profile, optionally capped at a knee height.

    f(z) = ln(zc / z0) / ln(zref / z0) with zc = min(max(z, 1.5 z0), knee). The knee min is
    smoothed over `smooth` metres (softplus) so the optimizer sees a C-infinity profile; the
    deviation from the sharp knee is at most smooth * ln 2 in zc. Complex-step safe.
    """

    def __init__(self, z0, zref=10.0, knee=None, smooth=0.4):
        self.z0, self.zref = float(z0), float(zref)
        self.knee = None if knee is None else float(knee)
        self.smooth = float(smooth)
        self.lo = 1.5 * self.z0
        self.L = np.log(self.zref / self.z0)

    @classmethod
    def from_wind(cls, wind, smooth=0.4):
        """Same profile as sim.seawind.Wind.speed(z) / wind.u10 (away from the knee)."""
        if wind.profile == "swell_knee":
            return cls(wind.z0, min(10.0, wind.knee_m), wind.knee_m, smooth)
        return cls(wind.z0, 10.0)

    def _zc(self, z):
        zr = np.real(z)
        zc = np.where(zr < self.lo, self.lo, z)
        if self.knee is None:
            return zc, 1.0
        a = (zc - self.knee) / self.smooth
        ar = np.real(a)
        big = ar > 30.0
        sp = np.where(big, a, np.log1p(np.exp(np.where(big, 0.0, a))))
        sig = 1.0 / (1.0 + np.exp(-np.where(np.abs(ar) > 60.0, 60.0 * np.sign(ar), a)))
        return zc - self.smooth * sp, 1.0 - sig

    def f(self, z):
        zc, _ = self._zc(z)
        return np.log(zc / self.z0) / self.L

    def fp(self, z):
        zc, dz = self._zc(z)
        inside = np.real(z) > self.lo
        return np.where(inside, dz / (zc * self.L), 0.0)


def derivs(S, C, ac, U, Up):
    """State derivative for states S (...,6), controls C (...,2), wind U and dU/dz."""
    V, g, p = S[..., 3], S[..., 4], S[..., 5]
    cl, mu = C[..., 0], C[..., 1]
    cg, sg, cp, sp = np.cos(g), np.sin(g), np.cos(p), np.sin(p)
    q = 0.5 * ac.rho * V * V * ac.wing_area
    L = q * cl
    D = q * (ac.cd0 + ac.k_induced * cl * cl)
    m = ac.mass
    dz = V * sg
    aw = Up * dz  # a_W along x
    out = np.empty(S.shape, dtype=np.result_type(S, C, U, Up))
    out[..., 0] = V * cg * cp + U
    out[..., 1] = V * cg * sp
    out[..., 2] = dz
    out[..., 3] = -D / m - G * sg - aw * cg * cp
    out[..., 4] = (L * np.cos(mu) / m - G * cg + aw * sg * cp) / V
    out[..., 5] = (L * np.sin(mu) / m + aw * sp) / (V * cg)
    return out


def dyn(S, C, ac, prof, u10):
    z = S[..., 2]
    return derivs(S, C, ac, u10 * prof.f(z), u10 * prof.fp(z))


_EPS = 1e-30


def dyn_jac(A, ac, prof, u10):
    """f and Jacobian by complex step. A (n,8)=[S,C]. Returns f (n,6), J (n,6,9) (last col: u10)."""
    n = A.shape[0]
    P = np.repeat(A[None].astype(complex), 9, axis=0)  # (9,n,8)
    u = np.full((9, n), u10, dtype=complex)
    for j in range(8):
        P[j, :, j] += 1j * _EPS
    u[8] += 1j * _EPS
    F = derivs(P[..., :NS], P[..., NS:], ac, u * prof.f(P[..., 2]), u * prof.fp(P[..., 2]))
    J = np.moveaxis(F.imag / _EPS, 0, -1)  # (n,6,9)
    return F[8].real, J


class Collocation:
    def __init__(self, ac, prof, mode="min_wind", cycle="travel", n=40, u10=None, chi=np.pi / 2,
                 h_min=1.0, h_tip_min=0.3, v_factor=1.1):
        if mode not in ("min_wind", "max_speed"):
            raise ValueError(mode)
        if cycle not in ("travel", "loiter"):
            raise ValueError(cycle)
        if mode == "max_speed" and u10 is None:
            raise ValueError("max_speed needs u10")
        self.ac, self.prof, self.mode, self.cycle, self.N = ac, prof, mode, cycle, n
        self.u_fixed = u10
        self.chi, self.h_min, self.h_tip = chi, h_min, h_tip_min
        self.vmin = max(v_factor * ac.v_stall, 1.0)
        vs = ac.v_stall
        self.sc = np.concatenate([np.tile([100.0, 100.0, 10.0, vs, 1.0, 1.0], n),
                                  np.tile([1.0, 1.0], n), [10.0, 10.0]])
        self.nv = n * 8 + 2
        self.iT, self.iU = n * 8, n * 8 + 1
        self.rowsc = 1.0 / np.tile([100.0, 100.0, 10.0, vs, 1.0, 1.0], n - 1)
        self._key, self._val = None, None

    # ---- packing
    def pack(self, S, C, T, u10):
        return np.concatenate([S.ravel(), C.ravel(), [T, u10]]) / self.sc

    def unpack(self, x):
        v = np.asarray(x) * self.sc
        N = self.N
        return (v[:N * NS].reshape(N, NS), v[N * NS:N * 8].reshape(N, NC), v[self.iT], v[self.iU])

    def bounds(self):
        ac, N = self.ac, self.N
        lo = np.concatenate([np.tile([-1e4, -1e4, self.h_min, self.vmin, -1.2, -1e3], N),
                             np.tile([ac.cl_min, -ac.mu_max], N), [1.0, 0.5]])
        hi = np.concatenate([np.tile([1e4, 1e4, 200.0, 80.0, 1.2, 1e3], N),
                             np.tile([ac.cl_max, ac.mu_max], N), [60.0, 60.0]])
        lo[0], hi[0], lo[1], hi[1] = 0.0, 0.0, 0.0, 0.0  # x0 = y0 = 0
        if self.mode == "max_speed":
            lo[self.iU] = hi[self.iU] = self.u_fixed
        return list(zip(lo / self.sc, hi / self.sc))

    # ---- collocation defects with Jacobian (physical units, then row- and column-scaled)
    def _defects(self, x):
        key = x.tobytes()
        if key == self._key:
            return self._val
        S, C, T, u = self.unpack(x)
        N, ac, prof = self.N, self.ac, self.prof
        h = T / (N - 1)
        A = np.concatenate([S, C], axis=1)
        F, J = dyn_jac(A, ac, prof, u)
        Fk, Fk1 = F[:-1], F[1:]
        Sm = 0.5 * (S[:-1] + S[1:]) + h / 8.0 * (Fk - Fk1)
        Cm = 0.5 * (C[:-1] + C[1:])
        Fm, Jm = dyn_jac(np.concatenate([Sm, Cm], axis=1), ac, prof, u)
        d = S[1:] - S[:-1] - h / 6.0 * (Fk + 4 * Fm + Fk1)
        M = N - 1
        I6 = np.eye(NS)
        Jk, Jk1 = J[:-1], J[1:]
        Jms, Jmc, Jmu = Jm[:, :, :NS], Jm[:, :, NS:8], Jm[:, :, 8]
        dSk = -I6 - h / 6.0 * (Jk[:, :, :NS] + 4 * Jms @ (0.5 * I6 + h / 8.0 * Jk[:, :, :NS]))
        dSk1 = I6 - h / 6.0 * (Jk1[:, :, :NS] + 4 * Jms @ (0.5 * I6 - h / 8.0 * Jk1[:, :, :NS]))
        dCk = -h / 6.0 * (Jk[:, :, NS:8] + 2 * Jmc)
        dCk += -h / 6.0 * 4 * (Jms @ (h / 8.0 * Jk[:, :, NS:8]))
        dCk1 = -h / 6.0 * (Jk1[:, :, NS:8] + 2 * Jmc)
        dCk1 += -h / 6.0 * 4 * (Jms @ (-h / 8.0 * Jk1[:, :, NS:8]))
        dh = -(Fk + 4 * Fm + Fk1) / 6.0 - h / 6.0 * 4 * np.einsum("kij,kj->ki", Jms, (Fk - Fk1) / 8.0)
        dT = dh / (N - 1)
        dU = -h / 6.0 * (Jk[:, :, 8] + 4 * (Jmu + np.einsum("kij,kj->ki", Jms,
                                                              h / 8.0 * (Jk[:, :, 8] - Jk1[:, :, 8])))
                         + Jk1[:, :, 8])
        Jac = np.zeros((M * NS, self.nv))
        for k in range(M):
            r = slice(k * NS, (k + 1) * NS)
            Jac[r, k * NS:(k + 1) * NS] = dSk[k]
            Jac[r, (k + 1) * NS:(k + 2) * NS] = dSk1[k]
            Jac[r, N * NS + k * NC:N * NS + (k + 1) * NC] = dCk[k]
            Jac[r, N * NS + (k + 1) * NC:N * NS + (k + 2) * NC] = dCk1[k]
            Jac[r, self.iT] = dT[k]
            Jac[r, self.iU] = dU[k]
        Jac = Jac * self.sc[None, :] * self.rowsc[:, None]
        self._key, self._val = key, (d.ravel() * self.rowsc, Jac)
        return self._val

    def _periodic(self, x):
        """Linear periodicity and phase conditions, scaled. Returns (c, Jac)."""
        N = self.N
        sc = self.sc
        rows = []
        idx = lambda node, comp: node * NS + comp
        for comp in (2, 3, 4):  # z, V, gamma
            rows.append(({idx(N - 1, comp): 1.0, idx(0, comp): -1.0}, 0.0))
        dpsi = 2 * np.pi if self.cycle == "loiter" else 0.0
        rows.append(({idx(N - 1, 5): 1.0, idx(0, 5): -1.0}, dpsi))
        rows.append(({idx(0, 4): 1.0}, 0.0))  # phase: gamma = 0 at the first node
        Jac = np.zeros((len(rows), self.nv))
        c = np.zeros(len(rows))
        v = x * sc
        for i, (terms, rhs) in enumerate(rows):
            for j, a in terms.items():
                Jac[i, j] = a * sc[j]
                c[i] += a * v[j]
            c[i] -= rhs
        return c, Jac

    def eq_fun(self, x):
        return np.concatenate([self._defects(x)[0], self._periodic(x)[0]])

    def eq_jac(self, x):
        return np.vstack([self._defects(x)[1], self._periodic(x)[1]])

    def _path(self, x):
        S, C, T, u = self.unpack(x)
        ac = self.ac
        V, z = S[:, 3], S[:, 2]
        cl, mu = C[:, 0], C[:, 1]
        K = 0.5 * ac.rho * ac.wing_area / (ac.mass * G)
        n = K * V * V * cl
        absmu = np.sqrt(mu * mu + 1e-4)
        tip = z - 0.5 * ac.span * absmu - self.h_tip
        N = self.N
        Jn = np.zeros((N, self.nv))
        Jt = np.zeros((N, self.nv))
        i = np.arange(N)
        Jn[i, i * NS + 3] = 2 * K * V * cl * self.sc[i * NS + 3]
        Jn[i, N * NS + i * NC] = K * V * V
        Jt[i, i * NS + 2] = self.sc[i * NS + 2]
        Jt[i, N * NS + i * NC + 1] = -0.5 * ac.span * mu / absmu
        return np.concatenate([(ac.n_max - n) / ac.n_max, tip]), \
            np.vstack([-Jn / ac.n_max, Jt])

    def ineq_fun(self, x):
        return self._path(x)[0]

    def ineq_jac(self, x):
        return self._path(x)[1]

    def objective(self, x):
        if self.mode == "min_wind":
            g = np.zeros(self.nv)
            g[self.iU] = 1.0
            return x[self.iU] * self.sc[self.iU] * 0.1, g * self.sc[self.iU] * 0.1
        N, sc = self.N, self.sc
        c, s = np.cos(self.chi), np.sin(self.chi)
        v = x * sc
        T = v[self.iT]
        xe, ye = v[(N - 1) * NS], v[(N - 1) * NS + 1]
        proj = c * xe + s * ye
        f = -0.1 * proj / T
        g = np.zeros(self.nv)
        g[(N - 1) * NS] = -0.1 * c / T * sc[(N - 1) * NS]
        g[(N - 1) * NS + 1] = -0.1 * s / T * sc[(N - 1) * NS + 1]
        g[self.iT] = 0.1 * proj / T ** 2 * sc[self.iT]
        return f, g

    def violation(self, x):
        eq = self.eq_fun(x)
        ie = self.ineq_fun(x)
        b = np.asarray(self.bounds())
        bv = max(0.0, float(np.max(b[:, 0] - x)), float(np.max(x - b[:, 1])))
        return max(float(np.max(np.abs(eq))), float(max(0.0, -np.min(ie))), bv)

    # ---- initial guess
    def guess(self, rng, t0=8.0):
        ac, N, prof = self.ac, self.N, self.prof
        T = t0 * rng.uniform(0.8, 1.25)
        w = 2 * np.pi / T
        t = np.linspace(0.0, T, N)
        zlo = max(1.5, self.h_min + 0.5)
        zhi = 10.0 * rng.uniform(0.8, 1.0)
        zm, A = 0.5 * (zlo + zhi), 0.5 * (zhi - zlo)
        z = zm - A * np.cos(w * t)
        dz = A * w * np.sin(w * t)
        V = np.full(N, 1.5 * ac.v_stall * rng.uniform(0.9, 1.4))
        V = np.maximum(V, self.vmin * 1.05)
        g = np.arcsin(np.clip(dz / V, -0.9, 0.9))
        psi0 = rng.uniform(0, 2 * np.pi)
        if self.cycle == "loiter":
            psi = psi0 + w * t
            dpsi = np.full(N, w)
        else:
            B = rng.uniform(0.8, 1.6)
            ph = rng.uniform(0, 2 * np.pi)
            psi = psi0 + B * (np.sin(w * t + ph) - np.sin(ph))
            dpsi = B * w * np.cos(w * t + ph)
        mu = np.clip(np.arctan(V * np.cos(g) * dpsi / G), -ac.mu_max * 0.9, ac.mu_max * 0.9)
        cl = np.clip(2 * ac.mass * G * np.cos(g) / (ac.rho * V * V * ac.wing_area * np.cos(mu)),
                     ac.cl_min, ac.cl_max)
        u = rng.uniform(6.0, 12.0) if self.mode == "min_wind" else self.u_fixed
        dx = V * np.cos(g) * np.cos(psi) + u * prof.f(z)
        dy = V * np.cos(g) * np.sin(psi)
        x = np.concatenate([[0.0], np.cumsum(0.5 * (dx[1:] + dx[:-1]) * np.diff(t))])
        y = np.concatenate([[0.0], np.cumsum(0.5 * (dy[1:] + dy[:-1]) * np.diff(t))])
        S = np.stack([x, y, z, V, g, psi], axis=1)
        return self.pack(S, np.stack([cl, mu], axis=1), T, u)


def _summary(col, x, res, viol, ok, seed, runtime):
    S, C, T, u = col.unpack(x)
    ac, N = col.ac, col.N
    q = 0.5 * ac.rho * S[:, 3] ** 2 * ac.wing_area
    n = q * C[:, 0] / (ac.mass * G)
    xe, ye = S[-1, 0], S[-1, 1]
    # midpoint heights (Hermite-Simpson interpolation) for the clearance check
    A = np.concatenate([S, C], axis=1)
    F, _ = dyn_jac(A, ac, col.prof, u)
    h = T / (N - 1)
    zm = 0.5 * (S[:-1, 2] + S[1:, 2]) + h / 8 * (F[:-1, 2] - F[1:, 2])
    tip = np.concatenate([S[:, 2], zm]).min()
    return {"success": bool(ok), "message": str(res.message), "violation": float(viol),
            "u10": float(u), "period": float(T), "z_min": float(S[:, 2].min()),
            "z_max": float(S[:, 2].max()), "z_min_mid": float(zm.min()), "n_max": float(n.max()),
            "V_min": float(S[:, 3].min()), "V_max": float(S[:, 3].max()),
            "ground_speed": float(np.hypot(xe, ye) / T),
            "speed_along_chi": float((xe * np.cos(col.chi) + ye * np.sin(col.chi)) / T),
            "x_end": float(xe), "y_end": float(ye),
            "x_extent": float(S[:, 0].max() - S[:, 0].min()),
            "y_extent": float(S[:, 1].max() - S[:, 1].min()),
            "net_turn_deg": float(np.degrees(S[-1, 5] - S[0, 5])),
            "mu_max_deg": float(np.degrees(np.abs(C[:, 1]).max())),
            "cl_range": [float(C[:, 0].min()), float(C[:, 0].max())],
            "near_converged": bool(ok or viol < 1e-3),
            "seed": seed, "runtime_s": runtime, "iterations": int(res.nit), "n_nodes": N,
            "objective": float(u if col.mode == "min_wind" else -res.fun * 10.0)}


def solve(ac, prof, mode="min_wind", cycle="travel", n=40, u10=None, chi=np.pi / 2,
          h_min=1.0, h_tip_min=0.3, v_factor=1.1, seeds=(0, 1, 2), maxiter=300, tol=1e-5,
          x0=None, return_x=False, verbose=False, stop_after=2):
    """Solve for the best converged cycle among the seeded initial guesses."""
    col = Collocation(ac, prof, mode, cycle, n, u10, chi, h_min, h_tip_min, v_factor)
    cons = [{"type": "eq", "fun": col.eq_fun, "jac": col.eq_jac},
            {"type": "ineq", "fun": col.ineq_fun, "jac": col.ineq_jac}]
    bnds = col.bounds()
    starts = [(None, x0)] if x0 is not None else [(s, None) for s in seeds]
    best = None
    n_ok = 0
    for seed, xs in starts:
        t0 = time.time()
        if xs is None:
            xs = col.guess(np.random.default_rng(seed))
        lo = np.array([b[0] for b in bnds])
        hi = np.array([b[1] for b in bnds])
        xs = np.clip(xs, lo, hi)
        res = minimize(col.objective, xs, jac=True, method="SLSQP", bounds=bnds,
                       constraints=cons, options={"maxiter": maxiter, "ftol": 1e-9})
        viol = col.violation(res.x)
        ok = bool(res.success and viol < tol)
        info = _summary(col, res.x, res, viol, ok, seed, time.time() - t0)
        if verbose:
            print(seed, info["success"], "%.2e" % viol, info["u10"], info["period"],
                  info["ground_speed"], info["iterations"], "%.1fs" % info["runtime_s"], flush=True)
        better = (best is None
                  or (ok and not best[0]["success"])
                  or (ok == best[0]["success"] and (
                      (ok and info["objective"] * (1 if mode == "min_wind" else -1)
                       < best[0]["objective"] * (1 if mode == "min_wind" else -1))
                      or (not ok and viol < best[0]["violation"]))))
        if better:
            best = (info, res.x.copy())
        n_ok += int(ok)
        if n_ok >= stop_after:
            break
    if return_x:
        return best[0], best[1], col
    return best[0]


def sachs_albatross():
    """Wandering albatross of Sachs 2005 as recomputed by Bower 2011 (record P45).

    Returns (Aircraft, Profile) with the log profile z0 = 0.03 m, reference height 10 m.
    The induced factor is set so that (L/D)max = 20: k = 1 / (4 cd0 20^2).
    """
    from sim.aircraft import Aircraft
    area, ar, cd0 = 0.65, 16.81, 0.033
    k = 1.0 / (4.0 * cd0 * 20.0 ** 2)
    ac = Aircraft("albatross-sachs2005", mass=8.5, span=float(np.sqrt(ar * area)),
                  wing_area=area, cd0=cd0, oswald=1.0 / (np.pi * ar * k), cl_max=1.5,
                  cl_min=-0.2, mu_max=float(np.radians(80.0)), n_max=3.0)
    return ac, Profile(0.03, 10.0)
