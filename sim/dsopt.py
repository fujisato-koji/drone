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
U10_FLOOR_WAVE = 0.1  # lower bound of u10 in the wave mode (0.5 in the flat mode)


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


SHELTER_SMOOTH = 0.05  # softplus width of the shelter clip, in units of (sigma - s_c) / s_c
SHELTER_GATE = 0.3  # m/s, width of the smooth gate urel > 0


def _softplus(y, w):
    """w ln(1 + exp(y / w)), complex-step safe."""
    a = y / w
    ar = np.real(a)
    big = ar > 30.0
    return w * np.where(big, a, np.log1p(np.exp(np.where(big, 0.0, a))))


def _sigmoid(y):
    yr = np.real(y)
    return 1.0 / (1.0 + np.exp(-np.where(np.abs(yr) > 60.0, 60.0 * np.sign(yr), y)))


class WaveParams:
    """One deep-water wave component: a (m), wavelength (m), to_deg (math-frame direction of travel).

    The frame is the optimizer's: the mean wind blows toward +x, angles counter-clockwise from +x,
    so to_deg = 0 is a wave running with the wind and 90 a wave running across it.

    decay = (a_u, a_w): horizontal / vertical perturbation decay exp(-a k h) (default (1, 1)).
    shelter: lee-side sheltering (see wave_wind_accel). midpoints: also constrain h and the tip
    height at the Hermite-Simpson midpoints (default True).
    """

    def __init__(self, a, wavelength, to_deg, decay=(1.0, 1.0), shelter=False, midpoints=True):
        from sim.record import value
        self.a, self.wavelength, self.to_deg = float(a), float(wavelength), float(to_deg)
        self.k = 2.0 * np.pi / self.wavelength
        self.om = float(np.sqrt(G * self.k))
        self.c = self.om / self.k
        th = np.radians(self.to_deg)
        self.cw, self.sw = float(np.cos(th)), float(np.sin(th))
        self.decay = (float(decay[0]), float(decay[1]))
        self.shelter = bool(shelter)
        self.midpoints = bool(midpoints)
        self.F = value("env.shelter_factor", 1.0)
        self.D = value("env.shelter_decay_kz", 0.5)


def shelter_onset(wv, prof, u10):
    """Onset slope s_c = sim.seawind.separation_steepness(ustar, c), ustar = kappa u10 / ln(10/z0).

    That function takes real numbers only, so for a complex-step u10 the value is the real value
    plus i Im(u10) ds_c/du10, with the derivative by a central difference of the same function.
    """
    from sim.seawind import KARMAN, separation_steepness

    def sc(u):
        return separation_steepness(KARMAN * u / np.log(10.0 / prof.z0), wv.c)

    ur = np.real(u10)
    val = sc(ur)
    if not np.iscomplexobj(u10):
        return val
    d = 1e-4
    return val + 1j * np.imag(u10) * (sc(ur + d) - sc(ur - d)) / (2 * d)


def shelter_r(th, urel, s_c, wv):
    """Separation ramp r(theta) in [0, 1] and dr/dtheta (smooth clip, see wave_wind_accel)."""
    st, ct = np.sin(th), np.cos(th)
    sig = wv.a * wv.k * wv.cw * st
    xs = (sig - s_c) / s_c
    gate = _sigmoid(urel / SHELTER_GATE)
    w = SHELTER_SMOOTH
    r = gate * (_softplus(xs, w) - _softplus(xs - 1.0, w))
    r_t = gate * (_sigmoid(xs / w) - _sigmoid((xs - 1.0) / w)) * (wv.a * wv.k * wv.cw * ct / s_c)
    return r, r_t


def wave_wind_accel(S, wv, prof, tt, u10, phi0):
    """Wind W (3 arrays) and wind acceleration along the path a_W (3 arrays) in the wave mode.

    Model of sim.seawind for one component: log profile at the height above the local surface
    h = z - a cos(theta), plus the wave-coherent perturbation with urel = u10 f(zref) cos(to) - c,
    zref = max(1/k, 2 z0), decaying as exp(-a_u k h) (horizontal) and exp(-a_w k h) (vertical).
    a_W = dW/dt + (v_g . grad) W is analytic: with Q(h, theta), dQ/dz = Q_h and
    dQ/d(x, y, t) = (Q_h a sin(theta) + Q_theta) d(theta)/d(x, y, t). Complex-step safe.

    Shelter (wv.shelter): lee steepness sigma = a k cos(to) sin(theta) (= -d eta/ds, s along the
    wind), r = clip((sigma - s_c)/s_c, 0, 1) times a gate on urel > 0, with the onset slope s_c
    from sim.seawind.separation_steepness at the current u10 and z0 (shelter_onset). The clip is a difference of
    two softplus functions (width SHELTER_SMOOTH), the gate a sigmoid of urel (width
    SHELTER_GATE m/s); both are smooth, with small leaks outside the sharp ranges. The along-wind
    wind (x: mean plus perturbation) is multiplied by 1 - F r exp(-k h / D), the vertical
    perturbation by 1 - r.
    """
    x, y, z, V, g, p = (S[..., i] for i in range(6))
    cg, sg, cp, sp = np.cos(g), np.sin(g), np.cos(p), np.sin(p)
    th = wv.k * (wv.cw * x + wv.sw * y) - wv.om * tt + phi0
    st, ct = np.sin(th), np.cos(th)
    h = z - wv.a * ct
    zref = max(1.0 / wv.k, 2.0 * prof.z0)
    urel = u10 * float(prof.f(zref)) * wv.cw - wv.c
    base = urel * wv.a * wv.k
    Pu = base * np.exp(-wv.decay[0] * wv.k * h)
    Pw = base * np.exp(-wv.decay[1] * wv.k * h)
    Pu_h = -wv.decay[0] * wv.k * Pu
    Pw_h = -wv.decay[1] * wv.k * Pw
    Bx = u10 * prof.f(h) + Pu * ct * wv.cw
    Bx_h = u10 * prof.fp(h) + Pu_h * ct * wv.cw
    Bx_t = -Pu * st * wv.cw
    if wv.shelter:
        r, r_t = shelter_r(th, urel, shelter_onset(wv, prof, u10), wv)
        E = np.exp(-wv.k * h / wv.D)
        Sf = 1.0 - wv.F * r * E
        Sf_h = wv.F * r * E * wv.k / wv.D
        Sf_t = -wv.F * E * r_t
        Wx = Bx * Sf
        Hx = Bx_h * Sf + Bx * Sf_h
        Ax = (Bx_h * Sf + Bx * Sf_h) * wv.a * st + Bx_t * Sf + Bx * Sf_t
        Wz = -Pw * st * (1.0 - r)
        Hz = -Pw_h * st * (1.0 - r)
        Az = Hz * wv.a * st - Pw * ct * (1.0 - r) + Pw * st * r_t
    else:
        Wx, Hx = Bx, Bx_h
        Ax = Hx * wv.a * st + Bx_t
        Wz = -Pw * st
        Hz = -Pw_h * st
        Az = Hz * wv.a * st - Pw * ct
    Wy = Pu * ct * wv.sw
    Hy = Pu_h * ct * wv.sw
    Ay = Hy * wv.a * st - Pu * st * wv.sw
    vgx, vgy, vgz = V * cg * cp + Wx, V * cg * sp + Wy, V * sg + Wz
    dth = wv.k * (wv.cw * vgx + wv.sw * vgy) - wv.om
    return ((Wx, Wy, Wz), (Ax * dth + Hx * vgz, Ay * dth + Hy * vgz, Az * dth + Hz * vgz))


def derivs_wave(S, C, ac, prof, wv, tt, u10, phi0):
    """State derivative in the wave mode (z is the absolute height, wind acts in x, y and z)."""
    V, g, p = S[..., 3], S[..., 4], S[..., 5]
    cl, mu = C[..., 0], C[..., 1]
    cg, sg, cp, sp = np.cos(g), np.sin(g), np.cos(p), np.sin(p)
    q = 0.5 * ac.rho * V * V * ac.wing_area
    L = q * cl
    D = q * (ac.cd0 + ac.k_induced * cl * cl)
    m = ac.mass
    (Wx, Wy, Wz), (ax, ay, az) = wave_wind_accel(S, wv, prof, tt, u10, phi0)
    out = np.empty(S.shape, dtype=np.result_type(S, C, Wx, ax))
    out[..., 0] = V * cg * cp + Wx
    out[..., 1] = V * cg * sp + Wy
    out[..., 2] = V * sg + Wz
    out[..., 3] = -D / m - G * sg - (ax * cg * cp + ay * cg * sp + az * sg)
    out[..., 4] = (L * np.cos(mu) / m - G * cg + (ax * sg * cp + ay * sg * sp - az * cg)) / V
    out[..., 5] = (L * np.sin(mu) / m + ax * sp - ay * cp) / (V * cg)
    return out


def dyn_jac_wave(A, tt, u10, phi0, ac, prof, wv):
    """f and Jacobian by complex step. A (n,8), tt (n,). J (n,6,10): S, C, u10, phi0.

    The time derivative is -om * (phi0 column): t and phi0 enter only as phi0 - om t.
    """
    n = A.shape[0]
    P = np.repeat(A[None].astype(complex), 10, axis=0)
    u = np.full((10, n), u10, dtype=complex)
    ph = np.full((10, n), phi0, dtype=complex)
    for j in range(8):
        P[j, :, j] += 1j * _EPS
    u[8] += 1j * _EPS
    ph[9] += 1j * _EPS
    F = derivs_wave(P[..., :NS], P[..., NS:], ac, prof, wv, np.broadcast_to(tt, (10, n)), u, ph)
    J = np.moveaxis(F.imag / _EPS, 0, -1)
    return F[8].real, J


class Collocation:
    def __init__(self, ac, prof, mode="min_wind", cycle="travel", n=40, u10=None, chi=np.pi / 2,
                 h_min=1.0, h_tip_min=0.3, v_factor=1.1, tip_model="abs", wave=None):
        if mode not in ("min_wind", "max_speed"):
            raise ValueError(mode)
        if cycle not in ("travel", "loiter"):
            raise ValueError(cycle)
        if tip_model not in ("abs", "geometric"):
            raise ValueError(tip_model)
        if mode == "max_speed" and u10 is None:
            raise ValueError("max_speed needs u10")
        self.ac, self.prof, self.mode, self.cycle, self.N = ac, prof, mode, cycle, n
        self.u_fixed = u10
        self.chi, self.h_min, self.h_tip = chi, h_min, h_tip_min
        self.tip_model = tip_model
        self.wv = None if wave is None else (wave if isinstance(wave, WaveParams)
                                             else WaveParams(**wave))
        self.m = 0
        self.vmin = max(v_factor * ac.v_stall, 1.0)
        vs = ac.v_stall
        extra = [] if self.wv is None else [1.0]
        self.sc = np.concatenate([np.tile([100.0, 100.0, 10.0, vs, 1.0, 1.0], n),
                                  np.tile([1.0, 1.0], n), [10.0, 10.0], extra])
        self.nv = n * 8 + 2 + len(extra)
        self.iT, self.iU = n * 8, n * 8 + 1
        self.iP = n * 8 + 2 if self.wv is not None else None
        self.rowsc = 1.0 / np.tile([100.0, 100.0, 10.0, vs, 1.0, 1.0], n - 1)
        self._key, self._val = None, None

    # ---- packing
    def pack(self, S, C, T, u10, phi0=0.0):
        tail = [T, u10] + ([phi0] if self.wv is not None else [])
        return np.concatenate([S.ravel(), C.ravel(), tail]) / self.sc

    def unpack(self, x):
        v = np.asarray(x) * self.sc
        N = self.N
        return (v[:N * NS].reshape(N, NS), v[N * NS:N * 8].reshape(N, NC), v[self.iT], v[self.iU])

    def phi0(self, x):
        return float(x[self.iP] * self.sc[self.iP])

    def phase_advance(self, x):
        """theta(T) - theta(0) in the wave mode (phi0 cancels; x0 = y0 = 0)."""
        S, _, T, _ = self.unpack(x)
        wv = self.wv
        return float(wv.k * (wv.cw * (S[-1, 0] - S[0, 0]) + wv.sw * (S[-1, 1] - S[0, 1]))
                     - wv.om * T)

    def set_m(self, x):
        """Choose the integer m of the periodicity theta(T) - theta(0) = -2 pi m from x."""
        if self.wv is not None:
            self.m = int(round(-self.phase_advance(x) / (2.0 * np.pi)))
        return self.m

    def node_theta(self, S, T, phi0):
        wv = self.wv
        t = np.arange(len(S)) * T / (len(S) - 1)
        return wv.k * (wv.cw * S[:, 0] + wv.sw * S[:, 1]) - wv.om * t + phi0

    def bounds(self):
        ac, N = self.ac, self.N
        zlo = self.h_min if self.wv is None else -100.0
        lo = np.concatenate([np.tile([-1e4, -1e4, zlo, self.vmin, -1.2, -1e3], N),
                             np.tile([ac.cl_min, -ac.mu_max], N), [1.0, 0.5]])
        hi = np.concatenate([np.tile([1e4, 1e4, 200.0, 80.0, 1.2, 1e3], N),
                             np.tile([ac.cl_max, ac.mu_max], N), [60.0, 60.0]])
        if self.wv is not None:
            lo, hi = np.append(lo, -100.0), np.append(hi, 100.0)
            lo[self.iU] = U10_FLOOR_WAVE  # a wave can carry the cycle with (almost) no mean wind
        lo[0], hi[0], lo[1], hi[1] = 0.0, 0.0, 0.0, 0.0  # x0 = y0 = 0
        if self.mode == "max_speed":
            lo[self.iU] = hi[self.iU] = self.u_fixed
        return list(zip(lo / self.sc, hi / self.sc))

    # ---- collocation defects with Jacobian (physical units, then row- and column-scaled)
    def _defects_wave(self, x):
        S, C, T, u = self.unpack(x)
        phi = self.phi0(x)
        N, ac, prof, wv = self.N, self.ac, self.prof, self.wv
        h = T / (N - 1)
        M = N - 1
        kk = np.arange(M)[:, None]
        A = np.concatenate([S, C], axis=1)
        F, J = dyn_jac_wave(A, np.arange(N) * h, u, phi, ac, prof, wv)
        Fk, Fk1 = F[:-1], F[1:]
        Sm = 0.5 * (S[:-1] + S[1:]) + h / 8.0 * (Fk - Fk1)
        Cm = 0.5 * (C[:-1] + C[1:])
        Fm, Jm = dyn_jac_wave(np.concatenate([Sm, Cm], axis=1), (np.arange(M) + 0.5) * h,
                              u, phi, ac, prof, wv)
        d = S[1:] - S[:-1] - h / 6.0 * (Fk + 4 * Fm + Fk1)
        I6 = np.eye(NS)
        Jk, Jk1 = J[:-1], J[1:]
        Jms, Jmc = Jm[:, :, :NS], Jm[:, :, NS:8]
        dSk = -I6 - h / 6.0 * (Jk[:, :, :NS] + 4 * Jms @ (0.5 * I6 + h / 8.0 * Jk[:, :, :NS]))
        dSk1 = I6 - h / 6.0 * (Jk1[:, :, :NS] + 4 * Jms @ (0.5 * I6 - h / 8.0 * Jk1[:, :, :NS]))
        dCk = -h / 6.0 * (Jk[:, :, NS:8] + 2 * Jmc)
        dCk += -h / 6.0 * 4 * (Jms @ (h / 8.0 * Jk[:, :, NS:8]))
        dCk1 = -h / 6.0 * (Jk1[:, :, NS:8] + 2 * Jmc)
        dCk1 += -h / 6.0 * 4 * (Jms @ (-h / 8.0 * Jk1[:, :, NS:8]))

        def dpar(col):  # derivative wrt a parameter that acts on F only (u10, phi0)
            return -h / 6.0 * (Jk[:, :, col] + 4 * (Jm[:, :, col] + np.einsum(
                "kij,kj->ki", Jms, h / 8.0 * (Jk[:, :, col] - Jk1[:, :, col]))) + Jk1[:, :, col])

        # step size h: F depends on t = (index) h through -om * dF/dphi0
        dFk = -wv.om * Jk[:, :, 9] * kk
        dFk1 = -wv.om * Jk1[:, :, 9] * (kk + 1)
        dSm = (Fk - Fk1) / 8.0 + h / 8.0 * (dFk - dFk1)
        dFm = np.einsum("kij,kj->ki", Jms, dSm) - wv.om * Jm[:, :, 9] * (kk + 0.5)
        dh = -(Fk + 4 * Fm + Fk1) / 6.0 - h / 6.0 * (dFk + 4 * dFm + dFk1)
        dT = dh / (N - 1)
        Jac = np.zeros((M * NS, self.nv))
        dU, dP = dpar(8), dpar(9)
        for k in range(M):
            r = slice(k * NS, (k + 1) * NS)
            Jac[r, k * NS:(k + 1) * NS] = dSk[k]
            Jac[r, (k + 1) * NS:(k + 2) * NS] = dSk1[k]
            Jac[r, N * NS + k * NC:N * NS + (k + 1) * NC] = dCk[k]
            Jac[r, N * NS + (k + 1) * NC:N * NS + (k + 2) * NC] = dCk1[k]
            Jac[r, self.iT] = dT[k]
            Jac[r, self.iU] = dU[k]
            Jac[r, self.iP] = dP[k]
        # midpoint state Sm (Hermite-Simpson interpolation) and its physical Jacobian, for the
        # midpoint clearance constraints
        Jsm = np.zeros((M, NS, self.nv))
        for k in range(M):
            Jsm[k, :, k * NS:(k + 1) * NS] = 0.5 * I6 + h / 8.0 * Jk[k, :, :NS]
            Jsm[k, :, (k + 1) * NS:(k + 2) * NS] = 0.5 * I6 - h / 8.0 * Jk1[k, :, :NS]
            Jsm[k, :, N * NS + k * NC:N * NS + (k + 1) * NC] = h / 8.0 * Jk[k, :, NS:8]
            Jsm[k, :, N * NS + (k + 1) * NC:N * NS + (k + 2) * NC] = -h / 8.0 * Jk1[k, :, NS:8]
            Jsm[k, :, self.iT] = dSm[k] / (N - 1)
            Jsm[k, :, self.iU] = h / 8.0 * (Jk[k, :, 8] - Jk1[k, :, 8])
            Jsm[k, :, self.iP] = h / 8.0 * (Jk[k, :, 9] - Jk1[k, :, 9])
        self._mid = (Sm, Jsm)
        Jac = Jac * self.sc[None, :] * self.rowsc[:, None]
        return d.ravel() * self.rowsc, Jac

    def _defects(self, x):
        key = x.tobytes()
        if key == self._key:
            return self._val
        if self.wv is not None:
            self._key, self._val = key, self._defects_wave(x)
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
        wv = self.wv
        if wv is not None and wv.a > 0.0:
            # the cycle repeats relative to the wave: theta(T) - theta(0) = -2 pi m
            rows.append(({idx(N - 1, 0): wv.k * wv.cw, idx(N - 1, 1): wv.k * wv.sw,
                          idx(0, 0): -wv.k * wv.cw, idx(0, 1): -wv.k * wv.sw,
                          self.iT: -wv.om}, -2.0 * np.pi * self.m))
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
        V, z, gam = S[:, 3], S[:, 2], S[:, 4]
        cl, mu = C[:, 0], C[:, 1]
        K = 0.5 * ac.rho * ac.wing_area / (ac.mass * G)
        n = K * V * V * cl
        absmu = np.sqrt(mu * mu + 1e-4)
        dabs = mu / absmu
        N = self.N
        i = np.arange(N)
        sc = self.sc
        Jn = np.zeros((N, self.nv))
        Jt = np.zeros((N, self.nv))
        Jn[i, i * NS + 3] = 2 * K * V * cl * sc[i * NS + 3]
        Jn[i, N * NS + i * NC] = K * V * V
        # height above the local surface: h = z (flat) or z - a cos(theta) (wave)
        Jh = np.zeros((N, self.nv))
        Jh[i, i * NS + 2] = sc[i * NS + 2]
        if self.wv is None:
            hh = z
        else:
            wv = self.wv
            th = self.node_theta(S, T, self.phi0(x))
            hh = z - wv.a * np.cos(th)
            dth = wv.a * np.sin(th)
            Jh[i, i * NS] = dth * wv.k * wv.cw * sc[i * NS]
            Jh[i, i * NS + 1] = dth * wv.k * wv.sw * sc[i * NS + 1]
            Jh[:, self.iT] = dth * (-wv.om * i / (N - 1)) * sc[self.iT]
            Jh[:, self.iP] = dth * sc[self.iP]
        Jt[:] = Jh
        if self.tip_model == "abs":
            tip = hh - 0.5 * ac.span * absmu - self.h_tip
            Jt[i, N * NS + i * NC + 1] = -0.5 * ac.span * dabs
        else:
            sa, ca = np.sin(absmu), np.cos(absmu)
            tip = hh - 0.5 * ac.span * sa * np.cos(gam) - self.h_tip
            Jt[i, N * NS + i * NC + 1] = -0.5 * ac.span * ca * np.cos(gam) * dabs
            Jt[i, i * NS + 4] = 0.5 * ac.span * sa * np.sin(gam) * sc[i * NS + 4]
        f = [(ac.n_max - n) / ac.n_max]
        J = [-Jn / ac.n_max]
        if self.wv is not None:
            f.append(hh - self.h_min)
            J.append(Jh)
        f.append(tip)
        J.append(Jt)
        if self.wv is not None and self.wv.midpoints:
            fm, Jm_ = self._path_mid(x)
            f.append(fm)
            J.append(Jm_)
        return np.concatenate(f), np.vstack(J)

    def _path_mid(self, x):
        """h >= h_min and the wingtip height at the Hermite-Simpson midpoints (wave mode)."""
        self._defects(x)  # fills self._mid for this x
        Sm, Jsm = self._mid
        _, C, T, _ = self.unpack(x)
        wv, ac, N = self.wv, self.ac, self.N
        M = N - 1
        kk = np.arange(M)
        sc = self.sc
        thm = wv.k * (wv.cw * Sm[:, 0] + wv.sw * Sm[:, 1]) - wv.om * (kk + 0.5) * T / (N - 1)             + self.phi0(x)
        dth = wv.k * (wv.cw * Jsm[:, 0, :] + wv.sw * Jsm[:, 1, :])
        dth[:, self.iT] += -wv.om * (kk + 0.5) / (N - 1)
        dth[:, self.iP] += 1.0
        hm = Sm[:, 2] - wv.a * np.cos(thm)
        dh = Jsm[:, 2, :] + (wv.a * np.sin(thm))[:, None] * dth
        mu = 0.5 * (C[:-1, 1] + C[1:, 1])
        dmu = np.zeros((M, self.nv))
        dmu[kk, N * NS + kk * NC + 1] = 0.5
        dmu[kk, N * NS + (kk + 1) * NC + 1] = 0.5
        absmu = np.sqrt(mu * mu + 1e-4)
        dabs = (mu / absmu)[:, None]
        gm = Sm[:, 4]
        if self.tip_model == "abs":
            tip = hm - 0.5 * ac.span * absmu - self.h_tip
            dtip = dh - 0.5 * ac.span * dabs * dmu
        else:
            sa, ca = np.sin(absmu), np.cos(absmu)
            tip = hm - 0.5 * ac.span * sa * np.cos(gm) - self.h_tip
            dtip = (dh - 0.5 * ac.span * (ca * np.cos(gm))[:, None] * dabs * dmu
                    + (0.5 * ac.span * sa * np.sin(gm))[:, None] * Jsm[:, 4, :])
        return (np.concatenate([hm - self.h_min, tip]),
                np.vstack([dh, dtip]) * sc[None, :])

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
        if self.wv is not None:
            wv = self.wv
            phi0 = rng.uniform(0.0, 2 * np.pi)  # drawn last: the other draws match the flat guess
            z = z + wv.a * np.cos(wv.k * (wv.cw * x + wv.sw * y) - wv.om * t + phi0)
        else:
            phi0 = 0.0
        S = np.stack([x, y, z, V, g, psi], axis=1)
        return self.pack(S, np.stack([cl, mu], axis=1), T, u, phi0)


def _summary(col, x, res, viol, ok, seed, runtime):
    S, C, T, u = col.unpack(x)
    ac, N = col.ac, col.N
    q = 0.5 * ac.rho * S[:, 3] ** 2 * ac.wing_area
    n = q * C[:, 0] / (ac.mass * G)
    xe, ye = S[-1, 0], S[-1, 1]
    # midpoint heights (Hermite-Simpson interpolation) for the clearance check
    A = np.concatenate([S, C], axis=1)
    h = T / (N - 1)
    if col.wv is None:
        F, _ = dyn_jac(A, ac, col.prof, u)
        zm = 0.5 * (S[:-1, 2] + S[1:, 2]) + h / 8 * (F[:-1, 2] - F[1:, 2])
    else:
        F, _ = dyn_jac_wave(A, np.arange(N) * h, u, col.phi0(x), ac, col.prof, col.wv)
        zm = 0.5 * (S[:-1, 2] + S[1:, 2]) + h / 8 * (F[:-1, 2] - F[1:, 2])
    out = _summary_flat(col, x, res, viol, ok, seed, runtime, S, C, T, u, n, xe, ye, zm)
    if col.wv is not None:
        out.update(_wave_extra(col, x, S, C, T, u, F))
    return out


def _wave_extra(col, x, S, C, T, u, F):
    wv, N = col.wv, col.N
    phi = col.phi0(x)
    th = col.node_theta(S, T, phi)
    hh = S[:, 2] - wv.a * np.cos(th)
    # midpoints (Hermite-Simpson interpolation)
    h = T / (N - 1)
    Sm = 0.5 * (S[:-1] + S[1:]) + h / 8 * (F[:-1] - F[1:])
    tm = (np.arange(N - 1) + 0.5) * h
    thm = wv.k * (wv.cw * Sm[:, 0] + wv.sw * Sm[:, 1]) - wv.om * tm + phi
    hm = Sm[:, 2] - wv.a * np.cos(thm)
    r_max = None
    if wv.shelter:
        zref = max(1.0 / wv.k, 2.0 * col.prof.z0)
        urel = u * float(col.prof.f(zref)) * wv.cw - wv.c
        s_c = shelter_onset(wv, col.prof, u)
        r_max = float(max(shelter_r(th, urel, s_c, wv)[0].max(),
                          shelter_r(thm, urel, s_c, wv)[0].max()))
    i = int(np.argmin(hh))
    th_i = float((th[i] + np.pi) % (2 * np.pi) - np.pi)
    c = np.cos(th_i)
    where = ("crest" if abs(th_i) < np.radians(30) else "trough" if abs(th_i) > np.radians(150)
             else "forward face" if th_i > 0 else "rear face")
    return {"wave": {"a": wv.a, "wavelength": wv.wavelength, "to_deg": wv.to_deg,
                     "k": wv.k, "omega": wv.om, "c": wv.c, "decay": list(wv.decay),
                     "shelter": wv.shelter, "midpoints": wv.midpoints},
            "m": int(col.m), "phi0": float(phi), "phase_advance": col.phase_advance(x),
            "h_min_achieved": float(min(hh.min(), hm.min())), "h_node_min": float(hh.min()), "h_mid_min": float(hm.min()),
            "theta_at_h_min_deg": float(np.degrees(th_i)), "eta_over_a_at_h_min": float(c),
            "h_min_position": where, "shelter_r_max": r_max, "shelter_s_c": (
                float(shelter_onset(wv, col.prof, u)) if wv.shelter else None),
            "u10_at_floor": bool(u <= U10_FLOOR_WAVE + 1e-3),
            "h_range": [float(hh.min()), float(hh.max())]}


def _summary_flat(col, x, res, viol, ok, seed, runtime, S, C, T, u, n, xe, ye, zm):
    ac, N = col.ac, col.N
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
          x0=None, return_x=False, verbose=False, stop_after=2, tip_model="abs", wave=None):
    """Solve for the best converged cycle among the seeded initial guesses.

    tip_model "abs" or "geometric" (wingtip height, see Collocation._path). wave = {a, wavelength,
    to_deg} switches to the wave-synchronized mode (heights above the moving surface, wave wind,
    theta periodicity with m taken from each initial guess); x0 then needs the extra phi0 slot.
    """
    col = Collocation(ac, prof, mode, cycle, n, u10, chi, h_min, h_tip_min, v_factor,
                      tip_model, wave)
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
        col.set_m(xs)
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


# ---- roughness options and the self-consistent minimum wind (check K-007)
def profile_for(u10, kind="log", roughness="charnock", z0=None, knee=7.5):
    """Profile at u10. roughness "charnock": z0 from sim.seawind.Wind at this u10; "fixed_z0": z0."""
    if roughness == "fixed_z0":
        if z0 is None:
            raise ValueError("fixed_z0 needs z0")
        z0v = float(z0)
    elif roughness == "charnock":
        from sim.seawind import Wind
        z0v = Wind(float(u10), 270.0).z0
    else:
        raise ValueError(roughness)
    if kind == "knee":
        return Profile(z0v, min(10.0, knee), knee)
    if kind != "log":
        raise ValueError(kind)
    return Profile(z0v, 10.0)


def _scaled_fixed_point(K, kind, knee, u_guess):
    """u with u = K ln(zref / z0(u)) (the minimum u10 scales with ln(zref / z0))."""
    from scipy.optimize import brentq
    zref = min(10.0, knee) if kind == "knee" else 10.0
    from sim.seawind import Wind

    def g(u):
        return u - K * np.log(zref / Wind(float(u), 270.0).z0)

    try:
        return float(brentq(g, 0.05, 80.0, xtol=1e-6))
    except ValueError:
        return float(u_guess)


def solve_min_wind(ac, z0_mode="fixed8", roughness="charnock", z0=None, u10_ref=8.0,
                   kind="log", knee=7.5, tol_u10=0.01, max_outer=8, return_x=False,
                   x0=None, seeds=(0, 1, 2), verbose=False, **kw):
    """Minimum-wind cycle with a chosen roughness treatment. kw go to solve (cycle, n, h_min,
    tip_model, wave, ...).

    z0_mode "fixed8": z0 at u10 = u10_ref (today's behaviour). "self_consistent": Charnock z0 at
    the u10 the solution finds, by an outer fixed point. Each outer step is a direct re-solve,
    warm-started from the previous one. The next trial u10 is predicted with the scaling
    u10_min = K ln(zref / z0) (K from the last solve); the prediction is only a start, the loop
    ends when a direct solve returns u10 within tol_u10 of the u10 that set its z0. With
    roughness "fixed_z0" there is nothing to iterate.
    """
    kw.pop("mode", None)
    if z0_mode not in ("fixed8", "self_consistent"):
        raise ValueError(z0_mode)

    def one(u_for_z0, xs):
        prof = profile_for(u_for_z0, kind, roughness, z0, knee)
        if xs is not None:
            r = solve(ac, prof, "min_wind", x0=xs, return_x=True, **kw)
            if r[0]["success"]:
                return r, prof
        return solve(ac, prof, "min_wind", seeds=seeds, return_x=True, x0=None, **kw), prof

    if z0_mode == "fixed8" or roughness == "fixed_z0":
        (info, x, col), prof = one(u10_ref, x0)
        info.update({"z0": prof.z0, "z0_mode": z0_mode, "roughness": roughness,
                     "outer_iterations": 1, "outer_converged": True})
        return (info, x, col) if return_x else info
    u_cur, xs, hist, predicted = float(u10_ref), x0, [], None
    info = x = col = None
    for it in range(max_outer):
        (info, x, col), prof = one(u_cur, xs)
        hist.append({"u10_for_z0": u_cur, "z0": prof.z0, "u10_found": info["u10"],
                     "success": info["success"], "scaling_predicted": predicted})
        if not info["success"]:
            break
        if abs(info["u10"] - u_cur) < tol_u10:
            break
        xs = x
        K = info["u10"] / prof.L
        predicted = _scaled_fixed_point(K, kind, knee, info["u10"])
        u_cur = predicted
    done = bool(info["success"] and abs(info["u10"] - hist[-1]["u10_for_z0"]) < tol_u10)
    err = None
    # error of the first scaling prediction against the direct re-solve it started
    if len(hist) >= 2 and hist[1]["scaling_predicted"] is not None:
        err = abs(hist[1]["u10_found"] - hist[1]["u10_for_z0"])
    info.update({"z0": hist[-1]["z0"], "z0_mode": z0_mode, "roughness": roughness,
                 "outer_iterations": len(hist), "outer_converged": done,
                 "outer_history": hist, "scaling_error_first_step": err})
    if verbose:
        print("outer", [(round(h["u10_for_z0"], 3), round(h["u10_found"], 3)) for h in hist],
              flush=True)
    return (info, x, col) if return_x else info


def main(argv=None):
    import argparse
    ap = argparse.ArgumentParser(description="Dynamic soaring collocation optimizer")
    ap.add_argument("--wave-study", action="store_true",
                    help="flat-sea references and wave-synchronized cycles -> sim/out/dsopt_wave.json")
    ap.add_argument("--wave-study2", action="store_true",
                    help="second pass: midpoint clearance, measured decay, steep waves with shelter")
    ap.add_argument("--nodes", type=int, default=30)
    ap.add_argument("--workers", type=int, default=7)
    args = ap.parse_args(argv)
    if args.wave_study:
        from sim.formstudy import wave_study
        wave_study(n=args.nodes, workers=args.workers)
    elif args.wave_study2:
        from sim.formstudy import wave_study2
        wave_study2(n=args.nodes, workers=args.workers)
    else:
        ap.print_help()


if __name__ == "__main__":
    main()
