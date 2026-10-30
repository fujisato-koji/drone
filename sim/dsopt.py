"""Direct collocation of periodic dynamic soaring cycles (point mass, height-only wind).

States s = [x, y, z, V, gamma, psi], controls (cl, mu), wind W = (u10 f(z), 0, 0).
Hermite-Simpson collocation (compressed form, no midpoint variables), SLSQP with an
analytic constraint Jacobian (the node Jacobians of the dynamics come from complex step).
"""
import time

import numpy as np
from scipy.optimize import minimize

from sim.aircraft import G, ground_effect_phi
from sim.record import value

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


def thrust_max(ac):
    """T_max = value("prop.thrust_max_per_weight", 0.5) m g (N)."""
    return value("prop.thrust_max_per_weight", 0.5) * ac.mass * G


def prop_efficiency():
    """eta_p eta_m: value("prop.prop_eff", 0.60) times value("prop.motor_eff", 0.75) (motor + ESC)."""
    return value("prop.prop_eff", 0.60) * value("prop.motor_eff", 0.75)


def folded_dcd(ac):
    """Delta CD0 of the folded propeller, value("prop.folded_drag_cda_m2", 3.0e-4) / S."""
    return value("prop.folded_drag_cda_m2", 3.0e-4) / ac.wing_area


def folded_aircraft(ac):
    """Copy of ac with the folded propeller drag in cd0 (the whole cycle, motor off)."""
    from dataclasses import replace
    return replace(ac, cd0=ac.cd0 + folded_dcd(ac))


def _drag(ac, q, cl, mu, h, ge, th, psi=None, ex=0.0, ey=0.0):
    """Drag. ge: induced drag times ground_effect_phi at the height h above the surface (slope
    ex, ey, heading psi). th is not None (motor model): the propeller is folded unless it
    thrusts, cd0 + Delta CD0 (1 - T / (T + 0.01 m g)), a smooth switch that is 1 at T = 0 and
    1/2 at T = 1 % of the weight. Without both the expression is the original one."""
    cd0 = ac.cd0
    if th is not None:
        cd0 = cd0 + folded_dcd(ac) * (1.0 - th / (th + 0.01 * ac.mass * G))
    if ge:
        K = ground_effect_phi(h, ac.span, mu, 0.0 if psi is None else psi, ex, ey)
        D = q * (cd0 + ac.k_induced * K * cl * cl)
    else:
        D = q * (cd0 + ac.k_induced * cl * cl)
    if ac.has_polar:   # profile-drag terms cdp1 CL + cdp2 CL^2 + cdp8 CL^8 (exact in the complex step)
        D = D + q * ac.profile_extra(cl)
    return D


def derivs(S, C, ac, U, Up, ge=False, th=None):
    """State derivative for states S (...,6), controls C (...,2), wind U and dU/dz.

    ge: ground effect on the induced drag (flat sea, height z). th: thrust (N) along the
    air-relative velocity, same shape as V (None: no motor)."""
    V, g, p = S[..., 3], S[..., 4], S[..., 5]
    cl, mu = C[..., 0], C[..., 1]
    cg, sg, cp, sp = np.cos(g), np.sin(g), np.cos(p), np.sin(p)
    q = 0.5 * ac.rho * V * V * ac.wing_area
    L = q * cl
    D = _drag(ac, q, cl, mu, S[..., 2], ge, th, p)
    m = ac.mass
    dz = V * sg
    aw = Up * dz  # a_W along x
    out = np.empty(S.shape, dtype=np.result_type(S, C, U, Up))
    out[..., 0] = V * cg * cp + U
    out[..., 1] = V * cg * sp
    out[..., 2] = dz
    out[..., 3] = -D / m - G * sg - aw * cg * cp
    if th is not None:
        out[..., 3] = out[..., 3] + th / m
    out[..., 4] = (L * np.cos(mu) / m - G * cg + aw * sg * cp) / V
    out[..., 5] = (L * np.sin(mu) / m + aw * sp) / (V * cg)
    return out


def dyn(S, C, ac, prof, u10, ge=False, th=None):
    z = S[..., 2]
    return derivs(S, C, ac, u10 * prof.f(z), u10 * prof.fp(z), ge, th)


_EPS = 1e-30


def dyn_jac(A, ac, prof, u10, ge=False, th=None):
    """f and Jacobian by complex step. A (n,8)=[S,C]. Returns f (n,6), J (n,6,9) (col 8: u10).
    With th (n,) the thrust is one more column: J (n,6,10), col 9 = d f / d th."""
    n = A.shape[0]
    nc = 10 if th is not None else 9
    P = np.repeat(A[None].astype(complex), nc, axis=0)  # (nc,n,8)
    u = np.full((nc, n), u10, dtype=complex)
    for j in range(8):
        P[j, :, j] += 1j * _EPS
    u[8] += 1j * _EPS
    tt = None
    if th is not None:
        tt = np.repeat(np.asarray(th, dtype=complex)[None], nc, axis=0)
        tt[9] += 1j * _EPS
    F = derivs(P[..., :NS], P[..., NS:], ac, u * prof.f(P[..., 2]), u * prof.fp(P[..., 2]), ge, tt)
    J = np.moveaxis(F.imag / _EPS, 0, -1)  # (n,6,nc)
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


class WingShape:
    """A bent wing for the wingtip-height constraint (skim study, D-018, docs/02-form.md#wingtip-skim).

    Each half wing is two straight panels of the flat span b: the inner panel from the root to the
    break at the fraction eta of the half span s = b/2, at inner_deg to the wing's reference plane
    (positive tip up), and the outer panel from the break to the tip at outer_deg. Plain dihedral
    is inner = outer; the bent-up outer panel only (polyhedral, or a reversed gull) is inner 0.
    At a bank |mu| and a flight-path angle gamma the depths below the centre are
        break  d_b = s cos(gamma) eta sin(|mu| - G_i)
        tip    d_t = s cos(gamma) (eta sin(|mu| - G_i) + (1 - eta) sin(|mu| - G_o))
    (body axes taken as wind axes, the angle of attack neglected). The vertical share of the panel
    lift is kappa = (1 - f_o) cos G_i + f_o cos G_o with f_o the share of an elliptic loading
    outboard of the break, 1 - (2/pi)(eta sqrt(1 - eta^2) + asin eta); the panels then fly at
    CL / kappa, so the induced and profile drag at a given lift follow (shaped_aircraft)."""

    def __init__(self, inner_deg=0.0, outer_deg=0.0, eta=1.0):
        self.gi, self.go = np.radians(float(inner_deg)), np.radians(float(outer_deg))
        self.eta = float(eta) if inner_deg != outer_deg else 1.0
        if not 0.0 < self.eta <= 1.0:
            raise ValueError(eta)

    @classmethod
    def dihedral(cls, deg):
        return cls(deg, deg, 1.0)

    @classmethod
    def outer_panel(cls, deg, eta):
        return cls(0.0, deg, eta)

    @property
    def outer_share(self):
        e = self.eta
        return 1.0 - 2.0 / np.pi * (e * np.sqrt(max(1.0 - e * e, 0.0)) + np.arcsin(e))

    @property
    def kappa(self):
        f = self.outer_share
        return float((1.0 - f) * np.cos(self.gi) + f * np.cos(self.go))

    def low_points(self, s, absmu, gam):
        """[(depth, d depth / d|mu|, d depth / d gamma)] of the tip and, for a bent wing, the break."""
        cg, sg = np.cos(gam), np.sin(gam)
        e = self.eta
        a_i, a_o = absmu - self.gi, absmu - self.go
        dep_t = e * np.sin(a_i) + (1.0 - e) * np.sin(a_o)
        der_t = e * np.cos(a_i) + (1.0 - e) * np.cos(a_o)
        out = [(s * cg * dep_t, s * cg * der_t, -s * sg * dep_t)]
        if e < 1.0:
            out.append((s * cg * e * np.sin(a_i), s * cg * e * np.cos(a_i), -s * sg * e * np.sin(a_i)))
        return out

    def describe(self):
        return {"inner_deg": float(np.degrees(self.gi)), "outer_deg": float(np.degrees(self.go)),
                "eta": self.eta, "kappa": self.kappa, "outer_share": float(self.outer_share)}


def shaped_aircraft(ac, shape):
    """Copy of ac whose lift coefficient is the vertical (effective) one, CL_e = kappa CL_panel:
    cl_max and cl_min times kappa, the induced factor over kappa^2 (oswald times kappa^2) and the
    profile terms at CL_e / kappa (cdp1 / kappa, cdp2 / kappa^2, cdp8 / kappa^8)."""
    from dataclasses import replace
    k = shape.kappa
    return replace(ac, oswald=ac.oswald * k * k, cl_max=ac.cl_max * k, cl_min=ac.cl_min * k,
                   cdp1=ac.cdp1 / k, cdp2=ac.cdp2 / k ** 2, cdp8=ac.cdp8 / k ** 8)


def _tip_heights(col, hh, mu, gam):
    """Lowest wing point above the surface for the replays: hh - (b/2) sin|mu| cos gamma for the flat
    wing or the lowest of the WingShape points."""
    s = 0.5 * col.ac.span
    if getattr(col, "shape", None) is None:
        return hh - s * np.sin(np.abs(mu)) * np.cos(gam)
    return hh - np.max([d for d, _, _ in col.shape.low_points(s, np.abs(mu), gam)], axis=0)


class WaveParams:
    """One deep-water wave component: a (m), wavelength (m), to_deg (math-frame direction of travel).

    The frame is the optimizer's: the mean wind blows toward +x, angles counter-clockwise from +x,
    so to_deg = 0 is a wave running with the wind and 90 a wave running across it.

    decay = (a_u, a_w): horizontal / vertical perturbation decay exp(-a k h) (default (1, 1)).
    shelter: lee-side sheltering (see wave_wind_accel). midpoints: also constrain h and the tip
    height at the Hermite-Simpson midpoints (default True).
    u_scale, w_scale multiply the horizontal / vertical wave-coherent wind (default 1.0; K-008
    used w_scale 0.6 to match the vertical wind measured by P64).
    """

    def __init__(self, a, wavelength, to_deg, decay=(1.0, 1.0), shelter=False, midpoints=True,
                 u_scale=1.0, w_scale=1.0):
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
        self.u_scale, self.w_scale = float(u_scale), float(w_scale)
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
    Pu = wv.u_scale * base * np.exp(-wv.decay[0] * wv.k * h)
    Pw = wv.w_scale * base * np.exp(-wv.decay[1] * wv.k * h)
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


def derivs_wave(S, C, ac, prof, wv, tt, u10, phi0, ge=False, th=None):
    """State derivative in the wave mode (z is the absolute height, wind acts in x, y and z).

    ge: ground effect at the height above the local surface z - a cos(theta). th: thrust (N)."""
    V, g, p = S[..., 3], S[..., 4], S[..., 5]
    cl, mu = C[..., 0], C[..., 1]
    cg, sg, cp, sp = np.cos(g), np.sin(g), np.cos(p), np.sin(p)
    q = 0.5 * ac.rho * V * V * ac.wing_area
    L = q * cl
    hs, ex, ey = None, 0.0, 0.0
    if ge:
        thw = wv.k * (wv.cw * S[..., 0] + wv.sw * S[..., 1]) - wv.om * tt + phi0
        hs = S[..., 2] - wv.a * np.cos(thw)
        ex, ey = -wv.a * wv.k * wv.cw * np.sin(thw), -wv.a * wv.k * wv.sw * np.sin(thw)
    D = _drag(ac, q, cl, mu, hs, ge, th, p, ex, ey)
    m = ac.mass
    (Wx, Wy, Wz), (ax, ay, az) = wave_wind_accel(S, wv, prof, tt, u10, phi0)
    out = np.empty(S.shape, dtype=np.result_type(S, C, Wx, ax))
    out[..., 0] = V * cg * cp + Wx
    out[..., 1] = V * cg * sp + Wy
    out[..., 2] = V * sg + Wz
    out[..., 3] = -D / m - G * sg - (ax * cg * cp + ay * cg * sp + az * sg)
    if th is not None:
        out[..., 3] = out[..., 3] + th / m
    out[..., 4] = (L * np.cos(mu) / m - G * cg + (ax * sg * cp + ay * sg * sp - az * cg)) / V
    out[..., 5] = (L * np.sin(mu) / m + ax * sp - ay * cp) / (V * cg)
    return out


def dyn_jac_wave(A, tt, u10, phi0, ac, prof, wv, ge=False, th=None):
    """f and Jacobian by complex step. A (n,8), tt (n,). J (n,6,10): S, C, u10, phi0.

    The time derivative is -om * (phi0 column): t and phi0 enter only as phi0 - om t.
    With th (n,) the thrust is one more column: J (n,6,11), col 10 = d f / d th.
    """
    n = A.shape[0]
    nc = 11 if th is not None else 10
    P = np.repeat(A[None].astype(complex), nc, axis=0)
    u = np.full((nc, n), u10, dtype=complex)
    ph = np.full((nc, n), phi0, dtype=complex)
    for j in range(8):
        P[j, :, j] += 1j * _EPS
    u[8] += 1j * _EPS
    ph[9] += 1j * _EPS
    tq = None
    if th is not None:
        tq = np.repeat(np.asarray(th, dtype=complex)[None], nc, axis=0)
        tq[10] += 1j * _EPS
    F = derivs_wave(P[..., :NS], P[..., NS:], ac, prof, wv, np.broadcast_to(tt, (nc, n)), u, ph,
                    ge, tq)
    J = np.moveaxis(F.imag / _EPS, 0, -1)
    return F[8].real, J


class Collocation:
    def __init__(self, ac, prof, mode="min_wind", cycle="travel", n=40, u10=None, chi=np.pi / 2,
                 h_min=1.0, h_tip_min=0.3, v_factor=1.1, tip_model="abs", wave=None,
                 midpoints=False, rate_limits=False, n_min=None, n_frac=1.0, mu_frac=1.0,
                 bank_rate_max_deg_s=None, cl_rate_max_per_s=None, phase_pin="theta",
                 rate_model="constant", roll_pb2v=None, ground_effect=False, motor=False,
                 v_travel_min=None, folded_prop=False, wing_shape=None):
        """wing_shape: a WingShape (tip_model "geometric" only): the wingtip constraint holds the
        lowest wing points of the bent wing and the aircraft is shaped_aircraft(ac, wing_shape).
        folded_prop (motor=False only): an airframe with a folded propeller, Delta CD0 =
        prop.folded_drag_cda_m2 / S added to cd0 over the whole cycle (with motor=True the smooth
        switch of _drag applies instead). ground_effect: induced drag times ground_effect_phi (default off).
        motor: thrust T in [0, T_max] per node as a third control (kept in extra variables after
        the others, see thrust()); mode "min_power" (needs motor and u10) minimises the cycle
        average electric power sum(w T V) / (eta_p eta_m) (trapezoid in the nodes); in a travel
        cycle v_travel_min (m/s, default 0) bounds the mean speed along chi from below."""
        if rate_model not in ("constant", "pb2v"):
            raise ValueError(rate_model)
        if mode not in ("min_wind", "max_speed", "min_power"):
            raise ValueError(mode)
        if mode == "min_power" and not motor:
            raise ValueError("min_power needs motor=True")
        if cycle not in ("travel", "loiter"):
            raise ValueError(cycle)
        if tip_model not in ("abs", "geometric"):
            raise ValueError(tip_model)
        if phase_pin not in ("theta", "gamma"):
            raise ValueError(phase_pin)
        if mode in ("max_speed", "min_power") and u10 is None:
            raise ValueError(mode + " needs u10")
        self.ge, self.motor = bool(ground_effect), bool(motor)
        if folded_prop and not motor:
            ac = folded_aircraft(ac)
        if wing_shape is not None and tip_model != "geometric":
            raise ValueError("wing_shape needs tip_model geometric")
        self.shape = wing_shape
        if wing_shape is not None:
            ac = shaped_aircraft(ac, wing_shape)
        self.v_travel_min = None if v_travel_min is None else float(v_travel_min)
        self.ac, self.prof, self.mode, self.cycle, self.N = ac, prof, mode, cycle, n
        self.u_fixed = u10
        self.chi, self.h_min, self.h_tip = chi, h_min, h_tip_min
        self.tip_model = tip_model
        self.wv = None if wave is None else (wave if isinstance(wave, WaveParams)
                                             else WaveParams(**wave))
        # midpoints: flat mode keyword; in the wave mode WaveParams.midpoints decides
        self.mid = bool(midpoints) if self.wv is None else self.wv.midpoints
        self.rate_limits = bool(rate_limits)
        from sim.record import value
        self.p_max = np.radians(float(bank_rate_max_deg_s) if bank_rate_max_deg_s is not None
                                else value("ctrl.bank_rate_max_deg_s", 150.0))
        self.rate_model = rate_model
        self.k_roll = float(roll_pb2v) if roll_pb2v is not None else value("ctrl.roll_pb2v", 0.08)
        self.r_max = (float(cl_rate_max_per_s) if cl_rate_max_per_s is not None
                      else (value("ctrl.cl_rate_max_per_s", 1.0) if rate_model == "pb2v"
                            else value("ctrl.const_cl_rate_max_per_s", 2.0)))
        self.n_min = None if n_min is None else float(n_min)
        self.n_frac, self.mu_frac = float(n_frac), float(mu_frac)
        self.phase_pin = phase_pin
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
        self.iTh = None
        if self.motor:  # thrust per node, scaled by m g (N), after all other variables
            self.iTh = self.nv
            self.sc = np.concatenate([self.sc, np.full(n, ac.mass * G)])
            self.nv += n
        self.eta = prop_efficiency()
        self._key, self._val = None, None

    # ---- packing
    def pack(self, S, C, T, u10, phi0=0.0, thrust=None):
        tail = [T, u10] + ([phi0] if self.wv is not None else [])
        parts = [S.ravel(), C.ravel(), tail]
        if self.motor:
            parts.append(np.zeros(self.N) if thrust is None else np.asarray(thrust, dtype=float))
        return np.concatenate(parts) / self.sc

    def thrust(self, x):
        """Node thrust (N); zeros without the motor."""
        if not self.motor:
            return np.zeros(self.N)
        return np.asarray(x[self.iTh:self.iTh + self.N]) * self.sc[self.iTh:self.iTh + self.N]

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
                             np.tile([ac.cl_min, -ac.mu_max * self.mu_frac], N), [1.0, 0.5]])
        hi = np.concatenate([np.tile([1e4, 1e4, 200.0, 80.0, 1.2, 1e3], N),
                             np.tile([ac.cl_max, ac.mu_max * self.mu_frac], N), [60.0, 60.0]])
        if self.wv is not None:
            lo, hi = np.append(lo, -100.0), np.append(hi, 100.0)
            lo[self.iU] = U10_FLOOR_WAVE  # a wave can carry the cycle with (almost) no mean wind
        if self.motor:
            lo = np.concatenate([lo, np.zeros(N)])
            hi = np.concatenate([hi, np.full(N, thrust_max(ac))])
        lo[0], hi[0], lo[1], hi[1] = 0.0, 0.0, 0.0, 0.0  # x0 = y0 = 0
        if self.mode in ("max_speed", "min_power"):
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
        Th = self.thrust(x) if self.motor else None
        Thm = None if Th is None else 0.5 * (Th[:-1] + Th[1:])
        F, J = dyn_jac_wave(A, np.arange(N) * h, u, phi, ac, prof, wv, self.ge, Th)
        Fk, Fk1 = F[:-1], F[1:]
        Sm = 0.5 * (S[:-1] + S[1:]) + h / 8.0 * (Fk - Fk1)
        Cm = 0.5 * (C[:-1] + C[1:])
        Fm, Jm = dyn_jac_wave(np.concatenate([Sm, Cm], axis=1), (np.arange(M) + 0.5) * h,
                              u, phi, ac, prof, wv, self.ge, Thm)
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
        if self.motor:
            dThk, dThk1 = self._thrust_blocks(h, Jk[:, :, 10], Jk1[:, :, 10], Jm[:, :, 10], Jms)
        for k in range(M):
            r = slice(k * NS, (k + 1) * NS)
            Jac[r, k * NS:(k + 1) * NS] = dSk[k]
            Jac[r, (k + 1) * NS:(k + 2) * NS] = dSk1[k]
            Jac[r, N * NS + k * NC:N * NS + (k + 1) * NC] = dCk[k]
            Jac[r, N * NS + (k + 1) * NC:N * NS + (k + 2) * NC] = dCk1[k]
            Jac[r, self.iT] = dT[k]
            Jac[r, self.iU] = dU[k]
            Jac[r, self.iP] = dP[k]
            if self.motor:
                Jac[r, self.iTh + k] = dThk[k]
                Jac[r, self.iTh + k + 1] = dThk1[k]
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
            if self.motor:
                Jsm[k, :, self.iTh + k] = h / 8.0 * Jk[k, :, 10]
                Jsm[k, :, self.iTh + k + 1] = -h / 8.0 * Jk1[k, :, 10]
        self._mid = (Sm, Jsm)
        Jac = Jac * self.sc[None, :] * self.rowsc[:, None]
        return d.ravel() * self.rowsc, Jac

    @staticmethod
    def _thrust_blocks(h, Jtk, Jtk1, Jtm, Jms):
        """d(defect)/d(thrust at node k, k+1), physical units; the midpoint thrust is the node
        average and enters the midpoint state through the Hermite-Simpson interpolation."""
        dk = -h / 6.0 * (Jtk + 2 * Jtm + 4 * np.einsum("kij,kj->ki", Jms, h / 8.0 * Jtk))
        dk1 = -h / 6.0 * (Jtk1 + 2 * Jtm + 4 * np.einsum("kij,kj->ki", Jms, -h / 8.0 * Jtk1))
        return dk, dk1

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
        Th = self.thrust(x) if self.motor else None
        Thm = None if Th is None else 0.5 * (Th[:-1] + Th[1:])
        F, J = dyn_jac(A, ac, prof, u, self.ge, Th)
        Fk, Fk1 = F[:-1], F[1:]
        Sm = 0.5 * (S[:-1] + S[1:]) + h / 8.0 * (Fk - Fk1)
        Cm = 0.5 * (C[:-1] + C[1:])
        Fm, Jm = dyn_jac(np.concatenate([Sm, Cm], axis=1), ac, prof, u, self.ge, Thm)
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
        if self.mid:  # midpoint state Sm and its physical Jacobian (as in the wave mode)
            Jsm = np.zeros((M, NS, self.nv))
            for k in range(M):
                Jsm[k, :, k * NS:(k + 1) * NS] = 0.5 * I6 + h / 8.0 * Jk[k, :, :NS]
                Jsm[k, :, (k + 1) * NS:(k + 2) * NS] = 0.5 * I6 - h / 8.0 * Jk1[k, :, :NS]
                Jsm[k, :, N * NS + k * NC:N * NS + (k + 1) * NC] = h / 8.0 * Jk[k, :, NS:8]
                Jsm[k, :, N * NS + (k + 1) * NC:N * NS + (k + 2) * NC] = -h / 8.0 * Jk1[k, :, NS:8]
                Jsm[k, :, self.iT] = (Fk[k] - Fk1[k]) / 8.0 / (N - 1)
                Jsm[k, :, self.iU] = h / 8.0 * (Jk[k, :, 8] - Jk1[k, :, 8])
                if self.motor:
                    Jsm[k, :, self.iTh + k] = h / 8.0 * Jk[k, :, 9]
                    Jsm[k, :, self.iTh + k + 1] = -h / 8.0 * Jk1[k, :, 9]
            self._mid = (Sm, Jsm)
        dU = -h / 6.0 * (Jk[:, :, 8] + 4 * (Jmu + np.einsum("kij,kj->ki", Jms,
                                                              h / 8.0 * (Jk[:, :, 8] - Jk1[:, :, 8])))
                         + Jk1[:, :, 8])
        Jac = np.zeros((M * NS, self.nv))
        if self.motor:
            dThk, dThk1 = self._thrust_blocks(h, Jk[:, :, 9], Jk1[:, :, 9], Jm[:, :, 9], Jms)
        for k in range(M):
            r = slice(k * NS, (k + 1) * NS)
            Jac[r, k * NS:(k + 1) * NS] = dSk[k]
            Jac[r, (k + 1) * NS:(k + 2) * NS] = dSk1[k]
            Jac[r, N * NS + k * NC:N * NS + (k + 1) * NC] = dCk[k]
            Jac[r, N * NS + (k + 1) * NC:N * NS + (k + 2) * NC] = dCk1[k]
            Jac[r, self.iT] = dT[k]
            Jac[r, self.iU] = dU[k]
            if self.motor:
                Jac[r, self.iTh + k] = dThk[k]
                Jac[r, self.iTh + k + 1] = dThk1[k]
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
        wv = self.wv
        if wv is not None and wv.a > 0.0 and self.phase_pin == "theta":
            # phase: theta(0) = 0 (x0 = y0 = 0, t = 0, so theta(0) = phi0); gamma stays free
            rows.append(({self.iP: 1.0}, 0.0))
        else:
            rows.append(({idx(0, 4): 1.0}, 0.0))  # phase: gamma = 0 at the first node
        if self.rate_limits:  # controls are continuous across the period seam
            for j in range(NC):
                rows.append(({N * NS + (N - 1) * NC + j: 1.0, N * NS + j: -1.0}, 0.0))
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
        elif self.shape is not None:
            tips, Jts = [], []
            for dep, ddmu, ddg in self.shape.low_points(0.5 * ac.span, absmu, gam):
                Jx = Jh.copy()
                Jx[i, N * NS + i * NC + 1] = -ddmu * dabs
                Jx[i, i * NS + 4] = -ddg * sc[i * NS + 4]
                tips.append(hh - dep - self.h_tip)
                Jts.append(Jx)
            tip, Jt = np.concatenate(tips), np.vstack(Jts)
        else:
            sa, ca = np.sin(absmu), np.cos(absmu)
            tip = hh - 0.5 * ac.span * sa * np.cos(gam) - self.h_tip
            Jt[i, N * NS + i * NC + 1] = -0.5 * ac.span * ca * np.cos(gam) * dabs
            Jt[i, i * NS + 4] = 0.5 * ac.span * sa * np.sin(gam) * sc[i * NS + 4]
        f = [(self.n_frac * ac.n_max - n) / ac.n_max]
        J = [-Jn / ac.n_max]
        if self.n_min is not None:
            f.append((n - self.n_min) / ac.n_max)
            J.append(Jn / ac.n_max)
        if self.wv is not None:
            f.append(hh - self.h_min)
            J.append(Jh)
        f.append(tip)
        J.append(Jt)
        if self.mid:
            fm, Jm_ = self._path_mid(x)
            f.append(fm)
            J.append(Jm_)
            if self.n_min is not None:
                fn, Jn_ = self._n_mid(x)
                f.append(fn)
                J.append(Jn_)
        if self.rate_limits:
            fr, Jr = self._rates(x)
            f.append(fr)
            J.append(Jr)
        if self.mode == "min_power" and self.cycle == "travel" and self.v_travel_min is not None:
            # progress along chi: (c x_end + s y_end - v_req T) / 100 >= 0 (linear in x_end, y_end, T)
            c, s_ = np.cos(self.chi), np.sin(self.chi)
            v = x * sc
            ie, je = (N - 1) * NS, (N - 1) * NS + 1
            Jv = np.zeros((1, self.nv))
            Jv[0, ie], Jv[0, je] = c * sc[ie] / 100.0, s_ * sc[je] / 100.0
            Jv[0, self.iT] = -self.v_travel_min * sc[self.iT] / 100.0
            f.append(np.array([(c * v[ie] + s_ * v[je] - self.v_travel_min * v[self.iT]) / 100.0]))
            J.append(Jv)
        return np.concatenate(f), np.vstack(J)

    def _n_mid(self, x):
        """n >= n_min at the Hermite-Simpson midpoints (state Sm, control = node average)."""
        self._defects(x)
        Sm, Jsm = self._mid
        _, C, _, _ = self.unpack(x)
        ac, N = self.ac, self.N
        M = N - 1
        kk = np.arange(M)
        K = 0.5 * ac.rho * ac.wing_area / (ac.mass * G)
        clm = 0.5 * (C[:-1, 0] + C[1:, 0])
        Vm = Sm[:, 3]
        dcl = np.zeros((M, self.nv))
        dcl[kk, N * NS + kk * NC] = 0.5
        dcl[kk, N * NS + (kk + 1) * NC] = 0.5
        n = K * Vm * Vm * clm
        dn = K * ((2.0 * Vm * clm)[:, None] * Jsm[:, 3, :] + (Vm * Vm)[:, None] * dcl)
        return (n - self.n_min) / ac.n_max, dn * self.sc[None, :] / ac.n_max

    def _rates(self, x):
        """|mu_{k+1} - mu_k| <= p_max dt and |cl_{k+1} - cl_k| <= r_max dt, dt = T / (N - 1).

        Linear in the controls and in T (the period). Each bound is two rows, in seconds:
        dt -/+ d / rate >= 0. The controls are piecewise linear between nodes (the midpoint
        control is the node average), so the node pairs are all there is.
        """
        _, C, T, _ = self.unpack(x)
        N = self.N
        M = N - 1
        kk = np.arange(M)
        dt = T / (N - 1)
        fs, Js = [], []
        if self.rate_model == "pb2v":
            # |dmu| <= (2 k V / b) dt with V the slower node of the interval (piecewise smooth:
            # the Jacobian takes the active node); rows in rad: p dt -/+ d >= 0
            ac = self.ac
            V = self.unpack(x)[0][:, 3]
            slow = V[1:] < V[:-1]
            Vm = np.where(slow, V[1:], V[:-1])
            pl = 2.0 * self.k_roll * Vm / ac.span
            d = C[1:, 1] - C[:-1, 1]
            for sign in (1.0, -1.0):
                Jr = np.zeros((M, self.nv))
                Jr[kk, N * NS + kk * NC + 1] = sign
                Jr[kk, N * NS + (kk + 1) * NC + 1] = -sign
                Jr[:, self.iT] = pl / (N - 1)
                vi = np.where(slow, kk + 1, kk)
                Jr[kk, vi * NS + 3] = 2.0 * self.k_roll / ac.span * dt
                fs.append(pl * dt - sign * d)
                Js.append(Jr * self.sc[None, :])
            comps = ((0, self.r_max),)
        else:
            comps = ((1, self.p_max), (0, self.r_max))
        for comp, rate in comps:
            d = C[1:, comp] - C[:-1, comp]
            for sign in (1.0, -1.0):
                Jr = np.zeros((M, self.nv))
                Jr[kk, N * NS + kk * NC + comp] = sign / rate
                Jr[kk, N * NS + (kk + 1) * NC + comp] = -sign / rate
                Jr[:, self.iT] = 1.0 / (N - 1)
                fs.append(dt - sign * d / rate)
                Js.append(Jr * self.sc[None, :])
        return np.concatenate(fs), np.vstack(Js)

    def _path_mid(self, x):
        """h >= h_min and the wingtip height at the Hermite-Simpson midpoints (flat and wave mode)."""
        self._defects(x)  # fills self._mid for this x
        Sm, Jsm = self._mid
        _, C, T, _ = self.unpack(x)
        wv, ac, N = self.wv, self.ac, self.N
        M = N - 1
        kk = np.arange(M)
        sc = self.sc
        if wv is None:
            hm, dh = Sm[:, 2], Jsm[:, 2, :]
        else:
            thm = (wv.k * (wv.cw * Sm[:, 0] + wv.sw * Sm[:, 1]) - wv.om * (kk + 0.5) * T / (N - 1)
                   + self.phi0(x))
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
        elif self.shape is not None:
            tips, dtips = [], []
            for dep, ddmu, ddg in self.shape.low_points(0.5 * ac.span, absmu, gm):
                tips.append(hm - dep - self.h_tip)
                dtips.append(dh - ddmu[:, None] * dabs * dmu - ddg[:, None] * Jsm[:, 4, :])
            tip, dtip = np.concatenate(tips), np.vstack(dtips)
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

    def power_weights(self):
        """Trapezoid weights of the cycle average over the nodes (sum 1)."""
        w = np.ones(self.N)
        w[0] = w[-1] = 0.5
        return w / (self.N - 1)

    def avg_power(self, x):
        """Cycle average electric power (W): mean of T V / (eta_p eta_m), trapezoid over the nodes."""
        S = self.unpack(x)[0]
        return float(np.sum(self.power_weights() * self.thrust(x) * S[:, 3]) / self.eta)

    def objective(self, x):
        if self.mode == "min_power":
            ac, N, sc = self.ac, self.N, self.sc
            scale = ac.mass * G * 10.0 * self.eta
            w = self.power_weights()
            Th = self.thrust(x)
            iv = np.arange(N) * NS + 3
            V = x[iv] * sc[iv]
            g = np.zeros(self.nv)
            g[self.iTh:self.iTh + N] = w * V / scale * sc[self.iTh:self.iTh + N]
            g[iv] = w * Th / scale * sc[iv]
            return float(np.sum(w * Th * V) / scale), g
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
        th0 = None
        if self.motor:  # a thrust that would hold the glide at the best L/D
            th0 = np.full(N, min(ac.mass * G / ac.best_ld, thrust_max(ac)))
        dx = V * np.cos(g) * np.cos(psi) + u * prof.f(z)
        dy = V * np.cos(g) * np.sin(psi)
        x = np.concatenate([[0.0], np.cumsum(0.5 * (dx[1:] + dx[:-1]) * np.diff(t))])
        y = np.concatenate([[0.0], np.cumsum(0.5 * (dy[1:] + dy[:-1]) * np.diff(t))])
        if self.wv is not None:
            wv = self.wv
            phi0 = rng.uniform(0.0, 2 * np.pi)  # drawn last: the other draws match the flat guess
            if self.phase_pin == "theta" and wv.a > 0.0:
                phi0 = 0.0  # theta(0) = 0 is pinned
            z = z + wv.a * np.cos(wv.k * (wv.cw * x + wv.sw * y) - wv.om * t + phi0)
        else:
            phi0 = 0.0
        S = np.stack([x, y, z, V, g, psi], axis=1)
        return self.pack(S, np.stack([cl, mu], axis=1), T, u, phi0, th0)


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
        F, _ = dyn_jac(A, ac, col.prof, u, col.ge, col.thrust(x) if col.motor else None)
        zm = 0.5 * (S[:-1, 2] + S[1:, 2]) + h / 8 * (F[:-1, 2] - F[1:, 2])
    else:
        F, _ = dyn_jac_wave(A, np.arange(N) * h, u, col.phi0(x), ac, col.prof, col.wv, col.ge,
                            col.thrust(x) if col.motor else None)
        zm = 0.5 * (S[:-1, 2] + S[1:, 2]) + h / 8 * (F[:-1, 2] - F[1:, 2])
    out = _summary_flat(col, x, res, viol, ok, seed, runtime, S, C, T, u, n, xe, ye, zm)
    out.update(control_stats(col, x))
    if col.motor:
        out.update(motor_stats(col, x))
        if col.mode == "min_power":
            out["objective"] = out["power_avg_w"]
    if col.wv is not None:
        out.update(_wave_extra(col, x, S, C, T, u, F))
    return out


def control_stats(col, x, sub=20):
    """Rates of the node controls, n range, and a dense replay of the interval states.

    The replay integrates the dynamics (RK4, sub steps per interval) from each node state with
    the piecewise-linear controls, so it is independent of the Hermite-Simpson midpoint
    interpolation; it reports the lowest height above the surface, the lowest geometric
    wingtip height (z - (b/2) sin|mu| cos gamma) and the largest state mismatch at the next node.
    """
    S, C, T, u = col.unpack(x)
    ac, N, wv = col.ac, col.N, col.wv
    dt = T / (N - 1)
    K = 0.5 * ac.rho * ac.wing_area / (ac.mass * G)
    n = K * S[:, 3] ** 2 * C[:, 0]
    dmu, dcl = np.diff(C[:, 1]), np.diff(C[:, 0])
    phi = col.phi0(x) if wv is not None else 0.0
    Th = col.thrust(x) if col.motor else None

    def f(s, c, t, th=None):
        if wv is None:
            return dyn(s, c, ac, col.prof, u, col.ge, th)
        return derivs_wave(s, c, ac, col.prof, wv, t, u, phi, col.ge, th)

    def height_tip(s, c, t):
        hh = s[:, 2]
        if wv is not None:
            hh = hh - wv.a * np.cos(wv.k * (wv.cw * s[:, 0] + wv.sw * s[:, 1]) - wv.om * t + phi)
        return hh, _tip_heights(col, hh, c[:, 1], s[:, 4])

    M = N - 1
    kt = np.arange(M) * dt
    s = S[:-1].copy()
    hs, tips = [], []
    for j in range(sub + 1):
        fr = j / sub
        c = C[:-1] + (C[1:] - C[:-1]) * fr
        h_, t_ = height_tip(s, c, kt + fr * dt)
        hs.append(h_.min())
        tips.append(t_.min())
        if j == sub:
            break
        hstep = dt / sub
        cm = C[:-1] + (C[1:] - C[:-1]) * (fr + 0.5 / sub)
        cn = C[:-1] + (C[1:] - C[:-1]) * (fr + 1.0 / sub)
        t0_ = kt + fr * dt
        if Th is None:
            tc = tm_ = tn = None
        else:
            dTh = Th[1:] - Th[:-1]
            tc, tm_, tn = (Th[:-1] + dTh * fr, Th[:-1] + dTh * (fr + 0.5 / sub),
                           Th[:-1] + dTh * (fr + 1.0 / sub))
        k1 = f(s, c, t0_, tc)
        k2 = f(s + 0.5 * hstep * k1, cm, t0_ + 0.5 * hstep, tm_)
        k3 = f(s + 0.5 * hstep * k2, cm, t0_ + 0.5 * hstep, tm_)
        k4 = f(s + hstep * k3, cn, t0_ + hstep, tn)
        s = s + hstep / 6.0 * (k1 + 2 * k2 + 2 * k3 + k4)
    Vm = np.minimum(S[1:, 3], S[:-1, 3])
    lim = (2.0 * col.k_roll * Vm / ac.span if col.rate_model == "pb2v"
           else np.full(N - 1, col.p_max))
    ip = int(np.argmax(np.abs(dmu) / dt / lim))
    return {"bank_rate_peak_deg_s": float(np.degrees(abs(dmu[ip]) / dt)),
            "bank_rate_limit_at_peak_deg_s": float(np.degrees(lim[ip])),
            "bank_rate_over_limit_max": float(abs(dmu[ip]) / dt / lim[ip]),
            "pb2v_used_max": float((np.abs(dmu) / dt * ac.span / (2.0 * Vm)).max()),
            "bank_rate_max_deg_s": float(np.degrees(np.abs(dmu).max() / dt)),
            "cl_rate_max_per_s": float(np.abs(dcl).max() / dt),
            "control_seam_jump": [float(abs(C[-1, 0] - C[0, 0])), float(abs(C[-1, 1] - C[0, 1]))],
            "n_min_node": float(n.min()), "n_max_node": float(n.max()),
            "replay_h_min": float(min(hs)), "replay_tip_min": float(min(tips)),
            "replay_closure": float(np.abs(s - S[1:]).max())}


def motor_stats(col, x, sub=20):
    """Electric power of a motor cycle: average (the optimizer's trapezoid and a dense one on the
    piecewise-linear thrust and V), peak, share of time with T > 1e-4 m g, Wh per hour (= the
    average power in W), thrust range. V is linear between nodes in the dense sample."""
    S = col.unpack(x)[0]
    ac, N = col.ac, col.N
    Th, V = col.thrust(x), S[:, 3]
    fr = np.linspace(0.0, 1.0, sub + 1)[None, :]
    Td = (Th[:-1, None] + (Th[1:] - Th[:-1])[:, None] * fr).ravel()
    Vd = (V[:-1, None] + (V[1:] - V[:-1])[:, None] * fr).ravel()
    pd = Td * Vd / col.eta
    avg = col.avg_power(x)
    return {"power_avg_w": avg, "power_avg_dense_w": float(pd.mean()),
            "power_peak_w": float(pd.max()),
            "thrust_on_share": float((Td > 1e-4 * ac.mass * G).mean()),
            "wh_per_hour": avg, "thrust_max_n": float(Th.max()), "thrust_limit_n": thrust_max(ac)}


def dense_replay(col, x, sub=None):
    """K-011 replay: integrate every interval from its start node with the interpolated controls.

    RK4 with sub substeps per interval (at least 20, and at most about 0.02 s each). Reports the
    largest closure error against the next node in height, airspeed (absolute and relative),
    flight-path angle and heading, the dense minimum of the height above the surface and of the
    geometric wingtip height, and the dense maximum of the load factor. replay_pass: closure
    within 0.05 m in height and 1% in airspeed, and dense h and tip not more than 0.02 m below
    their limits (col.h_min, col.h_tip).
    """
    S, C, T, u = col.unpack(x)
    ac, N, wv = col.ac, col.N, col.wv
    dt = T / (N - 1)
    if sub is None:
        sub = max(20, int(np.ceil(dt / 0.02)))
    K = 0.5 * ac.rho * ac.wing_area / (ac.mass * G)
    phi = col.phi0(x) if wv is not None else 0.0

    Th = col.thrust(x)

    def f(s, c, t, th):
        th = th if col.motor else None
        if wv is None:
            return dyn(s, c, ac, col.prof, u, col.ge, th)
        return derivs_wave(s, c, ac, col.prof, wv, t, u, phi, col.ge, th)

    def surf(s, t):
        if wv is None:
            return 0.0
        return wv.a * np.cos(wv.k * (wv.cw * s[:, 0] + wv.sw * s[:, 1]) - wv.om * t + phi)

    kt = np.arange(N - 1) * dt
    s = S[:-1].copy()
    hmin = tmin = np.inf
    nmax = -np.inf
    nmin = np.inf
    for j in range(sub + 1):
        fr = j / sub
        c = C[:-1] + (C[1:] - C[:-1]) * fr
        t_ = kt + fr * dt
        hh = s[:, 2] - surf(s, t_)
        tip = _tip_heights(col, hh, c[:, 1], s[:, 4])
        nn = K * s[:, 3] ** 2 * c[:, 0]
        hmin, tmin = min(hmin, hh.min()), min(tmin, tip.min())
        nmax, nmin = max(nmax, nn.max()), min(nmin, nn.min())
        if j == sub:
            break
        hs = dt / sub
        cm = C[:-1] + (C[1:] - C[:-1]) * (fr + 0.5 / sub)
        cn = C[:-1] + (C[1:] - C[:-1]) * (fr + 1.0 / sub)
        th0 = Th[:-1] + (Th[1:] - Th[:-1]) * fr
        thm = Th[:-1] + (Th[1:] - Th[:-1]) * (fr + 0.5 / sub)
        thn = Th[:-1] + (Th[1:] - Th[:-1]) * (fr + 1.0 / sub)
        k1 = f(s, c, t_, th0)
        k2 = f(s + 0.5 * hs * k1, cm, t_ + 0.5 * hs, thm)
        k3 = f(s + 0.5 * hs * k2, cm, t_ + 0.5 * hs, thm)
        k4 = f(s + hs * k3, cn, t_ + hs, thn)
        s = s + hs / 6.0 * (k1 + 2 * k2 + 2 * k3 + k4)
    err = np.abs(s - S[1:])
    out = {"rc_closure_h": float(err[:, 2].max()), "rc_closure_V": float(err[:, 3].max()),
           "rc_closure_V_rel": float((err[:, 3] / S[1:, 3]).max()),
           "rc_closure_gamma": float(err[:, 4].max()), "rc_closure_psi": float(err[:, 5].max()),
           "rc_h_min": float(hmin), "rc_tip_min": float(tmin), "rc_n_max": float(nmax),
           "rc_n_min": float(nmin), "rc_sub": int(sub)}
    out["replay_pass"] = bool(out["rc_closure_h"] <= 0.05 and out["rc_closure_V_rel"] <= 0.01
                              and hmin >= col.h_min - 0.02 and tmin >= col.h_tip - 0.02)
    return out


def wave_preset_params(name):
    """(w_scale, u_scale, decay) of a calibration preset; None for "model" (WaveParams defaults)."""
    from sim.record import value
    if name == "model":
        return None
    if name not in ("p63", "p64_low"):
        raise ValueError(name)
    w = 0.6 if name == "p64_low" else value("lit.wave_induced_amp_w", 1.07)
    return {"w_scale": float(w), "u_scale": 1.0,
            "decay": (value("lit.wave_induced_decay_u", 0.77),
                      value("lit.wave_induced_decay_w", 0.62))}


def _apply_preset(wave, preset):
    """Fill w_scale, u_scale and decay of a wave dict from the preset unless given explicitly."""
    if wave is None or isinstance(wave, WaveParams) or preset == "model":
        if preset not in ("model", "p63", "p64_low"):
            raise ValueError(preset)
        return wave
    wave = dict(wave)
    for k, v in wave_preset_params(preset).items():
        wave.setdefault(k, v)
    return wave


def _resample(col, x, col2):
    """Start vector for col2 (other node count) from the solution x of col (same T, u10, phi0)."""
    from scipy.interpolate import CubicSpline
    S, C, T, u = col.unpack(x)
    t1 = np.linspace(0.0, T, col.N)
    t2 = np.linspace(0.0, T, col2.N)
    S2 = CubicSpline(t1, S, axis=0)(t2)
    C2 = np.stack([np.interp(t2, t1, C[:, j]) for j in range(NC)], axis=1)
    S2[0, 0] = S2[0, 1] = 0.0
    th2 = np.interp(t2, t1, col.thrust(x)) if col.motor else None
    return col2.pack(S2, C2, T, u, col.phi0(x) if col.wv is not None else 0.0, thrust=th2)


def rotate_start(col, x, delta_deg):
    """Start vector x with the track rotated by delta_deg about the origin (x, y, psi).

    With the wave direction rotated by the same angle the wave phase at every point of the track is
    unchanged (phi0 stays); the wind direction is not rotated, so the result is only a start.
    """
    S, C, T, u = col.unpack(x)
    d = np.radians(delta_deg)
    c, s_ = np.cos(d), np.sin(d)
    S2 = S.copy()
    S2[:, 0] = c * S[:, 0] - s_ * S[:, 1]
    S2[:, 1] = s_ * S[:, 0] + c * S[:, 1]
    S2[:, 5] = S[:, 5] + d
    return col.pack(S2, C, T, u, col.phi0(x) if col.wv is not None else 0.0)


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
                     "shelter": wv.shelter, "midpoints": wv.midpoints,
                     "u_scale": wv.u_scale, "w_scale": wv.w_scale},
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
            "objective": float(u if col.mode == "min_wind" else
                               (res.fun * ac.mass * G * 10.0 if col.mode == "min_power"
                                else -res.fun * 10.0))}


def _better_info(a, b, mode):
    """True if result a beats b: converged first, then the objective, else the violation."""
    if b is None:
        return True
    if a["success"] != b["success"]:
        return a["success"]
    sgn = 1 if mode in ("min_wind", "min_power") else -1
    if a["success"]:
        return a["objective"] * sgn < b["objective"] * sgn
    return a["violation"] < b["violation"]


def _run_col(col, xs, m_offsets, seed, mode, tol, maxiter, verbose, check):
    """Solve one start for every m offset; returns (best candidate, all candidates).

    A candidate is (info, x, col, res). info["converged"]: optimizer and violation test only;
    info["success"] adds the dense replay when check is True.
    """
    t0 = time.time()
    cons = [{"type": "eq", "fun": col.eq_fun, "jac": col.eq_jac},
            {"type": "ineq", "fun": col.ineq_fun, "jac": col.ineq_jac}]
    bnds = col.bounds()
    lo = np.array([b[0] for b in bnds])
    hi = np.array([b[1] for b in bnds])
    xs = np.clip(xs, lo, hi)
    m0 = col.set_m(xs)
    offs = tuple(m_offsets) if (col.wv is not None and col.wv.a > 0.0) else (0,)
    cand, allc = None, []
    for off in offs:
        col.m = m0 + int(off)
        res = minimize(col.objective, xs, jac=True, method="SLSQP", bounds=bnds,
                       constraints=cons, options={"maxiter": maxiter, "ftol": 1e-9})
        viol = col.violation(res.x)
        conv = bool(res.success and viol < tol)
        rc = dense_replay(col, res.x) if (check and conv) else None
        ok = conv and (rc is None or rc["replay_pass"])
        info = _summary(col, res.x, res, viol, ok, seed, time.time() - t0)
        info["converged"] = conv
        if rc is not None:
            info.update(rc)
        info["m_offset"] = int(off)
        if verbose:
            print(seed, off, info["success"], "%.2e" % viol, info["u10"], info["period"],
                  info["ground_speed"], info["iterations"], "%.1fs" % info["runtime_s"],
                  flush=True)
        c = (info, res.x.copy(), col, res)
        allc.append(c)
        if _better_info(info, None if cand is None else cand[0], mode):
            cand = c
    return cand, allc


def refine_mesh(cand, ckw, tol, maxiter, dt_max=0.25, n_cap=160, verbose=False):
    """Re-solve a converged cycle on a finer mesh until its replay check passes.

    First step: n = max(n, ceil(T / dt_max) + 1) nodes (so that dt <= dt_max). If the replay
    check still fails, the node count grows by a factor 1.5 until it passes or reaches n_cap.
    Each solve is warm-started by cubic interpolation of the previous one and keeps its integer
    m. Returns a candidate tuple like _run_col; success is the replay result of the last
    converged solve (a failed re-solve keeps the previous one).
    """
    info0, x, col, res = cand
    n_init = col.N
    pass_init = dense_replay(col, x)["replay_pass"]
    cur = cand
    steps = 0
    while True:
        info, x, col, res = cur
        n_need = int(np.ceil(info["period"] / dt_max)) + 1
        if col.N < n_need:
            n_new = min(n_need, n_cap)
        elif not dense_replay(col, x)["replay_pass"]:
            n_new = min(int(np.ceil(1.5 * col.N)), n_cap)
        else:
            break
        if n_new <= col.N:
            break
        c2 = Collocation(**dict(ckw, n=n_new))
        c2.m = int(info.get("m", col.m))
        xs = _resample(col, x, c2)
        bnds = c2.bounds()
        xs = np.clip(xs, [b[0] for b in bnds], [b[1] for b in bnds])
        cons = [{"type": "eq", "fun": c2.eq_fun, "jac": c2.eq_jac},
                {"type": "ineq", "fun": c2.ineq_fun, "jac": c2.ineq_jac}]
        t0 = time.time()
        r2 = minimize(c2.objective, xs, jac=True, method="SLSQP", bounds=bnds,
                      constraints=cons, options={"maxiter": maxiter, "ftol": 1e-9})
        viol = c2.violation(r2.x)
        conv = bool(r2.success and viol < tol)
        steps += 1
        if verbose:
            print("refine", n_new, conv, "%.2e" % viol, "%.1fs" % (time.time() - t0), flush=True)
        if not conv:
            break
        rc2 = dense_replay(c2, r2.x)
        i2 = _summary(c2, r2.x, r2, viol, rc2["replay_pass"], info0["seed"],
                      info["runtime_s"] + time.time() - t0)
        i2["converged"] = True
        i2.update(rc2)
        i2["m_offset"] = info.get("m_offset")
        cur = (i2, r2.x.copy(), c2, r2)
    info, x, col, res = cur
    rc = dense_replay(col, x)
    info.update(rc)
    info["success"] = bool(info.get("converged", True) and rc["replay_pass"])
    info.update({"refined": steps > 0, "refine_steps": steps, "n_nodes_initial": n_init,
                 "replay_pass_initial": bool(pass_init), "n_nodes": col.N})
    return (info, x, col, res)


def solve(ac, prof, mode="min_wind", cycle="travel", n=40, u10=None, chi=np.pi / 2,
          h_min=1.0, h_tip_min=0.3, v_factor=1.1, seeds=(0, 1, 2), maxiter=300, tol=1e-5,
          x0=None, return_x=False, verbose=False, stop_after=2, tip_model="abs", wave=None,
          midpoints=False, rate_limits=False, n_min=None, n_frac=1.0, mu_frac=1.0,
          bank_rate_max_deg_s=None, cl_rate_max_per_s=None, phase_pin="theta", m_search=(0,),
          rate_model="constant", roll_pb2v=None, ground_effect=False, motor=False,
          v_travel_min=None, folded_prop=False, replay_check=False, refine=False,
          wave_preset="model", dt_max=0.25, n_cap=160, refine_tries=2, refine_maxiter=100,
          wing_shape=None):
    """Solve for the best converged cycle among the seeded initial guesses.

    wing_shape: a WingShape for a bent wing (see Collocation; the info then holds the shaped
    aircraft's numbers).

    ground_effect, motor, v_travel_min and mode "min_power": see Collocation.
    midpoints (flat mode; the wave mode has WaveParams.midpoints), rate_limits, n_min, n_frac,
    mu_frac, the two rate keywords and phase_pin: see Collocation. m_search: offsets added to the
    integer m taken from each start (wave mode, a > 0); each start is solved once per offset and
    the best feasible result is kept. The defaults reproduce the earlier flat results exactly.

    replay_check=True (see dense_replay): a converged solve is a success only if the dense replay
    closes (0.05 m in height, 1% in airspeed) and h and the tip stay within 0.02 m of their
    limits; the replay numbers (rc_*) are added to the result. refine=True (implies the replay
    check): the converged candidates (the refine_tries with the lowest objective) are re-solved
    on a finer mesh (see refine_mesh; at most refine_maxiter SLSQP iterations per solve) and the
    first whose replay passes is returned.
    wave_preset "model", "p63" or "p64_low" fills w_scale, u_scale and decay of a wave dict
    unless the dict gives them.

    tip_model "abs" or "geometric" (wingtip height, see Collocation._path). wave = {a, wavelength,
    to_deg} switches to the wave-synchronized mode (heights above the moving surface, wave wind,
    theta periodicity with m taken from each initial guess); x0 then needs the extra phi0 slot.
    """
    check = bool(replay_check) or bool(refine)
    wave = _apply_preset(wave, wave_preset)
    ckw = dict(ac=ac, prof=prof, mode=mode, cycle=cycle, n=n, u10=u10, chi=chi, h_min=h_min,
               h_tip_min=h_tip_min, v_factor=v_factor, tip_model=tip_model, wave=wave,
               midpoints=midpoints, rate_limits=rate_limits, n_min=n_min, n_frac=n_frac,
               mu_frac=mu_frac, bank_rate_max_deg_s=bank_rate_max_deg_s,
               cl_rate_max_per_s=cl_rate_max_per_s, phase_pin=phase_pin, rate_model=rate_model,
               roll_pb2v=roll_pb2v, ground_effect=ground_effect, motor=motor,
               v_travel_min=v_travel_min, folded_prop=folded_prop, wing_shape=wing_shape)
    col = Collocation(**ckw)
    ckw["wave"] = col.wv   # the same WaveParams object for every mesh
    if x0 is not None and len(x0) != col.nv:   # a start from another mesh: resample it
        per = NS + NC + (1 if col.motor else 0)
        n0 = (len(x0) - (3 if col.wv is not None else 2)) // per
        x0 = _resample(Collocation(**dict(ckw, n=n0)), np.asarray(x0), col)
    starts = [(None, x0)] if x0 is not None else [(s, None) for s in seeds]
    best, pool = None, []
    n_ok = 0
    for seed, xs in starts:
        if xs is None:
            xs = col.guess(np.random.default_rng(seed))
        cand, allc = _run_col(col, xs, m_search, seed, mode, tol, maxiter, verbose,
                              check and not refine)
        if refine:
            pool += [c for c in allc if c[0]["converged"]]
        if _better_info(cand[0], None if best is None else best[0], mode):
            best = cand
        n_ok += int(cand[0]["converged"] if refine else cand[0]["success"])
        if n_ok >= stop_after:
            break
    if refine and pool:
        sgn = 1 if mode == "min_wind" else -1
        pool.sort(key=lambda c: sgn * c[0]["objective"])
        first = None
        for c in pool[:refine_tries]:
            r = refine_mesh(c, ckw, tol, min(maxiter, refine_maxiter), dt_max, n_cap, verbose)
            if first is None:
                first = r
            if r[0]["success"]:
                first = r
                break
        best = first
    if best[0].get("m") is not None:
        best[2].m = int(best[0]["m"])
    if return_x:
        return best[0], best[1], best[2]
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
    """Profile at u10. roughness "charnock" or "coare35": z0 from sim.seawind.Wind at this u10; "fixed_z0": z0."""
    if roughness == "fixed_z0":
        if z0 is None:
            raise ValueError("fixed_z0 needs z0")
        z0v = float(z0)
    elif roughness in ("charnock", "coare35"):
        from sim.seawind import Wind
        z0v = Wind(float(u10), 270.0, roughness=roughness).z0
    else:
        raise ValueError(roughness)
    if kind == "knee":
        return Profile(z0v, min(10.0, knee), knee)
    if kind != "log":
        raise ValueError(kind)
    return Profile(z0v, 10.0)


def _scaled_fixed_point(K, kind, knee, u_guess, roughness="charnock"):
    """u with u = K ln(zref / z0(u)) (the minimum u10 scales with ln(zref / z0))."""
    from scipy.optimize import brentq
    zref = min(10.0, knee) if kind == "knee" else 10.0
    from sim.seawind import Wind

    def g(u):
        return u - K * np.log(zref / Wind(float(u), 270.0, roughness=roughness).z0)

    try:
        return float(brentq(g, 0.05, 80.0, xtol=1e-6))
    except ValueError:
        return float(u_guess)


def solve_min_wind(ac, z0_mode="fixed8", roughness="charnock", z0=None, u10_ref=8.0,
                   kind="log", knee=7.5, tol_u10=0.01, max_outer=8, return_x=False,
                   x0=None, seeds=(0, 1, 2), verbose=False, **kw):
    """Minimum-wind cycle with a chosen roughness treatment. kw go to solve (cycle, n, h_min,
    tip_model, wave, ...).

    z0_mode "fixed8": z0 at u10 = u10_ref (today's behaviour). "self_consistent": Charnock (or COARE 3.5) z0 at
    the u10 the solution finds, by an outer fixed point. Each outer step is a direct re-solve,
    warm-started from the previous one. The next trial u10 is predicted with the scaling
    u10_min = K ln(zref / z0) (K from the last solve); the prediction is only a start, the loop
    ends when a direct solve returns u10 within tol_u10 of the u10 that set its z0. With
    roughness "fixed_z0" there is nothing to iterate.
    """
    kw.pop("mode", None)
    if z0_mode not in ("fixed8", "self_consistent"):
        raise ValueError(z0_mode)
    cold_fallback = bool(kw.pop("cold_fallback", True))   # False: a failed x0 start is returned as is
    return _solve_min_wind(ac, z0_mode, roughness, z0, u10_ref, kind, knee, tol_u10, max_outer,
                           x0, seeds, verbose, kw, return_x, cold_fallback)


def _solve_min_wind(ac, z0_mode, roughness, z0, u10_ref, kind, knee, tol_u10, max_outer,
                    x0, seeds, verbose, kw, return_x, cold_fallback=True):

    if verbose:
        kw = dict(kw, verbose=True)

    def one(u_for_z0, xs):
        prof = profile_for(u_for_z0, kind, roughness, z0, knee)
        if xs is not None:
            r = solve(ac, prof, "min_wind", x0=xs, return_x=True, **kw)
            if r[0]["success"] or not cold_fallback:
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
        if it == 0:   # what the first solve of the outer loop needed (later ones start refined)
            first = {"n_nodes_first": info.get("n_nodes_initial", info.get("n_nodes")),
                     "replay_pass_first": info.get("replay_pass_initial", info.get("replay_pass"))}
        if not info["success"]:
            break
        if abs(info["u10"] - u_cur) < tol_u10:
            break
        xs = x
        if kw.get("refine"):
            kw = dict(kw, n=col.N)   # continue on the refined mesh
        K = info["u10"] / prof.L
        predicted = _scaled_fixed_point(K, kind, knee, info["u10"], roughness)
        u_cur = predicted
    done = bool(info["success"] and abs(info["u10"] - hist[-1]["u10_for_z0"]) < tol_u10)
    err = None
    # error of the first scaling prediction against the direct re-solve it started
    if len(hist) >= 2 and hist[1]["scaling_predicted"] is not None:
        err = abs(hist[1]["u10_found"] - hist[1]["u10_for_z0"])
    info.update({"z0": hist[-1]["z0"], "z0_mode": z0_mode, "roughness": roughness,
                 "outer_iterations": len(hist), "outer_converged": done,
                 "outer_history": hist, "scaling_error_first_step": err, **first})
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
    ap.add_argument("--wave-study3", action="store_true",
                    help="third pass: midpoints, rate limits, n >= 0, theta pin, m search, rotated "
                         "starts -> sim/out/dsopt_constraints.json")
    ap.add_argument("--wave-study4", action="store_true",
                    help="fourth pass: calibrated wave wind, replay check, mesh refinement, margins "
                         "-> sim/out/dsopt_wave4.json")
    ap.add_argument("--constraint-study", action="store_true",
                    help="flat-sea tables A and B -> sim/out/dsopt_constraints.json")
    ap.add_argument("--constraint-study2", action="store_true",
                    help="flat-sea tables A2 and B2 (roll rate p b / 2V) -> "
                         "sim/out/dsopt_constraints.json")
    ap.add_argument("--aircraft", default=None, help="comma list for --wave-study4 (default both)")
    ap.add_argument("--out", default=None, help="output path for --wave-study4")
    ap.add_argument("--budget-min", type=float, default=40.0,
                    help="--wave-study4: no case is started after this many minutes")
    ap.add_argument("--presets", default=None, help="comma list for --wave-study4")
    ap.add_argument("--tp", default=None, help="comma list of Tp (s) for --wave-study4")
    ap.add_argument("--hs", default=None, help="comma list of Hs (m) for --wave-study4")
    ap.add_argument("--no-flat", action="store_true", help="--wave-study4: skip the flat references")
    ap.add_argument("--nodes", type=int, default=30)
    ap.add_argument("--workers", type=int, default=7)
    args = ap.parse_args(argv)
    if args.wave_study:
        from sim.formstudy import wave_study
        wave_study(n=args.nodes, workers=args.workers)
    elif args.constraint_study:
        from sim.formstudy import constraint_study
        constraint_study(n=args.nodes, workers=args.workers)
    elif args.constraint_study2:
        from sim.formstudy import constraint_study2
        constraint_study2(n=args.nodes, workers=args.workers)
    elif args.wave_study4:
        from sim.formstudy import wave_study4
        kw = {}
        if args.aircraft:
            kw["names"] = tuple(args.aircraft.split(","))
        if args.presets:
            kw["presets"] = tuple(args.presets.split(","))
        if args.tp:
            kw["tps"] = tuple(float(v) for v in args.tp.split(","))
        if args.hs:
            kw["hs_list"] = tuple(float(v) for v in args.hs.split(","))
        if args.out:
            kw["path"] = args.out
        wave_study4(n=args.nodes, workers=min(args.workers, 4), budget_s=args.budget_min * 60.0,
                    with_flat=not args.no_flat, **kw)
    elif args.wave_study3:
        from sim.formstudy import wave_study3
        wave_study3(n=args.nodes, workers=args.workers)
    elif args.wave_study2:
        from sim.formstudy import wave_study2
        wave_study2(n=args.nodes, workers=args.workers)
    else:
        ap.print_help()


if __name__ == "__main__":
    main()
