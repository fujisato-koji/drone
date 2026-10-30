"""Induced drag of a rolled elliptic lifting line in ground effect (image method).

Model "image": a straight lifting line of span b with elliptic loading, centre at height hc over
a plane surface, rolled by the bank angle. In the Trefftz plane the trailing vortex sheet has point
vortices of strength -dGamma; the plane is replaced by the mirror image sheet with opposite
circulation. The induced drag is D = (rho / 2) * integral of Gamma times the downwash normal to the
wing, with the downwash of the sheet and of its image. The ratio to free air at the same
circulation is

    K(hc/b, bank) = 1 + I_image / I_self,   I_self = pi Gamma0^2 / 4.

K is tabulated on the grid x_t = (height of the lowest tip) / b = hc/b - sin|bank| / 2, from XT_MIN
to XT_MAX (log spaced), and bank from 0 to 80 deg, so that every entry has the tip above the
surface. Interpolation is a bicubic Catmull-Rom spline in (ln x_t, |bank|) that is complex-step
safe. Below XT_MIN the tip height is clipped (the tip constraint of the optimizer keeps the cycle
out of there); above XT_MAX (K set to 1 at the last node) the factor is 1.

The table is cached in sim/out/groundeffect_table.json (with a meta block) and rebuilt if absent.
"""
import json
import os

import numpy as np

XT_MIN, XT_MAX, N_XT = 0.02, 4.0, 48
BANK_MAX_DEG, D_BANK_DEG = 80.0, 5.0
N_QUAD = 360
TABLE_PATH = os.path.join(os.path.dirname(__file__), "out", "groundeffect_table.json")

_table = None


def image_ratio(x_c, bank, n=N_QUAD):
    """K by the image method for centre height x_c = hc/b and bank (rad); needs the tip above ground."""
    z, w = np.polynomial.legendre.leggauss(n)
    phi = 0.5 * np.pi * (z + 1.0)  # s = (b/2) cos(phi), phi in [0, pi]
    w = 0.5 * np.pi * w
    s = 0.5 * np.cos(phi)  # b = 1
    gam = np.sin(phi)  # Gamma / Gamma0
    kap = -np.cos(phi) * w  # vortex strength of each element, -dGamma / Gamma0
    cb, sb = np.cos(bank), np.sin(bank)
    py, pz = s * cb, x_c + s * sb  # points of the wing line
    qy, qz = py, -pz  # mirror image
    dy = py[:, None] - qy[None, :]
    dz = pz[:, None] - qz[None, :]
    r2 = dy * dy + dz * dz
    # image circulation is opposite (-kap); velocity of a counterclockwise point vortex:
    # (-dz, dy) kap / (2 pi r^2); downwash = -v . n with n = (-sin, cos)
    vy = (-(-kap)[None, :] * dz / r2).sum(1) / (2 * np.pi)
    vz = ((-kap)[None, :] * dy / r2).sum(1) / (2 * np.pi)
    down = -(-sb * vy + cb * vz)
    # ds = (b/2) sin(phi) dphi; w holds the dphi weights
    i_img = np.sum(gam * down * 0.5 * np.sin(phi) * w)
    # with this vortex orientation the "downwash" above is an upwash of the self sheet: sign flips
    return 1.0 - i_img / (np.pi / 4.0)


def build_table():
    xt = np.exp(np.linspace(np.log(XT_MIN), np.log(XT_MAX), N_XT))
    banks = np.radians(np.arange(0.0, BANK_MAX_DEG + 0.5 * D_BANK_DEG, D_BANK_DEG))
    K = np.empty((N_XT, len(banks)))
    for j, bk in enumerate(banks):
        for i, x in enumerate(xt):
            K[i, j] = image_ratio(x + 0.5 * np.sin(bk), bk)
    K[-1, :] = 1.0
    return xt, banks, K


def _meta():
    from sim import record
    return record.meta("python -c \"from sim import groundeffect; groundeffect.write_table()\"",
                       ["sim/groundeffect.py"])


def write_table(path=TABLE_PATH):
    xt, banks, K = build_table()
    out = {"meta": _meta(), "model": "image", "n_quad": N_QUAD, "xt": xt.tolist(),
           "bank_deg": np.degrees(banks).tolist(), "K": K.tolist()}
    with open(path, "w", newline="\n") as fh:
        json.dump(out, fh, indent=1)
        fh.write("\n")
    return xt, banks, K


def get_table():
    global _table
    if _table is None:
        if os.path.exists(TABLE_PATH):
            with open(TABLE_PATH, encoding="utf-8") as fh:
                d = json.load(fh)
            _table = (np.array(d["xt"]), np.radians(d["bank_deg"]), np.array(d["K"]))
        else:
            _table = write_table()
    return _table


def _cr(p0, p1, p2, p3, t):
    return 0.5 * (2 * p1 + (-p0 + p2) * t + (2 * p0 - 5 * p1 + 4 * p2 - p3) * t * t
                  + (-p0 + 3 * p1 - 3 * p2 + p3) * t * t * t)


def induced_drag_ratio(h_over_b, bank):
    """K(hc/b, bank): induced drag in ground effect over free air (arrays ok, complex-step safe).

    h_over_b: centre height over the plane surface in spans, measured along the surface normal;
    bank: roll angle (rad) relative to the surface."""
    xt, banks, K = get_table()
    h_over_b = np.asarray(h_over_b)
    bank = np.asarray(bank)
    ab = np.where(np.real(bank) < 0, -bank, bank)
    ab = np.where(np.real(ab) > banks[-1], banks[-1], ab)
    x_t = h_over_b - 0.5 * np.sin(ab)
    x_t = np.where(np.real(x_t) < XT_MIN, XT_MIN, x_t)
    done = np.real(x_t) >= xt[-1]
    u = np.log(np.where(done, xt[-1] * 0.999, x_t))
    du = (np.log(xt[-1]) - np.log(xt[0])) / (len(xt) - 1)
    fu = (u - np.log(xt[0])) / du
    fb = ab / (banks[1] - banks[0])
    iu = np.clip(np.floor(np.real(fu)).astype(int), 0, len(xt) - 2)
    ib = np.clip(np.floor(np.real(fb)).astype(int), 0, len(banks) - 2)
    tu, tb = fu - iu, fb - ib
    nb = len(banks)

    def col(i, j):  # node values, reflected at bank 0 and clamped at the ends
        i = np.clip(i, 0, len(xt) - 1)
        j = np.abs(j)
        j = np.where(j > nb - 1, 2 * (nb - 1) - j, j)
        return K[i, j]

    rows = []
    for di in (-1, 0, 1, 2):
        rows.append(_cr(*(col(iu + di, ib + dj) for dj in (-1, 0, 1, 2)), tb))
    val = _cr(*rows, tu)
    return np.where(done, 1.0, val)
