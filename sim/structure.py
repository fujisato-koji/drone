"""Parametric structural and mass model of a small glider wing.

Assumptions:
- Spanwise lift is elliptical, total lift = n * m * g.
- The half-wing is a cantilever fixed at the root.
- A round tube spar carries all bending; skin and core carry none.
- Limit load factor n_limit; ultimate = safety_factor * n_limit.
- Linear elastic beam theory (Euler-Bernoulli, small deflection).
- Material sigma_allow is the allowable tension/compression stress with the
  knockdown already included (assumed values, see MATERIALS).
- Body axes: x forward from the nose, y right, z UP (matches the simulation).
- Units SI.
"""
from __future__ import annotations

from dataclasses import dataclass, field

import numpy as np

G = 9.81


@dataclass
class Material:
    name: str
    density: float  # kg/m3
    E: float  # Pa
    # Allowable stress in tension/compression (Pa), knockdown already included.
    sigma_allow: float


# assumed, to be sourced (record Q)
MATERIALS = {
    "carbon_tube": Material("carbon tube pultruded", 1600.0, 120e9, 600e6),
    "fiberglass_skin": Material("fiberglass skin", 1900.0, 20e9, 150e6),
    "epp": Material("EPP foam", 30.0, 5e6, 0.3e6),
    "xps": Material("XPS foam", 33.0, 15e6, 0.4e6),
    "balsa": Material("balsa", 160.0, 3e9, 10e6),
}


@dataclass
class WingGeometry:
    span: float
    root_chord: float
    taper: float  # tip chord / root chord
    thickness_ratio: float = 0.10
    dihedral_deg: float = 3.0

    @property
    def area(self) -> float:
        return 0.5 * self.span * self.root_chord * (1.0 + self.taper)

    @property
    def mean_chord(self) -> float:
        return self.area / self.span

    @property
    def aspect_ratio(self) -> float:
        return self.span ** 2 / self.area

    def chord(self, y):
        y = np.asarray(y, dtype=float)
        return self.root_chord * (1.0 - (1.0 - self.taper) * y / (self.span / 2.0))

    def max_spar_od(self, y):
        return 0.9 * self.thickness_ratio * self.chord(y)


def elliptical_lift(y, total_lift, span):
    """Lift per unit span at spanwise station y (y=0 is the centreline)."""
    y = np.asarray(y, dtype=float)
    l0 = 4.0 * total_lift / (np.pi * span)
    r = np.clip(1.0 - (2.0 * y / span) ** 2, 0.0, None)
    return l0 * np.sqrt(r)


def shear_and_moment(y_grid, total_lift, span):
    """Shear V(y) and bending moment M(y) on the half-wing (y from root to tip).

    Cumulative integration from the tip towards the root.
    """
    y = np.asarray(y_grid, dtype=float)
    l = elliptical_lift(y, total_lift, span)
    dy = np.diff(y)
    V = np.zeros_like(y)
    M = np.zeros_like(y)
    # V(y) = integral_y^tip l ; M(y) = integral_y^tip V
    V[:-1] = np.cumsum(((l[:-1] + l[1:]) * 0.5 * dy)[::-1])[::-1]
    M[:-1] = np.cumsum(((V[:-1] + V[1:]) * 0.5 * dy)[::-1])[::-1]
    return V, M


def tip_deflection(y, M, EI):
    """Double integration of M/(EI) from the root (w = theta = 0 at y[0]).

    Returns the deflection w(y) along the grid.
    """
    kappa = np.asarray(M, dtype=float) / EI
    dy = np.diff(y)
    theta = np.concatenate(([0.0], np.cumsum(0.5 * (kappa[:-1] + kappa[1:]) * dy)))
    w = np.concatenate(([0.0], np.cumsum(0.5 * (theta[:-1] + theta[1:]) * dy)))
    return w


@dataclass
class TubeSpar:
    od: float
    wall: float
    material: Material

    @property
    def id(self) -> float:
        return self.od - 2.0 * self.wall

    @property
    def area(self) -> float:
        return np.pi / 4.0 * (self.od ** 2 - self.id ** 2)

    @property
    def I(self) -> float:
        return np.pi / 64.0 * (self.od ** 4 - self.id ** 4)

    @property
    def mass_per_length(self) -> float:
        return self.area * self.material.density

    def stress(self, M):
        return M * (self.od / 2.0) / self.I


def size_tube_spar(geometry, mass_total, n_limit, safety_factor, material,
                   wall_options=(0.5e-3, 0.75e-3, 1.0e-3, 1.5e-3), od_step=1e-3):
    """Lightest constant round tube that passes at the root.

    Simplest choice: the tube is constant over the whole half-span (the root
    section is used everywhere), so the "station where moment is small" is the tip.
    """
    span = geometry.span
    lift_limit = n_limit * mass_total * G
    m_root_limit = lift_limit * span / (3.0 * np.pi)
    m_root_ult = safety_factor * m_root_limit
    od_max = float(geometry.max_spar_od(0.0))
    best = None
    od = 4e-3
    while od <= od_max + 1e-12:
        for wall in wall_options:
            if 2.0 * wall >= od:
                continue
            spar = TubeSpar(od, wall, material)
            if spar.stress(m_root_ult) <= material.sigma_allow:
                if best is None or spar.mass_per_length < best.mass_per_length:
                    best = spar
        od += od_step
    if best is None:
        raise ValueError(
            f"No tube fits: root ultimate moment {m_root_ult:.2f} N m, "
            f"max od {od_max * 1e3:.1f} mm, sigma_allow {material.sigma_allow / 1e6:.0f} MPa")
    y = np.linspace(0.0, span / 2.0, 2001)
    _, M = shear_and_moment(y, lift_limit, span)
    w = tip_deflection(y, M, material.E * best.I)
    stress_ult = best.stress(m_root_ult)
    half = span / 2.0
    return {
        "od": best.od, "wall": best.wall, "I": best.I,
        "root_stress_ultimate": stress_ult,
        "margin": material.sigma_allow / stress_ult - 1.0,
        "mass": 2.0 * half * best.mass_per_length * 1.1,
        "tip_deflection_limit": w[-1],
        "deflection_ratio": w[-1] / half,
        "spar": best,
    }


def skin_mass(geometry, skin_material, skin_thickness, core_material):
    skin = 2.0 * geometry.area * skin_thickness * skin_material.density
    core = (0.6 * geometry.area * geometry.thickness_ratio * geometry.mean_chord
            * core_material.density)
    return {"skin": skin, "core": core, "total": skin + core}


@dataclass
class Component:
    name: str
    mass: float
    position: tuple  # (x forward, y right, z up) in body axes, m
    extent: tuple = (0.01, 0.01, 0.01)  # (lx, ly, lz) box size, m


@dataclass
class MassModel:
    components: list = field(default_factory=list)

    def total_mass(self) -> float:
        return float(sum(c.mass for c in self.components))

    def cg(self) -> np.ndarray:
        m = self.total_mass()
        return sum(c.mass * np.asarray(c.position, float) for c in self.components) / m

    def inertia(self) -> np.ndarray:
        cg = self.cg()
        I = np.zeros((3, 3))
        for c in self.components:
            lx, ly, lz = c.extent
            own = c.mass / 12.0 * np.diag([ly ** 2 + lz ** 2, lx ** 2 + lz ** 2, lx ** 2 + ly ** 2])
            d = np.asarray(c.position, float) - cg
            I += own + c.mass * (np.dot(d, d) * np.eye(3) - np.outer(d, d))
        return I

    def summary(self) -> dict:
        I = self.inertia()
        return {"mass": self.total_mass(), "cg": self.cg(),
                "Ixx": I[0, 0], "Iyy": I[1, 1], "Izz": I[2, 2]}


def wing_strips(geometry, spar_mass, skin_core_mass, extra_mass=0.0, n_strips=20,
                x_wing=0.0, z_wing=0.0):
    """Strip masses for both half-wings.

    Spar mass is uniform in span; skin, core and extra (solar film) mass are
    proportional to local chord. Wing reference line is at x = x_wing (simple).
    """
    half = geometry.span / 2.0
    edges = np.linspace(0.0, half, n_strips + 1)
    yc = 0.5 * (edges[:-1] + edges[1:])
    dy = half / n_strips
    c = geometry.chord(yc)
    w = c / c.sum()
    tan_d = np.tan(np.radians(geometry.dihedral_deg))
    comps = []
    for side, sgn in (("R", 1.0), ("L", -1.0)):
        for i in range(n_strips):
            m = 0.5 * spar_mass / n_strips + 0.5 * (skin_core_mass + extra_mass) * w[i]
            comps.append(Component(
                f"wing_{side}{i:02d}", m,
                (x_wing, sgn * yc[i], z_wing + yc[i] * tan_d),
                (c[i], dy, geometry.thickness_ratio * c[i])))
    return comps


def build_baseline(mass_total=1.5):
    geo = WingGeometry(span=2.0, root_chord=0.16, taper=0.6, thickness_ratio=0.10)
    spar = size_tube_spar(geo, mass_total, 4.0, 1.5, MATERIALS["carbon_tube"])
    sk = skin_mass(geo, MATERIALS["fiberglass_skin"], 0.2e-3, MATERIALS["xps"])
    solar = 0.08
    wing_mass = spar["mass"] + sk["total"] + solar
    comps = wing_strips(geo, spar["mass"], sk["total"], solar)
    comps += [
        Component("battery", 0.25, (0.05, 0, 0), (0.10, 0.04, 0.03)),
        Component("avionics", 0.12, (0.0, 0, 0), (0.08, 0.04, 0.02)),
        # camera position and all extents are chosen, not given by the spec
        Component("payload_camera", 0.05, (0.08, 0, -0.01), (0.04, 0.04, 0.04)),
        Component("tail_boom", 0.06, (-0.5, 0, 0), (0.8, 0.01, 0.01)),
        Component("tail_surfaces", 0.05, (-0.9, 0, 0), (0.10, 0.30, 0.10)),
        Component("fuselage_pod", 0.15, (-0.05, 0, 0), (0.40, 0.08, 0.08)),
    ]
    model = MassModel(comps)
    info = {"geometry": geo, "spar": spar, "skin": sk, "wing_mass": wing_mass,
            "mass_target": mass_total, "mass_difference": mass_total - model.total_mass()}
    return model, info


if __name__ == "__main__":
    model, info = build_baseline()
    s = info["spar"]
    print(f"spar od {s['od']*1e3:.1f} mm wall {s['wall']*1e3:.2f} mm mass {s['mass']:.3f} kg "
          f"margin {s['margin']:.2f} defl ratio {s['deflection_ratio']:.3f}")
    print(f"wing mass {info['wing_mass']:.3f} kg (skin {info['skin']['skin']:.3f}, core {info['skin']['core']:.3f})")
    print("summary:", model.summary())
    print(f"target {info['mass_target']} kg, difference {info['mass_difference']:.3f} kg")
