"""Registry of candidate aircraft for the form study."""
from sim.aircraft import BASELINE, Aircraft
from sim.record import value

# provenance: existing BASELINE from sim.aircraft (usual values for a 2 m RC-class glider; assumed).
baseline_2m = BASELINE

# provenance: mass 1.07 kg and span 1.5 m from the literature (ENAC demonstrator);
# wing_area 0.19, cd0 0.020, oswald 0.80, cl_max 1.1 assumed.
demo_1p5m = Aircraft("demo-1p5m", mass=1.07, span=1.5, wing_area=0.19,
                     cd0=0.020, oswald=0.80, cl_max=1.1)

# provenance: foam trainer; mass 1.35 kg and span 1.8 m from the literature;
# wing_area 0.40, cd0 0.030, oswald 0.75, cl_max 1.2 assumed.
easyglider_1p8m = Aircraft("easyglider-1p8m", mass=1.35, span=1.8, wing_area=0.40,
                           cd0=0.030, oswald=0.75, cl_max=1.2)

# provenance: albatross-like high aspect ratio glider (AR 25); all values assumed.
albatross_3m = Aircraft("albatross-3m", mass=3.0, span=3.0, wing_area=0.36,
                        cd0=0.012, oswald=0.90, cl_max=1.3)

# provenance: tailless flying wing (AR 10); all values assumed.
flyingwing_2m = Aircraft("flyingwing-2m", mass=1.3, span=2.0, wing_area=0.40,
                         cd0=0.018, oswald=0.80, cl_max=0.9)

# provenance: sealed buoyant fuselage (AR 18); all values assumed.
sealedhull_2p5m = Aircraft("sealedhull-2p5m", mass=2.2, span=2.5, wing_area=0.34,
                           cd0=0.016, oswald=0.85, cl_max=1.2)

# provenance: wandering albatross as a reference. mass, span and wing area are read from the
# record (lit.albatross_mass_kg, lit.albatross_span_m, lit.albatross_wing_area_m2); cd0, oswald
# and cl_max stay assumed.
bird_wandering = Aircraft("bird-wandering",
                          mass=value("lit.albatross_mass_kg", 9.0),
                          span=value("lit.albatross_span_m", 3.1),
                          wing_area=value("lit.albatross_wing_area_m2", 0.62),
                          cd0=0.015, oswald=0.90, cl_max=1.5)

_ALL = [baseline_2m, demo_1p5m, easyglider_1p8m, albatross_3m, flyingwing_2m,
        sealedhull_2p5m, bird_wandering]


def candidates():
    return list(_ALL)


def by_name(name):
    for ac in _ALL:
        if ac.name == name:
            return ac
    raise KeyError(name)


def parametric(span, aspect_ratio, mass, cd0=0.015, oswald=0.85, cl_max=1.2, name=None):
    area = span ** 2 / aspect_ratio
    if name is None:
        name = "param-b%.2f-ar%.1f-m%.2f" % (span, aspect_ratio, mass)
    return Aircraft(name, mass=mass, span=span, wing_area=area,
                    cd0=cd0, oswald=oswald, cl_max=cl_max)
