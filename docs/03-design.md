# 1.3 Design calculation

Calculations, one section per item, each with an id that values and checks can cite.

## Wing spar {#wing-spar}

The method is in `sim/structure.py` (see `docs/00-conventions.md` for how a model enters the record).
The wing is wing.span_m span, wing.root_chord_m root chord, taper wing.taper, thickness ratio wing.thickness_ratio, sized for a design mass of wing.design_mass_kg at ds.n_limit with ds.safety_factor to ultimate.
The lift is elliptical along the span and the half-wing is a cantilever from the root; a round carbon tube (spar.carbon_density, spar.carbon_modulus, spar.carbon_allowable) carries all the bending and must fit inside 0.9 of the airfoil thickness at the root.
The first pass assumed a carbon modulus of 120 GPa and an allowable of 600 MPa and found a 10 mm by 0.5 mm tube; the pultruded tube datasheet (P54) gives lit.pultruded_modulus_pa and lit.pultruded_flexural_pa, so the material values were replaced (spar.carbon_modulus, spar.carbon_allowable).
With those values the lightest tube that keeps the root stress under the allowable at ultimate load is spar.od_mm outer diameter and spar.wall_mm wall, with mass spar.mass_kg for the full span including a joiner allowance, margin spar.margin, and a tip deflection of spar.tip_deflection_ratio of the half span at limit load.
The sizing now also limits the tip deflection to spar.deflection_limit of the half span (Q-010); with the sourced pultruded values the lightest tube that passes both criteria is the one above, at the deflection limit, with a comfortable stress margin.
A roll-wrapped tube of higher modulus would give an 11 mm tube about a third lighter, but its values are not yet sourced (Q-010 stays open for that).

## Mass model {#mass-model}

The wing weighs wing.mass_kg (spar, 0.2 mm fiberglass skin over an XPS core, and solar film), and the baseline component list (battery, avionics, camera, pod, boom, tail) brings the airframe to airframe.baseline_mass_kg, under the design mass of wing.design_mass_kg by about 0.39 kg.
The gap is margin for sealing, buoyancy, and the satellite link, or it goes into a lower wing loading; the form study decides.
The centre of gravity and the inertia tensor from the same model are the inputs for the six-degree-of-freedom flight model when the point-mass model is replaced.
Material values are assumed (Q-009); the model's methods were checked in K-003.
