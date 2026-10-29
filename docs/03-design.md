# 1.3 Design calculation

Calculations, one section per item, each with an id that values and checks can cite.
The spar section below sizes the baseline (candidate 1 of docs/02-form.md#candidate-forms) as a yardstick for the form study of step 1.2, not as a design; the form is chosen in 1.2 first.

## Wing spar {#wing-spar}

The method is in `sim/structure.py` (see `docs/00-conventions.md` for how a model enters the record).
Command: `python -m sim.structure`.
Inputs by key: wing.span_m, wing.root_chord_m, wing.taper, wing.thickness_ratio, wing.design_mass_kg, ds.n_limit, ds.safety_factor, spar.carbon_density, spar.carbon_modulus, spar.carbon_allowable, spar.fit_fraction, spar.joiner_factor and spar.deflection_limit.
Outputs by key: spar.od_mm, spar.wall_mm, spar.mass_kg, spar.margin and spar.tip_deflection_ratio.
The wing is wing.span_m span, wing.root_chord_m root chord, taper wing.taper, thickness ratio wing.thickness_ratio, sized for a design mass of wing.design_mass_kg at ds.n_limit with ds.safety_factor to ultimate.
The lift is elliptical along the span and the half-wing is a cantilever from the root; a round carbon tube (spar.carbon_density, spar.carbon_modulus, spar.carbon_allowable) carries all the bending and must fit inside spar.fit_fraction of the airfoil thickness at the root.
The first pass used assumed material values (record at c92d9ac), replaced by the pultruded tube datasheet (P54), which gives lit.pultruded_modulus_pa and lit.pultruded_flexural_pa for spar.carbon_modulus and spar.carbon_allowable.
With those values the lightest tube that keeps the root stress under the allowable at ultimate load is spar.od_mm outer diameter and spar.wall_mm wall, with mass spar.mass_kg for the full span including a joiner allowance (spar.joiner_factor), margin spar.margin, and a tip deflection of spar.tip_deflection_ratio of the half span at limit load.
The sizing also limits the tip deflection to spar.deflection_limit of the half span (Q-010, D-014); with the sourced pultruded values the lightest tube that passes both criteria is the one above, at the deflection limit, with a comfortable stress margin.
A roll-wrapped tube of higher modulus would give a smaller and lighter tube, but its values are not yet sourced (Q-010 stays open for that).

## Mass model {#mass-model}

Command: `python -m sim.structure`.
Inputs by key: the wing keys of the spar section, skin.thickness_mm, skin.density, core.density, film.mass_kg, and the component masses battery.mass_kg, avionics.mass_kg, camera.mass_kg, pod.mass_kg, boom.mass_kg and tail.mass_kg.
Outputs by key: wing.mass_kg, airframe.baseline_mass_kg and airframe.mass_gap_kg.
The wing weighs wing.mass_kg (spar, a fiberglass skin of skin.thickness_mm over a foam core, and solar film), and the baseline component list (battery, avionics, camera, pod, boom, tail) brings the airframe to airframe.baseline_mass_kg, under the design mass of wing.design_mass_kg by airframe.mass_gap_kg.
The gap is margin for sealing, buoyancy, and the satellite link, or it goes into a lower wing loading; the form study decides.
The centre of gravity and the inertia tensor from the same model are the inputs for the six-degree-of-freedom flight model when the point-mass model is replaced.
The carbon tube values are sourced (P54); the other inputs are assumed (Q-015).
K-003 checked the methods of the first pass only (record at c92d9ac); the current spar and the mass sum have no independent check yet (Q-023).
