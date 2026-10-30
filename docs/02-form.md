# 1.2 Form

Candidate forms, the yardsticks, rough availability and cost of each, the comparison, and the selection.

## Computation environment

The form study needs a place to fly candidate aircraft in a wind field that looks like the sea, so the environment was built first (D-010).
The code lives in `sim/` and is a tool, not a fact; what it produces enters the record as calculated values that cite the section of this document, or a check, that explains them (see `docs/00-conventions.md`).

### Sea surface and wind model (`sim/seawind.py`) {#environment-model}

- Coordinates are x east, y north, z up, in metres and seconds.
- The mean wind is a neutral logarithmic profile over the local sea surface, with friction velocity and roughness length solved from U10 by the Charnock relation (env.charnock) plus the smooth-flow term (env.air_kinematic_viscosity), and env.karman as the von Karman constant.
- The height in the profile is measured above the local wave surface, so the wind field rides on the waves.
- Each wave component adds a wave-coherent perturbation from linear potential flow over a moving wavy surface: the air speeds up over crests and rises on the face it approaches, decaying as exp(-k z) with height; the reference wind for a component is the mean profile at the height 1/k relative to the wave's phase speed.
- The sea is a linear sum of components drawn from JONSWAP spectra (env.jonswap_gamma) with a cosine-squared directional spread on deep water; a fully developed wind sea follows from U10 by the Pierson-Moskowitz relations (env.pm_hs_coefficient, env.pm_peak_coefficient); swell is a second train.
- Frequencies are sampled by equal energy so the discrete sum reproduces the significant wave height.
- There is no turbulence yet; `gust()` is the hook for it.
- The constants are assumed values until Q-008 verifies them against P38 to P41 and against measured profiles over waves.

### Sea state history and wind-wave coupling (`sim/sea_history.py`) {#sea-coupling}

- Wind to waves: a wind sea grows with the time the wind has blown, by the equivalent-duration growth of the Coastal Engineering Manual (P71) with the constants env.cem_hm0_coeff, env.cem_tp_coeff and env.cem_duration_coeff and the friction velocity from the drag law env.cd_law_a and env.cd_law_b, capped at the fully developed sea (env.cem_full_hm0_limit, env.cem_full_tp_limit); the growth law was checked against the printed pages (K-006) and its repaired code awaits a re-check (Q-018).
- `SeaHistory` keeps an equivalent duration for the wind sea and lets the excess over what the wind can sustain decay as swell with env.sea_decay_tau_h when the wind drops; this is a stand-in for a spectral wave model.
- env.sea_grow_tau_h is no longer used by the code, since the equivalent-duration law replaced the relaxation; it stays in the record because K-005 and K-006 target it, until the standard has a withdrawn status.
- Waves to wind: the roughness length can follow the sea state instead of the Charnock constant, by the wave-steepness form (P59) or the wave-age form (P60); a young steep sea is rougher and slows the wind near the surface, which changes the shear the aircraft can use.
- Crests: an option adds the second-order self-interaction term of each component, so crests are sharper and troughs flatter; the flight model's wind still uses the linear terms.
- Breaking: the surface is flagged as breaking where its slope exceeds env.breaking_slope_limit, an assumed criterion to be sourced.
- Separation: an option takes wind out of the lee of steep crests, a crude stand-in for airflow separation (see the sheltering below).
- Turbulence: an option adds a frozen field of random Fourier modes with a von Karman spectrum of length scale env.turb_length_m, scaled to the offshore turbulence intensity of ISO 19901-1 (P67), with lateral and vertical intensities of env.turb_v_ratio and env.turb_w_ratio of the longitudinal one (P72, not read); its 3 s gust factor over 10 minutes came out at env.turb_gust_factor_check against lit.gust_factor_3s_10min_sea at sea (P68).
- Swell: an option replaces the log profile above a knee at lit.swell_knee_height_m by a constant wind, as measured under swell (P65), and another chooses between the two profiles from the wave age (see the swell knee below); the wave-induced perturbation can decay at the measured rates lit.wave_induced_decay_u and lit.wave_induced_decay_w (P63) instead of exp(-k z).
- The form study runs with the log profile and with the swell knee, since swell dominates most of the time in the Pacific (lit.pacific_swell_dominance, P62) and the knee removes the shear above it (Q-011).
- All of these constants and forms wait for Q-008.

#### Gust factor check {#gust-factor-check}

This check compares the turbulence model with a measured gust factor.
The number was first printed by the unit test at commit 86d6689; the function `gust_factor` in `sim/seawind.py` repeats the computation and gives the same number.
Command: `python -m unittest sim.tests.test_seawind` for now, which calls it.
Inputs by key: env.turb_length_m, env.turb_v_ratio, env.turb_w_ratio, env.karman and env.charnock.
Settings of the function, named here and not keyed: U10 of 10 m/s from 270 degrees, a point at 10 m height, a series of 600 s sampled every 0.1 s, a running window of 3 s, 64 Fourier modes, and seed 1.
Output by key: env.turb_gust_factor_check, to be compared with lit.gust_factor_3s_10min_sea (P68).
It is one realization of a random field and not a statistic, and the test asserts only a band around it.

#### Swell knee from the wave age {#wave-age-knee}

Under swell the measured wind has a knee or a maximum a few metres up and little shear above it (P65), because swell that runs faster than the wind gives momentum back to the air.
Hanley, Belcher and Sullivan sort the sea by the inverse wave age, the wind at 10 m along the waves divided by the phase speed of the spectral peak (P88).
Above env.inverse_wave_age_wind_driven the wind drives the waves, between 0 and env.inverse_wave_age_wave_driven the waves drive the wind, and between the two the sea is mixed; the authors say the limits are not hard and the change is likely smooth.
With `profile_mode="auto"` the model takes the most energetic wave train, computes its inverse wave age, and blends the log profile into the swell-knee profile with a smooth step between the two limits.
The smooth step is a model choice, since the source gives only the limits.
A counterswell, waves against the wind, gives a negative inverse wave age; the model then keeps the log profile, and the enhanced drag reported under counterswell is not modelled.
A fully developed wind sea sits at env.full_sea_inverse_wave_age, just inside the wind-driven limit, so it keeps the log profile.
In the storm scenario, at the hour the wind drops (hour 42), the swell gives an inverse wave age of env.storm_h42_inverse_wave_age and a blend of env.storm_h42_wave_driven_fraction toward the knee.
The wind difference between 2 m and 20 m then falls from env.storm_h42_shear_2_20_log_ms with the log profile to env.storm_h42_shear_2_20_auto_ms.
That is shear a soaring cycle loses after a storm, so the form study and the cycle optimizer will run with this option as well (Q-011).
Script: the numbers come from `Environment(profile_mode="auto").wave_age()` on `SeaHistory(WindHistory(STORM)).environment(42)` with `STORM` from `sim/export_field.py`; the unit tests in `sim/tests/test_seawind.py` cover the limits and the direction.

#### Sheltering behind steep crests {#crest-shelter}

When the wind outruns a steep wave, the air cannot follow the surface down the lee face and separates from it; this was observed at sea over short steep waves (P64).
The separated lee holds slower air near the water, and the layer above it carries a sharp change of wind; the gust soaring proposal places the albatross's energy there (P78, not read), and the birds' use of the troughs may come partly from this.

What the measurements say:

- Whether the air separates depends on the steepness of the single wave and on its age: the slope at which half the waves separate is env.separation_gamma_floor plus env.separation_gamma_span divided by 1 + ((u*/c)/env.separation_ustar_c_ref)^env.separation_exponent, with u* the friction velocity and c the phase speed (P89, eq. 4.8).
- So slow young waves separate at a modest steepness, while fast old waves, such as swell, need a steepness close to the limit at which they break.
- In the separated region over breaking waves the air nearly stops, with a backflow of at most about a fifth of the free stream (P92, P93); over wind waves the phase average stays positive but is much reduced (P91).
- No source read states how high the slow air reaches; the stress returns to its value above the waves at about half of 1/k (P90) and the profile becomes logarithmic again above about 0.7 of 1/k (P91).

The model, with `shelter=True`:

- It takes the slope of the summed surface along the wind, counting only the components the wind outruns at height 1/k, with k the wavenumber that carries the slope.
- The onset slope is the half-separated slope above divided by env.shelter_ramp_ratio; the separated share r grows from 0 there to 1 at twice the onset, so it is one half at the measured half-separated slope; the ramp is a model choice.
- The along-wind wind loses env.shelter_factor times r near the water, fading with height as exp(-k z / env.shelter_decay_kz), and the wave-coherent vertical wind is scaled by 1 - r, since the separated air does not follow the surface.
- The first version tested each component's own steepness against a fixed limit; it was dropped because in a spectral sea no single component is steep, so it never acted.

What it gives:

- For a steep single wave, a k of 0.35 under a wind three times the wave speed, the mean wind over the lee half at 1 m falls by env.shelter_case_lee_drop_fraction of its unsheltered value; the windward half is unchanged, and the effect is gone at three times 1/k.
- At a 10 m/s wind the onset slope is env.shelter_onset_30m_u10_10 for a 30 m wave and env.shelter_onset_3m_u10_10 for a 3 m wave, but the slow air over the 3 m wave reaches only about env.shelter_depth_3m_m.
- Over a fully developed sea at 15 m/s, the steepest slope of the linear surface is env.full_sea_u15_max_slope, below the onset, and the share of the surface that separates is env.shelter_full_sea_u15_fraction.
- So in this model the separation that reaches the flight height of R-12 needs steep crests that the linear sea does not make: breaking crests, or the sharp crests of a nonlinear sea.
- Where the wind does outrun a steep wave, the sheltering changes the soaring cycle markedly (see the cycles synchronized with the waves below), so it is not negligible when it acts.
- The trough as a shelter is still not shown by the model; the next step would let a share of the crests, matched to the measured whitecap cover, break and separate, and that needs a source for which crests break.
- All of this is a crude stand-in, and the real separated region needs a flow solver over moving waves; env.shelter_ramp_ratio and env.shelter_decay_kz wait for Q-008.

#### Roughness of the open sea {#sea-roughness}

The soaring literature sets the roughness length at lit.davenport_open_flat_z0_m, and the minimum winds of the form study fall by about half with it (see the form comparison), so which value the sea has decides much of R-02.

Where the literature's value comes from:

- It is passed on by citation from Sachs 2005, which could not be read; the only reason given is Bower's, that it suits cut grass or a very rough sea (P45).
- The WMO roughness classes give that value to open flat land with grass, and lit.davenport_open_sea_z0_m to the open sea whatever the waves (P99).
- No soaring paper read measured the shear the birds used; Sachs et al. took the wind at 10 m from a satellite with a log model and found the gradient far too small to explain the flight, lit.sachs2013_effective_gradient_per_s against lit.sachs2013_required_gradient_per_s (P47).

What the open sea has, in neutral air over a wind sea:

- Drag measured over the open ocean puts z0 between env.sea_z0_min_m and env.sea_z0_max_m for winds of 3 to 14 m/s (P94, P95, P96).
- The wind difference between 1 m and 10 m is then env.sea_shear_1_10_min to env.sea_shear_1_10_max of the wind at 10 m, against env.z003_shear_1_10 with the literature's value.
- At 10 m/s the wind at 1 m is env.u1_over_u10_charnock_10ms of the wind at 10 m over the sea, and env.u1_over_u10_z003_10ms with the literature's value.
- Profiles over a growing wind sea follow the log law over the sea's roughness down to about 1 m (P100, P102); nearer the water the waves act through the phase of each wave, which the model carries separately (P63, P64, P89).
- The model's Charnock constant env.charnock lies inside the spread of the measurements; COARE 3.5 (P94, P66) is now an option, `roughness="coare35"`, and the form study was rerun with it.

What can add shear near the water:

- Under swell with light wind the shear gathers below a knee or maximum at 5 to 10 m and nearly vanishes above (P65, P100, P101, P103).
- There the measured wind difference between 2.5 and 8 m was env.swell_shear_2p5_8_measured_min to env.swell_shear_2p5_8_measured_max of the wind at 8 m, against about env.swell_shear_2p5_8_log for a neutral log profile; the model's swell knee keeps the log shear below the knee, so it may understate the shear there (Q-061).
- Air warmer than the sea is stable and steepens the shear near the water: a calculation with the stability functions of P95 gives a factor of env.stability_factor_u8_cold_sea at 8 m/s and env.stability_factor_u4_cold_sea at 4 m/s for a difference of 2 K, and air colder than the sea lowers it to env.stability_factor_u8_warm_sea and env.stability_factor_u4_warm_sea; the model is neutral (Q-061).
- Swell across or against the wind can raise the drag severalfold (P98, abstract only).

What it means:

- The literature's roughness is a land value, and the open sea in neutral air has about half its shear near the water, so the form-study numbers with it are not sea values.
- The albatross model of Sachs and Bower itself (P45), gliding on the mean shear of the sea alone, needs dsopt.sachs_sea_charnock_u10_ms with Charnock's roughness and dsopt.sachs_sea_coare35_u10_ms with COARE 3.5 (geometric wingtip, self-consistent roughness), against lit.min_u10_albatross_ms with the land value; this agrees with Sachs et al., who found the mean gradient far too small for the observed flight (P47).
- The birds fill the gap in other ways: they ride the slopes of waves (P76, P77), flap less as the swell rises as well as the wind (P80), spend most of their flight energy in the little time they flap (P81), and sit on the water in light wind, taking off harder when both wind and waves are gentle (P82).
- The routes that remain towards R-02 are the same: a higher lift-to-drag ratio (K-009), the energy of the waves (Q-058), the extra shear under swell and in stable air (Q-061), the motor for the gaps (Q-055), and waiting on the water.

### Is the simulator fit to check low, wave-using flight? {#simulator-fitness}

The candidates that could reach R-02 make their low turn near the water and use the waves (see what carries the albatross), so the question is whether the sea and wind model can check such flight, not only rank it.
The answer on 2026-10-30 is that it can compare strategies but cannot yet verify a low turn within about a metre of the water (Q-064):

- Mean wind: the log profile with the roughness options holds for a wind sea in neutral air; stable air and the shear below the swell knee are missing (Q-061).
- Surface: a linear sum of components, with an optional second-order term that the flight model does not use; real crests are sharper and higher and breaking crests are not in the geometry, so a wingtip contact check on this surface is optimistic.
- Wave-coherent wind: linear potential flow, matched in form and size to the measurements from 2.6 to 15 m (P63); below 1 m it is an extrapolation, and the slow air of a sheltered trough exists only as the crude separation option.
- Turbulence: a frozen von Karman field with a fixed length scale of env.turb_length_m, scaled to the offshore intensity; near the water the eddies should shrink with height, which the model does not do.
- Ground effect: being added to the flight model; no measurement covers a banked wing over waves.
- Sensors: the altimeters return the point under the beam axis and a forward scanner looks ahead; nothing measures the height under the low wingtip, and footprint, spray and the limits of prediction are only partly modelled.

What was added on 2026-10-30 (`sim/seawind.py`, opt-in; `python -m sim.seawind_fitness`, `sim/out/seawind_fitness.json`):

- Second-order crests in the surface used by the contact check and the wind: they match the short-crested crest distribution of Forristall (S4) to env.crest_stokes2_over_forristall_1e2 at a probability of 1e-2, where the linear surface gives env.crest_linear_over_forristall_1e2.
- Breaking of dominant waves with the probability of Banner et al. (P125), lifting the crest towards a Stokes corner (P126) by at most env.breaking_lift_max_frac of the local amplitude and slowing the air in its lee (P92, P93); the marked share of the surface is only about env.breaking_marked_over_whitecap_u10 of the whitecap cover at 10 m/s (P127), since it counts only dominant waves.
- The wave-coherent wind evaluated at each height when the critical-height floor is on, so it changes sign at the critical height as measured and simulated (P128, P129), with the amplitudes and decays of P63; the floor turns the zero crossing into a small jump of about 0.05 m/s, which a flight model's central differences would see as a spike of shear (Q-064); without the floor the P63 preset keeps the wind speed at the height 1/k and does not change sign.
- Turbulence with length scales that shrink near the water, fitted to the marine spectral peaks (P132): the vertical scale is env.turb_lw_at_1m_height_law_m at 1 m.
- Stability by the functions of P95: at an Obukhov length of +50 m, stable air over a cold sea, the shear from 1 to 15 m at 9 m/s is env.stability_l50_shear_factor times neutral and over a 1.5 to 19 m cycle env.stability_l50_cycle_shear_factor; at -50 m it is env.stability_lm50_shear_factor; an independent profile reproduces all three (K-012).
- A swell profile with the shear below the knee as measured in the Baltic (P65, P100), giving env.swell_jet_shear_factor_min to env.swell_jet_shear_factor_max times the neutral shear from 1 to 15 m, the lower end for swell four times faster than the wind; below 1.2 m nothing is measured.
- The cycle optimizer still uses its own log or knee profile, so stability and the swell jet do not enter the minimum winds yet (Q-061).

What remains to add, in order: stability and the swell jet in the cycle optimizer's profile (Q-061); flights in the full near-water sea, with the second-order surface in every contact check; and the wingtip sensors of the next section in the closed loop.
After that, only a flight test can close it: low turns over a lake or a calm sea with RTK and a laser altimeter logging the height and the wind.

### Viewer (`sim/viewer/index.html`) {#viewer}

- `python -m sim.export_field --u10 10 --from 270` writes `sim/viewer/field.js` with the wind parameters and the wave components.
- Opening `sim/viewer/index.html` in a browser shows the sea, a wind mast with arrows at twelve heights above the local surface, particles advected by the wind, the wind-speed and vertical-wind profiles at the mast, and an aircraft trajectory when `sim/viewer/traj.js` exists.
- The viewer mirrors the Python formulas; it is a picture of the model, not a second model.
- Play runs the sea and the wind of the storm scenario through its hours; the Flight panel launches an aircraft at any moment of it, at the mast or at a clicked point of the sea, with a chosen candidate, cycle and heading, and can hold the scenario hour while it flies.
- The aircraft in the viewer is the same point mass as `sim/aircraft.py`, ported to `sim/viewer/flight.js`; a test runs the same flight in Python and in the browser code and requires the states to agree (`sim/tests/test_flight_js.py`). Its parameters come from `sim/viewer/aircraft.js` (`python -m sim.export_aircraft`).
- The reference tracker of `sim/autonomy.py` and its cycles (`refs.js`, `sim.export_refs`) are gone from the viewer: MPC is the only control. The tracker stays in `sim/autonomy.py` for the Python experiments.
- The panel can also fly the aircraft under live MPC: the library-seeded planner of `sim/mpc.py` is ported to `sim/viewer/mpc.js` and runs in a Web Worker every 0.5 s of flight, with the flat-sea or the true-sea world, the actuators and the load limit; its rollouts and its update match Python (rollout cost within 5e-15, update within 1e-11, `sim/tests/test_mpc_js.py`), and the library comes from `python -m sim.export_library`.
- In node a plan took about mpc.js_plan_flat_ms with the flat world and mpc.js_plan_truth_ms with the true sea of 48 components, both within the 0.5 s interval; while the browser also draws the sea on a machine without a graphics processor the true-sea plans ran late, so the time in an ordinary browser is still to be measured (Q-065).
- Section 1 of the panel has three settings. "Scenario" is the storm scenario. "Free setting" takes the 10 m wind, its direction, the sea development (fully developed or 1 to 24 h since the wind began) and an optional swell, and rebuilds the wave components with `sim/viewer/seagen.js` after the slider rests for 150 ms (`sim/viewer/envsea.js`); the flight and the planner see the new sea at once and at the next plan. "Varying" adds a slow change of the mean wind (a seeded sum of cosines with the shape of the mesoscale spectrum, or three open-cell swings), gusts from a frozen turbulence field carried by the set wind, and short waves (period under 2 s) that follow the mean wind with the response time of the wave growth. The gust generator is a port of the turbulence of `sim/seawind.py` with the height-scaled length scales and the ISO intensity (`sim/viewer/gust.js`); its modes come from a seeded generator that `sim/gustmodes.py` repeats in Python, and `sim/tests/test_gust_js.py` requires the same values from both. The planner is given the mean wind of the moment and the wave components of the moment, not the gusts ahead.
- The target is fixed at 20 deg off downwind as in the Python experiments, and the wave-aware phase shift, the estimators and the sensors are not ported yet.

### Aircraft model (`sim/aircraft.py`, `sim/dsearch.py`) {#aircraft-model}

- The aircraft is a point mass with a lift coefficient and a bank angle as controls, a parabolic drag polar (cd0, Oswald factor, aspect ratio), and limits on lift coefficient, bank, and load factor.
- The equations are written in the air-relative frame with the wind's acceleration along the path as a pseudo-force, which is where dynamic soaring gets its energy.
- `sim/dsearch.py` searches a periodic bank and lift schedule that gains air-relative energy over one cycle without going below ds.clearance_min_m or above ds.n_limit, and bisects the smallest U10 at which such a cycle exists.
- This search is the yardstick of the form study: for each candidate form, the smallest U10 that sustains soaring, and the ground speed it gives.

### Roll and pitch authority {#roll-authority}

The point mass takes its bank angle and lift coefficient as controls, and until this pass it could change them as fast as it liked; the optimal cycles used bank rates near 100 deg/s.
How fast a glider can roll scales with its speed over its span, so the measure is the non-dimensional roll rate p b/(2V).

What the sources give:

- A floor for piloted aircraft: below lit.gilruth_pb2v_min pilots always found the roll unsatisfactory (P106); military flying qualities ask a light aircraft to bank 60 deg in 1.3 s (P104).
- A 1.58 m glider flying autonomous circles reached p b/(2V) of lit.bronz_pb2v_p99_min to lit.bronz_pb2v_p99_max and a lift-coefficient rate of up to lit.bronz_cl_rate_p99_max_per_s at the 99th percentile of its logs (P87, computed from the logs).
- Wandering albatrosses roll through a half cycle at a mean rate with a median of lit.albatross_bank_rate_mean_median_deg_s, with peaks of lit.albatross_bank_rate_peak_median_deg_s (median) and lit.albatross_bank_rate_peak_p90_deg_s (90th percentile), from bird-borne loggers (P108, computed from the data).
- Bower's optimizer limited the roll rate to lit.bower_bank_rate_limit_deg_s and the lift-coefficient rate to lit.bower_cl_rate_limit_per_s per second and found them of little importance above about those values (P45).
- A small UAV identified in flight had actuators of about 50 rad/s and a delay of 0.05 s (P107).

What the models now assume (Q-062): a roll rate of ctrl.roll_pb2v in p b/(2V), a lift-coefficient rate of ctrl.cl_rate_max_per_s per second, and actuators with a lag of ctrl.actuator_tau_s and a delay of ctrl.actuator_delay_s; for a 2 m glider at 12 m/s this is about 55 deg/s, and for a 3 m glider about 37 deg/s.
The optimizer holds these limits between nodes, and the flight model, the tracker and the receding-horizon planner fly through the actuators and the load-factor limit.

What it costs, on a flat sea with the corrected settings (`python -m sim.dsopt --constraint-study` and `--constraint-study2`, `sim/out/dsopt_constraints.json`):

- Holding the heights between nodes raises the minimum wind to dsopt.midpoint_u10_baseline_ms for the baseline and dsopt.midpoint_u10_albatross_ms for the albatross-like glider.
- The roll limit at ctrl.roll_pb2v adds little more: dsopt.pb2v008_u10_baseline_ms and dsopt.pb2v008_u10_albatross_ms; only a limit as low as 30 deg/s costs about a metre per second (dsopt.roll30_u10_baseline_ms and dsopt.roll30_u10_albatross_ms).
- Margins for control cost more: with a lowest point of 2 m, a load factor of 0.8 of the limit, 0.9 of the bank limit and 1.2 times the stall speed, the minimum wind becomes dsopt.margin2_u10_baseline_ms and dsopt.margin2_u10_albatross_ms, and with 3 m, 0.7, 0.85 and 1.3 times it becomes dsopt.margin3_u10_baseline_ms and dsopt.margin3_u10_albatross_ms.
- Flying lower does not close the gap (`formstudy.clearance_study()`, `sim/out/dsopt_clearance.json`): with COARE 3.5, lowering the floor from 1.5 m to 0.3 m and the wingtip clearance from 0.3 to 0.05 m takes the Sachs albatross model from dsopt.sachs_coare_floor15_u10_ms to dsopt.sachs_coare_floor03_u10_ms and the albatross-like glider from dsopt.albatross_coare_floor15_u10_ms to dsopt.albatross_coare_floor03_u10_ms, so the thin layer of steep log shear near the water is not what the birds lack; the reason is the wingtip at a steep bank (see what carries the albatross).
The figures for the albatross-like glider are single runs of a local optimizer and move by up to about 0.8 m/s with the starting guess, while the Sachs model's are stable to a few hundredths (K-011).

So the controls are not the limit of soaring; the margins a real controller needs are, and they hit the baseline harder than the albatross-like glider, whose higher lift-to-drag ratio leaves room for them.

### Ground effect and the motor {#ground-motor}

Two things the birds have and the point mass lacked: the drag saving near the water, and flapping, for which the drone has a motor.

Ground effect (`sim/groundeffect.py`):

- The commonly printed McCormick formula omits a factor of pi and understates the effect near the water (P119); the model uses the image method for a rolled wing with elliptic loading, tabulated over the height of its centre above the local surface and its bank relative to the surface, which reproduces Wieselsberger's formula (P118) within 1% and an independent reader's values within 0.4%.
- Banking at a fixed centre height increases the saving (at half a span above the water the ratio falls from about 0.91 level to about 0.83 at 60 deg), but at a fixed wingtip clearance a steep bank removes most of it, which is the case of the low turn (P47 says the same).
- Birds save about 15 to 30% of their flight power in ground effect (P119, P121, P122, P123); it reduces the drag and is not an energy source (P47).
- In the cycle optimizer (`formstudy`, `sim/out/dsopt_ground_motor.json`) it lowers the minimum wind of the Sachs albatross model from dsopt.ge_sachs_off_u10_ms to dsopt.ge_sachs_on_u10_ms, of the albatross-like glider from dsopt.ge_albatross_off_u10_ms to dsopt.ge_albatross_on_u10_ms and of the baseline from dsopt.ge_baseline_off_u10_ms to dsopt.ge_baseline_on_u10_ms; even the impossible bound without a wingtip constraint moves only from dsopt.ge_sachs_notip01_off_u10_ms to dsopt.ge_sachs_notip01_on_u10_ms.
- So ground effect is real but small here, and it is not what separates the model from the birds.

The motor:

- A thrust control with the propulsion values of the sources: a propeller efficiency of prop.prop_eff and a motor and controller efficiency of prop.motor_eff (P140 to P144, P49), and a folded-propeller drag area of prop.folded_drag_cda_m2 when it is off (P145, P146); the smooth switch between the two keeps about a fifth of that drag while the motor runs.
- The optimizer's minimum-power mode finds the cycle that needs the least average electric power at a given wind, on a track 20 deg off downwind as the control library uses, holding at least half the best-glide speed along it; in calm air the best "cycle" is level flight at the speed floor, so the saving against a steady flap-glide at the best-glide speed comes from the slower speed and from ground effect (K-012).
- In calm air the baseline needs motor.baseline_calm_w without ground effect, about 0.7 W less with it (K-012), and the albatross-like glider motor.albatross_calm_w with it; at 6 m/s motor.baseline_u6_w (without) and motor.albatross_u6_w (with); at 9 m/s motor.baseline_u9_w and motor.albatross_u9_w (with); at its soaring minimum nothing; a steady flap-glide needs motor.baseline_flapglide_w and motor.albatross_flapglide_w at every wind. Each figure is a converged single local run after the ground-effect table was corrected; some neighbouring rows stopped at the iteration limit with a small violation and are not used.
- A calm day of motor flight therefore costs about motor.baseline_calm_day_wh or motor.albatross_calm_day_wh, far beyond what the solar film yields (Q-012), so the drone cannot motor through a calm day; it has to soar, rest on the water, or carry the energy (Q-055).

### Structural model (`sim/structure.py`) {#structural-model}

- The wing is a cantilever half-span with an elliptical lift distribution; a round carbon tube spar carries all bending; the skin and core carry none.
- The spar is sized so that the stress at the root under ds.n_limit times ds.safety_factor stays under the allowable, and fits inside the airfoil.
- Component masses placed in body axes give the total mass, the centre of gravity, and the inertia tensor that the flight model will use when it grows from a point mass to six degrees of freedom.
- The carbon tube values are sourced (P54); the skin, core and component inputs are assumed (Q-015).

## Wind and swell climate {#wind-climate}

The battery, the time the drone spends floating, and the share of time the swell can carry it depend on how often and for how long the wind stays below what the drone can soar on.
Albatrosses meet the same question by soaring when the wind serves, sitting on the water in calms, choosing routes by the wind, and using swell (P61, P62, P73, P80, P82); this section gives the drone the same numbers for the seas it may fly over.

### Data and points

- ERA5 (P165), hourly, from 2005 to 2024 in UTC: the 10 m wind speed through the Open-Meteo historical weather API (P166) at 0.25 deg, and the significant wave height with its mean and peak period through the Open-Meteo marine API with the ERA5 ocean model (P167) at 0.5 deg.
- ERA5 through that service has no swell and wind-sea partition (P167), so the swell is estimated (see the method).
- Three Southern Ocean points in the wandering albatross's range: open water north of the Crozet Islands where the birds breeding there forage; the latitude of the record's insolation point (P51) but upwind of the Kerguelen Islands in the westerlies, out of their wake; and the open South Pacific sector of the circumpolar range, far from land.
- Three North Pacific points in the Laysan and black-footed albatross range: the subtropical gyre north-east of the Hawaiian colonies, where breeding birds forage; the transition zone, the main foraging belt of both species; and the north-west Pacific south-east of the Kuril Islands, at the edge of the subarctic gyre north of the Kuroshio extension, a winter foraging area.
- Their coordinates are in `sim/climate.py` (`PUBLIC_POINTS`); all lie on the 0.5 deg grid, which both ERA5 grids share.
- The home operating area (private:loc-01): open-sea points on the same grid, kept with their raw downloads in the owner's private store; only statistics derived from them enter the record and this document.
- Each region pools its points: spells are found at each point and their lengths pooled, and the shares count the hours of all points.
- Weather data by [Open-Meteo.com](https://open-meteo.com/) (CC BY 4.0); contains modified Copernicus Climate Change Service information (ERA5).

### Method (`sim/climate.py`)

- `python -m sim.climate fetch` downloads each series once into a cache (one request every few minutes, within the free API's limits), `python -m sim.climate` writes `sim/out/climate.json`, and `python -m sim.climate_keys` writes the keys below and the tables of this section.
- Seasons are calendar months: DJF, MAM, JJA and SON, so DJF is the austral summer.
- Four thresholds of the 10 m wind: 6 m/s; ds.min_u10_ms, the design point of R-02; skim.ar20l_dih10_h03_c000_u10_ms, the best soaring minimum of the wingtip skim study, the best wing with 10 deg of dihedral and the skid touching the water (see the wingtip skim); and wing.best_u10_min_ms, the best soaring minimum on a flat sea.
- A spell is a run of consecutive hours with the wind below the threshold; a spell belongs to the season of its first hour, and its percentiles count spells, not hours.
- Swell during weak wind: the total height Hs, and a lower bound on the swell height, the part of Hs that the largest wind sea the hour's wind can hold does not explain (the fully developed sea of the environment model, with env.pm_hs_coefficient, removed in energy), a lower bound as long as the wind sea does not exceed that fully developed sea (K-019 found the share a point or two lower with a larger one); in weak wind the two are close, since the fully developed sea is small; the period is the median peak period of the weak hours with at least 2 m of that swell.
- Energy to ride out a spell on the motor: the motor's electric power plus ds.avionics_power_w over the spell; floating: ds.avionics_power_w alone over the same time; no solar yield is credited.
- The motor power: for the albatross-like glider motor.albatross_calm_w, its calm-air level flight with ground effect, over spells below ds.min_u10_ms (the drone as R-02 would have it); for the best wing of the wing sizing wing.front1_c9_w, level flight at the speed floor on the control library's track 20 deg off downwind (see the wing sizing), which stands for weak wind on that track, over spells below wing.best_u10_min_ms (the drone as it stands); straight upwind it needs wing.front1_c9_chi180_w, and its calm-air figure is not in the record.
- Cell mass at lit.cell_p45b_wh_per_kg, before any margin for ageing (ds.eol_capacity_fraction) or the storage charge.
- Daily solar yield on the darkest design day, with the cell assumptions of the flight patterns study over the wing area (form.albatross_3m.wing_area_m2; the best wing's from wing.best_span_m and wing.best_ar), from lit.insolation_50s_june_kwh_m2_day for the Southern Ocean and lit.insolation_home_december_kwh_m2_day for the home area; the record has no insolation for the North Pacific points.

### Results

The wind by season, and the share of hours below each threshold:

<!-- climate_keys wind begin -->
| region | season | mean | 10th | median | 90th | below 6 m/s | below ds.min_u10_ms | below skim.ar20l_dih10_h03_c000_u10_ms | below wing.best_u10_min_ms |
|---|---|---|---|---|---|---|---|---|---|
| Southern Ocean | DJF | climate.so_djf_u10_mean_ms | climate.so_djf_u10_p10_ms | climate.so_djf_u10_p50_ms | climate.so_djf_u10_p90_ms | climate.so_djf_below_u6_share | climate.so_djf_below_dsmin_share | climate.so_djf_below_skim_share | climate.so_djf_below_best_share |
| Southern Ocean | MAM | climate.so_mam_u10_mean_ms | climate.so_mam_u10_p10_ms | climate.so_mam_u10_p50_ms | climate.so_mam_u10_p90_ms | climate.so_mam_below_u6_share | climate.so_mam_below_dsmin_share | climate.so_mam_below_skim_share | climate.so_mam_below_best_share |
| Southern Ocean | JJA | climate.so_jja_u10_mean_ms | climate.so_jja_u10_p10_ms | climate.so_jja_u10_p50_ms | climate.so_jja_u10_p90_ms | climate.so_jja_below_u6_share | climate.so_jja_below_dsmin_share | climate.so_jja_below_skim_share | climate.so_jja_below_best_share |
| Southern Ocean | SON | climate.so_son_u10_mean_ms | climate.so_son_u10_p10_ms | climate.so_son_u10_p50_ms | climate.so_son_u10_p90_ms | climate.so_son_below_u6_share | climate.so_son_below_dsmin_share | climate.so_son_below_skim_share | climate.so_son_below_best_share |
| Southern Ocean | year | climate.so_year_u10_mean_ms | climate.so_year_u10_p10_ms | climate.so_year_u10_p50_ms | climate.so_year_u10_p90_ms | climate.so_year_below_u6_share | climate.so_year_below_dsmin_share | climate.so_year_below_skim_share | climate.so_year_below_best_share |
| North Pacific | DJF | climate.np_djf_u10_mean_ms | climate.np_djf_u10_p10_ms | climate.np_djf_u10_p50_ms | climate.np_djf_u10_p90_ms | climate.np_djf_below_u6_share | climate.np_djf_below_dsmin_share | climate.np_djf_below_skim_share | climate.np_djf_below_best_share |
| North Pacific | MAM | climate.np_mam_u10_mean_ms | climate.np_mam_u10_p10_ms | climate.np_mam_u10_p50_ms | climate.np_mam_u10_p90_ms | climate.np_mam_below_u6_share | climate.np_mam_below_dsmin_share | climate.np_mam_below_skim_share | climate.np_mam_below_best_share |
| North Pacific | JJA | climate.np_jja_u10_mean_ms | climate.np_jja_u10_p10_ms | climate.np_jja_u10_p50_ms | climate.np_jja_u10_p90_ms | climate.np_jja_below_u6_share | climate.np_jja_below_dsmin_share | climate.np_jja_below_skim_share | climate.np_jja_below_best_share |
| North Pacific | SON | climate.np_son_u10_mean_ms | climate.np_son_u10_p10_ms | climate.np_son_u10_p50_ms | climate.np_son_u10_p90_ms | climate.np_son_below_u6_share | climate.np_son_below_dsmin_share | climate.np_son_below_skim_share | climate.np_son_below_best_share |
| North Pacific | year | climate.np_year_u10_mean_ms | climate.np_year_u10_p10_ms | climate.np_year_u10_p50_ms | climate.np_year_u10_p90_ms | climate.np_year_below_u6_share | climate.np_year_below_dsmin_share | climate.np_year_below_skim_share | climate.np_year_below_best_share |
| home area | DJF | climate.home_djf_u10_mean_ms | climate.home_djf_u10_p10_ms | climate.home_djf_u10_p50_ms | climate.home_djf_u10_p90_ms | climate.home_djf_below_u6_share | climate.home_djf_below_dsmin_share | climate.home_djf_below_skim_share | climate.home_djf_below_best_share |
| home area | MAM | climate.home_mam_u10_mean_ms | climate.home_mam_u10_p10_ms | climate.home_mam_u10_p50_ms | climate.home_mam_u10_p90_ms | climate.home_mam_below_u6_share | climate.home_mam_below_dsmin_share | climate.home_mam_below_skim_share | climate.home_mam_below_best_share |
| home area | JJA | climate.home_jja_u10_mean_ms | climate.home_jja_u10_p10_ms | climate.home_jja_u10_p50_ms | climate.home_jja_u10_p90_ms | climate.home_jja_below_u6_share | climate.home_jja_below_dsmin_share | climate.home_jja_below_skim_share | climate.home_jja_below_best_share |
| home area | SON | climate.home_son_u10_mean_ms | climate.home_son_u10_p10_ms | climate.home_son_u10_p50_ms | climate.home_son_u10_p90_ms | climate.home_son_below_u6_share | climate.home_son_below_dsmin_share | climate.home_son_below_skim_share | climate.home_son_below_best_share |
| home area | year | climate.home_year_u10_mean_ms | climate.home_year_u10_p10_ms | climate.home_year_u10_p50_ms | climate.home_year_u10_p90_ms | climate.home_year_below_u6_share | climate.home_year_below_dsmin_share | climate.home_year_below_skim_share | climate.home_year_below_best_share |
<!-- climate_keys wind end -->

The spells over the whole year, in hours; the percentiles count spells, the next to last column says how many spells a year at one point are longer than the 99th percentile spell, and the last gives the 99th percentile by hours, the length that 99% of the weak hours do not exceed in their spell (K-019):

<!-- climate_keys spells begin -->
| region | below | median | 90th | 99th | longest | share of the weak hours in spells of 24 h or more | spells a year longer than the 99th | 99th by hours |
|---|---|---|---|---|---|---|---|---|
| Southern Ocean | 6 m/s | climate.so_spell_u6_p50_h | climate.so_spell_u6_p90_h | climate.so_spell_u6_p99_h | climate.so_spell_u6_max_h | climate.so_spell_u6_ge24h_share | climate.so_spell_u6_per_year_over_p99 | climate.so_spell_u6_hw_p99_h |
| Southern Ocean | ds.min_u10_ms | climate.so_spell_dsmin_p50_h | climate.so_spell_dsmin_p90_h | climate.so_spell_dsmin_p99_h | climate.so_spell_dsmin_max_h | climate.so_spell_dsmin_ge24h_share | climate.so_spell_dsmin_per_year_over_p99 | climate.so_spell_dsmin_hw_p99_h |
| Southern Ocean | skim.ar20l_dih10_h03_c000_u10_ms | climate.so_spell_skim_p50_h | climate.so_spell_skim_p90_h | climate.so_spell_skim_p99_h | climate.so_spell_skim_max_h | climate.so_spell_skim_ge24h_share | climate.so_spell_skim_per_year_over_p99 | climate.so_spell_skim_hw_p99_h |
| Southern Ocean | wing.best_u10_min_ms | climate.so_spell_best_p50_h | climate.so_spell_best_p90_h | climate.so_spell_best_p99_h | climate.so_spell_best_max_h | climate.so_spell_best_ge24h_share | climate.so_spell_best_per_year_over_p99 | climate.so_spell_best_hw_p99_h |
| North Pacific | 6 m/s | climate.np_spell_u6_p50_h | climate.np_spell_u6_p90_h | climate.np_spell_u6_p99_h | climate.np_spell_u6_max_h | climate.np_spell_u6_ge24h_share | climate.np_spell_u6_per_year_over_p99 | climate.np_spell_u6_hw_p99_h |
| North Pacific | ds.min_u10_ms | climate.np_spell_dsmin_p50_h | climate.np_spell_dsmin_p90_h | climate.np_spell_dsmin_p99_h | climate.np_spell_dsmin_max_h | climate.np_spell_dsmin_ge24h_share | climate.np_spell_dsmin_per_year_over_p99 | climate.np_spell_dsmin_hw_p99_h |
| North Pacific | skim.ar20l_dih10_h03_c000_u10_ms | climate.np_spell_skim_p50_h | climate.np_spell_skim_p90_h | climate.np_spell_skim_p99_h | climate.np_spell_skim_max_h | climate.np_spell_skim_ge24h_share | climate.np_spell_skim_per_year_over_p99 | climate.np_spell_skim_hw_p99_h |
| North Pacific | wing.best_u10_min_ms | climate.np_spell_best_p50_h | climate.np_spell_best_p90_h | climate.np_spell_best_p99_h | climate.np_spell_best_max_h | climate.np_spell_best_ge24h_share | climate.np_spell_best_per_year_over_p99 | climate.np_spell_best_hw_p99_h |
| home area | 6 m/s | climate.home_spell_u6_p50_h | climate.home_spell_u6_p90_h | climate.home_spell_u6_p99_h | climate.home_spell_u6_max_h | climate.home_spell_u6_ge24h_share | climate.home_spell_u6_per_year_over_p99 | climate.home_spell_u6_hw_p99_h |
| home area | ds.min_u10_ms | climate.home_spell_dsmin_p50_h | climate.home_spell_dsmin_p90_h | climate.home_spell_dsmin_p99_h | climate.home_spell_dsmin_max_h | climate.home_spell_dsmin_ge24h_share | climate.home_spell_dsmin_per_year_over_p99 | climate.home_spell_dsmin_hw_p99_h |
| home area | skim.ar20l_dih10_h03_c000_u10_ms | climate.home_spell_skim_p50_h | climate.home_spell_skim_p90_h | climate.home_spell_skim_p99_h | climate.home_spell_skim_max_h | climate.home_spell_skim_ge24h_share | climate.home_spell_skim_per_year_over_p99 | climate.home_spell_skim_hw_p99_h |
| home area | wing.best_u10_min_ms | climate.home_spell_best_p50_h | climate.home_spell_best_p90_h | climate.home_spell_best_p99_h | climate.home_spell_best_max_h | climate.home_spell_best_ge24h_share | climate.home_spell_best_per_year_over_p99 | climate.home_spell_best_hw_p99_h |
<!-- climate_keys spells end -->

The waves during the hours below each threshold, over the whole year (shares of those hours):

<!-- climate_keys swell begin -->
| region | wind below | Hs 2 m or more | Hs 3 m or more | swell 2 m or more | swell 3 m or more | median peak period with swell 2 m or more | Hs at or above ds.survival_hs_m |
|---|---|---|---|---|---|---|---|
| Southern Ocean | 6 m/s | climate.so_weak_u6_hs2_share | climate.so_weak_u6_hs3_share | climate.so_weak_u6_swell2_share | climate.so_weak_u6_swell3_share | climate.so_weak_u6_tp_s | climate.so_weak_u6_hs_survival_share |
| Southern Ocean | ds.min_u10_ms | climate.so_weak_dsmin_hs2_share | climate.so_weak_dsmin_hs3_share | climate.so_weak_dsmin_swell2_share | climate.so_weak_dsmin_swell3_share | climate.so_weak_dsmin_tp_s | climate.so_weak_dsmin_hs_survival_share |
| Southern Ocean | skim.ar20l_dih10_h03_c000_u10_ms | climate.so_weak_skim_hs2_share | climate.so_weak_skim_hs3_share | climate.so_weak_skim_swell2_share | climate.so_weak_skim_swell3_share | climate.so_weak_skim_tp_s | climate.so_weak_skim_hs_survival_share |
| Southern Ocean | wing.best_u10_min_ms | climate.so_weak_best_hs2_share | climate.so_weak_best_hs3_share | climate.so_weak_best_swell2_share | climate.so_weak_best_swell3_share | climate.so_weak_best_tp_s | climate.so_weak_best_hs_survival_share |
| Southern Ocean | all hours |  |  |  |  |  | climate.so_year_hs_survival_share |
| North Pacific | 6 m/s | climate.np_weak_u6_hs2_share | climate.np_weak_u6_hs3_share | climate.np_weak_u6_swell2_share | climate.np_weak_u6_swell3_share | climate.np_weak_u6_tp_s | climate.np_weak_u6_hs_survival_share |
| North Pacific | ds.min_u10_ms | climate.np_weak_dsmin_hs2_share | climate.np_weak_dsmin_hs3_share | climate.np_weak_dsmin_swell2_share | climate.np_weak_dsmin_swell3_share | climate.np_weak_dsmin_tp_s | climate.np_weak_dsmin_hs_survival_share |
| North Pacific | skim.ar20l_dih10_h03_c000_u10_ms | climate.np_weak_skim_hs2_share | climate.np_weak_skim_hs3_share | climate.np_weak_skim_swell2_share | climate.np_weak_skim_swell3_share | climate.np_weak_skim_tp_s | climate.np_weak_skim_hs_survival_share |
| North Pacific | wing.best_u10_min_ms | climate.np_weak_best_hs2_share | climate.np_weak_best_hs3_share | climate.np_weak_best_swell2_share | climate.np_weak_best_swell3_share | climate.np_weak_best_tp_s | climate.np_weak_best_hs_survival_share |
| North Pacific | all hours |  |  |  |  |  | climate.np_year_hs_survival_share |
| home area | 6 m/s | climate.home_weak_u6_hs2_share | climate.home_weak_u6_hs3_share | climate.home_weak_u6_swell2_share | climate.home_weak_u6_swell3_share | climate.home_weak_u6_tp_s | climate.home_weak_u6_hs_survival_share |
| home area | ds.min_u10_ms | climate.home_weak_dsmin_hs2_share | climate.home_weak_dsmin_hs3_share | climate.home_weak_dsmin_swell2_share | climate.home_weak_dsmin_swell3_share | climate.home_weak_dsmin_tp_s | climate.home_weak_dsmin_hs_survival_share |
| home area | skim.ar20l_dih10_h03_c000_u10_ms | climate.home_weak_skim_hs2_share | climate.home_weak_skim_hs3_share | climate.home_weak_skim_swell2_share | climate.home_weak_skim_swell3_share | climate.home_weak_skim_tp_s | climate.home_weak_skim_hs_survival_share |
| home area | wing.best_u10_min_ms | climate.home_weak_best_hs2_share | climate.home_weak_best_hs3_share | climate.home_weak_best_swell2_share | climate.home_weak_best_swell3_share | climate.home_weak_best_tp_s | climate.home_weak_best_hs_survival_share |
| home area | all hours |  |  |  |  |  | climate.home_year_hs_survival_share |
<!-- climate_keys swell end -->

The energy to ride out the spells of the whole year on the motor or floating, in Wh, and the cells for the 99th percentile spell, in kg:

<!-- climate_keys energy begin -->
| region | aircraft (spells below) | motor, median spell | motor, 90th | motor, 99th | floating, median spell | floating, 90th | floating, 99th | cells for the motor, 99th | cells for floating, 99th |
|---|---|---|---|---|---|---|---|---|---|
| Southern Ocean | albatross-like glider (ds.min_u10_ms) | climate.so_albatross_motor_p50_wh | climate.so_albatross_motor_p90_wh | climate.so_albatross_motor_p99_wh | climate.so_albatross_float_p50_wh | climate.so_albatross_float_p90_wh | climate.so_albatross_float_p99_wh | climate.so_albatross_motor_p99_cell_kg | climate.so_albatross_float_p99_cell_kg |
| Southern Ocean | best wing (wing.best_u10_min_ms) | climate.so_best_motor_p50_wh | climate.so_best_motor_p90_wh | climate.so_best_motor_p99_wh | climate.so_best_float_p50_wh | climate.so_best_float_p90_wh | climate.so_best_float_p99_wh | climate.so_best_motor_p99_cell_kg | climate.so_best_float_p99_cell_kg |
| North Pacific | albatross-like glider (ds.min_u10_ms) | climate.np_albatross_motor_p50_wh | climate.np_albatross_motor_p90_wh | climate.np_albatross_motor_p99_wh | climate.np_albatross_float_p50_wh | climate.np_albatross_float_p90_wh | climate.np_albatross_float_p99_wh | climate.np_albatross_motor_p99_cell_kg | climate.np_albatross_float_p99_cell_kg |
| North Pacific | best wing (wing.best_u10_min_ms) | climate.np_best_motor_p50_wh | climate.np_best_motor_p90_wh | climate.np_best_motor_p99_wh | climate.np_best_float_p50_wh | climate.np_best_float_p90_wh | climate.np_best_float_p99_wh | climate.np_best_motor_p99_cell_kg | climate.np_best_float_p99_cell_kg |
| home area | albatross-like glider (ds.min_u10_ms) | climate.home_albatross_motor_p50_wh | climate.home_albatross_motor_p90_wh | climate.home_albatross_motor_p99_wh | climate.home_albatross_float_p50_wh | climate.home_albatross_float_p90_wh | climate.home_albatross_float_p99_wh | climate.home_albatross_motor_p99_cell_kg | climate.home_albatross_float_p99_cell_kg |
| home area | best wing (wing.best_u10_min_ms) | climate.home_best_motor_p50_wh | climate.home_best_motor_p90_wh | climate.home_best_motor_p99_wh | climate.home_best_float_p50_wh | climate.home_best_float_p90_wh | climate.home_best_float_p99_wh | climate.home_best_motor_p99_cell_kg | climate.home_best_float_p99_cell_kg |
<!-- climate_keys energy end -->

The energy to ride out the 90th and 99th percentile spells by season, in Wh (spells counted by the season of their first hour); these are percentiles of spells, so a year aloft meets spells longer than them (the spells table):

<!-- climate_keys battery begin -->
| region | season | albatross-like glider: motor, 90th | albatross-like glider: motor, 99th | albatross-like glider: floating, 90th | albatross-like glider: floating, 99th | best wing: motor, 90th | best wing: motor, 99th | best wing: floating, 90th | best wing: floating, 99th |
|---|---|---|---|---|---|---|---|---|---|
| Southern Ocean | DJF | climate.so_djf_albatross_motor_p90_wh | climate.so_djf_albatross_motor_p99_wh | climate.so_djf_albatross_float_p90_wh | climate.so_djf_albatross_float_p99_wh | climate.so_djf_best_motor_p90_wh | climate.so_djf_best_motor_p99_wh | climate.so_djf_best_float_p90_wh | climate.so_djf_best_float_p99_wh |
| Southern Ocean | MAM | climate.so_mam_albatross_motor_p90_wh | climate.so_mam_albatross_motor_p99_wh | climate.so_mam_albatross_float_p90_wh | climate.so_mam_albatross_float_p99_wh | climate.so_mam_best_motor_p90_wh | climate.so_mam_best_motor_p99_wh | climate.so_mam_best_float_p90_wh | climate.so_mam_best_float_p99_wh |
| Southern Ocean | JJA | climate.so_jja_albatross_motor_p90_wh | climate.so_jja_albatross_motor_p99_wh | climate.so_jja_albatross_float_p90_wh | climate.so_jja_albatross_float_p99_wh | climate.so_jja_best_motor_p90_wh | climate.so_jja_best_motor_p99_wh | climate.so_jja_best_float_p90_wh | climate.so_jja_best_float_p99_wh |
| Southern Ocean | SON | climate.so_son_albatross_motor_p90_wh | climate.so_son_albatross_motor_p99_wh | climate.so_son_albatross_float_p90_wh | climate.so_son_albatross_float_p99_wh | climate.so_son_best_motor_p90_wh | climate.so_son_best_motor_p99_wh | climate.so_son_best_float_p90_wh | climate.so_son_best_float_p99_wh |
| North Pacific | DJF | climate.np_djf_albatross_motor_p90_wh | climate.np_djf_albatross_motor_p99_wh | climate.np_djf_albatross_float_p90_wh | climate.np_djf_albatross_float_p99_wh | climate.np_djf_best_motor_p90_wh | climate.np_djf_best_motor_p99_wh | climate.np_djf_best_float_p90_wh | climate.np_djf_best_float_p99_wh |
| North Pacific | MAM | climate.np_mam_albatross_motor_p90_wh | climate.np_mam_albatross_motor_p99_wh | climate.np_mam_albatross_float_p90_wh | climate.np_mam_albatross_float_p99_wh | climate.np_mam_best_motor_p90_wh | climate.np_mam_best_motor_p99_wh | climate.np_mam_best_float_p90_wh | climate.np_mam_best_float_p99_wh |
| North Pacific | JJA | climate.np_jja_albatross_motor_p90_wh | climate.np_jja_albatross_motor_p99_wh | climate.np_jja_albatross_float_p90_wh | climate.np_jja_albatross_float_p99_wh | climate.np_jja_best_motor_p90_wh | climate.np_jja_best_motor_p99_wh | climate.np_jja_best_float_p90_wh | climate.np_jja_best_float_p99_wh |
| North Pacific | SON | climate.np_son_albatross_motor_p90_wh | climate.np_son_albatross_motor_p99_wh | climate.np_son_albatross_float_p90_wh | climate.np_son_albatross_float_p99_wh | climate.np_son_best_motor_p90_wh | climate.np_son_best_motor_p99_wh | climate.np_son_best_float_p90_wh | climate.np_son_best_float_p99_wh |
| home area | DJF | climate.home_djf_albatross_motor_p90_wh | climate.home_djf_albatross_motor_p99_wh | climate.home_djf_albatross_float_p90_wh | climate.home_djf_albatross_float_p99_wh | climate.home_djf_best_motor_p90_wh | climate.home_djf_best_motor_p99_wh | climate.home_djf_best_float_p90_wh | climate.home_djf_best_float_p99_wh |
| home area | MAM | climate.home_mam_albatross_motor_p90_wh | climate.home_mam_albatross_motor_p99_wh | climate.home_mam_albatross_float_p90_wh | climate.home_mam_albatross_float_p99_wh | climate.home_mam_best_motor_p90_wh | climate.home_mam_best_motor_p99_wh | climate.home_mam_best_float_p90_wh | climate.home_mam_best_float_p99_wh |
| home area | JJA | climate.home_jja_albatross_motor_p90_wh | climate.home_jja_albatross_motor_p99_wh | climate.home_jja_albatross_float_p90_wh | climate.home_jja_albatross_float_p99_wh | climate.home_jja_best_motor_p90_wh | climate.home_jja_best_motor_p99_wh | climate.home_jja_best_float_p90_wh | climate.home_jja_best_float_p99_wh |
| home area | SON | climate.home_son_albatross_motor_p90_wh | climate.home_son_albatross_motor_p99_wh | climate.home_son_albatross_float_p90_wh | climate.home_son_albatross_float_p99_wh | climate.home_son_best_motor_p90_wh | climate.home_son_best_motor_p99_wh | climate.home_son_best_float_p90_wh | climate.home_son_best_float_p99_wh |
<!-- climate_keys battery end -->

The daily solar yield on the darkest design day is climate.solar_day_albatross_so_wh and climate.solar_day_best_so_wh in the Southern Ocean, and climate.solar_day_albatross_home_wh and climate.solar_day_best_home_wh in the home area, against climate.avionics_day_wh for the avionics.

`sim/out/climate.json` also holds each public point on its own, the spells by season for every threshold, the spells with breaks shorter than 3 h joined, the 10th and 90th percentile of the swell period, and the floating energy net of the darkest-day solar yield.

### What it means

The motor:

- On the motor even the median spell below ds.min_u10_ms costs the albatross-like glider climate.so_albatross_motor_p50_wh in the Southern Ocean, climate.np_albatross_motor_p50_wh in the North Pacific and climate.home_albatross_motor_p50_wh in the home area, and the 99th percentile spell needs climate.so_albatross_motor_p99_cell_kg, climate.np_albatross_motor_p99_cell_kg and climate.home_albatross_motor_p99_cell_kg of cells, against a whole aircraft of ds.mass_max_kg.
- For the best wing as it stands the spells are longer and its motor power higher, so riding them out on the motor is further out of reach (climate.so_best_motor_p99_cell_kg of cells for the 99th percentile spell in the Southern Ocean).
- So the motor bridges hours (a gap between soaring spells, a leg to wind seen nearby, a take-off) and not calms, which confirms the estimate of a calm day in the ground-effect section (Q-055).

Floating:

- Floating, the same spells cost only the avionics.
- Below ds.min_u10_ms the 99th percentile spell takes climate.so_albatross_float_p99_wh in the Southern Ocean, climate.so_albatross_float_p99_cell_kg of cells and a fraction of ds.mass_max_kg, but climate.np_albatross_float_p99_wh in the North Pacific and climate.home_albatross_float_p99_wh in the home area, climate.np_albatross_float_p99_cell_kg and climate.home_albatross_float_p99_cell_kg of cells, close to the whole of ds.mass_max_kg, with no solar yield credited.
- Below wing.best_u10_min_ms, the drone as it stands, the 99th percentile spell on the water takes climate.so_best_float_p99_wh in the Southern Ocean, climate.np_best_float_p99_wh in the North Pacific and climate.home_best_float_p99_wh in the home area, more than a drone of ds.mass_max_kg can carry as cells; there only the sun can carry the avionics.
- A year aloft meets spells longer than the 99th percentile spell: below ds.min_u10_ms climate.so_spell_dsmin_per_year_over_p99 a year at a point in the Southern Ocean, climate.np_spell_dsmin_per_year_over_p99 in the North Pacific and climate.home_spell_dsmin_per_year_over_p99 in the home area; a battery meant to last the year has to float through the 99th percentile by hours (climate.so_spell_dsmin_hw_p99_h, climate.np_spell_dsmin_hw_p99_h and climate.home_spell_dsmin_hw_p99_h at ds.avionics_power_w), or the sun has to carry the avionics.
- So the drone's answer to a calm is the bird's, to sit on the water (P82), and the battery is sized by the floating time, the avionics draw and the sun, not by the motor (Q-012).
- The share of hours below the soaring minimum is the share of time on the water, on the motor or on the swell: below ds.min_u10_ms it is climate.so_year_below_dsmin_share in the Southern Ocean, climate.np_year_below_dsmin_share in the North Pacific and climate.home_year_below_dsmin_share in the home area, and below wing.best_u10_min_ms it is climate.so_year_below_best_share, climate.np_year_below_best_share and climate.home_year_below_best_share.
- The worst season is the local summer, DJF in the Southern Ocean and JJA in the North Pacific and the home area (the battery table); in the Southern Ocean the seasons differ little, while in the North Pacific and the home area the summer spells are several times the winter ones.

The sun:

- On the darkest design day the best wing's yield in the home area is about the avionics' daily draw, so floating there costs mainly the night; in the Southern Ocean in June it is well short of it, so each day on the water drains the battery by the difference (the floating energy net of that yield is in `sim/out/climate.json`).
- The yields rest on the pattern study's cell efficiency and coverage, which the record does not hold, and assume a clean, dry, level panel on the water (Q-012).

The swell:

- Wave study 4 found that a 3 m swell carries both kept forms with no wind and a 1 m swell gives nothing (K-012); the heights between are not yet flown.
- In the Southern Ocean swell of 3 m or more is present in climate.so_weak_dsmin_swell3_share of the hours below ds.min_u10_ms and of 2 m or more in climate.so_weak_dsmin_swell2_share, at a median peak period of climate.so_weak_dsmin_tp_s, at or above the longer of the two periods the wave study flew.
- In the North Pacific the shares are climate.np_weak_dsmin_swell3_share and climate.np_weak_dsmin_swell2_share, and in the home area only climate.home_weak_dsmin_swell3_share and climate.home_weak_dsmin_swell2_share.
- So in the Southern Ocean the swell could carry the drone through a good part of the weak wind if the wave-following loop works (Q-060), as it carries the birds; in the North Pacific through some of it; in the home area almost never.

The sea on the water:

- R-05 puts the drone on the water when the wind is too weak and asks it to survive ds.survival_hs_m; in the Southern Ocean the waves are at or above that in climate.so_weak_dsmin_hs_survival_share of the hours below ds.min_u10_ms and in climate.so_year_hs_survival_share of all hours, in the North Pacific in climate.np_weak_dsmin_hs_survival_share and climate.np_year_hs_survival_share, and in the home area in climate.home_weak_dsmin_hs_survival_share and climate.home_year_hs_survival_share (Q-009).
- So the design point of R-05 is passed on the water in a sizeable share of the weak-wind hours in the Southern Ocean, and a landing in strong wind there meets higher waves still.

The home area:

- Beside the albatross regions the home operating area is a weak-wind sea with little swell: it suits tests of floating, take-off and motor legs, while a year aloft there would be spent mostly on the water or on the motor (Q-072).

### Against the literature

- P61 gives Southern Ocean monthly means of lit.so_u10_mean_winter_ms in winter and lit.so_u10_mean_summer_ms in summer from scatterometers; the Southern Ocean points give climate.so_jja_u10_mean_ms in JJA and climate.so_djf_u10_mean_ms in DJF, a little lower in both seasons; the point at 50S in the Indian sector matches P61 and the point north of the Crozet Islands is the calmest (`sim/out/climate.json`).
- P73's albatrosses met winds of lit.bird_experienced_wind_ms on average with a spread of lit.bird_experienced_wind_sd_ms (ERA5 at the birds' positions); the Southern Ocean year mean here is climate.so_year_u10_mean_ms with a spread of climate.so_year_u10_std_ms, so the birds fly in less wind than the open belt's mean, about the summer wind of the northern point, as expected of birds tied to a colony in summer that choose their routes.
- P80 found that the birds' flapping falls with the wind in the Southern Ocean and with the swell height in the North Pacific; the numbers here fit, with weak wind far more common in the North Pacific and swell present in part of it.
- P62 found swell dominant most of the time in the Pacific; here the swell lower bound is close to the total height in weak wind in every region (the swell table).

### Limits

- ERA5 is a model at about 31 km whose 10 m wind is a grid-box mean: it smooths calms, gusts and small fronts, so real calms are patchier and real spells may be shorter in places and longer in others; the drone, unlike a fixed point, can also move towards wind as the birds do (P73).
- Spells are hourly, and one hour above the threshold breaks a spell, so the counts are inflated by the wind flickering around the threshold and the percentiles by count are short; joining breaks under 3 h lengthens them (`sim/out/climate.json`), and the share of the weak hours in spells of 24 h or more shows how much of the weak time lies in long spells.
- A spell belongs to the season of its first hour, so a long summer spell that starts in spring counts in spring.
- The thresholds are the model's flat-sea soaring minima in neutral air; stability, the shear under swell (Q-061) and the waves (Q-060) move them, and skim.ar20l_dih10_h03_c000_u10_ms rests on the skid of D-018 and D-019, whose sizes are still assumed (Q-070).
- The swell is a lower bound from the total height, not ERA5's own partition, and the wave grid is 0.5 deg; the service has no peak period from 2024-11-19 to the end of 2024 at any point, and those hours are left out of the period medians (`missing_hours` in `sim/out/climate.json`).
- No solar yield is credited in the battery figures, the cell figures carry no margin for ageing or reserve, and the darkest-day insolation is known for two places only; the Southern Ocean value at 50S is used for the whole region, including the darker point at 55S.
- Two or three points stand for each region, and the results come from the merge of main into this branch, run before the merge was committed (the commit it started from and the stamp are in `sim/out/climate.json`).

## Numbers from the literature for the form study {#literature-numbers}

The yardsticks of the form study need the bird and the demonstrators as reference points; these are the numbers the record holds (keys `lit.*`), with their sources.

- The wandering albatross weighs lit.albatross_mass_kg with a span of lit.albatross_span_m, a wing area of lit.albatross_wing_area_m2, an aspect ratio of lit.albatross_aspect_ratio, and a best lift-to-drag ratio of about lit.albatross_lift_to_drag (P42, P44, P45).
- Its soaring cycle lasts about lit.ds_cycle_period_s (reported between lit.ds_cycle_period_min_s and lit.ds_cycle_period_max_s) and spans heights up to lit.ds_cycle_height_m; the turns are shallow arcs of about lit.ds_turn_angle_deg (between lit.ds_turn_angle_min_deg and lit.ds_turn_angle_max_deg), not half circles, and the shear it uses sits in a layer about lit.shear_layer_thickness_m thick (between lit.shear_layer_thickness_min_m and lit.shear_layer_thickness_max_m) (P44, P46, P47).
- The two-layer Rayleigh model needs a wind difference of about lit.rayleigh_min_wind_difference_ms across the layer (between lit.rayleigh_min_wind_difference_min_ms and lit.rayleigh_min_wind_difference_max_ms) for a glider of the albatross's lift-to-drag ratio (P42, P43).
- With a logarithmic profile, the smallest wind at 10 m that sustains soaring is lit.min_u10_albatross_ms for the albatross model and lit.min_u10_mariner_ms for a glider of lit.mariner_mass_kg with a lift-to-drag ratio of lit.mariner_lift_to_drag and a wing loading of lit.mariner_wing_loading_n_m2, while a sailplane of lit.sailplane_mass_kg with a high wing loading needs lit.min_u10_sailplane_15kg_ms (P45, P56).
- Those numbers are why the design point of R-02 is ds.min_u10_ms, now sourced rather than assumed; the form study will test it with the simulation.
- The demonstrator closest to this project is a foam trainer of lit.easyglider_span_m span and lit.easyglider_mass_kg with a wing area of lit.easyglider_wing_area_m2 (P48).
- A hand-launched solar aircraft that flew for days drew lit.atlantiksolar_avionics_w for its avionics including a satellite modem, and was designed for a load factor of lit.atlantiksolar_load_factor with a safety factor of lit.atlantiksolar_safety_factor (P49); sailplane certification uses lit.cs22_limit_load_factor with a safety factor of lit.cs22_safety_factor (P55).
- The darkest design day gives lit.insolation_50s_june_kwh_m2_day in the Southern Ocean in June and lit.insolation_home_december_kwh_m2_day at the home design point in December (P51); a 21700 cell holds about lit.cell_p45b_wh_per_kg and still discharges down to lit.cell_p45b_min_discharge_temp_degc (P50).
- A small ultrasonic anemometer weighs lit.anemometer_trisonica_mass_g and draws lit.anemometer_trisonica_power_w (P52); a research multi-hole probe weighs lit.probe_epfl_mass_g (P53).
- Pultruded carbon tube has a modulus of only lit.pultruded_modulus_pa and a flexural strength of lit.pultruded_flexural_pa (P54), far below the values first assumed for the spar; this changed the spar sizing (docs/03-design.md#wing-spar) and opened Q-010.
- A seaplane UAV of lit.flyingfish_mass_kg has taken off from the water on its own in waves up to lit.flyingfish_wave_height_m (P57); no water take-off of a drone in the mass range of this project (ds.mass_max_kg) was found in the sources read, which is why R-05 keeps its water take-off as the open question of the sealed-hull form.

What the literature does not give: a table of the minimum wind against wing loading and lift-to-drag ratio for small gliders over a real sea profile; that is the simulation's job.

### How the single numbers were reduced from ranges

The sources give ranges for four numbers, and the record keeps both ends as sourced keys and the single number as calculated.
lit.ds_cycle_period_s is a working figure chosen by the assistant inside the range between lit.ds_cycle_period_min_s (P47) and lit.ds_cycle_period_max_s (P46), where the two papers report different cycles.
lit.ds_turn_angle_deg is the midpoint of lit.ds_turn_angle_min_deg and lit.ds_turn_angle_max_deg (P44); P74 reports turn angles that change with the wind, so the midpoint is a reference and not a design value.
lit.shear_layer_thickness_m is the midpoint of lit.shear_layer_thickness_min_m and lit.shear_layer_thickness_max_m (P44).
lit.rayleigh_min_wind_difference_ms is the midpoint of lit.rayleigh_min_wind_difference_min_ms and lit.rayleigh_min_wind_difference_max_ms (P42).

## What carries the albatross, and what the model must include {#energy-sources}

The albatross model of Sachs and Bower needs about 15 m/s of wind at 10 m in this model (see the roughness of the open sea), while tracked albatrosses meet winds of lit.bird_experienced_wind_ms on average (P73) and are seen soaring in much less (P74).
Two readings of the literature asked what fills that gap.

A correction first: Sachs et al.'s gradient of lit.sachs2013_required_gradient_per_s is what a single steady climb needs to keep its airspeed, not what a closed cycle needs; they conclude that the energy comes from turning from windward to leeward in the upper curve, that is from the wind difference between the low and the high turn, and that the low turn loses little because the wind there is slow (P47).

The mechanisms, ranked by the evidence from the birds:

- The wind difference across the cycle, with the low turn close to the water or in a trough: energy gain from the wind is measured on birds with fast GPS (P46, P47), and the birds spend much of their time below 5 m and in troughs (P115, P117); crude estimates give it most of the energy in usual winds (P42).
- Updrafts over swell: seen and correlated with flapping rather than measured at a bird (P76, P80, P82); it matters in light wind with swell and carries everything in a calm (P42, P74).
- Flapping: measured on birds, lit.bird_flapping_time_fraction of the flight time and lit.bird_flapping_energy_fraction of the energy, in light wind, into the wind and at take-off (P81, P79, P80).
- Sitting on the water until the wind returns, which is not an energy source (P82).
- Separation behind crests (gust soaring) is weak: Sachs et al. saw the energy rise smoothly without pulses and found the same cycles over flat land (P47); turbulence has no bird data, and for small gliders it pays only well above the turbulence near the sea (P112, P113, P114).

What the model says about the first one: the gap is the wingtip, not the height.

- With the wingtip held clear, lowering the floor to 0.3 m and the clearance to 0.05 m takes the Sachs model only from dsopt.sachs_coare_floor15_u10_ms to dsopt.sachs_coare_floor03_u10_ms, and its centre never goes below dsopt.sachs_coare_floor03_zmin_m, because the low turn wants a steep bank and a 3.3 m wing then reaches the water.
- Without the wingtip constraint the same model needs dsopt.sachs_coare_notip_floor15_u10_ms at a 1.5 m floor, dsopt.sachs_coare_notip_floor05_u10_ms at 0.5 m and dsopt.sachs_coare_notip_floor01_u10_ms at 0.1 m, and with the land value of z0 only dsopt.sachs_z003_notip_floor05_u10_ms and dsopt.sachs_z003_notip_floor01_u10_ms; the albatross-like glider needs dsopt.albatross_coare_notip_floor01_u10_ms at 0.1 m (`formstudy.clearance_study()`).
- So the bird gains by making its low turn in the slowest air with its lower wingtip almost on the water, which a point mass with a wingtip margin cannot do; how the bird turns low without a steep bank, and how much a wave trough slows the air there, are what the model does not yet capture, and no study has measured the wind at the bird's height (P116).
- R-12 keeps the drone's wingtip well clear, so its low turn stays higher than the bird's; the owner has decided to let the low turn bring a wingtip close to the water with a skid and a tip sensor (D-018), and what that gains and needs is in [the wingtip skim section](#wingtip-skim).

The wind difference a cycle needs, from its lowest to its highest point (`formstudy.wind_difference_study()`, `sim/out/dsopt_wind_difference.json`):

- The Sachs model with its wingtip held clear needs dsopt.sachs_coare_dw_ms whether the roughness is the sea's or the land value; the roughness only decides how much wind at 10 m gives that difference over the heights the cycle spans, dsopt.sachs_coare_dw_ratio of the wind at 10 m over the sea against dsopt.sachs_z003_dw_ratio with the land value.
- Without the wingtip constraint it needs dsopt.sachs_notip_dw_ms, close to the two-layer estimate of lit.rayleigh_min_wind_difference_ms for its lift-to-drag ratio (P42, P43); but that cycle banks steeply 0.1 m above the water, which no wing can do, so it is a bound, not a flight.
- The candidates need less because they glide better: dsopt.albatross_coare_dw_ms for the albatross-like glider and dsopt.baseline_coare_dw_ms for the baseline; the difference times the lift-to-drag ratio is roughly constant.

What the physics says about the second one: the vertical wave wind measured over swell from 2.6 to 15 m has the model's own form with an amplitude factor of lit.wave_induced_amp_w and a decay of lit.wave_induced_decay_w (P63), about 0.3 to 0.5 m/s near the surface over a 2 m swell, comparable to the sink rate of a glider; P64's single case is lower (Q-058).
The log profile itself holds within a few percent down to about 0.1 m over waves (P111).

What the model must include, in this order:

1. The wave wind, calibrated by P63 and bounded below by P64, with cycles that follow the wave phase reliably, also at long periods, and a planner that knows the waves; this is the only documented way to soar in light wind and the reason every flight over waves still ends (see the receding horizon below).
2. The motor as the drone's flapping: the energy per hour it needs to bridge the winds below the soaring minimum (Q-055).
3. Turbulence as a cost on the margins of the cycle rather than as a source.
4. The profile shapes under swell and in stable air (Q-061).
5. Separation behind crests stays a crude option and is not developed further.

## Letting a wingtip skim the water {#wingtip-skim}

The owner decided to let the low turn bring a wingtip close to the water, with a skid on each tip that sheds water and survives a touch and a sensor that detects water at the tip so that the aircraft corrects its attitude at once; R-12 is to be revised once this study gives the clearance and the reaction it needs (D-018).
This section asks how much the low tip gains, what a touch does to the aircraft, and what the tip sensor and the reflex must do; the independent check K-018 corrected its first version.

### What the low tip gains

`formstudy.skim_study()` (`sim/out/skim_study.json`) solves the minimum wind at 10 m on a flat sea with COARE 3.5 and the control library's margins other than the heights (a load factor of 0.8 of the limit, 0.9 of the bank limit, 1.2 times the stall speed, the roll and lift-coefficient rates of Q-062, the heights and the tip held between nodes), with the wingtip constraint kept and its clearance varied.
The clearances are ds.clearance_min_m (the design point of R-12), the library's 0.3 m, 0.1 m and zero, where the skid touches; the body floor runs from 2 m down to 0.3 m.
The aircraft are the albatross-like glider, the wing pass's best design (aspect ratio 20, SD7037, 3.0 m, ballast to ds.mass_max_kg, its Reynolds number iterated as in the wing pass) and the same wing without ballast.
Each cell is the best of up to six cold starts (the first three that converge) and of warm starts from its tighter neighbours, whose solutions are feasible in it; a cell that failed was tried again from its looser neighbours with six more starts, and cells where the floor lies below the clearance are the same problem as the first such floor and carry its key.

| Aircraft | Body floor | Tip clearance ds.clearance_min_m | 0.3 m | 0.1 m | 0 (the skid touches) |
|---|---|---|---|---|---|
| albatross-like glider | 2 m | skim.alb_h20_c100_u10_ms | skim.alb_h20_c030_u10_ms | skim.alb_h20_c010_u10_ms | skim.alb_h20_c000_u10_ms |
| albatross-like glider | 1 m | skim.alb_h10_c100_u10_ms | skim.alb_h10_c030_u10_ms | skim.alb_h10_c010_u10_ms | skim.alb_h10_c000_u10_ms |
| albatross-like glider | 0.5 m | skim.alb_h10_c100_u10_ms | skim.alb_h05_c030_u10_ms | skim.alb_h05_c010_u10_ms | skim.alb_h05_c000_u10_ms |
| albatross-like glider | 0.3 m | skim.alb_h10_c100_u10_ms | skim.alb_h03_c030_u10_ms | skim.alb_h03_c010_u10_ms | skim.alb_h03_c000_u10_ms |
| AR 20 SD7037, ballast to the limit | 2 m | skim.ar20l_h20_c100_u10_ms | skim.ar20l_h20_c030_u10_ms | skim.ar20l_h20_c010_u10_ms | skim.ar20l_h20_c000_u10_ms |
| AR 20 SD7037, ballast to the limit | 1 m | skim.ar20l_h10_c100_u10_ms | skim.ar20l_h10_c030_u10_ms | skim.ar20l_h10_c010_u10_ms | skim.ar20l_h10_c000_u10_ms |
| AR 20 SD7037, ballast to the limit | 0.5 m | skim.ar20l_h10_c100_u10_ms | skim.ar20l_h05_c030_u10_ms | skim.ar20l_h05_c010_u10_ms | skim.ar20l_h05_c000_u10_ms |
| AR 20 SD7037, ballast to the limit | 0.3 m | skim.ar20l_h10_c100_u10_ms | skim.ar20l_h03_c030_u10_ms | skim.ar20l_h03_c010_u10_ms | skim.ar20l_h03_c000_u10_ms |
| AR 20 SD7037, no ballast | 2 m | skim.ar20n_h20_c100_u10_ms | skim.ar20n_h20_c030_u10_ms | skim.ar20n_h20_c010_u10_ms | skim.ar20n_h20_c000_u10_ms |
| AR 20 SD7037, no ballast | 1 m | skim.ar20n_h10_c100_u10_ms | skim.ar20n_h10_c030_u10_ms | skim.ar20n_h10_c010_u10_ms | skim.ar20n_h10_c000_u10_ms |
| AR 20 SD7037, no ballast | 0.5 m | skim.ar20n_h10_c100_u10_ms | skim.ar20n_h05_c030_u10_ms | skim.ar20n_h05_c010_u10_ms | skim.ar20n_h05_c000_u10_ms |
| AR 20 SD7037, no ballast | 0.3 m | skim.ar20n_h10_c100_u10_ms | skim.ar20n_h03_c030_u10_ms | skim.ar20n_h03_c010_u10_ms | skim.ar20n_h03_c000_u10_ms |

- With the floor at 2 m the clearance does not matter: the bank limit of the margins keeps the low tip well above every clearance below ds.clearance_min_m.
- Below a floor of 1 m nothing changes, because the wingtip constraint stays active along the arc where the tip runs on the water: there the bank sweeps from about skid.ar20l_arc_bank_min_deg to the bank limit (skid.alb_bank_deg, skid.ar20l_bank_deg), and the centre comes lowest at the shallow end of that sweep, about the half span times the sine of that bank above the water (skim.alb_h03_c000_zmin_m and skim.ar20l_h03_c000_zmin_m at the lowest floor with the skid touching).
- The skid gains nearly as much as the lower floor: taking the clearance from the library's 0.3 m to zero at the lowest floor moves the minimum from skim.alb_h03_c030_u10_ms to skim.alb_h03_c000_u10_ms for the albatross-like glider, from skim.ar20l_h03_c030_u10_ms to skim.ar20l_h03_c000_u10_ms for the best wing and from skim.ar20n_h03_c030_u10_ms to skim.ar20n_h03_c000_u10_ms without ballast, against the floor's step from 2 m to 1 m at 0.3 m (skim.ar20l_h20_c030_u10_ms to skim.ar20l_h10_c030_u10_ms for the best wing).
- From the library's standard (floor 2 m, clearance 0.3 m) the lower floor and the skid together take the best wing from skim.ar20l_h20_c030_u10_ms to skim.ar20l_h03_c000_u10_ms; from R-12's clearance (skim.ar20l_h10_c100_u10_ms) the step is larger still.
- The optimizer then runs the low tip along the water for skid.ar20l_arc_s (best wing) and skid.alb_arc_s (albatross-like glider) of each cycle within 1 cm of its lowest point, a skim along an arc rather than a touch, with the load factor between skid.ar20l_arc_n_min and skid.ar20l_arc_n_max for the best wing; the light wing banks less and skims only for skid.ar20n_arc_s.

### Dihedral and a bent-up outer panel

The point mass carries the wing's shape only through the height of its lowest point and the lift the shape loses (`dsopt.WingShape`); with the bank mu, the flight-path angle gamma and the half span s = b/2 measured along the panels:

- dihedral Gamma puts the low tip s cos(gamma) sin(|mu| - Gamma) below the centre and leaves cos(Gamma) of the panels' lift vertical, so at a given lift the panels fly at CL / cos(Gamma), the induced drag rises by 1 / cos^2(Gamma) and the stall speed by 1 / sqrt(cos(Gamma));
- an outer panel bent up by Gamma_o from the fraction eta of the half span (a polyhedral wing, not a gull) puts the tip s cos(gamma) (eta sin|mu| + (1 - eta) sin(|mu| - Gamma_o)) and the break s cos(gamma) eta sin|mu| below the centre, both held clear, and leaves kappa = (1 - f_o) + f_o cos(Gamma_o) of the lift vertical, with f_o = 1 - (2/pi)(eta sqrt(1 - eta^2) + asin(eta)) the share of an elliptic loading outboard of the break; the drag follows as for dihedral with kappa in place of cos(Gamma), slightly more than the optimum loading in the Trefftz plane would give (K-018).

The levers were solved at the lowest floor, which solved for every aircraft, with eta at half the span so that 10 and 20 deg on the outer panel lift the tip at a small bank as much as 5 and 10 deg of dihedral.

| Aircraft | Wing | Tip clearance 0.3 m | 0 (the skid touches) |
|---|---|---|---|
| albatross-like glider | flat wing | skim.alb_h03_c030_u10_ms | skim.alb_h03_c000_u10_ms |
| albatross-like glider | dihedral 5 deg | skim.alb_dih5_h03_c030_u10_ms | skim.alb_dih5_h03_c000_u10_ms |
| albatross-like glider | dihedral 10 deg | skim.alb_dih10_h03_c030_u10_ms | skim.alb_dih10_h03_c000_u10_ms |
| albatross-like glider | outer panel bent up 10 deg from half span | skim.alb_out10_h03_c030_u10_ms | skim.alb_out10_h03_c000_u10_ms |
| albatross-like glider | outer panel bent up 20 deg from half span | skim.alb_out20_h03_c030_u10_ms | skim.alb_out20_h03_c000_u10_ms |
| AR 20 SD7037, ballast to the limit | flat wing | skim.ar20l_h03_c030_u10_ms | skim.ar20l_h03_c000_u10_ms |
| AR 20 SD7037, ballast to the limit | dihedral 5 deg | skim.ar20l_dih5_h03_c030_u10_ms | skim.ar20l_dih5_h03_c000_u10_ms |
| AR 20 SD7037, ballast to the limit | dihedral 10 deg | skim.ar20l_dih10_h03_c030_u10_ms | skim.ar20l_dih10_h03_c000_u10_ms |
| AR 20 SD7037, ballast to the limit | outer panel bent up 10 deg from half span | skim.ar20l_out10_h03_c030_u10_ms | skim.ar20l_out10_h03_c000_u10_ms |
| AR 20 SD7037, ballast to the limit | outer panel bent up 20 deg from half span | skim.ar20l_out20_h03_c030_u10_ms | skim.ar20l_out20_h03_c000_u10_ms |
| AR 20 SD7037, no ballast | flat wing | skim.ar20n_h03_c030_u10_ms | skim.ar20n_h03_c000_u10_ms |
| AR 20 SD7037, no ballast | dihedral 5 deg | skim.ar20n_dih5_h03_c030_u10_ms | skim.ar20n_dih5_h03_c000_u10_ms |
| AR 20 SD7037, no ballast | dihedral 10 deg | skim.ar20n_dih10_h03_c030_u10_ms | skim.ar20n_dih10_h03_c000_u10_ms |
| AR 20 SD7037, no ballast | outer panel bent up 10 deg from half span | skim.ar20n_out10_h03_c030_u10_ms | skim.ar20n_out10_h03_c000_u10_ms |
| AR 20 SD7037, no ballast | outer panel bent up 20 deg from half span | skim.ar20n_out20_h03_c030_u10_ms | skim.ar20n_out20_h03_c000_u10_ms |

- For the same lift of the tip at a small bank both levers move the minimum wind by about the same, as the table shows.
- They work because they lift the low tip at the steep bank, so the centre comes lower (skim.alb_dih10_h03_c000_zmin_m and skim.ar20l_dih10_h03_c000_zmin_m against the flat wing's above), and with dihedral the tip skims over a wider sweep of the bank, down to skid.ar20l_dih10_arc_bank_min_deg for the best wing.
- For the best wing the three parts are the floor (skim.ar20l_h20_c030_u10_ms to skim.ar20l_h10_c030_u10_ms), the skid (skim.ar20l_h03_c030_u10_ms to skim.ar20l_h03_c000_u10_ms) and 10 deg of dihedral (to skim.ar20l_dih10_h03_c000_u10_ms); the dihedral is the smallest of the three, and the lower floor and the skid give about as much each; together they take the albatross-like glider from skim.alb_h20_c030_u10_ms to skim.alb_dih10_h03_c000_u10_ms on a flat sea.
- The point mass leaves out the wing's bending: a wing that just meets D-014 lifts its low tip by skid.ar20l_arc_bend_min_m to skid.ar20l_arc_bend_max_m along the arc as the load factor and the bank change, more than the immersion the skid can take, so the tip's height must be measured, not predicted from the rigid geometry.
- It also leaves out the roll that dihedral adds when a touch yaws the aircraft towards the low tip, which is in the moment budget below.

### Upwind and across the wind with the low tip

`formstudy.skim_upwind_study()` (`sim/out/skim_upwind.json`) solves the support values h(90), h(135) and h(180) of the achievable mean ground velocities (the largest mean progress along a course measured from downwind, with the drift across it left free, see [the navigation section](#navigation)) at u10 13 and 16 m/s for the best wing and the albatross-like glider, in three configurations: the standard (floor 2 m, clearance 0.3 m), the skid (floor 1 m, clearance zero) and the skid with 10 deg of dihedral, with the margins and starts above; each course is also started from the previous course and from the same course of the previous configuration, a course that failed is tried again from the others with six more starts, and each course of the best wing is solved at its own Reynolds number (the first version took that of the fastest course for every course, K-018).
h(180) is the best speed made good straight upwind; when it is negative every soaring cycle drifts downwind at least that fast, so no line across the wind can be held on soaring alone, whatever h(90) and h(135) say.

| Aircraft | Configuration | h(90) at 13 m/s | h(135) | h(180) | h(90) at 16 m/s | h(135) | h(180) |
|---|---|---|---|---|---|---|---|
| AR 20 SD7037, ballast to the limit | standard | skim.ar20l_std_u13_h90_ms | skim.ar20l_std_u13_h135_ms | skim.ar20l_std_u13_h180_ms | skim.ar20l_std_u16_h90_ms | skim.ar20l_std_u16_h135_ms | skim.ar20l_std_u16_h180_ms |
| AR 20 SD7037, ballast to the limit | skid | skim.ar20l_skid_u13_h90_ms | skim.ar20l_skid_u13_h135_ms | skim.ar20l_skid_u13_h180_ms | skim.ar20l_skid_u16_h90_ms | skim.ar20l_skid_u16_h135_ms | skim.ar20l_skid_u16_h180_ms |
| AR 20 SD7037, ballast to the limit | skid and 10 deg dihedral | skim.ar20l_skiddih10_u13_h90_ms | skim.ar20l_skiddih10_u13_h135_ms | skim.ar20l_skiddih10_u13_h180_ms | skim.ar20l_skiddih10_u16_h90_ms | skim.ar20l_skiddih10_u16_h135_ms | skim.ar20l_skiddih10_u16_h180_ms |
| albatross-like glider | standard | skim.alb_std_u13_h90_ms | skim.alb_std_u13_h135_ms | skim.alb_std_u13_h180_ms | skim.alb_std_u16_h90_ms | skim.alb_std_u16_h135_ms | skim.alb_std_u16_h180_ms |
| albatross-like glider | skid | skim.alb_skid_u13_h90_ms | skim.alb_skid_u13_h135_ms | skim.alb_skid_u13_h180_ms | skim.alb_skid_u16_h90_ms | skim.alb_skid_u16_h135_ms | skim.alb_skid_u16_h180_ms |
| albatross-like glider | skid and 10 deg dihedral | skim.alb_skiddih10_u13_h90_ms | skim.alb_skiddih10_u13_h135_ms | skim.alb_skiddih10_u13_h180_ms | skim.alb_skiddih10_u16_h90_ms | skim.alb_skiddih10_u16_h135_ms | skim.alb_skiddih10_u16_h180_ms |

- Reaching the slower air near the water cuts the best wing's downwind drift straight upwind, at 16 m/s by skim.ar20l_skid_u16_drift_cut with the skid and by skim.ar20l_skiddih10_u16_drift_cut with the skid and dihedral (from skim.ar20l_std_u16_h180_ms to skim.ar20l_skid_u16_h180_ms and skim.ar20l_skiddih10_u16_h180_ms), and at 13 m/s by skim.ar20l_skid_u13_drift_cut and skim.ar20l_skiddih10_u13_drift_cut (from skim.ar20l_std_u13_h180_ms to skim.ar20l_skid_u13_h180_ms and skim.ar20l_skiddih10_u13_h180_ms).
- The best wing still drifts downwind in every case, so on soaring alone it can neither make progress upwind nor hold a line across the wind; upwind legs and holding a line stay with the motor or the swell.
- The albatross-like glider comes to the edge: at 16 m/s from skim.alb_std_u16_h180_ms to skim.alb_skid_u16_h180_ms with the skid and skim.alb_skiddih10_u16_h180_ms with the skid and dihedral, and at 13 m/s from skim.alb_std_u13_h180_ms to skim.alb_skid_u13_h180_ms and skim.alb_skiddih10_u13_h180_ms; with the skid and dihedral it just holds a line across the wind at 16 m/s and nearly does at 13 m/s.
  Its starts agree within skim.alb_skiddih10_u16_h180_start_spread_ms, and a clearance of 1.5 cm keeps it positive (skim.alb_skiddih10_u16_h180_tip15_ms, K-018), but the margin is smaller than what the model leaves out (the waves, stable air, turbulence, the wing's bending) and rests on its assumed polar (Q-014).
- The soaring sector widens: the progress along a course 45 deg off the wind, h(135), grows for the best wing from skim.ar20l_std_u13_h135_ms to skim.ar20l_skiddih10_u13_h135_ms at 13 m/s and from skim.ar20l_std_u16_h135_ms to skim.ar20l_skiddih10_u16_h135_ms at 16 m/s, and h(90) across the wind grows too; these are support values, so the drift across those courses is left free.
- So the low tip is worth more for making way against the wind than for the soaring minimum, and it is the only lever so far that brings soaring alone close to holding a line across the wind.

### What a touch does

`python -m sim.skid` (`sim/out/skid.json`) takes the skid-touch cycles at the lowest floor, replays them densely, and computes the water forces on a skid at the tip against its immersion, their moments about the centre of mass, and the time budget of the reflex; it is a hand calculation made repeatable, with sizes that no source gives (Q-070).

- Along the arc the tip moves over the water at skid.ar20l_arc_water_min_ms to skid.ar20l_arc_water_max_ms for the best wing (skid.alb_arc_water_min_ms to skid.alb_arc_water_max_ms for the albatross-like glider, skid.ar20n_arc_water_min_ms to skid.ar20n_arc_water_max_ms for the light wing), against the black skimmer's skimming speed of lit.skimmer_skim_speed_ms (P121); the forces below are at the largest, skid.ar20l_force_water_speed_ms.
- It also moves at a crab angle to the aircraft's heading, since the centre's ground velocity carries the wind at its height and the yaw rate moves the low tip: skid.ar20l_arc_crab_min_deg to skid.ar20l_arc_crab_max_deg for the best wing, skid.alb_arc_crab_min_deg to skid.alb_arc_crab_max_deg for the albatross-like glider and skid.ar20n_arc_crab_min_deg to skid.ar20n_arc_crab_max_deg for the light wing.
- The skimmer cuts the water with a lower mandible at most lit.skimmer_lower_beak_thickness_max_mm thick (P159) while flying about lit.skimmer_skim_height_m above it (P158); P121 estimated the drag of the bill at only lit.skimmer_bill_drag_wt_n, but a bill towed with lit.skimmer_bill_immersion_m immersed at up to lit.skimmer_tow_speed_max_ms extrapolates, by the lower bound of P157's fit, to lit.skimmer_bill_drag_10ms_n at 10 m/s, and its central fit gives lit.skimmer_bill_drag_10ms_central_n if its printed coefficient is a misprint; the hydrodynamic drag is about lit.skimmer_hydro_drag_share of the cost of skimming flight (P157).
- The blade model (friction on both sides by the ITTC 1957 line, spray drag of skid.spray_cd_t2 on the thickness squared, P157, P160, and the sea water of skid.rho_seawater_kg_m3 and skid.nu_seawater_m2_s, S10), with the bill's thickness lit.skimmer_lower_beak_thickness_max_mm and the rake skid.blade_rake_deg, gives skid.blade_skimmer_check_10ms_n for that bill at 10 m/s, between the lower bound and the central fit of the towed bill.
- The candidates: a thin blade below the tip, skid.blade_thickness_m thick, its leading edge raked back by skid.blade_rake_deg so that the wetted area grows with the square of the immersion and the water runs off aft, skid.blade_depth_m deep; a planing ski skid.ski_beam_m by skid.ski_length_m at skid.ski_trim_deg (Savitsky's planing lift, S9); a tip float of skid.float_beam_m by skid.float_length_m, planing like the ski; and the bare wingtip at the wing's angle of attack.
- The best wing's ailerons hold skid.ar20l_roll_moment_max_nm (the moment that holds ctrl.roll_pb2v against the strip-theory roll damping) and its rudder gives skid.ar20l_yaw_moment_max_nm (skid.rudder_cl_max on the fin of the wing pass's tail volume, assumed); its roll inertia is skid.ar20l_ixx_kgm2 and its yaw inertia skid.ar20l_izz_kgm2; the roll damping settles in skid.ar20l_roll_tau_s, so a roll moment below the ailerons' is held and a larger one rolls the wing at the excess.

| Skid | Immersion | Drag | Normal or side force | Yaw moment over the rudder's | Roll moment over the ailerons' | Steady sideslip | Yaw after 0.2 s |
|---|---|---|---|---|---|---|---|
| thin raked blade trailing in the water flow | 5 mm | skid.ar20l_blade_d05_drag_n | skid.ar20l_blade_d05_side_n | skid.ar20l_blade_d05_yaw_over_rudder | skid.ar20l_blade_d05_roll_over_aileron | skid.ar20l_blade_d05_sideslip_deg | skid.ar20l_blade_d05_yaw_0p2s_deg |
| thin raked blade trailing in the water flow | 10 mm | skid.ar20l_blade_d10_drag_n | skid.ar20l_blade_d10_side_n | skid.ar20l_blade_d10_yaw_over_rudder | skid.ar20l_blade_d10_roll_over_aileron | skid.ar20l_blade_d10_sideslip_deg | skid.ar20l_blade_d10_yaw_0p2s_deg |
| thin raked blade trailing in the water flow | 20 mm | skid.ar20l_blade_d20_drag_n | skid.ar20l_blade_d20_side_n | skid.ar20l_blade_d20_yaw_over_rudder | skid.ar20l_blade_d20_roll_over_aileron | skid.ar20l_blade_d20_sideslip_deg | skid.ar20l_blade_d20_yaw_0p2s_deg |
| thin raked blade trailing in the water flow | 30 mm | skid.ar20l_blade_d30_drag_n | skid.ar20l_blade_d30_side_n | skid.ar20l_blade_d30_yaw_over_rudder | skid.ar20l_blade_d30_roll_over_aileron | skid.ar20l_blade_d30_sideslip_deg | skid.ar20l_blade_d30_yaw_0p2s_deg |
| the same blade fixed at the heading | 10 mm | skid.ar20l_blade_d10_drag_n | skid.ar20l_fixed_d10_side_min_n to skid.ar20l_fixed_d10_side_max_n | skid.ar20l_blade_d10_yaw_over_rudder | skid.ar20l_fixed_d10_roll_over_aileron_min to skid.ar20l_fixed_d10_roll_over_aileron_max | skid.ar20l_blade_d10_sideslip_deg | skid.ar20l_blade_d10_yaw_0p2s_deg |
| the same blade fixed at the heading | 20 mm | skid.ar20l_blade_d20_drag_n | skid.ar20l_fixed_d20_side_min_n to skid.ar20l_fixed_d20_side_max_n | skid.ar20l_blade_d20_yaw_over_rudder | skid.ar20l_fixed_d20_roll_over_aileron_min to skid.ar20l_fixed_d20_roll_over_aileron_max | skid.ar20l_blade_d20_sideslip_deg | skid.ar20l_blade_d20_yaw_0p2s_deg |
| planing ski | 10 mm | skid.ar20l_ski_d10_drag_n | skid.ar20l_ski_d10_normal_n | skid.ar20l_ski_d10_yaw_over_rudder | skid.ar20l_ski_d10_roll_over_aileron | skid.ar20l_ski_d10_sideslip_deg | skid.ar20l_ski_d10_yaw_0p2s_deg |
| planing ski | 20 mm | skid.ar20l_ski_d20_drag_n | skid.ar20l_ski_d20_normal_n | skid.ar20l_ski_d20_yaw_over_rudder | skid.ar20l_ski_d20_roll_over_aileron | skid.ar20l_ski_d20_sideslip_deg | skid.ar20l_ski_d20_yaw_0p2s_deg |
| tip float | 10 mm | skid.ar20l_float_d10_drag_n | skid.ar20l_float_d10_normal_n | skid.ar20l_float_d10_yaw_over_rudder | skid.ar20l_float_d10_roll_over_aileron | skid.ar20l_float_d10_sideslip_deg | skid.ar20l_float_d10_yaw_0p2s_deg |
| tip float | 20 mm | skid.ar20l_float_d20_drag_n | skid.ar20l_float_d20_normal_n | skid.ar20l_float_d20_yaw_over_rudder | skid.ar20l_float_d20_roll_over_aileron | skid.ar20l_float_d20_sideslip_deg | skid.ar20l_float_d20_yaw_0p2s_deg |
| bare wingtip | 10 mm | skid.ar20l_tip_d10_drag_n | skid.ar20l_tip_d10_normal_n | skid.ar20l_tip_d10_yaw_over_rudder | skid.ar20l_tip_d10_roll_over_aileron | skid.ar20l_tip_d10_sideslip_deg | skid.ar20l_tip_d10_yaw_0p2s_deg |
| bare wingtip | 20 mm | skid.ar20l_tip_d20_drag_n | skid.ar20l_tip_d20_normal_n | skid.ar20l_tip_d20_yaw_over_rudder | skid.ar20l_tip_d20_roll_over_aileron | skid.ar20l_tip_d20_sideslip_deg | skid.ar20l_tip_d20_yaw_0p2s_deg |

- The side force of the trailing blade is for skid.misalignment_deg of misalignment with the water flow; a blade fixed to the wing meets the crab angle as incidence, above the ventilation onset of lit.ventilation_inception_deg (P161, P162), and its side force passes the ailerons' authority at 1 cm, and near 1.5 cm even if ventilation halved it; so the blade must trail in the water flow (a castering blade) and accept the spread of the crab angle along the arc.
- With dihedral the touch also rolls the aircraft towards the water: the sideslip that the touch's yaw holds (column above) meets a rolling moment due to sideslip of skid.ar20l_dih10_clbeta_per_rad per radian for 10 deg of dihedral (strip theory), giving skid.ar20l_dih10_roll_d10_nm at 1 cm and skid.ar20l_dih10_roll_d20_nm at 2 cm, skid.ar20l_dih10_roll_d10_over_aileron and skid.ar20l_dih10_roll_d20_over_aileron of the ailerons' authority, on top of the blade's own.
- The bare wingtip must never touch: at 1 cm its drag yaws the aircraft with skid.ar20l_tip_d10_yaw_over_rudder times the rudder's moment and would hold a sideslip far beyond any flight (skid.ar20l_tip_d10_sideslip_deg).
- A ski or a float planes, or runs submerged, at any immersion with a lift of skid.ar20l_ski_d10_normal_n or skid.ar20l_float_d10_normal_n against the weight of skid.ar20l_weight_n, which rolls the wing up near or past the ailerons' authority and yaws it near or past the rudder's; the ski at these speeds is far outside Savitsky's range; seaplane rules size tip floats for landing and floating loads up to lit.tipfloat_load_cap_displaced times their displaced water (P163), not for a touch at speed; both are rejected.
- The trailing blade stays within the rudder's and the ailerons' authority at 1 cm and passes the ailerons' at 2 cm at the arc's largest water speed (skid.ar20l_blade_d10_roll_over_aileron and skid.ar20l_blade_d20_roll_over_aileron), while the blade fixed at the heading passes them near 1 cm; the speed it takes is negligible (skid.ar20l_blade_d20_impulse_0p2s of the momentum in 0.2 s at 2 cm, against skid.ar20l_tip_d20_impulse_0p2s for the bare tip): the moments, not the momentum, limit a touch.
- So the skid is a thin, raked, hydrophobic blade that trails in the water flow, canted to stand upright at the skim bank and deeper than the worst immersion, with a mass of skid.blade_mass_kg and an air drag area of skid.blade_cda_m2 each; the pair adds skid.blade_pair_over_body_cda of the best wing's body drag area (wing.best_body_cda_m2), against skid.ski_pair_over_body_cda for skis and skid.float_pair_over_body_cda for floats.
- The sources stop at the depth Froude numbers of P157's towed bill (lit.skimmer_tow_froude_min to lit.skimmer_tow_froude_max) and of the ventilation studies (up to lit.ventilation_froude_max, P161, P162), well below the blade's (skid.ar20l_blade_froude_d30 at 3 cm to skid.ar20l_blade_froude_d05 at 5 mm), so every force here is an extrapolation until a towing test settles it (Q-070).

### What the tip sensor and the reflex must do

On the skim arc the tip runs along the water, so the danger is the sea rising under it above the plan: the water speed times the slope of the sea plus the wave's own rise, taken here as 0.5 to 2 m/s for slopes of 2 to 10 percent.
The reflex rolls towards wings level at the rate of ctrl.roll_pb2v through the lag ctrl.actuator_tau_s with the load factor held at the arc's lowest (skid.ar20l_arc_n_min), after a latency of skid.wet_latency_s (skid.sense_process_s for sensing and processing plus ctrl.actuator_delay_s); rolling lifts the low tip at skid.ar20l_roll_rise_ms at first, from the bank limit, and faster as the wing comes level (skid.ar20l_roll_rise_shallow_ms at the shallow end of the arc), and a pull is left out, since the lift-coefficient rate ctrl.cl_rate_max_per_s makes it slow.
The table gives, for the best wing at the bank limit and a sea rising 0.1 m above the plan, the immersion with a wet sensor alone and the gap at which a range sensor at the tip must trigger to keep the immersion within 1 or 2 cm; the albatross-like glider and the light wing need slightly less (skid.alb_w20_dl30_trigger_1cm_m and skid.ar20n_w20_dl30_trigger_1cm_m for the largest gap).

| Sea rising under the tip at | Immersion with the wet sensor alone | Range trigger gap for 1 cm | for 2 cm | for 1 cm when the sea rises 0.3 m |
|---|---|---|---|---|
| 0.5 m/s | skid.ar20l_w05_wet_depth_m | skid.ar20l_w05_dl10_trigger_1cm_m | skid.ar20l_w05_dl10_trigger_2cm_m | skid.ar20l_w05_dl30_trigger_1cm_m |
| 1 m/s | skid.ar20l_w10_wet_depth_m | skid.ar20l_w10_dl10_trigger_1cm_m | skid.ar20l_w10_dl10_trigger_2cm_m | skid.ar20l_w10_dl30_trigger_1cm_m |
| 2 m/s | skid.ar20l_w20_wet_depth_m | skid.ar20l_w20_dl10_trigger_1cm_m | skid.ar20l_w20_dl10_trigger_2cm_m | skid.ar20l_w20_dl30_trigger_1cm_m |

- A wet sensor alone keeps the blade within its depth only when the sea rises slower than the roll lifts the tip; at 1 m/s and above the wing itself would touch, so the reflex must start before contact, from a range reading at the tip.
- The minima at a clearance of zero assume the sea under the arc rises above the plan no faster than the roll lifts the tip; where it may rise faster, the planned clearance must be the trigger gap, and the minima of the 0.1 m and 0.3 m columns apply.
- The tip sensor must therefore read the gap from the blade's bottom to the water over skid.req_range_span_m (the largest trigger gap above), to skid.req_range_accuracy_m (half the 1 cm immersion it guards), at skid.req_range_rate_hz or faster (each sample interval adds the rise rate times that interval to the immersion) and within skid.req_range_latency_s of the reading, looking at the water below the skid along the arc's sweep of the bank (a lidar loses its returns over water off nadir, P85, so it must point at nadir at the skim bank, or a radar with a wide beam must be used, M16); an ultrasonic sensor at lit.ultrasonic_rate_hz with a minimum range of lit.ultrasonic_min_range_m (M18) is too slow.
- The wet sensor stays as the last resort and the measure of the immersion: electrodes at skid.req_wet_levels heights from the blade's bottom up to its depth, so that spray or a film is not taken for immersion, reading conductivity (sea water skid.seawater_conductivity_s_m, P164) or capacitance (converters sample at up to lit.cdc_rate_max_sps, M19) at skid.req_wet_rate_hz within skid.req_wet_latency_s.
- The owner adopted the range reading as the primary trigger, with the wet sensor as the last resort, and the trailing blade (Q-071, D-019).
- The actuator delay is most of the latency; every hundredth of a second saved shortens the trigger gap by the rise rate times that time.
- The planner's fitted sea is off by mpc.fit_surface_error_min_m to mpc.fit_surface_error_max_m ahead in a wind sea (see the receding horizon), far more than these gaps, so a skim is possible only where the sea under the arc is smoother than the trigger gap: calm water or long, predictable swell; over a wind sea the low tip keeps a clearance of the order of the prediction error.

How this feeds the reflex redesign (Q-068): the tip test becomes the measured gap at the tip against a trigger gap that grows with the closing rate, with the wet signal as the last resort; the response to a tip event is a roll towards wings level at the full roll rate with the load factor held, not full thrust and a fixed lift coefficient; and the planner keeps the trigger gap above the predicted sea along the skim arc.

## Flight patterns {#flight-patterns}

Albatrosses use several patterns and switch between them with the wind and the sea; `sim/patterns.py` holds the catalogue with its sources and simulates the ones that are new to the environment.

- Dynamic soaring across the wind in shallow S-arcs is the main pattern (P42, P44, P46, P47); the birds prefer headings near lit.albatross_preferred_heading_deg off the wind, like sailing yachts (P73); they tack upwind (P43), loop in place to search or hold station (P44), and turn through smaller angles as the wind rises (P74, P75). These map onto the modes of the cycle optimizer.
- Wave-slope soaring rides the updraft ahead of a moving crest and dominates in light wind over big swell (P76, P42, P77); the model finds it sustainable for most candidates only over swell of about patterns.wave_slope_swell_period_s period and only at the floor of the search, patterns.wave_slope_floor_m above the water, which is below the clearance of R-12 (Q-013).
- Gust soaring is a proposal without a test (P78) and is not simulated.
- Flap-gliding in light or head winds costs the bird most of its flight energy in a small share of its time (P79, P80, P81); for the drone the motor replaces the wing beat, at patterns.flapglide_baseline_wh_per_km_calm in calm air, patterns.flapglide_baseline_wh_per_km_cross10 across a 10 m/s wind, and patterns.flapglide_baseline_wh_per_km_head10 into it, for the baseline with a battery of patterns.battery_fraction of its mass.
- Resting on the water and taking off when the wind returns is what the birds do in a calm, with an easier take-off in more wind and bigger waves (P82); this is the Float and Takeoff states of the control modes.
- The selector follows the birds: dynamic soaring at or above ds.min_u10_ms, wave-slope soaring when the swell allows it, the motor while the battery covers a leg, and the water otherwise.
- The same run showed that the baseline's solar film yields only patterns.solar_avg_baseline_w on average, far below the avionics budget, which is finding Q-012 against R-08.

## Autonomy on estimated states {#autonomy-check}

The question was whether the drone can fly the soaring cycle on what its sensors tell it, not on the true state.
`python -m sim.autonomy` (commit 86d6689 plus the working tree of this branch) takes the optimizer's travelling cycle for the baseline at autonomy.test_u10_ms, tracks it with feedforward and feedback on the estimate, and compares the minimal, standard and full sensor suites of `sim/sensors.py` with the true state as the upper bound, over a flat log wind, a fully developed sea, and the sea with turbulence and the swell knee.
Over the flat log wind the standard suite flew autonomy.flat_standard_cycles cycles without a crash and kept within a few percent of the true state's ground speed (autonomy.flat_standard_ground_speed_ms against autonomy.flat_truth_ground_speed_ms), with a wind error of about autonomy.flat_standard_wind_rms_ms; the full suite did the same.
The minimal suite, with no magnetometer, never got a valid wind estimate before it crashed (Q-052).
Over waves every run crashed, the true state included, because the cycle's lowest point is planned above the mean surface and the crests reach it (Q-050); the load factor also exceeded the limit of R-09 under feedback (Q-051).
So the sensors are not the limit on a flat sea; the plan is the limit on a real one, which is why the planning in height above the local surface follows below.

## Planning from the aircraft's own view of the sea {#receding-horizon}

The owner asked that the drone judge the waves as it sees them and decide its flight from moment to moment (Q-057).
`sim/seasense.py` gives it a downward radar or lidar altimeter and a forward scanner, and fits a few wave components to what they saw along the track (`LocalWaveEstimator`).
`sim/mpc.py` re-plans the next seconds of flight every half second by sampling many control sequences (MPPI) in the predicted surface and wave wind, and flies in the full sea model; the comparison runs the same planner on the true sea, on a flat sea, and on the fitted sea, against the earlier tracker.
Command: `python -m sim.mpc`, which writes `sim/out/mpc.json`; the library of cycles is `sim/out/mpc_library.json`.
Each row is one run with one seed, so small differences mean nothing.

What it found:

- Planning from scratch does not soar: with an energy cost over a horizon of about one cycle, every run glided down and touched the water within mpc.first_pass_survival_max_s, whatever it knew about the sea.
- Seeded with a library of mpc.library_cycles periodic cycles from the optimizer (corrected wingtip, self-consistent roughness, heights constrained between nodes), the planner flies them on a flat sea for mpc.flat_survival_min_s to mpc.flat_survival_max_s, but still touches the water before the run ends, because the cycles sit on the load-factor limit and on the clearance edge.
- Over the sea it lasts mpc.wave_survival_min_s to mpc.wave_survival_max_s, and the fitted sea is not consistently better than assuming a flat one.
- Shifting the cycle so that its low point falls on the forward face of the dominant fitted wave shortened the flight in mpc.phasing_worse_pairs of mpc.phasing_pairs pairs; the cause is not found yet.
- The fit explains most of the surface the aircraft has flown over, but its prediction of the surface ahead is off by mpc.fit_surface_error_min_m to mpc.fit_surface_error_max_m against a sea whose standard deviation is about mpc.sea_std_u13_m, so it has no skill ahead in a wind sea.

What it means:

- The limit is now the margin of the cycles and the prediction of the sea ahead, not the accuracy of the sensors on what is below.
- The next pass gives the library cycles margin, adds cycles synchronized with the waves once Q-058 and Q-059 are settled, models a scanner that sees a wider swath ahead, and finds why the phasing hurts (Q-060).

With margins and real control limits (`python -m sim.mpc --build-margin-library` and `--margin-exp`, `sim/out/mpc_library_margin.json`, `sim/out/mpc.json` key margin_exp):

- The library was rebuilt with a lowest point of 2 m, a load factor of 0.8 of the limit, 0.9 of the bank limit and 1.2 times the stall speed (lighter margins where those did not converge), with COARE 3.5, the roll and lift-coefficient rates of Q-062 and the heights held between nodes; mpc.margin_library_entries of mpc.margin_library_slots cycles converged.
- The flight now goes through the actuators and the load-factor limit, and ends when a wingtip touches the water.
- On a flat sea with the true state and wind, every run flew the full mpc.margin_flat_truth_run_s, where the library without margins touched the water in every run: the margins are what a closed loop needs.
- On a flat sea with the estimated state and wind, every run touched the water within mpc.margin_flat_est_survival_min_s to mpc.margin_flat_est_survival_max_s, so the estimate is the next limit there.
- Over waves every run touched the water, a wingtip or the centre, within mpc.margin_sea_survival_min_s to mpc.margin_sea_survival_max_s, with a planner that assumes a flat sea on the true or the estimated state and with a planner on the fitted waves; the planner needs the waves (item 1 of the list in the energy sources).

The motor as a third control (`python -m sim.mpc --burst-exp`, `sim/out/mpc.json` key burst_exp):

- The owner asked whether short bursts of the motor at the right moment could keep the drone up cheaply when soaring is not enough; the planner got the thrust as a third control, with a cost on the electric energy, a thrust lag of prop.thrust_tau_s and the folded-propeller drag while it is off.
- On a flat sea below the soaring minimum, with the middle energy weight, the baseline flew the whole run at 11 m/s on mpc.burst_baseline_u11_normal_w and at 9 m/s on mpc.burst_baseline_u9_normal_w, and the albatross-like glider at 9 m/s on mpc.burst_albatross_u9_normal_w and at 6 m/s on mpc.burst_albatross_u6_normal_w, against motor.baseline_flapglide_w and motor.albatross_flapglide_w for a steady flap-glide.
- The thrust was not a short burst but a low, cycle-modulated trickle of about 2% of the weight on average: above 1% of the weight it ran mpc.burst_on_share_min to mpc.burst_on_share_max of the time, and above 0.1% almost all the time; it was lowest at the low point and rose on the climb after it for the baseline, with no clear timing for the albatross-like glider; with energy almost free the motor ran all the time at mpc.burst_free_min_w to mpc.burst_free_max_w (K-013).
- A heavier energy weight saved energy in two of five cases and still flew the whole run (the baseline at 11 m/s on mpc.burst_costly_baseline_u11_w, the albatross-like glider at 9 m/s on mpc.burst_costly_albatross_u9_w); in the others the planner stopped using the motor and the flight ended, although its plans saw the contact mpc.burst_contact_seen_min_s to mpc.burst_contact_seen_max_s ahead: the cost and the sampling (no thrust in the library's cycles, independent clipped noise per knot) kept the thrust off, while a steady tenth of the weight would have kept the aircraft up (K-013).
- Like for like (the same margins, no ground effect, the same speed along the track) the optimizer's best motor-assisted cycle needs mpc.burst_like_for_like_opt_min_w to mpc.burst_like_for_like_opt_max_w at 9 m/s against the planner's mpc.burst_baseline_u9_normal_w, so the planner spends a quarter to a third more; most of the gap to the optimizer's motor.baseline_u9_w comes from the margins and ground effect (K-013).
- Over waves the thrust did not prevent wingtip contact unless it ran almost continuously; the planner lacks cycles that follow the waves, not power.
- The true wind of these runs used the fixed Charnock constant while the library used COARE 3.5, which helps the planner slightly; and the smooth folded-propeller switch kept most of the folded drag during the spells with little thrust (K-013).
- What follows: cycles with the optimal thrust timing in the library as the planner's seed, thrust samples that can propose sustained thrust, a separate reflex that gives a short strong burst when contact or stall is predicted a second or two ahead, and wave-synchronized cycles for the waves (Q-060).

Motor-assisted cycles in the library and a reflex (`python -m sim.mpc --motor-lib-exp`, `sim/out/mpc_library_motor.json`, `sim/out/mpc.json` key motor_library_exp):

- The library now holds, for the five kept forms from calm to 24 m/s, the soaring cycles where they exist and below them the optimizer's minimum-power cycles with their thrust schedule; those need, for example, mpc.motorlib_baseline_calm_w in calm air, mpc.motorlib_baseline_u9_w at 9 m/s and mpc.motorlib_baseline_u11_w at 11 m/s for the baseline, and mpc.motorlib_albatross_calm_w and mpc.motorlib_albatross_u9_w for the albatross-like glider.
- The planner starts from that thrust schedule, its samples can hold a thrust level over several knots, the folded drag follows the commanded motor state, and the true wind uses COARE 3.5 as the library does.
- On a flat sea below the soaring minimum every run flew the whole mpc.motorlib_run_s, including the baseline at 6 and 9 m/s that ended in the burst experiment, at mpc.motorlib_power_ratio_min to mpc.motorlib_power_ratio_max times the optimizer's power; per kilometre it ranged from mpc.motorlib_per_km_ratio_min to mpc.motorlib_per_km_ratio_max times the optimizer's figure, better where the planner flew faster than the cycle's travel speed (K-015).
- One calm-air library cycle flies straight but 49 deg off its intended course, so it wastes about a third of its energy per kilometre; the library build needs a check on the course of steady entries (K-015).
- The thrust still runs as a low trickle almost all the time rather than in bursts.
- The reflex, set to act when the airspeed is predicted below 1.05 stall speeds or a wingtip below 0.3 m, fired falsely in two of nine flat-sea rows: at 9 m/s the baseline took mpc.reflex_baseline_u9_count reflexes, all on the stall test at the end of the 1.5 s prediction while it actually flew well above the stall, was in reflex for about a fifth of the time, climbed to over 100 m and spent mpc.reflex_energy_factor_min to mpc.reflex_energy_factor_max times the energy (K-015).
- A stall threshold of 0.95 stall speeds removed the false firing and still rescued four constructed dives towards the water, since the height and wingtip tests act first (K-015).
- Over waves the reflex made things worse: mpc.motorlib_wave_ended of mpc.motorlib_wave_runs wave runs ended early, mpc.motorlib_wave_ended_reflex of them with the reflex on; in one, full thrust past the vertical sped the aircraft up so it could not pull out (K-015).
- So the reflex needs redesign (Q-068): a stall threshold below the planner's floor, a stall test that accounts for the load factor, an override that does not zoom (thrust and lift scaled to the attitude, no full thrust past the vertical), and a world that sees the waves; and the waves still need wave-synchronized cycles in the library (Q-060).


## Candidate forms {#candidate-forms}

The yardsticks are: the smallest U10 that sustains soaring; progress speed over the ground at the design wind; survival on the water (self-righting, sealing, take-off); solar area; mass; what one person can build and repair; cost.
The candidates to fly in the environment are:

1. **Conventional glider with a tail boom**, the baseline of the structural model (span wing.span_m, wing area form.baseline_2m.wing_area_m2), like the demonstrators: the foam trainer of P48 (lit.easyglider_span_m) and a smaller demonstrator (form.demo_1p5m.span_m) that has no source in the record yet (Q-014). Known handling; the boom and tail are fragile on the water.
2. **Flying wing or tailless with winglets**, span form.flyingwing_2m.span_m. Fewer parts to break, easier to seal, but a lower aspect ratio (its wing area is form.flyingwing_2m.wing_area_m2) and less pitch control margin.
3. **Albatross-like high-aspect-ratio glider**, span form.albatross_3m.span_m, wing area form.albatross_3m.wing_area_m2, mass form.albatross_3m.mass_kg. Best lift-to-drag and lowest wind to soar; the biggest to launch and the most to lose.
4. **Sealed hull glider that floats hull-down**, span form.sealedhull_2p5m.span_m, with the fuselage as a watertight buoyancy body and the wing sealed as a foam-core shell. The form that takes R-05 and R-06 most seriously; take-off from the water is its open question.

The parameters of each candidate are the keys form.<candidate>.span_m, mass_kg, wing_area_m2, cd0, oswald and cl_max, taken from the aircraft registry in `sim/aircraft_models.py` (assumed, Q-014); the span and mass of the baseline are wing.span_m and wing.design_mass_kg.
The comparison is in the next section; availability and rough cost follow the demonstrators' parts (carbon tubes, foam, fiberglass, PX4-class autopilot).

## Form comparison {#form-comparison}

The candidates were flown in the environment by the collocation optimizer of periodic cycles (`sim/dsopt.py`, a point mass in a height-only wind over a flat sea).
The run now uses the corrections of K-007: the wingtip height is the geometric (b/2) sin|mu| cos gamma, and the roughness length is solved together with the minimum wind instead of being fixed at its 8 m/s value (Q-054).
The numbers are in `sim/out/dsopt_corrected.json`, whose meta block holds the call, the commit and the stamp of its inputs; the first run, at commit 86d6689 in `sim/out/dsopt.json`, is kept for comparison.
Candidate 1 is form `baseline_2m`, candidate 2 `flyingwing_2m`, candidate 3 `albatross_3m` and candidate 4 `sealedhull_2p5m`; `demo_1p5m` and `easyglider_1p8m` are the demonstrators as references.
For each candidate the record holds the smallest U10 that sustains a travelling cycle and a loiter cycle, with the log profile and with the swell knee (Q-011), as the keys form.<candidate>.u10_min_travel_log, u10_min_loiter_log, u10_min_travel_knee and u10_min_loiter_knee.
Where the travelling minimum is below the wind of the cross-wind case, the record also holds the progress speed across the wind at that wind, the objective of the case, as form.<candidate>.cross12_speed_log and cross12_speed_knee; it now exists for baseline_2m, albatross_3m and sealedhull_2p5m with both profiles.
The mean ground speed of those cycles is higher than the progress speed and is in the file.
Only converged cycles are recorded, and the cases that were skipped because the travelling minimum is above the cross-wind wind are left out.

What the keys show:

- The lowest minimum wind belongs to the albatross-like glider, form.albatross_3m.u10_min_travel_log; the conventional baseline and the sealed hull come next, and the flying wing and the two demonstrators need much more wind.
- The corrections lowered every minimum, by about a tenth for the baseline and less for the albatross-like glider, against the first run; the order of the candidates did not change.
- A loiter cycle needs more wind than a travelling cycle for every candidate, with either profile.
- The swell knee raises the minimum for four of the six candidates and lowers it slightly for the other two (the flying wing and the foam trainer).
- No candidate reaches ds.min_u10_ms, the design point of R-02; the best, form.albatross_3m.u10_min_travel_log, is the nearest.
- This is raised as finding Q-024, and the owner decides whether to lower the wing loading, change the form, or raise the design wind.

The roughness of the sea decides much of this.
The literature on dynamic soaring uses a fixed z0 of 0.03 m (P45, P86), which is about a hundred times the Charnock value at these winds and gives about twice the shear.
With that roughness the same run (`sim/out/dsopt_z0_003.json`) gives form.baseline_2m.u10_min_travel_log_z003 for the baseline and form.albatross_3m.u10_min_travel_log_z003 for the albatross-like glider; the keys form.<candidate>.*_z003 hold the rest.
Then the baseline, the albatross-like glider, the sealed hull and the small demonstrator come below ds.min_u10_ms, the flying wing is close to it, and only the foam trainer stays far above.
The literature's roughness turned out to be a land value (see the roughness of the open sea), so the sea values are the Charnock and COARE 3.5 ones, and with them no candidate reaches the design point of R-02 on a flat sea in neutral air.
With COARE 3.5 (`sim/out/dsopt_coare35.json`, keys form.<candidate>.*_coare35) the minimum winds move little: form.albatross_3m.u10_min_travel_log_coare35 for the albatross-like glider and form.baseline_2m.u10_min_travel_log_coare35 for the baseline; the small, draggy demonstrators gain most, since COARE 3.5 gives a rougher sea than the constant Charnock value at their higher winds.
That roughness is about dsopt.z003_over_charnock_ratio times the Charnock value of the sea at such a wind, so the low numbers are not sea values yet (K-009).
They also rest on the assumed lift-to-drag ratios of the candidates (Q-014): the minimum wind scales about as the inverse of the lift-to-drag ratio and hardly with the wing loading, since the albatross-like glider with its ratio lowered to that of the Sachs albatross needs dsopt.albatross3m_ld20_u10_ms, close to the heavier bird (K-009).

The same optimizer was run against P45: for an albatross model it gives dsopt.validation_u10_ms where P45 gives lit.min_u10_albatross_ms; without the wingtip clearance, which P45 does not impose, it gives dsopt.validation_no_tip_u10_ms (K-007).
The speed floor (a minimum airspeed set from the stall speed) was off for the validation run and on for the form study, so the two are not the same experiment.
The values table in docs/01-requirements.md lists every key named here.
The optimizer has had three independent checks, K-007 and K-009 on the flat cycles and K-008 on the wave mode; all found it sound as a solver, and K-009 reproduced this rerun to every digit.
They also found defects in its constraints and its search: the flat mode keeps the wingtip clear only at the nodes, and between them it dips to dsopt.flat_tip_mid_min_baseline_m for the baseline (Q-059).

## Cycles synchronized with the waves {#wave-cycles}

The flat-sea cycles of the form comparison ignore the waves; the owner asked that the flight be planned in the height above the moving surface and in the wind that goes with the waves (Q-057).
`sim/dsopt.py` has a wave mode for one dominant deep-water wave component:

- it measures the height above the local surface and uses the wind of `sim/seawind.py` for that component, the log profile over the local surface plus the wave-coherent wind;
- it keeps the height and the wingtip above the moving surface at the nodes and at the midpoints between them;
- the cycle repeats relative to the wave: after one period the wave has slipped by a whole number of wavelengths, and the optimizer also chooses where in the wave the cycle starts;
- options give the measured decay rates of the wave wind (P63) and the sheltering of `sim/seawind.py`;
- the module also has the corrections of K-007, the geometric wingtip height and the roughness length consistent with the wind found.

Command: `python -m sim.dsopt --wave-study2`, which writes `sim/out/dsopt_wave.json`.
Each case is one run of a local optimizer, so the results are examples, not a survey.

What it found:

- Over a dominant swell wave of the grid (Hs 1 to 3 m, Tp 6 to 11 s, along and across the wind), dsopt.wave_floor_cases of dsopt.wave_cases cases closed a cycle at the lowest wind allowed: the air moved by the waves alone carried the glider at least 1 m above the surface.
- Where the wind is needed, the low turn sits on the forward face of the wave or on its crest, where the wave pushes the air up, and never in the trough.
- The measured decay rates moved the minimum wind by at most dsopt.measured_decay_max_shift_ms.
- With sheltering, the albatross-like glider over a steep 20 m wave needed dsopt.shelter_albatross_l20_on_ms instead of dsopt.shelter_albatross_l20_off_ms; the cycle changed and its low point moved to the crest, not into the trough.

What the independent check found (K-008):

- The wave-only cycles are real solutions of the model: a replay in a separate model closes them and balances the energy.
- They are unstable, growing by up to dsopt.wave_cycle_floquet_max per cycle without feedback, so flying them needs the closed loop of Q-057.
- The count is not meaningful: the optimizer keeps the number of wavelengths slipped per cycle from its starting guess and stays in local optima, and a steady-ride estimate puts at least dsopt.wave_steady_ride_cases cases at the floor (Q-059).
- The physics behind them is weak: in calm air the model's vertical wave wind is dsopt.wave_vertical_ratio_model of the water's orbital speed, where the measurement over swell gives about dsopt.wave_vertical_ratio_measured (P64); with that ratio the two replayed cases need dsopt.wave_scaled_u10_baseline_tp8_hs2_ms and dsopt.wave_scaled_u10_albatross_tp6_hs1_ms of wind (Q-058).
- The steep-wave cases in calm air are artefacts, since their controls swing at rates no aircraft has and linear wave theory does not hold at that steepness; they are dropped.
- The sheltering does act where the wind outruns a steep wave; the reading that it hardly acts because of the onset slope was wrong, since in calm air it is the waves outrunning the air that shut it off.

What it means for the design:

- Swell can carry part of the energy, and with a calibrated wave wind the wind needed over swell may still fall below the flat-sea minimum; that is a route towards ds.min_u10_ms of R-02, and it waits for Q-058 and Q-059.
- The flight control must follow the wave phase with feedback, which is the receding-horizon planner of Q-057.

### Second pass: replay-checked cycles over swell (wave study 4)

The second pass (`python -m sim.dsopt --wave-study4`, `sim/out/dsopt_wave4.json`) fixed what K-008 and K-011 found and asked the question again with the calibrated wave wind:

- every cycle is replayed densely and counts only if it closes and keeps its height and wingtip between nodes; long cycles are re-solved on a finer mesh until they do;
- the wave wind uses the vertical amplitude and the decays of P63 (the horizontal amplitude stays 1.0 against P63's 0.95, which K-012 found changes nothing at the floor), and the low bound of P64 for the vertical part;
- roll and lift-coefficient rates, the load-factor floor and COARE 3.5 are in, and each case runs with the control margins (lowest point 2 m, 0.8 of the load limit) and without them;
- the phase is pinned on the wave and several phase slips and rotated starts are tried.

Of dsopt.wave4_cases_planned cases, dsopt.wave4_cases_run ran within the time; the rest (the albatross-like glider at the longer period and part of the low bound) are left for later.

What it found:

- Over a 3 m swell both candidates sustain a cycle with no wind at all, with and without the margins: dsopt.wave4_floor_cases cases in all, every case run with the wave wind of P63 and dsopt.wave4_p64_floor_cases of dsopt.wave4_p64_cases_run with the low bound of P64 (the one miss looks like a local optimum, since the same case along the wave reaches it); the baseline's cycles last dsopt.wave4_baseline_floor_period_min_s to dsopt.wave4_baseline_floor_period_max_s and turn low mostly on the forward face or the crest, in two cases on the rear face.
- They are wave-slope soaring: an independent replay (K-012) closes them, keeps their margins, and finds the energy coming from the updraft of the forward face, about 1.2 m/s at 2 m above a 3 m swell with P63 and 0.65 m/s with the low bound, against a sink of 0.40 m/s; one cycle rides a single wave at about the wave speed.
- Three of the albatross-like glider's seven cycles over 3 m swell sit at the period bound of dsopt.wave4_period_bound_s and are not natural periods, so for those their existence is shown but not their shape.
- Over a 1 m swell the waves do not help: only one case beats the flat sea, by dsopt.wave4_hs1_best_gain_ms, against flat minima of dsopt.wave4_flat_margin_baseline_ms and dsopt.wave4_flat_margin_albatross_ms with margins and dsopt.wave4_flat_none_baseline_ms and dsopt.wave4_flat_none_albatross_ms without; the others need more wind than the flat sea, because a cycle forced to repeat with the wave pays for that when the wave gives little, so for small waves the right answer is the better of a wave-synchronized and an unsynchronized cycle.

What it means: a large swell can carry the drone with no wind, if it follows the wave phase; a small swell cannot, and the wind must do it.
This is the bird's light-wind behaviour (P42, P74, P80), and it makes the wave-aware closed loop the next thing to show (Q-060).

## Wing sizing with measured airfoil data {#wing-sizing}

Until the first pass the candidate forms carried assumed drag polars and masses (Q-014); the owner asked whether the wing itself had been optimized, and it had not.
`sim/wingopt.py` builds a design from its span, aspect ratio, taper, airfoil and ballast and scores it (`python -m sim.wingopt`, `sim/out/wingopt2.json`; the first pass is `sim/out/wingopt.json`).
The second pass (Q-067) is the current result.
It was run in steps, each a command of `sim/wingopt.py` that adds to `sim/out/wingopt2.json`:
`python -m sim.wingopt --ballast 0,0.5` (span 3.0 m, aspect ratios 12, 16, 20 and 25, both airfoils, ballast none, 0.5 kg and the mass limit); `--followup 4` (the top four by soaring minimum, with h(180) and the motor power at 6, 9 and 13 m/s and courses 20 and 180 deg from the library's full solver settings); and `--c9-steady` (the 9 m/s motor power of every design at both courses, with the steady-flight start and then the seeds).
The steps `--c9-all` and `--c9-standard` ran in between and are superseded by `--c9-steady`; their results stay in the file.
The file's main stamp is that of the first command; each later step carries its own stamp under `steps`, and a step that ran under an earlier version of the file is stamped with the file as it is now.

The model:

- Profile drag from measured low-speed airfoil polars, fitted with a Reynolds-number scaling over the range of the data (P155); a design whose mean chord falls below a Reynolds number of wing.re_floor at 1.2 times its stall speed is left out, since the data end there and the drag rises steeply below.
- Induced drag with the span efficiency of Nita and Scholz for the taper and the fuselage (P156), with their offset of the taper ratio for an unswept wing (their equation 37 at zero sweep, which the first pass had left out).
- Body drag built up, not fixed (the first pass used wing.cda_par_base_m2): a pod and a boom at the turbulent flat-plate friction of their own Reynolds numbers with the Hoerner form factor and the wetted area of a body of revolution; a horizontal and a vertical tail sized by volume coefficients on the mean chord and the span, with the profile drag of the SD7037 fit at the tail's Reynolds number; and a margin for gaps and linkages (P145, S7, S8).
  The sum is wing.body_cda_min_m2 to wing.body_cda_max_m2 over the designs and wing.best_body_cda_m2 for the best, so the fixed area of the first pass was low, as the checker suspected (K-014).
- Wing mass from an areal density of wing.rho_a_base_kg_m2, between thermal sailplanes and slope racers (M17), plus the spar caps for the ultimate load and a joiner; the rest from the component masses of the mass model, plus ballast that the spar also carries.
  The ballast grid is none, 0.5 kg and the mass limit ds.mass_max_kg.
- Objective A, the soaring minimum, with the control library's margins, COARE 3.5, the roll and lift-coefficient rates and the heights held between nodes, from six starts of the optimizer.
- Objective B, the support value h(180) of the achievable mean ground velocities at u10 16 m/s, which is the speed made good straight upwind; the first pass's upwind figure is withdrawn because it was not one (below).
- Objective C, the average electric power of a motor-assisted travel cycle (the optimizer's minimum-power mode with the motor and the control library's margins) at 9 m/s and course 20 deg from downwind, and objective D, its energy per kilometre; the follow-up adds 6 and 13 m/s and the course straight upwind, for the top four designs, with the library's full solver settings.
  The 9 m/s cases of all designs at both courses start from a steady level flight (at the polar of its own airspeed) and then from the seeds: at 20 deg the steady start solved every design, straight upwind it failed for every design and the seeds solved them.

What it found:

- The best soaring minimum is wing.best_u10_min_ms, of a design wing.best_span_m in span with an aspect ratio of wing.best_ar, the SD7037 airfoil and ballast of wing.best_ballast_kg up to the mass limit (wing.best_mass_kg, wing loading wing.best_wing_loading_npm2); its maximum lift-to-drag ratio is wing.best_ld_max at the cycle's mean Reynolds number and wing.best_ld_glide at its own best-glide speed.
- The best of several starts repeats to within wing.start_repeat_ms (K-016), but the tail sizing moves the minima more than the three best designs differ: a tail a quarter smaller lowers the soaring minima by wing.tail_quarter_gain_min_ms to wing.tail_quarter_gain_max_ms and half the tail lowers the best design's by wing.tail_half_gain_best_ms, against wing.top3_spread_ms between the three best, so their order is not reliable; what is reliable is that slender wings with ballast up to the limit beat the broad wing, and that the mass limit binds.
- Ballast helps by wing.ballast_gain_min_ms to wing.ballast_gain_max_ms for the designs that could be solved without it; without ballast the minimum is wing.unballasted_u10_min_ms to wing.unballasted_u10_max_ms.
  The first pass's broad wing ranked first only because ballast was not a variable.
- The price is motor power. At the standard margins the Pareto front of the soaring minimum against the motor power at 9 m/s and course 20 deg is the table below (keys; each row is a design and its ballast), with the motor power straight upwind in the last column; every design could be solved at both courses.
  The motor power is the optimizer's result at the Reynolds number of the motor cycle's own airspeed; at 6 and 9 m/s it is level flight at the speed floor of the margins (the speed of least power lies below it), found from a steady start and from random starts, so the model finds no soaring gain at these winds.
  An earlier comparison took the steady flight's polar at the soaring cycle's Reynolds number, found the optimizer above it, and reported the lower of the two; that is withdrawn (K-016).
  The thrust of the level flight is within the motor's thrust limit (prop.thrust_max_per_weight); the record has no power limit.

| Front row | Design | Ballast | Mass | Wing loading | L/D at glide | Soaring minimum | Motor power at 9 m/s, 20 deg | Energy per km | Motor power at 9 m/s, upwind |
|---|---|---|---|---|---|---|---|---|---|
| 1 | AR 20, SD7037, at the mass limit | wing.front1_ballast_kg | wing.front1_mass_kg | wing.front1_wing_loading_npm2 | wing.front1_ld_glide | wing.front1_u10_ms | wing.front1_c9_w | wing.front1_wh_per_km | wing.front1_c9_chi180_w |
| 2 | AR 16, SD7037, at the mass limit | wing.front2_ballast_kg | wing.front2_mass_kg | wing.front2_wing_loading_npm2 | wing.front2_ld_glide | wing.front2_u10_ms | wing.front2_c9_w | wing.front2_wh_per_km | wing.front2_c9_chi180_w |
| 3 | AR 12, SD7037, at the mass limit | wing.front3_ballast_kg | wing.front3_mass_kg | wing.front3_wing_loading_npm2 | wing.front3_ld_glide | wing.front3_u10_ms | wing.front3_c9_w | wing.front3_wh_per_km | wing.front3_c9_chi180_w |
| 4 | AR 16, SD7037, 0.5 kg of ballast | wing.front4_ballast_kg | wing.front4_mass_kg | wing.front4_wing_loading_npm2 | wing.front4_ld_glide | wing.front4_u10_ms | wing.front4_c9_w | wing.front4_wh_per_km | wing.front4_c9_chi180_w |
| 5 | AR 20, SD7037, 0.5 kg of ballast | wing.front5_ballast_kg | wing.front5_mass_kg | wing.front5_wing_loading_npm2 | wing.front5_ld_glide | wing.front5_u10_ms | wing.front5_c9_w | wing.front5_wh_per_km | wing.front5_c9_chi180_w |
| 6 | AR 16, SD7037, no ballast | wing.front6_ballast_kg | wing.front6_mass_kg | wing.front6_wing_loading_npm2 | wing.front6_ld_glide | wing.front6_u10_ms | wing.front6_c9_w | wing.front6_wh_per_km | wing.front6_c9_chi180_w |
| 7 | AR 20, SD7037, no ballast | wing.front7_ballast_kg | wing.front7_mass_kg | wing.front7_wing_loading_npm2 | wing.front7_ld_glide | wing.front7_u10_ms | wing.front7_c9_w | wing.front7_wh_per_km | wing.front7_c9_chi180_w |

- Over all designs the motor power at 9 m/s is wing.c9_chi20_min_w to wing.c9_chi20_max_w at 20 deg and wing.c9_chi180_min_w to wing.c9_chi180_max_w straight upwind; the unballasted designs need the least and the heaviest the most, and the heavy front rows give the soaring minimum up at a power cost that grows with the ballast.
  An earlier table of seven designs solved only at the light margins is withdrawn: every design is solved at the standard margins.

- Straight upwind, soaring alone makes no progress: the support value h(180) of the top four is wing.h180_ar20_sd7037_ms, wing.h180_ar25_sd7037_ms, wing.h180_ar16_sd7037_ms and wing.h180_ar20_ag35r_ms, all negative, so the drone drifts downwind at 16 m/s however it flies the cycle.
  The first pass reported the progress of the optimizer's maximum-speed cycle at courses off the wind times the cosine of the angle off the wind as a speed made good upwind; that is not one.
  The optimizer maximises only the component of the mean ground velocity along the course and leaves the cross-track velocity free, so its value at a course is a support value of the achievable mean ground velocities, not a speed along a track (see [the navigation section](#navigation)).
  The first pass's upwind figure is therefore withdrawn, and upwind legs are flown on the motor.
  `solve_b` took the Reynolds number of the fastest course for every course it solved together (K-018); these h(180) values were solved alone, where that rule makes no difference, and `solve_b` now iterates the Reynolds number per course.
- The motor power of the top four, with the library's full settings and the standard margins, is wing.follow_p9_chi20_min_w to wing.follow_p9_chi20_max_w at 9 m/s and course 20 deg (wing.follow_p6_chi20_min_w to wing.follow_p6_chi20_max_w at 6 m/s and wing.follow_p13_chi20_min_w to wing.follow_p13_chi20_max_w at 13 m/s); straight upwind it is wing.follow_p6_chi180_min_w to wing.follow_p6_chi180_max_w, wing.follow_p9_chi180_min_w to wing.follow_p9_chi180_max_w and wing.follow_p13_chi180_min_w to wing.follow_p13_chi180_max_w at 6, 9 and 13 m/s.
- On the geometry of a 3 m thermal sailplane the polar is a few percent pessimistic against its designer's calculation, and measured 3 m sailplanes glide in the range of these ratios, so they are plausible at the top end (K-014).

Fixes to the first pass:

- The first pass's `solve_a` reported the aircraft of a Reynolds number that had not been solved when its second Reynolds pass ended unconverged, and discarded a good first pass when the second failed; it now keeps the last converged pass and its own aircraft and flags a Reynolds number that did not converge.
- Each soaring minimum is the best of six starts with a passing dense replay; the second Reynolds pass is warm-started from it.
- The taper offset and the body drag build-up are above.

Caveats:

- The grid is cut for run time: span 3.0 m only (no 2.5 m rows), aspect ratios 12 to 25, two airfoils, and ballast of none, 0.5 kg and the limit; intermediate ballast and the 2.5 m rows are not run, and aspect ratio 25 without ballast falls below the Reynolds floor.
- The motor powers are local optima: the optimizer returns level flight at the speed floor, and it was not shown to find a cycle that mixes soaring and thrust, which could use less; the main run's cut solver settings failed on most designs, so the front uses the full settings; the top four's 6 and 13 m/s and straight-upwind figures in the follow-up are the optimizer's results from random starts.
- The tail's Reynolds number is below the data of the airfoil fit, so the tail's drag is an extrapolation, and the tail's arm and aspect ratios are assumed; the tails are a large share of the body drag, and the pod's and the boom's wetted areas use Torenbeek's fuselage formula, which the record does not hold (S7 gives the friction and the form factor).
- Everything rests on single airfoil fits (P155), a structure heavier or lighter than the areal density of wing.rho_a_base_kg_m2 changes the minimum (K-014), and the tail sizing moves the minima by more than the three best designs differ.

### Heavier than the limit

The owner asked whether the drone could simply be heavier than ds.mass_max_kg (Q-067, Q-069).
The limit is not changed (it stays assumed under Q-009); `python -m sim.wingopt --heavy` (`sim/out/wingopt_heavy.json`) waives it for the 3.0 m SD7037 wings of aspect ratio 16 and 20, with ballast at the fuselage up to 10 kg, the spar caps of the mass model sized for ds.n_limit times ds.safety_factor at each mass, and the standard margins.
The motor power is the optimizer's at 9 m/s, started from the steady flight, with the polar at the motor flight's own Reynolds number; the steady level flight at its own Reynolds number is shown beside it, and the two use the same power model.

| Design | Mass | Wing loading | Over the albatross's | Soaring minimum | Motor power at 9 m/s, 20 deg | Steady flight, 20 deg | Energy per km, 20 deg | Motor power at 9 m/s, upwind | Steady flight, upwind | Energy per km, upwind |
|---|---|---|---|---|---|---|---|---|---|---|
| AR 16, SD7037 | 3 kg | heavy.ar16_m03_wing_loading_npm2 | heavy.ar16_m03_over_albatross | heavy.ar16_m03_u10_ms | heavy.ar16_m03_p20_w | heavy.ar16_m03_p20_steady_w | heavy.ar16_m03_e20_wh_per_km | heavy.ar16_m03_p180_w | heavy.ar16_m03_p180_steady_w | heavy.ar16_m03_e180_wh_per_km |
| AR 16, SD7037 | 4 kg | heavy.ar16_m04_wing_loading_npm2 | heavy.ar16_m04_over_albatross | heavy.ar16_m04_u10_ms | heavy.ar16_m04_p20_w | heavy.ar16_m04_p20_steady_w | heavy.ar16_m04_e20_wh_per_km | heavy.ar16_m04_p180_w | heavy.ar16_m04_p180_steady_w | heavy.ar16_m04_e180_wh_per_km |
| AR 16, SD7037 | 6 kg | heavy.ar16_m06_wing_loading_npm2 | heavy.ar16_m06_over_albatross | heavy.ar16_m06_u10_ms | heavy.ar16_m06_p20_w | heavy.ar16_m06_p20_steady_w | heavy.ar16_m06_e20_wh_per_km | heavy.ar16_m06_p180_w | heavy.ar16_m06_p180_steady_w | heavy.ar16_m06_e180_wh_per_km |
| AR 16, SD7037 | 8 kg | heavy.ar16_m08_wing_loading_npm2 | heavy.ar16_m08_over_albatross | heavy.ar16_m08_u10_ms | heavy.ar16_m08_p20_w | heavy.ar16_m08_p20_steady_w | heavy.ar16_m08_e20_wh_per_km | heavy.ar16_m08_p180_w | heavy.ar16_m08_p180_steady_w | heavy.ar16_m08_e180_wh_per_km |
| AR 16, SD7037 | 10 kg | heavy.ar16_m10_wing_loading_npm2 | heavy.ar16_m10_over_albatross | heavy.ar16_m10_u10_ms | heavy.ar16_m10_p20_w | heavy.ar16_m10_p20_steady_w | heavy.ar16_m10_e20_wh_per_km | heavy.ar16_m10_p180_w | heavy.ar16_m10_p180_steady_w | heavy.ar16_m10_e180_wh_per_km |
| AR 20, SD7037 | 3 kg | heavy.ar20_m03_wing_loading_npm2 | heavy.ar20_m03_over_albatross | heavy.ar20_m03_u10_ms | heavy.ar20_m03_p20_w | heavy.ar20_m03_p20_steady_w | heavy.ar20_m03_e20_wh_per_km | heavy.ar20_m03_p180_w | heavy.ar20_m03_p180_steady_w | heavy.ar20_m03_e180_wh_per_km |
| AR 20, SD7037 | 4 kg | heavy.ar20_m04_wing_loading_npm2 | heavy.ar20_m04_over_albatross | heavy.ar20_m04_u10_ms | heavy.ar20_m04_p20_w | heavy.ar20_m04_p20_steady_w | heavy.ar20_m04_e20_wh_per_km | heavy.ar20_m04_p180_w | heavy.ar20_m04_p180_steady_w | heavy.ar20_m04_e180_wh_per_km |
| AR 20, SD7037 | 6 kg | heavy.ar20_m06_wing_loading_npm2 | heavy.ar20_m06_over_albatross | heavy.ar20_m06_u10_ms | heavy.ar20_m06_p20_w | heavy.ar20_m06_p20_steady_w | heavy.ar20_m06_e20_wh_per_km | heavy.ar20_m06_p180_w | heavy.ar20_m06_p180_steady_w | heavy.ar20_m06_e180_wh_per_km |
| AR 20, SD7037 | 8 kg | heavy.ar20_m08_wing_loading_npm2 | heavy.ar20_m08_over_albatross | heavy.ar20_m08_u10_ms | heavy.ar20_m08_p20_w | heavy.ar20_m08_p20_steady_w | heavy.ar20_m08_e20_wh_per_km | heavy.ar20_m08_p180_w | heavy.ar20_m08_p180_steady_w | heavy.ar20_m08_e180_wh_per_km |
| AR 20, SD7037 | 10 kg | heavy.ar20_m10_wing_loading_npm2 | heavy.ar20_m10_over_albatross | heavy.ar20_m10_u10_ms | heavy.ar20_m10_p20_w | heavy.ar20_m10_p20_steady_w | heavy.ar20_m10_e20_wh_per_km | heavy.ar20_m10_p180_w | heavy.ar20_m10_p180_steady_w | heavy.ar20_m10_e180_wh_per_km |

- The soaring minimum falls from heavy.ar20_m03_u10_ms at 3 kg to heavy.ar20_m06_u10_ms at 6 kg and then hardly moves (heavy.ar20_m10_u10_ms at 10 kg); with aspect ratio 16 from heavy.ar16_m03_u10_ms to heavy.ar16_m06_u10_ms and heavy.ar16_m10_u10_ms; beyond about 6 kg the gain is within the optimizer's repeatability (wing.start_repeat_ms).
- The wandering albatross's wing loading (heavy.albatross_wing_loading_npm2, from lit.albatross_mass_kg and lit.albatross_wing_area_m2) is reached near 8 kg with aspect ratio 16 and between 6 and 8 kg with aspect ratio 20.
- The motor power grows faster than the mass, since the cycle at 20 deg is level flight at the speed floor, whose power goes with the weight times the stall speed: for aspect ratio 20 from heavy.ar20_m03_p20_w at 3 kg to heavy.ar20_m10_p20_w at 10 kg, and straight upwind from heavy.ar20_m03_p180_w to heavy.ar20_m10_p180_w.
- At 20 deg the optimizer's cycle is that steady flight at 1.2 times the stall speed and needs the same power to four digits when both use the motor flight's Reynolds number, as K-016 found for the wing pass's designs; straight upwind the optimizer is at most a few percent below the steady flight.
- The mass model sizes its spar caps for stress only and takes the modulus of the pultruded tube (spar.carbon_modulus, P54), so fully stressed caps bend to heavy.ar16_stress_caps_deflection (aspect ratio 16) and heavy.ar20_stress_caps_deflection (aspect ratio 20) of the half span at limit load whatever the mass, and spar.deflection_limit (D-014) needs heavy.ar16_stiff_caps_factor and heavy.ar20_stiff_caps_factor times the cap mass; caps of unidirectional carbon at spar.ud_carbon_modulus (assumed, Q-010) would need only heavy.ar16_stiff_caps_factor_ud and heavy.ar20_stiff_caps_factor_ud times it (`python -m sim.wingopt --heavy-structure`, K-018).
  At a fixed total mass the stiffer caps come out of the ballast and leave the flight results unchanged, but the same holds at the mass limit, where the wing pass's best design carries too light a spar (Q-010, Q-053).
- Among the structural model's tube sizes (diameters in whole millimetres, walls up to 2 mm) none fits inside the root of the aspect ratio 20 wing at any mass of the grid, and for aspect ratio 16 one fits at 3 and 4 kg but not from 6 kg; with a free diameter and thicker walls a tube fits the aspect ratio 20 wing at 3 and 4 kg and the aspect ratio 16 wing at every mass, at heavy.ar16_tube_free_over_caps_min to heavy.ar16_tube_free_over_caps_max (aspect ratio 16) and heavy.ar20_tube_free_over_caps_min to heavy.ar20_tube_free_over_caps_max (aspect ratio 20) times the cap model's spar mass.
- No design breaks the Reynolds floor (wing.re_floor), since heavier wings fly faster; but the aspect ratio 16 wing at 8 and 10 kg soars at heavy.ar16_m08_re_cycle and heavy.ar16_m10_re_cycle, above the upper end of the airfoil data (P155), so those rows extrapolate the polar.
- Left open: launching and recovering an aircraft of 4 to 10 kg at sea (Q-048); the structure (the stiff spar above, landing on the water, and a skid touch with more momentum); the energy of a calm day, which grows with the mass (Q-055) and is what variable ballast would address (Q-069); and the mass thresholds of Japan's rules for unmanned aircraft, not researched here (Q-001, Q-007).

First pass, kept as history: with the mass fixed by the wing's size, a broad wing of aspect ratio 12 with the AG35 airfoil came first and the lighter slender wings ranked lower; rebuilt with the same polar and mass model the old candidates needed wing.rebuilt_albatross_u10_min_ms (the albatross-like glider), wing.rebuilt_baseline_u10_min_ms (the baseline) and wing.rebuilt_sealedhull_u10_min_ms (the sealed hull), higher than the form comparison's because its polars were too clean and it flew without the control margins; at equal mass an aspect ratio of 20 soared from wing.equal_mass_ar20_u10_ms and 25 from wing.equal_mass_ar25_u10_ms (K-014); a heavier skin lowered the minimum by wing.heavy_skin_gain_min_ms to wing.heavy_skin_gain_max_ms, half through the Reynolds number and half through the rate limits that loosen at higher speed (K-014); a fat sensor pod (twice the drag area) cost wing.fat_pod_loss_min_ms to wing.fat_pod_loss_max_ms (wing.best_fat_pod_u10_ms for the best design then); the taper hardly mattered.

- So on a flat sea in neutral air, with the margins a controller needs, the wind alone carries the drone only from wing.best_u10_min_ms at best; the routes to R-02's design point are the swell (see the cycles synchronized with the waves), the extra shear under swell and in stable air (Q-061), and the motor (Q-055).

## Navigation: the speed polar, the velocity hull and the tactical layer {#navigation}

The planner flies a cycle; which cycle to fly towards a target is the navigation's job, and like a yacht the drone cannot sail straight into the wind (Q-066).
Albatrosses keep headings about lit.albatross_preferred_heading_deg off the wind and route like yacht racers (P73), and can travel upwind (P43).

The speed polar (`python -m sim.polar`, `sim/out/ds_polar.json`): the progress speed along each course relative to the wind, from the cycle optimizer with the control library's margins, COARE 3.5 and the roll and lift-coefficient rates, for the albatross-like glider, the baseline and the sealed hull at 9 to 20 m/s.
Only the albatross-like glider soars at 11 m/s on that grid, the baseline and the sealed hull from 13 m/s, and at 13 m/s only with the light margins.

The progress in that table is a projection, not a speed along a track.
The optimizer maximises the speed along the course and leaves the velocity across it free, so a cycle's mean ground velocity lies off its course: for the albatross-like glider at 16 m/s the cycle of the 120 deg course makes nav.polar_albatross_u16_c120_progress_ms m/s along the course but flies at nav.polar_albatross_u16_c120_ground_ms m/s, nav.polar_albatross_u16_c120_drift_deg deg to the downwind side of it.
The progress is the support function of the cycles' mean ground velocities, and the speed made good towards a target is not the progress times the cosine of the angle to the target.
An earlier version of this section read the table that way (a best upwind course 60 deg off the wind and an upwind progress of several metres per second); those figures were wrong, the navigator built on them (a speed polar and a cosine) has been removed from `sim/navigator.py` and `sim/mpc.py`, and their record keys are replaced by the ones below.

- The best soaring velocity component towards straight upwind (the progress at the 180 deg course) is negative for each form on every row of the polar's grid where it soars: nav.albatross_upwind_support_max_ms to nav.albatross_upwind_support_min_ms for the albatross-like glider at 13 to 20 m/s, nav.baseline_upwind_support_max_ms to nav.baseline_upwind_support_min_ms for the baseline and nav.sealedhull_upwind_support_max_ms to nav.sealedhull_upwind_support_min_ms for the sealed hull at 13 to 20 m/s (the 13 m/s row of the latter two with the light margins).
  The albatross-like glider's 11 m/s row is not a support function: its single local optimizer runs are rough and not monotonic in the course, so its value is left out of the range, though it is negative as well.
  Soaring alone cannot go upwind with these margins, nor hold its position.
- The progress turns negative between the 135 and the 180 deg course, that is within nav.no_progress_within_deg_min to nav.no_progress_within_deg_max of the wind's eye (interpolated between the 15 deg grid points): the nearest to the eye for the albatross-like glider at 16 and 20 m/s, whose progress is still positive at 165 deg, the farthest for the baseline and the sealed hull.
- Going upwind or across the wind needs the motor, and a powered leg does not need a cycle: steady level flight at 10 m, aimed by the wind triangle at the airspeed of least electric energy per metre, makes good nav.albatross_steady_upwind_u16_ms straight upwind at 16 m/s for nav.albatross_steady_upwind_u16_j_per_m of electricity per metre for the albatross-like glider, and nav.albatross_steady_across_u16_ms across the wind for nav.albatross_steady_across_u16_j_per_m.
  The library's own steady upwind motor cycle (nav.albatross_motor_upwind_u16_ms for nav.albatross_motor_upwind_u16_j_per_m) stays as a point of the hull.
- With the margins the albatross-like glider soars from nav.albatross_polar_min_u10_ms and the baseline and the sealed hull from nav.baseline_polar_min_u10_ms; below that the planner's motor-assisted cycles take over.
- The polar comes from single runs of a local optimizer on a 15 deg grid, so the values are good to about the grid.

The velocity hull (`sim/navigator.py`, `VelocityHull`): the points are the mean ground velocities of the library's cycles (net displacement over a period divided by the period) and their mirror images, soaring and motor-assisted kept apart, each motor point with its average electric power; flying cycles in turn gives every velocity inside their convex hull.
At 16 m/s every soaring point has a downwind component, at least nav.albatross_hull_min_downwind_u16_ms for the albatross-like glider, nav.baseline_hull_min_downwind_u16_ms for the baseline and nav.sealedhull_hull_min_downwind_u16_ms for the sealed hull (the library's courses stop at 150 deg, so these are coarser than the support values above); the directions that soaring alone can make good along lie within nav.albatross_soaring_sector_u16_deg, nav.baseline_soaring_sector_u16_deg and nav.sealedhull_soaring_sector_u16_deg of the downwind axis, so a target across the wind or upwind is out of reach without the motor.
Holes of the library: the soaring 135 deg cycle of the albatross-like glider at 24 m/s failed the dense replay and was re-solved from other start seeds (it now passes); the easyglider has no straight upwind motor cycle at 6, 9 and 11 m/s (the optimizer did not converge from either seed set and either margin set) and none of its 120, 135 and 150 deg cycles at 9 and 11 m/s; the demo has no 120 deg cycle at 9 m/s.
The easyglider and the demo are not used in the experiments below.

The tactical layer (`HullNavigator`, mirrored in `sim/viewer/navigator.js`) chooses, for the line to the target, the pair of cycles on either side of the line whose time-weighted mean velocity lies on it, and flies them as legs inside a corridor around the line.
Soaring points are used first (the pair with the largest speed made good).
Only when no soaring pair makes progress does it use the motor, and then not a library cycle but a steady leg (`SteadyModel`): level flight at 10 m aimed by the wind triangle so that the ground track lies on the line, at the airspeed (from 1.2 stall speeds up to 3 best glide speeds, at most 80% of the maximum thrust) of least electric energy per metre made good, with a proportional cross-track guidance (the track turns towards the line by atan(error / 400 m), at most 35 deg).
The wind at 10 m is u10, the reference height of the profile.
The height is 10 m rather than the library's 2 m because level flight into the wind diverges in height where the wind shear exceeds g / V: at 2 m the shear is above that and any slip below the cycle is a contact, at 10 m it is far below it, and on the closed loop the planner then holds the reference.
A mix of a soaring and a motor cycle is not flown (it saves energy on paper, nav.albatross_mix_across_u16_j_per_m against nav.albatross_steady_across_u16_j_per_m for the steady leg across the wind at 16 m/s, but a change of cycle costs a transient of tens of seconds).
The corridor of a soaring pair is as wide as a leg of about max(90 s, tack cost / 10%) at the faster cross speed, is clipped on each side where the target would no longer be reachable by soaring, and narrows towards the target.
Inside five arrival radii the navigator steers straight at the target: a soaring flight stays on soaring cycles (it keeps the cycle it flies while that passes within the arrival radius, else it takes the one that passes nearest), a motor leg stays a motor leg, and only a flight with no leg yet starts a steady one for a miss.
The kind still changes where the plan changes it, at the start, and a soaring flight that finds its target upwind of it (a changed wind) would have to change to a steady leg from a state at the speed and height of a soaring zoom; that transient is not handled, and the first version of the final approach did change kind there and nearly lost the aircraft.
The planner flies what the navigator names (`fly_mpc(..., target=(x, y), navigator=True, nav_hull=..., nav_kw={"steady": SteadyModel})`), the steady leg as a one-period reference of constant height, airspeed, thrust and heading, every 5 s (every second in the final approach).
A change between two soaring cycles (a tack or a gybe) waits, for at most 15 s, for the phase at which the new cycle matches the aircraft's state, because at any other phase the mirrored heading and bank are a jump.

What the closed loop needed, found on the way:

- The motor ran on soaring legs, on cycles that need none.
  On the downwind case with the clipped update the share of time with more than 1% of the weight in thrust was nav.albatross_u16_downwind_clipped_motor_on_share, at nav.albatross_u16_downwind_clipped_avg_power_w W on average over the first minute (the soaring legs; the whole run is soaring).
  Cause: in the MPPI update the thrust perturbations are clipped at zero, so their weighted mean is above the nominal even when no sample asks for thrust; the held samples of the sustained sampling were also one-sided (half-normal).
  With `thrust_update="raw"` the held samples are symmetric about the nominal, the update uses the unclipped perturbations, the nominal stays unclipped inside the planner, and only the applied command and the rollouts clip it at zero.
  A synthetic test (`sim/tests/test_thrust_update.py`, costs independent of the thrust, through `MPPIPlanner.plan`) gives a mean nominal thrust of nav.thrust_bias_clipped_basic_pct % of the weight with the old update and nav.thrust_bias_raw_basic_signed_pct % with the new one (basic sampling), nav.thrust_bias_clipped_sustained_pct and nav.thrust_bias_raw_sustained_signed_pct % with the sustained sampling (the six constant-thrust candidates are real samples and add part of it).
  The applied command is the nominal clipped at zero, and its mean stays positive because of the spread of the weighted mean about zero (nav.thrust_bias_raw_basic_applied_pct % and nav.thrust_bias_raw_sustained_applied_pct %); the wording "unbiased" therefore holds for the signed nominal only.
  The default of the planner for other uses is unchanged ("clipped").
  The motor share of the soaring downwind runs is in the table below.
  What remains has two sources that this pass does not separate: the clip of a noisy nominal, which alone applies about nav.thrust_bias_raw_basic_applied_pct % of the weight when the thrust does not matter, and the shortfall that the planner's dynamics (folded-propeller drag, actuator lags) leave against the library's soaring cycles, which are solved without the propeller drag.
- The heading tolerance of the match to the cycle is 0.1 rad instead of 0.6 rad, the airspeed tolerance 3 m/s instead of a quarter of the mean speed and the flight path tolerance 0.15 rad (`NAV_GUIDE_SIG`): the mean cross velocity of a cycle is set by its mean heading, and with the loose airspeed tolerance and the tight heading the planner let the airspeed fall under the stall speed at the top of a soaring zoom.
- Powered legs climbed off their reference height, to medians of nav.first_run_height_median_min_m to nav.first_run_height_median_max_m m in the first closed-loop runs.
  Causes, each found and fixed: the shortfall term of the terminal cost measured the energy against the aircraft's present energy, so the planner stayed where it had climbed to (for a steady reference it is now measured against the reference energy); the height tolerance of the match was 3 m (now 0.5 m for a steady reference, with a terminal weight of 200 instead of 10, and a heading tolerance of 0.04 rad: with 0.1 rad the cross-track guidance, which turns the reference by a few degrees, was not followed and the upwind legs of the baseline missed the target); and the first plan assumed the thrust started at zero (it now starts at the cycle's own thrust).
  A soft ceiling in the planner (`h_ceil`, 28.5 m for the navigation runs) keeps soaring zooms below the band's top.
- A run counts as arrived only if it also stays inside the height band of R-12 (ds.clearance_min_m to ds.altitude_band_max_m) and breaks no margin: no loop (cos of the flight path angle above zero), the airspeed not below the stall speed, the wingtip at least 0.3 m above the surface, the load factor within its limit.
  The run record holds the median, the minimum and the maximum height and the margins.
- The baseline across the wind at 16 m/s touched the water after nav.first_run_baseline_contact_s s in the first closed-loop run, on a library 150 deg motor cycle (a low steady flight) that the navigator had switched to from a soaring cycle at the speed and height of a zoom: the planner dived to match the reference within its horizon, with the roll rate often at its limit and the load factor near its limit.
  That run also had the loose airspeed tolerance and the clipped thrust update; the same case now flies a steady leg from the start.

Closed loop (`python -m sim.mpc --nav-exp --nav-dist 5000`, key `nav_exp` in `sim/out/mpc.json`; `--nav-dist 10000 --nav-key nav_exp_10km` for the 10 km cases; each stamped with the commit the runs used; the stored `planner` string of their meta predates the wording "the update on the unclipped, symmetric perturbations" and still says "unbiased thrust update"): a flat COARE 3.5 sea, true state and wind, the motor on at the middle energy weight, an arrival radius of 200 m, the navigator called every 5 s.
The prediction (hull speed, hull time) is the kinematic run on the same hull and steady model (`sail_hull`).
The table lists the record keys of every result (`python -m sim.nav_keys` writes them); the values are in the generated value table of docs/01-requirements.md.

<!-- nav_keys begin -->
| aircraft | u10 | target | arrival time | counted | VMG | hull speed | hull time | energy | motor on | height median | height max |
|---|---|---|---|---|---|---|---|---|---|---|---|
| albatross | 13 | upwind 5 km | nav.run5_albatross_u13_upwind_arrival_s | nav.run5_albatross_u13_upwind_counted | nav.run5_albatross_u13_upwind_vmg_ms | nav.run5_albatross_u13_upwind_hull_ms | nav.run5_albatross_u13_upwind_hull_s | nav.run5_albatross_u13_upwind_energy_wh | nav.run5_albatross_u13_upwind_motor_share | nav.run5_albatross_u13_upwind_height_median_m | nav.run5_albatross_u13_upwind_height_max_m |
| albatross | 13 | across 5 km | nav.run5_albatross_u13_across_arrival_s | nav.run5_albatross_u13_across_counted | nav.run5_albatross_u13_across_vmg_ms | nav.run5_albatross_u13_across_hull_ms | nav.run5_albatross_u13_across_hull_s | nav.run5_albatross_u13_across_energy_wh | nav.run5_albatross_u13_across_motor_share | nav.run5_albatross_u13_across_height_median_m | nav.run5_albatross_u13_across_height_max_m |
| albatross | 16 | upwind 5 km | nav.run5_albatross_u16_upwind_arrival_s | nav.run5_albatross_u16_upwind_counted | nav.run5_albatross_u16_upwind_vmg_ms | nav.run5_albatross_u16_upwind_hull_ms | nav.run5_albatross_u16_upwind_hull_s | nav.run5_albatross_u16_upwind_energy_wh | nav.run5_albatross_u16_upwind_motor_share | nav.run5_albatross_u16_upwind_height_median_m | nav.run5_albatross_u16_upwind_height_max_m |
| albatross | 16 | across 5 km | nav.run5_albatross_u16_across_arrival_s | nav.run5_albatross_u16_across_counted | nav.run5_albatross_u16_across_vmg_ms | nav.run5_albatross_u16_across_hull_ms | nav.run5_albatross_u16_across_hull_s | nav.run5_albatross_u16_across_energy_wh | nav.run5_albatross_u16_across_motor_share | nav.run5_albatross_u16_across_height_median_m | nav.run5_albatross_u16_across_height_max_m |
| albatross | 16 | downwind 5 km | nav.run5_albatross_u16_downwind_arrival_s | nav.run5_albatross_u16_downwind_counted | nav.run5_albatross_u16_downwind_vmg_ms | nav.run5_albatross_u16_downwind_hull_ms | nav.run5_albatross_u16_downwind_hull_s | nav.run5_albatross_u16_downwind_energy_wh | nav.run5_albatross_u16_downwind_motor_share | nav.run5_albatross_u16_downwind_height_median_m | nav.run5_albatross_u16_downwind_height_max_m |
| baseline | 16 | upwind 5 km | nav.run5_baseline_u16_upwind_arrival_s | nav.run5_baseline_u16_upwind_counted | nav.run5_baseline_u16_upwind_vmg_ms | nav.run5_baseline_u16_upwind_hull_ms | nav.run5_baseline_u16_upwind_hull_s | nav.run5_baseline_u16_upwind_energy_wh | nav.run5_baseline_u16_upwind_motor_share | nav.run5_baseline_u16_upwind_height_median_m | nav.run5_baseline_u16_upwind_height_max_m |
| baseline | 16 | across 5 km | nav.run5_baseline_u16_across_arrival_s | nav.run5_baseline_u16_across_counted | nav.run5_baseline_u16_across_vmg_ms | nav.run5_baseline_u16_across_hull_ms | nav.run5_baseline_u16_across_hull_s | nav.run5_baseline_u16_across_energy_wh | nav.run5_baseline_u16_across_motor_share | nav.run5_baseline_u16_across_height_median_m | nav.run5_baseline_u16_across_height_max_m |
| albatross | 9 | upwind 5 km | nav.run5_albatross_u9_upwind_arrival_s | nav.run5_albatross_u9_upwind_counted | nav.run5_albatross_u9_upwind_vmg_ms | nav.run5_albatross_u9_upwind_hull_ms | nav.run5_albatross_u9_upwind_hull_s | nav.run5_albatross_u9_upwind_energy_wh | nav.run5_albatross_u9_upwind_motor_share | nav.run5_albatross_u9_upwind_height_median_m | nav.run5_albatross_u9_upwind_height_max_m |
| albatross | 13 | upwind 10 km | nav.run10_albatross_u13_upwind_arrival_s | nav.run10_albatross_u13_upwind_counted | nav.run10_albatross_u13_upwind_vmg_ms | nav.run10_albatross_u13_upwind_hull_ms | nav.run10_albatross_u13_upwind_hull_s | nav.run10_albatross_u13_upwind_energy_wh | nav.run10_albatross_u13_upwind_motor_share | nav.run10_albatross_u13_upwind_height_median_m | nav.run10_albatross_u13_upwind_height_max_m |
| albatross | 13 | across 10 km | nav.run10_albatross_u13_across_arrival_s | nav.run10_albatross_u13_across_counted | nav.run10_albatross_u13_across_vmg_ms | nav.run10_albatross_u13_across_hull_ms | nav.run10_albatross_u13_across_hull_s | nav.run10_albatross_u13_across_energy_wh | nav.run10_albatross_u13_across_motor_share | nav.run10_albatross_u13_across_height_median_m | nav.run10_albatross_u13_across_height_max_m |
| albatross | 16 | upwind 10 km | nav.run10_albatross_u16_upwind_arrival_s | nav.run10_albatross_u16_upwind_counted | nav.run10_albatross_u16_upwind_vmg_ms | nav.run10_albatross_u16_upwind_hull_ms | nav.run10_albatross_u16_upwind_hull_s | nav.run10_albatross_u16_upwind_energy_wh | nav.run10_albatross_u16_upwind_motor_share | nav.run10_albatross_u16_upwind_height_median_m | nav.run10_albatross_u16_upwind_height_max_m |
| albatross | 16 | across 10 km | nav.run10_albatross_u16_across_arrival_s | nav.run10_albatross_u16_across_counted | nav.run10_albatross_u16_across_vmg_ms | nav.run10_albatross_u16_across_hull_ms | nav.run10_albatross_u16_across_hull_s | nav.run10_albatross_u16_across_energy_wh | nav.run10_albatross_u16_across_motor_share | nav.run10_albatross_u16_across_height_median_m | nav.run10_albatross_u16_across_height_max_m |
<!-- nav_keys end -->

- The rows with "counted" 1 arrived inside the height band and the margins; the arrival time is 0 for a run that did not count.
- Downwind the albatross-like glider arrives on soaring cycles after nav.run5_albatross_u16_downwind_arrival_s s, with a motor share of nav.run5_albatross_u16_downwind_motor_share over the whole run (the run is all soaring).
- Upwind and across the wind the motor share is the share of a powered leg and the energy per kilometre made good is the steady leg's.
- Open item Q-066: the condition of the question (a closed-loop flight reaches a target 10 km upwind in the simulation) is met on a flat sea, with the truth state and wind, for the albatross-like glider at 13 and 16 m/s (nav.run10_albatross_u16_upwind_arrival_s s at 16 m/s) with the motor; it stays open for what is not shown: waves, the state and the wind from the estimator, targets at other angles to the wind, a wind that changes during the flight (and with it a change of kind from soaring to a powered leg, see above), the other forms at 10 km, and a mix of soaring and powered legs.
  A node closed loop in the viewer's own code reaches a target 1 km upwind at 16 m/s.

## Control modes {#control-modes}

The owner asked for a state transition diagram as well as a flowchart (2026-10-29).
The states are the modes of the whole aircraft; the flowchart is the loop inside the soaring mode.

### State transition diagram

```mermaid
stateDiagram-v2
    [*] --> Launch
    Launch --> Soar: airborne and shear usable
    Launch --> Float: launch failed on water
    Soar --> Loiter: at target
    Loiter --> Soar: task done or shear fading
    Loiter --> Task: on station and energy margin
    Task --> Loiter: data captured
    Soar --> Land: shear below threshold for T_hold or energy below reserve
    Loiter --> Land: shear below threshold for T_hold or energy below reserve
    Land --> Float: touchdown
    Float --> Sleep: battery low or night
    Sleep --> Float: wake timer or battery recovered
    Float --> Comms: link window
    Comms --> Float: sent or window closed
    Float --> Takeoff: wind above takeoff threshold and sea state allowed
    Takeoff --> Soar: airborne
    Takeoff --> Float: take-off failed
    Soar --> Safe: fault
    Loiter --> Safe: fault
    Task --> Safe: fault
    Takeoff --> Safe: fault
    Safe --> Land: still flying
    Safe --> Float: on water
    Safe --> Float: fault cleared on water
```

- Launch: hand launch at sea or take-off from the water; ends when airspeed and height are inside the soaring band.
- Soar: dynamic soaring cycles that make progress toward the waypoint; the loop below runs here.
- Loiter: soaring cycles that hold position near the target.
- Task: the photo or data capture, entered only with an energy margin.
- Land: a controlled descent to the water into the wind at stall speed plus margin.
- Float: on the water, sealed, self-righted, charging, estimating the wind from the drift and the motion.
- Sleep: everything off but the wake timer and the sealed-ness watchdog.
- Comms: the satellite or shore link; also used during Loiter when the link allows.
- Takeoff: the water take-off, whose mechanism is open (R-05).
- Safe: fault handling; the only exits are to the water.

### Flowchart of the soaring loop

```mermaid
flowchart TD
    A[Estimate wind profile and sea surface from airspeed, GNSS, IMU and the last cycles] --> B[Choose cycle: heading, bank schedule, lift schedule, period]
    B --> C[Fly one cycle]
    C --> D{Energy gained over the cycle?}
    D -- yes --> E[Advance toward the waypoint; keep the cycle]
    D -- no --> F[Adjust the cycle: tighter turn, lower turn height, or more upwind climb]
    E --> G{Shear usable for T_hold?}
    F --> G
    G -- yes --> A
    G -- no --> H[Leave Soar: Land]
```

The thresholds T_hold, the shear threshold, and the energy reserve are values to be set from the simulation (Q-009).

The owner asked on 2026-10-29 that the drone judge its flight moment to moment from its own view of the waves rather than fly a cycle planned for a flat sea; the soaring loop above becomes a receding-horizon planner that predicts the local surface and wind a few seconds ahead from what the aircraft senses and re-plans every half second (Q-057).
