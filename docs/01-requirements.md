# 1.1 Requirements

Why each requirement in record/requirements.yaml is what it is. Start with the questions that can rule a form out.


## Certification routes (Q-001) {#cert-routes}

The owner chose type certification (D-003) and flight beyond visual line of sight over places where people are (D-004).
D-012 and D-013 have since replaced both choices for the drone that stays over the sea.
Flight beyond visual line of sight over people is category III, which is allowed only with an aircraft holding class 1 aircraft certification, a first class remote pilot certificate, and a permission and approval (P2 Articles 132-85 and 132-86, P13).
Flight beyond visual line of sight also needs the visual line of sight limitation on the remote pilot certificate to be lifted (P2 Article 132-43).
Type certification certifies the design and the production process of a type, and it inspects the design, the production process, and the present state of one aircraft of the type (P2 Article 132-16, P3 Article 236-23).
Type certification also applies the uniformity standard, which asks for a quality control system and an audit independent of the working organization (P3 Article 236-24).
The guidelines do not allow a split in which the designer, maker, inspector, and person responsible for a component are all the same single person with nobody else, and they allow one person to hold several roles (P7 p244).
The auditor may hold other roles and need not be a standing organization (P7 pp268 to 269).
The law lets registered inspection bodies carry out inspections, but in current practice the State inspects class 1, and the registered inspection bodies are capable of class 2 only (P2 Article 132-24, P6 Part I 2, P11).
Most registered inspection bodies limit aircraft certification to type certified aircraft, so a class 2 aircraft without type certification may also be inspected by the State (P11).
A new aircraft of a type certified type skips the aircraft certification inspection in class 2, in principle entirely, while in class 1 an on-site inspection of its present state remains (P4 6-1).
A new aircraft here means one that has never flown, indoors included, keeps its equipment as shipped, and has not been modified (P4 section 4, notes 1 and 6).
Under the class 2 safety standard, an aircraft under cert.self_confirm_weight can meet many items by submitting the results of the applicant's own checks (P6 Part II chapter 3).
An aircraft without type certification can still obtain aircraft certification, but the State says the inspection of its design and production process will take considerable time (P3 Article 236-12, P1 note 7).
Aircraft certification is valid for cert.validity_aircraft_class1 in class 1 and cert.validity_aircraft_class2 in class 2, and type certification for cert.validity_type (P3).

### Against the budget

On every route, the State's fees alone use most of budget.total or more.
The fee for class 1 type certification is cert.fee_type_class1_min to cert.fee_type_class1_max (P10).
The fee for class 2 type certification under cert.self_confirm_weight is cert.fee_type_class2_under4kg (P10).
The fee for aircraft certification without type certification starts at cert.fee_aircraft_class1_untyped_min in class 1, and is cert.fee_aircraft_class2_untyped_under4kg in class 2 under cert.self_confirm_weight (P9).
The durability and reliability flight test for class 1 takes cert.dr_hours_example_390 in a US example, or cert.dr_hours_example_390_mitigated with harm mitigation (P7 p143).
The fee tables were made in 2022-11, but the amounts match the current fee ordinance (P24, K-001).
The number of types holding class 1 type certification is market.class1_type_models, and every one of the market.level4_flights Level 4 flights the State lists used that type (P22, P23 page 9).
So the scope of the budget is asked as Q-003, class 1 or class 2 as Q-004, and the role that one person cannot hold alone as Q-005.

## Values

<!-- tdr:table kind=value fields=key,v,unit,status,basis -->
| key | v | unit | status | basis |
|---|---|---|---|---|
| budget.total | 300000 | JPY | decided | D-005 |
| law.uas_min_weight | 100 | g | sourced | P3 |
| law.permit_weight | 25 | kg | sourced | P3 |
| cert.self_confirm_weight | 4 | kg | sourced | P6 |
| cert.fee_type_class1_min | 2209300 | JPY | sourced | P10 |
| cert.fee_type_class1_max | 2731800 | JPY | sourced | P10 |
| cert.fee_type_class2_under4kg | 290500 | JPY | sourced | P10 |
| cert.fee_aircraft_class1_untyped_min | 1481200 | JPY | sourced | P9 |
| cert.fee_aircraft_class2_untyped_under4kg | 284900 | JPY | sourced | P9 |
| cert.validity_aircraft_class1 | 1 | year | sourced | P3 |
| cert.validity_aircraft_class2 | 3 | year | sourced | P3 |
| cert.validity_type | 3 | year | sourced | P3 |
| cert.dr_hours_example_390 | 375 | h | sourced | P7 |
| cert.dr_hours_example_390_mitigated | 150 | h | sourced | P7 |
| market.class1_type_models | 1 | count | sourced | P22 |
| market.level4_flights | 8 | count | sourced | P23 |
| cost.license_exam_written_2nd | 8800 | JPY | sourced | P26 |
| cost.license_exam_practical_2nd_basic | 20400 | JPY | sourced | P26 |
| cost.license_exam_practical_2nd_bvlos | 19800 | JPY | sourced | P26 |
| cost.license_medical_documents | 5200 | JPY | sourced | P26 |
| cost.license_medical_venue | 19900 | JPY | sourced | P26 |
| cost.license_issue | 3000 | JPY | sourced | P27 |
| cost.license_renewal_issue | 2850 | JPY | sourced | P27 |
| cost.license_validity | 3 | year | sourced | P28 |
| cost.license_course_2nd_beginner_min | 230000 | JPY | sourced | M6, M7, M8 |
| cost.license_course_2nd_beginner_max | 300000 | JPY | sourced | M6, M7, M8 |
| cost.license_course_2nd_experienced_min | 80000 | JPY | sourced | M6, M7, M8 |
| cost.license_course_2nd_experienced_max | 128000 | JPY | sourced | M6, M7, M8 |
| cost.license_course_bvlos_min | 33000 | JPY | sourced | M7 |
| cost.license_course_bvlos_max | 35000 | JPY | sourced | M6, M8 |
| cost.license_renewal_course_min | 8800 | JPY | sourced | M9 |
| cost.license_renewal_course_max | 20000 | JPY | sourced | M9 |
| cost.registration_fee_mynumber | 900 | JPY | sourced | P21 |
| cost.registration_fee_other_id | 1450 | JPY | sourced | P21 |
| cost.registration_validity | 3 | year | sourced | P21 |
| cost.remote_id_external_min | 16900 | JPY | sourced | F1 |
| cost.remote_id_external_typ | 17600 | JPY | sourced | F1 |
| cost.permit_system_fee | 0 | JPY | sourced | P30 |
| cost.lte_docomo_flat_month | 49800 | JPY | sourced | M1 |
| cost.lte_docomo_metered_base_month | 12500 | JPY | sourced | M1 |
| cost.lte_softbank_month | 49800 | JPY | sourced | M2 |
| cost.insurance_hobby_two_years | 6500 | JPY | sourced | M3 |
| cost.insurance_hobby_cover | 100000000 | JPY | sourced | M3 |
| cost.insurance_business_year | 7350 | JPY | sourced | M4 |
| cost.insurance_business_cover | 100000000 | JPY | sourced | M4 |
| cost.test_field_private_day | 3000 | JPY | sourced | M10 |
| cost.test_field_rtf_half_day_third | 20200 | JPY | sourced | P36 |
| cost.non_airframe_once_min | 78000 | JPY | calculated | docs/05-parts-and-cost.md#totals |
| cost.non_airframe_once_typ | 391250 | JPY | calculated | docs/05-parts-and-cost.md#totals |
| cost.non_airframe_year_min | 9167 | JPY | calculated | docs/05-parts-and-cost.md#totals |
| cost.non_airframe_year_typ | 17183 | JPY | calculated | docs/05-parts-and-cost.md#totals |
| env.karman | 0.4 | 1 | assumed | Q-008 |
| env.coare35_charnock_slope | 0.0017 | s/m | sourced | P94, P66 |
| env.coare35_charnock_intercept | -0.005 | 1 | sourced | P94, P66 |
| env.coare35_charnock_max | 0.028 | 1 | sourced | P94 |
| lit.davenport_open_sea_z0_m | 0.0002 | m | sourced | P99 |
| lit.davenport_open_flat_z0_m | 0.03 | m | sourced | P99 |
| lit.sachs2013_effective_gradient_per_s | 0.1 | 1/s | sourced | P47 |
| lit.sachs2013_required_gradient_per_s | 0.7 | 1/s | sourced | P47 |
| env.sea_z0_min_m | 1e-05 | m | calculated | docs/02-form.md#sea-roughness |
| env.sea_z0_max_m | 0.0007 | m | calculated | docs/02-form.md#sea-roughness |
| env.sea_shear_1_10_min | 0.17 | 1 | calculated | docs/02-form.md#sea-roughness |
| env.sea_shear_1_10_max | 0.24 | 1 | calculated | docs/02-form.md#sea-roughness |
| env.z003_shear_1_10 | 0.4 | 1 | calculated | docs/02-form.md#sea-roughness |
| env.u1_over_u10_charnock_10ms | 0.79 | 1 | calculated | docs/02-form.md#sea-roughness |
| env.u1_over_u10_z003_10ms | 0.6 | 1 | calculated | docs/02-form.md#sea-roughness |
| env.swell_shear_2p5_8_measured_min | 0.11 | 1 | calculated | docs/02-form.md#sea-roughness |
| env.swell_shear_2p5_8_measured_max | 0.25 | 1 | calculated | docs/02-form.md#sea-roughness |
| env.swell_shear_2p5_8_log | 0.09 | 1 | calculated | docs/02-form.md#sea-roughness |
| env.stability_factor_u4_warm_sea | 0.74 | 1 | calculated | docs/02-form.md#sea-roughness |
| env.stability_factor_u4_cold_sea | 2.0 | 1 | calculated | docs/02-form.md#sea-roughness |
| env.stability_factor_u8_warm_sea | 0.9 | 1 | calculated | docs/02-form.md#sea-roughness |
| env.stability_factor_u8_cold_sea | 1.21 | 1 | calculated | docs/02-form.md#sea-roughness |
| dsopt.sachs_sea_charnock_u10_ms | 15.46 | m/s | calculated | docs/02-form.md#sea-roughness |
| dsopt.sachs_sea_coare35_u10_ms | 14.85 | m/s | calculated | docs/02-form.md#sea-roughness |
| ctrl.roll_pb2v | 0.08 | 1 | assumed | Q-062 |
| ctrl.cl_rate_max_per_s | 1.0 | 1/s | assumed | Q-062 |
| ctrl.actuator_tau_s | 0.05 | s | assumed | Q-062 |
| ctrl.actuator_delay_s | 0.05 | s | assumed | Q-062 |
| ctrl.const_cl_rate_max_per_s | 2.0 | 1/s | assumed | Q-062 |
| ctrl.const_actuator_tau_s | 0.1 | s | assumed | Q-062 |
| ctrl.bank_rate_max_deg_s | 150 | deg/s | assumed | Q-062 |
| lit.gilruth_pb2v_min | 0.07 | 1 | sourced | P106 |
| lit.bower_bank_rate_limit_deg_s | 90 | deg/s | sourced | P45 |
| lit.bower_cl_rate_limit_per_s | 0.5 | 1/s | sourced | P45 |
| lit.bronz_pb2v_p99_min | 0.057 | 1 | calculated | docs/02-form.md#roll-authority |
| lit.bronz_pb2v_p99_max | 0.073 | 1 | calculated | docs/02-form.md#roll-authority |
| lit.bronz_cl_rate_p99_max_per_s | 1.67 | 1/s | calculated | docs/02-form.md#roll-authority |
| lit.albatross_bank_rate_mean_median_deg_s | 25 | deg/s | calculated | docs/02-form.md#roll-authority |
| lit.albatross_bank_rate_peak_median_deg_s | 66 | deg/s | calculated | docs/02-form.md#roll-authority |
| lit.albatross_bank_rate_peak_p90_deg_s | 116 | deg/s | calculated | docs/02-form.md#roll-authority |
| dsopt.pb2v008_u10_baseline_ms | 10.81 | m/s | calculated | docs/02-form.md#roll-authority |
| dsopt.pb2v008_u10_albatross_ms | 9.19 | m/s | calculated | docs/02-form.md#roll-authority |
| dsopt.roll30_u10_baseline_ms | 11.87 | m/s | calculated | docs/02-form.md#roll-authority |
| dsopt.roll30_u10_albatross_ms | 9.85 | m/s | calculated | docs/02-form.md#roll-authority |
| dsopt.midpoint_u10_baseline_ms | 10.76 | m/s | calculated | docs/02-form.md#roll-authority |
| dsopt.midpoint_u10_albatross_ms | 9.06 | m/s | calculated | docs/02-form.md#roll-authority |
| dsopt.margin2_u10_baseline_ms | 13.29 | m/s | calculated | docs/02-form.md#roll-authority |
| dsopt.margin2_u10_albatross_ms | 10.09 | m/s | calculated | docs/02-form.md#roll-authority |
| dsopt.margin3_u10_baseline_ms | 16.22 | m/s | calculated | docs/02-form.md#roll-authority |
| dsopt.margin3_u10_albatross_ms | 12.34 | m/s | calculated | docs/02-form.md#roll-authority |
| dsopt.sachs_coare_floor15_u10_ms | 14.87 | m/s | calculated | docs/02-form.md#roll-authority |
| dsopt.sachs_coare_floor03_u10_ms | 14.15 | m/s | calculated | docs/02-form.md#roll-authority |
| dsopt.albatross_coare_floor15_u10_ms | 9.29 | m/s | calculated | docs/02-form.md#roll-authority |
| dsopt.albatross_coare_floor03_u10_ms | 8.77 | m/s | calculated | docs/02-form.md#roll-authority |
| dsopt.sachs_coare_floor03_zmin_m | 1.0 | m | calculated | docs/02-form.md#energy-sources |
| dsopt.sachs_coare_notip_floor15_u10_ms | 14.16 | m/s | calculated | docs/02-form.md#energy-sources |
| dsopt.sachs_coare_notip_floor05_u10_ms | 10.74 | m/s | calculated | docs/02-form.md#energy-sources |
| dsopt.sachs_coare_notip_floor01_u10_ms | 7.81 | m/s | calculated | docs/02-form.md#energy-sources |
| dsopt.sachs_z003_notip_floor05_u10_ms | 5.82 | m/s | calculated | docs/02-form.md#energy-sources |
| dsopt.sachs_z003_notip_floor01_u10_ms | 3.79 | m/s | calculated | docs/02-form.md#energy-sources |
| dsopt.albatross_coare_notip_floor01_u10_ms | 4.49 | m/s | calculated | docs/02-form.md#energy-sources |
| mpc.margin_library_entries | 27 | 1 | calculated | docs/02-form.md#receding-horizon |
| mpc.margin_library_slots | 32 | 1 | calculated | docs/02-form.md#receding-horizon |
| mpc.margin_flat_truth_run_s | 300 | s | calculated | docs/02-form.md#receding-horizon |
| mpc.margin_flat_est_survival_min_s | 60 | s | calculated | docs/02-form.md#receding-horizon |
| mpc.margin_flat_est_survival_max_s | 201 | s | calculated | docs/02-form.md#receding-horizon |
| mpc.margin_sea_survival_min_s | 4 | s | calculated | docs/02-form.md#receding-horizon |
| mpc.margin_sea_survival_max_s | 82 | s | calculated | docs/02-form.md#receding-horizon |
| lit.bird_experienced_wind_ms | 8.7 | m/s | sourced | P73 |
| lit.bird_flapping_time_fraction | 0.046 | 1 | sourced | P81 |
| lit.bird_flapping_energy_fraction | 0.133 | 1 | sourced | P81 |
| dsopt.sachs_coare_dw_ms | 4.02 | m/s | calculated | docs/02-form.md#energy-sources |
| dsopt.sachs_notip_dw_ms | 3.17 | m/s | calculated | docs/02-form.md#energy-sources |
| dsopt.albatross_coare_dw_ms | 1.92 | m/s | calculated | docs/02-form.md#energy-sources |
| dsopt.baseline_coare_dw_ms | 2.43 | m/s | calculated | docs/02-form.md#energy-sources |
| dsopt.sachs_coare_dw_ratio | 0.27 | 1 | calculated | docs/02-form.md#energy-sources |
| dsopt.sachs_z003_dw_ratio | 0.44 | 1 | calculated | docs/02-form.md#energy-sources |
| prop.prop_eff | 0.6 | 1 | sourced | P140, P141 |
| prop.motor_eff | 0.75 | 1 | sourced | P49, P142, P143 |
| prop.folded_drag_cda_m2 | 0.0003 | m2 | sourced | P145, P146 |
| prop.thrust_max_per_weight | 0.5 | 1 | assumed | Q-055 |
| env.breaking_crest_length_frac | 0.25 | 1 | assumed | Q-064 |
| env.breaking_lifetime_s | 2.0 | s | assumed | Q-064 |
| env.breaking_lift_max_frac | 0.3 | 1 | assumed | Q-064 |
| env.breaking_lee_deficit_factor | 1.2 | 1 | sourced | P92, P93 |
| env.swell_jet_height_m | 8.0 | m | sourced | P100 |
| env.swell_jet_max_ratio | 1.1 | 1 | assumed | Q-061 |
| env.swell_jet_shear_frac | 0.25 | 1 | sourced | P65 |
| env.turb_lu_per_height | 2.4 | 1 | sourced | P132 |
| env.turb_lw_per_height | 0.58 | 1 | sourced | P132 |
| env.wave_wind_critical_floor | 0.1 | 1 | assumed | Q-064 |
| env.z0t_m | 0.0001 | m | assumed | Q-061 |
| dsopt.ge_sachs_off_u10_ms | 14.875 | m/s | calculated | docs/02-form.md#ground-motor |
| dsopt.ge_sachs_on_u10_ms | 14.505 | m/s | calculated | docs/02-form.md#ground-motor |
| dsopt.ge_albatross_off_u10_ms | 9.286 | m/s | calculated | docs/02-form.md#ground-motor |
| dsopt.ge_albatross_on_u10_ms | 9.034 | m/s | calculated | docs/02-form.md#ground-motor |
| dsopt.ge_baseline_off_u10_ms | 10.822 | m/s | calculated | docs/02-form.md#ground-motor |
| dsopt.ge_baseline_on_u10_ms | 10.606 | m/s | calculated | docs/02-form.md#ground-motor |
| dsopt.ge_sachs_notip01_off_u10_ms | 7.814 | m/s | calculated | docs/02-form.md#ground-motor |
| dsopt.ge_sachs_notip01_on_u10_ms | 7.204 | m/s | calculated | docs/02-form.md#ground-motor |
| motor.baseline_calm_w | 12.42 | W | calculated | docs/02-form.md#ground-motor |
| motor.albatross_calm_w | 17.55 | W | calculated | docs/02-form.md#ground-motor |
| motor.baseline_u6_w | 10.98 | W | calculated | docs/02-form.md#ground-motor |
| motor.albatross_u6_w | 14.72 | W | calculated | docs/02-form.md#ground-motor |
| motor.baseline_u9_w | 5.86 | W | calculated | docs/02-form.md#ground-motor |
| motor.albatross_u9_w | 3.29 | W | calculated | docs/02-form.md#ground-motor |
| motor.baseline_flapglide_w | 13.4 | W | calculated | docs/02-form.md#ground-motor |
| motor.albatross_flapglide_w | 20.5 | W | calculated | docs/02-form.md#ground-motor |
| motor.baseline_calm_day_wh | 298 | Wh | calculated | docs/02-form.md#ground-motor |
| motor.albatross_calm_day_wh | 421 | Wh | calculated | docs/02-form.md#ground-motor |
| env.crest_stokes2_over_forristall_1e2 | 1.005 | 1 | calculated | docs/02-form.md#simulator-fitness |
| env.crest_linear_over_forristall_1e2 | 0.935 | 1 | calculated | docs/02-form.md#simulator-fitness |
| env.stability_l50_shear_factor | 1.37 | 1 | calculated | docs/02-form.md#simulator-fitness |
| env.stability_l50_cycle_shear_factor | 1.52 | 1 | calculated | docs/02-form.md#simulator-fitness |
| env.stability_lm50_shear_factor | 0.85 | 1 | calculated | docs/02-form.md#simulator-fitness |
| env.swell_jet_shear_factor_min | 1.43 | 1 | calculated | docs/02-form.md#simulator-fitness |
| env.swell_jet_shear_factor_max | 1.97 | 1 | calculated | docs/02-form.md#simulator-fitness |
| env.turb_lw_at_1m_height_law_m | 0.76 | m | calculated | docs/02-form.md#simulator-fitness |
| env.breaking_marked_over_whitecap_u10 | 0.015 | 1 | calculated | docs/02-form.md#simulator-fitness |
| dsopt.wave4_cases_planned | 64 | 1 | calculated | docs/02-form.md#wave-cycles |
| dsopt.wave4_cases_run | 31 | 1 | calculated | docs/02-form.md#wave-cycles |
| dsopt.wave4_floor_cases | 18 | 1 | calculated | docs/02-form.md#wave-cycles |
| dsopt.wave4_p64_floor_cases | 6 | 1 | calculated | docs/02-form.md#wave-cycles |
| dsopt.wave4_p64_cases_run | 7 | 1 | calculated | docs/02-form.md#wave-cycles |
| dsopt.wave4_flat_margin_baseline_ms | 13.11 | m/s | calculated | docs/02-form.md#wave-cycles |
| dsopt.wave4_flat_margin_albatross_ms | 10.32 | m/s | calculated | docs/02-form.md#wave-cycles |
| dsopt.wave4_flat_none_baseline_ms | 10.89 | m/s | calculated | docs/02-form.md#wave-cycles |
| dsopt.wave4_flat_none_albatross_ms | 9.41 | m/s | calculated | docs/02-form.md#wave-cycles |
| dsopt.wave4_hs1_best_gain_ms | 0.76 | m/s | calculated | docs/02-form.md#wave-cycles |
| dsopt.wave4_baseline_floor_period_min_s | 16.3 | s | calculated | docs/02-form.md#wave-cycles |
| dsopt.wave4_baseline_floor_period_max_s | 37 | s | calculated | docs/02-form.md#wave-cycles |
| dsopt.wave4_period_bound_s | 60 | s | calculated | docs/02-form.md#wave-cycles |
| mpc.js_plan_flat_ms | 6.7 | ms | calculated | docs/02-form.md#viewer |
| mpc.js_plan_truth_ms | 123 | ms | calculated | docs/02-form.md#viewer |
| mpc.burst_baseline_u11_normal_w | 8.8 | W | calculated | docs/02-form.md#receding-horizon |
| mpc.burst_baseline_u9_normal_w | 12.1 | W | calculated | docs/02-form.md#receding-horizon |
| mpc.burst_albatross_u9_normal_w | 16.0 | W | calculated | docs/02-form.md#receding-horizon |
| mpc.burst_albatross_u6_normal_w | 27.1 | W | calculated | docs/02-form.md#receding-horizon |
| mpc.burst_on_share_min | 0.55 | 1 | calculated | docs/02-form.md#receding-horizon |
| mpc.burst_on_share_max | 0.92 | 1 | calculated | docs/02-form.md#receding-horizon |
| mpc.burst_free_min_w | 16 | W | calculated | docs/02-form.md#receding-horizon |
| mpc.burst_free_max_w | 40 | W | calculated | docs/02-form.md#receding-horizon |
| ctrl.thrust_sigma_frac | 0.15 | 1 | assumed | Q-055 |
| prop.thrust_tau_s | 0.1 | s | assumed | Q-055 |
| ctrl.reflex_horizon_s | 1.5 | s | assumed | Q-055 |
| ctrl.reflex_clearance_m | 0.3 | m | assumed | Q-055 |
| ctrl.reflex_v_frac | 1.05 | 1 | assumed | Q-055 |
| mpc.burst_costly_baseline_u11_w | 6.42 | W | calculated | docs/02-form.md#receding-horizon |
| mpc.burst_costly_albatross_u9_w | 13.78 | W | calculated | docs/02-form.md#receding-horizon |
| mpc.burst_contact_seen_min_s | 5.8 | s | calculated | docs/02-form.md#receding-horizon |
| mpc.burst_contact_seen_max_s | 12.3 | s | calculated | docs/02-form.md#receding-horizon |
| mpc.burst_like_for_like_opt_min_w | 9.0 | W | calculated | docs/02-form.md#receding-horizon |
| mpc.burst_like_for_like_opt_max_w | 9.5 | W | calculated | docs/02-form.md#receding-horizon |
| nav.best_upwind_course_off_wind_deg | 60 | deg | calculated | docs/02-form.md#navigation |
| nav.albatross_upwind_vmg_u11_ms | 3.75 | m/s | calculated | docs/02-form.md#navigation |
| nav.albatross_upwind_vmg_u20_ms | 7.55 | m/s | calculated | docs/02-form.md#navigation |
| nav.baseline_upwind_vmg_u13_ms | 3.0 | m/s | calculated | docs/02-form.md#navigation |
| nav.baseline_upwind_vmg_u20_ms | 4.45 | m/s | calculated | docs/02-form.md#navigation |
| nav.no_progress_within_deg_min | 15 | deg | calculated | docs/02-form.md#navigation |
| nav.no_progress_within_deg_max | 30 | deg | calculated | docs/02-form.md#navigation |
| nav.albatross_polar_min_u10_ms | 11 | m/s | calculated | docs/02-form.md#navigation |
| nav.baseline_polar_min_u10_ms | 13 | m/s | calculated | docs/02-form.md#navigation |
| wing.best_span_m | 3.0 | m | calculated | docs/02-form.md#wing-sizing |
| wing.best_ar | 20 | 1 | calculated | docs/02-form.md#wing-sizing |
| wing.best_mass_kg | 3.0 | kg | calculated | docs/02-form.md#wing-sizing |
| wing.best_ld_max | 28.2 | 1 | calculated | docs/02-form.md#wing-sizing |
| wing.best_u10_min_ms | 12.87 | m/s | calculated | docs/02-form.md#wing-sizing |
| wing.rebuilt_albatross_u10_min_ms | 14.21 | m/s | calculated | docs/02-form.md#wing-sizing |
| wing.rebuilt_baseline_u10_min_ms | 14.24 | m/s | calculated | docs/02-form.md#wing-sizing |
| wing.rebuilt_sealedhull_u10_min_ms | 13.84 | m/s | calculated | docs/02-form.md#wing-sizing |
| wing.heavy_skin_gain_min_ms | 0.5 | m/s | calculated | docs/02-form.md#wing-sizing |
| wing.heavy_skin_gain_max_ms | 1.0 | m/s | calculated | docs/02-form.md#wing-sizing |
| wing.fat_pod_loss_min_ms | 0.7 | m/s | calculated | docs/02-form.md#wing-sizing |
| wing.fat_pod_loss_max_ms | 2.1 | m/s | calculated | docs/02-form.md#wing-sizing |
| wing.re_floor | 80000 | 1 | calculated | docs/02-form.md#wing-sizing |
| wing.rho_a_base_kg_m2 | 1.5 | kg/m2 | calculated | docs/02-form.md#wing-sizing |
| wing.cda_par_base_m2 | 0.001 | m2 | calculated | docs/02-form.md#wing-sizing |
| wing.best_ld_glide | 25.1 | 1 | calculated | docs/02-form.md#wing-sizing |
| wing.equal_mass_ar20_u10_ms | 12.99 | m/s | calculated | docs/02-form.md#wing-sizing |
| wing.equal_mass_ar25_u10_ms | 13.1 | m/s | calculated | docs/02-form.md#wing-sizing |
| wing.best_fat_pod_u10_ms | 14.95 | m/s | calculated | docs/02-form.md#wing-sizing |
| wing.best_ballast_kg | 1.55 | kg | calculated | docs/02-form.md#wing-sizing |
| wing.best_wing_loading_npm2 | 65.4 | N/m2 | calculated | docs/02-form.md#wing-sizing |
| wing.body_cda_min_m2 | 0.00198 | m2 | calculated | docs/02-form.md#wing-sizing |
| wing.body_cda_max_m2 | 0.00392 | m2 | calculated | docs/02-form.md#wing-sizing |
| wing.best_body_cda_m2 | 0.00231 | m2 | calculated | docs/02-form.md#wing-sizing |
| wing.unballasted_u10_min_ms | 15.22 | m/s | calculated | docs/02-form.md#wing-sizing |
| wing.unballasted_u10_max_ms | 15.65 | m/s | calculated | docs/02-form.md#wing-sizing |
| wing.ballast_gain_min_ms | 2.02 | m/s | calculated | docs/02-form.md#wing-sizing |
| wing.ballast_gain_max_ms | 2.53 | m/s | calculated | docs/02-form.md#wing-sizing |
| wing.top3_spread_ms | 0.14 | m/s | calculated | docs/02-form.md#wing-sizing |
| wing.start_scatter_ms | 0.9 | m/s | calculated | docs/02-form.md#wing-sizing |
| wing.h180_ar20_sd7037_ms | -6.32 | m/s | calculated | docs/02-form.md#wing-sizing |
| wing.h180_ar25_sd7037_ms | -5.56 | m/s | calculated | docs/02-form.md#wing-sizing |
| wing.h180_ar16_sd7037_ms | -7.26 | m/s | calculated | docs/02-form.md#wing-sizing |
| wing.h180_ar20_ag35r_ms | -6.33 | m/s | calculated | docs/02-form.md#wing-sizing |
| wing.follow_p6_chi20_min_w | 28.2 | W | calculated | docs/02-form.md#wing-sizing |
| wing.follow_p6_chi20_max_w | 33.3 | W | calculated | docs/02-form.md#wing-sizing |
| wing.follow_p9_chi20_min_w | 28.2 | W | calculated | docs/02-form.md#wing-sizing |
| wing.follow_p9_chi20_max_w | 33.3 | W | calculated | docs/02-form.md#wing-sizing |
| wing.follow_p13_chi20_min_w | 0.8 | W | calculated | docs/02-form.md#wing-sizing |
| wing.follow_p13_chi20_max_w | 2.6 | W | calculated | docs/02-form.md#wing-sizing |
| wing.follow_p6_chi180_min_w | 26.0 | W | calculated | docs/02-form.md#wing-sizing |
| wing.follow_p6_chi180_max_w | 33.3 | W | calculated | docs/02-form.md#wing-sizing |
| wing.follow_p9_chi180_min_w | 35.7 | W | calculated | docs/02-form.md#wing-sizing |
| wing.follow_p9_chi180_max_w | 36.8 | W | calculated | docs/02-form.md#wing-sizing |
| wing.follow_p13_chi180_min_w | 49.9 | W | calculated | docs/02-form.md#wing-sizing |
| wing.follow_p13_chi180_max_w | 53.1 | W | calculated | docs/02-form.md#wing-sizing |
| wing.front1_ballast_kg | 1.55 | kg | calculated | docs/02-form.md#wing-sizing |
| wing.front1_mass_kg | 3.0 | kg | calculated | docs/02-form.md#wing-sizing |
| wing.front1_wing_loading_npm2 | 65.4 | N/m2 | calculated | docs/02-form.md#wing-sizing |
| wing.front1_ld_glide | 25.1 | 1 | calculated | docs/02-form.md#wing-sizing |
| wing.front1_u10_ms | 12.87 | m/s | calculated | docs/02-form.md#wing-sizing |
| wing.front1_c9_w | 27.0 | W | calculated | docs/02-form.md#wing-sizing |
| wing.front1_wh_per_km | 0.4 | Wh/km | calculated | docs/02-form.md#wing-sizing |
| wing.front1_c9_chi180_w | 34.4 | W | calculated | docs/02-form.md#wing-sizing |
| wing.front2_ballast_kg | 1.39 | kg | calculated | docs/02-form.md#wing-sizing |
| wing.front2_mass_kg | 3.0 | kg | calculated | docs/02-form.md#wing-sizing |
| wing.front2_wing_loading_npm2 | 52.3 | N/m2 | calculated | docs/02-form.md#wing-sizing |
| wing.front2_ld_glide | 24.6 | 1 | calculated | docs/02-form.md#wing-sizing |
| wing.front2_u10_ms | 13.01 | m/s | calculated | docs/02-form.md#wing-sizing |
| wing.front2_c9_w | 25.1 | W | calculated | docs/02-form.md#wing-sizing |
| wing.front2_wh_per_km | 0.39 | Wh/km | calculated | docs/02-form.md#wing-sizing |
| wing.front2_c9_chi180_w | 35.3 | W | calculated | docs/02-form.md#wing-sizing |
| wing.front3_ballast_kg | 1.12 | kg | calculated | docs/02-form.md#wing-sizing |
| wing.front3_mass_kg | 3.0 | kg | calculated | docs/02-form.md#wing-sizing |
| wing.front3_wing_loading_npm2 | 39.2 | N/m2 | calculated | docs/02-form.md#wing-sizing |
| wing.front3_ld_glide | 23.4 | 1 | calculated | docs/02-form.md#wing-sizing |
| wing.front3_u10_ms | 13.49 | m/s | calculated | docs/02-form.md#wing-sizing |
| wing.front3_c9_w | 23.8 | W | calculated | docs/02-form.md#wing-sizing |
| wing.front3_wh_per_km | 0.4 | Wh/km | calculated | docs/02-form.md#wing-sizing |
| wing.front3_c9_chi180_w | 36.8 | W | calculated | docs/02-form.md#wing-sizing |
| wing.front4_ballast_kg | 0.5 | kg | calculated | docs/02-form.md#wing-sizing |
| wing.front4_mass_kg | 2.1 | kg | calculated | docs/02-form.md#wing-sizing |
| wing.front4_wing_loading_npm2 | 36.6 | N/m2 | calculated | docs/02-form.md#wing-sizing |
| wing.front4_ld_glide | 23.0 | 1 | calculated | docs/02-form.md#wing-sizing |
| wing.front4_u10_ms | 14.02 | m/s | calculated | docs/02-form.md#wing-sizing |
| wing.front4_c9_w | 15.5 | W | calculated | docs/02-form.md#wing-sizing |
| wing.front4_wh_per_km | 0.27 | Wh/km | calculated | docs/02-form.md#wing-sizing |
| wing.front4_c9_chi180_w | 26.8 | W | calculated | docs/02-form.md#wing-sizing |
| wing.front5_ballast_kg | 0.5 | kg | calculated | docs/02-form.md#wing-sizing |
| wing.front5_mass_kg | 1.93 | kg | calculated | docs/02-form.md#wing-sizing |
| wing.front5_wing_loading_npm2 | 42.2 | N/m2 | calculated | docs/02-form.md#wing-sizing |
| wing.front5_ld_glide | 23.1 | 1 | calculated | docs/02-form.md#wing-sizing |
| wing.front5_u10_ms | 14.04 | m/s | calculated | docs/02-form.md#wing-sizing |
| wing.front5_c9_w | 15.0 | W | calculated | docs/02-form.md#wing-sizing |
| wing.front5_wh_per_km | 0.25 | Wh/km | calculated | docs/02-form.md#wing-sizing |
| wing.front5_c9_chi180_w | 24.3 | W | calculated | docs/02-form.md#wing-sizing |
| wing.front6_ballast_kg | 0.0 | kg | calculated | docs/02-form.md#wing-sizing |
| wing.front6_mass_kg | 1.59 | kg | calculated | docs/02-form.md#wing-sizing |
| wing.front6_wing_loading_npm2 | 27.8 | N/m2 | calculated | docs/02-form.md#wing-sizing |
| wing.front6_ld_glide | 21.8 | 1 | calculated | docs/02-form.md#wing-sizing |
| wing.front6_u10_ms | 15.22 | m/s | calculated | docs/02-form.md#wing-sizing |
| wing.front6_c9_w | 10.8 | W | calculated | docs/02-form.md#wing-sizing |
| wing.front6_wh_per_km | 0.2 | Wh/km | calculated | docs/02-form.md#wing-sizing |
| wing.front6_c9_chi180_w | 22.5 | W | calculated | docs/02-form.md#wing-sizing |
| wing.front7_ballast_kg | 0.0 | kg | calculated | docs/02-form.md#wing-sizing |
| wing.front7_mass_kg | 1.43 | kg | calculated | docs/02-form.md#wing-sizing |
| wing.front7_wing_loading_npm2 | 31.1 | N/m2 | calculated | docs/02-form.md#wing-sizing |
| wing.front7_ld_glide | 21.7 | 1 | calculated | docs/02-form.md#wing-sizing |
| wing.front7_u10_ms | 15.4 | m/s | calculated | docs/02-form.md#wing-sizing |
| wing.front7_c9_w | 10.2 | W | calculated | docs/02-form.md#wing-sizing |
| wing.front7_wh_per_km | 0.18 | Wh/km | calculated | docs/02-form.md#wing-sizing |
| wing.front7_c9_chi180_w | 21.0 | W | calculated | docs/02-form.md#wing-sizing |
| wing.c9_chi20_min_w | 10.2 | W | calculated | docs/02-form.md#wing-sizing |
| wing.c9_chi20_max_w | 32.0 | W | calculated | docs/02-form.md#wing-sizing |
| wing.c9_chi180_min_w | 20.8 | W | calculated | docs/02-form.md#wing-sizing |
| wing.c9_chi180_max_w | 36.8 | W | calculated | docs/02-form.md#wing-sizing |
| wing.solver_over_steady_min | 1.11 | 1 | calculated | docs/02-form.md#wing-sizing |
| wing.solver_over_steady_max | 1.14 | 1 | calculated | docs/02-form.md#wing-sizing |
| mpc.motorlib_baseline_calm_w | 13.3 | W | calculated | docs/02-form.md#receding-horizon |
| mpc.motorlib_baseline_u9_w | 10.9 | W | calculated | docs/02-form.md#receding-horizon |
| mpc.motorlib_baseline_u11_w | 7.1 | W | calculated | docs/02-form.md#receding-horizon |
| mpc.motorlib_albatross_calm_w | 20.9 | W | calculated | docs/02-form.md#receding-horizon |
| mpc.motorlib_albatross_u9_w | 9.1 | W | calculated | docs/02-form.md#receding-horizon |
| mpc.motorlib_power_ratio_min | 1.0 | 1 | calculated | docs/02-form.md#receding-horizon |
| mpc.motorlib_power_ratio_max | 2.5 | 1 | calculated | docs/02-form.md#receding-horizon |
| mpc.motorlib_run_s | 180 | s | calculated | docs/02-form.md#receding-horizon |
| mpc.reflex_baseline_u9_count | 31 | 1 | calculated | docs/02-form.md#receding-horizon |
| mpc.reflex_energy_factor_min | 4.26 | 1 | calculated | docs/02-form.md#receding-horizon |
| mpc.reflex_energy_factor_max | 4.42 | 1 | calculated | docs/02-form.md#receding-horizon |
| mpc.motorlib_wave_ended | 6 | 1 | calculated | docs/02-form.md#receding-horizon |
| mpc.motorlib_wave_runs | 16 | 1 | calculated | docs/02-form.md#receding-horizon |
| mpc.motorlib_per_km_ratio_min | 0.81 | 1 | calculated | docs/02-form.md#receding-horizon |
| mpc.motorlib_per_km_ratio_max | 2.2 | 1 | calculated | docs/02-form.md#receding-horizon |
| mpc.motorlib_wave_ended_reflex | 4 | 1 | calculated | docs/02-form.md#receding-horizon |
| env.charnock | 0.0144 | 1 | assumed | Q-008 |
| env.air_kinematic_viscosity | 1.46e-05 | m2/s | assumed | Q-008 |
| env.jonswap_gamma | 3.3 | 1 | assumed | Q-008 |
| env.pm_hs_coefficient | 0.21 | 1 | assumed | Q-008 |
| env.pm_peak_coefficient | 0.877 | 1 | assumed | Q-008 |
| ds.endurance_days | 365 | d | decided | D-008 |
| ds.range_km | 10 | km | decided | D-009 |
| ds.photo_bytes | 10000 | B | assumed | Q-009 |
| ds.min_u10_ms | 8 | m/s | sourced | P45, P42 |
| ds.mass_max_kg | 3 | kg | assumed | Q-009 |
| ds.span_max_m | 3 | m | assumed | Q-009 |
| ds.avionics_power_w | 3 | W | assumed | Q-009 |
| ds.n_limit | 4 | 1 | assumed | Q-009 |
| ds.safety_factor | 1.5 | 1 | assumed | Q-009 |
| ds.survival_hs_m | 4 | m | assumed | Q-009 |
| ds.altitude_band_max_m | 30 | m | assumed | Q-009 |
| ds.clearance_min_m | 1 | m | assumed | Q-009 |
| wing.span_m | 2.0 | m | assumed | Q-009 |
| wing.root_chord_m | 0.16 | m | assumed | Q-009 |
| wing.taper | 0.6 | 1 | assumed | Q-009 |
| wing.thickness_ratio | 0.1 | 1 | assumed | Q-009 |
| wing.design_mass_kg | 1.5 | kg | assumed | Q-009 |
| spar.carbon_density | 1400 | kg/m3 | sourced | P54 |
| spar.carbon_modulus | 35000000000.0 | Pa | sourced | P54 |
| spar.carbon_allowable | 300000000.0 | Pa | sourced | P54 |
| spar.od_mm | 13.0 | mm | calculated | docs/03-design.md#wing-spar |
| spar.wall_mm | 0.75 | mm | calculated | docs/03-design.md#wing-spar |
| spar.mass_kg | 0.089 | kg | calculated | docs/03-design.md#wing-spar |
| spar.margin | 0.34 | 1 | calculated | docs/03-design.md#wing-spar |
| spar.tip_deflection_ratio | 0.15 | 1 | calculated | docs/03-design.md#wing-spar |
| wing.mass_kg | 0.428 | kg | calculated | docs/03-design.md#mass-model |
| airframe.baseline_mass_kg | 1.108 | kg | calculated | docs/03-design.md#mass-model |
| lit.albatross_mass_kg | 9.5 | kg | sourced | P44, P45 |
| lit.albatross_span_m | 3.3 | m | sourced | P45 |
| lit.albatross_wing_area_m2 | 0.65 | m2 | sourced | P44, P45 |
| lit.albatross_aspect_ratio | 16 | 1 | sourced | P45 |
| lit.albatross_lift_to_drag | 21 | 1 | sourced | P42 |
| lit.ds_cycle_period_min_s | 10 | s | sourced | P47 |
| lit.ds_cycle_period_max_s | 15 | s | sourced | P46 |
| lit.ds_cycle_period_s | 12 | s | calculated | docs/02-form.md#literature-numbers |
| lit.ds_cycle_height_m | 15 | m | sourced | P46, P47 |
| lit.ds_turn_angle_min_deg | 54 | deg | sourced | P44 |
| lit.ds_turn_angle_max_deg | 78 | deg | sourced | P44 |
| lit.ds_turn_angle_deg | 66 | deg | calculated | docs/02-form.md#literature-numbers |
| lit.shear_layer_thickness_min_m | 1 | m | sourced | P44 |
| lit.shear_layer_thickness_max_m | 3 | m | sourced | P44 |
| lit.shear_layer_thickness_m | 2 | m | calculated | docs/02-form.md#literature-numbers |
| lit.min_u10_albatross_ms | 8.6 | m/s | sourced | P45 |
| lit.min_u10_mariner_ms | 8.0 | m/s | sourced | P45 |
| lit.min_u10_sailplane_15kg_ms | 13.2 | m/s | sourced | P56 |
| lit.rayleigh_min_wind_difference_min_ms | 3.4 | m/s | sourced | P42 |
| lit.rayleigh_min_wind_difference_max_ms | 3.6 | m/s | sourced | P42 |
| lit.rayleigh_min_wind_difference_ms | 3.5 | m/s | calculated | docs/02-form.md#literature-numbers |
| lit.easyglider_mass_kg | 1.35 | kg | sourced | P48 |
| lit.easyglider_span_m | 1.8 | m | sourced | P48 |
| lit.easyglider_wing_area_m2 | 0.4 | m2 | sourced | P48 |
| lit.atlantiksolar_avionics_w | 6 | W | sourced | P49 |
| lit.insolation_50s_june_kwh_m2_day | 0.71 | kWh/m2/d | sourced | P51 |
| lit.insolation_home_december_kwh_m2_day | 1.64 | kWh/m2/d | sourced | P51, private:loc-01 |
| lit.cell_p45b_wh_per_kg | 231 | Wh/kg | sourced | P50 |
| lit.cs22_limit_load_factor | 5.3 | 1 | sourced | P55 |
| lit.cs22_safety_factor | 1.5 | 1 | sourced | P55 |
| lit.anemometer_trisonica_mass_g | 50 | g | sourced | P52 |
| lit.anemometer_trisonica_power_w | 0.36 | W | sourced | P52 |
| lit.probe_epfl_mass_g | 9 | g | sourced | P53 |
| lit.pultruded_modulus_pa | 35000000000.0 | Pa | sourced | P54 |
| lit.pultruded_flexural_pa | 300000000.0 | Pa | sourced | P54 |
| spar.deflection_limit | 0.15 | 1 | decided | D-014 |
| env.sea_grow_tau_h | 3 | h | assumed | Q-008 |
| env.sea_decay_tau_h | 12 | h | assumed | Q-008 |
| env.breaking_slope_limit | 0.3 | 1 | assumed | Q-008 |
| lit.so_u10_mean_winter_ms | 12 | m/s | sourced | P61 |
| lit.so_u10_mean_summer_ms | 10.5 | m/s | sourced | P61 |
| lit.so_u10_p99_ms | 22 | m/s | sourced | P61 |
| lit.pacific_swell_dominance | 0.9 | 1 | sourced | P62 |
| lit.wave_induced_decay_u | 0.77 | 1 | sourced | P63 |
| lit.wave_induced_amp_u | 0.95 | 1 | sourced | P63 |
| lit.wave_induced_amp_w | 1.07 | 1 | sourced | P63 |
| lit.wave_induced_decay_w | 0.62 | 1 | sourced | P63 |
| lit.swell_knee_height_m | 7.5 | m | sourced | P65 |
| lit.charnock_coare35_at_10ms | 0.012 | 1 | sourced | P66 |
| lit.gust_factor_3s_10min_sea | 1.23 | 1 | sourced | P68 |
| env.turb_length_m | 50 | m | assumed | Q-008 |
| env.turb_gust_factor_check | 1.21 | 1 | calculated | docs/02-form.md#sea-coupling |
| patterns.solar_avg_baseline_w | 0.74 | W | calculated | docs/02-form.md#flight-patterns |
| patterns.flapglide_baseline_wh_per_km_calm | 0.38 | Wh/km | calculated | docs/02-form.md#flight-patterns |
| patterns.flapglide_baseline_wh_per_km_head10 | 4.4 | Wh/km | calculated | docs/02-form.md#flight-patterns |
| patterns.flapglide_baseline_wh_per_km_cross10 | 0.9 | Wh/km | calculated | docs/02-form.md#flight-patterns |
| patterns.battery_fraction | 0.15 | 1 | assumed | Q-012 |
| patterns.wave_slope_floor_m | 0.5 | m | assumed | Q-013 |
| spar.fit_fraction | 0.9 | 1 | assumed | Q-015 |
| spar.joiner_factor | 1.1 | 1 | assumed | Q-015 |
| skin.thickness_mm | 0.2 | mm | assumed | Q-015 |
| skin.density | 1900 | kg/m3 | assumed | Q-015 |
| core.density | 33 | kg/m3 | assumed | Q-015 |
| film.mass_kg | 0.08 | kg | assumed | Q-015 |
| battery.mass_kg | 0.25 | kg | assumed | Q-015 |
| avionics.mass_kg | 0.12 | kg | assumed | Q-015 |
| camera.mass_kg | 0.05 | kg | assumed | Q-015 |
| pod.mass_kg | 0.15 | kg | assumed | Q-015 |
| boom.mass_kg | 0.06 | kg | assumed | Q-015 |
| tail.mass_kg | 0.05 | kg | assumed | Q-015 |
| env.cem_hm0_coeff | 0.0413 | 1 | sourced | P71 |
| env.cem_tp_coeff | 0.651 | 1 | sourced | P71 |
| env.cem_duration_coeff | 0.00523 | 1 | sourced | P71 |
| env.cem_full_hm0_limit | 211.5 | 1 | sourced | P71 |
| env.cem_full_tp_limit | 239.8 | 1 | sourced | P71 |
| env.cd_law_a | 1.1 | 1 | sourced | P71 |
| env.cd_law_b | 0.035 | s/m | sourced | P71 |
| env.turb_v_ratio | 0.8 | 1 | assumed | Q-008 |
| env.turb_w_ratio | 0.5 | 1 | assumed | Q-008 |
| env.inverse_wave_age_wind_driven | 0.83 | 1 | sourced | P88 |
| env.inverse_wave_age_wave_driven | 0.15 | 1 | sourced | P88 |
| env.separation_gamma_floor | 0.1 | 1 | sourced | P89 |
| env.separation_gamma_span | 0.3 | 1 | sourced | P89 |
| env.separation_ustar_c_ref | 0.3 | 1 | sourced | P89 |
| env.separation_exponent | 2.5 | 1 | sourced | P89 |
| env.shelter_ramp_ratio | 1.5 | 1 | assumed | Q-008 |
| env.shelter_factor | 1.0 | 1 | sourced | P92, P93 |
| env.shelter_decay_kz | 0.5 | 1 | assumed | Q-008 |
| env.full_sea_inverse_wave_age | 0.826 | 1 | calculated | docs/02-form.md#wave-age-knee |
| env.storm_h42_inverse_wave_age | 0.331 | 1 | calculated | docs/02-form.md#wave-age-knee |
| env.storm_h42_wave_driven_fraction | 0.824 | 1 | calculated | docs/02-form.md#wave-age-knee |
| env.storm_h42_shear_2_20_log_ms | 0.942 | m/s | calculated | docs/02-form.md#wave-age-knee |
| env.storm_h42_shear_2_20_auto_ms | 0.623 | m/s | calculated | docs/02-form.md#wave-age-knee |
| env.shelter_case_lee_drop_fraction | 0.137 | 1 | calculated | docs/02-form.md#crest-shelter |
| env.shelter_full_sea_u15_fraction | 0.0 | 1 | calculated | docs/02-form.md#crest-shelter |
| env.full_sea_u15_max_slope | 0.266 | 1 | calculated | docs/02-form.md#crest-shelter |
| env.shelter_onset_30m_u10_10 | 0.264 | 1 | calculated | docs/02-form.md#crest-shelter |
| env.shelter_onset_3m_u10_10 | 0.227 | 1 | calculated | docs/02-form.md#crest-shelter |
| env.shelter_depth_3m_m | 0.24 | m | calculated | docs/02-form.md#crest-shelter |
| airframe.mass_gap_kg | 0.392 | kg | calculated | docs/03-design.md#mass-model |
| lit.mariner_mass_kg | 2 | kg | sourced | P45 |
| lit.mariner_lift_to_drag | 25 | 1 | sourced | P45 |
| lit.mariner_wing_loading_n_m2 | 70.6 | N/m2 | sourced | P45 |
| lit.sailplane_mass_kg | 15 | kg | sourced | P56 |
| lit.atlantiksolar_load_factor | 4 | 1 | sourced | P49 |
| lit.atlantiksolar_safety_factor | 1.8 | 1 | sourced | P49 |
| lit.cell_p45b_min_discharge_temp_degc | -40 | degC | sourced | P50 |
| lit.flyingfish_mass_kg | 26 | kg | sourced | P57 |
| lit.flyingfish_wave_height_m | 2 | m | sourced | P57 |
| lit.albatross_preferred_heading_deg | 83 | deg | sourced | P73 |
| patterns.wave_slope_swell_period_s | 8 | s | calculated | docs/02-form.md#flight-patterns |
| cost.insurance_hobby_period | 2 | year | sourced | M3 |
| cost.renewal_medical | 5200 | JPY | assumed | Q-016 |
| form.baseline_2m.wing_area_m2 | 0.25 | m2 | assumed | Q-014 |
| form.baseline_2m.cd0 | 0.015 | 1 | assumed | Q-014 |
| form.baseline_2m.oswald | 0.85 | 1 | assumed | Q-014 |
| form.baseline_2m.cl_max | 1.2 | 1 | assumed | Q-014 |
| form.demo_1p5m.span_m | 1.5 | m | assumed | Q-014 |
| form.demo_1p5m.mass_kg | 1.07 | kg | assumed | Q-014 |
| form.demo_1p5m.wing_area_m2 | 0.19 | m2 | assumed | Q-014 |
| form.demo_1p5m.cd0 | 0.02 | 1 | assumed | Q-014 |
| form.demo_1p5m.oswald | 0.8 | 1 | assumed | Q-014 |
| form.demo_1p5m.cl_max | 1.1 | 1 | assumed | Q-014 |
| form.easyglider_1p8m.span_m | 1.8 | m | assumed | Q-014 |
| form.easyglider_1p8m.mass_kg | 1.35 | kg | assumed | Q-014 |
| form.easyglider_1p8m.wing_area_m2 | 0.4 | m2 | assumed | Q-014 |
| form.easyglider_1p8m.cd0 | 0.03 | 1 | assumed | Q-014 |
| form.easyglider_1p8m.oswald | 0.75 | 1 | assumed | Q-014 |
| form.easyglider_1p8m.cl_max | 1.2 | 1 | assumed | Q-014 |
| form.albatross_3m.span_m | 3.0 | m | assumed | Q-014 |
| form.albatross_3m.mass_kg | 3.0 | kg | assumed | Q-014 |
| form.albatross_3m.wing_area_m2 | 0.36 | m2 | assumed | Q-014 |
| form.albatross_3m.cd0 | 0.012 | 1 | assumed | Q-014 |
| form.albatross_3m.oswald | 0.9 | 1 | assumed | Q-014 |
| form.albatross_3m.cl_max | 1.3 | 1 | assumed | Q-014 |
| form.flyingwing_2m.span_m | 2.0 | m | assumed | Q-014 |
| form.flyingwing_2m.mass_kg | 1.3 | kg | assumed | Q-014 |
| form.flyingwing_2m.wing_area_m2 | 0.4 | m2 | assumed | Q-014 |
| form.flyingwing_2m.cd0 | 0.018 | 1 | assumed | Q-014 |
| form.flyingwing_2m.oswald | 0.8 | 1 | assumed | Q-014 |
| form.flyingwing_2m.cl_max | 0.9 | 1 | assumed | Q-014 |
| form.sealedhull_2p5m.span_m | 2.5 | m | assumed | Q-014 |
| form.sealedhull_2p5m.mass_kg | 2.2 | kg | assumed | Q-014 |
| form.sealedhull_2p5m.wing_area_m2 | 0.34 | m2 | assumed | Q-014 |
| form.sealedhull_2p5m.cd0 | 0.016 | 1 | assumed | Q-014 |
| form.sealedhull_2p5m.oswald | 0.85 | 1 | assumed | Q-014 |
| form.sealedhull_2p5m.cl_max | 1.2 | 1 | assumed | Q-014 |
| form.baseline_2m.u10_min_travel_log | 10.73 | m/s | calculated | docs/02-form.md#form-comparison |
| form.baseline_2m.u10_min_loiter_log | 11.09 | m/s | calculated | docs/02-form.md#form-comparison |
| form.baseline_2m.cross12_speed_log | 12.94 | m/s | calculated | docs/02-form.md#form-comparison |
| form.baseline_2m.u10_min_travel_knee | 10.93 | m/s | calculated | docs/02-form.md#form-comparison |
| form.baseline_2m.u10_min_loiter_knee | 11.7 | m/s | calculated | docs/02-form.md#form-comparison |
| form.demo_1p5m.u10_min_travel_log | 13.19 | m/s | calculated | docs/02-form.md#form-comparison |
| form.demo_1p5m.u10_min_loiter_log | 13.6 | m/s | calculated | docs/02-form.md#form-comparison |
| form.demo_1p5m.u10_min_travel_knee | 13.5 | m/s | calculated | docs/02-form.md#form-comparison |
| form.demo_1p5m.u10_min_loiter_knee | 14.31 | m/s | calculated | docs/02-form.md#form-comparison |
| form.easyglider_1p8m.u10_min_travel_log | 18.27 | m/s | calculated | docs/02-form.md#form-comparison |
| form.easyglider_1p8m.u10_min_loiter_log | 18.88 | m/s | calculated | docs/02-form.md#form-comparison |
| form.easyglider_1p8m.u10_min_travel_knee | 18.04 | m/s | calculated | docs/02-form.md#form-comparison |
| form.easyglider_1p8m.u10_min_loiter_knee | 18.75 | m/s | calculated | docs/02-form.md#form-comparison |
| form.albatross_3m.u10_min_travel_log | 9.03 | m/s | calculated | docs/02-form.md#form-comparison |
| form.albatross_3m.u10_min_loiter_log | 9.38 | m/s | calculated | docs/02-form.md#form-comparison |
| form.albatross_3m.cross12_speed_log | 17.94 | m/s | calculated | docs/02-form.md#form-comparison |
| form.albatross_3m.u10_min_travel_knee | 9.49 | m/s | calculated | docs/02-form.md#form-comparison |
| form.albatross_3m.u10_min_loiter_knee | 10.44 | m/s | calculated | docs/02-form.md#form-comparison |
| form.albatross_3m.cross12_speed_knee | 17.1 | m/s | calculated | docs/02-form.md#form-comparison |
| form.flyingwing_2m.u10_min_travel_log | 14.15 | m/s | calculated | docs/02-form.md#form-comparison |
| form.flyingwing_2m.u10_min_loiter_log | 15.1 | m/s | calculated | docs/02-form.md#form-comparison |
| form.flyingwing_2m.u10_min_travel_knee | 13.83 | m/s | calculated | docs/02-form.md#form-comparison |
| form.flyingwing_2m.u10_min_loiter_knee | 14.56 | m/s | calculated | docs/02-form.md#form-comparison |
| form.sealedhull_2p5m.u10_min_travel_log | 11.14 | m/s | calculated | docs/02-form.md#form-comparison |
| form.sealedhull_2p5m.u10_min_loiter_log | 11.51 | m/s | calculated | docs/02-form.md#form-comparison |
| form.sealedhull_2p5m.u10_min_travel_knee | 11.35 | m/s | calculated | docs/02-form.md#form-comparison |
| form.sealedhull_2p5m.u10_min_loiter_knee | 12.21 | m/s | calculated | docs/02-form.md#form-comparison |
| form.baseline_2m.cross12_speed_knee | 12.67 | m/s | calculated | docs/02-form.md#form-comparison |
| form.sealedhull_2p5m.cross12_speed_log | 12.47 | m/s | calculated | docs/02-form.md#form-comparison |
| form.sealedhull_2p5m.cross12_speed_knee | 12.21 | m/s | calculated | docs/02-form.md#form-comparison |
| dsopt.flat_tip_mid_min_baseline_m | 0.268 | m | calculated | docs/02-form.md#form-comparison |
| dsopt.flat_tip_mid_min_albatross_m | 0.255 | m | calculated | docs/02-form.md#form-comparison |
| dsopt.sachs_candidate_settings_u10_ms | 9.85 | m/s | calculated | docs/02-form.md#form-comparison |
| dsopt.albatross3m_ld20_u10_ms | 10.62 | m/s | calculated | docs/02-form.md#form-comparison |
| dsopt.z003_over_charnock_ratio | 660 | 1 | calculated | docs/02-form.md#form-comparison |
| mpc.first_pass_survival_max_s | 124 | s | calculated | docs/02-form.md#receding-horizon |
| mpc.library_cycles | 24 | 1 | calculated | docs/02-form.md#receding-horizon |
| mpc.flat_survival_min_s | 120 | s | calculated | docs/02-form.md#receding-horizon |
| mpc.flat_survival_max_s | 162 | s | calculated | docs/02-form.md#receding-horizon |
| mpc.wave_survival_min_s | 6 | s | calculated | docs/02-form.md#receding-horizon |
| mpc.wave_survival_max_s | 145 | s | calculated | docs/02-form.md#receding-horizon |
| mpc.phasing_worse_pairs | 4 | 1 | calculated | docs/02-form.md#receding-horizon |
| mpc.phasing_pairs | 4 | 1 | calculated | docs/02-form.md#receding-horizon |
| mpc.fit_surface_error_min_m | 1.1 | m | calculated | docs/02-form.md#receding-horizon |
| mpc.fit_surface_error_max_m | 1.61 | m | calculated | docs/02-form.md#receding-horizon |
| mpc.sea_std_u13_m | 1.3 | m | calculated | docs/02-form.md#receding-horizon |
| dsopt.validation_no_tip_u10_ms | 8.566 | m/s | calculated | docs/02-form.md#form-comparison |
| form.baseline_2m.u10_min_travel_log_z003 | 5.86 | m/s | calculated | docs/02-form.md#form-comparison |
| form.baseline_2m.u10_min_loiter_log_z003 | 6.11 | m/s | calculated | docs/02-form.md#form-comparison |
| form.baseline_2m.cross12_speed_log_z003 | 20.84 | m/s | calculated | docs/02-form.md#form-comparison |
| form.baseline_2m.u10_min_travel_knee_z003 | 5.87 | m/s | calculated | docs/02-form.md#form-comparison |
| form.baseline_2m.u10_min_loiter_knee_z003 | 6.38 | m/s | calculated | docs/02-form.md#form-comparison |
| form.baseline_2m.cross12_speed_knee_z003 | 20.63 | m/s | calculated | docs/02-form.md#form-comparison |
| form.demo_1p5m.u10_min_travel_log_z003 | 7.57 | m/s | calculated | docs/02-form.md#form-comparison |
| form.demo_1p5m.u10_min_loiter_log_z003 | 7.86 | m/s | calculated | docs/02-form.md#form-comparison |
| form.demo_1p5m.cross12_speed_log_z003 | 17.41 | m/s | calculated | docs/02-form.md#form-comparison |
| form.demo_1p5m.u10_min_travel_knee_z003 | 7.62 | m/s | calculated | docs/02-form.md#form-comparison |
| form.demo_1p5m.u10_min_loiter_knee_z003 | 8.2 | m/s | calculated | docs/02-form.md#form-comparison |
| form.demo_1p5m.cross12_speed_knee_z003 | 17.45 | m/s | calculated | docs/02-form.md#form-comparison |
| form.easyglider_1p8m.u10_min_travel_log_z003 | 11.43 | m/s | calculated | docs/02-form.md#form-comparison |
| form.easyglider_1p8m.u10_min_loiter_log_z003 | 11.89 | m/s | calculated | docs/02-form.md#form-comparison |
| form.easyglider_1p8m.cross12_speed_log_z003 | 8.52 | m/s | calculated | docs/02-form.md#form-comparison |
| form.easyglider_1p8m.u10_min_travel_knee_z003 | 11.0 | m/s | calculated | docs/02-form.md#form-comparison |
| form.easyglider_1p8m.u10_min_loiter_knee_z003 | 11.56 | m/s | calculated | docs/02-form.md#form-comparison |
| form.easyglider_1p8m.cross12_speed_knee_z003 | 9.12 | m/s | calculated | docs/02-form.md#form-comparison |
| form.albatross_3m.u10_min_travel_log_z003 | 4.76 | m/s | calculated | docs/02-form.md#form-comparison |
| form.albatross_3m.u10_min_loiter_log_z003 | 4.98 | m/s | calculated | docs/02-form.md#form-comparison |
| form.albatross_3m.cross12_speed_log_z003 | 26.1 | m/s | calculated | docs/02-form.md#form-comparison |
| form.albatross_3m.u10_min_travel_knee_z003 | 4.92 | m/s | calculated | docs/02-form.md#form-comparison |
| form.albatross_3m.u10_min_loiter_knee_z003 | 5.54 | m/s | calculated | docs/02-form.md#form-comparison |
| form.albatross_3m.cross12_speed_knee_z003 | 24.97 | m/s | calculated | docs/02-form.md#form-comparison |
| form.flyingwing_2m.u10_min_travel_log_z003 | 8.26 | m/s | calculated | docs/02-form.md#form-comparison |
| form.flyingwing_2m.u10_min_loiter_log_z003 | 9.04 | m/s | calculated | docs/02-form.md#form-comparison |
| form.flyingwing_2m.cross12_speed_log_z003 | 13.49 | m/s | calculated | docs/02-form.md#form-comparison |
| form.flyingwing_2m.u10_min_travel_knee_z003 | 7.86 | m/s | calculated | docs/02-form.md#form-comparison |
| form.flyingwing_2m.u10_min_loiter_knee_z003 | 8.38 | m/s | calculated | docs/02-form.md#form-comparison |
| form.flyingwing_2m.cross12_speed_knee_z003 | 13.83 | m/s | calculated | docs/02-form.md#form-comparison |
| form.sealedhull_2p5m.u10_min_travel_log_z003 | 6.14 | m/s | calculated | docs/02-form.md#form-comparison |
| form.sealedhull_2p5m.u10_min_loiter_log_z003 | 6.4 | m/s | calculated | docs/02-form.md#form-comparison |
| form.sealedhull_2p5m.cross12_speed_log_z003 | 20.29 | m/s | calculated | docs/02-form.md#form-comparison |
| form.sealedhull_2p5m.u10_min_travel_knee_z003 | 6.15 | m/s | calculated | docs/02-form.md#form-comparison |
| form.sealedhull_2p5m.u10_min_loiter_knee_z003 | 6.73 | m/s | calculated | docs/02-form.md#form-comparison |
| form.sealedhull_2p5m.cross12_speed_knee_z003 | 19.98 | m/s | calculated | docs/02-form.md#form-comparison |
| form.baseline_2m.u10_min_travel_log_coare35 | 10.8 | m/s | calculated | docs/02-form.md#form-comparison |
| form.baseline_2m.u10_min_loiter_log_coare35 | 11.13 | m/s | calculated | docs/02-form.md#form-comparison |
| form.baseline_2m.cross12_speed_log_coare35 | 13.08 | m/s | calculated | docs/02-form.md#form-comparison |
| form.baseline_2m.u10_min_travel_knee_coare35 | 10.99 | m/s | calculated | docs/02-form.md#form-comparison |
| form.baseline_2m.u10_min_loiter_knee_coare35 | 11.65 | m/s | calculated | docs/02-form.md#form-comparison |
| form.baseline_2m.cross12_speed_knee_coare35 | 12.69 | m/s | calculated | docs/02-form.md#form-comparison |
| form.demo_1p5m.u10_min_travel_log_coare35 | 12.97 | m/s | calculated | docs/02-form.md#form-comparison |
| form.demo_1p5m.u10_min_loiter_log_coare35 | 13.32 | m/s | calculated | docs/02-form.md#form-comparison |
| form.demo_1p5m.u10_min_travel_knee_coare35 | 13.24 | m/s | calculated | docs/02-form.md#form-comparison |
| form.demo_1p5m.u10_min_loiter_knee_coare35 | 13.93 | m/s | calculated | docs/02-form.md#form-comparison |
| form.easyglider_1p8m.u10_min_travel_log_coare35 | 17.28 | m/s | calculated | docs/02-form.md#form-comparison |
| form.easyglider_1p8m.u10_min_loiter_log_coare35 | 17.74 | m/s | calculated | docs/02-form.md#form-comparison |
| form.easyglider_1p8m.u10_min_travel_knee_coare35 | 17.03 | m/s | calculated | docs/02-form.md#form-comparison |
| form.easyglider_1p8m.u10_min_loiter_knee_coare35 | 17.61 | m/s | calculated | docs/02-form.md#form-comparison |
| form.albatross_3m.u10_min_travel_log_coare35 | 9.26 | m/s | calculated | docs/02-form.md#form-comparison |
| form.albatross_3m.u10_min_loiter_log_coare35 | 9.58 | m/s | calculated | docs/02-form.md#form-comparison |
| form.albatross_3m.cross12_speed_log_coare35 | 18.06 | m/s | calculated | docs/02-form.md#form-comparison |
| form.albatross_3m.u10_min_travel_knee_coare35 | 9.69 | m/s | calculated | docs/02-form.md#form-comparison |
| form.albatross_3m.u10_min_loiter_knee_coare35 | 10.54 | m/s | calculated | docs/02-form.md#form-comparison |
| form.albatross_3m.cross12_speed_knee_coare35 | 17.12 | m/s | calculated | docs/02-form.md#form-comparison |
| form.flyingwing_2m.u10_min_travel_log_coare35 | 13.8 | m/s | calculated | docs/02-form.md#form-comparison |
| form.flyingwing_2m.u10_min_loiter_log_coare35 | 14.61 | m/s | calculated | docs/02-form.md#form-comparison |
| form.flyingwing_2m.u10_min_travel_knee_coare35 | 13.52 | m/s | calculated | docs/02-form.md#form-comparison |
| form.flyingwing_2m.u10_min_loiter_knee_coare35 | 14.14 | m/s | calculated | docs/02-form.md#form-comparison |
| form.sealedhull_2p5m.u10_min_travel_log_coare35 | 11.17 | m/s | calculated | docs/02-form.md#form-comparison |
| form.sealedhull_2p5m.u10_min_loiter_log_coare35 | 11.5 | m/s | calculated | docs/02-form.md#form-comparison |
| form.sealedhull_2p5m.cross12_speed_log_coare35 | 12.61 | m/s | calculated | docs/02-form.md#form-comparison |
| form.sealedhull_2p5m.u10_min_travel_knee_coare35 | 11.36 | m/s | calculated | docs/02-form.md#form-comparison |
| form.sealedhull_2p5m.u10_min_loiter_knee_coare35 | 12.11 | m/s | calculated | docs/02-form.md#form-comparison |
| form.sealedhull_2p5m.cross12_speed_knee_coare35 | 12.23 | m/s | calculated | docs/02-form.md#form-comparison |
| dsopt.validation_u10_ms | 9.17 | m/s | calculated | docs/02-form.md#form-comparison |
| dsopt.wave_cases | 36 | 1 | calculated | docs/02-form.md#wave-cycles |
| dsopt.wave_floor_cases | 17 | 1 | calculated | docs/02-form.md#wave-cycles |
| dsopt.wave_steady_ride_cases | 24 | 1 | calculated | docs/02-form.md#wave-cycles |
| dsopt.wave_vertical_ratio_model | 0.95 | 1 | calculated | docs/02-form.md#wave-cycles |
| dsopt.wave_vertical_ratio_measured | 0.6 | 1 | calculated | docs/02-form.md#wave-cycles |
| dsopt.wave_model_w_at_p64_ms | 0.09 | m/s | calculated | docs/02-form.md#wave-cycles |
| dsopt.wave_scaled_u10_baseline_tp8_hs2_ms | 3.73 | m/s | calculated | docs/02-form.md#wave-cycles |
| dsopt.wave_scaled_u10_albatross_tp6_hs1_ms | 2.95 | m/s | calculated | docs/02-form.md#wave-cycles |
| dsopt.measured_decay_max_shift_ms | 0.45 | m/s | calculated | docs/02-form.md#wave-cycles |
| dsopt.shelter_albatross_l20_off_ms | 8.34 | m/s | calculated | docs/02-form.md#wave-cycles |
| dsopt.shelter_albatross_l20_on_ms | 6.83 | m/s | calculated | docs/02-form.md#wave-cycles |
| dsopt.wave_cycle_floquet_max | 2.45 | 1 | calculated | docs/02-form.md#wave-cycles |
| ds.day_sight_range_m | 500 | m | assumed | Q-045 |
| ds.light_range_m | 1000 | m | assumed | Q-045 |
| ds.radar_detect_range_m | 500 | m | assumed | Q-045 |
| ds.land_standoff_m | 1000 | m | assumed | Q-045 |
| ds.fix_loss_timeout_s | 10 | s | assumed | Q-045 |
| ds.termination_time_s | 30 | s | assumed | Q-045 |
| ds.geofence_margin_m | 100 | m | assumed | Q-045 |
| ds.label_read_distance_m | 1 | m | assumed | Q-045 |
| ds.recovery_response_h | 72 | h | assumed | Q-045 |
| ds.position_report_interval_h | 6 | h | assumed | Q-045 |
| ds.eol_capacity_fraction | 0.7 | 1 | assumed | Q-045 |
| ds.cell_charge_temp_min_c | 0 | degC | assumed | Q-045 |
| ds.cell_charge_temp_max_c | 45 | degC | assumed | Q-045 |
| ds.storage_soc_fraction | 0.3 | 1 | assumed | Q-045 |
| ds.launch_clear_radius_m | 10 | m | assumed | Q-045 |
| ds.launch_clear_distance_m | 50 | m | assumed | Q-045 |
| ds.motor_arm_delay_s | 1 | s | assumed | Q-045 |
| ds.recovery_approach_distance_m | 30 | m | assumed | Q-045 |
| ds.workshop_air_changes_per_h | 6 | 1/h | assumed | Q-045 |
| ds.vessel_standoff_m | 500 | m | assumed | Q-045 |
| autonomy.test_u10_ms | 13 | m/s | decided | D-010 |
| autonomy.flat_standard_cycles | 18 | count | calculated | docs/02-form.md#autonomy-check |
| autonomy.flat_standard_ground_speed_ms | 21.1 | m/s | calculated | docs/02-form.md#autonomy-check |
| autonomy.flat_truth_ground_speed_ms | 21.9 | m/s | calculated | docs/02-form.md#autonomy-check |
| autonomy.flat_standard_wind_rms_ms | 1.38 | m/s | calculated | docs/02-form.md#autonomy-check |
| autonomy.minimal_wind_rms_ms | 7.68 | m/s | calculated | docs/02-form.md#autonomy-check |
| autonomy.waves_truth_crash_s | 73 | s | calculated | docs/02-form.md#autonomy-check |
| autonomy.max_load_factor_waves | 7.34 | 1 | calculated | docs/02-form.md#autonomy-check |
| autonomy.guard_floor_m | 0.5 | m | assumed | Q-051 |
<!-- tdr:end -->

## Requirements for the dynamic soaring drone {#ds-requirements}

The owner moved the project from a certifiable drone to a drone that stays over the sea for a year with dynamic soaring as its core (D-008), that goes on its own to a given place and sends photos or data from there (D-009), and asked for the computation environment first (D-010).
Q-007 says which rules apply to such a flight.

The questions that rule a form out were asked before; the decisions recorded so far are D-001, D-005 (budget.total), D-006 and D-009, and the questions still open are Q-027.
The place of flight at sea and its climate are private facts and enter the record only as derived values.

R-01 and R-02 are the owner's core; R-03 and R-04 are the mission (D-009).
R-05 and R-06 follow from a year at sea: the wind will fail, so the drone must live on the water, and the sea will get in unless it is sealed.
R-07 follows from one person and the budget; R-08 from a year without a charger; R-09 and R-10 from the loads the environment imposes; R-11 from the law; R-12 from where the shear is.
The design points name their values by key, the status and basis of each value are in the table above, and the owner confirmed every requirement on 2026-10-29 (D-011).

<!-- tdr:table kind=requirement fields=id,text,design_point,verify,status -->
| id | text | design_point | verify | status |
|---|---|---|---|---|
| R-01 | The drone stays over the sea for a year without a person touching it. | Airborne or afloat for ds.endurance_days without intervention. | demonstration | confirmed |
| R-02 | The energy for flight comes from the wind shear over the waves (dynamic soaring), with no fuel and no propeller for cruise. | Sustained flight with no net energy loss at U10 >= ds.min_u10_ms over a neutral log profile. | test | confirmed |
| R-03 | The drone goes on its own to a position it is given and comes back. | Reaches a waypoint ds.range_km from launch within 1 km and returns. | demonstration | confirmed |
| R-04 | The drone sends a photo or a data record from the position. | An image of ds.photo_bytes is received on shore within one hour of capture. | demonstration | confirmed |
| R-05 | When the wind is too weak to soar, the drone lands on the water, stays sealed, rights itself, and takes off again when the wind returns. | Survives afloat at significant wave height ds.survival_hs_m and takes off from the water at U10 >= ds.min_u10_ms. | test | confirmed |
| R-06 | No sea water reaches the electronics in a year. | No ingress after immersion to 1 m for 24 h and after a year of spray. | test | confirmed |
| R-07 | One person can carry and launch the drone by hand. | Mass <= ds.mass_max_kg and span <= ds.span_max_m. | inspection | confirmed |
| R-08 | The avionics run for a year on solar power and a battery. | Average avionics draw <= ds.avionics_power_w and the solar yield on the darkest design day covers it with the battery bridging the night. | analysis | confirmed |
| R-09 | The airframe carries the loads of soaring turns and gusts. | No failure at ds.n_limit times ds.safety_factor; no permanent deformation at ds.n_limit. | analysis | confirmed |
| R-10 | The drone survives touching down on the water. | No damage after a stall-speed touchdown at significant wave height 2 m. | test | confirmed |
| R-11 | The flight is legal where it flies. | The rules of Q-007 are identified and met before the first flight beyond visual line of sight. | inspection | confirmed |
| R-12 | The drone flies in the layer where the shear is, and never below a safe height over the waves. | Operates between ds.clearance_min_m and ds.altitude_band_max_m above the local sea surface. | analysis | confirmed |
| R-13 | Vessels, small craft and aircraft can see the drone by day and by night, in flight and afloat. | The fin, the hull sides, the wing tips and the underside of the wing are fluorescent orange and are seen by eye from a small boat at ds.day_sight_range_m in daylight, and an all-round flashing light is seen at ds.light_range_m at night, in flight and afloat. | test | confirmed |
| R-14 | A vessel's radar shows the drone afloat. | A small-craft X-band marine radar detects the drone afloat and upright at ds.radar_detect_range_m. | test | confirmed |
| R-15 | The drone stays inside the operating area it is given, and ends any flight on the water when it cannot, without needing the link. | The operating area lies at least ds.land_standoff_m off the coast outside the launch and recovery corridor and leaves out the surroundings of airports and heliports, ports, fairways, fixed net zones, and bathing and surfing areas, and in hardware-in-the-loop and flight tests a boundary breach, a position fix lost for ds.fix_loss_timeout_s, a fault that enters Safe, and a termination command each put the drone on the water within ds.termination_time_s without passing more than ds.geofence_margin_m beyond the boundary. | test | confirmed |
| R-16 | A person who finds the drone, afloat or ashore, whole or broken, can tell what it is, that it holds lithium cells, and whom to call. | Labels on the hull and on the cell case give the owner's contact, the registration mark, a lithium cell warning and a request not to open it, in Japanese and English, and stay legible at ds.label_read_distance_m after ds.endurance_days of sea and sun. | test | confirmed |
| R-17 | Every drone is recovered at the end of its mission or when it fails, and no part of it is left at sea or ashore by plan. | Before each launch a recovery plan names the recovery point, the boat and its skipper, and a response within ds.recovery_response_h of a recovery call, the drone reports its position aloft or afloat at least every ds.position_report_interval_h until it is recovered, it flies to the recovery point at the end of the mission or when its battery capacity falls below ds.eol_capacity_fraction of the rating, and its cells go to a collection route for lithium ion cells and its composite to industrial waste. | demonstration | confirmed |
| R-18 | Every radio on the drone and on the ground is lawful for its band and its place of use, and none interferes with maritime, aviation or satellite radio. | Each transmitter carries the Japanese technical conformity mark or a station licence for its band and for use aloft (P34), the satellite terminal holds its service authorisation, and no transmitter emits in a maritime, aeronautical or radionavigation band. | inspection | confirmed |
| R-19 | The lithium cells cannot start a fire or release gas in flight, afloat, on charge, in storage or in shipping, and stay inside the drone if it breaks up. | The cells sit in a sealed case of their own with a water sensor, fixed to the hull and apart from the avionics, the protection circuit stops charging outside ds.cell_charge_temp_min_c to ds.cell_charge_temp_max_c, on overvoltage, on overcurrent or when the sensor finds water and disconnects the cells on a short circuit, the drone is charged on the ground only on a non-combustible surface under watch and stored at ds.storage_soc_fraction of full charge in a fire-resistant container, and the pack and its shipping meet the lithium ion battery shipping rules identified before the first shipment. | test | confirmed |
| R-20 | The hand launch, the landing for recovery and the lift from the water keep everyone but the launcher and the crew clear of the drone, and the motor cannot run in a person's hands. | The launch and recovery procedure keeps everyone but the launcher at least ds.launch_clear_radius_m from the launcher and keeps people, vessels and structures out of ds.launch_clear_distance_m ahead in the launch direction, the motor starts only ds.motor_arm_delay_s after the drone detects its release, and for recovery the drone lands at least ds.recovery_approach_distance_m from the boat and reports its motor disarmed before the boat comes closer, with the crew in life jackets and cut-resistant gloves. | demonstration | confirmed |
| R-21 | Building, repairing and cutting up the drone does not harm the builder or anyone in the workshop. | Every material is used as its safety data sheet says, carbon and glass are cut and sanded wet or under extraction with a P2 respirator and eye protection, epoxy is handled with nitrile gloves, hot-wire cutting of foam runs under local extraction with no visible smoke and is never left powered unattended, and the workshop gets at least ds.workshop_air_changes_per_h air changes while resin cures, foam is cut or parts are soldered. | inspection | confirmed |
| R-22 | The drone keeps clear of the vessels it can detect, in flight and when it takes off from the water. | With recorded AIS traffic replayed into the flight controller, the drone keeps at least ds.vessel_standoff_m horizontally from every vessel it receives, and never starts a take-off from the water within ds.vessel_standoff_m of one. | test | confirmed |
<!-- tdr:end -->
