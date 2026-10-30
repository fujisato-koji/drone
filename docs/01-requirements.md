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
| lit.bird_experienced_wind_sd_ms | 3.4 | m/s | sourced | P73 |
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
| nav.albatross_upwind_support_max_ms | -2.49 | m/s | calculated | docs/02-form.md#navigation |
| nav.albatross_upwind_support_min_ms | -2.99 | m/s | calculated | docs/02-form.md#navigation |
| nav.baseline_upwind_support_max_ms | -6.14 | m/s | calculated | docs/02-form.md#navigation |
| nav.baseline_upwind_support_min_ms | -7.14 | m/s | calculated | docs/02-form.md#navigation |
| nav.sealedhull_upwind_support_max_ms | -6.21 | m/s | calculated | docs/02-form.md#navigation |
| nav.sealedhull_upwind_support_min_ms | -7.18 | m/s | calculated | docs/02-form.md#navigation |
| nav.albatross_hull_min_downwind_u16_ms | 4.25 | m/s | calculated | docs/02-form.md#navigation |
| nav.baseline_hull_min_downwind_u16_ms | 9.21 | m/s | calculated | docs/02-form.md#navigation |
| nav.sealedhull_hull_min_downwind_u16_ms | 9.23 | m/s | calculated | docs/02-form.md#navigation |
| nav.albatross_soaring_sector_u16_deg | 74 | deg | calculated | docs/02-form.md#navigation |
| nav.baseline_soaring_sector_u16_deg | 54 | deg | calculated | docs/02-form.md#navigation |
| nav.sealedhull_soaring_sector_u16_deg | 54 | deg | calculated | docs/02-form.md#navigation |
| nav.albatross_motor_upwind_u16_ms | 6.02 | m/s | calculated | docs/02-form.md#navigation |
| nav.albatross_motor_upwind_u16_j_per_m | 11.49 | J/m | calculated | docs/02-form.md#navigation |
| nav.albatross_steady_upwind_u16_ms | 8.91 | m/s | calculated | docs/02-form.md#navigation |
| nav.albatross_steady_upwind_u16_j_per_m | 10.76 | J/m | calculated | docs/02-form.md#navigation |
| nav.albatross_steady_across_u16_ms | 12.67 | m/s | calculated | docs/02-form.md#navigation |
| nav.albatross_steady_across_u16_j_per_m | 4.42 | J/m | calculated | docs/02-form.md#navigation |
| nav.albatross_mix_across_u16_j_per_m | 3.23 | J/m | calculated | docs/02-form.md#navigation |
| nav.albatross_u16_downwind_clipped_motor_on_share | 0.99 | 1 | calculated | docs/02-form.md#navigation |
| nav.albatross_u16_downwind_clipped_avg_power_w | 27.8 | W | calculated | docs/02-form.md#navigation |
| nav.thrust_bias_clipped_basic_pct | 2.67 | percent | calculated | docs/02-form.md#navigation |
| nav.thrust_bias_raw_basic_signed_pct | -0.08 | percent | calculated | docs/02-form.md#navigation |
| nav.thrust_bias_raw_basic_applied_pct | 1.9 | percent | calculated | docs/02-form.md#navigation |
| nav.thrust_bias_clipped_sustained_pct | 3.87 | percent | calculated | docs/02-form.md#navigation |
| nav.thrust_bias_raw_sustained_signed_pct | 0.82 | percent | calculated | docs/02-form.md#navigation |
| nav.thrust_bias_raw_sustained_applied_pct | 2.41 | percent | calculated | docs/02-form.md#navigation |
| nav.polar_albatross_u16_c120_progress_ms | 12.7 | m/s | calculated | docs/02-form.md#navigation |
| nav.polar_albatross_u16_c120_ground_ms | 21.6 | m/s | calculated | docs/02-form.md#navigation |
| nav.polar_albatross_u16_c120_drift_deg | 54 | deg | calculated | docs/02-form.md#navigation |
| nav.first_run_height_median_min_m | 16 | m | calculated | docs/02-form.md#navigation |
| nav.first_run_height_median_max_m | 35 | m | calculated | docs/02-form.md#navigation |
| nav.first_run_baseline_contact_s | 153 | s | calculated | docs/02-form.md#navigation |
| nav.run5_albatross_u13_upwind_arrival_s | 588.0 | s | calculated | docs/02-form.md#navigation |
| nav.run5_albatross_u13_upwind_counted | 1 | 1 | calculated | docs/02-form.md#navigation |
| nav.run5_albatross_u13_upwind_vmg_ms | 8.16 | m/s | calculated | docs/02-form.md#navigation |
| nav.run5_albatross_u13_upwind_hull_ms | 8.16 | m/s | calculated | docs/02-form.md#navigation |
| nav.run5_albatross_u13_upwind_hull_s | 589.0 | s | calculated | docs/02-form.md#navigation |
| nav.run5_albatross_u13_upwind_energy_wh | 9.9 | Wh | calculated | docs/02-form.md#navigation |
| nav.run5_albatross_u13_upwind_motor_share | 1.0 | 1 | calculated | docs/02-form.md#navigation |
| nav.run5_albatross_u13_upwind_height_median_m | 9.1 | m | calculated | docs/02-form.md#navigation |
| nav.run5_albatross_u13_upwind_height_max_m | 11.6 | m | calculated | docs/02-form.md#navigation |
| nav.run5_albatross_u13_across_arrival_s | 419.0 | s | calculated | docs/02-form.md#navigation |
| nav.run5_albatross_u13_across_counted | 1 | 1 | calculated | docs/02-form.md#navigation |
| nav.run5_albatross_u13_across_vmg_ms | 11.45 | m/s | calculated | docs/02-form.md#navigation |
| nav.run5_albatross_u13_across_hull_ms | 11.58 | m/s | calculated | docs/02-form.md#navigation |
| nav.run5_albatross_u13_across_hull_s | 415.0 | s | calculated | docs/02-form.md#navigation |
| nav.run5_albatross_u13_across_energy_wh | 4.4 | Wh | calculated | docs/02-form.md#navigation |
| nav.run5_albatross_u13_across_motor_share | 1.0 | 1 | calculated | docs/02-form.md#navigation |
| nav.run5_albatross_u13_across_height_median_m | 9.9 | m | calculated | docs/02-form.md#navigation |
| nav.run5_albatross_u13_across_height_max_m | 12.3 | m | calculated | docs/02-form.md#navigation |
| nav.run5_albatross_u16_upwind_arrival_s | 571.0 | s | calculated | docs/02-form.md#navigation |
| nav.run5_albatross_u16_upwind_counted | 1 | 1 | calculated | docs/02-form.md#navigation |
| nav.run5_albatross_u16_upwind_vmg_ms | 8.41 | m/s | calculated | docs/02-form.md#navigation |
| nav.run5_albatross_u16_upwind_hull_ms | 8.91 | m/s | calculated | docs/02-form.md#navigation |
| nav.run5_albatross_u16_upwind_hull_s | 539.0 | s | calculated | docs/02-form.md#navigation |
| nav.run5_albatross_u16_upwind_energy_wh | 14.7 | Wh | calculated | docs/02-form.md#navigation |
| nav.run5_albatross_u16_upwind_motor_share | 1.0 | 1 | calculated | docs/02-form.md#navigation |
| nav.run5_albatross_u16_upwind_height_median_m | 10.7 | m | calculated | docs/02-form.md#navigation |
| nav.run5_albatross_u16_upwind_height_max_m | 14.3 | m | calculated | docs/02-form.md#navigation |
| nav.run5_albatross_u16_across_arrival_s | 370.0 | s | calculated | docs/02-form.md#navigation |
| nav.run5_albatross_u16_across_counted | 1 | 1 | calculated | docs/02-form.md#navigation |
| nav.run5_albatross_u16_across_vmg_ms | 12.96 | m/s | calculated | docs/02-form.md#navigation |
| nav.run5_albatross_u16_across_hull_ms | 12.67 | m/s | calculated | docs/02-form.md#navigation |
| nav.run5_albatross_u16_across_hull_s | 379.0 | s | calculated | docs/02-form.md#navigation |
| nav.run5_albatross_u16_across_energy_wh | 5.8 | Wh | calculated | docs/02-form.md#navigation |
| nav.run5_albatross_u16_across_motor_share | 1.0 | 1 | calculated | docs/02-form.md#navigation |
| nav.run5_albatross_u16_across_height_median_m | 9.1 | m | calculated | docs/02-form.md#navigation |
| nav.run5_albatross_u16_across_height_max_m | 12.1 | m | calculated | docs/02-form.md#navigation |
| nav.run5_albatross_u16_downwind_arrival_s | 179.0 | s | calculated | docs/02-form.md#navigation |
| nav.run5_albatross_u16_downwind_counted | 1 | 1 | calculated | docs/02-form.md#navigation |
| nav.run5_albatross_u16_downwind_vmg_ms | 26.88 | m/s | calculated | docs/02-form.md#navigation |
| nav.run5_albatross_u16_downwind_hull_ms | 28.91 | m/s | calculated | docs/02-form.md#navigation |
| nav.run5_albatross_u16_downwind_hull_s | 199.0 | s | calculated | docs/02-form.md#navigation |
| nav.run5_albatross_u16_downwind_energy_wh | 0.2 | Wh | calculated | docs/02-form.md#navigation |
| nav.run5_albatross_u16_downwind_motor_share | 0.05 | 1 | calculated | docs/02-form.md#navigation |
| nav.run5_albatross_u16_downwind_height_median_m | 12.9 | m | calculated | docs/02-form.md#navigation |
| nav.run5_albatross_u16_downwind_height_max_m | 29.8 | m | calculated | docs/02-form.md#navigation |
| nav.run5_baseline_u16_upwind_arrival_s | 604.0 | s | calculated | docs/02-form.md#navigation |
| nav.run5_baseline_u16_upwind_counted | 1 | 1 | calculated | docs/02-form.md#navigation |
| nav.run5_baseline_u16_upwind_vmg_ms | 7.95 | m/s | calculated | docs/02-form.md#navigation |
| nav.run5_baseline_u16_upwind_hull_ms | 8.74 | m/s | calculated | docs/02-form.md#navigation |
| nav.run5_baseline_u16_upwind_hull_s | 550.0 | s | calculated | docs/02-form.md#navigation |
| nav.run5_baseline_u16_upwind_energy_wh | 12.9 | Wh | calculated | docs/02-form.md#navigation |
| nav.run5_baseline_u16_upwind_motor_share | 1.0 | 1 | calculated | docs/02-form.md#navigation |
| nav.run5_baseline_u16_upwind_height_median_m | 12.1 | m | calculated | docs/02-form.md#navigation |
| nav.run5_baseline_u16_upwind_height_max_m | 16.3 | m | calculated | docs/02-form.md#navigation |
| nav.run5_baseline_u16_across_arrival_s | 391.0 | s | calculated | docs/02-form.md#navigation |
| nav.run5_baseline_u16_across_counted | 1 | 1 | calculated | docs/02-form.md#navigation |
| nav.run5_baseline_u16_across_vmg_ms | 12.29 | m/s | calculated | docs/02-form.md#navigation |
| nav.run5_baseline_u16_across_hull_ms | 12.39 | m/s | calculated | docs/02-form.md#navigation |
| nav.run5_baseline_u16_across_hull_s | 388.0 | s | calculated | docs/02-form.md#navigation |
| nav.run5_baseline_u16_across_energy_wh | 5.0 | Wh | calculated | docs/02-form.md#navigation |
| nav.run5_baseline_u16_across_motor_share | 1.0 | 1 | calculated | docs/02-form.md#navigation |
| nav.run5_baseline_u16_across_height_median_m | 11.0 | m | calculated | docs/02-form.md#navigation |
| nav.run5_baseline_u16_across_height_max_m | 13.8 | m | calculated | docs/02-form.md#navigation |
| nav.run5_albatross_u9_upwind_arrival_s | 662.0 | s | calculated | docs/02-form.md#navigation |
| nav.run5_albatross_u9_upwind_counted | 1 | 1 | calculated | docs/02-form.md#navigation |
| nav.run5_albatross_u9_upwind_vmg_ms | 7.25 | m/s | calculated | docs/02-form.md#navigation |
| nav.run5_albatross_u9_upwind_hull_ms | 7.66 | m/s | calculated | docs/02-form.md#navigation |
| nav.run5_albatross_u9_upwind_hull_s | 627.0 | s | calculated | docs/02-form.md#navigation |
| nav.run5_albatross_u9_upwind_energy_wh | 6.0 | Wh | calculated | docs/02-form.md#navigation |
| nav.run5_albatross_u9_upwind_motor_share | 1.0 | 1 | calculated | docs/02-form.md#navigation |
| nav.run5_albatross_u9_upwind_height_median_m | 9.8 | m | calculated | docs/02-form.md#navigation |
| nav.run5_albatross_u9_upwind_height_max_m | 11.4 | m | calculated | docs/02-form.md#navigation |
| nav.run10_albatross_u13_upwind_arrival_s | 1204.0 | s | calculated | docs/02-form.md#navigation |
| nav.run10_albatross_u13_upwind_counted | 1 | 1 | calculated | docs/02-form.md#navigation |
| nav.run10_albatross_u13_upwind_vmg_ms | 8.14 | m/s | calculated | docs/02-form.md#navigation |
| nav.run10_albatross_u13_upwind_hull_ms | 8.16 | m/s | calculated | docs/02-form.md#navigation |
| nav.run10_albatross_u13_upwind_hull_s | 1202.0 | s | calculated | docs/02-form.md#navigation |
| nav.run10_albatross_u13_upwind_energy_wh | 20.2 | Wh | calculated | docs/02-form.md#navigation |
| nav.run10_albatross_u13_upwind_motor_share | 1.0 | 1 | calculated | docs/02-form.md#navigation |
| nav.run10_albatross_u13_upwind_height_median_m | 9.0 | m | calculated | docs/02-form.md#navigation |
| nav.run10_albatross_u13_upwind_height_max_m | 11.6 | m | calculated | docs/02-form.md#navigation |
| nav.run10_albatross_u13_across_arrival_s | 872.0 | s | calculated | docs/02-form.md#navigation |
| nav.run10_albatross_u13_across_counted | 1 | 1 | calculated | docs/02-form.md#navigation |
| nav.run10_albatross_u13_across_vmg_ms | 11.24 | m/s | calculated | docs/02-form.md#navigation |
| nav.run10_albatross_u13_across_hull_ms | 11.58 | m/s | calculated | docs/02-form.md#navigation |
| nav.run10_albatross_u13_across_hull_s | 847.0 | s | calculated | docs/02-form.md#navigation |
| nav.run10_albatross_u13_across_energy_wh | 9.0 | Wh | calculated | docs/02-form.md#navigation |
| nav.run10_albatross_u13_across_motor_share | 1.0 | 1 | calculated | docs/02-form.md#navigation |
| nav.run10_albatross_u13_across_height_median_m | 10.0 | m | calculated | docs/02-form.md#navigation |
| nav.run10_albatross_u13_across_height_max_m | 12.3 | m | calculated | docs/02-form.md#navigation |
| nav.run10_albatross_u16_upwind_arrival_s | 1159.0 | s | calculated | docs/02-form.md#navigation |
| nav.run10_albatross_u16_upwind_counted | 1 | 1 | calculated | docs/02-form.md#navigation |
| nav.run10_albatross_u16_upwind_vmg_ms | 8.46 | m/s | calculated | docs/02-form.md#navigation |
| nav.run10_albatross_u16_upwind_hull_ms | 8.91 | m/s | calculated | docs/02-form.md#navigation |
| nav.run10_albatross_u16_upwind_hull_s | 1101.0 | s | calculated | docs/02-form.md#navigation |
| nav.run10_albatross_u16_upwind_energy_wh | 29.7 | Wh | calculated | docs/02-form.md#navigation |
| nav.run10_albatross_u16_upwind_motor_share | 1.0 | 1 | calculated | docs/02-form.md#navigation |
| nav.run10_albatross_u16_upwind_height_median_m | 10.7 | m | calculated | docs/02-form.md#navigation |
| nav.run10_albatross_u16_upwind_height_max_m | 14.3 | m | calculated | docs/02-form.md#navigation |
| nav.run10_albatross_u16_across_arrival_s | 765.0 | s | calculated | docs/02-form.md#navigation |
| nav.run10_albatross_u16_across_counted | 1 | 1 | calculated | docs/02-form.md#navigation |
| nav.run10_albatross_u16_across_vmg_ms | 12.82 | m/s | calculated | docs/02-form.md#navigation |
| nav.run10_albatross_u16_across_hull_ms | 12.67 | m/s | calculated | docs/02-form.md#navigation |
| nav.run10_albatross_u16_across_hull_s | 774.0 | s | calculated | docs/02-form.md#navigation |
| nav.run10_albatross_u16_across_energy_wh | 11.8 | Wh | calculated | docs/02-form.md#navigation |
| nav.run10_albatross_u16_across_motor_share | 1.0 | 1 | calculated | docs/02-form.md#navigation |
| nav.run10_albatross_u16_across_height_median_m | 9.0 | m | calculated | docs/02-form.md#navigation |
| nav.run10_albatross_u16_across_height_max_m | 12.7 | m | calculated | docs/02-form.md#navigation |
| nav.no_progress_within_deg_min | 13.5 | deg | calculated | docs/02-form.md#navigation |
| nav.no_progress_within_deg_max | 35 | deg | calculated | docs/02-form.md#navigation |
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
| wing.front1_c9_w | 30.3 | W | calculated | docs/02-form.md#wing-sizing |
| wing.front1_wh_per_km | 0.44 | Wh/km | calculated | docs/02-form.md#wing-sizing |
| wing.front1_c9_chi180_w | 35.7 | W | calculated | docs/02-form.md#wing-sizing |
| wing.front2_ballast_kg | 1.39 | kg | calculated | docs/02-form.md#wing-sizing |
| wing.front2_mass_kg | 3.0 | kg | calculated | docs/02-form.md#wing-sizing |
| wing.front2_wing_loading_npm2 | 52.3 | N/m2 | calculated | docs/02-form.md#wing-sizing |
| wing.front2_ld_glide | 24.6 | 1 | calculated | docs/02-form.md#wing-sizing |
| wing.front2_u10_ms | 13.01 | m/s | calculated | docs/02-form.md#wing-sizing |
| wing.front2_c9_w | 28.2 | W | calculated | docs/02-form.md#wing-sizing |
| wing.front2_wh_per_km | 0.44 | Wh/km | calculated | docs/02-form.md#wing-sizing |
| wing.front2_c9_chi180_w | 35.9 | W | calculated | docs/02-form.md#wing-sizing |
| wing.front3_ballast_kg | 1.12 | kg | calculated | docs/02-form.md#wing-sizing |
| wing.front3_mass_kg | 3.0 | kg | calculated | docs/02-form.md#wing-sizing |
| wing.front3_wing_loading_npm2 | 39.2 | N/m2 | calculated | docs/02-form.md#wing-sizing |
| wing.front3_ld_glide | 23.4 | 1 | calculated | docs/02-form.md#wing-sizing |
| wing.front3_u10_ms | 13.49 | m/s | calculated | docs/02-form.md#wing-sizing |
| wing.front3_c9_w | 26.6 | W | calculated | docs/02-form.md#wing-sizing |
| wing.front3_wh_per_km | 0.45 | Wh/km | calculated | docs/02-form.md#wing-sizing |
| wing.front3_c9_chi180_w | 36.8 | W | calculated | docs/02-form.md#wing-sizing |
| wing.front4_ballast_kg | 0.5 | kg | calculated | docs/02-form.md#wing-sizing |
| wing.front4_mass_kg | 2.1 | kg | calculated | docs/02-form.md#wing-sizing |
| wing.front4_wing_loading_npm2 | 36.6 | N/m2 | calculated | docs/02-form.md#wing-sizing |
| wing.front4_ld_glide | 23.0 | 1 | calculated | docs/02-form.md#wing-sizing |
| wing.front4_u10_ms | 14.02 | m/s | calculated | docs/02-form.md#wing-sizing |
| wing.front4_c9_w | 17.5 | W | calculated | docs/02-form.md#wing-sizing |
| wing.front4_wh_per_km | 0.3 | Wh/km | calculated | docs/02-form.md#wing-sizing |
| wing.front4_c9_chi180_w | 26.8 | W | calculated | docs/02-form.md#wing-sizing |
| wing.front5_ballast_kg | 0.5 | kg | calculated | docs/02-form.md#wing-sizing |
| wing.front5_mass_kg | 1.93 | kg | calculated | docs/02-form.md#wing-sizing |
| wing.front5_wing_loading_npm2 | 42.2 | N/m2 | calculated | docs/02-form.md#wing-sizing |
| wing.front5_ld_glide | 23.1 | 1 | calculated | docs/02-form.md#wing-sizing |
| wing.front5_u10_ms | 14.04 | m/s | calculated | docs/02-form.md#wing-sizing |
| wing.front5_c9_w | 17.0 | W | calculated | docs/02-form.md#wing-sizing |
| wing.front5_wh_per_km | 0.28 | Wh/km | calculated | docs/02-form.md#wing-sizing |
| wing.front5_c9_chi180_w | 24.3 | W | calculated | docs/02-form.md#wing-sizing |
| wing.front6_ballast_kg | 0.0 | kg | calculated | docs/02-form.md#wing-sizing |
| wing.front6_mass_kg | 1.59 | kg | calculated | docs/02-form.md#wing-sizing |
| wing.front6_wing_loading_npm2 | 27.8 | N/m2 | calculated | docs/02-form.md#wing-sizing |
| wing.front6_ld_glide | 21.8 | 1 | calculated | docs/02-form.md#wing-sizing |
| wing.front6_u10_ms | 15.22 | m/s | calculated | docs/02-form.md#wing-sizing |
| wing.front6_c9_w | 12.2 | W | calculated | docs/02-form.md#wing-sizing |
| wing.front6_wh_per_km | 0.23 | Wh/km | calculated | docs/02-form.md#wing-sizing |
| wing.front6_c9_chi180_w | 22.5 | W | calculated | docs/02-form.md#wing-sizing |
| wing.front7_ballast_kg | 0.0 | kg | calculated | docs/02-form.md#wing-sizing |
| wing.front7_mass_kg | 1.43 | kg | calculated | docs/02-form.md#wing-sizing |
| wing.front7_wing_loading_npm2 | 31.1 | N/m2 | calculated | docs/02-form.md#wing-sizing |
| wing.front7_ld_glide | 21.7 | 1 | calculated | docs/02-form.md#wing-sizing |
| wing.front7_u10_ms | 15.4 | m/s | calculated | docs/02-form.md#wing-sizing |
| wing.front7_c9_w | 11.4 | W | calculated | docs/02-form.md#wing-sizing |
| wing.front7_wh_per_km | 0.21 | Wh/km | calculated | docs/02-form.md#wing-sizing |
| wing.front7_c9_chi180_w | 21.0 | W | calculated | docs/02-form.md#wing-sizing |
| wing.c9_chi20_min_w | 11.4 | W | calculated | docs/02-form.md#wing-sizing |
| wing.c9_chi20_max_w | 35.9 | W | calculated | docs/02-form.md#wing-sizing |
| wing.c9_chi180_min_w | 20.8 | W | calculated | docs/02-form.md#wing-sizing |
| wing.c9_chi180_max_w | 37.8 | W | calculated | docs/02-form.md#wing-sizing |
| wing.start_repeat_ms | 0.1 | m/s | calculated | docs/02-form.md#wing-sizing |
| wing.tail_quarter_gain_min_ms | 0.155 | m/s | calculated | docs/02-form.md#wing-sizing |
| wing.tail_quarter_gain_max_ms | 0.24 | m/s | calculated | docs/02-form.md#wing-sizing |
| wing.tail_half_gain_best_ms | 0.33 | m/s | calculated | docs/02-form.md#wing-sizing |
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
| skid.blade_thickness_m | 0.001 | m | assumed | Q-070 |
| skid.blade_rake_deg | 60 | deg | assumed | Q-070 |
| skid.blade_depth_m | 0.04 | m | assumed | Q-070 |
| skid.ski_beam_m | 0.02 | m | assumed | Q-070 |
| skid.ski_length_m | 0.06 | m | assumed | Q-070 |
| skid.ski_trim_deg | 4 | deg | assumed | Q-070 |
| skid.float_beam_m | 0.04 | m | assumed | Q-070 |
| skid.float_length_m | 0.12 | m | assumed | Q-070 |
| skid.blade_mass_kg | 0.006 | kg | assumed | Q-070 |
| skid.ski_mass_kg | 0.01 | kg | assumed | Q-070 |
| skid.float_mass_kg | 0.03 | kg | assumed | Q-070 |
| skid.misalignment_deg | 5 | deg | assumed | Q-070 |
| skid.sense_process_s | 0.01 | s | assumed | Q-070 |
| skid.rudder_cl_max | 0.5 | 1 | assumed | Q-070 |
| skid.rho_seawater_kg_m3 | 1026 | kg/m3 | sourced | S10 |
| skid.nu_seawater_m2_s | 1.19e-06 | m2/s | sourced | S10 |
| skid.spray_cd_t2 | 0.24 | 1 | sourced | P157, P160 |
| skid.seawater_conductivity_s_m | 4.29 | S/m | sourced | P164, S10 |
| lit.skimmer_skim_speed_ms | 10.3 | m/s | sourced | P121 |
| lit.skimmer_bill_drag_wt_n | 0.0001 | N | sourced | P121 |
| lit.skimmer_bill_drag_10ms_n | 0.35 | N | sourced | P157 |
| lit.skimmer_bill_immersion_m | 0.04 | m | sourced | P157 |
| lit.skimmer_hydro_drag_share | 0.2 | 1 | sourced | P157 |
| lit.skimmer_skim_height_m | 0.1 | m | sourced | P158 |
| lit.skimmer_lower_beak_thickness_max_mm | 3.3 | mm | sourced | P159 |
| lit.ventilation_inception_deg | 15 | deg | sourced | P161, P162 |
| lit.tipfloat_load_cap_displaced | 3 | 1 | sourced | P163 |
| lit.ultrasonic_min_range_m | 0.2 | m | sourced | M18 |
| lit.ultrasonic_rate_hz | 10 | Hz | sourced | M18 |
| lit.cdc_rate_max_sps | 4080 | 1/s | sourced | M19 |
| skim.ar20l_h10_c100_u10_ms | 13.45 | m/s | calculated | docs/02-form.md#wingtip-skim |
| skim.ar20l_h20_c030_u10_ms | 12.85 | m/s | calculated | docs/02-form.md#wingtip-skim |
| skim.ar20l_h10_c030_u10_ms | 12.07 | m/s | calculated | docs/02-form.md#wingtip-skim |
| skim.ar20l_h05_c030_u10_ms | 12.07 | m/s | calculated | docs/02-form.md#wingtip-skim |
| skim.ar20l_h03_c030_u10_ms | 12.07 | m/s | calculated | docs/02-form.md#wingtip-skim |
| skim.ar20l_h03_c030_zmin_m | 1.61 | m | calculated | docs/02-form.md#wingtip-skim |
| skim.ar20l_h20_c010_u10_ms | 12.85 | m/s | calculated | docs/02-form.md#wingtip-skim |
| skim.ar20l_h10_c010_u10_ms | 11.62 | m/s | calculated | docs/02-form.md#wingtip-skim |
| skim.ar20l_h05_c010_u10_ms | 11.62 | m/s | calculated | docs/02-form.md#wingtip-skim |
| skim.ar20l_h03_c010_u10_ms | 11.62 | m/s | calculated | docs/02-form.md#wingtip-skim |
| skim.ar20l_h20_c000_u10_ms | 12.85 | m/s | calculated | docs/02-form.md#wingtip-skim |
| skim.ar20l_h10_c000_u10_ms | 11.39 | m/s | calculated | docs/02-form.md#wingtip-skim |
| skim.ar20l_h05_c000_u10_ms | 11.38 | m/s | calculated | docs/02-form.md#wingtip-skim |
| skim.ar20l_h03_c000_u10_ms | 11.38 | m/s | calculated | docs/02-form.md#wingtip-skim |
| skim.ar20l_h03_c000_zmin_m | 1.16 | m | calculated | docs/02-form.md#wingtip-skim |
| skim.ar20l_h20_c100_u10_ms | 13.46 | m/s | calculated | docs/02-form.md#wingtip-skim |
| skim.ar20l_dih5_h03_c030_u10_ms | 11.96 | m/s | calculated | docs/02-form.md#wingtip-skim |
| skim.ar20l_dih10_h03_c030_u10_ms | 11.85 | m/s | calculated | docs/02-form.md#wingtip-skim |
| skim.ar20l_dih10_h03_c030_zmin_m | 1.13 | m | calculated | docs/02-form.md#wingtip-skim |
| skim.ar20l_out10_h03_c030_u10_ms | 11.96 | m/s | calculated | docs/02-form.md#wingtip-skim |
| skim.ar20l_out20_h03_c030_u10_ms | 11.88 | m/s | calculated | docs/02-form.md#wingtip-skim |
| skim.ar20l_dih5_h03_c000_u10_ms | 11.2 | m/s | calculated | docs/02-form.md#wingtip-skim |
| skim.ar20l_dih10_h03_c000_u10_ms | 10.98 | m/s | calculated | docs/02-form.md#wingtip-skim |
| skim.ar20l_dih10_h03_c000_zmin_m | 0.62 | m | calculated | docs/02-form.md#wingtip-skim |
| skim.ar20l_out10_h03_c000_u10_ms | 11.21 | m/s | calculated | docs/02-form.md#wingtip-skim |
| skim.ar20l_out20_h03_c000_u10_ms | 11.01 | m/s | calculated | docs/02-form.md#wingtip-skim |
| skim.ar20n_h20_c100_u10_ms | 16.13 | m/s | calculated | docs/02-form.md#wingtip-skim |
| skim.ar20n_h10_c100_u10_ms | 16.13 | m/s | calculated | docs/02-form.md#wingtip-skim |
| skim.ar20n_h20_c030_u10_ms | 15.43 | m/s | calculated | docs/02-form.md#wingtip-skim |
| skim.ar20n_h10_c030_u10_ms | 14.35 | m/s | calculated | docs/02-form.md#wingtip-skim |
| skim.ar20n_h05_c030_u10_ms | 14.35 | m/s | calculated | docs/02-form.md#wingtip-skim |
| skim.ar20n_h03_c030_u10_ms | 14.35 | m/s | calculated | docs/02-form.md#wingtip-skim |
| skim.ar20n_h03_c030_zmin_m | 1.46 | m | calculated | docs/02-form.md#wingtip-skim |
| skim.ar20n_h20_c010_u10_ms | 15.43 | m/s | calculated | docs/02-form.md#wingtip-skim |
| skim.ar20n_h10_c010_u10_ms | 13.79 | m/s | calculated | docs/02-form.md#wingtip-skim |
| skim.ar20n_h05_c010_u10_ms | 13.79 | m/s | calculated | docs/02-form.md#wingtip-skim |
| skim.ar20n_h03_c010_u10_ms | 13.79 | m/s | calculated | docs/02-form.md#wingtip-skim |
| skim.ar20n_h20_c000_u10_ms | 15.43 | m/s | calculated | docs/02-form.md#wingtip-skim |
| skim.ar20n_h10_c000_u10_ms | 13.49 | m/s | calculated | docs/02-form.md#wingtip-skim |
| skim.ar20n_h05_c000_u10_ms | 13.49 | m/s | calculated | docs/02-form.md#wingtip-skim |
| skim.ar20n_h03_c000_u10_ms | 13.49 | m/s | calculated | docs/02-form.md#wingtip-skim |
| skim.ar20n_h03_c000_zmin_m | 1.13 | m | calculated | docs/02-form.md#wingtip-skim |
| skim.ar20n_dih5_h03_c030_u10_ms | 14.15 | m/s | calculated | docs/02-form.md#wingtip-skim |
| skim.ar20n_dih10_h03_c030_u10_ms | 13.97 | m/s | calculated | docs/02-form.md#wingtip-skim |
| skim.ar20n_dih10_h03_c030_zmin_m | 1.19 | m | calculated | docs/02-form.md#wingtip-skim |
| skim.ar20n_out10_h03_c030_u10_ms | 14.16 | m/s | calculated | docs/02-form.md#wingtip-skim |
| skim.ar20n_out20_h03_c030_u10_ms | 14.01 | m/s | calculated | docs/02-form.md#wingtip-skim |
| skim.ar20n_dih5_h03_c000_u10_ms | 13.25 | m/s | calculated | docs/02-form.md#wingtip-skim |
| skim.ar20n_dih10_h03_c000_u10_ms | 12.79 | m/s | calculated | docs/02-form.md#wingtip-skim |
| skim.ar20n_dih10_h03_c000_zmin_m | 0.4 | m | calculated | docs/02-form.md#wingtip-skim |
| skim.ar20n_out10_h03_c000_u10_ms | 13.26 | m/s | calculated | docs/02-form.md#wingtip-skim |
| skim.ar20n_out20_h03_c000_u10_ms | 12.77 | m/s | calculated | docs/02-form.md#wingtip-skim |
| skim.alb_h20_c100_u10_ms | 10.81 | m/s | calculated | docs/02-form.md#wingtip-skim |
| skim.alb_h10_c100_u10_ms | 10.81 | m/s | calculated | docs/02-form.md#wingtip-skim |
| skim.alb_h20_c030_u10_ms | 10.32 | m/s | calculated | docs/02-form.md#wingtip-skim |
| skim.alb_h10_c030_u10_ms | 9.67 | m/s | calculated | docs/02-form.md#wingtip-skim |
| skim.alb_h05_c030_u10_ms | 9.67 | m/s | calculated | docs/02-form.md#wingtip-skim |
| skim.alb_h03_c030_u10_ms | 9.67 | m/s | calculated | docs/02-form.md#wingtip-skim |
| skim.alb_h03_c030_zmin_m | 1.61 | m | calculated | docs/02-form.md#wingtip-skim |
| skim.alb_h20_c010_u10_ms | 10.32 | m/s | calculated | docs/02-form.md#wingtip-skim |
| skim.alb_h10_c010_u10_ms | 9.31 | m/s | calculated | docs/02-form.md#wingtip-skim |
| skim.alb_h05_c010_u10_ms | 9.31 | m/s | calculated | docs/02-form.md#wingtip-skim |
| skim.alb_h03_c010_u10_ms | 9.31 | m/s | calculated | docs/02-form.md#wingtip-skim |
| skim.alb_h20_c000_u10_ms | 10.32 | m/s | calculated | docs/02-form.md#wingtip-skim |
| skim.alb_h10_c000_u10_ms | 9.11 | m/s | calculated | docs/02-form.md#wingtip-skim |
| skim.alb_h05_c000_u10_ms | 9.11 | m/s | calculated | docs/02-form.md#wingtip-skim |
| skim.alb_h03_c000_u10_ms | 9.11 | m/s | calculated | docs/02-form.md#wingtip-skim |
| skim.alb_h03_c000_zmin_m | 1.18 | m | calculated | docs/02-form.md#wingtip-skim |
| skim.alb_dih5_h03_c030_u10_ms | 9.58 | m/s | calculated | docs/02-form.md#wingtip-skim |
| skim.alb_dih10_h03_c030_u10_ms | 9.48 | m/s | calculated | docs/02-form.md#wingtip-skim |
| skim.alb_dih10_h03_c030_zmin_m | 1.13 | m | calculated | docs/02-form.md#wingtip-skim |
| skim.alb_out10_h03_c030_u10_ms | 9.58 | m/s | calculated | docs/02-form.md#wingtip-skim |
| skim.alb_out20_h03_c030_u10_ms | 9.51 | m/s | calculated | docs/02-form.md#wingtip-skim |
| skim.alb_dih5_h03_c000_u10_ms | 8.96 | m/s | calculated | docs/02-form.md#wingtip-skim |
| skim.alb_dih10_h03_c000_u10_ms | 8.79 | m/s | calculated | docs/02-form.md#wingtip-skim |
| skim.alb_dih10_h03_c000_zmin_m | 0.63 | m | calculated | docs/02-form.md#wingtip-skim |
| skim.alb_out10_h03_c000_u10_ms | 8.96 | m/s | calculated | docs/02-form.md#wingtip-skim |
| skim.alb_out20_h03_c000_u10_ms | 8.82 | m/s | calculated | docs/02-form.md#wingtip-skim |
| skid.req_wet_latency_s | 0.005 | s | calculated | docs/02-form.md#wingtip-skim |
| skid.req_wet_rate_hz | 1000 | Hz | calculated | docs/02-form.md#wingtip-skim |
| skid.req_wet_levels | 3 | count | calculated | docs/02-form.md#wingtip-skim |
| skid.req_range_span_m | 0.45 | m | calculated | docs/02-form.md#wingtip-skim |
| skid.req_range_accuracy_m | 0.005 | m | calculated | docs/02-form.md#wingtip-skim |
| skid.req_range_rate_hz | 200 | Hz | calculated | docs/02-form.md#wingtip-skim |
| skid.req_range_latency_s | 0.005 | s | calculated | docs/02-form.md#wingtip-skim |
| lit.skimmer_tow_speed_min_ms | 1.8 | m/s | sourced | P157 |
| lit.skimmer_tow_speed_max_ms | 6.8 | m/s | sourced | P157 |
| lit.skimmer_bill_drag_10ms_central_n | 0.95 | N | calculated | docs/02-form.md#wingtip-skim |
| lit.ventilation_froude_max | 3.0 | 1 | sourced | P161, P162 |
| skid.blade_cda_m2 | 4.759e-05 | m2 | calculated | docs/02-form.md#wingtip-skim |
| skid.blade_pair_over_body_cda | 0.041 | 1 | calculated | docs/02-form.md#wingtip-skim |
| skid.ski_cda_m2 | 7.543e-05 | m2 | calculated | docs/02-form.md#wingtip-skim |
| skid.ski_pair_over_body_cda | 0.065 | 1 | calculated | docs/02-form.md#wingtip-skim |
| skid.float_cda_m2 | 8.517e-05 | m2 | calculated | docs/02-form.md#wingtip-skim |
| skid.float_pair_over_body_cda | 0.074 | 1 | calculated | docs/02-form.md#wingtip-skim |
| skid.ar20l_arc_bank_min_deg | 50.3 | deg | calculated | docs/02-form.md#wingtip-skim |
| skid.ar20l_arc_bank_max_deg | 61.9 | deg | calculated | docs/02-form.md#wingtip-skim |
| skid.ar20l_arc_n_min | 1.57 | 1 | calculated | docs/02-form.md#wingtip-skim |
| skid.ar20l_arc_n_max | 2.8 | 1 | calculated | docs/02-form.md#wingtip-skim |
| skid.ar20l_arc_water_min_ms | 15.9 | m/s | calculated | docs/02-form.md#wingtip-skim |
| skid.ar20l_arc_water_max_ms | 24.8 | m/s | calculated | docs/02-form.md#wingtip-skim |
| skid.ar20l_arc_crab_min_deg | 18.9 | deg | calculated | docs/02-form.md#wingtip-skim |
| skid.ar20l_arc_crab_max_deg | 30.0 | deg | calculated | docs/02-form.md#wingtip-skim |
| skid.ar20l_arc_bend_min_m | 0.043 | m | calculated | docs/02-form.md#wingtip-skim |
| skid.ar20l_arc_bend_max_m | 0.093 | m | calculated | docs/02-form.md#wingtip-skim |
| skid.ar20l_arc_s | 1.14 | s | calculated | docs/02-form.md#wingtip-skim |
| skid.ar20l_arc_zmin_m | 1.16 | m | calculated | docs/02-form.md#wingtip-skim |
| skid.ar20l_roll_rise_shallow_ms | 0.98 | m/s | calculated | docs/02-form.md#wingtip-skim |
| skid.ar20l_airspeed_ms | 19.6 | m/s | calculated | docs/02-form.md#wingtip-skim |
| skid.ar20l_bank_deg | 61.9 | deg | calculated | docs/02-form.md#wingtip-skim |
| skid.ar20l_skim_arc_s | 1.38 | s | calculated | docs/02-form.md#wingtip-skim |
| skid.ar20l_roll_rise_ms | 0.74 | m/s | calculated | docs/02-form.md#wingtip-skim |
| skid.ar20l_w05_wet_depth_m | 0.041 | m | calculated | docs/02-form.md#wingtip-skim |
| skid.ar20l_w10_wet_depth_m | 0.09 | m | calculated | docs/02-form.md#wingtip-skim |
| skid.ar20l_w20_wet_depth_m | 0.1 | m | calculated | docs/02-form.md#wingtip-skim |
| skid.ar20l_w05_dl10_trigger_1cm_m | 0.03 | m | calculated | docs/02-form.md#wingtip-skim |
| skid.ar20l_w05_dl10_trigger_2cm_m | 0.021 | m | calculated | docs/02-form.md#wingtip-skim |
| skid.ar20l_w05_dl30_trigger_1cm_m | 0.03 | m | calculated | docs/02-form.md#wingtip-skim |
| skid.ar20l_w05_dl30_trigger_2cm_m | 0.021 | m | calculated | docs/02-form.md#wingtip-skim |
| skid.ar20l_w10_dl10_trigger_1cm_m | 0.111 | m | calculated | docs/02-form.md#wingtip-skim |
| skid.ar20l_w10_dl10_trigger_2cm_m | 0.1 | m | calculated | docs/02-form.md#wingtip-skim |
| skid.ar20l_w10_dl30_trigger_1cm_m | 0.112 | m | calculated | docs/02-form.md#wingtip-skim |
| skid.ar20l_w10_dl30_trigger_2cm_m | 0.102 | m | calculated | docs/02-form.md#wingtip-skim |
| skid.ar20l_w20_dl10_trigger_1cm_m | 0.322 | m | calculated | docs/02-form.md#wingtip-skim |
| skid.ar20l_w20_dl10_trigger_2cm_m | 0.3 | m | calculated | docs/02-form.md#wingtip-skim |
| skid.ar20l_w20_dl30_trigger_1cm_m | 0.444 | m | calculated | docs/02-form.md#wingtip-skim |
| skid.ar20l_w20_dl30_trigger_2cm_m | 0.431 | m | calculated | docs/02-form.md#wingtip-skim |
| skid.wet_latency_s | 0.06 | s | calculated | docs/02-form.md#wingtip-skim |
| skid.ar20l_ixx_kgm2 | 0.468 | kg m2 | calculated | docs/02-form.md#wingtip-skim |
| skid.ar20l_izz_kgm2 | 0.552 | kg m2 | calculated | docs/02-form.md#wingtip-skim |
| skid.ar20l_roll_moment_max_nm | 19.6 | N m | calculated | docs/02-form.md#wingtip-skim |
| skid.ar20l_yaw_moment_max_nm | 3.19 | N m | calculated | docs/02-form.md#wingtip-skim |
| skid.ar20l_roll_tau_s | 0.025 | s | calculated | docs/02-form.md#wingtip-skim |
| skid.ar20l_weight_n | 29.4 | N | calculated | docs/02-form.md#wingtip-skim |
| skid.ar20l_force_water_speed_ms | 24.8 | m/s | calculated | docs/02-form.md#wingtip-skim |
| skid.ar20l_blade_froude_d05 | 112 | 1 | calculated | docs/02-form.md#wingtip-skim |
| skid.ar20l_blade_d05_drag_n | 0.17 | N | calculated | docs/02-form.md#wingtip-skim |
| skid.ar20l_blade_d05_side_n | 1.08 | N | calculated | docs/02-form.md#wingtip-skim |
| skid.ar20l_blade_d05_yaw_over_rudder | 0.08 | 1 | calculated | docs/02-form.md#wingtip-skim |
| skid.ar20l_blade_d05_roll_over_aileron | 0.07 | 1 | calculated | docs/02-form.md#wingtip-skim |
| skid.ar20l_blade_d05_sideslip_deg | 1.1 | deg | calculated | docs/02-form.md#wingtip-skim |
| skid.ar20l_blade_d05_yaw_0p2s_deg | 0.54 | deg | calculated | docs/02-form.md#wingtip-skim |
| skid.ar20l_blade_froude_d10 | 79 | 1 | calculated | docs/02-form.md#wingtip-skim |
| skid.ar20l_fixed_d10_side_min_n | 16.3 | N | calculated | docs/02-form.md#wingtip-skim |
| skid.ar20l_fixed_d10_roll_over_aileron_min | 1.1 | 1 | calculated | docs/02-form.md#wingtip-skim |
| skid.ar20l_fixed_d10_side_max_n | 26.0 | N | calculated | docs/02-form.md#wingtip-skim |
| skid.ar20l_fixed_d10_roll_over_aileron_max | 1.76 | 1 | calculated | docs/02-form.md#wingtip-skim |
| skid.ar20l_dih10_roll_d10_nm | 3.1 | N m | calculated | docs/02-form.md#wingtip-skim |
| skid.ar20l_dih10_roll_d10_over_aileron | 0.16 | 1 | calculated | docs/02-form.md#wingtip-skim |
| skid.ar20l_dih10_clbeta_per_rad | -0.217 | 1/rad | calculated | docs/02-form.md#wingtip-skim |
| skid.ar20l_blade_d10_drag_n | 0.4 | N | calculated | docs/02-form.md#wingtip-skim |
| skid.ar20l_blade_d10_side_n | 4.33 | N | calculated | docs/02-form.md#wingtip-skim |
| skid.ar20l_blade_d10_yaw_over_rudder | 0.19 | 1 | calculated | docs/02-form.md#wingtip-skim |
| skid.ar20l_blade_d10_roll_over_aileron | 0.29 | 1 | calculated | docs/02-form.md#wingtip-skim |
| skid.ar20l_blade_d10_sideslip_deg | 2.6 | deg | calculated | docs/02-form.md#wingtip-skim |
| skid.ar20l_blade_d10_yaw_0p2s_deg | 1.25 | deg | calculated | docs/02-form.md#wingtip-skim |
| skid.ar20l_ski_d10_drag_n | 2.54 | N | calculated | docs/02-form.md#wingtip-skim |
| skid.ar20l_ski_d10_normal_n | 12.08 | N | calculated | docs/02-form.md#wingtip-skim |
| skid.ar20l_ski_d10_yaw_over_rudder | 1.2 | 1 | calculated | docs/02-form.md#wingtip-skim |
| skid.ar20l_ski_d10_roll_over_aileron | 0.92 | 1 | calculated | docs/02-form.md#wingtip-skim |
| skid.ar20l_ski_d10_sideslip_deg | 16.4 | deg | calculated | docs/02-form.md#wingtip-skim |
| skid.ar20l_ski_d10_yaw_0p2s_deg | 7.92 | deg | calculated | docs/02-form.md#wingtip-skim |
| skid.ar20l_tip_d10_drag_n | 11.97 | N | calculated | docs/02-form.md#wingtip-skim |
| skid.ar20l_tip_d10_normal_n | 25.26 | N | calculated | docs/02-form.md#wingtip-skim |
| skid.ar20l_tip_d10_yaw_over_rudder | 5.63 | 1 | calculated | docs/02-form.md#wingtip-skim |
| skid.ar20l_tip_d10_roll_over_aileron | 1.93 | 1 | calculated | docs/02-form.md#wingtip-skim |
| skid.ar20l_tip_d10_sideslip_deg | 77.0 | deg | calculated | docs/02-form.md#wingtip-skim |
| skid.ar20l_tip_d10_yaw_0p2s_deg | 37.29 | deg | calculated | docs/02-form.md#wingtip-skim |
| skid.ar20l_float_d10_drag_n | 9.28 | N | calculated | docs/02-form.md#wingtip-skim |
| skid.ar20l_float_d10_normal_n | 48.4 | N | calculated | docs/02-form.md#wingtip-skim |
| skid.ar20l_float_d10_yaw_over_rudder | 4.36 | 1 | calculated | docs/02-form.md#wingtip-skim |
| skid.ar20l_float_d10_roll_over_aileron | 3.7 | 1 | calculated | docs/02-form.md#wingtip-skim |
| skid.ar20l_float_d10_sideslip_deg | 59.7 | deg | calculated | docs/02-form.md#wingtip-skim |
| skid.ar20l_float_d10_yaw_0p2s_deg | 28.9 | deg | calculated | docs/02-form.md#wingtip-skim |
| skid.ar20l_blade_froude_d20 | 56 | 1 | calculated | docs/02-form.md#wingtip-skim |
| skid.ar20l_fixed_d20_side_min_n | 65.4 | N | calculated | docs/02-form.md#wingtip-skim |
| skid.ar20l_fixed_d20_roll_over_aileron_min | 4.41 | 1 | calculated | docs/02-form.md#wingtip-skim |
| skid.ar20l_fixed_d20_side_max_n | 104.0 | N | calculated | docs/02-form.md#wingtip-skim |
| skid.ar20l_fixed_d20_roll_over_aileron_max | 7.02 | 1 | calculated | docs/02-form.md#wingtip-skim |
| skid.ar20l_dih10_roll_d20_nm | 9.1 | N m | calculated | docs/02-form.md#wingtip-skim |
| skid.ar20l_dih10_roll_d20_over_aileron | 0.47 | 1 | calculated | docs/02-form.md#wingtip-skim |
| skid.ar20l_blade_d20_drag_n | 1.18 | N | calculated | docs/02-form.md#wingtip-skim |
| skid.ar20l_blade_d20_side_n | 17.32 | N | calculated | docs/02-form.md#wingtip-skim |
| skid.ar20l_blade_d20_yaw_over_rudder | 0.55 | 1 | calculated | docs/02-form.md#wingtip-skim |
| skid.ar20l_blade_d20_roll_over_aileron | 1.17 | 1 | calculated | docs/02-form.md#wingtip-skim |
| skid.ar20l_blade_d20_sideslip_deg | 7.6 | deg | calculated | docs/02-form.md#wingtip-skim |
| skid.ar20l_blade_d20_yaw_0p2s_deg | 3.67 | deg | calculated | docs/02-form.md#wingtip-skim |
| skid.ar20l_blade_d20_impulse_0p2s | 0.004 | 1 | calculated | docs/02-form.md#wingtip-skim |
| skid.ar20l_ski_d20_drag_n | 2.54 | N | calculated | docs/02-form.md#wingtip-skim |
| skid.ar20l_ski_d20_normal_n | 12.08 | N | calculated | docs/02-form.md#wingtip-skim |
| skid.ar20l_ski_d20_yaw_over_rudder | 1.2 | 1 | calculated | docs/02-form.md#wingtip-skim |
| skid.ar20l_ski_d20_roll_over_aileron | 0.92 | 1 | calculated | docs/02-form.md#wingtip-skim |
| skid.ar20l_ski_d20_sideslip_deg | 16.4 | deg | calculated | docs/02-form.md#wingtip-skim |
| skid.ar20l_ski_d20_yaw_0p2s_deg | 7.92 | deg | calculated | docs/02-form.md#wingtip-skim |
| skid.ar20l_tip_d20_drag_n | 21.64 | N | calculated | docs/02-form.md#wingtip-skim |
| skid.ar20l_tip_d20_normal_n | 69.96 | N | calculated | docs/02-form.md#wingtip-skim |
| skid.ar20l_tip_d20_yaw_over_rudder | 10.17 | 1 | calculated | docs/02-form.md#wingtip-skim |
| skid.ar20l_tip_d20_roll_over_aileron | 5.35 | 1 | calculated | docs/02-form.md#wingtip-skim |
| skid.ar20l_tip_d20_sideslip_deg | 139.2 | deg | calculated | docs/02-form.md#wingtip-skim |
| skid.ar20l_tip_d20_yaw_0p2s_deg | 67.41 | deg | calculated | docs/02-form.md#wingtip-skim |
| skid.ar20l_tip_d20_impulse_0p2s | 0.0735 | 1 | calculated | docs/02-form.md#wingtip-skim |
| skid.ar20l_float_d20_drag_n | 9.28 | N | calculated | docs/02-form.md#wingtip-skim |
| skid.ar20l_float_d20_normal_n | 48.4 | N | calculated | docs/02-form.md#wingtip-skim |
| skid.ar20l_float_d20_yaw_over_rudder | 4.36 | 1 | calculated | docs/02-form.md#wingtip-skim |
| skid.ar20l_float_d20_roll_over_aileron | 3.7 | 1 | calculated | docs/02-form.md#wingtip-skim |
| skid.ar20l_float_d20_sideslip_deg | 59.7 | deg | calculated | docs/02-form.md#wingtip-skim |
| skid.ar20l_float_d20_yaw_0p2s_deg | 28.9 | deg | calculated | docs/02-form.md#wingtip-skim |
| skid.ar20l_blade_froude_d30 | 46 | 1 | calculated | docs/02-form.md#wingtip-skim |
| skid.ar20l_blade_d30_drag_n | 2.34 | N | calculated | docs/02-form.md#wingtip-skim |
| skid.ar20l_blade_d30_side_n | 38.97 | N | calculated | docs/02-form.md#wingtip-skim |
| skid.ar20l_blade_d30_yaw_over_rudder | 1.1 | 1 | calculated | docs/02-form.md#wingtip-skim |
| skid.ar20l_blade_d30_roll_over_aileron | 2.63 | 1 | calculated | docs/02-form.md#wingtip-skim |
| skid.ar20l_blade_d30_sideslip_deg | 15.1 | deg | calculated | docs/02-form.md#wingtip-skim |
| skid.ar20l_blade_d30_yaw_0p2s_deg | 7.3 | deg | calculated | docs/02-form.md#wingtip-skim |
| skid.ar20n_arc_bank_min_deg | 48.9 | deg | calculated | docs/02-form.md#wingtip-skim |
| skid.ar20n_arc_bank_max_deg | 52.8 | deg | calculated | docs/02-form.md#wingtip-skim |
| skid.ar20n_arc_n_min | 3.2 | 1 | calculated | docs/02-form.md#wingtip-skim |
| skid.ar20n_arc_n_max | 3.22 | 1 | calculated | docs/02-form.md#wingtip-skim |
| skid.ar20n_arc_water_min_ms | 16.5 | m/s | calculated | docs/02-form.md#wingtip-skim |
| skid.ar20n_arc_water_max_ms | 19.2 | m/s | calculated | docs/02-form.md#wingtip-skim |
| skid.ar20n_arc_crab_min_deg | 32.7 | deg | calculated | docs/02-form.md#wingtip-skim |
| skid.ar20n_arc_crab_max_deg | 39.3 | deg | calculated | docs/02-form.md#wingtip-skim |
| skid.ar20n_arc_bend_min_m | 0.109 | m | calculated | docs/02-form.md#wingtip-skim |
| skid.ar20n_arc_bend_max_m | 0.118 | m | calculated | docs/02-form.md#wingtip-skim |
| skid.ar20n_arc_s | 0.17 | s | calculated | docs/02-form.md#wingtip-skim |
| skid.ar20n_arc_zmin_m | 1.13 | m | calculated | docs/02-form.md#wingtip-skim |
| skid.ar20n_roll_rise_shallow_ms | 0.82 | m/s | calculated | docs/02-form.md#wingtip-skim |
| skid.ar20n_airspeed_ms | 15.6 | m/s | calculated | docs/02-form.md#wingtip-skim |
| skid.ar20n_bank_deg | 48.4 | deg | calculated | docs/02-form.md#wingtip-skim |
| skid.ar20n_skim_arc_s | 0.39 | s | calculated | docs/02-form.md#wingtip-skim |
| skid.ar20n_roll_rise_ms | 0.83 | m/s | calculated | docs/02-form.md#wingtip-skim |
| skid.ar20n_w05_wet_depth_m | 0.039 | m | calculated | docs/02-form.md#wingtip-skim |
| skid.ar20n_w10_wet_depth_m | 0.089 | m | calculated | docs/02-form.md#wingtip-skim |
| skid.ar20n_w20_wet_depth_m | 0.1 | m | calculated | docs/02-form.md#wingtip-skim |
| skid.ar20n_w05_dl10_trigger_1cm_m | 0.029 | m | calculated | docs/02-form.md#wingtip-skim |
| skid.ar20n_w05_dl10_trigger_2cm_m | 0.019 | m | calculated | docs/02-form.md#wingtip-skim |
| skid.ar20n_w05_dl30_trigger_1cm_m | 0.029 | m | calculated | docs/02-form.md#wingtip-skim |
| skid.ar20n_w05_dl30_trigger_2cm_m | 0.019 | m | calculated | docs/02-form.md#wingtip-skim |
| skid.ar20n_w10_dl10_trigger_1cm_m | 0.104 | m | calculated | docs/02-form.md#wingtip-skim |
| skid.ar20n_w10_dl10_trigger_2cm_m | 0.093 | m | calculated | docs/02-form.md#wingtip-skim |
| skid.ar20n_w10_dl30_trigger_1cm_m | 0.104 | m | calculated | docs/02-form.md#wingtip-skim |
| skid.ar20n_w10_dl30_trigger_2cm_m | 0.094 | m | calculated | docs/02-form.md#wingtip-skim |
| skid.ar20n_w20_dl10_trigger_1cm_m | 0.307 | m | calculated | docs/02-form.md#wingtip-skim |
| skid.ar20n_w20_dl10_trigger_2cm_m | 0.286 | m | calculated | docs/02-form.md#wingtip-skim |
| skid.ar20n_w20_dl30_trigger_1cm_m | 0.422 | m | calculated | docs/02-form.md#wingtip-skim |
| skid.ar20n_w20_dl30_trigger_2cm_m | 0.41 | m | calculated | docs/02-form.md#wingtip-skim |
| skid.alb_arc_bank_min_deg | 51.8 | deg | calculated | docs/02-form.md#wingtip-skim |
| skid.alb_arc_bank_max_deg | 61.9 | deg | calculated | docs/02-form.md#wingtip-skim |
| skid.alb_arc_n_min | 1.6 | 1 | calculated | docs/02-form.md#wingtip-skim |
| skid.alb_arc_n_max | 2.85 | 1 | calculated | docs/02-form.md#wingtip-skim |
| skid.alb_arc_water_min_ms | 16.7 | m/s | calculated | docs/02-form.md#wingtip-skim |
| skid.alb_arc_water_max_ms | 24.1 | m/s | calculated | docs/02-form.md#wingtip-skim |
| skid.alb_arc_crab_min_deg | 15.7 | deg | calculated | docs/02-form.md#wingtip-skim |
| skid.alb_arc_crab_max_deg | 23.1 | deg | calculated | docs/02-form.md#wingtip-skim |
| skid.alb_arc_bend_min_m | 0.042 | m | calculated | docs/02-form.md#wingtip-skim |
| skid.alb_arc_bend_max_m | 0.09 | m | calculated | docs/02-form.md#wingtip-skim |
| skid.alb_arc_s | 1.01 | s | calculated | docs/02-form.md#wingtip-skim |
| skid.alb_arc_zmin_m | 1.19 | m | calculated | docs/02-form.md#wingtip-skim |
| skid.alb_roll_rise_shallow_ms | 0.99 | m/s | calculated | docs/02-form.md#wingtip-skim |
| skid.alb_airspeed_ms | 20.1 | m/s | calculated | docs/02-form.md#wingtip-skim |
| skid.alb_bank_deg | 61.9 | deg | calculated | docs/02-form.md#wingtip-skim |
| skid.alb_skim_arc_s | 1.39 | s | calculated | docs/02-form.md#wingtip-skim |
| skid.alb_roll_rise_ms | 0.76 | m/s | calculated | docs/02-form.md#wingtip-skim |
| skid.alb_w05_wet_depth_m | 0.04 | m | calculated | docs/02-form.md#wingtip-skim |
| skid.alb_w10_wet_depth_m | 0.09 | m | calculated | docs/02-form.md#wingtip-skim |
| skid.alb_w20_wet_depth_m | 0.1 | m | calculated | docs/02-form.md#wingtip-skim |
| skid.alb_w05_dl10_trigger_1cm_m | 0.03 | m | calculated | docs/02-form.md#wingtip-skim |
| skid.alb_w05_dl10_trigger_2cm_m | 0.02 | m | calculated | docs/02-form.md#wingtip-skim |
| skid.alb_w05_dl30_trigger_1cm_m | 0.03 | m | calculated | docs/02-form.md#wingtip-skim |
| skid.alb_w05_dl30_trigger_2cm_m | 0.02 | m | calculated | docs/02-form.md#wingtip-skim |
| skid.alb_w10_dl10_trigger_1cm_m | 0.109 | m | calculated | docs/02-form.md#wingtip-skim |
| skid.alb_w10_dl10_trigger_2cm_m | 0.098 | m | calculated | docs/02-form.md#wingtip-skim |
| skid.alb_w10_dl30_trigger_1cm_m | 0.109 | m | calculated | docs/02-form.md#wingtip-skim |
| skid.alb_w10_dl30_trigger_2cm_m | 0.099 | m | calculated | docs/02-form.md#wingtip-skim |
| skid.alb_w20_dl10_trigger_1cm_m | 0.317 | m | calculated | docs/02-form.md#wingtip-skim |
| skid.alb_w20_dl10_trigger_2cm_m | 0.296 | m | calculated | docs/02-form.md#wingtip-skim |
| skid.alb_w20_dl30_trigger_1cm_m | 0.433 | m | calculated | docs/02-form.md#wingtip-skim |
| skid.alb_w20_dl30_trigger_2cm_m | 0.421 | m | calculated | docs/02-form.md#wingtip-skim |
| skid.ar20l_dih10_arc_bank_min_deg | 33.5 | deg | calculated | docs/02-form.md#wingtip-skim |
| skid.ar20l_dih10_arc_zmin_m | 0.6 | m | calculated | docs/02-form.md#wingtip-skim |
| skid.blade_skimmer_check_10ms_n | 0.89 | N | calculated | docs/02-form.md#wingtip-skim |
| lit.skimmer_tow_froude_min | 2.9 | 1 | calculated | docs/02-form.md#wingtip-skim |
| lit.skimmer_tow_froude_max | 10.9 | 1 | calculated | docs/02-form.md#wingtip-skim |
| spar.ud_carbon_modulus | 120000000000.0 | Pa | assumed | Q-010 |
| heavy.albatross_wing_loading_npm2 | 143.4 | N/m2 | calculated | docs/02-form.md#wing-sizing |
| heavy.ar16_m03_wing_loading_npm2 | 52.3 | N/m2 | calculated | docs/02-form.md#wing-sizing |
| heavy.ar16_m03_over_albatross | 0.36 | 1 | calculated | docs/02-form.md#wing-sizing |
| heavy.ar16_stiff_caps_factor | 3.64 | 1 | calculated | docs/02-form.md#wing-sizing |
| heavy.ar16_stress_caps_deflection | 0.55 | 1 | calculated | docs/02-form.md#wing-sizing |
| heavy.ar16_m03_u10_ms | 13.01 | m/s | calculated | docs/02-form.md#wing-sizing |
| heavy.ar16_m03_re_cycle | 195000 | 1 | calculated | docs/02-form.md#wing-sizing |
| heavy.ar16_m03_p20_w | 28.2 | W | calculated | docs/02-form.md#wing-sizing |
| heavy.ar16_m03_e20_wh_per_km | 0.44 | Wh/km | calculated | docs/02-form.md#wing-sizing |
| heavy.ar16_m03_p20_steady_w | 28.2 | W | calculated | docs/02-form.md#wing-sizing |
| heavy.ar16_m03_p180_w | 35.9 | W | calculated | docs/02-form.md#wing-sizing |
| heavy.ar16_m03_e180_wh_per_km | 1.8 | Wh/km | calculated | docs/02-form.md#wing-sizing |
| heavy.ar16_m03_p180_steady_w | 36.7 | W | calculated | docs/02-form.md#wing-sizing |
| heavy.ar16_m04_wing_loading_npm2 | 69.8 | N/m2 | calculated | docs/02-form.md#wing-sizing |
| heavy.ar16_m04_over_albatross | 0.49 | 1 | calculated | docs/02-form.md#wing-sizing |
| heavy.ar16_m04_u10_ms | 12.6 | m/s | calculated | docs/02-form.md#wing-sizing |
| heavy.ar16_m04_re_cycle | 220000 | 1 | calculated | docs/02-form.md#wing-sizing |
| heavy.ar16_m04_p20_w | 41.5 | W | calculated | docs/02-form.md#wing-sizing |
| heavy.ar16_m04_e20_wh_per_km | 0.6 | Wh/km | calculated | docs/02-form.md#wing-sizing |
| heavy.ar16_m04_p20_steady_w | 41.5 | W | calculated | docs/02-form.md#wing-sizing |
| heavy.ar16_m04_p180_w | 47.2 | W | calculated | docs/02-form.md#wing-sizing |
| heavy.ar16_m04_e180_wh_per_km | 2.04 | Wh/km | calculated | docs/02-form.md#wing-sizing |
| heavy.ar16_m04_p180_steady_w | 48.0 | W | calculated | docs/02-form.md#wing-sizing |
| heavy.ar16_m06_wing_loading_npm2 | 104.6 | N/m2 | calculated | docs/02-form.md#wing-sizing |
| heavy.ar16_m06_over_albatross | 0.73 | 1 | calculated | docs/02-form.md#wing-sizing |
| heavy.ar16_m06_u10_ms | 12.34 | m/s | calculated | docs/02-form.md#wing-sizing |
| heavy.ar16_m06_re_cycle | 265000 | 1 | calculated | docs/02-form.md#wing-sizing |
| heavy.ar16_m06_p20_w | 71.7 | W | calculated | docs/02-form.md#wing-sizing |
| heavy.ar16_m06_e20_wh_per_km | 0.91 | Wh/km | calculated | docs/02-form.md#wing-sizing |
| heavy.ar16_m06_p20_steady_w | 71.7 | W | calculated | docs/02-form.md#wing-sizing |
| heavy.ar16_m06_p180_w | 73.8 | W | calculated | docs/02-form.md#wing-sizing |
| heavy.ar16_m06_e180_wh_per_km | 2.59 | Wh/km | calculated | docs/02-form.md#wing-sizing |
| heavy.ar16_m06_p180_steady_w | 74.6 | W | calculated | docs/02-form.md#wing-sizing |
| heavy.ar16_m08_wing_loading_npm2 | 139.5 | N/m2 | calculated | docs/02-form.md#wing-sizing |
| heavy.ar16_m08_over_albatross | 0.97 | 1 | calculated | docs/02-form.md#wing-sizing |
| heavy.ar16_m08_u10_ms | 12.27 | m/s | calculated | docs/02-form.md#wing-sizing |
| heavy.ar16_m08_re_cycle | 304000 | 1 | calculated | docs/02-form.md#wing-sizing |
| heavy.ar16_m08_p20_w | 105.8 | W | calculated | docs/02-form.md#wing-sizing |
| heavy.ar16_m08_e20_wh_per_km | 1.21 | Wh/km | calculated | docs/02-form.md#wing-sizing |
| heavy.ar16_m08_p20_steady_w | 105.8 | W | calculated | docs/02-form.md#wing-sizing |
| heavy.ar16_m08_p180_w | 105.6 | W | calculated | docs/02-form.md#wing-sizing |
| heavy.ar16_m08_e180_wh_per_km | 3.19 | Wh/km | calculated | docs/02-form.md#wing-sizing |
| heavy.ar16_m08_p180_steady_w | 106.3 | W | calculated | docs/02-form.md#wing-sizing |
| heavy.ar16_m10_wing_loading_npm2 | 174.4 | N/m2 | calculated | docs/02-form.md#wing-sizing |
| heavy.ar16_m10_over_albatross | 1.22 | 1 | calculated | docs/02-form.md#wing-sizing |
| heavy.ar16_m10_u10_ms | 12.26 | m/s | calculated | docs/02-form.md#wing-sizing |
| heavy.ar16_m10_re_cycle | 339000 | 1 | calculated | docs/02-form.md#wing-sizing |
| heavy.ar16_m10_p20_w | 143.4 | W | calculated | docs/02-form.md#wing-sizing |
| heavy.ar16_m10_e20_wh_per_km | 1.52 | Wh/km | calculated | docs/02-form.md#wing-sizing |
| heavy.ar16_m10_p20_steady_w | 143.4 | W | calculated | docs/02-form.md#wing-sizing |
| heavy.ar16_m10_p180_w | 143.4 | W | calculated | docs/02-form.md#wing-sizing |
| heavy.ar16_m10_e180_wh_per_km | 3.66 | Wh/km | calculated | docs/02-form.md#wing-sizing |
| heavy.ar16_m10_p180_steady_w | 143.4 | W | calculated | docs/02-form.md#wing-sizing |
| heavy.ar20_m03_wing_loading_npm2 | 65.4 | N/m2 | calculated | docs/02-form.md#wing-sizing |
| heavy.ar20_m03_over_albatross | 0.46 | 1 | calculated | docs/02-form.md#wing-sizing |
| heavy.ar20_stiff_caps_factor | 4.55 | 1 | calculated | docs/02-form.md#wing-sizing |
| heavy.ar20_stress_caps_deflection | 0.68 | 1 | calculated | docs/02-form.md#wing-sizing |
| heavy.ar20_m03_u10_ms | 12.87 | m/s | calculated | docs/02-form.md#wing-sizing |
| heavy.ar20_m03_re_cycle | 168000 | 1 | calculated | docs/02-form.md#wing-sizing |
| heavy.ar20_m03_p20_w | 30.3 | W | calculated | docs/02-form.md#wing-sizing |
| heavy.ar20_m03_e20_wh_per_km | 0.44 | Wh/km | calculated | docs/02-form.md#wing-sizing |
| heavy.ar20_m03_p20_steady_w | 30.3 | W | calculated | docs/02-form.md#wing-sizing |
| heavy.ar20_m03_p180_w | 35.7 | W | calculated | docs/02-form.md#wing-sizing |
| heavy.ar20_m03_e180_wh_per_km | 1.67 | Wh/km | calculated | docs/02-form.md#wing-sizing |
| heavy.ar20_m03_p180_steady_w | 36.4 | W | calculated | docs/02-form.md#wing-sizing |
| heavy.ar20_m04_wing_loading_npm2 | 87.2 | N/m2 | calculated | docs/02-form.md#wing-sizing |
| heavy.ar20_m04_over_albatross | 0.61 | 1 | calculated | docs/02-form.md#wing-sizing |
| heavy.ar20_m04_u10_ms | 12.54 | m/s | calculated | docs/02-form.md#wing-sizing |
| heavy.ar20_m04_re_cycle | 190000 | 1 | calculated | docs/02-form.md#wing-sizing |
| heavy.ar20_m04_p20_w | 44.4 | W | calculated | docs/02-form.md#wing-sizing |
| heavy.ar20_m04_e20_wh_per_km | 0.6 | Wh/km | calculated | docs/02-form.md#wing-sizing |
| heavy.ar20_m04_p20_steady_w | 44.4 | W | calculated | docs/02-form.md#wing-sizing |
| heavy.ar20_m04_p180_w | 47.5 | W | calculated | docs/02-form.md#wing-sizing |
| heavy.ar20_m04_e180_wh_per_km | 1.92 | Wh/km | calculated | docs/02-form.md#wing-sizing |
| heavy.ar20_m04_p180_steady_w | 48.2 | W | calculated | docs/02-form.md#wing-sizing |
| heavy.ar20_m06_wing_loading_npm2 | 130.8 | N/m2 | calculated | docs/02-form.md#wing-sizing |
| heavy.ar20_m06_over_albatross | 0.91 | 1 | calculated | docs/02-form.md#wing-sizing |
| heavy.ar20_m06_u10_ms | 12.31 | m/s | calculated | docs/02-form.md#wing-sizing |
| heavy.ar20_m06_re_cycle | 229000 | 1 | calculated | docs/02-form.md#wing-sizing |
| heavy.ar20_m06_p20_w | 76.2 | W | calculated | docs/02-form.md#wing-sizing |
| heavy.ar20_m06_e20_wh_per_km | 0.89 | Wh/km | calculated | docs/02-form.md#wing-sizing |
| heavy.ar20_m06_p20_steady_w | 76.2 | W | calculated | docs/02-form.md#wing-sizing |
| heavy.ar20_m06_p180_w | 76.1 | W | calculated | docs/02-form.md#wing-sizing |
| heavy.ar20_m06_e180_wh_per_km | 2.49 | Wh/km | calculated | docs/02-form.md#wing-sizing |
| heavy.ar20_m06_p180_steady_w | 76.5 | W | calculated | docs/02-form.md#wing-sizing |
| heavy.ar20_m08_wing_loading_npm2 | 174.4 | N/m2 | calculated | docs/02-form.md#wing-sizing |
| heavy.ar20_m08_over_albatross | 1.22 | 1 | calculated | docs/02-form.md#wing-sizing |
| heavy.ar20_m08_u10_ms | 12.24 | m/s | calculated | docs/02-form.md#wing-sizing |
| heavy.ar20_m08_re_cycle | 264000 | 1 | calculated | docs/02-form.md#wing-sizing |
| heavy.ar20_m08_p20_w | 112.1 | W | calculated | docs/02-form.md#wing-sizing |
| heavy.ar20_m08_e20_wh_per_km | 1.19 | Wh/km | calculated | docs/02-form.md#wing-sizing |
| heavy.ar20_m08_p20_steady_w | 112.1 | W | calculated | docs/02-form.md#wing-sizing |
| heavy.ar20_m08_p180_w | 112.1 | W | calculated | docs/02-form.md#wing-sizing |
| heavy.ar20_m08_e180_wh_per_km | 2.86 | Wh/km | calculated | docs/02-form.md#wing-sizing |
| heavy.ar20_m08_p180_steady_w | 112.1 | W | calculated | docs/02-form.md#wing-sizing |
| heavy.ar20_m10_wing_loading_npm2 | 218.0 | N/m2 | calculated | docs/02-form.md#wing-sizing |
| heavy.ar20_m10_over_albatross | 1.52 | 1 | calculated | docs/02-form.md#wing-sizing |
| heavy.ar20_m10_u10_ms | 12.24 | m/s | calculated | docs/02-form.md#wing-sizing |
| heavy.ar20_m10_re_cycle | 293000 | 1 | calculated | docs/02-form.md#wing-sizing |
| heavy.ar20_m10_p20_w | 151.3 | W | calculated | docs/02-form.md#wing-sizing |
| heavy.ar20_m10_e20_wh_per_km | 1.48 | Wh/km | calculated | docs/02-form.md#wing-sizing |
| heavy.ar20_m10_p20_steady_w | 151.3 | W | calculated | docs/02-form.md#wing-sizing |
| heavy.ar20_m10_p180_w | 151.3 | W | calculated | docs/02-form.md#wing-sizing |
| heavy.ar20_m10_e180_wh_per_km | 3.21 | Wh/km | calculated | docs/02-form.md#wing-sizing |
| heavy.ar20_m10_p180_steady_w | 151.3 | W | calculated | docs/02-form.md#wing-sizing |
| heavy.ar16_stiff_caps_factor_ud | 1.06 | 1 | calculated | docs/02-form.md#wing-sizing |
| heavy.ar16_tube_free_over_caps_min | 6.3 | 1 | calculated | docs/02-form.md#wing-sizing |
| heavy.ar16_tube_free_over_caps_max | 8.4 | 1 | calculated | docs/02-form.md#wing-sizing |
| heavy.ar20_stiff_caps_factor_ud | 1.33 | 1 | calculated | docs/02-form.md#wing-sizing |
| heavy.ar20_tube_free_over_caps_min | 8.9 | 1 | calculated | docs/02-form.md#wing-sizing |
| heavy.ar20_tube_free_over_caps_max | 10.3 | 1 | calculated | docs/02-form.md#wing-sizing |
| skim.ar20l_std_u13_h90_ms | 11.88 | m/s | calculated | docs/02-form.md#wingtip-skim |
| skim.ar20l_std_u13_h135_ms | 0.45 | m/s | calculated | docs/02-form.md#wingtip-skim |
| skim.ar20l_std_u13_h180_ms | -10.27 | m/s | calculated | docs/02-form.md#wingtip-skim |
| skim.ar20l_skid_u13_h90_ms | 15.83 | m/s | calculated | docs/02-form.md#wingtip-skim |
| skim.ar20l_skid_u13_h135_ms | 4.82 | m/s | calculated | docs/02-form.md#wingtip-skim |
| skim.ar20l_skid_u13_h180_ms | -4.98 | m/s | calculated | docs/02-form.md#wingtip-skim |
| skim.ar20l_skiddih10_u13_h90_ms | 16.62 | m/s | calculated | docs/02-form.md#wingtip-skim |
| skim.ar20l_skiddih10_u13_h135_ms | 5.71 | m/s | calculated | docs/02-form.md#wingtip-skim |
| skim.ar20l_skiddih10_u13_h180_ms | -4.27 | m/s | calculated | docs/02-form.md#wingtip-skim |
| skim.ar20l_std_u16_h90_ms | 18.15 | m/s | calculated | docs/02-form.md#wingtip-skim |
| skim.ar20l_std_u16_h135_ms | 4.59 | m/s | calculated | docs/02-form.md#wingtip-skim |
| skim.ar20l_std_u16_h180_ms | -6.15 | m/s | calculated | docs/02-form.md#wingtip-skim |
| skim.ar20l_skid_u16_h90_ms | 20.7 | m/s | calculated | docs/02-form.md#wingtip-skim |
| skim.ar20l_skid_u16_h135_ms | 7.68 | m/s | calculated | docs/02-form.md#wingtip-skim |
| skim.ar20l_skid_u16_h180_ms | -3.51 | m/s | calculated | docs/02-form.md#wingtip-skim |
| skim.ar20l_skiddih10_u16_h90_ms | 21.53 | m/s | calculated | docs/02-form.md#wingtip-skim |
| skim.ar20l_skiddih10_u16_h135_ms | 8.44 | m/s | calculated | docs/02-form.md#wingtip-skim |
| skim.ar20l_skiddih10_u16_h180_ms | -2.89 | m/s | calculated | docs/02-form.md#wingtip-skim |
| skim.alb_std_u13_h90_ms | 17.45 | m/s | calculated | docs/02-form.md#wingtip-skim |
| skim.alb_std_u13_h135_ms | 6.28 | m/s | calculated | docs/02-form.md#wingtip-skim |
| skim.alb_std_u13_h180_ms | -2.98 | m/s | calculated | docs/02-form.md#wingtip-skim |
| skim.alb_skid_u13_h90_ms | 19.76 | m/s | calculated | docs/02-form.md#wingtip-skim |
| skim.alb_skid_u13_h135_ms | 8.91 | m/s | calculated | docs/02-form.md#wingtip-skim |
| skim.alb_skid_u13_h180_ms | -0.66 | m/s | calculated | docs/02-form.md#wingtip-skim |
| skim.alb_skiddih10_u13_h90_ms | 20.35 | m/s | calculated | docs/02-form.md#wingtip-skim |
| skim.alb_skiddih10_u13_h135_ms | 9.45 | m/s | calculated | docs/02-form.md#wingtip-skim |
| skim.alb_skiddih10_u13_h180_ms | -0.2 | m/s | calculated | docs/02-form.md#wingtip-skim |
| skim.alb_std_u16_h90_ms | 21.53 | m/s | calculated | docs/02-form.md#wingtip-skim |
| skim.alb_std_u16_h135_ms | 8.16 | m/s | calculated | docs/02-form.md#wingtip-skim |
| skim.alb_std_u16_h180_ms | -2.49 | m/s | calculated | docs/02-form.md#wingtip-skim |
| skim.alb_skid_u16_h90_ms | 23.62 | m/s | calculated | docs/02-form.md#wingtip-skim |
| skim.alb_skid_u16_h135_ms | 10.6 | m/s | calculated | docs/02-form.md#wingtip-skim |
| skim.alb_skid_u16_h180_ms | -0.3 | m/s | calculated | docs/02-form.md#wingtip-skim |
| skim.alb_skiddih10_u16_h90_ms | 24.24 | m/s | calculated | docs/02-form.md#wingtip-skim |
| skim.alb_skiddih10_u16_h135_ms | 11.24 | m/s | calculated | docs/02-form.md#wingtip-skim |
| skim.alb_skiddih10_u16_h180_ms | 0.16 | m/s | calculated | docs/02-form.md#wingtip-skim |
| skim.ar20l_skid_u13_drift_cut | 0.52 | 1 | calculated | docs/02-form.md#wingtip-skim |
| skim.ar20l_skiddih10_u13_drift_cut | 0.58 | 1 | calculated | docs/02-form.md#wingtip-skim |
| skim.ar20l_skid_u16_drift_cut | 0.43 | 1 | calculated | docs/02-form.md#wingtip-skim |
| skim.ar20l_skiddih10_u16_drift_cut | 0.53 | 1 | calculated | docs/02-form.md#wingtip-skim |
| skim.alb_skid_u13_drift_cut | 0.78 | 1 | calculated | docs/02-form.md#wingtip-skim |
| skim.alb_skiddih10_u13_drift_cut | 0.93 | 1 | calculated | docs/02-form.md#wingtip-skim |
| skim.alb_skid_u16_drift_cut | 0.88 | 1 | calculated | docs/02-form.md#wingtip-skim |
| skim.alb_skiddih10_u16_drift_cut | 1.0 | 1 | calculated | docs/02-form.md#wingtip-skim |
| skim.alb_skiddih10_u16_h180_start_spread_ms | 0.06 | m/s | calculated | K-018 |
| skim.alb_skiddih10_u16_h180_tip15_ms | 0.1 | m/s | calculated | K-018 |
| climate.so_djf_u10_mean_ms | 10.06 | m/s | calculated | docs/02-form.md#wind-climate |
| climate.so_djf_u10_p10_ms | 5.33 | m/s | calculated | docs/02-form.md#wind-climate |
| climate.so_djf_u10_p50_ms | 10.14 | m/s | calculated | docs/02-form.md#wind-climate |
| climate.so_djf_u10_p90_ms | 14.56 | m/s | calculated | docs/02-form.md#wind-climate |
| climate.so_djf_below_u6_share | 0.133 | 1 | calculated | docs/02-form.md#wind-climate |
| climate.so_djf_below_dsmin_share | 0.281 | 1 | calculated | docs/02-form.md#wind-climate |
| climate.so_djf_below_skim_share | 0.589 | 1 | calculated | docs/02-form.md#wind-climate |
| climate.so_djf_below_best_share | 0.779 | 1 | calculated | docs/02-form.md#wind-climate |
| climate.so_mam_u10_mean_ms | 10.94 | m/s | calculated | docs/02-form.md#wind-climate |
| climate.so_mam_u10_p10_ms | 5.89 | m/s | calculated | docs/02-form.md#wind-climate |
| climate.so_mam_u10_p50_ms | 11.04 | m/s | calculated | docs/02-form.md#wind-climate |
| climate.so_mam_u10_p90_ms | 15.76 | m/s | calculated | docs/02-form.md#wind-climate |
| climate.so_mam_below_u6_share | 0.105 | 1 | calculated | docs/02-form.md#wind-climate |
| climate.so_mam_below_dsmin_share | 0.22 | 1 | calculated | docs/02-form.md#wind-climate |
| climate.so_mam_below_skim_share | 0.493 | 1 | calculated | docs/02-form.md#wind-climate |
| climate.so_mam_below_best_share | 0.683 | 1 | calculated | docs/02-form.md#wind-climate |
| climate.so_jja_u10_mean_ms | 11.38 | m/s | calculated | docs/02-form.md#wind-climate |
| climate.so_jja_u10_p10_ms | 6.01 | m/s | calculated | docs/02-form.md#wind-climate |
| climate.so_jja_u10_p50_ms | 11.59 | m/s | calculated | docs/02-form.md#wind-climate |
| climate.so_jja_u10_p90_ms | 16.35 | m/s | calculated | docs/02-form.md#wind-climate |
| climate.so_jja_below_u6_share | 0.099 | 1 | calculated | docs/02-form.md#wind-climate |
| climate.so_jja_below_dsmin_share | 0.203 | 1 | calculated | docs/02-form.md#wind-climate |
| climate.so_jja_below_skim_share | 0.443 | 1 | calculated | docs/02-form.md#wind-climate |
| climate.so_jja_below_best_share | 0.623 | 1 | calculated | docs/02-form.md#wind-climate |
| climate.so_son_u10_mean_ms | 11.15 | m/s | calculated | docs/02-form.md#wind-climate |
| climate.so_son_u10_p10_ms | 6.15 | m/s | calculated | docs/02-form.md#wind-climate |
| climate.so_son_u10_p50_ms | 11.28 | m/s | calculated | docs/02-form.md#wind-climate |
| climate.so_son_u10_p90_ms | 15.9 | m/s | calculated | docs/02-form.md#wind-climate |
| climate.so_son_below_u6_share | 0.094 | 1 | calculated | docs/02-form.md#wind-climate |
| climate.so_son_below_dsmin_share | 0.204 | 1 | calculated | docs/02-form.md#wind-climate |
| climate.so_son_below_skim_share | 0.469 | 1 | calculated | docs/02-form.md#wind-climate |
| climate.so_son_below_best_share | 0.661 | 1 | calculated | docs/02-form.md#wind-climate |
| climate.so_year_u10_mean_ms | 10.88 | m/s | calculated | docs/02-form.md#wind-climate |
| climate.so_year_u10_p10_ms | 5.82 | m/s | calculated | docs/02-form.md#wind-climate |
| climate.so_year_u10_p50_ms | 10.99 | m/s | calculated | docs/02-form.md#wind-climate |
| climate.so_year_u10_p90_ms | 15.72 | m/s | calculated | docs/02-form.md#wind-climate |
| climate.so_year_below_u6_share | 0.108 | 1 | calculated | docs/02-form.md#wind-climate |
| climate.so_year_below_dsmin_share | 0.227 | 1 | calculated | docs/02-form.md#wind-climate |
| climate.so_year_below_skim_share | 0.498 | 1 | calculated | docs/02-form.md#wind-climate |
| climate.so_year_below_best_share | 0.686 | 1 | calculated | docs/02-form.md#wind-climate |
| climate.so_year_u10_std_ms | 3.79 | m/s | calculated | docs/02-form.md#wind-climate |
| climate.so_spell_u6_p50_h | 6 | h | calculated | docs/02-form.md#wind-climate |
| climate.so_spell_u6_p90_h | 20 | h | calculated | docs/02-form.md#wind-climate |
| climate.so_spell_u6_p99_h | 46 | h | calculated | docs/02-form.md#wind-climate |
| climate.so_spell_u6_max_h | 93 | h | calculated | docs/02-form.md#wind-climate |
| climate.so_spell_u6_ge24h_share | 0.271 | 1 | calculated | docs/02-form.md#wind-climate |
| climate.so_spell_u6_per_year_over_p99 | 1.07 | 1/year | calculated | docs/02-form.md#wind-climate |
| climate.so_spell_u6_hw_p99_h | 78 | h | calculated | docs/02-form.md#wind-climate |
| climate.so_spell_dsmin_p50_h | 7 | h | calculated | docs/02-form.md#wind-climate |
| climate.so_spell_dsmin_p90_h | 28 | h | calculated | docs/02-form.md#wind-climate |
| climate.so_spell_dsmin_p99_h | 71 | h | calculated | docs/02-form.md#wind-climate |
| climate.so_spell_dsmin_max_h | 169 | h | calculated | docs/02-form.md#wind-climate |
| climate.so_spell_dsmin_ge24h_share | 0.459 | 1 | calculated | docs/02-form.md#wind-climate |
| climate.so_spell_dsmin_per_year_over_p99 | 1.58 | 1/year | calculated | docs/02-form.md#wind-climate |
| climate.so_spell_dsmin_hw_p99_h | 119 | h | calculated | docs/02-form.md#wind-climate |
| climate.so_spell_skim_p50_h | 10 | h | calculated | docs/02-form.md#wind-climate |
| climate.so_spell_skim_p90_h | 48 | h | calculated | docs/02-form.md#wind-climate |
| climate.so_spell_skim_p99_h | 135 | h | calculated | docs/02-form.md#wind-climate |
| climate.so_spell_skim_max_h | 361 | h | calculated | docs/02-form.md#wind-climate |
| climate.so_spell_skim_ge24h_share | 0.701 | 1 | calculated | docs/02-form.md#wind-climate |
| climate.so_spell_skim_per_year_over_p99 | 2.25 | 1/year | calculated | docs/02-form.md#wind-climate |
| climate.so_spell_skim_hw_p99_h | 250 | h | calculated | docs/02-form.md#wind-climate |
| climate.so_spell_best_p50_h | 13 | h | calculated | docs/02-form.md#wind-climate |
| climate.so_spell_best_p90_h | 72 | h | calculated | docs/02-form.md#wind-climate |
| climate.so_spell_best_p99_h | 215 | h | calculated | docs/02-form.md#wind-climate |
| climate.so_spell_best_max_h | 532 | h | calculated | docs/02-form.md#wind-climate |
| climate.so_spell_best_ge24h_share | 0.82 | 1 | calculated | docs/02-form.md#wind-climate |
| climate.so_spell_best_per_year_over_p99 | 2.13 | 1/year | calculated | docs/02-form.md#wind-climate |
| climate.so_spell_best_hw_p99_h | 425 | h | calculated | docs/02-form.md#wind-climate |
| climate.so_year_hs_survival_share | 0.424 | 1 | calculated | docs/02-form.md#wind-climate |
| climate.so_weak_u6_hs2_share | 0.88 | 1 | calculated | docs/02-form.md#wind-climate |
| climate.so_weak_u6_hs3_share | 0.374 | 1 | calculated | docs/02-form.md#wind-climate |
| climate.so_weak_u6_swell2_share | 0.848 | 1 | calculated | docs/02-form.md#wind-climate |
| climate.so_weak_u6_swell3_share | 0.351 | 1 | calculated | docs/02-form.md#wind-climate |
| climate.so_weak_u6_hs_survival_share | 0.092 | 1 | calculated | docs/02-form.md#wind-climate |
| climate.so_weak_u6_tp_s | 11.8 | s | calculated | docs/02-form.md#wind-climate |
| climate.so_weak_dsmin_hs2_share | 0.902 | 1 | calculated | docs/02-form.md#wind-climate |
| climate.so_weak_dsmin_hs3_share | 0.413 | 1 | calculated | docs/02-form.md#wind-climate |
| climate.so_weak_dsmin_swell2_share | 0.816 | 1 | calculated | docs/02-form.md#wind-climate |
| climate.so_weak_dsmin_swell3_share | 0.344 | 1 | calculated | docs/02-form.md#wind-climate |
| climate.so_weak_dsmin_hs_survival_share | 0.105 | 1 | calculated | docs/02-form.md#wind-climate |
| climate.so_weak_dsmin_tp_s | 11.8 | s | calculated | docs/02-form.md#wind-climate |
| climate.so_weak_skim_hs2_share | 0.944 | 1 | calculated | docs/02-form.md#wind-climate |
| climate.so_weak_skim_hs3_share | 0.536 | 1 | calculated | docs/02-form.md#wind-climate |
| climate.so_weak_skim_swell2_share | 0.719 | 1 | calculated | docs/02-form.md#wind-climate |
| climate.so_weak_skim_swell3_share | 0.316 | 1 | calculated | docs/02-form.md#wind-climate |
| climate.so_weak_skim_hs_survival_share | 0.166 | 1 | calculated | docs/02-form.md#wind-climate |
| climate.so_weak_skim_tp_s | 11.8 | s | calculated | docs/02-form.md#wind-climate |
| climate.so_weak_best_hs2_share | 0.959 | 1 | calculated | docs/02-form.md#wind-climate |
| climate.so_weak_best_hs3_share | 0.627 | 1 | calculated | docs/02-form.md#wind-climate |
| climate.so_weak_best_swell2_share | 0.64 | 1 | calculated | docs/02-form.md#wind-climate |
| climate.so_weak_best_swell3_share | 0.288 | 1 | calculated | docs/02-form.md#wind-climate |
| climate.so_weak_best_hs_survival_share | 0.238 | 1 | calculated | docs/02-form.md#wind-climate |
| climate.so_weak_best_tp_s | 11.9 | s | calculated | docs/02-form.md#wind-climate |
| climate.so_albatross_motor_p50_wh | 144 | Wh | calculated | docs/02-form.md#wind-climate |
| climate.so_albatross_motor_p90_wh | 575 | Wh | calculated | docs/02-form.md#wind-climate |
| climate.so_albatross_motor_p99_wh | 1459 | Wh | calculated | docs/02-form.md#wind-climate |
| climate.so_albatross_float_p50_wh | 21 | Wh | calculated | docs/02-form.md#wind-climate |
| climate.so_albatross_float_p90_wh | 84 | Wh | calculated | docs/02-form.md#wind-climate |
| climate.so_albatross_float_p99_wh | 213 | Wh | calculated | docs/02-form.md#wind-climate |
| climate.so_albatross_motor_p99_cell_kg | 6.32 | kg | calculated | docs/02-form.md#wind-climate |
| climate.so_albatross_float_p99_cell_kg | 0.92 | kg | calculated | docs/02-form.md#wind-climate |
| climate.so_best_motor_p50_wh | 433 | Wh | calculated | docs/02-form.md#wind-climate |
| climate.so_best_motor_p90_wh | 2398 | Wh | calculated | docs/02-form.md#wind-climate |
| climate.so_best_motor_p99_wh | 7161 | Wh | calculated | docs/02-form.md#wind-climate |
| climate.so_best_float_p50_wh | 39 | Wh | calculated | docs/02-form.md#wind-climate |
| climate.so_best_float_p90_wh | 216 | Wh | calculated | docs/02-form.md#wind-climate |
| climate.so_best_float_p99_wh | 645 | Wh | calculated | docs/02-form.md#wind-climate |
| climate.so_best_motor_p99_cell_kg | 31.0 | kg | calculated | docs/02-form.md#wind-climate |
| climate.so_best_float_p99_cell_kg | 2.79 | kg | calculated | docs/02-form.md#wind-climate |
| climate.so_djf_albatross_motor_p90_wh | 658 | Wh | calculated | docs/02-form.md#wind-climate |
| climate.so_djf_albatross_motor_p99_wh | 1603 | Wh | calculated | docs/02-form.md#wind-climate |
| climate.so_djf_albatross_float_p90_wh | 96 | Wh | calculated | docs/02-form.md#wind-climate |
| climate.so_djf_albatross_float_p99_wh | 234 | Wh | calculated | docs/02-form.md#wind-climate |
| climate.so_djf_best_motor_p90_wh | 3293 | Wh | calculated | docs/02-form.md#wind-climate |
| climate.so_djf_best_motor_p99_wh | 9257 | Wh | calculated | docs/02-form.md#wind-climate |
| climate.so_djf_best_float_p90_wh | 297 | Wh | calculated | docs/02-form.md#wind-climate |
| climate.so_djf_best_float_p99_wh | 834 | Wh | calculated | docs/02-form.md#wind-climate |
| climate.so_mam_albatross_motor_p90_wh | 575 | Wh | calculated | docs/02-form.md#wind-climate |
| climate.so_mam_albatross_motor_p99_wh | 1378 | Wh | calculated | docs/02-form.md#wind-climate |
| climate.so_mam_albatross_float_p90_wh | 84 | Wh | calculated | docs/02-form.md#wind-climate |
| climate.so_mam_albatross_float_p99_wh | 201 | Wh | calculated | docs/02-form.md#wind-climate |
| climate.so_mam_best_motor_p90_wh | 2364 | Wh | calculated | docs/02-form.md#wind-climate |
| climate.so_mam_best_motor_p99_wh | 7074 | Wh | calculated | docs/02-form.md#wind-climate |
| climate.so_mam_best_float_p90_wh | 213 | Wh | calculated | docs/02-form.md#wind-climate |
| climate.so_mam_best_float_p99_wh | 637 | Wh | calculated | docs/02-form.md#wind-climate |
| climate.so_jja_albatross_motor_p90_wh | 534 | Wh | calculated | docs/02-form.md#wind-climate |
| climate.so_jja_albatross_motor_p99_wh | 1496 | Wh | calculated | docs/02-form.md#wind-climate |
| climate.so_jja_albatross_float_p90_wh | 78 | Wh | calculated | docs/02-form.md#wind-climate |
| climate.so_jja_albatross_float_p99_wh | 218 | Wh | calculated | docs/02-form.md#wind-climate |
| climate.so_jja_best_motor_p90_wh | 1865 | Wh | calculated | docs/02-form.md#wind-climate |
| climate.so_jja_best_motor_p99_wh | 5461 | Wh | calculated | docs/02-form.md#wind-climate |
| climate.so_jja_best_float_p90_wh | 168 | Wh | calculated | docs/02-form.md#wind-climate |
| climate.so_jja_best_float_p99_wh | 492 | Wh | calculated | docs/02-form.md#wind-climate |
| climate.so_son_albatross_motor_p90_wh | 555 | Wh | calculated | docs/02-form.md#wind-climate |
| climate.so_son_albatross_motor_p99_wh | 1356 | Wh | calculated | docs/02-form.md#wind-climate |
| climate.so_son_albatross_float_p90_wh | 81 | Wh | calculated | docs/02-form.md#wind-climate |
| climate.so_son_albatross_float_p99_wh | 198 | Wh | calculated | docs/02-form.md#wind-climate |
| climate.so_son_best_motor_p90_wh | 2198 | Wh | calculated | docs/02-form.md#wind-climate |
| climate.so_son_best_motor_p99_wh | 6327 | Wh | calculated | docs/02-form.md#wind-climate |
| climate.so_son_best_float_p90_wh | 198 | Wh | calculated | docs/02-form.md#wind-climate |
| climate.so_son_best_float_p99_wh | 570 | Wh | calculated | docs/02-form.md#wind-climate |
| climate.np_djf_u10_mean_ms | 10.33 | m/s | calculated | docs/02-form.md#wind-climate |
| climate.np_djf_u10_p10_ms | 4.85 | m/s | calculated | docs/02-form.md#wind-climate |
| climate.np_djf_u10_p50_ms | 10.24 | m/s | calculated | docs/02-form.md#wind-climate |
| climate.np_djf_u10_p90_ms | 15.8 | m/s | calculated | docs/02-form.md#wind-climate |
| climate.np_djf_below_u6_share | 0.16 | 1 | calculated | docs/02-form.md#wind-climate |
| climate.np_djf_below_dsmin_share | 0.303 | 1 | calculated | docs/02-form.md#wind-climate |
| climate.np_djf_below_skim_share | 0.565 | 1 | calculated | docs/02-form.md#wind-climate |
| climate.np_djf_below_best_share | 0.72 | 1 | calculated | docs/02-form.md#wind-climate |
| climate.np_mam_u10_mean_ms | 8.32 | m/s | calculated | docs/02-form.md#wind-climate |
| climate.np_mam_u10_p10_ms | 3.64 | m/s | calculated | docs/02-form.md#wind-climate |
| climate.np_mam_u10_p50_ms | 8.04 | m/s | calculated | docs/02-form.md#wind-climate |
| climate.np_mam_u10_p90_ms | 13.33 | m/s | calculated | docs/02-form.md#wind-climate |
| climate.np_mam_below_u6_share | 0.287 | 1 | calculated | docs/02-form.md#wind-climate |
| climate.np_mam_below_dsmin_share | 0.495 | 1 | calculated | docs/02-form.md#wind-climate |
| climate.np_mam_below_skim_share | 0.768 | 1 | calculated | docs/02-form.md#wind-climate |
| climate.np_mam_below_best_share | 0.88 | 1 | calculated | docs/02-form.md#wind-climate |
| climate.np_jja_u10_mean_ms | 6.09 | m/s | calculated | docs/02-form.md#wind-climate |
| climate.np_jja_u10_p10_ms | 2.62 | m/s | calculated | docs/02-form.md#wind-climate |
| climate.np_jja_u10_p50_ms | 5.89 | m/s | calculated | docs/02-form.md#wind-climate |
| climate.np_jja_u10_p90_ms | 9.82 | m/s | calculated | docs/02-form.md#wind-climate |
| climate.np_jja_below_u6_share | 0.516 | 1 | calculated | docs/02-form.md#wind-climate |
| climate.np_jja_below_dsmin_share | 0.772 | 1 | calculated | docs/02-form.md#wind-climate |
| climate.np_jja_below_skim_share | 0.947 | 1 | calculated | docs/02-form.md#wind-climate |
| climate.np_jja_below_best_share | 0.984 | 1 | calculated | docs/02-form.md#wind-climate |
| climate.np_son_u10_mean_ms | 8.21 | m/s | calculated | docs/02-form.md#wind-climate |
| climate.np_son_u10_p10_ms | 3.48 | m/s | calculated | docs/02-form.md#wind-climate |
| climate.np_son_u10_p50_ms | 7.92 | m/s | calculated | docs/02-form.md#wind-climate |
| climate.np_son_u10_p90_ms | 13.31 | m/s | calculated | docs/02-form.md#wind-climate |
| climate.np_son_below_u6_share | 0.301 | 1 | calculated | docs/02-form.md#wind-climate |
| climate.np_son_below_dsmin_share | 0.506 | 1 | calculated | docs/02-form.md#wind-climate |
| climate.np_son_below_skim_share | 0.773 | 1 | calculated | docs/02-form.md#wind-climate |
| climate.np_son_below_best_share | 0.881 | 1 | calculated | docs/02-form.md#wind-climate |
| climate.np_year_u10_mean_ms | 8.23 | m/s | calculated | docs/02-form.md#wind-climate |
| climate.np_year_u10_p10_ms | 3.4 | m/s | calculated | docs/02-form.md#wind-climate |
| climate.np_year_u10_p50_ms | 7.78 | m/s | calculated | docs/02-form.md#wind-climate |
| climate.np_year_u10_p90_ms | 13.66 | m/s | calculated | docs/02-form.md#wind-climate |
| climate.np_year_below_u6_share | 0.317 | 1 | calculated | docs/02-form.md#wind-climate |
| climate.np_year_below_dsmin_share | 0.52 | 1 | calculated | docs/02-form.md#wind-climate |
| climate.np_year_below_skim_share | 0.764 | 1 | calculated | docs/02-form.md#wind-climate |
| climate.np_year_below_best_share | 0.867 | 1 | calculated | docs/02-form.md#wind-climate |
| climate.np_year_u10_std_ms | 3.94 | m/s | calculated | docs/02-form.md#wind-climate |
| climate.np_spell_u6_p50_h | 7 | h | calculated | docs/02-form.md#wind-climate |
| climate.np_spell_u6_p90_h | 39 | h | calculated | docs/02-form.md#wind-climate |
| climate.np_spell_u6_p99_h | 131 | h | calculated | docs/02-form.md#wind-climate |
| climate.np_spell_u6_max_h | 366 | h | calculated | docs/02-form.md#wind-climate |
| climate.np_spell_u6_ge24h_share | 0.64 | 1 | calculated | docs/02-form.md#wind-climate |
| climate.np_spell_u6_per_year_over_p99 | 1.72 | 1/year | calculated | docs/02-form.md#wind-climate |
| climate.np_spell_u6_hw_p99_h | 304 | h | calculated | docs/02-form.md#wind-climate |
| climate.np_spell_dsmin_p50_h | 10 | h | calculated | docs/02-form.md#wind-climate |
| climate.np_spell_dsmin_p90_h | 62 | h | calculated | docs/02-form.md#wind-climate |
| climate.np_spell_dsmin_p99_h | 246 | h | calculated | docs/02-form.md#wind-climate |
| climate.np_spell_dsmin_max_h | 1409 | h | calculated | docs/02-form.md#wind-climate |
| climate.np_spell_dsmin_ge24h_share | 0.79 | 1 | calculated | docs/02-form.md#wind-climate |
| climate.np_spell_dsmin_per_year_over_p99 | 1.72 | 1/year | calculated | docs/02-form.md#wind-climate |
| climate.np_spell_dsmin_hw_p99_h | 1034 | h | calculated | docs/02-form.md#wind-climate |
| climate.np_spell_skim_p50_h | 15 | h | calculated | docs/02-form.md#wind-climate |
| climate.np_spell_skim_p90_h | 104 | h | calculated | docs/02-form.md#wind-climate |
| climate.np_spell_skim_p99_h | 553 | h | calculated | docs/02-form.md#wind-climate |
| climate.np_spell_skim_max_h | 3076 | h | calculated | docs/02-form.md#wind-climate |
| climate.np_spell_skim_ge24h_share | 0.904 | 1 | calculated | docs/02-form.md#wind-climate |
| climate.np_spell_skim_per_year_over_p99 | 1.4 | 1/year | calculated | docs/02-form.md#wind-climate |
| climate.np_spell_skim_hw_p99_h | 2667 | h | calculated | docs/02-form.md#wind-climate |
| climate.np_spell_best_p50_h | 18 | h | calculated | docs/02-form.md#wind-climate |
| climate.np_spell_best_p90_h | 152 | h | calculated | docs/02-form.md#wind-climate |
| climate.np_spell_best_p99_h | 821 | h | calculated | docs/02-form.md#wind-climate |
| climate.np_spell_best_max_h | 4419 | h | calculated | docs/02-form.md#wind-climate |
| climate.np_spell_best_ge24h_share | 0.942 | 1 | calculated | docs/02-form.md#wind-climate |
| climate.np_spell_best_per_year_over_p99 | 1.07 | 1/year | calculated | docs/02-form.md#wind-climate |
| climate.np_spell_best_hw_p99_h | 4155 | h | calculated | docs/02-form.md#wind-climate |
| climate.np_year_hs_survival_share | 0.175 | 1 | calculated | docs/02-form.md#wind-climate |
| climate.np_weak_u6_hs2_share | 0.348 | 1 | calculated | docs/02-form.md#wind-climate |
| climate.np_weak_u6_hs3_share | 0.11 | 1 | calculated | docs/02-form.md#wind-climate |
| climate.np_weak_u6_swell2_share | 0.322 | 1 | calculated | docs/02-form.md#wind-climate |
| climate.np_weak_u6_swell3_share | 0.103 | 1 | calculated | docs/02-form.md#wind-climate |
| climate.np_weak_u6_hs_survival_share | 0.028 | 1 | calculated | docs/02-form.md#wind-climate |
| climate.np_weak_u6_tp_s | 11.6 | s | calculated | docs/02-form.md#wind-climate |
| climate.np_weak_dsmin_hs2_share | 0.409 | 1 | calculated | docs/02-form.md#wind-climate |
| climate.np_weak_dsmin_hs3_share | 0.135 | 1 | calculated | docs/02-form.md#wind-climate |
| climate.np_weak_dsmin_swell2_share | 0.336 | 1 | calculated | docs/02-form.md#wind-climate |
| climate.np_weak_dsmin_swell3_share | 0.114 | 1 | calculated | docs/02-form.md#wind-climate |
| climate.np_weak_dsmin_hs_survival_share | 0.036 | 1 | calculated | docs/02-form.md#wind-climate |
| climate.np_weak_dsmin_tp_s | 11.6 | s | calculated | docs/02-form.md#wind-climate |
| climate.np_weak_skim_hs2_share | 0.534 | 1 | calculated | docs/02-form.md#wind-climate |
| climate.np_weak_skim_hs3_share | 0.207 | 1 | calculated | docs/02-form.md#wind-climate |
| climate.np_weak_skim_swell2_share | 0.347 | 1 | calculated | docs/02-form.md#wind-climate |
| climate.np_weak_skim_swell3_share | 0.128 | 1 | calculated | docs/02-form.md#wind-climate |
| climate.np_weak_skim_hs_survival_share | 0.062 | 1 | calculated | docs/02-form.md#wind-climate |
| climate.np_weak_skim_tp_s | 11.7 | s | calculated | docs/02-form.md#wind-climate |
| climate.np_weak_best_hs2_share | 0.588 | 1 | calculated | docs/02-form.md#wind-climate |
| climate.np_weak_best_hs3_share | 0.263 | 1 | calculated | docs/02-form.md#wind-climate |
| climate.np_weak_best_swell2_share | 0.341 | 1 | calculated | docs/02-form.md#wind-climate |
| climate.np_weak_best_swell3_share | 0.131 | 1 | calculated | docs/02-form.md#wind-climate |
| climate.np_weak_best_hs_survival_share | 0.089 | 1 | calculated | docs/02-form.md#wind-climate |
| climate.np_weak_best_tp_s | 11.8 | s | calculated | docs/02-form.md#wind-climate |
| climate.np_albatross_motor_p50_wh | 206 | Wh | calculated | docs/02-form.md#wind-climate |
| climate.np_albatross_motor_p90_wh | 1274 | Wh | calculated | docs/02-form.md#wind-climate |
| climate.np_albatross_motor_p99_wh | 5055 | Wh | calculated | docs/02-form.md#wind-climate |
| climate.np_albatross_float_p50_wh | 30 | Wh | calculated | docs/02-form.md#wind-climate |
| climate.np_albatross_float_p90_wh | 186 | Wh | calculated | docs/02-form.md#wind-climate |
| climate.np_albatross_float_p99_wh | 738 | Wh | calculated | docs/02-form.md#wind-climate |
| climate.np_albatross_motor_p99_cell_kg | 21.88 | kg | calculated | docs/02-form.md#wind-climate |
| climate.np_albatross_float_p99_cell_kg | 3.19 | kg | calculated | docs/02-form.md#wind-climate |
| climate.np_best_motor_p50_wh | 599 | Wh | calculated | docs/02-form.md#wind-climate |
| climate.np_best_motor_p90_wh | 5062 | Wh | calculated | docs/02-form.md#wind-climate |
| climate.np_best_motor_p99_wh | 27327 | Wh | calculated | docs/02-form.md#wind-climate |
| climate.np_best_float_p50_wh | 54 | Wh | calculated | docs/02-form.md#wind-climate |
| climate.np_best_float_p90_wh | 456 | Wh | calculated | docs/02-form.md#wind-climate |
| climate.np_best_float_p99_wh | 2462 | Wh | calculated | docs/02-form.md#wind-climate |
| climate.np_best_motor_p99_cell_kg | 118.3 | kg | calculated | docs/02-form.md#wind-climate |
| climate.np_best_float_p99_cell_kg | 10.66 | kg | calculated | docs/02-form.md#wind-climate |
| climate.np_djf_albatross_motor_p90_wh | 637 | Wh | calculated | docs/02-form.md#wind-climate |
| climate.np_djf_albatross_motor_p99_wh | 1837 | Wh | calculated | docs/02-form.md#wind-climate |
| climate.np_djf_albatross_float_p90_wh | 93 | Wh | calculated | docs/02-form.md#wind-climate |
| climate.np_djf_albatross_float_p99_wh | 268 | Wh | calculated | docs/02-form.md#wind-climate |
| climate.np_djf_best_motor_p90_wh | 2631 | Wh | calculated | docs/02-form.md#wind-climate |
| climate.np_djf_best_motor_p99_wh | 9894 | Wh | calculated | docs/02-form.md#wind-climate |
| climate.np_djf_best_float_p90_wh | 237 | Wh | calculated | docs/02-form.md#wind-climate |
| climate.np_djf_best_float_p99_wh | 891 | Wh | calculated | docs/02-form.md#wind-climate |
| climate.np_mam_albatross_motor_p90_wh | 1212 | Wh | calculated | docs/02-form.md#wind-climate |
| climate.np_mam_albatross_motor_p99_wh | 4155 | Wh | calculated | docs/02-form.md#wind-climate |
| climate.np_mam_albatross_float_p90_wh | 177 | Wh | calculated | docs/02-form.md#wind-climate |
| climate.np_mam_albatross_float_p99_wh | 607 | Wh | calculated | docs/02-form.md#wind-climate |
| climate.np_mam_best_motor_p90_wh | 6803 | Wh | calculated | docs/02-form.md#wind-climate |
| climate.np_mam_best_motor_p99_wh | 70920 | Wh | calculated | docs/02-form.md#wind-climate |
| climate.np_mam_best_float_p90_wh | 613 | Wh | calculated | docs/02-form.md#wind-climate |
| climate.np_mam_best_float_p99_wh | 6389 | Wh | calculated | docs/02-form.md#wind-climate |
| climate.np_jja_albatross_motor_p90_wh | 3119 | Wh | calculated | docs/02-form.md#wind-climate |
| climate.np_jja_albatross_motor_p99_wh | 8705 | Wh | calculated | docs/02-form.md#wind-climate |
| climate.np_jja_albatross_float_p90_wh | 455 | Wh | calculated | docs/02-form.md#wind-climate |
| climate.np_jja_albatross_float_p99_wh | 1271 | Wh | calculated | docs/02-form.md#wind-climate |
| climate.np_jja_best_motor_p90_wh | 27156 | Wh | calculated | docs/02-form.md#wind-climate |
| climate.np_jja_best_motor_p99_wh | 75784 | Wh | calculated | docs/02-form.md#wind-climate |
| climate.np_jja_best_float_p90_wh | 2446 | Wh | calculated | docs/02-form.md#wind-climate |
| climate.np_jja_best_float_p99_wh | 6827 | Wh | calculated | docs/02-form.md#wind-climate |
| climate.np_son_albatross_motor_p90_wh | 1110 | Wh | calculated | docs/02-form.md#wind-climate |
| climate.np_son_albatross_motor_p99_wh | 3505 | Wh | calculated | docs/02-form.md#wind-climate |
| climate.np_son_albatross_float_p90_wh | 162 | Wh | calculated | docs/02-form.md#wind-climate |
| climate.np_son_albatross_float_p99_wh | 512 | Wh | calculated | docs/02-form.md#wind-climate |
| climate.np_son_best_motor_p90_wh | 5415 | Wh | calculated | docs/02-form.md#wind-climate |
| climate.np_son_best_motor_p99_wh | 16501 | Wh | calculated | docs/02-form.md#wind-climate |
| climate.np_son_best_float_p90_wh | 488 | Wh | calculated | docs/02-form.md#wind-climate |
| climate.np_son_best_float_p99_wh | 1487 | Wh | calculated | docs/02-form.md#wind-climate |
| climate.home_djf_u10_mean_ms | 7.62 | m/s | calculated | docs/02-form.md#wind-climate, private:loc-01 |
| climate.home_djf_u10_p10_ms | 3.11 | m/s | calculated | docs/02-form.md#wind-climate, private:loc-01 |
| climate.home_djf_u10_p50_ms | 7.45 | m/s | calculated | docs/02-form.md#wind-climate, private:loc-01 |
| climate.home_djf_u10_p90_ms | 12.31 | m/s | calculated | docs/02-form.md#wind-climate, private:loc-01 |
| climate.home_djf_below_u6_share | 0.345 | 1 | calculated | docs/02-form.md#wind-climate, private:loc-01 |
| climate.home_djf_below_dsmin_share | 0.561 | 1 | calculated | docs/02-form.md#wind-climate, private:loc-01 |
| climate.home_djf_below_skim_share | 0.827 | 1 | calculated | docs/02-form.md#wind-climate, private:loc-01 |
| climate.home_djf_below_best_share | 0.923 | 1 | calculated | docs/02-form.md#wind-climate, private:loc-01 |
| climate.home_mam_u10_mean_ms | 6.82 | m/s | calculated | docs/02-form.md#wind-climate, private:loc-01 |
| climate.home_mam_u10_p10_ms | 2.77 | m/s | calculated | docs/02-form.md#wind-climate, private:loc-01 |
| climate.home_mam_u10_p50_ms | 6.58 | m/s | calculated | docs/02-form.md#wind-climate, private:loc-01 |
| climate.home_mam_u10_p90_ms | 11.11 | m/s | calculated | docs/02-form.md#wind-climate, private:loc-01 |
| climate.home_mam_below_u6_share | 0.433 | 1 | calculated | docs/02-form.md#wind-climate, private:loc-01 |
| climate.home_mam_below_dsmin_share | 0.657 | 1 | calculated | docs/02-form.md#wind-climate, private:loc-01 |
| climate.home_mam_below_skim_share | 0.893 | 1 | calculated | docs/02-form.md#wind-climate, private:loc-01 |
| climate.home_mam_below_best_share | 0.959 | 1 | calculated | docs/02-form.md#wind-climate, private:loc-01 |
| climate.home_jja_u10_mean_ms | 5.2 | m/s | calculated | docs/02-form.md#wind-climate, private:loc-01 |
| climate.home_jja_u10_p10_ms | 1.99 | m/s | calculated | docs/02-form.md#wind-climate, private:loc-01 |
| climate.home_jja_u10_p50_ms | 4.82 | m/s | calculated | docs/02-form.md#wind-climate, private:loc-01 |
| climate.home_jja_u10_p90_ms | 8.92 | m/s | calculated | docs/02-form.md#wind-climate, private:loc-01 |
| climate.home_jja_below_u6_share | 0.652 | 1 | calculated | docs/02-form.md#wind-climate, private:loc-01 |
| climate.home_jja_below_dsmin_share | 0.839 | 1 | calculated | docs/02-form.md#wind-climate, private:loc-01 |
| climate.home_jja_below_skim_share | 0.973 | 1 | calculated | docs/02-form.md#wind-climate, private:loc-01 |
| climate.home_jja_below_best_share | 0.992 | 1 | calculated | docs/02-form.md#wind-climate, private:loc-01 |
| climate.home_son_u10_mean_ms | 6.9 | m/s | calculated | docs/02-form.md#wind-climate, private:loc-01 |
| climate.home_son_u10_p10_ms | 2.77 | m/s | calculated | docs/02-form.md#wind-climate, private:loc-01 |
| climate.home_son_u10_p50_ms | 6.62 | m/s | calculated | docs/02-form.md#wind-climate, private:loc-01 |
| climate.home_son_u10_p90_ms | 11.34 | m/s | calculated | docs/02-form.md#wind-climate, private:loc-01 |
| climate.home_son_below_u6_share | 0.43 | 1 | calculated | docs/02-form.md#wind-climate, private:loc-01 |
| climate.home_son_below_dsmin_share | 0.649 | 1 | calculated | docs/02-form.md#wind-climate, private:loc-01 |
| climate.home_son_below_skim_share | 0.883 | 1 | calculated | docs/02-form.md#wind-climate, private:loc-01 |
| climate.home_son_below_best_share | 0.951 | 1 | calculated | docs/02-form.md#wind-climate, private:loc-01 |
| climate.home_year_u10_mean_ms | 6.63 | m/s | calculated | docs/02-form.md#wind-climate, private:loc-01 |
| climate.home_year_u10_p10_ms | 2.55 | m/s | calculated | docs/02-form.md#wind-climate, private:loc-01 |
| climate.home_year_u10_p50_ms | 6.3 | m/s | calculated | docs/02-form.md#wind-climate, private:loc-01 |
| climate.home_year_u10_p90_ms | 11.1 | m/s | calculated | docs/02-form.md#wind-climate, private:loc-01 |
| climate.home_year_below_u6_share | 0.466 | 1 | calculated | docs/02-form.md#wind-climate, private:loc-01 |
| climate.home_year_below_dsmin_share | 0.677 | 1 | calculated | docs/02-form.md#wind-climate, private:loc-01 |
| climate.home_year_below_skim_share | 0.895 | 1 | calculated | docs/02-form.md#wind-climate, private:loc-01 |
| climate.home_year_below_best_share | 0.956 | 1 | calculated | docs/02-form.md#wind-climate, private:loc-01 |
| climate.home_year_u10_std_ms | 3.34 | m/s | calculated | docs/02-form.md#wind-climate, private:loc-01 |
| climate.home_spell_u6_p50_h | 8 | h | calculated | docs/02-form.md#wind-climate, private:loc-01 |
| climate.home_spell_u6_p90_h | 40 | h | calculated | docs/02-form.md#wind-climate, private:loc-01 |
| climate.home_spell_u6_p99_h | 115 | h | calculated | docs/02-form.md#wind-climate, private:loc-01 |
| climate.home_spell_u6_max_h | 334 | h | calculated | docs/02-form.md#wind-climate, private:loc-01 |
| climate.home_spell_u6_ge24h_share | 0.624 | 1 | calculated | docs/02-form.md#wind-climate, private:loc-01 |
| climate.home_spell_u6_per_year_over_p99 | 2.48 | 1/year | calculated | docs/02-form.md#wind-climate, private:loc-01 |
| climate.home_spell_u6_hw_p99_h | 261 | h | calculated | docs/02-form.md#wind-climate, private:loc-01 |
| climate.home_spell_dsmin_p50_h | 11 | h | calculated | docs/02-form.md#wind-climate, private:loc-01 |
| climate.home_spell_dsmin_p90_h | 69 | h | calculated | docs/02-form.md#wind-climate, private:loc-01 |
| climate.home_spell_dsmin_p99_h | 212 | h | calculated | docs/02-form.md#wind-climate, private:loc-01 |
| climate.home_spell_dsmin_max_h | 799 | h | calculated | docs/02-form.md#wind-climate, private:loc-01 |
| climate.home_spell_dsmin_ge24h_share | 0.814 | 1 | calculated | docs/02-form.md#wind-climate, private:loc-01 |
| climate.home_spell_dsmin_per_year_over_p99 | 2.2 | 1/year | calculated | docs/02-form.md#wind-climate, private:loc-01 |
| climate.home_spell_dsmin_hw_p99_h | 606 | h | calculated | docs/02-form.md#wind-climate, private:loc-01 |
| climate.home_spell_skim_p50_h | 25 | h | calculated | docs/02-form.md#wind-climate, private:loc-01 |
| climate.home_spell_skim_p90_h | 187 | h | calculated | docs/02-form.md#wind-climate, private:loc-01 |
| climate.home_spell_skim_p99_h | 752 | h | calculated | docs/02-form.md#wind-climate, private:loc-01 |
| climate.home_spell_skim_max_h | 2389 | h | calculated | docs/02-form.md#wind-climate, private:loc-01 |
| climate.home_spell_skim_ge24h_share | 0.956 | 1 | calculated | docs/02-form.md#wind-climate, private:loc-01 |
| climate.home_spell_skim_per_year_over_p99 | 1.07 | 1/year | calculated | docs/02-form.md#wind-climate, private:loc-01 |
| climate.home_spell_skim_hw_p99_h | 2262 | h | calculated | docs/02-form.md#wind-climate, private:loc-01 |
| climate.home_spell_best_p50_h | 46 | h | calculated | docs/02-form.md#wind-climate, private:loc-01 |
| climate.home_spell_best_p90_h | 381 | h | calculated | docs/02-form.md#wind-climate, private:loc-01 |
| climate.home_spell_best_p99_h | 1432 | h | calculated | docs/02-form.md#wind-climate, private:loc-01 |
| climate.home_spell_best_max_h | 3910 | h | calculated | docs/02-form.md#wind-climate, private:loc-01 |
| climate.home_spell_best_ge24h_share | 0.985 | 1 | calculated | docs/02-form.md#wind-climate, private:loc-01 |
| climate.home_spell_best_per_year_over_p99 | 0.57 | 1/year | calculated | docs/02-form.md#wind-climate, private:loc-01 |
| climate.home_spell_best_hw_p99_h | 3910 | h | calculated | docs/02-form.md#wind-climate, private:loc-01 |
| climate.home_year_hs_survival_share | 0.014 | 1 | calculated | docs/02-form.md#wind-climate, private:loc-01 |
| climate.home_weak_u6_hs2_share | 0.063 | 1 | calculated | docs/02-form.md#wind-climate, private:loc-01 |
| climate.home_weak_u6_hs3_share | 0.007 | 1 | calculated | docs/02-form.md#wind-climate, private:loc-01 |
| climate.home_weak_u6_swell2_share | 0.054 | 1 | calculated | docs/02-form.md#wind-climate, private:loc-01 |
| climate.home_weak_u6_swell3_share | 0.006 | 1 | calculated | docs/02-form.md#wind-climate, private:loc-01 |
| climate.home_weak_u6_hs_survival_share | 0.001 | 1 | calculated | docs/02-form.md#wind-climate, private:loc-01 |
| climate.home_weak_u6_tp_s | 11.1 | s | calculated | docs/02-form.md#wind-climate, private:loc-01 |
| climate.home_weak_dsmin_hs2_share | 0.078 | 1 | calculated | docs/02-form.md#wind-climate, private:loc-01 |
| climate.home_weak_dsmin_hs3_share | 0.009 | 1 | calculated | docs/02-form.md#wind-climate, private:loc-01 |
| climate.home_weak_dsmin_swell2_share | 0.054 | 1 | calculated | docs/02-form.md#wind-climate, private:loc-01 |
| climate.home_weak_dsmin_swell3_share | 0.007 | 1 | calculated | docs/02-form.md#wind-climate, private:loc-01 |
| climate.home_weak_dsmin_hs_survival_share | 0.001 | 1 | calculated | docs/02-form.md#wind-climate, private:loc-01 |
| climate.home_weak_dsmin_tp_s | 11.1 | s | calculated | docs/02-form.md#wind-climate, private:loc-01 |
| climate.home_weak_skim_hs2_share | 0.127 | 1 | calculated | docs/02-form.md#wind-climate, private:loc-01 |
| climate.home_weak_skim_hs3_share | 0.016 | 1 | calculated | docs/02-form.md#wind-climate, private:loc-01 |
| climate.home_weak_skim_swell2_share | 0.05 | 1 | calculated | docs/02-form.md#wind-climate, private:loc-01 |
| climate.home_weak_skim_swell3_share | 0.007 | 1 | calculated | docs/02-form.md#wind-climate, private:loc-01 |
| climate.home_weak_skim_hs_survival_share | 0.002 | 1 | calculated | docs/02-form.md#wind-climate, private:loc-01 |
| climate.home_weak_skim_tp_s | 11.1 | s | calculated | docs/02-form.md#wind-climate, private:loc-01 |
| climate.home_weak_best_hs2_share | 0.167 | 1 | calculated | docs/02-form.md#wind-climate, private:loc-01 |
| climate.home_weak_best_hs3_share | 0.025 | 1 | calculated | docs/02-form.md#wind-climate, private:loc-01 |
| climate.home_weak_best_swell2_share | 0.049 | 1 | calculated | docs/02-form.md#wind-climate, private:loc-01 |
| climate.home_weak_best_swell3_share | 0.007 | 1 | calculated | docs/02-form.md#wind-climate, private:loc-01 |
| climate.home_weak_best_hs_survival_share | 0.003 | 1 | calculated | docs/02-form.md#wind-climate, private:loc-01 |
| climate.home_weak_best_tp_s | 11.1 | s | calculated | docs/02-form.md#wind-climate, private:loc-01 |
| climate.home_albatross_motor_p50_wh | 226 | Wh | calculated | docs/02-form.md#wind-climate, private:loc-01 |
| climate.home_albatross_motor_p90_wh | 1418 | Wh | calculated | docs/02-form.md#wind-climate, private:loc-01 |
| climate.home_albatross_motor_p99_wh | 4351 | Wh | calculated | docs/02-form.md#wind-climate, private:loc-01 |
| climate.home_albatross_float_p50_wh | 33 | Wh | calculated | docs/02-form.md#wind-climate, private:loc-01 |
| climate.home_albatross_float_p90_wh | 207 | Wh | calculated | docs/02-form.md#wind-climate, private:loc-01 |
| climate.home_albatross_float_p99_wh | 635 | Wh | calculated | docs/02-form.md#wind-climate, private:loc-01 |
| climate.home_albatross_motor_p99_cell_kg | 18.84 | kg | calculated | docs/02-form.md#wind-climate, private:loc-01 |
| climate.home_albatross_float_p99_cell_kg | 2.75 | kg | calculated | docs/02-form.md#wind-climate, private:loc-01 |
| climate.home_best_motor_p50_wh | 1548 | Wh | calculated | docs/02-form.md#wind-climate, private:loc-01 |
| climate.home_best_motor_p90_wh | 12677 | Wh | calculated | docs/02-form.md#wind-climate, private:loc-01 |
| climate.home_best_motor_p99_wh | 47684 | Wh | calculated | docs/02-form.md#wind-climate, private:loc-01 |
| climate.home_best_float_p50_wh | 140 | Wh | calculated | docs/02-form.md#wind-climate, private:loc-01 |
| climate.home_best_float_p90_wh | 1142 | Wh | calculated | docs/02-form.md#wind-climate, private:loc-01 |
| climate.home_best_float_p99_wh | 4296 | Wh | calculated | docs/02-form.md#wind-climate, private:loc-01 |
| climate.home_best_motor_p99_cell_kg | 206.42 | kg | calculated | docs/02-form.md#wind-climate, private:loc-01 |
| climate.home_best_float_p99_cell_kg | 18.6 | kg | calculated | docs/02-form.md#wind-climate, private:loc-01 |
| climate.home_djf_albatross_motor_p90_wh | 966 | Wh | calculated | docs/02-form.md#wind-climate, private:loc-01 |
| climate.home_djf_albatross_motor_p99_wh | 2323 | Wh | calculated | docs/02-form.md#wind-climate, private:loc-01 |
| climate.home_djf_albatross_float_p90_wh | 141 | Wh | calculated | docs/02-form.md#wind-climate, private:loc-01 |
| climate.home_djf_albatross_float_p99_wh | 339 | Wh | calculated | docs/02-form.md#wind-climate, private:loc-01 |
| climate.home_djf_best_motor_p90_wh | 9078 | Wh | calculated | docs/02-form.md#wind-climate, private:loc-01 |
| climate.home_djf_best_motor_p99_wh | 22961 | Wh | calculated | docs/02-form.md#wind-climate, private:loc-01 |
| climate.home_djf_best_float_p90_wh | 818 | Wh | calculated | docs/02-form.md#wind-climate, private:loc-01 |
| climate.home_djf_best_float_p99_wh | 2069 | Wh | calculated | docs/02-form.md#wind-climate, private:loc-01 |
| climate.home_mam_albatross_motor_p90_wh | 1305 | Wh | calculated | docs/02-form.md#wind-climate, private:loc-01 |
| climate.home_mam_albatross_motor_p99_wh | 3459 | Wh | calculated | docs/02-form.md#wind-climate, private:loc-01 |
| climate.home_mam_albatross_float_p90_wh | 190 | Wh | calculated | docs/02-form.md#wind-climate, private:loc-01 |
| climate.home_mam_albatross_float_p99_wh | 505 | Wh | calculated | docs/02-form.md#wind-climate, private:loc-01 |
| climate.home_mam_best_motor_p90_wh | 17669 | Wh | calculated | docs/02-form.md#wind-climate, private:loc-01 |
| climate.home_mam_best_motor_p99_wh | 88768 | Wh | calculated | docs/02-form.md#wind-climate, private:loc-01 |
| climate.home_mam_best_float_p90_wh | 1592 | Wh | calculated | docs/02-form.md#wind-climate, private:loc-01 |
| climate.home_mam_best_float_p99_wh | 7997 | Wh | calculated | docs/02-form.md#wind-climate, private:loc-01 |
| climate.home_jja_albatross_motor_p90_wh | 3288 | Wh | calculated | docs/02-form.md#wind-climate, private:loc-01 |
| climate.home_jja_albatross_motor_p99_wh | 7585 | Wh | calculated | docs/02-form.md#wind-climate, private:loc-01 |
| climate.home_jja_albatross_float_p90_wh | 480 | Wh | calculated | docs/02-form.md#wind-climate, private:loc-01 |
| climate.home_jja_albatross_float_p99_wh | 1107 | Wh | calculated | docs/02-form.md#wind-climate, private:loc-01 |
| climate.home_jja_best_motor_p90_wh | 46001 | Wh | calculated | docs/02-form.md#wind-climate, private:loc-01 |
| climate.home_jja_best_motor_p99_wh | 68418 | Wh | calculated | docs/02-form.md#wind-climate, private:loc-01 |
| climate.home_jja_best_float_p90_wh | 4144 | Wh | calculated | docs/02-form.md#wind-climate, private:loc-01 |
| climate.home_jja_best_float_p99_wh | 6164 | Wh | calculated | docs/02-form.md#wind-climate, private:loc-01 |
| climate.home_son_albatross_motor_p90_wh | 1274 | Wh | calculated | docs/02-form.md#wind-climate, private:loc-01 |
| climate.home_son_albatross_motor_p99_wh | 3393 | Wh | calculated | docs/02-form.md#wind-climate, private:loc-01 |
| climate.home_son_albatross_float_p90_wh | 186 | Wh | calculated | docs/02-form.md#wind-climate, private:loc-01 |
| climate.home_son_albatross_float_p99_wh | 495 | Wh | calculated | docs/02-form.md#wind-climate, private:loc-01 |
| climate.home_son_best_motor_p90_wh | 10226 | Wh | calculated | docs/02-form.md#wind-climate, private:loc-01 |
| climate.home_son_best_motor_p99_wh | 20781 | Wh | calculated | docs/02-form.md#wind-climate, private:loc-01 |
| climate.home_son_best_float_p90_wh | 921 | Wh | calculated | docs/02-form.md#wind-climate, private:loc-01 |
| climate.home_son_best_float_p99_wh | 1872 | Wh | calculated | docs/02-form.md#wind-climate, private:loc-01 |
| climate.solar_day_albatross_so_wh | 25.6 | Wh | calculated | docs/02-form.md#wind-climate |
| climate.solar_day_best_so_wh | 31.9 | Wh | calculated | docs/02-form.md#wind-climate |
| climate.solar_day_albatross_home_wh | 59.0 | Wh | calculated | docs/02-form.md#wind-climate, private:loc-01 |
| climate.solar_day_best_home_wh | 73.8 | Wh | calculated | docs/02-form.md#wind-climate, private:loc-01 |
| climate.avionics_day_wh | 72.0 | Wh | calculated | docs/02-form.md#wind-climate |
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
