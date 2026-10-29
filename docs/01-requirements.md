# 1.1 Requirements

Why each requirement in record/requirements.yaml is what it is. Start with the questions that can rule a form out.


## Certification routes (Q-001)

The owner chose type certification (D-003) and flight beyond visual line of sight over places where people are (D-004).
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
| env.charnock | 0.0144 | 1 | assumed | Q-008 |
| env.air_kinematic_viscosity | 1.46e-05 | m2/s | assumed | Q-008 |
| env.jonswap_gamma | 3.3 | 1 | assumed | Q-008 |
| env.pm_hs_coefficient | 0.21 | 1 | assumed | Q-008 |
| env.pm_peak_coefficient | 0.877 | 1 | assumed | Q-008 |
| ds.endurance_days | 365 | d | decided | D-008 |
| ds.range_km | 10 | km | decided | D-009 |
| ds.photo_bytes | 10000 | B | assumed | Q-009 |
| ds.min_u10_ms | 8 | m/s | assumed | Q-009 |
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
| spar.carbon_density | 1600 | kg/m3 | assumed | Q-009 |
| spar.carbon_modulus | 120000000000.0 | Pa | assumed | Q-009 |
| spar.carbon_allowable | 600000000.0 | Pa | assumed | Q-009 |
| spar.od_mm | 10.0 | mm | calculated | docs/03-design.md#wing-spar |
| spar.wall_mm | 0.5 | mm | calculated | docs/03-design.md#wing-spar |
| spar.mass_kg | 0.053 | kg | calculated | docs/03-design.md#wing-spar |
| spar.margin | 0.08 | 1 | calculated | docs/03-design.md#wing-spar |
| spar.tip_deflection_ratio | 0.14 | 1 | calculated | docs/03-design.md#wing-spar |
| wing.mass_kg | 0.392 | kg | calculated | docs/03-design.md#mass-model |
| airframe.baseline_mass_kg | 1.072 | kg | calculated | docs/03-design.md#mass-model |
<!-- tdr:end -->

## Requirements for the dynamic soaring drone {#ds-requirements}

The owner moved the project from a certifiable drone to a drone that stays over the sea for a year with dynamic soaring as its core (D-008), that goes on its own to a given place and sends photos or data from there (D-009), and asked for the computation environment first (D-010).
The certification questions Q-003 to Q-005 wait until Q-007 says which rules apply to such a flight.

The questions that rule a form out were asked before, and the answers that still stand are: a personal project built by one person (D-001), a budget of budget.total (D-005), Japan as the place of building and first flights, and a long aim of circling the globe (D-009).
The place of flight at sea and its climate are private facts and enter the record only as derived values.

R-01 and R-02 are the owner's core; R-03 and R-04 are the mission (D-009).
R-05 and R-06 follow from a year at sea: the wind will fail, so the drone must live on the water, and the sea will get in unless it is sealed.
R-07 follows from one person and the budget; R-08 from a year without a charger; R-09 and R-10 from the loads the environment imposes; R-11 from the law; R-12 from where the shear is.
The design points carry assumed numbers (basis Q-009) until the literature and the simulation settle them, and the owner confirmed every requirement on 2026-10-29 (D-011).

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
<!-- tdr:end -->
