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
<!-- tdr:end -->
