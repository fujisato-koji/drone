# 1.5 Parts and cost

The parts list and the cost, as tables generated from the record.

## Costs other than the airframe for Level 3.5 flight

This section counts what an individual needs, besides the airframe, to fly a self-built aircraft at Level 3.5, which is category II flight beyond visual line of sight with an onboard camera and no observers.
The owner's own time is not counted.
Level 3.5 asks for an onboard camera that shows the area below and ahead, a remote pilot certificate with the visual line of sight limitation lifted, and third party liability insurance that can compensate sufficiently (P14 5-4(3)c)(f)(iii)).
It names neither a certificate class nor an insured amount (P14, P29).
The same item also asks for emergency landing sites and other operating safety measures, and the rest of section 5-4 still applies, such as automatic control, position tracking, fail-safe functions, lights or paint, weather awareness, enough flight record, route setting, watching for manned aircraft, and notices to airmen (P14 5-4).
The standard manual says the limitation to lift is the one on visual line of sight and the like, so night flight also needs the night limitation lifted, and it prefers insurance that relieves victims quickly without their having to prove fault (P29 3-(7)).

### Remote pilot certificate

A second class certificate with the visual line of sight limitation lifted meets the requirement, since the review guidelines name no class (P14).
Taking the exams directly costs cost.license_exam_written_2nd for the written exam, cost.license_exam_practical_2nd_basic and cost.license_exam_practical_2nd_bvlos for the practical exam and its limitation change, cost.license_medical_documents for the medical check by documents, and cost.license_issue to issue the certificate (P26, P27).
These are the multicopter figures; a helicopter pays more for the practical exam (P26).
The medical check by documents assumes a document such as a driving licence; without one it costs cost.license_medical_venue at a venue, or a doctor's certificate (P26).
The practical exam can be taken only after passing the written exam, and lifting the limitation after the certificate is issued costs cost.license_renewal_issue again (P27, P28).
A registered course exempts the practical exam, but the written exam, the medical check, and the issue are still paid (P28).
A beginner course costs cost.license_course_2nd_beginner_min to cost.license_course_2nd_beginner_max, an experienced course cost.license_course_2nd_experienced_min to cost.license_course_2nd_experienced_max, and the limitation change course cost.license_course_bvlos_min to cost.license_course_bvlos_max (M6, M7, M8).
The certificate is valid for cost.license_validity, and renewal needs a renewal course of cost.license_renewal_course_min to cost.license_renewal_course_max, a medical check, and cost.license_renewal_issue (P27, P28, M9).

### Registration and Remote ID

Registration costs cost.registration_fee_mynumber online with the My Number card or cost.registration_fee_other_id with other identification, and lasts cost.registration_validity, as read from the renewal answer (P21).
A self-built aircraft can carry a notified Remote ID device inside or outside (P21, P32).
External devices cost about cost.remote_id_external_min to cost.remote_id_external_typ, a forum figure to be confirmed at a shop (F1).
Building Remote ID into the aircraft itself needs a key notification to the Civil Aviation Bureau and self verification by the developer (P33).

### Permission and approval

DIPS 2.0 charges no system fee (cost.permit_system_fee, P30), and no fee for the application was found, though no text says the application is free.
The Civil Aviation Bureau's standard manual for Level 3.5 can be used, and an own manual is reviewed case by case (P29, P31).

### Video and control link

Licence-free equipment on 2.4 GHz and 920 MHz must carry the technical conformity mark (P34).
The 5.7 GHz band for unmanned aircraft video is limited to industrial use and needs proof of membership in the coordinating body (P34), and that body does not take applications from individuals, though sole proprietors and corporations may apply (P35).
Using the mobile network in the air needs a carrier's plan for use in the air: cost.lte_docomo_flat_month a month flat, or cost.lte_docomo_metered_base_month a month plus usage (M1), and cost.lte_softbank_month on SoftBank's business page (M2).
The docomo plan is contracted at a docomo shop with a listed device, and the contract form has fields for individual subscribers, so an individual can likely subscribe (M1).
So an individual has two links to choose from: licence-free 2.4 GHz, or the mobile network.

### Insurance

The State sets no insured amount, and the application states the amounts for persons and property (P14, P29).
A hobby policy for individuals costs cost.insurance_hobby_two_years for two years with cost.insurance_hobby_cover of cover, but covers hobby flights only, excludes work and research, and pays only where there is legal liability, which is weaker than the no-fault relief the manual prefers (M3, P29).
A business policy costs cost.insurance_business_year a year for cost.insurance_business_cover, but is only for corporations and sole proprietors, only for industrial rotorcraft of 100 g to 150 kg with a sale price within a set range, and needs a free online course; a self-built aircraft has no sale price, so it likely does not qualify (M4).
Another product excludes self-built aircraft (M5).
Whether an individual can insure a self-built aircraft for Level 3.5 is open as Q-006.

### Ground station and tests

ArduPilot, PX4, Mission Planner, and QGroundControl are free and open source (P37).
A private flying field costs cost.test_field_private_day a day (M10), and a third of the netted field at the Fukushima Robot Test Field costs cost.test_field_rtf_half_day_third for half a day (P36).

### Totals

The totals add the items above and nothing else.
The one time minimum, cost.non_airframe_once_min, takes the exams directly: the written exam, the practical exam and its limitation change, the medical check, the issue, registration with the My Number card, the cheaper external Remote ID device, and one day at a private field.
The one time typical figure, cost.non_airframe_once_typ, takes the beginner course at the top of its range and the limitation change course at the top of its range, the written exam, the medical check, the issue, registration with other identification, the dearer external Remote ID device, and half a day on a third of the netted field.
The yearly minimum, cost.non_airframe_year_min, spreads over three years the cheaper renewal course, a medical check assumed equal to cost.license_medical_documents, and the renewal issue, adds a third of the My Number registration fee, and adds half the two year hobby policy.
The yearly typical figure, cost.non_airframe_year_typ, does the same with the dearer renewal course and registration with other identification, and adds the business policy, for which a self-built aircraft likely does not qualify (M4).
None of the totals include the airframe, a PC, radio equipment, or a mobile plan; the mobile network adds twelve times cost.lte_docomo_flat_month a year on the flat plan.
