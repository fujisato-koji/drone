# 1.1 Requirements

Why each requirement in record/requirements.yaml is what it is. Start with the questions that can rule a form out.

## 認証の経路（Q-001）

施主は型式認証（D-003）と、人のいる場所の上を目視外で飛ばすこと（D-004）を選んだ。
人のいる場所の上の目視外飛行はカテゴリーIII に当たり、一等の技能証明と第一種機体認証の機体と許可承認がそろって初めて行える（P2 132条の85、132条の86、P13）。
目視外の飛行には、技能証明の限定変更も要る（P2 132条の43）。
型式認証は型式の設計と製造過程を認証し、検査の対象は設計、製造過程、その型式の1機の現状である（P2 132条の16、P3 236条の23）。
型式認証には均一性基準が掛かり、品質管理制度と、業務の組織から独立した監査を求める（P3 236条の24）。
ガイドラインは、ある構成品の設計者、製造者、検査者、責任者がすべて同じ一人で、ほかに誰もいない分担を認めず、兼業は妨げないとする（P7 p244）。
監査者は兼務でよく、常設の組織でなくてよい（P7 p268から269）。
法は検査を登録検査機関に行わせることを認めるが、今の運用では第一種は国が検査し、登録検査機関の能力は第二種だけである（P2 132条の24、P6 第I部2、P11）。
登録検査機関の多くは機体認証を型式認証機に限るので、型式認証の無い第二種の機体も国の検査になる場合がある（P11）。
型式認証を受けた型式の新品は、第二種なら機体認証の検査を原則として全部省略でき、第一種では現状の実地検査が残る（P4 6-1）。
ここで新品とは、屋内も含めて一度も飛ばしておらず、出荷時の装備から変えておらず、改造していない機体を言う（P4 4項の注1、注6）。
第二種の安全基準では、cert.self_confirm_weight 未満の機体は多くの項目を申請者自身が確かめた結果の提出で済ませられる（P6 第3章）。
型式認証の無い機体も機体認証を受けられるが、設計と製造過程の検査に相当の時間が掛かると国は書いている（P3 236条の12、P1 特記7）。
機体認証の有効期間は第一種が cert.validity_aircraft_class1、第二種が cert.validity_aircraft_class2 で、型式認証は cert.validity_type である（P3）。

### 予算との突き合わせ

どの経路も、国の手数料だけで budget.total の大半か、それ以上を使う。
第一種の型式認証の手数料は cert.fee_type_class1_min から cert.fee_type_class1_max である（P10）。
第二種で cert.self_confirm_weight 未満の型式認証の手数料は cert.fee_type_class2_under4kg である（P10）。
型式認証の無い機体の機体認証の手数料は、第一種が cert.fee_aircraft_class1_untyped_min から、第二種の cert.self_confirm_weight 未満が cert.fee_aircraft_class2_untyped_under4kg である（P9）。
第一種の耐久性と信頼性の飛行試験の時間は、米国の計算例で cert.dr_hours_example_390、危害の軽減があれば cert.dr_hours_example_390_mitigated である（P7 p143）。
手数料の表は 2022-11 の作成だが、額は現行の航空法関係手数料規則と一致する（P24、K-001）。
第一種の型式認証を受けた型式は market.class1_type_models で、国が挙げるレベル４飛行 market.level4_flights 件はすべてその型式で行われた（P22、P23 9頁）。
このため、予算の範囲を Q-003、第一種か第二種かを Q-004、一人で担えない役を Q-005 として施主に聞く。

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
