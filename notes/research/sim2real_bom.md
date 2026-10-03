---
date: 2026-10-03
tags: [research, sim2real, hardware, purchase, bom]
status: active
type: research
---

# sim2real 구매 목록 (실외, X500 V2, 이륙중량 2 kg 이하)

관련: [[research/sim2real_indoor_vs_outdoor]] §7 · [[research/sim2real_strategy]] · [[00_index]]

> 2026-10-03 가격 조사(조사 에이전트 3개, 상품 페이지 직접 확인). 환율은 **1 USD ≈ 1,400원, 1 EUR ≈ 1,600원으로 가정**했다(당일 시세 아님). 배송비·관부가세는 미확인. "미확인"은 페이지에서 확인하지 못한 값이다. 엘레파츠 가격은 VAT 별도.

## 0. 먼저 해결할 것

- **비행 승인:** 관악구는 **비행제한구역(R-75)** 에 포함된다. 서울대 대운동장 비행은 무게와 관계없이 **드론원스톱(drone.onestop.go.kr)에서 사전 비행승인**을 받아야 한다. 투하 행위는 승인 신청서에 명시하는 것을 권한다([easylaw 비행승인](https://easylaw.go.kr/CSP/CnpClsMain.laf?popMenu=ov&csmSeq=1814&ccfNo=3&cciNo=1&cnpClsNo=1), [서울시 드론공간정보](https://news.seoul.go.kr/gov/archives/528178)).
- **무게:** X500 V2의 완성 기체 무게는 공식 미공개(프레임 610 g만 공개). 기체 1.1–1.3 kg(추정)이므로, **배터리 4S 3000–4000 mAh와 경량 RTK(Ultralight)** 를 써야 2 kg 안에 들어온다(§4). 구매 후 저울로 실측한다.
- **무선 규정:** SiK 915 MHz판은 국내 비면허 대역(917–923.5 MHz)을 벗어나므로 사지 않는다. 433 MHz판도 KC 인증 여부는 미확인 → **주 조종 링크는 KC 인증된 ELRS 2.4 GHz**, SiK 433은 보조.

## 1. 기체 (필수)

| 품목 | 제품 | 가격 | 구매처 | 비고 |
|---|---|---|---|---|
| 기체 키트 | **Holybro PX4 Development Kit – X500 v2** (Pixhawk 6C, M10 GPS, PM02 V3, SiK V3 **433 MHz**, 프레임, 2216 KV920 모터 4, 20A ESC 4, 1045 프롭 6) | **US$609** (≈85만 원) | [Holybro 공식](https://holybro.com/products/px4-development-kit-x500-v2) | 국내 낱개 조합(ARF 드론위 572,000 + Pixhawk 6C 세트 알씨뱅크 506,000 + SiK 138,000)은 약 121.6만 원. 엑스캅터 완제품 가격 미확인 |
| RTK 로버 | **H-RTK ZED-F9P Ultralight** (판매처 표기 21 g, 실측 필요) | US$279 / 드론위 599,500원 | [Holybro](https://holybro.com/products/h-rtk-f9p-ultralight) | 무게 우선. 대안: ZED-F9P Rover 117 g, US$245 / 드론위 684,200원 |
| RTK 보정 | **국토지리정보원 VRS/FKP (NTRIP)** | 무료 | 국토지리정보원 회원 가입 → 측위보정정보 통합회원 ID 등록(2025-08-01부터 로그인 의무), 문의 031-210-2655 | 기지국 불필요. 현장 LTE 필요, 클라이언트의 GGA 전송 지원 확인 필요. 대안: 두 번째 F9P(Helical US$219)를 기지국으로 |
| 조종기 | **RadioMaster Pocket ELRS** (모드 2, KC R-R-ssr-POCKETELRS) | 129,000원 | [수성RC](https://susungrc.com/product/라디오마스터-pocket-조종기-elrs/2385/) 외 동일가 | 18650 배터리 별도 |
| 수신기 | **RadioMaster RP1 ELRS 2.4 GHz** (2.2 g) | 32,000원 | [팰콘샵](https://www.falconshop.co.kr/shop/goods/goods_view.php?goodsno=100078192) | 송수신기 펌웨어 규제 도메인 일치 |
| 예비품 | 1045 프롭 1조 + 모터 CW·CCW 각 1 | 127,300원 / US$51.57 | [알씨뱅크](https://www.rcbank.co.kr/shop/goods/goods_view.php?goodsno=68702) | 예비 ESC 33,700원(선택) |

## 2. 탑재물 (필수)

| 품목 | 제품 | 무게 | 가격 | 구매처 |
|---|---|---|---|---|
| 컴패니언 | Raspberry Pi 5 4GB | 46–62 g(실측 필요) | 204,600원 | [디바이스마트](https://www.devicemart.co.kr/goods/view?no=15215450) |
| 쿨러 | 공식 Active Cooler | 약 28 g | 7,700원 | [디바이스마트](https://www.devicemart.co.kr/goods/view?no=15276241) |
| 저장장치 | MakerDisk microSD 64GB | – | 48,620원 | [디바이스마트](https://www.devicemart.co.kr/goods/view?no=14904843) |
| 전원 | **Matek BEC 20S-PRO** (5.2 V 5 A 연속) | 5 g | 34,000원 | [알씨파워](https://www.rcpower.co.kr/shop/goods/goods_view.php?goodsno=10011874) — Holybro UBEC 12A는 5 V 출력이 없어 부적합 |
| FC 연결 | TELEM2(GH 6P) → Pi GPIO 케이블 | 수 g | 미확인 | 키트 동봉 GH 6P 케이블 한쪽을 듀폰으로 압착(TX→GPIO15, RX→GPIO14, GND만) |
| 투하 서보 | **EMAX ES9052MD** (0.07 s/60° @6V) | 4.8 g | 16,500원 | [알씨파워](https://www.rcpower.co.kr/shop/goods/goods_view.php?goodsno=10011761) — 대안 MG90S 정품 13.4 g, 12,100원 |
| 투하 래치 | **3D 프린팅 자작**(서보 핀 래치) | 10–20 g(추정) | 재료비 | 기성품 PCS 드롭 릴리즈 19,140원(무게 미확인), LX 릴리즈 훅 87 g은 과함 |
| 거리센서 | **Benewake TFmini-S** (PX4 공식 지원) | 5 g | 69,300원 | [디바이스마트](https://www.devicemart.co.kr/goods/view?no=12509759) — TF-Luna는 PX4 공식 미지원 |
| 배터리 | **4S 3000–4000 mAh XT60 × 6** | 약 330–430 g(추정) | 미확인 | 조사한 5200 mAh(VEGA 497 g, 88,000원/개)는 무게 한도를 넘길 위험 → 3000–4000 mAh를 같은 판매처에서 확인 |
| 충전기 | **SkyRC T200** (AC 듀얼 100 W×2, KC) | – | 158,000원 | [알씨파워](https://www.rcpower.co.kr/shop/goods/goods_view.php?goodsno=10007760) |
| 배터리 안전 | 방화팩 × 2 | – | 19,800원 | 알씨파워·알씨뱅크 |

## 3. 지상 장비

| 품목 | 제품 | 가격 | 구매처 | 비고 |
|---|---|---|---|---|
| 풍속·풍향계(필수, 2세트) | **DFRobot SEN0483(풍속) + SEN0482(풍향), RS485 Modbus** × 2세트 | 각 US$45 → 4개 약 25.2만 원 | [DFRobot](https://wiki.dfrobot.com/sen0483/) · DigiKey | 컵형, PC 폴링으로 1 Hz 기록, NTP로 시각 동기. 갱신율은 데이터시트 미표기 |
| RS485 변환 | 강원전자 NM-UAR2285 USB-RS485 | 22,000원 | [다나와](https://prod.danawa.com/info/?pcode=99750236) | |
| 풍속계(상위 대안) | Calypso Ultrasonic Portable Mini × 2 (초음파, 1 Hz, BLE) | 59.5–84.1만 원/개(국내 해외구매) | 다나와 "칼립소 풍속계" | 돌풍 응답이 컵형보다 빠름. Davis Vantage Vue는 갱신 2.5 s라 부적합 |
| 마스트 | KUPO 045 스탠드(4 m 이상, 하중 1 kg) × 2 + 모래주머니 × 2 | 131,890원 × 2 + 8,790원 × 2 | [다나와](https://prod.danawa.com/info/?pcode=3528755) | 4 m에서 바람을 받으므로 가이 로프 필수 |
| 착지 카메라 | **DJI Osmo Action 5 Pro** (4K 120, 1080p 240 fps) | 453,740원 | [다나와](https://prod.danawa.com/info/?pcode=67404887) | 높이 5 m에서 약 2.4 mm/px(가정 화각 85°). 투하 지연 측정(240 fps)에도 사용 — 별도 고속 카메라 불필요 |
| 카메라 거치 | KUPO 045 1대 추가 + 붐 암 | 131,890 + 84,020원 | 다나와 | |
| 지상 마커 | 50 m 줄자(코메론 KMC-330) + 라인 테이프 + AprilTag 인쇄 | 14,790 + 13,030원 | 다나와 | |
| 페이로드 재료 | eSUN PLA+ 1 kg + eSUN TPU 95A 1 kg + 쇠구슬 1 kg | 19,590 + 44,070 + 5,140원 | 다나와 | 본체 PLA 저충전, 바닥 패드 TPU, 쇠구슬은 격실·에폭시로 고정해 무게중심 유지, 100 g ± 1 g |
| 현장 안전 | 라바콘 × 4, 안전띠 100 m, 에어로졸 소화기 × 2 | 54,560 + 3,630 + 34,980원 | 디바이스마트 | 리튬 전용 소화기(60–80만 원대)는 성능 미검증 보도(2024-12) |
| RTK 측량(선택) | 보유 F9P를 측량봉에 올려 VRS 보정(측량봉 약 20만 원) 또는 ArduSimple RTK Calibrated Surveyor Kit €444 | +20만–71만 원 | ardusimple.com | 표적 좌표를 드론 좌표계에 묶을 때 필요. 2 m 봉 1° 기울면 약 3.5 cm 오차 → 원형 수준기 |

## 4. 무게 예산 (2 kg 한도)

| 항목 | 무게 |
|---|---|
| 기체(프레임·모터·ESC·FC·전원모듈·M10 GPS·SiK, 배터리 제외) | 1.1–1.3 kg (**추정, 실측 필요**) |
| RTK Ultralight | 21 g (판매처 표기, 실측 필요) |
| Pi 5 + 쿨러 + BEC + 서보 + 래치 + 거리센서 + 케이블 | 약 110–150 g |
| 배터리 4S 3000–4000 mAh | 약 330–430 g |
| 페이로드 | 100 g |
| **합계** | **약 1.66–2.0 kg** |

상한에 가깝다. 실측 후 ① M10 GPS 제거(RTK만 사용), ② 쿨러 대신 방열판, ③ 3000 mAh로 맞추는 순서로 줄인다. 비행 한 번이 1–2분이므로 3000 mAh로도 충분하다.

## 5. 합계 (원화 추정, 배송비·관부가세 별도)

| 묶음 | 최소안 | 비고 |
|---|---|---|
| 기체 | 약 148만 원(직구 키트 + Ultralight + 예비품, 국내 RC) | 국내 낱개 조합이면 약 210–220만 원 |
| 탑재물 | 약 67만 원 + 배터리 6개(미확인, 5200 mAh 기준이면 +52.8만 원) | 배터리 제외 시 약 67만 원 |
| 지상 | 약 119만 원(컵형 풍속계) | 초음파 풍속계면 약 211만 원 |
| **총계** | **약 385만 원 + 배터리(약 50만 원 내외 추정) ≈ 430–440만 원** | RTK 측량·초음파 풍속계·예비 ESC 추가 시 +100–160만 원 |
