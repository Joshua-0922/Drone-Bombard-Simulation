---
date: 2026-10-24
tags: [research, sim2real, release-delay, survey]
status: active
type: research
---

# 드론 투하 기구 지연: 측정 방법과 시뮬 반영 방식 (추가 조사, 2026-10-24)

상위 보고서: [[reports/강화학습 sim2real DR 설계 조사]] (§사용자 설계와의 비교·수정안)

**결론:** 평균·표준편차·n을 공개한 새 드론 투하 연구는 찾지 못했다. 측정값은 기존 4건(Zhai 0.11 s 고정, Werner 258 ± 15 ms n=39 고정, MC-PILOT 폭 ≤10 ms 균등, AeroThrow 미보고)이 전부다.

| 연구 | 기구 | 측정 | 값 | 반영 |
|---|---|---|---|---|
| Mathisen 외, IFAC 2017 (NTNU, Skywalker X8 고정익) [pdf](https://torarnj.folk.ntnu.no/drop.pdf) | E-flite EFLA405 서보리스 투하기 | 실험실, 장비·n·편차 미보고 | 0.4 s | 고정 상수. "비행 중 미검증" 명시 |
| Mathisen 외, Autonomous Robots 2020 [doi](https://doi.org/10.1007/s10514-020-09902-3) | 같음 | 기구 지연 수치 없음, GPS 지연 0.1 s 예측 보상 | — | 50 m·18 m/s에서 평균 5.5 m |
| FEKF 데이터링크 지연 보상 [RG](https://www.researchgate.net/publication/318130275) | — | 초록만 확인(미검증) | — | 온라인 보상 |
| RISCuer, arXiv 2006.06966 | 영구자석 + 서보 밀어내기 | — | EPM은 "수 초" 걸려 기각 | — |
| ArduPilot EPM 문서 [link](https://ardupilot.org/copter/docs/common-electro-permanent-magnet-gripper.html) | EPM688 / OpenGrab v3 | 사양 | 0.4 s / 약 1 s | EPM은 수백 ms 이상 |
| FlipItRight, arXiv 2606.01713 (UR10e) | 그리퍼 | 폭 센서로 동작 시작·분리 시점 분리 | 두 시점 차이: 고파지력 20–23 ms, 저파지력 1.2–1.4 ms | 명령을 미리 내 보상 |
| 업체 가이드(미검증) | 서보 래치 / 전자석 / 솔레노이드 | 240 fps 이상 영상 권장 | 200–500 ms / <50 ms / 10–100 ms | — |

**서보만 측정하는 것의 의미:** 통신, PWM 주기(50 Hz면 최대 20 ms), 서보 불감대·이동 시간만 포함하는 **하한**이다. 래치 마찰(하중 예압), 걸림, 분리 후 자유낙하까지는 빠진다. FlipItRight에서 예압에 따라 10배 이상 달라졌다.

**권장 측정 절차:** (1) 명령을 내는 코드 자리에서 GPIO 토글로 시각을 찍고 PWM 선도 함께 기록(로직 분석기 또는 ESP32 `micros()`). (2) 분리 검출은 래치 아래 광센서·하중이 누르는 스위치·하중 IMU(자유낙하 ≈ 0 g) 중 2개로 교차 확인. (3) 시행마다 명령·첫 PWM·혼 동작·래치 해제·분리·자유낙하 시각, 하중 질량, 전압, 벤치/정지 비행 기록. (4) 조건당 30–40회 이상. (5) 평균·표준편차·최소·최대·히스토그램 보고.
