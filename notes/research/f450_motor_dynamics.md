---
date: 2026-10-25
tags: [research, sim2real, hardware, f450, motor, gazebo, sysid]
status: active
type: research
---

# F450 모터 동특성 — 시뮬레이션 기본값보다 느리다

관련: [[experiments/exp_044_f450_motor_step_response]] · [[research/sim2real_field_feasibility_f450]] · [[research/sim2real_hardware_spec]] · [[research/rl_rules]]

## 발견
F450(2212 920KV + DJI 9450 + 30 A PWM ESC, 4S)의 로터 회전수 스텝 응답은 1차 지연으로 근사하면 가속 $\tau \approx 80$ ms, 감속 $\tau \approx 65$–$95$ ms다
(exp_044, 90 % 구간 대략 40–115 / 35–140 ms). PX4 X500 Gazebo 기본값(가속 12.5 ms, 감속 25 ms)은 구간 하한보다도 빠르다.

## 원인 (추정)
- 9450 프롭은 회전 관성이 크고, 2212급 모터는 토크 여유가 작다.
- 일반 PWM ESC는 능동 제동이 없어 감속이 공기 저항에만 의존한다.
- ESC 입력 필터와 PWM 갱신(약 2.5 ms @400 Hz)은 수 ms 수준이라 주원인은 아니다(명령 → PWM 출력은 약 5 ms).

## 적용 규칙
- Gazebo `x500_drop`의 모터 시정수는 실측 근사값(0.08 / 0.08 s)을 쓴다. 기본값을 쓰면 SITL이 실기보다 공격적으로 반응한다.
- ~~Isaac에는 모터 모델을 넣지 않는다~~ → 11-11 대체(Rule 49): Isaac에 Quad X 믹서 + 로터 1차 지연(상승 τ, 하강 r·τ)을 넣었다. 기존 이득은 이 지연에서 진동하므로 PX4 모드 이득을 12/4.5로 낮췄다([[experiments/exp_045_isaac_px4_plant_dryrun]]).
- 비행 전 PX4 자세 게인은 기본값보다 보수적으로 시작한다(모터가 느리면 같은 게인에서 위상 여유가 줄어든다).

## 향후 조건
- 모터·ESC·프롭 중 하나라도 바꾸면 다시 잰다(`hardware/bench/motor_step/`).
- 정밀값이 필요하면 광학 회전수 센서나 DShot 회전수 피드백으로 다시 잰다. 오디오 방식은 τ ±40 ms가 한계였다.

## 문헌 비교 (2026-11-11 조사)

2212 920KV + 9450 조합의 **공개 계단응답·시정수 데이터는 찾지 못했다.** Tyto/RCbenchmark 데이터베이스는 정적 시험만 다루고, 이 급의 동정 논문들(E-MAX XA2212 + 10×4.5 NARX, arXiv 2311.03985)도 모터 시정수는 보고하지 않았다. 가장 가까운 값은 다음과 같다.

| 출처 | 구성 | 측정 | 값 |
|---|---|---|---|
| Gauthier 외, arXiv 2312.09981 | Holybro ESC + **AIR 2216 880KV + 1045**, 14.8 V | Tyto 1585 로드셀, 스로틀 0.34 → 0.45 계단 | 1차 지연으로 맞추면 $\tau_{ESC}\approx35$ ms(반값 시점 일치). 실제 응답은 1차보다 빨리 정상상태에 도달 |
| Al-Zubaidi·Stol, IMAV 2023 | MN1806 + 6″ (프로토콜 비교) | 회전수 계단 | 정착시간 DShot 약 100 ms, PWM 50 Hz 약 113 ms(+약 12 ms) |
| Eschmann 외, arXiv 2404.07837 | Crazyflie 2.1 | 비행 로그 MAP, 제조사 계단응답 | 72 ms(상승·하강 평균 73 ms). 3.35 kg 기체는 "훨씬 작다"고만 서술 |

**해석:** 같은 급(2216/880, 10″, 14.8 V)이 약 35 ms라서 exp_044의 80 ms(오디오, ±40 ms)는 **과대 추정일 가능성이 크다.** 하강은 능동 제동 없는 PWM ESC라 더 느릴 수 있다. 광학 회전수로 다시 재기 전까지 τ 사전은 넓게 두고, 측정 후 중심을 옮긴다.
