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
- Isaac에는 모터 모델이 없으므로 모터 지연은 PX4 속도 스텝 응답 동정(P0)으로 통째로 맞춘다. 별도 모터 모델을 넣지 않는다.
- 비행 전 PX4 자세 게인은 기본값보다 보수적으로 시작한다(모터가 느리면 같은 게인에서 위상 여유가 줄어든다).

## 향후 조건
- 모터·ESC·프롭 중 하나라도 바꾸면 다시 잰다(`hardware/bench/motor_step/`).
- 정밀값이 필요하면 광학 회전수 센서나 DShot 회전수 피드백으로 다시 잰다. 오디오 방식은 τ ±40 ms가 한계였다.
