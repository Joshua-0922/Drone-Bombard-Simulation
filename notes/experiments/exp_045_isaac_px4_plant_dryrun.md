---
date: 2026-11-11
tags: [experiment, sim2real, isaac, px4, controller, motor, dry-run]
status: complete
type: experiment
wandb_run: "none (tensorboard dry-run, /tmp/dry80/logs in isaac-verify)"
---

# exp_045 — Isaac 플랜트를 PX4 경로·실측 상수로 교체: 제어 루프 점검과 80 iteration dry-run

> **한 줄.** 실측 로터 지연(80 ms)을 넣자 기존 각속도·자세 이득(18/6.5, X500 SITL 12.5 ms 로터 기준)이 진동했다. PX4 모드 전용 이득을 12/4.5로 낮췄다. 무작위 행동에서는 여전히 2 rad/s 자세 종료가 지배적이지만, 학습은 진행된다(80 iteration에서 성공 0 → 38%, 자세 종료 94 → 26%).

관련: [[research/sim2real_hardware_f450]] §0 · [[research/f450_motor_dynamics]] · [[experiments/exp_044_f450_motor_step_response]] · [[experiments/exp_042_px4_controller_noretrain]] · [[research/rl_rules]] (Rule 49) · [[experiments/training_history]]

## 1. 코드 변경 (Isaac만, Gazebo는 ROS Jazzy 이전 때)

- `controller.mode` 기본값 `"p"` → `"px4"`. 추가: 각속도 PID(I·D, 적분 한계, D항 LPF 30 Hz, yaw 토크 LPF 2 Hz), heading 유지(MC_YAW_P 2.8), Quad X 믹서 + 모터별 포화 + 로터 1차 지연(상승·하강 분리). `--controller p`는 구 플랜트(P 루프 + 직접 렌치)를 재현한다.
- 실측 상수: 질량, T/W, 투하 지연(성분 합 + 기계 지연 19회 재표집), 링크·관측·YOLO 지연, 바람 U[0, 6] m/s + Dryden 돌풍, 속도 관측 잡음(m/s). 값은 [[research/sim2real_hardware_f450]] §0.7.
- 단위테스트 107개 통과(신규: 바람·Dryden·투하 지연 평균 보존·재표집·믹서·로터 지연).

## 2. 폐루프 점검 (`_probe_ctrl.py`, 1024 env, 150 step)

| 조건 | 영(0) 속도 명령 종료율 | 무작위 행동 종료율 |
|---|---|---|
| 구 플랜트 (P, 모터 없음) | 8.9% | 27–29% |
| PX4, 모터 없음 | 3.5% | 11% |
| PX4 + 로터 80 ms, 이득 18/6.5 | 33–40% | — |
| PX4 + 로터 80 ms, 이득 12/4.5 (채택) | 15.5% | 92–100% |

로터 τ 민감도(DR 끔, 무작위 행동): 5 ms 12%(믹서 정상), 30 ms 42%, 50 ms 82%, 80 ms 100%. D 비율(0.02–0.15)·yaw 채널 제거는 효과가 없었다. 종료는 모두 `bad_attitude`(각속도 > 2 rad/s)다. PX4 자체 한계는 220°/s ≈ 3.8 rad/s다.

부수 버그(수정): 관측 지연 버퍼가 리셋 직후 이전 에피소드 상태를 보여 줬다 → 리셋 시 무효화.

## 3. 학습 dry-run (2048 env, 80 iter, `--dr_scale 1.0`)

| iter | 평균 보상 | 에피소드 길이 | bad_attitude | success |
|---|---|---|---|---|
| 0 | −40.5 | 34 | 0.94 | 0.00 |
| 16 | −71.0 | 118 | 0.50 | 0.00 |
| 32 | −28.8 | 65 | 0.43 | 0.24 |
| 48 | −13.2 | 54 | 0.32 | 0.40 |
| 79 | −8.9 | 48 | 0.26 | 0.38 |

학습은 진행된다. 다만 자세 종료가 26% 남아 있어 본 학습 곡선에서 계속 확인해야 한다.

## 4. 남은 것

- 이득 12/4.5는 **보정 노브**다. 실기 PX4 이득(autotune `MC_AT_APPLY=0` 로그, G1 속도 계단응답)으로 교체한다.
- 로터 τ 80 ms는 오디오 측정(±40 ms)이다. 광학 회전수로 다시 재면 이득과 종료율이 같이 바뀐다.
- 미구현: 관성 스폰 변형(env별 prim 저작 필요, 경미), 각속도 적분기의 할당 포화 피드백.
