---
date: 2026-06-15
tags: [research, throughput, CRUISE-timeout, arming, PX4, EKF, teleport]
status: active
type: research
---

# CRUISE 타임아웃의 근본 원인 — Teleport 후 PX4 Arm 거부 (Stale EKF)

> **발견 run:** [[experiments/exp_005_rl_yolo_v12_arm_fix_arming-throughput-fix]] (선행 분석: `rl_yolo_v11_cam_fix` / k1uqgs8i)

---

## 요약

CRUISE 타임아웃은 "느린 비행" 문제가 아니라 **arming-rejection** 문제다.
드론이 TAKEOFF에 고착되어 절대 상승하지 못하는 것이며, 원인은 `gz_reset_poses`
teleport 직후 **stale EKF** 상태에서 PX4가 arm을 거부하기 때문이다.

---

## 증상

- `rl_yolo_v11_cam_fix`(k1uqgs8i) 17.6h 중 443건 CRUISE 타임아웃.
- 각 타임아웃 ≈ 45s (20s 대기 + 25s 재시작) → 총 ~5.5h / 17.6h wall-clock 낭비.
- fps가 낮게 유지됨 (full infra restart 반복).

---

## 진단 데이터

9123건 `mission_manager` 기동을 집계:

| 결과 | 비율 |
|------|------|
| CRUISE 도달 | 69.6% |
| **NEVER ARMED** | **28.2%** |
| armed-but-no-climb | 2.1% |

추가 관찰:

- `drone_controller`가 2s마다 "--> Arming Drone"을 스팸했으나 PX4 거부.
- 거부 원인: teleport 후 `pre_flight_checks_pass=False` (stale EKF).
- 443건 타임아웃 **전부 attempt 1/3** — attempt 2/3에서는 0건.
  → fresh infra 재시작이 다음 시도에서 항상 EKF를 재수렴시켜 복구.

---

## 원인 분석

1. 에피소드 종료 시 `gz_reset_poses`로 드론을 teleport.
2. EKF는 teleport를 즉각 반영하지 못함 (위치 점프 → innovation 큼 → 사전점검 실패).
3. PX4 `pre_flight_checks_pass=False` 동안 모든 arm 명령을 **DENIED/TEMPORARILY_REJECTED**.
4. `drone_controller`는 상태 확인 없이 고정 5s warmup 후 무조건 arm 스팸 → 무한 거부.
5. `_wait_for_cruise`는 arm 여부와 무관하게 20s를 끝까지 대기 → 타임아웃 → full restart.

---

## ⚠️ 정정 — cruise_poll_timeout

`cruise_poll_timeout`은 config에 **이미 20.0s**였다. "60s"라는 이전 주장은
코드의 fallback 기본값 `cfg_env.get('cruise_poll_timeout', 60.0)` 오독.
**타임아웃 값 자체는 레버가 아니다** — 진짜 레버는 타임아웃 *횟수* 감소.

---

## 적용 규칙 (수정)

| # | 위치 | 내용 |
|---|------|------|
| #3 진단 | `drone_controller_node.py` | `/fmu/out/vehicle_command_ack` 구독 → `ARM REJECTED by PX4: <reason>` + pre_flight_checks_pass/nav_state/failsafe 로깅 |
| #2 예방 | `drone_controller_node.py` | `vehicle_status.pre_flight_checks_pass=True`일 때만 arm() (고정 warmup 스팸 제거). `Delaying arm — pre_flight_checks_pass=False` 1회 로깅 |
| #4 복구 | `drone_drop_env.py` | bridge가 `/fmu/out/vehicle_status` 구독, `arm_bail_timeout=10.0s` 내 arm 안 되면 즉시 full infra restart (20s 대기 회피) |

→ 규칙화: [[research/rl_rules]] Rule 8

---

## 향후 조건 / 검증

- v12 run에서 #3 로그로 PX4 거부 사유 확인 → 게이팅(#2)이 거부를 실제로 제거했는지.
- NEVER ARMED 비율이 28.2%에서 유의하게 감소하는지.
- (장기) teleport 후 EKF 재수렴을 능동적으로 트리거(예: 명시적 EKF reset)하면 #4
  early-bail조차 불필요해질 수 있음.

---

## 06-17 정정 — v12 "수정"은 증상만 잡았고, 10s 컷이 진짜 병목이었다

> **발견 run:** [[experiments/exp_006_xgzum51v_armdiag_dryrun]] (armdiag dry-run, 25s 타임아웃 계측)

v13 학습 로그(46y4xtiw)에서 `ARM REJECTED`는 **0회**였다. v12 게이팅(#2)이 실제 arm 거부는 제거했고, 남은 현상은 컨트롤러가 `pre_flight_checks_pass=False`로 **대기**하다 bridge가 10s에 bail하는 것. 즉 "arm-reject suspected"는 오해의 소지 — 실제는 **EKF가 10s 안에 재수렴 실패**.

`pre_flight_checks_pass` 첫 True까지 시간을 계측하니 **bimodal**:

- **0.0s** (7/12, warm infra) vs **13.0–15.7s** (5/12 ≈ 42%, stuck-EKF cohort, 좁게 군집)
- 25s 타임아웃에서 **bail 0 / SUCCESS 4** — late cohort가 **전부 회복**.

→ **결론 정정:** stuck-EKF resets는 *full-restart-only가 아니라 recoverable-with-time*. v12의 `arm_bail_timeout=10.0`이 복구(13–16s) 직전에 단두대질 → 멀쩡한 PX4 버리고 full restart 강제. **이것이 진짜 throughput 싱크.**

**적용:** `hyperparams_v13.yaml` `arm_bail_timeout: 10.0 → 20.0` (max 관측 15.7s + 마진). → [[research/rl_rules]] Rule 11.

(#4 early-bail 자체는 유지 — 진짜 죽은 인프라는 20s에 fast-fail. 장기적으로 teleport 후 EKF 재수렴(13–16s)을 줄이는 것이 다음 과제.)

---

## 관련 노트

- [[experiments/exp_005_rl_yolo_v12_arm_fix_arming-throughput-fix]]
- [[experiments/exp_006_xgzum51v_armdiag_dryrun]] — 10s→20s 근거 (bimodal 재수렴 계측)
- [[errors/err_20260615_cruise-timeout-arming]]
- [[research/rl_rules]] — Rule 8, Rule 11, Known Failure Modes
- [[research/rtf_fps_analysis]] — RTF/FPS 분석 (별개의 throughput 레버)
