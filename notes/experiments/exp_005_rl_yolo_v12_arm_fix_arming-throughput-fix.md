---
date: 2026-06-15
tags: [experiment, SAC, YOLO, throughput, arming, CRUISE-timeout, PX4]
status: in_progress
type: experiment
wandb_run: rl_yolo_v12_arm_fix
---

# Exp 005: rl_yolo_v12_arm_fix — Arming-Rejection Throughput Fix

## 실험 목표

`rl_yolo_v11_cam_fix` (k1uqgs8i) 분석에서 드러난 **CRUISE 타임아웃 throughput 병목**을 제거한다.
근본 원인은 teleport 직후 stale EKF로 인한 **PX4 arm 거부**였다.
→ 근본 원인 분석: [[research/cruise_timeout_arming]]

## 설정 (v12)

| 항목 | 값 |
|------|-----|
| WandB Run name | `rl_yolo_v12_arm_fix` |
| 로그 | `/workspace/train_v12.log` |
| Timesteps | 500,000 |
| 알고리즘 | SAC, SB3, `net_arch=[256,256]`, RTF=2 |
| 신규 config | `arm_bail_timeout: 10.0` (hyperparams.yaml) |
| run_name 변경 | `rl_yolo_v11_cam_fix` → `rl_yolo_v12_arm_fix` |
| `cruise_poll_timeout` | 20.0s (이미 20.0 — 변경 아님, 아래 정정 참조) |

## 선행 run 분석 — rl_yolo_v11_cam_fix (k1uqgs8i)

~42k / 500k steps, 17.6h 후 중단.

### 학습은 개선 중이었다 (진짜 신호)

| 메트릭 | 추세 | 판정 |
|--------|------|------|
| `ep_best_d_xy` | 0.92 → 0.68 m | ✅ |
| `mean_d_xy` | 4.21 → 1.62 m | ✅ |
| `env/ep_reward` (per-episode) | 20.4 → 53.8 | ✅ 진짜 상승 |
| `reached_close` (≤3m) | 1077 / 1162 ep (93%) | ✅ |
| successes (d_xy ≤ 0.5m) | 404 | ✅ |

> **⚠️ SB3 `rollout/ep_rew_mean`는 오해 유발:** 70 → 48로 *하락*. 이유는
> `ep_len`이 151 → 36.5로 붕괴했기 때문 — 드론이 마커에 빨리 도달해 에피소드가 일찍
> 종료된다. **per-episode `env/ep_reward` (20→54)가 진짜 상승 신호.**
> → 규칙화: [[research/rl_rules]] Rule 3 / Rule 9

### Throughput 근본 원인 (이번 세션의 초점)

443건 CRUISE 타임아웃. CRUISE 타임아웃 = 드론이 TAKEOFF에 고착, 절대 상승 못 함.

9123건 `mission_manager` 기동 분석:

| 결과 | 비율 | 의미 |
|------|------|------|
| CRUISE 도달 | 69.6% | 정상 |
| **NEVER ARMED** | **28.2%** | arm 거부 |
| armed-but-no-climb | 2.1% | drone flip 등 |

→ 본질은 **arming-rejection 문제**. `drone_controller`가 2s마다 "--> Arming Drone"을
스팸했지만 PX4가 거부 (teleport 후 stale EKF → `pre_flight_checks_pass=False`).

- 443건 타임아웃 **전부 "attempt 1/3"** (attempt 2/3에서는 0건) — fresh infra
  재시작이 항상 다음 시도에서 복구.
- 비용: 타임아웃당 ~20s 대기 + ~25s 재시작 ≈ 45s. 총 ~5.5h / 17.6h wall-clock.

### ⚠️ 중요 정정 — cruise_poll_timeout

`cruise_poll_timeout`은 config에 **이미 20.0s**였다. 이전의 "60s" 주장은
코드의 fallback 기본값 `cfg_env.get('cruise_poll_timeout', 60.0)`을 오독한 것.
**타임아웃을 낮추는 것은 no-op** — 진짜 레버는 타임아웃 *횟수*를 줄이는 것.

## 적용한 3가지 수정

### #3 — arm 거부 사유 로깅 (`drone_controller_node.py`)

`/fmu/out/vehicle_command_ack` 구독. arm 거부 시
`ARM REJECTED by PX4: <DENIED/TEMPORARILY_REJECTED/...>`를
`pre_flight_checks_pass` / `nav_state` / `failsafe`와 함께 로깅.
→ 다음 run이 정확한 PX4 거부 사유를 드러낸다.

### #2 — arm 게이팅 (`drone_controller_node.py`)

고정 5s warmup 후 무조건 스팸하는 대신 `vehicle_status.pre_flight_checks_pass`로
arm()을 게이팅. `Delaying arm — pre_flight_checks_pass=False`를 1회 로깅.
→ 거부를 **소스에서** 방지.

### #4 — early-bail stuck takeoff (`drone_drop_env.py`)

bridge 노드가 `/fmu/out/vehicle_status`를 구독 (`arming_state` 추적).
`_wait_for_cruise`가 armed 베이스라인을 리셋한 뒤, 신규 config key
`arm_bail_timeout=10.0s` 내에 PX4가 arm 안 하면 즉시 bail → 20s cruise 타임아웃을
끝까지 기다리지 않고 **즉시 full infra restart**.

### 코드 변경 파일

- `ros2_ws/src/drone_controller/drone_controller/drone_controller_node.py` (#2, #3)
- `ros2_ws/src/rl_navigation/rl_navigation/drone_drop_env.py` (#4)
- `ros2_ws/src/rl_navigation/config/hyperparams.yaml` (`arm_bail_timeout: 10.0`, run_name)

## 검증

- `colcon build` clean (`drone_controller`, `rl_navigation --symlink-install`);
  py/yaml syntax + `px4_msgs` import OK.
- **Dry-run** (400 steps, `/workspace/dryrun_v11cam.log`): 에러 없음;
  early-bail (#4) 1회 발화 후 복구 ("not armed after" =1); full CRUISE 타임아웃 0건;
  TRACKING 3회 도달; "Training complete".
- **Full run 기동:** `rl_yolo_v12_arm_fix` 정상 startup (PX4 ready, Starting training,
  TRACKING reached), `/workspace/train_v12.log`.

## 다음 단계 (Open Issues)

### ⚠️ OPEN — YOLO per-step trigger target_lost_rate ~29% (이번 세션 미해결)

`env/target_lost_rate`가 **bimodal**:

- 70.7% 에피소드: 모든 step에서 YOLO 탐지 (rate=0)
- 29.3% 에피소드: 어떤 step에서도 탐지 없음 (rate=1)
- 0% partial

→ 약 29% step에서 YOLO가 기여 없음 (`obs[9-11]` zeroed **AND** `-10` target_lost
페널티). 추세 **악화 중: 0.24 → 0.35.**
이번 세션에서 **수정 안 함** — 별도 처리 필요.
→ 규칙: [[research/rl_rules]] Known Failure Modes

### 기타

- v12 full run 모니터링 — arm rejection 사유 로그 (#3) 확인 후 게이팅(#2) 효과 검증.
- CRUISE 타임아웃 횟수가 실제로 감소했는지 wall-clock fps로 확인.

## 관련 노트

- [[research/cruise_timeout_arming]] — 근본 원인 분석 (이 세션의 신규 발견)
- [[errors/err_20260615_cruise-timeout-arming]] — 진단 + 수정 기록
- [[research/rl_rules]] — Rule 8 (arm 게이팅 + early-bail), Rule 9 (ep_rew_mean 오해)
- [[experiments/training_history]]
- [[experiments/exp_004_rl_yolo_debug_vision]] — 선행 vision run
