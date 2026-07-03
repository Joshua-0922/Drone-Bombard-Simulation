---
date: 2026-06-15
tags: [error, CRUISE-timeout, arming, PX4, EKF, throughput]
status: resolved
type: error
---

# Err — CRUISE 타임아웃 (Teleport 후 PX4 Arm 거부)

> **발견/수정 run:** [[experiments/exp_005_rl_yolo_v12_arm_fix_arming-throughput-fix]]
> **근본 원인 분석:** [[research/cruise_timeout_arming]]

---

## 증상

- `rl_yolo_v11_cam_fix`(k1uqgs8i) 17.6h 중 443건 CRUISE 타임아웃.
- fps 낮음; 타임아웃당 ≈ 45s 낭비 → 총 ~5.5h wall-clock.
- `drone_controller` 로그에 "--> Arming Drone"이 2s마다 반복되나 드론은 TAKEOFF에 고착.

---

## 진단

9123건 `mission_manager` 기동 집계:

- CRUISE 도달 69.6% / **NEVER ARMED 28.2%** / armed-but-no-climb 2.1%.
- 443건 타임아웃 전부 attempt 1/3 (attempt 2/3은 0건) → fresh restart가 항상 복구.

**결론:** "느린 비행"이 아니라 **arming-rejection**. teleport(`gz_reset_poses`) 직후
stale EKF → `pre_flight_checks_pass=False` → PX4가 arm 거부.

> ⚠️ 정정: `cruise_poll_timeout`은 이미 20.0s였음 ("60s"는 fallback 기본값 오독).
> 타임아웃 값 낮추기는 no-op. 진짜 레버 = 타임아웃 횟수 감소.

---

## 해결책 (3 수정)

### #3 진단 로깅 — `drone_controller_node.py`

`/fmu/out/vehicle_command_ack` 구독 → arm 거부 시
`ARM REJECTED by PX4: <DENIED/TEMPORARILY_REJECTED/...>`를
`pre_flight_checks_pass`/`nav_state`/`failsafe`와 함께 로깅.

### #2 소스 예방 — `drone_controller_node.py`

고정 5s warmup 후 무조건 스팸 대신 `vehicle_status.pre_flight_checks_pass`로
arm() 게이팅. `Delaying arm — pre_flight_checks_pass=False` 1회 로깅.

### #4 즉시 복구 — `drone_drop_env.py`

bridge가 `/fmu/out/vehicle_status` 구독(`arming_state` 추적).
`arm_bail_timeout=10.0s`(신규 config) 내 arm 안 되면 즉시 full infra restart —
20s cruise 타임아웃을 끝까지 기다리지 않음.

### Config — `hyperparams.yaml`

- `arm_bail_timeout: 10.0` 추가
- `run_name`: `rl_yolo_v11_cam_fix` → `rl_yolo_v12_arm_fix`

---

## 검증

- `colcon build` clean (`drone_controller`, `rl_navigation --symlink-install`).
- Dry-run (400 steps, `/workspace/dryrun_v11cam.log`): 에러 0;
  early-bail 1회 발화 후 복구; full CRUISE 타임아웃 0건; TRACKING 3회; "Training complete".
- Full run `rl_yolo_v12_arm_fix` 정상 startup (`/workspace/train_v12.log`).

---

## 관련 노트

- [[research/cruise_timeout_arming]]
- [[experiments/exp_005_rl_yolo_v12_arm_fix_arming-throughput-fix]]
- [[research/rl_rules]] — Rule 8
