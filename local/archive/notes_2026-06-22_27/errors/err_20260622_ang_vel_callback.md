---
date: 2026-06-22
tags: [error, px4, ros2, ang_vel, dds_topics, observation, learning_bug]
status: fixed
type: error
---

# PX4 `vehicle_angular_velocity` topic — publisher 0 (callback dead-loop)

## 증상

v8 학습된 모델의 평가 중 obs 분석:

```
v8 학습 전체 500 ep sample 의 obs[:, 6:9] (ang_vel 부분):
  mean: 0.000000
  max:  0.000000
  std:  0.000000
  >0 인 비율: 0.00%
```

**모든 step 의 ang_vel obs 가 정확히 0**. linear vel 은 정상.

## Root cause

`/opt/PX4-Autopilot/src/modules/uxrce_dds_client/dds_topics.yaml`:

```yaml
# PX4 default
publications:
#  - topic: /fmu/out/vehicle_angular_velocity    ← 주석 처리됨
#    type: px4_msgs::msg::VehicleAngularVelocity
  - topic: /fmu/out/vehicle_attitude            ← 활성
  - topic: /fmu/out/vehicle_local_position
  - topic: /fmu/out/vehicle_status
```

→ PX4 의 micro-XRCE-DDS bridge 가 `vehicle_angular_velocity` topic 을 ROS2 로 publish 안 함. 다른 fmu/out topic 은 publish.

```bash
# 확인
ros2 topic info /fmu/out/vehicle_angular_velocity -v
# Publisher count: 0     ← root cause
# Subscription count: 1  ← env subscriber 정상
```

PX4 maintainers 가 의도적으로 disable (attitude 와 redundant — quaternion 의 derivative).

## 영향

| 시스템 부분 | 영향 |
|---|---|
| **obs[6:9]** | 항상 0 → 정책의 ang_vel feature 학습 안 됨 |
| `_compute_reward` line 848 | `omega_mag > limit_ang_vel` 항상 False → crash detection 무력 |
| **v8 학습 전체** | 8,736 ep + 303k step 모두 ang_vel 없이 학습됨 |
| v8 정책 동작 | success 80% 달성 — 다른 obs (pos, vel, CCIP) 만으로 toss 학습 |

## Fix

### 1. dds_topics.yaml 주석 제거

```bash
docker exec drone-bombard-harmonic bash -c "
sed -i 's|^  # - topic: /fmu/out/vehicle_angular_velocity\$|  - topic: /fmu/out/vehicle_angular_velocity|' \\
    /opt/PX4-Autopilot/src/modules/uxrce_dds_client/dds_topics.yaml
sed -i 's|^  #   type: px4_msgs::msg::VehicleAngularVelocity\$|    type: px4_msgs::msg::VehicleAngularVelocity|' \\
    /opt/PX4-Autopilot/src/modules/uxrce_dds_client/dds_topics.yaml
"
```

### 2. PX4 rebuild

```bash
docker exec drone-bombard-harmonic bash -c "cd /opt/PX4-Autopilot && make px4_sitl_default"
# 10-15분, binary 51 MB
```

### 3. hyperparams.yaml: `limit_ang_vel: 2.0 → 10.0`

이유: line 848 의 `omega_mag > limit_ang_vel` crash detection.
- 기존 `2.0` 이지만 실측 max ang_vel = 4.89 rad/s (toss pitch back peak).
- fix 후 false crash trigger 위험 → `10.0` 으로 완화.

### 4. install/share sync

```bash
cp ros2_ws/src/rl_navigation/config/hyperparams.yaml \
   ros2_ws/install/rl_navigation/share/rl_navigation/config/hyperparams.yaml
cp ros2_ws/src/rl_navigation/rl_navigation/drone_drop_env.py \
   ros2_ws/install/rl_navigation/lib/python3.10/site-packages/rl_navigation/drone_drop_env.py
```

## 검증

### 1 ep test

```
spawn ang_vel = (-0.146, -0.134, -0.001)   ← non-zero ✓
drop  ang_vel = (0.003, -0.032, 0.273)      ← drop 시점 거의 정지
max ang_vel mag = 4.890 rad/s               ← toss pitch back peak
drop_err = 1.92m ✓
```

### 5 ep test (v8 정책 영향 검증)

```
EP1: 1.82m ✓ (max ang_vel 2.66)
EP2: 1.87m ✓ (max ang_vel 2.31)
EP3: 1.95m ✓ (max ang_vel 2.48)
EP4: 1.65m ✓ (max ang_vel 2.36)
EP5: 1.96m ✓ (max ang_vel 2.13)

success ≤2m: 5/5 = 100%
mean err:    1.852m
```

→ v8 정책 동작 영향 미미 (fix 전 5 ep 결과 4/5 와 통계 안). 정책 weights 가 ang_vel input 에 무지 (학습 시 항상 0) → fix 후 real value 받아도 큰 변화 없음.

## 측정된 toss dynamics (fix 후)

| 시점 | ang_vel mag |
|---|---|
| spawn (drone 정지) | 0.05 ~ 0.25 rad/s |
| **toss pitch back peak** | **max 2.1 ~ 2.7 rad/s** (5 ep), 최대 4.89 rad/s |
| drop 직전 | < 0.5 rad/s (drone 안정화 후 detach) |

## Backup (rollback 시)

| 파일 | 위치 |
|---|---|
| dds_topics.yaml (원본) | `/tmp/ang_vel_fix_backup/dds_topics.yaml` (container) |
| hyperparams.yaml (fix 전) | `local/backups/hyperparams_v8_pre_angvel_fix_20260622_042523.yaml` |
| PX4 binary | rebuild 명령으로 재생성 |

## Rollback 절차

```bash
# 1. dds_topics 복원
docker exec drone-bombard-harmonic bash -c "
cp /tmp/ang_vel_fix_backup/dds_topics.yaml \\
   /opt/PX4-Autopilot/src/modules/uxrce_dds_client/dds_topics.yaml
cd /opt/PX4-Autopilot && make px4_sitl_default
"

# 2. hyperparams 복원
cp local/backups/hyperparams_v8_pre_angvel_fix_20260622_042523.yaml \\
   ros2_ws/src/rl_navigation/config/hyperparams.yaml
cp ros2_ws/src/rl_navigation/config/hyperparams.yaml \\
   ros2_ws/install/rl_navigation/share/rl_navigation/config/hyperparams.yaml
```

## v9a 학습에의 영향

이 fix 가 **v9a 의 drop_angaccel penalty 처방의 prerequisite** — ang_vel 측정 가능해야 penalty 계산 가능.

자세한 v9a 처방: [[experiments/exp_006_zjexq20k_v9a_payload_dist_angaccel]]

## 관련 노트

- [[research/toss_strategy_analysis]] — toss 의 ang_vel dynamics 분석
- [[experiments/exp_006_zjexq20k_v9a_payload_dist_angaccel]] — fix 후 ang_vel-aware 처방 실험
- [[research/rl_rules]] — Rule 추가: PX4 default 토픽 disable 확인
