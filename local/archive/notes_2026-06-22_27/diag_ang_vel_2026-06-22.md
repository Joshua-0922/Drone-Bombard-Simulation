# ang_vel callback 진단 + fix 보고서

**날짜**: 2026-06-22 (사용자 수면 중 자율 진행)
**결론**: v8 학습 전체가 ang_vel obs 없이 진행됨 (obs[6:9] 항상 0). PX4 의 dds_topics.yaml 에서 vehicle_angular_velocity 가 주석 처리되어 있던 게 root cause. Fix 적용 후 정상 작동 확인.

---

## 발견 경위

사용자가 v9 처방으로 "drop 시 drone 의 angular velocity 변화 penalty" 를 제안. 검증 위해 v8 학습 데이터 (drop_episodes 500 개) 의 ang_vel 분포 분석.

```
v8 학습 500 ep 의 obs[:, 6:9] (ang_vel) 분석:
  mean: 0.000000
  max:  0.000000
  >0 인 비율: 0.00%
```

→ **v8 학습 전체에서 ang_vel obs 가 항상 정확히 0**. linear vel 은 정상 작동.

## Root cause

PX4 의 micro-XRCE-DDS bridge 설정 (`/opt/PX4-Autopilot/src/modules/uxrce_dds_client/dds_topics.yaml`) 에서 `vehicle_angular_velocity` 가 **주석 처리됨**:

```yaml
# Before (PX4 default)
#  - topic: /fmu/out/vehicle_angular_velocity
#    type: px4_msgs::msg::VehicleAngularVelocity
  - topic: /fmu/out/vehicle_attitude
  - topic: /fmu/out/vehicle_local_position
  - topic: /fmu/out/vehicle_status
```

→ PX4 가 ROS2 로 publish 안 함. 다른 fmu/out 토픽 (position, attitude, status) 은 정상 publish.

## 진단 path (재현 가능)

```bash
# 1. 토픽 list 확인
docker exec drone-bombard-harmonic ros2 topic list | grep angular
# → /fmu/out/vehicle_angular_velocity (subscriber 1, publisher 0)

# 2. publisher count 확인
docker exec drone-bombard-harmonic ros2 topic info /fmu/out/vehicle_angular_velocity -v
# → Publisher count: 0   ← 핵심

# 3. dds_topics.yaml 확인
docker exec drone-bombard-harmonic grep -B 1 -A 2 vehicle_angular /opt/PX4-Autopilot/src/modules/uxrce_dds_client/dds_topics.yaml
# → 주석 처리됨
```

## Fix 적용

### 1. dds_topics.yaml 주석 제거

```bash
# 두 줄 모두 uncomment
sed -i 's|^  # - topic: /fmu/out/vehicle_angular_velocity\$|  - topic: /fmu/out/vehicle_angular_velocity|' \
    /opt/PX4-Autopilot/src/modules/uxrce_dds_client/dds_topics.yaml
sed -i 's|^  #   type: px4_msgs::msg::VehicleAngularVelocity\$|    type: px4_msgs::msg::VehicleAngularVelocity|' \
    /opt/PX4-Autopilot/src/modules/uxrce_dds_client/dds_topics.yaml
```

### 2. PX4 rebuild

```bash
docker exec drone-bombard-harmonic bash -c "cd /opt/PX4-Autopilot && make px4_sitl_default"
# → 10-15분, binary 51 MB
```

### 3. hyperparams.yaml — limit_ang_vel 변경

이유: line 848 의 `omega_mag > limit_ang_vel` crash detection. 기존 `limit_ang_vel: 2.0` 였으나 실측 max ang_vel = 4.89 rad/s (toss 의 pitch back 시점). fix 후 false crash 감지 trigger 위험 → `10.0` 로 변경.

```yaml
limit_ang_vel: 10.0   # 2026-06-22 ang_vel callback fix 후 false crash 방지
```

### 4. install/share sync

```bash
# src → install 의 hyperparams + drone_drop_env.py
cp ros2_ws/src/rl_navigation/config/hyperparams.yaml \
   ros2_ws/install/rl_navigation/share/rl_navigation/config/hyperparams.yaml
cp ros2_ws/src/rl_navigation/rl_navigation/drone_drop_env.py \
   ros2_ws/install/rl_navigation/lib/python3.10/site-packages/rl_navigation/drone_drop_env.py
```

## Fix 검증 (Step D: 1 ep)

```
spawn ang_vel = (-0.146, -0.134, -0.001)   ← non-zero ✓
drop  ang_vel = (0.003, -0.032, 0.273)      ← 정책이 drop 시점 drone 안정화 학습됨
max ang_vel mag = 4.890 rad/s               ← toss 의 pitch back 시점 측정값
drop_err = 1.92m ✓ success
```

→ **ang_vel callback 정상 작동 확인**.
→ **정책의 drop 시점 drone state**: 거의 정지 (ang_vel < 0.3 rad/s). 사용자 의도와 일치 — 정책이 drop 직전에 drone 안정화.
→ **toss 의 pitch back 시점**: max 4.89 rad/s. PX4 의 MC_PITCHRATE_MAX = 220 deg/s ≈ 3.84 rad/s 와 유사 (vector magnitude 라 약간 큼).

## Backup 위치

| 파일 | 위치 |
|---|---|
| dds_topics.yaml (원본) | `/tmp/ang_vel_fix_backup/dds_topics.yaml` (container 안) |
| hyperparams.yaml (변경 전) | `/home/juns/Drone-Bombard-Simulation/local/backups/hyperparams_v8_pre_angvel_fix_20260622_042523.yaml` |
| PX4 binary (이전) | rebuild 로 overwrite, backup 없음 (rebuild 명령으로 재생성 가능) |

## v8 정책에의 영향 분석

학습 시 obs[6:9] = 0 이었기 때문에, 정책의 policy network 의 ang_vel input channel weights 가 noise / 작은 값 (sparse signal). Fix 후 obs[6:9] 가 real value 가 됨.

가능 영향:
- A. 정책 weights 가 ang_vel input 에 무지 → 영향 거의 없음, success rate 같음 (가장 가능성 높음)
- B. weights 가 약간 random response → success rate 약간 변화 (분포 안)
- C. 정책 reasonably robust 라면 success rate 유지

5 ep 평가로 검증.

## v9 학습 처방에의 영향

이제 ang_vel obs 가 정상 → 사용자 제안 (drop 시 ang_vel penalty) 구현 가능.

```python
# v9 plan 예시:
# step() 안에 매 step ang_vel 추적
# drop 발생 시점에 max_ang_vel_mag penalty
reward -= drop_angvel_penalty_scale * max_ang_vel_mag_during_episode

# 또는 매 step time penalty (smooth)
reward -= ang_vel_penalty_scale * |ang_vel|
```

## Step E: 5 EP 평가 결과 (fix 후)

```
EP1: drop_err=1.82m ✓, steps=59, max_ang_vel=2.66 rad/s
EP2: drop_err=1.87m ✓, steps=42, max_ang_vel=2.31 rad/s
EP3: drop_err=1.95m ✓, steps=49, max_ang_vel=2.48 rad/s
EP4: drop_err=1.65m ✓, steps=50, max_ang_vel=2.36 rad/s
EP5: drop_err=1.96m ✓, steps=53, max_ang_vel=2.13 rad/s

success ≤2m: 5/5 = 100.0%
mean err:    1.852m
min/max:     1.654m / 1.964m
모든 drop 시점 ang_vel magnitude < 0.5 rad/s (drone 거의 안정)
```

## 비교 — fix 전후

| 평가 | success | mean | min | max |
|---|---|---|---|---|
| **fix 전** (settle 5s + D1 + H, 5 ep) | 5/5 = 100% | **1.719m** | 1.558m | 1.941m |
| **fix 후** (+ ang_vel callback + limit 10) | 5/5 = 100% | **1.852m** | 1.654m | 1.964m |

→ success rate 동일. mean 약간 큼 (1.72 vs 1.85). 분포 안 variability. fix 가 정책 동작에 큰 영향 없음.

## 결론

1. **v8 학습 전체가 ang_vel obs 없이 진행됨** — 모든 obs[6:9] = 0
2. **Root cause = PX4 dds_topics.yaml 의 vehicle_angular_velocity 주석 처리**
3. **Fix 적용 완료** — dds_topics.yaml uncomment + PX4 rebuild + limit_ang_vel 10.0
4. **v8 정책 동작 영향 미미** — fix 후 5/5 success 유지, mean 1.85m
5. **이제 v9 처방으로 drop 시 ang_vel penalty 구현 가능**

## 측정된 toss dynamics

| 시점 | ang_vel magnitude |
|---|---|
| spawn (drone 정지) | 0.05 ~ 0.25 rad/s |
| toss 의 pitch back 시점 | **max 2.1 ~ 2.7 rad/s** (5 ep 평균), 최대 4.89 rad/s (1 ep test) |
| drop 직전 | < 0.5 rad/s (drone 안정화 후 detach) |

→ 정책이 자연스럽게 drop 직전 drone 안정화 학습됨. 사용자가 원했던 "각속도 제한" 이 이미 일부 달성됨.

## v9 처방 후보 (사용자 결정 대기)

### A. 단순 fine-tune (warm start v8)
```yaml
# hyperparams.yaml 변경:
payload_distance_reward_scale: 0.01 ~ 0.05     # 처방 2 (사용자 제안)
drop_angvel_penalty_scale: 0.5                  # 처방 1 (사용자 제안)
```

```python
# env step() 안 추가:
# payload distance reward
payload_dist = ||payload_xy - target_xy||
delta = self._prev_payload_dist - payload_dist
reward += payload_distance_reward_scale * delta

# drop 시점 ang_vel penalty
if drop_triggered:
    reward -= drop_angvel_penalty_scale * max_ang_vel_during_episode
```

- v8 warm start, 새 replay buffer (old reward 와 충돌 방지)
- 50-100k step fine-tune
- 시간: 10-20시간
- 예상 결과: toss 의 정확도 개선 + ang_vel max 줄어듬

### B. 처방 plan 시작 전 추가 진단
- v8 정책의 ang_vel obs (이제 정상) 에 대한 weight 분석
- 또는 추가 5 ep 평가로 mean error 변동성 확인

## Backup 위치 (rollback 시)

- container: `/tmp/ang_vel_fix_backup/dds_topics.yaml`
- host: `local/backups/hyperparams_v8_pre_angvel_fix_20260622_042523.yaml`

Rollback 방법:
```bash
docker exec drone-bombard-harmonic bash -c "
cp /tmp/ang_vel_fix_backup/dds_topics.yaml \
   /opt/PX4-Autopilot/src/modules/uxrce_dds_client/dds_topics.yaml
cd /opt/PX4-Autopilot && make px4_sitl_default
"
cp local/backups/hyperparams_v8_pre_angvel_fix_20260622_042523.yaml \
   ros2_ws/src/rl_navigation/config/hyperparams.yaml
cp ros2_ws/src/rl_navigation/config/hyperparams.yaml \
   ros2_ws/install/rl_navigation/share/rl_navigation/config/hyperparams.yaml
```

---
*자율 진행 완료: 2026-06-22 04:25 ~ ~04:50 (사용자 수면 중)*
*총 진행: 25분 (PX4 rebuild 15분 + 평가 10분)*

## 평가 결과 파일들

```
local/eval_logs/
├── eval_2026-06-22T01-54-12_v8_peak_step217040_err0.87m.json   # 초기 5 ep
├── eval_2026-06-22T02-15-22_v8_peak_step217040_err0.87m.json   # settle 2s fix
├── eval_2026-06-22T02-40-39_v8_peak_step217040_err0.87m.json   # H+E+G
├── eval_2026-06-22T03-14-15_v8_peak_step217040_err0.87m.json   # E 처방 5 ep
├── eval_2026-06-22T03-21-52_v8_peak_step217040_err0.87m.json   # settle 5s 복원 5 ep
├── eval_2026-06-22T03-29-00_v8_best_step157201_err0.07m.json   # best 모델
├── eval_2026-06-22T03-52-36_v8_peak_step217040_err0.87m.json   # camera SDF 5 -5 5
├── eval_2026-06-22T13-43-40_v8_peak_step217040_err0.87m.json   # ang_vel fix 1 ep (Step D)
└── eval_<2026-06-22T_5ep>...json                               # ang_vel fix 5 ep (Step E)
```
