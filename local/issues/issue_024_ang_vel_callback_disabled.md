================================================================================
 Issue #024 — PX4 ang_vel callback disabled (dds_topics 주석)
================================================================================
상태   : 해결됨 (2026-06-22)
발견일 : 2026-06-22
해결일 : 2026-06-22
파일   : issue_024_ang_vel_callback_disabled.md
연관   : #019 (SAC entropy — limit_ang_vel 의 crash detection 관련), v9a (#028 처방의 prerequisite)


================================================================================
 1. 문제점
================================================================================

  v9a 의 처방 (drop 시점 drone 의 angular acceleration penalty) 검증을 위해
  v8 학습 데이터 (drop_episodes 의 npz) 의 obs 분석.

  ```
  v8 학습 500 ep sample 의 obs[:, 6:9] (ang_vel 부분):
    mean: 0.000000
    max:  0.000000
    std:  0.000000
    >0 인 비율: 0.00%

  비교 (linear vel obs[:, 3:6]):
    mean: 0.02 (정상 분포)
  ```

  → **모든 step 의 ang_vel obs 가 정확히 0**.

  v8 (303k step, 8,736 ep, success 80.6%) 학습 전체가 ang_vel obs 없이
  진행됐는데도 80% success 달성. 정책이 다른 obs (pos, vel, CCIP) 만으로
  toss 전략 학습.


================================================================================
 2. 분석
================================================================================

  PX4 → ROS2 bridge 진단:

  ```bash
  ros2 topic info /fmu/out/vehicle_angular_velocity -v
  # Type: px4_msgs/msg/VehicleAngularVelocity
  # Publisher count: 0     ← root cause
  # Subscription count: 1  ← env subscriber 정상
  ```

  다른 fmu/out topic 들은 publisher 정상:
  - /fmu/out/vehicle_local_position: 1 publisher
  - /fmu/out/vehicle_attitude: 1 publisher
  - /fmu/out/vehicle_status: 1 publisher
  - /fmu/out/vehicle_angular_velocity: **0 publishers**

  PX4 의 micro-XRCE-DDS bridge 설정 확인:
  `/opt/PX4-Autopilot/src/modules/uxrce_dds_client/dds_topics.yaml`

  ```yaml
  publications:
  #  - topic: /fmu/out/vehicle_angular_velocity    ← 주석 처리됨!
  #    type: px4_msgs::msg::VehicleAngularVelocity
    - topic: /fmu/out/vehicle_attitude            ← 활성
    - topic: /fmu/out/vehicle_local_position
    - topic: /fmu/out/vehicle_status
  ```

  → **PX4 default 에서 vehicle_angular_velocity 가 의도적으로 disable**.

  PX4 maintainers 의 판단:
    - quaternion attitude 의 derivative 가 ang_vel
    - bridge 토픽 수 줄여 throughput 보장
    - 대부분의 사용자가 ang_vel 직접 필요 안 함

  RL 학습에는 명백한 손실:
    - obs space 17 차원 중 3 차원 (obs[6:9]) 이 항상 0
    - crash detection (line 848 의 omega_mag > limit_ang_vel) 무력화
    - drop 시점 stability penalty 의 omega term 비활성


================================================================================
 3. 영향 평가
================================================================================

  3-1. v8 정책에의 영향:
    - 학습 시 obs[6:9] = 0 → policy network 의 그 input channel weights 가
      sparse / noise (학습 신호 없음)
    - Fix 후 real value (0.1~5 rad/s) 받아도 큰 변화 없음 (검증됨, 80% success 유지)

  3-2. crash detection (line 848):
    - 이전: omega_mag > limit_ang_vel(2.0) 검사 → 항상 False (omega_mag=0)
    - 즉 episode 중 ang_vel 폭주에 의한 truncation 0 회
    - 실측 max ang_vel 4.89 rad/s (toss pitch back) — 만약 ang_vel obs 가 정상이었다면
      false crash trigger 빈번 (2.0 limit 초과)

  3-3. v9 처방 (drop_angaccel penalty) 의 prerequisite:
    - drop 시점 직전 N step 의 max ang_accel penalty
    - obs 가 0 이면 ang_accel = 0 → penalty 계산 불가
    - 이 fix 가 필수


================================================================================
 4. 해결 방안
================================================================================

  4-1. PX4 dds_topics.yaml uncomment:

  ```bash
  docker exec drone-bombard-harmonic bash -c "
  sed -i 's|^  # - topic: /fmu/out/vehicle_angular_velocity\$|  - topic: /fmu/out/vehicle_angular_velocity|' \\
      /opt/PX4-Autopilot/src/modules/uxrce_dds_client/dds_topics.yaml
  sed -i 's|^  #   type: px4_msgs::msg::VehicleAngularVelocity\$|    type: px4_msgs::msg::VehicleAngularVelocity|' \\
      /opt/PX4-Autopilot/src/modules/uxrce_dds_client/dds_topics.yaml
  "
  ```

  4-2. PX4 rebuild:

  ```bash
  docker exec drone-bombard-harmonic bash -c "cd /opt/PX4-Autopilot && make px4_sitl_default"
  # ~10-15분, binary 51 MB
  ```

  4-3. hyperparams.yaml: limit_ang_vel 변경:

  - `reward.limit_ang_vel`: 2.0 → 10.0
  - 이유: line 848 의 crash detection. 실측 max ang_vel 4.89 rad/s (toss pitch back peak).
         2.0 limit 이면 false crash trigger.

  4-4. install/share sync:

  ```bash
  cp ros2_ws/src/rl_navigation/config/hyperparams.yaml \\
     ros2_ws/install/rl_navigation/share/rl_navigation/config/hyperparams.yaml
  cp ros2_ws/src/rl_navigation/rl_navigation/drone_drop_env.py \\
     ros2_ws/install/rl_navigation/lib/python3.10/site-packages/rl_navigation/drone_drop_env.py
  ```


================================================================================
 5. 검증
================================================================================

  5-1. 1 ep 평가 (dgui, v8_peak):
    spawn ang_vel = (-0.146, -0.134, -0.001)   ← non-zero ✓
    drop  ang_vel = (0.003, -0.032, 0.273)      ← 정책이 drop 시점 안정화
    max ang_vel mag = 4.890 rad/s               ← toss pitch back peak
    drop_err = 1.92m ✓

  5-2. 5 ep 평가 (v8_peak):
    EP1: 1.82m ✓, max ang_vel 2.66
    EP2: 1.87m ✓, max ang_vel 2.31
    EP3: 1.95m ✓, max ang_vel 2.48
    EP4: 1.65m ✓, max ang_vel 2.36
    EP5: 1.96m ✓, max ang_vel 2.13

    success 5/5 = 100% ✓
    mean err 1.852m
    max ang_vel 평균 2.39 rad/s

  → v8 정책 동작 영향 미미 (학습 시 항상 0 이었으므로 weights 가 그 input 에 무지).


================================================================================
 6. 측정된 toss dynamics (fix 후)
================================================================================

  | 시점                  | ang_vel magnitude        |
  |----------------------|--------------------------|
  | spawn (drone 정지)    | 0.05 ~ 0.25 rad/s        |
  | toss pitch back peak | **2.1 ~ 2.7 rad/s** (avg)|
  |                      | 최대 4.89 rad/s (single)  |
  | drop 직전             | < 0.5 rad/s (안정화 후)   |

  정책이 자연스럽게 drop 직전 drone 안정화 학습 — 사용자 의도 (드롭 시 부드러움)
  의 일부가 이미 달성됨.


================================================================================
 7. Backup (rollback 시)
================================================================================

  - container: /tmp/ang_vel_fix_backup/dds_topics.yaml
  - host: local/backups/hyperparams_v8_pre_angvel_fix_20260622_042523.yaml

  Rollback 절차:
  ```bash
  docker exec drone-bombard-harmonic bash -c "
  cp /tmp/ang_vel_fix_backup/dds_topics.yaml \\
     /opt/PX4-Autopilot/src/modules/uxrce_dds_client/dds_topics.yaml
  cd /opt/PX4-Autopilot && make px4_sitl_default
  "
  cp local/backups/hyperparams_v8_pre_angvel_fix_20260622_042523.yaml \\
     ros2_ws/src/rl_navigation/config/hyperparams.yaml
  ```


================================================================================
 8. 결정 이력
================================================================================

  2026-06-22 (자율 진행, 사용자 수면 중):
    - 발견 (dgui 의 v9 처방 검증 중 ang_vel 분석)
    - root cause 진단 (dds_topics.yaml 주석)
    - Fix 적용 (uncomment + rebuild + limit_ang_vel 10)
    - 1 ep + 5 ep 검증 (v8 정책 영향 없음 확인)
    - v9a (#028) 의 drop_angaccel penalty 의 prerequisite 로 확보

  2026-06-26 (v9a 학습 시작):
    - drop_angaccel_penalty_scale = 0.5, drop_angaccel_window_n = 5
    - 검증 (5 ep 평가): max ang_vel 2.10 rad/s (v8 의 2.5 대비 -16%)
    - 처방 효과 명확
