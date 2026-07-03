---
date: 2026-07-03
tags: [research, isaac-lab, controller, px4, calibration, migration]
status: active
type: research
---

# Isaac Lab 속도 컨트롤러 — PX4 게인 매핑 + 검정 상태

> **한 줄:** Isaac 측 액추에이션은 PX4 내부 루프를 그대로 흉내낸 캐스케이드
> 속도→자세→토크 컨트롤러(단일 rigid-body wrench)다. 게인은 **초기값**이며
> 아직 PX4 SITL 실측 스텝응답과 대조 검정되지 않았다. 검정 전까지 v15-vs-Isaac
> 행동 비교는 이 한계를 감안해야 한다.

관련: [[experiments/exp_012_isaac_migration_phase2]] · [[research/control_smoothness_wobble]] (Rule 15)

---

## 왜 rigid-body wrench인가 (per-rotor 아님)

PX4의 로터/믹서 내부 루프는 학습된 정책의 plant에 한 번도 포함된 적 없다 —
정책이 실제로 본 것은 "속도 setpoint → EKF 상태"뿐이다. Isaac 측에서
per-rotor 힘을 articulation에 가하는 방식은 튜닝 비용만 늘고 train==deploy
이득이 없다. 따라서 단일 rigid body에 thrust+torque를 직접 가하는 캐스케이드
P 컨트롤러를 채택(`Isaac-Quadcopter-Direct-v0` 패턴과 동일 계열).

USD 에셋(Crazyflie 쉘)은 순수 시각/충돌 형상일 뿐 — 질량/관성은 x500 SDF
실측값(2.07kg, diag(0.0217,0.0217,0.040))으로 덮어씀. x500 SDF/URDF의 실제
USD 변환은 **의도적으로 보류** — per-rotor 액추에이션으로 전환할 때만 필요.

## 캐스케이드 구조 (`_run_velocity_controller`, `drone_bombard_env.py`)

```
v_filt (LPF 출력, 20Hz)
  → 속도 P: a_des = kp_vel * (v_filt - v_actual), accel clamp
  → f_des = m*(a_des + g·ẑ); tilt clamp 35°
  → thrust_mag = f_des·thrust_dir (0..max_thrust)
  → 자세 오차: rot_err = axis(body_z, thrust_dir)*angle + yaw_err
  → 자세 P: rate_sp = k_att * rot_err
  → rate P: torque = k_rate * (rate_sp - ω)
  → set_external_force_and_torque(thrust_mag·body_z, torque)
```

## 초기 게인 (미검정)

| 게인 | 값 | 근거 |
|---|---|---|
| kp_vel_xy / kp_vel_z | 1.8 / 4.0 | PX4 `MPC_XY_VEL_P_ACC`/`MPC_Z_VEL_P_ACC` 근사 |
| accel_xy_clamp / accel_z_clamp | 8.0 / 4.0 m/s² | 임의 초기값 |
| k_att_rp / k_att_yaw | 6.5 / 4.0 | PX4 `MC_ROLL_P` 근사 |
| k_rate (roll,pitch,yaw) | 18, 18, 8 | 임의 초기값 |
| tilt_clamp | 35° | 임의 초기값 (Gazebo `limit_inverted_tilt`=60°보다 보수적으로 설정 — 정상 비행 범위) |
| max_thrust | 40.6N | T/W=2.0(무탑재 airframe 고정 스펙) × 2.07kg × 9.81 — §질량/추력 계산 참조 |

## 질량/추력 계산 재확인

- T/W는 **airframe/모터의 고정 스펙**이며 무탑재(2.07kg) 기준으로 정의 —
  탑재(2.17kg=드론+payload) 기준이 아님. 초기 계산에서 T/W×탑재질량으로
  라벨링해 42.6N이 나왔던 것은 **표기 오류**(실측 데이터 아님) — 40.6N으로 정정.
- 탑재 T/W = 40.6/(2.17×9.81) ≈ 1.91 — 여전히 호버링에 충분한 여유.
- 정상 운용 시 실제 필요 추력은 컨트롤러 accel clamp(8/4 m/s²)에 의해 먼저
  제한됨: 최악 케이스 `2.17·√(8² + (9.81+4)²) ≈ 34.8N < 40.6N` — 추력
  상한이 정상 동작 중 발동하지 않음.

## 검정 계획 (미실행 — L4 VM 필요)

**측정 지표 4종** (rise time만으로는 damping 차이를 못 잡음): 10-90% rise
time(±20%), overshoot(±5pp), 2% settling time(±30%), steady-state gain(±5%).

**운용점 매트릭스** (7포인트, `play.py --step-response`):
전진 {1,2,4} m/s · 측방 {1.5,3} m/s · 수직 {±1,±3} m/s · 대각 (2,2,1) m/s.

**참조 트레이스 커버리지 재확인: 0/7.** 기존 `vel_logger.py`는
`/fmu/in/trajectory_setpoint`(커맨드)만 기록하고 응답 채널(`vehicle_local_position`)이
없으며, 정책 주행 중 기록이라 순수 스텝 응답이 아니고, raw CSV도 보존되지
않음(파생 jerk 통계/플롯만 `rl_abtest_*`에 남음). **7포인트 전부 새로 캡처
필요** — `vel_logger_v2.py`(응답 채널 추가) + 스크립트 스텝 시퀀스로 v15
학습 일시정지 중 또는 완료 후 Gazebo 세션 1회(~3-4분 sim, ~15분 wall) 예정.

**미검정 상태로 학습을 시작해야 한다면**: 컨트롤러는 "구조적으로 일치,
**PX4 대비 미검정**(게인은 초기값)"으로 명시하고, v15-vs-Isaac 행동 비교의
명시적 한계로 취급한다. 검정은 그런 비교를 하기 전의 게이팅 항목이 된다.

## 남은 작업

- [ ] `vel_logger_v2.py` 작성 (커맨드+응답 페어 로깅)
- [ ] Gazebo 7-포인트 스텝 시퀀스 캡처 세션
- [ ] `play.py --step-response`로 Isaac 측 동일 매트릭스 캡처
- [ ] 4지표 비교 → 게인 재조정 (rise time ±20% 목표)
