---
date: 2026-03-20
updated: 2026-04-14
tags: [research, architecture, method-A, gazebo, PX4]
status: implemented
type: research
---

# Method A — 1-World-4-Payload 아키텍처

> **현재 채택 방식.** 단일 Gazebo 월드에 4개 페이로드 사전 배치, PX4는 동적 스폰.

---

## 아키텍처 다이어그램

```
Gazebo Harmonic (단일 프로세스)
├── x_marker_world.sdf
│   ├── x_marker_0  @ (11, 10, 0)
│   ├── x_marker_1  @ (11, 160, 0)
│   ├── x_marker_2  @ (11, 310, 0)
│   └── x_marker_3  @ (11, 460, 0)
│   ├── payload_0   @ (0, 0, 0.14)
│   ├── payload_1   @ (0, 150, 0.14)
│   ├── payload_2   @ (0, 300, 0.14)
│   └── payload_3   @ (0, 450, 0.14)
│
PX4 SITL × 4 (동적 스폰, PX4_SIM_MODEL=gz_x500_bombard_rN)
│   └── Gazebo entity: x500_bombard_rN_N
│
DroneDropEnv × N (SubprocVecEnv)
    ├── instance_id=0 → ROS_DOMAIN_ID=0, UXRCE port 8888
    ├── instance_id=1 → ROS_DOMAIN_ID=1, UXRCE port 8889
    └── ...
```

---

## 멀티-인스턴스 테스트 결과 (2026-03-20)

| 설정 | 결과 |
|------|------|
| `num_envs=1` | ✅ 31–33 fps, 안정, ODE 크래시 없음 |
| `num_envs=2` | ❌ 총 10 fps (6× 오버헤드); Gazebo 컴퓨팅 병목으로 CRUISE 타임아웃 |
| `num_envs=4` | ❌ PX4 2개 크래시 (lockstep 타임아웃) |

**결론:** Gazebo 단일 프로세스가 PX4 lockstep을 직렬화 → `num_envs=1` 고정.

---

## 핵심 설정

| 파라미터 | 값 |
|---------|-----|
| `PX4_SIM_MODEL` | `gz_x500_bombard_rN` |
| `PX4_GZ_MODEL_POSE` | `0, {N*150}, 0.5, 0, 0, 0` |
| `PX4_SIM_SPEED_FACTOR` | 1 |
| `real_time_factor` | 1.0 (RTF=1) |
| `COM_OF_LOSS_T` | 10.0 s |
| `obs_wait_timeout` | 0.02 s (20 ms) |

---

## 에피소드 리셋 흐름

```
reset()
  └─ _kill_episode()      # mission_manager + drone_controller + drop_calculator 종료
  └─ gz service set_pose  # 드론 + 페이로드 텔레포트
  └─ _start_episode()     # 에피소드 노드 재시작
  └─ _wait_for_cruise()   # CRUISE 상태 대기 (타임아웃 시 1회 재시도)
  
  ※ Gazebo + PX4 SITL은 에피소드 간 유지 (재시작 없음)
```

---

## 관련 링크

- [[research/reward_design]] — 보상 함수 상세
- [[research/system_overview]] — 패키지·토픽·좌표계 전체 구조
- [[errors/err_20260319_ode_aabb_crash]] — 스폰 고도 크래시
- [[errors/err_20260320_physics_explosion]] — ODE 물리 폭발 방어
