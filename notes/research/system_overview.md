---
date: 2026-04-14
tags: [research, architecture, ROS2, PX4, gazebo]
status: active
type: research
---

# 시스템 전체 구조 (Gazebo/PX4/ROS2 — `jekyun` 브랜치)

> **참고:** CLAUDE.md에서 이전. 아키텍처 상세 레퍼런스.
>
> ⚠️ **이 문서는 `jekyun` 브랜치(라이브 SAC 학습)의 아키텍처를 설명한다.**
> `feat/isaac-env-migration` 브랜치(Isaac Lab + PPO)의 아키텍처는
> [[research/isaac_lab_architecture]] 참조 — 별도 워크트리, 완전히 다른 스택
> (isaaclab DirectRLEnv, 단일 프로세스, GPU-vectorized, PX4/ROS2/Gazebo 없음).

---

## Mission State Machine (`mission_manager`, 10 Hz)

```
TAKEOFF → CRUISE → TRACKING → DROP
                 ↑         ↓
                 └─ (target lost) ─┘
```

- `TAKEOFF`: ENU (0,0,10)으로 상승
- `CRUISE`: 북동 방향 1 m/s 이동, target 탐지 대기
- `TRACKING`: `rl_navigation`에 속도 제어 위임; target 소실 시 CRUISE 복귀
- `DROP`: `/payload/drop_cmd` 후 호버링
- `STATE_RETURN`: 미구현 placeholder

트리거: `/target/pixel_coords` → CRUISE→TRACKING; `/payload/drop_cmd` → any→DROP

---

## 패키지 구성

| 패키지 | 실행파일 | 루프 속도 | 역할 |
|--------|---------|----------|------|
| `mission_manager` | `mission_manager_node` | 10 Hz | FSM 커맨더; position/velocity cmd 발행; launch 파일 소유 |
| `drone_controller` | `controller` | 20 Hz | PX4 브리지; ENU→NED 변환; 시작 5초 후 arm + OFFBOARD |
| `vision_detection` | `xmarker_detector` | 10 Hz | YOLOv8n 추론; `/target/pixel_coords` 발행 |
| `rl_navigation` | `rl_navigation_node` | 10 Hz | 추적 컨트롤러; 속도 명령 발행; 투하 트리거 |
| `drop_calculator` | `calculator` | 20 Hz | 투하 후 착탄 오차 계산 → `/rl/drop_error` |
| `px4_msgs` | — | — | PX4 v1.15.4 메시지 정의 |

> `drop_calculator/node.py`는 구버전 탄도 예측기 — 진입점 아님. 활성 진입점: `drop_calculator_node.py`

---

## 핵심 토픽

| 토픽 | 타입 | 방향 |
|------|------|------|
| `/target/pixel_coords` | `geometry_msgs/Point` | vision → mission_manager, rl_nav (z=confidence; z=0=미탐지) |
| `/mission/state` | `std_msgs/String` | mission_manager → rl_nav |
| `/drone/cmd/position` | `geometry_msgs/Vector3` | mission_manager → drone_ctrl (ENU) |
| `/drone/cmd/velocity` | `geometry_msgs/Twist` | rl_nav → drone_ctrl (ENU) |
| `/fmu/in/trajectory_setpoint` | `px4_msgs/TrajectorySetpoint` | drone_ctrl → PX4 (NED) |
| `/payload/drop_cmd` | `std_msgs/Empty` | rl_nav → ros_gz_bridge → DetachableJoint |
| `/drone/payload/drop_cmd_raw` | `std_msgs/Bool` | rl_nav → drop_calculator (`False`=drop 이벤트) |
| `/rl/drop_error` | `std_msgs/Float32` | drop_calculator → RL reward |

---

## 좌표계

- **PX4: NED** (North-East-Down), 고도 = −z
- **ROS2: ENU**, 고도 = +z
- 변환 (`drone_controller`): position `(x,y,z)→(x,−y,−z)`, velocity `(vx,vy,vz,yaw)→(vx,−vy,−vz,−yaw)`
- PX4 토픽: **반드시 `BEST_EFFORT` QoS** — `RELIABLE` 사용 시 silent subscription failure

---

## 페이로드 투하 메커니즘

1. `rl_navigation` → `/payload/drop_cmd` (Empty)
2. `ros_gz_bridge` → `/x500_bombard/drop` → `DetachableJoint` 물리 해제
3. `rl_navigation` → `/drone/payload/drop_cmd_raw` (Bool=False) → `drop_calculator` 신호
4. `drop_calculator`: payload z ≤ 0.04 m 도달 시 → `/rl/drop_error` 발행 (단위: m)

> **주의:** Bool semantics 반전 — `False` = drop 이벤트, `True` = 정상

---

## Gazebo Bridge 설정

파일: `mission_manager/config/ros_gz_bridge.yaml`

| gz 토픽 | ROS2 토픽 | 방향 |
|---------|----------|------|
| `/clock` | `/clock` | GZ→ROS |
| `/x500_bombard/down_camera/image_raw` | `/camera/rgb/image_raw` | GZ→ROS |
| `/model/payload_cylinder/odometry` | `/drone/payload/position` | GZ→ROS |
| `/payload/drop_cmd` | `/x500_bombard/drop` | ROS→GZ |
| `/imu/data` | `/imu/data` | GZ→ROS (diagnostics) |

---

## YOLO 모델

- 컨테이너 경로: `/workspace/ros2_ws/yolo_workspace/runs/train/drone_bombard_train2/weights/best.pt`
- 레포 루트 복사본: `drone_bombard_best.pt`
- YOLOv8n, X-marker 학습, mAP@0.5 = 99.5%, 10 Hz 추론

---

## Docker 이미지 구성

빌드: `drone_drop_system/docker/Dockerfile`

- Ubuntu 22.04 + CUDA 12.6.2
- ROS2 Humble (desktop) + Gazebo Harmonic + `ros-humble-ros-gzharmonic`
- PX4 Autopilot v1.15.4 @ `/opt/PX4-Autopilot`
- uXRCE-DDS-Agent @ `/opt/Micro-XRCE-DDS-Agent`
- px4_msgs 사전 빌드 @ `/root/ros2_ws` (볼륨 마운트와 별개)
- Python 의존성: `drone_drop_system/docker/requirements.txt`

---

## 관련 링크

- [[research/architecture]] — Method A 1-World-4-Payload 멀티 인스턴스 설계
- [[research/reward_design]] — `_compute_reward()` 4-layer 보상 함수
- [[research/rl_rules]] — 학습 실행 전 체크리스트 & Known Failure Modes
- [[Environment/README]] — VM 인프라 복구 가이드 (Docker, VNC, Guacamole)
