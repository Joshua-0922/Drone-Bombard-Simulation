# 01. 환경 — Sim Stack

> 카테고리 1. Gazebo Harmonic + PX4 SITL + micro-XRCE-DDS + ROS2 가 어떻게 한 덩어리로 도는지.

---

## Session 1.1 — 4 레이어 큰 그림

### 학습 목표
- 시뮬레이션이 "한 프로세스" 가 아닌 **4 개의 독립 프로세스가 협력** 하는 구조라는 걸 이해.
- 각 레이어의 역할을 1 줄로 말할 수 있게 됨.
- "내 ROS2 코드가 publish 한 setpoint 가 드론을 움직이기까지" 의 데이터 흐름을 따라갈 수 있게 됨.

---

### 4 레이어 다이어그램

```
┌─────────────────────────────────────────────────────────────┐
│  Layer 4: ROS2 (사용자 코드 공간)                            │
│  ┌──────────────────────────────────────────────────┐       │
│  │ DroneDropEnv / train_sac / dgui                  │       │
│  │ 토픽: /fmu/in/*, /fmu/out/*                       │       │
│  └──────────────────────────────────────────────────┘       │
└────────────────────────────┬────────────────────────────────┘
                             │  DDS (RTPS over UDP, port 8888)
┌────────────────────────────▼────────────────────────────────┐
│  Layer 3: micro-XRCE-DDS Agent (브리지)                      │
│  · uORB <-> DDS 양방향 번역                                   │
│  · "PX4 의 내부 IPC 메시지" ↔ "ROS2 의 외부 IPC 메시지"        │
└────────────────────────────┬────────────────────────────────┘
                             │  micro-XRCE-DDS (UDP)
┌────────────────────────────▼────────────────────────────────┐
│  Layer 2: PX4 SITL (오토파일럿)                              │
│  · 위치 → 속도 → 자세 → 각속도 → 모터 PWM (제어 계층)         │
│  · EKF, 안전 로직, 모드 관리 (Manual/Offboard/...)            │
│  · ROMFS 안에 airframe 정의 (gz_x500_bombard_r0 등)          │
└────────────────────────────┬────────────────────────────────┘
                             │  Gazebo-PX4 plugin (TCP 4560)
┌────────────────────────────▼────────────────────────────────┐
│  Layer 1: Gazebo Harmonic (물리 엔진)                        │
│  · 강체 dynamics, 충돌, 공기역학                              │
│  · 센서 시뮬 (IMU, GPS, 카메라, lidar)                        │
│  · world (.sdf), model (.sdf), plugin (DetachableJoint 등)   │
└─────────────────────────────────────────────────────────────┘
```

핵심: **위로 갈수록 추상적, 아래로 갈수록 물리적.** 4 개가 모두 살아있어야 한 에피소드가 돈다.

---

### 각 레이어의 역할 — 1 줄

| 레이어 | 한 줄 요약 | 죽으면 무슨 일이? |
|---|---|---|
| **Gazebo Harmonic** | 시간을 진행시키고 물리 법칙을 푼다 (없으면 드론이 존재하지 않음) | 시뮬 정지, PX4 가 "센서 안 옴" 으로 EKF 발산 |
| **PX4 SITL** | "목표 위치" 같은 추상 명령을 모터 회전 속도로 풀어 낸다 | 모터가 안 돔, 드론 떨어짐 |
| **micro-XRCE-DDS Agent** | PX4 의 uORB 메시지를 ROS2 가 알아들을 수 있는 DDS 메시지로 번역 | ROS2 가 드론 상태 못 봄, 우리 코드 silent crash |
| **ROS2 (우리 코드)** | 정책 평가 / 학습 / 명령 송신 | RL 학습 불가, PX4 만 있으면 드론은 hover 만 함 |

---

### 한 명령의 일생 — Offboard setpoint 전송 추적

예: 우리가 `(x=2, y=0, z=-1.5)` 로 가라고 하면?

```
[ROS2] DroneDropEnv 가 TrajectorySetpoint 메시지 publish
       토픽: /fmu/in/trajectory_setpoint
        │
        ▼
[DDS]  RTPS 패킷 직렬화 → UDP 8888 로 전송
        │
        ▼
[Agent] micro-XRCE-DDS Agent 가 패킷 수신
        → uORB message: trajectory_setpoint
        │  (DDS 의 외부 표현을 PX4 내부 표현으로 변환)
        ▼
[PX4]  Commander 가 OFFBOARD mode 인지 확인
       → mc_pos_control 모듈이 setpoint 받음
       → position controller: 목표 위치 - 현재 위치 = 속도 명령
       → velocity controller: 속도 오차 = 가속도 명령
       → attitude controller: 가속도 = 자세 명령 (roll/pitch/thrust)
       → rate controller: 자세 오차 = 각속도 명령
       → mixer: 각속도/추력 → 4 개 모터 PWM
        │
        ▼
[Gazebo-PX4 plugin] PWM 값을 Gazebo 모터 모델에 전달
        │
        ▼
[Gazebo] 모터 회전 → 추력 / 토크 → 강체 dynamics 풀이
        → 새 위치/속도/자세
        → IMU/GPS sensor plugin 이 노이즈 섞어서 출력
        │
        ▼ (역방향)
[PX4]  센서 값 받아 EKF 로 자기 상태 추정
       → vehicle_local_position uORB 메시지 발행
        │
        ▼
[Agent] uORB → DDS 번역
        │
        ▼
[ROS2] 우리 env 가 /fmu/out/vehicle_local_position 구독
       → obs 의 일부로 사용
```

이 한 사이클이 PX4 기준 250 Hz, Gazebo 기준 250 Hz (real-time factor ≈ 1) 로 돈다.

---

### 우리 컨테이너에서의 매핑

| 레이어 | 실제 프로세스 (컨테이너 `drone-bombard-harmonic` 안) |
|---|---|
| Gazebo Harmonic | `gz sim` (gz-sim8) |
| PX4 SITL | `px4` 바이너리 (multi-instance: `px4 -i 0 ~ -i 3` for 4 drones) |
| Agent | `MicroXRCEAgent udp4 -p 8888` |
| ROS2 | `ros2 launch` 로 띄운 우리 노드들 |

→ Session 1.2 에서 이걸 `docker exec` 로 실제 보면서 확인 예정.

---

### Self-check (다음 세션 가기 전 답할 수 있어야 함)

1. **Agent 가 죽으면** ROS2 에서 어떤 토픽이 사라지나? PX4 는 어떤 상태인가?
2. PX4 의 **"position controller"** 와 **"rate controller"** 중 어느 쪽이 더 빠르게(높은 Hz) 돌까? 왜?
3. 우리가 `_kill_infra` 로 매 drop 마다 죽이는 건 위 4 레이어 중 **어느 것**? 왜 그게 필요했나?
4. multi-instance (`r0~r3`) 일 때 위 그림에서 **레이어 몇 개가 4 벌**이 되나?

---

---

### Session 1.1 Self-check 답안

**Q1. Agent 가 죽으면?**
- ROS2 쪽: `/fmu/in/*`, `/fmu/out/*` 토픽이 **publisher/subscriber 0 명** 으로 바뀜 (토픽 자체는 DDS 가 캐싱하므로 잠시 살아 있어 보임).
- PX4 쪽: uXRCE-DDS client 가 timeout 되며 "Session disconnected" 로그. 단 PX4 는 죽지 않음 — 자체 OFFBOARD timeout (`COM_OF_LOSS_T`, 우리 세팅 10 s) 후 failsafe 모드로 빠짐.
- **사용자 코드 영향**: env.step() 이 publish 해도 PX4 가 못 받음 → 드론이 명령 무시 → silent failure. 우리가 겪었던 "14 m offset" 의 근본 원인 중 하나.

**Q2. position vs rate controller — 어느 쪽이 빠른가?**
- **rate controller 가 압도적으로 빠름**. PX4 의 표준 cascade 는 position ≈ 50 Hz / velocity ≈ 50 Hz / attitude ≈ 250 Hz / **rate ≈ 1 kHz** / mixer = rate Hz.
- 이유: **outer loop (느림) → inner loop (빠름)** 가 cascade 제어의 기본 원리. 안쪽일수록 자세 안정성을 책임지므로 latency budget 이 짧음. 바깥은 reference 만 갱신.
- 우리 세팅: Gazebo 가 `real_time_update_rate=250` (4 ms step) 이므로 PX4 attitude/rate 도 sim time 기준 250 Hz 가 상한. 실제 1 kHz rate 는 hardware 에서만 의미.

**Q3. `_kill_infra` 가 죽이는 건?**
- **Layer 4 (ROS2 mission nodes) 만 죽임** — `mission_manager` / `drone_controller` / `drop_calculator` (그리고 `rl_navigation` 이 활성이면 함께).
- Layer 1~3 (Gazebo / PX4 / Agent) 은 `infra.launch.py` 로 한 번 띄워 **전 학습 세션 동안 살려 둠**.
- 왜 필요했나: episode 간 mission node 의 내부 상태 (queue, target, state machine) 가 누적되어 **silent drift** 일으킴. PX4 의 EKF / Gazebo 의 world 는 reset service 로 처리 가능하지만 ROS2 노드는 새로 띄우는 게 가장 깔끔.
- → 우리 v2/v3 시절 "14 m offset" 의 원인은 **mission node 의 stale state 가 spin thread 와 결합** 한 것. jekyun_v2 의 4 patch 가 이걸 끊었음.

**Q4. multi-instance (r0~r3) 일 때 몇 벌이 4 벌?**
- **Layer 2 (PX4 SITL) 만 4 벌** (`px4 -i 0`, `-i 1`, `-i 2`, `-i 3`).
- Layer 1 (Gazebo) 는 **1 벌** — 한 world 안에 4 드론 모델을 공존시킴.
- Layer 3 (Agent) 도 **1 벌** — UDP 8888 단일 포트로 다수 PX4 instance 의 client_id 별로 라우팅.
- Layer 4 (ROS2) 는 **1 벌의 코드, 4 namespace** — `/px4_0/fmu/*`, `/px4_1/...` 로 분리.
- 핵심: "한 world 안 다수 드론" 패턴. payload 도 4 개 (`payload_0~3`), DetachableJoint 도 4 개, drop topic 도 `/x500_0/drop` ~ `/x500_3/drop`.

---

## Session 1.2 — 컨테이너 내부 관찰

### 컨테이너 1줄

```
NAME: drone-bombard-harmonic       IMAGE: drone-bombard:local
```

이 한 컨테이너 안에 4 레이어 전부 들어 있음. host VM 은 git/문서/모니터링만 함.

### 정상 학습 시 프로세스 트리 (개념)

```
docker exec drone-bombard-harmonic ps -ef

PID 1   tini / bash                     ← 컨테이너 entrypoint
 ├─ ros2 launch ... infra.launch.py     ← Layer 4 의 launch 매니저
 │   ├─ MicroXRCEAgent udp4 -p 8888     ← Layer 3
 │   ├─ gz sim -r -s x_marker_world.sdf ← Layer 1 (headless)
 │   ├─ ros_gz_bridge parameter_bridge  ← gz topic ↔ ROS2 topic (camera 등)
 │   └─ px4 (PX4_GZ_STANDALONE=1)       ← Layer 2 (persistent)
 │
 ├─ ros2 launch ... episode.launch.py   ← Episode-level (매 reset 마다 재시작)
 │   ├─ mission_manager_node
 │   ├─ drone_controller (controller)
 │   └─ drop_calculator (calculator)    ← x_marker_x=11.0, y=10.0
 │
 └─ python train_sac.py                 ← 학습 메인 프로세스
     └─ DroneDropEnv (Gymnasium)         ← env.step/reset/close
```

### infra.launch.py 의 타임라인 (실제 코드)

| t (s) | 띄우는 것 | 명령 |
|---|---|---|
| 0 | `MicroXRCEAgent` | `MicroXRCEAgent udp4 -p 8888` |
| 0 | Gazebo | `gz sim -r [-s] x_marker_world.sdf` |
| 10 | ros_gz_bridge | `parameter_bridge --config ros_gz_bridge.yaml` |
| 20 | PX4 SITL | `px4` with `PX4_GZ_STANDALONE=1 PX4_SIM_MODEL=gz_x500_bombard PX4_GZ_MODEL_POSE='0,0,5,0,0,0'` |
| 22 | YOLO detector | `xmarker_detector` (옵션, vision 모드만) |

→ **20 초 stagger** 의 이유: Gazebo world 완전 로드 (~10 s) → bridge 가 토픽 인식 (~10 s) → PX4 가 gz transport 통해 모델/world 연결.

### episode.launch.py 의 핵심

```
Node: mission_manager     (mission_manager_node)
Node: drone_controller    (controller)
Node: drop_calculator     (calculator)
     parameters: x_marker_x=11.0, x_marker_y=10.0
```

→ 이 3 개만 `_kill_infra` 가 죽이고 재기동. PX4/Gazebo/Agent 는 살려 둠. Q3 답안 그대로.

### 우리가 자주 보는 토픽 (4 layer 통신 흔적)

| 토픽 | 방향 | 의미 |
|---|---|---|
| `/fmu/in/trajectory_setpoint` | ROS2 → PX4 | 위치/속도 명령 (offboard) |
| `/fmu/in/offboard_control_mode` | ROS2 → PX4 | "지금 position/velocity 모드야" 선언 |
| `/fmu/in/vehicle_command` | ROS2 → PX4 | arm/disarm, mode 전환 |
| `/fmu/out/vehicle_local_position` | PX4 → ROS2 | EKF 가 추정한 현재 위치/속도 (NED) |
| `/fmu/out/vehicle_attitude` | PX4 → ROS2 | 자세 quaternion |
| `/fmu/out/vehicle_status` | PX4 → ROS2 | arming state, nav state, failsafe |
| `/x500_0/drop` | ROS2 → Gazebo | DetachableJoint 분리 신호 (uXRCE 안 거침, gz transport 직통) |

핵심: **PX4 와의 통신은 모두 `/fmu/*` 네임스페이스 (uXRCE-DDS 경유)**. Gazebo 와의 직통 통신은 `gz topic` (ros_gz_bridge 또는 gz-transport 라이브러리) — drop signal 이 대표적.

### Session 1.2 Self-check

1. infra 와 episode 를 **굳이 두 launch 파일로 분리한 이유** 는? (지금 답할 수 있어야 함 — Q3 와 직결)
2. `PX4_GZ_STANDALONE=1` 이 의미하는 건? (힌트: 기본값에서 PX4 는 Gazebo 를 자기가 띄움)
3. `ros_gz_bridge` 가 없으면 어떤 흐름이 끊기나? (힌트: `/fmu/*` 는 영향 없음)

---

## Session 1.3 — SDF 해부 (Gazebo 모델 정의)

### 우리 world: `x_marker_world.sdf`

```xml
<world name="x_marker_world">
  <physics type="ode">
    <max_step_size>0.004</max_step_size>      <!-- 4 ms = 250 Hz sim step -->
    <real_time_factor>1</real_time_factor>     <!-- wall-clock 과 일치 -->
    <real_time_update_rate>250</real_time_update_rate>
  </physics>

  <plugin filename="gz-sim-physics-system" .../>
  <plugin filename="gz-sim-scene-broadcaster-system" .../>
  <plugin filename="gz-sim-contact-system" .../>
  <plugin filename="gz-sim-imu-system" .../>
  <plugin filename="gz-sim-air-pressure-system" .../>
  <plugin filename="gz-sim-navsat-system" .../>
  <plugin filename="gz-sim-magnetometer-system" .../>
  <!-- 의도적으로 빠진 것: gz-sim-sensors-system (카메라 ogre2 렌더링) -->
</world>
```

핵심 포인트:
- **RTF=1 이 절대 조건**. RTF=2 시도했더니 uXRCE-DDS time sync drift → EKF velocity NaN → "Failsafe: blind land" → ODE AABB overflow crash → Gazebo 사망. 주석에 그대로 적혀 있음.
- **camera sensors-system 의도적 제거**. RL training 에서 vision 안 쓰니까. 켜 두면 GPU rendering overhead → physics timestep 흔들림 → motor aliasing → ODE crash. → vision 변형은 별도 world 또는 별도 환경 변수로 활성화.
- **물리는 ODE**. dartsim 도 옵션이었으나 우리 케이스에서 AABB overflow 발생 이력 있어 ODE 고정.

### 우리 드론: `gz_x500_bombard_r0/model.sdf`

핵심 구조:

```xml
<model name="gz_x500_bombard_r0">
  <include merge="true"><uri>model://x500</uri></include>   <!-- PX4 표준 x500 베이스 -->

  <!-- 1. magnetometer 센서 추가 -->
  <link name="magnetometer_link">
    <sensor name="magnetometer_sensor" type="magnetometer">
      <update_rate>100</update_rate>
      <magnetometer>
        <x><noise stddev="0.0004"/></x>
        ...
      </magnetometer>
    </sensor>
  </link>

  <!-- 2. payload 부착점 (base_link 아래 10 cm 지점) -->
  <link name="payload_mount">
    <pose relative_to="base_link">0 0 -0.10 0 0 0</pose>
    <visual><geometry><cylinder r=0.015 l=0.01/></geometry></visual>
  </link>

  <!-- 3. DetachableJoint plugin — 핵심 -->
  <plugin filename="gz-sim-detachable-joint-system">
    <parent_link>payload_mount</parent_link>
    <child_model>payload_0</child_model>
    <child_link>payload_link</child_link>
    <detach_topic>/x500_0/drop</detach_topic>
  </plugin>
</model>
```

**DetachableJoint 가 이 프로젝트의 심장.**
- 부착: world 시작 시 `payload_mount` ↔ `payload_0/payload_link` 가 **fixed joint** 로 묶임.
- 분리: `/x500_0/drop` 토픽에 메시지 한 번 publish → joint 가 **영구 분리** (재부착 불가, world reset 만이 복구).
- **velocity preservation**: 분리 순간 payload 는 부착 시점의 드론 속도를 그대로 물려받음. 이게 우리 toss 전략의 물리적 근거 — 드론이 빠르게 움직이며 drop 하면 payload 는 그 momentum 으로 날아감.

### r0 / r1 / r2 / r3 가 따로 있는 이유

| 변수 | r0 | r1 | r2 | r3 |
|---|---|---|---|---|
| `child_model` | payload_0 | payload_1 | payload_2 | payload_3 |
| `detach_topic` | /x500_0/drop | /x500_1/drop | /x500_2/drop | /x500_3/drop |

→ DetachableJoint plugin 은 **모델 SDF 안에 child 이름이 하드코딩** 되어야 함 (런타임 치환 불가). 그래서 4 instance 학습용으로 model 자체를 4 벌 복제. 본 학습은 우선 1 drone 만 쓰지만 multi-instance 학습 옵션은 살려 둠.

### Session 1.3 Self-check

1. `<include merge="true">` 가 의미하는 건? merge 안 하면 어떻게 되나?
2. magnetometer 가 우리 모델 SDF 와 world SDF **양쪽** 에 등장하는데 둘의 역할 차이는?
3. world 의 `max_step_size=0.004` 와 `real_time_update_rate=250` 이 동시에 250 이 아닐 수도 있나? (sim time 과 wall time)

---

## Session 1.4 — ROMFS 와 PX4 airframe

### airframe 파일이란?

PX4 의 "이 모델이 무엇이며 어떤 파라미터를 기본값으로 가져야 하는가" 정의. shell script 형식. PX4 부팅 시 `param set-default` 들이 실행되어 EKF/제어 파라미터를 모델에 맞게 세팅.

### 우리 r0 airframe: `4016_gz_x500_bombard_r0`

```sh
#!/bin/sh
# @name Gazebo x500 Bombard r0
# @type Quadrotor

. ${R}etc/init.d/rc.mc_defaults                    # 표준 멀티콥터 기본값 로드

PX4_SIMULATOR=gz
PX4_SIM_MODEL=x500_bombard_r0                       # ← Gazebo 모델 이름과 매칭

param set-default SIM_GZ_EN 1                       # Gazebo simulation 활성
param set-default SENS_EN_GPSSIM 1                  # GPS 시뮬 ON
param set-default SENS_EN_BAROSIM 0                 # Baro OFF (우리 안 씀)
param set-default SENS_EN_MAGSIM 1                  # Mag ON

# 제어 할당 (Control Allocation)
param set-default CA_AIRFRAME 0                     # multirotor
param set-default CA_ROTOR_COUNT 4                  # 쿼드
param set-default CA_ROTOR0_PX 0.13                 # 로터 0 위치 (X)
param set-default CA_ROTOR0_PY 0.22                 # 로터 0 위치 (Y)
param set-default CA_ROTOR0_KM 0.05                 # 회전 방향 (양수=CCW)
... (CA_ROTOR1..3)

# Gazebo motor 출력 매핑
param set-default SIM_GZ_EC_FUNC1..4 = 101..104     # Servo function 101~104
param set-default SIM_GZ_EC_MIN1..4 = 150           # min PWM
param set-default SIM_GZ_EC_MAX1..4 = 1000          # max PWM

param set-default MPC_THR_HOVER 0.60                # hover 시 throttle 60%

# Failsafe / GPS / Mag 검증 완화 (SITL용)
param set COM_ARM_WO_GPS 1                          # GPS 없어도 arm 허용
param set FD_FAIL_R 0                               # roll failsafe 끔
param set SIM_BAT_DRAIN 0                           # 배터리 소모 없음
param set EKF2_MAG_CHECK 0                          # mag 검증 끔
param set COM_OF_LOSS_T 10.0                        # Offboard 끊김 허용 시간 10 s
param set UXRCE_DDS_SYNCT 0                         # uXRCE time sync 끔 (drift 방지)
```

핵심 포인트:
- **`UXRCE_DDS_SYNCT 0`**: time sync drift 가 EKF 발산 → ODE crash 의 원인이라 끔. world SDF 의 RTF=1 주석과 같은 issue.
- **`COM_OF_LOSS_T 10.0`**: Offboard 명령이 10 s 안 오면 PX4 가 failsafe (loiter/land). 우리는 step 마다 명령 보내니 정상 동작에선 안 걸리지만, ROS2 node crash 시 10 s 후 자동 안전 동작.
- **`MPC_THR_HOVER 0.60`**: 드론 + payload 매단 상태에서의 hover throttle. payload 떨어뜨리면 갑자기 가벼워져 위로 튐 — 이것이 우리 drop 직후 자세 흔들림의 한 원인.
- **CA_ROTOR* 좌표**: X-config 쿼드의 표준. rotor 0/2 우측, 1/3 좌측. KM 부호로 회전 방향.

### ROMFS / rootfs 양쪽이 필요한 이유 (메모리에 기록된 issue)

| 경로 | 역할 | 사용 시점 |
|---|---|---|
| `/opt/PX4-Autopilot/ROMFS/px4fmu_common/init.d-posix/airframes/4016_*` | **소스 트리** 의 airframe 정의 | `make px4_sitl_default` 빌드 시 rootfs 에 복사됨 |
| `/opt/PX4-Autopilot/build/px4_sitl_default/etc/init.d-posix/airframes/4016_*` | **빌드 결과 rootfs**. 런타임에 실제 읽는 곳 | `px4` 바이너리 실행 시 |

문제: airframe 파일을 수정/추가해도 **빌드를 다시 안 하면 rootfs 에 반영 안 됨.** 우리 `4017_r1` ~ `4019_r3` 가 multi-instance 학습에 필요했는데 default build 뒤에 rootfs 에 없어서 인식 안 되는 issue 가 있었음.

해결책 (메모리 `project_px4_airframe_sync.md`):
1. **`make px4_sitl_default`** — 정공법. 전체 rebuild.
2. **`cp`** — quick fix. 소스의 airframe 파일을 빌드 rootfs 로 직접 복사.

→ 어떤 방법을 쓰든 **소스 + rootfs 양쪽 동기화 필수**.

### 4015 vs 4016/4017/4018/4019

```
4015_gz_x500_bombard      ← 단일 인스턴스용 (PX4_SIM_MODEL=x500_bombard, payload 이름 동적)
4016_gz_x500_bombard_r0   ← 멀티 인스턴스 rank 0
4017_gz_x500_bombard_r1   ← rank 1
4018_gz_x500_bombard_r2   ← rank 2
4019_gz_x500_bombard_r3   ← rank 3
```

각 airframe 의 `PX4_SIM_MODEL` 이 다르고, 그에 대응하는 Gazebo 모델 SDF (`x500_bombard`, `x500_bombard_r0~r3` 또는 `gz_x500_bombard_r0~r3`) 가 있어야 PX4 가 Gazebo 와 연결됨.

### Session 1.4 Self-check

1. airframe 파일에서 `param set-default` 와 `param set` 의 차이는?
2. `MPC_THR_HOVER` 가 0.60 인 게 payload 매단 상태 기준이라면, payload 분리 후 같은 throttle 을 유지하면 어떻게 되나? (drop 직후 거동 예측)
3. ROMFS 의 4016 airframe 을 수정했는데 PX4 가 변경을 못 알아본다면 어디부터 의심할까?

---

## 카테고리 1 정리 — 다음 카테고리 가기 전 핵심 4 줄

1. 시뮬레이션은 **4 프로세스 (Gazebo / PX4 / Agent / ROS2) 의 협력**. 한 군데 죽으면 나머지가 silent failure.
2. 우리 launch 는 **infra (persistent) + episode (reset 마다 재기동)** 로 분리. RL stability 의 핵심 패턴.
3. **DetachableJoint plugin (gz-sim) + payload velocity preservation** 가 toss 전략의 물리적 기반.
4. **RTF=1, UXRCE_DDS_SYNCT=0, ODE physics, camera off (RL 시)** — 우리가 발견한 "안정 동작 4 조건" 이 world/airframe 에 박혀 있음.

→ 다음: **카테고리 2 — 드론 제어** (PX4 cascade 5 단, 좌표계, mixer, control mode). 우리가 publish 하는 TrajectorySetpoint / OffboardControlMode 가 어떻게 모터까지 풀리는지 깊이.
