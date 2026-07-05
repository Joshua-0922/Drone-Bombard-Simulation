# 02. 드론 제어 — PX4 control hierarchy & 좌표계

> 카테고리 2. 우리가 `/drone/cmd/position` 혹은 `/drone/cmd/velocity` 로 명령을 던지면 그게 모터까지 어떻게 풀리는가.

---

## Session 2.1 — 좌표계 3개

드론 시뮬에서 좌표계 혼동이 모든 버그의 출발점. 우리 코드에 3 개가 동시에 등장.

### ENU (ROS2 표준)
```
+X = East
+Y = North
+Z = Up
```
ROS2 의 `geometry_msgs/Vector3`, `Twist`, RViz 시각화, 우리 `drop_calculator` 의 `x_marker_x/y` 등이 다 ENU.

### NED (PX4 표준 — 항공/해양)
```
+X = North
+Y = East
+Z = Down  ← 핵심: Z 가 아래 양수
```
`/fmu/out/vehicle_local_position`, `/fmu/in/trajectory_setpoint` 의 position/velocity, 모든 PX4 내부 EKF 가 NED.

→ **고도 10 m 는 NED 로 z=-10**. 우리 코드 56줄: `target_pos = [0.0, 0.0, -10.0]` 가 정확히 이 의미.

### FRD (body frame — 드론 기체축)
```
+X = Forward (드론 머리 방향)
+Y = Right
+Z = Down
```
드론의 자세 (roll/pitch/yaw) 와 angular velocity 가 FRD. world frame 과 attitude quaternion 으로 연결.

### 우리 변환 코드 (drone_controller_node.py:72)

```python
# ROS(ENU) -> PX4(NED): x->N, y->-E, z->-D
self.target_pos = [msg.x, -msg.y, -msg.z]
```

| ENU 축 | NED 축 | 부호 |
|---|---|---|
| x (East) | y (East) | + |
| y (North) | x (North) | + |
| z (Up) | z (Down) | − |

…잠깐, 코드는 `[msg.x, -msg.y, -msg.z]` 인데 ENU→NED 의 정석은 `[msg.y, msg.x, -msg.z]` 가 맞음.

→ 이건 **우리 컨벤션이 ENU 가 아니라 "X 가 North 인 ENU 변종"** 임을 의미. drop_calculator 가 `x_marker_x=11.0` 을 보낼 때 그 의미가 "North 11 m" 라면 이 변환이 맞고, "East 11 m" 라면 버그. → 이건 카테고리 3 (MDP/env 코드) 에서 검증.

**기억할 것**: 좌표계 변환은 우리 코드에 묵시적 가정이 박혀 있고, 한 번 잘못되면 silent drift 형태로 나타남. 14 m offset 의 한 후보였음.

---

## Session 2.2 — PX4 5단 cascade controller

### 큰 그림

```
       사용자 명령 (TrajectorySetpoint)
                  │
                  ▼
   [1] Position Controller (~50 Hz)
       입력: 목표 위치 (NED)
       출력: 목표 속도 (NED)
                  │  pos error × Kp_pos
                  ▼
   [2] Velocity Controller (~50 Hz)
       입력: 목표 속도, 현재 속도
       출력: 목표 가속도 (NED)
                  │  vel error × Kp_vel + integrator
                  ▼
   [3] Attitude Controller (~250 Hz)
       입력: 목표 가속도 → roll/pitch/thrust 분해
       출력: 목표 자세 (quaternion) + thrust
                  │  Hibernate: 가속도 벡터의 수평성분 = tilt, 수직성분 = thrust
                  ▼
   [4] Rate Controller (~1 kHz, SITL 에선 sim Hz 상한)
       입력: 목표 각속도 (자세 오차에서 계산)
       출력: 목표 토크 (roll/pitch/yaw)
                  │  PID with feed-forward
                  ▼
   [5] Mixer / Control Allocation
       입력: 토크 (3축) + thrust (1축) = 4 자유도
       출력: 모터 4 개 PWM (0~1 정규화)
                  │  CA_ROTOR* 좌표로 4×4 행렬 곱
                  ▼
       Gazebo motor model → 추력 / 토크 → 강체 dynamics
```

### 각 단의 의미 직관

- **Position → Velocity**: "여기로 가고 싶다" → "이 정도 속도로 움직여라". 위치 오차가 클수록 빠르게.
- **Velocity → Acceleration**: "이 속도로 가고 싶다" → "이 정도 힘 주면 그 속도가 된다". 마찰/중력 보정 포함.
- **Acceleration → Attitude+Thrust**: 쿼드콥터는 직접 옆으로 못 가고 **기울여서** 옆으로 감. 수평 가속도 = sin(tilt) × thrust 의 수평 성분. **이게 multicopter 제어의 핵심.**
- **Attitude → Rate**: "이 각도가 되고 싶다" → "이 각속도로 회전하라". P 제어 (오차 × gain).
- **Rate → Motor**: 각속도 오차 → PID → "각 모터에 이만큼 PWM". mixer 가 4 자유도 (roll/pitch/yaw 토크 + thrust) 를 4 모터 PWM 으로 분배.

### 왜 cascade 인가?

- **bandwidth 분리**: 안쪽은 빠르게 (자세 안정성), 바깥은 느리게 (위치 트래킹). 안쪽이 충분히 빨라야 바깥이 작동.
- **plug-in 식 명령**: 사용자가 어느 레벨에서든 진입 가능. 위치 setpoint = top 진입, 자세 setpoint = 중간 진입, **모터 직접 = 사실상 우회**.
- **failsafe**: 한 레벨이 안 풀리면 그 위 레벨이 자동 보정.

### 우리가 진입하는 레벨

| 우리 명령 | OffboardControlMode 플래그 | 진입 단 |
|---|---|---|
| `cmd_position` (Vector3) | position=True, velocity=False | **[1] Position** |
| `cmd_velocity` (Twist) | position=False, velocity=True | **[2] Velocity** |

코드 `drone_controller_node.py:122-124` 를 보면 우리는 **항상 position=True, velocity=True 둘 다 켜고** TrajectorySetpoint 의 NaN 으로 무시 처리. PX4 에선 이 패턴이 표준 — 어느 필드가 유효한가는 setpoint 값으로 판단.

→ **우리는 [1] 또는 [2] 에 진입한다.** [3]~[5] 는 PX4 가 알아서 함. attitude/rate 단에 직접 진입하려면 `OffboardControlMode.attitude=True` + `VehicleAttitudeSetpoint` 필요 — 우리 코드엔 없음.

---

## Session 2.3 — Offboard mode 와 arming 시퀀스

### Offboard 가 뭔가

PX4 의 nav_state 중 하나. "외부 컴퓨터 (companion / GCS) 가 직접 setpoint 를 던지는 모드". 우리 ROS2 가 정확히 이 역할.

조건:
- setpoint 가 일정 주기 이상으로 들어와야 함 (안 들어오면 failsafe)
- 우리 세팅 `COM_OF_LOSS_T=10.0` → 10 s 안 오면 failsafe (loiter 또는 land)

### Arming 이 뭔가

모터가 회전할 수 있는 상태로 PX4 를 락 해제. arming_state == 2 (ARMED).
- arm 전엔 어떤 setpoint 도 무시 — 모터 회전 안 함.
- arm 은 한 번 사용자 명령 (VEHICLE_CMD_COMPONENT_ARM_DISARM param1=1.0) 으로 수행.

### 우리 시퀀스 (drone_controller_node.py:85-117)

```
t=0  : Node 시작, OffboardControlMode + setpoint publishing 즉시 시작 (20 Hz)
       └─ PX4 가 setpoint stream 을 "본" 이후에야 OFFBOARD 진입 가능
t=?  : /fmu/out/vehicle_status 첫 수신 → px4_connected=True
       └─ "PX4 가 살아 있고 DDS 가 통한다" 확인
t=+5s: warmup 끝 (100 ticks @ 20Hz)
       └─ EKF2 가 GPS/IMU 로 자기 위치 수렴
       └─ 이 전에 arm 하면 NaN velocity → blind land → motor NaN → ODE crash (!)
t=+5s~: 매 2 s 마다 arm 재시도, 매 0.5 s 마다 offboard 진입 재시도
       └─ 한 번에 안 되면 (예: setpoint stream 끊김) 재시도
```

### 핵심 트랩

코드 주석 `drone_controller_node.py:107-110`:
> Arming before EKF converges causes NaN velocity → 'Failsafe: blind land' → NaN motor commands → Gazebo ODE AABB integer overflow → Gazebo crash.

→ **arm 타이밍이 sim crash 의 직접 원인이 될 수 있음.** 5 s warmup 은 안전 마진. 실 hardware 에선 더 짧지만 SITL EKF 가 GPS 시뮬에서 noise 큰 게 변수.

### Offboard 진입 명령

```python
VEHICLE_CMD_DO_SET_MODE
  param1 = 1.0   # 모드 사용
  param2 = 6.0   # 6 = OFFBOARD
```

PX4 가 받아서 nav_state 를 14 (OFFBOARD) 로 전환. **이후 우리 setpoint 가 제어 cascade 의 top 으로 들어감.**

---

## Session 2.4 — Mixer / Control Allocation

### 4 자유도 → 4 모터

쿼드콥터는 4 자유도 입력 (roll torque, pitch torque, yaw torque, total thrust) 을 4 모터 PWM 으로 분배.

```
[ω₀]     [ +Px₀  +Py₀  +Km₀  +Kt ] [ τ_roll  ]
[ω₁]  =  [ +Px₁  +Py₁  +Km₁  +Kt ] [ τ_pitch ]
[ω₂]     [ +Px₂  +Py₂  +Km₂  +Kt ] [ τ_yaw   ]
[ω₃]     [ +Px₃  +Py₃  +Km₃  +Kt ] [ thrust  ]
```
(개념도 — 실제는 부호와 정규화가 복잡)

### 우리 airframe 의 CA_ROTOR (4016_gz_x500_bombard_r0:23-37)

| 로터 | PX (m) | PY (m) | KM (yaw moment) | 위치 직관 |
|---|---|---|---|---|
| 0 | +0.13 | +0.22 | +0.05 (CCW) | 전방 우측 |
| 1 | −0.13 | −0.20 | +0.05 (CCW) | 후방 좌측 |
| 2 | +0.13 | −0.22 | −0.05 (CW)  | 전방 좌측 |
| 3 | −0.13 | +0.20 | −0.05 (CW)  | 후방 우측 |

**X-config 쿼드 표준**:
- 대각선 페어 (0+1) 와 (2+3) 가 같은 방향으로 돔 → yaw torque cancel
- 한 페어 빠르게 = yaw 회전
- 앞 두 개 (0,2) 빠르게 = pitch down (앞으로 기울임)
- 우측 두 개 (0,3) 빠르게 = roll left

### Hover throttle (MPC_THR_HOVER=0.60)

- 정의: hover 시 (가속도 0) 평균 모터 throttle. 0.60 = max thrust 의 60%.
- payload **달린** 상태 기준 — 우리 세팅이 그러함.
- payload 분리 후엔 같은 throttle 이 과추력 → 위로 튐 → controller 가 적응할 시간 필요 → **drop 직후 자세 진동** 의 한 원인.

### Gazebo motor model 과의 연결

```
SIM_GZ_EC_FUNC1..4 = 101..104   ← Servo function ID
SIM_GZ_EC_MIN1..4 = 150          ← idle PWM (μs)
SIM_GZ_EC_MAX1..4 = 1000         ← max PWM (μs)
```

PX4 mixer 출력 (0~1 정규화 throttle) → SIM_GZ_EC_MIN ~ MAX 사이로 매핑 → Gazebo 모터 플러그인 → 각속도 → 추력 (∝ ω²) → 강체 force/torque.

---

## Session 2.5 — 우리 컨트롤 흐름 종합

### 우리 RL agent 가 명령을 던지면?

```
[RL agent (Python)]
  action = [vx, vy, vz, yaw_rate]    ← network 출력
       │
       ▼
[DroneDropEnv (Gymnasium)]
  ENU 변환 → cmd_velocity publish
       │ ROS2 topic /drone/cmd/velocity (Twist)
       ▼
[drone_controller_node (20 Hz)]
  ENU → NED 변환 (msg.linear.x, -msg.linear.y, -msg.linear.z, -msg.angular.z)
  OffboardControlMode publish
  TrajectorySetpoint (velocity field 만 채움) publish
       │ /fmu/in/trajectory_setpoint (uXRCE-DDS)
       ▼
[PX4 SITL]
  [2] Velocity controller 진입
       → [3] Attitude → [4] Rate → [5] Mixer
       → 4 모터 PWM
       │ gz transport
       ▼
[Gazebo]
  모터 회전 → 추력 → 드론 이동
       │ EKF feedback
       ▼
[PX4 → ROS2 → env]
  /fmu/out/vehicle_local_position → 새 obs
```

→ **한 step 은 위 사이클이 여러 번 (>1 회) 회전**. 우리 env step 은 보통 100 ms (10 Hz) 이고 controller 는 50 ms (20 Hz), attitude/rate 는 더 빠름.

### 그래서 우리가 학습하는 건 뭔가?

**[2] velocity controller 의 입력 (vx, vy, vz, yaw_rate) 을 시간에 따라 어떻게 줄지** — 이게 우리 RL policy 가 배우는 것.

낮은 레벨 ([3]~[5]) 은 PX4 가 알아서 함. 우리는 **항공 역학을 직접 학습하지 않음**. 우리가 학습하는 건 "어떤 속도로 어디로 가서 언제 drop 할까" 의 **trajectory + timing** 정책.

### Drop action 의 특별함

drop 은 **연속 action 의 일부가 아님**. 별도 discrete signal — `/x500_0/drop` 토픽에 한 번 publish. 한 번 분리되면 끝. → MDP 관점에서 drop 은 **terminal-like action** (이후 trajectory 가 의미가 줄어듦).

---

## 카테고리 2 정리 — 3 줄

1. **5단 cascade**: position → velocity → **attitude → rate → mixer**. 우리는 [1] 또는 [2] 진입.
2. **좌표계 3종 (ENU/NED/FRD)** 변환이 silent bug 의 단골. 코드 줄별로 부호 확인 필수.
3. 우리 RL policy 가 학습하는 것 = **velocity setpoint trajectory + drop timing**. 항공 역학은 PX4 가 처리.

→ 다음: **카테고리 3 — MDP & DroneDropEnv 해부** (state machine, observation, action space, reward, termination 1줄씩).
