# 현재 구현된 기능 및 데이터 흐름

## 📊 구현 상태 요약

| 패키지 | 노드 | 상태 | 주요 기능 |
|--------|------|------|-----------|
| `vision_detection` | `xmarker_detector` | ✅ **완전 구현** | YOLOv8 기반 X자 표식 탐지 |
| `path_generation` | `path_generation_node` | ✅ **완전 구현** | 픽셀 좌표 기반 yaw/altitude 제어 |
| `path_generation` | `circle_path` | ✅ **완전 구현** | 원형 비행 패턴 |
| `drop_calculator` | `drop_calculator_node` | ✅ **완전 구현** | 탄도학 계산 기반 투하 타이밍 |
| `mechanism_controller` | - | ❌ **미구현** | Payload 투하 메커니즘 제어 |

## 🔄 실제 데이터 흐름

```
[카메라 (Gazebo)]
    ↓
    ├─→ /camera/rgb/image_raw
    ├─→ /camera/depth/image_raw
    ├─→ /camera/rgb/camera_info
    └─→ /fmu/out/vehicle_local_position
    ↓
[vision_detection::xmarker_detector]
    ├─→ /vision/detections (DetectionResult) ──→ [모니터링/분석용]
    ├─→ /vision/annotated_image (Image) ──→ [시각화용]
    └─→ /target/pixel_coords (Point) ──→ ✅ [path_generation]
                                         ✅ [drop_calculator]
    ↓
[path_generation::path_generation_node]
    Subscribe: /target/pixel_coords
    Subscribe: /mavros/local_position/odom
    Publish: /drone/cmd_vel (Twist) ──→ [PX4/드론]
    ↓
[PX4/드론]
    ├─→ /mavros/local_position/odom ──→ [path_generation]
    └─→ /drone/odometry ──→ [drop_calculator]
    ↓
[drop_calculator::drop_calculator_node]
    Subscribe: /target/pixel_coords
    Subscribe: /drone/odometry
    Publish: /payload/drop_cmd (Bool) ──→ ❌ [mechanism_controller 미구현]
    ↓
[mechanism_controller]
    ❌ 미구현 - Gazebo joint 해제 기능 필요
```

## 📦 패키지별 상세 구현 내용

### 1. vision_detection 패키지

**노드:** `xmarker_detector`

#### 구현된 기능:
- ✅ Downward-facing depth 카메라 이미지 수신
- ✅ YOLOv8 모델 기반 X자 표식 탐지 (10Hz)
- ✅ 픽셀 좌표 추출 (bbox_center_x, bbox_center_y)
- ✅ Depth 정보 기반 3D 위치 계산
- ✅ Camera Frame → Body Frame → NED Frame 좌표 변환
- ✅ 탐지 결과 publish (`/vision/detections`)
- ✅ 바운딩 박스가 그려진 이미지 publish (`/vision/annotated_image`)
- ✅ **픽셀 좌표 publish (`/target/pixel_coords`)** ← 최근 추가

#### 입력 토픽:
- `/camera/rgb/image_raw` (sensor_msgs/Image)
- `/camera/depth/image_raw` (sensor_msgs/Image)
- `/camera/rgb/camera_info` (sensor_msgs/CameraInfo)
- `/fmu/out/vehicle_local_position` (px4_msgs/VehicleLocalPosition)

#### 출력 토픽:
- `/vision/detections` (vision_detection/DetectionResult)
- `/vision/annotated_image` (sensor_msgs/Image)
- `/target/pixel_coords` (geometry_msgs/Point) - **u, v 좌표**

---

### 2. path_generation 패키지

#### 노드 1: `path_generation_node`

**구현된 기능:**
- ✅ `/target/pixel_coords` 구독
- ✅ `/mavros/local_position/odom` 구독 (드론 위치/자세)
- ✅ Yaw 각도 제어 (픽셀 좌표 u를 화면 중앙으로 정렬)
- ✅ 고도(Altitude) 제어 (PID 기반)
- ✅ 전진 속도 제어 (정렬 상태에 따라)
- ✅ `/drone/cmd_vel` publish (드론 속도 명령)

**제어 알고리즘:**
```
1. 타겟 픽셀 좌표 (u, v) 수신
2. 화면 중심과의 차이 계산: pixel_error_u = u - center_u
3. Yaw 각속도 계산: angular.z = -kp_yaw * pixel_error_u
4. 고도 제어: linear.z = kp_altitude * (target_altitude - current_altitude)
5. 전진 제어: 
   - 정렬이 잘 되면 (|pixel_error_u| < 20% 화면) → 전진
   - 정렬 중이면 → 천천히 전진
```

**입력 토픽:**
- `/target/pixel_coords` (geometry_msgs/Point)
- `/mavros/local_position/odom` (nav_msgs/Odometry)

**출력 토픽:**
- `/drone/cmd_vel` (geometry_msgs/Twist)

#### 노드 2: `circle_path`

**구현된 기능:**
- ✅ 원형 비행 패턴 생성
- ✅ PX4 Offboard 모드 제어
- ✅ `/fmu/in/offboard_control_mode` publish
- ✅ `/fmu/in/trajectory_setpoint` publish

**입력 토픽:**
- `/fmu/out/vehicle_status` (px4_msgs/VehicleStatus)

**출력 토픽:**
- `/fmu/in/offboard_control_mode` (px4_msgs/OffboardControlMode)
- `/fmu/in/trajectory_setpoint` (px4_msgs/TrajectorySetpoint)
- `/fmu/in/vehicle_command` (px4_msgs/VehicleCommand)

---

### 3. drop_calculator 패키지

**노드:** `drop_calculator_node`

#### 구현된 기능:
- ✅ `/target/pixel_coords` 구독 (타겟 픽셀 좌표)
- ✅ `/drone/odometry` 구독 (드론 위치/속도)
- ✅ 물리 시뮬레이션 기반 낙하 시간 계산
- ✅ 예상 낙하 지점 계산 (속도 × 시간)
- ✅ 타겟과 예상 낙하 지점 간 거리 계산
- ✅ 오차 범위 내일 때 투하 명령 발행
- ✅ `/payload/drop_cmd` publish

**알고리즘:**
```
1. 드론 고도(H), 속도(vx, vy, vz) 수신
2. 낙하 시간 계산: t = (vz + sqrt(vz² + 2*g*H)) / g
3. 예상 낙하 지점 = 현재 위치 + (vx, vy) * (t + mechanism_delay)
4. 타겟 위치와 예상 낙하 지점 간 거리 계산
5. 거리 <= drop_tolerance → drop_cmd = true
```

**입력 토픽:**
- `/target/pixel_coords` (geometry_msgs/Point)
- `/drone/odometry` (nav_msgs/Odometry)

**출력 토픽:**
- `/payload/drop_cmd` (std_msgs/Bool)

**파라미터:**
- `gravity` (기본값: 9.81 m/s²)
- `drop_tolerance` (기본값: 0.2 m)
- `mechanism_delay` (기본값: 0.1 sec)

---

### 4. mechanism_controller 패키지

**상태:** ❌ **미구현**

**필요한 기능:**
- `/payload/drop_cmd` 구독
- Gazebo 시뮬레이션에서 드론과 payload 간 joint 해제
- 실제 투하 메커니즘 제어

---

## ✅ 완전히 작동하는 데이터 흐름

### 흐름 1: Vision Detection → Path Generation

```
[카메라] → [vision_detection] → /target/pixel_coords → [path_generation] → /drone/cmd_vel → [PX4/드론]
```

**상태:** ✅ **완전 작동** (방금 수정 완료)

### 흐름 2: Vision Detection → Drop Calculator

```
[카메라] → [vision_detection] → /target/pixel_coords → [drop_calculator] → /payload/drop_cmd
```

**상태:** ✅ **완전 작동** (drop_calculator가 pixel_coords를 구독 중)

### 흐름 3: PX4 → Drop Calculator

```
[PX4/드론] → /drone/odometry → [drop_calculator] → /payload/drop_cmd
```

**상태:** ✅ **완전 작동**

### 흐름 4: Drop Calculator → Mechanism Controller

```
[drop_calculator] → /payload/drop_cmd → [mechanism_controller]
```

**상태:** ❌ **작동 안 함** (mechanism_controller 미구현)

---

## 🔗 실제 연결 가능한 전체 파이프라인

### 현재 작동 가능한 부분:

```
1. 카메라 이미지 수신 (Gazebo)
   ↓
2. X자 표식 탐지 (vision_detection)
   ↓
3. 픽셀 좌표 추출 및 publish (/target/pixel_coords)
   ↓
4. 드론 경로 제어 (path_generation) ← Yaw/Altitude/전진 제어
   ↓
5. 드론 비행 (PX4)
   ↓
6. 투하 타이밍 계산 (drop_calculator) ← 탄도학 계산
   ↓
7. 투하 명령 발행 (/payload/drop_cmd)
   ↓
8. ❌ 실제 투하 (mechanism_controller) ← 미구현
```

---

## 📝 요약

### ✅ 구현 완료된 기능:
1. **Vision Detection**: X자 표식 탐지 및 픽셀 좌표 publish
2. **Path Generation**: 픽셀 좌표 기반 드론 제어 (yaw, altitude, 전진)
3. **Drop Calculator**: 탄도학 계산 기반 투하 타이밍 결정
4. **원형 비행**: circle_path 노드로 원형 패턴 비행

### ❌ 미구현 기능:
1. **Mechanism Controller**: 실제 payload 투하 메커니즘

### 🔄 데이터 흐름 상태:
- **vision_detection → path_generation**: ✅ 연결됨
- **vision_detection → drop_calculator**: ✅ 연결됨
- **PX4 → drop_calculator**: ✅ 연결됨
- **drop_calculator → mechanism_controller**: ❌ 연결 안 됨 (미구현)

---

## 🚀 다음 단계

1. **mechanism_controller 패키지 구현**
   - `/payload/drop_cmd` 구독
   - Gazebo joint 해제 기능 구현
   - 실제 payload 투하 메커니즘 제어

2. **통합 테스트**
   - 전체 파이프라인 통합 테스트
   - 실제 시뮬레이션 환경에서 테스트
