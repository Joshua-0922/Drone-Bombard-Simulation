# 시스템 아키텍처 (System Architecture)

Drone Bombard System의 전체 시스템 구조와 패키지 간 통신 방식을 설명하는 문서입니다.

## 📋 목차

1. [시스템 개요](#시스템-개요)
2. [패키지 구조](#패키지-구조)
3. [데이터 흐름](#데이터-흐름)
4. [ROS2 토픽 구조](#ros2-토픽-구조)
5. [알고리즘 개요](#알고리즘-개요)

## 🎯 시스템 개요

Drone Bombard System은 카메라 기반 지상 타겟 인식 후 투하 지점을 계산하여 시뮬레이션 환경에서 투하 임무를 수행하는 자율비행 시스템입니다.

**주요 기능:**
- 카메라 이미지에서 X자 표식(X-marker) 실시간 탐지
- 탐지된 X자와 드론의 위치를 정렬(align)
- 물리 시뮬레이션 기반 투하 타이밍 계산
- 드론 payload 자동 투하

## 📦 패키지 구조

시스템은 4개의 주요 ROS2 패키지로 구성됩니다:

### 1. vision_detection

**역할:** 카메라 이미지에서 X자 표식 탐지

**주요 기능:**
- Downward-facing depth 카메라에서 이미지 수신
- YOLOv8 기반 X자 표식 실시간 탐지 (10Hz)
- 픽셀 좌표 및 NED 월드 좌표 변환
- Depth 정보를 활용한 3D 위치 계산

**입력 (Subscribe):**
- `/camera/rgb/image_raw` (sensor_msgs/Image) - RGB 카메라 이미지
- `/camera/depth/image_raw` (sensor_msgs/Image) - Depth 맵
- `/camera/rgb/camera_info` (sensor_msgs/CameraInfo) - 카메라 intrinsics
- `/fmu/out/vehicle_local_position` (px4_msgs/VehicleLocalPosition) - 드론 NED 위치

**출력 (Publish):**
- `/vision/detections` (vision_detection/DetectionResult) - 탐지 결과 (픽셀 좌표 + NED 좌표 포함)
- `/vision/annotated_image` (sensor_msgs/Image) - 바운딩 박스가 그려진 이미지

**참고:** 
- 픽셀 좌표는 `/vision/detections` 메시지의 `bbox_center_x`, `bbox_center_y` 필드에 포함됩니다.
- 별도의 `/target/pixel_coords` 토픽은 현재 구현되지 않았습니다.

### 2. path_generation

**역할:** X자 표식과 align되게 드론의 경로 생성

**주요 기능:**
- X자 표식의 픽셀 좌표를 받아 드론 경로 생성
- 드론의 yaw 각도를 조절하여 X자와 정렬
- PX4 Offboard 모드를 통한 드론 제어

**입력 (Subscribe):**
- `/target/pixel_coords` (geometry_msgs/Point) - 타겟 픽셀 좌표 (현재 테스트용)
- `/mavros/local_position/odom` (nav_msgs/Odometry) - 드론 위치 및 속도 정보

**출력 (Publish):**
- `/drone/cmd_vel` (geometry_msgs/Twist) - 드론 속도 명령

**노드 구성:**
- `path_generation_node`: 메인 경로 생성 노드 (`/drone/cmd_vel` 사용)
- `circle_path`: 원형 비행 노드 (`/fmu/in/offboard_control_mode`, `/fmu/in/trajectory_setpoint` 사용)

**알고리즘:**
- 드론의 yaw 방향 각도를 조절
- 카메라의 u 좌표(중심선)와 드론의 중심이 align되도록 회전
- 특정 정해진 각속도로 회전 제어
- (향후 계획) Bezier curve 등을 활용한 매끄러운 경로 생성

### 3. drop_calculator

**역할:** 드론의 payload를 투하할지 말지 결정

**주요 기능:**
- 드론의 현재 위치와 속도 정보 수신
- 물리 시뮬레이션 기반 낙하 시간 계산
- X자 표식과의 거리를 기반으로 투하 타이밍 결정

**입력 (Subscribe):**
- `/drone/odometry` (nav_msgs/Odometry) - 드론의 위치 및 속도 정보
- `/target/pixel_coords` (geometry_msgs/Point) - 타겟 픽셀 좌표

**출력 (Publish):**
- `/payload/drop_cmd` (std_msgs/Bool) - 투하 명령 (true: 투하, false: 유지)

**알고리즘:**
- 드론의 현재 고도, 속도 정보를 바탕으로 낙하 시간 계산
- 투하 지점에서의 예상 낙하 지점 계산
- X자 표식 위치와 예상 낙하 지점 간의 거리 계산
- 거리가 일정 오차 범위(error rate) 이하일 때 투하 명령 발행
- **주의사항:** align이 되기 전에 투하 신호를 줘야 할 경우를 고려한 추가 알고리즘 보완 필요

**물리 모델:**
- 중력 가속도: 9.81 m/s²
- 자유 낙하 운동 방정식 사용
- 항력은 무시 (초기 단계)

### 4. mechanism_controller

**역할:** 드론이 payload를 직접 투하하는 노드

**주요 기능:**
- drop_calculator에서 투하 명령 수신
- 드론 밑에 있는 payload와의 joint 해제
- 실제 투하 메커니즘 제어

**입력 (Subscribe):**
- `/payload/drop_cmd` (std_msgs/Bool) - drop_calculator에서 발행

**출력 (Action):**
- 드론과 payload 간의 joint 해제 (Gazebo 시뮬레이션)

**구현 상태:**
- (현재 개발 예정)

## 🔄 데이터 흐름

```
[카메라]
    ↓
[vision_detection] ──/vision/detections──> [다른 노드/모니터링]
    │
    └── (픽셀 좌표는 DetectionResult 내부에 포함)
                                                    │
[테스트/외부] ──/target/pixel_coords──> [path_generation]
                                                    │
                                                    ↓
                                            [/drone/cmd_vel]
                                                    │
                                                    ↓
                                            [PX4 / 드론]
                                                    │
                                                    ↓
                                            [/drone/odometry]
                                                    │
                                                    ↓
                                        [drop_calculator]
                                                    │
                                                    ↓
                                            [/payload/drop_cmd]
                                                    │
                                                    ↓
                                        [mechanism_controller]
                                                    │
                                                    ↓
                                            [Payload 투하]
```

### 상세 데이터 흐름

1. **카메라 → vision_detection**
   - 카메라 이미지 수신
   - YOLOv8 모델로 X자 표식 탐지
   - 픽셀 좌표 및 3D 좌표 변환

2. **vision_detection → path_generation** (향후 구현)
   - 현재는 `/vision/detections` 메시지에 픽셀 좌표가 포함됨
   - 향후 `/target/pixel_coords` 토픽으로 직접 전달 예정
   - path_generation이 yaw 각도 계산 및 제어

3. **path_generation → PX4/드론**
   - 드론 제어 명령 전달
   - 드론 위치/속도 정보 피드백

4. **PX4/드론 → drop_calculator**
   - 드론의 현재 위치 및 속도 정보 전달

5. **drop_calculator → mechanism_controller**
   - 투하 타이밍이 되면 투하 명령 전달

6. **mechanism_controller → Gazebo**
   - Joint 해제를 통한 payload 투하

## 📡 ROS2 토픽 구조

### 주요 토픽 요약

| 토픽명 | 타입 | Publisher | Subscriber | 설명 |
|--------|------|-----------|------------|------|
| `/camera/rgb/image_raw` | sensor_msgs/Image | 카메라 | vision_detection | RGB 카메라 이미지 |
| `/camera/depth/image_raw` | sensor_msgs/Image | 카메라 | vision_detection | Depth 맵 |
| `/target/pixel_coords` | geometry_msgs/Point | (테스트/외부) | path_generation, drop_calculator | 타겟 픽셀 좌표 (현재 테스트용) |
| `/vision/detections` | vision_detection/DetectionResult | vision_detection | - | 탐지 결과 (픽셀 좌표 + NED 좌표 포함) |
| `/vision/annotated_image` | sensor_msgs/Image | vision_detection | - | 바운딩 박스가 그려진 이미지 |
| `/mavros/local_position/odom` | nav_msgs/Odometry | PX4/MAVROS | path_generation | 드론 위치 정보 (path_generation용) |
| `/drone/odometry` | nav_msgs/Odometry | PX4 | drop_calculator | 드론 위치/속도 정보 (drop_calculator용) |
| `/drone/cmd_vel` | geometry_msgs/Twist | path_generation | PX4 | 드론 속도 명령 |
| `/payload/drop_cmd` | std_msgs/Bool | drop_calculator | mechanism_controller | 투하 명령 |

### 토픽 세부사항

#### `/vision/detections` (vision_detection/DetectionResult)
- `detected` (bool): 탐지 여부
- `confidence` (float32): 탐지 신뢰도
- `bbox_center_x`, `bbox_center_y`: 바운딩 박스 중심
- `ned_coords` (geometry_msgs/Point): NED 월드 좌표
- `depth` (float32): 깊이 정보

#### `/target/pixel_coords` (geometry_msgs/Point) - 현재 테스트용
```python
x = u  # 픽셀 좌표 u
y = v  # 픽셀 좌표 v
z = 0.0  # 사용 안 함
```
- 현재는 `system_tester.py`에서 발행하는 테스트 토픽
- 향후 `vision_detection`에서 직접 발행하도록 구현 예정

#### `/payload/drop_cmd` (std_msgs/Bool)
- `true`: 투하 실행
- `false`: 투하 안 함

## 🧮 알고리즘 개요

### 1. X자 표식 탐지 (vision_detection)

**YOLOv8 기반 객체 탐지:**
- 입력: 카메라 이미지 (848x480, 10Hz)
- 출력: 바운딩 박스 + 신뢰도
- 모델: YOLOv8n (nano), 학습된 모델 파일 사용

**좌표 변환:**
1. Pixel → Camera Frame
2. Camera Frame → Body Frame
3. Body Frame → NED Frame

### 2. 경로 생성 및 정렬 (path_generation)

**Yaw 각도 조절 알고리즘:**
- 카메라 이미지의 중심선(u 좌표)을 기준으로
- X자 표식이 중심선에 오도록 드론의 yaw 각도 조절
- PID 제어 또는 각속도 기반 제어 사용

**향후 개선:**
- Bezier curve를 활용한 매끄러운 경로 생성
- 특정 지점을 지나가는 경로 생성

### 3. 투하 타이밍 계산 (drop_calculator)

**낙하 시간 계산:**
```
H = 현재 고도
Vz = 수직 속도 (아래 방향)
g = 중력 가속도 (9.81 m/s²)

낙하 시간: t = (Vz + sqrt(Vz² + 2*g*H)) / g
```

**투하 지점 계산:**
```
Vx, Vy = 수평 속도
t_total = mechanism_delay + t_impact
예상 낙하 지점 = 현재 위치 + (Vx, Vy) * t_total
```

**투하 결정:**
```
distance = ||예상 낙하 지점 - X자 표식 위치||
if distance <= drop_tolerance:
    drop = True
else:
    drop = False
```

## 🚧 향후 개발 사항

1. **mechanism_controller 패키지 구현**
   - Joint 해제 메커니즘 구현
   - Gazebo 연동

2. **path_generation 알고리즘 개선**
   - Bezier curve 경로 생성
   - 더 정밀한 정렬 알고리즘

3. **drop_calculator 알고리즘 보완**
   - align이 되기 전 투하 케이스 처리
   - 항력 고려 (고급 물리 모델)

4. **통합 테스트**
   - 전체 시스템 통합
   - 시나리오 테스트

## 📚 관련 문서

- [VISION_DETECTION_README.md](./ros2_ws/VISION_DETECTION_README.md) - vision_detection 패키지 상세 가이드
- [MODEL_INFO.md](./MODEL_INFO.md) - YOLO 모델 정보
- [README.md](./README.md) - 개발 환경 가이드
