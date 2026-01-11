# X-Marker Vision Detection System

드론 시뮬레이션을 위한 YOLOv8 기반 X자 표식 탐지 시스템입니다.

## 시스템 구성

### 패키지
- **vision_detection**: YOLOv8 기반 X-marker 탐지 노드
- **path_generation**: 드론 경로 제어 및 launch 파일

### 주요 기능
- Downward-facing depth 카메라에서 이미지 수신
- YOLOv8로 X자 표식 실시간 탐지 (10Hz)
- 픽셀 좌표 + NED 월드 좌표 동시 출력
- Depth 정보를 활용한 3D 위치 계산

## 설치 및 빌드

### 1. Python 의존성 설치
```bash
# NumPy 다운그레이드 (cv_bridge 호환성)
pip3 install 'numpy<2.0' --user --force-reinstall

# YOLO 및 OpenCV
pip3 install ultralytics opencv-python --user
```

### 2. 패키지 빌드
```bash
cd /workspace/ros2_ws
source setup_simulation.bash
colcon build --packages-select vision_detection path_generation
source install/setup.bash
```

## 사용 방법

### 1. 전체 시뮬레이션 실행 (카메라 + Vision)

```bash
# Headless 모드 (GUI 없음, 추천)
ros2 launch path_generation px4_gazebo_sitl.launch.py \
    headless:=true \
    enable_vision:=true

# GUI 모드
ros2 launch path_generation px4_gazebo_sitl.launch.py \
    enable_vision:=true

# Vision + 원형 경로 비행
ros2 launch path_generation px4_gazebo_sitl.launch.py \
    headless:=true \
    enable_vision:=true \
    enable_path_generation:=true
```

### 2. Vision 노드만 단독 실행

```bash
ros2 run vision_detection xmarker_detector
```

### 3. 탐지 결과 확인

```bash
# 탐지 토픽 모니터링
ros2 topic echo /vision/detections

# 주파수 확인
ros2 topic hz /vision/detections

# 이미지 시각화 (rqt 필요)
ros2 run rqt_image_view rqt_image_view /vision/annotated_image
```

## ROS 2 토픽

### 입력 (Subscribe)
| 토픽 | 타입 | 설명 |
|------|------|------|
| `/camera/rgb/image_raw` | sensor_msgs/Image | RGB 카메라 이미지 |
| `/camera/depth/image_raw` | sensor_msgs/Image | Depth 맵 |
| `/camera/rgb/camera_info` | sensor_msgs/CameraInfo | 카메라 intrinsics |
| `/fmu/out/vehicle_local_position` | px4_msgs/VehicleLocalPosition | 드론 NED 위치 |

### 출력 (Publish)
| 토픽 | 타입 | 설명 |
|------|------|------|
| `/vision/detections` | vision_detection/DetectionResult | 탐지 결과 |
| `/vision/annotated_image` | sensor_msgs/Image | 바운딩 박스가 그려진 이미지 |

## DetectionResult 메시지 구조

```msg
std_msgs/Header header

# 탐지 상태
bool detected                 # X-marker 탐지 여부
float32 confidence           # 탐지 신뢰도 [0-1]

# 픽셀 좌표 (바운딩 박스)
float32 bbox_center_x        # 중심 X (픽셀)
float32 bbox_center_y        # 중심 Y (픽셀)
float32 bbox_width           # 폭 (픽셀)
float32 bbox_height          # 높이 (픽셀)

# 카메라 프레임 좌표 (미터)
geometry_msgs/Point camera_coords

# NED 월드 좌표 (미터)
geometry_msgs/Point ned_coords
bool ned_valid               # NED 좌표 유효성
float32 depth                # 깊이 (미터)
```

## 좌표 변환

### 1. Pixel → Camera Frame
```
X_cam = (u - cx) * depth / fx
Y_cam = (v - cy) * depth / fy
Z_cam = depth
```

### 2. Camera Frame → Body Frame
카메라 마운트: `[0.108, 0, -0.01]`, pitch -90°
```
X_body = -Z_cam + 0.108
Y_body = Y_cam
Z_body = X_cam - 0.01
```

### 3. Body Frame → NED Frame
```
X_ned = vehicle_x + (X_body * cos(yaw) - Y_body * sin(yaw))
Y_ned = vehicle_y + (X_body * sin(yaw) + Y_body * cos(yaw))
Z_ned = vehicle_z + Z_body
```

## 파라미터

Vision detection 노드의 파라미터:

| 파라미터 | 기본값 | 설명 |
|----------|--------|------|
| `model_path` | `/workspace/ros2_ws/yolo_workspace/runs/train/drone_bombard_train2/weights/best.pt` | YOLO 모델 경로 |
| `inference_rate` | 10.0 | 추론 주파수 (Hz) |

## 문제 해결

### NumPy 버전 에러
```bash
pip3 install 'numpy<2.0' --user --force-reinstall
```

### 카메라 토픽이 없음
드론 모델이 `iris_downward_depth_camera`인지 확인:
```bash
ros2 param get /px4_sitl PX4_SIM_MODEL
```

### YOLO 모델 로드 실패
모델 파일 경로 확인:
```bash
ls -lh /workspace/ros2_ws/yolo_workspace/runs/train/drone_bombard_train2/weights/best.pt
```

### Gazebo가 멈춤
Headless 모드 사용 (`headless:=true`)

## 드론 모델 정보

**iris_downward_depth_camera**:
- 카메라 위치: `[0.108, 0, -0.01]` (base_link 기준)
- 카메라 방향: Pitch -90° (아래 향함)
- 이미지 크기: 848x480
- FOV: 86° (horizontal)
- Depth 범위: 0.001m ~ 65.535m
- 주파수: 10Hz

## 다음 단계

1. **제어 통합**: 탐지된 좌표를 사용하여 드론이 마커 위로 이동
2. **자동 착륙**: 마커 중심에 정밀 착륙
3. **다중 마커**: 여러 마커 동시 탐지
4. **위치 필터링**: Kalman 필터로 스무딩

## 참고

- YOLO 모델: `/workspace/ros2_ws/yolo_workspace/`
- YOLO 데이터셋: "Drone-Bombard-Simulation-1" (2,220 이미지)
- 학습 클래스: `x_marker` (1개 클래스)
- 모델 타입: YOLOv8n (nano)

---

## X-Marker 시뮬레이션 환경

### X-Marker 모델
시뮬레이션에 추가된 빨간색 X-marker:
- **위치**: `/opt/PX4-Autopilot/Tools/simulation/gazebo-classic/sitl_gazebo-classic/models/x_marker/`
- **크기**: 1m x 1m  
- **색상**: 빨간색 X + 검은색 테두리 (흰색 배경)
- **타입**: Static model (고정)

### 테스트 World
`x_marker_test.world` - X-marker가 배치된 테스트 환경:
- X-marker 3개: `(0,0)`, `(5,5)`, `(-5,5)`
- Ground plane + Sun 포함
- 경로: `/opt/PX4-Autopilot/Tools/simulation/gazebo-classic/sitl_gazebo-classic/worlds/x_marker_test.world`

### 간편 테스트 스크립트
`/workspace/ros2_ws/test_vision_detection.sh` - 전체 시스템 자동 실행:
```bash
cd /workspace/ros2_ws
./test_vision_detection.sh
```

**스크립트 기능**:
1. ✅ MicroXRCE-DDS Agent 시작
2. ✅ Gazebo + X-marker world 로드
3. ✅ PX4 SITL 시작
4. ✅ 드론 모델 스폰 (고도 5m)
5. ✅ Vision detection 노드 실행
6. ✅ 자동 상태 확인 (카메라/탐지 토픽)

**로그 파일**:
- `/tmp/agent.log` - MicroXRCE-DDS Agent
- `/tmp/gazebo.log` - Gazebo server
- `/tmp/px4.log` - PX4 SITL
- `/tmp/vision.log` - Vision detection node

