# Vision-Based Drone System

## 1. Project Overview
본 프로젝트는 사전에 적의 정확한 좌표를 알지 못하는 상황에서, 드론이 지정된 경로를 **순항(Cruise)**하던 중 목표물(X Marker)을 **식별(Detection)**하면 즉시 경로를 변경하여 **정밀 추적(Terminal Guidance)** 및 **공중 투하(Air Drop)**를 수행하는 자율 비행 시스템입니다.

1.  **Phase 1: 순항 (Cruise Mode)**
    * 드론은 10m 고도를 유지하며 북동 방향으로 1 m/s 속도로 비행합니다.
    * GPS 기반 위치 제어로 웨이포인트를 점진적으로 증가시켜 비행합니다.
2.  **Phase 2: 요격 전환 (Intercept Transition)**
    * 하방 카메라가 목표물('X' 표식)을 감지하는 즉시 순항 모드를 중단합니다.
    * 제어권을 Mission Manager에서 추적 알고리즘(RL Navigation)으로 넘깁니다.
3.  **Phase 3: 정밀 유도 (Terminal Guidance)**
    * GPS 좌표가 아닌 카메라 영상 내 타겟 위치(Pixel Error)를 기반으로 비행합니다.
    * 드론의 속도 벡터를 타겟 방향으로 지속적으로 수정하며 접근합니다.
4.  **Phase 4: 동적 투하 (Dynamic Drop)**
    * 타겟 상공 도달 및 투하 조건 만족 시 페이로드를 투하합니다.

## 2. Operating Principles
Warning: GCP VM은 사용자 계정별 홈 디렉토리가 분리되어 있다.

모든 팀 공용 작업은 다음 경로에서 수행한다.
/opt/drone-bombard

개인 홈 디렉토리(/home/username)는 사용하지 않는다.

### 1. Repository에는 "소스 코드만" 관리한다

* ROS2 패키지는 반드시 ros2_ws/src/<package_name>에 생성
* build/, install/, log/ 디렉토리는 Git 관리 대상이 아님

### 2. 개발은 항상 Docker 컨테이너 내부에서 수행한다
* VM에는 Docker만 설치
* ROS2, Python 의존성, 빌드 도구는 dockerfile 수정으로 변경

### 3. ros2_ws는 Host ↔ Container 볼륨으로 공유한다
* 컨테이너 삭제 후에도 코드 유지
* 동일 워크스페이스를 여러 컨테이너에서 재사용 가능

### 4. Docker 이미지 빌드는 GitHub Actions가 담당한다
* 로컬에서 docker build 금지
* main 또는 feature/migration-harmonic 브랜치 push → 자동 빌드 → Artifact Registry 저장

### 5. ROS2 패키지는 컨테이너 내부에서 생성한다.
* 패키지 생성 위치
```
cd /workspace/ros2_ws/src
ros2 pkg create <package_name> --build-type ament_python
```

* 빌드
```
cd /workspace/ros2_ws
colcon build
source install/setup.bash
```

*  생성 결과는 VM의 /opt/drone-bombard/ros2_ws/src 에 자동 반영됨.
* 이후 VM에서 github repository로 push하기.

### 6. 모든 package 개발은 branch로 나누어서 개발하기. 이후 합치면 된다.
* 개발 종류마다 나누어서 branch 작성
* 이름은 feature/migration-harmonic 등으로, 만들기 전 회의 통해 결정

## 3. 기술 스택
OS : Ubuntu 22.04 LTS
ROS2 : Humble
Simulation : Gazebo Harmonic
ML Framework : Pytorch
GPU : NVIDIA L4 & CUDA 12.6.2
CI/CD : Github Actions & Google Cloud Platform

## 4. 개발 환경 흐름
```
GitHub Repository
        ↓ (git pull)
GCP VM (Host)
        ↓ (-v volume mount)
Docker Container
        ↓ (ros2 build / 개발)
ros2_ws/src/*
        ↓ (git add / commit / push)
GitHub Repository
```

## 5. Repository
```
.
├── LICENSE
├── README.md                  # 개발 환경 가이드
├── CLAUDE.md                  # Claude Code 가이드 (시스템 아키텍처 상세)
├── drone_drop_system/         # Docker 빌드 관련
│   └── docker/                # Dockerfile, entrypoint, requirements
├── gazebo_models/             # Gazebo Harmonic 시뮬레이션 모델
│   ├── x500_bombard/          # 드론 모델 (페이로드 탑재)
│   ├── payload_cylinder/      # 투하 페이로드 모델
│   ├── x_marker/              # X자 표식 모델
│   └── worlds/                # 시뮬레이션 월드 (x_marker_world.sdf)
├── ros2_ws/                   # ROS2 전용 워크스페이스 (Git으로 관리)
│   ├── src/                   # ROS2 패키지 소스
│   │   ├── mission_manager/   # FSM 커맨더 + 통합 launch 파일
│   │   ├── drone_controller/  # PX4 브릿지 (ENU→NED 변환)
│   │   ├── vision_detection/  # YOLOv8 X마커 탐지 패키지
│   │   ├── rl_navigation/     # 추적 컨트롤러 + SAC RL 학습
│   │   ├── drop_calculator/   # 투하 후 착탄 오차 계산
│   │   └── px4_msgs/          # PX4 메시지 타입
│   ├── yolo_workspace/        # YOLO 학습 관련
│   │   ├── datasets/          # 학습 데이터셋
│   │   ├── runs/              # 학습 결과
│   │   └── scripts/           # 학습 스크립트
│   └── system_tester.py       # 시뮬레이션 테스트 스크립트
└── drone_bombard_best.pt      # 학습된 YOLO 모델 (최적 가중치)
```

디렉토리 역할 요약
* `ros2_ws/`
  * ROS2 Humble 기준 워크스페이스
  * `src/` 아래에만 ROS2 패키지 생성
  * `build/`, `install/`, `log/`는 로컬/컨테이너 빌드 산출물 (Git 관리 제외)

* `gazebo_models/`
  * Gazebo Harmonic 시뮬레이션용 3D 모델 및 월드 파일

* `yolo_workspace/`
  * YOLO 모델 학습 및 평가 관련 파일

**시스템 아키텍처 및 기능에 대한 자세한 내용은 [CLAUDE.md](./CLAUDE.md)를 참고하세요.**

## 6.  Repository / VM / Container 역할
| 구분 | 역할 |
| :-: | :-: |
| Github Repository | 코드 관리 |
| Github Actions | Docker 이미지 자동 빌드 |
| Artifact Registry | 이미지 저장 |
| VM | 실행 환경, 볼륨 유지 |
| Docker Container | ROS2 개발 및 실행 |


## 7. VM에서 Docker 실행 가이드
### 7.1. 최초 컨테이너 생성
```bash
# 1. (필수) 호스트에서 화면 권한 허용 (실행 전 1회)
xhost +local:docker

# 2. 최신 이미지 pull
docker pull us-central1-docker.pkg.dev/charming-league-481306-d8/drone-bombard/drone-bombard:latest

# 3. 컨테이너 실행
docker run -itd \
  --gpus all \
  --net=host \
  --privileged \
  --ipc=host \
  --name drone-bombard-harmonic \
  --env="DISPLAY=$DISPLAY" \
  --env="QT_X11_NO_MITSHM=1" \
  --env="NVIDIA_DRIVER_CAPABILITIES=all" \
  --volume="/tmp/.X11-unix:/tmp/.X11-unix:rw" \
  -v /opt/drone-bombard/Drone-Bombard-Simulation/ros2_ws:/workspace/ros2_ws \
  -v /opt/drone-bombard/Drone-Bombard-Simulation/gazebo_models:/workspace/gazebo_models \
  -v ~/.cache:/root/.cache \
  us-central1-docker.pkg.dev/charming-league-481306-d8/drone-bombard/drone-bombard:latest \
  /bin/bash
```
* 컨테이너 이름: `drone-bombard-harmonic` (팀 공용 단일 컨테이너)
* `ros2_ws`, `gazebo_models` 모두 VM과 컨테이너 공유 볼륨

### 7.2 기존 컨테이너 재접속
```bash
xhost +local:docker
docker start -ai drone-bombard-harmonic
```

### 7.3 기존 컨테이너 삭제
```bash
docker stop drone-bombard-harmonic
docker rm drone-bombard-harmonic
```

## 8. Github repository와 VM
### 8.1 Github Repository로 코드 반영(VM기준)
```bash
cd /opt/drone-bombard/Drone-Bombard-Simulation
git status
git add ros2_ws
git commit -m "Add ROS2 package"
git pull --rebase
git push
```
### 8.2 Github Action에서 완성된 Image를 pull해서 VM에서 실행하는 방법
1. Docker Image pull하기
```bash
cd /opt/drone-bombard/Drone-Bombard-Simulation
git pull --rebase
gcloud auth configure-docker us-central1-docker.pkg.dev  # 최초 한번만
docker pull us-central1-docker.pkg.dev/charming-league-481306-d8/drone-bombard/drone-bombard:latest
```

2. 기존 컨테이너 삭제 후 재생성
```bash
docker stop drone-bombard-harmonic
docker rm drone-bombard-harmonic
# 이후 7.1 컨테이너 생성 명령 실행
```

## 9. 팀원 VM 접근 가이드
### 9.1 GCP 권한 부여
* 각 팀원의 개인 Google 계정 사용
* GCP IAM에서 권한 부여
  * Computer Instance Admin (v1)
  * Service Account User

## 9.2 VM 접속 방법

### 9.1 GCP Console의 웹 SSH 방식으로 접근
* 절차:
1. https://console.cloud.google.com 접속
2. 개인 Google 계정 로그인
3. 프로젝트 선택
4. Compute Engine -> VM -> SSH 버튼 클릭

### 9.2 로컬 환경에서 접속
#### 9.2.1 로컬 PC에서 최초 1회 설정
```
gcloud auth login
gcloud config set project charming-league-481306-d8
```
* Google 계정 로그인

#### 9.2.2 VM 접속 명령
```
gcloud compute ssh l4-dev-spot \
  --zone us-central1-a
```
* l4-dev-spot : VM 이름
* us-central1-a : VM이 생성된 zone

#### 9.2.3 VM 접속 확인
```
hostname
nvidia-smi
```
* GPU : nvidia-l4가 보이면 정상


## 10. 프로젝트 원격 데스크톱 접속 가이드

프로젝트의 Ubuntu VM에 GUI로 접속
PC, 태블릿, 휴대폰 어디서든 웹 브라우저만 있으면 접속 가능

### 1. 접속 정보
- **접속 URL:** https://dronebombard.ddns.net/guacamole
- **ID/PW:** (개별 전달받은 계정 사용) Discord / ide-server-cloud 참조

### 2. 접속 방법
1. Google Cloud Platform에서 VM 실행
2. 웹 SSH에 들어가서 사용자 전환 및 VNC 실행
```
sudo su - ubuntu
vncserver :1 -geometry 1920x1080 -localhost no
```
3. https://dronebombard.ddns.net/guacamole에 접속 후 로그인
4. **[모든연결]** 목록에 있는 **"dronebombard"** 아이콘을 클릭
5. 잠시 기다리면 Ubuntu Desktop 화면 나타남

### 3. ⚠️ 주의사항 (필독!)
이 원격 데스크톱은 **"하나의 모니터를 다 같이 공유하는 방식"**
- **화면 공유:** 내가 마우스를 움직이면 다른 접속자의 화면에서도 마우스가 움직임
- **동시 작업 불가:** 한 명이 코딩 중일 때 다른 사람이 마우스를 뺏으면 작업이 중단됨
- **협업:** "내가 잠깐 확인할게"라고 말하고 사용하는 '페어 프로그래밍' 용도로 쓰기
- **개별 작업:** 개별 코드 작업은 각자 local pc나 github codespace에서 작업해서 git pull하고, 그걸 VM server에서 받기.

### 4. 문제 해결
- **화면이 안 나올 때:** 브라우저 새로고침(F5)을 하기
- **로그인이 안 될 때:** 접속 URL이 `https://`로 시작하는지 확인하기
- **한영 전환:** (설정한 방식에 따라 기입, 예: Shift+Space 또는 한영키)
- **클립보드 복사/붙여넣기:** `Ctrl+Alt+Shift`를 누르면 Guacamole 메뉴가 열림
    - 여기서 클립보드 내용을 입력해야 VM 내부로 텍스트가 전달됨


## 11. 노드 통합(Launch) 실행 — Gazebo Harmonic

### 시뮬레이션 구성

| 항목 | 값 |
|------|---|
| 시뮬레이터 | Gazebo Harmonic (`gz sim`) |
| 드론 모델 | `x500_bombard` (페이로드 탑재, `gazebo_models/x500_bombard/`) |
| 월드 파일 | `gazebo_models/worlds/x_marker_world.sdf` |
| 타겟 위치 | X마커 (11, 10) m — 월드 원점에서 북동쪽 |

### 11.1 최초 1회: PX4 빌드

컨테이너 내부에서 최초 1회만 실행합니다.

```bash
cd /opt/PX4-Autopilot
DONT_RUN=1 make px4_sitl gz_x500
```

### 11.2 ROS2 빌드

```bash
cd /workspace/ros2_ws
colcon build
source install/setup.bash
```

### 11.3 전체 시뮬레이션 실행 (단일 명령 — 권장)

```bash
cd /workspace/ros2_ws
source install/setup.bash
ros2 launch mission_manager drone_mission.launch.py
```

이 명령 하나로 다음 순서대로 모든 컴포넌트가 자동 기동됩니다:

| 시간 | 컴포넌트 |
|------|---------|
| t=0s | MicroXRCE-DDS Agent (PX4 ↔ ROS2 DDS 통신) |
| t=0s | Gazebo Harmonic (`x_marker_world.sdf`, `x500_bombard` 드론) |
| t=5s | PX4 SITL (`gz_x500` 타겟, Gazebo에 드론 스폰) |
| t=8s | ros_gz_bridge (Gazebo ↔ ROS2 토픽 브릿지) |
| t=12s | vision_detection (YOLOv8 X마커 탐지, 10 Hz) |
| t=0s | mission_manager / drone_controller / rl_navigation / drop_calculator |

옵션 인수:

```bash
# GUI 없이 headless 모드 (서버 환경)
ros2 launch mission_manager drone_mission.launch.py headless:=true

# 비전 노드 비활성화 (카메라 없이 FSM 테스트)
ros2 launch mission_manager drone_mission.launch.py enable_vision:=false

# RL 학습 모드 (rl_navigation_node 제외, RL 환경이 직접 속도 제어)
ros2 launch mission_manager drone_mission.launch.py rl_mode:=true
```

### 11.4 수동 실행 (개별 컴포넌트 — 디버깅용)

단일 launch 대신 각 컴포넌트를 별도 터미널에서 수동으로 실행할 수 있습니다.

**공통 환경 변수:**
```bash
GZ_PATH=/workspace/gazebo_models:/opt/PX4-Autopilot/Tools/simulation/gz/models:/opt/PX4-Autopilot/Tools/simulation/gz/worlds
```

**Terminal 1 — MicroXRCE-DDS Agent:**
```bash
MicroXRCEAgent udp4 -p 8888
```

**Terminal 2 — Gazebo Harmonic:**
```bash
# GUI 있음
GZ_SIM_RESOURCE_PATH=$GZ_PATH \
  gz sim -r /workspace/gazebo_models/worlds/x_marker_world.sdf

# GUI 없음 (headless, 서버 환경)
GZ_SIM_RESOURCE_PATH=$GZ_PATH \
  gz sim -r -s /workspace/gazebo_models/worlds/x_marker_world.sdf
```

**Terminal 3 — PX4 SITL (Gazebo 기동 후 실행):**
```bash
cd /opt/PX4-Autopilot
PX4_GZ_STANDALONE=1 \
PX4_GZ_MODEL=x500_bombard \
GZ_SIM_RESOURCE_PATH=$GZ_PATH \
  make px4_sitl gz_x500
```

> PX4는 실행 시 Gazebo에 자동으로 접속해 `x500_bombard` 드론을 스폰합니다.

**Terminal 4 — ros_gz_bridge (PX4 기동 후 실행):**
```bash
cd /workspace/ros2_ws && source install/setup.bash
ros2 run ros_gz_bridge parameter_bridge \
  --ros-args -p "config_file:=/workspace/ros2_ws/src/mission_manager/config/ros_gz_bridge.yaml"
```

**Terminal 5 — ROS2 미션 노드:**
```bash
cd /workspace/ros2_ws && source install/setup.bash
ros2 run mission_manager mission_manager_node &
ros2 run drone_controller controller &
ros2 run rl_navigation rl_navigation_node &
ros2 run drop_calculator calculator
```

### 11.5 테스트 (비전 탐지 시뮬레이션)

별도 터미널에서 가짜 탐지 신호를 발행하여 미션 FSM 동작을 테스트할 수 있습니다:

```bash
docker exec -it drone-bombard-harmonic bash

cd /workspace/ros2_ws
python3 system_tester.py
```

## Phase 5: Reinforcement Learning — Fighter Jet Fly-by Drop

### Overview

Phase 5 replaces the hard-coded `rl_navigation_node` with a trained **Soft Actor-Critic (SAC)**
policy that learns to execute a high-speed fly-by drop. Instead of hovering over the target, the
drone maintains maximum velocity through the target zone and releases the payload at the optimal
moment so that projectile physics carry it to the X-marker.

### Architecture

| Component | File | Role |
|-----------|------|------|
| Gymnasium env | `rl_navigation/drone_drop_env.py` | 15-dim obs / 5-dim action space; Gazebo reset via `gz service` |
| SAC trainer | `rl_navigation/train_sac.py` | Stable-Baselines3 SAC; TensorBoard logging; checkpoint resume |
| Evaluator | `rl_navigation/evaluate.py` | Runs N episodes; saves report, plots, JSON summary |

### Why SAC over PPO

- **Off-policy** → more sample-efficient (crucial since each Gazebo reset takes ~30 s)
- **Entropy regularization** → naturally explores diverse drop trajectories
- **Continuous actions** → maps directly to velocity commands

### Observation Space (15-dim)

| Indices | Feature | Normalization |
|---------|---------|---------------|
| 0–2 | ENU position | ÷ 50 m |
| 3–5 | ENU velocity | ÷ 15 m/s |
| 6–8 | Angular velocity (body FRD) | ÷ π rad/s |
| 9–10 | Pixel u, v (normalized) | (px/640)×2−1 |
| 11 | Detection confidence | [0, 1] |
| 12 | Payload attached flag | 0 or 1 |
| 13–14 | Relative distance to target | ÷ 50 m |

### Action Space (5-dim, all in [-1, 1])

| Index | Meaning | Physical scale |
|-------|---------|----------------|
| 0 | vx (East) | × 15 m/s |
| 1 | vy (North) | × 5 m/s |
| 2 | vz (Up) | × 3 m/s |
| 3 | yaw_rate | × 1 rad/s |
| 4 | drop_trigger | > 0 fires drop |

### Reward Design

```
per step:   time_penalty = -0.005
            hover_penalty = (tanh(speed/5) - 1) × 0.5   # penalise hovering

at drop:    speed_reward = tanh(speed/3) × 5.0           # reward high speed
            stability_penalty = -‖ang_vel‖

at impact:  accuracy_reward = -drop_error_m × 2.0        # from /rl/drop_error
```

### Training

```bash
# 1. Launch simulation in RL mode (skips rl_navigation_node)
ros2 launch mission_manager drone_mission.launch.py rl_mode:=true

# 2. In a second terminal, start SAC training
ros2 run rl_navigation train_sac

# Optional flags:
ros2 run rl_navigation train_sac --timesteps 1000000 --resume /workspace/ros2_ws/rl_checkpoints/sac_drop_final.zip

# Monitor training
tensorboard --logdir /workspace/ros2_ws/rl_logs/sac_drop
```

Checkpoints are saved to `/workspace/ros2_ws/rl_checkpoints/` every 5,000 steps.

### Evaluation

```bash
ros2 run rl_navigation evaluate --model /workspace/ros2_ws/rl_checkpoints/sac_drop_final.zip --episodes 20
```

Outputs written to `/workspace/ros2_ws/rl_eval_results/`:
- `evaluation_report.md` — metrics table (mean/std/min miss distance, mean drop speed)
- `evaluation_summary.json` — machine-readable summary
- `drop_error_dist.png` — miss distance histogram
- `episode_rewards.png` — per-episode reward curve
- `trajectory_top.png` — 2D overhead trajectory of final episode
- `speed_vs_accuracy.png` — drop speed vs miss distance scatter
