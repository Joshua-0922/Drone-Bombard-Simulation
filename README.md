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
  --env-file /opt/drone-bombard/.wandb.env \
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
4. Compute Engine → VM → SSH 버튼 클릭

### 9.2 로컬 환경에서 접속
#### 9.2.1 로컬 PC에서 최초 1회 설정
```bash
gcloud auth login
gcloud config set project charming-league-481306-d8
```
* Google 계정 로그인

#### 9.2.2 VM 접속 명령
```bash
gcloud compute ssh l4-spot \
  --zone asia-east1-a
```
* `l4-spot` : 현재 VM 이름
* `asia-east1-a` : VM이 생성된 zone

#### 9.2.3 VM 접속 확인
```bash
hostname    # l4-spot
nvidia-smi  # NVIDIA L4가 보이면 정상
```


## 10. 프로젝트 원격 데스크톱 접속 가이드

프로젝트의 Ubuntu VM에 GUI로 접속
PC, 태블릿, 휴대폰 어디서든 웹 브라우저만 있으면 접속 가능

> ⚠️ **Spot VM 특성상 선점(preemption) 후 재기동 시 외부 IP가 바뀔 수 있습니다.**
> IP 변경 시 Discord `#vm-status` 채널에서 최신 IP를 확인하세요.
> 현재 외부 IP: `gcloud compute instances describe l4-spot --zone=asia-east1-a --format="value(networkInterfaces[0].accessConfigs[0].natIP)"`

### 1. 접속 정보
- **접속 URL:** https://130.211.241.166 (HTTP 접속 시 자동 HTTPS 리다이렉트)
- **ID/PW:** (개별 전달받은 계정 사용) Discord / ide-server-cloud 참조
- ⚠️ **자체 서명 SSL 인증서** 사용 — 브라우저에서 "연결이 안전하지 않음" 경고가 뜨면 **고급 → 계속 진행** 클릭 (1회만)

#### 접속 전 확인 — GCP 방화벽 (포트 443)
브라우저에서 접속이 안 될 때는 GCP VPC 방화벽에 HTTPS(443) 규칙이 없는 경우입니다.
아래 명령으로 한 번만 추가하면 됩니다 (팀 관리자가 설정):
```bash
gcloud compute firewall-rules create allow-https-novnc \
  --direction=INGRESS \
  --action=ALLOW \
  --rules=tcp:443 \
  --source-ranges=0.0.0.0/0 \
  --target-tags=l4-spot \
  --project=charming-league-481306-d8
```

### 2. 아키텍처

```
[브라우저]
    │ HTTPS :443
    ▼
[nginx container]              ← Docker Compose
    │ proxy :8080
    ▼
[guacamole container]          ← Docker Compose
    │ guacd protocol :4822
    ▼
[guacd container]              ← Docker Compose
    │ VNC :5901  (host.docker.internal)
    ▼
[TigerVNC — VM 호스트, systemd 자동 관리]
    │ X11 display :1
    ▼
[XFCE4 데스크탑 on VM]
```

### 3. 접속 방법

#### 3.1 정상 접속 (VNC + Guacamole 모두 실행 중인 경우)
1. **https://130.211.241.166** 접속 → SSL 경고 무시 후 진행
2. Guacamole 로그인
3. **[모든연결]** 목록에서 **"VM-XFCE4"** 클릭
4. 잠시 기다리면 XFCE4 Ubuntu Desktop 화면 나타남

#### 3.2 VM 재부팅 또는 서비스 재시작 후
VNC는 systemd가 자동으로 재시작합니다. 아래 명령으로 상태를 확인하고 필요 시 수동 시작합니다.

```bash
# VNC 서버 상태 확인
sudo systemctl status vncserver@1

# VNC 서버 수동 시작 (자동 시작이 안 됐을 때)
sudo systemctl start vncserver@1

# VNC 실행 여부 확인
vncserver -list      # ":1" 라인이 보이면 정상
ss -tlnp | grep 5901 # LISTEN 상태 확인
```

```bash
# Guacamole Docker 스택 상태 확인
cd /opt/drone-bombard/guacamole-stack
docker compose ps    # guacd, guacamole, postgres, nginx 4개 모두 Up (healthy) 확인

# Guacamole 스택 시작 (내려가 있을 때)
docker compose up -d

# 로그 확인 (기동 완료까지 ~30–60초)
docker compose logs -f guacamole   # "Guacamole is now listening on port 8080" 확인
```

#### 3.3 VNC를 수동으로 직접 실행해야 할 때
```bash
# 기존 세션 종료 후 재시작
vncserver -kill :1
vncserver :1 -geometry 1920x1080 -depth 24 -localhost no -SecurityTypes VncAuth
```

### 4. ⚠️ 주의사항 (필독!)
이 원격 데스크톱은 **"하나의 모니터를 다 같이 공유하는 방식"**
- **화면 공유:** 내가 마우스를 움직이면 다른 접속자의 화면에서도 마우스가 움직임
- **동시 작업 불가:** 한 명이 코딩 중일 때 다른 사람이 마우스를 뺏으면 작업이 중단됨
- **협업:** "내가 잠깐 확인할게"라고 말하고 사용하는 '페어 프로그래밍' 용도로 쓰기
- **개별 작업:** 개별 코드 작업은 각자 local pc나 github codespace에서 작업해서 git pull하고, 그걸 VM server에서 받기.
- **VNC 비밀번호 8자 한도:** DES 기반 암호화로 앞 8자만 유효 — 비밀번호는 8자 이하로 설정할 것

### 5. 문제 해결
| 증상 | 해결 |
|------|------|
| 화면이 안 나올 때 | 브라우저 새로고침(F5) → 안 되면 `sudo systemctl restart vncserver@1` |
| SSL 인증서 경고 | 브라우저에서 **고급 → 계속 진행** 클릭 (1회) |
| Guacamole 페이지 자체가 안 열릴 때 | `cd /opt/drone-bombard/guacamole-stack && docker compose up -d` |
| 접속은 되는데 화면이 검정일 때 | `vncserver -kill :1` 후 `sudo systemctl start vncserver@1` |
| 클립보드 복사/붙여넣기 | `Ctrl+Alt+Shift`로 Guacamole 사이드 메뉴 열기 → 클립보드 탭에서 텍스트 입력 |
| 한영 전환 | Shift+Space 또는 한영키 |


## 11. 시뮬레이션 실행 — Gazebo Harmonic

### 시뮬레이션 구성

| 항목 | 값 |
|------|---|
| 시뮬레이터 | Gazebo Harmonic (`gz sim`) |
| 드론 모델 | `x500_bombard` (페이로드 탑재, `gazebo_models/x500_bombard/`) |
| 월드 파일 | `gazebo_models/worlds/x_marker_world.sdf` |
| 타겟 위치 | X마커 (11, 10) m — 월드 원점에서 북동쪽 |

---

### 11.1 최초 1회: PX4 빌드

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

---

### 11.3 단일 실행 (데모·디버그용)

Launch 파일 하나로 전체 스택을 기동합니다.

```bash
cd /workspace/ros2_ws && source install/setup.bash
ros2 launch mission_manager drone_mission.launch.py
```

자동 기동 순서:

| 시간 | 컴포넌트 |
|------|---------|
| t=0s | MicroXRCE-DDS Agent |
| t=0s | Gazebo Harmonic |
| t=12s | PX4 SITL |
| t=16s | ros_gz_bridge |
| t=22s | vision_detection (YOLOv8) |
| t=0s | mission_manager / drone_controller / rl_navigation / drop_calculator |

옵션 인수:

```bash
ros2 launch mission_manager drone_mission.launch.py headless:=true
ros2 launch mission_manager drone_mission.launch.py enable_vision:=false
ros2 launch mission_manager drone_mission.launch.py rl_mode:=true
```

재실행 전 이전 프로세스 정리:

```bash
pkill -f "gz sim" ; pkill -f "px4" ; pkill -f "MicroXRCEAgent" ; pkill -f "ros2"
```

---

### 11.4 RL 학습용 2-레이어 실행 (권장)

에피소드를 수만 번 반복하는 RL 학습에서는 프로세스를 **역할별로 분리**합니다.

```
┌─────────────────────────────────────────────────┐
│  INFRA LAYER  (학습 세션 내내 유지)              │
│  infra.launch.py                                 │
│  - MicroXRCEAgent  (DDS 브릿지, 완전 무상태)    │
│  - Gazebo Harmonic (물리 엔진; 에피소드 간 리셋)│
│  - ros_gz_bridge   (토픽 포워더)                │
│  - xmarker_detector (YOLO 모델 상주)            │
└─────────────────────────────────────────────────┘
             ↕  에피소드마다 재시작
┌─────────────────────────────────────────────────┐
│  EPISODE LAYER  (DroneDropEnv.reset()가 자동 관리)│
│  episode.launch.py                               │
│  - PX4 SITL         (t=2s, Gazebo 이미 기동 중) │
│  - mission_manager  (FSM 커맨더)                │
│  - drone_controller (PX4 브릿지)                │
│  - drop_calculator  (착탄 오차 계산)            │
└─────────────────────────────────────────────────┘
```

**에피소드당 소요 시간 (단일 환경 기준):**

| 단계 | 시간 |
|------|-----|
| 에피소드 프로세스 종료 | ~1.5 s |
| Gazebo 월드 리셋 | ~0.5 s |
| PX4 재기동 및 드론 스폰 | ~3 s |
| ARM + Offboard 진입 | ~2 s |
| 10 m 고도 도달 후 CRUISE 진입 | ~5 s |
| **합계** | **~12 s** |

#### Step 1 — Infra Layer 기동 (1회)

```bash
cd /workspace/ros2_ws && source install/setup.bash
ros2 launch mission_manager infra.launch.py          # headless=true 기본값
ros2 launch mission_manager infra.launch.py headless:=false  # GUI 포함
```

Gazebo가 완전히 로드될 때까지(~25 s) 기다린 후 학습을 시작합니다.

#### Step 2 — SAC 학습 시작 (별도 터미널)

```bash
cd /workspace/ros2_ws && source install/setup.bash
ros2 run rl_navigation train_sac
```

`DroneDropEnv.reset()` 가 호출될 때마다 자동으로:
1. 이전 에피소드 프로세스 종료 (`SIGTERM` → 프로세스 그룹)
2. Gazebo 월드 리셋 (`gz service`)
3. `episode.launch.py` 재기동
4. CRUISE 상태까지 대기

체크포인트는 `/workspace/ros2_ws/rl_checkpoints/`에 5,000 스텝마다 저장됩니다.

```bash
# 학습 재개
ros2 run rl_navigation train_sac --resume /workspace/ros2_ws/rl_checkpoints/sac_drop_final.zip

# 학습 모니터링
tensorboard --logdir /workspace/ros2_ws/rl_logs/sac_drop
```

#### Step 3 — 평가

```bash
ros2 run rl_navigation evaluate \
  --model /workspace/ros2_ws/rl_checkpoints/sac_drop_final.zip \
  --episodes 20
```

결과는 `/workspace/ros2_ws/rl_eval_results/`에 저장됩니다.

---

### 11.5 수동 실행 (개별 컴포넌트 — 디버깅용)

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
GZ_SIM_RESOURCE_PATH=$GZ_PATH gz sim -r -s /workspace/gazebo_models/worlds/x_marker_world.sdf
```

**Terminal 3 — PX4 SITL (Gazebo 기동 후):**
```bash
cd /opt/PX4-Autopilot/build/px4_sitl_default/src/modules/simulation/gz_bridge
PX4_GZ_STANDALONE=1 PX4_GZ_WORLD=x_marker_world PX4_SIM_MODEL=gz_x500_bombard \
GZ_SIM_RESOURCE_PATH=$GZ_PATH \
  /opt/PX4-Autopilot/build/px4_sitl_default/bin/px4
```

**Terminal 4 — ros_gz_bridge:**
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

---

### 11.6 테스트 (비전 탐지 시뮬레이션)

```bash
docker exec -it drone-bombard-harmonic bash
cd /workspace/ros2_ws && python3 system_tester.py
```

### 11.7 실시간 모니터링

```bash
# 미션 상태 + 투하 오차
ros2 topic echo /mission/state &
ros2 topic echo /rl/drop_error

# 페이로드 위치
ros2 topic echo /drone/payload/position

# 카메라 영상 (RViz2)
rviz2
# Add → By topic → /vision/annotated_image
```

### 11.8 전체 미션 시퀀스

```
[INFRA]  Gazebo 기동 → x_marker_world 로드
[INFRA]  t=16s ros_gz_bridge 기동
[INFRA]  t=22s YOLOv8 노드 기동

--- 에피소드 시작 (reset() 호출) ---
[EPISODE] Gazebo 월드 리셋 → 드론·페이로드 초기 위치 복원
[EPISODE] PX4 SITL 재기동 → t=2s 후 드론 스폰

[TAKEOFF]   drone_controller: ARM + OFFBOARD 진입, ENU (0,0,10) 상승
[CRUISE]    고도 10m 도달 → 북동쪽 1 m/s 순항
[TRACKING]  X마커 감지 → rl_navigation 속도 제어 인수
[DROP]      /payload/drop_cmd → DetachableJoint 해제 → 자유낙하
            drop_calculator → /rl/drop_error (m)
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
| Gymnasium env | `rl_navigation/drone_drop_env.py` | 15-dim obs / 5-dim action space; manages episode.launch.py process group |
| SAC trainer | `rl_navigation/train_sac.py` | Stable-Baselines3 SAC; TensorBoard logging; checkpoint resume |
| Evaluator | `rl_navigation/evaluate.py` | Runs N episodes; saves report, plots, JSON summary |

### Why SAC over PPO

- **Off-policy** → more sample-efficient (crucial since each episode reset takes ~12 s)
- **Entropy regularization** → naturally explores diverse drop trajectories
- **Continuous actions** → maps directly to velocity commands

### Reset Architecture

`DroneDropEnv.reset()` runs this sequence on every episode boundary:

```
1. os.killpg(SIGTERM)   → kill PX4 + mission nodes (process group)  ~1.5 s
2. gz service call       → Gazebo world reset (physics + poses)       ~0.5 s
3. episode.launch.py     → PX4 reconnects to Gazebo, drone spawned    ~2 s
4. _wait_for_cruise()    → blocks until TAKEOFF → CRUISE complete     ~8 s
                                                          Total: ~12 s/episode
```

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

### Hyperparameter Config

All training, SAC, environment, and WandB settings live in one file:

```
ros2_ws/src/rl_navigation/config/hyperparams.yaml
```

Edit the yaml and rebuild (`colcon build --packages-select rl_navigation`) to apply changes. No code edits needed for tuning runs.

### WandB Setup

```bash
# Inside container — one-time login
wandb login

# Set your entity (username or team) in hyperparams.yaml:
#   wandb:
#     entity: "your-wandb-username"
```

WandB run resumes automatically after Spot VM preemption (uses `resume="allow"`).

### CUDA / GPU

`device: "cuda"` in `hyperparams.yaml` enables the NVIDIA L4 GPU. Change to `"cpu"` if training without a GPU. SB3 runs fp32 on CUDA by default.

### Spot VM Resilience

The training script handles GCP Spot VM preemption automatically:

- **SIGTERM handler** — when GCP sends SIGTERM (~30 s before eviction), saves `sac_drop_preempt.zip` + `sac_drop_preempt_replay.pkl` and uploads to WandB
- **Replay buffer checkpoint** — saved every 5,000 steps alongside model checkpoints; avoids cold-start after preemption
- **WandB resume** — same run ID continues after restart

```bash
# After preemption, resume from emergency checkpoint
ros2 run rl_navigation train_sac \
  --resume /workspace/ros2_ws/rl_checkpoints/sac_drop_preempt.zip
```

### Training

```bash
# Terminal 1 — start persistent infra (once per session)
ros2 launch mission_manager infra.launch.py

# Terminal 2 — start SAC training (episode.launch.py managed automatically)
ros2 run rl_navigation train_sac

# With explicit config path
ros2 run rl_navigation train_sac \
  --config /workspace/ros2_ws/src/rl_navigation/config/hyperparams.yaml

# Resume from checkpoint
ros2 run rl_navigation train_sac \
  --resume /workspace/ros2_ws/rl_checkpoints/sac_drop_final.zip

# Monitor (TensorBoard + WandB both active)
tensorboard --logdir /workspace/ros2_ws/rl_logs/sac_drop
```

Checkpoints saved to `/workspace/ros2_ws/rl_checkpoints/` every 5,000 steps (model + replay buffer).

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

---

## 12. VM 접속 가이드 (팀원용)

### VM 정보

| 항목 | 값 |
|------|----|
| VM 이름 | `l4-spot` |
| Zone | `asia-east1-a` |
| GCP 프로젝트 | `charming-league-481306-d8` |
| GPU | NVIDIA L4 |
| 타입 | Spot VM (선점 시 자동 재시작, IP 변경 가능) |
| 현재 외부 IP | `130.211.241.166` (변경될 수 있음) |

> **IP 확인 명령:**
> ```bash
> gcloud compute instances describe l4-spot \
>   --zone=asia-east1-a --project=charming-league-481306-d8 \
>   --format="value(networkInterfaces[0].accessConfigs[0].natIP)"
> ```

### 접속 방법

**방법 1 — gcloud SSH (권장)**
```bash
gcloud compute ssh l4-spot \
  --zone=asia-east1-a --project=charming-league-481306-d8
```

**방법 2 — Guacamole 웹 GUI (브라우저)**
```
https://130.211.241.166
계정: Discord / ide-server-cloud 채널 참조
```
- SSL 경고 시 **고급 → 계속 진행** 클릭
- Spot VM 선점 후 재기동 시 IP가 바뀔 수 있으니 위 IP 확인 명령으로 최신 IP 확인

**방법 3 — VNC 직접 (SSH 터널)**
```bash
gcloud compute ssh l4-spot \
  --zone=asia-east1-a -- -L 5901:localhost:5901
# 이후 VNC 클라이언트로 localhost:5901 접속
```

모든 팀 공용 작업 경로: `/opt/drone-bombard/`

---

## 13. Spot VM 무인 학습 자동화

### 아키텍처 개요

```
[Spot VM — Training]
    │
    ├── SIGTERM handler (train_sac.py)
    │       └── 선점 30초 전 경고 → sac_drop_preempt.zip + _replay.pkl 저장
    │
    └── VM TERMINATED
            ↓
[Cloud Scheduler, 5분 주기]
            ↓
[Cloud Function: drone-watchdog]
    ├── TERMINATED + training_active=true + operation=preempted → 재시작
    ├── TERMINATED + training_active=false → 수동 종료, 재시작 안 함
    ├── Cooldown 15분 미만 → skip
    └── 연속 3회 초과 → 중단 + Cloud Logging 경고
            ↓
[VM 재시작 → startup.sh 자동 실행]
    └── train_managed.sh → 최신 체크포인트 탐지 → 학습 재개
```

### VM 수동 종료 (중요!)

> ⚠️ **GCP 콘솔 또는 `gcloud compute instances stop` 직접 사용 금지!**
> watchdog이 선점으로 오인하여 자동 재시작할 수 있습니다.

반드시 아래 스크립트 사용:
```bash
bash infra/stop.sh
```

이 스크립트는:
1. `drone_training_active=false` 메타데이터 설정 (watchdog 재시작 방지)
2. 학습 프로세스에 SIGTERM 전송 → `sac_drop_preempt.zip` 저장
3. VM 종료

### 체크포인트 구조

| 타입 | 경로 | 용도 |
|------|------|------|
| 선점 저장 | `rl_checkpoints/sac_drop_preempt.zip` + `_replay.pkl` | **자동 resume 1순위** |
| 정기 저장 | `rl_checkpoints/sac_drop_*_steps.zip` | 자동 resume 2순위 (최신 기준) |
| Best model | `rl_checkpoints/best_model/best_model.zip` | 저장 전용 — 자동 resume 대상 아님 |
| Milestone | `rl_checkpoints/archive/sac_drop_milestone_*.zip` | 아카이브 전용 |

> **best_model에서 학습 재시작하려면 수동으로:**
> WandB 결과를 보고 판단 후 `infra/stop.sh`로 종료 → 직접 `train_managed.sh --fresh --resume best_model/best_model.zip` 실행

### 학습 모니터링

```bash
# VM 로그 (부팅 및 startup.sh 실행 결과)
sudo cat /var/log/drone-bombard-startup.log

# 학습 로그 (컨테이너 내부)
docker exec drone-bombard-harmonic tail -f /tmp/production_train.log

# 컨테이너 로그
docker logs -f drone-bombard-harmonic

# WandB: https://wandb.ai/nayoonho0922-seoul-national-university/drone-bombard-sac
```

### 인프라 배포 (최초 1회)

```bash
bash infra/deploy.sh
```

포함 내용: GPU 가용성 확인 → 디스크 스냅샷 → VM 마이그레이션 → Cloud Function 배포 → Cloud Scheduler 설정

### 재시작 카운터 수동 리셋 (watchdog 중단 시)

연속 3회 재시작 실패로 watchdog이 멈췄을 때:
```bash
gcloud compute instances add-metadata l4-spot \
  --zone=asia-east1-a --project=charming-league-481306-d8 \
  --metadata watchdog_restart_count=0
```
- `speed_vs_accuracy.png` — drop speed vs miss distance scatter
