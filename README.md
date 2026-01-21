# Vision-Based Drone System

## 1. Project Overview
Vision Based Drone Bombard System은 카메라 기반 지상 타겟 인식 후 투하 지점을 계산하여 시뮬레이션 환경에서 투하 임무를 수행하는 자율비행 기능을 목표로 한다.

## 2. Operating Principles
Warning: GCP VM은 사용자 계정별 홈 디렉토리가 분리되어 있다.

모든 팀 공용 작업은 다음 경로에서 수행한다.
/opt/drone-bombard

개인 홈 디렉토리(/home/username)는 사용하지 않는다.

### 1. Repository에는 “소스 코드만” 관리한다

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
* main 브랜치 push → 자동 빌드 → Artifact Registry 저장

### 5. ROS2 패키지는 컨테이너 내부에서 생성한다.
* 패키지 생성 위치
```
cd /workspace/ros2_ws/src
ros2 pkg create <package_name> --build-type ament_pthon
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
* 이름은 path-generation, CV, autonomy, px4 등으로, 만들기 전 회의 통해 결정

#### 6.1 path-generation branch 만들기.
```
git checkout main
git pull
git checkout -b feature/path-generation
```
#### 6.2 main과 합치기
```
git checkout main
git pull
git merge feature/path-generation
git push
```

## 3. 기술 스택
OS : Ubuntu 22.04 LTS
ROS2 : Humble
Simulation : Gazebo
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
├── SYSTEM_ARCHITECTURE.md     # 시스템 아키텍처 및 기능 문서
├── MODEL_INFO.md              # YOLO 모델 정보
├── drone_drop_system/         # 드론 투하 시뮬레이션 및 ML/CV 로직
│   └── docker/                # Dockerfile, entrypoint, requirements
├── gazebo_models/             # Gazebo 시뮬레이션 모델
│   ├── iris_bombard/          # 드론 모델
│   ├── x_marker/              # X자 표식 모델
│   └── worlds/                # 시뮬레이션 월드
├── ros2_ws/                   # ROS2 전용 워크스페이스 (Git으로 관리)
│   ├── src/                   # ROS2 패키지 소스
│   │   ├── vision_detection/  # X자 표식 탐지 패키지
│   │   ├── path_generation/   # 드론 경로 생성 패키지
│   │   ├── drop_calculator/   # 투하 타이밍 계산 패키지
│   │   ├── mechanism_controller/  # (개발 예정) 투하 메커니즘 제어
│   │   └── px4_msgs/          # PX4 메시지 타입
│   ├── yolo_workspace/        # YOLO 학습 관련
│   │   ├── datasets/          # 학습 데이터셋
│   │   ├── runs/              # 학습 결과
│   │   └── scripts/           # 학습 스크립트
│   ├── VISION_DETECTION_README.md  # vision_detection 패키지 가이드
│   ├── setup_simulation.bash  # 시뮬레이션 환경 설정 스크립트
│   └── test_vision_detection.sh    # Vision 테스트 스크립트
└── drone_bombard_best.pt      # 학습된 YOLO 모델 (최적 가중치)
``` 

디렉토리 역할 요약
* `ros2_ws/`
  * ROS2 Humble 기준 워크스페이스
  * `src/` 아래에만 ROS2 패키지 생성
  * `build/`, `install/`, `log/`는 로컬/컨테이너 빌드 산출물 (Git 관리 제외)

* `gazebo_models/`
  * Gazebo 시뮬레이션용 3D 모델 및 월드 파일

* `yolo_workspace/`
  * YOLO 모델 학습 및 평가 관련 파일

**📖 시스템 아키텍처 및 기능에 대한 자세한 내용은 [SYSTEM_ARCHITECTURE.md](./SYSTEM_ARCHITECTURE.md)를 참고하세요.**

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
```
# 1. (필수) 호스트에서 화면 권한 허용 (실행 전 1회)
xhost +local:docker

# 2. 컨테이너 실행
docker run -itd \
  --gpus all \
  --net=host \
  --privileged \
  --ipc=host \
  --name drone-bombard-dev \
  --env="DISPLAY=$DISPLAY" \
  --env="QT_X11_NO_MITSHM=1" \
  --env="NVIDIA_DRIVER_CAPABILITIES=all" \
  --volume="/tmp/.X11-unix:/tmp/.X11-unix:rw" \
  --volume="$HOME/.Xauthority:/root/.Xauthority:rw" \
  -v /opt/drone-bombard/Drone-Bombard-Simulation/ros2_ws:/workspace/ros2_ws \
  -v /opt/drone-bombard/Drone-Bombard-Simulation/gazebo_models:/workspace/gazebo_models \
  -v ~/.cache:/root/.cache \
  -v "$(pwd)/claude_config:/root/.anthropic" \
  us-central1-docker.pkg.dev/charming-league-481306-d8/drone-bombard/drone-bombard:latest \
  /bin/bash
```
* {username}은 각자 user name 입력하기. 같은 이미지를 쓰되 사용자까리 컨테이너 분리
* ros2_ws는 VM과 컨테이너 공유 볼륨
* **gazebo_models는 Gazebo 시뮬레이션 모델 및 월드 파일을 공유하는 볼륨 (추가됨)**
* 컨테이너 삭제 전까지 데이터 유지

#### 7.1.1. gazebo_models 볼륨 마운트 설명
* `gazebo_models` 폴더에는 커스텀 드론 모델(`iris_bombard`)과 X 마커 모델, 테스트 월드 파일이 포함되어 있습니다.
* 이 볼륨을 마운트하면 launch 파일이 자동으로 커스텀 모델을 인식하고 사용합니다.
* 마운트하지 않아도 PX4 기본 모델로 동작하지만, 프로젝트의 커스텀 모델을 사용하려면 반드시 마운트해야 합니다.

### 7.2 기존 컨테이너 재접속
매번 접속하고 xhost +local:docker는 해줘야 한다.
```
xhost +local:docker 
docker start -ai drone-bombard-dev-{username}
```

### 7.3 기존 컨테이너 삭제
1. 기존 컨테이너 중지
```
docker stop drone-bombard-dev-{username}
```

2. 기존 컨테이너 삭제
```
docker rm drone-bombard-dev-{username}
```

### 7.4 Gazebo 시뮬레이션 실행 방법
컨테이너 내부에서 다음 명령어로 시뮬레이션을 실행할 수 있습니다:

```bash
# 컨테이너 접속
docker exec -it drone-bombard-dev-{username} /bin/bash

# 환경 설정
cd /workspace/ros2_ws
source setup_simulation.bash

# 시뮬레이션 실행 (기본값: iris_bombard 모델, x_marker_test 월드)
ros2 launch path_generation px4_gazebo_sitl.launch.py

# 또는 옵션 지정하여 실행
ros2 launch path_generation px4_gazebo_sitl.launch.py \
    world:=x_marker_test \
    model:=iris_bombard \
    enable_vision:=true \
    enable_path_generation:=true
```

**주요 변경사항:**
* launch 파일이 자동으로 `gazebo_models` 폴더를 찾아 커스텀 모델을 사용합니다.
* `gazebo_models` 볼륨이 마운트되어 있으면 `iris_bombard` 모델과 `x_marker_test` 월드가 기본값으로 설정됩니다.
* 마운트되지 않아도 PX4 기본 모델로 정상 동작합니다 (하위 호환성 유지).
## 8. Github repository와 VM
### 8.1 Github Repository로 코드 반영(VM기준)
```
cd ~/Drone-Bombard-Simulation
git status
git add ros2_ws
git commit -m "Add ROS2 package"
git pull --rebase
git push
```
### 8.2 Github Action에서 완성된 Image를 pull해서 VM에서 실행하는 방법
1. Docker Image pull하기
```
cd /opt/drone_drop_system/Drone_Bombard_Simulation
git pull --rebase
gcloud auth configure-docker us-central1-docker.pkg.dev (최초 한번만)
docker pull us-central1-docker.pkg.dev/charming-league-481306-d8/drone-bombard/drone-bombard:latest
```

2. Image 기반으로 컨테이너 만들기
```
docker ps -a 
docker rm -rf drone-bombard-dev-{$USERNAME} (이미 존재하는 컨테이너와 중복을 막기위해 삭제해야 할 때)
```
이후 docker 실행

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
- **클립보드 복사/붙여넣기:** - `Ctrl+Alt+Shift`를 누르면 Guacamole 메뉴가 열림
    - 여기서 클립보드 내용을 입력해야 VM 내부로 텍스트가 전달됨
