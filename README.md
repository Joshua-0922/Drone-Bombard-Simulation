# Vision-Based Drone System

## 1. Project Overview
Vision Based Drone Bombard System은 카메라 기반 지상 타겟 인식 후 투하 지점을 계산하여 시뮬레이션 환경에서 투하 임무를 수행하는 자율비행 기능을 목표로 한다.

## 2. Operating Principles
###1. Repository에는 “소스 코드만” 관리한다

* ROS2 패키지는 반드시 ros2_ws/src/<package_name>에 생성
* build/, install/, log/ 디렉토리는 Git 관리 대상이 아님

###2. 개발은 항상 Docker 컨테이너 내부에서 수행한다
* VM에는 Docker만 설치
* ROS2, Python 의존성, 빌드 도구는 dockerfile 수정으로 변경

###3. ros2_ws는 Host ↔ Container 볼륨으로 공유한다
* 컨테이너 삭제 후에도 코드 유지
* 동일 워크스페이스를 여러 컨테이너에서 재사용 가능

###4. Docker 이미지 빌드는 GitHub Actions가 담당한다
* 로컬에서 docker build 금지
* main 브랜치 push → 자동 빌드 → Artifact Registry 저장

###5. ROS2 패키지는 컨테이너 내부에서 생성한다.
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

*  생성 결과는 VM의 ~/Drone-Bombard-Simulation/ros2_ws/src 에 자동 반영됨.
* 이후 VM에서 github repository로 push하기.

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
├── README.md
├── drone_drop_system        # 드론 투하 시뮬레이션 및 ML/CV 로직
│   ├── docker               # Dockerfile, entrypoint, requirements
│   ├── docs                 # 프로젝트 문서
│   ├── models               # 학습된 모델 및 가중치
│   ├── scripts              # 실험·유틸 스크립트
│   ├── simulation           # 시뮬레이션 관련 코드
│   └── src                  # 핵심 Python / C++ 소스
└── ros2_ws                  # ROS2 전용 워크스페이스 (Git으로 관리)
    └── src
        └── path_generation  # ROS2 패키지 (실제 개발 대상)
``` 

디렉토리 역할 요약
* drone_drop_system/
  * Docker, 시뮬레이션, CV/ML 로직을 포함하는 메인 프로젝트 영역

* ros2_ws/
  * ROS2 Humble 기준 워크스페이스
  * src/ 아래에만 ROS2 패키지 생성
  * build/, install/, log/는 로컬/컨테이너 빌드 산출물

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
docker run -it \
  --gpus all \
  --name drone-bombard-dev \
  -v ~/Drone-Bombard-Simulation/ros2_ws:/workspace/ros2_ws \
  -v ~/.cache:/root/.cache \
  us-central1-docker.pkg.dev/charming-league-481306-d8/drone-bombard/drone-bombard:latest
```
* ros2_ws는 VM과 컨테이너 공유 볼륨
* 컨테이너 삭제 전까지 데이터 유지

### 7.2. 기존 컨테이너 재접속
```
docker start -ai drone-bombard-dev
```
## 8. Github repository로 코드 반영(VM 기준)
```
cd ~/Drone-Bombard-Simulation
git status
git add ros2_ws
git commit -m "Add ROS2 package"
git pull --rebase
git push
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
1. 로컬 PC에서 최초 1회 설정
```
gcloud auth login
gcloud config set project charming-league-481306-d8
```
* Google 계정 로그인

2. VM 접속 명령
```
gcloud compute ssh l4-dev-spot \
  --zone us-central1-a
```
* l4-dev-spot : VM 이름
* us-central1-a : VM이 생성된 zone

3. VM 접속 확인
```
hostname
nvidia-smi
```
* GPU : nvidia-l4가 보이면 정상