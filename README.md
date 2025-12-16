# Vision-Based Drone System

## 1. Project Overview
Vision Based Drone Bombard System은 카메라 기반 지상 타겟 인식 후 투하 지점을 계산하여 시뮬레이션 환경에서 투하 임무를 수행하는 자율비행 기능을 목표로 한다.

## 2. 기술 스택
OS : Ubuntu 22.04 LTS
ROS2 : Humble
Simulation : Gazebo
ML Framework : Pytorch
GPU : NVIDIA L4 & CUDA 12.6
CI/CD : Github Actions & Google Cloud Platform

## 3. Repository

```
/drone_drop_system
│
├── docker/
│   ├── Dockerfile              # GPU 기반 ROS2 Humble + PyTorch 환경
│   └── requirements.txt
│
|── docs/ # 여러 Documents
|
|
|
├── src/
│   ├── vision/                 # 타겟 인식, segmentation, detection
│   ├── localization/           # Target coordinate estimation
│   ├── control/                # PID, MPC 등 드론 제어 알고리즘
│   ├── drop_module/            # 투하 시점 계산 로직
│   └── utils/                  # 공통 util 모음
│
├── ros2_ws/
│   ├── src/
│   │   ├── vision_pkg/         # ROS2 Vision Node
│   │   ├── control_pkg/        # ROS2 Control Node
│   │   └── drop_pkg/           # ROS2 Drop Node
│   └── install / build         # 빌드 결과
│
├── simulation/
│   ├── worlds/                 # Gazebo world
│   ├── models/                 # UAV, target model
│   └── launch/                 # ROS2 launch 파일
│
|
|── models/                     # 사용하는 드론과 카메라에 대한 xacro 파일
|
|
├── tests/                      # 유닛 테스트 & 시뮬레이션 평가 코드
│
├── scripts/                    # 헬퍼 스크립트
│
├── .github/
│   └── workflows/
│       └── deploy.yml          # CI/CD 배포 파이프라인
│
└── README.md
```

# 4. 개발 규칙

# 5. 실행 가이드