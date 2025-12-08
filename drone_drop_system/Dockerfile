FROM nvidia/cuda:12.6.0-cudnn-runtime-ubuntu22.04

ENV DEBIAN_FRONTEND=noninteractive

# -------------------------------------------------------------
# 1) 기본 패키지 설치 (ROS 관련 패키지는 설치하지 않음)
# -------------------------------------------------------------
RUN apt-get update && apt-get install -y \
    wget curl git vim nano lsb-release \
    build-essential cmake \
    python3 python3-pip \
    locales software-properties-common \
    && apt-get clean && rm -rf /var/lib/apt/lists/*

# -------------------------------------------------------------
# 2) Locale 설정
# -------------------------------------------------------------
RUN locale-gen en_US en_US.UTF-8
ENV LANG=en_US.UTF-8 LC_ALL=en_US.UTF-8

# -------------------------------------------------------------
# 3) ROS2 리포지토리 추가
# -------------------------------------------------------------
RUN add-apt-repository universe && \
    curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.asc | apt-key add - && \
    sh -c 'echo "deb http://packages.ros.org/ros2/ubuntu $(lsb_release -cs) main" > /etc/apt/sources.list.d/ros2.list'

# -------------------------------------------------------------
# 4) ROS2 + colcon 관련 패키지 설치
# -------------------------------------------------------------
RUN apt-get update && apt-get install -y \
    ros-humble-desktop \
    ros-dev-tools \
    python3-colcon-common-extensions \
    && apt-get clean && rm -rf /var/lib/apt/lists/*

# ROS2 소스
SHELL ["/bin/bash", "-c"]
RUN echo "source /opt/ros/humble/setup.bash" >> ~/.bashrc

# -------------------------------------------------------------
# 5) Python 패키지
# -------------------------------------------------------------
RUN pip3 install --upgrade pip && \
    pip3 install numpy opencv-python scipy matplotlib

# -------------------------------------------------------------
# 6) PyTorch (CUDA 12.6)
# -------------------------------------------------------------
RUN pip3 install torch torchvision --index-url https://download.pytorch.org/whl/cu126

# -------------------------------------------------------------
# 7) 작업 디렉토리
# -------------------------------------------------------------
WORKDIR /workspace
CMD ["bash"]
