# -----------------------------------------------------------
# Base image: NVIDIA CUDA + Ubuntu 22.04
# -----------------------------------------------------------
FROM nvidia/cuda:12.2.2-devel-ubuntu22.04

# Prevent interactive prompts
ENV DEBIAN_FRONTEND=noninteractive

# -----------------------------------------------------------
# Install base tools
# -----------------------------------------------------------
RUN apt-get update && apt-get install -y \
    curl \
    wget \
    git \
    lsb-release \
    build-essential \
    cmake \
    python3 \
    python3-pip \
    python3-colcon-common-extensions \
    python3-vcstool \
    locales \
    && rm -rf /var/lib/apt/lists/*

# Locale 설정
RUN locale-gen en_US en_US.UTF-8
ENV LANG en_US.UTF-8
ENV LC_ALL en_US.UTF-8

# -----------------------------------------------------------
# Install ROS2 Humble
# -----------------------------------------------------------
RUN apt-get update && apt-get install -y \
    software-properties-common \
    && add-apt-repository universe

RUN curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -o /usr/share/keyrings/ros-archive-keyring.gpg && \
    echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] \
    http://packages.ros.org/ros2/ubuntu $(lsb_release -cs) main" \
    > /etc/apt/sources.list.d/ros2.list

RUN apt-get update && apt-get install -y \
    ros-humble-desktop \
    ros-humble-cv-bridge \
    ros-humble-image-transport \
    ros-humble-vision-msgs \
    && rm -rf /var/lib/apt/lists/*

# ROS2 환경변수
SHELL ["/bin/bash", "-c"]
RUN echo "source /opt/ros/humble/setup.bash" >> ~/.bashrc
ENV ROS_DISTRO=humble

# -----------------------------------------------------------
# Python Dependencies (requirements.txt)
# -----------------------------------------------------------
WORKDIR /workspace

COPY requirements.txt .
RUN python3 -m pip install --upgrade pip
RUN pip install -r requirements.txt

# -----------------------------------------------------------
# Project workspace
# -----------------------------------------------------------
RUN mkdir -p /workspace/ros2_ws/src
WORKDIR /workspace/ros2_ws

# Source 환경 자동 로드
RUN echo "source /workspace/ros2_ws/install/setup.bash" >> ~/.bashrc

CMD ["bash"]
