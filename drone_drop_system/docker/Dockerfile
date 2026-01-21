# [변경됨] NVIDIA CUDA 대신 일반 Ubuntu 22.04 사용 (CPU 환경 최적화)
FROM nvidia/cuda:12.6.2-cudnn-runtime-ubuntu22.04

ENV DEBIAN_FRONTEND=noninteractive
ENV LANG=en_US.UTF-8 LC_ALL=en_US.UTF-8


# --------------------------------------------------
# 1. 기본 툴 설치
# --------------------------------------------------
RUN apt-get update && apt-get install -y \
    wget curl git vim nano \
    build-essential cmake \
    python3 python3-pip python3-jinja2 python3-numpy \
    python3-yaml python3-empy \
    locales lsb-release software-properties-common \
    gpg \
    && rm -rf /var/lib/apt/lists/* \
    && locale-gen en_US.UTF-8

# --------------------------------------------------
# 2. ROS2 Humble 설치
# --------------------------------------------------
RUN add-apt-repository universe && \
    curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key \
    | gpg --dearmor -o /usr/share/keyrings/ros-archive-keyring.gpg && \
    echo "deb [signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] \
    http://packages.ros.org/ros2/ubuntu $(lsb_release -cs) main" \
    > /etc/apt/sources.list.d/ros2.list

RUN apt-get update && apt-get install -y \
    ros-humble-desktop \
    ros-dev-tools \
    python3-colcon-common-extensions \
    && rm -rf /var/lib/apt/lists/*

# [중요] 시스템에 깔린 기본 px4_msgs는 충돌 원인이므로 삭제
RUN apt-get remove -y ros-humble-px4-msgs || true

# --------------------------------------------------
# 3. Gazebo Classic & ROS 2 연동 플러그인 (핵심 추가 사항)
# --------------------------------------------------
# [추가됨] 이전에 수동 설치했던 gazebo_ros_pkgs 등을 미리 설치
RUN apt-get update && apt-get install -y \
    gazebo \
    libgazebo11 \
    libgazebo-dev \
    ros-humble-gazebo-ros-pkgs \
    ros-humble-cv-bridge \
    ros-humble-image-transport \
    && rm -rf /var/lib/apt/lists/*

# --------------------------------------------------
# 4. PX4 의존성
# --------------------------------------------------
RUN apt-get update && apt-get install -y \
    protobuf-compiler \
    libeigen3-dev \
    libxml2-dev \
    libopencv-dev \
    libcurl4-openssl-dev \
    libtinyxml2-dev \
    libgstreamer1.0-dev \
    libgstreamer-plugins-base1.0-dev \
    && rm -rf /var/lib/apt/lists/*

# --------------------------------------------------
# 5. Python 패키지
# --------------------------------------------------
COPY requirements.txt /tmp/requirements.txt
RUN pip3 install --no-cache-dir -r /tmp/requirements.txt || true

# --------------------------------------------------
# 6. Claude Code (선택 사항)
# --------------------------------------------------
RUN curl -fsSL https://deb.nodesource.com/setup_20.x | bash - && \
    apt-get install -y nodejs && \
    npm install -g @anthropic-ai/claude-code

# --------------------------------------------------
# 7. uXRCE-DDS-Agent 설치
# --------------------------------------------------
WORKDIR /opt
RUN git clone https://github.com/eProsima/Micro-XRCE-DDS-Agent.git && \
    cd Micro-XRCE-DDS-Agent && \
    mkdir build && cd build && \
    cmake .. && \
    make -j$(nproc) && \
    make install && \
    ldconfig

# --------------------------------------------------
# 8. PX4 Autopilot 소스 (펌웨어) - 1.15.4 version
# --------------------------------------------------
WORKDIR /opt
RUN git clone --recursive -b v1.15.4 https://github.com/PX4/PX4-Autopilot.git

# --------------------------------------------------
# 9. px4_msgs 설치 및 빌드
# --------------------------------------------------
WORKDIR /root/ros2_ws/src
RUN git clone https://github.com/PX4/px4_msgs.git

# [핵심 1] GitHub 원본 msg 파일을 지우고, 설치된 PX4 버전의 msg 파일로 교체
RUN rm -rf px4_msgs/msg/*.msg && \
    cp /opt/PX4-Autopilot/msg/*.msg px4_msgs/msg/

# [핵심 2] 우리가 만든 강력한 CMakeLists.txt 복사 (GLOB 적용 버전)
COPY config/custom_CMakeLists.txt px4_msgs/CMakeLists.txt

# [핵심 3] 미리 빌드해둠 (이미지 안에 박제)
WORKDIR /root/ros2_ws
RUN /bin/bash -c "source /opt/ros/humble/setup.bash && \
    colcon build --packages-select px4_msgs --symlink-install"

# --------------------------------------------------
# 10. 환경 설정
# --------------------------------------------------
SHELL ["/bin/bash", "-c"]

RUN echo "source /opt/ros/humble/setup.bash" >> ~/.bashrc && \
    echo "source /root/ros2_ws/install/setup.bash" >> ~/.bashrc && \
    echo "export GAZEBO_MODEL_PATH=\$GAZEBO_MODEL_PATH:/opt/PX4-Autopilot/Tools/simulation/gazebo-classic/sitl_gazebo-classic/models" >> ~/.bashrc && \
    echo "export ROS_DOMAIN_ID=0" >> ~/.bashrc

# Entrypoint 스크립트 복사 및 권한 설정
COPY entrypoint.sh /entrypoint.sh
RUN chmod +x /entrypoint.sh

WORKDIR /workspace
ENTRYPOINT ["/entrypoint.sh"]
CMD ["bash"]