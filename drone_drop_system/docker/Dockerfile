# 1. NVIDIA CUDA 12.6.2 베이스 (GPU 가속용)
FROM nvidia/cuda:12.6.2-cudnn-runtime-ubuntu22.04

ENV DEBIAN_FRONTEND=noninteractive
ENV LANG=en_US.UTF-8 LC_ALL=en_US.UTF-8

# --------------------------------------------------
# 2. 기본 툴 설치
# --------------------------------------------------
RUN apt-get update && apt-get install -y \
    wget curl git vim nano \
    build-essential cmake \
    python3 python3-pip python3-jinja2 python3-numpy \
    python3-yaml python3-empy \
    locales lsb-release software-properties-common \
    gpg tzdata \
    && rm -rf /var/lib/apt/lists/* \
    && locale-gen en_US.UTF-8

# --------------------------------------------------
# 3. ROS 2 Humble 설치
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
# 4. 🚀 Gazebo Harmonic 설치
# --------------------------------------------------
RUN wget https://packages.osrfoundation.org/gazebo.gpg -O /usr/share/keyrings/pkgs-osrf-archive-keyring.gpg && \
    echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/pkgs-osrf-archive-keyring.gpg] \
    http://packages.osrfoundation.org/gazebo/ubuntu-stable $(lsb_release -cs) main" \
    > /etc/apt/sources.list.d/gazebo-stable.list

RUN apt-get update && apt-get install -y \
    gz-harmonic \
    && rm -rf /var/lib/apt/lists/*

# --------------------------------------------------
# 5. PX4 의존성 (🌟 gflags 에러 해결)
# --------------------------------------------------
RUN apt-get update && apt-get install -y \
    protobuf-compiler libeigen3-dev libxml2-dev libopencv-dev \
    libcurl4-openssl-dev libtinyxml2-dev \
    libgstreamer1.0-dev libgstreamer-plugins-base1.0-dev \
    libgflags-dev libgoogle-glog-dev \
    && rm -rf /var/lib/apt/lists/*

# --------------------------------------------------
# 6. Python 패키지 & 🧠 PyTorch(cu126) 설치
# --------------------------------------------------
COPY requirements.txt /tmp/requirements.txt
RUN pip3 install --no-cache-dir -r /tmp/requirements.txt || true

# [핵심] PyTorch 12.6 설치
RUN pip3 install torch torchvision torchaudio \
    --index-url https://download.pytorch.org/whl/cu126

# --------------------------------------------------
# 7. Claude Code (선택 사항)
# --------------------------------------------------
RUN curl -fsSL https://deb.nodesource.com/setup_20.x | bash - && \
    apt-get install -y nodejs && \
    npm install -g @anthropic-ai/claude-code

# --------------------------------------------------
# 8. uXRCE-DDS-Agent 설치
# --------------------------------------------------
WORKDIR /opt
RUN git clone https://github.com/eProsima/Micro-XRCE-DDS-Agent.git && \
    cd Micro-XRCE-DDS-Agent && mkdir build && cd build && \
    cmake .. && make -j$(nproc) && make install && ldconfig

# --------------------------------------------------
# 9. PX4 Autopilot 소스 (v1.15.4)
# --------------------------------------------------
WORKDIR /opt
RUN git clone --recursive -b v1.15.4 https://github.com/PX4/PX4-Autopilot.git

# --------------------------------------------------
# 10. ⚡ ROS 2 <-> Gazebo Harmonic 브릿지 (🌟 캐시 마운트 적용)
# --------------------------------------------------
WORKDIR /root/ros2_ws/src
RUN git clone -b humble https://github.com/gazebosim/ros_gz.git

RUN apt-get update && apt-get install -y \
    ros-humble-actuator-msgs \
    ros-humble-vision-msgs \
    ros-humble-gps-msgs \
    && rm -rf /var/lib/apt/lists/*
    
WORKDIR /root/ros2_ws
# [핵심] build와 log 폴더를 캐시로 지정하여 이전 컴파일 기록을 보존! (install은 제외)
RUN --mount=type=cache,target=/root/ros2_ws/build \
    --mount=type=cache,target=/root/ros2_ws/log \
    /bin/bash -c "source /opt/ros/humble/setup.bash && \
    export GZ_VERSION=harmonic && \
    colcon build --packages-select ros_gz_bridge ros_gz_interfaces ros_gz_sim --symlink-install"

# --------------------------------------------------
# 11. px4_msgs 설치 및 빌드 (🌟 캐시 마운트 적용)
# --------------------------------------------------
WORKDIR /root/ros2_ws/src
RUN git clone https://github.com/PX4/px4_msgs.git
RUN rm -rf px4_msgs/msg/*.msg && \
    cp /opt/PX4-Autopilot/msg/*.msg px4_msgs/msg/
COPY config/custom_CMakeLists.txt px4_msgs/CMakeLists.txt

WORKDIR /root/ros2_ws
# [핵심] 여기서도 이전 빌드 기록을 활용하여 엄청나게 빠른 증분 빌드 수행
RUN --mount=type=cache,target=/root/ros2_ws/build \
    --mount=type=cache,target=/root/ros2_ws/log \
    /bin/bash -c "source /opt/ros/humble/setup.bash && \
    colcon build --packages-select px4_msgs --symlink-install"

# --------------------------------------------------
# 12. 환경 설정 및 Entrypoint
# --------------------------------------------------
SHELL ["/bin/bash", "-c"]

RUN echo "source /opt/ros/humble/setup.bash" >> ~/.bashrc && \
    echo "source /root/ros2_ws/install/setup.bash" >> ~/.bashrc && \
    echo "export GZ_SIM_RESOURCE_PATH=/opt/PX4-Autopilot/Tools/simulation/gz/models" >> ~/.bashrc && \
    echo "export ROS_DOMAIN_ID=0" >> ~/.bashrc

COPY entrypoint.sh /entrypoint.sh
RUN chmod +x /entrypoint.sh

WORKDIR /workspace
ENTRYPOINT ["/entrypoint.sh"]
CMD ["bash"]