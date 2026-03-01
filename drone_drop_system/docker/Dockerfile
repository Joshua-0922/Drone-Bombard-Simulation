# [cite_start]1. NVIDIA CUDA 12.6.2 베이스 (GPU 가속용) [cite: 1]
FROM nvidia/cuda:12.6.2-cudnn-runtime-ubuntu22.04

[cite_start]ENV DEBIAN_FRONTEND=noninteractive [cite: 1]
[cite_start]ENV LANG=en_US.UTF-8 LC_ALL=en_US.UTF-8 [cite: 1]

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
    [cite_start]&& locale-gen en_US.UTF-8 [cite: 1]

# --------------------------------------------------
# 3. ROS 2 Humble 설치
# --------------------------------------------------
RUN add-apt-repository universe && \
    curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key \
    | gpg --dearmor -o /usr/share/keyrings/ros-archive-keyring.gpg && \
    echo "deb [signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] \
    http://packages.ros.org/ros2/ubuntu $(lsb_release -cs) main" \
    > [cite_start]/etc/apt/sources.list.d/ros2.list [cite: 1, 2]

RUN apt-get update && apt-get install -y \
    ros-humble-desktop \
    ros-dev-tools \
    python3-colcon-common-extensions \
    [cite_start]&& rm -rf /var/lib/apt/lists/* [cite: 2]

# [cite_start][중요] 시스템에 깔린 기본 px4_msgs는 충돌 원인이므로 삭제 [cite: 2]
[cite_start]RUN apt-get remove -y ros-humble-px4-msgs || true [cite: 2, 3]

# --------------------------------------------------
# 4. 🚀 Gazebo Harmonic 설치
# --------------------------------------------------
RUN wget https://packages.osrfoundation.org/gazebo.gpg -O /usr/share/keyrings/pkgs-osrf-archive-keyring.gpg && \
    echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/pkgs-osrf-archive-keyring.gpg] \
    http://packages.osrfoundation.org/gazebo/ubuntu-stable $(lsb_release -cs) main" \
    > [cite_start]/etc/apt/sources.list.d/gazebo-stable.list [cite: 3]

RUN apt-get update && apt-get install -y \
    gz-harmonic \
    [cite_start]&& rm -rf /var/lib/apt/lists/* [cite: 3]

# --------------------------------------------------
# 5. PX4 의존성 (🌟 gflags 에러 해결)
# --------------------------------------------------
RUN apt-get update && apt-get install -y \
    protobuf-compiler libeigen3-dev libxml2-dev libopencv-dev \
    libcurl4-openssl-dev libtinyxml2-dev \
    libgstreamer1.0-dev libgstreamer-plugins-base1.0-dev \
    libgflags-dev libgoogle-glog-dev \
    [cite_start]&& rm -rf /var/lib/apt/lists/* [cite: 3, 4]

# --------------------------------------------------
# 6. Python 패키지 & 🧠 PyTorch(cu126) 설치
# --------------------------------------------------
[cite_start]COPY requirements.txt /tmp/requirements.txt [cite: 4]
[cite_start]RUN pip3 install --no-cache-dir -r /tmp/requirements.txt || true [cite: 4, 5]

# [cite_start][핵심] PyTorch 12.6 설치 [cite: 5]
RUN pip3 install torch torchvision torchaudio \
    [cite_start]--index-url https://download.pytorch.org/whl/cu126 [cite: 5]

# --------------------------------------------------
# 7. Claude Code (선택 사항)
# --------------------------------------------------
RUN curl -fsSL https://deb.nodesource.com/setup_20.x | bash - && \
    apt-get install -y nodejs && \
    [cite_start]npm install -g @anthropic-ai/claude-code [cite: 5, 6]

# --------------------------------------------------
# 8. uXRCE-DDS-Agent 설치
# --------------------------------------------------
[cite_start]WORKDIR /opt [cite: 6]
RUN git clone https://github.com/eProsima/Micro-XRCE-DDS-Agent.git && \
    cd Micro-XRCE-DDS-Agent && mkdir build && cd build && \
    [cite_start]cmake .. && make -j$(nproc) && make install && ldconfig [cite: 6]

# --------------------------------------------------
# 9. PX4 Autopilot 소스 (v1.15.4)
# --------------------------------------------------
[cite_start]WORKDIR /opt [cite: 6]
[cite_start]RUN git clone --recursive -b v1.15.4 https://github.com/PX4/PX4-Autopilot.git [cite: 6]

# --------------------------------------------------
# 10. ⚡ ROS 2 <-> Gazebo Harmonic 브릿지 (🌟 캐시 마운트 적용)
# --------------------------------------------------
[cite_start]WORKDIR /root/ros2_ws/src [cite: 6]
[cite_start]RUN git clone -b humble https://github.com/gazebosim/ros_gz.git [cite: 6]

RUN apt-get update && apt-get install -y \
    ros-humble-actuator-msgs \
    ros-humble-vision-msgs \
    ros-humble-gps-msgs \
    [cite_start]&& rm -rf /var/lib/apt/lists/* [cite: 6, 7]
    
[cite_start]WORKDIR /root/ros2_ws [cite: 7]
# 🔥 [수정됨] --symlink-install 옵션 삭제
# [핵심] build와 log 폴더를 캐시로 지정하여 이전 컴파일 기록을 보존! (install은 제외) [cite_start][cite: 7, 8]
RUN --mount=type=cache,target=/root/ros2_ws/build \
    --mount=type=cache,target=/root/ros2_ws/log \
    /bin/bash -c "source /opt/ros/humble/setup.bash && \
    export GZ_VERSION=harmonic && \
    [cite_start]colcon build --packages-select ros_gz_bridge ros_gz_interfaces ros_gz_sim" [cite: 8]

# --------------------------------------------------
# 11. px4_msgs 설치 및 빌드 (🌟 캐시 마운트 적용)
# --------------------------------------------------
[cite_start]WORKDIR /root/ros2_ws/src [cite: 8]
[cite_start]RUN git clone https://github.com/PX4/px4_msgs.git [cite: 8]
RUN rm -rf px4_msgs/msg/*.msg && \
    [cite_start]cp /opt/PX4-Autopilot/msg/*.msg px4_msgs/msg/ [cite: 8]
[cite_start]COPY config/custom_CMakeLists.txt px4_msgs/CMakeLists.txt [cite: 8]

[cite_start]WORKDIR /root/ros2_ws [cite: 8]
# 🔥 [수정됨] --symlink-install 옵션 삭제
# [cite_start][핵심] 여기서도 이전 빌드 기록을 활용하여 엄청나게 빠른 증분 빌드 수행 [cite: 8]
RUN --mount=type=cache,target=/root/ros2_ws/build \
    --mount=type=cache,target=/root/ros2_ws/log \
    /bin/bash -c "source /opt/ros/humble/setup.bash && \
    [cite_start]colcon build --packages-select px4_msgs" [cite: 8]

# --------------------------------------------------
# [cite_start]12. 환경 설정 및 Entrypoint [cite: 8, 9]
# --------------------------------------------------
[cite_start]SHELL ["/bin/bash", "-c"] [cite: 9]

RUN echo "source /opt/ros/humble/setup.bash" >> ~/.bashrc && \
    echo "source /root/ros2_ws/install/setup.bash" >> ~/.bashrc && \
    echo "export GZ_SIM_RESOURCE_PATH=/opt/PX4-Autopilot/Tools/simulation/gz/models" >> ~/.bashrc && \
    [cite_start]echo "export ROS_DOMAIN_ID=0" >> ~/.bashrc [cite: 9]

[cite_start]COPY entrypoint.sh /entrypoint.sh [cite: 9]
[cite_start]RUN chmod +x /entrypoint.sh [cite: 9]

[cite_start]WORKDIR /workspace [cite: 9]
[cite_start]ENTRYPOINT ["/entrypoint.sh"] [cite: 9]
[cite_start]CMD ["bash"] [cite: 9]