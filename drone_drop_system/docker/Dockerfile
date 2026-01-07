FROM nvidia/cuda:12.6.2-cudnn-runtime-ubuntu22.04

ENV DEBIAN_FRONTEND=noninteractive

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
    && rm -rf /var/lib/apt/lists/*

RUN locale-gen en_US.UTF-8
ENV LANG=en_US.UTF-8 LC_ALL=en_US.UTF-8

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

# --------------------------------------------------
# 3. Gazebo Classic (PX4 v1.15.4 호환)
# --------------------------------------------------
RUN apt-get update && apt-get install -y \
    gazebo \
    libgazebo11 \
    libgazebo-dev \
    && rm -rf /var/lib/apt/lists/*

# --------------------------------------------------
# 4. PX4 의존성 (ubuntu.sh 대체)
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
RUN pip3 install --no-cache-dir \
    kconfiglib \
    symforce \
    pyros-genmsg \
    jsonschema \
    future \
    pyserial

# --------------------------------------------------
# 6. Claude Code 설치
# --------------------------------------------------
RUN curl -fsSL https://deb.nodesource.com/setup_20.x | bash - && \
    apt-get install -y nodejs

RUN npm install -g @anthropic-ai/claude-code

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
# 8. PX4 Autopilot 소스 (펌웨어)
# --------------------------------------------------
WORKDIR /opt
RUN git clone --recursive -b v1.15.4 https://github.com/PX4/PX4-Autopilot.git

# ==================================================
# [추가됨] 9. px4_msgs 설치 및 빌드 (핵심 수정 사항)
# ==================================================
# 설명: 사용자가 마운트할 /workspace와 겹치지 않게
# 시스템 기본 워크스페이스(/root/ros2_ws)에 설치합니다.
WORKDIR /root/ros2_ws/src
RUN git clone https://github.com/PX4/px4_msgs.git

WORKDIR /root/ros2_ws
# ROS2 환경을 로드한 상태에서 colcon build 수행
RUN /bin/bash -c "source /opt/ros/humble/setup.bash && \
    colcon build"

# --------------------------------------------------
# 10. 환경 설정 (.bashrc)
# --------------------------------------------------
SHELL ["/bin/bash", "-c"]

# ROS2 기본 설정 + 미리 빌드한 px4_msgs 설정을 모두 source 하도록 추가
RUN echo "source /opt/ros/humble/setup.bash" >> ~/.bashrc && \
    echo "source /root/ros2_ws/install/setup.bash" >> ~/.bashrc

WORKDIR /workspace
CMD ["bash"]