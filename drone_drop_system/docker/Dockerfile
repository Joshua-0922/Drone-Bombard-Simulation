FROM nvidia/cuda:12.6.2-cudnn-runtime-ubuntu22.04

ENV DEBIAN_FRONTEND=noninteractive
ENV LANG=en_US.UTF-8
ENV LC_ALL=en_US.UTF-8

# --------------------------------------------------
# 1. 기본 시스템 패키지
# --------------------------------------------------
RUN apt-get update && apt-get install -y \
    wget curl git vim nano \
    build-essential cmake \
    python3 python3-pip \
    locales lsb-release software-properties-common \
    gpg ca-certificates \
    sudo \
    && locale-gen en_US.UTF-8 \
    && rm -rf /var/lib/apt/lists/*

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
# 3. Gazebo Classic 11 (PX4 v1.15.x 호환)
# --------------------------------------------------
RUN apt-get update && \
    apt-get remove -y \
        gz-harmonic \
        gz-garden \
        gz-fortress || true && \
    apt-get install -y aptitude && \
    aptitude install -y \
        gazebo \
        libgazebo11 \
        libgazebo-dev && \
    rm -rf /var/lib/apt/lists/*

# OpenGL / X11 (Gazebo 필수)
RUN apt-get update && apt-get install -y \
    mesa-utils \
    libgl1 \
    libgl1-mesa-dri \
    libx11-6 \
    libxext6 \
    libxrender1 \
    libxtst6 \
    && rm -rf /var/lib/apt/lists/*

# --------------------------------------------------
# 4. Python 패키지
# --------------------------------------------------
RUN pip3 install --upgrade pip && \
    pip3 install --no-cache-dir \
        numpy \
        scipy \
        matplotlib \
        opencv-python

# --------------------------------------------------
# 5. Micro XRCE-DDS Agent
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
# 6. PX4-Autopilot v1.15.4 (gazebo-classic SITL)
# --------------------------------------------------
WORKDIR /opt

RUN git clone --recursive \
    --branch v1.15.4 \
    https://github.com/PX4/PX4-Autopilot.git

# PX4 의존성 설치
RUN bash /opt/PX4-Autopilot/Tools/setup/ubuntu.sh

# --------------------------------------------------
# 7. ROS2 환경 자동 로딩
# --------------------------------------------------
SHELL ["/bin/bash", "-c"]
RUN echo "source /opt/ros/humble/setup.bash" >> /root/.bashrc

# --------------------------------------------------
# 8. Entrypoint
# --------------------------------------------------
COPY docker/entrypoint.sh /entrypoint.sh
RUN chmod +x /entrypoint.sh

WORKDIR /workspace

ENTRYPOINT ["/entrypoint.sh"]
CMD ["bash"]
