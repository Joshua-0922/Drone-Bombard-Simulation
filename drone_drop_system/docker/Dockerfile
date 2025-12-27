FROM nvidia/cuda:12.6.2-cudnn-runtime-ubuntu22.04

ENV DEBIAN_FRONTEND=noninteractive

# --------------------------------------------------
# 기본 툴
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
# ROS2 Humble
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
# Gazebo Classic (PX4 v1.15.4 호환)
# --------------------------------------------------
RUN apt-get update && apt-get install -y \
    gazebo \
    libgazebo11 \
    libgazebo-dev \
    && rm -rf /var/lib/apt/lists/*

# --------------------------------------------------
# PX4 의존성 (ubuntu.sh 대체)
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
# uXRCE-DDS-Agent
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
# PX4 Autopilot (clone only)
# --------------------------------------------------
WORKDIR /opt
RUN git clone --recursive -b v1.15.4 https://github.com/PX4/PX4-Autopilot.git

# --------------------------------------------------
# ROS 환경
# --------------------------------------------------
SHELL ["/bin/bash", "-c"]
RUN echo "source /opt/ros/humble/setup.bash" >> ~/.bashrc

WORKDIR /workspace
CMD ["bash"]
