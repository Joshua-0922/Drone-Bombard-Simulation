FROM nvidia/cuda:12.6.2-cudnn-runtime-ubuntu22.04

ENV DEBIAN_FRONTEND=noninteractive

RUN apt-get update && apt-get install -y \
    wget curl git vim nano lsb-release \
    build-essential cmake \
    python3 python3-pip \
    locales software-properties-common \
    gpg \
    && apt-get clean && rm -rf /var/lib/apt/lists/*

RUN locale-gen en_US en_US.UTF-8
ENV LANG=en_US.UTF-8 LC_ALL=en_US.UTF-8

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
    && apt-get clean && rm -rf /var/lib/apt/lists/*

SHELL ["/bin/bash", "-c"]
RUN echo "source /opt/ros/humble/setup.bash" >> ~/.bashrc

RUN pip3 install --upgrade pip && \
    pip3 install --no-cache-dir \
    numpy opencv-python scipy matplotlib

RUN apt-get remove -y python3-sympy || true

RUN pip3 install --no-cache-dir \
    --index-url https://download.pytorch.org/whl/cu126 \
    torch==2.9.1 torchvision==0.24.1

WORKDIR /workspace
CMD ["bash"]
