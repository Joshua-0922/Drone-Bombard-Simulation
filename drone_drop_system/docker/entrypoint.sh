#!/bin/bash
set -e

echo "[ENTRYPOINT] Starting container..."

# 1. ROS2 기본 환경
if [ -f /opt/ros/humble/setup.bash ]; then
  source /opt/ros/humble/setup.bash
  echo "[ENTRYPOINT] ROS2 Humble sourced"
else
  echo "[ENTRYPOINT] ROS2 not found"
fi

# 2. ROS2 workspace (있을 경우)
if [ -d /workspace/ros2_ws ]; then
  cd /workspace/ros2_ws

  if [ -d install ]; then
    source install/setup.bash
    echo "[ENTRYPOINT] ros2_ws install sourced"
  elif [ -d src ]; then
    echo "[ENTRYPOINT] Building ros2_ws..."
    colcon build --symlink-install
    source install/setup.bash
  fi
fi

# 3. NVIDIA 확인 (디버그용)
if command -v nvidia-smi &> /dev/null; then
  echo "[ENTRYPOINT] NVIDIA GPU detected"
  nvidia-smi || true
else
  echo "[ENTRYPOINT] NVIDIA not available"
fi

echo "[ENTRYPOINT] Environment ready"

# 4. CMD 실행 (중요)
exec "$@"
