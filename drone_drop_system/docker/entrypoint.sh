#!/bin/bash
set -e

echo "[ENTRYPOINT] Starting container setup..."

# ---------------------------------------------
# 1. 환경 설정 로드 (순서 중요)
# ---------------------------------------------
# (1) 시스템 ROS 2
if [ -f /opt/ros/humble/setup.bash ]; then
  source /opt/ros/humble/setup.bash
fi

# (2) 이미지에 미리 빌드된 px4_msgs (root 워크스페이스)
#     이게 있어야 드론과 통신이 됩니다.
if [ -f /root/ros2_ws/install/setup.bash ]; then
  source /root/ros2_ws/install/setup.bash
  echo "[ENTRYPOINT] Loaded pre-built px4_msgs"
fi

# (3) 사용자 워크스페이스 (마운트된 경우)
#     /workspace/ros2_ws 에 코드를 마운트했다면 여기서 처리
if [ -d "/workspace/ros2_ws" ]; then
  echo "[ENTRYPOINT] Found user workspace at /workspace/ros2_ws"
  cd /workspace/ros2_ws
  
  # 빌드된 게 없으면 빌드 시도 (선택 사항)
  if [ ! -d "install" ] && [ -d "src" ]; then
    echo "[ENTRYPOINT] Building user workspace..."
    colcon build --packages-ignore gazebo_ros_link_attacher
  fi
  
  if [ -f "install/setup.bash" ]; then
    source install/setup.bash
    echo "[ENTRYPOINT] Loaded user workspace"
  fi
fi

# ---------------------------------------------
# 2. [핵심] ROS 2 데몬 리셋
# ---------------------------------------------
# 이 과정이 없으면 VM 재부팅 시 87바이트 구버전 캐시가 남을 수 있음
echo "[ENTRYPOINT] Resetting ROS 2 Daemon to prevent DDS mismatch..."
ros2 daemon stop > /dev/null 2>&1 || true
ros2 daemon start > /dev/null 2>&1 || true

# ---------------------------------------------
# 3. 디버깅 정보 (GPU 확인)
# ---------------------------------------------
if command -v nvidia-smi &> /dev/null; then
  echo "[ENTRYPOINT] NVIDIA GPU detected:"
  nvidia-smi -L || true
else
  echo "[ENTRYPOINT] NVIDIA GPU not available (Running on CPU mode)"
fi

echo "[ENTRYPOINT] ✅ Environment Ready! Executing command..."

# 4. CMD 실행
exec "$@"