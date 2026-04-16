---
date: 2026-04-16
tags: [commands, docker, training, reference]
status: active
type: reference
---

# 자주 쓰는 명령어 모음

> CLAUDE.md에서 참조. 상세 bash 명령은 여기서 관리.

---

## Docker

```bash
# 이미지 pull
docker pull us-central1-docker.pkg.dev/charming-league-481306-d8/drone-bombard/drone-bombard:latest

# 컨테이너 최초 실행
xhost +local:docker
docker run -itd --gpus all --net=host --privileged --ipc=host \
  --name drone-bombard-harmonic \
  --env="DISPLAY=$DISPLAY" --env="QT_X11_NO_MITSHM=1" --env="NVIDIA_DRIVER_CAPABILITIES=all" \
  --volume="/tmp/.X11-unix:/tmp/.X11-unix:rw" \
  -v /opt/drone-bombard/Drone-Bombard-Simulation/ros2_ws:/workspace/ros2_ws \
  -v /opt/drone-bombard/Drone-Bombard-Simulation/gazebo_models:/workspace/gazebo_models \
  -v ~/.cache:/root/.cache \
  --log-driver=json-file --log-opt max-size=10m --log-opt max-file=3 \
  us-central1-docker.pkg.dev/charming-league-481306-d8/drone-bombard/drone-bombard:latest /bin/bash

# 기존 컨테이너 재접속
xhost +local:docker && docker start -ai drone-bombard-harmonic
```

---

## 빌드 (컨테이너 내부)

```bash
cd /workspace/ros2_ws && colcon build && source install/setup.bash
colcon build --packages-select <package_name> && source install/setup.bash
```

---

## RL 학습

> ⚠️ source 순서 필수: `/root/ros2_ws/install/setup.bash` → `/workspace/ros2_ws/install/setup.bash`

```bash
# Fresh start (보상 함수 변경 후)
docker exec -d drone-bombard-harmonic bash -c \
  "source /opt/ros/humble/setup.bash && \
   source /root/ros2_ws/install/setup.bash && \
   source /workspace/ros2_ws/install/setup.bash && \
   export GZ_SIM_RESOURCE_PATH=/workspace/gazebo_models:/opt/PX4-Autopilot/Tools/simulation/gz/models:/opt/PX4-Autopilot/Tools/simulation/gz/worlds && \
   cd /workspace/ros2_ws && ros2 run rl_navigation train_sac \
   > /tmp/production_train.log 2>&1"

# Resume from checkpoint
docker exec -d drone-bombard-harmonic bash -c \
  "source /opt/ros/humble/setup.bash && \
   source /root/ros2_ws/install/setup.bash && \
   source /workspace/ros2_ws/install/setup.bash && \
   export GZ_SIM_RESOURCE_PATH=/workspace/gazebo_models:/opt/PX4-Autopilot/Tools/simulation/gz/models:/opt/PX4-Autopilot/Tools/simulation/gz/worlds && \
   cd /workspace/ros2_ws && ros2 run rl_navigation train_sac \
     --resume /workspace/ros2_ws/rl_checkpoints/sac_drop_preempt.zip \
   > /tmp/production_train.log 2>&1"

# 모니터
docker exec drone-bombard-harmonic bash -c "tail -f /tmp/production_train.log"

# 정상 종료 (→ sac_drop_preempt.zip + _replay.pkl 저장)
docker exec drone-bombard-harmonic bash -c "pkill -SIGTERM -f train_sac"
```

---

## VM Preemption 후 빠른 재시작

```bash
xhost +local:docker && docker start -ai drone-bombard-harmonic
docker exec drone-bombard-harmonic bash /workspace/ros2_ws/start_infra_clean.sh
# 코드 변경 시 빌드 후 학습 재시작
```

---

## Git 동기화

```bash
git add .
git commit -m "Auto-sync: [작업 요약]"
git push origin main
```
