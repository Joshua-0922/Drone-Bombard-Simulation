#!/bin/bash
# B+C reward dry-run (1200 steps, temp ckpt dir, offline wandb).
# Validates the velocity-damping (B) + smoothness-weight (C) reward changes
# without touching the real v14 checkpoints. Safe to run in tmux.
source /opt/ros/humble/setup.bash
source /root/ros2_ws/install/setup.bash
source /workspace/ros2_ws/install/setup.bash
export GZ_SIM_RESOURCE_PATH=/workspace/gazebo_models:/opt/PX4-Autopilot/Tools/simulation/gz/models:/opt/PX4-Autopilot/Tools/simulation/gz/worlds
export WANDB_MODE=offline
cd /workspace/ros2_ws
bash start_infra_clean.sh
ros2 run rl_navigation train_sac \
  --timesteps 1200 \
  --checkpoint-dir /tmp/rl_dryrun_bc \
  --run-name dryrun_bc 2>&1 | tee /tmp/dryrun_bc.log
echo "DRYRUN_BC_EXIT=$?"
