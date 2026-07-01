#!/bin/bash
# FULL Fresh-Start training with B+C reward changes + LPF (velocity_lpf_alpha=0.4).
# Fresh Start (no --resume) DELETES sac_drop_*_steps.zip in the real ckpt dir —
# v14 is backed up under rl_checkpoints/v14_backup/. Uses config total_timesteps
# (300k) and wandb online (as configured). Designed to run detached in tmux.
source /opt/ros/humble/setup.bash
source /root/ros2_ws/install/setup.bash
source /workspace/ros2_ws/install/setup.bash
export GZ_SIM_RESOURCE_PATH=/workspace/gazebo_models:/opt/PX4-Autopilot/Tools/simulation/gz/models:/opt/PX4-Autopilot/Tools/simulation/gz/worlds
cd /workspace/ros2_ws
bash start_infra_clean.sh
ros2 run rl_navigation train_sac \
  --config src/rl_navigation/config/hyperparams_v13.yaml \
  --run-name v15_bc_stable 2>&1 | tee /tmp/train_bc.log
echo "TRAIN_BC_EXIT=$?"
