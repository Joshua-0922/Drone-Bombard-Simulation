#!/bin/bash
# Supervisor for v15_bc_stable: the training periodically dies with the infra
# reset-recursion abort (spin thread / infra unrecoverable). This loop resumes
# from the LATEST checkpoint after each abort so the run reaches 300k unattended.
#
# --resume preserves checkpoints (no fresh-start deletion) and continues the same
# wandb run (resume='allow'). Periodic checkpoints save weights only (no replay
# buffer), so each resume restarts SAC's buffer but keeps the learned policy.
source /opt/ros/humble/setup.bash
source /root/ros2_ws/install/setup.bash
source /workspace/ros2_ws/install/setup.bash
export GZ_SIM_RESOURCE_PATH=/workspace/gazebo_models:/opt/PX4-Autopilot/Tools/simulation/gz/models:/opt/PX4-Autopilot/Tools/simulation/gz/worlds
cd /workspace/ros2_ws
CKPT_DIR=/workspace/ros2_ws/rl_checkpoints
CFG=src/rl_navigation/config/hyperparams_v13.yaml

for i in $(seq 1 60); do
  bash start_infra_clean.sh
  LATEST=$(ls -t "$CKPT_DIR"/sac_drop_*_steps.zip 2>/dev/null | head -1)
  if [ -n "$LATEST" ]; then
    RESUME_ARG="--resume $LATEST"
    echo "[SUPERVISOR] attempt $i — resuming from $LATEST"
  else
    RESUME_ARG=""
    echo "[SUPERVISOR] attempt $i — no checkpoint, fresh start"
  fi
  ros2 run rl_navigation train_sac --config "$CFG" --run-name v15_bc_stable $RESUME_ARG 2>&1 | tee -a /tmp/train_bc.log
  if [ -f "$CKPT_DIR/sac_drop_final.zip" ]; then
    echo "[SUPERVISOR] final model present — training complete after attempt $i."
    break
  fi
  echo "[SUPERVISOR] train_sac exited (attempt $i) without final model; restarting in 15s..."
  sleep 15
done
echo "[SUPERVISOR] loop ended."
