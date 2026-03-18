#!/bin/bash
# start_infra_clean.sh - Kill all existing instances, clean state
#
# Infra is now self-managed by each DroneDropEnv instance. This script is
# a cleanup tool only — use it before starting training to ensure no stale
# processes remain from a previous session.
#
# NOTE: Does NOT delete parameters.bson — the bson has calibration data
# (CAL_MAG0_ID etc.) that PX4 needs. Only delete bson manually when a
# complete param reset is needed.

LOG=/tmp/infra_clean_start.log
echo "[$(date)] Cleaning all infra processes..." > "$LOG"

# 1. Kill everything by PID (not pattern to avoid self-kill)
PIDS=$(ps aux | grep -v grep | grep -E 'gz sim|bin/px4|MicroXRCE|parameter_bridge|infra.launch|episode.launch|train_sac|drone_controller|mission_manager|drop_calculator|drone_drop_rl' | awk '{print $2}' | tr '\n' ' ')
if [ -n "$PIDS" ]; then
    echo "Killing PIDs: $PIDS" | tee -a "$LOG"
    kill -9 $PIDS 2>/dev/null || true
    sleep 5
fi

# 2. Clean state — keep bson (calibration data), clear stale shm and locks
rm -f /dev/shm/fastrtps_*
# Clean lock/socket files for instances 0-3
for i in 0 1 2 3; do
    rm -f "/tmp/px4_lock-$i" "/tmp/px4-sock-$i"
done
rm -f /tmp/train_managed.lock
echo "State cleaned (bson preserved)" | tee -a "$LOG"

# 3. Verify clean
REMAINING=$(ps aux | grep -v grep | grep -E 'gz sim|bin/px4|MicroXRCE|parameter_bridge|infra.launch' | wc -l)
if [ "$REMAINING" -gt 0 ]; then
    echo "ERROR: $REMAINING processes still running!" | tee -a "$LOG"
    exit 1
fi

echo "All clean. Infra is now self-managed by DroneDropEnv._start_infra()." | tee -a "$LOG"
echo "Start training with: ros2 run rl_navigation train_sac" | tee -a "$LOG"
