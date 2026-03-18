#!/bin/bash
# start_infra_clean.sh - Kill all existing instances, clean state, start single fresh infra
#
# NOTE: Does NOT delete parameters.bson — the bson has calibration data (CAL_MAG0_ID etc.)
# that PX4 needs. Airframe 'param set' commands override specific params at runtime.
# Only delete bson manually when a complete param reset is needed.

LOG=/tmp/infra_clean_start.log
echo "[$(date)] Starting clean infra..." > "$LOG"

# 1. Kill everything by PID (not pattern to avoid self-kill)
# Note: PX4 binary has no trailing space after path — use 'bin/px4' not '/px4 '
PIDS=$(ps aux | grep -v grep | grep -E 'gz sim|bin/px4|MicroXRCE|parameter_bridge|infra.launch|episode.launch|train_sac|drone_controller|mission_manager|drop_calculator|drone_drop_rl' | awk '{print $2}' | tr '\n' ' ')
if [ -n "$PIDS" ]; then
    echo "Killing PIDs: $PIDS" | tee -a "$LOG"
    kill -9 $PIDS 2>/dev/null || true
    sleep 5
fi

# 2. Clean state — keep bson (calibration data), only clear stale shm and locks
rm -f /dev/shm/fastrtps_*
rm -f /tmp/px4_lock-0 /tmp/px4-sock-0 /tmp/train_managed.lock
echo "State cleaned (bson preserved)" | tee -a "$LOG"

# 3. Verify clean
REMAINING=$(ps aux | grep -v grep | grep -E 'gz sim|bin/px4|MicroXRCE|parameter_bridge|infra.launch' | wc -l)
if [ "$REMAINING" -gt 0 ]; then
    echo "ERROR: $REMAINING processes still running!" | tee -a "$LOG"
    exit 1
fi

# 4. Start single infra instance
echo "Starting fresh infra..." | tee -a "$LOG"
source /opt/ros/humble/setup.bash
source /root/ros2_ws/install/setup.bash
source /workspace/ros2_ws/install/setup.bash
cd /workspace/ros2_ws
XDG_RUNTIME_DIR=/tmp/runtime-root ros2 launch mission_manager infra.launch.py headless:=true enable_vision:=false > /tmp/infra_main.log 2>&1
