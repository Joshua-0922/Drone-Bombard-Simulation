#!/bin/bash
# train_managed.sh — Safe training wrapper
#
# - Finds the latest checkpoint (periodic or preempt) before starting
# - Aborts if no checkpoint found
# - Monitors disk every 60s; sends SIGTERM to training if usage ≥ 90%
# - Logs to /tmp/train_managed.log

LOG=/tmp/train_managed.log
CKPT_DIR=/workspace/ros2_ws/rl_checkpoints
DISK_LIMIT=90
LOCK=/tmp/train_managed.lock

log() { echo "[$(date '+%Y-%m-%d %H:%M:%S')] $*" | tee -a "$LOG"; }

# --- [0] Singleton lock — abort if already running ---
exec 9>"$LOCK"
if ! flock -n 9; then
    log "ERROR: Another train_managed.sh is already running (lock held). Aborting."
    exit 1
fi

# --- [1] Find latest valid checkpoint (skip corrupt files) ---
LATEST_CKPT=""
for f in $(ls -t "$CKPT_DIR"/*.zip 2>/dev/null); do
    if python3 -c "import sys; from stable_baselines3 import SAC; SAC.load(sys.argv[1], device='cpu')" "$f" 2>/dev/null; then
        LATEST_CKPT="$f"
        break
    else
        log "WARNING: Checkpoint $f failed zip test — skipping"
    fi
done
if [ -z "$LATEST_CKPT" ]; then
    log "ERROR: No valid checkpoint found in $CKPT_DIR — aborting."
    exit 1
fi
log "Latest valid checkpoint: $LATEST_CKPT"

# --- [2] Source ROS ---
source /opt/ros/humble/setup.bash
source /root/ros2_ws/install/setup.bash
source /workspace/ros2_ws/install/setup.bash

# --- [3] Start training in background ---
log "Starting training from: $LATEST_CKPT"
cd /workspace/ros2_ws
python3 -m rl_navigation.train_sac --resume "$LATEST_CKPT" >> "$LOG" 2>&1 &
TRAIN_PID=$!
log "Training PID: $TRAIN_PID"
echo "$TRAIN_PID" > /workspace/ros2_ws/train_sac.pid

# --- [4] Disk monitor loop ---
while kill -0 "$TRAIN_PID" 2>/dev/null; do
    DISK_PCT=$(df / | awk 'NR==2 {gsub(/%/, "", $5); print $5}')
    if [ "$DISK_PCT" -ge "$DISK_LIMIT" ]; then
        log "DISK FULL: ${DISK_PCT}% >= ${DISK_LIMIT}% — sending SIGTERM to training (PID $TRAIN_PID)"
        kill -SIGTERM "$TRAIN_PID" 2>/dev/null
        sleep 10
        # If still alive, force kill
        if kill -0 "$TRAIN_PID" 2>/dev/null; then
            log "Training still alive — sending SIGKILL"
            kill -9 "$TRAIN_PID" 2>/dev/null
        fi
        log "Training stopped due to disk limit."
        exit 2
    fi
    log "Disk: ${DISK_PCT}%  Training PID $TRAIN_PID alive"
    sleep 60
done

wait "$TRAIN_PID"
EXIT_CODE=$?
log "Training exited with code $EXIT_CODE"
rm -f /workspace/ros2_ws/train_sac.pid
exit "$EXIT_CODE"
