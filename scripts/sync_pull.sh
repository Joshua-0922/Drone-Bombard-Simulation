#!/bin/bash
# sync_pull.sh — 다른 환경의 변경사항 가져오기: pull + install 미러링
#
# 사용법: bash scripts/sync_pull.sh

set -euo pipefail

REPO=$(git -C "$(dirname "$0")/.." rev-parse --show-toplevel)
CONTAINER=drone-bombard-harmonic
BRANCH=junsang

SRC_YAML="$REPO/ros2_ws/src/rl_navigation/config/hyperparams.yaml"
SRC_ENV="$REPO/ros2_ws/src/rl_navigation/rl_navigation/drone_drop_env.py"
SRC_TRAIN="$REPO/ros2_ws/src/rl_navigation/rl_navigation/train_sac.py"

DST_YAML="/workspace/ros2_ws/install/rl_navigation/share/rl_navigation/config/hyperparams.yaml"
DST_ENV="/workspace/ros2_ws/install/rl_navigation/lib/python3.10/site-packages/rl_navigation/drone_drop_env.py"
DST_TRAIN="/workspace/ros2_ws/install/rl_navigation/lib/python3.10/site-packages/rl_navigation/train_sac.py"

echo "=== sync_pull ==="

# 1. Git pull
echo "[1/2] Git pull origin $BRANCH..."
cd "$REPO"
git pull origin "$BRANCH"
echo "  pull 완료"

# 2. Install 미러링
echo "[2/2] Install 미러링..."
if docker ps --format '{{.Names}}' | grep -q "^${CONTAINER}$"; then
    docker cp "$SRC_YAML"  "$CONTAINER:$DST_YAML"
    docker cp "$SRC_ENV"   "$CONTAINER:$DST_ENV"
    docker cp "$SRC_TRAIN" "$CONTAINER:$DST_TRAIN"
    echo "  미러링 완료"
else
    echo "  컨테이너 미실행 — 시작 후 다시 실행하세요"
    echo "  docker start $CONTAINER && bash scripts/sync_pull.sh"
fi

echo "=== 완료. 학습 시작 가능 ==="
