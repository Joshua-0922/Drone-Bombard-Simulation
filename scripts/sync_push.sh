#!/bin/bash
# sync_push.sh — 파라미터 수정 후: install 미러링 + commit + push
#
# 사용법: bash scripts/sync_push.sh "커밋 메시지"
# 예시:   bash scripts/sync_push.sh "Round 3: auto_drop threshold 2m"

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

MSG="${1:-parameter update}"

echo "=== sync_push ==="

# 1. Install 미러링
echo "[1/3] Install 미러링..."
if docker ps --format '{{.Names}}' | grep -q "^${CONTAINER}$"; then
    docker cp "$SRC_YAML"  "$CONTAINER:$DST_YAML"
    docker cp "$SRC_ENV"   "$CONTAINER:$DST_ENV"
    docker cp "$SRC_TRAIN" "$CONTAINER:$DST_TRAIN"
    echo "  미러링 완료"
else
    echo "  컨테이너 미실행 — 미러링 스킵 (다음 시작 시 sync_pull.sh 실행)"
fi

# 2. Git commit
echo "[2/3] Git commit..."
cd "$REPO"
git add ros2_ws/src/rl_navigation/config/hyperparams.yaml \
        ros2_ws/src/rl_navigation/rl_navigation/drone_drop_env.py \
        ros2_ws/src/rl_navigation/rl_navigation/train_sac.py \
        local/ \
        2>/dev/null || true
if git diff --cached --quiet; then
    echo "  변경사항 없음 — commit 스킵"
else
    git commit -m "$MSG"
    echo "  커밋 완료"
fi

# 3. Git push
echo "[3/3] Git push origin $BRANCH..."
git push origin "$BRANCH"
echo "  push 완료"

echo "=== 완료. 다른 환경에서 sync_pull.sh 실행하세요 ==="
