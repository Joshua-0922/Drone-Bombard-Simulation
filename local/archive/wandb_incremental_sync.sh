#!/usr/bin/env bash
#
# WandB Incremental Sync
# ─────────────────────────────────────────────────────────────────────────────
# 학습은 WANDB_MODE=offline 으로 돌고, 이 스크립트가 별도 프로세스로
# 주기적으로 클라우드 sync 한다.
#
# 장점:
#   - 학습 프로세스가 WandB IPC 데드락에 빠지지 않음 (이전 hang 사고 방지)
#   - 네트워크가 끊겨도 학습은 영향 없음
#   - sync 자체가 실패해도 무시하고 다음 사이클 시도
#   - "약 10분 지연 모니터링" 수준은 가능
#
# 사용법:
#   # 백그라운드 실행
#   nohup /home/juns/wandb_incremental_sync.sh > /tmp/wandb_sync.log 2>&1 &
#
#   # 또는 tmux 세션의 별도 페인에서
#   /home/juns/wandb_incremental_sync.sh
#
#   # 중단
#   pkill -f wandb_incremental_sync.sh
#
# 환경변수:
#   SYNC_INTERVAL_SECS  : 사이클 사이 대기 (default 600 = 10분)
#   CONTAINER           : 도커 컨테이너 이름 (default drone-bombard-harmonic)
#   WANDB_DIR           : 컨테이너 안의 wandb 디렉토리
# ─────────────────────────────────────────────────────────────────────────────

set -u   # undef var 에러
SYNC_INTERVAL_SECS="${SYNC_INTERVAL_SECS:-600}"
CONTAINER="${CONTAINER:-drone-bombard-harmonic}"
WANDB_DIR="${WANDB_DIR:-/workspace/ros2_ws/wandb}"

echo "[wandb-sync] start  interval=${SYNC_INTERVAL_SECS}s  container=${CONTAINER}  dir=${WANDB_DIR}"
echo "[wandb-sync] pid=$$  $(date -Iseconds)"

trap 'echo "[wandb-sync] stop  $(date -Iseconds)"; exit 0' INT TERM

while true; do
  START=$(date +%s)
  TS=$(date -Iseconds)

  # 컨테이너 살아있는지 먼저 (없으면 그냥 대기, 죽지 않음)
  if ! docker inspect "$CONTAINER" --format='{{.State.Running}}' 2>/dev/null | grep -q true; then
    echo "[wandb-sync] $TS  container not running — skip"
  else
    # offline-run-* 폴더 찾기
    RUNS=$(docker exec "$CONTAINER" bash -c "ls -d ${WANDB_DIR}/offline-run-* 2>/dev/null" || true)
    if [ -z "$RUNS" ]; then
      echo "[wandb-sync] $TS  no offline-run-* found — skip"
    else
      N=$(echo "$RUNS" | wc -l)
      echo "[wandb-sync] $TS  syncing ${N} run(s)…"

      # --include-offline 와 --sync-all 둘 다 의도는 같으나 wandb 버전 차이로
      # 안전하게 폴더별로 sync. 실패하면 다음 폴더 계속.
      while IFS= read -r RUN_DIR; do
        [ -z "$RUN_DIR" ] && continue
        RUN_NAME=$(basename "$RUN_DIR")
        if docker exec "$CONTAINER" bash -c \
             "cd /workspace/ros2_ws && timeout 300 wandb sync '$RUN_DIR' 2>&1" \
             | tail -3; then
          echo "[wandb-sync]   ✓ $RUN_NAME"
        else
          echo "[wandb-sync]   ✗ $RUN_NAME (실패 — 다음 사이클에 재시도)"
        fi
      done <<< "$RUNS"
    fi
  fi

  DUR=$(( $(date +%s) - START ))
  echo "[wandb-sync] $TS  cycle done in ${DUR}s, sleep ${SYNC_INTERVAL_SECS}s"
  sleep "$SYNC_INTERVAL_SECS"
done
