#!/bin/bash
# stop.sh — Spot VM 수동 종료 스크립트
#
# ⚠️  중요: GCP 콘솔이나 gcloud compute instances stop 직접 사용 금지!
#    이 스크립트를 사용해야 watchdog이 재시작을 시도하지 않습니다.
#
# 사용법 (로컬 머신에서):
#   bash infra/stop.sh
#
# 또는 SSH로:
#   gcloud compute ssh g2-standard-16-nvidia-l4-dev \
#     --zone=us-central1-a --project=charming-league-481306-d8 \
#     -- 'bash /opt/drone-bombard/Drone-Bombard-Simulation/infra/stop.sh'

PROJECT="charming-league-481306-d8"
ZONE="us-central1-a"
INSTANCE="g2-standard-16-nvidia-l4-dev"

log() { echo "[$(date '+%Y-%m-%d %H:%M:%S')] $*"; }

# --- [1] 메타데이터 플래그 해제 (watchdog 재시작 방지) ---
log "메타데이터 drone_training_active=false 설정..."
gcloud compute instances add-metadata "$INSTANCE" \
  --zone="$ZONE" --project="$PROJECT" \
  --metadata drone_training_active=false

log "메타데이터 설정 완료. Watchdog은 이 VM을 재시작하지 않습니다."

# --- [2] 컨테이너 내 학습 프로세스에 SIGTERM (graceful checkpoint 저장) ---
log "학습 프로세스에 SIGTERM 전송 (체크포인트 저장 중)..."
gcloud compute ssh "$INSTANCE" --zone="$ZONE" --project="$PROJECT" \
  --command='
    PID_FILE=/opt/drone-bombard/Drone-Bombard-Simulation/ros2_ws/train_sac.pid
    # train_sac.pid는 컨테이너 volume mount 경로에 저장됨
    if docker exec drone-bombard-harmonic test -f /workspace/ros2_ws/train_sac.pid 2>/dev/null; then
      TRAIN_PID=$(docker exec drone-bombard-harmonic cat /workspace/ros2_ws/train_sac.pid 2>/dev/null)
      if [ -n "$TRAIN_PID" ]; then
        echo "학습 PID $TRAIN_PID 에 SIGTERM 전송..."
        docker exec drone-bombard-harmonic kill -SIGTERM "$TRAIN_PID" 2>/dev/null
        echo "체크포인트 저장 대기 중 (최대 15초)..."
        sleep 15
        echo "SIGTERM 완료"
      else
        echo "학습 PID 파일이 비어있음 (학습 미실행 중일 수 있음)"
      fi
    else
      echo "학습 PID 파일 없음 (학습 미실행 중)"
    fi
  ' 2>/dev/null || log "WARNING: SSH SIGTERM 실패 (학습 중이 아니거나 SSH 접근 불가)"

# --- [3] VM 종료 ---
log "VM 종료 중..."
gcloud compute instances stop "$INSTANCE" --zone="$ZONE" --project="$PROJECT"
log "VM 종료 완료."
