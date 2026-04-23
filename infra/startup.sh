#!/bin/bash
# startup.sh — GCP VM metadata startup-script
#
# 등록 방법:
#   gcloud compute instances add-metadata g2-standard-16-nvidia-l4-dev \
#     --zone=us-central1-a --project=charming-league-481306-d8 \
#     --metadata-from-file startup-script=infra/startup.sh
#
# 이 스크립트는 VM 부팅 시마다 root로 자동 실행됩니다.

set -euo pipefail
LOG=/var/log/drone-bombard-startup.log
exec > >(tee -a "$LOG") 2>&1

log() { echo "[$(date '+%Y-%m-%d %H:%M:%S')] $*"; }

# --- GCP 메타데이터에서 자신의 정보 읽기 ---
INSTANCE=$(curl -sf -H "Metadata-Flavor: Google" \
  "http://metadata.google.internal/computeMetadata/v1/instance/name")
ZONE=$(curl -sf -H "Metadata-Flavor: Google" \
  "http://metadata.google.internal/computeMetadata/v1/instance/zone" | cut -d/ -f4)
PROJECT=$(curl -sf -H "Metadata-Flavor: Google" \
  "http://metadata.google.internal/computeMetadata/v1/project/project-id")

log "=== Drone-Bombard Startup ==="
log "Instance: $INSTANCE  Zone: $ZONE  Project: $PROJECT"

# --- [0] watchdog 재시작 카운터 리셋 ---
# (정상 부팅 = 이전 재시작 시도 성공)
gcloud compute instances add-metadata "$INSTANCE" \
  --zone="$ZONE" --project="$PROJECT" \
  --metadata watchdog_restart_count=0 \
  --quiet 2>/dev/null || log "WARNING: metadata reset 실패 (권한 확인 필요)"

# --- [1] Docker 데몬 준비 대기 ---
log "Docker daemon 대기 중..."
for i in $(seq 1 60); do
  if docker info &>/dev/null; then
    log "Docker 준비 완료 (${i}초)"
    break
  fi
  sleep 2
  if [ "$i" -eq 60 ]; then
    log "ERROR: Docker 120초 내 준비 안 됨. 종료."
    exit 1
  fi
done

# --- [2] VNC (port 5901) 준비 대기 ---
log "VNC port 5901 대기 중..."
for i in $(seq 1 30); do
  if ss -tlnp 2>/dev/null | grep -q ':5901'; then
    log "VNC 준비 완료 (${i}초)"
    break
  fi
  sleep 2
  if [ "$i" -eq 30 ]; then
    log "WARNING: VNC 60초 내 준비 안 됨. 계속 진행 (headless 모드 가능)."
    break
  fi
done

# --- [3] X11 권한 설정 ---
sudo -u nachoigpt DISPLAY=:1 xhost +local:docker 2>/dev/null \
  && log "xhost 설정 완료" \
  || log "WARNING: xhost 설정 실패 (VNC 미기동 가능, 계속 진행)"

# --- [4] 컨테이너 시작 ---
log "drone-bombard-harmonic 컨테이너 시작..."
if docker ps -a --format '{{.Names}}' | grep -q '^drone-bombard-harmonic$'; then
  docker start drone-bombard-harmonic
  log "컨테이너 시작됨"
else
  log "ERROR: drone-bombard-harmonic 컨테이너가 존재하지 않음."
  log "  최초 VM 설정 시 notes/Environment/README.md Step 7 참조"
  exit 1
fi

# 컨테이너 안정화 대기
sleep 5

# --- [4-1] WandB API 키 설정 ---
WANDB_ENV=/opt/drone-bombard/.wandb.env
if [ -f "$WANDB_ENV" ]; then
  WANDB_KEY=$(grep WANDB_API_KEY "$WANDB_ENV" | cut -d= -f2)
  if [ -n "$WANDB_KEY" ]; then
    docker exec drone-bombard-harmonic bash -c "wandb login $WANDB_KEY" &>/dev/null \
      && log "WandB 로그인 완료" \
      || log "WARNING: WandB 로그인 실패"
  fi
else
  log "WARNING: $WANDB_ENV 없음 — WandB 인증 스킵"
fi

# --- [5] GPU 접근 확인 ---
if docker exec drone-bombard-harmonic nvidia-smi -L &>/dev/null; then
  log "GPU 확인 완료"
else
  log "WARNING: GPU 접근 실패. 학습이 CPU로 시작될 수 있음."
fi

# --- [6] 학습 시작 (train_managed.sh 위임) ---
# train_managed.sh가 다음을 자동 처리:
#   - 최신 체크포인트 탐지 (preempt > rolling, best_model 제외)
#   - 체크포인트 무결성 검증
#   - replay buffer 자동 로딩 (sac_drop_preempt_replay.pkl)
#   - 디스크 90% 도달 시 graceful stop
#   - 학습 완료 후 train_sac.pid 정리
log "학습 시작 (train_managed.sh 위임)..."
docker exec -d drone-bombard-harmonic bash -c \
  "bash /workspace/ros2_ws/train_managed.sh >> /tmp/production_train.log 2>&1"

# train_managed.sh 실행 확인 (3초 후)
sleep 3
if docker exec drone-bombard-harmonic bash -c \
    "test -f /tmp/train_managed.lock && flock -n /tmp/train_managed.lock true 2>/dev/null && echo LOCKED || echo RUNNING" \
    2>/dev/null | grep -q RUNNING; then
  log "학습 프로세스 실행 확인됨"
else
  log "WARNING: 학습 프로세스 확인 불가 (잠시 후 재확인 권장)"
fi

# --- [7] 메타데이터 플래그 활성화 ---
gcloud compute instances add-metadata "$INSTANCE" \
  --zone="$ZONE" --project="$PROJECT" \
  --metadata drone_training_active=true \
  --quiet 2>/dev/null \
  && log "메타데이터 drone_training_active=true 설정 완료" \
  || log "WARNING: 메타데이터 설정 실패. watchdog이 재시작을 못 할 수 있음."

log "=== Startup 완료 ==="
