#!/bin/bash
# startup.sh — GCP VM metadata startup-script (Isaac Sim + Isaac Lab)
#
# 등록 방법:
#   gcloud compute instances add-metadata g2-standard-16-nvidia-l4-dev \
#     --zone=us-central1-a --project=charming-league-481306-d8 \
#     --metadata-from-file startup-script=infra/startup.sh

set -euo pipefail
LOG=/var/log/drone-bombard-startup.log
exec > >(tee -a "$LOG") 2>&1

log() { echo "[$(date '+%Y-%m-%d %H:%M:%S')] $*"; }

REGISTRY="us-central1-docker.pkg.dev/charming-league-481306-d8/drone-bombard"
IMAGE="${REGISTRY}/isaac-lab:latest"
CONTAINER="drone-bombard-isaac"

# Isaac Sim shader/pip 캐시 — 재빌드 방지
CACHE_ROOT="/opt/isaacsim-cache"
ISAACSIM_CACHE_ARGS=(
  -v "${CACHE_ROOT}/kit:/isaac-sim/kit/cache"
  -v "${CACHE_ROOT}/ov:/root/.cache/ov"
  -v "${CACHE_ROOT}/pip:/root/.cache/pip"
  -v "${CACHE_ROOT}/glcache:/root/.cache/nvidia/GLCache"
  -v "${CACHE_ROOT}/computecache:/root/.nv/ComputeCache"
)
mkdir -p \
  "${CACHE_ROOT}/kit" \
  "${CACHE_ROOT}/ov" \
  "${CACHE_ROOT}/pip" \
  "${CACHE_ROOT}/glcache" \
  "${CACHE_ROOT}/computecache"

# --- GCP 메타데이터에서 자신의 정보 읽기 ---
INSTANCE=$(curl -sf -H "Metadata-Flavor: Google" \
  "http://metadata.google.internal/computeMetadata/v1/instance/name")
ZONE=$(curl -sf -H "Metadata-Flavor: Google" \
  "http://metadata.google.internal/computeMetadata/v1/instance/zone" | cut -d/ -f4)
PROJECT=$(curl -sf -H "Metadata-Flavor: Google" \
  "http://metadata.google.internal/computeMetadata/v1/project/project-id")

log "=== Drone-Bombard Startup (Isaac Sim) ==="
log "Instance: $INSTANCE  Zone: $ZONE  Project: $PROJECT"

# --- [0] watchdog 재시작 카운터 리셋 ---
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

# --- [2] Artifact Registry 인증 + 이미지 pull ---
log "Artifact Registry 인증..."
gcloud auth configure-docker us-central1-docker.pkg.dev --quiet

log "이미지 pull: $IMAGE"
docker pull "$IMAGE"

# --- [3] 기존 컨테이너 정리 (있으면) ---
if docker ps -a --format '{{.Names}}' | grep -q "^${CONTAINER}$"; then
  log "기존 컨테이너 중지 및 제거: $CONTAINER"
  docker stop "$CONTAINER" 2>/dev/null || true
  docker rm "$CONTAINER" 2>/dev/null || true
fi

# --- [4] 컨테이너 시작 (headless, GPU, Isaac Sim 캐시) ---
log "컨테이너 시작: $CONTAINER"
docker run -d \
  --name "$CONTAINER" \
  --gpus all \
  --runtime=nvidia \
  -e ACCEPT_EULA=Y \
  -e PRIVACY_CONSENT=Y \
  "${ISAACSIM_CACHE_ARGS[@]}" \
  "$IMAGE" \
  bash -c "
    cd \$ISAACLAB_PATH && \
    ./isaaclab.sh -p scripts/reinforcement_learning/rsl_rl/train.py \
      --task Isaac-Cartpole-Direct-v0 \
      --headless \
      --num_envs 64 \
      --max_iterations 5 \
      >> /tmp/smoke_test.log 2>&1
  "

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
if docker exec "$CONTAINER" nvidia-smi -L &>/dev/null; then
  log "GPU 확인 완료"
else
  log "WARNING: GPU 접근 실패"
fi

# --- [6] 메타데이터 플래그 활성화 ---
gcloud compute instances add-metadata "$INSTANCE" \
  --zone="$ZONE" --project="$PROJECT" \
  --metadata drone_training_active=true \
  --quiet 2>/dev/null \
  && log "메타데이터 drone_training_active=true 설정 완료" \
  || log "WARNING: 메타데이터 설정 실패"

log "=== Startup 완료 ==="
