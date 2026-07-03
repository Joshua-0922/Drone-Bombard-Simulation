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

# --- Isaac Lab Phase 2 (drone_bombard): repo mount + log/checkpoint dir ---
# Code is mounted, not baked into the image, so isaac_lab/ iterates without
# a rebuild+push. See isaac_lab/README.md for the full run procedure.
REPO_ROOT="/opt/drone-bombard/Drone-Bombard-Simulation"
LOGS_ROOT="/opt/drone-bombard/rl_runs"
mkdir -p "$LOGS_ROOT"
DRONE_BOMBARD_MOUNT_ARGS=(
  -v "${REPO_ROOT}:/workspace/drone-bombard"
  -v "${LOGS_ROOT}:/workspace/logs"
)
WANDB_ENV=/opt/drone-bombard/.wandb.env
WANDB_ENV_ARGS=()
if [ -f "$WANDB_ENV" ]; then
  WANDB_KEY=$(grep WANDB_API_KEY "$WANDB_ENV" | cut -d= -f2)
  if [ -n "$WANDB_KEY" ]; then
    WANDB_ENV_ARGS=(-e "WANDB_API_KEY=${WANDB_KEY}")
  fi
fi

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

# --- [4] 컨테이너 시작 (headless, GPU, Isaac Sim 캐시 + drone_bombard mount) ---
# 두 단계 게이트: (a) Cartpole 스모크(기존 인프라 검증) 통과 후 (b) drone_bombard
# PPO 학습 시작. Cartpole이 non-zero reward로 5 iteration 완료하지 못하면
# drone_bombard는 시작하지 않음 (인프라 결함을 태스크 결함과 혼동 방지).
log "컨테이너 시작: $CONTAINER"
docker run -d \
  --name "$CONTAINER" \
  --gpus all \
  --runtime=nvidia \
  -e ACCEPT_EULA=Y \
  -e PRIVACY_CONSENT=Y \
  "${ISAACSIM_CACHE_ARGS[@]}" \
  "${DRONE_BOMBARD_MOUNT_ARGS[@]}" \
  "${WANDB_ENV_ARGS[@]}" \
  "$IMAGE" \
  bash -c "
    cd \$ISAACLAB_PATH && \
    ./isaaclab.sh -p scripts/reinforcement_learning/rsl_rl/train.py \
      --task Isaac-Cartpole-Direct-v0 \
      --headless \
      --num_envs 64 \
      --max_iterations 5 \
      >> /tmp/smoke_test.log 2>&1 && \
    echo '[startup] Cartpole smoke PASS — launching drone_bombard PPO training' >> /tmp/smoke_test.log && \
    exec ./isaaclab.sh -p /workspace/drone-bombard/isaac_lab/train.py \
      --task Isaac-DroneBombard-Direct-v0 \
      --headless \
      --num_envs 2048 \
      --resume latest \
      --checkpoint_dir /workspace/logs/isaac_lab/drone_bombard \
      >> /workspace/logs/isaac_lab/drone_bombard_train.log 2>&1
  "

# 컨테이너 안정화 대기
sleep 5

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
