#!/bin/bash
# create_spot_vm.sh — L4 Spot VM 멀티존 자동 재시도 생성
#
# GPU 가용성이 없을 경우 여러 존을 순서대로 시도.
# 성공한 존으로 Cloud Function 환경변수 자동 업데이트.
#
# 사용법:
#   bash infra/create_spot_vm.sh              # 기본 (가용성 우선순위 순서)
#   bash infra/create_spot_vm.sh --check-only # 가용성 확인만 (VM 생성 안 함)
#   bash infra/create_spot_vm.sh --zone asia-northeast3-a  # 특정 존 강제

set -euo pipefail

PROJECT="charming-league-481306-d8"
INSTANCE="g2-standard-16-nvidia-l4-dev"
MACHINE_TYPE="g2-standard-8"          # CPU 8개 (g2-standard-16은 CPU 16개)
ACCELERATOR="type=nvidia-l4,count=1"
STARTUP_SCRIPT="$(dirname "$0")/startup.sh"
FUNCTION_NAME="drone-watchdog"
FUNCTION_REGION="us-central1"

# ----------------------------------------------------------------
# 존 우선순위 (L4 가용성 + 비용 균형 기준)
#
# 선정 기준:
#   1. us-central1-*   : GCP L4 풀이 가장 크고 Spot 가용성 높음
#   2. asia-east1-*    : 대만, 서울보다 L4 풀 큼, 지연 적당
#   3. asia-northeast1-*: 도쿄, 서울보다 가용성 높음
#   4. us-east4-*      : 버지니아, 백업용
#   5. asia-northeast3-*: 서울, 가격 가장 저렴하지만 가용성 낮음
# ----------------------------------------------------------------
CANDIDATE_ZONES=(
  "us-central1-a"
  "us-central1-b"
  "us-central1-c"
  "us-central1-f"
  "asia-east1-a"
  "asia-east1-b"
  "asia-east1-c"
  "asia-northeast1-a"
  "asia-northeast1-b"
  "us-east4-a"
  "us-east4-b"
  "asia-northeast3-a"
  "asia-northeast3-b"
)

CHECK_ONLY=false
FORCE_ZONE=""

for arg in "$@"; do
  case "$arg" in
    --check-only) CHECK_ONLY=true ;;
    --zone) shift; FORCE_ZONE="$1" ;;
    --zone=*) FORCE_ZONE="${arg#--zone=}" ;;
  esac
done

log() { echo "[$(date '+%H:%M:%S')] $*"; }

# ----------------------------------------------------------------
# 가용성 확인 함수
# gcloud compute accelerator-types list가 존재해도
# 실제 Spot 할당 가능 여부와 다를 수 있음 → 실제 생성 시도가 가장 정확
# ----------------------------------------------------------------
check_l4_listed() {
  local zone="$1"
  gcloud compute accelerator-types list \
    --filter="zone:${zone} AND name:nvidia-l4" \
    --project="$PROJECT" \
    --format="value(name)" 2>/dev/null | grep -q "nvidia-l4"
}

# ----------------------------------------------------------------
# STEP 0: 가용성 사전 스캔
# ----------------------------------------------------------------
log "=== L4 GPU 가용성 스캔 ==="
AVAILABLE_ZONES=()
for zone in "${CANDIDATE_ZONES[@]}"; do
  if check_l4_listed "$zone"; then
    log "  OK  $zone — L4 등록됨"
    AVAILABLE_ZONES+=("$zone")
  else
    log "  --  $zone — L4 미등록"
  fi
done

echo ""
log "L4 등록 존: ${AVAILABLE_ZONES[*]:-없음}"
echo ""

if [ "$CHECK_ONLY" = "true" ]; then
  log "check-only 모드 종료."
  exit 0
fi

if [ -n "$FORCE_ZONE" ]; then
  AVAILABLE_ZONES=("$FORCE_ZONE")
  log "강제 존: $FORCE_ZONE"
fi

if [ ${#AVAILABLE_ZONES[@]} -eq 0 ]; then
  log "ERROR: 어느 존에도 L4 GPU가 등록되어 있지 않습니다."
  log "  gcloud compute accelerator-types list --filter='name:nvidia-l4' 로 직접 확인하세요."
  exit 1
fi

# ----------------------------------------------------------------
# STEP 1: 존 순서대로 VM 생성 시도
# ----------------------------------------------------------------
CREATED_ZONE=""

for zone in "${AVAILABLE_ZONES[@]}"; do
  log "=== $zone 에서 Spot VM 생성 시도 ==="
  region="${zone%-*}"  # us-central1-a → us-central1

  # 이미 존재하면 스킵
  if gcloud compute instances describe "$INSTANCE" \
       --zone="$zone" --project="$PROJECT" &>/dev/null; then
    log "이미 존재: $INSTANCE ($zone)"
    CREATED_ZONE="$zone"
    break
  fi

  if gcloud compute instances create "$INSTANCE" \
       --zone="$zone" \
       --project="$PROJECT" \
       --machine-type="$MACHINE_TYPE" \
       --accelerator="$ACCELERATOR" \
       --provisioning-model="SPOT" \
       --instance-termination-action="STOP" \
       --maintenance-policy="TERMINATE" \
       --boot-disk-size="200GB" \
       --boot-disk-type="pd-ssd" \
       --image-family="ubuntu-2204-lts" \
       --image-project="ubuntu-os-cloud" \
       --metadata-from-file="startup-script=${STARTUP_SCRIPT}" \
       --metadata="drone_training_active=false,watchdog_restart_count=0,last_watchdog_restart=0" \
       --scopes="https://www.googleapis.com/auth/cloud-platform" \
       --tags="drone-bombard,ssh-allowed" \
       2>&1; then
    log "VM 생성 성공: $INSTANCE ($zone)"
    CREATED_ZONE="$zone"
    break
  else
    log "FAILED: $zone — 다음 존 시도..."
    sleep 2
  fi
done

if [ -z "$CREATED_ZONE" ]; then
  log "ERROR: 모든 존에서 VM 생성 실패."
  log ""
  log "권장 조치:"
  log "  1. 잠시 후(10-30분) 재시도: Spot 가용성은 수시로 변함"
  log "  2. 시간대 변경: 미국 새벽(한국 오후)에 us-central1 가용성 높아짐"
  log "  3. 할당량 확인: gcloud compute regions describe \$REGION --format='yaml(quotas)'"
  exit 1
fi

# ----------------------------------------------------------------
# STEP 2: Cloud Function 환경변수 업데이트 (존이 바뀐 경우)
# ----------------------------------------------------------------
CREATED_REGION="${CREATED_ZONE%-*}"
log "=== Cloud Function 환경변수 업데이트 ==="
gcloud functions deploy "$FUNCTION_NAME" \
  --gen2 \
  --region="$FUNCTION_REGION" \
  --project="$PROJECT" \
  --set-env-vars="GCP_PROJECT=${PROJECT},GCP_ZONE=${CREATED_ZONE},VM_INSTANCE_NAME=${INSTANCE}" \
  --quiet 2>/dev/null \
  && log "Cloud Function 환경변수 업데이트 완료" \
  || log "WARNING: Cloud Function 업데이트 실패 (수동으로 GCP_ZONE=${CREATED_ZONE} 설정 필요)"

# ----------------------------------------------------------------
# STEP 3: 결과 요약
# ----------------------------------------------------------------
echo ""
log "==========================="
log "VM        : $INSTANCE"
log "Zone      : $CREATED_ZONE"
log "Machine   : $MACHINE_TYPE + L4 x1 (Spot)"
log "==========================="
echo ""
log "다음 단계:"
log "  gcloud compute ssh $INSTANCE --zone=$CREATED_ZONE --project=$PROJECT"
log "  tail -f /var/log/drone-bombard-startup.log"
