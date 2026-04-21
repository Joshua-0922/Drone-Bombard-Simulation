#!/bin/bash
# deploy.sh — Spot VM + Cloud Function 무인 학습 자동화 인프라 배포
#
# 일회성 실행 스크립트. 새 VM 생성 시 한 번만 실행.
# 반드시 순서대로 실행할 것 (각 단계 독립 실행 가능).
#
# 사전 요구사항:
#   gcloud auth login && gcloud auth application-default login
#   gcloud config set project charming-league-481306-d8

set -euo pipefail

PROJECT="charming-league-481306-d8"
OLD_ZONE="us-central1-c"
NEW_ZONE="us-central1-a"
INSTANCE="g2-standard-16-nvidia-l4-dev"
OLD_DISK="$INSTANCE"          # 기존 디스크 이름 (보통 인스턴스명과 동일)
REGION="us-central1"
FUNCTION_NAME="drone-watchdog"
SCHEDULER_JOB="drone-watchdog-trigger"
SA_NAME="drone-watchdog-sa"
SA_EMAIL="${SA_NAME}@${PROJECT}.iam.gserviceaccount.com"

log() { echo ""; echo "=== $* ==="; }

# ============================================================
# STEP 0: GPU 가용성 확인
# ============================================================
log "STEP 0: us-central1-a L4 GPU 가용성 확인"
echo "아래 목록에 'nvidia-l4'가 us-central1-a에 있으면 진행 가능:"
gcloud compute accelerator-types list \
  --filter="zone:us-central1-a AND name:nvidia-l4" \
  --format="table(name,zone)"

echo ""
read -p "계속 진행하시겠습니까? (y/N) " confirm
[ "$confirm" = "y" ] || { echo "중단."; exit 0; }

# ============================================================
# STEP 1: 기존 VM 스냅샷 생성 (안전망)
# ============================================================
log "STEP 1: 기존 VM 디스크 스냅샷 생성"
SNAPSHOT_NAME="${INSTANCE}-migration-$(date +%Y%m%d)"
gcloud compute disks snapshot "$OLD_DISK" \
  --zone="$OLD_ZONE" \
  --project="$PROJECT" \
  --snapshot-names="$SNAPSHOT_NAME" \
  --description="Migration to $NEW_ZONE"
echo "스냅샷 생성 완료: $SNAPSHOT_NAME"

# ============================================================
# STEP 2: 새 디스크 생성 (스냅샷에서, us-central1-a)
# ============================================================
log "STEP 2: us-central1-a에 새 디스크 생성"
NEW_DISK="${INSTANCE}-disk-a"
gcloud compute disks create "$NEW_DISK" \
  --zone="$NEW_ZONE" \
  --project="$PROJECT" \
  --source-snapshot="$SNAPSHOT_NAME" \
  --type="pd-ssd"
echo "새 디스크 생성 완료: $NEW_DISK"

# ============================================================
# STEP 3: 기존 VM 종료
# ============================================================
log "STEP 3: 기존 VM 종료 (us-central1-c)"
echo "기존 VM에서 학습이 실행 중이면 먼저 infra/stop.sh로 종료하세요."
read -p "기존 VM 종료 진행? (y/N) " confirm
if [ "$confirm" = "y" ]; then
  gcloud compute instances stop "$INSTANCE" \
    --zone="$OLD_ZONE" --project="$PROJECT" || true
fi

# ============================================================
# STEP 4: 새 Spot VM 생성 (us-central1-a)
# ============================================================
log "STEP 4: 새 Spot VM 생성 (us-central1-a, L4 GPU)"
# startup.sh를 메타데이터에 인라인 등록
STARTUP_SCRIPT_PATH="$(dirname "$0")/startup.sh"

gcloud compute instances create "$INSTANCE" \
  --zone="$NEW_ZONE" \
  --project="$PROJECT" \
  --machine-type="g2-standard-16" \
  --accelerator="type=nvidia-l4,count=1" \
  --provisioning-model="SPOT" \
  --instance-termination-action="STOP" \
  --maintenance-policy="TERMINATE" \
  --boot-disk-size="200GB" \
  --boot-disk-type="pd-ssd" \
  --disk="name=${NEW_DISK},boot=yes,auto-delete=no" \
  --metadata-from-file="startup-script=${STARTUP_SCRIPT_PATH}" \
  --metadata="drone_training_active=false,watchdog_restart_count=0,last_watchdog_restart=0" \
  --scopes="https://www.googleapis.com/auth/cloud-platform" \
  --tags="drone-bombard,ssh-allowed"
echo "VM 생성 완료: $INSTANCE ($NEW_ZONE)"

# ============================================================
# STEP 5: Service Account 생성 및 IAM 권한 부여
# ============================================================
log "STEP 5: Cloud Function Service Account 설정"
gcloud iam service-accounts create "$SA_NAME" \
  --project="$PROJECT" \
  --display-name="Drone Watchdog Cloud Function SA" 2>/dev/null || echo "SA 이미 존재"

# 필요 권한 부여
for ROLE in \
  "roles/compute.viewer" \
  "roles/compute.instanceAdmin.v1"; do
  gcloud projects add-iam-policy-binding "$PROJECT" \
    --member="serviceAccount:${SA_EMAIL}" \
    --role="$ROLE" --quiet
done
echo "IAM 권한 설정 완료"

# ============================================================
# STEP 6: Cloud Function 배포
# ============================================================
log "STEP 6: Cloud Function 배포"
FUNCTION_DIR="$(dirname "$0")/watchdog"

gcloud functions deploy "$FUNCTION_NAME" \
  --gen2 \
  --runtime="python311" \
  --region="$REGION" \
  --project="$PROJECT" \
  --source="$FUNCTION_DIR" \
  --entry-point="watchdog" \
  --trigger-http \
  --no-allow-unauthenticated \
  --service-account="$SA_EMAIL" \
  --set-env-vars="GCP_PROJECT=${PROJECT},GCP_ZONE=${NEW_ZONE},VM_INSTANCE_NAME=${INSTANCE}" \
  --timeout=120 \
  --memory=256Mi

FUNCTION_URL=$(gcloud functions describe "$FUNCTION_NAME" \
  --gen2 --region="$REGION" --project="$PROJECT" \
  --format="value(serviceConfig.uri)")
echo "Cloud Function URL: $FUNCTION_URL"

# ============================================================
# STEP 7: Cloud Scheduler 생성 (5분마다 트리거)
# ============================================================
log "STEP 7: Cloud Scheduler 설정"

# Scheduler용 SA (invoker 권한)
SCHEDULER_SA="scheduler-invoker@${PROJECT}.iam.gserviceaccount.com"
gcloud iam service-accounts create "scheduler-invoker" \
  --project="$PROJECT" \
  --display-name="Scheduler Cloud Function Invoker" 2>/dev/null || echo "SA 이미 존재"

gcloud functions add-invoker-policy-binding "$FUNCTION_NAME" \
  --gen2 --region="$REGION" --project="$PROJECT" \
  --member="serviceAccount:${SCHEDULER_SA}" 2>/dev/null || true

gcloud scheduler jobs create http "$SCHEDULER_JOB" \
  --location="$REGION" \
  --project="$PROJECT" \
  --schedule="*/5 * * * *" \
  --uri="$FUNCTION_URL" \
  --oidc-service-account-email="$SCHEDULER_SA" \
  --message-body='{"check": true}' \
  --description="Drone Spot VM watchdog — 5min interval" 2>/dev/null \
  || gcloud scheduler jobs update http "$SCHEDULER_JOB" \
    --location="$REGION" --project="$PROJECT" \
    --schedule="*/5 * * * *"

echo "Cloud Scheduler 설정 완료"

# ============================================================
# STEP 8: startup-script 메타데이터 등록 확인
# ============================================================
log "STEP 8: 배포 완료 요약"
echo ""
echo "VM          : $INSTANCE ($NEW_ZONE)"
echo "Spot VM     : 선점 시 STOP (디스크 보존)"
echo "Watchdog    : $FUNCTION_URL"
echo "Scheduler   : 매 5분 트리거"
echo ""
echo "다음 단계:"
echo "  1. VM이 부팅되면 /var/log/drone-bombard-startup.log 확인"
echo "  2. WandB 대시보드에서 학습 재개 확인"
echo "  3. 수동 종료 시: bash infra/stop.sh"
