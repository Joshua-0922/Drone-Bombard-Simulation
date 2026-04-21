"""drone-watchdog — Cloud Function (Gen2)

Spot VM 선점 감지 후 자동 재시작.
Cloud Scheduler가 5분마다 HTTP POST로 트리거.

재시작 조건 (3중 안전장치):
  1. instance.status == TERMINATED
  2. metadata drone_training_active == "true"  (수동 종료 방지)
  3. 최근 zone operation == compute.instances.preempted  (선점만 처리)

추가 보호:
  - cooldown: 마지막 재시작으로부터 15분 미만이면 skip
  - max retries: 연속 3회 이상 재시작 시 중단 + Cloud Logging 경고
"""

import logging
import os
import time

import functions_framework
from google.cloud import compute_v1

logger = logging.getLogger(__name__)
logging.basicConfig(level=logging.INFO)

PROJECT = os.environ["GCP_PROJECT"]
ZONE = os.environ["GCP_ZONE"]
INSTANCE_NAME = os.environ["VM_INSTANCE_NAME"]

COOLDOWN_SECONDS = 15 * 60   # 15분
MAX_RESTART_COUNT = 3


def _get_instance(client: compute_v1.InstancesClient):
    return client.get(project=PROJECT, zone=ZONE, instance=INSTANCE_NAME)


def _get_metadata_value(instance, key: str, default: str = "") -> str:
    if instance.metadata and instance.metadata.items:
        for item in instance.metadata.items:
            if item.key == key:
                return item.value or default
    return default


def _set_metadata(client: compute_v1.InstancesClient, instance, updates: dict):
    """인스턴스 metadata 특정 키 업데이트 (기존 키 보존)."""
    items = list(instance.metadata.items or [])
    existing_keys = {item.key for item in items}

    for key, value in updates.items():
        if key in existing_keys:
            for item in items:
                if item.key == key:
                    item.value = str(value)
        else:
            items.append(compute_v1.Items(key=key, value=str(value)))

    new_metadata = compute_v1.Metadata(
        items=items,
        fingerprint=instance.metadata.fingerprint,
    )
    op = client.set_metadata(
        project=PROJECT,
        zone=ZONE,
        instance=INSTANCE_NAME,
        metadata_resource=new_metadata,
    )
    op.result(timeout=30)


def _was_preempted(ops_client: compute_v1.ZoneOperationsClient) -> bool:
    """최근 zone operation이 선점인지 확인."""
    request = compute_v1.ListZoneOperationsRequest(
        project=PROJECT,
        zone=ZONE,
        filter=f'targetLink="https://www.googleapis.com/compute/v1/projects/{PROJECT}/zones/{ZONE}/instances/{INSTANCE_NAME}"',
        max_results=5,
    )
    ops = list(ops_client.list(request=request))
    if not ops:
        logger.warning("zone operations 없음 — preemption 확인 불가")
        return False

    # insert_time 기준 최신 operation
    latest = sorted(ops, key=lambda o: o.insert_time or "", reverse=True)[0]
    logger.info(f"최근 operation: {latest.operation_type} ({latest.status})")
    return latest.operation_type == "compute.instances.preempted"


@functions_framework.http
def watchdog(request):
    """Cloud Function entrypoint."""
    instances_client = compute_v1.InstancesClient()
    ops_client = compute_v1.ZoneOperationsClient()

    try:
        instance = _get_instance(instances_client)
    except Exception as e:
        logger.error(f"인스턴스 조회 실패: {e}")
        return f"ERROR: {e}", 500

    status = instance.status
    logger.info(f"Instance status: {status}")

    # --- 정상 상태 ---
    if status in ("RUNNING", "STAGING", "STOPPING", "SUSPENDED", "SUSPENDING"):
        return f"OK: {status}", 200

    if status != "TERMINATED":
        logger.warning(f"알 수 없는 상태: {status}")
        return f"OK: unknown status {status}", 200

    # --- TERMINATED 처리 ---
    logger.info("TERMINATED 감지 — 재시작 조건 확인 중")

    # [조건 1] training_active 플래그 확인
    training_active = _get_metadata_value(instance, "drone_training_active", "false")
    if training_active.lower() != "true":
        logger.info(f"drone_training_active={training_active} — 수동 종료 또는 학습 완료. 재시작 안 함.")
        return "OK: not training", 200

    # [조건 2] 선점 여부 확인
    try:
        preempted = _was_preempted(ops_client)
    except Exception as e:
        logger.warning(f"preemption 확인 실패: {e} — 보수적으로 재시작 안 함.")
        return "OK: preemption check failed", 200

    if not preempted:
        logger.info("최근 operation이 preemption이 아님 — 재시작 안 함.")
        return "OK: not preempted", 200

    # [조건 3] Cooldown 확인
    last_restart_str = _get_metadata_value(instance, "last_watchdog_restart", "0")
    try:
        last_restart_ts = float(last_restart_str)
    except ValueError:
        last_restart_ts = 0.0

    elapsed = time.time() - last_restart_ts
    if elapsed < COOLDOWN_SECONDS:
        remaining = int(COOLDOWN_SECONDS - elapsed)
        logger.warning(f"Cooldown 중 — 마지막 재시작으로부터 {int(elapsed)}초 경과. {remaining}초 후 재시도 가능.")
        return f"OK: cooldown ({remaining}s remaining)", 200

    # [조건 4] 연속 재시작 횟수 확인
    restart_count_str = _get_metadata_value(instance, "watchdog_restart_count", "0")
    try:
        restart_count = int(restart_count_str)
    except ValueError:
        restart_count = 0

    if restart_count >= MAX_RESTART_COUNT:
        logger.error(
            f"연속 재시작 {restart_count}회 달성 — 자동 재시작 중단. "
            f"수동으로 확인 후 watchdog_restart_count=0 으로 리셋하세요."
        )
        return f"ERROR: max restarts ({MAX_RESTART_COUNT}) reached", 500

    # --- 재시작 실행 ---
    new_count = restart_count + 1
    now_ts = str(time.time())
    logger.info(f"선점 확인됨 — VM 재시작 시도 (#{new_count})...")

    try:
        # metadata 업데이트 (재시작 전)
        _set_metadata(instances_client, instance, {
            "last_watchdog_restart": now_ts,
            "watchdog_restart_count": str(new_count),
        })
    except Exception as e:
        logger.warning(f"metadata 업데이트 실패: {e} — 재시작은 계속 진행")

    try:
        op = instances_client.start(
            project=PROJECT,
            zone=ZONE,
            instance=INSTANCE_NAME,
        )
        op.result(timeout=60)
        logger.info(f"VM 재시작 성공 (#{new_count}). startup.sh가 학습을 재개합니다.")
        return f"OK: restarted (attempt #{new_count})", 200
    except Exception as e:
        logger.error(f"VM 재시작 실패: {e}")
        return f"ERROR: start failed: {e}", 500
