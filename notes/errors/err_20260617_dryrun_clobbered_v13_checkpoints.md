---
date: 2026-06-17
tags: [error, incident, checkpoint, data-loss, train_sac, yaml, isolation]
status: resolved
type: errors
---

# 인시던트 — armdiag dry-run이 v13 30K 체크포인트를 파괴 (YAML 중복 키 + fresh-start 삭제)

> **영향:** `rl_yolo_v13_terminal_reward`(46y4xtiw)의 ~30K 체크포인트 + 리플레이 **영구 소실**. 30K 재개 불가 → fresh 재시작(iyhfy5ps)으로 복구.
> **관련:** [[experiments/exp_006_xgzum51v_armdiag_dryrun]] / [[research/cruise_timeout_arming]]

---

## 무슨 일이 있었나

arm_bail 진단 dry-run을 **격리** 의도로 `checkpoint_dir: .../armdiag_dryrun`로 설정했다고 믿었으나, 실제로는 **메인 체크포인트 디렉터리**(`/workspace/ros2_ws/rl_checkpoints`)에서 실행됨. 결과:

1. **dry-run startup (no `--resume`):** train_sac L379-382가 `checkpoint_dir`의 모든 `sac_drop_*_steps.zip`를 `os.remove`. → v13의 주기 체크포인트 5개(10K/15K/20K/25K/**30K**) 삭제. 로그: `[Startup] Removed 5 stale checkpoint(s)`.
2. **dry-run SIGTERM:** `_emergency_save`가 `sac_drop_preempt.zip` + `_replay.pkl`를 **599-step** 상태로 덮어씀(12:10).

복구 시도 전수 조사: 디스크의 모든 `.zip`에서 v13 크기(~30K) 없음. wandb run 디렉터리의 `sac_drop_preempt.zip`은 **심볼릭 링크**라 덮어쓴 599-step 파일을 가리킴. `best_model/` 비어 있음. → **v13 30K 디스크 복구 불가.** (리플레이는 cloud 미업로드라 어차피 소실.)

## 근본 원인 (2겹)

1. **🔴 YAML 중복 키 (내 실수):** `hyperparams_v13_armdiag.yaml`의 `training:` 아래에 `checkpoint_dir`가 **두 번** 존재 —
   - L26: `.../armdiag_dryrun` (내가 추가)
   - L29: `.../rl_checkpoints` (**기존 키, 못 보고 위에 중복 추가**)

   YAML은 중복 키에서 **마지막 값이 승리** → L29(메인)가 L26을 덮음. 격리가 조용히 무효화됨.
2. **🟠 train_sac fresh-start의 파괴적 삭제:** `--resume` 없으면 resolve된 `checkpoint_dir`의 `sac_drop_*_steps.zip`를 **무조건 삭제** (L379). 공유 디렉터리에서 fresh-start는 footgun.

## 해결 / 재발 방지 규칙

- **기존 키를 추가가 아니라 *편집*하라.** 새 설정 추가 전 `grep -n '<key>' <config>`로 기존 키 확인. (이번엔 `checkpoint_dir`가 이미 있었음.)
- **격리는 실행 *전* 로그로 검증하라.** train_sac startup의 `Checkpoints : <dir>` 줄을 확인하고, 파괴적 fresh-start는 그 줄이 격리 경로일 때만 진행.
- **공유 dir에서 진단 dry-run 금지.** dry-run은 별도 부모 dir(또는 `--checkpoint-dir` CLI 명시 + 검증)로.
- **(개선 후보)** train_sac fresh-start 삭제를 격리 경로 밖이면 거부하거나, `--checkpoint-dir` 미지정 시 config의 첫/유일 키만 허용하도록 방어.

## 교훈

진단 도구(격리 dry-run)가 진단 대상(학습 상태)을 파괴했다. "격리했다"는 *믿음*을 startup 로그로 *검증*하지 않은 것이 핵심 실패. 데이터 손실은 비가역 — 파괴적 작업(fresh-start, rm, overwrite) 전 대상 경로를 눈으로 확인하는 절차를 강제할 것.
