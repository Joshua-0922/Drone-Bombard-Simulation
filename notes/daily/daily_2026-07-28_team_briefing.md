---
date: 2026-07-28
tags: [team, briefing, merge, moving-target, kalman-filter]
status: active
type: daily
---

# 팀 브리핑 — isaac_jk 브랜치 현황 (2026-07-28)

> 대상: 전체 팀원. isaac_jk 브랜치에서 오늘 무엇이 바뀌었고,
> 각자 무엇을 확인/진행해야 하는지 정리한 문서.

---

## 1. 오늘 한 일 (What we did)

### ① Issac_JS(준상) 브랜치 머지 → isaac_jk
- 준상의 최신 6커밋을 머지 커밋 **`a099de3`** 으로 반영, GitHub `origin/isaac_jk`에 푸시 완료:
  - v19 collapse fix (A+B) + best-checkpoint retention + viz
  - v19 precision push: continuous landing reward (toggle)
  - **v19 warm-start 체크포인트 공유**: `checkpoints/v19/{precise,abd}/model_{best,final}.pt` + `WARMSTART.md`
  - `isaac_lab/select_best_checkpoint.py` 신규, `REVIEW_GUI.md` 신규 (play.py `--show` 검토용 GUI 사용법)
- 워크트리의 로컬 구버전 파일들(`drone_bombard_env.py`, `play.py`, `v11_env.py`, `train.py`,
  `__init__.py`, `math_utils.py`, `test_math.py`)은 **전부 머지된 HEAD(준상+제균 통합) 버전으로 확정**.
  구버전은 `stash@{0}` "pre-merge-IssacJS backup 2026-07-28"에 보존.

### ② 이동 타겟(X marker) + Kalman 트래커 구현 (커밋 `7f7217f`)
- **모션 모델 3종 신설** — `--target_motion {gm,cv,ca,ct}` (base env 전용):
  - `gm`(기본값): 기존 Phase-3 Gauss-Markov 속도워크 (동작 불변)
  - `cv`: 등속 직선 / `ca`: 등가속 / `ct`: 협조선회(정확한 원호 적분)
- `--moving_target`: phase와 무관하게 이동 타겟 강제 ON
- **`--target_kf`**: Singer(Gauss-Markov 가속) Kalman 필터 트래커 — YOLO 검출 픽셀을
  지면에 역투영해 필터링, 정책 관측에 추정 타겟 위치/속도/가속 추가 (**obs 14 → 21**)
- 검증: 단위테스트 57/57 PASS + isaac-verify 스모크 4종 PASS (KF 추적오차 0.09–0.12 m)
- 상세 설계/파라미터: [[research/moving_target_models]]

---

## 2. 현재 상태 (Where we are)

| 항목 | 상태 |
|------|------|
| 브랜치 | `isaac_jk` = `origin/isaac_jk` (HEAD `7f7217f`), upstream도 `origin/isaac_jk` |
| 코드베이스 | 준상 Issac_JS 최신 + 제균 exp020 + 이동타겟/KF 기능 통합 |
| base env | `Isaac-DroneBombard-Direct-v0` — phase 커리큘럼 + 이동타겟/KF (신규) |
| v-track | v11–v19 등록 복원됨 (`--v11_test` ~ `--v19`) — 이동타겟/KF는 **미적용** |
| 체크포인트 | `checkpoints/v19/`(준상 warm-start용), `/opt/drone-bombard/checkpoints/exp020/`(제균 물리 페이로드) |
| 미커밋 | `record_episode.py`·`00_index.md` 수정 + 렌더 작업 파일들(07-24, 제균) — 처리 방침 미정 |

---

## 3. 각자 확인/진행할 일 (What you should do)

### 모두 (공통)
- `git pull origin isaac_jk` 후 작업. **이 브랜치에서 `Issac_JS`로 직접 push 금지**
  (upstream은 isaac_jk로 고정해 둠).
- 학습/평가는 컨테이너(`isaac-verify`)에서:
  ```bash
  docker exec -u root -e PYTHONUNBUFFERED=1 --env-file /opt/drone-bombard/.wandb.env \
    isaac-verify bash -c "cd /workspace/drone-bombard/isaac_lab && \
    /workspace/isaaclab/isaaclab.sh -p train.py <flags>"
  ```
- 단위테스트:
  ```bash
  docker exec isaac-verify bash -c "cd /workspace/drone-bombard && \
    PYTEST_DISABLE_PLUGIN_AUTOLOAD=1 /workspace/isaaclab/isaaclab.sh -p -m pytest \
    isaac_lab/tests/test_math.py -q -p no:cacheprovider"
  ```

### 이동 타겟 학습을 돌릴 사람
- 예시:
  ```bash
  # phase 1 + 이동 타겟(CV) + KF 관측 — fresh start 필수
  train.py --phase 1 --moving_target --target_motion cv --target_kf --headless --num_envs 2048
  # phase 3(자동 이동 타겟) + 협조선회 + KF
  train.py --phase 3 --target_motion ct --target_kf --headless --num_envs 2048
  ```
- ⚠️ **`--target_kf`는 obs 14→21 → 기존 14-dim 체크포인트 warm-start 불가 (Fresh Start).**
  `--target_motion`/`--moving_target`만 쓰면 obs 불변 → 기존 ckpt warm-start 가능.
- ⚠️ **평가(play.py)는 학습과 동일한 타겟/KF 플래그로** — 차원 불일치 시 즉사.
- 새 지표: `Episode_Metric/kf_track_rate · kf_pos_err_m · kf_vel_err_mps`, (phase 3) `lead_error_m`

### 준상 (v-track 오너)
- v19 warm-start 가이드(`checkpoints/v19/WARMSTART.md`)는 그대로 유효.
- 이동타겟/KF의 **v19 포팅 여부 결정** 필요 — 현재는 base env에만 있음. 원하면 요청.

### 제균 (worktree 오너)
- 미커밋 렌더 작업(`record_episode.py`, `animate_episode.py`, `sdg_dtype_patch.py`,
  `00_index.md`) 커밋/정리 방침 결정 대기.
- 다음 학습 후보: exp020 페이로드 ckpt에서 `--moving_target` warm-start (obs 불변 경로),
  또는 `--target_kf` fresh start 비교군.

---

## 4. 관련 노트
- [[daily/daily_2026-07-28]] — 상세 작업 로그
- [[research/moving_target_models]] — 이동타겟/KF 설계·파라미터·주의사항
- [[research/rl_rules]] / [[experiments/training_history]]
