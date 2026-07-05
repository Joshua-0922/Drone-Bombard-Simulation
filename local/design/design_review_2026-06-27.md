# Design Review — Drone-Bombard-Simulation (2026-06-27)

> **배경**: v9a (`zjexq20k`) 17k step fine-tune 결과 분석 → 사용자 의도 (지나치는 현상 해결) 미달성 → 다음 처방 방향 결정

> **이전 design_review**: [design_review.md](design_review.md) — Phase 1 redux v3 시점 (2026-06-05)
> **이전 라운드들 (v3~v8)**: master.txt 의 Round 이력 + parameter_log.md §3 표

작성: 2026-06-27
관련:
- [../issues/master.txt](../issues/master.txt) — 안건 전체 관리
- [../parameter_log.md](../parameter_log.md) — entry #38 (v9a)
- [../meeting_notes/meeting_notes_2026-06-22.txt](../meeting_notes/meeting_notes_2026-06-22.txt) — v9 처방 결정
- [../meeting_notes/meeting_notes_2026-06-26.txt](../meeting_notes/meeting_notes_2026-06-26.txt) — v9a 학습 모니터 + 평가
- [../meeting_notes/meeting_notes_2026-06-27.txt](../meeting_notes/meeting_notes_2026-06-27.txt) — 결과 분석 + 다음 처방
- [../issues/issue_026_toss_environment_indistinguishable.md](../issues/issue_026_toss_environment_indistinguishable.md)
- [../issues/issue_027_payload_tracking_after_detach.md](../issues/issue_027_payload_tracking_after_detach.md)

---

## 1. 현재 상태 요약

### v8 baseline (Phase 1 redux v8, 96bokgae 303k)
- success 80.6%, mean drop_err 1.85m, 13 jackpots, best 0.07m
- 정책 행동: **toss 전략** — drone 이 marker 지나친 후 멈춤 + pitch back + drop → payload toss
- 모든 ep 마다 일관 행동 = 정책 완전 수렴
- 8.6 GB backup (`local/backups/phase1_redux_v8_2026-06-21/`)

### v9a fine-tune (zjexq20k 313k = v8 + 17k step)
- 처방: w_dist 1.0 → 1.5, drop_angaccel_penalty_scale = 0.5 NEW, drop_angaccel_window_n = 5 NEW
- 결과 (5 ep 첫 평가):
  - success 80% (v8 동일), mean 1.89m (분포 안)
  - **max ang_vel 2.10 rad/s (v8 2.5 대비 -16% ↓)** ← drop_angaccel 효과 명확
  - 정책 행동: **toss 그대로 유지**
- 17k step (의도 100k 의 17%) — 본질 행동 변화 부족

### v9a resume (xzoz52cw 432k = +119k, CUDA error 종료)
- 의도: 313k preempt 에서 resume + 300k 추가 학습 (~600k 까지)
- 진행: 7시간 18분, 119k step 진행
- 종료: CUDA error → container SIGKILL → preempt save 실패
- 구원: rolling checkpoint 5k 마다 저장 → `eval_models/v9a_step432806.zip`
- 평가 (5 ep): drops 3/5, success 2/3 = 66.7%, mean 1.96m, 2 hover_timeout
- → 추가 학습이 정책 악화 (catastrophic forgetting 의심)

### 10 EP 통계 비교 (v8 vs v9a 313k, 2026-06-27)

| 모델 | step | success ≤2m | mean err | hover_timeout |
|---|---|---|---|---|
| **v8** | 217k | **5/10 = 50%** | 2.002m | 0 |
| **v9a 313k** | 313k | 3/9 = 33% | 2.006m | 1 |
| v9a 432k (5 ep) | 432k | 2/3 = 66.7% | 1.96m | 2 ⚠️ |

**핵심**:
- 이전 5 ep 100%/80% 는 **표본 운**
- v8 가 v9a 보다 success 17% ↑
- mean err 둘 다 ≈ 2.0m (success_threshold 경계)
- **v8 = best baseline 확정**

### 사용자 최종 결정 (2026-06-27)
- v9a 의 fine-tune 처방으로는 사용자 의도 (지나치는 현상 해결) 달성 어려움
- **새 design framework (2 단계 모드 분리, two_stage_learning_plan.md) 진행**
- v10a (단계 1 만 학습) 시도 → 사용자 중단 → v9a 평가 → 최종 결정

### ang_vel callback fix (Issue #024, 2026-06-22)
- v8 학습 전체가 ang_vel obs 없이 진행됨에도 80% 달성
- root cause: PX4 dds_topics.yaml 의 `vehicle_angular_velocity` 주석 처리
- fix: uncomment + PX4 rebuild + `limit_ang_vel: 2.0 → 10.0`
- v9a 의 drop_angaccel penalty 의 prerequisite

### dgui 도구 (2026-06-22)
- `local/scripts/evaluate_gui.py` — 학습된 SAC 모델의 GUI 시각 평가
- alias `dgui` (~/.bashrc)
- 처방 history (D-1, M-1, B, settle 5s, H, camera mode)
- v8 + v9a 모델 모두 검증됨

---

## 2. v9a 결과 분석 — 무엇이 효과 있고 무엇이 없었나

### 효과 명확

**drop_angaccel_penalty_scale = 0.5** (처방 1, drop 시 ang_accel penalty):
- v8 의 max ang_vel 2.5 rad/s → v9a 의 2.10 rad/s (-16%)
- 정책이 부드러운 drop 학습
- 사용자 의도 "드롭 시 각속도 변화 제한" 달성

### 효과 미미

**w_dist 1.0 → 1.5** (처방 2, payload distance reward 강화):
- mean err 1.85 → 1.89 (분포 안, variance noise)
- 정책의 toss 전략 본질 안 바뀜
- 원인:
  - 누적 +1.4 per ep (terminal +30 의 5%) 의 신호 크기 부족
  - v8 의 강한 toss prior 가 fine-tune 17k step 동안 흡수

### 사용자 의도 미달성

**"지나치는 현상 해결"** — drone 이 marker 위에서 drop (hover-drop) 또는 가까이서 drop 학습:
- toss 전략 그대로 유지
- 환경 자체가 hover-drop vs toss 구분 못 함 (Issue #026)

---

## 3. 환경 의 약점 (Issue #026)

| # | 환경 요소                          | toss 유리하게 만든 부분                          |
|---|-----------------------------------|----------------------------------------------|
| 1 | `auto_drop_threshold = 2.0`       | 정책이 explicit drop 결정 안 함. ballistic CCIP   |
|   |                                   | < 2m 이면 자동 drop. dynamics 가 drop 시점 결정    |
| 2 | sparse reward (terminal `drop_err` 만) | 어디서 drop 할지 명시적 안내 없음              |
| 3 | target NE vs cruise NW            | 정책이 cruise 명령 override → forward vel 활용     |
| 4 | detach 후 ep 즉시 종료              | payload trajectory 직접 평가 안 됨                |

---

## 4. 다음 처방 후보 (사용자 결정 대기)

### 옵션 A — v9a 100k 까지 더 학습 (Issue #025)
- 현재 17k → +83k (~12-18시간)
- v9a 의 처방 효과가 누적되어 본질 행동 변화 가능
- 가장 보수적, 안전

### 옵션 B — 사용자 두번째 처방 1 (target 거리 제한)
```python
if d_xy < target_distance_min:
    reward -= scale * (target_distance_min - d_xy)
```
- drone 이 target 가까이 가면 penalty → early shot 강제
- 안전: ★★★, 효과 명확

### 옵션 C — 사용자 두번째 처방 3 (drop 좌표 강제)
```python
if drop_triggered:
    reward -= scale * ||drone_xy - desired_drop_xy||
```
- 정책이 그 좌표에서 drop + 속도 조절 학습
- 안전: ★★, 강력 효과, 좌표 결정이 미묘

### 옵션 D — payload tracking 완전 구현 (Issue #027)
- Gazebo payload pose 받기 + detach 후 ep 연장
- 사용자 의도 100% 달성
- 코드 변경 크기: 큼 (~80 줄 + bridge yaml)

### 옵션 E — 환경 변경
- spawn/target randomization → 정책 generalize
- cruise 명령 비활성 → 정책 free flight (NW prior 제거)

### 옵션 F — Fresh start with stronger shaping
- v8 prior 폐기, 새 학습 (300k+)
- 시간 비용 큼

### 옵션 G — 2 단계 모드 분리 학습 (사용자 thoughts framework — design 폴더)
사용자가 직접 정리한 새 모델 framework:
- 단계 1 (접근 모드): target R 거리 안 진입까지 제약 만족 자유 비행
- 단계 2 (투하 모드): 거리 안 진입 후 drop 정확도 + 시간 제한 + 각속도/각가속도 minimization
- 학습: 단계 1 → 다양 모델 풀 → 그 중 한 모델 + 단계 2 → 확률적 mixing 융합
- 원본: `local/thoughts`
- plan: [two_stage_learning_plan.md](two_stage_learning_plan.md)

### 폐기 (위험)
- 사용자 두번째 처방 2 (시간 패널티 강화) — per-step density 변경 SAC 발산 위험 (Round 4 교훈)

---

## 5. Claude 추천 진행 순서

```
1. v9a → 100k 까지 더 학습 (Issue #025)
   └─ 결과 보고:
       └─ 본질 변화 ✓ → 종료
       └─ 본질 변화 ✗ → 2 단계
2. v9b: 두번째 처방 옵션 1 (target 거리 제한) 도입
   └─ 결과 보고:
       └─ early shot 학습 ✓ → 종료
       └─ ✗ → 3 단계
3. v10 또는 fresh start: Issue #027 (payload tracking) + 환경 변경
   └─ 큰 변경 but 사용자 의도 100%
```

---

## 6. 보상 구조 (v9a 활성, 변경 후보 highlight)

```
Drop 보상 (Layer 4, 1회만):
  reward = drop_attempt_bonus * exp(-k_drop_proximity * d_xy)
         + w_drop_base * exp(-k2_precision * d_error)
         + w_prediction * exp(-k_prediction * |d_impact - d_error|)   ← 활성 가능 (현재 0)
         + r_success_jackpot * [d_error ≤ jackpot_threshold]
         - alt_penalty (sigmoid bounded)
         - penalty_instability (omega 또는 tilt 초과 시)
         - drop_angaccel_penalty_scale * max_ang_accel    ← v9a NEW
         - invalid_drop_penalty * [d_error > invalid_drop_threshold]  ← v8 = 0

Per-step 보상 (Layer 2+3):
  R = -w_time
    - w_ang_vel * ||omega||²
    - w_action_smooth * ||Δa||²
    + w_dist * (d_prev - d_now)              ← v9a: 1.0 → 1.5
    + w_heading * cos(heading) * speed_gate
    + w_impact * exp(-k_impact * d_impact)
```

| 파라미터 | v9a 활성 값 | 변경 후보 (다음 처방) |
|---|---|---|
| `auto_drop_threshold` | 2.0 | 0.5 ~ 1.0 (옵션 C 의 일부) |
| `w_dist` | 1.5 | ?  |
| `w_prediction` | 0.0 | 5.0 (CCIP 정확도 학습 강화) |
| `drop_angaccel_penalty_scale` | 0.5 | 유지 |
| `target_distance_min` (NEW) | — | 1.0 ~ 2.0 (옵션 B) |
| `desired_drop_xy` (NEW) | — | 좌표 (옵션 C) |

---

## 7. 환경 (v9a 활성)

| 요소 | 현재 | 변경 후보 |
|---|---|---|
| target_enu | (4, 3) 고정 | randomization (옵션 E) |
| spawn pose | (0, 0, 0.24) 고정 | randomization (옵션 E) |
| cruise 명령 | NW (1, -1) 자동 | 비활성 (옵션 E) |
| detach 후 ep 종료 | 즉시 | landing 까지 연장 (옵션 D, Issue #027) |

---

## 8. 결정 이력

- 2026-06-22: v9 처방 1 (drop_angaccel) + 처방 2 (payload distance via w_dist 1.5) 결정 → v9a 시작
- 2026-06-26: v9a 17k 진행 → SIGTERM stop (사용자 결정)
- 2026-06-27: v9a 평가 + 분석 → 사용자 "별도 trajectory 새 모델" 결정
- 2026-06-27: 다음 처방 후보 정리 (이 문서). 사용자 결정 대기.
- 2026-06-27: 사용자가 `local/thoughts` 작성 → 2 단계 모드 분리 framework → plan 작성 (옵션 G 추가, 큰 변경이라 issues 가 아닌 design 폴더만)
