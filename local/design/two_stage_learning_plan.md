# Two-Stage Learning Plan — 접근 모드 + 투하 모드

> **배경**: 사용자 `thoughts` 정리 — v8 의 toss 단일 strategy + v9a fine-tune 미달성 → 2 단계 모드 분리 + 확률적 융합 framework 제안

> **위치 결정**: 너무 큰 변경 (env state machine refactoring, ~10 일 학습) → 안건 관리 (issues/) 가 아닌 설계 plan (design/) 에 두기로 결정 (2026-06-27).

> **원본**: [../thoughts](../thoughts)
> **분석 (장단점)**: [../archive/issue_028_two_stage_approach_drop_modes.md](../archive/issue_028_two_stage_approach_drop_modes.md) — 처음 issue 로 등록했다가 design 으로 이전, 분석 자료 보존
> **연관**:
>   - [../issues/issue_026_toss_environment_indistinguishable.md](../issues/issue_026_toss_environment_indistinguishable.md) — root motivation
>   - [../issues/issue_027_payload_tracking_after_detach.md](../issues/issue_027_payload_tracking_after_detach.md) — 단계 2 의 drop 정확도
>   - [../issues/issue_018_vision_based_observation.txt](../issues/issue_018_vision_based_observation.txt) — 상대좌표 prerequisite

작성: 2026-06-27 (사용자 thoughts 기반)
관련:
- `local/parameter_log.md` — 새 hyperparams 후보
- `local/meeting_notes/meeting_notes_2026-06-27.txt` — 결정 history

---

## 1. 핵심 framework (사용자 thoughts 5 가지)

### 1-1. 상대좌표
- 카메라로 인지한 target 과의 상대좌표 사용
- 매 step 마다 새 target 좌표 업데이트
- 절대좌표 (4, 3) 의 한계 해결

### 1-2. 2 단계 모드 분리

| 모드 | 전환 조건 | reward | action 제약 |
|---|---|---|---|
| **1. 접근 모드** | 시작 | 거리 감소 (r3_dist) + 제약 만족 reward | 최대속도, 최종속도, 최대각속도, 상대속도 방향 |
| **2. 투하 모드** | 거리 안 진입 (`x² + y² + z² < R`) | drop 정확도 + 시간 제한 + 자세 비틀 정도 minimization | 최대 각속도, 각가속도, 시간 |

### 1-3. 학습 단계

```
Phase B — 단계 1 학습 (다양 접근 모델 풀 생성)
   ↓
Phase C — 단계 1 의 한 모델 선택 + 단계 2 학습
   ↓ (성공 시)
Phase D — 단계 1 확장 (확률적 mixing) + 단계 2 재학습
   ↓
Phase E — 평가 + 융합 안정성 확인
```

### 1-4. 확률적 융합 (사용자 제안)
- 단계 1 의 두 모델을 50:50 으로 선택
- 단계 2 학습 시 처음 헤맴 → 점진 우수 학습 기대
- 안 되면 단계 2 실패 시 단계 2 비중 높임 (제한)

### 1-5. 자각 우려
- 단계 1 → 2 reward 체계 급변 영향 (작을 것 추정)
- 융합 시 정책 문제
- toss 의 불확실성 가능

---

## 2. 단계별 detailed plan

### Phase A — 설계 (1-2 일)

#### A-1. env state machine

```python
class DroneDropEnv:
    def __init__(self):
        self.mode = "approach"      # "approach" | "drop"
        self.cfg_approach_radius = R   # 모드 전환 거리

    def step(self, action):
        # 거리 계산
        d = ||drone - target||

        # 모드 전환 (approach → drop)
        if self.mode == "approach" and d < self.cfg_approach_radius:
            self.mode = "drop"
            info['mode_transition'] = True

        # 모드 별 reward 계산
        if self.mode == "approach":
            reward = self._compute_approach_reward(...)
        else:
            reward = self._compute_drop_reward(...)
```

#### A-2. Reward 분리

**접근 모드 (단계 1)**:
```
r_approach = w_dist * (d_prev - d_now)              # 거리 감소
           + w_speed_max * [v_max - speed]+          # 최대속도 미달 페널티
           + w_speed_final * [speed_at_R - target]  # 최종속도 매칭 (R 도달 시)
           + w_angvel_max * [omega_max - |ω|]+      # 최대각속도 제약
           + w_dir * cos(velocity, target_dir)        # 상대속도 방향
```

**투하 모드 (단계 2)**:
```
r_drop = w_predict * exp(-k * |CCIP - target|)     # 예상 투하지점 정확도
       - w_time * t_in_drop_mode                    # 시간 제한
       - w_angvel * max(|ω|)                        # 각속도 최소화
       - w_angaccel * max(|d/dt ω|)                 # 각가속도 최소화 (v9a 와 동일)
       + w_drop_base * exp(-k * d_error)            # 실제 drop 정확도 (terminal)
```

#### A-3. R (전환 거리) 결정

| R 후보 | 의미 | trade-off |
|---|---|---|
| 1.0m | 매우 정밀 | 단계 1 학습 어려움 |
| 2.0m | balance | 추천 시작 |
| 3.0m | 여유 | 단계 2 의 dynamics 풍부 |

### Phase B — 단계 1 학습 (다양 접근 모델 풀, 1-2 일)

**run_name**: `phase1_redux_v10a_approach_only`

**hyperparams 변경**:
```yaml
environment.mode_two_stage: true   # NEW
environment.approach_radius: 2.0   # NEW

reward.drop_disabled: true         # NEW — drop 모드 시 reward 0 (단계 1 만)
reward.w_speed_max: ?              # NEW
reward.w_speed_final: ?            # NEW
reward.w_angvel_max: ?             # NEW

# 단계 1 의 entropy 강화 (다양 접근 학습)
sac.target_entropy: -15 → -10      # 더 많은 exploration
```

**학습**:
- Fresh start (v8 prior 없이)
- ~200k step
- success metric: target 거리 R 도달 ratio

**결과 평가**:
- 정책의 다양성 (deterministic 모드에서 여러 trajectory 가능)
- 거리 R 도달 시점의 vel 분포 (다양해야 좋음)
- 시간 분포

### Phase C — 단계 1 의 한 모델 선택 + 단계 2 학습 (1-2 일)

**선택 기준** (가장 직관적):
- 단계 1 모델 중 "marker 향해 직진 + 적당한 vel" 모델

**run_name**: `phase1_redux_v10b_drop_mode`

**hyperparams 변경**:
```yaml
reward.drop_disabled: false        # drop 모드 reward 활성
reward.w_predict: 10.0             # 예상 투하지점 정확도
reward.w_time: 0.1                 # 시간 제한
reward.w_angvel: 0.2               # 각속도 최소화
reward.w_angaccel: 0.5             # 각가속도 최소화 (v9a 와 동일)
```

**학습 방법**:
- Resume from Phase B 의 선택 모델
- 또는 weights 고정 (단계 1 freeze) + 단계 2 만 학습
- ~100k step

**평가**:
- 단계 2 의 drop 정확도
- toss 회피 여부 (사용자 의도 핵심)

### Phase D — 융합 (단계 1 풀 확장)

**run_name**: `phase1_redux_v10c_ensemble`

**확장 방법** (사용자 제안):
- Phase B 의 단계 1 풀에서 2 개 모델 선택
- 매 ep 시작 시 50:50 으로 선택 → 단계 1 진행
- 단계 2 는 같은 정책 → 다양 단계 1 에 대해 학습

**Fallback** (단계 2 실패 시):
```python
if success_rate < threshold:
    p_stage2 = min(1.0, p_stage2 * 1.1)   # 단계 2 비중 점진 증가
```

### Phase E — 평가 + 분석

- dgui 로 정책 행동 시각 검증
- 사용자 의도 (지나치는 현상 해결) 검증
- 다양 trajectory 확인 (Phase B 의 단계 1 풀 효과)

---

## 3. Prerequisite

### 3-1. Vision-based obs (Issue #018)

상대좌표 의도면 카메라 가 필요:
- `x_marker_world_vision.sdf` (camera + Sensors 플러그인 포함)
- `use_vision: true`
- env obs 의 vision channel (pixel_coords) 활성

**대안 (임시)**:
- 일단 절대좌표 (4, 3) 로 시작 → 상대좌표 변환 추가
- 카메라 없이도 framework 검증 가능

### 3-2. env state machine refactoring

`drone_drop_env.py` 의 큰 변경 필요:
- `_compute_reward()` 분리 → `_compute_approach_reward()` + `_compute_drop_reward()`
- step() 의 mode 관리
- 모드 별 termination 조건

### 3-3. train_sac.py 확장

- 단계 1 weights freeze 옵션 (Phase C 의 단순 단계 2 학습)
- ensemble policy (Phase D 의 확률적 mixing)

---

## 4. 위험 / 우려

### 4-1. 구현 복잡성 (큼)

| 항목 | 영향 |
|---|---|
| env state machine | drone_drop_env.py 200+ 줄 변경 |
| Reward 분리 | reward weights 새로 결정 (실험적) |
| 단계 1 풀 관리 | 모델 저장 + 로드 logic |
| Ensemble policy | train_sac.py 확장 |

### 4-2. 학습 시간 (큼)

- Phase B: 200k step (~30시간)
- Phase C: 100k step (~15시간)
- Phase D: 200k step (~30시간)
- 총: 75시간+

### 4-3. 융합 안정성

- 단계 1 의 두 모델 사이 transition smooth 필수
- 확률적 mixing 시 정책 발산 위험

### 4-4. 사용자 자각 우려 (분석)

| 우려 | 해석 |
|---|---|
| 단계 1 → 2 reward 급변 | 단계 1 영역에서는 drop reward = 0 이므로 영향 거의 없음 (사용자 추측 동의) |
| 융합 시 정책 문제 | 점진적 확률 증가로 catastrophic forgetting 회피 가능 |
| toss 불확실성 | 단계 분리 + 단계 2 의 ang_vel/각가속도 minimization 으로 부드러운 drop |

---

## 5. 단순화 alternative — 작은 처방으로 시작

이 plan 이 큰 변경이라 너무 위험하면, **단순 처방으로 framework 의 일부 검증** 가능:

### Alt-1. Issue #028 의 단계 1 거리 R 만 적용

```python
# drone_drop_env.py 의 step() 안:
if d_xy < self._cfg_approach_radius and not self._approach_done:
    self._approach_done = True
    info['approach_done'] = True
# 그 외 v9a 와 동일
```

- 가장 작은 변경
- 사용자 의도 부분 검증
- 결과 보고 thoughts framework 전체로 확장

### Alt-2. 사용자 처방 후보 1 (target 거리 제한) — 이전 결정

- drone 이 target 가까이 가면 penalty
- → early shot 강제 학습
- thoughts framework 와 일관 (단계 1 → 2 전환 거리 의 다른 표현)

---

## 6. 추천 진행 순서

| Phase | 의미 | 시간 |
|---|---|---|
| **0** | 사용자 결정 (어느 alt 또는 full thoughts framework) | — |
| **A** | 설계 + env state machine 구현 | 1-2 일 |
| **B** | 단계 1 학습 (다양 접근 모델 풀) | 1-2 일 |
| **C** | 단계 1 의 한 모델 + 단계 2 학습 | 1-2 일 |
| **D** | 융합 (단계 1 풀 확장 + 확률적) | 2-3 일 |
| **E** | 평가 + 분석 | 0.5 일 |
| **총** | | **~10 일** |

또는 단순화 (Alt-1 또는 Alt-2):
- 1-2 일 fine-tune 으로 framework 부분 검증

---

## 7. 결정 사항 (사용자)

- [ ] full thoughts framework (Phase A~E)
- [ ] Alt-1 (단순 — 단계 1 거리 R 만)
- [ ] Alt-2 (target 거리 제한 — 이전 처방 후보 1)
- [ ] Vision 환경 (Issue #018) 먼저
- [ ] v9a 100k 까지 더 학습 (Issue #025) 먼저
- [ ] 다른 방향

(사용자 결정 후 진행)
