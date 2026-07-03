# Parameter 조절 의사결정 Guide [DEPRECATED]

> ⚠ **DEPRECATED (2026-05-23)**: 이 문서는 junsang N1=B v2 시절 (5/22) 작성됨. 그 reward 설계 (`w_impact=8` 등) 는 critic 발산으로 폐기됨.
> **현재 가이드**: `/home/juns/local_dronebombard_simulation/design_review_2026-05-23.md` (Tier 1/2/3 처방 매트릭스 + § 2A/2B 의 RL 설계 원칙 + 학습 진행 자동 판단 기준)
>
> 보존 이유: One-lever-at-a-time 원칙, 3D 진단 매트릭스의 아이디어는 design_review 에 흡수됨. 역사적 참고용.
>
> ---
>
> 작성 목적: 200k B v2 학습 종료 후 wandb 지표를 보고 "어떤 parameter 를 어떤 방향으로 조절할지" 명확한 절차를 제공.
> 대상 parameter: B 의 3개 (`w_impact`, `k_impact`, `speed_gate`) + A 의 1개 (`auto_drop_threshold`) + terminal precision (`k2_precision`, `w_drop_base`).
> 핵심 원칙: One-lever-at-a-time. A 와 B 동시 조절 금지.
> 관련 문서: `~/A_phased_curriculum_도입방안.md`, `~/Downloads/N1_drop_sparse_AB방안.md`
> 작성 기준: 2026-05-22
>
> 변경 이력:
>   - v1 (2026-05-22): 초안
>   - v2 (2026-05-22): 8k step 실측 데이터 반영 — drop_error (정밀도) 차원 추가, "B 추가 학습 +100k" 옵션 신설,
>     env/total_drop_count (누적) 카운터 추가 반영. §3 매트릭스 2D→3D, §5 분리(5-1/5-2/5-3), §6 흐름도 갱신.

---

## 0. 한 줄 결론

> **B 와 A 는 *경쟁* 이 아니라 *서로 다른 차원* 의 문제를 다룬다. 증상이 "*어떻게* 비행하는지" 의 문제면 B, "*언제* 발사하는지" 의 문제면 A. 한 학습 사이클엔 한 parameter 만 변경. 어느 차원의 문제인지 먼저 진단하고, 그 차원 안에서 어떤 parameter 인지 결정.**

---

## 1. 핵심 원칙: One-Lever-At-A-Time

### 왜 필요한가

- 두 parameter 를 동시에 바꾸면 어느 것이 효과 / 부작용 냈는지 분리 불가 (**attribution 상실**)
- 한 사이클 결과 보고 다음 사이클 결정해야 하는데, 데이터가 흐려져 다음 결정도 어려움
- 디버깅이 기하급수적으로 어려워짐

### A 와 B 의 동시 변경이 특히 문제인 이유

B 는 reward shaping, A 는 environment dynamics 변경.
- 둘 다 drop_count 에 영향 미침
- 둘 다 켜면 drop_count 변화가 어디서 왔는지 모름
- 한 쪽이 정 (positive), 다른 쪽이 부 (negative) 효과면 net 만 보고 둘 다 좋다고 오해 가능

→ **B 변경 사이클 후, 다음 사이클은 A 만. A 변경 사이클 후, 다음 사이클은 B 만 (필요하면).**

---

## 2. Parameter 들의 역할 (정확히 무엇을 결정하는가)

### B 의 3 parameter

#### `w_impact` (현재 8.0)
- 식: `r3_impact = (w_impact) * exp(-k * d) * speed_gate`
- **역할:** r3_impact 의 *절대 크기*. 다른 reward 항(rew_orient, rew_dist 등) 대비 *상대적 중요도* 결정.
- **곡선 변화:** 세로축 단순 스케일. 모양은 그대로.
- **언제 만질까:** r3_impact 가 *전체적으로* 너무 작아 다른 신호에 묻힐 때 ↑, 또는 너무 커서 terminal 학습 묻을 때 ↓.

#### `k_impact` (현재 0.03)
- 식: `r3_impact = w * exp(-(k_impact) * d) * speed_gate`
- **역할:** 보상 곡선의 *모양* (감쇠 속도). 작을수록 평평, 클수록 가파름.
- **곡선 변화:** k=0.03 (현재): 80m→0.73, 10m→5.93. k=0.05: 80m→0.09, 10m→3.03. k=0.025: 80m→1.35, 10m→7.79.
- **언제 만질까:** *멀리서* 의 신호 부족이면 ↓ (평평하게), *가까이서* 신호 부족이면 ↑ (가파르게 + w 같이 키움).

#### `speed_gate` (현재 적용됨)
- 식: `r3_impact = w * exp(-k * d) * (speed_gate)`
- speed_gate = min(speed_xy / 2.0, 1.0)
- **역할:** loitering hack 차단 안전장치. 정지 시 r3_impact = 0.
- **언제 만질까:** 거의 안 만짐. 만약 drone 이 *너무 빨리* 학습하면 speed gate threshold 를 2 → 3 으로 올려 더 강하게 강제할 수도 있지만, 일반적으로 default 유지.

### A 의 1 parameter

#### `auto_drop_threshold` (현재 2.0m)
- 위치: `drone_drop_env.py:575` 부근 `d_impact <= self._cfg_auto_drop_threshold`
- **역할:** drop 이 *언제* 발사될지 결정. d_impact 가 이 값 이하일 때만 CCIP 가 drop 발사.
- **변화 효과:** 풀수록 drop 빈도 ↑ (정확도는 떨어짐). 조일수록 drop 빈도 ↓ (정확도 ↑).
- **언제 만질까:** drop_count 가 직접적 문제일 때만. *비행 자체*가 문제면 B 로 먼저 해결.

---

## 3. 진단: 어느 차원의 문제인가

200k 학습 끝나면 wandb 에서 다음 **5개 지표** 확인 (이전 4개 + drop_error 정밀도):

```
지표 (학습 후반 평균):
  1. env/mean_d_impact          : CCIP 예측 거리 (드론이 얼마나 가까이 가나)
  2. env/total_drop_count       : 누적 drop 횟수 (NEW — 누적 그래프 stair-step)
     env/drop_count             : per-rollout drop 횟수 (기존, instantaneous rate 용)
  3. env/drop_error_actual_m    : 떨어뜨린 경우의 명중 오차 (정밀도)
  4. env/mean_rew_impact        : r3_impact 평균 (signal 흐름 검증)
  5. env/success_rate           : drop 한 경우 중 정확도 0.5m 이내 비율
```

### 3D 진단 매트릭스 (drop_error 차원 추가)

문제를 3가지 차원으로 나눠 봄:
- **거리 차원** (`mean_d_impact`): 드론이 가까이 가는가?
- **발사 차원** (`drop_count` / `total_drop_count`): 떨어뜨리는가?
- **정밀도 차원** (`drop_error_actual_m`): 정확하게 떨어뜨리는가? ← *NEW*

| `d_impact` | `drop_count` | `drop_error` | 진단 | 조절 차원 |
|---|---|---|---|---|
| **< 10m** | ≥ 100 | < 5m | **완전 성공** | ✓ 종료 / 미세조정 |
| **< 10m** | ≥ 100 | 5~15m | **발사는 되는데 정밀도 부족** | ★ **정밀도 차원** (NEW) |
| **< 10m** | ≥ 100 | > 15m | 거의 우연 명중 수준 | ★ **정밀도 차원** (NEW) |
| **< 10m** | 30~100 | — | 잘 가는데 가끔 발사 | A (single-step) |
| **< 10m** | < 30 | — | 잘 가는데 거의 안 발사 | A (phased) |
| **10~30m** | ≥ 100 | — | 약간 멀리서 떨어뜨림 | A threshold ↓ + 정밀도 |
| **10~30m** | 30~100 | — | 어느 정도 학습됨 | **B 추가 학습 +100k** (NEW) |
| **10~30m** | < 30 | — | 어느 정도 가지만 드물게 발사 | A 우선, B 보조 |
| **> 30m** | < 30 | — | 멀리서만 비행 | B 강화 (k 또는 w) |
| **< 10m** | 0 | — | drive-through hack 의심 | B 약화 (w ↓) |

### 핵심 3차원 판정 룰

```
[1] d_impact 차원 먼저:
    > 30m  → B 의 문제 (비행 자체)
    ≤ 30m  → 다음 차원으로

[2] 그 다음 drop_count 차원:
    < 30   → A 의 문제 (발사 조건)
    ≥ 30   → 다음 차원으로

[3] 마지막 drop_error 차원 (NEW):
    < 5m   → 성공
    5~15m  → 정밀도 차원 (B 추가 학습 또는 terminal reward 조정)
    > 15m  → 우연 명중, 정밀도 학습 본격 필요
```

### Drop 빈도 외삽 룰

학습 도중 drop 추세 확인은 `env/total_drop_count` 의 *기울기* 로:

```
8k step 시점에 total_drop_count = 7 → 기울기 ≈ 0.875 /1000step
200k 외삽: 0.875 × 200 = 175 drops
```

기울기가 시간 따라 **증가** 하면 → 진짜 학습 가속 (Pattern C, 더 많은 drop 기대)
기울기가 **일정** 하면 → 안정 상태 (외삽 신뢰 가능)
기울기가 **감소** 하면 → 정책이 보수적으로 변함 / hack 의심

---

## 4. B 차원: B 의 3 parameter 중 어느 것을 조절할까

B 의 문제로 진단됐다면 다음 sub-진단:

### Sub-진단: wandb 의 distance bin 별 r3_impact 분포

학습 후반에 d_impact 가 어느 범위에서 머무는지 + 그 거리의 r3_impact 값:

```
d_impact ≈ 80m  → r3_impact = 0.73 (현재 w=8, k=0.03 기준)
d_impact ≈ 50m  → r3_impact = 1.78
d_impact ≈ 30m  → r3_impact = 3.25
d_impact ≈ 10m  → r3_impact = 5.93
d_impact ≈ 2m   → r3_impact = 7.53
```

(현재 값 — 학습 후 데이터 보고 비교)

### B 의 3가지 처방

| 증상 | 가설 | 처방 | 구체 변경 |
|---|---|---|---|
| 드론이 80m+ 에서만 머묾 | 장거리 신호 약함 | `k_impact` ↓ (평평하게) | 0.03 → 0.02 (또는 0.015) |
| 30m 안에 들어옴, 더 못 들어옴 | 가까이서 신호 부족 | `w_impact` ↑ | 8 → 12 |
| 10m 부근에서 머묾, 안 떨어뜨림 | 비행 학습 충분, drop 못 함 | **B 가 아닌 A 의 영역** | A 도입 (3장 진단표) |
| d_impact 작은데 drop 0 (hack) | 신호 너무 강함, drive-through | `w_impact` ↓ | 8 → 5 (보수적으로) |
| 과속 다발, safety_violation ↑ | r3_impact 의 속도 보상이 과함 | `w_impact` ↓ | 8 → 6 |

핵심 분류:
- **장거리 신호** 부족 → `k_impact` 변경
- **전체 비중** 문제 → `w_impact` 변경
- **안전장치 부족** → `speed_gate` 검토 (거의 안 만짐)

---

## 5. A 차원 / 정밀도 차원 / B 추가 학습 — 각 처방

### 5-1. A 차원: A 의 phase 어떻게 시작할까

A 의 문제로 진단됐다면 `A_phased_curriculum_도입방안.md` 의 §5 참조. 요약:

| `drop_count` | A 도입 형태 | 시작 threshold |
|---|---|---|
| ≥ 100 | A 보류 | — |
| 30~100 | Single-step | 5.0m |
| < 30 | Phased (3 phase) | 10.0m → 5.0m → 2.0m |

→ A 도입 후 다음 학습 사이클은 **A 만 조절** (B 는 그대로). Phase 끝나면 결과 보고 다음 phase 의 threshold 결정.

### 5-2. 정밀도 차원 (NEW): drop_error 가 큰 경우

**진단 결과**: `drop_count ≥ 100` 인데 `drop_error_actual_m` 가 5m+ 인 경우.

발사는 잘 되는데 정확도가 부족. 이건 A 도 B 도 아닌 *terminal reward 의 영역*.

**3가지 옵션:**

| 옵션 | 변경 | 효과 |
|---|---|---|
| **(a) B 추가 학습** | yaml 변경 없음, 추가 100k 학습 | 가장 보수적. 학습 시간 더 주면 자연스럽게 정밀도 ↑ 기대 |
| **(b) `k2_precision` ↑** | 5.0 → 8.0 또는 10.0 | terminal drop reward 의 정밀도 보상 가파르게. 가까이 떨어뜨릴수록 점수 급증 |
| **(c) `w_drop_base` ↑** | 20.0 → 30.0 | terminal 정밀도 보상 *비중* 자체 증가. 단 H1 (terminal scale 축소) 의도와 충돌 가능 |

**선택 가이드:**
- drop_count 충분히 높고 (≥ 100) drop_error 5~10m → **(a)** 먼저 (학습 시간 더 줌)
- drop_count 충분하고 drop_error 10m+ 정체 → **(b)** k2_precision 키움
- drop_count 적당하고 drop_error 큼 → **(b)** 가 가성비 좋음
- (c) 는 마지막 보루 — H1 결정과 충돌해서 critic 학습 흔들 위험

**(a) B 추가 학습의 실행:**
```bash
# yaml 변경 없음
# 200k model 에서 resume 하되, training.total_timesteps 만 100000 추가
# 또는 새 fresh start 300k
ros2 run rl_navigation train_sac \
  --resume /workspace/ros2_ws/rl_checkpoints/sac_drop_final.zip
```

비용: +100k step (~6시간 wall-clock)

### 5-3. B 강화 또는 B 약화 (이미 §4 에 있음)

`d_impact > 30m` 또는 drive-through hack 의심 시 → §4 의 B 처방 표 참조.

---

## 6. 전체 의사결정 흐름 (정밀도 차원 추가)

```
[200k B v2 종료]
       │
       ▼
  wandb 지표 5개 확인
  (mean_d_impact, total_drop_count, drop_count, drop_error, mean_rew_impact)
       │
       ▼
  [§3 3D 진단 매트릭스] — 어느 차원의 문제?
       │
   ┌───┼───────────┬───────────────┐
   ▼   ▼           ▼               ▼
[완전성공] [정밀도]  [B 차원]      [A 차원]
   │       │       │               │
   │       ▼       ▼               ▼
   │   [§5-2]    [§4 sub-진단]  [§5-1 phase 결정]
   │   (a)+학습 → w 또는 k     → threshold 값
   │   (b)k2↑    (B 만 변경)     (A 만 변경)
   │       │       │               │
   │       └───┬───┴───────────────┘
   │           │
   │           ▼
   │   결과 보고 다시 [§3 진단]
   │           │
   │           └──────────────────► (반복)
   │
   ▼
[종료]


의사결정 룰 (정리):
   d_impact > 30m              → B 강화 (k↓ 또는 w↑)
   d_impact 작음, drop 적음    → A 도입 (single 또는 phased)
   drop 많고 d_impact 작음:
     drop_error < 5m           → 완전 성공
     drop_error 5~15m          → 정밀도 차원 (B 추가 학습 또는 k2_precision↑)
     drop_error > 15m          → 정밀도 차원 (k2_precision 또는 w_drop_base↑)
   d_impact 작은데 drop = 0    → drive-through hack (w_impact↓)
```

---

## 7. 변경의 예상 효과 + 위험 시그널 (체크리스트)

각 parameter 변경 후 학습 도중 보는 시그널:

### `w_impact` ↑ 했을 때
- ✓ 정상: mean_rew_impact 증가, mean_d_impact 감소 추세
- ⚠ 위험: drive-through hack 의 시그널 (d_impact 작은데 drop_count 안 늚), 과속 다발 (safety_violation_rate ↑), terminal 학습 묻힘 (critic_loss 폭주)

### `w_impact` ↓ 했을 때
- ✓ 정상: terminal reward 학습 회복 (drop_count ↑), 과속 감소
- ⚠ 위험: r3_impact 가 너무 약해 정책이 코스 학습 안 함 → mean_d_impact 정체

### `k_impact` ↓ 했을 때 (평평하게)
- ✓ 정상: 학습 초반부터 mean_d_impact 빠르게 감소 (장거리 신호 작동)
- ⚠ 위험: 멀리서도 점수가 충분히 크면 *가까이 갈 절박함* 약해질 수 있음 → 가까이서 효과 감소

### `k_impact` ↑ 했을 때 (가파르게)
- ✓ 정상: 가까이서 학습 가속 (이미 가까이 갔다면)
- ⚠ 위험: 멀리서 신호 죽음. 멀리서 학습이 r3_dist 만으론 부족할 수도

### `auto_drop_threshold` ↑ 했을 때 (풀어줌)
- ✓ 정상: drop_count 즉시 증가, terminal reward 학습 시작
- ⚠ 위험: drop_error 큼 (당연 — 부정확한 drop). 그러나 학습 진행하면 점진 개선되어야 함. 개선 안 되면 정책이 정확도 못 배우는 것.

### `auto_drop_threshold` ↓ 했을 때 (조임)
- ✓ 정상: drop_error 감소 (정밀도 향상)
- ⚠ 위험: drop_count 떨어짐 (정책이 새 빡센 기준 못 맞춤). 회복 안 되면 phase 회귀 필요.

---

## 8. 예시 walkthrough (구체적 수치로)

### 시나리오 X: 200k 종료 후

```
WandB 결과 (가상):
  env/mean_d_impact     : 18.5m
  env/drop_count        : 12
  env/drop_error_actual : 9.3m
  env/mean_rew_impact   : 2.1
  rollout/ep_rew_mean   : -120 (안정)
```

### Step 1: 진단

§3 매트릭스 적용:
- mean_d_impact = 18.5m → "10~30m"
- drop_count = 12 → "< 30"
- 진단: **A 우선, B 보조**

→ A 차원이 dominant.

### Step 2: A 의 phase 결정

§5 표 적용:
- drop_count = 12 < 30 → **Phased (3 phase)**
- 시작 threshold = 10.0m

(B 는 *이번 사이클에선 안 만짐*)

### Step 3: Phase 1 실행

```yaml
# yaml 변경 (one lever)
auto_drop_threshold: 2.0 → 10.0
# w_impact, k_impact 은 그대로 유지
```

학습 50k 진행.

### Step 4: Phase 1 결과 확인

```
WandB 결과:
  env/mean_d_impact     : 12m   (전보다 작아짐 — A 가 가까이 안 와도 drop 가능하게 한 효과)
  env/drop_count        : 380   (200k 대비 32배 증가)
  env/drop_error_actual : 6.8m  (drop 정확도 적정)
  rollout/ep_rew_mean   : -85   (개선)
```

→ 정상. Phase 2 로.

### Step 5: Phase 2 실행

```yaml
auto_drop_threshold: 10.0 → 5.0
learning_starts: 1000 → 500
# w_impact, k_impact 그대로
```

학습 50k.

### Step 6: 만약 Phase 2 결과 나빠지면

```
WandB 결과:
  env/mean_d_impact     : 14m   (조금 증가)
  env/drop_count        : 95    (Phase 1 의 1/4)
  env/drop_error_actual : 4.2m  (정확도는 좋아짐)
  rollout/ep_rew_mean   : -110  (소폭 악화)
```

해석: drop_count 떨어짐 = "정책이 5m 기준에 못 맞춤". 두 선택:
- (a) Phase 2 더 길게 (+50k) — 적응 시간 줌
- (b) Phase 1 model 로 회귀, threshold 7.0 으로 재설계

선택 기준: ep_rew_mean 추세. 회복 추세면 (a), 정체면 (b).

→ *여전히 B 는 안 만짐*. A 의 문제는 A 로 해결.

### Step 7: 만약 Phase 3 후에도 drop_count 회복 안 하면

```
WandB 결과 (Phase 3 끝):
  env/mean_d_impact     : 11m
  env/drop_count        : 50
  env/drop_error_actual : 7.0m
```

해석: 본래 threshold (2m) 에 정책이 못 맞춤. drop 가능한 거리(11m)와 발사 거리(2m) 사이 9m 갭이 남음.

이건 진짜 *B 의 영역* — "가까이 못 감". 이제 B 조절:
- §4 표: "10m 부근에서 머묾, 더 못 들어옴" → `w_impact` ↑ (8 → 12)
- 또는 `k_impact` ↑ (0.03 → 0.05) — 가까이서 신호 더 가파르게

이제 **B 만 변경**. A 는 그대로 (threshold=2.0).

---

## 9. 절대 하지 말 것

| 안 됨 | 왜 |
|---|---|
| 같은 사이클에 B 의 w 와 k 동시 변경 | 어느 게 효과 냈는지 모름 |
| 같은 사이클에 A 와 B 동시 변경 | 더더욱 분리 불가 |
| Phase 도중 yaml 수정 (학습 안 멈추고) | 비정상성 + buffer 오염 |
| Phase 결과 안 보고 다음 phase 시작 | adaptive 의미 사라짐 |
| speed_gate 끄기 | loitering hack 다시 위험 (특별한 이유 없으면) |

---

## 10. 한눈 표 (가장 압축된 가이드)

```
┌──────────────────────────────────────────────────────────────────────────┐
│ 학습 끝났을 때                                                            │
│   wandb 보고 진단 (3차원):                                                │
│                                                                          │
│   [B 차원]   d_impact > 30m                                              │
│              → 멀리서 신호 약함  : k_impact ↓ (0.03 → 0.02)             │
│              → 전체 비중 부족    : w_impact ↑ (8 → 12)                  │
│              → drive-through hack: w_impact ↓ (8 → 5)                   │
│              → 과속 다발         : w_impact ↓                            │
│                                                                          │
│   [A 차원]   d_impact 작음, drop_count < 100                             │
│              → < 30  : Phased A (10m → 5m → 2m)                         │
│              → 30~100: Single-step A (threshold = 5m)                    │
│                                                                          │
│   [정밀도]   d_impact 작음, drop_count ≥ 100, drop_error 큼  (NEW)      │
│              → 5~15m : B 추가 학습 +100k 또는 k2_precision ↑            │
│              → > 15m : k2_precision ↑ + w_drop_base 검토                │
│                                                                          │
│   [성공]    d_impact < 10m, drop_count ≥ 100, drop_error < 5m           │
│              → 종료 또는 정밀도 미세조정                                 │
│                                                                          │
│ 한 사이클엔 한 parameter 만 변경. A 와 B 절대 동시 X.                    │
│ 결과 보고 다시 진단. 반복.                                               │
└──────────────────────────────────────────────────────────────────────────┘
```

---

## 11. 관련 문서 / 메모리

- `~/A_phased_curriculum_도입방안.md` — A 도입의 phase 운영 상세 (이 문서의 §5 와 연결)
- `~/Downloads/N1_drop_sparse_AB방안.md` — A vs B 원래 분석
- `~/meeting_notes_2026-05-22.txt` — N1=B v2 결정 기록
- `~/.claude/projects/-home-juns/memory/project_n1b_200k_training.md` — 현재 학습 상태
- `~/.claude/projects/-home-juns/memory/feedback_train_sac_graceful_kill.md` — phase 전환 시 SIGINT 사용

---

================================================================================
 끝 — 다음 단계: 200k B v2 종료 시 §3 진단 매트릭스로 시작
================================================================================
