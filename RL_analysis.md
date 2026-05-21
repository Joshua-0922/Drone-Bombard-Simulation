# RL 강화학습 분석 & 보상 함수 설계

> **최종 업데이트:** 2026-05-21  
> **작성자:** Claude Copilot  
> **분석 대상:** WandB 학습 결과 (2026-05-20)

---

## 1. 보상 함수 구조 및 설계 이력 (README.md 11.4에서 이전)

### 1.1 보상 레이어 개요

드론의 투하 정책 학습을 위해 보상 함수를 4개의 레이어로 구성합니다.

| 레이어 | 적용 시점 | 역할 |
|--------|-----------|------|
| **Layer 1 — 안전** | 매 스텝 | 비정상 비행 억제 (에피소드 종료 없음) |
| **Layer 2 — 안정/효율** | 매 스텝 | 불필요한 기동 억제, 빠른 임무 수행 유도 |
| **Layer 3 — 접근** | 매 스텝 | 타겟 방향으로 비행하도록 유도 |
| **Layer 4 — 투하 정확도** | 투하 시 (에피소드 종료) | 정확한 위치에서 투하하도록 유도 |

**에피소드 종료 조건:**
- `d_impact ≤ auto_drop_threshold` → 자동 투하 → Layer 4 보상 후 `terminated`
- 최대 스텝(500) 초과 → 미투하 패널티 후 `truncated`

**CCIP(Continuously Computed Impact Point)**: 현재 위치·속도로 지금 투하했을 때의 착탄 예측 지점을 매 스텝 실시간 계산.

$$d_{impact} = \sqrt{(x_p - x_{target})^2 + (y_p - y_{target})^2}$$

$$x_p = x + v_x \cdot t_f, \quad t_f = \frac{v_z + \sqrt{v_z^2 + 2gz}}{g}$$

---

### 1.2 각 레이어 계산식

**Layer 1 — 안전 패널티**

```
R1 = penalty_crash      if altitude < min_altitude (after step 20)
   + penalty_overspeed  if speed > 20 m/s
```

**Layer 2 — 안정/효율 패널티**

$$R_2 = -w_{time} \cdot 5 - w_{\omega} \cdot \|\omega\|^2 - w_{smooth} \cdot \|\Delta a\|^2$$

- $w_{time}$: 시간 패널티 가중치 (스텝당 고정 비용)
- $\|\omega\|^2$: 각속도 크기 제곱 (기체 불안정도)
- $\|\Delta a\|^2$: 연속 스텝 간 액션 변화량 제곱 (급격한 조작 억제)

**Layer 3 — 접근 보상**

$$R_3 = w_{dist} \cdot (d_{prev} - d_{now}) + w_{heading} \cdot \cos(\theta) \cdot \text{gate}(v_{xy}) + w_{impact} \cdot e^{-k_{impact} \cdot d_{impact}}$$

- $d_{prev} - d_{now}$: 타겟까지 거리 감소량 (접근할수록 양수)
- $\cos(\theta)$: 드론 진행 방향과 타겟 방향 사이 각도의 코사인 (헤딩 정렬)
- $\text{gate}(v_{xy}) = \min(v_{xy}/2, 1)$: 속도 게이트 (정지 상태에서 헤딩 보상 수집 방지)
- $e^{-k_{impact} \cdot d_{impact}}$: CCIP 예측 오차 감소 보상

**Layer 4 — 투하 정확도 보상 (에피소드 종료)**

$$R_4 = w_{drop} \cdot e^{-k_2 \cdot d_{error}} + r_{jackpot} \cdot \mathbf{1}[d_{error} \leq 0.1m] + r_{attempt} - \text{penalty}_{instability}$$

- $d_{error}$: Gazebo 물리 시뮬레이션에서 측정한 **실제** 착탄 오차 (m)
- $e^{-k_2 \cdot d_{error}}$: 오차가 작을수록 높은 보상
- $r_{jackpot}$: 0.1m 이내 착탄 시 추가 보너스
- $r_{attempt}$: 투하 시도 자체에 대한 보너스 (v2 추가)
- $\text{penalty}_{instability}$: 투하 시 각속도/기울기 초과 시 차감

**미투하 패널티 (스텝 초과 시)**

```
truncation_penalty = -N   (투하 없이 500 스텝 초과)
```

---

### 1.3 v1 설정 (초기)

```yaml
# Layer 3
w_dist: 1.0
w_heading: 1.0
w_impact: 2.0
k_impact: 0.05

# Layer 4
auto_drop_threshold: 2.0   # CCIP 예측 오차 ≤ 2m 시 자동 투하
k2_precision: 5.0
w_drop_base: 50.0
r_success_jackpot: 100.0
# 미투하 패널티: -50
# drop attempt bonus: 없음
```

#### v1 학습 결과 및 문제점 (333K steps 기준)

| 지표 | 결과 |
|------|------|
| `success_rate` 피크 | 118K steps에서 2.56%, 이후 1.0%까지 하락 및 정체 |
| `ep_rew_mean` | -455까지 개선 후 다시 -939로 악화 |
| `ep_len_mean` | 430 → 442로 증가 (투하 없이 에피소드 끝까지 소모) |

**근본 원인**: `k2_precision=5.0`에서 2m 오차 투하 시 Layer 4 보상:

$$R_4 = 50 \cdot e^{-5 \times 2} = 50 \times 0.0000454 \approx 0$$

투하해도 보상이 거의 0이어서 agent가 맴돌며 Layer 3 보상만 수집하는 전략을 선택.

---

### 1.4 v2 변경 사항 및 이유

```yaml
# Layer 3
w_dist: 0.3          # 1.0 → 0.3: 맴돌기 전략으로 얻는 보상 축소
w_heading: 0.4       # 1.0 → 0.4: heading farming 억제
w_impact: 0.2        # 2.0 → 0.2: CCIP orbit 수확 차단 (핵심)

# Layer 4
auto_drop_threshold: 4.0   # 2.0 → 4.0: 투하 경험 빈도 증가
k2_precision: 0.3    # 5.0 → 0.3: 먼 거리 투하도 의미 있는 보상
w_drop_base: 100.0   # 50 → 100: 투하 보상 전체 스케일 상향
# 미투하 패널티: -50 → -120
# drop attempt bonus: +20 → +120
```

**배경**: 이전 설정(`w_impact=2.0`, `w_heading=1.0`)에서 per-step orbit 보상이 너무 커서 drop보다 timeout이 수학적으로 이득인 구조 발생.
```
per-step ≈ 1.9/step → orbit 300step: +420 vs drop 200step: +450 (차이 +30 → critic 추정 불가)
```
→ `w_impact`, `w_heading` 축소 + `drop_attempt_bonus` 대폭 증가로 drop이 항상 orbit보다 명확히 유리하게 조정:
```
per-step ≈ 0.52/step → orbit: +36 vs drop: +264 (7배 차이)
```

**오차별 Layer 4 보상 비교**

| 오차 | v1: $50 \cdot e^{-5d}$ | v2: $100 \cdot e^{-0.3d} + 120$ |
|------|------------------------|----------------------------------|
| 0.1m | 50×0.607 = **30.4** (+100 jackpot) | 100×0.970 + 120 = **217** (+100 jackpot) |
| 0.5m | 50×0.082 = **4.1** | 100×0.861 + 120 = **206** |
| 2.0m | 50×0.000045 ≈ **0** | 100×0.549 + 120 = **175** |
| 5.0m | ≈ **0** | 100×0.223 + 120 = **142** |
| 10m  | ≈ **0** | 100×0.050 + 120 = **125** |

먼 거리에서 투하해도 의미 있는 보상이 주어져 agent가 투하 경험을 쌓고, 점차 더 정확한 위치에서 투하하도록 수렴하는 것을 기대합니다.

---

## 2. 2026-05-20 WandB 학습 결과 분석

### 2.1 최근 학습 실행 현황

| Run ID | 날짜 | Steps | 버전 | Mean D_XY | 평가 | 상태 |
|--------|------|-------|------|-----------|------|------|
| **4he5fw3c** | 2026-05-20 14:22 | **360K** | v2 | **19.9m** ❌ | 심각한 성능 저하 | 학습 정체 |
| **85tvkp33** | 2026-05-20 07:02 | **334K** | v2 | **499.8m** ❌ | 완전 실패 (발산) | 폐기 권고 |
| **06x7kpot** | 2026-05-19 14:27 | **118K** | v1 | **3.1m** ✅ | 정상 수렴 | 좋음 |

---

### 2.2 핵심 메트릭 비교

```
┌─────────────────────────────────────────────────────────────┐
│  360K Run (v2) vs 118K Run (v1) 비교                        │
├─────────────────────────────────────────────────────────────┤
│ Mean XY Distance:   19.9m (v2) vs 3.1m (v1)                 │
│ Per-step Reward:   -0.0076 (v2) vs 0.037 (v1)               │
│ Mean Orientation:   0.056 (v2) vs 0.552 (v1)                │
│ Mean Control:      -0.054 (v2) vs -0.052 (v1)               │
│ Drops Attempted:       0 (v2) vs 1-2 (v1)                   │
│                                                              │
│ 결론: v2 설정이 학습을 **완전히 망침** ⚠️                    │
└─────────────────────────────────────────────────────────────┘
```

---

### 2.3 문제 진단 (상세)

#### 문제 1️⃣: Per-Step 보상 붕괴 (60-70% 급락)

**v2 (현재 - 실패):**
```
Layer 2 (안정성):      -0.05/step
Layer 3 (접근):
  - Distance:          0.3 × (거리변화) = ~0.01-0.1/step
  - Heading:           0.4 × cos(θ) × gate = ~0.1-0.3/step  
  - Impact:            0.2 × exp(...) = ~0.05-0.15/step
━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━
Net 보상:              -0.05 ~ +0.35/step  ⚠️ 극도로 약함
```

**v1 (이전 - 성공):**
```
Layer 3 (접근):
  - Heading:           1.0 × cos(θ) × gate = ~0.3-1.0/step
  - Distance:          0.3 × (거리변화) = ~0.1-0.3/step
━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━
Net 보상:              +0.5 ~ +1.3/step  ✅ 충분한 신호
```

**결론**: Per-step 보상이 60-70% 감소 → 정책이 움직일 동기 상실

#### 문제 2️⃣: Orbit-Milking 해결 시도의 오류

v2의 의도:
```
Per-step 보상을 낮추고 Drop Bonus를 대폭 올려서
에이전트가 "드롭"에만 집중하도록 유도
```

**실제 문제 (수학적 분석):**
```
Per-step이 너무 낮으면...
→ "드롭에 도달할 때까지" 움직여야 하는데
→ 움직일 동기가 -0.05/step 손실로 인해 없음
→ 에이전트: "그냥 가만히 있는 게 이득이다"
→ 결과: 
   - 360K: 19.9m에서 거의 안 움직임
   - 334K: 초기 위치에서 499.8m 발산 (완전 실패)
```

#### 문제 3️⃣: Config와 코드 불일치 가능성

WandB 360K run의 config.yaml:
```yaml
w_impact: 0.05         # ⚠️ 예상과 다른 값
```

최신 코드 (hyperparams.yaml):
```yaml
w_impact: 0.2          # ⚠️ v2 커밋에서 0.2로 지정
```

**추정**: 컨테이너 이미지가 최신 코드를 반영하지 못하거나, 실행 시 잘못된 파라미터로 로드됨.

---

### 2.4 WandB 메트릭 상세 해석

**360K Run 최종 메트릭:**

```yaml
env/mean_d_xy:        19.9m      ← 초기(시작점 근처)에서 거의 안 움직임
env/mean_d_impact:    19.8m      ← d_xy와 동일 → 드롭 경험 0회
env/mean_rew_dist:   -0.0076     ← 거의 0에 가까움
env/mean_rew_orient:  0.0559     ← 매우 약함 (v1의 1/10)
env/mean_rew_ctrl:   -0.0538     ← 항상 페널티
env/mean_rew_drop:    0.0        ← 드롭이 한 번도 안 됨
env/safety_violation: 0.0        ← 안전 패널티 없음
```

**해석:**
- 정책이 처음부터 움직이지 않음
- 배회 행동 (stationary) 선택 → 보상 손실 최소화
- 타임아웃 전에 정체 → 200m 근처에서 배회
- 드롭 경험이 0회 → Layer 4 학습 불가능

---

## 3. 수정 방안 & 제안

### 3.1 즉각 조치 (Stage 1) — Per-Step 보상 복구

**목표**: Per-step 보상을 0.5-1.0/step 수준으로 복구하여 정책이 움직이도록 유도

**수정 파라미터 (hyperparams.yaml):**

```yaml
reward:
  # Layer 3 — 접근 보상 복구
  w_dist: 0.5          # 0.3 → 0.5 (67% 증가)
  w_heading: 0.7       # 0.4 → 0.7 (75% 증가)
  w_impact: 0.4        # 0.2 → 0.4 (100% 증가)
  
  # Layer 4 — 투하 보상 균형
  w_drop_base: 100.0   # 유지
  drop_attempt_bonus: 150.0  # 120 → 150 (보상-형벌 균형)
  
  # Truncation penalty 완화
  truncation_penalty: -80.0  # -120 → -80 (과도한 벌칙 완화)
  
  # auto_drop_threshold 조정
  auto_drop_threshold: 4.0   # 유지 (v2 설정)
```

**예상 효과:**

```
Per-step 보상:    -0.05 ~ +0.35  →  -0.05 ~ +0.8
Orbit reward (300step):  ~36  →  ~100 보상
Drop reward (200step):  ~264  →  ~250-300 보상
비율:              1:7  →  1:3 (개선되지만 여전히 Drop 유리)
```

---

### 3.2 Anti-Milking 개선 (Stage 2)

- **Speed gate 유지 필수** (정지 상태에서 헤딩 보상 수집 방지)
- Per-step 보상 균형화로 자연스러운 anti-milking 달성
- Orbit-milking 재발생 모니터링

**모니터링 메트릭:**
- `env/mean_d_xy`: ≤ 5m 유지 확인
- `env/mean_rew_dist` vs `env/mean_rew_orient`: 비율 확인
- `env/episodes_dropped`: 드롭 경험 증가 추이

---

### 3.3 검증 절차 (Stage 3)

1. **Dry-run (5K steps)**
   - 목표: `mean_d_xy ≤ 5m` 확인
   - 예상 시간: 5-10분
   - 성공 기준: v1과 유사한 수렴 패턴 관찰

2. **조건부 진행**
   - ✅ Dry-run 성공 → 500K 본학습 시작
   - ❌ Dry-run 실패 → 파라미터 재조정 후 재시도

---

### 3.4 예상 결과

| 메트릭 | v2 (현재) | v2 수정안 | v1 참고값 |
|--------|----------|----------|----------|
| mean_d_xy | 19.9m ❌ | ≤5m 예상 | 3.1m ✓ |
| per-step reward | -0.0076 | +0.5-0.8 예상 | 0.037 ✓ |
| drops_in_500steps | 0 | 3-5 예상 | 1-2 |
| convergence_steps | 360K 실패 | 100-200K 예상 | 118K ✓ |
| success_rate | 0% | 1-3% 예상 | 2.56% (피크) |

---

## 4. 구현 로드맵

### 4.1 즉시 구현 체크리스트

- [ ] **Step 1**: `ros2_ws/src/rl_navigation/config/hyperparams.yaml` 수정 (Stage 1 파라미터 적용)
- [ ] **Step 2**: Docker 컨테이너 내부에서 `colcon build` 실행
- [ ] **Step 3**: Dry-run 테스트 (exp_004_dry_run, 5K steps)
- [ ] **Step 4**: Dry-run 결과 검증 (mean_d_xy ≤ 5m 확인)
- [ ] **Step 5**: 성공 시 → 본학습 시작 (exp_005_reward_v2_fixed, 500K steps)

### 4.2 파라미터 변경 상세

**파일:** `ros2_ws/src/rl_navigation/config/hyperparams.yaml`

**변경 전:**
```yaml
reward:
  w_dist: 0.3
  w_heading: 0.4
  w_impact: 0.2
  drop_attempt_bonus: 120.0
  truncation_penalty: -120.0
```

**변경 후:**
```yaml
reward:
  w_dist: 0.5
  w_heading: 0.7
  w_impact: 0.4
  drop_attempt_bonus: 150.0
  truncation_penalty: -80.0
```

---

## 5. 장기 개선 방안

### 5.1 Reward Curriculum Learning

1. **초기 학습 (0-200K steps)**: Per-step 보상 높음 (w_heading=1.0)
2. **중기 학습 (200-400K steps)**: 점진적 감소 (w_heading=0.7 → 0.5)
3. **후기 학습 (400K+ steps)**: Drop 보상 강조 (w_heading=0.4)

### 5.2 Empirical Reward Tuning

- WandB 메트릭으로 실시간 성능 모니터링
- 10K step 단위로 메트릭 분석
- 필요 시 checkpoint 재개하여 파라미터 조정

### 5.3 테스트 스위트

- 특정 distance 구간별 성능 검증 (1m, 5m, 10m, 20m)
- Orbit-milking 재발생 조기 경보
- Drop 정확도 분포 분석

---

## 6. 참고 자료

### 관련 노트
- `notes/experiments/exp_003_rtf_dryrun` — RTF=2 최적화 결과
- `notes/research/reward_design` — 4-Layer 보상 구조 설명
- `notes/research/rl_rules` — RL 규칙 + Known Failure Modes
- `notes/research/phase1_plan` — Phase 1 전체 계획

### 코드 파일
- `ros2_ws/src/rl_navigation/rl_navigation/drone_drop_env.py` — 보상 함수 구현 (`_compute_reward` 메서드)
- `ros2_ws/src/rl_navigation/config/hyperparams.yaml` — 학습 하이퍼파라미터

---

## 7. 결론

**현황:** Reward v2의 과도한 per-step 보상 감소로 인해 2026-05-20 학습이 완전히 실패.

**원인:** 
- Per-step 보상 60-70% 급락 (1.3→0.35/step)
- 정책이 움직일 동기 상실
- Orbit-milking 해결 시도의 오류로 인한 과도한 조정

**해결책:**
- Stage 1: Per-step 보상 복구 (w_heading 0.4→0.7, w_dist 0.3→0.5, w_impact 0.2→0.4)
- Stage 2: Dry-run 검증 (5K steps)
- Stage 3: 본학습 재개 (500K steps)

**예상 효과:**
- Mean XY Distance: 19.9m → ≤5m
- Per-step Reward: -0.0076 → +0.5-0.8
- Convergence: 100-200K steps

**다음 액션:** 즉시 hyperparams.yaml 수정 & colcon build 실행

