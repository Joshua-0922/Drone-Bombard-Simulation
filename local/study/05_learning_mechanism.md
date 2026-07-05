# 05. 학습 메커니즘

> 카테고리 5. exploration / replay buffer / fine-tune 시 catastrophic forgetting / curriculum 등 "왜 학습이 잘 되거나 안 되는가" 의 원리. v8 → v9a 실패를 메커니즘 측면에서 다시 해석.

---

## Session 5.1 — Exploration vs Exploitation

### 둘의 정의

- **Exploration**: 아직 안 가본 (s, a) 시도 — buffer 다양성 확보.
- **Exploitation**: 지금까지 학습한 Q 가 가리키는 best action 선택 — reward maximize.

### SAC 가 이걸 어떻게 푸나

- Actor 가 Gaussian 분포 출력 → sample → action. **stochastic policy 자체가 exploration**.
- α (entropy coef) 가 explore-exploit trade-off knob:
  - α 크면 entropy term ↑ → 더 무작위
  - α 작으면 Q term ↑ → 더 deterministic
- Auto-tuning 으로 α 가 자동 조정. 우리는 4.4 에서 본 대로 안전망 추가.

### Exploration 의 mode 3 가지

| Mode | 우리 코드 | 효과 |
|---|---|---|
| **Action noise** | actor 의 stochastic sample | local 탐색 (현재 state 근방 다양화) |
| **Random restart** | epi reset (initial state 분포) | global 탐색 (다른 trajectory 시작점) |
| **Curiosity / RND** | 우리 안 씀 | unexplored state 적극 추구 |

→ 우리 환경은 **reset 분포가 매번 거의 같음** (spawn 위치 고정, target 고정). 즉 global exploration 이 약함. 정책이 한번 local optimum 에 빠지면 빠져나오기 어려움.

### "Toss 전략 발견" 의 우연

- v8 (96bokgae) 에서 toss 가 발견된 이유 = epi 진행 중 stochastic action 의 운 좋은 sequence 가 "pitch back + drop momentum" 발견 → critic 이 큰 reward 기억 → policy 가 reproduce.
- 같은 reward 구조의 v7 / v6 에서는 안 나옴 — **운 (random seed) 차이**.
- → toss 가 "robust 한 학습 결과" 가 아니라 **path-dependent jackpot**.

---

## Session 5.2 — Replay buffer 의 deep structure

### Buffer 의 transition 분포

```
buffer_size = 500k
초기:    random action transitions   (learning_starts=1000)
초중반:  early policy transitions    (대부분 실패, 작은 reward)
중반:    learning policy transitions (가끔 큰 reward)
후반:    competent policy transitions (다양한 시도)
```

→ buffer 는 **시간순 누적**. 50만 개 차면 oldest 부터 덮어씀 (FIFO).

### 우리 epi 평균 ~500 step → buffer 가 ~1000 ep 분량

- 300k step 학습 → 약 600 ep 분량 buffer 채움 (full 안 됨)
- 500k step 학습 → buffer 가 거의 full
- **600k step (v9a 목표) 에서 oldest 100k 덮어쓰기 시작** → 초기 random transition 사라짐

### Catastrophic forgetting

- buffer 가 새 transition 으로 채워지면 정책은 자기 최신 trajectory 만 학습.
- 만약 한 시점에 "잘못된 trajectory" (예: 호버에 빠진 정책) 이 buffer 채우기 시작하면 → 옛날 좋은 transition 이 sampling 빈도 ↓ → 점점 잊혀짐.
- **PER 가 큰 reward transition 의 sampling 확률 ↑** 으로 어느 정도 보완하지만 한계.

### 우리 v9a 실패의 이 측면 해석

- v8 의 좋은 checkpoint (217k) 로 시작 → fine-tune (w_dist 1.5, drop_angaccel 0.5 추가)
- v8 의 replay buffer 도 함께 load → 초기엔 좋은 transition 다수
- 하지만 새 reward 구조에서 v8 policy 가 즉시 적응 못함 → 새 transition 은 "나쁜 정책" 의 결과
- 새 reward 의 큰 penalty (drop_angaccel) 가 PER priority 잡음 → bad transition 의 sampling ↑
- ~17k step 만에 정책이 v8 보다 나빠짐 (10 ep eval 50% → 33%)

→ **off-policy fine-tune 의 본질적 위험**: 새 reward 가 transition distribution 의 priority 분포를 흔들고, 그게 critic 학습을 흔든다.

---

## Session 5.3 — Fine-tune 시 3 가지 선택

v8 checkpoint 에서 새 reward 로 가려면:

### 옵션 1: Full fresh start (v8 무시)
- pros: clean baseline, distribution mismatch 없음
- cons: v8 의 toss 학습 잃음, 300k step 다시 필요
- 비유: 책 처음부터 새로 읽기

### 옵션 2: v8 weights load + buffer fresh
- pros: policy 의 prior 유지, 새 buffer 가 새 reward 분포 대표
- cons: 초기엔 actor 는 v8, critic 은 백지 → critic 이 actor 따라잡기 전까지 actor 가 잘못된 방향 학습
- 비유: 옛 기억으로 새 환경 적응

### 옵션 3: v8 weights + v8 buffer (우리 v9a 가 한 것)
- pros: 즉시 학습 가능, 옛 좋은 transition 활용
- cons: **catastrophic forgetting + critic 혼란**. 새 reward 와 옛 transition reward 가 다르게 계산되는데 buffer 의 reward 값은 옛 값.
- 비유: 옛 답안지 들고 새 시험 보기 → 옛날 답에 점수가 안 맞아 혼란

### 우리 경험상 어느 게 나았나

v3~v8 를 거치며:
- v3 ~ v4: 옵션 2 시도 → 매번 발산 (critic NaN, ent_coef 6.0+)
- v5 ~ v7: 옵션 1 (fresh) → 안정적이지만 매번 처음부터 → 시간 비용 큼
- v8: 옵션 1 (fresh) → toss 발견 → success 80.6%
- v9a: 옵션 3 → 17k 만에 실패 확인

→ **결론: 새 reward 면 옵션 1 (fresh) 가 가장 안전**. 옵션 3 는 hyperparam 미세 조정 (예: LR 만 바꾸기) 에만 안전.

---

## Session 5.4 — Reward shaping 의 trade-off

### Dense vs Sparse

| 측면 | Dense reward | Sparse reward |
|---|---|---|
| 학습 속도 | 빠름 (매 step gradient) | 느림 (terminal 만 신호) |
| Local optimum 위험 | 높음 (shaping 따라가다 본 목표 잊음) | 낮음 (목표만 보고 학습) |
| 설계 난이도 | 어려움 (shaping bias 의도치 않게 줌) | 쉬움 (최종 목적만 정의) |
| 우리 case | 4 Layer hierarchical | (Layer 4 가 사실상 sparse signal) |

### 우리는 Hybrid

- Layer 1-3: dense per-step
- Layer 4: sparse terminal (drop 한 번)

→ 이론적으로 Layer 1-3 는 **shaping reward — 학습 속도 향상 + 정책 가이드**.

### Shaping 의 함정 — Reward Hacking

- Layer 3 의 r3_dist 가 "타겟으로 가까이 가면 +" 라면, 정책은 **타겟 근처 hover** 만 해도 큰 reward.
- 해결: speed_gate (속도 < 2 m/s 면 cos_heading reward 감쇠), w_distance_penalty (멀수록 cost) 등.
- 우리 코드의 `speed_gate_enabled` 가 정확히 이 hack 차단용.

### "Spiral Milking" 이라는 우리 용어

- v3 시절: 정책이 타겟 주변을 빙빙 도는 것만으로 r3_orient reward 챙김 발견
- 원인: speed_gate 없음 + cos_heading 단독 reward.
- 해결: speed_gate 도입 후 사라짐 (v4 이후).

### v9a 의 reward 설계가 왜 망했나 — 가설들

가설 A: **drop_angaccel penalty 가 toss 의 핵심 동작을 직접 처벌**
- toss = pitch back (= 빠른 ang 변화). 이걸 penalty 로 잡으면 정책이 toss 회피 → 결국 hover drop 으로 수렴 → low reward.

가설 B: **w_dist 증가가 cruise 단계 부풀림**
- w_dist 1.0 → 1.5 → cruise 단계 reward 비중 ↑ → drop 의 상대적 비중 ↓ → 정책이 drop attempt 회피.

가설 C: **두 변경의 결합 효과**
- A + B 가 시너지로 정책의 incentive 구조 흔들기.

→ **세 가설 모두 reward shaping 의 unintended side effect**. v10 에서 이걸 어떻게 분리/측정할 것인가가 design 쟁점.

---

## Session 5.5 — Curriculum

### Curriculum 의 정의

학습 task 를 쉬운 것 → 어려운 것 순으로 단계화. 정책이 단계마다 "안전한 기반" 위에서 다음 단계 학습.

### 우리가 한 curriculum 들

| 시도 | 단계 1 | 단계 2 | 결과 |
|---|---|---|---|
| Phase 1 redux | target 가까이 (5m) | (없음, 한 단계) | v8 의 baseline |
| v10a 시도 (시도만) | stage1_only=true (호버) | stage1_only=false (drop) | 코드는 있지만 미실행 |
| 사용자 신 안 | "두 단계 모드 분리" | (아직 정의 안 함) | v10 design 의 핵심 |

### Curriculum 의 함정

- 단계 1 학습 후 단계 2 진입 시 **catastrophic forgetting**: 단계 1 에서 배운 안정 비행이 단계 2 에서 새 reward 에 흔들림.
- 해결책 후보:
  - 단계 1 weight freeze (lower layer 만)
  - replay buffer 의 단계 1 transition 비율 유지
  - 단계 1 reward 를 단계 2 reward 의 subset 으로 (curriculum 이 아닌 ramp-up)

### 우리 stage1 코드 (drone_drop_env.py:950-958)

```python
if (not terminated and not truncated
        and self._cfg_stage1_only
        and d_3d < self._cfg_stage1_R):
    reward += self._cfg_stage1_reach_bonus
    terminated = True
    info['stage1_reached'] = True
```

→ "stage1 도달 시 ep 즉시 종료 + bonus 100". 단순 hovering 학습용. 단 drop 은 비활성.

→ 만약 stage1 학습 후 weights load → stage1_only=false 로 학습 시 위 catastrophic forgetting 위험. v10 design 에서 이 transition 어떻게 처리할지가 쟁점.

---

## Session 5.6 — Bias / Variance / Bootstrap

### Bias (편향)

- Q 가 실제 return 보다 systematically 크거나 작게 추정.
- 원인: function approximation (NN), overestimation bias (max operator).
- 우리 대응: twin critic (min), target clip 500.

### Variance (분산)

- 같은 (s, a) 에서 Q 추정이 흔들림.
- 원인: rollout 의 stochastic 성, batch sampling 의 randomness.
- 우리 대응: batch_size 256 (큰 편), Huber loss (outlier 영향 감쇠).

### Bootstrap

- Q(s, a) ← r + γ * Q(s', a'). **Q 가 Q 로부터 학습**.
- 위험: Q 추정 오류가 다음 update 의 target 에 누적 → 발산.
- 우리 대응: tau=0.002 (target Q 의 slow update), target_q_clip=500, learning_starts=1000.

### Episode 중 bootstrap 가 안 되는 지점

- terminated=True (우리는 drop 또는 stage1 reached): target = r (다음 Q 안 씀).
- truncated=True (crash/timeout): target = r + γ * Q(s', a') — **여전히 bootstrap**.

→ Gymnasium 의 두 신호 구분이 critic 학습 신호에 직접 영향. "drop 한 epi" 와 "crash 한 epi" 의 critic gradient 가 다른 형태.

---

## Session 5.7 — v8 → v9a 실패의 메커니즘 종합 진단

### 사실 (statistical)

- v8 (217k): 10 ep success 50%, mean d_error 2.002 m
- v9a (313k): 10 ep success 33%, mean d_error 2.006 m
- v9a (432k, resume): 5 ep success 67% (3 drops, 2 hover_timeout) → 정책 악화

### 메커니즘 1: catastrophic forgetting (5.2)

v8 의 toss 정책이 새 reward 에서 즉시 reproduce 안 됨 → 17k step 동안 bad transition 다수 → critic 이 잘못된 방향 학습 → policy 도 따라감.

### 메커니즘 2: reward hacking 의 새 모드 (5.4)

drop_angaccel penalty 가 toss 의 pitch back 을 직접 처벌 → 정책이 회피 → toss 사라짐 → drop 시도가 hover_timeout 으로 바뀜.

### 메커니즘 3: fine-tune 의 옵션 3 (5.3)

v8 buffer 의 transition reward 가 옛 reward 함수로 계산됨. 새 critic 이 그 reward 를 보고 학습 → distribution mismatch.

### 메커니즘 4: rolling checkpoint 의 한계 (training infra)

432k 까지 가다 CUDA error 종료. preempt save 는 SIGTERM 만 트리거 (Ctrl+C SIGINT 는 trigger 안 됨, memory:`feedback_train_sac_graceful_kill.md`). rolling checkpoint 가 살린 step432k 모델은 cuda crash 직전 — 안정성 보장 못함.

### 4 메커니즘의 결론

→ **v9a 실패는 알고리즘 측 문제와 reward 설계 문제가 결합**.

근본 해결책:
- (A) fresh 학습 (옵션 1) — 시간 비용 크나 안전
- (B) reward 구조를 단계화 (curriculum) — design 으로 풀기
- (C) buffer 재초기화 + actor freeze 후 critic 만 warm-up — 절충
- (D) 알고리즘 교체 (TD3, PPO) — 우리 환경에 더 잘 맞을지 검증 필요

→ v10 design 에서 위 4 옵션 중 어디로 갈지 결정 필요.

---

## Session 5.8 — 그래서 "잘 학습되는 RL" 의 4 조건

문헌 + 우리 경험 종합:

### 조건 1: 단조 학습 신호

- 매 step 의 reward 가 "지금 잘하고 있나?" 의 답.
- 우리 Layer 1-3 가 이걸 시도. 단 Layer 3 가 dense 라 shaping bias.

### 조건 2: Buffer 분포의 안정성

- replay 가 stale 정책의 transition 으로 채워지면 안 됨.
- 우리 PER 가 큰 reward transition 의 sampling ↑ 로 대응. 단 fine-tune 시 깨짐.

### 조건 3: Bootstrap 의 수렴성

- Q 학습이 수렴해야 actor 가 안정한 gradient.
- 우리 customization 4가지 (twin, target clip, Huber, tau 작게) 가 이걸 보장.

### 조건 4: Exploration 충분성

- random restart + stochastic policy 가 local optimum 회피.
- 우리는 reset 분포 고정 → exploration 약함 → v8 의 toss 는 운.

→ v10 design 시 위 4 조건 각각을 어떻게 강화/보장할지를 명시적으로 답해야.

---

## 카테고리 5 정리 — 5 줄

1. **off-policy SAC 의 핵심 자산이 buffer**. 그 분포가 학습 quality 결정.
2. **fine-tune 시 옵션 3 (weights+buffer load) 가 가장 위험** — 우리 v9a 가 정확히 이걸로 실패.
3. **reward shaping 은 reward hacking 위험과 동전 양면** — v9a 의 drop_angaccel 이 toss 동작을 직접 처벌하는 unintended effect.
4. **curriculum 은 catastrophic forgetting 과 떼어 생각 못함**. 단계 전환 시 옛 학습 잃기 쉬움.
5. **잘 학습되는 RL 4 조건**: 단조 학습 신호 / buffer 안정성 / bootstrap 수렴성 / exploration 충분성. v10 이 4가지에 각각 답해야 함.

→ **카테고리 1~5 학습 완료**. 다음: 5 카테고리 통합 정리 + v10 design 의 question 목록 도출.
