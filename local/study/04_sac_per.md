# 04. SAC + PER 알고리즘

> 카테고리 4. Soft Actor-Critic 가 우리 MDP 에서 정확히 무엇을 학습하며, 왜 우리 코드엔 customization (`DampedEntropySAC`, `PrioritizedReplayBuffer`) 이 있는가.

---

## Session 4.1 — SAC 한 페이지 요약

### 한 문장

**Off-policy actor-critic with entropy regularization** — value function 을 학습 (critic) 하면서 동시에 policy 를 maximize expected return + entropy 로 학습 (actor). off-policy 라서 replay buffer 재사용, continuous action 에 강함.

### 학습하는 게 뭔가

```
maximize E[ Σ γ^t * (r_t + α * H(π(·|s_t))) ]

  r_t       : 한 step reward (우리는 4-Layer)
  H(π(·|s)) : policy entropy — "얼마나 무작위인가"
  α         : entropy coefficient (= 우리 코드의 ent_coef)
  γ         : discount (우리 0.995)
```

→ **"reward 잘 모으되 결정론적으로 빠지지 말고 탐색도 해라"**. α 가 두 목표 사이 trade-off.

### 4 개 네트워크

| 네트워크 | 역할 | 우리 size |
|---|---|---|
| **Actor π(a|s)** | state → action 분포 (mean + log_std → tanh squash) | [256, 256] MLP |
| **Critic Q₁(s,a)** | (state, action) → Q 값 (return 추정) | [256, 256] MLP |
| **Critic Q₂(s,a)** | 동일 (twin critic, overestimation 차단) | [256, 256] MLP |
| **Target critics Q̄₁, Q̄₂** | Q 의 slow copy (polyak average, τ=0.002) | 동일 |

→ critic 2 개 쓰는 이유: Q 학습이 overestimation bias 가 강함. min(Q₁, Q₂) 가 보수적 추정.

### 한 update step 의 흐름

```
1. replay buffer 에서 transition (s, a, r, s', done) 256 개 sample
2. Critic 업데이트:
   target_Q = r + γ * (1 - done) * (min(Q̄₁(s', a'), Q̄₂(s', a')) - α * log π(a'|s'))
                                                   ↑ entropy 포함
   loss_critic = MSE(Q(s, a), target_Q)
   Adam step on Q₁, Q₂

3. Actor 업데이트:
   sample a ~ π(·|s)
   loss_actor = α * log π(a|s) - min(Q₁(s, a), Q₂(s, a))
   Adam step on π

4. α 업데이트 (auto-tuning):
   loss_α = -log_α * (log π(a|s) + H_target).detach()
   Adam step on log_α

5. Target Q 업데이트 (polyak):
   Q̄ ← τ * Q + (1 - τ) * Q̄
```

우리 `tau=0.002` 는 매 step Q̄ 가 Q 쪽으로 0.2% 만 이동 → 매우 느림 → 매우 안정.

### Off-policy 의 의의

- on-policy (PPO 등) 는 매 update 후 buffer 비움 → sample efficiency 낮음.
- off-policy (SAC, TD3) 는 buffer 재사용 → 한 transition 을 여러 번 학습 가능 → **sim FPS 가 병목인 우리에게 결정적**.
- 단점: 오래된 transition 이 stale policy 에서 나왔으니 distribution mismatch — replay 가 길어질수록 학습 안정성 ↓ (catastrophic forgetting).

---

## Session 4.2 — 우리 hyperparams.yaml 해부

### Training 블록

```yaml
total_timesteps: 600000        # 60만 step
checkpoint_freq: 5000          # 5천 step 마다 ckpt
max_checkpoints_kept: 5        # rolling 5 개 유지
num_envs: 1                    # 단일 env (multi-instance 미사용)
eval_freq: 10000               # 1만 step 마다 eval
eval_episodes: 3               # eval 시 3 ep 평균
```

→ **60 만 step ≈ 60 만 transition.** v8 학습이 ~300k 였고 이번 v9a 가 600k 까지 가 보려다 사용자 SIGTERM.

### SAC 핵심 6개

```yaml
learning_rate: 1.0e-4    # Round 3: 3e-4 → 1e-4 (critic overshoot 완화)
buffer_size: 500000      # 50만 transition 저장
batch_size: 256          # update 당 256 sample
tau: 0.002               # Round 3: 0.005 → 0.002 (target Q 안정화)
gamma: 0.995             # discount — 우리 epi 800 step 의 future 도 고려
learning_starts: 1000    # 처음 1000 step 은 random action (buffer warmup)
gradient_steps: 1        # 매 env step 마다 1 회 update (1:1 ratio)
```

각 의미:

- **learning_rate 1e-4**: critic Q 값이 빠르게 흔들리면 actor 가 따라가다 발산. 우리는 reward 가 ±200 으로 큰 편 → conservative LR.
- **buffer_size 500k**: 80 episode 정도 분량 (epi 평균 500 step 가정). 학습 진행 동안 옛 transition 점진적 replacement.
- **tau 0.002**: target network 이 정책에서 너무 빨리 변하면 target 자체가 흔들림 → bootstrap noise. 0.002 면 500 update 마다 절반 갱신.
- **gamma 0.995**: 한 step 의 미래 가치 = 0.995^200 ≈ 0.37 (200 step 뒤). 우리 epi 길이 800 step 의 후반 reward 도 considered.

### PER 3개 + Damped Entropy 3개 (customization)

```yaml
use_per: true
per_alpha: 0.6           # priority 강도 (0=uniform, 1=full)
per_eps: 0.1             # 최소 priority (zero priority 방지)
per_priority_max: 30.0   # outlier 차단 cap
ent_damping_threshold: 5.0   # log_prob 집중도 임계
ent_coef_hard_cap: 1.0       # α 최대값
target_entropy: -15.0        # H_target (자세히 4.4 에서)
target_q_clip: 500.0         # critic bootstrap 차단
```

→ **여기가 우리 customization 의 본체**. 4.3, 4.4 에서 깊이.

---

## Session 4.3 — PrioritizedReplayBuffer (우리 PER)

### 표준 PER 와의 차이

**표준 PER** (Schaul 2015):
1. priority = |TD-error|^α + ε
2. importance sampling weight 로 bias 보정
3. update 마다 사용한 transition 의 priority 갱신

**우리 PER (`train_sac.py:171-207`)**:
1. priority = (|reward|.max() + eps)^α — **TD-error 아님, reward magnitude**
2. importance sampling weight **안 씀** (bias 보정 생략)
3. **static priority** — add 시점에 결정, 갱신 안 함
4. priority_max 로 cap (우리 30) — outlier 차단

### 왜 simplified PER?

- 우리 reward 구조가 매우 sparse 함. 4-Layer 중 Layer 4 가 한 episode 에 한 번 (drop 시) → 보통 reward 작은데 drop step 만 ~200.
- 표준 PER 는 모든 update 후 priority 갱신 비용 큼.
- **"큰 reward transition 자주 보자"** 만 필요 → reward magnitude 기반 static priority 가 효율적.

### priority 계산

```python
priority = min(priority_max, (|reward| + eps) ^ alpha)
        = min(30, (|reward| + 0.1) ^ 0.6)

# 예:
# reward=0    → (0.1)^0.6 = 0.25
# reward=10   → (10.1)^0.6 = 4.07
# reward=100  → (100.1)^0.6 = 15.86
# reward=200  → (200.1)^0.6 = 24.07
# reward=500  → 30 (capped)
```

→ 큰 reward (drop terminal) 의 sampling 확률이 작은 reward (per-step) 의 ~100배 정도.

### 효과 — 우리 경험

- v3~v4 에서 critic 발산 했을 때 PER 가 큰 outlier reward 만 계속 뽑아 와서 발산 가속.
- **`priority_max=30` cap 도입** 으로 outlier 영향 제한.
- 그래도 critic 폭주 가능 → Huber loss + target Q clip 으로 보완 (다음 섹션).

---

## Session 4.4 — DampedEntropySAC (우리 SAC 변형)

### 4 개의 수정

#### 1) Entropy 자동 튜닝의 polarity 문제

표준 SAC 의 α 자동 튜닝:
```
loss_α = -log_α * (log π(a|s) + H_target).detach()
```

- log π(a|s) > -H_target (즉 entropy 가 너무 낮음) → loss < 0 → α 증가 → 탐색 강제
- log π(a|s) < -H_target (entropy 가 너무 높음) → loss > 0 → α 감소 → 탐색 약화

**문제**: Bounded action space (tanh squash) 에선 정책이 deterministic 으로 수렴해도 entropy 가 자연스럽게 낮아짐. → α 가 계속 증가 → policy 의 exploitation 신호 약화 → 다시 entropy 증가 → **양성 피드백 발산**. Round 4/5 에서 α 가 6.0+ 로 폭주.

#### 2) Per-sample damping (Round 7 v3)

우리 수정 (`train_sac.py:85-96`):
```python
concentration_per_sample = max(log_prob - (-target_entropy), 0)
damping_per_sample = threshold / (threshold + concentration_per_sample)
loss_α = -log_α * (log π + H_target).detach() * damping_per_sample
```

직관:
- transition 의 log_prob 이 target 보다 훨씬 크면 (정책이 매우 deterministic) → concentration 커짐 → damping 작아짐 → α 증가 신호 약화.
- "deterministic 으로 수렴하려는 정책을 막지 마라" 가 목표.
- element-wise 라 outlier 만 강하게 damped, 정상 transition 은 그대로.

#### 3) Hard cap on α

```python
log_ent_coef.clamp_(max=log(ent_coef_hard_cap))
# 우리 hard_cap=1.0 → log_α ≤ 0 → α ≤ 1.0
```

soft damping 이 실패해도 α=1 이상 못 올라감. 발산 안전망.

#### 4) Target Q clipping + Huber loss

```python
target_q_clipped = target_q.clamp(-500, 500)
critic_loss = 0.5 * Σ smooth_l1_loss(current_q, target_q_clipped)
```

- **target Q clip 500**: reward scale ±200 고려한 4×배 한도. bootstrap (γ × next_Q) 의 폭주 차단.
- **Huber loss (smooth_l1)**: MSE 는 큰 TD-error 에 gradient 비례 → polarity 문제 가속. Huber 는 |error| > 1 에서 gradient 1.0 saturate → 안전한 step.

### 왜 이렇게 복잡한가?

→ **우리 reward 가 sparse + large magnitude (drop 시 ±200)** 라 표준 SAC 의 가정 (smooth dense reward) 을 어김. 4 가지 보정이 누적되어야 학습이 안정.

→ 흥미로운 점: v10 에서 reward 구조를 단순화하면 이 customization 들도 단순화 가능. 현재는 "버그 fix 누적" 의 형태.

---

## Session 4.5 — target_entropy 의 미묘함

### 기본값 = -|A|

SB3 SAC 의 default `target_entropy = -|A| = -5` (action 5 차원). 이론적 근거: continuous Gaussian 의 expected entropy 가 0 근처일 때 "균등 탐색" 수준.

### 왜 -15.0?

```yaml
target_entropy: -15.0   # bounded action space 에 맞게 deterministic 허용
```

- **bounded (tanh squash) action** 에선 정책이 양 끝 (-1 또는 +1) 으로 수렴 → log_prob 크게 양수 → 실제 entropy 음수.
- target=-5 면 "더 무작위해라" 신호 계속 → α 증가 → 위 발산 문제 트리거.
- target=-15 (3배 더 낮음) → 정책이 deterministic 으로 가는 것을 허용 → α 가 자연스럽게 0 근처로 수렴.

→ **target_entropy = -15** + **damping** + **hard cap = 1** 의 3 단 안전망이 우리 ent_coef 발산을 막음.

---

## Session 4.6 — 한 epi 의 학습 흐름 (full picture)

```
[Env]                                    [SAC + PER]
ep 시작
  reset → obs_0                          ─→ obs_0 in buffer (future)
  policy(obs_0) → action_0               ←─ actor.action_log_prob
  step(action_0) → obs_1, r_0            ─→ transition (obs_0, a_0, r_0, obs_1, done=False)
                                                ↓
                                          buffer.add()
                                                ↓
                                          priority = min(30, (|r_0|+0.1)^0.6)
                                                ↓
                                          if step ≥ learning_starts (1000):
                                              SAMPLE 256 from buffer (PER weighted)
                                              ─ critic update (Huber, target clip)
                                              ─ actor update
                                              ─ α update (per-sample damped)
                                              ─ target Q polyak (τ=0.002)

  ... step 반복 ...

  drop step → terminated=True, r_drop=200
                                          ─→ buffer.add() with priority ≈ 24 (높음)
                                          (다음 sample 시 자주 뽑힐 가능성)

ep 끝
  reset → 새 ep
```

### gradient_steps = 1 의 의미

매 env step 마다 SAC 가 1 회 update. 즉 60 만 step → 60 만 update. **env step 과 SAC update 가 1:1 비율**. 이 비율을 늘리면 (e.g. 4) sample efficiency 향상하지만 wallclock 4 배.

### learning_starts = 1000

처음 1000 step 은 actor 가 random action (uniform). buffer 에 다양한 transition 쌓아 둠. SAC 가 빈 buffer 에서 학습 시작하면 critic NaN.

---

## Session 4.7 — 우리 환경에 SAC 가 잘 맞는가?

### 맞는 이유

- continuous action (velocity setpoint) — SAC 의 강점.
- off-policy → buffer 재사용 → sim FPS 가 병목인 우리에 효율적.
- entropy regularization → 좁은 valley (toss 궤적) 에 빠지지 않게 탐색 유지.

### 안 맞는 이유

- **sparse + large terminal reward**. Layer 4 의 ±200 가 critic Q 분포를 비대칭으로 만듦 → bootstrap 폭주 위험. → Huber + target clip 으로 대응했지만 근본은 reward 설계 문제.
- **terminated 의 비대칭**. drop=terminated 라 bootstrap 안 되지만 truncated (crash/timeout) 는 bootstrap. drop 한 ep 와 안 한 ep 의 critic 학습 신호 분포가 다름.
- **action[4] dead dimension**. policy 가 학습하지 않는 차원을 actor 가 5차원으로 출력 → α 계산에 영향. (target_entropy=-15 가 이걸 어느 정도 보정.)
- **PX4/Gazebo non-determinism**. 같은 (s, a) 가 다른 s' 로 갈 수 있음 → critic 의 variance 증가.

→ v10 design 시 위 4 가지 mismatch 가 어디까지 reward/MDP 재설계로 풀리는지, 어디까지 알고리즘 교체 (PPO, TD3, DreamerV3) 가 더 나은지 검토 필요.

---

## 카테고리 4 정리 — 4 줄

1. **SAC = off-policy actor-critic with entropy regularization**. 우리 environment 의 sample efficiency 요구에 정확히 맞음.
2. **우리 커스텀**: PrioritizedReplayBuffer (reward 기반 static priority) + DampedEntropySAC (per-sample damping + hard cap + target Q clip + Huber loss).
3. 커스텀의 원인은 **sparse + large terminal reward** 가 SAC 표준 가정을 어겨서. 4 가지 보정 누적으로 안정화.
4. **hyperparams 의 핵심 안전망**: LR=1e-4 (보수적), tau=0.002 (slow target), target_entropy=-15 (deterministic 허용), ent_coef_hard_cap=1.0 (발산 cap), target_q_clip=500 (bootstrap cap).

→ 다음: **카테고리 5 — 학습 메커니즘**. exploration / replay 의 forgetting / fine-tune 시 critic 재초기화 trade-off / curriculum. v8 → v9a 실패를 algorithm 측면에서 다시 해석.
