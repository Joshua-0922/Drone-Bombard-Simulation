---
date: 2026-06-03
updated: 2026-06-05
tags: [research, SAC, target-entropy, bounded-action, tanh-squash, entropy-divergence]
status: active
type: research
owner: junsang
---

# SAC Entropy 발산의 근본 원인 — Bounded Action + Target Entropy

> **발견 맥락:** Round 4~6의 반복된 ent_coef 발산이 처방 형태(per-step/terminal/damping)와 무관함을 확인 → 근본 원인이 **action space 구조 + target_entropy 설정**에 있음을 규명 (2026-06-03).
> **원자료:** `local/meeting_notes/meeting_notes_2026-06-03.txt` PART 2, `local/design/model_history.md`(Round 7), `local/issues/issue_019_sac_entropy_divergence.txt`
> **규칙화:** [[research/rl_rules]] · 관련: [[research/sac_reward_density_junsang]]

---

## 핵심 명제

**Bounded action space(tanh squash)에서는 `log_prob`가 구조적으로 큰 양수가 되고, default `target_entropy = −|A|`(= −5)는 이 환경에서 ent_coef를 끝없이 키우는 방향으로 작동한다. → ent_coef가 hard cap까지 단조 증가하여 학습이 망가진다.**

처방은 `target_entropy`를 충분히 작은 음수(**−15**)로 내리는 것. 이게 발산의 *원인*을 직접 차단한다 (damping·cap은 증상 완화일 뿐).

---

## SAC 자동 entropy 튜닝 메커니즘

SAC는 entropy 계수 `alpha`(=ent_coef)를 자동 조절한다:

$$
\mathcal{L}(\alpha) = -\mathbb{E}\big[\log\alpha \cdot (\log\pi(a|s) + \mathcal{H}_{target})\big]
$$

$$
\frac{\partial \mathcal{L}}{\partial \log\alpha} = -\big(\log\pi(a|s) + \mathcal{H}_{target}\big)
$$

- `log_prob > −target_entropy` 이면 → **alpha ↑** (탐색 강화 압력)
- `log_prob < −target_entropy` 이면 → **alpha ↓** (활용 강화 압력)

즉 정책이 목표 엔트로피보다 **더 deterministic**(log_prob 큼)하면 SAC가 "탐색이 부족하다"며 alpha를 키운다.

---

## Bounded action(tanh squash)의 특성

action을 $[-1, 1]$로 제한하기 위해 SAC는 가우시안에 tanh를 씌운다:

$$
a = \tanh(u), \quad u \sim \mathcal{N}(\mu, \sigma^2)
$$

$$
\log\pi(a|s) = \underbrace{\log\mathcal{N}(u|\mu,\sigma)}_{\text{가우시안}} - \underbrace{\sum_i \log\big(1 - \tanh^2(u_i)\big)}_{\text{Jacobian 보정}}
$$

**핵심:** Jacobian 보정항 $-\sum\log(1-\tanh^2 u_i)$ 은 action이 $\pm1$ 근처일수록 **큰 양수**가 된다($\tanh^2 \to 1$ 이면 $\log(1-\tanh^2)\to-\infty$, 부호 반전으로 $+\infty$).

→ bounded action 환경에서 `log_prob`는 일반적으로 큰 양수이고, **`log_prob > 5`는 정상 학습 중에도 흔하다.**

---

## 왜 default target_entropy(−5)가 발산을 부르는가

SB3 default는 $\mathcal{H}_{target} = -\dim(A) = -5$ (action 5차원).

- `−target_entropy = 5`
- 위에서 봤듯 `log_prob > 5`가 거의 항상 성립
- → `∂L/∂logα` 부호상 **거의 항상 alpha ↑ 압력**
- → 학습이 진행될수록 ent_coef가 단조 증가 → hard cap에 갇힘

> **비유 (회의록):** "중력이 **위쪽**으로 작용하는 세계 + 천장(cap)" → 천장에 영원히 붙어버림. ← Round 6 v2의 모습.

cap에 갇히면 entropy bonus가 reward를 압도($\alpha \cdot \mathcal{H} \approx 2 \times 3.5 = +7$ vs reward $\sim0.7$) → 정책이 거의 random → critic이 random policy를 추종하다 `critic_loss` 14M+ 폭주.

---

## 처방: target_entropy = −15

- `−target_entropy = 15`
- `log_prob > 15`는 매우 극단적 → 거의 발생 안 함
- → 대부분 **alpha ↓ gradient** → cap에 안 닿거나, 닿아도 자기 복구

> **비유:** "중력이 **아래쪽**으로 작용하는 세계 + 천장" → 가끔 위로 튀어도 다시 내려옴. ← Round 7의 안정.

### 검증 결과 (Round 7)
- Round 7 1차(`iobwvcrm`): ent_coef **0.29 안정** (이전 2.0 cap 갇힘 대비) — 처방 효과 확인
- 다만 Round 7 v2에서 **장기 학습 시 critic_loss 폭주(250× 점프)가 다시 ent 발산 유발** 발견 → target_entropy만으로는 부족, critic 안정화 병행 필요 (아래)

---

## 잔여 문제: critic 폭주가 2차 트리거

target_entropy를 고쳐도, 장기 학습에서 **critic_loss 폭주가 ent 발산의 또 다른 경로**임이 Round 7 v2(`dx5fmck6`, 280k+)에서 드러났다:

```
critic_loss 폭주(68 → 17,100, 250× 점프) → Q 값 inflation
  → 정책 행동 비정상 → log_prob 거대화 → alpha 증가 → cap
```

즉 **critic 폭주가 entropy 발산의 원인**(반대 아님). → Round 7 v3에서 critic 안정화 3종 처방으로 해결:

| 처방 | 효과 |
|------|------|
| **Per-sample damping** (q95 scalar → element-wise) | batch outlier만 정확히 damped |
| **Huber loss (smooth_l1)** | 큰 TD error의 gradient saturation (MSE의 outlier 폭주 차단) |
| **target_q_clip = 500** | bootstrap inflation 차단 (reward 스케일 ±200 고려 ±500 충분) |

→ Round 7 v3: critic_loss **200k → 35**, ent_coef **1.0 cap → 0.055 회복**. Phase 1 마감.

---

## 발산 방어 4계층 (최종 정리)

| 계층 | 처방 | 역할 |
|------|------|------|
| 1차 (근본) | `target_entropy = −15` | bounded action에서 alpha↑ 압력 차단 |
| 2차 | per-sample(percentile) damping | alpha 증가 둔화 |
| 3차 | `ent_coef_hard_cap = 1.0` | 절대 상한 (학습 가능 수준) |
| 4차 | critic 안정화 (Huber + target_q_clip=500) | critic 폭주發 2차 발산 차단 |
| + | reward hard cap [−200,+300] | 단일 step reward 폭주 차단 |

---

## 적용 규칙

1. **Bounded action(tanh) + SAC 자동 entropy 환경에서는 default target_entropy(−|A|)를 그대로 쓰지 말 것.** 충분히 작은 음수로 내려라(본 프로젝트: −15).
2. ent_coef가 단조 증가하며 cap에 붙으면 → damping/cap을 더 조이기 전에 **target_entropy부터 의심**.
3. target_entropy를 고쳤는데도 장기 학습에서 다시 발산하면 → **critic_loss 추세**를 보라. critic 폭주가 2차 트리거일 수 있다(→ Huber + target_q_clip).
4. 이 발산은 [[research/sac_reward_density_junsang]]의 "per-step density" 관점과 **별개의 더 깊은 층위**다. density는 sparsity를 통한 간접 트리거였고, 여기서는 action space 구조 자체가 원인.

---

## 관련 노트

- [[research/sac_reward_density_junsang]] — per-step density 관점의 발산 (5월 Round 4)
- [[research/rl_rules]] — Rule 8/9
- [[experiments/training_history]]
- [[daily/daily_2026-06-03_junsang]] — 근본 원인 규명 당일
- [[daily/daily_2026-06-05_junsang]] — Round 7 v3 critic 안정화 → Phase 1 마감
- [[00_index_junsang]]
- local: `meeting_notes/meeting_notes_2026-06-03` PART 2, `design/model_history`, `issues/issue_019_sac_entropy_divergence`
