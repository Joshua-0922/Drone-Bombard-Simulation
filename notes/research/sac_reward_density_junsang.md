---
date: 2026-05-31
tags: [research, SAC, reward-density, divergence, entropy, sparse-reward]
status: active
type: research
---

# SAC Per-step Reward Density와 발산

> **발견 맥락:** Round 4(`4j46qwpk`, 146k) per-step 보상 축소 직후 학습 발산.
> **원자료:** `local/archive/sac_oscillation_mechanism.md`, `local/issues/issue_017_hover_exploit.txt`
> **규칙화:** [[research/rl_rules]] Rule 8

---

## 핵심 명제

**SAC의 auto-entropy tuning은 per-step reward density에 민감하다. per-step density를 줄이면(= reward를 더 sparse하게 만들면) `ent_coef`가 양성 피드백으로 폭주하여 발산한다.**

per-step 보상은 절대 크기 자체보다 **drop terminal reward 대비 비율(density)** 이 중요하다.

---

## 사건: Round 4 발산

hover exploit 차단을 위해 per-step 보상을 줄였다:

- `w_heading` 0.7 → 0.3, `w_distance_penalty` 0.03 신규
- per-step : drop reward 비율 **1:300 → 1:700** (sparsity 2배)

결과 (`4j46qwpk`, 56~70k에서 발산 시작):

| 지표 | 값 | 정상 |
|------|-----|------|
| `ent_coef` | 0.58 → 1.5 → **6.03** | 0.3~0.5 |
| `critic_loss` | 100 → 1000 → **230,000+** | 100~500 |
| `ep_rew_mean` | −13 → −33 | — |

---

## 발산 모드 메커니즘 (양성 피드백)

```
per-step density ↓  (reward sparsity ↑)
   ↓
critic estimate variance ↑   (드문 큰 신호가 Q를 지배)
   ↓
policy가 dominant 신호(drop)에 집중 → entropy ↓
   ↓
SAC auto-entropy: "탐색 부족" 판단 → ent_coef ↑
   ↓
bounded action space [-1, 1] → entropy 더 못 올림
   ↓
SAC: "여전히 부족" → ent_coef ↑↑
   ↓
양성 피드백 루프 → ent_coef 6.03 발산
```

SAC는 target entropy를 맞추려 `ent_coef`를 자동 조절하는데, action이 bounded라 달성 가능한 최대 entropy에 한계가 있다. sparse reward로 정책이 과도하게 deterministic해지면 SAC가 무한히 `ent_coef`를 키우다 발산한다.

---

## 정상 oscillation vs 진짜 발산

sparse + 큰 terminal reward 환경에서 critic spike는 **정상**이다. 구별 기준:

| | 정상 oscillation (회복) | 진짜 발산 (회복 불가) |
|---|---|---|
| `critic_loss` | spike 후 100 batch 내 회복 | 1000+ 지속 후 증가 |
| `ep_len` | 짧아진 후 다시 길어짐 | 1로 수렴 |
| `ep_rew_mean` | 장기 추세 개선 | 더 음수로 |
| `ent_coef` | 안정 또는 점진 감소 | 1.0+ 폭주 |
| 사례 | Round 4 `4j46qwpk` 2.7k~4.6k 구간 | Round 4 후반 70k+, junsang_v2 `zn7xrm7e` |

> 단일 drop transition(magnitude +200~+550) 이 γ=0.995(horizon ~200 step)로 거슬러 모든 transition에 파급 → oscillation은 SAC + sparse reward의 본질적 특성. **버그가 아니다.**

---

## 처방

### 적용된 안전망 (Round 3+)

- LR 3e-4 → 1e-4 (update 점진화)
- tau 0.005 → 0.002 (target network 안정)
- PER priority cap 30 (outlier sampling 독점 방지)
- Reward Hard Cap [−200, +300]
- Sigmoid altitude penalty (지수 폭주 차단)

### 핵심 원칙 (Round 5)

1. **per-step density를 바꾸지 말 것.** hover/loitering exploit은 **terminal signal**로 차단 (Round 5 Hover Terminal Penalty: 종료 시 −15 1회 → per-step density 보존, SAC 안정).
2. per-step weight를 부득이 바꾸면 짧은 run에서 `ent_coef` 추세부터 확인 (단조 감소 = 정상).
3. 발산 의심 시: 학습 중단 → 마지막 안정 체크포인트 → 원인 분석 → fresh start.

### 추가 후보 (미적용)

gradient clipping, distributional RL, CQL, EMA policy.

---

## 관련 노트

- [[research/rl_rules]] — Rule 8 (density 변경 금지), Rule 9 (post-success regression)
- [[experiments/exp_004_round5_hover_junsang]] — Round 4 발산 / Round 5 설계
- [[research/reward_design]]
- [[experiments/training_history]]
- [[daily/daily_2026-05-31_junsang]]
