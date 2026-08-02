---
date: 2026-05-31
tags: [experiment, round4, round5, hover, sac, divergence, reward-density]
status: in-progress
type: experiment
wandb_run: "4j46qwpk (Round 4) / sdjytkpv (Round 5)"
---

# Exp 004 — Hover Exploit 대응: Round 4 발산 → Round 5 Terminal Penalty

> **목적:** Round 3에서 발견된 hover exploit(스폰 근처 정지 후 random_drop 대기)을 차단한다.
> **핵심 교훈:** per-step reward density를 건드리면 SAC가 발산한다 → terminal signal로 우회.
> 상세 출처: `local/issues/issue_017_hover_exploit.txt`, `local/meeting_notes/meeting_notes_2026-05-31.txt`

---

## 배경 — Hover Exploit (Issue #017)

Round 3(`lidq3ydu`, 157k) 분석에서 발견:

- `drop_error` 13~16m 구간에 16건(15.4%), 특히 **14.87m = √(11²+10²) = 스폰→타겟 거리**에 정확히 매칭
- `n_steps` 600+ → 드론이 스폰에서 안 움직이고 random_drop 발동까지 대기
- 25k 구간별 hover 비율: 25~50k **45%**, 100~125k **36%**

**원인:** `w_heading` 0.7 × 600 step hover = **+420** (안전한 큰 수익) vs success(+50~−50, 위험) → agent에게 hover가 합리적 선택.

---

## Round 4 (A+C 조합) — 발산 실패

### 설정

| 변경 | old → new | 의도 |
|------|-----------|------|
| `w_heading` | 0.7 → **0.3** | hover heading 수익 절반 감소 (해결책 A) |
| `w_distance_penalty` | NEW **0.03** | per-step 거리 페널티 `−0.03·d_xy/50` (해결책 C) |

- 첫 시도 `vo1l9wl6`: 14k에서 `ep_len=1` stuck — reset 버그 부작용(pos_enu z=0 → ground_contact). Fix: z=5m 마킹 → `4j46qwpk` 재시작.

### 결과 (`4j46qwpk`, 146k 중단) — 🚨 발산

| 지표 | 추이 | 정상 |
|------|------|------|
| `ent_coef` | 0.58 → 1.5 → **6.03 폭주** | 0.3~0.5 |
| `critic_loss` | 100 → 1000 → **230,000+** | 100~500 |
| `ep_len_mean` | 310 → 143 감소 | — |
| `ep_rew_mean` | −13 → −33 악화 | — |

- 초반 0~50k는 정상 학습(success 1건, drops 34건) → 후반 발산.

### 원인 분석

per-step / drop reward 비율: Round 3 **1:300** → Round 4 **1:700** (sparsity 2배).
→ SAC auto-entropy 발산 모드 트리거 (상세: [[research/sac_reward_density_junsang]]).

> **교훈:** per-step 보상 magnitude 변경은 SAC 발산 위험. hover 차단은 terminal signal로 해야 한다.

---

## Round 5 (Hover Terminal Penalty) — 학습 중

### 설계

per-step density는 그대로 두고, **episode 종료 시점에만** 1회 페널티.

| 항목 | 값 |
|------|----|
| `w_heading` (복원) | 0.3 → **0.7** |
| `w_distance_penalty` (제거) | 0.03 → **0** |
| `hover_speed_threshold` | 1.0 m/s (정지 판단) |
| `hover_consecutive_threshold` | 200 step (episode 25%) |
| `penalty_hover` | **−15** (timeout과 동일) |
| 제외 | drop 발생(terminated) 시 — 정밀 hover 정당 |

**Hover 감지(방법 A — 속도 기준 연속 정체):** Round 3 hover 클러스터(14.87m)가 "스폰에서 거의 안 움직임" = 속도 < 1 m/s와 정확히 일치 → 단순·직관적.

### 코드 변경

```
drone_drop_env.py:
  reset(): _consecutive_still = 0, _max_consecutive_still = 0
  step(): speed_xy < threshold → consecutive++; else 0; max 갱신
          episode 종료 시 (truncated AND not dropped):
            if max_consecutive_still > 200: reward -= 15
hyperparams.yaml:
  hover_speed_threshold / hover_consecutive_threshold / penalty_hover 추가
  run_name → round5_hover_terminal_penalty
```

### 핵심 장점

- per-step density 보존 → **SAC 안정성 유지** (Round 4 발산 회피)
- terminal 신호만 추가 → "잠시 hover OK, 지속 hover BAD" 자연스럽게 학습

### 결과 (`sdjytkpv`, 300k — 진행 중)

- 모니터링: hover 비율 감소 여부, `ent_coef` 단조 안정, 100k 중간 평가(avg 명중거리/success rate)

---

## 관련 노트

- [[experiments/training_history]]
- [[research/sac_reward_density_junsang]] — 발산 메커니즘
- [[research/rl_rules]] — Rule 8(density), Rule 9(post-success regression)
- [[research/reward_design]]
- [[daily/daily_2026-05-31_junsang]]
