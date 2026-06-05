---
date: 2026-03-22
updated: 2026-04-14
tags: [research, reward, SAC, RL]
status: implemented
type: research
---

# 4-Layer 계층형 보상 함수 설계

> **파일:** `ros2_ws/src/rl_navigation/rl_navigation/drone_drop_env.py`
> **메서드:** `_compute_reward()`, `step()`

---

## 보상 구조 개요

$$R_{total} = R_1^{safety} + R_2^{stability} + R_3^{approach} + R_4^{drop}$$

---

## Layer 1 — 안전 페널티 (Safety)

매 스텝마다 적용 (스텝 20 이후, 고도 < 2 m 또는 과속):

$$R_1 = \begin{cases} -10 & \text{if } z < 2\,\text{m (step > 20)} \\ -8 & \text{if } \|\mathbf{v}\| > 20\,\text{m/s} \end{cases}$$

---

## Layer 2 — 안정성 페널티 (Stability)

$$R_2 = -w_{time} - w_{\omega} \|\boldsymbol{\omega}\|^2 - w_{\Delta a} \|\Delta \mathbf{a}\|^2$$

| 파라미터 | 값 | 비고 |
|---------|-----|------|
| $w_{time}$ | **0.05** | 2026-03-22 패치: 0.01 → 0.05 (5× 긴급성) |
| $w_{\omega}$ | 0.05 | 각속도 페널티 |
| $w_{\Delta a}$ | 0.05 | 액션 스무딩 |

---

## Layer 3 — 접근 보상 (Approach)

### 거리 보상 (Linear)

$$R_{3,dist} = w_{dist} \cdot (d_{prev} - d_{xy})$$

> **주의:** 지수 포텐셜 $e^{-k_1 d}$는 $k_1=1.0$, $d=45\,\text{m}$일 때 $\approx 6.5 \times 10^{-20}$ (machine zero) → 절대 사용 금지.
>
> 규칙: $e^{-k_1 d_{max}} > 10^{-4}$ 를 만족해야 함. 50 m 운용 범위에서 $k_1 < 0.18$.

$w_{dist} = 1.0$

### 방향 보상 (Orientation, Anti-Milking 적용)

$$R_{3,orient} = w_{heading} \cdot \cos\theta_{target} \cdot g_{speed}$$

$$g_{speed} = \min\!\left(\frac{v_{xy}}{2.0},\, 1.0\right)$$

> **Anti-Milking (2026-03-22):** 속도 < 0.1 m/s (hover)이면 $g_{speed} \approx 0$ → 방향 보상 없음.
> 이전에는 제자리에서 카메라를 표적에 맞추는 것만으로 500스텝 × $w_{heading}$ >> 투하 보상이 가능했음.

$w_{heading} = 1.0$

---

## Layer 4 — 투하 보상 (Drop, Terminal)

$$R_4 = w_{drop} \cdot \exp(-k_2 \cdot d_{error}) + r_{jackpot} \cdot \mathbf{1}[d_{error} \leq 0.1\,\text{m}]$$

| 파라미터 | 값 |
|---------|-----|
| $w_{drop}$ | 50 |
| $k_2$ | 5.0 |
| $r_{jackpot}$ | 100 |

- $d_{error}$: `drop_calculator` → `/rl/drop_error` (실제 Gazebo 물리 결과, 10 s timeout)
- Auto-drop 조건: $d_{xy} \leq 0.5\,\text{m}$ (2D 수평 거리)

---

## Truncation Penalty (2026-03-22 신규)

$$R_{trunc} = -50 \quad \text{if step = 500 and not dropped}$$

투하 없이 에피소드 종료 방지 (anti-quitting).

---

## 변경 이력

| 날짜 | 변경 내용 |
|------|----------|
| 2026-03-20 | 선형 거리 보상 도입 (지수 포텐셜 → 선형), $w_{dist}$ 10.0 → 1.0 |
| 2026-03-22 | Anti-milking speed gate, $w_{time}$ 0.01→0.05, Truncation penalty −50 추가 |
| 2026-05-22 | **branch `jekyun_v2`(reward v3) 채택.** $w_{drop\_base}$ 50→100, $k_2$ 5.0→0.3(먼 거리도 보상), NEW $drop\_attempt\_bonus$=150, $truncation\_penalty$=−80. (이후 N1=B per-step 강화 w_impact=8은 실패로 폐기) |
| 2026-05-25 | **Round 1: hybrid drop + terminal scale 축소.** manual drop 비활성, random_drop(step≥150, 0.5%/step) + auto_drop 병행. $w_{drop\_base}$ 100, $k_2$ 0.2 |
| 2026-05-26 | **Round 2: orbit-milking 대응.** $w_{dist}$ 0.5→1.0, $w_{heading}$ 0.3→0.7, proximity bonus 추가($30e^{-0.15 d_{xy}}$), random_drop_start 150→600, max_steps 500→800. success 16건(16배↑) |
| 2026-05-30 | **Round 3: 발산 방어.** Drop 고도 페널티 지수→Sigmoid(max −50), max_alt truncate 50m, Reward Hard Cap [−200,+300] (지수 페널티 −6.77e9 폭주 후 교체) |
| 2026-05-31 | **Round 4 (폐기): per-step density 변경 → SAC 발산.** $w_{heading}$ 0.7→0.3 + $w_{distance\_penalty}$ 0.03 신규 → ent_coef 6.03 폭주. → [[research/sac_reward_density_junsang]] |
| 2026-05-31 | **Round 5: Hover Terminal Penalty.** Round 4 복원($w_{heading}$ 0.7, distance_penalty 0) + episode 종료 시 −15 (sustained hover만, drop 시 제외). per-step density 보존 |

> ⚠️ **현행 값은 이 노트와 다름.** 위 수식·표는 2026-03-22 시점 설계 기준이다. 5월 Round 1~5의 최신 파라미터·드롭 메커니즘 전체는 `local/design/design_review.md`(최종 설계) 및 `local/parameter_log.md`(키별 변경 이력) 참조.

---

## 관련 링크

- [[experiments/exp_001_8otphxy8_linear_reward]] — 선형 보상 첫 적용 run
- [[experiments/exp_002_reward_shaping_patches]] — anti-milking + truncation penalty
- [[experiments/exp_004_round5_hover_junsang]] — Round 4 발산 + Round 5 hover terminal penalty
- [[research/sac_reward_density_junsang]] — per-step density와 SAC 발산
- [[research/rl_rules]] — Rule 8(density), Rule 9(post-success regression)
