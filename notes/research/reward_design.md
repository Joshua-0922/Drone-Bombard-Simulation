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

---

## 관련 링크

- [[experiments/exp_001_8otphxy8_linear_reward]] — 선형 보상 첫 적용 run
- [[experiments/exp_002_reward_shaping_patches]] — anti-milking + truncation penalty
