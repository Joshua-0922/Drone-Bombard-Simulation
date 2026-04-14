---
date: 2026-03-22
tags: [experiment, SAC, reward-shaping, anti-milking, pending]
status: pending
type: experiment
wandb_run: TBD
---

# Exp 002 — 보상 함수 패치 적용 Fresh Training

> **상태:** ⏳ 대기 중 — Fresh Start 필요
> **시작 체크포인트:** 없음 (fresh start; 기존 replay buffer는 구 보상으로 오염)

---

## 적용된 패치 (2026-03-22)

### 1. Anti-Milking Speed Gate

$$R_{3,orient}^{new} = w_{heading} \cdot \cos\theta_{target} \cdot \min\!\left(\frac{v_{xy}}{2.0},\, 1.0\right)$$

이전에는 제자리 hover에서도 방향 보상 수령 가능:
$$500\,\text{steps} \times w_{heading} = 500 \gg R_4^{drop} = 50$$

패치 후: $v_{xy} < 0.1\,\text{m/s}$이면 보상 ≈ 0.

### 2. 긴급성 시간 페널티 강화

$$w_{time}: 0.01 \rightarrow 0.05 \quad (5\times)$$

500 스텝 기준 추가 페널티: $500 \times 0.04 = 20$.

### 3. Truncation Penalty

$$R_{trunc} = -50 \quad \text{if step=500 and not dropped}$$

투하 없이 에피소드 종료 방지.

### 4. 선형 거리 보상 유지

$R_{3,dist} = w_{dist}(d_{prev} - d_{xy})$ (Exp 001에서 이미 적용됨)

---

## Fresh Start 이유

- Replay buffer에 구 보상 공식으로 계산된 transition 저장됨
- 재개 시 critic이 혼합 보상으로 $\sim 100K$ 스텝 학습 → Q-value 추정 오염
- 보상 함수 변경 후에는 항상 fresh start (CLAUDE.md 규칙 §4)

---

## 학습 설정

| 파라미터 | 값 |
|---------|-----|
| `total_timesteps` | 1,000,000 |
| `num_envs` | 1 |
| Algorithm | SAC |
| `net_arch` | [256, 256] |
| `device` | cuda (L4 GPU) |
| 예상 fps | ~33 |

---

## 모니터링 체크리스트

- [ ] WandB 첫 롤아웃 (~2분) 후 `env/mean_rew_dist > 0` 확인
- [ ] `env/mean_rew_orient` ≠ 0 확인 (speed gate 작동)
- [ ] `env/physics_glitch_count = 0` 유지
- [ ] `ep_len_mean < 500` (에피소드가 타임아웃 전에 완료되는지)

---

## 결과 (학습 후 업데이트)

_TBD_

---

## 관련 링크

- [[experiments/exp_001_8otphxy8_linear_reward]] — 이전 베이스라인 run
- [[research/reward_design]] — 패치된 보상 함수 전체 수식
- [[research/rl_rules]] — Fresh Start 규칙 §3, §4
