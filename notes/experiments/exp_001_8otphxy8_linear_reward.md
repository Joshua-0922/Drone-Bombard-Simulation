---
date: 2026-03-20
tags: [experiment, SAC, linear-reward, CRUISE-retry]
status: completed
type: experiment
wandb_run: 8otphxy8
---

# Exp 001 — 선형 거리 보상 + CRUISE Retry

> **WandB Run:** `8otphxy8`
> **기간:** 2026-03-20
> **시작 체크포인트:** `sac_drop_95000_steps.zip` (clean, Mar 18)
> **결과:** 마지막 정상 베이스라인 체크포인트

---

## 변경 사항

### 선형 거리 보상 도입

이전 지수 포텐셜:
$$R_{3,dist}^{old} = w_{dist} \cdot e^{-k_1 d_{xy}}, \quad k_1 = 1.0$$

$d_{xy} = 45\,\text{m}$일 때:
$$e^{-1.0 \times 45} \approx 6.5 \times 10^{-20} \approx 0$$

→ `env/mean_rew_dist = 0` (WandB에서 확인됨)

신규 선형 보상:
$$R_{3,dist}^{new} = w_{dist} \cdot (d_{prev} - d_{xy}), \quad w_{dist} = 1.0$$

모든 거리에서 0이 아닌 기울기 보장.

### CRUISE Retry

`reset()`에서 CRUISE 미도달 시 1회 재시도.
PX4 arm race로 인한 크래시 페널티 에피소드 방지.

---

## 주요 지표

| 지표 | 값 |
|-----|-----|
| `env/mean_d_xy` | 45.8 m (안정) |
| `env/mean_rew_dist` | **> 0** (이전 run의 0에서 개선) |
| `env/physics_glitch_count` | 0 |
| fps | ~31 |
| Steps | ~114K |

---

## 결론

- 선형 보상: 기울기 문제 해결 확인
- CRUISE retry: 타임아웃 오염 감소
- **이 run의 `sac_drop_preempt.zip`이 보상 패치 전 마지막 안전 체크포인트**

---

## 다음 실험

→ [[exp_002_reward_shaping_patches]] (패치 후 Fresh 1M-step)
