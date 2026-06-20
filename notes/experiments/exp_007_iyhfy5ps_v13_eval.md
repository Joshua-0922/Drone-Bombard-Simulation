---
date: 2026-06-20
tags: [experiment, evaluation, SAC, v13, EKF, deterministic]
status: complete
type: experiments
wandb_run: iyhfy5ps (training source; eval 자체는 WandB 미연동)
---

# Exp 007 — v13 정책 평가 (deterministic eval)

> 학습 run `rl_yolo_v13_terminal_reward` (iyhfy5ps)의 157.7K-step 스냅샷을 deterministic으로 평가.
> 관련: [[experiments/exp_006_xgzum51v_armdiag_dryrun]] · [[research/eval_terminal_env_metrics]] · [[research/cruise_timeout_arming]]

---

## 설정

| 항목 | 값 |
|------|-----|
| 모델 | `rl_checkpoints/sac_drop_preempt.zip` (SIGTERM 저장, **~157.7K steps**) |
| 스크립트 | `ros2 run rl_navigation evaluate` |
| config | `hyperparams_v13.yaml` (학습 env와 동일) |
| 요청 에피소드 | 20 (13개 실행 후 수동 중단) |
| 로그 | `/workspace/eval_v13.log` |
| 사전 작업 | 학습(iyhfy5ps) SIGTERM graceful stop → preempt + 70MB replay 저장(재개 가능) |

---

## 결과

### ✅ 유효 에피소드 (드론이 정상 시작) — 3/3 성공

| Ep | Reward | 결과 | step-to-reach |
|----|--------|------|---------------|
| 1 | 126.35 | SUCCESS (d_xy=0.80m) | 72 |
| 2 | 114.46 | SUCCESS (d_xy=0.80m) | 41 |
| 3 | 131.81 | SUCCESS (d_xy=0.80m) | 63 |

- **평균 reward 124.2** — 학습 stochastic `ep_rew_mean ≈ 100`보다 *높음* (deterministic 정책의 정상 신호).
- 깨끗하게 시작한 에피소드는 100% 0.8m 성공원 도달 → **정책 자체는 양호.**

### ❌ 무효 에피소드 (4–13, 10개) — EKF divergence 루프

- 전부 **step 1에서 truncation**, `d_xy ≈ 11.5–12.1m > 5.0m` 가드, **reward −15.00**.
- step 1 d_xy ≈ home→target 거리(~11.9m) → EKF 위치 추정이 실제 대비 ~12m 발산.
- 카메라는 마커 탐지(→TRACKING 전이) 정상, 그러나 d_xy에 쓰이는 **EKF 위치만 쓰레기**.
- 자기강화 루프: drift truncation → `EKF drift flag` → full infra restart → 여전히 drift → 반복. **자체 회복 안 됨** → 수동 중단.

---

## 진단

`reset()`의 EKF-drift fast path(L453-466)는 restart 후 정상 reset로 **재귀**하므로 CRUISE를 건너뛰진 않음.
문제는 **반복적 rapid full-infra restart가 PX4 EKF를 ~21s 안에 충분히 수렴시키지 못해** 위치가 발산하는 것.
이는 06-17 진단의 *teleport 후 EKF 재수렴 13–16s* 문제와 동일 뿌리이며, eval에서는 연속 restart로 더 악화되어
한 번 빠지면 못 나오는 흡수 상태(absorbing loop)가 됨.

### 부수 발견 (harness 결함)
1. **`evaluate.py` miss-distance/CEP/drop-speed 컬럼 = dead code.** `info['drop_error_actual_m']`를 읽지만
   현재 env는 그 키를 emit하지 않음 → 전부 NaN. reward만 유효.
2. **v13 env는 0.8m 성공원에서 종료**(탄도 투하 미모델링) → "CEP"는 이 env에서 실재 수치가 아님.
   올바른 지표는 **success rate + step-to-reach**.

---

## 결론 & 다음

- **정책:** 추가 학습 불필요 신호 (plateau + 깨끗한 시작 100% 성공).
- **블로커:** restart 후 EKF 재수렴이 신뢰 가능한 20-ep eval을 막음.
- **다음:**
  1. **에피소드 시작 health gate** — step 1 직전 EKF 위치 ↔ 카메라-마커 일치(또는 fresh infra 후 settle-wait) 확인,
     불일치면 −15 대신 retry. (학습 throughput에도 동일 이득.)
  2. `evaluate.py` 패치 — success-rate/step-to-reach 보고, miss-distance 사망 컬럼 제거.
  3. 위 수정 후 20-ep eval 재실행.

→ 규칙화: [[research/rl_rules]] Rule 12 / [[research/eval_terminal_env_metrics]]
