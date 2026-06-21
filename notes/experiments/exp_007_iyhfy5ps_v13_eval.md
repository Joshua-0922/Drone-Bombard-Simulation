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
- **블로커(해결됨):** EKF 발산 흡수 루프 = **누적 leaked YOLO + stale 프로세스** (fundamental EKF 버그 아님).

---

## ✅ 06-21 수정 구현 & 검증

**3가지 코드 변경 (`drone_drop_env.py` / `evaluate.py` / `hyperparams_v13.yaml`):**
1. **health gate** (reset() step 8b): TRACKING 후 `d_xy_prev > start_drift_max(5.0)`면 corrupted start →
   full restart + progressive settle 후 retry, max_retries(6) 초과 시 abort. config `start_drift_*`.
2. **YOLO 누수 fix** (진짜 근본 원인): `_start_infra` fresh-start kill 리스트에 `xmarker_detector` 추가
   — 기존엔 restart마다 YOLO 노드를 안 죽이고 새로 spawn → 누적(3개) → 충돌 탐지 → spurious TRACKING.
3. **`evaluate.py` 재작성:** success_rate + step-to-reach + final/closest d_xy(obs[12,13]에서 복원) +
   outcome breakdown. 죽은 `info['drop_error_actual_m']`(NaN) 의존 제거.

**진단 결정타:** gate 첫 retry `d_xy=11.4m while TRACKING (conf=0.82)` — 카메라는 마커 봄(드론 물리적으로 위)인데 EKF는 home → EKF 위치 freeze.

**Dry-run 검증 (clean slate: 전 sim teardown + YOLO fix):**
| | 결과 |
|---|---|
| Health gate fires | **0** |
| Success rate | **3/3 (1.000)** |
| Handoff d_xy | 0.9m, conf 0.96 (정상) |
| Mean reward | 162.4 |
| evaluate.py report | NaN 없음, 신규 지표 정상 |

→ **발산은 누적 degradation이 원인. clean 상태에선 teleport-EKF 정상.** 원래 ep 1–3 성공도 이로써 설명.

## ✅ 전체 20-ep clean eval 결과 (06-21, 확정)

| 지표 | 값 |
|------|-----|
| **Success rate** | **0.800 (16/20)** |
| Health-gate fires | **0** (EKF-drift 0, timeout 0) |
| Mean steps-to-success | 54.9 (median 41.5) |
| Mean closest d_xy | 0.812 m |
| Best closest d_xy | 0.709 m |
| Mean episode reward | 134.0 (std 39.3) |
| Outcome breakdown | 16 success · **4 stagnation** · 0 ekf_drift |

- **fixes 검증 완료:** 20-ep 전부 gate 0 / EKF-drift 0 — 이전 run(10/13 drift 쓰레기)과 대조. eval이 이제 **정책**을 측정.
- **80% ≈ 학습 시 success ~82%** → 신뢰 가능한 수치(우연 윈도우 아님).
- **4개 실패 전부 동일 모드 = 종단 stagnation**(ep 7/8/9/13, closest 0.81–1.09m). 정책이 ~0.8m까지 안정적으로
  접근하나 0.8m success gate를 꿰는 마지막 0.2m를 못 닫고 호버→stagnation 절단. ep8은 min 0.80m인데도 경계 밖.
  → v13의 축소된 action authority(vx4/vy3)와의 trade-off (overshoot 방지 ↔ 종단 정밀도 한계).
- **>80% 레버:** success_radius 0.8→1.0 또는 커리큘럼 / 종단 authority 소폭↑ / stagnation 컷 완화.
- 산출물: `/workspace/ros2_ws/rl_eval_results/` (report.md, summary.json, *.png)

→ 규칙화: [[research/rl_rules]] Rule 12 / [[research/eval_terminal_env_metrics]]
