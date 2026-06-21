---
date: 2026-06-20
tags: [research, evaluation, EKF, metrics, harness]
status: active
type: research
---

# v13 정책 평가 — EKF divergence 루프 & harness 지표 정합성

> 역링크: [[experiments/exp_007_iyhfy5ps_v13_eval]] · [[research/cruise_timeout_arming]] · [[research/rl_rules]] (Rule 12)

---

## 발견 1 — eval 중 EKF divergence absorbing loop

deterministic eval(20-ep 요청)에서 ep 1–3은 성공(reward ~124, d_xy≤0.8m)했으나
ep 4–13은 **전부 step 1에서 `d_xy ≈ 11.5–12.1m > 5.0m` 가드로 truncate (reward −15)**.

- `d_xy ≈ 11.9m` = **home→target 거리** → PX4 EKF 위치 추정이 실제 대비 ~12m 발산.
- 카메라/YOLO는 마커 탐지 → TRACKING 전이는 정상. 즉 **vision은 맞고 EKF position만 틀림**.
- `reset()` EKF-drift fast path(`drone_drop_env.py` L453-466)는 full restart 후 *정상 reset로 재귀*하므로
  CRUISE를 건너뛰진 않음. 근본 원인은 **연속 rapid full-infra restart가 EKF를 ~21s 안에 수렴 못 시킴**.
- 한 번 빠지면 drift→restart→drift로 **자체 회복 불가(absorbing state)**.

### 06-17 진단과의 관계
같은 뿌리(**teleport/restart 후 EKF 재수렴 지연**, [[research/cruise_timeout_arming]] Rule 11)이나
manifestation이 다름: 학습 때는 산발적 truncation으로 끝나지만, eval의 연속 restart에선
누적되어 흡수 루프가 됨. → eval은 학습보다 이 결함에 더 취약.

### ✅ 처방 (06-21 구현·검증 완료)
- **에피소드 시작 health gate** (`drone_drop_env.py` reset() step 8b, Rule 12): TRACKING 확정 후
  `d_xy_prev > start_drift_max`(=5.0)면 corrupted start → −15 카운트 대신 full infra restart +
  progressive settle(`start_drift_settle × retry#`) 후 retry. `start_drift_max_retries`(=6) 초과 시
  loop 대신 RuntimeError abort. config: `hyperparams_v13.yaml` `start_drift_*`.
- **(진짜 근본 원인) YOLO `xmarker_detector` 누수:** `_start_infra`가 fresh start마다 YOLO 노드를
  **죽이지 않고 새로 spawn** → 누적(검증 시 3개 동시 실행). 다중 detector가 충돌하는 pixel_coords
  발행 → **spurious CRUISE→TRACKING(conf=0.00) + stale 탐지** → EKF↔camera 불일치 흡수 루프.
  **Fix:** fresh-start kill 리스트(iid==0)에 `xmarker_detector` 추가.

### 🔬 진단 결정타 (health gate 로그)
gate가 잡은 첫 retry: `d_xy=11.4m > 5.0m while TRACKING (marker conf=0.82)` —
**카메라는 마커를 봄(conf 0.82 = 드론이 물리적으로 마커 위)인데 EKF는 home(11.4m)을 보고**.
→ EKF 수평 위치 추정이 실제를 추종 못 하고 freeze/diverge. 이후 retry는 conf=0.00 (stale/다중 YOLO).

### ✅ 근본 원인 = 누적 sim degradation (fundamental EKF 버그 아님)
**clean slate(전 sim 프로세스 teardown + YOLO 누수 fix) 후 dry-run 3/3 SUCCESS, gate 0회**,
handoff `d_xy:0.9m conf:0.96`(정상). → 발산은 **누적된 leaked YOLO + stale 프로세스**가 원인이었고,
teleport-EKF 자체는 clean 상태에서 정상 수렴. 원래 eval의 ep 1–3이 성공한 것도 이로써 설명됨
(누적이 ep 4부터 임계 초과). **교훈: 장기 run/연속 restart 후엔 sim을 clean teardown 후 평가.**

### evaluate.py 검증 (06-21)
새 `evaluate.py`(success_rate/step-to-reach/closest d_xy from obs[12,13]/outcome breakdown) dry-run:
3/3 SUCCESS, report NaN 없음, per-ep `reward/steps/SUCCESS/final_d_xy/min_d_xy/[outcome]` 정상 출력.

---

## 발견 2 — `evaluate.py`가 현재 env와 비정합

- `evaluate.py`는 `info['drop_error_actual_m']`를 읽어 miss-distance/CEP/drop-speed/success_rate(≤0.5m)를
  계산하지만, **현재 env는 그 키를 emit하지 않음** → 전 컬럼 NaN. `mean_episode_reward`만 유효.
- env는 성공 시 `info['success']=True`만 세팅하고 실제 도달 거리는 stdout `[RL] SUCCESS: d_xy=…`에만 로깅
  (`'d_xy'` 키는 WandB 평균 오염 방지로 **의도적 생략**, L632-634).

### v13 env는 "탄도 투하"를 모델링하지 않음
v13 terminal-reward env는 **0.8m 성공원 도달 시 종료**한다 (payload drop + 착탄 오차 측정 없음).
따라서 "CEP / drop_error / drop_speed"는 이 env에서 **실재하지 않는 레거시 지표**다.

올바른 평가 지표:
| 지표 | 정의 | 비고 |
|------|------|------|
| success_rate | d_xy ≤ success_radius 도달 비율 | 0.8m 기준 |
| step-to-reach | 성공까지 step 수 | 효율 (eval: 41–72) |
| ep_reward (deterministic) | 에피소드 총보상 | eval ~124 > 학습 ~100 |
| 실패 모드 분해 | ekf_drift / stagnation / overshoot / CRUISE t/o | harness 건전성 |

### 처방
`evaluate.py`를 success-rate/step-to-reach 보고로 교체하고, miss-distance/CEP/drop-speed 사망 컬럼 제거
(또는 탄도 투하를 실제 모델링하기 전까지 비활성).
