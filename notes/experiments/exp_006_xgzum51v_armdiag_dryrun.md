---
date: 2026-06-17
tags: [experiment, dry-run, arming, EKF, teleport, throughput, arm_bail_timeout]
status: done
type: experiments
wandb_run: xgzum51v (offline)
---

# exp_006 — armdiag dry-run: v12 arm-reject는 "복구 불가"가 아니라 "10s 컷이 너무 빨랐다"

> **목적:** v13(rl_yolo_v13_terminal_reward) 학습의 지배적 처리량 병목인 `PX4 not armed after 10s` early-bail의 근본 성격 규명 — teleport 후 stuck-EKF가 **시간이 지나면 복구되는가(타임아웃만 늘리면 됨)**, 아니면 **full restart가 유일한 길인가**.
> **관련:** [[research/cruise_timeout_arming]] · [[experiments/exp_005_rl_yolo_v12_arm_fix_arming-throughput-fix]] · [[research/rl_rules]] (Rule 11)

---

## 배경 — v12 "수정"이 증상만 잡았다

v12(722737a)는 `arm-reject`를 진단하고 ① `pre_flight_checks_pass` 게이팅 ② 10s `arm_bail_timeout` early-bail을 넣었다. 그러나 v13 학습 로그(46y4xtiw, 29.9K steps / ~10h)에서:

- **`ARM REJECTED`: 0회** — PX4는 더 이상 arm을 *거부*하지 않음. v12 게이팅이 실제 거부는 제거함.
- 대신 컨트롤러가 `Delaying arm — pre_flight_checks_pass=False`로 **대기**하다가 bridge가 10s에 bail → full infra restart.
- 즉 "arm-reject suspected" 메시지는 **오해의 소지**. 실제는 *EKF가 10s 안에 재수렴하지 못함*.

남은 질문: **그 stuck-EKF가 10s를 넘기면 언제 복구되는가?**

## 계측 (instrumentation)

`drone_controller_node.py`에 추가:
- `_connect_time` = 첫 `vehicle_status` 수신 시각
- `pre_flight_checks_pass`가 **처음 True**가 되는 순간 `PREFLIGHT-PASS: ... flipped True {dt:.1f}s after PX4-connect` 1회 로깅

수집 난점: env가 에피소드마다 컨트롤러를 **재기동**하고 `/tmp/ctrl_0.log`를 `'w'`로 truncate → episode별 데이터 소실. → `tail -F`로 append-only `preflight_timings.log` 수집기 구성.

## 설정

- config: `hyperparams_v13_armdiag.yaml` (v13 복사본)
  - **`arm_bail_timeout: 10.0 → 25.0`** (재수렴 한계 관측용 넉넉한 창)
  - 격리: `checkpoint_dir: .../armdiag_dryrun`, `run_name: v13_armdiag_dryrun`, `WANDB_MODE=offline` — v13 체크포인트/리플레이/프로젝트 무접촉
- `--timesteps 1000` (조기 종료)

## 결과 — EKF 재수렴은 BIMODAL, 그리고 복구 가능

`pre_flight_checks_pass` 첫 True까지 걸린 시간 분포 (12 resets):

| flip time | count | 해석 |
|-----------|-------|------|
| **0.0 s** | 7 | warm/즉시 (fresh infra) |
| 13.0 s | 1 | stuck-EKF cohort |
| 13.3 s | 1 | |
| 13.4 s | 1 | |
| 13.5 s | 1 | |
| **15.7 s** | 1 | 관측 최댓값 |

- **late cohort(>10s) = 5/12 ≈ 42%** — v12의 `~28% NEVER ARMED`와 정합. 이들은 모두 **13–16s에 좁게 군집**.
- **bails: 0 / TRACKING timeout: 0 / SUCCESS: 4** (7 infra starts 기준).
- 즉 13.0/13.3/13.4/13.5/15.7s 케이스는 **10s 컷이었다면 전부 bail → full restart**였을 것. 25s에서는 **전부 회복**.

## 결론

> **stuck-EKF resets는 full-restart-only가 아니라 recoverable-with-time이다.** EKF는 teleport 후 13–16s에 재수렴한다. v12의 `arm_bail_timeout=10.0`이 복구 직전(3–6s 전)에 단두대질을 해서 멀쩡한 PX4를 버리고 ~25–30s full restart를 강제했다 — 이것이 v13의 진짜 처리량 싱크.

## 적용 (fix)

- `hyperparams_v13.yaml`: **`arm_bail_timeout: 10.0 → 20.0`** (관측 최댓값 15.7s + 마진; 진짜 죽은 인프라는 여전히 20s에 fast-fail).
- → [[research/cruise_timeout_arming]] 업데이트, [[research/rl_rules]] Rule 11 추가.

## 미해결 / 다음

- v13 본학습은 30K 체크포인트(`sac_drop_30000_steps.zip`) + 리플레이(`sac_drop_preempt_replay.pkl`, 06-17 11:52)로 **재개 가능** (arm_bail은 보상식이 아니므로 Fresh Start 불필요 — Rule 4 해당 없음).
- OPEN: late cohort의 13–16s 재수렴 자체를 줄이려면 teleport reset이 EKF를 덜 흔들게 해야 함(원인 미해결, 별도 과제).
- OPEN: YOLO target_lost ~29% (무관, 지속).
