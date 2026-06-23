---
date: 2026-06-22
tags: [experiment, throughput, soft-reset, EKF, reset, ablation, v14]
status: active
wandb_run: byxyaf4d (rl_yolo_v14_softreset, full run) / EKF-AB + proto were offline
type: experiments
---

# Exp 009 — 리셋 처리량: EKF param A/B (실패) → soft reset (성공 ~3.9×)

> **문제:** v14 학습 fps≈2, ETA ~2.5일. 에피소드마다 CRUISE timeout(~42s 대기) + full infra restart(~22s).
> **근본 원인(확정):** teleport(`gz_reset_poses`)+disarm 후 PX4 **EKF 추정기 재수렴** 대기. `ctrl_0.log`:
> `Delaying arm — pre_flight_checks_pass=False (EKF not yet reconverged)`. fresh restart도 동일하게 timeout.
> 관련: [[research/reset_throughput_bottleneck]] · [[research/rl_rules]] Rule 14 · [[research/cruise_timeout_arming]] (Rule 11)

---

## Part A — EKF/GPS param A/B (실패, 음성 결과)

가설: PX4 GPS pre-arm 품질 게이트(`EKF2_GPS_CHECK`)가 arming 지연 원인 → 0으로 끄면 빨라질 것.

| 지표 | A baseline (v14) | B (`EKF2_GPS_CHECK 0`) |
|------|------------------|------------------------|
| handoffs/min | 0.93 | 0.98 |
| timeouts/handoff | 0.80 | 0.83 |
| restarts/min | 0.89 | 0.89 |

**→ 차이 없음(노이즈 범위). 음성.** 이유: airframe가 이미 `COM_ARM_WO_GPS 1`이라 GPS는 arming 게이트가
**아니었음**. 실제 게이트 = EKF 추정기 validity/innovation 수렴(attitude/velocity/position) — **이를 끄는 param은
설계상 없음**(안전 게이트). airframe `param set EKF2_GPS_CHECK 0` 추가 후 측정, 이후 baseline로 revert.
→ **param/timeout 레버는 고갈.** (이전 arm_bail 10→20, mag check off 등으로 이미 합리적 작업 완료됨.)

## Part B — Soft reset prototype (성공 ~3.9×)

핵심 아이디어: **teleport/disarm 안 함.** 에피소드 종료 시 드론이 flyable이면 armed+airborne 유지한 채
position setpoint로 출발점(0,0,10m)까지 **날아서 복귀** → mission_manager FSM만 재시작 → CRUISE→TRACKING
재핸드오프. **EKF가 한 번도 교란되지 않음 → preflight 재수렴 대기 0.** drone_controller는 살려둬서
20Hz offboard heartbeat 유지(offboard 안 끊김). flyable 아니면(전복/저고도/EKF발산) 기존 teleport+restart로 fallback.

### 결과 (9.2 min, 32 soft resets, 미학습 정책, offline)

| 지표 | A baseline | Soft-reset proto |
|------|-----------|------------------|
| **throughput** | 0.93 handoffs/min | **3.61/min (~3.9×)** |
| **fps** | ~2 | **~9 (4.5×)** |
| reset 시간 | ~65s cycle | **~11s** (mean 11.0, 9.6–12.3) |
| soft 성공률 | — | **100% (32/32 flyable, fallback 0)** |
| CRUISE timeouts | 0.80/handoff | **0** |
| infra restarts | 24/26min | **1 (startup만)** |

- **ETA 500K: ~2.5일 → ~15시간.**
- **EKF 건강(최대 불확실성 해소):** 32연속 soft reset에서 handoff d_xy 안정(첫8 mean 4.97m / 끝8 mean 5.23m,
  range 4.5–5.8m). teleport 없이도 **발산 없음.** drift guard(10m) 한참 안쪽.

### 정직한 한계 (9분/32회/미학습 — 장기 증명 아님)
1. **100% 성공률은 정책 운빨 일부.** 전 에피소드가 stagnation(안정 hover)으로 끝나 flyable 이상적. 학습된
   공격적 정책은 overshoot/고속/전복 종료↑ → fallback↑ 가능. **단 downside는 bounded**(fallback=기존 경로,
   최악도 baseline 근처, 그 이하 아님 + flyback 시도 몇 초 낭비).
2. **handoff d_xy 미세 상승**(4.97→5.23/32회). 노이즈 or 매우 느린 EKF 누적 — 9분으론 판별 불가. 누적이어도
   10m guard 넘으면 그 에피소드만 teleport fallback으로 self-correct. **장기 run으로 bounded 확인 필요.**
3. 11s는 여전히 1 m/s cruise가 지배(flyback ~3s + mm restart ~3s + cruise-to-handoff ~5s). 추가 최적화 여지.

## 코드 변경 (미커밋)
- `drone_drop_env.py`: `publish_position`(Vector3→/drone/cmd/position) + `_try_soft_reset` + `_kill/_start_mission_manager`
  (controller 유지, mm만 재시작) + proc tracking(`_mm_procs`/`_ctrl_proc`) + `reset()`에 soft 경로(성공 시 early-return,
  실패 시 기존 teleport 경로 fallthrough).
- `hyperparams_v13.yaml`: `soft_reset_enabled: true`, `soft_reset_altitude: 10.0`.

## 다음
- **full run `rl_yolo_v14_softreset` (byxyaf4d, 500K, soft reset ON, online) 기동.** 검증 항목:
  ① 수천 회 soft reset에서 EKF drift bounded? ② 정책 성숙 시 fallback율? ③ 실제 fps/ETA.
- ①② 양호하면 **commit.**

→ 규칙화: [[research/rl_rules]] Rule 14 / [[research/reset_throughput_bottleneck]]
