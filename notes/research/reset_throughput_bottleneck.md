---
date: 2026-06-22
tags: [research, throughput, reset, EKF, soft-reset, px4, teleport]
status: active
type: research
---

# 리셋 처리량 병목 = teleport 후 EKF 재수렴 (soft reset로 회피)

> **한 줄:** 에피소드 리셋의 지배적 비용은 `gz_reset_poses`(teleport)+disarm 후 PX4 **EKF 추정기 재수렴**
> 대기다. param/timeout으로는 못 줄인다(안전 게이트, 바이패스 param 없음). **teleport를 안 하는 soft reset**이
> 유일하게 통하는 레버다(검증: ~3.9× 처리량). 검증: [[experiments/exp_009_softreset_throughput]] · Rule 14

---

## 확정된 근본 원인

v14(fps≈2) 리셋 사이클 ~65s = CRUISE timeout 대기(~42s) + full infra restart(~22s). 계측:
- `ctrl_0.log`: **`Delaying arm — pre_flight_checks_pass=False (EKF not yet reconverged after reset)`**
- **fresh PX4 restart도 동일하게 timeout** (restart > handoff): 즉 teleport뿐 아니라 PX4 cold-start도 EKF 수렴 대기.
- 측정: READY→TIMEOUT ~42s, restarts ≈ handoffs (거의 매 에피소드 1회 timeout+restart).

`pre_flight_checks_pass`는 commander의 EKF validity/innovation 게이트. teleport는 위치 점프(또는 cold-start)로
EKF attitude/velocity/position 추정 innovation을 크게 만들고, 그게 tolerance 안에 들어올 때까지(06-17 armdiag의
**bimodal 0s/13–16s**) arm이 막힌다. 이것이 Rule 11(arm_bail)에서 다룬 현상의 뿌리.

## 안 통하는 것 (param/timeout) — 음성 결과

- **`EKF2_GPS_CHECK 0` A/B = 차이 없음.** airframe가 이미 `COM_ARM_WO_GPS 1` → GPS는 게이트가 아님.
  실제 게이트 = EKF 추정기 수렴, **이를 끄는 param은 설계상 없음**(안전).
- 이전 작업(arm_bail 10→20s, mag check off, fast-path 제거)도 전부 **증상 완화**였지 원인 제거 아님.
- → **param/timeout 레버 고갈.** "여러 번 시도했는데 안 됐다"는 사용자 경험과 일치하며, 이유가 규명됨.

## 통하는 것 — Soft reset (teleport 회피)

에피소드 종료 시 드론이 flyable이면:
1. **disarm/teleport 안 함.** armed+airborne 유지.
2. position setpoint로 출발점(0,0,cruise_alt) 날아서 복귀. `drone_controller` 살려둠 → 20Hz offboard
   heartbeat 지속 → offboard 안 끊김(failsafe 없음, 재arm 없음).
3. mission_manager FSM만 kill+restart → TAKEOFF(이미 고도/arm) → CRUISE → 재핸드오프.
4. flyable 아니면(전복/저고도/EKF발산/비유한) **기존 teleport+restart로 fallback**(downside bounded).

**EKF가 연속 비행 내내 교란 안 됨 → 재수렴 대기 0.** 결과: reset ~65s→~11s, fps 2→9, throughput ~3.9×,
32연속 soft reset에서 EKF d_xy 안정(4.5–5.8m, 발산 없음). 상세: [[experiments/exp_009_softreset_throughput]].

## 적용 규칙 (→ Rule 14)
- **리셋 처리량 문제는 "EKF를 교란하지 않는 리셋"으로 풀어라.** teleport/disarm은 PX4 EKF 재수렴을 강제하고,
  그 대기는 param으로 못 줄인다. 연속 비행 복귀(soft reset)가 그 비용을 0으로 만든다.
- **항상 fallback을 둬라.** 전복/EKF발산/저고도 종료는 soft reset 불가 → 기존 teleport+restart로. 최악도 baseline.
- **drift guard가 self-correct 장치다.** soft reset로 EKF가 느리게 누적되더라도 handoff d_xy > start_drift_max면
  그 에피소드만 teleport fallback → 누적 리셋. soft reset + drift guard = 빠르되 안전.

## 향후 조건 / 열린 질문
- ✅ **장기 EKF drift bounded — 확인됨** (byxyaf4d 학습 0→196.5K, 3096 soft resets). health gate(10m) 안쪽 유지, 발산 루프 없음. exp_009의 미세 상승(4.97→5.23/32회)은 노이즈였음(누적 아님).
- ✅ **정책 성숙 시 fallback율 — bounded** (학습 정책에서 soft 성공 ~91%, fallback ~9%만 teleport). 공격적 종료가 일부 fallback 유발하나 downside bounded.
- **→ Rule 14 production 검증완료** ([[experiments/exp_010_byxyaf4d_v14_195k_eval]]).
- (잔여) 11s 중 1 m/s cruise가 지배 → flyback/cruise 속도 상향으로 추가 단축 여지.

## 역링크
- [[experiments/exp_009_softreset_throughput]] — A/B + 프로토타입 측정
- [[experiments/exp_010_byxyaf4d_v14_195k_eval]] — 장기 검증(3096 resets) + 195K eval
- [[research/cruise_timeout_arming]] — 같은 뿌리(EKF 재수렴 bimodal), arm_bail 증상 완화 (Rule 11)
- [[research/rl_rules]] Rule 14
