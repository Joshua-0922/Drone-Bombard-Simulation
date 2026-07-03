---
date: 2026-07-01
tags: [research, wobble, smoothness, control, reward, lpf]
status: active
type: research
---

# RL 인수 후 wobble = smoothness-control 문제 (진단·교정)

역링크: [[experiments/exp_011_wobble_lpf_reward_damping]] · [[research/rl_rules]] Rule 15 · [[research/terminal_overshoot_trap]]

## 증상
10 m 고도 핸드오프로 RL 제어 윈도우는 확보됐으나, RL 인수 후 드론이 **wobble**(순항 POSITION 제어의 매끈함과 대비). 사용자: 순항처럼 안정 이동 원함.

## 원인 (정량 규명)
1. **탐험 노이즈 아님.** eval은 `deterministic=True`. wobble은 학습된 oscillatory 정책 자체.
2. **정책이 bang-bang.** PX4가 받는 속도 명령의 **jerk RMS = 2.92 m/s / 50 ms sample**(축별 std ~2.8 m/s). 즉 50 ms마다 명령이 ~m/s 단위로 튐 → 자세 흔들림 → 시각적 wobble.
3. **왜 그렇게 학습됐나:**
   - **과도한 액션 권한**: `action_vx_scale=8`, `vy=5` m/s. [-1,1] 정책 출력의 작은 변화가 m/s 단위 속도 스윙.
   - **순수 진행 보상 + 거의 0인 댐핑**: `w_dist=2.0`(진행)에 비해 `w_ang_vel=w_action_smooth=0.05`(40×약함), **속도 크기 페널티 부재**. → 최적 정책 = 표적 향해 질주→초과→역질주.
   - **출력 평활 없음**: RL 속도 setpoint가 PX4로 직행(`publish_velocity_setpoint`).

## 교정 (증상 vs 원인)
- **증상(로직): 출력 LPF.** 컨트롤러 20 Hz 루프에 EMA(`velocity_lpf_alpha`, 0.4≈75 ms tau). **A/B: command jerk −45%, 평균 속력 불변(1.13×).** 값싸고 기존 모델에도 즉효지만, 정책이 진동을 원하면 lag만 추가 → 원인 교정 병행 필수.
- **원인(보상):**
  - **(B) 근접-게이팅 속도 댐핑** `−w_vel·speed_xy·max(0,1−d_xy/R)`: **먼 구간 0(순항 자유)**, 마커 근처서만 속도 비용. 위치 컨트롤러가 표적으로 감속하는 것을 모사. `w_vel=0.15, R=4 m`.
  - **(C) smoothness 가중 ↑** `w_ang_vel 0.05→0.15`, `w_action_smooth 0.05→0.20`. jerk만 억제(전진 속도 무관).
  - **LPF는 학습 시에도 켠다**(train==deploy plant) → 정책이 필터된 플랜트 상대로 학습.

## 향후 조건 / 레버 우선순위
1. 보상 shaping(B+C) + LPF로 Fresh 재학습 → wobble·success 재측정.
2. 부족 시 **액션 스케일 vx/vy ↓**(근본 권한 축소; 이번엔 속도 유지 위해 보류).
3. 구조적: **이전 액션을 observation에 추가**(현재 14-dim에 없음) → smoothness 학습성↑(obs 형상 변경 → fresh 필요).

## 진단 재현 도구
`vel_logger.py`(`/fmu/in/trajectory_setpoint.velocity` → CSV) + jerk RMS = 연속 차분 RMS. `velocity_lpf_alpha` 1.0(raw) vs 0.4(filtered)로 A/B.
