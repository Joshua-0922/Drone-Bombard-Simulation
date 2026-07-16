---
date: 2026-07-15
tags: [research, isaac, controller, reset, handoff, bad-attitude, v11]
status: active
type: research
owner: junsang
---

# Isaac cruise 핸드오프 — reset 시 컨트롤러 setpoint seed

> **한 줄:** 드론을 **이미 움직이는 속도로 spawn**할 때는, reset에서 속도 컨트롤러
> 상태(`_v_filt`, `_prev_action`)를 그 속도로 **seed**해야 첫 스텝 자세 폭주를 막는다.
> **발견:** [[experiments/exp_006_v11_dryrun_junsang]]

---

## 현상

v11(완화 테스트)은 "cruise하다 넘겨받는" 시나리오라 드론이 `cruise_speed=4 m/s`로
**이미 순항 중** spawn한다. 그런데 학습이 전혀 안 됨:

- `Mean episode length 1.00` — 모든 에피소드가 첫 스텝에 종료
- `Episode_Termination/bad_attitude 1.0`
- 학습 신호 0 (reward 부호만 왔다갔다)

## 원인 (캐스케이드 속도 컨트롤러 전이)

캐스케이드 컨트롤러는 **속도 setpoint(`_v_filt`, LPF)**를 실제 속도와 비교해 목표
가속도 → 목표 tilt → rate → body torque로 변환한다.

`super()._reset_idx`는 매 reset마다:
- `_v_filt[env_ids] = 0`, `_lpf_snap[env_ids] = True`
- `_prev_action[env_ids] = 0`

로 컨트롤러를 **정지 기준으로 초기화**한다. 부모 env는 거의 정지(`init_vel_std`)로
spawn하므로 이 초기화가 실제 속도와 일치 → 문제 없음.

하지만 v11은 실제로 **4 m/s로 이동 중**인데 setpoint는 0/랜덤. 첫 스텝:
1. `_lpf_snap=True` → `_v_filt`가 첫 정책 명령(랜덤)으로 snap
2. 실제 4 m/s vs setpoint(랜덤) → **~cruise_speed 크기의 속도오차**
3. 컨트롤러가 이를 없애려 **급격히 tilt** → `ang_vel > limit_ang_vel(2.0)` 또는 tilt 초과
4. `bad_attitude` 종료 → 에피소드 즉사

즉 **spawn 운동상태와 컨트롤러 초기 setpoint의 불일치**가 첫 스텝 코너 케이스를 만든다.

## 수정 (규칙화)

reset에서 컨트롤러를 **cruise setpoint로 seed**:

```python
a = self.cfg.action
cruise_vx = self._cruise_unit[0] * self.cfg.cruise_speed
cruise_vy = self._cruise_unit[1] * self.cfg.cruise_speed
self._prev_action[env_ids, 0] = clamp(cruise_vx / a.vx_scale, -1, 1)  # 정규화 명령
self._prev_action[env_ids, 1] = clamp(cruise_vy / a.vy_scale, -1, 1)
self._v_filt[env_ids, 0] = cruise_vx   # world-frame 속도 setpoint
self._v_filt[env_ids, 1] = cruise_vy
```

- `_prev_action`을 cruise로 두면 `rate_limit(0.2)`이 첫 스텝 명령을 cruise 근처로 묶어
  `_vel_cmd ≈ cruise` → snap된 `_v_filt`도 cruise ≈ 실제 → **속도오차 소멸**.
- 물리적 의미: "이미 순항 중인 드론을 넘겨받아, 명령이 없으면 순항을 유지한다."

**결과:** 에피소드 길이 1 → 40+ → 정상 학습, dry-run 100% success.

## 적용 규칙 (일반화)

> **spawn 운동상태 ≠ 정지**일 때는, 컨트롤러의 내부 setpoint/필터/prev_action을 그
> spawn 상태로 함께 초기화하라. 컨트롤러 상태를 0으로만 두면 첫 스텝에 큰 추종오차 →
> 급기동 → 종료 조건(bad_attitude/overspeed) 즉발.

향후 확장 시 주의:
- cruise_speed를 올리면 오차 여유는 seed로 해결되지만, `vx_scale`(=4)보다 크면 정규화
  명령이 clamp되어 setpoint < 실제 → 잔류 오차. 필요 시 `vx_scale`도 함께 상향.
- 이동타겟/DR 확장 시에도 spawn 운동상태를 바꾸면 이 seed 로직을 반드시 재검토.

관련: [[experiments/exp_006_v11_dryrun_junsang]], [[research/rl_rules]] (Rule 10)
