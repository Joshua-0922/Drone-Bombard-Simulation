---
date: 2026-06-16
tags: [research, reward, RL, SAC, overshoot, vision, terminal-guidance]
status: implemented
type: research
---

# v12 종단 보상 트랩 (Terminal "Overshoot Moat")

> **대상 코드:** `ros2_ws/src/rl_navigation/rl_navigation/drone_drop_env.py`
> **메서드:** `step()` (overshoot 가드 L732-737, success L680-683), `reset()` 핸드오프 시딩 L584-587
> **계기:** `rl_yolo_v12_arm_fix` (run-20260615_084745-61tgwc5t) 학습 정체 진단
> **수정 config:** `ros2_ws/src/rl_navigation/config/hyperparams_v13.yaml`

---

## 증상 (v12, 93K steps 시점)

| 신호 | 값 | 해석 |
|------|-----|------|
| `ep_rew_mean` | **≈ -20.5** (추세 없음, -28~+33 진동) | 학습 정체 |
| `ep_len_mean` | **40 → 16 으로 *감소*** | 정책이 "빨리 죽기" 학습 |
| success (d_xy≤0.5m) | **전체 run 0회** | 종단 정밀 실패 |
| YOLO conf / d_xy 도달 | conf 0.92–0.98, d_xy **~1.0m 일관 도달** | 접근·탐지는 정상 |

→ 인프라(arming/YOLO/cruise)는 정상. **종단 보상·에피소드 설계 문제.**

---

## 근본 원인 — 핸드오프가 트랩 안에서 시작

### 1. RL 에피소드는 핸드오프(d_xy≈1.0m)에서 시작한다

`reset()` (L584-587)은 **CRUISE→TRACKING 핸드오프 위치**에서 `d_xy_prev`를 시드한다.
v12에서 카메라를 **정하방(straight-down, pitch 180°→90°, commit `24135e9`)**으로 바꾼 뒤로는
마커가 **드론이 거의 머리 위(~1.0m)일 때만** 화면에 들어온다 → 핸드오프 = d_xy≈1.0m.

> v11(전방·경사 카메라)은 마커를 멀리서 탐지 → 핸드오프가 표적에서 *멀리* →
> RL이 접근 활주로(runway)를 확보 → 404 successes, best d_xy 0.68m.
> v12(정하방)는 활주로 없이 **트랩 한가운데**에서 시작. → v11→v12 회귀의 정체.

### 2. Overshoot 가드가 step 1부터 무장된다

```python
# drone_drop_env.py L732-737
if (self._d_xy_min < overshoot_close_threshold(1.5)
        and d_xy > self._d_xy_min + overshoot_margin(1.5)):
    reward += penalty_overshoot(-20); truncated = True
```

핸드오프 d_xy≈1.0m < `overshoot_close_threshold`(1.5) → **가드가 첫 스텝부터 무장.**
이후 최근접점에서 1.5m만 밀려나도 **-20 + 에피소드 종료.**

### 3. 8 m/s 액추에이터로 0.5m 원을 못 꿴다

`action_vx_scale=8`, `action_vy_scale=5` m/s. 1.0m에서 0.5m 성공원으로 제동하려면
한 제어스텝(~10Hz)에 0.5–0.8m 이동 → 성공원을 지나쳐 overshoot 무장 조건 발동.

### 4. 종단 보상 지형이 비대칭 (위험 >> 보상)

- 거리보상 `r3_dist = w_dist·(d_prev-d_now)`는 에피소드 전체에서 망원경처럼 합산 → **`2.0·(1.0-0) = 단 +2.0`.**
- `r3_proximity` @1m = `0.3·(1-1/5) = 0.24/step`.
- `r3_vision` ≤ `1.5·centering·conf·(1-1/30) ≈ 0.7/step`.
- 성공 +100은 0.5m 뒤에 있고, 그 앞을 **-20 해자(moat)**가 둘러쌈.

⇒ 정책은 "표적으로 돌진 = 위험"을 정확히 학습 → **~1.0m에서 호버.**
`ep_len` 40→16 감소 = 성공을 못 찾으니 **종료 페널티를 빨리 트리거**하도록 수렴 (Rule 9의 착시가 아니라 진짜 트랩).

---

## 수치로 본 에피소드 (관측 -20.5와 일치)

```
시작 d_xy≈1.0m → ~16 step 기동 (proximity+vision 누적 +5~+14)
 → overshoot 무장 조건 발동 → -20 → 종료
 합계 ≈ -6 ~ -15, 관측 -20.5 (overshoot -20 지배)
```

---

## 수정 (v13, config-only — 정하방 카메라는 유지)

| 파라미터 | v12 | v13 | 이유 |
|---------|-----|-----|------|
| `overshoot_close_threshold` | 1.5 | **0.6** | 핸드오프(~1m)에서 무장하지 않도록. 성공원(0.8) 안에서만 무장 → 종단 접근을 죽이지 않음 |
| `overshoot_margin` | 1.5 | **2.0** | 종료 전 여유 ↑ |
| `penalty_overshoot` | -20.0 | **-10.0** | ep_rew를 -20에 못박던 지뢰 완화 |
| `success_radius` | 0.5 | **0.8** | 도달 가능한 게이트 → +100 신호 최초 발생. 커리큘럼: 안정 후 0.8→0.5 |
| `action_vx_scale` | 8.0 | **4.0** | 종단 제동력. RL phase는 ~1m에서 시작하므로 8 m/s 불필요 |
| `action_vy_scale` | 5.0 | **3.0** | 동상 |
| `w_proximity` | 0.3 | **0.6** | 에피소드 전부 0–2m 영역 → pull-in 기울기 강화 |
| `proximity_radius` | 5.0 | **2.0** | 밀집 보상을 실제 작동 영역에 집중 |

> ⚠️ 보상 공식 변경 → **Fresh Start 필수** (replay buffer 재사용 금지, Rule 4).
> action_vx/vy_scale은 `step()` L605-609에서만 적용 → **RL/TRACKING 단계 전용** (CRUISE는 mission_manager가 별도 제어). 안전.

### 향후(선택) — 코드 레벨 후속
config로 부족하면 `_compute_reward`에 **종단 hover-over-target 밀집 보너스**(d_xy→0에서 가파르게 증가)를 추가해
이진 성공 게이트 대신 연속 기울기를 부여. v13 결과 본 뒤 결정.

---

## 관련 링크

- [[research/reward_design]] — 4-layer 보상 함수 원설계
- [[research/rl_rules]] — Rule 10 (이 발견의 규칙화)
- [[experiments/exp_005_rl_yolo_v12_arm_fix_arming-throughput-fix]] — v12 run (정체 관측)
- [[experiments/training_history]] — v13 prepared row
- [[daily/daily_2026-06-16]]
