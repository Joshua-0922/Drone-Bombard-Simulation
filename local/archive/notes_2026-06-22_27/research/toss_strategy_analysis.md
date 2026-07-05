---
date: 2026-06-22
tags: [research, v8, toss, strategy, reward_design, fine_tune, payload_distance]
status: ongoing
type: research
---

# v8 의 toss 전략 분석 + v9 처방 설계

## v8 정책의 학습된 행동 (사용자 GUI 관찰)

```
1. drone 이 (0, 0, 0.24) spawn
2. ARM + TAKEOFF → cruise alt 5m
3. 정책이 cruise 명령 (NW) 무시 + target (4, 3) NE 방향 비행
4. drone 이 marker (4, 3) 를 지나친 후 (예: (5, 4))
5. 그 위치에서 멈춤 + 몸을 marker 방향으로 기울임 (pitch back)
6. detach → payload 가 forward momentum 으로 marker 향해 toss
7. drop_calculator 가 payload 의 ground contact 측정 → err 1-2m
```

## 핵심 — toss 가 모든 ep 마다 일관 = 정책 완전 수렴

```
v8 학습 결과:
- 80.6% success (drop_err ≤ 2m)
- 13 jackpot (≤ 0.3m)
- mean err 1.85m
- 8,736 episodes, 3,342 drops
```

## 사용자 의도 ≠ 실제 학습 행동

| 의도 | 실제 |
|---|---|
| drone 이 marker 위 hover 후 free-fall drop | toss (marker 지나친 후 throw) |

→ **고무적이지만 막연하게 생각하는 방향은 아님**.

## 왜 toss 학습됐나 — 환경 분석

### 잘못된 가설 (1차 분석 시 잘못 알린 것)

> `hover_drop_block_threshold = 0.0` 이 hover-drop 차단 → toss 강제

**틀림**. yaml comment: `# 0 = 비활성 (모든 drop 허용)`. hover-drop 허용됨.

### 진짜 원인 — 환경의 약점

| # | 환경 요소 | toss 유리하게 만든 부분 |
|---|---|---|
| 1 | **`auto_drop_threshold = 2.0`** | 정책이 explicit drop 결정 안 함. ballistic CCIP < 2m 이면 자동 drop |
| 2 | **sparse reward** (terminal `drop_err` 만) | 어디서 drop 할지 명시적 안내 없음. 정책이 dynamics 안에서 최적 발견 → toss |
| 3 | **target NE vs cruise NW** | 정책이 cruise 명령 override → forward vel 활용 |
| 4 | **detach 후 ep 즉시 종료** | payload trajectory 직접 평가 안 됨. ballistic 의 randomness 가 dynamics 의 자연 해법 |

## 사용자 두 가설 (정확)

### 가설 1 — 탐험 부족 (local optimum)

- v8 = 8,736 ep, 3,644 drop. SAC 의 entropy reg 으로 충분히 탐색.
- **그러나** toss 가 "초기 우연 발견 + 정확도 좋음" → 그 경험 위주 학습 강화 → local optimum.

**부분 정확**. local optimum 의 강도.

### 가설 2 — 환경이 두 행동 구분 못 함 (root cause)

| 항목 | toss | hover-drop |
|---|---|---|
| auto_drop_threshold 판정 | 같음 | 같음 |
| terminal reward (drop_err) | 같음 | 같음 |
| time penalty | 비슷 | 약간 더 길 수도 |
| **학습 입장에서 차이** | **거의 없음** | **거의 없음** |

→ **환경 자체가 두 행동 선호 안 함**. 학습이 발견하기 쉬운 + 정확도 좋은 toss 선택.

**가장 본질적 원인**. 가설 2 가 root cause, 가설 1 은 그 결과.

## v9 처방 — 사용자 결정

### 처방 1 (drop_angaccel penalty)

```yaml
drop_angaccel_penalty_scale: 0.5
drop_angaccel_window_n: 5
```

- drop 시점 직전 5 step 의 max ang_accel penalty
- 측정 (E 방법): 인접 step ang_vel diff 의 max
- prerequisite: [[errors/err_20260622_ang_vel_callback]]

### 처방 2 (payload distance reward — w_dist 강화)

```yaml
w_dist: 1.0 → 1.5
```

- 분석 결과: 사용자 의도가 기존 r3_dist 와 동일
- A 옵션: w_dist scale 증가 (간단, 효과 명확)
- B 옵션 (폐기): detach 후 payload tracking — 큰 변경 필요

## payload tracking 의 현실

### 현재 시스템

| 시점 | 측정 | 학습 신호 |
|---|---|---|
| 매 step | **예측** (ballistic CCIP, drone vel+alt 로 계산) | `w_impact * exp(-k * d_impact)` |
| Drop 시점 | **실제** (drop_calculator 가 Gazebo payload contact) | `w_drop_base * exp(-k * d_error)` + jackpot |
| Drop 직후 | (현재) ep 즉시 종료 | — |

### 진정한 payload tracking 옵션

**A. Gazebo payload pose 직접 받기**:
```
gz_ros2_bridge 에 /world/x_marker_world/model/payload_0/pose 추가
env 에 subscriber 추가
매 step payload position 측정
```

**C. detach 후 ep 연장**:
- drop trigger 후 즉시 종료 X → payload landing 까지 (~2-3초) 계속
- 매 step payload distance reward

**A + C** = 사용자 의도 100%.

`w_prediction` (현재 0): drop 시점에 CCIP 정확도 reward. 활성화 가능 (1줄). 작은 변화.

## v8 vs v9a 결과 (실측)

자세한 분석: [[experiments/exp_006_zjexq20k_v9a_payload_dist_angaccel]]

| 항목 | v8 | v9a (preempt step 313k) | 변화 |
|---|---|---|---|
| success ≤2m | 80% | 80% | 동일 |
| mean err | 1.852m | 1.888m | 분포 안 (+0.04m) |
| **max ang_vel** | **2.5 rad/s** | **2.10 rad/s** | **-16% ↓** |
| 정책 행동 | toss | **toss 유지** | 본질 동일 |

→ **drop_angaccel penalty 효과 명확**, w_dist 1.5 효과 미미, **사용자 의도 (지나치는 현상 해결) 안 됨**.

## 17k step fine-tune 의 한계

- 사용자 의도 100k 의 17% 만 진행
- v8 의 toss prior 가 매우 강함
- 처방의 누적 효과가 작아서 본질 행동 변화 못 만듦

## 다음 단계 — 사용자가 제안한 두번째 처방 후보

### 옵션 1 — target 거리 제한 (가장 안전) ★★★

```python
# drone 이 target 가까이 가면 penalty
if d_xy < target_distance_min:
    reward -= scale * (target_distance_min - d_xy)
```

- 효과: drone 이 target 위 까지 안 가게 강제 → early shot 학습 강제
- 명확. 안전.

### 옵션 2 — 시간 패널티 강화 (위험) ★

```yaml
w_time: 0.01 → 0.05
```

- 효과: 짧은 ep 유도 → early shot
- **사용자 자각**: "기존 학습이 정말 잘 된 경우에만 사용가능. 그 외에는 발산 가능"
- v9 단계 적합 안 함

### 옵션 3 — drop 좌표 강제 ★★

```python
# drop 시점에 drone 위치가 사용자 정의 좌표 (e.g. (2, 1.5)) 에서 거리 측정 + penalty
if drop_triggered:
    reward -= scale * ||drone_xy - desired_drop_xy||
```

- 효과: 정책이 그 좌표에서 drop 학습 + 그 좌표에 맞춘 속도 조절
- 좌표 결정이 어려움 — 미리 정해야

## 다른 가능 처방 (v9b, v9c 등)

| # | 처방 | 효과 |
|---|---|---|
| D | spawn / target randomization | 정책 generalize |
| E | cruise 명령 비활성 | 정책 free flight |
| F | `w_prediction` 활성 (CCIP 정확도) | ballistic 학습 강화 |
| G | `auto_drop_threshold` 좁힘 (2.0 → 0.5) | 더 정확한 drop 위치 학습 |
| H | auto_drop 비활성 + 정책 explicit drop | 정책 self-deciding (어려움) |
| I | Gazebo payload pose 받기 + ep 연장 (A+C) | 실제 payload trajectory 학습 |
| J | Fresh start with strong shaping | v8 prior 제거 |

## 결론 및 권장

1. **v9a 의 본 의도** — drop 시점 부드러운 dynamics + payload trajectory 강화 — **부분 달성**
2. **사용자의 막연한 의도** (지나치는 현상 해결) — **달성 안 됨**. fine-tune 으로는 부족
3. **다음 단계**: 두번째 처방 (1, 2, 3) 또는 환경 변경 (D, E) 또는 payload tracking (I)

## 관련 노트

- [[experiments/exp_006_zjexq20k_v9a_payload_dist_angaccel]] — v9a 실험 결과
- [[errors/err_20260622_ang_vel_callback]] — drop_angaccel 의 prerequisite
- [[research/dgui_tool]] — 평가 도구
- [[research/reward_design]] — 4-layer 보상 (기존)
- [[research/rl_rules]] — fine-tune 의 step 영향 rule
