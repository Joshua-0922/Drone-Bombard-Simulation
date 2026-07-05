---
date: 2026-06-26
tags: [experiment, v9a, fine_tune, payload_distance, drop_angaccel, warm_start]
status: stopped_early
type: experiment
wandb_run: zjexq20k
---

# Exp 006 — v9a payload_dist + drop_angaccel (v8 warm start fine-tune)

## Wandb

- **run_id**: `zjexq20k`
- **run_name**: `phase1_redux_v9a_payload_dist_angaccel`
- **url**: https://wandb.ai/nayoonho0922-seoul-national-university/drone-bombard-sac/runs/zjexq20k

## Motivation

v8 (`phase1_redux_v8_no_invalid_penalty`) 의 정책이 **toss 전략 학습** — drone 이 marker 지나친 후 pitch back + drop. 사용자가 직접 시각 검증 후:
- "drone 이 marker 를 지나치는 건 사실 좋지 않다"
- "drop 시 drone 의 각속도 변화에 제한이 있으면 될 것 같다"
- "payload 가 처음부터 끝까지 marker 향해 가까워지면 +reward, 멀어지면 -penalty"

→ 두 처방 도입.

## 처방 — 사용자 결정

### 처방 1 (drop_angaccel penalty)

**Drop 시점 직전 N step 의 max angular acceleration penalty**.

- 의도: toss 의 급격한 pitch back 차단, 정책이 부드러운 drop 학습
- 측정 (E 방법): drop trigger 시 직전 5 step 의 인접 ang_vel diff magnitude 의 max
- 단위: rad/s (per step, dt 무시 simple diff)
- **prerequisite**: ang_vel callback fix → [[errors/err_20260622_ang_vel_callback]]

코드 (`drone_drop_env.py` step() 의 drop trigger 부분):
```python
hist = list(self._ang_vel_history)   # deque(maxlen=6)
max_ang_accel = 0
for i in range(1, len(hist)):
    accel = ||hist[i] - hist[i-1]||
    if accel > max_ang_accel: max_ang_accel = accel
reward -= drop_angaccel_penalty_scale * max_ang_accel
```

### 처방 2 (payload distance reward — 사용자 의도, 구현은 w_dist 강화)

사용자 의도: **payload distance reward** — 가까워지면 +, 멀어지면 -penalty.

분석 결과: 기존 `r3_dist = w_dist * (d_xy_prev - d_xy)` 와 본질적으로 동일.
- attached 동안 drone == payload 위치 (offset 0.14m 작음)
- detach 후 ep 즉시 종료 (terminal reward 만)

선택: **A. w_dist scale 증가** (1.0 → 1.5, +50%). 구현 간단, 효과 명확.

자세한 분석: [[research/toss_strategy_analysis]]

## Hyperparams 변경

| 항목 | v8 | v9a |
|---|---|---|
| `w_dist` | 1.0 | **1.5** |
| `drop_angaccel_penalty_scale` | (없음) | **0.5** |
| `drop_angaccel_window_n` | (없음) | **5** |
| `limit_ang_vel` | 2.0 | 10.0 (ang_vel fix 후) |
| `total_timesteps` | 300,000 | 100,000 (override 후 400k) |
| `run_name` | `phase1_redux_v8_no_invalid_penalty` | `phase1_redux_v9a_payload_dist_angaccel` |

## 학습 명령

```bash
ros2 run rl_navigation train_sac \\
    --resume /workspace/ros2_ws/eval_models/v8_peak_step217040_err0.87m.zip \\
    --timesteps 400000
```

- **warm start**: v8 weights load (model.num_timesteps = 217,040)
- **fresh replay buffer**: `_replay.pkl` 없으므로 자동 새 buffer (CLAUDE.md "보상 공식 변경 → Fresh Start" 준수)
- **reset_num_timesteps=False**: counter 유지, total 400k 까지 = **+183k step**
- 의도는 100k 추가 (= step 317k 까지) — 사용자가 SIGTERM 으로 stop

## env 코드 변경

`drone_drop_env.py`:
- `from collections import deque` (line 16 부근)
- `__init__`:
  - `self._cfg_drop_angaccel_penalty_scale = r.get('drop_angaccel_penalty_scale', 0.0)`
  - `self._cfg_drop_angaccel_window_n = int(r.get('drop_angaccel_window_n', 5))`
  - `self._ang_vel_history = deque(maxlen=self._cfg_drop_angaccel_window_n + 1)`
- `reset()`: `self._ang_vel_history.clear()`
- `step()`: 매 step `self._ang_vel_history.append(ang.copy())`
- drop trigger 후 instability penalty 다음: max ang_accel 계산 + penalty

## 학습 진행

| 시점 | total_step | success_rate | ep_rew_mean | ep_len_mean | actor_loss |
|---|---|---|---|---|---|
| 시작 (18:41 UTC) | 217,040 | (v8 baseline 0.80) | — | — | — |
| 3시간 | 234,301 | 0.51 ⚠️ | -50 | 47 | 60-80 |
| 4시간 | 296,706 | 0.55 | -43 | 40 | 29-33 |
| **4.8시간** | 303,127 | 0.54 | -37 | **6.27** ⚠️ | 22-35 |
| 5시간 (SIGTERM) | **313k** | **0.58** | **-15.5** | **96.3** ✓ | 35.1 |

**관찰**:
- 4.8시간 시점에 ep_len_mean 6.27 — 정책 발산 의심
- 5시간에 96.3 로 회복 — 가능: 정책이 새 reward 적응 마무리
- success_rate 0.58 < v8 의 0.80 — 학습 중 stochastic 영향
- **17k step 만 추가 학습** (의도 100k 의 17%)

## 평가 (preempt step 313k)

`dgui v9a_preempt_step313k --episodes 5 --no-gui`:

```
EP1: 1.79m ✓, steps=42, max_ang_vel=2.47
EP2: 1.81m ✓, steps=46, max_ang_vel=1.87
EP3: 1.65m ✓, steps=41, max_ang_vel=2.39
EP4: 1.98m ✓, steps=61, max_ang_vel=1.96
EP5: 2.22m (margin), steps=47, max_ang_vel=1.83

success ≤2m:   4/5 = 80%
mean err:      1.888m
max ang_vel:   2.10 rad/s (평균)
```

## v8 vs v9a 비교

| 항목 | v8 (5 ep) | v9a (5 ep) | 변화 |
|---|---|---|---|
| success rate | 80% | **80%** | 동일 |
| mean err | 1.852m | **1.888m** | +0.036m (분포 안) |
| max ang_vel | 2.5 rad/s | **2.10 rad/s** | **-16% ↓** |
| 정책 행동 | toss (marker 지나친 후) | **toss 유지** | 본질 동일 |

## 결론

| 처방 | 효과 |
|---|---|
| **drop_angaccel penalty 0.5** | ✓ **명확** — max ang_vel 16% 감소. 정책이 부드러운 drop 학습 |
| **w_dist 1.5 (1.0→1.5)** | ✗ **미미** — mean err 거의 동일 (1.85 vs 1.89, variance 안) |
| **사용자 의도 (지나치는 현상 해결)** | ✗ **달성 안 됨** — toss 전략 그대로 유지 |

## 원인 분석

1. **17k step fine-tune 의 한계**: v8 의 toss 가 강한 local optimum. 100k 까지 진행했어야 더 명확한 결과
2. **w_dist 1.5 의 영향 작음**: 처방 2 의 누적 효과가 50 step 에 +1.4 (terminal reward +30 의 5%)
3. **drop_angaccel penalty 의 효과**: dynamics 만 제한, toss 전략 본질 안 바꿈

## 다음 단계 후보

자세한 분석: [[research/toss_strategy_analysis]] 의 "다음 처방 후보"

| # | 방향 |
|---|---|
| A | 100k 까지 더 학습 (현재 17k → +83k, ~12시간 더) |
| B | 두번째 처방 도입 (target 거리 제한 등) |
| C | 환경 변경 (cruise 비활성, target randomize) |
| D | payload tracking 완전 구현 (Gazebo pose + ep 연장) |
| E | Fresh start with strong shaping |

## Backup

| 자산 | 위치 |
|---|---|
| v9a preempt model | `eval_models/v9a_preempt_step313k.zip` (3.2 MB) |
| v9a replay buffer | `rl_checkpoints/sac_drop_preempt_replay.pkl` (86 MB) |
| v9a drop_episodes | `rl_checkpoints/drop_episodes/` (4,527 drops 누적) |
| pre-v9a hyperparams | `local/backups/hyperparams_v8_pre_angvel_fix_20260622_042523.yaml` |
| 평가 결과 json | `local/eval_logs/eval_2026-06-27T00-36-50_v9a_preempt_step313k.json` |

## 관련 노트

- [[errors/err_20260622_ang_vel_callback]] — ang_vel callback fix (prerequisite)
- [[research/toss_strategy_analysis]] — v8 의 toss 전략 + v9 처방 분석
- [[research/dgui_tool]] — 평가 도구 사용
- [[experiments/training_history]] — 전체 학습 history
- [[research/rl_rules]] — Fine-tune step 수 영향
