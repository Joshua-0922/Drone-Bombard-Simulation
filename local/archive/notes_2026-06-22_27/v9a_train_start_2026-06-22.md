# v9a 학습 시작 — phase1_redux_v9a_payload_dist_angaccel

**시작**: 2026-06-22 18:41 (container 시간 = UTC)
**wandb run id**: `zjexq20k`
**wandb url**: https://wandb.ai/nayoonho0922-seoul-national-university/drone-bombard-sac/runs/zjexq20k

## 처방 — 사용자 결정

### 처방 1 (사용자 제안)
**Drop 시점 직전 N step max angular acceleration penalty**
- 의도: toss 의 급격한 pitch back 차단, 정책이 부드러운 drop 학습
- 측정: drop trigger 시 직전 5 step 의 ang_vel diff magnitude 의 max
- 단위: rad/s (per step, dt 무시 simple diff)
- 측정 가능 조건: `ang_vel callback fix (2026-06-22)` — PX4 dds_topics 의 vehicle_angular_velocity uncomment
- 코드:
```python
hist = list(self._ang_vel_history)
for i in range(1, len(hist)):
    accel = ||hist[i] - hist[i-1]||
    if accel > max_ang_accel:
        max_ang_accel = accel
reward -= drop_angaccel_penalty_scale * max_ang_accel
```

### 처방 2 (사용자 제안)
**Payload distance reward** — 기존 r3_dist 강화로 대체
- 분석: 사용자 의도가 기존 `w_dist * (d_xy_prev - d_xy)` 와 거의 동일 (attached 동안 drone == payload)
- 선택: A. w_dist scale 증가 (1.0 → 1.5, +50%)
- detach 후 payload 추적은 ep 즉시 종료라 무관

## hyperparams 변경

| 항목 | 이전 | 새로 |
|---|---|---|
| w_dist | 1.0 | **1.5** |
| drop_angaccel_penalty_scale | (없음) | **0.5** |
| drop_angaccel_window_n | (없음) | **5** |
| total_timesteps | 300000 | 100000 (override 로 +400k) |
| run_name | phase1_redux_v8_no_invalid_penalty | phase1_redux_v9a_payload_dist_angaccel |

## env 코드 변경

`drone_drop_env.py`:
- import deque (line 16)
- `__init__` 에 `_cfg_drop_angaccel_penalty_scale`, `_cfg_drop_angaccel_window_n`, `_ang_vel_history` (line ~413)
- `reset()` 에 `_ang_vel_history.clear()` (line ~683)
- `step()` 매 step 에 `_ang_vel_history.append(ang.copy())` (line ~760)
- drop trigger 후 instability penalty 다음에 ang_accel penalty 계산 + reward 차감 (line ~857)

## 학습 명령

```bash
ros2 run rl_navigation train_sac \
    --resume /workspace/ros2_ws/eval_models/v8_peak_step217040_err0.87m.zip \
    --timesteps 400000      # v8 의 303k + 100k 추가
```

- replay buffer: zip 옆 `_replay.pkl` 없음 → 자동 fresh (CLAUDE.md "보상 공식 변경 → Fresh Start" 준수)
- v8 weights warm start (actor/critic)
- reset_num_timesteps = False (v8 의 num_timesteps 303k 유지, total 400k 까지 학습)

## 예상 결과 (사용자 가설)

| 가설 | 가능성 |
|---|---|
| A. Hover-drop (사용자 예상) | 40-50% |
| B. Early shot (조준) | 30% |
| C. 부드러운 toss | 15% |
| D. v8 toss 유지 (local optimum) | 10% |
| E. 예상치 못한 dynamic | 5% |

## 예상 시간

- v8 fps ≈ 1-2 step/s
- 100k step → **14-28시간** (대략 18시간)
- ~2026-06-23 12-15시 완료 예상

## 다음 단계

- 학습 종료 후:
  1. v9a peak window + best 모델 백업
  2. dgui 평가 (v9a 모델, GUI 로 새 전략 시각 확인)
  3. 결과에 따라 두번째 처방 결정 (사용자 의도: 1, 2, 3 중 안전 단계)

## Backup (rollback 시)

- 이전 hyperparams: `local/backups/hyperparams_v8_pre_angvel_fix_20260622_042523.yaml` (ang_vel fix 전)
- v8 모델 (학습 자산): `local/backups/phase1_redux_v8_2026-06-21/` (8.6 GB)
- 새 v9a checkpoint: `ros2_ws/rl_checkpoints/` (학습 진행 중 누적)

---
*v9a 학습 시작: 2026-06-22 18:41 (UTC), 학습 진행 중 ~18 시간*
