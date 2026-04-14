---
date: 2026-03-20
tags: [error, gazebo, physics, ODE, replay-buffer]
status: resolved
type: error
---

# Err — Gazebo 물리 폭발 (d_xy = 1.98×10¹¹ m)

---

## 증상

- `env/mean_d_xy` WandB 그래프가 $\sim 10^{8}$ 으로 급등
- Run `mjfet61f` 에서 발생 (52K steps)
- Replay buffer 오염 → 체크포인트 폐기 필요

---

## 원인

Gazebo ODE 물리 엔진이 좌표 폭발 생성.
단일 스텝 $d_{xy} = 1.98 \times 10^{11}\,\text{m}$ → 2048 스텝 롤아웃 평균 $\approx 10^{8}$.

---

## 해결책 — 3중 방어 레이어

### Layer 1: 소스 차단 (`_on_local_pos`)

```python
if abs(pos).max() > 1000.0:   # 1000 m 초과 좌표 거부
    return                     # 마지막 정상 pos_enu 유지
```

### Layer 2: 스텝 가드 (`step()`)

```python
if not math.isfinite(d_xy) or d_xy > 500.0:
    reward = -100.0
    info['physics_glitch'] = True
    info['glitch_d_xy'] = d_xy   # 별도 키 사용!
    return obs, reward, True, False, info
```

> **핵심:** `glitch_d_xy` 키를 사용해 WandB 콜백이 평균에 포함시키지 않도록 함.

### Layer 3: WandB 콜백 (`WandbMetricsCallback`)

```python
# 글리치 스텝은 'd_xy' 키 없음 → 누산 안 됨
if 'd_xy' in info:
    d_xy_accum.append(info['d_xy'])
if 'glitch_d_xy' in info:
    glitch_count += 1
```

---

## 체크포인트 관리 규칙

폭발 직후 SIGTERM → `_emergency_save()` 가 오염된 상태로 `sac_drop_preempt.zip` 덮어씀.

**절대 preempt 체크포인트에서 재개 금지.** 대신 최신 rolling checkpoint 사용:

```bash
# 오염된 버퍼 격리 (삭제 금지 — 감사 추적)
mv sac_drop_preempt_replay.pkl sac_drop_preempt_replay.pkl.CORRUPTED_20260320

# 롤링 체크포인트에서 재개 (replay buffer 없음 → SB3가 자동으로 새 버퍼 시작)
--resume sac_drop_95000_steps.zip
```

---

## 관련 실험

- [[experiments/exp_001_8otphxy8_linear_reward]] — 3중 방어 적용 후 안정화
