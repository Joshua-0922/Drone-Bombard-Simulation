---
date: 2026-07-03
tags: [error, isaac-lab, vision, frames, vectorization]
status: resolved
type: error
---

# Isaac Lab 비전 완전 사멸 — env-origin 좌표 프레임 혼용

## 증상

exp013 첫 2048-env PPO 학습에서 `Episode_Reward/rew_vision`이 **80+ iteration 내내
정확히 0.0000**. success는 오르는데(6.7%→17%) 비전 보상만 정확히 0 — 비전 채널
(obs[9:12] = u_norm/v_norm/conf)이 완전히 죽은 상태로 학습이 "잘" 진행되는 것처럼 보임
(privileged rel_dx/dy 채널만으로 학습).

## 원인

`DroneBombardEnv._update_vision()`이 프레임을 혼용:

```python
pos_w = self._robot.data.root_pos_w          # WORLD 프레임 (env origin 포함)
project_target_pinhole(pos_w, quat_w, self._target_xy, ...)   # target은 env-LOCAL (±10m)
```

`_target_xy`는 env 원점 기준 ±10m로 샘플링되는데, `root_pos_w`는 grid 원점들
(2048 envs × 16m 간격 → 좌표 최대 ±360m)이 포함된 world 좌표. 상대 벡터가 수백 m로
계산돼 타겟이 **항상 프레임 밖으로 투영** → `visible=False` → `conf=0` → obs[9:12]≡0,
`r3_vision≡0`.

같은 파일의 다른 소비자들은 전부 올바르게 처리하고 있었다 — `_current_d_xy`/`
_get_observations`는 `- self.scene.env_origins`, `_update_markers`는 target에 `+ env_origins`.
`_update_vision`만 누락.

**같은 버그가 `yolo_eval.py:run_calibrate`에도** (YOLO 픽셀 vs 기하 투영 비교 시
world pos + local target) — 동일 수정.

## 왜 못 잡았나 (검증 사각)

- `verify_one_episode.py` / `record_episode.py`는 **num_envs=1** → env origin ≈ (0,0,0)
  → local == world → 버그 은폐.
- 256-env dry-run(exp_012 §6d)은 reward가 오르고 d_xy_min 0.747까지 도달해 "정상"으로
  판정 — **rew_vision을 아무도 안 봤다.** privileged rel_dx/dy만으로도 태스크가 풀리기
  때문에 비전이 죽어도 표면 지표는 다 좋아 보인다.
- `pytest test_math.py`는 `project_target_pinhole` 자체(순수 함수)를 검증 — 함수는 옳고
  **호출부의 프레임 계약**이 틀렸다. 유닛테스트 사각.

## 수정

`drone_bombard_env.py::_update_vision`: `pos_local = root_pos_w - self.scene.env_origins`
를 투영에 전달. `yolo_eval.py::run_calibrate` 동일.

**수치 검증** (pure torch, 4096 스폰 샘플):
- 버그 재현(world frame + 46×46 grid): visible **0.00%**
- 수정(local frame): 스폰 시 visible **63.1%** (3-4m 100% → 6-7m 16.9%, 핀홀 기하와 정합),
  타겟 근처(0.5m 오프셋) **100%**

## 재발 방지 규칙

1. **vectorized env에서 위치를 쓰는 모든 신규 코드는 "이 값이 world인지 env-local인지"를
   주석으로 명시**하고, env-local 상대량(±수십 m)과 world 절대량을 한 식에 섞지 말 것.
2. **num_envs=1 검증은 origin-프레임 버그를 구조적으로 못 잡는다** — 스모크는 최소
   num_envs≥16으로, 그리고 `Episode_Reward/rew_vision`(또는 해당 채널의 활성 지표)이
   **0이 아닌지**를 dry-run 체크리스트에 포함.
3. **"정확히 0.0000"인 보상 성분은 dormant 설계(w_heading=0 → rew_orient)인지 즉시
   대조** — 설계상 0이 아닌 성분이 0이면 채널 사멸 신호. exp013에서 rew_orient=0은
   정상(w_heading=0), rew_vision=0은 버그였다.

## 관련

- [[experiments/exp_013_wcjklw7a_isaac_ppo_first_training]] — 이 버그를 발견한 학습 run
- [[research/isaac_lab_wandb_guide]] §3 — rew_vision 해석
- [[research/rl_rules]] Rule 16 — 이식 시 parity 검증 (이 버그는 그 연장: **벡터화 시 프레임 계약도 검증 대상**)
