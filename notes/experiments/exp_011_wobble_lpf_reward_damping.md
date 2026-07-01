---
date: 2026-07-01
tags: [experiment, wobble, smoothness, lpf, reward, SAC, v15]
status: active
type: experiments
wandb_run: v15_bc_stable
---

# exp_011 — 10m 핸드오프 후 RL wobble: LPF A/B 진단 + 보상 댐핑(B+C)

> 사용자 관찰: 10 m 고도로 올려 YOLO 조기 탐지 → RL 제어 윈도우 확보됨. 그러나 **RL 인수 후 드론이 wobble**. 순항(POSITION 제어)처럼 안정 이동 원함. "보상이냐 로직이냐?"

관련: [[research/control_smoothness_wobble]] · [[experiments/exp_010_byxyaf4d_v14_195k_eval]] · [[research/rl_rules]] Rule 15

---

## 1. 진단 (A/B empirical check)

**핵심 사실:** eval은 이미 `deterministic=True`(`evaluate.py:75`) → wobble은 SAC 탐험 노이즈가 **아님**. 학습된 oscillatory(bang-bang) 정책 자체.

**A/B 방법:** v14 195K(`sac_drop_195000_steps.zip`) 고정, 컨트롤러 출력에 EMA 저역통과 필터 추가(`velocity_lpf_alpha`). PX4가 받는 `/fmu/in/trajectory_setpoint.velocity`를 로깅(`vel_logger.py`), 연속 샘플 차분 RMS(=command jerk)로 wobble 정량화.

| 지표 | Baseline (α=1.0, raw) | Filtered (α=0.4) | 비율 |
|------|----------------------|------------------|------|
| **command jerk RMS** | **2.92 m/s / sample** | 1.61 | **0.55 (−45%)** |
| 방향 반전 / sample | 0.204 | 0.158 | 0.78 |
| 평균 속력 | 3.46 m/s | 3.90 | 1.13 (안 느려짐) |

→ **raw 명령이 50 ms마다 ~2.9 m/s 진폭으로 진동**(축별 std ~2.8 m/s) = wobble의 정체. LPF가 45% 저감, 평균 속력은 오히려 소폭 증가 → **smoothness-control 문제 확정.** (착지 정확도는 불변: 두 조건 모두 0/2, min d_xy ~1.3–1.5 m — 필터는 정책 정확도를 안 건드림, 예상대로.)

**답(사용자 질문):** 보상 **and** 로직 둘 다. 하지만 근본은 정책 인센티브. LPF는 증상 완화, 보상 shaping이 원인 교정.

## 2. 적용 변경 (B + C) — 사용자 "너무 느리게 말고" 제약 반영

- **(B) 근접-게이팅 속도 댐핑** (`drone_drop_env._compute_reward`): `r3_vel = -w_vel * speed_xy * max(0, 1 - d_xy/vel_damp_radius)`. **멀리서는 0(순항 자유), 마커 근처서만 속도 비용** → 궤도 선회 대신 감속 착지. `w_vel=0.15`, `vel_damp_radius=4.0`.
- **(C) smoothness 가중 상향**: `w_ang_vel 0.05→0.15`, `w_action_smooth 0.05→0.20`. body-rate·per-step jerk 페널티(전진 속도 아님) → wobble만 억제.
- **(로직) LPF 유지**: `velocity_lpf_alpha: 0.4` — 학습==배포 플랜트(정책이 필터된 컨트롤러 상대로 학습). 컨트롤러 subprocess(`drone_drop_env.py:1509`)에 `-p velocity_lpf_alpha` 주입.
- 액션 스케일(vx=8/vy=5)은 **이번엔 미변경**(사용자 속도 유지 요청; 필요 시 fallback 레버).

## 3. 검증

- **B+C dry-run (1200 steps, temp ckpt, wandb offline):** `Training complete`, 크래시 0(Traceback/px4_msgs 0), ep_rew_mean 15.3→3.46(댐핑으로 하락은 예상), cos_heading 등 populate. 보상 코드 healthy.
- **Fresh full training 기동:** `run_train_bc.sh` → tmux `rl_train`, `v15_bc_stable`, 300K, wandb online. Fresh Start가 v14 5개 체크포인트 삭제 → **`rl_checkpoints/v14_backup/`에 백업**.

## 4. 남은 확인

- v15 성숙 후 eval + `vel_logger`로 wobble 재측정: jerk RMS가 raw 대비 크게↓ + success ≥ v13 80%?
- rew_vel이 근접서만 활성인지(먼 구간 0) 학습 로그로 확인.
- 부족 시 fallback: 액션 스케일 vx/vy ↓, w_vel/smoothness 추가 튜닝.

## 5. ⚠️ Postmortem — 잘못된 base config로 첫 v15 크래시 (2026-07-01)

**첫 v15 run(`41qq0tpd`)은 10.8K/300K(3.6%)에서 크래시.** `RuntimeError: reset() called recursively >5 → infra unrecoverable`. 원인 = **CRUISE-timeout→IDLE-stuck 캐스케이드**(드론이 IDLE서 arm/climb 못 함, 3회 full-restart 실패 → reset-recursion guard abort). RL 핸드오프 전 죽어서 **보상 변경과 무관**.

**근본 원인 = 잘못된 base config(실행 오류):** `run_train_bc.sh`가 `--config` 없이 실행 → 기본 `hyperparams.yaml` 사용. 그러나 v14/10m 인프라·종말 수정은 전부 **`hyperparams_v13.yaml`**에 있음. 증거:
- **soft reset 0회 발동**(v14는 ~91%) ← `soft_reset_enabled` 없음 → 느린 full-restart-only → 48 CRUISE timeouts / 60 restarts.
- action_vx/vy **8/5**(v13은 4/3), success_radius **0.5**(v13 0.8), overshoot **1.5**(v13 0.6) → v13 종말-overshoot 수정도 되돌아감.

**수정:** B+C+LPF를 **`hyperparams_v13.yaml`에 이식** + 런처 `--config hyperparams_v13.yaml`. **`w_vel` 0.15→0.08로 완화**(⚠️ v14 실패=final-approach stagnation인데 근접 속도 페널티가 이를 악화 가능 → smoothness 가중(C)을 주 레버로, B는 gentle). v13 base는 이미 action 4/3라 wobble이 원래 더 약함.

**재검증(corrected dry-run):** Config=hyperparams_v13.yaml 확인, **soft reset 7/7 성공(~11–14s, no teleport)**, reset-recursion 0, `Training complete`. → **full retrain 재기동** `v15_bc_stable`(run `53v6ehpx`, hyperparams_v13.yaml, 300K).

**교훈:** train_sac는 `--config` 없으면 기본 `hyperparams.yaml`로 폴백. v14 이후 작업은 반드시 `--config hyperparams_v13.yaml`. 새 보상 변경은 그 파일에 이식할 것. (A/B eval도 hyperparams.yaml(scale 8)로 돌아 raw jerk 절대값이 ~2× 부풀려졌을 수 있음 — LPF −45% 비율은 유효.)

## 코드/도구
- `drone_controller_node.py`(EMA `_filter_velocity` + `velocity_lpf_alpha` param), `drone_drop_env.py`(param 주입 + `r3_vel`), `hyperparams.yaml`(w_vel/vel_damp_radius/w_ang_vel/w_action_smooth/velocity_lpf_alpha), `episode.launch.py`(launch arg).
- 신규 도구(미커밋): `vel_logger.py`, `run_abtest.sh`, `run_dryrun_bc.sh`, `run_train_bc.sh`.
