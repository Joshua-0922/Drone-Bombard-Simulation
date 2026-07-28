---
date: 2026-07-28
tags: [moving-target, kalman-filter, singer-model, phase3, isaac-lab]
status: implemented
type: research
---

# 이동 타겟 모션 모델(CV/CA/CT) + Singer-KF 타겟 트래커

> 2026-07-28 구현. base env(`drone_bombard_env.py`) 전용 — v-track(v11~v19)은 미적용.
> 단위테스트 57/57 PASS + isaac-verify 스모크 4종(2-iter) PASS.

## 1. 타겟(X marker) 모션 모델 — `--target_motion {gm,cv,ca,ct}`

`DroneBombardPhaseCfg.target_motion_model`, 디스패처 `math_utils.step_target_motion` (policy dt=0.1s):

| 모델 | 동역학 | 에피소드 파라미터 |
|------|--------|------------------|
| `gm` (기본) | OU 속도워크 $v \leftarrow (1-\theta dt)v + \sigma\sqrt{dt}\,w$ (기존 Phase-3와 bit-identical) | `target_vel_theta=0.3`, `target_vel_sigma=0.5` |
| `cv` | 등속 직선 $x \leftarrow x + v\,dt$ | $\|v_0\| \sim U[0,\,$`target_init_speed`$]$ |
| `ca` | 등가속 $x \leftarrow x + v\,dt + \tfrac12 a\,dt^2$ | $\|a\| \sim U[0,\,$`target_accel_max`$=0.5]$ 랜덤 방향 |
| `ct` | 협조선회 — $\|v\|$ 보존, 정확한 원호 적분($\omega\to0$은 CV 극한 가드) | $\|\omega\| \sim U$`target_omega_range`$=(0.2,0.6)$ rad/s, 랜덤 부호 |

- `--moving_target`: phase와 무관하게 이동 타겟 강제 ON (`moving_target_force`, `release_terminal`과 같은 독립 노브 패턴). Phase-1/2 과제에 Phase-3 residual/DR/release 기계 없이 이동 타겟만 부여 가능.
- 속도/가속/선회율 오버라이드: `--target_speed --target_accel --target_omega_min --target_omega_max`.
- 샘플러(`mdp/domain_rand.py`): `sample_target_accel`/`sample_target_turn_rate` — 해당 모델일 때만 샘플(비활성 시 zero identity, 공유 RNG 스트림 보존).

## 2. Singer(Gauss-Markov 가속) Kalman 필터 트래커 — `--target_kf`

에이전트가 YOLO 검출로부터 타겟 운동을 **예측**할 수 있게 하는 관측 전용 필터:

- **상태** $[x,y,v_x,v_y,a_x,a_y]$, 가속도 = 1차 Gauss-Markov($\dot a = -a/\tau + w$) — 고전 Singer 기동표적 모델. $\tau\to\infty$에서 CA, $\tau\to0$에서 CV 극한 → 단일 필터로 CV/CA 추적 + CT 근사.
- **측정**: 해석적 YOLO 파이프라인의 노이즈 픽셀을 카메라 역투영(`pixel_to_ground_xy`, 평지 z=0 교차)한 지면 좌표. **fresh 검출만 업데이트**(hold-buffer 재방송은 정보 없음 — 이중계상 방지), dropout 중엔 예측 coast(공분산 증가).
- **측정 노이즈**: $\sigma = \max($`pixel_noise_std`$\cdot\text{slant}/f_x,\ 0.15\,\text{m})$ — 거리비례.
- **수치**: 이산 F/Q는 정확 폐형식(Q는 Bar-Shalom Singer 적분, F는 `expm1`로 CA 극한 정밀), 업데이트는 Joseph form + 2×2 폐형식 역행렬(N=2048 벡터화).
- **관측 확장(플래그 시)**: obs 14 → **21** = [KF 상대위치(2), KF 속도(2), KF 가속(2), validity(1)]. validity=0이면 전부 0 (no-track 명시 신호).
- 에피소드 리셋 시 트랙 완전 폐기(타겟 텔레포트 → 이전 트랙은 자신 있게 틀린 추정).
- **referee/보상 불변**: 트래커는 순수 관측 전용 — release trigger·lead aim은 기존 특권 정보 경로 그대로.

지표 신설: `Episode_Metric/kf_track_rate`, `kf_pos_err_m`, `kf_vel_err_mps`.

## 3. 스모크 결과 (isaac-verify, 16 envs × 2 iters)

| 구성 | 결과 |
|------|------|
| `--phase 1 --moving_target --target_motion cv --target_kf` | PASS — kf_track_rate 1.0, **kf_pos_err 0.093 m / kf_vel_err 0.077 m/s** |
| `--phase 1 --moving_target --target_motion ca` (obs 14) | PASS |
| `--phase 3 --target_motion ct --target_kf` | PASS — release_rate 1.0, lead_error 로깅, kf_pos_err 0.118 m(선회 타겟) |
| 기본 경로(플래그 없음, 회귀 확인) | PASS — obs 14, 기존 동작 불변 |

## 4. 주의 (체크포인트 호환)

- **`--target_kf`는 obs 14→21 → 기존 14-dim 체크포인트 warm-start 불가**(입력층 차원 상이). Fresh start 필수 (관측 변경 = Fresh Start 규칙과 동일 맥락, [[research/rl_rules]]).
- `--target_motion`/`--moving_target`만 쓰면 obs 불변 → warm-start 가능(단 과제 분포가 바뀌므로 보상 변화 관찰 필요).
- play.py에 동일 플래그 세트 미러링 — **학습과 동일 플래그로 평가**해야 함(21-dim 정책 ↔ 14-dim env 불일치 즉사).

## 관련
- [[experiments/training_history]] / [[research/rl_rules]] / [[research/phased_curriculum]]
- 코드: `math_utils.py`(step_target_ca/ct/motion, pixel_to_ground_xy, singer_*, kf_*), `mdp/domain_rand.py`, `drone_bombard_env.py`(`DroneBombardTrackerCfg`, `_update_target_tracker`), `train.py`/`play.py` CLI
