---
date: 2026-07-30
tags: [experiment, isaac-lab, v19, moving-target, warm-start, cv, ca, ct]
status: done
type: experiment
wandb_run: "a6saa42b / 29jqq1lu / ntumqwoz"
---

# exp_021 — v19 + 이동 타겟(CV/CT/CA) warm-start 학습

> 목표: v-track 미적용이던 이동 타겟 모션 모델(CV/CT/CA — [[research/moving_target_models]])을
> **v19 env에 포팅**(obs 28-D 불변)하고, 준상 v19 warm-start 체크포인트에서 각 모션별로
> 이어 학습해 이동 타겟 대응 성능을 비교.

## 1. 셋업

- **포팅**: `V11Env._step_moving_target()` 신설 — base env `_advance_phase_dynamics`와 동일
  통합 지점(`_get_dones` 최상단)에서 `step_target_motion` 호출. V11/V16/V19 세 `_get_dones`
  모두 wire. 초기 속도/가속/선회율은 base `_reset_idx` 샘플러 그대로(obs 불변 → lossless warm-start).
- `train.py`: 이동 타겟 플래그를 base 전용 분기에서 **전 env 공통**으로 승격.
  `--target_kf`+v-track 조합은 명시적 에러(v-track obs builder에 KF 채널 없음 — silent no-op 방지).
- `play.py`: `parse_env_cfg` 기반이라 변경 불필요(같은 플래그로 평가 가능).
- **warm-start**: `checkpoints/v19/precise/model_best.pt`의 **사본**
  (`/opt/drone-bombard/checkpoints/v19_junsang_copy/` + 컨테이너 `/workspace/v19_warmstart/`,
  md5 검증) — 원본 무접촉. release 100% / drop_err ~0.39 m 시드.
- **학습**: `--v19 --moving_target --target_motion {cv,ct,ca}`, 2048 envs × 1000 iters,
  seed 42, wandb project `drone-bombard-isaac`, run `exp021_mt_{cv,ct,ca}`.
- 모션 파라미터(기본값): |v0|~U[0,2] m/s, CA |a|~U[0,0.5] m/s², CT |ω|~U[0.2,0.6] rad/s.

## 2. 사전 검증 (dry-run 규칙)

- 단위테스트 57/57 PASS.
- `_probe_moving_v19.py` (8 envs × 30 steps, zero action): 3모션 모두 PASS —
  CV |v| bit-불변, CT |v| 보존(~1e-6, 원호 적분), CA |v| 증가. obs 28-D, NaN 없음.
- 2-iter warm-start 스모크(cv, 16 envs): 로드 무손실(차원 불일치 없음), model_final 저장 확인.

## 3. 결과 (학습 커브 — 종반 5-iter 창, 학습 내 지표라 σ-지터 큼)

| motion | wandb run | release_rate | drop err @release (m) | mean reward | 소요 |
|---|---|---|---|---|---|
| cv | `a6saa42b` | 0.67–0.90 | 0.39–0.90 | 25–52 | 67 min |
| ct | `29jqq1lu` | 0.43–0.80 | 0.24–1.24 | 6–29 | 69 min |
| ca | `ntumqwoz` | 0.40–0.78 | 0.32–1.96 | −19–+4 | 69 min |

- 3 run 전부 1000 iters 완주(rc=0, 에러 0), ~4.05 s/iter(2048 envs; exp_020 3.1 s 대비
  이동타겟 오버헤드 소폭).
- **warm-start 이월 확인**: 정지-타겟 시드에서 첫 롤아웃부터 접근/조준 동작 이월,
  release가 초반부터 발생(cv 중반 계측 release 0.77 / drop err @release 0.187 m).
- 난이도 서열(학습 지표 기준): **cv < ct < ca** — cv는 보상 우상향·release 최고,
  ct는 선회 리드 필요로 분산 큼, ca는 종반까지 mean reward 음수 구간 잔존(가속
  타겟의 리드 예측이 가장 어려움). 정지-타겟 v19(release 100%/0.39 m) 대비 전반적
  성능 하락은 과제 자체가 어려워진 것.
- ⚠️ **판정은 deterministic eval로** (학습 내 지표는 episode 표본 13–14개/iter 수준의
  σ-지터에 가려짐 — exp_017 교훈) → §3b.

## 3b. Deterministic 200-ep eval (판정)

`play.py --task Isaac-DroneBombard-V19-Direct-v0 --moving_target --target_motion {m}
--policy /workspace/exp021/exp021_mt_{m}_final.pt --episodes 200 --num_envs 64 --wandb`

| motion | success | release_rate | drop err @release (med/p90/max) | d_xy_min med | 지배 실패 | wandb (job_type=eval) |
|---|---|---|---|---|---|---|
| cv | **44.5%** | 81.5% (163/200) | 0.775 / 1.463 / 2.170 m | 0.688 m | timeout 45 · OOR 18 | `1nvvuogg` |
| ct | **33.8%** | 82.6% (166/201) | 0.783 / 1.870 / 3.106 m | 0.814 m | timeout 56 · OOR 5 | `prdqujah` |
| ca | **16.5%** | 63.5% (127/200) | 1.063 / 2.408 / 4.835 m | 1.195 m | **OOR 50(25%)** · timeout 41 · bad_att 8 | `gdow3vfg` |

**판정: 이동 타겟에서 릴리스 능력은 이월되나(63–83% 발화) 명중 능력이 리드 부재로
경계에 몰림.** released-but-miss > success(cv 74/89, ct 98/68, ca 94/33), 착탄 med가
성공반경 1.0 m 부근 — CCIP가 현재 위치를 조준해 낙하시간(t_f≈1.2–1.5 s) 동안의 타겟
이동분(|v|·t_f ≤ 2–3 m)이 구조적 편차로 남는 서명. ca는 가속 이탈로 OOR 25%가 추가
병목(release 자체가 63.5%로 하락).

- ⚠️ **eval 표본 노이즈**: play.py seed 미고정 — 동일 cv ckpt 2회 eval 32.0%↔44.5%
  (env 랜덤화 표본 차, binomial σ≈3.5pp 초과). 모델 간 비교는 동일-표본/대표본 필요.
  (1차 cv eval은 play.py `--wandb` 잠복 NameError로 figure 미기록 — 수정 후 재실행분이
  본 표. 수정: `impacted=released` — land-terminal에선 release_impact_err=실착탄.)

## 3c. 개선 방향

1. **리드 개입(1순위)**: (a) Singer-KF obs v19 포팅(28→35, fresh/부분로드) ·
   (b) privileged target-vel obs 2-dim(28→30)으로 리드 학습 가능성 분리 검증 ·
   (c) base env Phase-3 `w_lead` 이식(보상 단독은 Rule 22 주의).
2. **ca**: `target_accel` 커리큘럼 또는 타겟 |v| cap → OOR 25% 완화.
3. **timeout 20–28%**: 게이트(d_impact≤1.5 m)가 이동 타겟에서 안 열림 —
   release_radius 재검 or 게이트 조준 리드 반영.
4. iter 예산 확대는 구조 개입 후(Rule 20f).

## 4. 체크포인트

- 컨테이너 `/workspace/exp021/exp021_mt_{cv,ct,ca}_final.pt`
- 호스트 백업 `/opt/drone-bombard/checkpoints/exp021/`

## 관련

- [[research/moving_target_models]] — 모션 모델/KF 설계
- [[experiments/training_history]] / [[research/rl_rules]]
- [[daily/daily_2026-07-30]]
