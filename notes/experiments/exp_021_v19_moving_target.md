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
  σ-지터에 가려짐 — exp_017 교훈). det 200-ep eval(`play.py --moving_target
  --target_motion {m}` 동일 플래그)은 후속 작업.

## 4. 체크포인트

- 컨테이너 `/workspace/exp021/exp021_mt_{cv,ct,ca}_final.pt`
- 호스트 백업 `/opt/drone-bombard/checkpoints/exp021/`

## 관련

- [[research/moving_target_models]] — 모션 모델/KF 설계
- [[experiments/training_history]] / [[research/rl_rules]]
- [[daily/daily_2026-07-30]]
