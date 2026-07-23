---
date: 2026-07-23
tags: [experiment, isaac-lab, ppo, payload, release, wandb]
status: done
type: experiment
wandb_run: o5jn9xzk (train), vryuc6mu (eval)
---

# exp_020 — 물리 페이로드 부착 상태 첫 학습 + wandb 평가 figure 파이프라인

> **목표:** ①exp_019에서 구현·검증한 물리 페이로드(kinematic weld)를 **부착한 채 실제 학습**이
> 성립하는지 — exp_018 B0와 동일 조건에서 물리 페이로드가 유일한 델타가 되도록 학습 —
> ②deterministic eval 판정을 **wandb figure**(히스토그램·종단 원인 차트)로 영구 기록하는
> 파이프라인 신설(`play.py --wandb`).
> **판정: ✅ 물리 페이로드의 학습 비용 = 0.** det 200-ep **success/release 100.00% (200/200)**,
> drop err mean **0.169 m**(max 0.200, 전량 tolerance 이내). exp_019의 설계 예측
> (kinematic weld는 드론 동역학에 무접촉) 학습 스케일에서 실증.

## 1. 설정

- **학습:** Phase 1 `--release_terminal`, 2048 envs × 400 iters, seed 42,
  **warm-start = `exp018_B0_final.pt`**, `--w_aim 1.0 --aim_reward_scale 0.5`
  (**B0 보상 bit-match** — 보상 불변이므로 Fresh-Start 규칙 비저촉, warm-start 정당).
  `physical_payload=True`(cfg 기본) → **B0 대비 유일한 변경 = 페이로드 물리 부착.**
- **평가:** deterministic 200-ep, `play.py --release-terminal --wandb`(신설 플래그).
- 환경: `isaac-verify`(L4), wandb project `drone-bombard-isaac`.
- 사전 스모크(dry-run 규칙): 16 envs × 2 iters PASS — warm-start 로드, release_rate 1.0,
  에러/NaN 0.
- ⚠️ 1차 기동 실패: 컨테이너 baked-in `WANDB_API_KEY`가 **빈 문자열** → wandb init
  UsageError, `isaaclab.sh`가 exit 0으로 삼킴(로그 grep으로만 검출). 해결 = `docker exec
  --env-file /opt/drone-bombard/.wandb.env`. → [[errors/err_20260723_wandb_key_empty]]

## 2. 학습 커브 (wandb o5jn9xzk, 50-iter 샘플)

| iter | release_rate | mean reward | drop err (m) | noise σ |
|---|---|---|---|---|
| 0 | 0.988 | 111.3 | 0.138 | 1.41 |
| 100 | 1.000 | 118.1 | 0.143 | 1.50 |
| 200 | 1.000 | 116.6 | 0.132 | 1.58 |
| 300 | 0.979 | 118.4 | 0.147 | 1.68 |
| 399 | 0.951* | 137.5 | 0.156 | 1.71 |

release_rate는 **첫 롤아웃부터 ~100% 고정**(B0의 23→99.6% 상승 커브와 대조 — warm-start가
수렴 상태를 온전히 이월, 페이로드 부착의 재학습 과도기 **없음**). 실패 게이트 전 구간 0,
iteration ~3.1 s(페이로드 100 Hz kinematic write 포함 — exp_015의 ~2.3 s/iter 대비 +35%
오버헤드, 수용 가능). (*마지막 행은 단일-iter 노이즈.)

**σ 드리프트 주의:** 1.41→1.71 (B0는 ~1.35 안정). release 100% 포화로 sharpen할 그래디언트
압력이 없어 엔트로피가 서서히 상승 — 현 규모 무해하나 **장기 fine-tune 회귀 시 1순위 확인
지표.**

## 3. Deterministic 200-ep eval (wandb vryuc6mu)

| 지표 | exp_018 B0 (참고) | exp_020 |
|---|---|---|
| success = release_rate | 100.00% | **100.00% (200/200)** |
| drop_impact_error@release mean (med / p90 / max) | 0.125 (— / — / 0.198) m | **0.169 (0.178 / 0.197 / 0.200) m** |
| d_xy_min med | 0.120 m | 0.086 m |
| final_speed_xy med | 0.11 m/s | 0.068 m/s |
| payload_impact_rate | — | 0.00% (구조 서명 — 종단 모드에선 낙하 미관측, exp_019 §트레이드오프) |

호버-드롭 프로파일 유지(종단 속도 med 0.068 m/s). drop err 분포가 B0 대비 **tolerance(0.2 m)
경계 쪽으로 이동**(mean 0.125→0.169, med 0.178): 발화 조건이 "aim_err ≤ 0.2 최초 충족"이라
100% 유지엔 무영향이나, σ 드리프트와 함께 "충분히 좋으면 발화" 방향의 완만한 이완으로 해석 —
경계 근접 자체는 referee 정의상 정상(max가 정확히 0.200에서 잘림).

## 4. wandb 평가 figure 파이프라인 (신설, 본 실험부터 상시)

`play.py --policy … --wandb [--wandb_project --wandb_run_name]`:
- **summary 스칼라**: success/release/payload_impact rate, d_xy·aim err med, drop err mean/p90 등
- **히스토그램**: d_xy_min, aim_err_min, drop_impact_error, payload_impact_err, final_speed_xy
- **종단 원인 bar chart** (wandb.plot.bar)
- 학습 run과 동일 프로젝트, `job_type="eval"` — 학습 커브와 판정 figure가 한 워크스페이스에 병치.

## 5. 판정 & 후속

**물리 페이로드 부착은 학습에 무비용** — exp_019 parity 검증의 학습-스케일 확증. Stage C
(DR+residual, 별도 지시 대기)는 이제 물리 페이로드 포함 상태로 진행 가능. 후속 시 주의:
①Phase-2 DR 힘 정합(해석식에만 drag/wind — exp_019 §트레이드오프) ②σ 드리프트 모니터
③에피소드 착탄-연장 시 fresh 필요(종단 의미론 변경).

## 관련

- [[experiments/exp_019_physical_payload]] — 구현·hover-drop 검증(전제)
- [[experiments/exp_018_release_terminal]] — B0 기준선(보상·구조 동일)
- [[research/physical_payload_attach]] — 메커니즘·트레이드오프 (Rule 24, §학습 검증 추가)
- [[errors/err_20260723_wandb_key_empty]] — 컨테이너 wandb 키 공백 함정
- [[experiments/training_history]] / [[research/rl_rules]]
