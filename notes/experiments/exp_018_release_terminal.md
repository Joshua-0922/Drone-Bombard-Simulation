---
date: 2026-07-06
tags: [experiment, isaac-lab, ppo, termination, ccip, release]
status: done
type: experiment
wandb_run: xt0hrr1c (B0), 0ns10yso (B1), 4vaodj0o (B2), kk06wsbx (B3)
---

# exp_018 — Stage B: 릴리스-종단 이벤트 + 보상 재탐색

> [[research/release_terminal_stageB]]의 실행 기록. exp_017(판정 b) 직후 사용자
> 지시: ①근접 종단 → 릴리스-종단(실패 게이트 불변) ②aim_err 보상 nominal 전용
> 고정 ③Stage-A 보상 탐색을 소폭·단일-노브로 재실행. DR/residual 없음(전 런
> Phase 1 nominal).

## 1. 설정

- 공통: seed 42, num_envs 2048, 400 iters, **warm-start = `exp017_stageA_final.pt`
  (v1)**, `--release_terminal`, deterministic 200-ep eval (`play.py
  --release-terminal` — 평가 의미론 학습과 일치 필수).
- 지표 의미론 주의: 이 모드에선 발화=종단이라 **release_rate ≡ success rate**.
  Stage A의 5.5%(비종단 래치)와 비교 시 이 차이를 인지할 것.
- 사전 적대 검증 4-lens: parity-off(레거시 bit-identical)/termination(정상)/
  economics(정지-hazard 균형 리스크 경고)/eval-plumbing — **alias 버그 1건
  발견·수정**(§ research 노트 §1; play.py가 success 0%로 보고할 뻔함).
- 아티팩트: 컨테이너 `/workspace/logs/isaac_lab/drone_bombard/exp018_{B0,B1,B2,B3}_final.pt`
  + 호스트 `/opt/drone-bombard/checkpoints/exp018/`.

| Run | wandb | w_aim | knee(m) | 변경 축 |
|---|---|---|---|---|
| B0 | xt0hrr1c | 1.0 | 0.5 | 종단만 (= Stage A v1 보상) |
| B1 | 0ns10yso | 1.5 | 0.5 | w_aim +50% |
| B2 | 4vaodj0o | 1.0 | 0.75 | knee +50% |
| B3 | kk06wsbx | 0.0 | — | aim 항 제거 (하한 앵커) |

## 2. B0 학습 커브 (1차 신호 = 학습 내 release_rate)

23% → 33% → 72% → 92% → **99.6%** (iters 800→1199, 25-iter 버킷; § research 노트
표). max_alt 78%→0.5%(초기 과도기 = v1의 미학습 post-approach 행동),
stagnation/timeout/overshoot 전 구간 0, ep_len 52→36(farm 없음), σ ~1.35 안정,
drop_impact_error@release 전 구간 ~0.13 m.

**Stage A 대비**: 동일 보상·동일 warm-start에서 학습 내 단조 하락(12→3.7%)이
단조 상승(23→99.6%)으로 반전 — 종단 구조가 지배 요인.

## 3. Deterministic 200-ep eval

| 지표 | Stage A v1 (참고) | B0 | B1 | B2 | B3 |
|---|---|---|---|---|---|
| release_rate@0.2m | 5.5% (비종단 래치) | **100.00%** | **100.00%** | 98.51% (max_alt 4) | **100.00%** |
| drop_impact_error@release mean (max) | 0.138 m | 0.125 (0.198) m | 0.131 (0.200) m | 0.137 (0.199) m | 0.128 (0.198) m |
| aim_err_min med | 0.889 m | 0.125 m | 0.133 m | 0.155 m | 0.133 m |
| final_speed_xy med | 2.72 m/s | 0.11 m/s | 0.07 m/s | 0.30 m/s | 0.11 m/s |
| d_xy_min med | 0.697 m | 0.120 m | 0.043 m | 0.555 m | 0.087 m |

B0 수렴 행동 = **호버-드롭**: 타겟 직상 저속(med 0.11 m/s) 진입 → aim_err→0 →
발화. 속도 캐리 소멸로 레거시 terminal 지표(0.125 m)도 릴리스 지표와 일치.

## 4. 판정

**릴리스-종단 구조가 문제를 완전 해결.** 종단 교체 단독(B0, 보상은 Stage A v1
그대로)으로 det release_rate 5.5%→**100.00%**, drop err 0.125 m, 학습 내 추세
단조 상승(Stage A의 단조 하락 반전). 보상 노브 스윕(w 0/1.0/1.5, knee
0.5/0.75)은 결과에 거의 불감 — **aim 항은 이 구조에서 사실상 잉여**(B3=100%;
단 warm-start에 Stage-A aim 학습이 내재된 점 유의), 넓은 knee만 근소 열화.
학습 내 커브 요약(50-iter 버킷 release_rate): B0 0.22→0.43→0.90→0.99 /
B1 0.22→0.36→0.93→1.00 / B2 0.21→0.49→0.82→0.90(48% 늦음) / B3 0.20→0.81→0.94→0.98.
구조 요인 3종 전부 해소/역전(상세 § research §4). **Stage C(DR+residual, 별도
지시) warm-start = `exp018_B0_final.pt`.** Rule 23.

구조 요인 3종 해소/잔존 분석: [[research/release_terminal_stageB]] §4.

## 관련
- [[research/release_terminal_stageB]] ←→ 본 노트
- [[experiments/exp_017_stageA_aim_reward]] — 직전 단계 (판정 b)
- [[experiments/exp_015_phased_curriculum]] — Phase 2 본선(Stage C: DR+residual)
- [[experiments/training_history]]
