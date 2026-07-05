---
date: 2026-07-05
tags: [research, isaac-lab, curriculum, ccip, residual, warm-start, ppo]
status: active
type: research
---

# Phase별 순차 커리큘럼 — 설계·수식·warm-start 원리

> exp_015 구현의 설계 근거. 이미지의 3단계 표(접근/nominal → CCIP+Residual/정지 → 이동타겟)를
> Isaac Lab env에서 작동시키기 위한 핵심 결정과 수식.

관련: [[experiments/exp_015_phased_curriculum]] · [[experiments/exp_012_isaac_migration_phase2]] · [[research/isaac_velocity_controller]] · [[research/rl_rules]]

---

## 1. 고정 6-dim 액션 스페이스 — warm-start 무손실의 전제

커리큘럼의 핵심은 페이즈 간 **weight 인계(warm-start)**. rsl_rl `OnPolicyRunner`는 env의
action space에서 actor/critic MLP를 구성하므로, 페이즈마다 action 차원이 다르면
`runner.load()`가 shape 불일치로 실패한다. 따라서 **모든 페이즈에서 action=6 고정**:

- `action[0:4]` = ENU 속도 setpoint (vx, vy, vz, yaw) — 전 페이즈 활성
- `action[4:6]` = CCIP 착탄점 residual (δx, δy) — Phase 2+ 만 활성

Phase 1은 env가 `action[4:6]`을 완전히 무시(`residual_enabled=False`, `_pre_physics_step`에서
zero-out)하므로, exp_014 4-dim 접근 태스크와 **동작 동일**하면서 아키텍처는 6-dim이라 Phase 2가
Phase 1 체크포인트를 그대로 로드한다. 대가: Phase 1 정책이 2개 출력 dim을 낭비(무해). 대안인
부분 로딩(4→6 surgery)보다 견고해 이 방식을 택함.

## 2. 릴리스 이벤트 & model-mismatch 보정 (Phase 2)

온보드 CCIP는 **nominal**(항력·바람 무시) 탄도로 착탄점을 예측하고, 정책 residual이 그것을
보정한다. 실제 낙하는 도메인랜덤화된 물리(drag/wind)로 계산 — 이 **예측-실제 격차**가 residual이
닫아야 할 model mismatch다.

- 자유낙하 시간: $t_{fall} = \sqrt{2H/g}$ (`time_to_fall`)
- nominal 예측 착탄: $\hat{p} = p_{xy} + v_{xy}(t_{fall}+\tau_{delay})$ (`predict_impact_nominal`)
- residual 보정: $\hat{p}' = \hat{p} + \delta \cdot s_{res}$, $\delta\in[-1,1]^2$ (`apply_ccip_residual`)
- 릴리스 트리거: $\lVert \hat{p}' - p_{target} \rVert \le tol$ 이고 $H > H_{min}$
- 실제 착탄(DR): $p_{real} = \text{ballistic}(p, v, H, \tau, g, c_{drag}, w)$
- 진짜 오차: $err = \lVert p_{real} - p_{target}\rVert$
- 터미널 보상: $r_{impact} = w_{impact}\,e^{-err/s_{impact}}$ (`impact_terminal_reward`)

정책은 "예측이 타겟에 맞도록" 비행하면서, residual로 drag/wind 편의를 상쇄해 **실제** 착탄이
타겟에 떨어지게 학습한다. 성공 = 릴리스 & $err \le r_{success}$.

## 3. 이동 타겟 & 리드 (Phase 3)

- 타겟 속도 Gauss-Markov(OU): $v \leftarrow (1-\theta\,dt)v + \sigma\sqrt{dt}\,\epsilon$
  (`step_target_velocity`) — 평균회귀로 발산 방지. 위치는 policy step마다 $p_t \mathrel{+}= v\,dt$.
- lead aim: 예측 착탄시점의 타겟 위치 $p_{target} + v_{target}\,t_{fall}$ 를 조준점으로 사용.
- lead 보상: $r_{lead} = w_{lead}\,e^{-\lVert \hat{p}' - p_{target}^{impact}\rVert / s_{lead}}$
  (`lead_prediction_reward`).

**한계(중요):** rsl_rl actor는 MLP(무기억)라 **단일 obs에서 타겟 속도를 추론할 수 없다**. 현재
구현은 env가 lead를 해석적으로 계산(aim에 $v_{target}$ 사용)하고 residual/보상이 그 위에서
작동한다 — "요격기하는 env가 제공, residual은 모델오차 보정" 설계. 진짜 정책-학습 lead를 원하면
**obs에 타겟 속도 2채널 추가(14→16)** 가 후속 과제. (obs superset은 exp_012에서 "append만"으로
설계됐으므로 index 0-13 불변 유지 가능.)

## 4. 순차 오케스트레이션

Isaac Sim은 프로세스당 1 sim만 안전 → `train.py --phases 1,2,3`은 각 페이즈를 별도
서브프로세스(`--phase N --resume <이전 final.pt> --final_out <경로>`)로 실행하고, 각 페이즈의
`model_final.pt`를 다음 페이즈 `--resume`으로 체인. 단일 페이즈(`--phase N`)는 in-process 학습.
오케스트레이터는 sim을 건드리지 않고 서브프로세스 dispatch만 하므로 AppLauncher 이전에 분기·종료.

## 5. Phase 1 불변성 (bit-parity 유지)

`residual_enabled=False`/`dr_enabled=False`/`moving_target_enabled=False`/`release_enabled=False`
일 때: residual zero-out, `sample_*`가 0 반환(drag 브랜치 skip·wind 0벡터), 이동 없음, 릴리스
없음, 성공=proximity + `reward_success`. 즉 Phase 1은 exp_014와 알고리즘적으로 동일 → exp_014
plant/비전 수정(Rule 19)의 100% 결과가 유효한 baseline.

## 관련 노트
- [[experiments/exp_015_phased_curriculum]] — 구현 기록 / L4 실행 절차
- [[research/rl_rules]] Rule 20 — 커리큘럼 warm-start & 태스크 재정의 재검증 규칙
- [[research/isaac_velocity_controller]] — 속도 컨트롤러(액션 0:4 소비처)
