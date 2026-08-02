---
date: 2026-07-15
tags: [experiment, isaac, v11, relaxed-test, dry-run, drop-signal, release-envelope, success-rate]
status: done
type: experiment
wandb_run: "N/A (tensorboard local: 2026-07-15_*_v11_dryrun)"
owner: junsang
---

# Exp 006 — Isaac v11 완화(relaxed) 테스트: 첫 통합 단일-phase dry-run 성공

> **목적:** Gazebo→Isaac 마이그레이션 이후 첫 Isaac 학습. doc 53 "Isaac v11" 스펙을
> **완화 버전**으로 구현해 (a) Isaac Sim이 실제로 학습되는지, (b) 통합 단일 phase에서
> drop_signal + release envelope 과제가 학습되는지 확인.
> **출처:** `final_integrated_document_set/53`, branch `Issac_JS`, `isaac_lab/drone_bombard/v11_env.py`

---

## 완화 버전 설계 (삭제 없이 토글 — 확장 대비)

기존 `DroneBombardEnv`를 **서브클래스**(`DroneBombardV11Env`)로 얹음. 씬·캐스케이드
속도 컨트롤러·물리·spawn-time mass는 상속 그대로. 억제 기능(DR/residual/이동타겟/
vision/커리큘럼)은 **비활성 hook으로 보존** — 나중에 플래그로 확장.

| 결정 | 구현 |
|------|------|
| 단일 통합 phase (커리큘럼 제거) | `phase=1` 고정, DR/residual/moving/release 플래그 모두 inert |
| random marker spawn 제거 → 고정 | marker를 cruise 방향 **정면 20m**에 고정 (`marker_dist=20`) |
| C=A: 정책 drop_signal | `action[4]=drop_signal` + release envelope 게이트. 게이트 밖 drop = **no-op·무페널티** (Gazebo v7 교훈) |
| A=실제 cruise 속도 | 드론이 **4 m/s로 순항 중** spawn (`cruise_speed=4`), 고정 방향/고도 10m |
| B: vision 제거 | obs **24D** (marker상대·ENU속도·roll/pitch·yaw sin/cos·ang_vel·CCIP오차·d_impact·t_f·속력·고도·payload·prev_action) |
| envelope/reward 상수 | doc 53 §6·§8 그대로 |

---

## 🐞 핵심 버그 & 수정 — cruise 핸드오프 자세 폭주 (step-1 즉사)

**증상 (smoke 1차, 16env/2iter):** `Mean episode length 1.00`, `bad_attitude 1.0`,
모든 에피소드가 **첫 스텝에 종료** → 학습 신호 0.

**원인:** v11은 드론이 이미 4 m/s로 순항하며 spawn하는데, `super()._reset_idx`가 속도
LPF(`_v_filt`)와 `_prev_action`을 **0으로 리셋**. 첫 스텝 컨트롤러 setpoint = 랜덤 정책
명령 → 실제 4 m/s와 **~cruise_speed 크기 속도오차** → 컨트롤러가 급격히 tilt →
`ang_vel > limit_ang_vel(2.0)` → `bad_attitude` → 즉시 종료.
(부모 env는 거의 정지 spawn(`init_vel_std`)이라 이 전이가 없음.)

**수정:** reset 시 `_prev_action[:, :4]`(정규화)와 `_v_filt`(world-frame)를 **cruise
setpoint로 seed**. rate-limit(0.2)이 첫 스텝 명령을 cruise 근처로 유지 → 속도오차 소멸
→ "이미 순항 중인 드론을 넘겨받아 명령 없으면 순항 유지". → [[research/isaac_cruise_handoff_junsang]]

---

## 결과

VM(L4) 격리 환경: 공용 `isaac-worktree` 무손상 + 우리 소유 복사본(`wt-js`) + 전용
컨테이너 `js-v11`(`--user root`, 자체 캐시).

| 단계 | 설정 | 결과 |
|------|------|------|
| smoke (수정 전) | 16env / 2iter | ❌ length 1.00, bad_attitude 1.0 |
| smoke (수정 후) | 16env / 3iter | ✅ length ~40+, 종료 정상(timeout/탐험 bad_attitude), EXIT 0 |
| **dry-run** | **512env / 300iter** | ✅ **~iter 88 수렴** |

**dry-run 수렴 지표 (안정):**
- `Episode_Termination/success` = **1.00 (100%)**
- `Episode_Metric/release_rate` = **1.00 (100%)**
- `drop_impact_error_terminal_m` = **~0.75 m** (success_radius 1.0 이내)
- `Mean reward` = **~314** (landing 최대보상 300 근처)

→ v11이 통합 과제를 완전 학습: cruise 핸드오프 → 하강해 release envelope 진입(spawn
10m → band [3,8]m) → 정책 drop_signal로 최적 시점 투하 → payload가 marker 0.75m
이내 착탄.

---

## 남은 관찰 / 다음

- `Episode_Reward/*`가 부모 키(rew_dist/rew_aim…)라 v11 보상 성분이 로깅에 안 잡힘
  (총보상·success·release·착탄은 로깅됨, 학습엔 무영향). 로깅 성분화는 폴리시.
- 성공 확인됨 → 서서히 확장: wind/DR → residual(action[5]) → 이동타겟 → vision.
- 관련: [[research/isaac_cruise_handoff_junsang]], [[experiments/training_history]], [[research/rl_rules]]
