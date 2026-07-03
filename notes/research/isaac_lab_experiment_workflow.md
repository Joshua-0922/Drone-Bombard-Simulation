---
date: 2026-07-03
tags: [research, isaac-lab, workflow, experiment, onboarding, wandb]
status: active
type: research
---

# Isaac Lab 실험 워크플로우 (`feat/isaac-env-migration`) — 신규 연구자 온보딩

> **대상 독자:** 이 브랜치에서 처음 실험을 돌리는 연구자. "보상을 바꿨는데 어떻게
> 실행하고, 뭘 기록하고, 결과를 어떻게 읽나"의 처음-끝 절차.
> 파라미터 레퍼런스는 [[research/isaac_lab_reward_tuning]], 메트릭 해석은
> [[research/isaac_lab_wandb_guide]] — 이 문서는 그 둘을 잇는 절차서.

---

## 0. 먼저 알아야 할 것 — 이 브랜치의 현재 상태 (2026-07-03 기준)

- 코드는 완성되어 **물리 검증(`verify_one_episode.py`, 무학습 1에피소드) PASS** +
  **256-env·20-iteration PPO dry-run 성공**까지 확인됨 (커밋 `5a6a71b`): reward
  −74.98→+28.65, ep_len 2.2→~100, `Episode_Metric/d_xy_min` 0.747(성공 반경 0.8m 도달),
  WandB 프로젝트 `drone-bombard-isaac`에 커스텀 parity 메트릭 전부 정상 로깅 확인.
  즉 §1 사다리의 1)~4)는 이미 한 번 통과했고, 그 결과가 §5의 `Episode_*` 메트릭 이름이
  실제 대시보드에 존재함을 보증한다([[research/isaac_lab_wandb_guide]] 참조).
  **하지만 본 학습(2048 envs, `max_iterations` 기본 3000)은 아직 아무도 완주하지 않았다** —
  새 실험을 시작하는 사람은 "첫 프로덕션 run"의 파이오니어다.
- GPU 드라이버는 이 박스에서 535→580으로 이미 업그레이드됨(커밋 `2778088`, 리부트 없이
  모듈 reload) — RTX 렌더링(GUI, 카메라, `record_episode.py`) 정상 동작 확인됨. Isaac Lab
  실행 자체는 **이 dev VM에서 바로 가능**하다(`isaac_lab/README.md`의 "L4 Spot VM" 서술은
  `infra/deploy.sh` 기반 무인 대규모 학습 배포 경로를 가리키던 것 — 이 dev 박스도 L4 GPU를
  공유하므로 대화형 dry-run/디버깅은 여기서도 된다). 다만 Gazebo SAC 학습(`rl_train` tmux)과
  **GPU를 공유**하므로 대규모 학습 전엔 리소스 경합 여부를 확인할 것.
- 속도 컨트롤러 게인은 **여전히 PX4 대비 미검정**([[research/isaac_velocity_controller]]) —
  안정 비행은 확인됐지만 정량 스텝응답 비교는 아직. 이상 행동이 보이면 "보상이 잘못됐나"
  이전에 "컨트롤러가 명령을 제대로 못 따라가나"부터 의심할 것 — `play.py --step-response`로
  게인 자체를 진단할 수 있다.
- 코드만 검증하고 싶으면(Isaac Sim 불필요) `pytest isaac_lab/tests/test_math.py`(순수 torch,
  30/30 — 브로드캐스트 회귀 테스트 추가로 29→30) 어디서든 가능.

---

## 1. 실행 전 필수 — 4단계 dry-run 사다리

Gazebo Rule 1(Fail-Fast)의 Isaac 버전. `isaac_lab/README.md` "Running on the L4 Spot VM"
절차를 그대로 따를 것 — **각 단계를 건너뛰지 말고 순서대로**:

```bash
# 1) Cartpole 스모크 — Isaac Sim/Isaac Lab/rsl_rl 설치 자체가 정상인지 (infra/startup.sh에 이미 배선)

# 2) env import + 초소형 스모크 (16 envs, 2 iteration)
./isaaclab.sh -p /workspace/drone-bombard/isaac_lab/train.py \
  --task Isaac-DroneBombard-Direct-v0 --headless --num_envs 16 --max_iterations 2

# 3) 물리/액추에이션 sanity
./isaaclab.sh -p /workspace/drone-bombard/isaac_lab/play.py --zero-actions --num_envs 4 --headless
#   -> 10초간 고도 드리프트 < 1m, NaN 없음
./isaaclab.sh -p /workspace/drone-bombard/isaac_lab/play.py --scripted --num_envs 4 --headless
#   -> d_xy가 단조 감소

# 4) 처리량 프로브 (본 학습 전 최종 확인, 20 iteration)
./isaaclab.sh -p /workspace/drone-bombard/isaac_lab/train.py \
  --num_envs 2048 --max_iterations 20 --headless
```

**선택: 정성적 확인.** `isaac_lab/record_episode.py`로 한 에피소드를 mp4로 녹화해 눈으로
확인할 수 있다(페이로드 실린더 + 타겟 X 마커 표시, `_recordings/`에 저장, gitignored) —
숫자 지표만으로 판단하기 애매할 때(예: overshoot 비율이 애매하게 높을 때) 실제로 어떻게
날아가는지 보는 것이 가장 빠른 진단이다.

**보상 공식을 바꿨다면** 3)은 건너뛰어도 되지만(물리는 안 바뀜) 4)는 반드시 다시 —
`Episode_Reward/*`(§[[research/isaac_lab_wandb_guide]] §3)가 기대한 부호/크기로 나오는지
20 iteration만에 확인하고 본 학습(3000 iteration, `max_iterations` 기본값) 진입.

---

## 2. 파라미터를 바꾸는 법

[[research/isaac_lab_reward_tuning]] §0 참조 — 요약하면:

1. `isaac_lab/drone_bombard/drone_bombard_env.py`의 해당 `@configclass` 기본값을 직접 수정
   (git diff로 이력 남음 — **권장 경로**).
2. 또는 `train.py` 상단에서 `env_cfg.<group>.<field> = ...`로 오버라이드 (여러 값을
   스크립트로 스윕할 때). 이 경우 코드 diff가 안 남으므로 **아래 §4 로깅에 반드시 명시**.

바꾼 뒤 §1의 4단계 사다리를 처음부터 다시 밟을 것 — 특히 termination/reward cfg 변경은
2)~4) 모두 재실행 권장.

---

## 3. Fresh start vs. `--resume` 판단 기준

Gazebo Rule 4("보상 공식 변경 → 반드시 fresh start")의 PPO 버전. PPO는 replay buffer가
없어 Rule 4의 문자 그대로의 메커니즘(오염된 버퍼)은 없지만, 판단 원칙은 같다:

| 바꾼 것 | Fresh start (`--run_name` 새로 지정) | `--resume <checkpoint>.pt` |
|---|---|---|
| 보상 가중치(`w_dist`, `w_vel` 등) | ✅ 필수 | ❌ |
| 종단 조건 반경(`success_radius`, `overshoot_close_threshold`) 대폭 변경 | ✅ 필수 | ❌ |
| `success_radius` **커리큘럼 조임**(0.8→0.5, 코드 주석의 의도된 사용법) | ❌ | ✅ 권장 (기존 정책 위에서 조이는 것이 커리큘럼의 취지) |
| PPO 하이퍼파라미터(`learning_rate`, `clip_param`, `entropy_coef` 등, `agents/rsl_rl_ppo_cfg.py`) | 상황에 따라 — 큰 폭 변경(예: lr 10배)이면 fresh 권장 | 소폭 조정이면 resume 무방 |
| 컨트롤러 게인(`DroneBombardControllerCfg`) | ✅ 필수 — 정책이 다른 plant를 다시 배워야 함(Rule 16) | ❌ |
| 스폰/타겟 랜덤화 범위(`reset.*`) | 범위를 좁히는 방향(쉬워짐)이면 resume 가능, 넓히는 방향(어려워짐)이면 fresh 권장 | — |

체크포인트는 `--log_root`(기본 `/workspace/logs/isaac_lab/drone_bombard`) 아래
`{run_name}/model_{iteration}.pt` 및 SIGTERM 시 `model_preempt.pt`(Spot VM 선점 대비,
Gazebo Rule 5의 `_emergency_save`와 동일 역할)로 저장된다. `--resume`은 그 경로를 그대로
받는다.

---

## 4. WandB run 관리

- `--wandb_project drone-bombard-isaac` (기본값, `train.py`) — Gazebo 트랙(`drone-bombard`
  추정 프로젝트명, [[research/rl_rules]] 참조 필요)과 **분리된 프로젝트**. 헷갈리지 말 것.
- `--run_name <설명>`을 항상 명시적으로 줄 것 (안 주면 타임스탬프만 찍혀 나중에 뭘
  바꿨는지 run 이름만으로 알 수 없음). 권장 형식: `{목적}_{바꾼값}` 예:
  `v1_wdist3_success05`.
- 메트릭 읽는 법은 [[research/isaac_lab_wandb_guide]] — 특히 첫 실 run에서는 §5(rsl_rl
  표준 키)의 정확한 이름을 대시보드에서 확인하고 그 문서를 갱신할 것.

---

## 5. 실험 완료 후 — CLAUDE.md MANDATORY 로깅 절차 (이 브랜치도 동일 적용)

Gazebo 트랙과 동일한 3단계가 이 브랜치에도 그대로 적용된다:

1. **`notes/experiments/exp_0NN_{run_name}_{title}.md`** 생성 — WandB run 이름/링크,
   바꾼 cfg 필드(§2 방법 2를 썼다면 특히 명시), `Episode_Termination/success` 최종값,
   `[[research/{finding}]]` 링크.
2. 새로운 발견이 있으면 `notes/research/{finding}.md` — 원인/규칙/향후 조건.
3. 허브 갱신: `notes/experiments/training_history.md`, `notes/research/rl_rules.md`
   (발견이 규칙화될 정도면 새 Rule로), `notes/00_index.md`.

**Isaac 트랙 실험은 Gazebo 트랙과 실험 번호(`exp_0NN`)를 공유한다** — 이미
`exp_012_isaac_migration_phase2`가 이 브랜치 소속이므로 다음은 `exp_013`부터.

---

## 관련 링크

- [[research/isaac_lab_reward_tuning]] — 무엇을 바꿀 수 있는지
- [[research/isaac_lab_wandb_guide]] — 바꾼 결과를 어떻게 읽는지
- [[research/isaac_lab_architecture]] — 왜 이렇게 구성되어 있는지
- [[research/isaac_velocity_controller]] — 컨트롤러 검정 상태(실행 전 반드시 인지)
- [[research/rl_rules]] — Gazebo 트랙 규칙(Rule 1/4/5/16 원리가 여기서도 적용)
- [[experiments/exp_012_isaac_migration_phase2]] — 이 브랜치의 실험 번호 시작점, 현재 상태
