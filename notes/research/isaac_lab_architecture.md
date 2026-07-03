---
date: 2026-07-03
tags: [research, architecture, isaac-lab, ppo, rsl_rl, migration]
status: active
type: research
---

# 시스템 전체 구조 (Isaac Lab — `feat/isaac-env-migration` 브랜치)

> **한 줄:** Gazebo/PX4/ROS2/SAC 4-패키지 파이프라인([[research/system_overview]])을
> **단일 프로세스, GPU-vectorized Isaac Lab `DirectRLEnv` 하나**로 대체. PX4 미들웨어,
> ROS2 토픽, Gazebo 물리엔진, 멀티노드 IPC가 전부 사라지고, 그 역할이 `isaac_lab/`
> 안의 순수 torch 함수 + isaaclab API 호출로 대체됨. PPO(rsl_rl)로 SAC 대체.
>
> **저장 위치:** `/opt/drone-bombard/isaac-worktree` (별도 git worktree, `feat/isaac-env-migration`
> 브랜치). `jekyun`(라이브 SAC 학습 중인 메인 체크아웃)과는 완전히 분리된 디렉터리 —
> 서로 영향 없음. 상세 배경: [[experiments/exp_012_isaac_migration_phase2]] / Rule 16.

---

## 저장소 전체 레이아웃 (이 워크트리)

`jekyun`에서 병합(`git merge jekyun`, `940c88b`)해왔기 때문에 Gazebo/PX4/ROS2 관련 디렉터리가
전부 그대로 남아있다 — **의도적으로 미삭제** (§"Gazebo/PX4 잔존 이유" 참조).

```
isaac-worktree/
├── isaac_lab/                  ← 신규. 이 브랜치의 핵심 산출물 (아래 상세)
├── drone_drop_system/docker/   ← Isaac Sim 5.1.0 + Isaac Lab v2.3.2 + rsl_rl 이미지 빌드
│                                  (Phase 1, 이 브랜치에서만 — jekyun의 Dockerfile은 여전히
│                                  Gazebo/PX4/ROS2 기반, 별개)
├── infra/                       ← GCP L4 Spot VM 배포 (deploy.sh, startup.sh) — Isaac 전용
├── ros2_ws/, gazebo_models/     ← 구 Gazebo/PX4/ROS2 워크스페이스. Isaac 학습 이미지에는
│                                  포함 안 됨(.dockerignore로 제외), 코드에서도 미참조.
│                                  삭제는 게이트 조건부로 보류 중(§"Gazebo/PX4 잔존 이유")
├── drone_bombard_best.pt        ← YOLO 가중치 (레포 루트, ros2_ws 밖) — isaac_lab/yolo_eval.py가 직접 참조
├── yolo_workspace/               ← YOLO 학습 산출물 (구 시스템 학습 결과물, 참고용)
└── notes/                        ← 이 문서를 포함한 전체 연구 노트 (jekyun과 공유 이력)
```

## `isaac_lab/` 내부 구조 및 역할

```
isaac_lab/
├── README.md                          실행 절차, parity 요약, 알려진 리스크
├── drone_bombard/
│   ├── __init__.py                    gym.register("Isaac-DroneBombard-Direct-v0")
│   ├── math_utils.py                  ★ 순수 torch 함수 전부 — isaaclab 무의존
│   ├── drone_bombard_env.py           ★ DirectRLEnv — isaaclab lifecycle 배선
│   ├── mdp/domain_rand.py             Phase-2 도메인 랜덤화 스텁 (Phase 1은 항등)
│   └── agents/rsl_rl_ppo_cfg.py       PPO 하이퍼파라미터
├── train.py                           rsl_rl 학습 진입점 (SIGTERM 프리엠션 저장 포함)
├── play.py                            sanity-check 모드 (--zero-actions/--scripted/--step-response)
├── yolo_eval.py                       실제 YOLOv8 평가 + vision 캘리브레이션
└── tests/test_math.py                 29개 순수 torch 유닛 테스트 (isaaclab 불필요)
```

### 왜 `math_utils.py`와 `drone_bombard_env.py`가 분리되어 있나

`drone_bombard_env.py`는 최상단에서 `import isaaclab...`를 하기 때문에, isaaclab이 설치된
환경(L4 GPU VM)에서만 import 가능하다. 이 dev VM은 GPU driver가 낡아(535 < Isaac Sim
5.1.0 요구 580) isaaclab을 설치할 수 없다 — 그래서 **보상/액션/비전/탄도/가드 계산 로직
전부를 `math_utils.py`로 분리**해 isaaclab 없이 순수 torch로 단위테스트 가능하게 만들었다.
`drone_bombard_env.py`는 이 순수 함수들을 isaaclab의 `DirectRLEnv` 생명주기(씬 구성, 물리
스텝, 관측/보상/종료 계산, 리셋)에 "배선"만 한다. `tests/test_math.py`는 파일 경로를 직접
로드해 `drone_bombard/__init__.py`의 isaaclab import 체인을 우회한다.

## 데이터 흐름 (한 policy step, 10 Hz)

```
policy action [4] (±1, ENU vx,vy,vz,yaw_rate)
  │
  ├─ _pre_physics_step(): clip → rate-limit(±0.2, math_utils.rate_limit_action) → 물리단위 스케일(4/3/3/1)
  │
  ├─ _apply_action() ×10 (100Hz 물리 스텝, decimation=10 = 10Hz policy와 정합)
  │     ├─ 5스텝마다(=20Hz): LPF 틱(math_utils.lpf_step, α=0.4) — PX4 controller 흉내
  │     └─ 매 스텝: 캐스케이드 속도→자세→토크 컨트롤러 → set_external_force_and_torque
  │           (PX4 내부 루프를 흉내낸 단일 rigid-body wrench — 로터 개별 시뮬레이션 안 함)
  │
  ├─ _get_observations(): pos/vel/ang_vel(GT, 정규화) + _update_vision()(analytic pinhole
  │     투영, math_utils.project_target_pinhole + hold_buffer_update) + rel_dx/dy(GT, 특권 정보)
  │     → obs[14]
  │
  ├─ _get_dones(): crash/overspeed/bad_attitude/out_of_range/max_altitude/overshoot
  │     (math_utils.overshoot_guard)/stagnation(math_utils.stagnation_guard)/success/timeout
  │     → self._done_flags에 캐시 (다음 두 단계가 재사용)
  │
  ├─ _get_rewards(): math_utils.compute_reward(3-layer) + self._done_flags 기반 종단 보상
  │
  └─ terminated|truncated된 env → _reset_idx(): 종료 상태 스냅샷(§ 아래) → 타겟/스폰 랜덤화 →
        텔레포트 → 버퍼 초기화 → 로그
```

**핵심 설계 원칙 (Rule 16 참조):** Gazebo 버전의 값(상수)뿐 아니라 **타이밍**(10Hz policy,
20Hz LPF tick)과 **메커니즘**(guard의 의도된 dormancy 등)까지 그대로 이식 — 그래야 plant가
같아지고 v13/v15와의 행동 비교가 시뮬레이터 차이만 반영하게 된다.

## Isaac 쪽에서만 존재하는 개념 (Gazebo와의 구조적 차이)

| 개념 | Gazebo/PX4 | Isaac Lab |
|---|---|---|
| 병렬화 | `num_envs=1`(단일 Gazebo가 PX4 lockstep 직렬화) | `num_envs=2048`(GPU-vectorized, 단일 프로세스) |
| 타겟 위치 | 고정 ENU (11,10) | env마다 ±10m 랜덤화 |
| 스폰 거리 | 스크립트 CRUISE 후 자연 발생(~5m) | env마다 3-7m 랜덤화 |
| Vision | 실제 카메라 렌더링 + YOLOv8 항상 추론 | 학습=analytic pinhole 투영(YOLO 캘리브레이션 노이즈), 평가만 실제 YOLOv8(`yolo_eval.py`) |
| 액추에이션 | PX4 전체 비행 컨트롤 스택(EKF, 믹서, 로터) | 단일 rigid-body에 직접 wrench (§"캐스케이드 컨트롤러") |
| 리셋 | teleport+disarm 또는 soft reset(비행 유지, EKF 재수렴 회피) | 순간 텔레포트(EKF 개념 자체가 없음 — ground-truth state) |
| terminated/truncated | 실패=truncated(SB3가 bootstrap) | 실패=terminated(PPO는 bootstrap 안 함, §Rule 16) |
| Drop | 스크립트 referee (DetachableJoint + `/rl/drop_error`) | 스크립트 CCIP 메트릭(analytic ballistic, 물리 조인트 없음) |

## Phase-2 훅 (현재 비활성, 배선만 되어 있음)

`math_utils.py`에 4개 인터페이스가 미리 잡혀 있으나 Phase 1 설정으로는 완전 우회됨
(hook-parity 유닛테스트로 bit-identical 보장):

1. **CCIP residual** (`ccip_residual`, `DropCfg.residual_enabled=False`) — 학습된 탄도 보정.
2. **release 상수 cfg화** (`DropCfg.release_tolerance/release_delay`) — inline 리터럴 제거.
3. **obs superset 고정** — Phase 2는 index 0-13 뒤에만 채널 추가 (체크포인트 warm-start 호환).
4. **도메인 랜덤화 스텁** (`mdp/domain_rand.py`) — 항력/바람. Phase 1은 항등(0).

## Gazebo/PX4 잔존 이유 (삭제 보류 결정, 2026-07-03)

이 브랜치의 `ros2_ws/`/`gazebo_models`는 Isaac 학습 이미지에 포함되지 않고(`.dockerignore`)
`isaac_lab/` 코드 어디서도 참조되지 않는다 — 기능적으로는 죽은 무게다. 그럼에도 지금
삭제하지 않는 이유:

1. **Isaac 코드가 아직 한 번도 실행된 적 없음** — L4 GPU VM 미기동, 이 dev VM은 driver가
   낡아 Isaac Sim 자체를 못 돌림. 검증 전 유일한 레퍼런스 구현을 지우는 리스크.
2. **속도 컨트롤러가 PX4 대비 미검정** ([[research/isaac_velocity_controller]]) — 게인
   검정에 Gazebo/PX4 세션에서 캡처한 실측 스텝응답이 필요(`vel_logger_v2.py` 신규 예정).
   Gazebo/PX4가 사라지면 이 캡처를 다시 할 수 없다.

삭제는 두 조건(L4 VM env 스모크 통과 + 스텝응답 캡처 완료) 충족 후 진행 예정으로
[[experiments/exp_012_isaac_migration_phase2]] §8에 추적 중. **대상은 이 워크트리
(`feat/isaac-env-migration`)뿐 — `jekyun`(라이브 SAC 학습 체크아웃)은 절대 정리 대상 아님.**

## 관련 링크

- [[experiments/exp_012_isaac_migration_phase2]] — 이식 작업 상세, parity 표, 검증 결과
- [[research/isaac_velocity_controller]] — 캐스케이드 컨트롤러 게인, 검정 계획
- [[research/rl_rules]] Rule 16 — 시뮬레이터 이식 시 plant/reward parity 원칙
- [[research/system_overview]] — 대조: `jekyun` 브랜치의 Gazebo/PX4/ROS2 아키텍처
- [[daily/daily_2026-07-03]] — 이 브랜치 작업의 세션 로그
