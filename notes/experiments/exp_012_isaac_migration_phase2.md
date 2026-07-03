---
date: 2026-07-03
tags: [experiment, isaac-lab, migration, ppo, rsl_rl, phase2]
status: active
type: experiments
wandb_run: N/A (not yet run — L4 driver required, not built)
---

# exp_012 — Isaac Lab migration Phase 2: env + PPO 코드 이식

> Gazebo+PX4+ROS2 → Isaac Lab. Phase 1(인프라: Isaac Sim 5.1.0 + Isaac Lab v2.3.2 + rsl_rl Docker
> image, `feat/isaac-env-migration` 3커밋)은 완료됐으나 빌드/실행된 적 없음. Phase 2 = 태스크
> 자체(YOLO obs + PX4 속도명령 + drop 스코어링)를 **하나의 env 파일**로 통합 이식 + PPO(rsl_rl)
> 학습 코드 작성.

관련: [[research/isaac_lab_architecture]] · [[research/isaac_velocity_controller]] · [[research/rl_rules]] · [[experiments/exp_011_wobble_lpf_reward_damping]]

---

## 1. 작업 환경

- **워크트리 분리:** `git worktree add /opt/drone-bombard/isaac-worktree feat/isaac-env-migration` +
  `git merge jekyun` (merge commit `940c88b`). 이유: v15 SAC 학습이 `jekyun` 체크아웃에서
  tmux `rl_train`로 라이브 진행 중이라 브랜치 전환 불가.
  - 충돌 2건: `notes/00_index.md`(현재 상태 섹션 — 병행 트랙으로 병합), `notes/daily/daily_2026-05-30.md`
    (add/add — Isaac 인프라 노트와 별개 비전 리팩터링 세션 노트가 같은 날짜에 존재 →
    후자를 [[daily/daily_2026-05-30_vision-refactor]]로 분리 보존).
- **로컬 실행 불가:** dev VM GPU driver 535.309 < Isaac Sim 5.1.0 요구치 580.65.06.
  검증은 (a) 순수 torch 유닛 테스트(로컬, isaaclab 불필요) + (b) L4 Spot VM 스모크 커맨드(문서화만).

## 2. 코드 구조 (`isaac_lab/`)

```
isaac_lab/
  drone_bombard/
    math_utils.py          순수 torch (action rate-limit/LPF, pinhole 투영, hold-buffer,
                            ballistic/CCIP, reward, overshoot/stagnation guard) — isaaclab 무의존
    drone_bombard_env.py    DirectRLEnv: math_utils를 isaaclab lifecycle에 연결
    mdp/domain_rand.py      Phase-2 DR stub (Phase 1은 항등)
    agents/rsl_rl_ppo_cfg.py
  train.py / play.py / yolo_eval.py / tests/test_math.py
```

**설계 결정 (사용자 확인):** vision = 학습 시 analytic pinhole 투영(YOLO 캘리브레이션 노이즈) +
평가 시 실제 YOLOv8(`yolo_eval.py`); drop = 액션 아님, 스크립트 CCIP 메트릭(v15와 동일 태스크
스코프); PPO 라이브러리 = rsl_rl(Docker 이미지 기설치).

## 3. Parity 표 — v13/v15(Gazebo) → Isaac 값 (모두 동일, 상수 그대로 이식)

| 항목 | Gazebo (`hyperparams_v13.yaml` / `drone_drop_env.py`) | Isaac (`DroneBombard*Cfg`) |
|---|---|---|
| pos/vel/ang_vel scale | 50 / 15 / π | 동일 |
| action scale (vx,vy,vz,yaw) | 4.0 / 3.0 / 3.0 / 1.0 | 동일 |
| action rate limit | 0.2 | 동일 |
| velocity LPF alpha | 0.4 @20Hz (컨트롤러 서브프로세스) | 동일, env 내부 20Hz tick(2회/policy step) |
| w_dist / w_proximity / proximity_radius | 2.0 / 0.6 / 2.0 | 동일 |
| w_vision_center | 1.5 | 동일 |
| w_time / w_ang_vel / w_action_smooth | 0.05 / 0.15 / 0.20 | 동일 |
| w_vel / vel_damp_radius | 0.08 / 3.0 | 동일 |
| success_radius / reward_success | 0.8 / 100.0 | 동일 (curriculum 0.5 훅 유지) |
| penalty_crash/overspeed/out_of_range/max_altitude/bad_attitude | -50/-30/-30/-30/-30 | 동일 |
| overshoot_close_threshold / margin / penalty | 0.6 / 2.0 / -10 | 동일 + 진단용 `overshoot_flythrough_radius=1.2`(비종단, §5) |
| stagnation window/progress/start | 150 / 1.0 / 50 | 동일 |
| max_steps | 300 (10Hz) | `episode_length_s=30.0`, decimation=10 @ sim.dt=1/100 → 정확히 300 |
| limit_ang_vel / limit_inverted_tilt | 2.0 / 1.047 rad | 동일 (`limit_tilt: 0.26`는 Gazebo yaml에 있지만 코드에서 미사용 — **의도적으로 이식 안 함**, §4 참조) |
| target 위치 | 고정 ENU (11, 10) | **랜덤화** env당 ±10m (신규 기능) |
| 드론/payload 질량·관성 | x500 SDF 실측 2.07kg + payload 0.1kg, diag(0.0217,0.0217,0.040) | 동일 (rigid-body wrench actuation) |
| max thrust | (PX4 모터 스펙, 미측정) | T/W=2.0(무탑재 airframe 고정값) × 2.07kg × g = **40.6N** |

## 4. 이식 중 발견한 것 (죽은 config, 코드 재확인)

- `hyperparams_v13.yaml`의 `limit_tilt: 0.26`은 `drone_drop_env.py`에서 `_cfg_limit_tilt`로
  로드되지만 어디에도 참조되지 않는 **죽은 설정**. 실제 전도 판정은 코드 기본값
  `limit_inverted_tilt=1.047`(yaml에 미설정)로 이뤄짐. Isaac 측은 1.047만 이식(정확).
- `_filter_velocity`의 tau 관례: 코드 주석은 `tau ≈ dt*(1/alpha-1) ≈ 75ms`(20Hz, alpha=0.4).
  다른 연속시간 근사(`τ=-T/ln(1-α)≈97.9ms`)와 다름 — 이산 재귀식 자체는 동일(`v=α·cmd+(1-α)·v`),
  라벨링 관례 차이일 뿐. 코드베이스 기존 관례(75ms)를 그대로 따름 — Rule 15 노트와 일치.

## 5. 리뷰에서 나온 게이트 3건 반영 (설계 확정)

1. **Overshoot guard는 success_radius=0.8에서 설계상 dormant** — Gazebo 소스
   (`drone_drop_env.py:775-835`) 재확인: success가 먼저 발동하므로 arm_radius(0.6) <
   success_radius(0.8)면 종단 guard는 도달 불가능. Rule 10의 의도된 설계(0.5 curriculum
   전환 시 활성화)이지 이식 버그 아님. **비종단 진단 카운터** `overshoot_flythrough`
   (arm=1.2>0.8) 신설 — 실제 fly-through 패턴을 태스크에 영향 없이 로깅.
2. **속도 컨트롤러는 PX4 대비 미검정** — 게인은 초기값(§ [[research/isaac_velocity_controller]]).
   L4 VM `play.py --step-response`로 7-포인트 매트릭스 캡처 예정 (Gazebo 쪽 캡처 세션 필요,
   기존 `vel_logger.py`는 커맨드만 기록해 응답 채널 없음 — 커버리지 0/7 확인됨).
3. **Phase 2 인터페이스 훅 4종** wiring: CCIP residual slot(비활성 시 완전 우회),
   release_tolerance/delay를 `DropCfg`로(inline 리터럴 제거), obs superset 고정
   (index 0-13 불변, Phase 2는 append만), domain_rand 스텁(Phase 1 항등). 전부
   hook-parity 가정 하에 Phase 1 출력 불변 — `test_math.py`가 ballistic_impact의
   드래그/윈드 항등 케이스로 이 불변성을 검증.

## 6. 검증

- `python3 -m py_compile` 전체 통과 (11개 파일).
- `pytest isaac_lab/tests/test_math.py` — **29/29 통과** (drone-bombard-harmonic 컨테이너,
  torch 2.4.1, isaaclab 미설치 상태로 실행 — 파일 경로 직접 로드로 `drone_bombard/__init__.py`의
  isaaclab import 체인 우회). 커버리지: rate-limit, LPF(이산 스텝응답 y_k=1-0.6^k, 정책스텝
  경계 연속성, per-env 리셋 격리+snap), pinhole 투영(축 부호, 가시성), hold-buffer 카운트다운,
  ballistic/CCIP 훅 항등성, overshoot/stagnation guard, reward 공식 7개 격리 시나리오.
  초기 3건 실패는 전부 테스트 자체의 손계산 상수 오차(구현 버그 아님) — fx 근사값 및 LPF
  격리 테스트의 잘못된 재귀 논리 수정 후 전부 통과.
### 6b. 실제 실행 검증 (2026-07-03, 이 dev 박스에서 라이브) — **VERIFY: PASS**

dev 박스(driver 535)에 `isaac-sim:5.1.0` 이미지를 pull하고 컨테이너에서 Isaac Lab v2.3.2
+ rsl_rl 설치 후 `verify_one_episode.py`(신규 무학습 하네스)로 `Isaac-DroneBombard-Direct-v0`
**1 에피소드 실제 실행**. 결과: env 등록·USD 씬 구성(Crazyflie 드론+ground+light)·질량
오버라이드(2.173kg)·reset(obs (1,14))·**148스텝 안정 호버**(고도 9.5m 유지, 중력보상, 초기
속도 감쇠)·obs/reward/termination 전부 유한(NaN 0)·**stagnation guard 정상 발동**(zero-action
호버라 타겟 미접근 → 정체 감지, 설계대로). `VERIFY: PASS`.

**실제 실행으로만 잡히는 버그 5종 발견·수정**(순수-math 유닛테스트 범위 밖 — env 배선/API):
1. `root_physx_view`를 `_setup_scene()`에서 접근 → 너무 이름(sim play 전엔 view 없음). 질량/관성
   오버라이드를 `super().__init__()` 이후 `__init__`으로 이동(best-effort + fallback).
2. 액추에이션 API: `set_external_force_and_torque` → v2.3.2 stock quadcopter env는
   `articulation.permanent_wrench_composer.set_forces_and_torques`(BODY 프레임, body +Z 추력,
   `find_bodies("body")`) 사용. 컨트롤러 재작성.
3. **컨트롤러 토크에 관성항 누락**: rate loop의 `k_rate*rate_err`는 목표 *각가속도*지 토크가
   아님. 이 기체 관성 ~0.022 kg·m²에선 ~46× 과토크 → step 0에서 즉시 스핀아웃(overspeed+
   bad_attitude). **수정: `torque = inertia · (k_rate·rate_err)`.** CTRL DBG 덤프로 확정
   (torque 10.28→0.05 N·m). → [[research/isaac_velocity_controller]]에 반영.
4. reset에 `self._robot.reset()` + 로터 조인트 상태 리셋 누락, `env_ids=None`(전체 리셋) 미처리.
5. 적용 wrench에 `nan_to_num` 가드(과도 컨트롤러 상태가 sim 전체를 NaN화 못 하게).
+ gym `entry_point` 문자열 `isaac_lab.drone_bombard...` → `drone_bombard...`(스크립트의 sys.path 정합).

**부수: 이미지 자체 버그 2종**(우리 코드 아님, Dockerfile에 수정 반영): (a) 번들 python
아카이브 전반의 dangling `packaging/_structures.py` 심링크가 pip/torch/isaac import를 깨뜨림,
(b) `isaaclab.sh --install`이 core `isaaclab` 패키지 설치 실패(pkg_resources 없음).

**렌더링(카메라/GUI)은 이 박스에서 불가**: Isaac Sim 5.1.0 RTX 렌더러는 driver ≥580.65.06 필요,
이 박스는 535 → "rtx driver verification failed". 물리/CUDA는 정상(검증이 그 위에서 돔).
시각/GUI 산출물은 L4 Spot VM(driver ≥580) 필요. 사용자 결정(2026-07-03): headless 수치 검증을
증거로 수용, 시각화는 L4 VM 기동 시로 연기.

### 6c. 로컬 순수-math 검증 (변동 없음)

- `python3 -m py_compile` 전체 통과.
- `pytest isaac_lab/tests/test_math.py` — **29/29 통과** (drone-bombard-harmonic 컨테이너,
  torch 2.4.1, isaaclab 미설치 상태로 실행 — 파일 경로 직접 로드로 `drone_bombard/__init__.py`의
  isaaclab import 체인 우회). 커버리지: rate-limit, LPF(이산 스텝응답 y_k=1-0.6^k, 정책스텝
  경계 연속성, per-env 리셋 격리+snap), pinhole 투영(축 부호, 가시성), hold-buffer 카운트다운,
  ballistic/CCIP 훅 항등성, overshoot/stagnation guard, reward 공식 7개 격리 시나리오.

## 7. Vision 캘리브레이션 버전 로그 (신규 필드마다 갱신)

| 버전 | 날짜 | 소스 | 비고 |
|---|---|---|---|
| v0 (초기값) | 2026-07-03 | 스펙 추정 (YOLO conf 실측 범위 0.73-0.95 재사용, 나머지는 추정) | `yolo_eval.py --calibrate` 미실행 — L4 VM에서 range×angle 스윕 후 갱신 필요 |

## 8. 남은 작업

- [x] **Isaac env 실제 실행 검증(무학습 1 에피소드)** — 이 dev 박스에서 `isaac-sim:5.1.0`로
  `VERIFY: PASS`(§6b). 버그 5+2종 수정. 렌더링만 driver 535 제약으로 불가.
- [ ] L4 Spot VM 기동 (`infra/deploy.sh` 빌드+push, `infra/startup.sh` 실행) — GUI/렌더링·본 학습
- [ ] Cartpole → env 스모크 → zero-actions/scripted 물리 검증(L4, 렌더링 포함)
- [ ] PX4 속도-스텝응답 Gazebo 캡처 세션 (7-포인트, `vel_logger_v2.py` 신규 필요) →
  `isaac_velocity_controller.md` 캘리브레이션 결과 채우기
- [ ] `yolo_eval.py --calibrate` 첫 실행 → vision 캘리브레이션 v1
- [ ] 2048-env 처리량 프로브 → 본 학습 시작
- [ ] **(게이트 조건부, 지금 실행 안 함) `ros2_ws/`/`gazebo_models`/PX4 관련 파일 정리** —
  `feat/isaac-env-migration` 브랜치(이 워크트리)에서만, `jekyun`(라이브 SAC 학습 중)은 절대
  손대지 않음. **선행 조건 둘 다 충족 후 진행**: ① L4 VM에서 Isaac env 스모크(Cartpole →
  2-iter → zero-actions/scripted) 통과, ② 위 PX4 스텝응답 캡처 세션 완료(Isaac 컨트롤러
  게인 검정 자료 확보 — Gazebo/PX4가 사라지면 재캡처 불가). 두 조건 충족 전 삭제 시
  검정 불가능한 컨트롤러로 영구 고정되는 리스크. 사용자 요청(2026-07-03)으로 트래킹만
  해두고 보류.
