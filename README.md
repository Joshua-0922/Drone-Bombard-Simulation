# Drone Bombard — Isaac Lab (Isaac Sim) 구현

> **브랜치:** `feat/isaac-env-migration`. 이 브랜치는 기존 **Gazebo + PX4 + ROS2 + SAC**
> 파이프라인을 **NVIDIA Isaac Sim 5.1.0 + Isaac Lab v2.3.2 + rsl_rl PPO**로 이전(migration)한
> 것이다. Gazebo/PX4/ROS2 스택 설명은 이 브랜치의 README에서 제거되었으며, 해당 스택은
> `main`/`jekyun` 등 다른 브랜치에서 계속 유지된다.

---

## 1. 프로젝트 개요

적의 정확한 좌표를 사전에 모르는 상황에서, 드론이 지정 경로를 순항하다 지상 목표물
(**X-marker**)을 하방 카메라로 식별하면 정밀 추적(terminal guidance)으로 전환해 목표 상공에
도달하고 페이로드를 투하하는 자율 비행 시스템이다.

강화학습(RL)이 담당하는 구간은 **종말 유도(terminal guidance)** 한 구간이다: 순항→탐지
핸드오프 시점(목표에서 약 3–7 m, 고도 ~10 m)에서 시작해 목표 상공(수평거리 `d_xy ≤ 0.8 m`)에
도달할 때까지의 시각 서보잉(visual servoing)을 학습한다. 순항·투하 로직은 스크립트(비학습)로
처리한다.

**태스크 스코프(SAC v15와 동일):** 4-D 속도 명령 액션, 성공 = 목표 도달. 투하(drop)는 액션이
아니라 스크립트 CCIP(탄도) 메트릭으로 스코어링한다.

---

## 2. Gazebo/PX4/ROS2 → Isaac Lab 이전 개요

| 항목 | 기존 (Gazebo/PX4/ROS2, SAC) | 현재 (Isaac Lab, PPO) |
|---|---|---|
| 시뮬레이터 | Gazebo Harmonic + PX4 SITL | Isaac Sim 5.1.0 (PhysX, GPU) |
| 프로세스 구조 | 다중 ROS2 노드 (vision/path/drop/controller) + PX4 + MicroXRCE + gz_bridge | **단일 프로세스** `DirectRLEnv` 하나 |
| 병렬화 | `num_envs=1` (단일 Gazebo가 PX4 lockstep 직렬화) | `num_envs=2048` (GPU-vectorized) |
| RL 알고리즘 | SAC (Stable-Baselines3) | **PPO (rsl_rl)** |
| Vision | 실제 카메라 렌더 + YOLOv8 항상 추론 | 학습=analytic pinhole 투영(YOLO 캘리브레이션 노이즈), 평가=실제 YOLOv8 |
| 드론 제어 | PX4 전체 비행 스택(EKF·믹서·로터) | rigid-body wrench + 캐스케이드 속도→자세→토크 컨트롤러(PX4 모방) |
| 리셋 | teleport+disarm / soft reset(EKF 재수렴 회피) | 즉시 텔레포트 (ground-truth state, EKF 개념 없음) |
| 타겟/스폰 | 고정 ENU (11,10) | env마다 랜덤화(신규) |

기존 다패키지 ROS2 시스템의 모든 기능(YOLO 관측, PX4식 속도 명령 + LPF, drop 스코어링)을
**하나의 env 파일**로 통합 이식했다. v13/v15의 관측(14-D)·액션(4-D)·3-layer 보상·종료 조건
상수는 전부 그대로 옮겼다(전체 parity 표: `notes/experiments/exp_012_isaac_migration_phase2.md`).

---

## 3. 코드 구조 (`isaac_lab/`)

```
isaac_lab/
├── drone_bombard/
│   ├── __init__.py            gym.register("Isaac-DroneBombard-Direct-v0")
│   ├── math_utils.py          순수 torch — action rate-limit/LPF, pinhole 투영, hold-buffer,
│   │                          ballistic/CCIP, 3-layer reward, overshoot/stagnation guard.
│   │                          isaaclab 무의존 → GPU/Isaac 없이도 유닛테스트 가능.
│   ├── drone_bombard_env.py   DirectRLEnv — 위 순수 math를 isaaclab lifecycle(씬·액추에이션·
│   │                          관측·종료·보상·리셋)에 연결. 캐스케이드 속도 컨트롤러 + 마커.
│   ├── mdp/domain_rand.py     Phase-2 도메인 랜덤화 스텁 (Phase 1은 항등)
│   └── agents/rsl_rl_ppo_cfg.py   PPO 하이퍼파라미터
├── train.py                   rsl_rl PPO 학습 진입점 (wandb + SIGTERM preempt save)
├── play.py                    sanity 체크 (--zero-actions / --scripted / --step-response)
├── verify_one_episode.py      무학습 1-에피소드 검증 하네스 (+ --with_camera 스크린샷)
├── record_episode.py          RTX 렌더 1-에피소드 영상 녹화 (드론+페이로드+타겟 마커)
├── yolo_eval.py               실제 YOLOv8 평가 + vision 캘리브레이션 (TiledCamera)
├── tests/test_math.py         순수 torch 유닛테스트 (isaaclab 불필요, 30/30)
└── README.md                  실행 절차 상세
```

### 관측 / 액션 / 보상 (v13/v15 이식)

- **관측** `Box(14)`, clip ±1: `[0-2]` pos ENU/50, `[3-5]` vel ENU/15, `[6-8]` body ang_vel/π,
  `[9-10]` YOLO u/v 픽셀 정규화, `[11]` conf, `[12-13]` 목표 상대 오프셋(metric)/50.
- **액션** `Box(4)` ±1: vx·4.0, vy·3.0, vz·3.0 m/s, yaw_rate·1.0 rad/s. per-step rate limit 0.2,
  이후 EMA LPF(α=0.4, 20 Hz tick — `drone_controller` 이식, 학습==배포 플랜트).
- **보상** 3-layer: 시간·각속도·액션 스무스니스 페널티 + 거리 그래디언트 + 근접 보너스 +
  vision centering + 근접-게이팅 속도 댐핑. 성공 +100, 각종 종료 페널티(crash/overspeed/
  bad_attitude/out_of_range/max_altitude/overshoot/stagnation/timeout).

### 드론 액추에이션

Isaac은 rigid body에 직접 wrench(추력+토크)를 가한다. PX4의 로터/믹서 내부 루프는 학습된
정책의 plant에 포함된 적이 없으므로, 단일 rigid body에 **캐스케이드 속도→자세→토크
컨트롤러**(PX4 게인 모방)를 적용한다. 질량/관성은 x500 SDF 실측값(2.07 kg 드론 + 0.1 kg
페이로드, `diag(0.0217,0.0217,0.040)`)으로 오버라이드한다. **주의:** 토크는 `τ = I·(k_rate·
rate_err)`로 관성을 곱해야 한다(Isaac은 직접 토크 → 관성 미곱 시 ~46× 과토크로 스핀아웃).
컨트롤러 게인은 PX4 실측 스텝응답 대비 **아직 미검정**(초기값) — `notes/research/isaac_velocity_controller.md`.

---

## 4. 환경 요구사항

- **GPU 드라이버 ≥ 580.65.06** (Isaac Sim 5.1.0 RTX 렌더러 필수). 이 프로젝트 VM은 driver
  535 → 580.159.03으로 업그레이드해 GUI/렌더링을 활성화했다. driver 535에서도 **헤드리스
  물리/학습은 동작**하나(CUDA), 카메라/GUI 렌더링은 불가.
- **Isaac Sim 5.1.0** 도커 이미지(`nvcr.io/nvidia/isaac-sim:5.1.0`) + **Isaac Lab v2.3.2** +
  **rsl_rl**. 빌드: `drone_drop_system/docker/Dockerfile`.

> **Dockerfile 주의(이미지 자체 버그 대응, 우리 코드 아님):** isaac-sim:5.1.0 이미지는
> (a) 번들 python 아카이브 전반에 dangling `packaging/_structures.py` 심링크가 있어
> pip/torch/isaac import를 깨뜨리고, (b) `isaaclab.sh --install`이 core `isaaclab` 패키지 설치에
> 실패한다(pkg_resources 없음). Dockerfile에 두 버그의 수정이 포함되어 있다.

---

## 5. 실행 방법

컨테이너(Isaac Sim 5.1.0 + Isaac Lab v2.3.2 + rsl_rl) 안에서, 이 저장소를
`/workspace/drone-bombard`로 마운트하고 `PYTHONPATH`에 `isaac_lab/`을 추가한다.

```bash
# 0. (헤드리스, isaaclab 불필요) 순수 로직 유닛테스트
pytest isaac_lab/tests/test_math.py -v          # 30/30

# 1. 무학습 1-에피소드 검증 (env 구성 → reset → 1 에피소드)
./isaaclab.sh -p isaac_lab/verify_one_episode.py --headless --enable_cameras --num_steps 300

# 2. PPO 학습 (헤드리스, wandb)
./isaaclab.sh -p isaac_lab/train.py --task Isaac-DroneBombard-Direct-v0 \
    --headless --num_envs 2048
#   dry-run: --num_envs 256 --max_iterations 20 --run_name dryrun_256

# 3. RTX 1-에피소드 영상 녹화 (드론 + 페이로드 + 타겟 마커; driver ≥580 필요)
./isaaclab.sh -p isaac_lab/record_episode.py --headless --enable_cameras \
    --video_length 300 --out /workspace/logs/isaac_lab/videos

# 4. 컨트롤러 물리 sanity + PX4 대비 스텝응답
./isaaclab.sh -p isaac_lab/play.py --zero-actions
./isaaclab.sh -p isaac_lab/play.py --scripted
./isaaclab.sh -p isaac_lab/play.py --step-response --out-csv .../step_response.csv

# 5. 실제 YOLOv8 평가 + vision 캘리브레이션 (TiledCamera, num_envs ≤ 8)
./isaaclab.sh -p isaac_lab/yolo_eval.py --calibrate --num_envs 8 --headless
./isaaclab.sh -p isaac_lab/yolo_eval.py --eval --policy .../model_final.pt --num_envs 8 --headless
```

상세 절차·주의사항은 `isaac_lab/README.md` 참조.

---

## 6. 검증 현황 (2026-07-03, 라이브)

- **유닛테스트:** `pytest tests/test_math.py` — **30/30** (isaaclab 미설치 상태, 순수 torch).
- **무학습 1-에피소드:** `VERIFY: PASS` — env 구성·USD 씬·질량 오버라이드·reset(obs (1,14))·
  148스텝 안정 호버·NaN 0·stagnation guard 정상.
- **PPO 학습 dry-run(256 envs, 20 iters):** 정상 학습 — reward **−75 → +29**, ep_len **2 → ~100**,
  `d_xy_min 0.747`(일부 드론이 이미 0.8 m 성공반경 도달). wandb 프로젝트 `drone-bombard-isaac`.
- **RTX 영상 녹화:** driver 580 업그레이드 후 드론+페이로드+타겟 1-에피소드 녹화(§5-3).

상세: `notes/experiments/exp_012_isaac_migration_phase2.md`,
`notes/research/isaac_lab_architecture.md`, `notes/research/isaac_velocity_controller.md`.

---

## 7. 관련 문서

- `isaac_lab/README.md` — Isaac Lab 코드 실행 상세
- `notes/research/isaac_lab_architecture.md` — 폴더 구조·데이터 흐름·Gazebo 대비 차이
- `notes/experiments/exp_012_isaac_migration_phase2.md` — 이전 작업·parity 표·검증 결과
- `notes/research/isaac_velocity_controller.md` — 속도 컨트롤러·PX4 게인 매핑·검정 상태
- `notes/research/rl_rules.md` Rule 16 — 시뮬레이터 이전 시 plant/reward parity 원칙
- `drone_drop_system/docker/Dockerfile` — Isaac Sim + Isaac Lab + rsl_rl 이미지
- `infra/deploy.sh` / `infra/startup.sh` — GCP L4 Spot VM 빌드·기동
