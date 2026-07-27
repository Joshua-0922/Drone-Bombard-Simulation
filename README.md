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

## 5. 컨테이너 시작 및 진입 방법

> **⚠️ 이 브랜치의 코드는 별도 git worktree에 있다.** `feat/isaac-env-migration`은
> `/opt/drone-bombard/isaac-worktree`에 **별도 worktree**로 체크아웃되어 있다
> (`git worktree add /opt/drone-bombard/isaac-worktree feat/isaac-env-migration`로 생성 —
> `notes/daily/daily_2026-07-03.md` 참조). `/opt/drone-bombard/Drone-Bombard-Simulation`은
> **다른 worktree**(`jekyun` 브랜치, 라이브 SAC 학습 중)이며 **`isaac_lab/` 코드가 없다**.
> 로컬(dev VM)에서 컨테이너를 띄울 때는 반드시 `/opt/drone-bombard/isaac-worktree`를
> 마운트해야 한다 — `Drone-Bombard-Simulation`을 마운트하면 컨테이너 안에
> `isaac_lab/`이 아예 보이지 않는다.

### 가장 빠른 방법 — 이미 떠 있는 컨테이너 재사용 (이 dev VM, 권장)

> **⚠️ `drone-bombard-isaac:latest`는 어디에도 존재하지 않는다** (로컬에도, GCP Artifact
> Registry `isaac-lab` 저장소에도 — pull 대상이 아예 없다). 아래 "컨테이너 빌드" 절의
> `docker build -t drone-bombard-isaac:latest .`로 **직접 빌드**하거나, 이 dev VM에서는
> 보통 아래처럼 **이미 실행 중인 컨테이너를 재사용**하면 된다(빌드 20–30분 불필요).

```bash
# 1. 실행 중인 Isaac 컨테이너 확인 (보통 isaac-verify, 이미지 isaac-lab-local:580)
docker ps --filter ancestor=isaac-lab-local:580

# 2. 있으면 exec로 바로 진입 — docker run 불필요
docker exec -it -w /workspace/drone-bombard isaac-verify bash
```

**주의 (non-root exec):** `isaac-verify`는 기본적으로 root가 아닌 `isaac-sim`(uid 1234)
사용자로 exec된다. 과거에 `docker exec -u root`로 들어가 만들어진 root 소유 캐시/로그
파일이 `/isaac-sim/kit/cache`, `/isaac-sim/kit/data`, `/root/.cache/ov`,
`/root/.nv/ComputeCache`, `/tmp/isaaclab/logs` 등에 남아있으면 `isaac-sim` 사용자로 실행 시
`PermissionError`로 **바로 크래시**한다(경고가 아님 — 로그 파일 자체를 못 열면 스크립트가
죽는다). 발생 시:
```bash
docker exec -u root isaac-verify chown -R isaac-sim:isaac-sim \
  /isaac-sim/kit/cache /isaac-sim/kit/data /tmp/isaaclab/logs \
  /root/.cache/ov /root/.cache/nvidia/GLCache /root/.nv/ComputeCache
```

**출력을 파일/로그로 리다이렉트할 때는 `PYTHONUNBUFFERED=1`을 함께 넘긴다** —
`simulation_app.close()`가 프로세스를 하드 종료하는 것으로 보여, unbuffered가 아니면 마지막
`print()`(PASS/FAIL 결과 줄 포함)가 버퍼에 남은 채 유실될 수 있다:
```bash
docker exec -e PYTHONUNBUFFERED=1 -w /workspace/drone-bombard isaac-verify \
  bash -lc '/workspace/isaaclab/isaaclab.sh -p isaac_lab/play.py --zero-actions --headless'
```

컨테이너가 떠 있지 않다면 아래 "컨테이너 빌드" → "컨테이너 진입" 절차를 따른다.

### 컨테이너 빌드

```bash
# Dockerfile을 사용해 Isaac Sim 5.1.0 + Isaac Lab v2.3.2 + rsl_rl 이미지 빌드
# (첫 빌드 시 ~20–30분 소요). GCP Artifact Registry에는 이 이미지가 push된 적이
# 없으므로 pull로 대체할 수 없다 — 반드시 로컬 빌드가 필요하다.
cd /opt/drone-bombard/isaac-worktree/drone_drop_system/docker
docker build -t drone-bombard-isaac:latest .
```

### 컨테이너 진입 (대화형 bash) — 이 dev VM

**`isaac-worktree`를 마운트한다 (`Drone-Bombard-Simulation`이 아님).** GPU
셰이더/컴파일 캐시를 호스트에 마운트해두면 재시작 시 Isaac Sim 첫 구동이 훨씬 빨라진다
(`isaac-verify`가 쓰는 방식):
```bash
docker run -it --rm \
  --gpus all \
  --runtime=nvidia \
  -e ACCEPT_EULA=Y \
  -e PRIVACY_CONSENT=Y \
  -v /opt/drone-bombard/isaac-worktree:/workspace/drone-bombard \
  -v /opt/drone-bombard/isaacsim-cache/kit:/isaac-sim/kit/cache \
  -v /opt/drone-bombard/isaacsim-cache/ov:/root/.cache/ov \
  -v /opt/drone-bombard/isaacsim-cache/glcache:/root/.cache/nvidia/GLCache \
  -v /opt/drone-bombard/isaacsim-cache/computecache:/root/.nv/ComputeCache \
  drone-bombard-isaac:latest \
  bash

# 컨테이너 내부:
cd /workspace/drone-bombard
```

### 컨테이너 진입 — GCP L4 Spot VM (원격 학습 서버)

L4 Spot VM은 **이 dev VM과 별개의 머신**이며, `infra/startup.sh`가 그 VM 위에서
저장소를 `/opt/drone-bombard/Drone-Bombard-Simulation`(그 VM 기준 단일 체크아웃 경로,
worktree 분리 없음)으로 마운트한다. 즉 위 dev VM 표의 `Drone-Bombard-Simulation` 문제는
**dev VM에서 마운트를 혼동할 때만** 발생하며, L4 VM의 `startup.sh` 자체는 정상 동작한다
(`infra/startup.sh` 참조). L4 VM은 `infra/deploy.sh`로 기동되며 컨테이너 진입은 자동
(startup script)이므로 수동 `docker run`이 보통 불필요하다 — 디버깅 시:
```bash
# L4 VM에 SSH 접속 후
docker exec -it drone-bombard-isaac bash
```

### Isaac Sim GUI 열기

**조건:**
- GPU 드라이버 **≥ 580.65.06** 필수 (RTX 렌더러)
- 디스플레이가 있는 머신 (X11 또는 Wayland)
- 원격 연결 시 X11 포워딩 설정 필요

**GUI 시작 (컨테이너 내부에서):**
```bash
# 방법 1: isaaclab.sh 래퍼 사용 (권장)
# --headless 플래그 없이 실행하면 GUI 활성화
/workspace/isaaclab/isaaclab.sh -p isaac_lab/play.py --zero-actions

# 방법 2: 직접 python.sh 사용
/isaac-sim/python.sh isaac_lab/play.py --zero-actions
```

**원격 머신에서 GUI 접속 (SSH X11 포워딩):**
```bash
# 로컬 터미널에서:
ssh -X user@remote-host

# 원격 호스트에서:
docker run -it --rm \
  --gpus all \
  --runtime=nvidia \
  -e DISPLAY=$DISPLAY \
  -e ACCEPT_EULA=Y \
  -e PRIVACY_CONSENT=Y \
  -v /tmp/.X11-unix:/tmp/.X11-unix:rw \
  -v /opt/drone-bombard/isaac-worktree:/workspace/drone-bombard \
  drone-bombard-isaac:latest \
  bash

# 컨테이너 내부:
cd /workspace/drone-bombard
/workspace/isaaclab/isaaclab.sh -p isaac_lab/play.py --zero-actions
```

### 어떤 Isaac Sim을 열어야 하는가?

컨테이너 내부의 Isaac Sim은 **기본적으로 헤드리스 모드**(CPU 렌더링, GUI 없음)로 설정되어 있다.
GUI를 활성화하려면:

1. **학습 중 모니터링이 필요한 경우:**
   - `/workspace/isaaclab/isaaclab.sh -p isaac_lab/play.py` — sanity check 모드 (작은 규모, 빠름)
   - `/workspace/isaaclab/isaaclab.sh -p isaac_lab/record_episode.py` — 영상 녹화 (RTX 렌더링 필요)

2. **headless 모드가 필요한 경우 (권장):**
   - 전체 학습: `/workspace/isaaclab/isaaclab.sh -p isaac_lab/train.py --headless --num_envs 2048`
   - 무학습 검증: `/workspace/isaaclab/isaaclab.sh -p isaac_lab/verify_one_episode.py --headless`
   - 성능이 중요하므로 `--headless` 권장

3. **드라이버 업그레이드 전 현재 상태:**
   - 현재 드라이버 535에서는 **headless 학습만 가능** (CUDA 연산은 OK, RTX 렌더링은 불가)
   - GPU 드라이버를 580 이상으로 업그레이드하면 GUI/렌더링 기능 활성화

---

### 실행 명령어 (headless 모드, GPU 학습 최적화)

컨테이너 내부에서 아래 명령을 실행한다. `--headless` 플래그는 성능상 **권장**이다.

```bash
# 0. (헤드리스, isaaclab 불필요) 순수 로직 유닛테스트
pytest isaac_lab/tests/test_math.py -v          # 30/30

# 1. 무학습 1-에피소드 검증 (env 구성 → reset → 1 에피소드)
/workspace/isaaclab/isaaclab.sh -p isaac_lab/verify_one_episode.py --headless --enable_cameras --num_steps 300

# 2. PPO 학습 (헤드리스, wandb)
/workspace/isaaclab/isaaclab.sh -p isaac_lab/train.py --task Isaac-DroneBombard-Direct-v0 \
    --headless --num_envs 2048
#   dry-run: --num_envs 256 --max_iterations 20 --run_name dryrun_256

# 3. RTX 1-에피소드 영상 녹화 (드론 + 페이로드 + 타겟 마커; driver ≥580 필요)
/workspace/isaaclab/isaaclab.sh -p isaac_lab/record_episode.py --headless --enable_cameras \
    --video_length 300 --out /workspace/logs/isaac_lab/videos

# 4. 컨트롤러 물리 sanity + PX4 대비 스텝응답
/workspace/isaaclab/isaaclab.sh -p isaac_lab/play.py --zero-actions --headless
/workspace/isaaclab/isaaclab.sh -p isaac_lab/play.py --scripted --headless
/workspace/isaaclab/isaaclab.sh -p isaac_lab/play.py --step-response --headless --out-csv .../step_response.csv

# 5. 실제 YOLOv8 평가 + vision 캘리브레이션 (TiledCamera, num_envs ≤ 8)
/workspace/isaaclab/isaaclab.sh -p isaac_lab/yolo_eval.py --calibrate --num_envs 8 --headless
/workspace/isaaclab/isaaclab.sh -p isaac_lab/yolo_eval.py --eval --policy .../model_final.pt --num_envs 8 --headless
```

상세 절차·주의사항은 `isaac_lab/README.md` 참조.

---

## 7. 검증 현황 (2026-07-03, 라이브)

- **유닛테스트:** `pytest tests/test_math.py` — **30/30** (isaaclab 미설치 상태, 순수 torch).
- **무학습 1-에피소드:** `VERIFY: PASS` — env 구성·USD 씬·질량 오버라이드·reset(obs (1,14))·
  148스텝 안정 호버·NaN 0·stagnation guard 정상.
- **PPO 학습 dry-run(256 envs, 20 iters):** 정상 학습 — reward **−75 → +29**, ep_len **2 → ~100**,
  `d_xy_min 0.747`(일부 드론이 이미 0.8 m 성공반경 도달). wandb 프로젝트 `drone-bombard-isaac`.
- **RTX 영상 녹화:** driver 580 업그레이드 후 드론+페이로드+타겟 1-에피소드 녹화(§5-3).

상세: `notes/experiments/exp_012_isaac_migration_phase2.md`,
`notes/research/isaac_lab_architecture.md`, `notes/research/isaac_velocity_controller.md`.

---

## 8. 트러블슈팅 (컨테이너 진입 관련)

| 문제 | 원인 | 해결 방법 |
|------|------|---------|
| `docker: command not found` | Docker 미설치 또는 PATH 미설정 | `apt install docker.io` 또는 `snap install docker` |
| `permission denied while trying to connect to Docker` | 사용자가 docker 그룹에 없음 | `sudo usermod -aG docker $USER` 및 재로그인 |
| 컨테이너 빌드 실패 (`pip install` 에러) | 네트워크 타임아웃 또는 PyPI 미러 문제 | 빌드 재시도 또는 `--build-arg REGISTRY=...` 사용 |
| `rtx driver verification failed` (GUI 실행 시) | GPU 드라이버 < 580.65.06 | 드라이버 업그레이드 필수 (headless 모드 사용 권장) |
| `DISPLAY not set` (X11 포워딩) | SSH 연결에서 `-X` 플래그 미사용 | `ssh -X user@host` 재연결 |
| 컨테이너 내부 `/workspace` 폴더 비어있음 | 볼륨 마운트 경로 오류 | `-v` 플래그 경로 확인: `docker run ... -v /정확한/경로:/workspace/drone-bombard ...` |

---

## 9. 관련 문서

- [`checkpoints/v19/WARMSTART.md`](checkpoints/v19/WARMSTART.md) — **v19 학습 결과 이어받기(warm-start/resume) 방법 + 공유 체크포인트**
- `isaac_lab/README.md` — Isaac Lab 코드 실행 상세
- `notes/research/isaac_lab_architecture.md` — 폴더 구조·데이터 흐름·Gazebo 대비 차이
- `notes/experiments/exp_012_isaac_migration_phase2.md` — 이전 작업·parity 표·검증 결과
- `notes/research/isaac_velocity_controller.md` — 속도 컨트롤러·PX4 게인 매핑·검정 상태
- `notes/research/rl_rules.md` Rule 16 — 시뮬레이터 이전 시 plant/reward parity 원칙
- `drone_drop_system/docker/Dockerfile` — Isaac Sim + Isaac Lab + rsl_rl 이미지
- `infra/deploy.sh` / `infra/startup.sh` — GCP L4 Spot VM 빌드·기동

---

## 빠른 시작 (Quick Start)

### 로컬 개발 (헤드리스 학습, 이 dev VM)
```bash
# 1. 컨테이너가 이미 떠 있는지 먼저 확인 — 있으면 바로 exec (§5 "가장 빠른 방법" 참조)
docker ps --filter ancestor=isaac-lab-local:580
docker exec -it -w /workspace/drone-bombard isaac-verify bash

# 없으면 이미지 빌드(20-30분, pull 불가 — §5 참조) 후 컨테이너 진입
cd /opt/drone-bombard/isaac-worktree/drone_drop_system/docker
docker build -t drone-bombard-isaac:latest .
docker run -it --rm --gpus all --runtime=nvidia \
  -e ACCEPT_EULA=Y -e PRIVACY_CONSENT=Y \
  -v /opt/drone-bombard/isaac-worktree:/workspace/drone-bombard \
  drone-bombard-isaac:latest bash

# 2. 학습 시작 (컨테이너 내부)
cd /workspace/drone-bombard
/workspace/isaaclab/isaaclab.sh -p isaac_lab/train.py --headless --num_envs 256 --max_iterations 20
```

### Warm-start (v19 학습 결과 이어받기)
공유된 v19 체크포인트에서 이어서 학습할 수 있다. 상세·주의사항: [`checkpoints/v19/WARMSTART.md`](checkpoints/v19/WARMSTART.md)
```bash
# 이어학습(옵티마이저까지) — precise 예시
/workspace/isaaclab/isaaclab.sh -p isaac_lab/train.py \
  --task Isaac-DroneBombard-V19-Direct-v0 \
  --resume ./checkpoints/v19/precise/model_final.pt
```

### GUI 모니터링 (드라이버 ≥ 580 필요)
```bash
# 컨테이너 진입 후:
/workspace/isaaclab/isaaclab.sh -p isaac_lab/play.py --zero-actions
# (디스플레이가 있는 머신에서만 작동)
