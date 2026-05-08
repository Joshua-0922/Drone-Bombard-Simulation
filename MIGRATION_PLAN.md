# Gazebo → Isaac Sim + Isaac Lab 단계적 마이그레이션 계획

## Context

현재 프로젝트는 Gazebo Harmonic + PX4 SITL + ROS2 기반으로, RL 학습 속도가 매우 느림 (~1 episode/min). Isaac Sim + Isaac Lab으로 전환하면 GPU 병렬화(수천 환경)로 학습 속도를 1000배 이상 향상 가능. Phase 1에서 RL 학습 환경을 먼저 전환하고, Phase 2에서 나머지 컴포넌트를 순차 전환.

## Phase 1: RL 학습 환경 전환 (이번 작업)

### 핵심 설계 결정

| 항목 | 결정 | 이유 |
|------|------|------|
| **드론 에셋** | Isaac Lab Multirotor 기반, x500 물리 파라미터 적용 | SDF→USD 변환 시 Gazebo 플러그인 손실. Isaac Lab 내장 모터 모델 활용이 효율적 |
| **액션 공간** | 기존 5D velocity command 유지 + PD 컨트롤러 | PX4 velocity controller 대체. 기존 policy와 의미 호환 |
| **RL 알고리즘** | SB3 SAC (1차) → skrl/rsl_rl (2차) | 기존 파이프라인 검증 후 GPU 네이티브로 전환 |
| **페이로드** | PhysicsFixedJoint 런타임 enable/disable | Gazebo DetachableJoint 대체. 텐서 배치 연산 가능 |
| **PX4/ROS2** | 제거 | Isaac Lab 네이티브 Python API 사용. subprocess 관리 불필요 |

### 디렉토리 구조

```
Drone-Bombard-Simulation/
  isaac_lab_tasks/                        # NEW
    setup.py
    drone_bombard/
      __init__.py
      __init__env__.py                    # Gymnasium 등록
      config/
        drone_drop_env_cfg.py             # ManagerBasedRLEnvCfg
        agents/
          sac_cfg.py                      # SAC 하이퍼파라미터
          ppo_cfg.py                      # PPO 하이퍼파라미터 (옵션)
      mdp/
        observations.py                   # 17D 관측 텐서 함수
        actions.py                        # PD velocity controller 액션 텀
        rewards.py                        # 4-layer 보상 함수
        terminations.py                   # 종료 조건
        events.py                         # 리셋/랜덤화/커리큘럼
      assets/
        drone.py                          # ArticulationCfg (x500 파라미터)
        payload.py                        # RigidObjectCfg (0.1kg 실린더)
        target_marker.py                  # StaticObjectCfg (1.5m X-marker)
      scripts/
        train.py                          # 학습 진입점
        evaluate.py                       # 평가
        play.py                           # 시각화
  docker/
    Dockerfile.isaac                      # Isaac Sim 컨테이너
```

### 구현 순서

#### Step 1: Docker 환경 (신규 파일 1개)

`docker/Dockerfile.isaac` — `nvcr.io/nvidia/isaac-sim:4.2.0` 기반, Isaac Lab + SB3 + WandB 설치

#### Step 2: 에셋 정의 (신규 파일 3개)

- `assets/drone.py` — x500 물리 파라미터로 ArticulationCfg 정의
  - mass: 2.0kg, arm: 0.25m, motor max thrust: ~8.5N
  - 초기 상태: altitude 5.0m (TAKEOFF skip 대체)
- `assets/payload.py` — RigidObjectCfg (cylinder r=0.03m, h=0.005m, mass=0.1kg)
- `assets/target_marker.py` — 지면 X-marker (1.5m x 1.5m)

#### Step 3: 환경 설정 (신규 파일 1개)

`config/drone_drop_env_cfg.py`:
- `ManagerBasedRLEnvCfg` 서브클래스
- Scene: drone + payload + target + ground_plane + fixed_joint
- sim dt=0.01 (100Hz), decimation=10 (→10Hz 제어, 현재와 동일)
- episode_length_s=50.0 (500 steps @ 10Hz)
- num_envs=4096 (Gazebo 4개 대비 1000배)

#### Step 4: MDP 컴포넌트 (신규 파일 5개) — 핵심 로직

**`mdp/actions.py`** — PD velocity controller:
- 5D action → velocity command 디코딩 (vx*15, vy*5, vz*3, yaw*1)
- PD 컨트롤러: `F = Kp*(v_des - v_cur) + Kd*a` → `set_external_force_and_torque()`
- PD 게인 초기값: Kp_xy=3.0, Kd_xy=1.0, Kp_z=5.0, Kd_z=2.0

**`mdp/observations.py`** — 17D 관측 텐서:

| Index | Content | Isaac Lab Source |
|-------|---------|-----------------|
| 0-2 | Position ENU /50 | `drone.data.root_pos_w` |
| 3-5 | Velocity ENU /15 | `drone.data.root_lin_vel_w` |
| 6-8 | Angular vel /pi | `drone.data.root_ang_vel_w` |
| 9-11 | u, v, confidence | 0, 0, 1.0 (use_vision=false) |
| 12 | Payload attached | 자체 bool 텐서 |
| 13-14 | Relative x, y /50 | drone_pos - target_pos |
| 15 | CCIP d_impact /50 | 벡터화 kinematic predictor |
| 16 | CCIP t_f /10 | 동일 predictor |

**`mdp/rewards.py`** — 4-layer 보상:
- Layer 1 Safety: crash(-10, alt<2m after step 20), overspeed(-8, speed>20)
- Layer 2 Stability: -0.05 - 0.05*||omega||^2 - 0.05*||delta_a||^2
- Layer 3 Approach: 1.0*(d_prev - d_xy) + 1.0*cos(heading)*speed_gate
- Layer 4 Drop: 50*exp(-5*d_error) + 100 jackpot(d<=0.1m)
- `d_xy_prev` 텐서를 환경 버퍼로 유지

**`mdp/terminations.py`**:
- timeout (500 steps)
- payload_landed (payload z <= 0.04m && dropped)
- physics_glitch (d_xy > 500m or !isfinite)

**`mdp/events.py`**:
- drone/payload 포즈 리셋 (텐서 write)
- fixed joint 재활성화
- target 위치 랜덤화 (커리큘럼 스테이지별 거리)
- 커리큘럼 4단계: 3-8m → 8-20m → 20-50m → 20-50m+vision masking

#### Step 5: 페이로드 드롭 메커니즘

1. Scene 생성 시 drone<->payload 간 `PhysicsFixedJoint` 생성
2. Auto-drop: CCIP d_impact <= 0.5m일 때 joint disable (텐서 배치)
3. 매 step payload z 체크 → z <= 0.04m이면 miss distance 계산
4. 리셋 시 joint 재활성화 + payload 텔레포트
- **Gazebo drop_calculator 노드 대체**: 인라인 텐서 연산으로 처리

#### Step 6: 학습 파이프라인 (신규 파일 3개)

**`scripts/train.py`**:
- Isaac Lab `Sb3VecEnvWrapper`로 SB3 SAC 연동
- 기존 hyperparams.yaml 설정 → config dataclass로 이전
- WandB 콜백 (기존 WandbMetricsCallback 적응)
- 체크포인트: SIGTERM 핸들러 + 롤링 체크포인트

**`config/agents/sac_cfg.py`**: lr=3e-4, buffer=100K, batch=256, tau=0.005, gamma=0.99, net=[256,256]

#### Step 7: Docker 컨테이너 설정

```dockerfile
FROM nvcr.io/nvidia/isaac-sim:4.2.0
# Isaac Lab 설치
RUN pip install isaac-lab
# 추가 의존성
RUN pip install stable-baselines3[extra] wandb gymnasium skrl
```

### 기존 코드 → Isaac Lab 매핑

| 기존 (Gazebo) | Isaac Lab 대체 |
|--------------|---------------|
| `drone_drop_env.py` (1333줄) | `config/` + `mdp/` (모듈 분리) |
| `_RLBridgeNode` (ROS2 sub) | `scene["drone"].data` 텐서 직접 접근 |
| `subprocess.Popen` (PX4/Gazebo) | 불필요 — 전부 in-process |
| `_gz_reset_poses()` | `write_root_pose_to_sim()` |
| `DetachableJoint` | `PhysicsFixedJoint` enable/disable |
| `drop_calculator_node.py` | termination 텀에서 인라인 계산 |
| `mission_manager FSM` | 제거 — RL env가 직접 제어 |
| `drone_controller` | PD velocity controller (actions.py) |
| `ros_gz_bridge` | 불필요 |

### 검증 계획

1. **Smoke Test**: 단일 환경에서 드론 호버링, 페이로드 탈착, 지면 충돌 감지 확인
2. **관측 패리티**: 동일 상태에서 17D 관측값이 Gazebo와 일치하는지 비교
3. **보상 패리티**: 고정 궤적에서 step별 보상이 Gazebo와 동일한 범위인지 확인
4. **학습 검증**: 10K steps SB3 SAC → ep_rew_mean 상승, WandB 메트릭 정상 확인
5. **성능 벤치마크**: num_envs=[1, 64, 256, 1024, 4096]에서 FPS 측정

### 리스크 및 대응

| 리스크 | 대응 |
|--------|------|
| PD 컨트롤러가 PX4 비행 특성과 다름 | Gazebo 궤적 대비 게인 튜닝; 필요시 PID 추가 |
| PhysX joint 배치 disable 불가 | Fallback: 수동 constraint force로 "attached" 시뮬레이션 |
| GPU 드라이버 호환성 | Isaac Sim 4.2는 driver >= 535 필요; 사전 확인 |
| PhysX vs ODE 물리 차이 | PhysX contact 파라미터 튜닝; 자유낙하 궤적 비교 |
| L4 24GB OOM (4096 envs) | 1024 envs부터 시작; buffer_size 조절; skrl SAC 검토 |

---

## Phase 2 (향후): 나머지 컴포넌트 전환

- Vision pipeline: Isaac Sim GPU 렌더링 + YOLO → 커리큘럼 Stage 3
- 실기체 배포: Isaac Sim에서 학습한 policy를 PX4 실기체에 배포하는 별도 파이프라인
- Gazebo 코드 보존: ROS2 패키지는 삭제하지 않고 유지 (fallback)

---

## 다른 PC에서 작업하기

이 파일을 참조하여 다른 PC에서 구현을 시작할 수 있습니다:

```bash
git clone https://github.com/Joshua-0922/Drone-Bombard-Simulation.git
cd Drone-Bombard-Simulation
# Claude Code 실행 후:
# "MIGRATION_PLAN.md를 따라서 구현해줘"
```

### 다른 PC 요구사항
- NVIDIA GPU (RTX 3070+ 또는 L4/A10G 이상 권장)
- NVIDIA Driver >= 535
- Docker + NVIDIA Container Toolkit
- 최소 16GB GPU VRAM (4096 envs 시 24GB+ 권장)
