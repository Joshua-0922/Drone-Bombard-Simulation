# Restore Manifest — 2026-05-21 백업 기준 [DEPRECATED]

> ⚠ **DEPRECATED (2026-05-23)**: 이 manifest 는 5/21 시점 junsang branch 의 변경 inventory 였음. 그 후 5/22 branch 교체 (junsang→jekyun_v2) + 5/23 Tier 1 처방으로 시스템 변경됨.
> **현재 대체**: `local_dronebombard_simulation/archive/change_inventory_pre_pull_2026-05-22.md` (5/22 시점 inventory) + `design_review_2026-05-23.md` (현재 시스템)
> 보존 이유: 5/21 시점 변경 history 의 historical reference
>
> ---
>
> 출처: `/home/juns/conversation_backup_2026-05-21.txt` (10,475줄)
> 대상: drone-bombard-harmonic Docker 컨테이너 + 호스트 bind-mount
> 호스트 bind mount 매핑:
> - `/home/juns/Drone-Bombard-Simulation/gazebo_models` ↔ `/workspace/gazebo_models`
> - `/home/juns/Drone-Bombard-Simulation/ros2_ws` ↔ `/workspace/ros2_ws`

## 중요 환경 사실 (재적용 전 반드시 확인)

1. **호스트 git 저장소는 컨테이너 `/workspace/...`에 bind-mount** 되어 있다. 그래서 컨테이너에서 한 변경은 곧 호스트 git working tree 변경이다. 사용자는 `git push` 금지가 영구 원칙이다.
2. **ROS2 install/share 캐시 함정**: `ros2 launch`/`ros2 run`은 `src/`가 아니라 `install/`을 읽는다. src 수정 후 반드시 둘 중 하나:
   - `colcon build --packages-select <pkg>` (정석), 또는
   - `cp` 또는 `docker cp`로 `install/share` 또는 `install/lib/python3.10/site-packages` 쪽에 동일 파일 미러링.
3. **PX4 airframe ROMFS/rootfs 구조**: `ROMFS/px4fmu_common/init.d-posix/airframes/`에 정의된 airframe 중 `4015_gz_x500_bombard`만 기본 빌드 시 `build/px4_sitl_default/rootfs/etc/init.d-posix/airframes/`로 들어가고, `4016_~4019_gz_x500_bombard_r0~r3`는 누락된다. PX4 SITL이 런타임에 보는 곳은 rootfs이므로 multi-instance 학습 전에 ROMFS→rootfs cp 또는 `make px4_sitl_default` 풀빌드 필요.
4. **`git pull`로 코드 받은 직후** 본 manifest의 변경분이 사라진다. 다시 적용해야 한다.

---

## 변경된 파일 목록 (적용 순서 권장)

| # | 파일 | 상태 | 요약 |
|---|---|---|---|
| 1 | `/workspace/gazebo_models/worlds/x_marker_world_vision.sdf` | APPLIED (신규 생성) | 카메라용 vision 변형 월드. `gz::sim::systems::Sensors`(ogre2) 플러그인 포함 |
| 2 | `/workspace/gazebo_models/worlds/x_marker_world.sdf` | APPLIED (주석만) | base RL용. line 58-66 주석을 "비전 변형 분리" 안내로 갱신 |
| 3 | `/workspace/ros2_ws/src/mission_manager/launch/drone_mission.launch.py` | APPLIED | gz sim 명령 경로 3곳을 vision 변형으로. PX4_GZ_WORLD 환경변수는 그대로 |
| 4 | `/workspace/ros2_ws/src/rl_navigation/config/hyperparams.yaml` | APPLIED | H1 reward scale 축소 / H3 gamma 0.995 / M1 buffer 500k / M2 gradient_steps 4 / L6 eval_freq 추가 / use_vision false / wandb entity 팀으로 / total_timesteps 200000 |
| 5 | `/workspace/ros2_ws/src/rl_navigation/rl_navigation/drone_drop_env.py` | APPLIED | H2: action space 5→4-d. action[4] 흔적 정리 |
| 6 | `/workspace/ros2_ws/src/rl_navigation/rl_navigation/train_sac.py` | APPLIED | L6: hyperparams의 eval_freq를 `BestModelCallback`에 연결 + docstring 보강 |
| 7 | `/opt/PX4-Autopilot/build/px4_sitl_default/rootfs/etc/init.d-posix/airframes/4016_~4019_gz_x500_bombard_r0~r3` | APPLIED (cp from ROMFS) | PX4 multi-instance airframe (총 4개 파일). 컨테이너 재생성/PX4 재빌드 시 사라짐 |
| 8 | `/home/juns/wandb_incremental_sync.sh` | APPLIED (호스트, 신규) | WandB offline 10분 주기 incremental sync 스크립트 (3.2KB, 실행권한) |
| 9 | `/workspace/ros2_ws/src/rl_navigation/rl_navigation/drone_drop_env.py` (line 1131) | REVERTED | PX4 stderr → `/tmp/px4.log` 임시 진단 패치. 원복 완료 (`os.devnull`로 되돌림) |

추가로 install/ 디렉토리에 위 src 변경을 미러링해야 한다 (자세한 명령은 각 항목 검증 단계 참고).

---

## 파일별 상세

### 1. `/workspace/gazebo_models/worlds/x_marker_world_vision.sdf` [APPLIED — 신규 생성]

**이유**: 베이스 월드 `x_marker_world.sdf`에는 `gz::sim::systems::Sensors` 플러그인이 의도적으로 빠져 있어서 카메라 렌더링이 실제로 일어나지 않음 (gz 토픽은 등록되지만 데이터 없음). 비전 미션에서는 카메라 필요, RL 학습에서는 GPU 오버헤드/ODE 크래시 위험. 두 경로를 별도 SDF로 분리.

**작업 방법**:
1. `cp /workspace/gazebo_models/worlds/x_marker_world.sdf /workspace/gazebo_models/worlds/x_marker_world_vision.sdf`
2. 새 파일의 헤더 주석을 vision 변형 안내로 교체 (아래 After 박스)
3. **Contact 플러그인 다음, Imu 플러그인 직전 (대략 line 80 부근)** 에 `Sensors` 플러그인 1개 추가

**변경 위치 (헤더 주석)**:
- Before: 베이스와 동일 (line 3 부근: `x_marker_world.sdf - Gazebo Harmonic world for X-marker drop mission`)
- After (vision sdf의 line 3~19, 백업 line 921-937에서 인용):
```
x_marker_world_vision.sdf - Vision variant of x_marker_world
=========================================================================
This is the VISION-MISSION variant of x_marker_world.sdf.
Difference from base: adds gz::sim::systems::Sensors (ogre2) so that
the x500_bombard down-camera actually renders and publishes frames.

Use this world from drone_mission.launch.py (vision-guided runs).
RL training (infra.launch.py) keeps using the base x_marker_world.sdf,
which omits Sensors to avoid GPU overhead → ODE AABB crash.

IMPORTANT: world name="x_marker_world" is intentionally kept identical
to the base so PX4_GZ_WORLD and the ros_gz_bridge IMU topic
(/world/x_marker_world/...) work unchanged.

When updating the base world, mirror the change here too — diff the
two files; the only intended divergence is the Sensors plugin block
inserted between Contact and Imu plugins below.
```

**변경 위치 (Sensors 플러그인 삽입)**: vision sdf의 line 81-83 부근 (Contact 플러그인 ~55-56 다음, Imu 플러그인 직전):

```xml
<plugin name="gz::sim::systems::Sensors"
        filename="gz-sim-sensors-system">
  <render_engine>ogre2</render_engine>
</plugin>
```

**중요**: `<world name="x_marker_world">` 태그는 베이스와 **동일하게 유지**. PX4_GZ_WORLD 환경변수와 `/world/x_marker_world/...` IMU 브릿지 토픽이 변경 없이 동작해야 하기 때문.

**의존성**: 베이스 `x_marker_world.sdf`가 먼저 존재해야 함 (clone 직후엔 보통 존재).

**검증**:
```bash
docker exec drone-bombard-harmonic ls /workspace/gazebo_models/worlds/
# x_marker_world_vision.sdf 존재 확인 (~15 KB)
docker exec drone-bombard-harmonic diff /workspace/gazebo_models/worlds/x_marker_world.sdf /workspace/gazebo_models/worlds/x_marker_world_vision.sdf
# diff가 두 곳만 — 헤더 주석 블록 + Sensors 플러그인 블록
```

**런타임 검증 (drone_mission launch 후)**:
```bash
docker exec drone-bombard-harmonic bash -c 'ros2 topic hz /camera/rgb/image_raw'
# 약 30Hz 근처가 나와야 정상
```

---

### 2. `/workspace/gazebo_models/worlds/x_marker_world.sdf` [APPLIED — 주석만 변경]

**이유**: 변경 1을 만든 후, 베이스 sdf의 옛 주석("Re-enable only for vision-based runs on a GPU VM with use_vision=True")이 더 이상 정확하지 않으므로 갱신.

**변경 위치**: line 58-66 부근 (Sensor systems 주석 블록)

**Before** (백업 line 939-945에서 인용):
```
<!-- Sensor systems: handle IMU, baro, GPS, magnetometer for PX4 SITL -->
<!-- NOTE: gz-sim-sensors-system (ogre2 camera rendering) is intentionally
     omitted from THIS file (RL-training variant). Enabling it causes GPU
     rendering overhead → physics timestep instability → Motor Aliasing
     → ODE AABB crash during long RL rollouts.
     For vision-guided missions, use the sibling world file instead:
```

**After** (백업 truncation 후 확인된 사실: 비전 미션은 별도 변형 `x_marker_world_vision.sdf`를 사용. 이 파일은 RL 학습 전용):
```
<!-- Sensor systems: handle IMU, baro, GPS, magnetometer for PX4 SITL -->
<!-- NOTE: gz-sim-sensors-system (ogre2 camera rendering) intentionally
     omitted here. This file is the RL-training variant.
     For vision-guided missions, use the sibling file:
       x_marker_world_vision.sdf
     The vision variant adds Sensors (ogre2) but keeps
     world name="x_marker_world" so PX4_GZ_WORLD and the IMU bridge
     topic (/world/x_marker_world/...) work unchanged.
     Mirror any base-file change into the vision variant. -->
```

**의존성**: 변경 1과 함께 적용.

**검증**: 위 (1)의 diff 명령으로 두 곳만 다른지 확인.

---

### 3. `/workspace/ros2_ws/src/mission_manager/launch/drone_mission.launch.py` [APPLIED]

**이유**: 비전 미션 launch가 vision 변형 월드를 띄우도록 변경.

**변경 위치**: line 196, 204, 321 — `x_marker_world.sdf` → `x_marker_world_vision.sdf` (총 3곳).

**Before** (백업 line 738-741):
```python
196:             f"gz sim -r {worlds_dir}/x_marker_world.sdf"],
204:             f"gz sim -r -s {worlds_dir}/x_marker_world.sdf"],
223:                 f"PX4_GZ_WORLD=x_marker_world "
321:            f"  World:       {worlds_dir}/x_marker_world.sdf\n",
```

**After** (백업 line 851-854):
```python
196:             f"gz sim -r {worlds_dir}/x_marker_world_vision.sdf"],
204:             f"gz sim -r -s {worlds_dir}/x_marker_world_vision.sdf"],
223:                 f"PX4_GZ_WORLD=x_marker_world "      # ← 의도적으로 그대로 (world name 매칭)
321:            f"  World:       {worlds_dir}/x_marker_world_vision.sdf\n",
```

**중요**: line 223의 `PX4_GZ_WORLD=x_marker_world`는 **변경하지 않는다**. SDF 파일명이 아니라 `<world name="...">`의 이름이고, 변형도 같은 world name을 쓴다.

**의존성**: 변경 1이 먼저 완료되어야 launch가 동작.

**install/share 동기화 (필수)**:
```bash
docker cp /home/juns/Drone-Bombard-Simulation/ros2_ws/src/mission_manager/launch/drone_mission.launch.py \
  drone-bombard-harmonic:/workspace/ros2_ws/install/mission_manager/share/mission_manager/launch/drone_mission.launch.py

# __pycache__도 삭제 권장
docker exec drone-bombard-harmonic bash -c '
  find /workspace/ros2_ws -path "*/launch/__pycache__" -exec rm -rf {} + 2>/dev/null
'
```

**검증**:
```bash
docker exec drone-bombard-harmonic grep -n x_marker_world /workspace/ros2_ws/install/mission_manager/share/mission_manager/launch/drone_mission.launch.py
# line 196, 204, 321이 vision 변형을 가리키고, 223은 base name 유지하는지 확인
```

---

### 4. `/workspace/ros2_ws/src/rl_navigation/config/hyperparams.yaml` [APPLIED — 다중 변경]

**이유**: 회의 권장값 (H1/H3/M1/M2/L6) 적용 + wandb entity 팀으로 이전 + RL 학습용 use_vision off + 본 학습 step 200k.

**변경 위치 및 Before → After** (백업 곳곳: line 2207-2208, 6843-6853, 8400-8410, 10117-10123 등에서 종합):

#### 4-A. `training:` 블록

| key | Before (원본) | After (현재) | 코멘트 |
|---|---|---|---|
| `total_timesteps` | `1000000` (원본 팀 default) | `200000` | 본 학습. smoke 검증 단계에선 1000/5000/10000으로 일시 축소했었음 |
| `eval_freq` | (없음) | `10000` | L6: deterministic eval every N env steps |
| `eval_episodes` | (없음) | `3` | L6 |

#### 4-B. `sac:` 블록

| key | Before | After | 코멘트 |
|---|---|---|---|
| `buffer_size` | `100000` | `500000` | M1: catastrophic forgetting 방지 |
| `gamma` | `0.99` | `0.995` | H3: effective horizon ~100 → ~200 |
| `gradient_steps` | `1` | `4` | M2: off-policy sample reuse 가속 |

#### 4-C. `environment:` 블록

| key | Before | After | 코멘트 |
|---|---|---|---|
| `use_vision` | `true` (사용자가 원래 켜놨었음) | `false` | RL 학습에는 camera obs/termination 비활성. 비전 학습 따로 분리 |
| `auto_drop_threshold` | `0.5` (원본) | `2.0` | 사용자가 이전부터 변경. 그대로 유지 |
| `max_steps`, scale 등 | 변경 없음 | 변경 없음 | — |

#### 4-D. `reward:` 블록 (H1)

| key | Before | After | 코멘트 |
|---|---|---|---|
| `w_drop_base` | `50.0` | `20.0` | H1 |
| `r_success_jackpot` | `100.0` | `30.0` | H1 |
| `penalty_instability` | `50.0` | `15.0` | H1 |
| `w_impact` | (없음) | `2.0` | 사용자 prior — d_impact reward shaping 추가 |
| `k_impact` | (없음) | `0.05` | 사용자 prior |

#### 4-E. `wandb:` 블록

| key | Before | After | 코멘트 |
|---|---|---|---|
| `entity` | `"team"` 또는 `"junsanglee64"` (개인 free 만료) | `"nayoonho0922-seoul-national-university"` | 팀 entity (SNU). 개인 free tier 만료로 이전 |

**의존성**: 없음. 단독 변경 가능.

**install/share 동기화 (필수)**:
```bash
docker exec drone-bombard-harmonic cp \
  /workspace/ros2_ws/src/rl_navigation/config/hyperparams.yaml \
  /workspace/ros2_ws/install/rl_navigation/share/rl_navigation/config/hyperparams.yaml
```

**검증**:
```bash
docker exec drone-bombard-harmonic diff \
  /workspace/ros2_ws/src/rl_navigation/config/hyperparams.yaml \
  /workspace/ros2_ws/install/rl_navigation/share/rl_navigation/config/hyperparams.yaml && echo OK
```

---

### 5. `/workspace/ros2_ws/src/rl_navigation/rl_navigation/drone_drop_env.py` [APPLIED]

**이유**: H2 — `action[4]` (manual drop trigger)가 이미 코드에서 disabled 상태였지만 `action_space`는 여전히 5-d로 선언돼 있어서 SAC가 의미 없는 5번째 dimension을 학습하려고 시도. action space를 4-d로 줄여 학습 효율 개선.

**변경 위치** (백업 line 6315-6321, 6244-6256 등에서 인용):

#### 5-A. action_space 선언 (line ~400-403)

**Before**:
```python
self.action_space = gym.spaces.Box(
    low=-1.0, high=1.0, shape=(5,), dtype=np.float32)
```

**After**:
```python
self.action_space = gym.spaces.Box(
    low=-1.0, high=1.0, shape=(4,), dtype=np.float32)  # H2: 5 → 4
```

#### 5-B. action_prev 초기화 (line ~417 및 ~453, 두 곳)

**Before**:
```python
self.action_prev = np.zeros(5, dtype=np.float32)
```

**After**:
```python
self.action_prev = np.zeros(4, dtype=np.float32)  # H2: 5 → 4
```

#### 5-C. 클래스 docstring (line ~302-307)

**Before**:
```
Action:      Box(5,)  float32 in [-1, 1]
  [0] vx command (scaled by action_vx_scale)
  [1] vy command (scaled by action_vy_scale)
  [2] vz command (scaled by action_vz_scale)
  [3] yaw_rate command (scaled by action_yaw_scale)
  [4] manual drop trigger (> 0 fires drop; auto-drop overrides via kinematics)
```

**After** (요약: line [4] 제거 + H2 코멘트 추가, 정확한 자구는 백업 line 6315에서 발췌됨):
```
Action:      Box(4,)  float32 in [-1, 1]
  [0] vx command (scaled by action_vx_scale)
  [1] vy command (scaled by action_vy_scale)
  [2] vz command (scaled by action_vz_scale)
  [3] yaw_rate command (scaled by action_yaw_scale)

H2: 옛 action[4] (manual drop trigger) 는 CCIP auto-drop 과 충돌해서
    학습 신호 못 받음 → 제거.
```

#### 5-D. step() 내 옛 manual_drop 주석 (line ~572-574)

**Before**:
```python
# Phase 1 curriculum: manual drop disabled. action[4] is kept as a
# dummy dimension (always ignored) for checkpoint compatibility.
# manual_drop = float(action[4]) > 0.0
```

**After**:
```python
# H2: 옛 manual drop (action[4]) 는 auto-drop 과 충돌해서 학습 신호 못 받음 → 제거.
# action[4] 자체가 action_space 에서 사라졌으므로 별도 코드 없음.
```

**L4 (obs clip)는 추가 작업 불필요**: 이미 line 881-882에 `np.clip(0, 1)`이 들어있다는 사실을 백업 line 6116-6117에서 확인. clone 직후 그 상태일 가능성 높음. 만약 빠져있으면 (백업 인용):
```python
obs_d_impact = float(np.clip(d_impact_obs / self._cfg_pos_scale, 0.0, 1.0))
obs_t_f = float(np.clip(t_f_obs / 10.0, 0.0, 1.0))
```

**의존성**: 변경 4 (hyperparams)와 짝. action space 변경하면 옛 체크포인트 (5-d action) 와 호환 안 됨 — 본 학습 fresh start 필수.

**install/lib 동기화 (필수)**:
```bash
docker cp /home/juns/Drone-Bombard-Simulation/ros2_ws/src/rl_navigation/rl_navigation/drone_drop_env.py \
  drone-bombard-harmonic:/workspace/ros2_ws/install/rl_navigation/lib/python3.10/site-packages/rl_navigation/drone_drop_env.py
```

**검증**:
```bash
docker exec drone-bombard-harmonic grep -n -E "shape=\(4|shape=\(5|np\.zeros\(4|np\.zeros\(5|action\[4\]" \
  /workspace/ros2_ws/install/rl_navigation/lib/python3.10/site-packages/rl_navigation/drone_drop_env.py
# shape=(4,) 1곳, np.zeros(4,) 2곳, action[4] 참조는 없거나 H2 주석 안에만 남아야 함
```

---

### 6. `/workspace/ros2_ws/src/rl_navigation/rl_navigation/train_sac.py` [APPLIED]

**이유**: L6 — 회의 결정 "현 상태 유지 + 주석 보강" (별도 EvalCallback 추가는 우리 환경 무게 때문에 부적합). 기존 `BestModelCallback`의 docstring 보강하고, hyperparams.yaml의 `eval_freq`를 콜백에 연결.

**변경 위치 및 내용** (백업 line 6395-6411, 6582-6590):

#### 6-A. hyperparams 로드 시점에 `eval_freq` 추출 (line ~273 부근)

**Before**:
```python
checkpoint_freq = cfg_train.get('checkpoint_freq', 5_000)
max_checkpoints_kept = cfg_train.get('max_checkpoints_kept', 5)
```

**After** (추가):
```python
checkpoint_freq = cfg_train.get('checkpoint_freq', 5_000)
max_checkpoints_kept = cfg_train.get('max_checkpoints_kept', 5)
eval_freq = cfg_train.get('eval_freq', 10_000)  # L6
```

#### 6-B. `BestModelCallback` 호출에 eval_freq 전달 (line ~335)

**Before**:
```python
best_model_callback = BestModelCallback(
    save_path=best_model_dir, eval_freq=10_000, verbose=1)
```

**After**:
```python
best_model_callback = BestModelCallback(
    save_path=best_model_dir, eval_freq=eval_freq, verbose=1)
```

#### 6-C. `BestModelCallback` docstring 보강 (line ~142-156)

L6 회의 결정 ("현 상태 유지 + 주석 보강")의 일환으로 — 별도 EvalCallback 안 쓰는 이유 명시. 정확한 문구는 백업에서 직접 인용된 부분이 없지만, "이 워크스페이스의 env 부팅 비용이 90초+라 별도 eval env 띄우는 비용이 학습 가속 이득보다 크다. rollout `ep_rew_mean` 기반 best 추적이 현실적인 절충"의 취지로 작성됨. **이 docstring 정확한 자구는 백업에 없으므로 재작성 시 임의 변경 가능 — 핵심 의도만 유지**.

**의존성**: 변경 4의 `eval_freq` key 가 yaml에 있어야 함.

**install/lib 동기화 (필수)**:
```bash
docker cp /home/juns/Drone-Bombard-Simulation/ros2_ws/src/rl_navigation/rl_navigation/train_sac.py \
  drone-bombard-harmonic:/workspace/ros2_ws/install/rl_navigation/lib/python3.10/site-packages/rl_navigation/train_sac.py
```

**검증**:
```bash
docker exec drone-bombard-harmonic grep -nE "eval_freq = cfg|eval_freq=eval_freq" \
  /workspace/ros2_ws/install/rl_navigation/lib/python3.10/site-packages/rl_navigation/train_sac.py
```

---

### 7. PX4 airframes `4016_~4019_gz_x500_bombard_r0~r3` [APPLIED — ROMFS→rootfs cp]

**이유**: ROMFS에는 multi-instance용 airframe 4개가 정의돼 있지만, 일반 PX4 빌드 시 rootfs로 자동 복사되지 않음. 그 결과 `drone_drop_env.py`가 `PX4_SIM_MODEL=gz_x500_bombard_r0`로 PX4를 띄우려 하면 `ERROR [init] Unknown model gz_x500_bombard_r0` 로 즉사 → train_sac이 90초 timeout만 뱉다 죽음.

**변경 종류**: 컨테이너 내부 rootfs 파일 4개 복사 (코드 변경 아님).

**작업**:
```bash
docker exec drone-bombard-harmonic bash -c '
SRC=/opt/PX4-Autopilot/ROMFS/px4fmu_common/init.d-posix/airframes
DST=/opt/PX4-Autopilot/build/px4_sitl_default/rootfs/etc/init.d-posix/airframes
for f in 4016_gz_x500_bombard_r0 4017_gz_x500_bombard_r1 4018_gz_x500_bombard_r2 4019_gz_x500_bombard_r3; do
  cp -v "$SRC/$f" "$DST/$f"
done
'
```

**의존성**: PX4가 빌드된 상태여야 함 (`/opt/PX4-Autopilot/build/px4_sitl_default/rootfs/` 존재).

**검증**:
```bash
docker exec drone-bombard-harmonic ls /opt/PX4-Autopilot/build/px4_sitl_default/rootfs/etc/init.d-posix/airframes/ | grep -E "401[5-9]_gz_x500_bombard"
# 5개 (base + r0/r1/r2/r3) 모두 보여야 함
```

**중요 (메모리에 기록된 사실)**:
- 이 cp는 **컨테이너 lifetime 동안만 유효**. `docker rm` 후 새 컨테이너 만들면 다시 사라짐.
- 영구 해결은 `cd /opt/PX4-Autopilot && make px4_sitl_default` 풀빌드 (15-20분 소요).
- 컨테이너 stop/start만으로는 사라지지 않음 (5/20 cp가 5/21 시점에도 보존돼 있던 사실로 확인).

**런타임 검증**:
```bash
# train_sac 시작 후 PX4 startup 로그에 "SYS_AUTOSTART=4016" 보여야 정상
# Unknown model 에러 뜨면 위 cp 다시 실행
```

---

### 8. `/home/juns/wandb_incremental_sync.sh` [APPLIED — 호스트 신규 파일]

**이유**: WandB 클라우드 sync를 학습 main process에서 분리해서 WandB IPC 데드락 (3,612 step에서 발생했던 사고) 방지. 10분 주기로 background에서만 sync.

**작업**: 호스트에 새 셸 스크립트 작성 (3,246 bytes, 실행 권한). 이 파일은 git 저장소가 아닌 호스트 홈에 있으므로 `git pull`로 사라지지 않는다. **단, 처음 환경 셋업 시(=다른 머신 또는 호스트 reset) 다시 작성 필요**.

**스크립트 핵심 동작**:
1. 무한 루프, 10분(`interval=600s`) 주기.
2. 각 사이클마다 컨테이너 `/workspace/ros2_ws/wandb/offline-run-*` 폴더들을 폴더별로 `wandb sync` (per-folder, timeout 300s).
3. stdout/stderr를 `/tmp/wandb_sync.log`에 누적 기록.
4. 시작 시 `[wandb-sync] start interval=600s container=drone-bombard-harmonic dir=/workspace/ros2_ws/wandb` 라인 출력.

**정확한 스크립트 내용은 백업에서 단편적으로만 보임 (line 6786-6788)**. 재작성 시 핵심:
```bash
#!/usr/bin/env bash
# /home/juns/wandb_incremental_sync.sh
INTERVAL=${INTERVAL:-600}
CTR=${CTR:-drone-bombard-harmonic}
DIR=${DIR:-/workspace/ros2_ws/wandb}
echo "[wandb-sync] start  interval=${INTERVAL}s  container=${CTR}  dir=${DIR}"
echo "[wandb-sync] pid=$$  $(date -Iseconds)"
while true; do
  RUNS=$(docker exec "$CTR" bash -c "ls -1d ${DIR}/offline-run-* 2>/dev/null" || true)
  if [ -n "$RUNS" ]; then
    COUNT=$(printf '%s\n' "$RUNS" | wc -l)
    echo "[wandb-sync] $(date -Iseconds)  syncing ${COUNT} run(s)…"
    while IFS= read -r RUN; do
      [ -z "$RUN" ] && continue
      timeout 300 docker exec "$CTR" bash -c \
        "cd /workspace/ros2_ws && wandb sync '$RUN'" \
        >> /tmp/wandb_sync.log 2>&1
    done <<< "$RUNS"
  fi
  sleep "$INTERVAL"
done
```

> 위 코드는 백업의 출력 패턴 (`[wandb-sync] start  interval=600s  container=drone-bombard-harmonic dir=/workspace/ros2_ws/wandb` / `[wandb-sync] pid=37089  2026-05-20T16:45:05+09:00` / `[wandb-sync] 2026-05-20T16:45:05+09:00  syncing 6 run(s)…`)을 재현하기 위한 **합리적 재구성**이다. 원본 스크립트와 자구가 다를 수 있으므로 UNCERTAIN으로 표시한다.

**상태 정정**: 스크립트 **파일 자체는 APPLIED**. 위 코드의 정확한 자구는 **UNCERTAIN (재작성)**.

**가동 명령**:
```bash
pgrep -f wandb_incremental_sync.sh >/dev/null \
  || nohup /home/juns/wandb_incremental_sync.sh > /tmp/wandb_sync.log 2>&1 &
```

**종료 명령**:
```bash
pkill -KILL -f wandb_incremental_sync.sh
# 또는 PID 직접: pgrep -af wandb_incremental_sync.sh 후 kill -9 <PID>
```

**중요 운영 메모**:
- WandB 무료 tier가 만료된 상태에서 옛 `junsanglee64` entity로 sync 시도하는 옛 offline-run들은 매 사이클 실패. 새 학습 시작 전 옛 offline-run 폴더 정리 권장.
- 2026-05-21 마지막에 사용자가 모든 wandb 폴더 삭제 (~175MB 회수) 후 sync 스크립트를 SIGKILL로 종료한 상태.

---

### 9. `/workspace/ros2_ws/src/rl_navigation/rl_navigation/drone_drop_env.py` line 1131 [REVERTED]

**이유**: PX4 startup 실패 진단을 위해 일시적으로 stderr를 `/tmp/px4.log`로 redirect했었음. 진단 후 원복.

**일시 변경 (REVERTED, 정보용)**:
- Before (원본): `px4_log = open(os.devnull, 'w')`
- 일시 적용: `px4_log = open('/tmp/px4.log', 'w')  # TEMP: diagnose PX4 startup`
- 현재 (원복 완료): `px4_log = open(os.devnull, 'w')`

**왜 원복?**
- pxh interactive shell의 출력이 ~100 KB/s로 폭증해서 디스크 IO가 PX4 메인 loop을 느리게 함. 진단 시 `/tmp/px4.log`가 1.3GB까지 부풀어 있었음.
- 원본 코드 의도가 정상.

**조치**: 재적용 작업 없음. 단 만약 `git pull` 후 다시 진단 필요 시 동일하게 임시 redirect 가능.

---

## install/share + install/lib 동기화 일괄 명령 (재적용 직후 실행)

clone 직후 colcon build를 새로 했다면 자동 처리되지만, build 안 하고 src 수정만 했을 때는 다음 일괄 명령:

```bash
docker exec drone-bombard-harmonic bash -c '
set -e
# launch
cp /workspace/ros2_ws/src/mission_manager/launch/drone_mission.launch.py \
   /workspace/ros2_ws/install/mission_manager/share/mission_manager/launch/drone_mission.launch.py
# yaml
cp /workspace/ros2_ws/src/rl_navigation/config/hyperparams.yaml \
   /workspace/ros2_ws/install/rl_navigation/share/rl_navigation/config/hyperparams.yaml
# python modules
cp /workspace/ros2_ws/src/rl_navigation/rl_navigation/drone_drop_env.py \
   /workspace/ros2_ws/install/rl_navigation/lib/python3.10/site-packages/rl_navigation/drone_drop_env.py
cp /workspace/ros2_ws/src/rl_navigation/rl_navigation/train_sac.py \
   /workspace/ros2_ws/install/rl_navigation/lib/python3.10/site-packages/rl_navigation/train_sac.py
# 캐시 정리
find /workspace/ros2_ws -path "*/__pycache__" -exec rm -rf {} + 2>/dev/null || true
echo "src → install sync OK"
'
```

또는 완전히 안전하게:
```bash
docker exec drone-bombard-harmonic bash -c '
cd /workspace/ros2_ws && \
source /opt/ros/humble/setup.bash && \
colcon build --packages-select rl_navigation mission_manager
'
```

---

## 컨테이너 재생성 후 일회성 (best-effort) 적용 순서

`git pull` + 빈 컨테이너 시나리오:

1. (호스트) 변경 1, 2, 3을 git working tree에 패치 또는 수동 적용.
2. (호스트) 변경 4, 5, 6을 git working tree에 패치 또는 수동 적용.
3. (호스트) `docker start drone-bombard-harmonic` (또는 재생성).
4. (컨테이너) `cd /workspace/ros2_ws && colcon build --packages-select rl_navigation mission_manager` — 정석. 또는 위 일괄 cp 명령.
5. (컨테이너) PX4 airframe cp (변경 7).
6. (호스트, 선택) `wandb_incremental_sync.sh` 재작성 (변경 8).
7. (컨테이너) 학습 시작:
   ```bash
   source /opt/ros/humble/setup.bash
   source /root/ros2_ws/install/setup.bash
   source /workspace/ros2_ws/install/setup.bash
   export WANDB_MODE=offline
   ros2 run rl_navigation train_sac > /tmp/train.log 2>&1
   ```

---

## 주요 시행착오 (REVERTED / 교훈)

- **카메라 안 흐름 → world에 Sensors 추가 (한 파일)**: 처음에는 base sdf 한 파일에 Sensors 추가도 옵션이었으나 사용자가 "분리하는 방향" 선택 → vision 변형 SDF 신설로 결론. 단일 파일 수정 시도는 REVERTED.
- **install/share 동기화 누락**: src만 고쳐서 "왜 안 되지" 헤매다 install/share를 직접 cp로 동기화. 메모리 `feedback_ros2_install_cache.md`에 기록.
- **PX4 stderr → /tmp/px4.log 임시 redirect** (변경 9): 진단 후 원복.
- **L6 SB3 EvalCallback 추가**: 처음엔 추가 방향이었으나 env 부팅 비용 90초+ 사정으로 회의에서 "현 상태 유지 + 주석 보강"으로 결정 → 변경 6의 형태로 축소.
- **N1 Drop Sparse Signal**: smoke 두 번 연속 `drop_count=1`, `mean_rew_drop=0`로 sparse signal 문제 발견. 회의 결정은 "D — 그대로 진행하면서 관찰". 코드 변경 없음.
- **PX4 silent fail "Unknown model" 첫 진단**: 옵션 A(full rebuild) / B(cp) / C(env 코드 변경) 중 B 선택 → 변경 7로 적용.
- **WandB IPC 데드락 (3,612 step)**: `WANDB_MODE=offline` + incremental sync 패턴으로 회피. 학습 코드 자체에는 변경 없음.
- **2026-05-21 모든 wandb 폴더 삭제**: 사용자가 "모두 지움 (백업 포함)" 선택 → 약 175MB 회수. 데이터 손실 없음 (이미 분석 완료된 run들). 본 manifest에는 영향 없음 (학습 결과물일 뿐 코드/설정이 아니라서).
- **action_yaw_scale 1.0 → 2-3 (L1)**: 회의 후순위, **적용 안 함**.
- **num_envs 1 → multi (L2)**: 회의 후순위, **적용 안 함**.
- **learning_starts 1000 → 5000~10000 (L3)**: 회의 후순위, **적용 안 함**.
- **Frame stacking / RecurrentSAC (L5)**: 회의 후순위, **적용 안 함**.
- **`--symlink-install` 전환**: 사용자에게 설명만 함, 실제 전환 **안 함**. clone 후에도 보통 정식 colcon build로 진행.

---

## 미해결 / UNCERTAIN

1. **`x_marker_world.sdf`의 새 주석 정확한 자구**: 백업에 truncate되어 "비전 미션은 별도 변형..." 의도만 확인됨. 위 [#2] 의 After 블록은 그 의도를 만족하는 재작성이며 원본과 자구가 다를 수 있다. UNCERTAIN — 의도만 충족하면 OK.

2. **`x_marker_world_vision.sdf`의 Sensors 플러그인 정확한 위치 (line 80~83)**: diff 결과로 line 81-83 부근에 추가됐다는 사실은 확인. 정확히 어느 들여쓰기 레벨인지는 부분만 보임. SDF parser는 들여쓰기 무관하지만 가독성 위해 다른 plugin과 동일한 들여쓰기 유지 권장.

3. **`drone_drop_env.py` docstring (line ~302-307) After 자구**: 백업에서 부분만 인용됨. 의도(`Action: Box(4,)`로 줄이고 [4] 줄 제거 + H2 주석 추가)는 명확. 정확한 자구는 다소 변경되어도 무방.

4. **`train_sac.py` `BestModelCallback` docstring 보강 (변경 6-C) 정확한 자구**: 백업에 인용 없음. L6 결정 ("현 상태 유지 + 주석 보강")의 의도만 충족하면 됨. UNCERTAIN.

5. **`wandb_incremental_sync.sh` 스크립트 본문 (변경 8)**: 백업에 부분 출력만 있음. 위 #8 에서 재구성한 코드는 합리적 추정이지만 원본과 자구가 다를 수 있음. UNCERTAIN.

6. **`hyperparams.yaml`의 정확한 `wandb:` 블록 구조**: `entity`, `project`, `log_freq` 등이 어떤 들여쓰기/key로 있는지 단편적으로만 확인. clone 후 그 블록의 기존 key 이름 그대로 따르고 `entity` 값만 교체하는 것이 안전.

7. **`auto_drop_threshold` 값**: 사용자 prior 변경(0.5 → 2.0)이 hyperparams.yaml에 있음. 본 manifest는 이를 보존하는 것을 가정. clone 후 원본 값(0.5)으로 돌아오면 다시 2.0으로 변경할지는 N1 의사결정과 연관.

8. **`infra.launch.py`**: RL 학습용 launch. 변경 없음을 확인 (백업 line 1007: "infra.launch.py, ros_gz_bridge.yaml, model.sdf, PX4_GZ_WORLD는 손대지 않았습니다"). clone 직후 상태 그대로 사용.

---

## 검증용 다중-위치 동기화 체크리스트 (clone 후 한 번에 확인)

```bash
docker exec drone-bombard-harmonic bash -c '
echo "[1] vision world sdf"; ls -la /workspace/gazebo_models/worlds/x_marker_world_vision.sdf
echo "[2] base world Sensors 부재 확인 (있으면 fail)"; grep -c "gz-sim-sensors-system" /workspace/gazebo_models/worlds/x_marker_world.sdf
echo "[2v] vision world Sensors 존재 확인 (1이어야 함)"; grep -c "gz-sim-sensors-system" /workspace/gazebo_models/worlds/x_marker_world_vision.sdf
echo "[3] launch points to vision"; grep -n x_marker_world /workspace/ros2_ws/install/mission_manager/share/mission_manager/launch/drone_mission.launch.py
echo "[4] hyperparams (src vs install)"; diff /workspace/ros2_ws/src/rl_navigation/config/hyperparams.yaml /workspace/ros2_ws/install/rl_navigation/share/rl_navigation/config/hyperparams.yaml && echo "yaml OK"
echo "[5] env action 4-d"; grep -nE "shape=\(4|shape=\(5|action\[4\]" /workspace/ros2_ws/install/rl_navigation/lib/python3.10/site-packages/rl_navigation/drone_drop_env.py
echo "[6] train_sac eval_freq wired"; grep -nE "eval_freq = cfg|eval_freq=eval_freq" /workspace/ros2_ws/install/rl_navigation/lib/python3.10/site-packages/rl_navigation/train_sac.py
echo "[7] PX4 airframes (5개)"; ls /opt/PX4-Autopilot/build/px4_sitl_default/rootfs/etc/init.d-posix/airframes/ | grep -E "401[5-9]_gz_x500_bombard" | wc -l
'
```

각 항목 기대값:
- [1] 파일 존재
- [2] 0 (base는 Sensors 없음)
- [2v] 1 (vision은 1개 있음)
- [3] line 196, 204, 321 → vision sdf / line 223 → `PX4_GZ_WORLD=x_marker_world`
- [4] `yaml OK`
- [5] `shape=(4,)` 1곳 매치, `shape=(5,)` 0 매치, `action[4]` 참조는 H2 주석 안에만
- [6] 두 패턴 모두 매치
- [7] 5

---

## 참고: 관련 가이드 / 메모리 파일들 (호스트 보존)

호스트 측에 있는 다음 파일들은 git이 아니라 단순 호스트 파일이므로 `git pull`로 사라지지 않는다. 단, 새 머신/호스트 reset 시 재작성 필요.

- `/home/juns/drone_sim_fresh_training_start_guide.txt` (43KB, 788줄) — 본 학습 단일-사이클 walkthrough
- `/home/juns/drone_sim_tmux_training_guide.txt` (31KB, 515줄) — tmux 사용법
- `/home/juns/drone_sim_training_guide.txt` (12KB, 252줄) — 종합 매뉴얼
- `/home/juns/meeting_notes_2026-05-20.txt` (624줄) — 회의 노트 (최종 갱신 2026-05-21)
- `/home/juns/meeting_slides_2026-05-20_v2.pptx` (17 슬라이드)
- `/home/juns/wandb_incremental_sync.sh` (변경 8)
- `/home/juns/backup_2026-05-20_drone-bombard/` (이전 백업, 일부는 사용자가 2026-05-21에 wandb 부분만 삭제)
- `/home/juns/.claude/projects/-home-juns/memory/MEMORY.md`(인덱스) + `feedback_local_only.md`, `project_drone_sim.md`, `feedback_ros2_install_cache.md`, `project_px4_airframe_sync.md` — Claude auto-memory (다음 세션 자동 로드)

## 부가: 로컬 git branch `junsang` (commit `29b1c9b`)

2026-05-19 시점에 호스트 git 저장소에 로컬 branch `junsang`이 만들어졌고 변경 1, 2, 3이 commit됨. `git push` 안 됨. **`git pull`로 다른 branch로 가면 이 branch는 그대로 남아있지만 working tree는 떠나간 branch 기준이 됨**. 새 코드 위에 이 manifest 다시 적용하려면:

- 옵션 A: `git checkout junsang` (옛 commit 기준) — 새 코드와 분리됨. 비추.
- 옵션 B: `git cherry-pick 29b1c9b` (새 branch 위에 변경 1,2,3 재적용) — 충돌 가능성 있음.
- 옵션 C: 이 manifest 보고 변경 1~6 수동 재적용 — 권장. 변경 4,5,6은 `29b1c9b`엔 없는 추가 변경이라 어차피 수동 작업 필요.
