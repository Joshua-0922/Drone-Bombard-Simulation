# 변경 사항

> **기준 브랜치:** `feature/migration-harmonic` → **변경 브랜치:** `donghyeok`
> **작성일:** 2026-03-21

---

## 개요

기존 SAC 학습 시스템에 4가지 기능을 추가하였습니다.

| # | 기능 | 핵심 목적 |
|---|------|----------|
| 1 | TAKEOFF 대기 시간 제거 | 에피소드 리셋 속도 개선 |
| 2 | 커리큘럼 학습 | 단계적 난이도 상승으로 학습 효율 향상 |
| 3 | YOLO 비전 연결 | 실제 카메라 기반 관측값으로 전환 |
| 4 | Optuna 하이퍼파라미터 최적화 | SAC 파라미터 자동 탐색 |

---

## 1. TAKEOFF 대기 시간 제거

### 기존 동작
에피소드 리셋 시 드론을 **지면(z=0)**에 텔레포트한 뒤, mission_manager의 상태 기계가 TAKEOFF → CRUISE 순서로 진행됨. 드론이 이륙하여 순항 고도(5m)에 도달할 때까지 **약 10~20초** 대기.

```
리셋 → 지면 텔레포트 → arm → 이륙 → 고도 도달 → CRUISE → 학습 시작
        (z=0)                    (~10~20s 소요)
```

### 변경 후 동작
드론을 처음부터 **순항 고도(z=5.0m)**에 직접 텔레포트. `skip_takeoff=true` 파라미터로 mission_manager가 TAKEOFF 단계를 건너뜀.

```
리셋 → 순항 고도 텔레포트 → arm → CRUISE 즉시 진입 → 학습 시작
        (z=5.0m)                   (<5s 소요)
```

### 수정된 파일

**`ros2_ws/src/rl_navigation/rl_navigation/drone_drop_env.py`**
- `_gz_reset_poses()`: 텔레포트 높이를 `cruise_altitude`(기본 5.0m)로 변경
- `_start_episode()`: mission_manager 실행 시 `--ros-args -p skip_takeoff:=true` 추가
- `__init__()`: `_cfg_cruise_altitude` 설정값 로드 추가

**`ros2_ws/src/mission_manager/mission_manager/mission_manager_node.py`**
- `skip_takeoff` ROS 파라미터 선언 (기본값: `false`)
- `control_loop()`: TAKEOFF 상태에서 `skip_takeoff=true`이면 즉시 CRUISE 진입

**`ros2_ws/src/rl_navigation/config/hyperparams.yaml`**
```yaml
# 추가된 설정
environment:
  cruise_altitude: 5.0        # 텔레포트 목표 고도
  cruise_poll_timeout: 20.0   # 60.0 → 20.0초로 단축
```

### 효과
- 에피소드당 리셋 시간 **10~20초 절감**
- 기존 일반 시뮬레이션에는 영향 없음 (`skip_takeoff` 기본값 `false`)

---

## 2. 커리큘럼 학습

### 기존 동작
처음부터 드론이 타겟으로부터 **~45m 거리** 고정 위치에서 스폰. 탐색 공간이 넓어 초기 학습이 매우 느림. 단계 전환 없음.

### 변경 후 동작
4단계 거리 기반 커리큘럼. 가까운 거리부터 시작하여 성공률이 기준치를 넘으면 다음 단계로 자동 전환.

| 스테이지 | 이름 | 스폰 거리 | max_steps | 비전 | 전환 조건 |
|---------|------|----------|-----------|------|----------|
| 0 | Close | 3~8m | 150 | 끄기 | success_rate > 0.6 (50에피소드) |
| 1 | Medium | 8~20m | 300 | 끄기 | success_rate > 0.5 (50에피소드) |
| 2 | Full | 20~50m | 500 | 끄기 | success_rate > 0.4 (50에피소드) |
| 3 | Vision | 20~50m | 500 | **켜기** | (최종 단계) |

**Stage 3 (Vision):** `rel_x`, `rel_y` (ground truth 상대 위치)를 0으로 마스킹 → 에이전트가 오직 YOLO 비전 정보만으로 네비게이션하도록 강제.

### 수정된 파일

**`ros2_ws/src/rl_navigation/rl_navigation/drone_drop_env.py`**
- `__init__()`: 커리큘럼 상태 변수 초기화 (`_curriculum_stages`, `_curriculum_idx`, `_curriculum_successes`)
- `_gz_reset_poses()`: 현재 스테이지의 거리 범위에서 타겟 주변 랜덤 스폰
- `_update_curriculum_settings()`: 스테이지 전환 시 `max_steps`, `use_vision` 자동 갱신
- `_check_curriculum_advance()`: 최근 N 에피소드 성공률 계산 후 전환 조건 확인
- `step()`: 에피소드 종료 시 성공 여부 기록 및 전환 체크
- `_get_obs()`: Stage 3에서 `rel_x`, `rel_y`를 0.0으로 마스킹

**`ros2_ws/src/rl_navigation/rl_navigation/train_sac.py`**
- `CurriculumCallback` 클래스 추가: 스테이지 전환을 WandB에 기록 (`curriculum/stage`, `curriculum/name`)

**`ros2_ws/src/rl_navigation/config/hyperparams.yaml`**
```yaml
# 새로 추가된 섹션
curriculum:
  enabled: true
  stages:
    - name: "close"
      spawn_distance_min: 3.0
      spawn_distance_max: 8.0
      max_steps: 150
      use_vision: false
      mask_ground_truth: false
      advance_threshold: 0.6
      advance_window: 50
    # ... (medium, full, vision 단계)
```

---

## 3. YOLO 비전 에이전트 연결

### 기존 동작
`use_vision: false`로 고정. 관측값의 픽셀 좌표(`pixel_u`, `pixel_v`)는 항상 0, confidence는 합성값 1.0 사용. 에이전트가 ground truth 상대 위치(`rel_x`, `rel_y`)에만 의존.

관측 공간: **15차원**

### 변경 후 동작
실제 YOLO 탐지 결과를 관측값에 포함. 커리큘럼 Stage 3에서 ground truth를 마스킹하여 **완전한 비전 기반 모델** 구현.

관측 공간: **17차원** (+bbox 크기 2개)

| 인덱스 | 내용 | 기존 | 변경 후 |
|-------|------|------|--------|
| 0~2 | position ENU | 동일 | 동일 |
| 3~5 | velocity ENU | 동일 | 동일 |
| 6~8 | angular velocity | 동일 | 동일 |
| 9~11 | pixel_u, pixel_v, confidence | 합성값 | **실제 YOLO 값** |
| 12 | payload_attached | 동일 | 동일 |
| 13~14 | rel_x, rel_y (ground truth) | 항상 사용 | **Stage 3에서 0으로 마스킹** |
| **15** | **bbox_width_norm** | **없음** | **신규** (bbox_w/640) |
| **16** | **bbox_height_norm** | **없음** | **신규** (bbox_h/480) |

bbox 크기는 거리 프록시로 활용: bbox가 클수록 타겟이 가까움.

### 수정된 파일

**`ros2_ws/src/rl_navigation/rl_navigation/drone_drop_env.py`**
- 파일 상단: `vision_detection.msg.DetectionResult` 조건부 import 추가
- `_RLBridgeNode.__init__()`: `/vision/detections` 토픽 구독 추가
- `_RLBridgeNode._on_detection()`: bbox 크기 콜백 추가
- `observation_space`: shape `(15,)` → `(17,)`로 변경
- `_get_obs()`: `bbox_w_norm`, `bbox_h_norm` 2개 특성 추가
- `_start_infra()` bridge YAML: 카메라 이미지 브리지 항목 추가
- `_start_infra()`: YOLO node(`xmarker_detector`)를 인프라 레벨에서 실행 (에피소드마다 재시작 불필요)

**`ros2_ws/src/rl_navigation/package.xml`**
```xml
<!-- 추가 -->
<depend>vision_detection</depend>
```

> **⚠️ 주의:** obs 15→17 변경으로 **기존 체크포인트와 호환 불가**. 반드시 fresh start 필요.

---

## 4. Optuna 하이퍼파라미터 최적화

### 기존 동작
SAC 하이퍼파라미터(`learning_rate`, `buffer_size` 등)가 `hyperparams.yaml`에 수동으로 고정되어 있었음.

### 변경 후 동작
Optuna TPE sampler + MedianPruner로 SAC 파라미터를 자동 탐색. SQLite DB로 Spot VM 중단 후에도 연구 재개 가능.

**탐색 범위:**

| 파라미터 | 범위 |
|---------|------|
| learning_rate | 1e-5 ~ 1e-3 (log scale) |
| buffer_size | [50K, 100K, 200K] |
| batch_size | [128, 256, 512] |
| gamma | 0.95 ~ 0.999 |
| tau | 0.001 ~ 0.02 |
| net_arch | small[128,128] / medium[256,256] / large[256,256,128] |

**실행 방법:**
```bash
ros2 run rl_navigation tune_optuna
# 옵션
ros2 run rl_navigation tune_optuna --n-trials 50 --trial-budget 50000
```

최적 파라미터는 `best_hyperparams.yaml`로 자동 저장됨.

### 추가된 파일

**`ros2_ws/src/rl_navigation/rl_navigation/tune_optuna.py`** (신규)
- `objective()`: 각 trial에서 SAC 생성 → 학습 → success_rate 반환
- `OptunaEvalCallback`: 중간 pruning 지원, WandB 실시간 기록
- `MedianPruner`: 하위 성능 trial 조기 종료
- Spot VM 재시작 후 `load_if_exists=True`로 연구 이어서 진행

**`ros2_ws/src/rl_navigation/rl_navigation/drone_drop_env.py`**
- `close(keep_infra=False)` 파라미터 추가: trial 간 Gazebo 재사용
- `set_reward_weights(**kwargs)`: 런타임 보상 가중치 오버라이드

**`ros2_ws/src/rl_navigation/setup.py`**
```python
'tune_optuna = rl_navigation.tune_optuna:main',  # 엔트리포인트 추가
```

**`ros2_ws/src/rl_navigation/config/hyperparams.yaml`**
```yaml
optuna:
  trial_budget: 50000
  n_trials: 50
  eval_freq: 5000
  storage: "sqlite:///optuna_drone.db"
  restart_infra_every: 10  # 10 trial마다 Gazebo 재시작
```

---

## 전체 변경 파일 목록

| 파일 | 변경 종류 | 관련 기능 |
|------|---------|---------|
| `drone_drop_env.py` | 수정 | 1, 2, 3, 4 |
| `mission_manager_node.py` | 수정 | 1 |
| `hyperparams.yaml` | 수정 | 1, 2, 3, 4 |
| `train_sac.py` | 수정 | 2 |
| `tune_optuna.py` | **신규** | 4 |
| `setup.py` | 수정 | 4 |
| `package.xml` | 수정 | 3 |
| `CLAUDE.md` | 수정 | 문서화 |
| `RL_Project_Log.md` | 수정 | 문서화 |

---

## 학습 시작 전 주의사항

1. **obs 17차원 변경** → 기존 체크포인트 사용 불가, **fresh start 필수**
2. 커리큘럼 Stage 3(Vision) 진입 전에는 YOLO 비전 노드가 실행 중이어야 함 (인프라 레벨에서 자동 실행)
3. Optuna는 커리큘럼 학습으로 baseline 성능 확보 후 실행 권장

## 빌드

```bash
# 컨테이너 내부에서
cd /workspace/ros2_ws
source /opt/ros/humble/setup.bash && source /root/ros2_ws/install/setup.bash
colcon build --packages-select rl_navigation mission_manager
source install/setup.bash
```
