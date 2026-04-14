# CLAUDE.md

---

## Obsidian 연구 비서 규칙 (MANDATORY)

모든 연구 기록·에러 해결·실험 결과는 `notes/` 폴더에 마크다운으로 작성한다.

```
notes/
├── 00_index.md          # 전체 대시보드 (항상 최신 유지)
├── daily/               # 하루 마감 연구 일지 ← NEW
├── research/            # 이론·설계·아키텍처
├── experiments/         # 학습 실험 (WandB 연동)
├── errors/              # 에러 해결 기록
├── sessions/            # 세션 작업 상세 기록
└── references/          # 논문·문서
```

**파일 네이밍:** `research/{topic}.md` / `experiments/exp_{NNN}_{run_id}_{title}.md` / `errors/err_{YYYYMMDD}_{slug}.md` / `sessions/session_{YYYY-MM-DD}.md` / `daily/daily_{YYYY-MM-DD}.md`

**YAML frontmatter 필수:** `date`, `tags`, `status`, `type` (experiments는 `wandb_run` 추가)

**수식:** 모든 수식은 LaTeX (`$...$` / `$$...$$`)

**세션 종료 전 자동 수행:**
1. `notes/sessions/session_{YYYY-MM-DD}.md` 생성/업데이트
2. `notes/00_index.md` 현재 상태 업데이트
3. 에러 해결 → `notes/errors/` 기록
4. 실험 시작/완료 → `notes/experiments/` 기록

**VM 종료 전 자동 수행 (MANDATORY):**

`notes/daily/daily_{YYYY-MM-DD}.md` 를 생성/업데이트. 양식은 `notes/daily/_template.md` 참고.

필수 섹션:
- **오늘 한 일** — 수행한 작업 전체 목록
- **주요 결정 & 발견** — 왜 그런 판단을 내렸는지 이유 포함
- **코드 변경 사항** — 파일명 + 변경 내용 표
- **문제 & 해결** — 에러 발생 여부, 해결 방법, 미해결 상태 명시
- **내일 할 일** — 다음 세션 시작 시 가장 먼저 할 것 (우선순위 순)
- **관련 노트** — 오늘 작성/수정된 notes/ 파일 wikilink

> 이 일지는 다음 세션에서 Claude가 "어제 무엇을 했는지" 파악하는 **핵심 컨텍스트**다.
> 세션 시작 시 가장 최근 `daily/` 파일을 먼저 읽어 맥락을 복원한다.

---

## Development Environment

개발은 **Docker 컨테이너 내부**에서만. 호스트 VM(`/opt/drone-bombard`)은 git 작업 전용.

```bash
# 이미지 pull
docker pull us-central1-docker.pkg.dev/charming-league-481306-d8/drone-bombard/drone-bombard:latest

# 컨테이너 최초 실행
xhost +local:docker
docker run -itd --gpus all --net=host --privileged --ipc=host \
  --name drone-bombard-harmonic \
  --env="DISPLAY=$DISPLAY" --env="QT_X11_NO_MITSHM=1" --env="NVIDIA_DRIVER_CAPABILITIES=all" \
  --volume="/tmp/.X11-unix:/tmp/.X11-unix:rw" \
  -v /opt/drone-bombard/Drone-Bombard-Simulation/ros2_ws:/workspace/ros2_ws \
  -v /opt/drone-bombard/Drone-Bombard-Simulation/gazebo_models:/workspace/gazebo_models \
  -v ~/.cache:/root/.cache \
  --log-driver=json-file --log-opt max-size=10m --log-opt max-file=3 \
  us-central1-docker.pkg.dev/charming-league-481306-d8/drone-bombard/drone-bombard:latest /bin/bash

# 기존 컨테이너 재접속
xhost +local:docker && docker start -ai drone-bombard-harmonic
```

### 빌드 (컨테이너 내부)

```bash
cd /workspace/ros2_ws && colcon build && source install/setup.bash
colcon build --packages-select <package_name> && source install/setup.bash
```

---

## RL Training (Method A, Self-Managed)

`DroneDropEnv._start_infra()`가 모든 인프라를 내부 관리. 별도 인프라 시작 불필요.

> **⚠️ source 순서 필수:** `/root/ros2_ws/install/setup.bash` → `/workspace/ros2_ws/install/setup.bash`
> 순서 틀리면 `px4_msgs` import 에러로 에피소드 노드 silent crash.

```bash
# Fresh start (보상 함수 변경 후)
docker exec -d drone-bombard-harmonic bash -c \
  "source /opt/ros/humble/setup.bash && \
   source /root/ros2_ws/install/setup.bash && \
   source /workspace/ros2_ws/install/setup.bash && \
   export GZ_SIM_RESOURCE_PATH=/workspace/gazebo_models:/opt/PX4-Autopilot/Tools/simulation/gz/models:/opt/PX4-Autopilot/Tools/simulation/gz/worlds && \
   cd /workspace/ros2_ws && ros2 run rl_navigation train_sac \
   > /tmp/production_train.log 2>&1"

# Resume from checkpoint
docker exec -d drone-bombard-harmonic bash -c \
  "source /opt/ros/humble/setup.bash && \
   source /root/ros2_ws/install/setup.bash && \
   source /workspace/ros2_ws/install/setup.bash && \
   export GZ_SIM_RESOURCE_PATH=/workspace/gazebo_models:/opt/PX4-Autopilot/Tools/simulation/gz/models:/opt/PX4-Autopilot/Tools/simulation/gz/worlds && \
   cd /workspace/ros2_ws && ros2 run rl_navigation train_sac \
     --resume /workspace/ros2_ws/rl_checkpoints/sac_drop_preempt.zip \
   > /tmp/production_train.log 2>&1"

# 모니터
docker exec drone-bombard-harmonic bash -c "tail -f /tmp/production_train.log"

# 정상 종료 (→ sac_drop_preempt.zip + _replay.pkl 저장)
docker exec drone-bombard-harmonic bash -c "pkill -SIGTERM -f train_sac"
```

### Quick-Start After VM Preemption

```bash
xhost +local:docker && docker start -ai drone-bombard-harmonic
docker exec drone-bombard-harmonic bash /workspace/ros2_ws/start_infra_clean.sh
# 코드 변경 시 빌드 후 학습 재시작
```

---

## Auto-Logging & Git Sync (MANDATORY)

모든 주요 작업·코드 수정·세션 종료 전 Claude는 **자동으로**:

1. `RL_Project_Log.md` 4-section 구조로 업데이트 (허락 불필요)
2. Git push:

```bash
git add .
git commit -m "Auto-sync: [작업 요약]"
git push origin main
```

### RL_Project_Log.md 구조

```
# 1. Current State     — 현재 보상 공식, 핵심 하이퍼파라미터, 체크포인트 경로
# 2. Recent Progress   — 완료된 작업 (최근 5개 이내)
# 3. Remaining Tasks   — [ ] 체크리스트
# 4. Training History  — 추가만 가능 (날짜 | Run ID | Steps | 요약)
```

---

## RL 핵심 규칙 (MANDATORY)

상세 내용: `notes/research/rl_rules.md`

1. **Fail-Fast:** 코드 변경 후 dry-run (2–3 에피소드) 먼저. 성공 확인 후 full training.
2. **병렬화:** `num_envs=1` 고정 (Gazebo lockstep 병목). SubprocVecEnv 시 15 s 스태거.
3. **보상 공식 변경 → 항상 Fresh Start.** Replay buffer 재사용 금지.
4. **거리 보상:** 지수 포텐셜 사용 전 $e^{-k_1 d_{max}} > 10^{-4}$ 확인. 권장: 선형 보상.
5. **체크포인트:** 물리 폭발 후 preempt 재개 금지 → rolling checkpoint 사용.
6. **WandB:** 첫 롤아웃 후 `env/mean_rew_dist ≠ 0` 확인. 물리 글리치 값은 별도 키(`glitch_d_xy`) 사용.

### Known Failure Modes

| 증상 | 원인 | 해결 |
|------|------|------|
| `mean_rew_dist = 0` | 지수 포텐셜 포화 | 선형 보상으로 전환 |
| `mean_d_xy` → 1e11 | Gazebo ODE 폭발 | 3중 방어 레이어 (notes/errors/) |
| CRUISE 타임아웃 반복 | PX4 arm race | `reset()` 재시도; fps 하락 확인 |
| fps 급감 | CRUISE 65 s 대기 / ODE 크래시 | 로그에서 "Timed out waiting for CRUISE" |

---

## 시스템 아키텍처 & 상세 레퍼런스

→ `notes/research/system_overview.md` (패키지, 토픽, 좌표계, 브리지 설정, YOLO)  
→ `notes/research/reward_design.md` (보상 함수 LaTeX 상세)  
→ `notes/research/rl_rules.md` (RL 규칙 상세, WandB 메트릭 레퍼런스)  
→ `notes/research/architecture.md` (Method A 아키텍처)
