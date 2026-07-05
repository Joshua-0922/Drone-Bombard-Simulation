---
date: 2026-06-21
tags: [research, tool, evaluation, gui, dgui, gazebo, infrastructure]
status: completed
type: research
---

# `dgui` — GUI 학습 모델 평가 도구

학습된 SAC 모델을 GUI 로 시각 검증하기 위한 wrapper. v8 (`phase1_redux_v8_no_invalid_penalty`) 완료 직후 개발 시작 (2026-06-21).

## 위치

| 파일 | 위치 |
|---|---|
| **main script** | `local/scripts/evaluate_gui.py` |
| **config** | `local/eval_config.yaml` |
| **모델 폴더** | `ros2_ws/eval_models/` (host = container bind mount) |
| **결과 저장** | `local/eval_logs/eval_<ts>_<model>.json` |
| **alias** | `~/.bashrc` 의 `alias dgui='python3 .../evaluate_gui.py'` |
| **README** | `ros2_ws/eval_models/README.md` |

## 사용법

```bash
# (1) 인터랙티브 — 모델 + 카메라 묻기
dgui

# (2) 직접 지정
dgui v8_peak_step217040_err0.87m

# (3) 옵션
dgui v8_peak_step217040_err0.87m --episodes 5 --camera follow
dgui --no-gui                   # GUI 안 띄움 (통계용)
dgui --no-watcher               # GUI watcher 비활성
dgui --keep-alive               # 평가 후 sim 유지 (inspection)
dgui --stochastic               # stochastic policy
dgui --help                     # 전체 옵션
```

## 핵심 처방 history

도구 개발 중 발견한 여러 issue 와 fix.

### D-1 (강제 `_kill_infra` 보장)

- **문제**: 학습은 매 drop 후 `_kill_infra` (38s fresh PX4) 보장. evaluate 시 drop 0 인 경우 — vel 잔존 + payload reattach silent fail (DetachableJoint plugin).
- **fix**: inline script 에서 매 ep 직전 `env._consecutive_fast_resets = env._cfg_max_consecutive_fast_resets` → `env.reset()` 안에서 `_forced_restart=True` 자동 trigger → 학습 D1 path 그대로.

```python
# inline script:
env._consecutive_fast_resets = env._cfg_max_consecutive_fast_resets
obs, _ = env.reset()   # 안에서 _obs_ready.clear + _kill_infra + _start_infra
```

### M-1 (tmux 우회)

- **문제**: tmux session 의 capture-pane 결과가 wrap 되어 trace 잘림. 실제 hang 아닌데 hang 으로 추정함.
- **fix**: tmux 대신 `subprocess.Popen` + `stdout=PIPE` 으로 직접 stream. wrap 없음.

```python
EVAL_PROC = subprocess.Popen(
    ["docker", "exec", CONTAINER, "bash", "-lc", inner_cmd],
    stdout=subprocess.PIPE, stderr=subprocess.STDOUT,
    text=True, bufsize=1,
)
# tail_results: line = EVAL_PROC.stdout.readline()
```

### B 처방 (env reset 안 GZ_SERVER_READY marker)

- **문제**: GUI 가 cruise 도달 후에야 launch → spawn / ARM / TAKEOFF 못 봄.
- **fix**: `drone_drop_env.py` 의 reset() 안 `_start_infra()` 직후 print marker. host 가 marker 보면 즉시 launch_gui.

```python
# drone_drop_env.py reset() (line ~604):
self._start_infra()
print(f'[GZ_SERVER_READY] reset_count={self._reset_count + 1}', flush=True)
```

```python
# evaluate_gui.py host:
m = re.search(r"\[GZ_SERVER_READY\] reset_count=(\d+)", line)
if m and m.group(1) not in seen:
    launch_gui()
```

→ spawn 부터 cruise 까지 다 보임.

### settle sleep 5s

- **문제**: settle 2s (G 처방 시도) 로 정책 헛돔. 5 ep 평가 success 33%.
- **fix**: 5s 복원. 정책의 PX4 EKF settle 확보.

```python
obs, _ = env.reset()
time.sleep(settle_sleep)   # 5.0s
# step loop ...
```

### H 처방 (watcher backoff 25s)

- **문제**: GUI watcher (2s 간격) 가 _kill_infra → _start_infra 사이 server 없을 때 GUI 죽은 거 감지 → relaunch → 또 죽음 → 또 relaunch (반복).
- **fix**: launch 직후 25s 동안 watcher skip. server 시작 대기.

```python
def gui_watcher_loop():
    while WATCHER_RUNNING:
        time.sleep(CFG["gui_watcher_interval"])
        if GUI_PROC.poll() is not None:
            elapsed = time.monotonic() - GUI_LAST_LAUNCH
            if elapsed < backoff:   # 25s
                continue
            launch_gui()
```

### E 처방 (강제 set 제거) — 폐기

- 시도: 매 ep 강제 `_kill_infra` 제거. drop 없는 ep 후 fast reset.
- **문제**: fast reset 의 `_gz_reset_poses` 가 비행 중 drone teleport 못함 → spawn drift 누적 (EP4 spawn=(-1.76, 2.14, 7.25)).
- 폐기. 매 ep `_kill_infra` 강제 유지.

### Camera mode (follow / origin / default)

3 가지 모드 + interactive 묻기:

```python
def apply_camera_mode(mode):
    if mode == "follow":
        docker_exec("gz service -s /gui/follow --req 'data: \"x500_bombard_0\"'")
    elif mode == "origin":
        docker_exec("gz service -s /gui/move_to --req 'data: \"x500_bombard_0\"'")
    # default: SDF 의 initial pose 만 적용
```

추가: SDF 의 default camera_pose 변경 (`gazebo_models/worlds/x_marker_world.sdf`):
- 기존: `15 6 5 0 0.7 2.35` (NW 방향, drone 안 보임)
- 새로: `5 -5 5 0 0.615 2.356` (SE 방향, drone spawn (0, 0) 쪽 looking)

### settle_sleep + interactive camera 묻기

interactive 모드 (`dgui` 단독):
1. 모델 list 보고 번호 선택
2. 카메라 모드 묻기 (follow / origin / default, Enter = config default)
3. 평가 시작

```python
def interactive_camera():
    options = [
        ("follow", "drone 자동 추적"),
        ("origin", "drone 위치로 카메라 한 번 이동"),
        ("default", "기본 Gazebo 카메라"),
    ]
    # ...
```

## eval_config.yaml 키 정리

```yaml
container: drone-bombard-harmonic
default_episodes: 5
gui_wait_time: 30              # EP1 시작 전 GUI 안정화
gui_relaunch_settle: 5
gui_watcher_enabled: true
gui_watcher_interval: 2.0
gui_watcher_backoff: 25.0      # H 처방
reset_settle_sleep: 5.0        # PX4 EKF settle
camera_mode: follow            # follow | origin | default
camera_settle_secs: 3.0
camera_target_model: x500_bombard_0
keep_sim_alive: false          # 평가 후 sim 자동 종료
deterministic: true
tail_min: 30
container_models_path: /workspace/ros2_ws/eval_models
host_models_path: /home/juns/Drone-Bombard-Simulation/ros2_ws/eval_models
results_dir: /home/juns/Drone-Bombard-Simulation/local/eval_logs
display: ":1"
```

## eval_models/ 구조

```
ros2_ws/eval_models/             # host 와 container 자동 sync (bind mount)
├── README.md                     # 모델 설명 + 사용법
├── v8_peak_step217040_err0.87m.zip    # peak window best (success 80%)
├── v8_best_step157201_err0.07m.zip    # global best drop (7cm)
├── v8_final_step303801.zip             # 학습 종료 시점
└── v9a_preempt_step313k.zip            # v9a 학습 중간 (SIGTERM stop)
```

## 결과 json 형식

```json
{
  "episodes": 5,
  "drops": 5,
  "success_2m": 4,
  "jackpot_03m": 0,
  "mean_err": 1.888,
  "min_err": 1.649,
  "max_err": 2.216,
  "results": [
    {"ep": 1, "steps": 42, "trigger": "auto", "drop_error": 1.79, ...}
  ],
  "model": "v9a_preempt_step313k.zip",
  "timestamp": "2026-06-27T00-36-50",
  "deterministic": true
}
```

## 검증 결과 (v8)

```
v8_peak (5 ep): 5/5 = 100%, mean 1.72m
v8_best (5 ep): 5/5 = 100%, mean 1.82m
```

→ dgui 가 학습 환경 100% 재현 (D-1 처방). 학습 결과 직접 검증 가능.

## 관련 노트

- [[errors/err_20260622_ang_vel_callback]] — dgui 평가 중 ang_vel 0 발견
- [[experiments/exp_006_zjexq20k_v9a_payload_dist_angaccel]] — dgui 로 v9a 평가
- [[research/rl_rules]] — D-1, B 처방 규칙 추가
