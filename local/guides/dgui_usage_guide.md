================================================================================
 dgui Usage Guide — 학습된 SAC 모델의 GUI 시각 평가
================================================================================
최종 갱신 : 2026-06-22 (모든 처방 적용 완료 + camera mode 추가)
관련     : local/scripts/evaluate_gui.py, local/eval_config.yaml,
           ros2_ws/eval_models/, local/eval_logs/
도구 자체 history (D-1, M-1, B, settle, H, camera) : local/archive/notes_2026-06-22_27/research/dgui_tool.md


================================================================================
 1. 빠른 시작
================================================================================

  1-1. alias 활성화 (한 번만):

    $ source ~/.bashrc

    (또는 새 terminal 열기 — 자동 적용)

  1-2. interactive 모드 (가장 단순):

    $ dgui

    출력:
      📦 Available models in eval_models/:
        1. v8_best_step157201_err0.07m.zip
        2. v8_final_step303801.zip
        3. v8_peak_step217040_err0.87m.zip
        4. v9a_preempt_step313k.zip
      Select model (1-4): 3

      📷 Camera mode:
        1. follow (default) — drone 자동 추적
        2. origin — drone 위치로 카메라 한 번 이동
        3. default — 기본 Gazebo 카메라
      Select camera (1-3, Enter for default 'follow'): _

      → 평가 시작 (5 ep, follow camera, GUI 자동 launch)

  1-3. 직접 호출:

    $ dgui v8_peak_step217040_err0.87m                     # 5 ep, default camera
    $ dgui v8_peak_step217040_err0.87m --episodes 1        # 1 ep 만
    $ dgui v8_peak_step217040_err0.87m --camera follow     # camera 명시
    $ dgui v8_peak_step217040_err0.87m --no-gui            # GUI 안 띄움 (CI/통계)
    $ dgui v8_peak_step217040_err0.87m --no-watcher        # GUI watcher 비활성
    $ dgui v8_peak_step217040_err0.87m --keep-alive        # 평가 후 sim 유지 (inspection)
    $ dgui v8_peak_step217040_err0.87m --stochastic        # stochastic policy
    $ dgui --help                                          # 전체 옵션


================================================================================
 2. 결과 파일
================================================================================

  자동 저장 위치:
    local/eval_logs/eval_<timestamp>_<model_name>.json

  예시:
    local/eval_logs/eval_2026-06-27T00-36-50_v9a_preempt_step313k.json

  형식 (json):
    {
      "episodes": 5,
      "drops": 5,
      "success_2m": 4,
      "jackpot_03m": 0,
      "mean_err": 1.888,
      "min_err": 1.649,
      "max_err": 2.216,
      "results": [
        {"ep": 1, "steps": 42, "trigger": "auto", "drop_error": 1.79, ...},
        ...
      ],
      "model": "v9a_preempt_step313k.zip",
      "timestamp": "2026-06-27T00-36-50",
      "deterministic": true
    }


================================================================================
 3. 모델 추가
================================================================================

  3-1. 모델 zip 파일 cp:

    $ cp <model.zip> ros2_ws/eval_models/

    (host = container bind mount 라 자동 sync)

  3-2. 이름 규칙 (권장):

    {round}_{type}_step{N}_err{X}m.zip

    예시:
      v8_peak_step217040_err0.87m.zip      ← peak window best
      v8_best_step157201_err0.07m.zip      ← global best single drop
      v8_final_step303801.zip               ← 학습 종료 시점
      v9a_preempt_step313k.zip              ← SIGTERM preempt

  3-3. README.md 갱신 (선택):

    ros2_ws/eval_models/README.md 의 표에 행 추가


================================================================================
 4. config 변경 (local/eval_config.yaml)
================================================================================

  주요 키:

    container: drone-bombard-harmonic
    default_episodes: 5
    deterministic: true                  # false 면 stochastic

    # 시간 관련
    gui_wait_time: 30                    # EP1 시작 전 GUI 안정화 대기
    gui_relaunch_settle: 5               # 매 _kill_infra 후 GUI relaunch 대기
    gui_watcher_enabled: true            # GUI 죽으면 자동 부활
    gui_watcher_interval: 2.0            # watcher poll 간격
    gui_watcher_backoff: 25.0            # H 처방 — _kill_infra → _start_infra 사이 빈 launch 방지
    reset_settle_sleep: 5.0              # reset 후 PX4 EKF settle (G 처방 2s 폐기, 5s 복원)

    # 카메라
    camera_mode: follow                  # follow | origin | default
    camera_settle_secs: 3.0
    camera_target_model: x500_bombard_0

    # 평가 후 동작
    keep_sim_alive: false                # true 면 평가 후 sim 유지

    # 경로
    container_models_path: /workspace/ros2_ws/eval_models
    host_models_path: /home/juns/Drone-Bombard-Simulation/ros2_ws/eval_models
    results_dir: /home/juns/Drone-Bombard-Simulation/local/eval_logs


================================================================================
 5. 핵심 처방 (도구 내부 동작)
================================================================================

  도구 의 단계적 처방 history — 자세히: local/archive/notes_2026-06-22_27/research/dgui_tool.md

  D-1 (강제 _kill_infra 보장):
    - 매 ep 직전 _consecutive_fast_resets 를 max 로 set
    - reset() 안에서 _forced_restart=True 자동 trigger
    - 학습 D1 path (매 drop 후 _kill_infra) 그대로
    - drop 0 인 ep 도 fresh PX4 보장 → DetachableJoint silent fail 우회

  M-1 (tmux 우회):
    - subprocess.Popen + stdout=PIPE 직접 stream
    - tmux capture-pane 의 wrap 문제 회피

  B (env GZ_SERVER_READY marker):
    - drone_drop_env.py 의 reset() 안 _start_infra() 직후 print marker
    - host 가 marker 보면 즉시 launch_gui
    - → spawn 부터 ARM → TAKEOFF → CRUISE 모두 보임

  settle 5s:
    - PX4 EKF settle time (G 처방 2s 시도 → variability 큼 → 5s 복원)

  H 처방 (watcher backoff 25s):
    - launch 후 25s 동안 watcher skip
    - _kill_infra → _start_infra 사이 server 없는 시점에 GUI 빈 launch 반복 방지

  camera mode:
    - follow: gz service /gui/follow → drone 자동 추적
    - origin: gz service /gui/move_to → drone 위치로 한 번 이동
    - default: SDF 의 initial pose (5 -5 5 0 0.615 2.356 — SE 방향 drone 쪽 looking)


================================================================================
 6. 알려진 issue / 주의 사항
================================================================================

  6-1. ang_vel callback fix prerequisite:
    - PX4 dds_topics.yaml 의 vehicle_angular_velocity 가 uncomment 되어 있어야
    - 그렇지 않으면 obs[6:9] = 0 (Issue #024 참조)

  6-2. install/share sync:
    - drone_drop_env.py 또는 hyperparams.yaml 변경 시 install/share 도 sync 해야
    - cp 또는 colcon build (CLAUDE memory: ROS2 install/share cache trap)

  6-3. SIGTERM 만 preempt save:
    - 학습 중 Ctrl+C (SIGINT) 쓰면 모델 + replay buffer 손실
    - 반드시 `kill -TERM <pid>` 사용

  6-4. container 종료 시:
    - 학습 process (PID 259 = 실제 train_sac python) 에 SIGTERM
    - PID 258 (ros2 launcher) 에는 effect 없음

  6-5. dgui interactive 모드:
    - container 가 살아있어야 list_models() 가 작동
    - 죽었으면 docker start drone-bombard-harmonic


================================================================================
 7. 검증된 평가 결과 (참고)
================================================================================

  v8_peak_step217040_err0.87m (5 ep):
    success 5/5 = 100%, mean 1.72m, max ang_vel 2.5 rad/s

  v8_best_step157201_err0.07m (5 ep):
    success 5/5 = 100%, mean 1.82m

  v9a_preempt_step313k (5 ep):
    success 4/5 = 80%, mean 1.89m, max ang_vel 2.10 rad/s (-16%)


================================================================================
 8. 관련 문서
================================================================================

  - local/scripts/evaluate_gui.py — 소스
  - local/eval_config.yaml — 설정
  - local/eval_logs/ — 결과
  - ros2_ws/eval_models/README.md — 모델 카탈로그
  - local/archive/notes_2026-06-22_27/research/dgui_tool.md — 처방 history (D-1, M-1, B, settle, H, camera)
  - local/archive/notes_2026-06-22_27/errors/err_20260622_ang_vel_callback.md — Issue #024 의 fix 자세히
