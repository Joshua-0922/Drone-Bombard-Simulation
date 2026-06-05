#!/bin/bash
# 통합 GUI evaluate 도구
# - 사용 가능한 모델을 자동 검색해서 메뉴로 보여줌
# - episode 수 선택
# - 환경 정리 → eval 실행 → GUI 띄움 → GUI watcher 자동 재실행
#
# 사용:
#   bash local/tools/eval_gui.sh
#   또는 alias:
#   alias train-eval='bash /home/juns/Drone-Bombard-Simulation/local/tools/eval_gui.sh'

set -u

CONTAINER="drone-bombard-harmonic"
CONFIG_PATH="/workspace/ros2_ws/install/rl_navigation/share/rl_navigation/config/hyperparams.yaml"
EVAL_SCRIPT_PATH="/tmp/eval_inner.py"
EVAL_LOG_PATH="/tmp/eval_run.log"
WATCHER_PID=""

# ---------- 색 ----------
RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
BLUE='\033[0;34m'
NC='\033[0m'

# ---------- 컨테이너 확인 ----------
check_container() {
  if ! docker ps --format '{{.Names}}' | grep -q "^${CONTAINER}\$"; then
    echo -e "${RED}[ERROR]${NC} 컨테이너 ${CONTAINER} 가 실행 중이 아닙니다."
    echo "다음 명령으로 시작:"
    echo "  docker start ${CONTAINER}"
    exit 1
  fi
}

# ---------- 모델 자동 검색 ----------
discover_models() {
  # docker exec 로 컨테이너 안 파일 시스템 검색
  docker exec "$CONTAINER" bash -c '
    # SuccessReplay (학습 중 자동 저장된 success 모델)
    find /workspace/ros2_ws/success_replay -maxdepth 2 -type f \
      \( -name "success_step*" -not -name "*.zip" \) 2>/dev/null \
      | sed "s|^|SUCCESS|"

    # best_drops 폴더
    find /workspace/ros2_ws/rl_checkpoints/drop_episodes/best_drops -maxdepth 1 -type f \
      -name "best_err*" 2>/dev/null \
      | sed "s|^|BEST   |"

    # archive (수동 보존된 milestone/preempt)
    find /workspace/ros2_ws/rl_checkpoints/archive -maxdepth 3 -type f \
      \( -name "sac_drop_*.zip" -o -name "*_preempt.zip" \) 2>/dev/null \
      | sed "s|^|ARCHIVE|"

    # 현재 rolling checkpoints
    find /workspace/ros2_ws/rl_checkpoints -maxdepth 1 -type f \
      -name "sac_drop_*_steps.zip" 2>/dev/null \
      | sed "s|^|ROLLING|"
  '
}

# ---------- 모델 표시 + 선택 ----------
select_model() {
  echo -e "${BLUE}사용 가능한 모델 검색 중...${NC}" >&2
  local raw
  raw=$(discover_models)

  if [ -z "$raw" ]; then
    echo -e "${RED}[ERROR]${NC} 사용 가능한 모델을 찾지 못했습니다." >&2
    exit 1
  fi

  # 카테고리별로 분류 + 정렬
  # SUCCESS: err 작은 순
  # BEST: err 작은 순
  # ARCHIVE/ROLLING: step 큰 순 (최근)

  local items=()

  # SUCCESS: 파일명에서 err 추출, 작은 순 정렬
  while IFS= read -r line; do
    [ -z "$line" ] && continue
    items+=("$line")
  done < <(echo "$raw" | grep "^SUCCESS" | sed 's/^SUCCESS//' | awk -F'_err' '{print $2 "\t" $0}' | sort -n | cut -f2 | sed 's|^|SUCCESS|')

  while IFS= read -r line; do
    [ -z "$line" ] && continue
    items+=("$line")
  done < <(echo "$raw" | grep "^BEST" | sed 's/^BEST//' | awk -F'_err' '{print $2 "\t" $0}' | sort -n | cut -f2 | sed 's|^|BEST|')

  while IFS= read -r line; do
    [ -z "$line" ] && continue
    items+=("$line")
  done < <(echo "$raw" | grep -E "^(ARCHIVE|ROLLING)" | awk -F'_' '{print $0}' | sort -r)

  # 출력
  echo "" >&2
  echo -e "${GREEN}=== 사용 가능한 모델 ===${NC}" >&2
  local i=1
  for item in "${items[@]}"; do
    local category="${item:0:7}"
    local path="${item:7}"
    local name
    name=$(basename "$path")
    case "$category" in
      SUCCESS) printf "  %2d. [success] %s\n" "$i" "$name" >&2 ;;
      BEST   ) printf "  %2d. [best   ] %s\n" "$i" "$name" >&2 ;;
      ARCHIVE) printf "  %2d. [archive] %s\n" "$i" "$name" >&2 ;;
      ROLLING) printf "  %2d. [rolling] %s\n" "$i" "$name" >&2 ;;
    esac
    i=$((i+1))
  done

  echo "" >&2
  local selection
  while true; do
    read -r -p "$(echo -e "${YELLOW}모델 번호를 선택: ${NC}")" selection
    if [[ "$selection" =~ ^[0-9]+$ ]] && [ "$selection" -ge 1 ] && [ "$selection" -le "${#items[@]}" ]; then
      break
    fi
    echo "1~${#items[@]} 사이 숫자를 입력해주세요." >&2
  done

  # 선택된 항목의 path 반환
  local idx=$((selection-1))
  echo "${items[$idx]:7}"
}

# ---------- Episode 수 입력 ----------
get_episodes() {
  local n
  read -r -p "$(echo -e "${YELLOW}Episode 수 (default 5): ${NC}")" n
  echo "${n:-5}"
}

# ---------- 환경 정리 ----------
clean_infra() {
  echo -e "${BLUE}환경 정리 중...${NC}"
  docker exec "$CONTAINER" bash -c "
    pkill -9 -f 'gz sim' 2>/dev/null
    pkill -9 -f 'bin/px4' 2>/dev/null
    pkill -9 -f 'MicroXRCEAgent' 2>/dev/null
    pkill -9 -f 'parameter_bridge' 2>/dev/null
    pkill -9 -f 'mission_manager' 2>/dev/null
    pkill -9 -f 'drone_controller' 2>/dev/null
    pkill -9 -f 'drop_calculator' 2>/dev/null
    pkill -9 -f 'eval_inner' 2>/dev/null
    rm -f /tmp/drone_env_gz_ready /tmp/x_marker_world_rl_*.sdf 2>/dev/null
    rm -f $EVAL_LOG_PATH 2>/dev/null
    sleep 2
  " >/dev/null 2>&1 || true
}

# ---------- Eval 스크립트 컨테이너 안에 작성 ----------
prepare_eval_script() {
  local model_path="$1"
  local episodes="$2"

  # SB3 .zip 자동 추가 문제 회피
  local clean_path="$model_path"
  if [[ "$model_path" == *.zip ]]; then
    clean_path="${model_path%.zip}"
  fi

  # v1.1 fix: 이전에는 docker exec bash -c "cat > ... << EOF" 방식 썼는데
  # 중첩 bash 안의 heredoc end-marker 가 인식 안 되어 파일 생성 실패.
  # 호스트에서 임시파일 작성 후 docker cp 로 안전하게 전송.
  local tmp_host="/tmp/eval_inner_v1_${$}.py"

  cat > "$tmp_host" << PYEOF
from rl_navigation.drone_drop_env import DroneDropEnv
from rl_navigation.train_sac import DampedEntropySAC
import numpy as np, time, sys

print('=' * 60, flush=True)
print(f'Model:    ${model_path}', flush=True)
print(f'Episodes: ${episodes}', flush=True)
print('=' * 60, flush=True)

env = DroneDropEnv(config_path='${CONFIG_PATH}')

model = DampedEntropySAC.load(
    '${clean_path}',
    env=env,
    custom_objects=dict(
        ent_damping_threshold=5.0,
        ent_coef_hard_cap=1.0,
        target_q_clip=500.0,
    ),
)

print('Environment ready. GUI 준비 대기 (30초)...', flush=True)
sys.stdout.flush()
time.sleep(30)

results = []
EPISODES = ${episodes}
for ep in range(EPISODES):
    obs, _ = env.reset()
    done = False
    total_reward = 0
    steps = 0
    while not done:
        action, _ = model.predict(obs, deterministic=True)
        obs, reward, term, trunc, info = env.step(action)
        total_reward += reward
        steps += 1
        done = term or trunc
    drop_err = info.get('drop_error_actual_m', None)
    trigger = info.get('drop_trigger', 'none')
    d_xy = info.get('d_xy', None)
    trunc_reason = info.get('truncate_reason', '')

    results.append((ep+1, steps, trigger, drop_err, d_xy, total_reward, trunc_reason))
    drop_str = f'{drop_err:.2f}m' if drop_err is not None else 'N/A'
    d_xy_str = f'{d_xy:.2f}m' if d_xy is not None else 'N/A'
    print(f'EP{ep+1:>2}: steps={steps:>4} trigger={trigger:<6} drop_err={drop_str:<6} d_xy={d_xy_str:<6} reward={total_reward:>7.1f}  {trunc_reason}', flush=True)

print('', flush=True)
print('=' * 60, flush=True)
print('SUMMARY', flush=True)
print('=' * 60, flush=True)
valid_errs = [r[3] for r in results if r[3] is not None]
if valid_errs:
    print(f'  Drops:   {len(valid_errs)}/{EPISODES}', flush=True)
    print(f'  Best:    {min(valid_errs):.2f}m', flush=True)
    print(f'  Worst:   {max(valid_errs):.2f}m', flush=True)
    print(f'  Mean:    {sum(valid_errs)/len(valid_errs):.2f}m', flush=True)
    print(f'  < 5m:    {sum(1 for e in valid_errs if e < 5)}/{len(valid_errs)}', flush=True)
    print(f'  < 3m:    {sum(1 for e in valid_errs if e < 3)}/{len(valid_errs)}', flush=True)
    print(f'  < 1m:    {sum(1 for e in valid_errs if e < 1)}/{len(valid_errs)}', flush=True)
else:
    print('  No drops occurred.', flush=True)

print('', flush=True)
print('Sim alive for inspection (1h). Ctrl+C to exit early.', flush=True)
sys.stdout.flush()
time.sleep(3600)
env.close()
PYEOF

  docker cp "$tmp_host" "$CONTAINER:$EVAL_SCRIPT_PATH" >/dev/null 2>&1
  rm -f "$tmp_host"
}

# ---------- Eval 실행 ----------
start_eval() {
  echo -e "${BLUE}eval Python 시작...${NC}"
  docker exec -d "$CONTAINER" bash -lc "
    source /root/ros2_ws/install/setup.bash
    source /workspace/ros2_ws/install/setup.bash
    cd /workspace/ros2_ws
    python3 $EVAL_SCRIPT_PATH > $EVAL_LOG_PATH 2>&1
  "
}

# ---------- "Environment ready" 대기 ----------
wait_for_ready() {
  echo -e "${BLUE}환경 부팅 중 (~90초)...${NC}"
  local timeout=180
  local elapsed=0
  while [ $elapsed -lt $timeout ]; do
    if docker exec "$CONTAINER" grep -q "Environment ready" "$EVAL_LOG_PATH" 2>/dev/null; then
      echo -e "${GREEN}Ready!${NC}"
      return 0
    fi
    if docker exec "$CONTAINER" grep -qE "Traceback|FileNotFound|ImportError" "$EVAL_LOG_PATH" 2>/dev/null; then
      echo -e "${RED}[ERROR]${NC} eval 시작 실패. 로그:"
      docker exec "$CONTAINER" tail -10 "$EVAL_LOG_PATH"
      return 1
    fi
    sleep 5
    elapsed=$((elapsed+5))
  done
  echo -e "${RED}[ERROR]${NC} timeout (${timeout}s). 환경 부팅 실패."
  return 1
}

# ---------- GUI 실행 ----------
launch_gui() {
  echo -e "${BLUE}GUI 실행...${NC}"
  xhost +local:root >/dev/null 2>&1 || true
  docker exec -d "$CONTAINER" bash -c "DISPLAY=:1 gz sim -g" 2>/dev/null
  sleep 2
}

# ---------- GUI Watcher (background) ----------
gui_watcher() {
  # eval 살아있을 동안 gz sim -g 가 죽으면 자동 재실행
  while true; do
    # eval 죽었으면 종료
    if ! docker exec "$CONTAINER" pgrep -f "python3 $EVAL_SCRIPT_PATH" >/dev/null 2>&1; then
      break
    fi
    # gz sim -s 있는데 gz sim -g 없으면 재실행
    if docker exec "$CONTAINER" pgrep -f "gz sim -s" >/dev/null 2>&1 && \
       ! docker exec "$CONTAINER" pgrep -f "gz sim -g" >/dev/null 2>&1; then
      docker exec -d "$CONTAINER" bash -c "DISPLAY=:1 gz sim -g" 2>/dev/null
    fi
    sleep 2
  done
}

# ---------- Cleanup on Ctrl+C ----------
cleanup() {
  echo ""
  echo -e "${YELLOW}정리 중...${NC}"
  if [ -n "$WATCHER_PID" ]; then
    kill "$WATCHER_PID" 2>/dev/null || true
  fi
  clean_infra
  echo -e "${GREEN}정리 완료.${NC}"
  exit 0
}
trap cleanup INT TERM

# ---------- 결과 실시간 출력 ----------
follow_log() {
  echo ""
  echo -e "${GREEN}=== Episode 진행 (Ctrl+C 로 종료) ===${NC}"
  # eval 살아있을 동안 tail -f
  while docker exec "$CONTAINER" pgrep -f "python3 $EVAL_SCRIPT_PATH" >/dev/null 2>&1; do
    docker exec "$CONTAINER" tail -n 1 -f "$EVAL_LOG_PATH" 2>/dev/null &
    local tail_pid=$!
    # 30초마다 죽었는지 확인
    sleep 30
    kill "$tail_pid" 2>/dev/null || true
  done
}

# ====================================================================
# MAIN
# ====================================================================

echo -e "${GREEN}============================${NC}"
echo -e "${GREEN}  Drone GUI Evaluate 도구  ${NC}"
echo -e "${GREEN}============================${NC}"
echo ""

check_container

MODEL_PATH=$(select_model)
EPISODES=$(get_episodes)

echo ""
echo "선택:"
echo "  Model:    $MODEL_PATH"
echo "  Episodes: $EPISODES"
echo ""

clean_infra
prepare_eval_script "$MODEL_PATH" "$EPISODES"
start_eval

if ! wait_for_ready; then
  echo -e "${RED}eval 실패. 정리 후 종료.${NC}"
  clean_infra
  exit 1
fi

launch_gui

# GUI watcher 백그라운드로
gui_watcher &
WATCHER_PID=$!

follow_log

# eval 자연 종료 시
cleanup
