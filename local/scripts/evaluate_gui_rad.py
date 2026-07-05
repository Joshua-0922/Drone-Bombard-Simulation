#!/usr/bin/env python3
"""dgui RAD — RAD v1 학습 모델 평가 (GUI + watcher + result save).

기존 dgui (v8 용) 의 RAD 버전. 차이:
  - DroneDropEnvRAD 사용 (obs 14d 상대좌표)
  - hyperparams_rad.yaml 로드
  - --phase 1 (Approach) / --phase 2 (Drop, --phase1-model 필요)
  - Phase 2 시 Phase1RolloutWrapper 자동 적용

사용법 (host 에서):

    # Phase 1 evaluate (default)
    dgui_rad sac_phase1_final --episodes 5

    # Phase 2 evaluate
    dgui_rad sac_phase2_final --phase 2 --phase1-model sac_phase1_final --episodes 5

    # interactive
    dgui_rad --phase 1
    dgui_rad --no-gui --stochastic           # CI mode

이 script 는:
  1. config (local/eval_config.yaml) 의 기본값 + CLI 인자 합침
  2. (인자 없으면) eval_models/ 의 .zip 모델들 list → interactive select
  3. cleanup 이전 process
  4. docker exec subprocess 로 평가 시작 (tmux 없이 직접 stream)
  5. env init 후 GUI 자동 launch
  6. 매 ep 의 [GZ_SERVER_READY] marker 마다 GUI relaunch (spawn → cruise 다 보임)
  7. GUI watcher thread — 죽으면 2 초 안에 자동 부활
  8. SUMMARY 후 결과 json 저장 + (keep_alive=False 이면) 즉시 cleanup
"""
from __future__ import annotations

import argparse
import json
import os
import re
import signal
import subprocess
import sys
import threading
import time
from datetime import datetime
from pathlib import Path

try:
    import yaml
except ImportError:
    print("[host] PyYAML 필요: pip install pyyaml", file=sys.stderr)
    sys.exit(1)

# ───────────────────────────────────────────────────────── config ─────
CONFIG_PATH = Path(__file__).resolve().parent.parent / "eval_config.yaml"
with open(CONFIG_PATH) as f:
    CFG = yaml.safe_load(f)

CONTAINER = CFG["container"]
TMUX_SESSION = "eval"  # legacy cleanup target

# 전역 — 동적 변수
EVAL_PROC: subprocess.Popen | None = None
GUI_PROC: subprocess.Popen | None = None
GUI_LAST_LAUNCH = 0.0    # last launch_gui() timestamp (monotonic)
WATCHER_RUNNING = False
WATCHER_THREAD: threading.Thread | None = None
DETERMINISTIC = CFG["deterministic"]
CAMERA_MODE = CFG.get("camera_mode", "follow")    # follow | origin | default


# ─────────────────────────────────────────────── inline eval script ───
EVAL_SCRIPT_TEMPLATE = r"""
from rl_navigation.drone_drop_env_rad import DroneDropEnvRAD
from rl_navigation.train_sac_rad import Phase1RolloutWrapper
from stable_baselines3 import SAC
import numpy as np
import time
import sys
import json
import yaml

CFG = "/workspace/ros2_ws/install/rl_navigation/share/rl_navigation/config/hyperparams_rad.yaml"
MODEL_PATH = "{model_path}"
EPISODES = {episodes}
DETERMINISTIC = {deterministic}
GUI_WAIT = {gui_wait}
KEEP_ALIVE = {keep_alive}
PHASE = "{phase}"           # "phase1" or "phase2"
PHASE1_MODEL = "{phase1_model}"   # Phase 2 시 Phase 1 모델 경로

# Phase 분기 yaml 임시 override
with open(CFG) as f:
    _cfg = yaml.safe_load(f)
_cfg['training']['phase'] = PHASE
_runtime_cfg = "/tmp/hyperparams_rad_eval.yaml"
with open(_runtime_cfg, 'w') as f:
    yaml.safe_dump(_cfg, f)

env = DroneDropEnvRAD(config_path=_runtime_cfg)
print(f"[eval-RAD] env created. Phase={{PHASE}}. Loading model: {{MODEL_PATH}}", flush=True)
model = SAC.load(MODEL_PATH)

# Phase 2 시 Phase1RolloutWrapper 적용
if PHASE == "phase2":
    if not PHASE1_MODEL:
        print("[ERROR] Phase 2 evaluate 시 PHASE1_MODEL 필요", file=sys.stderr, flush=True)
        sys.exit(1)
    phase1 = SAC.load(PHASE1_MODEL, device='cpu')
    env = Phase1RolloutWrapper(env, phase1, max_rollout_steps=300)
    print(f"[eval-RAD] Phase 2: Phase1RolloutWrapper 적용 (phase1={{PHASE1_MODEL}})", flush=True)

print("Environment ready. Open GUI now.", flush=True)

results = []
for ep in range(EPISODES):
    # D-1: 매 ep 직전 _consecutive_fast_resets=max → reset 안에서 학습 D1 path
    # (_obs_ready.clear + _kill_infra + _start_infra). 학습 환경 100% 복제.
    # E 처방 (강제 set 제거) 시도했지만 drop 없는 ep 후 fast reset 의 spawn drift
    # 누적 부작용 발생 → 복원.
    if ep == 0:
        print(f"[eval] EP1: GUI open wait ({{GUI_WAIT}}s) ...", flush=True)
        time.sleep(GUI_WAIT)
    env._consecutive_fast_resets = env._cfg_max_consecutive_fast_resets
    try:
        obs, _ = env.reset()
        p = env._node.pos_enu.tolist(); v = env._node.vel_enu.tolist()
        a = env._node.ang_vel.tolist()
        print(f"[trace] EP{{ep+1}}: spawn pos=({{p[0]:.2f}}, {{p[1]:.2f}}, {{p[2]:.2f}}) "
              f"vel=({{v[0]:.2f}}, {{v[1]:.2f}}, {{v[2]:.2f}}) "
              f"ang_vel=({{a[0]:.3f}}, {{a[1]:.3f}}, {{a[2]:.3f}})", flush=True)
    except Exception as e:
        print(f"[ERROR] env.reset() failed: {{type(e).__name__}}: {{e}}", flush=True)
        import traceback; traceback.print_exc()
        break
    time.sleep({settle_sleep})
    done = False
    total_reward = 0.0
    steps = 0
    max_ang_vel_mag = 0.0
    while not done:
        action, _ = model.predict(obs, deterministic=DETERMINISTIC)
        obs, reward, term, trunc, info = env.step(action)
        total_reward += reward
        steps += 1
        try:
            import numpy as _np
            cur_a = env._node.ang_vel
            mag = float(_np.linalg.norm(cur_a))
            if mag > max_ang_vel_mag:
                max_ang_vel_mag = mag
        except Exception:
            pass
        done = term or trunc
    try:
        a_drop = env._node.ang_vel.tolist()
        print(f"[trace] EP{{ep+1}}: drop ang_vel=({{a_drop[0]:.3f}}, {{a_drop[1]:.3f}}, {{a_drop[2]:.3f}}) "
              f"max_ang_vel_mag={{max_ang_vel_mag:.3f}} rad/s", flush=True)
    except Exception:
        pass
    drop_err = info.get("drop_error_actual_m", None)
    trigger = info.get("drop_trigger", "none")
    d_xy = info.get("d_xy", None)
    trunc_reason = info.get("truncate_reason", "")
    results.append({{
        "ep": ep + 1, "steps": steps, "trigger": trigger,
        "drop_error": drop_err, "d_xy": d_xy,
        "reward": float(total_reward), "truncate_reason": trunc_reason,
    }})
    err_str = f"{{drop_err:.2f}}" if isinstance(drop_err, (int, float)) else "N/A"
    dxy_str = f"{{d_xy:.2f}}" if isinstance(d_xy, (int, float)) else "N/A"
    print(
        f"EP{{ep+1}}: steps={{steps}} trigger={{trigger}} "
        f"drop_err={{err_str}}m d_xy={{dxy_str}}m reward={{total_reward:.1f}} {{trunc_reason}}",
        flush=True,
    )

# SUMMARY (json line for host to parse)
drops = [r for r in results if isinstance(r["drop_error"], (int, float))]
errs = [r["drop_error"] for r in drops]
summary = {{
    "episodes": EPISODES, "drops": len(drops),
    "success_2m": sum(1 for e in errs if e <= 2.0),
    "jackpot_03m": sum(1 for e in errs if e <= 0.3),
    "mean_err": (sum(errs) / len(errs)) if errs else None,
    "min_err": min(errs) if errs else None,
    "max_err": max(errs) if errs else None,
    "results": results,
}}
print(f"[RESULT_JSON] {{json.dumps(summary)}}", flush=True)
print("\n[eval] === SUMMARY ===", flush=True)
if drops:
    n = len(drops)
    print(f"  episodes: {{EPISODES}}, drops: {{n}}", flush=True)
    print(f"  success ≤2m:  {{summary['success_2m']}}/{{n}} = {{100*summary['success_2m']/n:.1f}}%", flush=True)
    print(f"  jackpot ≤0.3m:{{summary['jackpot_03m']}}/{{n}} = {{100*summary['jackpot_03m']/n:.1f}}%", flush=True)
    print(f"  mean err: {{summary['mean_err']:.3f}}m  min: {{summary['min_err']:.3f}}m  max: {{summary['max_err']:.3f}}m", flush=True)
else:
    print(f"  No drops in {{EPISODES}} episodes.", flush=True)
print("\nEvaluation complete.", flush=True)
if KEEP_ALIVE:
    print("[eval] Sim stays alive (keep_alive=True). Stop manually.", flush=True)
    time.sleep(3600)
env.close()
"""


# ─────────────────────────────────────────────────── host helpers ─────
def run_host(cmd, check=True, capture=False):
    return subprocess.run(cmd, check=check, capture_output=capture, text=True)


def docker_exec(bash_cmd, capture=False, check=True):
    return run_host(["docker", "exec", CONTAINER, "bash", "-lc", bash_cmd],
                    check=check, capture=capture)


def cleanup():
    """모든 잔존 process 정리. pkill 자기-자신 회피."""
    global EVAL_PROC, GUI_PROC, WATCHER_RUNNING, WATCHER_THREAD
    print("[host] Cleanup ...", flush=True)
    WATCHER_RUNNING = False
    if WATCHER_THREAD and WATCHER_THREAD.is_alive():
        WATCHER_THREAD.join(timeout=3)
    run_host(["tmux", "kill-session", "-t", TMUX_SESSION], check=False)
    for proc in (GUI_PROC, EVAL_PROC):
        if proc and proc.poll() is None:
            try:
                proc.terminate()
                proc.wait(timeout=3)
            except Exception:
                try: proc.kill()
                except Exception: pass
    # narrow pkill — 자기-bash 안 죽임 (-f 의 pattern 이 docker exec 명령 자체 포함 안 함).
    docker_exec(
        "pkill -9 -f 'gz sim' 2>/dev/null; "
        "pkill -9 -f 'bin/px4' 2>/dev/null; "
        "pkill -9 -f MicroXRCEAgent 2>/dev/null; "
        "pkill -9 -f parameter_bridge 2>/dev/null; "
        "pkill -9 -f '_evaluate_gui_session' 2>/dev/null; "
        "rm -f /dev/shm/fastrtps_* /tmp/px4_lock-* /tmp/px4-sock-* "
        "/tmp/drone_env_gz_ready /tmp/x_marker_world_rl_*.sdf 2>/dev/null; "
        "true",
        check=False,
    )


def list_models():
    """eval_models/ 의 .zip 모델 list (정렬)."""
    res = docker_exec(f"ls -1 {CFG['container_models_path']} 2>/dev/null",
                      capture=True, check=False)
    items = [l.strip() for l in res.stdout.splitlines() if l.strip().endswith(".zip")]
    return sorted(items)


def interactive_select() -> str:
    models = list_models()
    if not models:
        print(f"[host] eval_models/ 에 .zip 모델 없음.", file=sys.stderr)
        print(f"       host path: {CFG['host_models_path']}", file=sys.stderr)
        sys.exit(1)
    print(f"\n📦 Available models in eval_models/:")
    for i, m in enumerate(models, 1):
        print(f"  {i}. {m}")
    while True:
        sel = input(f"\nSelect model (1-{len(models)}): ").strip()
        try:
            idx = int(sel) - 1
            if 0 <= idx < len(models):
                return models[idx]
        except ValueError:
            pass
        print(f"  잘못된 입력. 1-{len(models)} 사이의 숫자.")


def interactive_camera() -> str:
    """모델 선택 후 카메라 모드 묻기. Enter 면 config default."""
    options = [
        ("follow", "drone 자동 추적 (계속 따라다님)"),
        ("origin", "drone 위치로 카메라 한 번 이동"),
        ("default", "기본 (Gazebo default 카메라)"),
    ]
    default = CFG.get("camera_mode", "follow")
    print(f"\n📷 Camera mode:")
    for i, (k, desc) in enumerate(options, 1):
        marker = " (default)" if k == default else ""
        print(f"  {i}. {k}{marker} — {desc}")
    while True:
        sel = input(f"\nSelect camera (1-{len(options)}, Enter for default '{default}'): ").strip()
        if not sel:
            return default
        try:
            idx = int(sel) - 1
            if 0 <= idx < len(options):
                return options[idx][0]
        except ValueError:
            pass
        print(f"  잘못된 입력. 1-{len(options)} 또는 Enter.")


# ────────────────────────────────────────────────── eval / stream ─────
def start_eval(model_path: str, episodes: int, gui_wait: int, keep_alive: bool, settle_sleep: float,
               phase: str = "phase1", phase1_model: str = ""):
    global EVAL_PROC
    script = EVAL_SCRIPT_TEMPLATE.format(
        model_path=model_path,
        episodes=episodes,
        deterministic=DETERMINISTIC,
        gui_wait=gui_wait,
        keep_alive=str(keep_alive),
        settle_sleep=settle_sleep,
        phase=phase,
        phase1_model=phase1_model,
    )
    eval_path = "/tmp/_evaluate_gui_session.py"
    subprocess.run(
        ["docker", "exec", "-i", CONTAINER, "bash", "-c", f"cat > {eval_path}"],
        input=script, text=True, check=True,
    )
    print(f"[host] Wrote eval script: {eval_path}")
    inner_cmd = (
        "source /opt/ros/humble/setup.bash && "
        "source /root/ros2_ws/install/setup.bash && "
        "source /workspace/ros2_ws/install/setup.bash && "
        f"cd /workspace/ros2_ws && exec python3 -u {eval_path} 2>&1"
    )
    EVAL_PROC = subprocess.Popen(
        ["docker", "exec", CONTAINER, "bash", "-lc", inner_cmd],
        stdout=subprocess.PIPE, stderr=subprocess.STDOUT,
        text=True, bufsize=1,
    )
    print(f"[host] eval subprocess pid={EVAL_PROC.pid}")


def wait_ready(timeout: int = 180) -> bool:
    print(f"[host] Waiting for env init (up to {timeout}s) ...")
    deadline = time.monotonic() + timeout
    assert EVAL_PROC is not None
    while time.monotonic() < deadline:
        if EVAL_PROC.poll() is not None:
            print(f"[host] eval subprocess exited early (code={EVAL_PROC.returncode})")
            return False
        line = EVAL_PROC.stdout.readline()
        if not line:
            time.sleep(0.05)
            continue
        sys.stdout.write(line); sys.stdout.flush()
        if "Environment ready" in line:
            print("[host] Environment READY.")
            return True
    print("[host] Timed out waiting for env.")
    return False


def apply_camera_mode(mode: str):
    """launch_gui 후 카메라 적용. mode: follow | origin | default."""
    if mode == "default":
        return
    target = CFG.get("camera_target_model", "x500_bombard_0")
    settle = CFG.get("camera_settle_secs", 3.0)
    # GUI 가 server 와 연결할 시간 줌
    def _async():
        time.sleep(settle)
        try:
            if mode == "follow":
                docker_exec(
                    f"gz service -s /gui/follow "
                    f"--reqtype gz.msgs.StringMsg --reptype gz.msgs.Boolean "
                    f"--timeout 2000 --req 'data: \"{target}\"' 2>/dev/null; true",
                    check=False,
                )
                print(f"[host] Camera: follow → {target}", flush=True)
            elif mode == "origin":
                docker_exec(
                    f"gz service -s /gui/move_to "
                    f"--reqtype gz.msgs.StringMsg --reptype gz.msgs.Boolean "
                    f"--timeout 2000 --req 'data: \"{target}\"' 2>/dev/null; true",
                    check=False,
                )
                print(f"[host] Camera: moved to {target}", flush=True)
        except Exception as e:
            print(f"[host] Camera command failed: {e}", flush=True)
    threading.Thread(target=_async, daemon=True, name=f"camera-{mode}").start()


def launch_gui():
    global GUI_PROC, GUI_LAST_LAUNCH
    run_host(["xhost", "+local:root"], check=False)
    if GUI_PROC and GUI_PROC.poll() is None:
        try: GUI_PROC.terminate(); GUI_PROC.wait(timeout=2)
        except Exception:
            try: GUI_PROC.kill()
            except Exception: pass
    docker_exec("pkill -9 -f 'gz sim -g' 2>/dev/null; true", check=False)
    GUI_PROC = subprocess.Popen(
        ["docker", "exec", CONTAINER, "bash", "-c", f"DISPLAY={CFG['display']} gz sim -g"],
        stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL,
    )
    GUI_LAST_LAUNCH = time.monotonic()
    # 카메라 모드 적용 (현재 선택된 mode 는 global CAMERA_MODE)
    apply_camera_mode(CAMERA_MODE)


def gui_watcher_loop():
    """GUI process 죽으면 자동 재launch. backoff 로 server 시작 대기."""
    global GUI_PROC
    backoff = CFG.get("gui_watcher_backoff", 25.0)
    while WATCHER_RUNNING:
        time.sleep(CFG["gui_watcher_interval"])
        if not WATCHER_RUNNING:
            break
        if GUI_PROC is not None and GUI_PROC.poll() is not None:
            # backoff: 마지막 launch 후 N초 안에는 또 launch 안 함.
            # 이유: _kill_infra → _start_infra 사이 server 없음 → GUI 가 즉시 죽음.
            # backoff 없으면 watcher 가 빈 launch 반복.
            elapsed = time.monotonic() - GUI_LAST_LAUNCH
            if elapsed < backoff:
                continue   # server 안정 대기, watcher skip
            print("[watcher] GUI died (post-backoff), relaunching ...", flush=True)
            try: launch_gui()
            except Exception as e:
                print(f"[watcher] relaunch failed: {e}", flush=True)




def start_watcher():
    global WATCHER_RUNNING, WATCHER_THREAD
    if not CFG.get("gui_watcher_enabled", True):
        return
    WATCHER_RUNNING = True
    WATCHER_THREAD = threading.Thread(target=gui_watcher_loop, daemon=True, name="gui-watcher")
    WATCHER_THREAD.start()
    print(f"[host] GUI watcher started (interval {CFG['gui_watcher_interval']}s)")


def tail_results(max_secs: int, with_gui: bool, model_name: str, results_dir: Path) -> dict | None:
    print(f"\n[host] Following eval (up to {max_secs//60} min)\n" + "=" * 60)
    deadline = time.monotonic() + max_secs
    seen = set()
    saw_summary = saw_complete = False
    summary_json = None
    assert EVAL_PROC is not None
    while time.monotonic() < deadline:
        if EVAL_PROC.poll() is not None:
            for line in EVAL_PROC.stdout:
                sys.stdout.write(line); sys.stdout.flush()
                jm = re.search(r"\[RESULT_JSON\] (.+)", line)
                if jm and summary_json is None:
                    try: summary_json = json.loads(jm.group(1))
                    except json.JSONDecodeError: pass
            print(f"\n[host] eval subprocess exited (code={EVAL_PROC.returncode})")
            break
        line = EVAL_PROC.stdout.readline()
        if not line:
            time.sleep(0.05); continue
        sys.stdout.write(line); sys.stdout.flush()
        # B: env reset 의 _start_infra() 직후 [GZ_SERVER_READY] → 즉시 GUI relaunch.
        if with_gui:
            m = re.search(r"\[GZ_SERVER_READY\] reset_count=(\d+)", line)
            if m and m.group(1) not in seen:
                seen.add(m.group(1))
                print(f"\n[host] reset_count={m.group(1)}: GUI relaunch (spawn-ready)", flush=True)
                launch_gui()
        # 결과 json 추출
        jm = re.search(r"\[RESULT_JSON\] (.+)", line)
        if jm and summary_json is None:
            try: summary_json = json.loads(jm.group(1))
            except json.JSONDecodeError: pass
        if "[eval] === SUMMARY ===" in line: saw_summary = True
        if "Evaluation complete" in line: saw_complete = True
        if saw_summary and saw_complete:
            time.sleep(1)
            break
    print("\n" + "=" * 60)
    # 결과 저장
    if summary_json is not None:
        results_dir.mkdir(parents=True, exist_ok=True)
        ts = datetime.now().strftime("%Y-%m-%dT%H-%M-%S")
        out = results_dir / f"eval_{ts}_{Path(model_name).stem}.json"
        summary_json["model"] = model_name
        summary_json["timestamp"] = ts
        summary_json["deterministic"] = DETERMINISTIC
        with open(out, "w") as f:
            json.dump(summary_json, f, indent=2)
        print(f"💾 Results saved: {out}")
    return summary_json


# ──────────────────────────────────────────────────────────── main ─────
def main():
    global DETERMINISTIC
    parser = argparse.ArgumentParser(
        description="GUI 학습 모델 평가 (interactive + watcher + auto-save)",
        formatter_class=argparse.RawDescriptionHelpFormatter,
    )
    parser.add_argument("model", nargs="?",
                        help="모델 이름 (eval_models/ 안). 생략하면 interactive list")
    parser.add_argument("-e", "--episodes", type=int, default=CFG["default_episodes"],
                        help=f"평가 episode 수 (default {CFG['default_episodes']})")
    parser.add_argument("--no-gui", action="store_true", help="GUI 안 띄움")
    parser.add_argument("--no-watcher", action="store_true", help="GUI watcher 비활성")
    parser.add_argument("--stochastic", action="store_true",
                        help="stochastic policy (default: deterministic)")
    parser.add_argument("--keep-alive", action="store_true",
                        help="평가 후 sim 유지 (config 의 keep_sim_alive override)")
    parser.add_argument("--tail-min", type=int, default=CFG["tail_min"],
                        help=f"max tail 시간 (분, default {CFG['tail_min']})")
    parser.add_argument("--camera", choices=["follow", "origin", "default"],
                        default=None,
                        help=f"카메라 모드 (default: config 의 '{CFG.get('camera_mode', 'follow')}', "
                             "또는 interactive 시 묻기)")
    # RAD specific
    parser.add_argument("--phase", choices=["1", "2"], default="1",
                        help="RAD phase (default: 1 = Approach). "
                             "Phase 2 시 --phase1-model 필요")
    parser.add_argument("--phase1-model", default="",
                        help="Phase 2 evaluate 시 Phase 1 모델 zip 경로 (container path 또는 eval_models name)")
    args = parser.parse_args()

    # RAD phase 처리
    rad_phase = f"phase{args.phase}"
    phase1_model_path = ""
    if rad_phase == "phase2":
        if not args.phase1_model:
            print("[ERROR] --phase 2 사용 시 --phase1-model 필요", file=sys.stderr)
            return 1
        p1 = args.phase1_model
        # eval_models name 만 받으면 container path 로 확장
        if "/" not in p1:
            if not p1.endswith(".zip"):
                p1 = p1 + ".zip"
            p1 = f"{CFG['container_models_path']}/{p1}"
        phase1_model_path = p1

    global CAMERA_MODE
    DETERMINISTIC = not args.stochastic

    # model 선택
    interactive = not args.model
    if args.model:
        model_name = args.model
        if not model_name.endswith(".zip"):
            model_name = model_name + ".zip"
    else:
        model_name = interactive_select()
    model_path = f"{CFG['container_models_path']}/{model_name}"

    # 카메라 모드 결정: CLI > interactive 선택 > config default
    if args.camera:
        CAMERA_MODE = args.camera
    elif interactive and not args.no_gui:
        CAMERA_MODE = interactive_camera()
    else:
        CAMERA_MODE = CFG.get("camera_mode", "follow")

    keep_alive = args.keep_alive or CFG.get("keep_sim_alive", False)

    print(f"\n[host] Model      : {model_name}")
    print(f"[host] Episodes   : {args.episodes}")
    print(f"[host] GUI        : {'yes' if not args.no_gui else 'no'}")
    print(f"[host] Watcher    : {'yes' if not args.no_watcher and CFG['gui_watcher_enabled'] else 'no'}")
    print(f"[host] Camera     : {CAMERA_MODE}")
    print(f"[host] Policy     : {'deterministic' if DETERMINISTIC else 'stochastic'}")
    print(f"[host] Keep alive : {'yes' if keep_alive else 'no (auto-stop)'}")
    print(f"[host] RAD phase  : {rad_phase}")
    if rad_phase == "phase2":
        print(f"[host] Phase1 model: {phase1_model_path}")

    # signal handler
    def handler(signum, _frame):
        print(f"\n[host] Signal {signum} → cleanup"); cleanup()
        run_host(["xhost", "-local:root"], check=False)
        sys.exit(130)
    signal.signal(signal.SIGINT, handler)
    signal.signal(signal.SIGTERM, handler)

    cleanup()
    start_eval(
        model_path, args.episodes, CFG["gui_wait_time"], keep_alive,
        settle_sleep=CFG.get("reset_settle_sleep", 5.0),
        phase=rad_phase, phase1_model=phase1_model_path,
    )
    if not wait_ready(timeout=180):
        cleanup(); return 1

    if not args.no_gui:
        launch_gui()
        if not args.no_watcher and CFG.get("gui_watcher_enabled", True):
            start_watcher()

    try:
        tail_results(
            max_secs=args.tail_min * 60,
            with_gui=not args.no_gui,
            model_name=model_name,
            results_dir=Path(CFG["results_dir"]),
        )
    except KeyboardInterrupt:
        pass

    cleanup()
    run_host(["xhost", "-local:root"], check=False)
    print("[host] Bye.")
    return 0


if __name__ == "__main__":
    sys.exit(main())
