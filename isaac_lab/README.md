# isaac_lab/ — Isaac Lab drone payload-drop task (RL flight + supervised impact residual)

**Current state (2026-09-25).** One GPU-vectorized Isaac Lab `DirectRLEnv`
(`Isaac-DroneBombard-Task-v0`) trains the flight policy L0 with PPO (rsl_rl,
2048 envs, 1000 iters, model-error scale 1.5). A small GRU (obs 26 -> 64 -> 2)
is then fitted offline on frozen-L0 flight logs to predict the impact-point
drift and injected at evaluation with an EMA (alpha 0.3). No wind sensor.
Method + numbers: `notes/research/l1_sl_pipeline.md`, `notes/research/final_tables_v6.md`.
Where things are: `notes/research/code_map.md`, `notes/research/isaac_lab_architecture.md`.
Commands: `notes/sessions/commands.md`.

Runs inside container `isaac-verify` at `/tmp/rebuild` (copy of this dir):
`docker cp isaac_lab/. isaac-verify:/tmp/rebuild/` after editing; python is
`/workspace/isaaclab/isaaclab.sh -p`. Artifacts live in the container's `/tmp`.

Historical: this directory started (2026-07) as the port of the Gazebo/PX4/ROS2
SAC task; that migration record is `notes/experiments/legacy/exp_012_isaac_migration_phase2.md`.
`ros2_ws/` and `gazebo_models/` at the repo root are the old stack, unreferenced.

## Layout

```
isaac_lab/
  drone_bombard/
    math_utils.py         pure torch action/vision/ballistic/reward/guard
                           math — zero isaaclab dependency, unit-tested
    drone_bombard_env.py  the env: DirectRLEnv wiring math_utils.py into
                           the isaaclab lifecycle (scene, actuation, obs,
                           dones, rewards, reset)
    mdp/domain_rand.py     Phase-2 domain-randomization stubs (identity in
                           Phase 1)
    agents/rsl_rl_ppo_cfg.py  PPO hyperparameters
  train.py                 rsl_rl training entrypoint
  play.py                  sanity/calibration modes (see below)
  yolo_eval.py              real-YOLO eval + vision calibration
  tests/test_math.py        pure-torch unit tests (no isaaclab needed)
```

## Status: VERIFIED running (physics) — rendering needs driver >= 580

**The env was executed live** on isaac-sim:5.1.0 (2026-07-03) and passed a
one-episode, no-training verification (`verify_one_episode.py`): env
constructs the USD scene, resets to a 14-dim obs, the drone **hovers stably
for ~148 steps**, obs/reward/termination are all finite (no NaN), and the
stagnation guard fires correctly. That live run also fixed the real
env/controller bugs the pure-math unit tests can't reach (see
`notes/experiments/exp_012_isaac_migration_phase2.md` §6b) — most notably a
missing inertia term in the rate-loop torque.

**What still needs the L4 Spot VM**: RTX **rendering** — cameras, viewport,
GUI. Isaac Sim 5.1.0 requires GPU driver >= 580.65.06; the current dev box
has 535, so the RTX renderer will not initialize ("rtx driver verification
failed"). Physics/CUDA is unaffected (that's what the verification ran on),
but any visual output — `play.py --with_camera`, `yolo_eval.py`, a GUI
session — requires the L4 Spot VM (`l4-spot`, `asia-east1-a`, driver >= 580)
with the `isaac-lab` image built from `drone_drop_system/docker/Dockerfile`.

To reproduce the headless verification anywhere with a working install:
```bash
./isaaclab.sh -p verify_one_episode.py --headless --enable_cameras --num_steps 300
# or, unbuffered with the true exit code:
/isaac-sim/python.sh verify_one_episode.py --headless --enable_cameras --num_steps 300
```
(`tests/test_math.py` still runs with just torch, no isaaclab: 29/29 passing.)

## Running on the L4 Spot VM

Code is mounted into the container (`infra/startup.sh` mounts this repo at
`/workspace/drone-bombard`), not baked into the image — edit locally,
no rebuild needed to iterate.

```bash
# 1. Cartpole smoke (infra gate, already wired into infra/startup.sh)
#    Confirms the base Isaac Sim + Isaac Lab + rsl_rl install works before
#    trusting any drone_bombard-specific failure signal.

# 2. env import + tiny smoke (2 iterations, 16 envs)
./isaaclab.sh -p /workspace/drone-bombard/isaac_lab/train.py \
  --task Isaac-DroneBombard-Direct-v0 --headless --num_envs 16 --max_iterations 2

# 3. physics/actuation sanity
./isaaclab.sh -p /workspace/drone-bombard/isaac_lab/play.py --zero-actions --num_envs 4 --headless
#   -> hover alt drift must stay < 1m over 10s, no NaNs

./isaaclab.sh -p /workspace/drone-bombard/isaac_lab/play.py --scripted --num_envs 4 --headless
#   -> d_xy must fall monotonically toward the target

# 4. velocity-controller calibration (see notes/research/isaac_velocity_controller.md)
./isaaclab.sh -p /workspace/drone-bombard/isaac_lab/play.py --step-response \
  --num_envs 4 --headless --out-csv /workspace/logs/isaac_lab/step_response.csv
#   -> compare against recorded PX4 SITL traces (rise time/overshoot/settling/gain)

# 5. throughput probe, then the real run (already the default startup.sh command)
./isaaclab.sh -p /workspace/drone-bombard/isaac_lab/train.py \
  --num_envs 2048 --max_iterations 20 --headless
tmux new -s isaac_train \
  './isaaclab.sh -p /workspace/drone-bombard/isaac_lab/train.py --headless --num_envs 2048 --resume latest'

# 6. YOLO vision calibration / eval (num_envs<=8; needs drone_bombard_best.pt)
./isaaclab.sh -p /workspace/drone-bombard/isaac_lab/yolo_eval.py --calibrate \
  --num_envs 8 --headless --out-csv /workspace/logs/isaac_lab/yolo_calibration.csv
./isaaclab.sh -p /workspace/drone-bombard/isaac_lab/yolo_eval.py --eval \
  --policy /workspace/logs/isaac_lab/drone_bombard/model_final.pt --num_envs 8 --headless
```

## Task parity with Gazebo v13/v15 (summary — see the experiment note for the full table)

- Obs: 14-dim, same normalization (pos/50, vel/15, ang_vel/pi, YOLO u/v/conf,
  metric rel_dx/dy).
- Action: 4-dim ENU velocity setpoint, same scales (4/3/3/1), same 0.2
  rate-limit, same 0.4-alpha LPF (ticked at 20Hz — 2 ticks per 10Hz policy
  step, matching `drone_controller_node._filter_velocity`).
- Reward: identical 3-layer formula and constants (`math_utils.compute_reward`).
- Termination: identical crash/overspeed/bad-attitude/out-of-range/
  max-altitude/overshoot/stagnation/timeout guards and constants
  (`math_utils.overshoot_guard`, `stagnation_guard`).
- **Deliberate deltas** (documented, not bugs): policy control loop
  (SAC->PPO, rsl_rl instead of SB3); failures are `terminated` not
  `truncated` (PPO doesn't bootstrap through them, unlike SB3's SAC
  which did); target position AND spawn distance are randomized per-env
  (Gazebo had a fixed target and handoff distance emerged from the
  scripted CRUISE phase); vision is an analytic pinhole model in training
  (YOLO-calibrated) with a real-YOLO eval mode, instead of always running
  YOLOv8 inference in the loop.
- **Phase-2 hooks, inert in Phase 1** (see `drone_bombard/math_utils.py`
  docstrings): CCIP learned-residual slot (`ccip_residual`, `DropCfg.
  residual_enabled`), domain-randomization stubs (`mdp/domain_rand.py`).
  Phase 1 output is bit-identical to a hook-free implementation.

## Testing

```bash
# no isaaclab required:
pip install torch pytest   # or run inside any container with torch, e.g.
                            # this repo's drone-bombard-harmonic Gazebo container
pytest isaac_lab/tests/test_math.py -v   # 29 tests, all passing as of Phase-2 delivery
```
