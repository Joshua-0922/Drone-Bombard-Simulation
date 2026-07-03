# isaac_lab/ — Isaac Lab port of the drone-bombard RL task

Phase 2 of the Gazebo+PX4+ROS2 -> Isaac Lab migration (Phase 1 = the Docker
image in `drone_drop_system/docker/`, Isaac Sim 5.1.0 + Isaac Lab v2.3.2 +
rsl_rl). This directory replicates the terminal visual-servoing task learned
by SAC on Gazebo (`ros2_ws/src/rl_navigation/rl_navigation/drone_drop_env.py`,
v13/v15 baseline) as one GPU-vectorized Isaac Lab environment, trained with
PPO (rsl_rl) instead of SAC.

See `notes/experiments/exp_012_isaac_migration_phase2.md` for the full
parity table (every v13 constant -> Isaac cfg field) and design rationale,
and `notes/research/isaac_velocity_controller.md` for the velocity
controller's PX4-gain mapping and calibration status.

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

## IMPORTANT: this dev box cannot run Isaac Sim

Isaac Sim 5.1.0 requires GPU driver >= 580.65.06; this dev box has 535.309.
All code here was written and unit-tested (`tests/test_math.py`, 29/29
passing) without ever importing `isaaclab` — verified by running the tests
against a plain torch install (this box's `drone-bombard-harmonic` Gazebo
container has torch 2.4.1 and no isaaclab). Actually running the
environment requires the L4 Spot VM (`l4-spot`, `asia-east1-a`) with the
`isaac-lab` image built from `drone_drop_system/docker/Dockerfile`.

**Caveat**: because the isaaclab-dependent code in `drone_bombard_env.py`
(scene setup, actuation, Articulation data access) could not be executed
against a real Isaac Lab v2.3.2 install, some API call names (e.g.
`write_root_pose_to_sim`, `write_root_velocity_to_sim`, `root_physx_view`
mass/inertia overrides) are written from general Isaac Lab API knowledge
and may need small name/signature fixups on first run against the actual
v2.3.2 API surface. This is expected and should be the first thing checked
if the env-smoke step below fails at import/reset rather than at a
numerical/behavioral issue.

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
