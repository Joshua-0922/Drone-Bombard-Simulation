# RL Training Pipeline — Project Log

> **Purpose:** Context-recovery log for Spot VM preemption. Update after each session.
> **Branch:** `feature/migration-harmonic`

---

## Completed Tasks

- [x] Phase 1–4: Gazebo Harmonic simulation stack (PX4 SITL, ros_gz_bridge, DetachableJoint payload)
- [x] Phase 5 base: SAC environment (`drone_drop_env.py`) with 15-dim obs / 5-dim action space
- [x] Phase 5 base: `train_sac.py` with TensorBoard logging + CheckpointCallback
- [x] 2-layer launch architecture (`infra.launch.py` + `episode.launch.py`)
- [x] Episode cycle optimised to ~12 s/reset
- [x] **Phase 5 enhancements:**
  - [x] `config/hyperparams.yaml` — centralised hyperparameter file
  - [x] `drone_drop_env.py` — loads config from yaml; `_compute_reward()` blanked (returns 0.0)
  - [x] `train_sac.py` — WandB integration, CUDA device, SIGTERM preemption handler, replay buffer checkpoint
  - [x] `setup.py` — registers `config/hyperparams.yaml` as package data
  - [x] `requirements.txt` — adds `wandb>=0.18.0`
  - [x] `README.md` Phase 5 — documents WandB, CUDA, Spot VM resilience

---

## Current Architecture

### Key Design Decisions

| Decision | Rationale |
|----------|-----------|
| SAC (off-policy) over PPO | ~12 s/reset → sample efficiency critical |
| 2-layer launch (infra + episode) | Gazebo + bridge stay up; only PX4 + mission nodes reset |
| `_compute_reward()` returns 0.0 | Per-step reward is placeholder; terminal accuracy reward from `drop_calculator` kept |
| `save_replay_buffer=True` | Spot VM preemption-safe; avoids cold start after 5k steps |
| `resume="allow"` in wandb.init | Same run continues after preemption |
| SIGTERM → emergency save | GCP Spot VMs send SIGTERM 30 s before preemption |

### File Map

| File | Role |
|------|------|
| `ros2_ws/src/rl_navigation/rl_navigation/drone_drop_env.py` | Gymnasium env; loads `hyperparams.yaml`; manages episode process group |
| `ros2_ws/src/rl_navigation/rl_navigation/train_sac.py` | SAC trainer; WandB + SIGTERM handler |
| `ros2_ws/src/rl_navigation/config/hyperparams.yaml` | All hyperparameters (training, SAC, env, reward, wandb) |
| `ros2_ws/src/rl_navigation/setup.py` | Registers yaml as installed package data |
| `drone_drop_system/docker/requirements.txt` | Python deps including `wandb` |

### Observation Space (15-dim)

| Indices | Feature | Normalisation |
|---------|---------|---------------|
| 0–2 | ENU position | ÷ 50 m |
| 3–5 | ENU velocity | ÷ 15 m/s |
| 6–8 | Angular velocity | ÷ π rad/s |
| 9–10 | Pixel u, v | (px/640)×2−1 |
| 11 | Detection confidence | [0, 1] |
| 12 | Payload attached flag | 0 or 1 |
| 13–14 | Relative pos to target | ÷ 50 m |

### Action Space (5-dim, [-1, 1])

| Index | Meaning | Scale |
|-------|---------|-------|
| 0 | vx (East) | ×15 m/s |
| 1 | vy (North) | ×5 m/s |
| 2 | vz (Up) | ×3 m/s |
| 3 | yaw_rate | ×1 rad/s |
| 4 | drop_trigger | >0 fires drop |

---

## Remaining / Next Steps

- [ ] **Implement `_compute_reward()`** — design per-step reward (currently returns 0.0)
  - Suggested shape: `time_penalty + hover_penalty + (speed_reward + stability_penalty at drop)`
  - Constants already defined in `hyperparams.yaml` under `reward:` section
- [ ] **WandB entity setup** — fill in `entity:` field in `config/hyperparams.yaml`
- [ ] **Run first training session** — `wandb login` → start infra → `ros2 run rl_navigation train_sac --config <path>`
- [ ] **Tune hyperparameters** — adjust `learning_rate`, `buffer_size`, `net_arch` based on first run results
- [ ] **Multi-env parallelism** — `SubprocVecEnv` if multiple GPUs available
- [ ] **Custom SB3 policy** — add PyTorch AMP (mixed precision) for faster L4 training

---

## Training Runs

| Run ID | WandB Link | Checkpoint | Steps | Mean Drop Error | Notes |
|--------|-----------|------------|-------|-----------------|-------|
| — | — | — | — | — | No runs yet |

---

## Quick-Start After Preemption

```bash
# 1. Reconnect to container
xhost +local:docker && docker start -ai drone-bombard-harmonic

# 2. Find latest checkpoint
ls -lt /workspace/ros2_ws/rl_checkpoints/ | head -5

# 3. Resume training (WandB run resumes automatically via resume="allow")
cd /workspace/ros2_ws && source install/setup.bash
ros2 launch mission_manager infra.launch.py &
sleep 25
ros2 run rl_navigation train_sac \
  --resume /workspace/ros2_ws/rl_checkpoints/sac_drop_XXXXXX_steps.zip

# 4. If preempt checkpoint exists:
ros2 run rl_navigation train_sac \
  --resume /workspace/ros2_ws/rl_checkpoints/sac_drop_preempt.zip
```
