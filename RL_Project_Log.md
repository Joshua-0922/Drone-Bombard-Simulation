# RL Training Pipeline — Project Log

> **Branch:** `feature/migration-harmonic` | **Purpose:** Context-recovery log for Spot VM preemption and cross-session continuity.

---

# 1. Current State

## Reward Formula

`_compute_reward()` is currently a **placeholder returning 0.0**. Terminal accuracy reward from `drop_calculator` via `/rl/drop_error` is the only active signal.

**Planned per-step formula (not yet implemented):**

```
R = time_penalty
  + hover_penalty_scale  × (-|vx| - |vy|)   [penalise hovering, encourage forward motion]
  - stability_penalty_scale × |angular_velocity|  [penalise instability at drop moment]
  + speed_reward_scale   × forward_speed      [reward closing on target]
  + accuracy_reward_scale × (1 / drop_error)  [terminal: reward at episode end]
```

**Constants (from `config/hyperparams.yaml`):**

| Constant | Value |
|---|---|
| `time_penalty` | −0.005 / step |
| `hover_penalty_scale` | 0.5 |
| `speed_reward_scale` | 5.0 |
| `stability_penalty_scale` | 1.0 |
| `accuracy_reward_scale` | 2.0 |

## Key Hyperparameters

| Parameter | Value |
|---|---|
| Algorithm | SAC (Stable-Baselines3) |
| `total_timesteps` | 500,000 |
| `learning_rate` | 3.0 × 10⁻⁴ |
| `buffer_size` | 100,000 |
| `batch_size` | 256 |
| `tau` | 0.005 |
| `gamma` | 0.99 |
| `learning_starts` | 1,000 |
| `net_arch` | [256, 256] |
| `device` | cuda |
| `checkpoint_freq` | 5,000 steps |
| `max_steps` / episode | 500 |

## Best Checkpoint

No training runs completed yet. Checkpoints will be saved to:
`/workspace/ros2_ws/rl_checkpoints/sac_drop_<N>_steps.zip`

---

# 2. Recent Progress

- **Phase 1–4:** Full Gazebo Harmonic simulation stack (PX4 SITL, ros_gz_bridge, DetachableJoint payload drop)
- **Phase 5 base:** SAC Gymnasium environment (`drone_drop_env.py`) — 15-dim obs, 5-dim action space
- **Phase 5 base:** `train_sac.py` with TensorBoard logging + CheckpointCallback
- **2-layer launch architecture:** `infra.launch.py` (Gazebo + bridge, stays up) + `episode.launch.py` (PX4 + mission nodes, resets each episode) → episode cycle ~12 s
- **Phase 5 enhancements (latest commit `4c652de`):**
  - `config/hyperparams.yaml` — centralised hyperparameter file; loaded by env at runtime
  - `drone_drop_env.py` — loads yaml config; `_compute_reward()` stubbed to 0.0
  - `train_sac.py` — WandB integration (`resume="allow"` for preemption continuity), CUDA device selection, SIGTERM handler (emergency checkpoint save 30 s before GCP preemption), replay buffer checkpointing
  - `setup.py` — registers `config/hyperparams.yaml` as installed package data
  - `requirements.txt` — added `wandb>=0.18.0`
  - `README.md` Phase 5 — documents WandB setup, CUDA, Spot VM resilience workflow

---

# 3. Remaining Tasks (Next Steps)

- [ ] **Implement `_compute_reward()`** in `drone_drop_env.py`
  - Shape: `time_penalty + speed_reward - hover_penalty - stability_penalty + accuracy_reward`
  - All constants already defined in `hyperparams.yaml` under `reward:` section
- [ ] **WandB entity setup** — fill in `entity:` field in `config/hyperparams.yaml` with WandB username/team
- [ ] **Run first training session**
  - `wandb login`
  - Start infra: `ros2 launch mission_manager infra.launch.py`
  - Start training: `ros2 run rl_navigation train_sac`
- [ ] **Evaluate first run** — inspect WandB dashboard; check episode length, mean reward, drop error
- [ ] **Tune hyperparameters** — adjust `learning_rate`, `buffer_size`, `net_arch` based on first run results
- [ ] **Multi-env parallelism** — `SubprocVecEnv` if multiple GPUs available
- [ ] **Custom SB3 policy** — add PyTorch AMP (mixed precision) for faster L4 training

---

# 4. Training History (Append-only)

| Date | Run ID | WandB Link | Steps | Mean Drop Error | Notes |
|------|--------|-----------|-------|-----------------|-------|
| — | — | — | — | — | No runs yet — reward function not yet implemented |

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
