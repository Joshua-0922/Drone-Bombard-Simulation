# RL Training Pipeline — Project Log

> **Branch:** `feature/migration-harmonic` | **Purpose:** Context-recovery log for Spot VM preemption and cross-session continuity.

---

# 1. Current State

## Reward Formula — 4-Layer Hierarchical System

Fully implemented in `drone_drop_env.py`. Active on every `step()` call.

### Layer 1 — Safety (hard termination)
| Trigger | Reward | `done` |
|---|---|---|
| altitude < `min_altitude` (2 m), after step 20 | −10 | True |
| speed > 20 m/s | −8 | True |
| vision confidence == 0 (target lost) | −10 | True |

### Layer 2 — Efficiency/Stability (per step)
```
R2 = -w_time  -  w_ang_vel × ||ω||²  -  w_action_smooth × ||Δa||²
```

### Layer 3 — Approach (per step, gradient of predicted impact)
```
R3 = w_dist × (exp(-k1 × d_impact_now) - exp(-k1 × d_impact_prev))
   + w_heading × cos(Δyaw_to_target)
```
`d_impact` comes from the kinematic predictor, not raw XY distance.

### Layer 4 — Terminal (fires once at drop moment)
```
R4 = w_drop_base × exp(-k2 × d_error)
   + r_success_jackpot  [if d_error ≤ success_threshold]
   - penalty_instability [if ||ω|| > limit_ang_vel OR |roll|/|pitch| > limit_tilt]
```

**Auto-drop:** If `d_impact ≤ auto_drop_threshold` (0.5 m), drop is forced regardless of policy action[4].

## Key Reward Hyperparameters (`config/hyperparams.yaml`)

| Parameter | Value |
|---|---|
| `g` | 9.81 m/s² |
| `auto_drop_threshold` | 0.5 m |
| `k1_potential` | 1.0 |
| `k2_precision` | 5.0 |
| `w_dist` | 10.0 |
| `w_heading` | 1.0 |
| `w_time` | 0.01 |
| `w_ang_vel` | 0.05 |
| `w_action_smooth` | 0.05 |
| `w_drop_base` | 50.0 |
| `r_success_jackpot` | 100.0 |
| `success_threshold` | 0.1 m |
| `penalty_instability` | 50.0 |
| `limit_ang_vel` | 2.0 rad/s |
| `limit_tilt` | 0.26 rad (≈15°) |

## Key SAC Hyperparameters

| Parameter | Value |
|---|---|
| Algorithm | SAC (Stable-Baselines3) |
| `total_timesteps` | 500,000 |
| `learning_rate` | 3.0 × 10⁻⁴ |
| `buffer_size` | 100,000 |
| `batch_size` | 256 |
| `gamma` | 0.99 |
| `learning_starts` | 1,000 |
| `net_arch` | [256, 256] |
| `device` | cuda |

## Best Checkpoint

No training runs completed yet. Checkpoints saved to:
`/workspace/ros2_ws/rl_checkpoints/sac_drop_<N>_steps.zip`

---

# 2. Recent Progress

- **Phase 1–4:** Full Gazebo Harmonic simulation stack (PX4 SITL, ros_gz_bridge, DetachableJoint payload drop)
- **Phase 5 base:** SAC Gymnasium environment (`drone_drop_env.py`) — 15-dim obs, 5-dim action space; `train_sac.py` with TensorBoard + WandB
- **2-layer launch architecture:** `infra.launch.py` + `episode.launch.py` → episode cycle ~12 s
- **Phase 5 enhancements:** `hyperparams.yaml` centralised config; WandB + CUDA + SIGTERM preemption checkpoint; `setup.py` registers yaml as package data
- **Phase 6 — 4-Layer Hierarchical Reward:**
  - `_compute_reward()` fully implemented with Layers 1–4; AeroThrow kinematic predictor; auto-drop at 0.5 m; Layer 4 jackpot + instability penalty
- **Phase 7 — RL Speed Optimisations (2026-03-12):**
  - **Diagnosis:** First run completed 386 steps / 188 episodes in ~3 hours (0 FPS); ~60 s/episode dominated by real-time TAKEOFF→CRUISE; conf==0 termination with camera off caused 1-step episodes
  - **Gazebo physics unlocked** (`x_marker_world.sdf`): `real_time_factor=0`, `real_time_update_rate=0`, `max_step_size=0.01` (100 Hz). Simulation now runs at max CPU speed — expected 3–8× episode speedup
  - **Camera sensor removed** (`x500_bombard/model.sdf`): eliminated 640×480@30 Hz ogre2 GPU rendering; `gz-sim-sensors-system` removed from world plugins
  - **Bridge stripped** (`ros_gz_bridge.yaml`): removed camera image/info and IMU bridges; only clock, payload odometry, drop cmd remain
  - **`use_vision=False` mode** (`drone_drop_env.py`): new `_cfg_use_vision` param; skips `conf==0` Layer 1 termination; synthesises `conf=1.0` in obs when camera absent. Prevents immediate 1-step-episode termination
  - **Kill-episode sleep** reduced 1.5 s → 0.5 s; saves 1 s per reset
  - **infra.launch.py**: vision default `false`; bridge start delay 16 s → 10 s
  - **hyperparams.yaml**: `use_vision: false`, `obs_wait_timeout: 0.10`, `log_freq: 1000`
  - **colcon build**: both `rl_navigation` and `mission_manager` build cleanly

---

# 3. Remaining Tasks (Next Steps)

- [ ] **WandB login inside container** — run `wandb login` and enter API key; verify entity name matches `hyperparams.yaml`
- [ ] **Run optimised training session** (resume from preempt checkpoint or fresh start)
  - `ros2 launch mission_manager infra.launch.py`  (Gazebo unlocked, no camera, bridge starts at 10 s)
  - `ros2 run rl_navigation train_sac --resume /workspace/ros2_ws/rl_checkpoints/sac_drop_preempt.zip`
- [ ] **Verify RTF=0 speedup** — watch WandB `time/fps` metric; expect 3–8× vs. previous 0 FPS
- [ ] **Evaluate first meaningful run** — check: episode_reward, d_impact trend, Layer 4 reward frequency, jackpot rate
- [ ] **Tune reward weights** — start with `w_dist`, `w_drop_base`, `r_success_jackpot`; adjust if approach gradient dominates or drops too early
- [ ] **Verify auto-drop threshold** — log `d_impact` at each drop; confirm 0.5 m threshold yields meaningful Layer 4 rewards
- [ ] **Multi-env parallelism** — `SubprocVecEnv` if multiple GPUs available
- [ ] **Custom SB3 policy** — add PyTorch AMP (mixed precision) for faster L4 training

---

# 4. Training History (Append-only)

| Date | Run ID | WandB Link | Steps | Mean Drop Error | Notes |
|------|--------|-----------|-------|-----------------|-------|
| 2026-03-12 | vekkz83a | drone-bombard-sac / L4-AutoDrop-v1 | 386 | — | First run aborted; 0 FPS (60 s/episode real-time locked). Optimisations applied in Phase 7. |

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
