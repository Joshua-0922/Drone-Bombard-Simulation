# RL Training Pipeline — Project Log

> **Branch:** `feature/migration-harmonic` | **Purpose:** Context-recovery log for Spot VM preemption and cross-session continuity.

---

# 1. Current State

## Disk Space Optimization (2026-03-16 → 2026-03-17)
All disk-space fixes applied. Root cause of 92 GB overflow identified and resolved.

| Fix | Status |
|---|---|
| TensorBoard disabled (`tensorboard_log=None`, `sync_tensorboard` removed) | ✅ |
| `CheckpointCallback`: `save_replay_buffer=False` | ✅ |
| `CleanupOldCheckpointsCallback`: keep last 3 `.zip` files | ✅ |
| WandB offline-run directory pruning (>7 days) on startup | ✅ |
| `hyperparams.yaml`: `max_checkpoints_kept: 3`, `log_dir` removed | ✅ |
| `docker-compose.yml` at `/opt/drone-bombard/`: log driver `json-file` max 10 MB × 3 | ✅ |
| **`px4_build.log` (92 GB) deleted** — PX4 SITL runtime flooded log via `pxh>` ANSI prompts | ✅ |
| **`build_px4.sh` created** — enforces `DONT_RUN=1` so build exits immediately, no SITL runtime | ✅ |
| **`start_training.sh` Fix 3** — truncates `px4_build.log` + kills stray PX4 SITL after build confirms | ✅ |
| **Cron fixed** — removed `-a` from `docker system prune` (was deleting 50 GB image when container stopped) | ✅ |
| **Cron added** — `docker exec drone-bombard-harmonic rm -rf /root/.ros/log/*` cleans container ROS logs | ✅ |
| **`docker run` updated in CLAUDE.md** — added `--log-driver=json-file --log-opt max-size=10m --log-opt max-file=3` | ✅ |

## Environment Setup (2026-03-16)
| Component | Status |
|---|---|
| GPU | NVIDIA L4, driver 580.126.09, CUDA 13.0 — `nvidia-smi` ✅ |
| DKMS modules | Built + installed for kernel `6.8.0-1048-gcp` |
| NVIDIA Container Toolkit | 1.19.0 installed; Docker nvidia runtime configured ✅ |
| Docker | 29.3.0, `--gpus all` working inside containers ✅ |
| Old packages purged | `nvidia-settings` 510 fully purged |

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

Preempt checkpoint (from Phase 7 Spot VM preemption):
- Model: `/workspace/ros2_ws/rl_checkpoints/sac_drop_preempt.zip`
- Replay buffer: `/workspace/ros2_ws/rl_checkpoints/sac_drop_preempt_replay.pkl`
- Periodic checkpoints: `/workspace/ros2_ws/rl_checkpoints/sac_drop_<N>_steps.zip`

---

# 2. Recent Progress

- **Phase 1–4:** Full Gazebo Harmonic simulation stack (PX4 SITL, ros_gz_bridge, DetachableJoint payload drop)
- **Phase 5 base:** SAC Gymnasium environment (`drone_drop_env.py`) — 15-dim obs, 5-dim action space; `train_sac.py` with TensorBoard + WandB
- **2-layer launch architecture:** `infra.launch.py` + `episode.launch.py` → episode cycle ~12 s
- **Phase 5 enhancements:** `hyperparams.yaml` centralised config; WandB + CUDA + SIGTERM preemption checkpoint; `setup.py` registers yaml as package data
- **Phase 6 — 4-Layer Hierarchical Reward:**
  - `_compute_reward()` fully implemented with Layers 1–4; AeroThrow kinematic predictor; auto-drop at 0.5 m; Layer 4 jackpot + instability penalty
- **Phase 10 — Disk Space Optimization (2026-03-16):**
  - TensorBoard logging disabled (`tensorboard_log=None`, `sync_tensorboard` removed from `wandb.init`)
  - `CheckpointCallback.save_replay_buffer=False` — only SIGTERM preempt handler saves replay buffer
  - Added `CleanupOldCheckpointsCallback`: deletes oldest `.zip` files, keeping only last 3
  - Added WandB offline-run directory pruning (entries older than 7 days) on training startup
  - `hyperparams.yaml`: removed `log_dir`, added `max_checkpoints_kept: 3`
  - Created `/opt/drone-bombard/docker-compose.yml` with `json-file` log driver (10 MB × 3 = 30 MB max)
  - `nachoigpt` cron job: every 6 hours — clears ROS2 logs, `docker system prune -af`, `journalctl --vacuum-time=1d`
- **Phase 9 — Env Setup + Reward Refactor (2026-03-16):**
  - GPU driver 580 DKMS modules built for kernel `6.8.0-1048-gcp`; nvidia-smi working
  - NVIDIA Container Toolkit 1.19.0 installed; Docker nvidia runtime configured
  - Old `nvidia-settings` 510 package purged
  - **Layer 1 reward refactored**: crash / overspeed / target-lost now apply configurable penalties (`penalty_crash`, `penalty_overspeed`, `penalty_target_lost`) instead of hard termination — episode continues
  - **num_envs=8**: added to `hyperparams.yaml`; `train_sac.py` now wraps with `SubprocVecEnv` (each subprocess gets its own `ROS_DOMAIN_ID`)
  - W&B API key configured
- **Phase 8 — Training Resume (2026-03-13):**
  - Spot VM preempted after 386 steps; container restarted, infra relaunched headless
  - Resumed from `sac_drop_preempt.zip` + replay buffer; WandB run `vekkz83a` (L4-AutoDrop-v1) reattached via `WANDB_RUN_ID`
  - All processes running detached (`docker exec -d`): survive Termius disconnection
  - Fixed missing source: must source `/root/ros2_ws/install/setup.bash` for `ros_gz_bridge`
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

- [x] **WandB login inside container** — API key set; entity `nayoonho0922-seoul-national-university` confirmed in hyperparams.yaml
- [x] **Run optimised training session** — resumed 2026-03-13 from preempt checkpoint; WandB run vekkz83a reattached; replay buffer restored
- [x] **Disk space fixes applied** — TB disabled, checkpoints capped at 3, docker log limits, 6-hour cron cleanup
- [ ] **Verify disk fix works** — after 15k steps: only 3 `.zip` files in `rl_checkpoints/`, no `rl_logs/` dir, `docker inspect` shows json-file log config
- [ ] **Verify RTF=0 speedup** — watch WandB `time/fps` metric; expect 3–8× vs. previous 0 FPS
- [ ] **Evaluate first meaningful run** — check: episode_reward, d_impact trend, Layer 4 reward frequency, jackpot rate
- [ ] **Tune reward weights** — start with `w_dist`, `w_drop_base`, `r_success_jackpot`; adjust if approach gradient dominates or drops too early
- [ ] **Verify auto-drop threshold** — log `d_impact` at each drop; confirm 0.5 m threshold yields meaningful Layer 4 rewards
- [x] **Multi-env parallelism** — `SubprocVecEnv` with num_envs=8; each env isolated via `ROS_DOMAIN_ID`
- [ ] **Custom SB3 policy** — add PyTorch AMP (mixed precision) for faster L4 training

---

# 4. Training History (Append-only)

| Date | Run ID | WandB Link | Steps | Mean Drop Error | Notes |
|------|--------|-----------|-------|-----------------|-------|
| 2026-03-12 | vekkz83a | drone-bombard-sac / L4-AutoDrop-v1 | 386 | — | First run aborted; 0 FPS (60 s/episode real-time locked). Optimisations applied in Phase 7. |
| 2026-03-13 | vekkz83a | drone-bombard-sac / L4-AutoDrop-v1 | resumed | — | Resumed from preempt checkpoint + replay buffer. RTF=0, headless, no camera. WANDB_RUN_ID set to reattach existing run. |

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
