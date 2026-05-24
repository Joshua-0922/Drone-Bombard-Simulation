# RL Training Pipeline — Project Log

> **Branch:** `feature/migration-harmonic` | **Purpose:** Context-recovery log for Spot VM preemption and cross-session continuity.

---

# 1. Current State

## Method A — 1-World-4-Payload Architecture (2026-03-21)
**Major architecture upgrade:** 4 enhancements implemented — TAKEOFF skip (altitude teleport), 4-stage curriculum learning, YOLO vision integration (17D obs), Optuna HPO. Requires fresh training start (obs space changed 15→17).

### Training Environment Summary

| Parameter | Value |
|-----------|-------|
| **num_envs** | **1** (stable; multi-env Gazebo lockstep overhead > parallelism benefit) |
| **Simulator** | Gazebo Harmonic + PX4 SITL v1.15.4 |
| **RTF** | **1.0** (`real_time_factor=1`, `real_time_update_rate=100 Hz`, `max_step_size=0.01 s`) |
| **PX4_SIM_SPEED_FACTOR** | **1** (runs at wall-clock speed) |
| **Training started** | **Fresh start** (no `--resume` flag; new SAC model from scratch; WandB run `cj3ytvq2`) |
| **Checkpoint (unused)** | `sac_drop_preempt.zip` exists from dry-run (8K steps) but was NOT loaded |
| **Algorithm** | SAC (Soft Actor-Critic), `net_arch=[256,256]`, `device=cuda` (L4 GPU) |
| **total_timesteps** | 1,000,000 |
| **fps** | **33 fps** (at 20K steps) |
| **ep_len_mean** | 500 (all episodes hit max_steps — policy not navigating to target yet) |
| **ep_rew_mean** | −545 (at 20K steps; improving from −1380 at 2K steps) |
| **Drone model** | `gz_x500_bombard_r0` → PX4 spawns `x500_bombard_r0_0` in Gazebo |
| **Payload** | `payload_0` pre-spawned at (0, 0, 0.14) |
| **Drop trigger** | `d_xy ≤ 0.5 m` (2D horizontal distance, no kinematic prediction) |
| **Drop reward** | Actual physics from `drop_calculator` (`/rl/drop_error`), 10s timeout |
| **Target** | (11, 10, 0) ENU — X-marker `x_marker_0` |
| **Bridge config** | `/tmp/ros_gz_bridge_0.yaml` — `payload_0/odometry` + `x500_0/drop` + camera image |
| **Obs space** | **17D** (was 15D): +bbox_width, +bbox_height (vision features) |
| **Curriculum** | 4-stage: Close(3-8m) → Medium(8-20m) → Full(20-50m) → Vision(20-50m, pure vision) |
| **TAKEOFF** | Skipped via altitude teleport (`skip_takeoff=true`) — saves 10-20s/episode |
| **Optuna** | `tune_optuna.py` — SAC HPO with MedianPruner, SQLite persistence |
| **WandB** | `cj3ytvq2` (production) · `ljbn3wfg` (dry-run) |

### Multi-env Test Results (2026-03-20)

| Config | Result |
|--------|--------|
| num_envs=1 | ✅ 31-33 fps, stable, no ODE crashes |
| num_envs=2 | ❌ 10 fps total (6× overhead); both CRUISE timeouts due to Gazebo compute bottleneck |
| num_envs=4 | ❌ 2 of 4 PX4 instances crash (lockstep timeout); shared Gazebo can't serve 4 drones at RTF=1 |
| **Root cause** | Single Gazebo process handles all drone physics; shared lockstep serializes at >1 drone |

### Key Fixes (Method A Debugging, 2026-03-20)

1. **ODE AABB crash fix**: `PX4_GZ_MODEL_POSE` changed from `0,0,5` to `0,y,0`. Drone spawned at z=5 fell 5m during 5s EKF warmup, hitting ground at ~10 m/s → extreme contact + motor forces on spin-up → ODE integer overflow. Spawning at z=0 (sitting on ground) eliminates free-fall.
2. **Gz world reset removed from episode cycle**: `model_only` reset does NOT reposition PX4-spawned models (they keep their current position). Calling reset during flight caused DetachableJoint inconsistency. Episode reset now just restarts drone_controller + mission_manager; drone takes off from wherever it is.
3. **COM_OF_LOSS_T 5.0→10.0s**: Race condition — episode kill stops OFFBOARD, PX4 AUTO.LAND fired at t=5s exactly when new drone_controller (5s EKF warmup) tried to re-enter OFFBOARD. Increasing timeout eliminates the race.
4. **PX4 namespace fix**: Instances 1-3 use `/px4_N/fmu/*` topics (PX4 `-i N` flag). `_RLBridgeNode` and drone_controller now use `px4_topic_prefix` to correctly address namespaced topics.

**Current infra config:**
- RTF=1 (`x_marker_world.sdf`: `real_time_factor=1, real_time_update_rate=100`)
- `PX4_SIM_SPEED_FACTOR=1`, `PX4_GZ_MODEL_POSE=0,{model_y},0,0,0,0`
- `obs_wait_timeout: 0.02` (20ms), `use_vision: false`

---

# 2. Recent Progress

- **2026-05-22:** Isaac Lab Phase 1 — `isaac_lab_tasks/` (`--task Isaac-DroneDrop-v0`, payload kinematic_sync/fixed_joint, 보상 패리티, smoke/benchmark 스크립트). 학습은 ROS2 없이 GPU 텐서 경로.

- **4-Enhancement Architecture Upgrade (2026-03-21):**
  - **Task 4 — TAKEOFF skip:** `_gz_reset_poses()` now teleports drone to `cruise_altitude` (5.0m). `mission_manager_node` gains `skip_takeoff` ROS parameter → bypasses TAKEOFF state, directly enters CRUISE once armed. Saves 10-20s per episode reset. `cruise_poll_timeout` reduced 60→20s.
  - **Task 2 — Curriculum learning:** 4-stage progression (Close 3-8m, Medium 8-20m, Full 20-50m, Vision 20-50m). Auto-advances when `success_rate > threshold` over `advance_window` episodes. Stage-specific `max_steps` and `use_vision` toggle. Stage 4 masks `rel_x, rel_y` for pure vision-based navigation. `CurriculumCallback` logs stage to WandB.
  - **Task 1 — YOLO vision integration:** Obs space expanded 15→17D (+`bbox_width_norm`, +`bbox_height_norm`). `_RLBridgeNode` subscribes to `/vision/detections` (DetectionResult). Camera image bridge added to infra bridge YAML. YOLO node launched at infra level (not per-episode). Stage 4 masks ground-truth `rel_x, rel_y` → agent uses only vision features.
  - **Task 3 — Optuna HPO:** New `tune_optuna.py` script. Searches SAC params (lr, buffer_size, batch_size, gamma, tau, net_arch) with TPE sampler + MedianPruner. SQLite storage for Spot VM resilience. Each trial logs to WandB group. `close(keep_infra=True)` for Gazebo reuse between trials.
  - **Breaking change:** obs 15→17D requires fresh training start. All existing checkpoints incompatible.

- **Method A Dry-run Passed (2026-03-20):**
  - Discovered `PX4_GZ_MODEL_NAME` (pre-spawn mode) causes ODE crash: all 4 motor plugins activate simultaneously at physics step 1. Switched to dynamic spawn via `PX4_SIM_MODEL`.
  - **PX4 airframe discovery**: rcS `sed` searches `[digits]_${PX4_SIM_MODEL}` in rootfs airframes. Created `4016~4019_gz_x500_bombard_r{0-3}` in `/opt/PX4-Autopilot/build/px4_sitl_default/rootfs/etc/init.d-posix/airframes/`.
  - **Model naming**: `PX4_SIM_MODEL=gz_x500_bombard_rN` → PX4 strips `gz_` prefix → spawns `x500_bombard_rN` → appends instance suffix → final Gazebo entity: `x500_bombard_rN_N`.
  - Created `gz_x500_bombard_r{0-3}/` model dirs (copies of `x500_bombard_r{0-3}/` with updated model.config + SDF names).
  - `drone_drop_env.py`: `_px4_sim_model = gz_x500_bombard_rN`, `_model_name = x500_bombard_rN_N` (actual Gazebo entity).
  - **px4_msgs fix**: Must source `/root/ros2_ws/install/setup.bash` before training launch (container .bashrc-only, not inherited by non-interactive `docker exec`).
  - Removed world SDF drone pre-spawns; updated `_start_infra()` to use `PX4_SIM_MODEL` dynamic spawn.
  - git large-file fix: `git-filter-repo` removed `ros2_ws/wandb/` + `ros2_ws/rl_checkpoints/` from history; added to `.gitignore`.
  - **Dry-run result**: 31 fps, 16 episodes, 0 ODE crashes, WandB run `ljbn3wfg`.

- **Method A Architecture (2026-03-20, initial implementation):**
  - 1-World-4-Payload architecture: 4 payloads (payload_0~3) pre-spawned at 150m Y-offsets in shared Gazebo world SDF.
  - Created 4 drone model variants (`x500_bombard_r0~3`) with per-instance `DetachableJoint` (child_model=payload_N, topic=/x500_N/drop).
  - Created 4 payload model variants (`payload_0~3`) with per-instance `OdometryPublisher` (/model/payload_N/odometry).
  - Updated `x_marker_world.sdf`: 4 X-markers, 4 payloads (drones spawned dynamically by PX4).
  - `drone_drop_env.py`: per-instance bridge config (`/tmp/ros_gz_bridge_N.yaml`); each instance starts own bridge.
  - Replaced `_predict_impact_point` with `_compute_d_xy` (pure 2D horizontal distance, zero kinematics).
  - Layer 4 reward: waits for actual `drop_error` from `drop_calculator` queue (real Gazebo physics) — `info['drop_error_actual_m']` + `info['is_success']`.
  - `train_sac.py`: `time.sleep(rank * 10)` stagger in SubprocVecEnv factory.
  - `hyperparams.yaml`: `num_envs: 1` (DRYRUN), `env_stagger_secs: 10`, `drop_wait_timeout: 10.0`.

- **Phase 1.5 Debugging (2026-03-19):**
  - Diagnosed ODE AABB crash: drone spawning at z=5 falls 5m during EKF warmup → high-speed ground impact → crash on motor spin-up. Fixed by `PX4_GZ_MODEL_POSE=0,y,0` (spawn on ground).
  - Removed gz world reset from episode cycle: model_only reset doesn't work for PX4-spawned models. Episode reset now just kills/restarts episode nodes (no Gazebo service call).
  - Fixed COM_OF_LOSS_T race: 5s timeout matched EKF warmup exactly → AUTO.LAND preempted new OFFBOARD. Increased to 10s.
  - Single-env self-managed training stable at fps=30-31. WandB run `nynxn6b5`.

- **Phase 1.5 — Multi-Instance Parallel Training (2026-03-18):**
  - Implemented self-managed infra in `DroneDropEnv` — each instance launches its own Gazebo+PX4+bridge+agent
  - Added `instance_id` parameter, per-instance MicroXRCEAgent port, ROS_DOMAIN_ID isolation
  - PX4 namespace fix: `px4_topic_prefix` for `/px4_N/fmu/*` topics (instances > 0)
  - drone_controller topic remapping via `--ros-args -r` for instances > 0
  - Replaced global `pkill` in `_kill_episode()` with process-group-only kill
  - Updated `train_sac.py` factory to pass `instance_id` to `DroneDropEnv`
  - `start_infra_clean.sh` updated for multi-instance lock file cleanup

- **Phase 1 Curriculum — Fresh Start (2026-03-18):**
  - Disabled manual drop in `drone_drop_env.py` — auto-drop only (`d_impact <= 0.5m`)
  - Added `WandbMetricsCallback` to `train_sac.py` — logs `ep_rew_mean`, `ep_len_mean`, losses to WandB
  - Discarded old replay buffer, started fresh training (1M timesteps), WandB run `pbpqa0rp`
  - Updated `train_managed.sh` to support `--fresh` flag; bumped `total_timesteps` to 1M
  - Fixed launch bug: manual restart missed px4_msgs source → episode nodes crashed

- **Phase 13 — Stability Fixes (2026-03-18):**
  - Fixed `start_infra_clean.sh` kill pattern (`bin/px4` not `/px4 `)
  - Fixed stale FastRTPS shm causing IMU timestamp chaos
  - Fixed NaN observation SB3 crash: `nan_to_num` guards in ROS2 callbacks + `_get_obs()`
  - Fixed `PX4_GZ_MODEL_POSE` missing from infra.launch.py (drone spawned at z=0.5, payload at z=5.14 → 4.5m DetachableJoint gap → physics explosion → Gazebo ODE crash)
  - Added 5s EKF warmup in `drone_controller_node.py` (wait 100 ticks before first arm to let EKF2 converge)
  - Set RTF=1 in world.sdf + `PX4_SIM_SPEED_FACTOR=1` in infra.launch.py
  - Set `UXRCE_DDS_SYNCT=0` in airframe (safe at RTF=1, eliminates startup time jump → EKF NaN chain)
  - Airframe file persisted to repo: `drone_drop_system/docker/config/airframes/4015_gz_x500_bombard`
  - Dockerfile updated to COPY airframe into PX4 ROMFS during image build
  - **fps=12 stable**, 0 CRUISE timeouts, 0 ODE crashes, Gazebo alive

- **Phase 1–4:** Full Gazebo Harmonic simulation stack (PX4 SITL, ros_gz_bridge, DetachableJoint payload drop)
- **Phase 5 base:** SAC Gymnasium environment (`drone_drop_env.py`) — 15-dim obs, 5-dim action space; `train_sac.py` with TensorBoard + WandB
- **Phase 6 — 4-Layer Hierarchical Reward:** `_compute_reward()` with Layers 1–4; AeroThrow kinematic predictor; auto-drop at 0.5m; Layer 4 jackpot + instability penalty
- **Phase 12 — OFFBOARD/TAKEOFF Fix (2026-03-18):** Fixed OFFBOARD retry race condition, absolute TAKEOFF altitude check; fps 4→23.

---

# 3. Remaining Tasks (Next Steps)

- [x] **Multi-instance self-managed infra** — Phase 1.5: DroneDropEnv._start_infra() implemented
- [x] **Verify single-env training** — stable at fps=30-31 with self-managed infra
- [x] **Method A architecture** — 1-World-4-Payload: PX4 dynamic spawn (PX4_SIM_MODEL=gz_x500_bombard_rN), per-instance bridge, actual physics reward, staggered SubprocVecEnv init
- [x] **Single-env dry-run** — 31 fps, 16 episodes, 0 ODE crashes (run `ljbn3wfg`)
- [x] **Multi-env testing** — 4-env: PX4 lockstep crash (2 of 4 PX4s die); 2-env: 10 fps total vs 31 fps single (lockstep overhead). Fallback to num_envs=1.
- [x] **Production training started** — num_envs=1, 31 fps, actual physics reward (d_xy + drop_calculator)
- [x] **WandB reward component monitoring** — `WandbMetricsCallback` now logs `env/mean_d_xy`, `env/mean_rew_ctrl`, `env/mean_rew_dist`, `env/mean_rew_orient`, `env/mean_rew_drop`, `env/drop_error_actual_m`, `env/success_rate`, `env/drop_count`; `drone_drop_env.py` exposes split `rew_*` keys in info dict for both terminal and non-terminal steps
- [x] **Physics explosion guard** — `step()` checks `d_xy > 500 m` immediately after computing distance; terminates episode with −100 penalty before any corrupted transition reaches the replay buffer; logs `info['physics_glitch'] = True`
- [x] **Spawn altitude fix** — `PX4_GZ_MODEL_POSE` changed from `z=0` to `z=0.5` (0.5 m above ground) to prevent geometry overlap between drone landing gear and ground plane at spawn
- [x] **Corrupted replay buffer quarantined** — `sac_drop_preempt_replay.pkl` from the explosion session (Mar 20 07:53) renamed to `.CORRUPTED_20260320`; resumed from `sac_drop_95000_steps.zip` (Mar 18, clean) with fresh replay buffer
- [x] **Three-layer physics explosion defence** — (1) `_on_local_pos` now rejects finite positions with `|pos| > 1000 m` (retains last-known-good), blocking explosions at source; (2) explosion guard upgraded: `not isfinite(d_xy) or d_xy > 500` catches NaN hole; (3) glitch step uses key `glitch_d_xy` not `d_xy`, so WandbMetricsCallback never averages it into `env/mean_d_xy`; glitch count logged as `env/physics_glitch_count`
- [x] **Linear distance reward** — replaced `exp(−k1·d)` potential (k1=1.0 → saturated to ~0 for d > 10 m, giving `rew_dist = 0` at d=45 m) with linear `r3_dist = w_dist × (d_prev − d_xy)`; nonzero gradient at any distance; `w_dist` rescaled 10.0 → 1.0 (linear is unbounded)
- [x] **CRUISE retry on timeout** — `reset()` now retries `_start_episode()` + `_wait_for_cruise()` once if CRUISE not reached; prevents 500-step crash-penalty episodes from polluting replay buffer when PX4 arm race fires
- [x] **TAKEOFF skip** — teleport to cruise altitude, mission_manager `skip_takeoff=true`
- [x] **4-stage curriculum** — distance-based stages + vision stage (Stage 4)
- [x] **YOLO vision 17D obs** — bbox features, DetectionResult subscription, camera bridge
- [x] **Optuna HPO script** — `tune_optuna.py` with SAC param search
- [ ] **Fresh training with 4-stage curriculum** — build, deploy, dry-run 3 episodes, then full 1M steps
- [ ] **Run Optuna study** — 50 trials × 50K steps after curriculum training baseline
- [ ] **Investigate RTF>1** — try PX4_SIM_SPEED_FACTOR>1 to increase training speed
- [ ] **Phase 2 curriculum** — re-enable manual drop once policy navigates to target reliably
- [ ] **Custom SB3 policy** — PyTorch AMP for faster L4 GPU training
- [ ] **Redirect PX4 logs to /dev/null** — `/tmp/px4_{i}.log` files grow to 100+ MB per session

---

# 4. Training History (Append-only)

| Date | Run ID | WandB Link | Steps | Mean Drop Error | Notes |
|------|--------|-----------|-------|-----------------|-------|
| 2026-03-12 | vekkz83a | drone-bombard-sac / L4-AutoDrop-v1 | 386 | — | First run aborted; 0 FPS (60 s/episode real-time locked). Optimisations applied in Phase 7. |
| 2026-03-13 | vekkz83a | drone-bombard-sac / L4-AutoDrop-v1 | resumed | — | Resumed from preempt checkpoint + replay buffer. RTF=0, headless, no camera. WANDB_RUN_ID set to reattach existing run. |
| 2026-03-17 | apax52d7 | drone-bombard-sac / L4-AutoDrop-v1 | ~3K (started) | — | New session: fixed dartsim crash (model_only reset), EKF yaw fix (COM_ARM_WO_GPS=1), removed PX4_SIM_SPEED_FACTOR. ~25 steps/sec, ep_len=6 (early exploration). |
| 2026-03-17 | 27mbu6qk | drone-bombard-sac / L4-AutoDrop-v1 | 49,242 | — | Resumed from preempt checkpoint after replay buffer incompatibility fix. TAKEOFF optimisations: min_armed_secs=0.3, altitude_hold_ticks≥2, world_reset_sleep=0.5s. fps improved 2→4. ep_len_mean=219, ep_rew_mean=-367. |
| 2026-03-18 | 9nbwg71r | drone-bombard-sac / L4-AutoDrop-v1 | 53,428 | — | Phase 12: Fixed OFFBOARD retry race (2s→0.5s) + absolute TAKEOFF altitude check. fps jumped 4→23. 0 CRUISE timeouts. |
| 2026-03-18 | 2h1cvmer | drone-bombard-sac / L4-AutoDrop-v1 | 78,200 | — | Phase 13 early: Fixed multi-instance (bin/px4 pattern), stale shm, NaN obs crash. fps=7 briefly. |
| 2026-03-18 | 53xx3o8u | drone-bombard-sac / L4-AutoDrop-v1 | 80,292+ | — | Phase 13 final: Fixed PX4_GZ_MODEL_POSE (drone-payload gap), 5s EKF warmup in drone_controller, RTF=1, UXRCE_DDS_SYNCT=0. **fps=12 stable, 0 ODE crashes.** |
| 2026-03-18 | dy97unuj | drone-bombard-sac / L4-AutoDrop-v1 | 546K–560K | — | **Broken run** — missing px4_msgs source caused episode nodes to crash. All episodes stuck in IDLE, 60s CRUISE timeout per reset, fps=5, reward spiraled to -1,560. Killed. |
| 2026-03-18 | izf10080 | drone-bombard-sac / L4-AutoDrop-v1 | 564K–568K | — | **Stale buffer run** — resumed from polluted checkpoint. ep_rew=-1,900, killed in favor of fresh start. |
| 2026-03-18 | pbpqa0rp | drone-bombard-sac / L4-AutoDrop-v1 | 0 (fresh) | — | **Fresh Phase 1 training.** Manual drop disabled, WandB metrics callback added. ep_rew=-91 at 6K steps, ent_coef=0.74, fps=28-30. 1M timesteps target. |
| 2026-03-19 | a9f6lk57 | drone-bombard-sac / L4-AutoDrop-v1 | — | — | **Debug run** — first attempt at self-managed infra (_start_infra). ODE crash immediately (z=5 spawn → free-fall → ground impact → motor spin-up crash). Killed. |
| 2026-03-19 | 6dopfyjn | drone-bombard-sac / L4-AutoDrop-v1 | ~96K–102K | — | **Debug run** — second attempt (world reset removed). Still crashed until spawn height fixed. Then CRUISE timeouts from COM_OF_LOSS_T race. Killed. |
| 2026-03-19 | nynxn6b5 | drone-bombard-sac / L4-AutoDrop-v1 | 102K+ (running) | — | **Self-managed infra stable.** Spawn at z=0 (no free-fall), world reset removed, COM_OF_LOSS_T=10s. **fps=30-31 stable**, 0 ODE crashes, 0 CRUISE timeouts. 1M timesteps target. |
| 2026-03-20 | ljbn3wfg | drone-bombard-sac / L4-AutoDrop-v1 | 8K (dry-run) | — | **Method A dry-run.** Dynamic PX4 spawn (PX4_SIM_MODEL=gz_x500_bombard_r0), px4_msgs fix (source /root/ros2_ws). **31 fps, 16 episodes, 0 ODE crashes.** Infra stable — proceeding to 4-env test. |
| 2026-03-20 | cj3ytvq2 | drone-bombard-sac / L4-AutoDrop-v1 | 20K (running) | — | **Production Method A training — FRESH START.** num_envs=1, RTF=1, 33 fps, actual physics reward (d_xy + drop_calculator). ep_rew_mean=−545 at 20K. Multi-env failed (see table above). 1M timesteps. |
| 2026-03-20 | mjfet61f | drone-bombard-sac / L4-AutoDrop-v1 | 52K (resumed) | — | **WandB reward monitoring added.** Resumed from preempt (~52K steps). New metrics: `env/mean_d_xy`, `env/mean_rew_ctrl`, `env/mean_rew_dist`, `env/mean_rew_orient`, `env/drop_error_actual_m`, `env/success_rate`. **KILLED** — d_xy exploded to 1.98e11 (Gazebo physics glitch). Replay buffer corrupted. |
| 2026-03-20 | 53samoqz | drone-bombard-sac / L4-AutoDrop-v1 | 95K (resumed) | — | **Post-explosion recovery.** Resumed from `sac_drop_95000_steps.zip` (clean, Mar 18). Fresh replay buffer. Physics explosion guard added (d_xy > 500 → terminate + −100 penalty). Spawn altitude changed 0→0.5 m. **KILLED** — mean_d_xy still spiking (WandB callback was still logging the glitch d_xy value; _on_local_pos magnitude guard missing). |
| 2026-03-20 | naf4zyhm | drone-bombard-sac / L4-AutoDrop-v1 | 104K (resumed) | — | **Three-layer explosion defence.** (1) `_on_local_pos` rejects \|pos\| > 1000 m; (2) guard catches NaN + > 500 m; (3) glitch key renamed → WandB mean clean. `env/physics_glitch_count` now monitored. **KILLED** — `mean_rew_dist=0`, `mean_d_xy` stuck 45.8 m: exponential potential k1=1.0 saturated to ~0; CRUISE timeouts worsening (ep_rew −110 → −591). |
| 2026-03-20 | 8otphxy8 | drone-bombard-sac / L4-AutoDrop-v1 | 114K (resumed) | — | **Linear distance reward + CRUISE retry.** `r3_dist = w_dist*(d_prev−d_xy)`, w_dist=1.0. Nonzero gradient at any distance. CRUISE retry prevents crash-penalty episodes. |

---

## Quick-Start After Preemption

```bash
# 1. Reconnect to container
xhost +local:docker && docker start -ai drone-bombard-harmonic

# 2. Clean all stale processes (infra is now self-managed by each env)
docker exec drone-bombard-harmonic bash /workspace/ros2_ws/start_infra_clean.sh

# 3. Build rl_navigation package (if code changed)
docker exec drone-bombard-harmonic bash -c "cd /workspace/ros2_ws && source /opt/ros/humble/setup.bash && source /root/ros2_ws/install/setup.bash && colcon build --packages-select rl_navigation && source install/setup.bash"

# 4. Start training (self-manages all infra internally)
# IMPORTANT: source /root/ros2_ws before train_sac for px4_msgs availability
docker exec -d drone-bombard-harmonic bash -c "source /opt/ros/humble/setup.bash && source /root/ros2_ws/install/setup.bash && source /workspace/ros2_ws/install/setup.bash && cd /workspace/ros2_ws && ros2 run rl_navigation train_sac > /tmp/train.log 2>&1"

# 5. Monitor
docker exec drone-bombard-harmonic tail -20 /tmp/train_managed.log
```
