# RL Training Pipeline — Project Log

> **Branch:** `feature/migration-harmonic` | **Purpose:** Context-recovery log for Spot VM preemption and cross-session continuity.

---

# 1. Current State

## Phase 1.5 — Multi-Instance Parallel Training (2026-03-18)
**Implemented self-managed infra** — each `DroneDropEnv` instance launches its own Gazebo + PX4 + MicroXRCEAgent + ros_gz_bridge. No more external `infra.launch.py` required.

| Metric | Value |
|--------|-------|
| num_envs | **4** (via `SubprocVecEnv`) |
| Isolation | `ROS_DOMAIN_ID` per instance + `GZ_PARTITION` per Gazebo |
| PX4 port | `PX4_UXRCE_DDS_PORT=8888+instance_id` |
| PX4 lock | `-i instance_id` → `/tmp/px4_lock-{id}` |
| PX4 namespace | Instance 0: none; Instance N>0: `/px4_N/` (remapped for drone_controller) |
| Episode launch | Direct `ros2 run` per node (no launch file) — enables per-node remapping |
| Episode kill | Process-group SIGTERM only — no global `pkill` (multi-instance safe) |
| Infra startup | Sequential: UXRCE(t=0) → Gazebo(t=0) → bridge(t=10s) → PX4(t=20s) |
| PX4 readiness | Poll `_obs_ready` (set by `_on_local_pos` callback), 90s timeout |
| Curriculum | **Phase 1: manual drop disabled** (auto-drop at d_impact ≤ 0.5m) |
| Best model | `BestModelCallback` saves to `checkpoints/best_model/` on new ep_rew_mean high |
| total_timesteps | 1,000,000 |

### Key Changes (Phase 1.5)

1. **`drone_drop_env.py`:**
   - `instance_id` parameter in `__init__` — drives all isolation
   - `_start_infra()` / `_kill_infra()` — self-managed Gazebo+PX4+bridge+agent per env
   - `_RLBridgeNode` accepts `px4_topic_prefix` for namespaced PX4 topics
   - `_start_episode()` launches nodes via `ros2 run` with PX4 topic remapping
   - `_kill_episode()` uses process-group kill only (removed global pkill)
   - `_gz_world_reset()` passes `GZ_PARTITION` to `gz service`
   - `close()` calls `_kill_infra()`
2. **`train_sac.py`:** Updated `_make_env` factory to pass `instance_id`
3. **`hyperparams.yaml`:** `num_envs: 4`
4. **`start_infra_clean.sh`:** Cleanup-only (cleans instances 0-3 lock files)

**Current infra config:**
- RTF=1 (`x_marker_world.sdf`: `real_time_factor=1, real_time_update_rate=100`)
- `PX4_SIM_SPEED_FACTOR=1`, `PX4_GZ_MODEL_POSE=0,0,5,0,0,0`
- `obs_wait_timeout: 0.02` (20ms), `use_vision: false`

---

# 2. Recent Progress

- **Phase 1.5 — Multi-Instance Parallel Training (2026-03-18):**
  - Implemented self-managed infra in `DroneDropEnv` — each instance launches its own Gazebo+PX4+bridge+agent
  - Added `instance_id` parameter, `GZ_PARTITION` isolation, `PX4_UXRCE_DDS_PORT` per-instance agent port
  - Replaced global `pkill` in `_kill_episode()` with process-group-only kill
  - Episode nodes launched via direct `ros2 run` with PX4 topic remapping for instances > 0
  - Updated `train_sac.py` factory to pass `instance_id` to `DroneDropEnv`
  - `hyperparams.yaml` set to `num_envs: 4`
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
- **2-layer launch architecture:** `infra.launch.py` + `episode.launch.py` → episode cycle ~12 s
- **Phase 5 enhancements:** `hyperparams.yaml` centralised config; WandB + CUDA + SIGTERM preemption checkpoint; `setup.py` registers yaml as package data
- **Phase 6 — 4-Layer Hierarchical Reward:**
  - `_compute_reward()` fully implemented with Layers 1–4; AeroThrow kinematic predictor; auto-drop at 0.5 m; Layer 4 jackpot + instability penalty
- **Phase 12 — OFFBOARD/TAKEOFF Fix (2026-03-18):**
  - Fixed OFFBOARD retry race condition (2s→0.5s retry in drone_controller)
  - Fixed absolute TAKEOFF altitude check (was relative to arm_ned_z, broken with persistent PX4 EKF lag)
  - fps jumped 4→23 (before multiple instances broke it again)
- **Phase 11 — Training Running (2026-03-17):**
  - Fixed Gazebo Harmonic dartsim crash: `reset: {all: true}` → `{model_only: true}` in `_gz_world_reset()`
  - Fixed EKF yaw preflight failure: added `COM_ARM_WO_GPS=1` + `EKF2_MAG_TYPE=1` to airframe
  - Training started: WandB run `apax52d7`, ~25 steps/sec
- **Phase 10 — Disk Space Optimization (2026-03-16):**
  - TensorBoard disabled, checkpoints capped at 3, docker log limits, 6-hour cron cleanup
- **Phase 9 — Env Setup + Reward Refactor (2026-03-16):**
  - GPU driver 580 DKMS, NVIDIA Container Toolkit 1.19.0
  - Layer 1 reward refactored; num_envs=8→1; W&B configured

---

# 3. Remaining Tasks (Next Steps)

- [x] **Multi-instance parallel training** — Phase 1.5: self-managed infra, 4 parallel envs via SubprocVecEnv
- [ ] **Verify 4-env training** — Set `num_envs: 1` first to test self-managed infra, then scale to 4
- [ ] **Try RTF=3 or higher** — only after multi-env is stable; update `x_marker_world.sdf` + `PX4_SIM_SPEED_FACTOR`
- [ ] **Tune reward weights** — start with `w_dist`, `w_drop_base`, `r_success_jackpot`
- [ ] **Custom SB3 policy** — add PyTorch AMP (mixed precision) for faster L4 training
- [ ] **Phase 2 curriculum** — re-enable manual drop once policy navigates to target reliably

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
docker exec -d drone-bombard-harmonic bash -c "cd /workspace/ros2_ws && /workspace/ros2_ws/train_managed.sh --fresh > /tmp/train_managed.log 2>&1"

# 5. Monitor
docker exec drone-bombard-harmonic tail -20 /tmp/train_managed.log
```
