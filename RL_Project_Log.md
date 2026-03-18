# RL Training Pipeline — Project Log

> **Branch:** `feature/migration-harmonic` | **Purpose:** Context-recovery log for Spot VM preemption and cross-session continuity.

---

# 1. Current State

## Training Phase 13 — Running (2026-03-18)
Training actively running. WandB run: `2h1cvmer` (L4-AutoDrop-v1).

| Metric | Value |
|--------|-------|
| Steps so far | ~78,200 (at 45s elapsed from Phase 13 start) |
| Throughput | **7 fps** (RTF=2 stable, single clean instance) |
| ep_len_mean | 46.7 |
| ep_rew_mean | -40.6 |
| Episodes | 1,452 (cumulative) |
| Gazebo | Alive |
| PX4 | Armed + Offboard confirmed |
| CRUISE timeouts | 0 |

**Root causes diagnosed and fixed in Phase 13 (this session):**
1. **Multiple instance proliferation (4 PX4, 3 Gazebo, 7 drone_controllers)** — Multiple `docker exec -d` calls each started a full new infra. Fix: `start_infra_clean.sh` kills all matching processes by PID before starting.
2. **Stale FastRTPS shm** — 169 stale `/dev/shm/fastrtps_*` segments served stale DDS timestamps to new processes. Fix: `rm -f /dev/shm/fastrtps_*` in clean script.
3. **`start_infra_clean.sh` kill pattern bug** — Pattern `/px4 ` (trailing space) didn't match PX4 processes with no arguments. Fix: changed to `bin/px4`.
4. **Battery failsafe** — `SIM_BAT_DRAIN=60 mAh/s` drains battery over long persistent PX4 sessions. Fix: `param set SIM_BAT_DRAIN 0` in airframe.
5. **Magnetometer arming denial** — RTF>1 DDS time sync disruptions cause mag spikes. Fix: `EKF2_MAG_CHECK=0`, `COM_ARM_MAG_STR=0`, `COM_ARM_MAG_ANG=180`.
6. **OFFBOARD loss failsafe** — DDS time sync gaps trigger OFFBOARD loss. Fix: `COM_OF_LOSS_T=5.0s`.
7. **NaN observation crash** — EKF velocity→NaN during DDS time sync reset → SB3 receives NaN obs → `ValueError: Expected loc to satisfy Real()` → exit code 134. Fix: `np.nan_to_num(..., nan=0.0)` in `drone_drop_env._get_obs()`.
8. **`UXRCE_DDS_SYNCT=0` attempted but reverted** — Disabling DDS timestamp sync eliminated time jumps but broke OFFBOARD control (ROS2 sim-time setpoint timestamps mismatched PX4 POSIX clock at RTF=2). Reverted to default (1).

**Current airframe params** (`4015_gz_x500_bombard` in container + persisted to repo):
- `FD_FAIL_R=0`, `SIM_BAT_DRAIN=0`, `COM_ARM_WO_GPS=1`
- `EKF2_MAG_CHECK=0`, `COM_ARM_MAG_STR=0`, `COM_ARM_MAG_ANG=180`
- `COM_OF_LOSS_T=5.0`

**Current config:**
- `obs_wait_timeout: 0.02` (20ms)
- `num_envs: 1`
- `use_vision: false`
- `target_altitude: 5.0`
- Checkpoint: `/workspace/ros2_ws/rl_checkpoints/sac_drop_preempt.zip`
- WandB run: `2h1cvmer`

---

# 2. Recent Progress

- **Phase 13 — Stability Fixes (2026-03-18):**
  - Diagnosed and fixed multiple instance proliferation via `start_infra_clean.sh` with correct `bin/px4` pattern
  - Fixed stale FastRTPS shm causing IMU timestamp chaos
  - Fixed NaN observation SB3 crash with `nan_to_num` in `_get_obs()`
  - Investigated `UXRCE_DDS_SYNCT=0` to stop DDS time jumps — reverted (breaks OFFBOARD at RTF>1)
  - Airframe file persisted to repo: `drone_drop_system/docker/config/airframes/4015_gz_x500_bombard`
  - Dockerfile updated to COPY airframe into PX4 ROMFS during image build
  - **fps=7 stable**, 0 CRUISE timeouts

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

- [x] **Multiple instances fixed** — `start_infra_clean.sh` with correct `bin/px4` pattern
- [x] **NaN crash fixed** — `nan_to_num` guard in `_get_obs()`
- [x] **Airframe persisted to repo** — `drone_drop_system/docker/config/airframes/4015_gz_x500_bombard`
- [ ] **Monitor fps stability** — watch for regressions; target: 7 fps sustained over 1000+ episodes
- [ ] **Try RTF=3 or higher** — once fps=7 is confirmed stable for 30+ min; update `x_marker_world.sdf` + `PX4_SIM_SPEED_FACTOR`
- [ ] **Evaluate first meaningful run** — check: episode_reward trend, d_impact trend, Layer 4 reward frequency
- [ ] **Tune reward weights** — start with `w_dist`, `w_drop_base`, `r_success_jackpot`
- [ ] **Custom SB3 policy** — add PyTorch AMP (mixed precision) for faster L4 training
- [ ] **Verify disk fix works** — after 15k steps: only 3 `.zip` files, no `rl_logs/`, json-file log config

---

# 4. Training History (Append-only)

| Date | Run ID | WandB Link | Steps | Mean Drop Error | Notes |
|------|--------|-----------|-------|-----------------|-------|
| 2026-03-12 | vekkz83a | drone-bombard-sac / L4-AutoDrop-v1 | 386 | — | First run aborted; 0 FPS (60 s/episode real-time locked). Optimisations applied in Phase 7. |
| 2026-03-13 | vekkz83a | drone-bombard-sac / L4-AutoDrop-v1 | resumed | — | Resumed from preempt checkpoint + replay buffer. RTF=0, headless, no camera. WANDB_RUN_ID set to reattach existing run. |
| 2026-03-17 | apax52d7 | drone-bombard-sac / L4-AutoDrop-v1 | ~3K (started) | — | New session: fixed dartsim crash (model_only reset), EKF yaw fix (COM_ARM_WO_GPS=1), removed PX4_SIM_SPEED_FACTOR. ~25 steps/sec, ep_len=6 (early exploration). |
| 2026-03-17 | 27mbu6qk | drone-bombard-sac / L4-AutoDrop-v1 | 49,242 | — | Resumed from preempt checkpoint after replay buffer incompatibility fix. TAKEOFF optimisations: min_armed_secs=0.3, altitude_hold_ticks≥2, world_reset_sleep=0.5s. fps improved 2→4. ep_len_mean=219, ep_rew_mean=-367. |
| 2026-03-18 | 9nbwg71r | drone-bombard-sac / L4-AutoDrop-v1 | 53,428 | — | Phase 12: Fixed OFFBOARD retry race (2s→0.5s) + absolute TAKEOFF altitude check. fps jumped 4→23. 0 CRUISE timeouts. |
| 2026-03-18 | 2h1cvmer | drone-bombard-sac / L4-AutoDrop-v1 | 78,200+ | — | Phase 13: Fixed multi-instance bug (bin/px4 pattern), stale shm, NaN obs crash. fps=7 stable at RTF=2. 0 CRUISE timeouts. |

---

## Quick-Start After Preemption

```bash
# 1. Reconnect to container
xhost +local:docker && docker start -ai drone-bombard-harmonic

# 2. Clean restart infra (kills stale processes, starts fresh PX4+Gazebo)
docker exec -d drone-bombard-harmonic bash /workspace/ros2_ws/start_infra_clean.sh
sleep 35  # wait for PX4 "Ready for takeoff!"

# 3. Start training (finds latest checkpoint automatically)
docker exec -d drone-bombard-harmonic bash -c "cd /workspace/ros2_ws && /workspace/ros2_ws/train_managed.sh > /tmp/train_managed.log 2>&1"

# 4. Monitor
docker exec drone-bombard-harmonic tail -20 /tmp/train_managed.log
docker exec drone-bombard-harmonic tail -5 /tmp/infra_main.log
```
