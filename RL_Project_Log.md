# RL Training Pipeline — Project Log

> **Branch:** `feature/migration-harmonic` | **Purpose:** Context-recovery log for Spot VM preemption and cross-session continuity.

---

# 1. Current State

## Training Phase 13 — Running (2026-03-18)
Training actively running. WandB run: `53xx3o8u` (L4-AutoDrop-v1).

| Metric | Value |
|--------|-------|
| Steps so far | ~456,814 |
| Throughput | **28 fps** (RTF=1, stable) |
| ep_len_mean | 429 (stuck — see bug below) |
| ep_rew_mean | -36 (regressed from peak +33 at 128K steps) |
| Episodes | 2,328 (cumulative) |
| Gazebo | Alive (0 ODE crashes) |
| PX4 | Armed + Offboard confirmed |
| CRUISE timeouts | 0 |

### Bug Diagnosed: Degenerate Drop Timing (Phase 14)

**Symptom:** `ep_len_mean` locked at 429 across hundreds of episodes; `ep_rew_mean` regressed from +33 (peak at 128K steps) to -36 (at 456K steps).

**Root cause:** The policy learned a **degenerate local optimum** — it fires the manual drop trigger (`action[4] > 0.0`) at a fixed step ~429 regardless of drone position, earning a small drop reward (`10 * exp(-0.3 * d_large)`) to escape the accumulating Layer 2 time penalty. This is structurally enabled by:
1. `action[4] ∈ [-1, 1]` → drop fires whenever `action[4] > 0`, covering 50% of the action space
2. `ent_coef` decayed 0.13 → 0.056, collapsing exploration and locking the policy into this local optimum
3. `w_time=0.01/step` penalty makes hovering indefinitely costly, incentivising early drop-and-end

**Evidence:** Episodes that end via `terminated=True` (not truncated — `max_steps=500`, but episodes end at 429, 71 steps early). `ep_len` frozen exactly at 429 for 280+ consecutive log entries.

**Reward trajectory:**
- 80K→111K steps: reward -94 → -20 (policy learning to approach)
- 111K→128K steps: reward -20 → **+33** (peak — policy discovered approach + drop)
- 128K→415K steps: +33 → -8 (plateau then decline as entropy collapsed)
- 415K→456K steps: -8 → -36 (degenerate drop timing locked in)

**Proposed fixes (not yet applied):**
- [ ] **Option A — Remove manual drop** from action space; make drop auto-only (`d_impact <= 0.5m`). Cleanest fix — forces policy to navigate to target.
- [ ] **Option B — Penalise imprecise drops** — add `reward -= w_bad_drop * d_impact` when drop fires with `d_impact > threshold (e.g. 5m)`, making early drops costly.
- [ ] **Option C — Raise entropy floor** — set `ent_coef_min` in SAC config to prevent entropy collapse.

**Root causes diagnosed and fixed in Phase 13 (this session):**
1. **Multiple instance proliferation** — `start_infra_clean.sh` kill pattern `/px4 ` missed PX4 (no trailing space). Fix: `bin/px4`.
2. **Stale FastRTPS shm** — 169 stale segments caused IMU timestamp chaos. Fix: `rm -f /dev/shm/fastrtps_*`.
3. **NaN observation SB3 crash** — EKF velocity→NaN during DDS time sync → ValueError exit 134. Fix: `nan_to_num` guards in ROS2 callbacks (`_on_local_pos`, `_on_ang_vel`) and `_get_obs`.
4. **`UXRCE_DDS_SYNCT=0` at RTF=2 broke OFFBOARD** — tried and reverted; breaks at RTF>1.
5. **Gazebo ODE AABB overflow crash** — Root cause: drone arms before EKF2 converges → NaN motor commands → NaN forces → ODE integer overflow. Fix: 5s EKF warmup delay in `drone_controller` before first arm attempt (`warmup_ticks=100` at 20Hz).
6. **`PX4_GZ_MODEL_POSE` missing** — Drone spawned at z=0.5 while payload at z=5.14 → DetachableJoint tries to bridge 4.5m gap → physics explosion. Fix: Added `PX4_GZ_MODEL_POSE='0,0,5,0,0,0'` to infra.launch.py.
7. **RTF=2 time jumps → EKF NaN** — At RTF=2, uXRCE-DDS sync diverges periodically. Fix: RTF=1 (`world.sdf` + `PX4_SIM_SPEED_FACTOR=1`) + `UXRCE_DDS_SYNCT=0` (safe at RTF=1 since clocks are aligned).

**Current airframe params** (`4015_gz_x500_bombard`):
- `FD_FAIL_R=0`, `SIM_BAT_DRAIN=0`, `COM_ARM_WO_GPS=1`
- `EKF2_MAG_CHECK=0`, `COM_ARM_MAG_STR=0`, `COM_ARM_MAG_ANG=180`
- `COM_OF_LOSS_T=5.0`, `UXRCE_DDS_SYNCT=0`

**Current infra config:**
- RTF=1 (`x_marker_world.sdf`: `real_time_factor=1, real_time_update_rate=100`)
- `PX4_SIM_SPEED_FACTOR=1`, `PX4_GZ_MODEL_POSE=0,0,5,0,0,0` (infra.launch.py)
- `obs_wait_timeout: 0.02` (20ms), `num_envs: 1`, `use_vision: false`
- `target_altitude: 5.0`, Checkpoint: `/workspace/ros2_ws/rl_checkpoints/sac_drop_preempt.zip`
- WandB run: `53xx3o8u`

---

# 2. Recent Progress

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

- [x] **Multiple instances fixed** — `start_infra_clean.sh` with correct `bin/px4` pattern
- [x] **NaN crash fixed** — `nan_to_num` guard in `_get_obs()`
- [x] **Airframe persisted to repo** — `drone_drop_system/docker/config/airframes/4015_gz_x500_bombard`
- [x] **fps stability confirmed** — fps=28 sustained, 0 ODE crashes, 0 CRUISE timeouts
- [x] **Reward trend evaluated** — peaked at +33 (128K steps), regressed to -36 (456K steps); root cause diagnosed (see Phase 14 bug above)
- [ ] **Fix degenerate drop timing** — choose and apply one of: (A) remove manual drop from action space, (B) penalise imprecise drops, (C) raise entropy floor
- [ ] **Try RTF=3 or higher** — only after drop bug is fixed and policy is healthy; update `x_marker_world.sdf` + `PX4_SIM_SPEED_FACTOR`
- [ ] **Tune reward weights** — start with `w_dist`, `w_drop_base`, `r_success_jackpot` after drop fix
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
| 2026-03-18 | 2h1cvmer | drone-bombard-sac / L4-AutoDrop-v1 | 78,200 | — | Phase 13 early: Fixed multi-instance (bin/px4 pattern), stale shm, NaN obs crash. fps=7 briefly. |
| 2026-03-18 | 53xx3o8u | drone-bombard-sac / L4-AutoDrop-v1 | 80,292+ | — | Phase 13 final: Fixed PX4_GZ_MODEL_POSE (drone-payload gap), 5s EKF warmup in drone_controller, RTF=1, UXRCE_DDS_SYNCT=0. **fps=12 stable, 0 ODE crashes.** |

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
