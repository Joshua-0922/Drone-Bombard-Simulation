# RL Training Pipeline — Project Log

> **Branch:** `feature/migration-harmonic` | **Purpose:** Context-recovery log for Spot VM preemption and cross-session continuity.

---

# 1. Current State

## Phase 1 Curriculum — Training (2026-03-18)
Training actively running. WandB run: `dy97unuj` (L4-AutoDrop-v1).

| Metric | Value |
|--------|-------|
| Steps so far | ~97,849 (resumed from preempt checkpoint) |
| Throughput | ~28 fps (RTF=1) |
| Curriculum | **Phase 1: manual drop disabled** |
| Action space | Box(5,) — action[4] ignored (dummy dim for ckpt compat) |
| Drop mode | Auto-drop only (`d_impact <= 0.5m`) |
| Checkpoint | `/workspace/ros2_ws/rl_checkpoints/sac_drop_preempt.zip` |
| WandB run | `dy97unuj` |

### Phase 1 Curriculum Fix (applied this session)

**Problem:** Policy learned degenerate local optimum — fires `action[4] > 0` at step ~429 to escape time penalty. Reward regressed +33 → -36 over 128K→456K steps.

**Fix:** Disabled manual drop in `drone_drop_env.py` (line 411-412). `action[4]` kept as dummy dimension for checkpoint compatibility. Episodes now terminate only via auto-drop (`d_impact <= 0.5m`) or truncation at `max_steps=500`.

**Expected behavior:** `ep_len_mean` should initially be ~500 (truncation) as policy can no longer shortcut. Over time, policy must learn to navigate to target to earn drop reward.

**Current infra config:**
- RTF=1 (`x_marker_world.sdf`: `real_time_factor=1, real_time_update_rate=100`)
- `PX4_SIM_SPEED_FACTOR=1`, `PX4_GZ_MODEL_POSE=0,0,5,0,0,0` (infra.launch.py)
- `obs_wait_timeout: 0.02` (20ms), `num_envs: 1`, `use_vision: false`
- `target_altitude: 5.0`

---

# 2. Recent Progress

- **Phase 1 Curriculum — Disable Manual Drop (2026-03-18):**
  - Disabled `manual_drop = float(action[4]) > 0.0` in `drone_drop_env.py`
  - Removed `or manual_drop` from drop condition — auto-drop only (`d_impact <= 0.5m`)
  - Action space stays Box(5,) for checkpoint compatibility (action[4] is dummy)
  - Training resumed from `sac_drop_preempt.zip` (~97K steps), WandB run `dy97unuj`

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
- [x] **Fix degenerate drop timing** — Phase 1 curriculum: disabled manual drop (Option A); action[4] kept as dummy for ckpt compat
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
| 2026-03-18 | dy97unuj | drone-bombard-sac / L4-AutoDrop-v1 | 97,849+ | — | **Phase 1 curriculum: manual drop disabled.** action[4] kept as dummy dim. Auto-drop only (d_impact≤0.5m). Resumed from preempt checkpoint. |

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
