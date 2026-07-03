"""Pure torch math for the drone-bombard task — zero isaaclab dependency.

Deliberately kept import-clean of isaaclab so ``isaac_lab/tests/test_math.py``
can validate the action pipeline, vision projection, ballistic/CCIP hooks,
and reward formula on any machine with just ``torch`` installed (this repo's
dev box has no Isaac Sim install — see the migration plan §7a). Do not add
an isaaclab import to this file; keep isaaclab-dependent code in
``drone_bombard_env.py``.

All formulas here are ports of ``ros2_ws/src/rl_navigation/rl_navigation/
drone_drop_env.py`` (``_compute_reward``, ``_get_obs``, ``step``) and
``ros2_ws/src/drone_controller/drone_controller/drone_controller_node.py``
(``_filter_velocity``) — the v13/v15 Gazebo+PX4 baseline.
"""

from __future__ import annotations

import torch


def rate_limit_action(action: torch.Tensor, prev_action: torch.Tensor, limit: float) -> torch.Tensor:
    """Clip ``action`` to ``prev_action`` +/- ``limit``, per-element. Matches
    the Gazebo env's P2 (junsang_v4) acceleration hard-clip, applied BEFORE
    scaling to physical units."""
    return torch.clamp(action, prev_action - limit, prev_action + limit)


def lpf_step(cmd: torch.Tensor, v_filt: torch.Tensor, alpha: float, snap: torch.Tensor) -> torch.Tensor:
    """One EMA low-pass tick: ``v_filt = alpha*cmd + (1-alpha)*v_filt``.

    ``snap`` is a per-env bool mask — True rows snap to ``cmd`` directly
    (replicates ``_filter_velocity``'s mode-switch-to-VELOCITY snap so there
    is no ramp-from-zero lag when RL takes over). Ticking this at 20 Hz (every
    5 physics steps at 100 Hz / decimation 10) reproduces
    ``drone_controller_node._filter_velocity`` exactly.
    """
    filtered = alpha * cmd + (1.0 - alpha) * v_filt
    return torch.where(snap.unsqueeze(-1), cmd, filtered)


def quat_apply_inverse_pure(q: torch.Tensor, v: torch.Tensor) -> torch.Tensor:
    """Rotate world-frame vector ``v`` into the body frame of quaternion
    ``q`` (w,x,y,z convention, matching Isaac Lab/PhysX). Pure torch — no
    isaaclab dependency. Equivalent to isaaclab.utils.math.quat_apply_inverse
    (rotates by the conjugate of q, i.e. world -> body)."""
    w, x, y, z = q[:, 0], q[:, 1], q[:, 2], q[:, 3]
    qw, qvec = w, -torch.stack([x, y, z], dim=-1)  # conjugate: negate vector part
    t = 2.0 * torch.linalg.cross(qvec, v, dim=-1)
    return v + qw.unsqueeze(-1) * t + torch.linalg.cross(qvec, t, dim=-1)


def project_target_pinhole(
    pos_w: torch.Tensor,
    quat_w: torch.Tensor,
    target_xy: torch.Tensor,
    fx: float,
    fy: float,
    cx: float,
    cy: float,
    img_w: int,
    img_h: int,
    near_clip: float,
    far_clip: float,
) -> tuple[torch.Tensor, torch.Tensor, torch.Tensor]:
    """Project the (ground-level) target point into a straight-down-mounted
    body camera. Camera mount matches ``x500_bombard`` SDF (down_camera_sensor,
    pitch=+90 deg so optical +Z looks straight down = body -Z).

    Camera-frame convention (chosen to match Gazebo's optical-frame pixel
    layout: image +u follows body forward/+X, image +v follows body
    right/+Y — verified against a level-hover projection in test_math.py):
        X_c (right in image)  = body +Y component of (target - drone)
        Y_c (down in image)   = body +X component of (target - drone)
        Z_c (depth, into cam) = body -Z component of (target - drone)

    Returns: (u_px [N], v_px [N], visible [N] bool) — no noise applied here;
    callers add sensor noise/dropout/hold on top (see DroneBombardEnv._update_vision).
    """
    rel_w = torch.zeros_like(pos_w)
    rel_w[:, 0] = target_xy[:, 0] - pos_w[:, 0]
    rel_w[:, 1] = target_xy[:, 1] - pos_w[:, 1]
    rel_w[:, 2] = 0.0 - pos_w[:, 2]

    rel_b = quat_apply_inverse_pure(quat_w, rel_w)  # world -> body frame

    x_c = rel_b[:, 1]
    y_c = rel_b[:, 0]
    z_c = -rel_b[:, 2]

    z_safe = torch.where(z_c.abs() < 1e-6, torch.full_like(z_c, 1e-6), z_c)
    u_px = fx * (x_c / z_safe) + cx
    v_px = fy * (y_c / z_safe) + cy

    visible = (
        (z_c > near_clip)
        & (z_c < far_clip)
        & (u_px >= 0.0) & (u_px < img_w)
        & (v_px >= 0.0) & (v_px < img_h)
    )
    return u_px, v_px, visible


def ballistic_impact(
    pos_xy: torch.Tensor,
    vel_xy: torch.Tensor,
    altitude: torch.Tensor,
    release_delay: float,
    gravity: float,
    drag_coef: torch.Tensor,
    wind_xy: torch.Tensor,
) -> torch.Tensor:
    """Predicted impact point under free-fall ballistics — exact parity with
    ``drop_calculator_node``'s ``t = (vz + sqrt(vz^2 + 2*g*H)) / g`` formula
    specialised to release-from-rest-vertically (vz~=0 at release, matching
    the Gazebo referee which triggers at/near success): ``t = sqrt(2H/g)``.

    ``drag_coef`` / ``wind_xy`` are Phase-2 domain-randomization hooks (see
    ``mdp/domain_rand.py``). In Phase 1 both are exactly zero:
      * ``wind_xy == 0`` -> the wind term below adds the zero vector (a
        genuine no-op, not merely a multiply-by-zero).
      * the drag-affected branch is skipped entirely when ``drag_coef == 0``
        (guarded, not computed-then-discarded), so Phase 1 reduces
        ALGEBRAICALLY EXACTLY to the drag-free closed form used by the
        Gazebo referee.
    """
    t_fall = torch.sqrt(torch.clamp(2.0 * altitude / gravity, min=0.0))
    impact = pos_xy + (vel_xy + wind_xy) * (t_fall + release_delay)

    has_drag = drag_coef != 0.0
    if torch.any(has_drag):
        # Phase-2 placeholder: first-order drag correction, applied only where
        # drag_coef != 0. Not exercised in Phase 1 (sample_drag_coefficient
        # returns all-zero in Phase 1 — see mdp/domain_rand.py).
        drag_correction = -drag_coef.unsqueeze(-1) * vel_xy * t_fall.unsqueeze(-1) ** 2
        impact = torch.where(has_drag.unsqueeze(-1), impact + drag_correction, impact)
    return impact


def ccip_residual(obs: torch.Tensor) -> torch.Tensor:
    """Phase-2 learned-residual hook for the CCIP impact prediction.

    Pure function, no parameters, no network — returns zeros of the correct
    shape/dtype/device. Phase 1 never calls this (``DropCfg.residual_enabled
    = False`` skips the call site entirely — see
    ``DroneBombardEnv._predicted_impact``), so this stub exists purely so
    Phase 2 can wire in a learned residual without touching the CCIP or
    reward code paths.
    """
    return torch.zeros((obs.shape[0], 2), dtype=obs.dtype, device=obs.device)


def compute_reward(
    d_xy: torch.Tensor,
    d_xy_prev: torch.Tensor,
    pos_xy: torch.Tensor,
    target_xy: torch.Tensor,
    vel_xy: torch.Tensor,
    ang_vel: torch.Tensor,
    delta_action: torch.Tensor,
    u_norm: torch.Tensor,
    v_norm: torch.Tensor,
    conf: torch.Tensor,
    cfg,
) -> tuple[torch.Tensor, dict]:
    """Exact v15 3-layer per-step reward (Layer 1 terminal penalties are
    applied separately in ``DroneBombardEnv._get_rewards`` from cached done
    flags — this computes the always-on Layer 2 + Layer 3 terms plus the
    diagnostic breakdown dict, mirroring ``_compute_reward`` in the Gazebo
    env line-for-line.

    ``cfg`` needs only attribute access (``cfg.w_dist`` etc.) — pass either
    the real ``DroneBombardRewardCfg`` or, in tests, any plain object/
    namespace with the same field names (see test_math.py's ``_RewardCfg``).
    """
    omega_sq = (ang_vel * ang_vel).sum(dim=-1)
    action_smooth_sq = (delta_action * delta_action).sum(dim=-1)

    r2 = (
        -cfg.w_time
        - cfg.w_ang_vel * omega_sq
        - cfg.w_action_smooth * action_smooth_sq
    )

    r3_dist = cfg.w_dist * (d_xy_prev - d_xy)

    speed_xy = torch.linalg.norm(vel_xy, dim=-1)
    near_factor = torch.clamp(1.0 - d_xy / cfg.vel_damp_radius, min=0.0)
    r3_vel = -cfg.w_vel * speed_xy * near_factor

    # w_heading == 0.0 in v13/v15 (disabled — geometric noise at d_xy<3m).
    # Computed anyway for parity/telemetry; contributes exactly zero reward.
    to_target = target_xy - pos_xy
    dist_to_target = torch.linalg.norm(to_target, dim=-1)
    dot = (vel_xy * to_target).sum(dim=-1)
    denom = torch.clamp(speed_xy * dist_to_target, min=0.01)
    cos_heading = torch.where(speed_xy > 0.1, dot / denom, torch.zeros_like(dot))
    speed_gate = torch.clamp(speed_xy / 2.0, max=1.0) if cfg.speed_gate_enabled else torch.ones_like(speed_xy)
    r3_orient = cfg.w_heading * cos_heading * speed_gate

    center_dist = torch.sqrt(u_norm * u_norm + v_norm * v_norm)
    proximity_factor = torch.clamp(1.0 - d_xy / 30.0, min=0.0)
    r3_vision = torch.where(
        conf > 0.0,
        cfg.w_vision_center * torch.clamp(1.0 - center_dist, min=0.0) * conf * proximity_factor,
        torch.zeros_like(d_xy),
    )

    r3_proximity = cfg.w_proximity * torch.clamp(1.0 - d_xy / cfg.proximity_radius, min=0.0)

    r3 = r3_dist + r3_orient + r3_proximity + r3_vision + r3_vel
    reward = r2 + r3

    breakdown = {
        "rew_ctrl": r2, "rew_dist": r3_dist, "rew_orient": r3_orient,
        "rew_proximity": r3_proximity, "rew_vision": r3_vision, "rew_vel": r3_vel,
    }
    return reward, breakdown


def hold_buffer_update(
    detected: torch.Tensor,
    fresh_uv_conf: torch.Tensor,
    held_uv_conf: torch.Tensor,
    hold_remaining: torch.Tensor,
    hold_frames: int,
) -> tuple[torch.Tensor, torch.Tensor, torch.Tensor, torch.Tensor, torch.Tensor]:
    """One vision-hold tick, matching ``xmarker_detector``'s temporal
    smoothing: on a miss, keep republishing the last detection for
    ``hold_frames`` ticks (10 @ 10 Hz policy rate = 1 s), then zero.

    Returns: (u, v, conf, held_uv_conf', hold_remaining') — the first three
    are what the observation should show this tick; the last two are the
    updated persistent state for next tick.
    """
    held_uv_conf = torch.where(detected.unsqueeze(-1), fresh_uv_conf, held_uv_conf)
    hold_remaining = torch.where(detected, torch.full_like(hold_remaining, hold_frames), hold_remaining)

    use_held = (~detected) & (hold_remaining > 0)
    hold_remaining = torch.where(use_held, hold_remaining - 1, hold_remaining)

    show = detected | use_held
    out = torch.where(show.unsqueeze(-1), held_uv_conf, torch.zeros_like(held_uv_conf))
    return out[:, 0], out[:, 1], out[:, 2], held_uv_conf, hold_remaining


def overshoot_guard(d_xy: torch.Tensor, d_xy_min: torch.Tensor, arm_radius: float, margin: float) -> torch.Tensor:
    """Shared logic for both the terminating overshoot guard
    (arm_radius=overshoot_close_threshold=0.6) and the non-terminating
    diagnostic (arm_radius=overshoot_flythrough_radius=1.2). Fires when the
    closest approach so far was within ``arm_radius`` and the drone has since
    receded more than ``margin`` past that closest point."""
    return (d_xy_min < arm_radius) & (d_xy > d_xy_min + margin)


def stagnation_guard(
    d_xy_now: torch.Tensor,
    d_xy_past: torch.Tensor,
    step_in_ep: torch.Tensor,
    window: int,
    min_progress: float,
) -> torch.Tensor:
    """Fires once ``step_in_ep >= window`` and progress over the trailing
    ``window`` steps is below ``min_progress`` metres."""
    return (step_in_ep >= window) & ((d_xy_past - d_xy_now) < min_progress)
