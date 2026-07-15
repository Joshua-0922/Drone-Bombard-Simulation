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

import math

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
    # t_fall is [N]; unsqueeze to [N,1] so it broadcasts against the [N,2]
    # horizontal vectors (a bare [N] would try to broadcast N against the
    # size-2 last dim and fail for any N != 2 — only N==1 slips through).
    impact = pos_xy + (vel_xy + wind_xy) * (t_fall + release_delay).unsqueeze(-1)

    has_drag = drag_coef != 0.0
    if torch.any(has_drag):
        # Phase-2 placeholder: first-order drag correction, applied only where
        # drag_coef != 0. Not exercised in Phase 1 (sample_drag_coefficient
        # returns all-zero in Phase 1 — see mdp/domain_rand.py).
        drag_correction = -drag_coef.unsqueeze(-1) * vel_xy * t_fall.unsqueeze(-1) ** 2
        impact = torch.where(has_drag.unsqueeze(-1), impact + drag_correction, impact)
    return impact


def ccip_residual(obs: torch.Tensor) -> torch.Tensor:
    """Legacy Phase-2 zero-stub for the CCIP impact prediction residual.

    Pure function, no parameters, no network — returns zeros of the correct
    shape/dtype/device. Superseded in the phased-curriculum implementation by
    ``apply_ccip_residual`` (which applies a policy-produced delta from the
    extended 6-dim action space); kept here so the Phase-1 bit-parity contract
    and its unit test remain valid.
    """
    return torch.zeros((obs.shape[0], 2), dtype=obs.dtype, device=obs.device)


def time_to_fall(altitude: torch.Tensor, gravity: float) -> torch.Tensor:
    """Free-fall time from ``altitude`` to the ground: ``sqrt(2H/g)``.

    Specialised to release-from-rest-vertically (vz~=0 at release), matching
    the Gazebo ``drop_calculator_node`` referee. Clamped at 0 so a
    below-ground altitude never yields a NaN.
    """
    return torch.sqrt(torch.clamp(2.0 * altitude / gravity, min=0.0))


def predict_impact_nominal(
    pos_xy: torch.Tensor,
    vel_xy: torch.Tensor,
    altitude: torch.Tensor,
    release_delay: float,
    gravity: float,
) -> torch.Tensor:
    """CCIP impact prediction under the NOMINAL model (no drag, no wind).

    This is what the on-board CCIP predictor "believes" the impact point is.
    In Phase 2+ the real drop uses domain-randomized drag/wind
    (``ballistic_impact`` with nonzero ``drag_coef``/``wind_xy``), so the gap
    between this nominal prediction and the real impact is exactly the
    model-mismatch the learned residual must close. Reduces algebraically to
    ``ballistic_impact`` with zero drag/wind.
    """
    zero_drag = torch.zeros(pos_xy.shape[0], device=pos_xy.device, dtype=pos_xy.dtype)
    zero_wind = torch.zeros(pos_xy.shape[0], 2, device=pos_xy.device, dtype=pos_xy.dtype)
    return ballistic_impact(pos_xy, vel_xy, altitude, release_delay, gravity, zero_drag, zero_wind)


def apply_ccip_residual(pred_impact: torch.Tensor, residual_action: torch.Tensor, scale: float) -> torch.Tensor:
    """Apply the policy-produced CCIP residual (Phase 2+).

    ``residual_action`` is the [N,2] slice of the extended action space
    (``action[:, 4:6]``, already clamped to [-1,1] by the env), scaled by
    ``scale`` (metres) and ADDED to the nominal CCIP impact prediction. The
    policy learns to bias the predicted impact so that, despite the nominal
    predictor ignoring drag/wind (and, in Phase 3, target motion), the real
    drop lands on target — i.e. it learns the model-uncertainty correction.
    """
    return pred_impact + residual_action * scale


def step_target_velocity(
    vel_xy: torch.Tensor,
    theta: float,
    sigma: float,
    dt: float,
    noise: torch.Tensor,
) -> torch.Tensor:
    """One Ornstein-Uhlenbeck / Gauss-Markov step for the moving-target
    velocity (Phase 3): ``v <- (1 - theta*dt) * v + sigma * sqrt(dt) * noise``.

    ``theta`` is the mean-reversion rate (pulls the velocity back toward zero
    so the target wanders rather than drifting away unbounded), ``sigma`` the
    volatility, ``noise`` a standard-normal [N,2] draw supplied by the caller
    (kept as an argument so this stays a pure, deterministically-testable
    function). Phase 1/2 never call this (target is stationary).
    """
    return (1.0 - theta * dt) * vel_xy + sigma * math.sqrt(dt) * noise


def impact_terminal_reward(real_err: torch.Tensor, w_impact: float, reward_scale: float) -> torch.Tensor:
    """Terminal reward for the real (domain-randomized physics) impact error
    at payload release (Phase 2+): ``w_impact * exp(-real_err / reward_scale)``.

    Monotonically decreasing in the true miss distance ``real_err`` (metres),
    saturating to ``w_impact`` at a perfect hit and decaying smoothly — a
    dense terminal shaping that rewards closing the sim-model gap rather than
    a hard pass/fail (the success bonus is applied separately in the env).
    """
    return w_impact * torch.exp(-real_err / reward_scale)


def lead_prediction_reward(
    pred_target_xy: torch.Tensor,
    actual_target_xy: torch.Tensor,
    w_lead: float,
    reward_scale: float,
) -> torch.Tensor:
    """Reward for correctly predicting the moving target's future (lead)
    position at impact time (Phase 3): ``w_lead * exp(-lead_err / reward_scale)``
    where ``lead_err = |pred_target_xy - actual_target_xy|``.

    ``pred_target_xy`` is where the policy/CCIP aimed (the lead point it
    released against); ``actual_target_xy`` is where the target actually was at
    impact. Encourages learning the intercept geometry / lead computation.
    """
    lead_err = torch.linalg.norm(pred_target_xy - actual_target_xy, dim=-1)
    return w_lead * torch.exp(-lead_err / reward_scale)


def aim_error_reward(aim_err: torch.Tensor, w_aim: float, reward_scale: float) -> torch.Tensor:
    """Dense per-step shaping on the CCIP predicted-impact error (exp_017
    Stage A): ``w_aim * (1 - tanh(aim_err / reward_scale))``.

    ``aim_err`` is the NOMINAL-only |predict_impact_nominal - aim_point| the
    env stashes each policy step (``_aim_err_last``). DELIBERATE: callers
    must never feed a residual-corrected error here — the residual is an
    unconstrained policy output, and rewarding a quantity the policy can
    shift without flying differently is a reward-hacking channel (exp_018;
    the Phase-2+ release TRIGGER stays residual-inclusive, only the reward
    quantity is nominal-only). The env-computed lead aim_point is fine. So
    the policy is rewarded every step for holding the predicted impact point
    on the target — the release capability that raw d_xy proximity never
    asked for (see research/ccip_release_decoupling: a proximity-optimal
    path's closest CCIP approach equals its cross-track error, hence
    release_rate@0.2m of only ~6%). Smooth and bounded like the other
    Layer-3 terms: ~w_aim inside ~reward_scale/3 metres, ~0 beyond
    ~3*reward_scale. ``w_aim == 0`` returns exact zeros (term off), and
    ``aim_err == inf`` (pre-first-evaluation sentinel) maps to exactly 0.
    """
    return w_aim * (1.0 - torch.tanh(aim_err / reward_scale))


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


def release_gate(
    d_impact: torch.Tensor,
    altitude: torch.Tensor,
    speed_xy: torch.Tensor,
    vz: torch.Tensor,
    roll: torch.Tensor,
    pitch: torch.Tensor,
    ang_vel_norm: torch.Tensor,
    payload_attached: torch.Tensor,
    release_radius: float,
    alt_min: float,
    alt_max: float,
    max_speed: float,
    max_vz: float,
    max_tilt: float,
    max_ang_vel: float,
) -> torch.Tensor:
    """Release-envelope gate for the v11 policy-``drop_signal`` model
    (Isaac-v11 spec, doc 53 §6). Returns a per-env bool mask that is True only
    when the drone is in a state where a released payload can plausibly land on
    target: the CCIP predicted-impact error is within ``release_radius``, the
    altitude is inside [alt_min, alt_max], and speed/vertical-speed/tilt/
    angular-rate are all under their caps, with the payload still attached.

    Pure torch (no isaaclab dependency) so it is unit-tested in
    ``tests/test_math.py``. The env combines this with the policy's
    ``action[4] > 0.5`` drop-intent: fire only when BOTH hold. A drop-intent
    with the gate closed is a NO-OP with NO penalty (Gazebo v7 lesson — a
    penalty on gate-closed intent teaches drop avoidance; doc 53 §6.2).
    ``payload_attached`` is passed as a float/bool tensor and treated
    truthily so drop cannot re-fire after release.
    """
    return (
        (d_impact <= release_radius)
        & (altitude >= alt_min)
        & (altitude <= alt_max)
        & (speed_xy <= max_speed)
        & (vz.abs() <= max_vz)
        & (roll.abs() <= max_tilt)
        & (pitch.abs() <= max_tilt)
        & (ang_vel_norm <= max_ang_vel)
        & (payload_attached > 0)
    )
