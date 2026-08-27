"""Domain-randomization samplers for the phased curriculum.

Pure functions, fixed signatures, wired into ``DroneBombardEnv._reset_idx`` ->
``ballistic_impact`` (drag/wind) and the moving-target dynamics (Phase 3).
Each sampler takes an ``enabled`` flag so a single call site serves every
phase:

  * Phase 1 (``enabled=False``, the default): every sampler returns the
    identity value (all zero), so ``ballistic_impact`` reduces algebraically
    exactly to the drag-free, wind-free closed form used by the Gazebo
    ``drop_calculator_node`` referee (see ``ballistic_impact`` in
    ``math_utils.py`` — the drag branch is skipped, not multiplied by zero;
    the wind term adds a true zero vector) and the target is stationary.
  * Phase 2 (``enabled=True`` for drag/wind): per-env drag coefficient and
    horizontal wind are sampled, injecting the model mismatch the learned
    CCIP residual must correct.
  * Phase 3 (additionally ``enabled=True`` for target velocity): an initial
    moving-target velocity is sampled; ``math_utils.step_target_velocity``
    then evolves it as a Gauss-Markov process each policy step.

``cfg`` is the phase config object (``DroneBombardPhaseCfg``); only attribute
access is used (``cfg.drag_max`` etc.), so tests can pass any namespace with
the same fields.
"""

from __future__ import annotations

import math

import torch


def sample_drag_coefficient(cfg, num_envs: int, device: torch.device, enabled: bool = False) -> torch.Tensor:
    """Phase 1: identity (zero drag). Phase 2+: per-env drag coefficient from
    a physically-motivated uniform prior ``U[0, cfg.drag_max]``."""
    if not enabled:
        return torch.zeros(num_envs, device=device)
    return torch.rand(num_envs, device=device) * cfg.drag_max


def sample_wind(cfg, num_envs: int, device: torch.device, enabled: bool = False) -> torch.Tensor:
    """Phase 1: identity (zero wind). Phase 2+: per-env horizontal wind as an
    [N, 2] ENU vector drawn from ``N(0, cfg.wind_std)``."""
    if not enabled:
        return torch.zeros(num_envs, 2, device=device)
    return torch.randn(num_envs, 2, device=device) * cfg.wind_std


def sample_wind_capped(num_envs: int, device: torch.device, std: float,
                       cap: float, enabled: bool = False) -> torch.Tensor:
    """Per-episode horizontal wind, [N, 2] in ENU, ``N(0, std)`` per axis with the
    VECTOR MAGNITUDE clipped to ``cap`` (scaled down, never rotated).

    A-group sampler (see ``DroneBombardModelErrorCfg``). Takes std/cap as plain
    floats rather than reading a cfg, because the caller has already multiplied
    them by ``DR_SCALE`` -- keeping the multiplication at one call site is what
    makes the sweep knob mean exactly one thing.

    The wind is CONSTANT for the whole episode. That is deliberate, not a
    missing feature: a wind that changes during the payload's ~1 s fall would
    make the T3 oracle unable to be exact (it would need the future wind), and
    the oracle's exactness is what makes the paper's "residual recovers X% of
    the oracle gap" number interpretable. Time-varying wind belongs in the
    unseen-robustness table, not in training.

    Identity (zeros, no RNG draw) when disabled or ``std <= 0``, so the zero
    point of the DR_SCALE sweep does not perturb any other random stream."""
    if not enabled or std <= 0.0:
        return torch.zeros(num_envs, 2, device=device)
    wind = torch.randn(num_envs, 2, device=device) * std
    if cap > 0.0:
        mag = torch.linalg.norm(wind, dim=-1, keepdim=True)
        wind = wind * (cap / mag.clamp(min=cap))  # scale down only if |wind| > cap
    return wind


def sample_target_velocity(cfg, num_envs: int, device: torch.device, enabled: bool = False) -> torch.Tensor:
    """Phase 1/2: identity (stationary target). Phase 3: initial target
    velocity as an [N, 2] ENU vector with random heading and speed
    ``U[0, cfg.target_init_speed]`` (m/s). Evolved thereafter by
    ``math_utils.step_target_motion`` (per ``cfg.target_motion_model``)."""
    if not enabled:
        return torch.zeros(num_envs, 2, device=device)
    speed = torch.rand(num_envs, device=device) * cfg.target_init_speed
    heading = torch.rand(num_envs, device=device) * 2.0 * math.pi
    return torch.stack([speed * torch.cos(heading), speed * torch.sin(heading)], dim=-1)


def sample_target_accel(cfg, num_envs: int, device: torch.device, enabled: bool = False) -> torch.Tensor:
    """Per-episode target acceleration for the CA motion model: [N, 2] ENU
    vector with random direction and magnitude ``U[0, cfg.target_accel_max]``
    (m/s^2). Identity (zero) when the moving target is off or the model does
    not use acceleration — ``step_target_motion`` reads it only in "ca"."""
    if not enabled:
        return torch.zeros(num_envs, 2, device=device)
    mag = torch.rand(num_envs, device=device) * cfg.target_accel_max
    heading = torch.rand(num_envs, device=device) * 2.0 * math.pi
    return torch.stack([mag * torch.cos(heading), mag * torch.sin(heading)], dim=-1)


def sample_uniform(lo: float, hi: float, num_envs: int, device: torch.device) -> torch.Tensor:
    """Per-env ``U[lo, hi]`` as an [N] tensor.

    ``hi <= lo`` returns the exact constant ``lo`` and draws NO random numbers —
    so a "range" collapsed to a point is bit-identical to the old scalar code and
    leaves the RNG stream untouched (see the handoff sampler in
    ``DroneBombardV11Env._reset_idx``)."""
    if hi <= lo:
        return torch.full((num_envs,), lo, device=device)
    return lo + torch.rand(num_envs, device=device) * (hi - lo)


def sample_scale(num_envs: int, device: torch.device, rel: float, enabled: bool = False) -> torch.Tensor:
    """Multiplicative model-mismatch factor ``U[1-rel, 1+rel]`` as an [N] tensor.

    Identity (all ones, no RNG draw) when disabled or ``rel <= 0`` — the P0
    dynamics-DR knobs (controller mass belief, controller gains, payload
    ballistic coefficient) all go through this so an OFF config is bit-identical
    to the pre-DR env."""
    if not enabled or rel <= 0.0:
        return torch.ones(num_envs, device=device)
    return 1.0 + (torch.rand(num_envs, device=device) * 2.0 - 1.0) * rel


def sample_bias(num_envs: int, dim: int, device: torch.device, std: float, enabled: bool = False) -> torch.Tensor:
    """Per-EPISODE constant additive bias ``N(0, std)`` as an [N, dim] tensor.

    The slow half of the two-timescale sensor/actuator noise model (per-step
    white noise + per-episode constant bias). The bias term is what forces
    robustness to *calibration* offsets: white noise alone averages out over an
    episode, so a policy can ignore it. Identity (zeros, no RNG draw) when
    disabled or ``std <= 0``."""
    if not enabled or std <= 0.0:
        return torch.zeros(num_envs, dim, device=device)
    return torch.randn(num_envs, dim, device=device) * std


def sample_target_turn_rate(cfg, num_envs: int, device: torch.device, enabled: bool = False) -> torch.Tensor:
    """Per-episode signed turn rate for the CT motion model: [N] rad/s with
    magnitude ``U[cfg.target_omega_range[0], cfg.target_omega_range[1]]`` and
    a random sign (left/right turns equally likely). Identity (zero) when the
    moving target is off — ``step_target_motion`` reads it only in "ct"."""
    if not enabled:
        return torch.zeros(num_envs, device=device)
    lo, hi = cfg.target_omega_range
    mag = lo + torch.rand(num_envs, device=device) * (hi - lo)
    sign = torch.where(torch.rand(num_envs, device=device) < 0.5, -1.0, 1.0)
    return mag * sign
