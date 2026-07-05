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


def sample_target_velocity(cfg, num_envs: int, device: torch.device, enabled: bool = False) -> torch.Tensor:
    """Phase 1/2: identity (stationary target). Phase 3: initial target
    velocity as an [N, 2] ENU vector with random heading and speed
    ``U[0, cfg.target_init_speed]`` (m/s). Evolved thereafter by
    ``math_utils.step_target_velocity`` (Gauss-Markov)."""
    if not enabled:
        return torch.zeros(num_envs, 2, device=device)
    speed = torch.rand(num_envs, device=device) * cfg.target_init_speed
    heading = torch.rand(num_envs, device=device) * 2.0 * math.pi
    return torch.stack([speed * torch.cos(heading), speed * torch.sin(heading)], dim=-1)
