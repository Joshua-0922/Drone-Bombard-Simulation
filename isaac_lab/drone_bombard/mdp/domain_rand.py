"""Phase-2 domain-randomization stubs.

Pure functions, fixed signatures, wired into ``DroneBombardEnv._reset_idx`` ->
``ballistic_impact`` now so the call sites never need to change. Phase 1
returns identity values (all zero) so the ballistic calculation reduces
algebraically exactly to the drag-free, wind-free closed form used by the
Gazebo ``drop_calculator_node`` referee (see ``ballistic_impact`` in
``drone_bombard_env.py`` for how the zero case is handled — the drag branch
is skipped, not multiplied by zero; the wind term adds a true zero vector).

Phase 2 fills in the actual sampling distributions (Gauss-Markov wind gusts,
drag-coefficient priors from payload aerodynamics, etc.) without touching
any call site or the ``ballistic_impact`` signature.
"""

from __future__ import annotations

import torch


def sample_drag_coefficient(cfg, num_envs: int, device: torch.device) -> torch.Tensor:
    """Phase 1: identity (zero drag). Phase 2: sample a per-env drag
    coefficient from a physically-motivated prior."""
    return torch.zeros(num_envs, device=device)


def sample_wind(cfg, num_envs: int, device: torch.device) -> torch.Tensor:
    """Phase 1: identity (zero wind). Phase 2: sample per-env wind (e.g. a
    Gauss-Markov gust process) as an [N, 2] ENU horizontal wind vector."""
    return torch.zeros(num_envs, 2, device=device)
