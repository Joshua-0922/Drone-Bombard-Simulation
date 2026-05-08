"""SB3 PPO 하이퍼파라미터 설정 (옵션).

SAC 1차 검증 후 GPU 병렬 PPO 로 전환 시 사용.
skrl/rsl_rl 과의 호환을 위해 별도 dataclass 로 관리.
"""

from __future__ import annotations

from dataclasses import dataclass, field
from typing import List


@dataclass
class PpoAgentCfg:
    """Stable-Baselines3 PPO 하이퍼파라미터."""

    learning_rate: float = 3e-4
    n_steps: int = 2048          # 환경당 수집 스텝 (rollout 길이)
    batch_size: int = 64
    n_epochs: int = 10
    gamma: float = 0.99
    gae_lambda: float = 0.95
    clip_range: float = 0.2
    clip_range_vf: float | None = None
    normalize_advantage: bool = True
    ent_coef: float = 0.0
    vf_coef: float = 0.5
    max_grad_norm: float = 0.5
    net_arch: List[int] = field(default_factory=lambda: [256, 256])
    total_timesteps: int = 5_000_000
    wandb_project: str = "drone-bombard-isaac-ppo"
    checkpoint_freq: int = 50_000
    log_interval: int = 10
