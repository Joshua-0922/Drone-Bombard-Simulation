"""Drone Bombard Isaac Lab 태스크 패키지.

Gymnasium 등록:
  gymnasium.make("Isaac-DroneDrop-v0", cfg=DroneBombardEnvCfg())
  gymnasium.make("Isaac-DroneDrop-Play-v0", cfg=DroneBombardEnvCfg_PLAY())

학습:
  python drone_bombard/scripts/train.py --task Isaac-DroneDrop-v0 --num_envs 1024
"""

import gymnasium as gym

from .config.drone_drop_env_cfg import DroneBombardEnvCfg, DroneBombardEnvCfg_PLAY
from .env import DroneBombardEnv

# cfg는 make() 시 전달 (num_envs 오버라이드용)
gym.register(
    id="Isaac-DroneDrop-v0",
    entry_point="drone_bombard.env:DroneBombardEnv",
    disable_env_checker=True,
)

gym.register(
    id="Isaac-DroneDrop-Play-v0",
    entry_point="drone_bombard.env:DroneBombardEnv",
    disable_env_checker=True,
)

__all__ = [
    "DroneBombardEnv",
    "DroneBombardEnvCfg",
    "DroneBombardEnvCfg_PLAY",
]
