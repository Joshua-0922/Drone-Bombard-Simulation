"""Drone Bombard Isaac Lab task registration."""

import gymnasium as gym

from . import agents
from .drone_bombard_env import DroneBombardEnv, DroneBombardEnvCfg

gym.register(
    id="Isaac-DroneBombard-Direct-v0",
    entry_point="isaac_lab.drone_bombard.drone_bombard_env:DroneBombardEnv",
    disable_env_checker=True,
    kwargs={
        "env_cfg_entry_point": DroneBombardEnvCfg,
        "rsl_rl_cfg_entry_point": f"{agents.__name__}.rsl_rl_ppo_cfg:DroneBombardPPORunnerCfg",
    },
)

__all__ = ["DroneBombardEnv", "DroneBombardEnvCfg"]
