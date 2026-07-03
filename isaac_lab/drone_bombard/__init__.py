"""Drone Bombard Isaac Lab task registration."""

import gymnasium as gym

from . import agents
from .drone_bombard_env import DroneBombardEnv, DroneBombardEnvCfg

gym.register(
    id="Isaac-DroneBombard-Direct-v0",
    # NOTE: matches how train.py/play.py/yolo_eval.py import this package
    # (`import drone_bombard`, i.e. isaac_lab/ itself is on sys.path/cwd,
    # not the repo root) — using "isaac_lab.drone_bombard..." here would
    # make gym.make() fail to resolve the entry point.
    entry_point="drone_bombard.drone_bombard_env:DroneBombardEnv",
    disable_env_checker=True,
    kwargs={
        "env_cfg_entry_point": DroneBombardEnvCfg,
        "rsl_rl_cfg_entry_point": f"{agents.__name__}.rsl_rl_ppo_cfg:DroneBombardPPORunnerCfg",
    },
)

__all__ = ["DroneBombardEnv", "DroneBombardEnvCfg"]
