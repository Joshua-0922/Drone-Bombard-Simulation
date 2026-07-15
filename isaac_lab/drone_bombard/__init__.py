"""Drone Bombard Isaac Lab task registration."""

import gymnasium as gym

from . import agents
from .drone_bombard_env import DroneBombardEnv, DroneBombardEnvCfg
from .v11_env import (
    DroneBombardV11Env, DroneBombardV11Cfg, DroneBombardV12Cfg,
    DroneBombardV13Env, DroneBombardV13Cfg,
)

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

gym.register(
    # Isaac-v11 relaxed test: single integrated phase, fixed marker ahead in the
    # cruise direction, policy drop_signal + release envelope, nominal physics.
    # See drone_bombard/v11_env.py and final_integrated_document_set/53.
    id="Isaac-DroneBombard-V11-Direct-v0",
    entry_point="drone_bombard.v11_env:DroneBombardV11Env",
    disable_env_checker=True,
    kwargs={
        "env_cfg_entry_point": DroneBombardV11Cfg,
        "rsl_rl_cfg_entry_point": f"{agents.__name__}.rsl_rl_ppo_cfg:DroneBombardPPORunnerCfg",
    },
)

gym.register(
    # v12: v11 + random marker inside a 5 m disk around the 20 m point.
    # Same env class (DroneBombardV11Env); only the cfg toggles marker_random.
    id="Isaac-DroneBombard-V12-Direct-v0",
    entry_point="drone_bombard.v11_env:DroneBombardV11Env",
    disable_env_checker=True,
    kwargs={
        "env_cfg_entry_point": DroneBombardV12Cfg,
        "rsl_rl_cfg_entry_point": f"{agents.__name__}.rsl_rl_ppo_cfg:DroneBombardPPORunnerCfg",
    },
)

gym.register(
    # v13: v12 + partial observability. Blind +X cruise until within reveal_radius
    # (horizontal) of the random marker; marker obs masked + penalized until then.
    # Uses DroneBombardV13Env (25-D obs, reward gated on detection).
    id="Isaac-DroneBombard-V13-Direct-v0",
    entry_point="drone_bombard.v11_env:DroneBombardV13Env",
    disable_env_checker=True,
    kwargs={
        "env_cfg_entry_point": DroneBombardV13Cfg,
        "rsl_rl_cfg_entry_point": f"{agents.__name__}.rsl_rl_ppo_cfg:DroneBombardPPORunnerCfg",
    },
)

__all__ = [
    "DroneBombardEnv", "DroneBombardEnvCfg",
    "DroneBombardV11Env", "DroneBombardV11Cfg", "DroneBombardV12Cfg",
    "DroneBombardV13Env", "DroneBombardV13Cfg",
]
