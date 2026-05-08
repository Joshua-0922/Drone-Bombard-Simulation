"""Drone Bombard Isaac Lab 태스크 패키지.

환경을 Gymnasium 레지스트리에 등록한다.
Isaac Lab 설치 후 `pip install -e .` 로 패키지를 설치하면
`gymnasium.make('DroneDrop-v0')` 로 환경을 생성할 수 있다.
"""

import gymnasium as gym

from .config.drone_drop_env_cfg import DroneBombardEnvCfg, DroneBombardEnvCfg_PLAY
from .env import DroneBombardEnv

# ---------------------------------------------------------------------------
# Gymnasium 환경 등록
# ---------------------------------------------------------------------------

gym.register(
    id="Isaac-DroneDrop-v0",
    entry_point="drone_bombard.env:DroneBombardEnv",
    kwargs={"cfg": DroneBombardEnvCfg()},
    disable_env_checker=True,
)

gym.register(
    id="Isaac-DroneDrop-Play-v0",
    entry_point="drone_bombard.env:DroneBombardEnv",
    kwargs={"cfg": DroneBombardEnvCfg_PLAY()},
    disable_env_checker=True,
)

__all__ = [
    "DroneBombardEnv",
    "DroneBombardEnvCfg",
    "DroneBombardEnvCfg_PLAY",
]
