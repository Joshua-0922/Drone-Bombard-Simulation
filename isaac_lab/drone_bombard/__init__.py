"""Drone Bombard Isaac Lab task registration."""

import gymnasium as gym

from . import agents
from .drone_bombard_env import DroneBombardEnv, DroneBombardEnvCfg
from .task_env import DroneBombardTaskEnv, DroneBombardTaskCfg

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
    # THE task. Cruise handoff -> acquire the marker -> aim with CCIP + learned
    # impact residual -> release inside the envelope -> scored on the payload's
    # REAL landing point. Replaces the retired Isaac-DroneBombard-V11..V20 chain
    # (see task_env.py's module docstring for why that chain was retired).
    #
    # Domain randomization is configured on the BASE cfg, in two groups that are
    # deliberately independent: `model_err` (A group -- wind, payload ballistic
    # coefficient, release latency; the ONLY group `model_err.scale` scales) and
    # `dyn_dr` (C group -- controller mass belief, gains, sensor/actuator noise;
    # held FIXED across the sweep). Scenario spread is `handoff` (B group).
    id="Isaac-DroneBombard-Task-v0",
    entry_point="drone_bombard.task_env:DroneBombardTaskEnv",
    disable_env_checker=True,
    kwargs={
        "env_cfg_entry_point": DroneBombardTaskCfg,
        "rsl_rl_cfg_entry_point": f"{agents.__name__}.rsl_rl_ppo_cfg:DroneBombardPPORunnerCfg",
    },
)

# The v11-v20 task registrations were removed on 2026-08-27. `v11_env.py` itself
# is KEPT and still imports: exp_022 and exp_024 are cited in the paper as
# negative results (fixed initial conditions teach a representation, not a
# skill), so the code that produced them has to stay readable. It is simply no
# longer reachable through gym.make, and no new code imports it.

__all__ = [
    "DroneBombardEnv", "DroneBombardEnvCfg",
    "DroneBombardTaskEnv", "DroneBombardTaskCfg",
]
