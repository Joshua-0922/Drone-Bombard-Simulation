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

def register_retired_lineage() -> None:
    """Re-register the v11..v20 tasks, for REPRODUCING archived experiments only.

    `v11_env.py` was kept when its task registrations were removed on
    2026-08-27, because exp_022 and exp_024 are cited in the paper as negative
    results (fixed initial conditions teach a representation, not a skill) and
    the code behind a cited result has to remain runnable. Keeping the file
    without a way to instantiate it would have made that claim hollow.

    It is opt-in rather than automatic so that no new work can reach the old
    lineage by accident, and so importing this package does not pay for ten
    registrations nobody uses. Call it explicitly:

        import drone_bombard
        drone_bombard.register_retired_lineage()
        env = gym.make("Isaac-DroneBombard-V20-Direct-v0", ...)

    ⚠️ These environments predate the 2026-08-27 physics fixes (payload drag
    frame, CCIP self-velocity drag term, real release latency). Numbers produced
    now will NOT match the archived ones, and should not be compared against
    results from the current task.
    """
    from . import v11_env as _v

    lineage = [
        ("V11", _v.DroneBombardV11Env, _v.DroneBombardV11Cfg),
        ("V12", _v.DroneBombardV11Env, _v.DroneBombardV12Cfg),
        ("V13", _v.DroneBombardV13Env, _v.DroneBombardV13Cfg),
        ("V14", _v.DroneBombardV14Env, _v.DroneBombardV14Cfg),
        ("V15", _v.DroneBombardV14Env, _v.DroneBombardV15Cfg),
        ("V16", _v.DroneBombardV16Env, _v.DroneBombardV16Cfg),
        ("V17", _v.DroneBombardV17Env, _v.DroneBombardV17Cfg),
        ("V18", _v.DroneBombardV18Env, _v.DroneBombardV18Cfg),
        ("V19", _v.DroneBombardV19Env, _v.DroneBombardV19Cfg),
        ("V20", _v.DroneBombardV19Env, _v.DroneBombardV20Cfg),
    ]
    for tag, env_cls, cfg_cls in lineage:
        task_id = f"Isaac-DroneBombard-{tag}-Direct-v0"
        if task_id in gym.registry:
            continue
        gym.register(
            id=task_id,
            entry_point=f"drone_bombard.v11_env:{env_cls.__name__}",
            disable_env_checker=True,
            kwargs={
                "env_cfg_entry_point": cfg_cls,
                "rsl_rl_cfg_entry_point": f"{agents.__name__}.rsl_rl_ppo_cfg:DroneBombardPPORunnerCfg",
            },
        )

__all__ = [
    "DroneBombardEnv", "DroneBombardEnvCfg",
    "DroneBombardTaskEnv", "DroneBombardTaskCfg",
    "register_retired_lineage",
]
