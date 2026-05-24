#!/usr/bin/env python3
"""Isaac Lab smoke test — num_envs=1, 호버·CCIP 투하·착지 확인.

Docker:
  /isaac-sim/python.sh isaac_lab_tasks/drone_bombard/scripts/smoke_test.py --headless
"""

from __future__ import annotations

import argparse

from isaaclab.app import AppLauncher

parser = argparse.ArgumentParser(description="Drone Bombard smoke test")
parser.add_argument("--num_envs", type=int, default=1)
parser.add_argument("--max_steps", type=int, default=500)
parser.add_argument("--headless", action="store_true", default=True)
AppLauncher.add_app_launcher_args(parser)
args, _ = parser.parse_known_args()

app_launcher = AppLauncher(args)
simulation_app = app_launcher.app

import gymnasium as gym
import torch

import drone_bombard  # noqa: F401

from drone_bombard.config.drone_drop_env_cfg import DroneBombardEnvCfg


def main():
    cfg = DroneBombardEnvCfg()
    cfg.scene.num_envs = args.num_envs
    cfg.payload_attachment_mode = "kinematic_sync"

    try:
        env = gym.make("Isaac-DroneDrop-v0", cfg=cfg)
    except TypeError:
        from drone_bombard.env import DroneBombardEnv

        env = DroneBombardEnv(cfg=cfg)

    obs, _ = env.reset()
    print(f"[smoke] obs shape: {obs['policy'].shape}")

    dropped_ever = False
    landed_ever = False

    for step in range(args.max_steps):
        action = torch.zeros(env.num_envs, 5, device=env.device)
        action[:, 2] = 0.1  # slight upward vz command
        obs, rew, terminated, truncated, extras = env.step(action)

        if env.dropped.any().item():
            dropped_ever = True
        payload_z = env.scene["payload"].data.root_pos_w[:, 2]
        if (env.dropped & (payload_z <= 0.04)).any().item():
            landed_ever = True

        if (terminated | truncated).any().item():
            print(f"[smoke] episode done at step {step}")
            print(f"  dropped={dropped_ever} landed={landed_ever}")
            if hasattr(env, "drop_error_actual"):
                err = env.drop_error_actual[torch.isfinite(env.drop_error_actual)]
                if err.numel():
                    print(f"  drop_error_actual_m mean={err.mean().item():.3f}")
            break
    else:
        print(f"[smoke] reached max_steps={args.max_steps} without done")

    assert obs["policy"].shape[-1] == 17, "obs must be 17D"
    print("[smoke] PASS — 17D obs, step loop OK")
    env.close()
    simulation_app.close()


if __name__ == "__main__":
    main()
