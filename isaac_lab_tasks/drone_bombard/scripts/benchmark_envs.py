#!/usr/bin/env python3
"""num_envs 스윕 FPS 벤치마크 (scale-train 검증).

사용법:
  python benchmark_envs.py --env_counts 1,64,256,1024 --steps 200 --headless
"""

from __future__ import annotations

import argparse
import time

from isaaclab.app import AppLauncher

parser = argparse.ArgumentParser()
parser.add_argument("--env_counts", type=str, default="1,64,256,1024")
parser.add_argument("--steps", type=int, default=200)
parser.add_argument("--headless", action="store_true", default=True)
AppLauncher.add_app_launcher_args(parser)
args, _ = parser.parse_known_args()

app_launcher = AppLauncher(args)
simulation_app = app_launcher.app

import gymnasium as gym
import torch

import drone_bombard  # noqa: F401

from drone_bombard.config.drone_drop_env_cfg import DroneBombardEnvCfg


def bench_one(n: int, steps: int) -> float:
    cfg = DroneBombardEnvCfg()
    cfg.scene.num_envs = n
    try:
        env = gym.make("Isaac-DroneDrop-v0", cfg=cfg)
    except TypeError:
        from drone_bombard.env import DroneBombardEnv

        env = DroneBombardEnv(cfg=cfg)

    env.reset()
    action = torch.zeros(n, 5, device=env.device)
    t0 = time.perf_counter()
    for _ in range(steps):
        env.step(action)
    elapsed = time.perf_counter() - t0
    env.close()
    return (steps * n) / max(elapsed, 1e-9)


def main():
    counts = [int(x.strip()) for x in args.env_counts.split(",")]
    print("num_envs,env_steps_per_sec")
    for n in counts:
        try:
            fps = bench_one(n, args.steps)
            print(f"{n},{fps:.1f}")
        except Exception as exc:
            print(f"{n},FAIL ({exc})")
    simulation_app.close()


if __name__ == "__main__":
    main()
