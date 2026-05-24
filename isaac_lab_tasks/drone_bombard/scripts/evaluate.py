#!/usr/bin/env python3
"""드론 폭격 정책 평가 스크립트.

사용법:
  python evaluate.py --checkpoint logs/sac_drone_drop/model_final --num_envs 64 --num_episodes 200

결과:
  - 평균 에피소드 보상
  - 투하 성공률 (miss distance <= 0.5 m)
  - 평균 miss distance
  - 평균 에피소드 길이
"""

from __future__ import annotations

import argparse

from isaaclab.app import AppLauncher

parser = argparse.ArgumentParser(description="드론 폭격 정책 평가")
parser.add_argument("--task", type=str, default="Isaac-DroneDrop-v0")
parser.add_argument("--checkpoint", type=str, required=True, help="모델 체크포인트 경로")
parser.add_argument("--num_envs", type=int, default=64)
parser.add_argument("--num_episodes", type=int, default=200)
parser.add_argument("--headless", action="store_true", default=True)
AppLauncher.add_app_launcher_args(parser)
args, _ = parser.parse_known_args()

app_launcher = AppLauncher(args)
simulation_app = app_launcher.app

import numpy as np
import torch
from stable_baselines3 import SAC
from isaaclab_rl.sb3 import Sb3VecEnvWrapper  # type: ignore

import gymnasium as gym

import drone_bombard  # noqa: F401

from drone_bombard.config.drone_drop_env_cfg import DroneBombardEnvCfg, DroneBombardEnvCfg_PLAY


def main():
    if "Play" in args.task:
        env_cfg = DroneBombardEnvCfg_PLAY()
    else:
        env_cfg = DroneBombardEnvCfg()
    env_cfg.scene.num_envs = args.num_envs

    try:
        env = gym.make(args.task, cfg=env_cfg)
    except TypeError:
        from drone_bombard.env import DroneBombardEnv

        env = DroneBombardEnv(cfg=env_cfg)
    env = Sb3VecEnvWrapper(env)

    model = SAC.load(args.checkpoint, env=env)
    model.policy.set_training_mode(False)

    episode_rewards = []
    episode_lengths = []
    drop_errors = []
    success_count = 0
    total_episodes = 0

    obs = env.reset()
    ep_reward = np.zeros(args.num_envs)
    ep_length = np.zeros(args.num_envs, dtype=int)

    while total_episodes < args.num_episodes:
        action, _ = model.predict(obs, deterministic=True)
        obs, rewards, dones, infos = env.step(action)

        ep_reward += rewards
        ep_length += 1

        for i, done in enumerate(dones):
            if done:
                episode_rewards.append(ep_reward[i])
                episode_lengths.append(ep_length[i])

                info = infos[i] if infos else {}
                d_err = info.get("drop_error_actual_m", None)
                if d_err is not None:
                    drop_errors.append(d_err)
                    if d_err <= 0.5:
                        success_count += 1

                total_episodes += 1
                ep_reward[i] = 0.0
                ep_length[i] = 0

                if total_episodes >= args.num_episodes:
                    break

    # 결과 출력
    print("\n" + "=" * 50)
    print("평가 결과")
    print("=" * 50)
    print(f"총 에피소드:          {total_episodes}")
    print(f"평균 에피소드 보상:   {np.mean(episode_rewards):.2f} ± {np.std(episode_rewards):.2f}")
    print(f"평균 에피소드 길이:   {np.mean(episode_lengths):.1f} steps")
    if drop_errors:
        print(f"투하 성공률 (≤0.5m): {success_count / len(drop_errors) * 100:.1f}%")
        print(f"평균 miss distance:  {np.mean(drop_errors):.3f} m")
        print(f"중앙값 miss distance:{np.median(drop_errors):.3f} m")
    else:
        print("투하 없음 (정책이 페이로드를 투하하지 않음)")
    print("=" * 50)

    env.close()
    simulation_app.close()


if __name__ == "__main__":
    main()
