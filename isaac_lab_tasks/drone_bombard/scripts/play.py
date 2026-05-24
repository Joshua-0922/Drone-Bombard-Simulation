#!/usr/bin/env python3
"""드론 폭격 정책 시각화 스크립트.

사용법:
  python play.py --checkpoint logs/sac_drone_drop/model_final

헤드리스 모드를 끄고 Isaac Sim GUI 에서 실시간 렌더링한다.
환경 수를 4로 줄여 GPU 부하를 낮춘다.
"""

from __future__ import annotations

import argparse

from isaaclab.app import AppLauncher

parser = argparse.ArgumentParser(description="드론 폭격 정책 시각화")
parser.add_argument("--checkpoint", type=str, default=None, help="모델 체크포인트 (없으면 랜덤)")
parser.add_argument("--num_envs", type=int, default=4)
AppLauncher.add_app_launcher_args(parser)
args, _ = parser.parse_known_args()

# GUI 모드 강제
args.headless = False
app_launcher = AppLauncher(args)
simulation_app = app_launcher.app

import time
import torch
from stable_baselines3 import SAC
from isaaclab_rl.sb3 import Sb3VecEnvWrapper  # type: ignore

import gymnasium as gym

import drone_bombard  # noqa: F401

from drone_bombard.config.drone_drop_env_cfg import DroneBombardEnvCfg_PLAY


def main():
    env_cfg = DroneBombardEnvCfg_PLAY()
    env_cfg.scene.num_envs = args.num_envs

    try:
        env = gym.make("Isaac-DroneDrop-Play-v0", cfg=env_cfg)
    except TypeError:
        from drone_bombard.env import DroneBombardEnv

        env = DroneBombardEnv(cfg=env_cfg)
    env = Sb3VecEnvWrapper(env)

    if args.checkpoint:
        print(f"[play] 체크포인트 로드: {args.checkpoint}")
        model = SAC.load(args.checkpoint, env=env)
        model.policy.set_training_mode(False)
        use_model = True
    else:
        print("[play] 체크포인트 없음 — 랜덤 액션 실행")
        use_model = False

    obs = env.reset()
    ep = 0

    print("[play] 시뮬레이션 시작 (Ctrl+C 로 종료)")
    try:
        while simulation_app.is_running():
            if use_model:
                action, _ = model.predict(obs, deterministic=True)
            else:
                action = env.action_space.sample()

            obs, rewards, dones, infos = env.step(action)

            for i, done in enumerate(dones):
                if done:
                    ep += 1
                    info = infos[i] if infos else {}
                    d_err = info.get("drop_error_actual_m", "N/A")
                    rew = info.get("episode_reward", rewards[i])
                    print(f"  [Episode {ep}] env={i}  reward={rew:.1f}  miss={d_err}")

            # GUI 렌더 주기 유지
            time.sleep(0.01)

    except KeyboardInterrupt:
        print("[play] 종료")

    env.close()
    simulation_app.close()


if __name__ == "__main__":
    main()
