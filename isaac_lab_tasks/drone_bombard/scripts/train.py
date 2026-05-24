#!/usr/bin/env python3
"""드론 폭격 RL 학습 진입점 (SB3 SAC + Isaac Lab).

사용법:
  python train.py --task Isaac-DroneDrop-v0 --num_envs 4096 --wandb
  python train.py --task Isaac-DroneDrop-Play-v0 --num_envs 4 --headless False

Isaac Lab Sb3VecEnvWrapper 로 SB3 SAC 연결. ROS2 미사용 (GPU 텐서 직접 접근).
"""

from __future__ import annotations

import argparse
import os
import signal
import sys
import time

from isaaclab.app import AppLauncher

parser = argparse.ArgumentParser(description="Drone Bombard SAC 학습")
parser.add_argument(
    "--task",
    type=str,
    default="Isaac-DroneDrop-v0",
    help="Gymnasium task id (Isaac-DroneDrop-v0 | Isaac-DroneDrop-Play-v0)",
)
parser.add_argument("--num_envs", type=int, default=4096, help="병렬 환경 수")
parser.add_argument("--total_timesteps", type=int, default=5_000_000, help="총 학습 스텝")
parser.add_argument("--checkpoint", type=str, default=None, help="이어서 학습할 체크포인트")
parser.add_argument("--wandb", action="store_true", help="WandB 로깅 활성화")
parser.add_argument("--wandb_project", type=str, default="drone-bombard-isaac")
parser.add_argument("--log_dir", type=str, default="logs/sac_drone_drop")
parser.add_argument("--headless", action="store_true", default=True, help="GUI 없이 실행")
parser.add_argument(
    "--benchmark",
    action="store_true",
    help="학습 없이 num_envs 스텝 속도만 측정 후 종료",
)
parser.add_argument("--benchmark_steps", type=int, default=200, help="벤치마크 스텝 수")
AppLauncher.add_app_launcher_args(parser)
args, _ = parser.parse_known_args()

app_launcher = AppLauncher(args)
simulation_app = app_launcher.app

import gymnasium as gym
import torch
from stable_baselines3 import SAC
from stable_baselines3.common.callbacks import CallbackList, CheckpointCallback

from isaaclab_rl.sb3 import Sb3VecEnvWrapper  # type: ignore

import drone_bombard  # noqa: F401 — gym.register side effect

from drone_bombard.config.agents.sac_cfg import SacAgentCfg
from drone_bombard.config.drone_drop_env_cfg import DroneBombardEnvCfg, DroneBombardEnvCfg_PLAY


def _resolve_cfg(task_id: str, num_envs: int):
    """태스크 ID → env cfg (num_envs 오버라이드)."""
    if "Play" in task_id:
        cfg = DroneBombardEnvCfg_PLAY()
    else:
        cfg = DroneBombardEnvCfg()
    cfg.scene.num_envs = num_envs
    return cfg


def make_env(task_id: str, num_envs: int):
    """Gymnasium registry 또는 직접 cfg 로 환경 생성."""
    cfg = _resolve_cfg(task_id, num_envs)
    try:
        env = gym.make(task_id, cfg=cfg)
    except TypeError:
        from drone_bombard.env import DroneBombardEnv

        env = DroneBombardEnv(cfg=cfg)
    return env


def make_wandb_callback(project: str, cfg: SacAgentCfg):
    try:
        import wandb
        from stable_baselines3.common.callbacks import BaseCallback

        class WandbMetricsCallback(BaseCallback):
            def __init__(self):
                super().__init__()
                wandb.init(project=project, config=vars(cfg))

            def _on_step(self) -> bool:
                if self.n_calls % cfg.log_interval == 0:
                    infos = self.locals.get("infos", [])
                    for info in infos:
                        if not isinstance(info, dict):
                            continue
                        for k, v in info.items():
                            if isinstance(v, (int, float)) and not (isinstance(v, float) and v != v):
                                wandb.log({f"env/{k}": v}, step=self.num_timesteps)
                return True

            def _on_training_end(self):
                wandb.finish()

        return WandbMetricsCallback()
    except ImportError:
        print("[train] wandb 패키지 없음 — WandB 비활성화")
        return None


_model_ref = None


def _sigterm_handler(signum, frame):
    if _model_ref is not None:
        save_path = os.path.join(args.log_dir, "model_sigterm")
        _model_ref.save(save_path)
        print(f"[train] SIGTERM — 모델 저장: {save_path}")
    sys.exit(0)


signal.signal(signal.SIGTERM, _sigterm_handler)


def run_benchmark(env, steps: int) -> None:
    """병렬 환경 FPS 측정 (scale-train 검증)."""
    env = Sb3VecEnvWrapper(env)
    obs = env.reset()
    t0 = time.perf_counter()
    for _ in range(steps):
        action = env.action_space.sample()
        obs, _, _, _ = env.step(action)
    elapsed = time.perf_counter() - t0
    n = env.unwrapped.num_envs if hasattr(env.unwrapped, "num_envs") else args.num_envs
    fps = (steps * n) / max(elapsed, 1e-9)
    print(f"[benchmark] num_envs={n} steps={steps} elapsed={elapsed:.2f}s env_steps_per_sec={fps:.0f}")
    env.close()


def main():
    global _model_ref

    os.makedirs(args.log_dir, exist_ok=True)
    print(f"[train] task={args.task} num_envs={args.num_envs}")

    env = make_env(args.task, args.num_envs)

    if args.benchmark:
        run_benchmark(env, args.benchmark_steps)
        simulation_app.close()
        return

    env = Sb3VecEnvWrapper(env)
    sac_cfg = SacAgentCfg()
    device = "cuda" if torch.cuda.is_available() else "cpu"

    if args.checkpoint:
        model = SAC.load(args.checkpoint, env=env, device=device)
    else:
        model = SAC(
            policy="MlpPolicy",
            env=env,
            learning_rate=sac_cfg.learning_rate,
            buffer_size=sac_cfg.buffer_size,
            batch_size=sac_cfg.batch_size,
            tau=sac_cfg.tau,
            gamma=sac_cfg.gamma,
            learning_starts=sac_cfg.learning_starts,
            ent_coef=sac_cfg.ent_coef,
            target_entropy=sac_cfg.target_entropy,
            train_freq=sac_cfg.train_freq,
            gradient_steps=sac_cfg.gradient_steps,
            policy_kwargs={"net_arch": sac_cfg.net_arch},
            tensorboard_log=args.log_dir,
            verbose=1,
            device=device,
        )

    _model_ref = model
    callbacks = [
        CheckpointCallback(
            save_freq=sac_cfg.checkpoint_freq,
            save_path=os.path.join(args.log_dir, "checkpoints"),
            name_prefix="drone_drop_sac",
            save_replay_buffer=False,
        )
    ]
    if args.wandb:
        wb = make_wandb_callback(args.wandb_project, sac_cfg)
        if wb is not None:
            callbacks.append(wb)

    print(f"[train] 학습 시작 — {args.total_timesteps} timesteps")
    model.learn(
        total_timesteps=args.total_timesteps,
        callback=CallbackList(callbacks),
        log_interval=sac_cfg.log_interval,
        reset_num_timesteps=(args.checkpoint is None),
    )

    final_path = os.path.join(args.log_dir, "model_final")
    model.save(final_path)
    print(f"[train] 완료 — {final_path}")
    env.close()
    simulation_app.close()


if __name__ == "__main__":
    main()
