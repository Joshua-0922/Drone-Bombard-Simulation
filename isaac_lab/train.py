"""Train Isaac-DroneBombard-Direct-v0 with rsl_rl PPO.

Ports the Spot-VM-preemption and checkpoint-hygiene behavior of
``ros2_ws/src/rl_navigation/rl_navigation/train_sac.py`` (SIGTERM emergency
save, keep-last-N rolling checkpoints, periodic milestone archives) onto
Isaac Lab's standard rsl_rl launch shape.

Usage (inside the isaac-lab Docker image, on the L4 Spot VM):
    ./isaaclab.sh -p /workspace/drone-bombard/isaac_lab/train.py \\
        --task Isaac-DroneBombard-Direct-v0 --headless --num_envs 2048
"""

import argparse
import glob
import os
import signal
import sys

from isaaclab.app import AppLauncher

parser = argparse.ArgumentParser(description="Train Isaac-DroneBombard-Direct-v0 with rsl_rl PPO.")
parser.add_argument("--task", type=str, default="Isaac-DroneBombard-Direct-v0")
parser.add_argument("--num_envs", type=int, default=None)
parser.add_argument("--max_iterations", type=int, default=None)
parser.add_argument("--seed", type=int, default=None)
parser.add_argument("--resume", type=str, default=None, help="Path to a checkpoint to resume from, or 'latest'.")
parser.add_argument("--checkpoint_dir", type=str, default="/workspace/logs/isaac_lab/drone_bombard")
parser.add_argument("--keep_last", type=int, default=5)
parser.add_argument("--milestone_interval", type=int, default=500, help="Iterations between permanent milestone archives.")
parser.add_argument("--wandb_project", type=str, default="drone-bombard-isaac")
AppLauncher.add_app_launcher_args(parser)
args_cli = parser.parse_args()
args_cli.headless = getattr(args_cli, "headless", True) or True

app_launcher = AppLauncher(args_cli)
simulation_app = app_launcher.app

import gymnasium as gym
import torch
from rsl_rl.runners import OnPolicyRunner

from isaaclab_rl.rsl_rl import RslRlVecEnvWrapper, RslRlOnPolicyRunnerCfg
from isaaclab_tasks.utils import parse_env_cfg

import drone_bombard  # noqa: F401 - registers Isaac-DroneBombard-Direct-v0
from drone_bombard.agents.rsl_rl_ppo_cfg import DroneBombardPPORunnerCfg


def _latest_checkpoint(ckpt_dir: str) -> str | None:
    files = sorted(glob.glob(os.path.join(ckpt_dir, "model_*.pt")), key=os.path.getmtime)
    return files[-1] if files else None


def _cleanup_old_checkpoints(ckpt_dir: str, keep_last: int):
    files = sorted(glob.glob(os.path.join(ckpt_dir, "model_*.pt")), key=os.path.getmtime)
    for old in files[:-keep_last] if keep_last > 0 else []:
        try:
            os.remove(old)
            print(f"[Cleanup] Deleted old checkpoint: {old}")
        except OSError:
            pass


def main():
    env_cfg = parse_env_cfg(args_cli.task, num_envs=args_cli.num_envs)
    agent_cfg: RslRlOnPolicyRunnerCfg = DroneBombardPPORunnerCfg()
    if args_cli.max_iterations is not None:
        agent_cfg.max_iterations = args_cli.max_iterations
    if args_cli.seed is not None:
        agent_cfg.seed = args_cli.seed

    os.makedirs(args_cli.checkpoint_dir, exist_ok=True)

    env = gym.make(args_cli.task, cfg=env_cfg)
    env = RslRlVecEnvWrapper(env)

    agent_cfg.logger = "wandb"
    agent_cfg.wandb_project = args_cli.wandb_project

    runner = OnPolicyRunner(env, agent_cfg.to_dict(), log_dir=args_cli.checkpoint_dir, device=agent_cfg.device)

    resume_path = args_cli.resume
    if resume_path == "latest":
        resume_path = _latest_checkpoint(args_cli.checkpoint_dir)
    if resume_path:
        print(f"[Resume] Loading checkpoint: {resume_path}")
        runner.load(resume_path)

    def _emergency_save(signum, frame):
        preempt_path = os.path.join(args_cli.checkpoint_dir, "model_preempt.pt")
        try:
            runner.save(preempt_path)
            print(f"[Preempt] Emergency checkpoint saved: {preempt_path}")
        finally:
            try:
                import wandb
                if wandb.run:
                    wandb.save(preempt_path)
                    wandb.finish()
            except ImportError:
                pass
        sys.exit(0)

    signal.signal(signal.SIGTERM, _emergency_save)

    total_iterations = agent_cfg.max_iterations
    milestone_dir = os.path.join(args_cli.checkpoint_dir, "milestones")
    os.makedirs(milestone_dir, exist_ok=True)

    done_iterations = 0
    while done_iterations < total_iterations:
        chunk = min(agent_cfg.save_interval, total_iterations - done_iterations)
        runner.learn(num_learning_iterations=chunk, init_at_random_ep_len=(done_iterations == 0))
        done_iterations += chunk

        _cleanup_old_checkpoints(args_cli.checkpoint_dir, args_cli.keep_last)

        if done_iterations % args_cli.milestone_interval == 0 or done_iterations >= total_iterations:
            milestone_path = os.path.join(milestone_dir, f"model_milestone_{done_iterations}.pt")
            runner.save(milestone_path)
            print(f"[Milestone] Saved {milestone_path}")

    final_path = os.path.join(args_cli.checkpoint_dir, "model_final.pt")
    runner.save(final_path)
    print(f"[Done] Final model saved: {final_path}")


if __name__ == "__main__":
    main()
    simulation_app.close()
