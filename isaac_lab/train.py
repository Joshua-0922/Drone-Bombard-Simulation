"""Train Isaac-DroneBombard-Direct-v0 with rsl_rl PPO.

Mirrors Isaac Lab v2.3.2's stock rsl_rl train script API
(RslRlVecEnvWrapper(clip_actions=...), OnPolicyRunner, single learn call)
while building the env cfg directly (the path proven by
verify_one_episode.py) and adding a SIGTERM preempt-save for Spot VMs.

Usage (inside the isaac-lab container, on the L4 Spot VM or any host with a
working install):
    ./isaaclab.sh -p train.py --task Isaac-DroneBombard-Direct-v0 \\
        --headless --num_envs 2048
Small dry-run:
    ./isaaclab.sh -p train.py --headless --num_envs 256 --max_iterations 20
"""

import argparse
import os
import signal
import sys
from datetime import datetime

from isaaclab.app import AppLauncher

parser = argparse.ArgumentParser(description="Train Isaac-DroneBombard-Direct-v0 with rsl_rl PPO.")
parser.add_argument("--task", type=str, default="Isaac-DroneBombard-Direct-v0")
parser.add_argument("--num_envs", type=int, default=2048)
parser.add_argument("--max_iterations", type=int, default=None)
parser.add_argument("--seed", type=int, default=42)
parser.add_argument("--resume", type=str, default=None, help="Path to a checkpoint (.pt) to resume from.")
parser.add_argument("--log_root", type=str, default="/workspace/logs/isaac_lab/drone_bombard")
parser.add_argument("--run_name", type=str, default="")
parser.add_argument("--wandb_project", type=str, default="drone-bombard-isaac")
parser.add_argument("--logger", type=str, default="wandb", choices=["wandb", "tensorboard"])
AppLauncher.add_app_launcher_args(parser)
args_cli = parser.parse_args()

app_launcher = AppLauncher(args_cli)
simulation_app = app_launcher.app

import gymnasium as gym  # noqa: E402
import torch  # noqa: E402
from rsl_rl.runners import OnPolicyRunner  # noqa: E402

from isaaclab_rl.rsl_rl import RslRlVecEnvWrapper  # noqa: E402

import drone_bombard  # noqa: F401,E402 - registers the task
from drone_bombard.drone_bombard_env import DroneBombardEnvCfg  # noqa: E402
from drone_bombard.agents.rsl_rl_ppo_cfg import DroneBombardPPORunnerCfg  # noqa: E402

torch.backends.cuda.matmul.allow_tf32 = True
torch.backends.cudnn.allow_tf32 = True


def main():
    # --- env cfg (built directly — the path proven by verify_one_episode.py) ---
    env_cfg = DroneBombardEnvCfg()
    env_cfg.scene.num_envs = args_cli.num_envs
    env_cfg.sim.device = args_cli.device if args_cli.device is not None else "cuda:0"
    env_cfg.seed = args_cli.seed

    # --- agent cfg ---
    agent_cfg: DroneBombardPPORunnerCfg = DroneBombardPPORunnerCfg()
    agent_cfg.seed = args_cli.seed
    if args_cli.max_iterations is not None:
        agent_cfg.max_iterations = args_cli.max_iterations
    agent_cfg.logger = args_cli.logger
    agent_cfg.wandb_project = args_cli.wandb_project
    if args_cli.run_name:
        agent_cfg.run_name = args_cli.run_name

    log_root = os.path.abspath(os.path.join(args_cli.log_root, agent_cfg.experiment_name))
    log_dir = datetime.now().strftime("%Y-%m-%d_%H-%M-%S")
    if agent_cfg.run_name:
        log_dir += f"_{agent_cfg.run_name}"
    log_dir = os.path.join(log_root, log_dir)
    os.makedirs(log_dir, exist_ok=True)
    print(f"[INFO] Logging experiment to: {log_dir}")

    # --- build env + rsl_rl runner (stock v2.3.2 API) ---
    env = gym.make(args_cli.task, cfg=env_cfg)
    env = RslRlVecEnvWrapper(env, clip_actions=agent_cfg.clip_actions)
    runner = OnPolicyRunner(env, agent_cfg.to_dict(), log_dir=log_dir, device=agent_cfg.device)

    if args_cli.resume:
        print(f"[INFO] Resuming from checkpoint: {args_cli.resume}")
        runner.load(args_cli.resume)

    # --- Spot-VM preemption: save on SIGTERM before the box dies ---
    def _emergency_save(signum, frame):
        path = os.path.join(log_dir, "model_preempt.pt")
        try:
            runner.save(path)
            print(f"[Preempt] Emergency checkpoint saved: {path}", flush=True)
        finally:
            try:
                import wandb
                if wandb.run is not None:
                    wandb.finish()
            except Exception:  # noqa: BLE001
                pass
        sys.exit(0)

    signal.signal(signal.SIGTERM, _emergency_save)

    # --- train (single call; rsl_rl saves every save_interval to log_dir) ---
    runner.learn(num_learning_iterations=agent_cfg.max_iterations, init_at_random_ep_len=True)

    runner.save(os.path.join(log_dir, "model_final.pt"))
    print(f"[Done] Final model saved to {log_dir}/model_final.pt")
    env.close()


if __name__ == "__main__":
    main()
    simulation_app.close()
