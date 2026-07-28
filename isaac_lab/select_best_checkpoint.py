"""Sweep the model_{it}.pt checkpoints in a training log dir, evaluate each on
the (physical-drop) env, and copy the best one to ``model_best.pt``.

Motivation: v19 collapsed AFTER convergence (iter-375 released 100% at 0.56 m;
iter-499 released 0%). Stock rsl_rl only saves periodic checkpoints and has no
"best by metric" concept, so keeping the right one is manual. This picks the
checkpoint that maximises real-landing SUCCESS (tie-break: higher release rate,
then lower median landing error) — the metric that actually matters, not raw
training return (which the collapse inflates by hoarding shaping reward).

Loads the env ONCE and only reloads policy weights per checkpoint, so sweeping
~100 checkpoints costs one Isaac Sim boot, not one per file.

Usage:
  ./isaaclab.sh -p select_best_checkpoint.py --task Isaac-DroneBombard-V19-Direct-v0 \
      --log_dir /workspace/.../<run> --num_envs 64 --episodes 200 --headless
"""
import argparse
import glob
import os
import re
import shutil

from isaaclab.app import AppLauncher

parser = argparse.ArgumentParser()
parser.add_argument("--task", type=str, default="Isaac-DroneBombard-V19-Direct-v0")
parser.add_argument("--log_dir", type=str, required=True,
                    help="Directory containing model_*.pt checkpoints to sweep.")
parser.add_argument("--num_envs", type=int, default=64)
parser.add_argument("--episodes", type=int, default=200,
                    help="Episodes evaluated PER checkpoint.")
parser.add_argument("--every", type=int, default=1,
                    help="Evaluate every Nth checkpoint (1 = all).")
parser.add_argument("--out", type=str, default=None,
                    help="Where to write the best model (default: <log_dir>/model_best.pt).")
AppLauncher.add_app_launcher_args(parser)
args_cli = parser.parse_args()

app_launcher = AppLauncher(args_cli)
simulation_app = app_launcher.app

import gymnasium as gym  # noqa: E402
import torch  # noqa: E402
from isaaclab_tasks.utils import parse_env_cfg  # noqa: E402
from isaaclab_rl.rsl_rl import RslRlVecEnvWrapper  # noqa: E402
from rsl_rl.runners import OnPolicyRunner  # noqa: E402
from drone_bombard.agents.rsl_rl_ppo_cfg import DroneBombardPPORunnerCfg  # noqa: E402


def _ckpt_iter(path):
    m = re.search(r"model_(\d+)\.pt$", os.path.basename(path))
    return int(m.group(1)) if m else -1


def evaluate(env, policy, episodes):
    """Roll out `episodes` and return (success_rate, release_rate, med_land_err)."""
    obs, _ = env.reset()
    n_done = 0
    n_success = 0
    released_all, land_err_all = [], []
    while n_done < episodes:
        with torch.inference_mode():
            action = policy(obs)
        obs, _, dones, _ = env.step(action)
        done = dones.bool()
        if done.any():
            u = env.unwrapped
            f = u._done_flags
            n = int(done.sum().item())
            n_done += n
            n_success += int(f["success"][done].sum().item())
            snap = getattr(u, "_last_final_snapshot", None)
            if snap is not None:
                rel = snap["released"].bool()
                released_all.append(rel.cpu())
                land_err_all.append(snap["release_impact_err"][rel].cpu())
    success_rate = n_success / max(n_done, 1)
    rel_cat = torch.cat(released_all) if released_all else torch.zeros(1, dtype=torch.bool)
    release_rate = rel_cat.float().mean().item()
    land_cat = torch.cat(land_err_all) if land_err_all else torch.tensor([float("nan")])
    med_err = float(torch.nan_to_num(land_cat, nan=99.0).median()) if land_cat.numel() else 99.0
    return success_rate, release_rate, med_err


def main():
    ckpts = sorted(glob.glob(os.path.join(args_cli.log_dir, "model_*.pt")), key=_ckpt_iter)
    ckpts = [c for c in ckpts if _ckpt_iter(c) >= 0][:: args_cli.every]
    if not ckpts:
        print(f"[best] no model_*.pt found in {args_cli.log_dir}")
        simulation_app.close()
        return

    env_cfg = parse_env_cfg(args_cli.task, num_envs=args_cli.num_envs)
    env = gym.make(args_cli.task, cfg=env_cfg)
    env = RslRlVecEnvWrapper(env)

    agent_cfg = DroneBombardPPORunnerCfg()
    runner = OnPolicyRunner(env, agent_cfg.to_dict(), log_dir=None, device=agent_cfg.device)

    print(f"[best] sweeping {len(ckpts)} checkpoints from {args_cli.log_dir}")
    print(f"{'iter':>6} {'success':>8} {'release':>8} {'med_err':>8}")
    results = []
    for c in ckpts:
        runner.load(c)
        policy = runner.get_inference_policy(device=env.unwrapped.device)
        succ, rel, med = evaluate(env, policy, args_cli.episodes)
        results.append((c, succ, rel, med))
        print(f"{_ckpt_iter(c):>6} {succ:>8.2%} {rel:>8.2%} {med:>8.3f}")

    # Rank: success first, then release rate, then lower landing error.
    best = max(results, key=lambda r: (r[1], r[2], -r[3]))
    out = args_cli.out or os.path.join(args_cli.log_dir, "model_best.pt")
    shutil.copyfile(best[0], out)
    print(f"[best] winner: {os.path.basename(best[0])} "
          f"success={best[1]:.2%} release={best[2]:.2%} med_err={best[3]:.3f} m")
    print(f"[best] copied -> {out}")

    env.close()
    simulation_app.close()


if __name__ == "__main__":
    main()
