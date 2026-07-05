"""_probe_traj.py — exp_014 pre-ablation probes A+B data capture.

Runs the exp013_v2 policy DETERMINISTICALLY on the kick-fixed plant
(spawn-time mass authoring, cd0c617) and records full per-step trajectories
alongside the Probe-A termination census:

  Probe A — does the termination distribution (esp. max_altitude rate) match
            the original eval taken on the kick-era plant? Printed at exit.
  Probe B — z(t) / vision-centering readout of max_altitude episodes:
            deliberate climb-with-target-framed vs loss-of-control spike.
            Analyzed OFFLINE from the saved npz.
  Probe C — offline reward counterfactual (range-falloff conf) — same npz.

Per loop iteration the PRE-step state is recorded (post-step reads would show
the fresh spawn for envs that terminated inside step()). The terminal state
itself lives one physics-decimation inside the step; for shape classification
the last pre-terminal sample (~0.1 s earlier) is sufficient.

Output npz arrays, all [T, N] unless noted:
  z, d_xy, u_norm, v_norm, conf, vz, speed, ang_speed  — float16/32 state
  done (bool)   — episode ended at this transition
  cause (int8)  — 0=running, else 1-based index into CAUSES at done steps

Run (inside isaac-verify container):
  ./isaaclab.sh -p /workspace/drone-bombard/isaac_lab/_probe_traj.py \
      --headless --episodes 200 --num_envs 32 \
      --policy /workspace/logs/isaac_lab/drone_bombard/drone_bombard_ppo/2026-07-03_15-15-30_exp013_v2_visionfix/model_final.pt \
      --out /workspace/drone-bombard/isaac_lab/_probe_out/probeA_traj.npz
"""

import argparse
import os

from isaaclab.app import AppLauncher

parser = argparse.ArgumentParser(description="exp_014 probe A/B trajectory capture.")
parser.add_argument("--task", type=str, default="Isaac-DroneBombard-Direct-v0")
parser.add_argument("--num_envs", type=int, default=32)
parser.add_argument("--episodes", type=int, default=200)
parser.add_argument("--policy", type=str, required=True)
parser.add_argument("--out", type=str, required=True)
AppLauncher.add_app_launcher_args(parser)
args_cli = parser.parse_args()

app_launcher = AppLauncher(args_cli)
simulation_app = app_launcher.app

import numpy as np
import torch

import gymnasium as gym
from isaaclab_rl.rsl_rl import RslRlVecEnvWrapper
from isaaclab_tasks.utils import parse_env_cfg

import drone_bombard  # noqa: F401

CAUSES = ("success", "crash", "overspeed", "bad_attitude", "out_of_range",
          "max_altitude", "overshoot", "stagnation", "timeout")


def main():
    env_cfg = parse_env_cfg(args_cli.task, num_envs=args_cli.num_envs)
    env = gym.make(args_cli.task, cfg=env_cfg)
    env = RslRlVecEnvWrapper(env)

    from rsl_rl.runners import OnPolicyRunner
    from drone_bombard.agents.rsl_rl_ppo_cfg import DroneBombardPPORunnerCfg

    agent_cfg = DroneBombardPPORunnerCfg()
    runner = OnPolicyRunner(env, agent_cfg.to_dict(), log_dir=None, device=agent_cfg.device)
    runner.load(args_cli.policy)
    policy = runner.get_inference_policy(device=env.unwrapped.device)

    obs, _ = env.reset()
    u = env.unwrapped

    rec = {k: [] for k in ("z", "d_xy", "u_norm", "v_norm", "conf", "vz",
                           "speed", "ang_speed", "done", "cause")}
    n_done = 0
    cause_counts = {c: 0 for c in CAUSES}

    while n_done < args_cli.episodes:
        pos = u._robot.data.root_pos_w - u.scene.env_origins
        vel = u._robot.data.root_lin_vel_w
        ang = u._robot.data.root_ang_vel_b
        un, vn, conf = u._last_vision
        d_xy = u._current_d_xy()

        rec["z"].append(pos[:, 2].cpu().numpy().astype(np.float32))
        rec["d_xy"].append(d_xy.cpu().numpy().astype(np.float32))
        rec["u_norm"].append(un.cpu().numpy().astype(np.float32))
        rec["v_norm"].append(vn.cpu().numpy().astype(np.float32))
        rec["conf"].append(conf.cpu().numpy().astype(np.float32))
        rec["vz"].append(vel[:, 2].cpu().numpy().astype(np.float32))
        rec["speed"].append(torch.linalg.norm(vel, dim=-1).cpu().numpy().astype(np.float32))
        rec["ang_speed"].append(torch.linalg.norm(ang, dim=-1).cpu().numpy().astype(np.float32))

        with torch.inference_mode():
            action = policy(obs)
        obs, rew, dones, info = env.step(action)
        done = dones.bool()

        cause_code = torch.zeros(u.num_envs, dtype=torch.int8)
        if done.any():
            f = u._done_flags
            n_done += int(done.sum().item())
            for i, c in enumerate(CAUSES):
                hit = (f[c] & done)
                cause_counts[c] += int(hit.sum().item())
                cause_code[hit.cpu()] = i + 1
        rec["done"].append(done.cpu().numpy())
        rec["cause"].append(cause_code.numpy())

    os.makedirs(os.path.dirname(args_cli.out), exist_ok=True)
    np.savez_compressed(args_cli.out, **{k: np.stack(v) for k, v in rec.items()},
                        causes=np.array(CAUSES))

    total = sum(cause_counts.values())
    print(f"[probeA] episodes={total} (requested {args_cli.episodes}, batch overshoot possible)")
    print(f"[probeA] success_rate={cause_counts['success']/max(total,1):.2%}")
    print("[probeA] termination causes: " + ", ".join(
        f"{c}={cause_counts[c]}" for c in CAUSES))
    print(f"[probeA] wrote trajectories to {args_cli.out}")


if __name__ == "__main__":
    main()
    simulation_app.close()
