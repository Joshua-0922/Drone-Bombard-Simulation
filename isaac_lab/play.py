"""Play / sanity-check Isaac-DroneBombard-Direct-v0.

Modes:
  --policy CKPT     roll out a trained checkpoint, print per-episode
                     success/final-d_xy table.
  --zero-actions     hover sanity check: altitude must hold (+/-1 m over 10s),
                     no NaNs. Validates the physics/actuation wiring alone.
  --scripted          constant-velocity-toward-target flight: d_xy must fall
                     monotonically and u/v must trend toward 0 (frame centre).
  --step-response     drives the velocity controller through the
                     calibration operating-point matrix (see
                     notes/research/isaac_velocity_controller.md) and dumps
                     per-axis command/response traces to CSV for comparison
                     against recorded PX4 SITL step responses.
"""

import argparse

from isaaclab.app import AppLauncher

parser = argparse.ArgumentParser(description="Play / sanity-check Isaac-DroneBombard-Direct-v0.")
parser.add_argument("--task", type=str, default="Isaac-DroneBombard-Direct-v0")
parser.add_argument("--num_envs", type=int, default=4)
parser.add_argument("--policy", type=str, default=None)
parser.add_argument("--zero-actions", action="store_true")
parser.add_argument("--scripted", action="store_true")
parser.add_argument("--step-response", action="store_true")
parser.add_argument("--episodes", type=int, default=10)
parser.add_argument("--out-csv", type=str, default="/workspace/logs/isaac_lab/step_response.csv")
AppLauncher.add_app_launcher_args(parser)
args_cli = parser.parse_args()

app_launcher = AppLauncher(args_cli)
simulation_app = app_launcher.app

import csv
import torch

import gymnasium as gym
from isaaclab_rl.rsl_rl import RslRlVecEnvWrapper
from isaaclab_tasks.utils import parse_env_cfg

import drone_bombard  # noqa: F401


# Operating-point matrix — see the migration plan §7b. Each entry is a
# constant ENU velocity command (vx, vy, vz, yaw_rate) as normalized [-1,1]
# action, held for HOLD_STEPS policy steps then returned to hover.
_ACTION_SCALES = (4.0, 3.0, 3.0, 1.0)
_STEP_MATRIX = [
    ("fwd_1ms", (1.0 / _ACTION_SCALES[0], 0, 0, 0)),
    ("fwd_2ms", (2.0 / _ACTION_SCALES[0], 0, 0, 0)),
    ("fwd_4ms", (4.0 / _ACTION_SCALES[0], 0, 0, 0)),
    ("lat_1p5ms", (0, 1.5 / _ACTION_SCALES[1], 0, 0)),
    ("lat_3ms", (0, 3.0 / _ACTION_SCALES[1], 0, 0)),
    ("vert_1ms", (0, 0, 1.0 / _ACTION_SCALES[2], 0)),
    ("vert_neg1ms", (0, 0, -1.0 / _ACTION_SCALES[2], 0)),
    ("vert_3ms", (0, 0, 3.0 / _ACTION_SCALES[2], 0)),
    ("vert_neg3ms", (0, 0, -3.0 / _ACTION_SCALES[2], 0)),
    ("diag_2_2_1", (2.0 / _ACTION_SCALES[0], 2.0 / _ACTION_SCALES[1], 1.0 / _ACTION_SCALES[2], 0)),
]
_HOLD_STEPS = 150  # 15 s @ 10 Hz — enough for settle at these gains


def run_zero_actions(env, steps=100):
    obs, _ = env.reset()
    action_dim = env.unwrapped.single_action_space.shape[0]
    device = env.unwrapped.device
    n = env.unwrapped.num_envs
    zero = torch.zeros(n, action_dim, device=device)
    alt0 = env.unwrapped._robot.data.root_pos_w[:, 2].clone()
    max_drift = torch.zeros(n, device=device)
    for _ in range(steps):
        obs, rew, dones, info = env.step(zero)
        alt = env.unwrapped._robot.data.root_pos_w[:, 2]
        max_drift = torch.maximum(max_drift, (alt - alt0).abs())
        if torch.isnan(obs["policy"]).any():
            print("[FAIL] NaN detected in observations during zero-action hover.")
            return False
    ok = bool((max_drift < 1.0).all())
    print(f"[zero-actions] max altitude drift over {steps} steps: {max_drift.max().item():.3f} m -> {'PASS' if ok else 'FAIL'}")
    return ok


def run_scripted(env, episodes=5):
    obs, _ = env.reset()
    device = env.unwrapped.device
    n = env.unwrapped.num_envs
    action_dim = env.unwrapped.single_action_space.shape[0]  # 6 (vel[0:4] + CCIP residual[4:6])
    successes, finals = 0, []
    for ep in range(episodes):
        done = torch.zeros(n, dtype=torch.bool, device=device)
        d_xy_trace = []
        for _ in range(300):
            pos = env.unwrapped._robot.data.root_pos_w - env.unwrapped.scene.env_origins
            to_target = env.unwrapped._target_xy - pos[:, :2]
            dist = torch.linalg.norm(to_target, dim=-1, keepdim=True).clamp(min=1e-6)
            dir_xy = to_target / dist
            action = torch.zeros(n, action_dim, device=device)  # residual dims stay 0
            action[:, 0] = dir_xy[:, 0] * 0.5
            action[:, 1] = dir_xy[:, 1] * 0.5
            obs, rew, dones, info = env.step(action)
            d_xy = env.unwrapped._current_d_xy()
            d_xy_trace.append(d_xy.mean().item())
            done = done | dones.bool()
            if done.all():
                break
        finals.append(d_xy_trace[-1])
        successes += int(env.unwrapped._done_flags["success"].sum().item())
        print(f"[scripted] episode {ep}: d_xy trace start={d_xy_trace[0]:.2f} end={d_xy_trace[-1]:.2f}")
    print(f"[scripted] successes={successes} finals_mean={sum(finals)/len(finals):.2f}")


def run_step_response(env, out_csv):
    device = env.unwrapped.device
    n = env.unwrapped.num_envs
    action_dim = env.unwrapped.single_action_space.shape[0]
    rows = []
    for name, act_vals in _STEP_MATRIX:
        env.reset()
        for _ in range(30):  # settle to hover first
            env.step(torch.zeros(n, action_dim, device=device))
        # _STEP_MATRIX entries are 4-tuples (vel dims); pad the CCIP residual
        # dims with zeros to match the 6-dim action space.
        act = torch.zeros(n, action_dim, device=device)
        act[:, :len(act_vals)] = torch.tensor(act_vals, device=device).unsqueeze(0).repeat(n, 1)
        for t in range(_HOLD_STEPS):
            env.step(act)
            vel = env.unwrapped._robot.data.root_lin_vel_w[0].tolist()
            rows.append([name, t * 0.1, act_vals[0], act_vals[1], act_vals[2], *vel])
    with open(out_csv, "w", newline="") as f:
        writer = csv.writer(f)
        writer.writerow(["point", "t_s", "cmd_vx_norm", "cmd_vy_norm", "cmd_vz_norm", "vx", "vy", "vz"])
        writer.writerows(rows)
    print(f"[step-response] wrote {len(rows)} rows to {out_csv}")


def run_policy(env, policy_path, episodes=10):
    from rsl_rl.runners import OnPolicyRunner
    from drone_bombard.agents.rsl_rl_ppo_cfg import DroneBombardPPORunnerCfg

    agent_cfg = DroneBombardPPORunnerCfg()
    runner = OnPolicyRunner(env, agent_cfg.to_dict(), log_dir=None, device=agent_cfg.device)
    runner.load(policy_path)
    policy = runner.get_inference_policy(device=env.unwrapped.device)

    obs, _ = env.reset()
    causes = ("success", "crash", "overspeed", "bad_attitude", "out_of_range",
              "max_altitude", "overshoot", "stagnation", "timeout")
    n_done = 0
    cause_counts = {c: 0 for c in causes}
    # d_xy readings taken after env.step() are post-reset (DirectRLEnv resets
    # done envs inside step), so terminal state must come from the env's own
    # pre-reset snapshot. _last_final_snapshot carries the per-episode arrays
    # for each reset batch (extras["log"] only has batch means) — accumulate
    # them so the summary can report full distributions, not just means.
    ep = {k: [] for k in ("d_xy_min", "released", "release_impact_err",
                          "aim_err_min", "terminal_impact_err", "final_speed_xy")}
    near_miss_wsum, log_w = 0.0, 0
    while n_done < episodes:
        with torch.inference_mode():
            action = policy(obs)
        obs, rew, dones, info = env.step(action)
        done = dones.bool()
        if done.any():
            f = env.unwrapped._done_flags
            n = int(done.sum().item())
            n_done += n
            for c in causes:
                cause_counts[c] += int(f[c][done].sum().item())
            log = info.get("log", {}) if isinstance(info, dict) else {}
            near_miss_wsum += float(log.get("Episode_Termination/timeout_near_miss", 0.0)) * n
            log_w += n
            snap = getattr(env.unwrapped, "_last_final_snapshot", None)
            if snap is not None:
                u = env.unwrapped
                terminal_impact = u._predicted_impact_from_snapshot(snap)
                ep["terminal_impact_err"].append(
                    torch.linalg.norm(terminal_impact - snap["target_xy"], dim=-1).cpu())
                ep["d_xy_min"].append(snap["d_xy_min"].cpu())
                ep["released"].append(snap["released"].cpu())
                ep["release_impact_err"].append(snap["release_impact_err"].cpu())
                ep["aim_err_min"].append(snap["aim_err_min"].cpu())
                ep["final_speed_xy"].append(
                    torch.linalg.norm(snap["final_vel_xy"], dim=-1).cpu())
    print(f"[policy] episodes={n_done} success_rate={cause_counts['success']/max(n_done,1):.2%}")
    print(f"[policy] termination causes: " + ", ".join(
        f"{c}={cause_counts[c]}" for c in causes if cause_counts[c] > 0))
    if ep["d_xy_min"]:
        cat = {k: torch.cat(v) for k, v in ep.items()}
        released = cat["released"].bool()
        rel_rate = released.float().mean().item()

        def stats(t):
            if t.numel() == 0:
                return "n/a"
            q = torch.quantile(t, torch.tensor([0.5, 0.9]))
            return f"mean={t.mean():.3f} med={q[0]:.3f} p90={q[1]:.3f} max={t.max():.3f}"

        print(f"[policy] d_xy_min: {stats(cat['d_xy_min'][torch.isfinite(cat['d_xy_min'])])} m")
        print(f"[policy] release_rate={rel_rate:.2%} ({int(released.sum())}/{released.numel()})"
              f" | drop_impact_error@release: {stats(cat['release_impact_err'][released])} m")
        print(f"[policy] aim_err_min (per-episode min CCIP error): "
              f"{stats(cat['aim_err_min'][torch.isfinite(cat['aim_err_min'])])} m")
        print(f"[policy] drop_impact_error_terminal (legacy, at episode end): "
              f"{stats(cat['terminal_impact_err'])} m")
        print(f"[policy] final_speed_xy at termination: {stats(cat['final_speed_xy'])} m/s")
        print(f"[policy] timeout_near_miss_rate={near_miss_wsum/max(log_w,1):.2%}")


def main():
    env_cfg = parse_env_cfg(args_cli.task, num_envs=args_cli.num_envs)
    env = gym.make(args_cli.task, cfg=env_cfg)
    env = RslRlVecEnvWrapper(env)

    if args_cli.zero_actions:
        run_zero_actions(env)
    elif args_cli.scripted:
        run_scripted(env, episodes=args_cli.episodes)
    elif args_cli.step_response:
        run_step_response(env, args_cli.out_csv)
    elif args_cli.policy:
        run_policy(env, args_cli.policy, episodes=args_cli.episodes)
    else:
        print("Specify one of --zero-actions / --scripted / --step-response / --policy CKPT")

    env.close()


if __name__ == "__main__":
    main()
    simulation_app.close()
