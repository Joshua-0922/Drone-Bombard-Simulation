"""_diag_noise.py — measure how much policy Gaussian noise survives the action
pipeline (clip -> rate-limit +/-0.2 -> LPF 0.4 -> velocity controller) and
whether EXECUTED behavior saturates in sigma.

Claim under test (exp_013 Rule 18b): growing sigma (0.8 -> 3.9 during training)
produces almost no additional change in executed behavior — the entropy bonus
inflates sigma for free. Falsifiable form: executed-trajectory statistics are
~invariant for sigma in {0.8, 2.0, 3.9} while raw noise magnitude grows ~5x.

Method: load the trained exp013 checkpoint, take its DETERMINISTIC mean action
per step, add manual Gaussian noise of fixed sigma, and per policy step record
stage-by-stage magnitudes:
  raw |a|  ->  clip absorption / saturation  ->  executed post-rate-limit
  action delta (env._prev_action)  ->  LPF output delta (env._v_filt, m/s)
  ->  actual body velocity delta (m/s)
plus sign-agreement between the noisy action and the policy mean, and
task-level outcomes (termination causes, d_xy_min).

Masking: samples are dropped for envs with episode_length_buf < WARMUP (15)
(covers the post-reset transient AND any reset-kick pollution) and for envs
that terminated on that step (their deltas span the teleport).

Run (inside isaac-verify container):
  ./isaaclab.sh -p /workspace/drone-bombard/isaac_lab/_diag_noise.py --headless
"""

import argparse

from isaaclab.app import AppLauncher

parser = argparse.ArgumentParser(description="Action-pipeline noise attenuation diagnostic.")
parser.add_argument("--policy", type=str,
                    default="/workspace/logs/isaac_lab/drone_bombard/drone_bombard_ppo/2026-07-03_15-15-30_exp013_v2_visionfix/model_final.pt")
parser.add_argument("--num_envs", type=int, default=8)
parser.add_argument("--steps", type=int, default=300, help="policy steps per sigma condition")
parser.add_argument("--sigmas", type=str, default="0.0,0.8,2.0,3.9")
AppLauncher.add_app_launcher_args(parser)
args_cli = parser.parse_args()
args_cli.enable_cameras = True

app_launcher = AppLauncher(args_cli)
simulation_app = app_launcher.app

import os
import sys
import traceback

import torch

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))

import drone_bombard  # noqa: F401
from drone_bombard.drone_bombard_env import DroneBombardEnv, DroneBombardEnvCfg

WARMUP = 15  # policy steps after reset to discard
CAUSES = ("success", "crash", "overspeed", "bad_attitude", "out_of_range",
          "max_altitude", "overshoot", "stagnation", "timeout")


def main():
    from rsl_rl.runners import OnPolicyRunner
    from isaaclab_rl.rsl_rl import RslRlVecEnvWrapper
    from drone_bombard.agents.rsl_rl_ppo_cfg import DroneBombardPPORunnerCfg

    cfg = DroneBombardEnvCfg()
    cfg.scene.num_envs = args_cli.num_envs
    cfg.sim.device = "cuda:0"
    env = DroneBombardEnv(cfg)
    wrapped = RslRlVecEnvWrapper(env)
    device = env.device

    agent_cfg = DroneBombardPPORunnerCfg()
    runner = OnPolicyRunner(wrapped, agent_cfg.to_dict(), log_dir=None, device=agent_cfg.device)
    runner.load(args_cli.policy)
    policy = runner.get_inference_policy(device=device)

    sigmas = [float(s) for s in args_cli.sigmas.split(",")]
    torch.manual_seed(0)

    results = {}
    for sigma in sigmas:
        obs, _ = wrapped.reset()
        # accumulators (lists of per-step masked means)
        acc = {k: [] for k in (
            "raw_mag", "sat_frac", "clip_absorb", "exec_delta", "bind_frac",
            "vfilt_delta", "vel_delta", "sign_agree", "vz_abs", "speed")}
        cause_counts = {c: 0 for c in CAUSES}
        n_done = 0
        dxy_min_samples = []

        for t in range(args_cli.steps):
            with torch.inference_mode():
                mean_act = policy(obs)
            noise = torch.randn_like(mean_act) * sigma
            a_raw = mean_act + noise

            prev_before = env._prev_action.clone()
            vfilt_before = env._v_filt[:, :3].clone()
            vel_before = env._robot.data.root_lin_vel_w.clone()
            ep_len_before = env.episode_length_buf.clone()

            obs, rew, dones, extras = wrapped.step(a_raw)
            done = dones.bool()

            if done.any():
                f = env._done_flags
                n = int(done.sum().item())
                n_done += n
                for c in CAUSES:
                    cause_counts[c] += int(f[c][done].sum().item())
                log = extras.get("log", {}) if isinstance(extras, dict) else {}
                if "Episode_Metric/d_xy_min" in log:
                    dxy_min_samples.append((float(log["Episode_Metric/d_xy_min"]), n))

            valid = (ep_len_before >= WARMUP) & (~done)
            if not bool(valid.any()):
                continue
            v = valid

            a_clip = torch.clamp(a_raw, -1.0, 1.0)
            exec_delta = (env._prev_action - prev_before)[v]
            vfilt_delta = (env._v_filt[:, :3] - vfilt_before)[v]
            vel_delta = (env._robot.data.root_lin_vel_w - vel_before)[v]

            acc["raw_mag"].append(a_raw[v].abs().mean().item())
            acc["sat_frac"].append((a_raw[v].abs() > 1.0).float().mean().item())
            acc["clip_absorb"].append((a_raw - a_clip)[v].abs().mean().item())
            acc["exec_delta"].append(exec_delta.abs().mean().item())
            acc["bind_frac"].append((exec_delta.abs() >= 0.2 - 1e-6).float().mean().item())
            acc["vfilt_delta"].append(vfilt_delta.norm(dim=-1).mean().item())
            acc["vel_delta"].append(vel_delta.norm(dim=-1).mean().item())
            meaningful = mean_act[v].abs() > 0.05
            if bool(meaningful.any()):
                agree = (torch.sign(a_raw[v]) == torch.sign(mean_act[v]))[meaningful]
                acc["sign_agree"].append(agree.float().mean().item())
            acc["vz_abs"].append(env._robot.data.root_lin_vel_w[v, 2].abs().mean().item())
            acc["speed"].append(env._robot.data.root_lin_vel_w[v].norm(dim=-1).mean().item())

        def m(key):
            vals = acc[key]
            return sum(vals) / max(len(vals), 1)

        # variability of executed velocity across steps (behavioral spread)
        vel_series = torch.tensor(acc["vel_delta"]) if acc["vel_delta"] else torch.zeros(1)
        dxy_w = sum(n for _, n in dxy_min_samples)
        dxy_mean = (sum(val * n for val, n in dxy_min_samples) / dxy_w) if dxy_w else float("nan")
        results[sigma] = dict(
            raw_mag=m("raw_mag"), sat_frac=m("sat_frac"), clip_absorb=m("clip_absorb"),
            exec_delta=m("exec_delta"), bind_frac=m("bind_frac"),
            vfilt_delta=m("vfilt_delta"), vel_delta=m("vel_delta"),
            vel_delta_std=float(vel_series.std()) if vel_series.numel() > 1 else 0.0,
            sign_agree=m("sign_agree"), vz_abs=m("vz_abs"), speed=m("speed"),
            n_done=n_done, causes={c: n for c, n in cause_counts.items() if n},
            dxy_min=dxy_mean, n_samples=len(acc["raw_mag"]),
        )
        r = results[sigma]
        print(f"\n[SIGMA {sigma:４.1f}]" if False else f"\n[SIGMA {sigma:.1f}] steps_used={r['n_samples']}")
        print(f"  stage magnitudes : raw|a|={r['raw_mag']:.3f}  sat_frac={r['sat_frac']:.2%}  "
              f"clip_absorbed={r['clip_absorb']:.3f}")
        print(f"  executed         : |d prev_action|={r['exec_delta']:.4f}  rate-limiter binding={r['bind_frac']:.2%}")
        print(f"  plant            : |d v_filt|={r['vfilt_delta']:.4f} m/s  |d vel_actual|={r['vel_delta']:.4f} m/s "
              f"(step-to-step std {r['vel_delta_std']:.4f})")
        print(f"  policy-mean info : sign_agree(raw vs mean)={r['sign_agree']:.2%}")
        print(f"  behavior         : |vz|={r['vz_abs']:.3f}  speed={r['speed']:.3f}  d_xy_min(done)={r['dxy_min']:.3f}  "
              f"episodes_done={r['n_done']}  causes={r['causes']}")

    print("\n===== CROSS-SIGMA SUMMARY =====")
    base = results.get(0.8) or results[sigmas[0]]
    hdr = f"{'sigma':>6} {'raw|a|':>8} {'sat%':>7} {'exec_d':>8} {'bind%':>7} {'vfilt_d':>8} {'vel_d':>8} {'agree%':>7} {'done':>5}"
    print(hdr)
    for s in sigmas:
        r = results[s]
        print(f"{s:6.1f} {r['raw_mag']:8.3f} {r['sat_frac']*100:6.1f}% {r['exec_delta']:8.4f} "
              f"{r['bind_frac']*100:6.1f}% {r['vfilt_delta']:8.4f} {r['vel_delta']:8.4f} "
              f"{r['sign_agree']*100:6.1f}% {r['n_done']:5d}")
    if 3.9 in results and 0.8 in results:
        a, b = results[3.9], results[0.8]
        print(f"\nratio sigma3.9/sigma0.8: raw={a['raw_mag']/max(b['raw_mag'],1e-9):.2f}x  "
              f"exec_delta={a['exec_delta']/max(b['exec_delta'],1e-9):.2f}x  "
              f"vel_delta={a['vel_delta']/max(b['vel_delta'],1e-9):.2f}x")
    if 0.0 in results and 3.9 in results:
        a, b = results[3.9], results[0.0]
        print(f"ratio sigma3.9/deterministic: exec_delta={a['exec_delta']/max(b['exec_delta'],1e-9):.2f}x  "
              f"vel_delta={a['vel_delta']/max(b['vel_delta'],1e-9):.2f}x")

    env.close()


if __name__ == "__main__":
    try:
        main()
    except Exception:
        print("=== DIAG: FAIL — exception ===")
        traceback.print_exc()
        simulation_app.close()
        sys.exit(1)
    simulation_app.close()
