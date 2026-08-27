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
parser.add_argument("--task", type=str, default="Isaac-DroneBombard-Task-v0")
parser.add_argument("--num_envs", type=int, default=4)
parser.add_argument("--seed", type=int, default=42,
                    help="Env seed (same default as train.py) — without it eval resets are random "
                         "and runs are not comparable.")
parser.add_argument("--policy", type=str, default=None)
parser.add_argument("--zero-actions", action="store_true")
parser.add_argument("--scripted", action="store_true")
parser.add_argument("--step-response", action="store_true")
parser.add_argument("--wind-test", action="store_true",
                    help="Empirically verify the v15 airframe wind force: hover under a known forced wind "
                         "with cfg.wind_force_enabled OFF then ON, and compare the steady-state tilt "
                         "against atan(k*v^2/(m*g)).")
parser.add_argument("--wind-speed", type=float, default=5.0, help="Wind speed (m/s, +X) for --wind-test.")
parser.add_argument("--show", action="store_true",
                    help="Enable visual aids for watching (target beacon/disc + payload marker) and a "
                         "close chase camera. Use with --policy for livestream/GUI viewing.")
parser.add_argument("--drop-test", action="store_true",
                    help="Verify the v16 physical payload: cruise, force a release, and check the payload "
                         "physically falls along the drag-free ballistic curve (mid-air, before landing).")
parser.add_argument("--episodes", type=int, default=10)
parser.add_argument("--no_handoff_dr", action="store_true",
                    help="Evaluate with the handoff pinned to its nominal (fixed heading/speed/altitude/"
                         "attitude) even if the task randomizes it — the train/test-mismatch protocol "
                         "needs both directions.")
parser.add_argument("--dr_scale", type=float, default=None,
                    help="A-GROUP domain-randomization strength at EVALUATION time (wind, payload ballistic "
                         "coefficient, release-latency spread). Set it ABOVE the training value for the "
                         "unseen-robustness arm, or to 0 for the deterministic-model control. Leaves the "
                         "sensor/actuator group untouched.")
parser.add_argument("--observe_wind", action="store_true",
                    help="Evaluate with the true wind in the observation. Must MATCH how the policy was "
                         "trained — the observation width differs, so a mismatch fails to load.")
parser.add_argument("--release_10hz", action="store_true",
                    help="Resolve the release on the 10 Hz policy grid instead of at "
                         "physics rate (the ablation arm for release-timing resolution).")
parser.add_argument("--pixel_vision", action="store_true",
                    help="Evaluate with the pixel-quantized marker instead of the true position.")
parser.add_argument("--no_dyn_dr", action="store_true",
                    help="Evaluate with the nominal plant: no mass-belief/gain/ballistic-coefficient "
                         "mismatch and no obs/action noise, even if the task randomizes them.")
parser.add_argument("--paired_eval", action="store_true",
                    help="Score the policy through the shared eval harness (eval_harness.py): one "
                         "episode per env slot, so this arm sees bit-identical scenarios to every "
                         "baseline_drop.py arm run at the same --seed/--num_envs. Adds CEP50/90, "
                         "Wilson/bootstrap CIs, delivery time, feasible release window, JSON out.")
parser.add_argument("--unpaired", action="store_true",
                    help="--paired_eval variant that scores every episode instead of one per env "
                         "slot (larger n, but no cross-arm pairing).")
parser.add_argument("--arm_name", type=str, default=None,
                    help="Label for this arm in the JSON/report (e.g. T5_ours_full).")
parser.add_argument("--out-json", dest="out_json", type=str, default=None,
                    help="Write the per-episode records + summary here (--paired_eval).")
parser.add_argument("--handoff_heading_deg", type=float, default=None,
                    help="Override the handoff heading range to +/- this many degrees (0 = the fixed +X "
                         "cruise). Isolates world-frame heading generalization from the rest of the "
                         "handoff randomization.")
parser.add_argument("--release-terminal", action="store_true",
                    help="Evaluate under the Stage-B (exp_018) release-as-terminal semantics — "
                         "must match how the policy was trained, or proximity termination "
                         "truncates the loiter behavior being measured.")
# --- moving target (X marker) + Singer-KF tracker — must match training ---
parser.add_argument("--moving_target", action="store_true",
                    help="Force the moving target ON (must match how the policy was trained).")
parser.add_argument("--target_motion", type=str, default=None, choices=["gm", "cv", "ca", "ct"],
                    help="Target motion model: gm (Gauss-Markov, default) / cv / ca / ct.")
parser.add_argument("--target_speed", type=float, default=None,
                    help="Max initial target speed (m/s): |v0| ~ U[0, this].")
parser.add_argument("--target_accel", type=float, default=None,
                    help="CA model: max target acceleration (m/s^2).")
parser.add_argument("--target_omega_min", type=float, default=None,
                    help="CT model: min |turn rate| (rad/s).")
parser.add_argument("--target_omega_max", type=float, default=None,
                    help="CT model: max |turn rate| (rad/s).")
parser.add_argument("--target_kf", action="store_true",
                    help="Singer-KF tracker in the observation (obs 14 -> 21) — must match training "
                         "(a 21-dim policy cannot run on 14-dim obs and vice versa).")
parser.add_argument("--kf_tau", type=float, default=None,
                    help="Tracker: Gauss-Markov acceleration correlation time (s).")
parser.add_argument("--kf_sigma_a", type=float, default=None,
                    help="Tracker: steady-state maneuver-acceleration std (m/s^2).")
parser.add_argument("--out-csv", type=str, default="/workspace/logs/isaac_lab/step_response.csv")
parser.add_argument("--wandb", action="store_true",
                    help="Log the --policy eval summary + distribution figures (histograms, "
                         "termination-cause bar) to wandb as a job_type='eval' run.")
parser.add_argument("--wandb_project", type=str, default="drone-bombard-isaac")
parser.add_argument("--wandb_run_name", type=str, default=None,
                    help="wandb run name; default eval_<checkpoint-stem>.")
AppLauncher.add_app_launcher_args(parser)
args_cli = parser.parse_args()

app_launcher = AppLauncher(args_cli)
simulation_app = app_launcher.app

import csv
import os

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


def run_wind_test(env, wind_speed=5.0, steps=60):
    """Empirically verify that the wind actually pushes the AIRFRAME (v15).

    Commands hover (zero action) under a known forced wind and measures the
    steady-state tilt and downwind speed with cfg.wind_force_enabled toggled OFF
    then ON. With the force off the wind only ever displaced the payload's
    ballistic impact, so the airframe hovers level; with it on, holding station
    costs a tilt of ~atan(F_drag/(m*g)) — a falsifiable prediction.
    """
    import math as _m

    u = env.unwrapped
    n, device = u.num_envs, u.device
    action_dim = u.single_action_space.shape[0]
    zero = torch.zeros(n, action_dim, device=device)
    wind = torch.tensor([wind_speed, 0.0], device=device)
    print(f"[wind-test] forced wind = {wind_speed:.1f} m/s (+X), hover command, {steps} steps/condition")

    out = {}
    for enabled in (False, True):
        u.cfg.wind_force_enabled = enabled
        env.reset()
        for _ in range(steps):
            u._wind_xy[:] = wind  # hold the wind fixed across any mid-test reset
            env.step(zero)
        _, vel, _, roll, pitch, _ = u._kinematics()
        tilt_deg = _m.degrees(torch.sqrt(roll ** 2 + pitch ** 2).mean().item())
        speed = torch.linalg.norm(vel[:, :2], dim=-1).mean().item()
        out[enabled] = (tilt_deg, speed)
        print(f"[wind-test] wind_force {'ON ' if enabled else 'OFF'}: "
              f"tilt={tilt_deg:6.2f} deg | downwind speed={speed:5.3f} m/s")

    k, m = u.cfg.wind_drag_k, u._ctrl_mass
    pred = _m.degrees(_m.atan(k * wind_speed ** 2 / (m * 9.81)))
    print(f"[wind-test] analytic tilt for a station-holding drone: "
          f"atan({k}*{wind_speed}^2/({m:.2f}*9.81)) = {pred:.2f} deg")
    off_t, on_t = out[False][0], out[True][0]
    ok = on_t > off_t + 0.5
    print(f"[wind-test] {'PASS' if ok else 'FAIL'}: wind {'DOES' if ok else 'does NOT'} push the airframe "
          f"(tilt {off_t:.2f} -> {on_t:.2f} deg)")
    return ok


def run_drop_test(env, settle=8, air_steps=8):
    """Verify the v16 physical payload actually falls like a projectile.

    Cruise forward at altitude, force a physical release, then for a few airborne
    steps compare the payload's real position to the analytic drag-free ballistic
    trajectory from its release state. In nominal physics (no wind/drag) they must
    agree; a mismatch means the carry/release/gravity wiring is wrong. Stops well
    before landing so the env's auto-reset never clears the payload.
    """
    u = env.unwrapped
    if not getattr(u.cfg, "payload_physics_enabled", False):
        print("[drop-test] SKIP: env has no physical payload (payload_physics_enabled=False)")
        return False
    n, device = u.num_envs, u.device
    action_dim = u.single_action_space.shape[0]
    dt = u.cfg.sim.dt * u.cfg.decimation
    g = u.cfg.drop.gravity
    origins = u.scene.env_origins

    env.reset()
    act = torch.zeros(n, action_dim, device=device)
    act[:, 0] = 1.0  # cruise forward, hold altitude
    for _ in range(settle):
        env.step(act)

    pp0 = (u._payload.data.root_pos_w - origins).clone()       # payload release position
    v0 = u._robot.data.root_lin_vel_w.clone()                  # inherited drone velocity
    print(f"[drop-test] release: payload alt={pp0[:,2].mean():.2f} m, fwd speed={v0[:,0].mean():.2f} m/s, {n} payloads")

    u._payload_free[:] = True
    u._payload_attached[:] = False
    u._released[:] = True
    zero = torch.zeros(n, action_dim, device=device)

    max_zerr, max_xerr, last_z = 0.0, 0.0, pp0[:, 2].mean().item()
    for k in range(1, air_steps + 1):
        env.step(zero)
        pp = (u._payload.data.root_pos_w - origins)
        t = k * dt
        z_pred = pp0[:, 2] + v0[:, 2] * t - 0.5 * g * t * t
        x_pred = pp0[:, :2] + v0[:, :2] * t
        max_zerr = max(max_zerr, (pp[:, 2] - z_pred).abs().mean().item())
        max_xerr = max(max_xerr, torch.linalg.norm(pp[:, :2] - x_pred, dim=-1).mean().item())
        last_z = pp[:, 2].mean().item()

    print(f"[drop-test] airborne {air_steps} steps ({air_steps*dt:.1f}s) -> payload alt ~{last_z:.2f} m "
          f"(descending: {last_z < pp0[:,2].mean().item()})")
    print(f"[drop-test] real vs drag-free ballistic: max |dz|={max_zerr:.3f} m, max |dxy|={max_xerr:.3f} m")
    ok = (last_z < pp0[:, 2].mean().item() - 0.5) and max_zerr < 0.5 and max_xerr < 0.5
    print(f"[drop-test] {'PASS' if ok else 'FAIL'}: payload physically falls along the ballistic curve")
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


def _load_policy(env, policy_path):
    from rsl_rl.runners import OnPolicyRunner
    from drone_bombard.agents.rsl_rl_ppo_cfg import DroneBombardPPORunnerCfg

    agent_cfg = DroneBombardPPORunnerCfg()
    runner = OnPolicyRunner(env, agent_cfg.to_dict(), log_dir=None, device=agent_cfg.device)
    runner.load(policy_path)
    return runner.get_inference_policy(device=env.unwrapped.device)


def run_policy_paired(env, policy_path, episodes):
    """Learned arm (T4/T5) through the SHARED harness — same collector, same
    metric definitions, same JSON schema as the rule-based baselines, so Table 1
    rows are produced by one code path. Scores each env slot's FIRST episode,
    which is the only reset that is policy-independent and therefore the only
    way two arms see bit-identical scenarios."""
    import eval_harness

    policy = _load_policy(env, policy_path)
    eval_harness.run_and_report(
        env, lambda obs, u: policy(obs),
        name=args_cli.arm_name or f"policy:{os.path.basename(policy_path)}",
        episodes=episodes,
        paired=not args_cli.unpaired,
        meta={"task": args_cli.task, "seed": args_cli.seed, "learned": True,
              "privileged": False, "policy": policy_path},
        out_json=args_cli.out_json,
    )


def run_policy(env, policy_path, episodes=10):
    policy = _load_policy(env, policy_path)

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
            action = policy(obs)  # noqa: F821 (bound above)
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

        if args_cli.wandb:
            # The final-episode snapshot carries no separate payload channel:
            # in the land-terminal envs (v16/v19) release_impact_err IS the
            # real payload landing error (latched on landing), and in the
            # analytic envs the drop resolves at release — either way the
            # payload-impact population is the released episodes.
            impacted = released
            cat = dict(cat, payload_impact_err=cat["release_impact_err"])
            _log_eval_to_wandb(policy_path, n_done, cause_counts, causes, cat,
                               released, impacted, near_miss_wsum / max(log_w, 1))


def _log_eval_to_wandb(policy_path, n_done, cause_counts, causes, cat,
                       released, impacted, near_miss_rate):
    """Push the deterministic-eval verdict to wandb as figures: summary
    scalars, per-episode distribution histograms, and a termination-cause
    bar chart. Lives in the same project as training runs (job_type='eval')
    so training curves and eval verdicts sit side by side."""
    import wandb

    stem = os.path.basename(policy_path).rsplit(".", 1)[0]
    run = wandb.init(
        project=args_cli.wandb_project,
        name=args_cli.wandb_run_name or f"eval_{stem}",
        job_type="eval",
        config={
            "policy": policy_path,
            "episodes": n_done,
            "num_envs": args_cli.num_envs,
            "release_terminal": args_cli.release_terminal,
        },
    )

    def _q(t, q):
        return torch.quantile(t, q).item() if t.numel() else float("nan")

    d_xy = cat["d_xy_min"][torch.isfinite(cat["d_xy_min"])]
    aim = cat["aim_err_min"][torch.isfinite(cat["aim_err_min"])]
    rel_err = cat["release_impact_err"][released]
    pay_err = cat["payload_impact_err"][impacted]
    summary = {
        "eval/success_rate": cause_counts["success"] / max(n_done, 1),
        "eval/release_rate": released.float().mean().item(),
        "eval/payload_impact_rate": impacted.float().mean().item(),
        "eval/timeout_near_miss_rate": near_miss_rate,
        "eval/d_xy_min_med_m": _q(d_xy, 0.5),
        "eval/aim_err_min_med_m": _q(aim, 0.5),
        "eval/drop_impact_error_mean_m": rel_err.mean().item() if rel_err.numel() else float("nan"),
        "eval/drop_impact_error_p90_m": _q(rel_err, 0.9),
        "eval/payload_impact_err_mean_m": pay_err.mean().item() if pay_err.numel() else float("nan"),
        "eval/final_speed_xy_med_ms": _q(cat["final_speed_xy"], 0.5),
    }
    hists = {
        "eval/hist_d_xy_min_m": d_xy,
        "eval/hist_aim_err_min_m": aim,
        "eval/hist_drop_impact_error_m": rel_err,
        "eval/hist_payload_impact_err_m": pay_err,
        "eval/hist_final_speed_xy_ms": cat["final_speed_xy"],
    }
    payload = dict(summary)
    payload.update({k: wandb.Histogram(v.numpy()) for k, v in hists.items() if v.numel()})
    cause_table = wandb.Table(
        data=[[c, cause_counts[c]] for c in causes], columns=["cause", "count"])
    payload["eval/termination_causes"] = wandb.plot.bar(
        cause_table, "cause", "count", title="Termination causes")
    wandb.log(payload)
    run.summary.update(summary)
    wandb.finish()
    print(f"[policy] wandb eval figures logged: project={args_cli.wandb_project} run={run.name}")


def main():
    env_cfg = parse_env_cfg(args_cli.task, num_envs=args_cli.num_envs)
    env_cfg.seed = args_cli.seed
    # Chase camera locked on the drone so --policy playback is actually visible
    # in the livestream/GUI (default world camera stares at the ground grid and
    # the drone flies off-frame at ~10 m altitude). Pulled in from (-8,-8,4) —
    # that framed the drone too small; this sits just behind/above it.
    env_cfg.viewer.origin_type = "asset_root"
    # Track the PAYLOAD (scene rigid object) instead of the drone: while carried
    # it rides under the drone (so we see the drone), and after release the camera
    # FOLLOWS the payload down as it falls + lands — the separation reads clearly.
    env_cfg.viewer.asset_name = "payload"
    env_cfg.viewer.env_index = 0
    env_cfg.viewer.eye = (-2.5, -2.0, 1.1)
    env_cfg.viewer.lookat = (0.0, 0.0, 0.0)
    if args_cli.show:
        # visual aids for watching: the target beacon/disc + payload marker.
        env_cfg.show_markers = True
    if args_cli.release_terminal:
        env_cfg.release_terminal = True
    # moving target (X marker) + Singer-KF tracker — must mirror train.py
    if args_cli.moving_target:
        env_cfg.moving_target_force = True
    if args_cli.target_motion is not None:
        env_cfg.phase_cfg.target_motion_model = args_cli.target_motion
    if args_cli.target_speed is not None:
        env_cfg.phase_cfg.target_init_speed = args_cli.target_speed
    if args_cli.target_accel is not None:
        env_cfg.phase_cfg.target_accel_max = args_cli.target_accel
    if args_cli.target_omega_min is not None or args_cli.target_omega_max is not None:
        lo, hi = env_cfg.phase_cfg.target_omega_range
        env_cfg.phase_cfg.target_omega_range = (
            args_cli.target_omega_min if args_cli.target_omega_min is not None else lo,
            args_cli.target_omega_max if args_cli.target_omega_max is not None else hi,
        )
    if args_cli.target_kf:
        env_cfg.target_kf_obs = True
    if args_cli.kf_tau is not None:
        env_cfg.tracker.tau = args_cli.kf_tau
    if args_cli.kf_sigma_a is not None:
        env_cfg.tracker.sigma_a = args_cli.kf_sigma_a
    # train/test distribution mismatch knobs (P0): turn OFF what the task turns on.
    if args_cli.no_handoff_dr and hasattr(env_cfg, "handoff"):
        env_cfg.handoff.randomize = False
    if args_cli.handoff_heading_deg is not None and hasattr(env_cfg, "handoff"):
        h = abs(args_cli.handoff_heading_deg)
        env_cfg.handoff.heading_range_deg = (-h, h)
    if args_cli.no_dyn_dr:
        env_cfg.dyn_dr.enabled = False
    if args_cli.dr_scale is not None and hasattr(env_cfg, "model_err"):
        env_cfg.model_err.scale = args_cli.dr_scale
    if hasattr(env_cfg, "model_err") and args_cli.observe_wind:
        env_cfg.model_err.observe_wind = True
    if hasattr(env_cfg, "release") and args_cli.release_10hz:
        env_cfg.release.decide_at_physics_rate = False
    if hasattr(env_cfg, "perception") and args_cli.pixel_vision:
        env_cfg.perception.pixel_quantize = True
    # The observation width depends on observe_wind, and the env allocates its
    # per-episode observation-bias buffer from cfg.observation_space -- re-derive
    # after the edits above or a checkpoint will load against the wrong width.
    if hasattr(env_cfg, "model_err"):
        env_cfg.__post_init__()
    env = gym.make(args_cli.task, cfg=env_cfg)
    env = RslRlVecEnvWrapper(env)

    if args_cli.zero_actions:
        run_zero_actions(env)
    elif args_cli.scripted:
        run_scripted(env, episodes=args_cli.episodes)
    elif args_cli.wind_test:
        run_wind_test(env, wind_speed=args_cli.wind_speed)
    elif args_cli.drop_test:
        run_drop_test(env)
    elif args_cli.step_response:
        run_step_response(env, args_cli.out_csv)
    elif args_cli.policy:
        if args_cli.paired_eval or args_cli.unpaired or args_cli.out_json:
            run_policy_paired(env, args_cli.policy, episodes=args_cli.episodes)
        else:
            run_policy(env, args_cli.policy, episodes=args_cli.episodes)
    else:
        print("Specify one of --zero-actions / --scripted / --step-response / --wind-test / --policy CKPT")

    env.close()


if __name__ == "__main__":
    main()
    simulation_app.close()
