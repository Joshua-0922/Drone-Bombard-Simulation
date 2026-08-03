"""Probe: P0 handoff randomization + dynamics/sensing DR (v19 vs v20).

Gate checks before any v20 training run:
  1. v19 is UNCHANGED — spawn is still the single fixed handoff (origin, 10 m,
     4 m/s along +X, level, zero rates) and every dyn_dr buffer is identity.
  2. v20 actually randomizes — heading / speed / altitude / entry offset /
     attitude / body rates all spread, and the mass-belief, gain and payload
     ballistic-coefficient scales sit inside their configured bands.
  3. The geometry invariant survives randomization: the marker still sits
     marker_dist ahead along EACH env's own sampled heading (within the
     marker disk + entry-offset budget).
  4. obs stays 28-D with no NaNs, and the observation actually carries the
     injected noise (a zero-action rollout on v20 must differ from v19).

ONE VARIANT PER PROCESS: building a second Isaac Sim scene after closing the
first one stalls, so the v19 run persists its zero-action rollout trace to
``--trace`` and the v20 run loads it back for check 4.

Usage (inside isaac-verify, in this order):
    /workspace/isaaclab/isaaclab.sh -p _probe_p0_dr.py --variant v19
    /workspace/isaaclab/isaaclab.sh -p _probe_p0_dr.py --variant v20
"""

import argparse
import math
import os
import sys

from isaaclab.app import AppLauncher

parser = argparse.ArgumentParser()
parser.add_argument("--variant", type=str, default="v19", choices=["v19", "v20"])
parser.add_argument("--trace", type=str, default="/tmp/probe_p0_v19_trace.pt")
parser.add_argument("--envs", type=int, default=64)
parser.add_argument("--steps", type=int, default=20)
AppLauncher.add_app_launcher_args(parser)
args_cli = parser.parse_args()
args_cli.headless = True

app_launcher = AppLauncher(args_cli)
simulation_app = app_launcher.app

import gymnasium as gym  # noqa: E402
import torch  # noqa: E402

import drone_bombard  # noqa: F401,E402
from drone_bombard.v11_env import DroneBombardV19Cfg, DroneBombardV20Cfg  # noqa: E402

FAILS = []


def check(name, ok, detail=""):
    print(f"  [{'PASS' if ok else 'FAIL'}] {name}{(' — ' + detail) if detail else ''}", flush=True)
    if not ok:
        FAILS.append(name)


def build(cfg, task):
    cfg.scene.num_envs = args_cli.envs
    cfg.sim.device = "cuda:0"
    cfg.seed = 42
    env = gym.make(task, cfg=cfg)
    obs, _ = env.reset()
    return env, (obs["policy"] if isinstance(obs, dict) else obs)


def spawn_state(u):
    """Post-reset root state in env-local frame: pos, vel, roll/pitch/yaw, rates."""
    from isaaclab.utils.math import euler_xyz_from_quat

    pos = u._robot.data.root_pos_w - u.scene.env_origins
    vel = u._robot.data.root_lin_vel_w
    ang = u._robot.data.root_ang_vel_b
    roll, pitch, yaw = euler_xyz_from_quat(u._robot.data.root_quat_w)
    wrap = lambda a: torch.atan2(torch.sin(a), torch.cos(a))  # noqa: E731
    return pos, vel, wrap(roll), wrap(pitch), wrap(yaw), ang


def zero_rollout(env, u):
    zero = torch.zeros(u.num_envs, u.cfg.action_space, device=u.device)
    for _ in range(args_cli.steps):
        o, *_ = env.step(zero)
    return (o["policy"] if isinstance(o, dict) else o).clone()


def probe_v19():
    print("[v19] fixed handoff + no dynamics DR", flush=True)
    env19, obs19 = build(DroneBombardV19Cfg(), "Isaac-DroneBombard-V19-Direct-v0")
    u19 = env19.unwrapped
    pos, vel, roll, pitch, yaw, ang = spawn_state(u19)
    speed_xy = torch.linalg.norm(vel[:, :2], dim=-1)

    check("v19 spawn xy == env origin", float(pos[:, :2].abs().max()) < 1e-5,
          f"max |xy| = {float(pos[:, :2].abs().max()):.2e}")
    check("v19 spawn altitude == 10.0 m", float((pos[:, 2] - 10.0).abs().max()) < 1e-5,
          f"alt range [{float(pos[:, 2].min()):.4f}, {float(pos[:, 2].max()):.4f}]")
    check("v19 cruise speed == 4.0 m/s", float((speed_xy - 4.0).abs().max()) < 1e-4,
          f"speed range [{float(speed_xy.min()):.4f}, {float(speed_xy.max()):.4f}]")
    check("v19 heading == +X (yaw 0)", float(yaw.abs().max()) < 1e-5)
    check("v19 attitude level", float(roll.abs().max()) < 1e-5 and float(pitch.abs().max()) < 1e-5)
    check("v19 body rates zero", float(ang.abs().max()) < 1e-5)
    check("v19 ctrl mass belief == nominal",
          float((u19._ctrl_mass - u19._ctrl_mass_nominal).abs().max()) < 1e-6)
    check("v19 gain scales == 1", float((u19._kp_vel_scale - 1.0).abs().max()) < 1e-6
          and float((u19._k_att_rate_scale - 1.0).abs().max()) < 1e-6)
    check("v19 payload bc scale == 1", float((u19._payload_bc_scale - 1.0).abs().max()) < 1e-6)
    check("v19 obs/act bias == 0",
          float(u19._obs_bias.abs().max()) < 1e-12 and float(u19._act_bias.abs().max()) < 1e-12)
    check("v19 obs is 28-D, finite", obs19.shape[-1] == 28 and bool(torch.isfinite(obs19).all()),
          f"shape {tuple(obs19.shape)}")

    # zero-action rollout: the reference trace the v20 run compares against
    o19 = zero_rollout(env19, u19)
    check("v19 rollout finite", bool(torch.isfinite(o19).all()))
    torch.save(o19.cpu(), args_cli.trace)
    print(f"  [info] wrote reference trace -> {args_cli.trace}", flush=True)


def probe_v20():
    print("[v20] randomized handoff + dynamics/sensing DR", flush=True)
    cfg20 = DroneBombardV20Cfg()
    hc, dd = cfg20.handoff, cfg20.dyn_dr
    env20, obs20 = build(cfg20, "Isaac-DroneBombard-V20-Direct-v0")
    u20 = env20.unwrapped
    pos, vel, roll, pitch, yaw, ang = spawn_state(u20)
    speed_xy = torch.linalg.norm(vel[:, :2], dim=-1)

    check("v20 heading spread over the full circle",
          float(yaw.std()) > 1.0 and float(yaw.max()) > 2.0 and float(yaw.min()) < -2.0,
          f"yaw std {float(yaw.std()):.2f} rad, range [{float(yaw.min()):.2f}, {float(yaw.max()):.2f}]")
    check("v20 speed inside speed_range",
          float(speed_xy.min()) >= hc.speed_range[0] - 1.0 and float(speed_xy.max()) <= hc.speed_range[1] + 1.0
          and float(speed_xy.std()) > 0.3,
          f"[{float(speed_xy.min()):.2f}, {float(speed_xy.max()):.2f}] std {float(speed_xy.std()):.2f}")
    check("v20 altitude inside alt_range",
          float(pos[:, 2].min()) >= hc.alt_range[0] - 1e-3 and float(pos[:, 2].max()) <= hc.alt_range[1] + 1e-3
          and float(pos[:, 2].std()) > 0.3,
          f"[{float(pos[:, 2].min()):.2f}, {float(pos[:, 2].max()):.2f}]")
    entry = torch.linalg.norm(pos[:, :2], dim=-1)
    budget = math.hypot(hc.lateral_offset_max, hc.along_offset_max) + 1e-3
    check("v20 entry offset inside budget", float(entry.max()) <= budget and float(entry.std()) > 0.3,
          f"max {float(entry.max()):.2f} m (budget {budget:.2f})")
    check("v20 attitude perturbed", float(roll.std()) > 0.01 and float(pitch.std()) > 0.01,
          f"roll std {math.degrees(float(roll.std())):.2f} deg")
    check("v20 body rates perturbed", float(ang.std()) > 0.05,
          f"rate std {float(ang.std()):.3f} rad/s")

    rel = u20._target_xy - pos[:, :2]
    bearing_err = torch.atan2(rel[:, 1], rel[:, 0]) - yaw
    bearing_err = torch.atan2(torch.sin(bearing_err), torch.cos(bearing_err))
    check("v20 marker is ahead along EACH env's own heading",
          float(bearing_err.abs().max()) < math.radians(35.0),
          f"max bearing error {math.degrees(float(bearing_err.abs().max())):.1f} deg")
    rng = torch.linalg.norm(rel, dim=-1)
    check("v20 approach range near marker_dist",
          float((rng - cfg20.marker_dist).abs().max()) < cfg20.marker_spawn_radius + budget + 1e-3,
          f"range [{float(rng.min()):.2f}, {float(rng.max()):.2f}] m")

    mass_rel = (u20._ctrl_mass / u20._ctrl_mass_nominal - 1.0).abs().max()
    check("v20 mass belief inside +-ctrl_mass_rel",
          float(mass_rel) <= dd.ctrl_mass_rel + 1e-6 and float(u20._ctrl_mass.std()) > 0,
          f"max |rel| {float(mass_rel):.4f} (cfg {dd.ctrl_mass_rel})")
    check("v20 gain scales inside band",
          float((u20._kp_vel_scale - 1.0).abs().max()) <= dd.kp_vel_rel + 1e-6
          and float((u20._k_att_rate_scale - 1.0).abs().max()) <= dd.k_att_rate_rel + 1e-6
          and float(u20._kp_vel_scale.std()) > 0)
    check("v20 payload bc inside band",
          float((u20._payload_bc_scale - 1.0).abs().max()) <= dd.payload_bc_rel + 1e-6
          and float(u20._payload_bc_scale.std()) > 0)
    check("v20 obs bias non-zero and per-env",
          float(u20._obs_bias.abs().max()) > 0 and not torch.allclose(u20._obs_bias[0], u20._obs_bias[1]))
    check("v20 obs is 28-D, finite", obs20.shape[-1] == 28 and bool(torch.isfinite(obs20).all()),
          f"shape {tuple(obs20.shape)}")

    o20 = zero_rollout(env20, u20)
    check("v20 rollout finite", bool(torch.isfinite(o20).all()))
    if os.path.exists(args_cli.trace):
        o19 = torch.load(args_cli.trace).to(o20.device)
        check("v20 rollout differs from v19 (DR is actually reaching the policy)",
              float((o20 - o19).abs().max()) > 1e-3,
              f"max |delta obs| {float((o20 - o19).abs().max()):.4f}")
    else:
        print(f"  [skip] no v19 trace at {args_cli.trace} — run --variant v19 first", flush=True)


def main():
    (probe_v19 if args_cli.variant == "v19" else probe_v20)()
    print(f"\n=== [{args_cli.variant}] "
          f"{'ALL PASS' if not FAILS else 'FAILURES: ' + ', '.join(FAILS)} ===", flush=True)
    return 1 if FAILS else 0


if __name__ == "__main__":
    rc = main()
    simulation_app.close()
    sys.exit(rc)
