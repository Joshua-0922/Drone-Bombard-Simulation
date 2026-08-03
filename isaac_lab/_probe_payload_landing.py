"""Probe: does the physical payload's landing latch actually fire?

Motivation: paired evaluation showed episodes with released=True, landed=False
that then ran to the 30 s timeout (v19-on-v19 too: release 100% but 17/200
timeouts). The landing test is ``z_local <= cfg.payload_ground_z`` with
``payload_ground_z = 0.0``, while the payload is a cylinder of height
``payload_phys_height`` — a payload RESTING on the ground plane sits at
half-height, i.e. strictly above 0, so the latch can only fire if a physics
sample happens to catch it mid-penetration.

This forces a release on every env and reports the minimum z each payload ever
reaches plus whether ``_payload_landed`` ever latched.

Usage (inside isaac-verify):
    /workspace/isaaclab/isaaclab.sh -p _probe_payload_landing.py
"""

import argparse
import sys

from isaaclab.app import AppLauncher

parser = argparse.ArgumentParser()
parser.add_argument("--envs", type=int, default=32)
parser.add_argument("--steps", type=int, default=60)
AppLauncher.add_app_launcher_args(parser)
args_cli = parser.parse_args()
args_cli.headless = True

app_launcher = AppLauncher(args_cli)
simulation_app = app_launcher.app

import gymnasium as gym  # noqa: E402
import torch  # noqa: E402

import drone_bombard  # noqa: F401,E402
from drone_bombard.v11_env import DroneBombardV19Cfg  # noqa: E402


def main():
    cfg = DroneBombardV19Cfg()
    cfg.scene.num_envs = args_cli.envs
    cfg.sim.device = "cuda:0"
    cfg.seed = 42
    env = gym.make("Isaac-DroneBombard-V19-Direct-v0", cfg=cfg)
    u = env.unwrapped
    env.reset()

    half_h = 0.5 * u.cfg.payload_phys_height
    print(f"[probe] payload height={u.cfg.payload_phys_height} half={half_h} "
          f"ground_z={u.cfg.payload_ground_z}", flush=True)

    # settle a moment, then force the release on every env
    zero = torch.zeros(u.num_envs, u.cfg.action_space, device=u.device)
    for _ in range(5):
        env.step(zero)
    u._released[:] = True
    u._payload_attached[:] = False
    u._payload_free[:] = True

    min_z = torch.full((u.num_envs,), 1e9, device=u.device)
    latched = torch.zeros(u.num_envs, dtype=torch.bool)
    for _ in range(args_cli.steps):
        env.step(zero)
        z = u._payload.data.root_pos_w[:, 2] - u.scene.env_origins[:, 2]
        min_z = torch.minimum(min_z, z)
        # A latched landing TERMINATES the episode, and _reset_idx clears
        # _payload_landed inside the same step() — so a post-step read of the
        # live buffer always misses it. The pre-reset snapshot is the only
        # honest source (same aliasing trap as Rule 23d).
        snap = getattr(u, "_last_final_snapshot", None)
        if snap is not None:
            ids = snap["env_ids"].detach().cpu().long()
            latched[ids] |= snap["landed"].detach().cpu()

    n = u.num_envs
    latched = int(latched.sum())
    rest_z = (u._payload.data.root_pos_w[:, 2] - u.scene.env_origins[:, 2])
    print(f"[probe] landed latched : {latched}/{n}")
    print(f"[probe] min z reached  : min={float(min_z.min()):.4f} "
          f"med={float(min_z.median()):.4f} max={float(min_z.max()):.4f}")
    print(f"[probe] resting z      : min={float(rest_z.min()):.4f} "
          f"med={float(rest_z.median()):.4f} max={float(rest_z.max()):.4f}")
    ok = latched == n
    print(f"=== {'PASS — every payload latched' if ok else 'FAIL — latch misses payloads at rest'} ===",
          flush=True)
    return 0 if ok else 1


if __name__ == "__main__":
    rc = main()
    simulation_app.close()
    sys.exit(rc)
