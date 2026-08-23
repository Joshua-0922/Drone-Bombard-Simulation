"""Ad-hoc physical-payload attach/detach validation (hover-drop scenario).

Checks, in order:
  1. ATTACH:  while attached, the payload tracks the drone at the mount
              offset (kinematic weld) — reports max follow error over 1 s.
  2. DETACH:  after force-arming the release countdown, _payload_attached
              flips False exactly release_delay later.
  3. FALL:    the payload free-falls and _payload_impacted latches.
  4. PARITY:  measured impact error (latched at ground crossing) matches the
              analytic ballistic prediction taken at arming time (Phase 1 is
              drag/wind-free, so they must agree to ~cm for a hover drop).

Run inside the isaac-lab container:
  ./isaaclab.sh -p /workspace/drone-bombard/isaac_lab/_test_payload_drop.py --headless
"""

import argparse

from isaaclab.app import AppLauncher

parser = argparse.ArgumentParser()
AppLauncher.add_app_launcher_args(parser)
args_cli = parser.parse_args()
app_launcher = AppLauncher(args_cli)
simulation_app = app_launcher.app

import torch  # noqa: E402
import gymnasium as gym  # noqa: E402

import drone_bombard  # noqa: F401,E402
from drone_bombard.drone_bombard_env import DroneBombardEnvCfg  # noqa: E402
from drone_bombard.math_utils import predict_impact_nominal  # noqa: E402


def main():
    cfg = DroneBombardEnvCfg()
    cfg.scene.num_envs = 8
    cfg.sim.device = "cuda:0"
    env = gym.make("Isaac-DroneBombard-Direct-v0", cfg=cfg).unwrapped
    env.reset()
    N = env.num_envs
    zero = torch.zeros(N, 6, device=env.device)

    # --- 1. attached follow check over 1 s of hover -----------------------
    max_follow_err = 0.0
    for _ in range(10):
        env.step(zero)
        dpos = env._robot.data.root_pos_w
        ppos = env._payload_obj.data.root_pos_w
        expected = dpos.clone()
        expected[:, 2] += env.cfg.drop.payload_mount_offset_z
        max_follow_err = max(max_follow_err, (ppos - expected).norm(dim=-1).max().item())
    attached_ok = bool(env._payload_attached.all())
    print(f"[test] 1 ATTACH: follow_err_max={max_follow_err:.4f} m, "
          f"all_attached={attached_ok} -> {'PASS' if attached_ok and max_follow_err < 0.05 else 'FAIL'}",
          flush=True)

    # --- analytic prediction at arming time -------------------------------
    pos = env._robot.data.root_pos_w - env.scene.env_origins
    vel = env._robot.data.root_lin_vel_w
    pred = predict_impact_nominal(
        pos[:, :2], vel[:, :2], vel[:, 2], pos[:, 2],
        env.cfg.drop.release_delay, env.cfg.drop.gravity
    )
    analytic_err = torch.linalg.norm(pred - env._target_xy, dim=-1)

    # --- 2./3. force-release every env, let physics run -------------------
    env._released[:] = True  # keep the referee from re-firing
    env._detach_countdown[:] = env._detach_delay_steps

    detach_step = -1
    for i in range(40):  # 4 s; fall from ~10 m takes ~1.43 s
        env.step(zero)
        if detach_step < 0 and not bool(env._payload_attached.any()):
            detach_step = i
        if bool(env._payload_impacted.all()):
            break

    n_att = int(env._payload_attached.sum())
    n_imp = int(env._payload_impacted.sum())
    print(f"[test] 2 DETACH: still_attached={n_att}/{N} (want 0), "
          f"detached_by_policy_step={detach_step} -> {'PASS' if n_att == 0 else 'FAIL'}", flush=True)
    print(f"[test] 3 FALL/IMPACT: impacted={n_imp}/{N} (want {N}) -> "
          f"{'PASS' if n_imp == N else 'FAIL'}", flush=True)

    # --- 4. measured vs analytic ------------------------------------------
    gap = (env._payload_impact_err - analytic_err).abs()
    print(f"[test] 4 PARITY: |measured - analytic| mean={gap.mean():.3f} "
          f"max={gap.max():.3f} m -> {'PASS' if gap.max().item() < 0.15 else 'FAIL'}", flush=True)
    print(f"[test]   measured_err={env._payload_impact_err.cpu().numpy().round(3)}", flush=True)
    print(f"[test]   analytic_err={analytic_err.cpu().numpy().round(3)}", flush=True)
    print("[test] DONE", flush=True)
    env.close()


if __name__ == "__main__":
    main()
    simulation_app.close()
