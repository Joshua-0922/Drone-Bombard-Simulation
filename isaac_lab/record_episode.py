"""Record a one-episode video of Isaac-DroneBombard: drone + payload + target.

Renders the RTX viewport to an mp4 (needs a working RTX renderer, i.e. driver
>= 580). Shows a single drone carrying a payload cylinder marker flying toward
the target X marker.

Recommended: pass a trained --policy — the RL policy learned smooth flight.
The built-in --scripted fallback drives a constant velocity, which the
(uncalibrated) cascaded controller handles poorly during aggressive maneuvers,
so it is mainly for plumbing checks.

Frames are captured via env.render() each policy step and written with imageio
(decoupled from gym RecordVideo so it composes with the rsl_rl vec wrapper).

Usage (isaac-lab container, driver >= 580):
    ./isaaclab.sh -p record_episode.py --headless --enable_cameras \\
        --policy /workspace/logs/.../model_final.pt --video_length 250
"""

import argparse
import os

from isaaclab.app import AppLauncher

parser = argparse.ArgumentParser(description="Record a one-episode drone-bombard video.")
parser.add_argument("--task", type=str, default="Isaac-DroneBombard-Direct-v0")
parser.add_argument("--policy", type=str, default=None, help="Trained checkpoint (.pt). Recommended.")
parser.add_argument("--scripted", action="store_true", help="Fallback: constant-velocity approach (crude).")
parser.add_argument("--approach_speed", type=float, default=0.1)
parser.add_argument("--video_length", type=int, default=250)
parser.add_argument("--fps", type=int, default=10)
parser.add_argument("--out", type=str, default="/tmp/isaac_rec")
parser.add_argument("--seed", type=int, default=0)
parser.add_argument("--spawn_dist", type=float, default=0.0,
                    help="override spawn distance from target (m); 0 = env default 3-7 m.")
parser.add_argument("--hold", action="store_true",
                    help="disable success/stagnation termination so the drone reaches the target and hovers "
                         "over it for the full video (nicer demo).")
AppLauncher.add_app_launcher_args(parser)
args_cli = parser.parse_args()
args_cli.enable_cameras = True

app_launcher = AppLauncher(args_cli)
simulation_app = app_launcher.app

import numpy as np
import torch
import imageio
import gymnasium as gym

from isaaclab_rl.rsl_rl import RslRlVecEnvWrapper

import drone_bombard  # noqa: F401
from drone_bombard.drone_bombard_env import DroneBombardEnvCfg


def main():
    os.makedirs(args_cli.out, exist_ok=True)

    env_cfg = DroneBombardEnvCfg()
    env_cfg.scene.num_envs = 1
    env_cfg.sim.device = args_cli.device if args_cli.device is not None else "cuda:0"
    env_cfg.seed = args_cli.seed
    env_cfg.show_markers = True  # payload cylinder under the drone + target X plate
    if args_cli.spawn_dist > 0:
        env_cfg.reset.handoff_dist_range = (args_cli.spawn_dist, args_cli.spawn_dist)
    if args_cli.hold:
        env_cfg.reward.success_radius = 0.0        # never terminate on "success"
        env_cfg.termination.stagnation_window = 10 ** 9  # effectively disable stagnation
    # follow-camera: the Crazyflie mesh is ~10cm (we only override mass/inertia,
    # not visual scale), invisible at a wide fixed-world framing — track the
    # drone instead so it's always in frame regardless of randomized spawn/target.
    env_cfg.viewer.origin_type = "asset_root"
    env_cfg.viewer.asset_name = "robot"
    env_cfg.viewer.eye = (2.2, -2.2, 1.1)     # offset from the drone (close — Crazyflie mesh is ~10cm)
    env_cfg.viewer.lookat = (0.0, 0.0, -0.3)  # look slightly down/ahead of the drone
    env_cfg.viewer.resolution = (1280, 720)

    env = gym.make(args_cli.task, cfg=env_cfg, render_mode="rgb_array")
    base = env.unwrapped

    from drone_bombard.agents.rsl_rl_ppo_cfg import DroneBombardPPORunnerCfg
    agent_cfg = DroneBombardPPORunnerCfg()
    wrapped = RslRlVecEnvWrapper(env, clip_actions=agent_cfg.clip_actions)
    device = base.device

    policy = None
    if args_cli.policy:
        from rsl_rl.runners import OnPolicyRunner
        runner = OnPolicyRunner(wrapped, agent_cfg.to_dict(), log_dir=None, device=device)
        runner.load(args_cli.policy)
        policy = runner.get_inference_policy(device=device)
        print(f"[record] using trained policy: {args_cli.policy}", flush=True)
    else:
        print("[record] scripted constant-velocity approach (no policy)", flush=True)

    obs = wrapped.get_observations()  # rsl_rl vec wrapper returns the obs (TensorDict) directly
    frames = []
    for step in range(args_cli.video_length):
        if policy is not None:
            with torch.inference_mode():
                action = policy(obs)
        else:
            # distance-proportional approach: velocity ~ k*distance, capped and
            # decaying to zero at the target so the drone decelerates and settles
            # over it instead of overshooting (a constant-velocity command has no
            # deceleration term and flies past).
            pos = base._robot.data.root_pos_w - base.scene.env_origins
            to_t = base._target_xy - pos[:, :2]
            dist = torch.linalg.norm(to_t, dim=-1, keepdim=True)
            dir_xy = to_t / dist.clamp(min=1e-6)
            vx_scale = base.cfg.action.vx_scale
            speed = torch.clamp(0.5 * dist, max=1.2)  # m/s, tapers to 0 at target
            act_mag = (speed / vx_scale) * args_cli.approach_speed / 0.12  # scale by CLI knob
            action = torch.zeros(base.num_envs, 4, device=device)
            action[:, 0] = (dir_xy[:, 0:1] * act_mag).squeeze(-1)
            action[:, 1] = (dir_xy[:, 1:2] * act_mag).squeeze(-1)
            # active altitude hold (the velocity controller has no integral term,
            # so command vz toward the spawn altitude to stop the slow sink).
            alt_err = 10.0 - (base._robot.data.root_pos_w[:, 2] - base.scene.env_origins[:, 2])
            action[:, 2] = torch.clamp(0.6 * alt_err / base.cfg.action.vz_scale, -0.5, 0.5)

        obs, _, dones, _ = wrapped.step(action)
        frame = base.render()
        if frame is not None:
            frames.append(np.asarray(frame))
        if step % 30 == 0:
            d_xy = base._current_d_xy().mean().item()
            print(f"[record] step {step:3d} d_xy={d_xy:.2f} frames={len(frames)}", flush=True)
        if bool(dones.any()):
            causes = [k for k, v in base._done_flags.items() if bool(v.any())]
            print(f"[record] episode ended at step {step}: {causes}", flush=True)
            break

    # --- drop animation: analytic free-fall from the release point to the
    # ground (same t=sqrt(2H/g) ballistic model _predicted_impact_from_snapshot
    # uses for CCIP scoring), purely visual — the drone/physics are not
    # stepped further, only the payload marker is moved and re-rendered.
    release_pos = (base._robot.data.root_pos_w[0] - base.scene.env_origins[0]).clone()
    release_vel = base._robot.data.root_lin_vel_w[0, :2].clone()
    t_fall = float(torch.sqrt(torch.clamp(2.0 * release_pos[2] / 9.81, min=0.0)))
    n_drop_frames = max(int(t_fall * args_cli.fps), 1)
    base._payload_attached[:] = False
    print(f"[record] payload release at alt={release_pos[2].item():.2f}m, "
          f"t_fall={t_fall:.2f}s -> {n_drop_frames} drop frames", flush=True)
    # switch the follow-camera off asset_root (it would keep re-snapping to the
    # now-frozen drone every tick, fighting our manual updates below) so it can
    # track the falling payload instead.
    cam_ctrl = base.viewport_camera_controller
    cam_ctrl.cfg.origin_type = "world"
    cam_ctrl.viewer_origin = torch.zeros(3, device=device)
    cam_offset = torch.tensor([1.8, -1.8, 0.9], device=device)
    for i in range(n_drop_frames + 3):  # a few extra frames resting on the ground
        t = min(i / args_cli.fps, t_fall)
        drop_xy = release_pos[:2] + release_vel * t
        drop_z = max(release_pos[2].item() - 0.5 * 9.81 * t * t, 0.05)
        payload_pos = torch.zeros(1, 3, device=device)
        payload_pos[0, :2] = drop_xy + base.scene.env_origins[0, :2]
        payload_pos[0, 2] = drop_z
        base._payload_marker.visualize(translations=payload_pos)
        cam_ctrl.update_view_location(
            eye=(payload_pos[0] + cam_offset).tolist(), lookat=payload_pos[0].tolist()
        )
        # env.render() alone reads a stale camera pose here — outside of env.step()'s
        # own render-product scheduling, the Replicator annotator only picks up a
        # sim.set_camera_view() move after an explicit app tick.
        simulation_app.update()
        frame = base.render(recompute=True)
        if frame is not None:
            frames.append(np.asarray(frame))

    out_path = os.path.join(args_cli.out, "drone_bombard_episode.mp4")
    if frames:
        imageio.mimsave(out_path, frames, fps=args_cli.fps, macro_block_size=None)
        print(f"[record] wrote {len(frames)} frames -> {out_path}", flush=True)
    else:
        print("[record] NO FRAMES captured (render returned None)", flush=True)

    env.close()


if __name__ == "__main__":
    main()
    simulation_app.close()
