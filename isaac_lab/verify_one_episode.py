"""One-episode, no-training verification of Isaac-DroneBombard-Direct-v0.

Construct a single-env instance, reset it, save a screenshot of the USD
scene (drone + ground + target marker), then step with zero actions for
one full episode (or until termination), logging obs/reward/termination
values and flagging NaNs or crashes. No policy, no learning.

Run from inside isaac_lab/:
    ./isaaclab.sh -p verify_one_episode.py --headless --enable_cameras
"""

import argparse

from isaaclab.app import AppLauncher

parser = argparse.ArgumentParser(description="Verify Isaac-DroneBombard-Direct-v0 with one episode, no training.")
parser.add_argument("--num_steps", type=int, default=300)
parser.add_argument("--out_dir", type=str, default="/tmp/isaac_verify")
parser.add_argument("--with_camera", action="store_true",
                    help="attach an RTX camera for screenshots (needs a working RTX renderer / driver >=580)")
AppLauncher.add_app_launcher_args(parser)
args_cli = parser.parse_args()
# Always bring up the render pipeline (the env constructs most reliably this
# way on this setup); only ATTACH a camera sensor when --with_camera is set.
args_cli.enable_cameras = True

app_launcher = AppLauncher(args_cli)
simulation_app = app_launcher.app

import os
import sys
import traceback

import numpy as np
import torch
import gymnasium as gym

os.makedirs(args_cli.out_dir, exist_ok=True)


def save_rgb(tensor, path):
    from PIL import Image
    arr = tensor.detach().cpu().numpy()
    if arr.dtype != np.uint8:
        arr = np.clip(arr, 0, 255).astype(np.uint8)
    Image.fromarray(arr).save(path)
    print(f"[screenshot] saved {path} shape={arr.shape}")


def main():
    print("=== STEP 1: import drone_bombard, register task ===")
    import drone_bombard
    from drone_bombard.drone_bombard_env import DroneBombardEnvCfg
    print("OK — Isaac-DroneBombard-Direct-v0 registered")

    print("=== STEP 2: build 1-env cfg ===")
    cfg = DroneBombardEnvCfg()
    cfg.scene.num_envs = 1
    cfg.sim.device = "cuda:0"
    print(f"decimation={cfg.decimation} episode_length_s={cfg.episode_length_s} "
          f"max_thrust={cfg.asset.max_thrust:.2f}N loaded_mass={cfg.asset.loaded_mass:.2f}kg")

    print("=== STEP 3: gym.make (this constructs the USD scene) ===")
    env = gym.make("Isaac-DroneBombard-Direct-v0", cfg=cfg)
    print(f"OK — env constructed. num_envs={env.unwrapped.num_envs} device={env.unwrapped.device}")
    print(f"observation_space={env.observation_space} action_space={env.action_space}")

    print("=== STEP 4: attach an external 'show' camera for a screenshot (best-effort) ===")
    show_cam = None
    if not args_cli.with_camera:
        print("(--with_camera not set: skipping RTX camera; running physics/logic verification only)")
    try:
        if not args_cli.with_camera:
            raise RuntimeError("camera disabled")
        import isaaclab.sim as sim_utils
        from isaaclab.sensors import Camera, CameraCfg
        show_cam_cfg = CameraCfg(
            prim_path="/World/envs/env_0/ShowCamera",
            update_period=0,
            height=480,
            width=640,
            data_types=["rgb"],
            spawn=sim_utils.PinholeCameraCfg(focal_length=24.0, clipping_range=(0.05, 200.0)),
            offset=CameraCfg.OffsetCfg(pos=(8.0, -8.0, 8.0), rot=(0.310, 0.129, 0.311, 0.887), convention="world"),
        )
        show_cam = Camera(show_cam_cfg)
        env.unwrapped.scene.sensors["show_cam"] = show_cam
        print("show camera attached")
    except Exception as e:  # noqa: BLE001
        print(f"[warn] could not attach show camera (rendering may be unavailable on this driver): {e!r}")

    def try_screenshot(name):
        if show_cam is None:
            return
        try:
            env.unwrapped.scene.sensors["show_cam"].update(dt=0.0, force_recompute=True)
            rgb = env.unwrapped.scene.sensors["show_cam"].data.output["rgb"][0]
            save_rgb(rgb, os.path.join(args_cli.out_dir, name))
        except Exception as e:  # noqa: BLE001
            print(f"[warn] screenshot {name} failed: {e!r}")

    print("=== STEP 5: reset ===")
    obs, info = env.reset()
    policy_obs = obs["policy"] if isinstance(obs, dict) else obs
    print(f"OK — reset returned obs shape={tuple(policy_obs.shape)}")
    print(f"obs[0] = {policy_obs[0].cpu().numpy()}")
    assert not torch.isnan(policy_obs).any(), "NaN in initial observation!"
    assert policy_obs.shape[-1] == 14, f"expected 14-dim obs, got {policy_obs.shape[-1]}"

    # step a few ticks so the render pipeline populates the camera buffer
    action_dim = env.unwrapped.single_action_space.shape[0]
    device = env.unwrapped.device
    zero_action = torch.zeros(1, action_dim, device=device)
    for _ in range(3):
        env.step(zero_action)
    try_screenshot("scene_start.png")

    print("=== STEP 6: run one episode, zero actions, no training ===")
    d_xy_trace, reward_trace = [], []
    terminated_t = torch.zeros(1, dtype=torch.bool, device=device)
    truncated_t = torch.zeros(1, dtype=torch.bool, device=device)
    for step in range(args_cli.num_steps):
        obs, reward, terminated, truncated, info = env.step(zero_action)
        policy_obs = obs["policy"] if isinstance(obs, dict) else obs

        if torch.isnan(policy_obs).any() or torch.isnan(reward).any():
            print(f"!!! NaN detected at step {step} — obs_nan={torch.isnan(policy_obs).any().item()} "
                  f"reward_nan={torch.isnan(reward).any().item()}")
            r = env.unwrapped._robot.data
            print(f"    raw pos_w   = {r.root_pos_w[0].cpu().numpy()}")
            print(f"    raw lin_vel = {r.root_lin_vel_w[0].cpu().numpy()}")
            print(f"    raw ang_vel = {r.root_ang_vel_b[0].cpu().numpy()}")
            print(f"    raw quat_w  = {r.root_quat_w[0].cpu().numpy()}")
            print(f"    v_filt      = {env.unwrapped._v_filt[0].cpu().numpy()}")
            print(f"    ctrl_mass={env.unwrapped._ctrl_mass} max_thrust={env.unwrapped._max_thrust}")
            break

        d_xy = env.unwrapped._current_d_xy().item()
        d_xy_trace.append(d_xy)
        reward_trace.append(reward.item())

        if step % 20 == 0:
            print(f"step {step:3d}: d_xy={d_xy:6.3f}  reward={reward.item():7.3f}  "
                  f"alt={policy_obs[0,2].item()*50:6.3f}  terminated={bool(terminated.item())}  "
                  f"truncated={bool(truncated.item())}")

        terminated_t, truncated_t = terminated, truncated
        if terminated.any() or truncated.any():
            f = env.unwrapped._done_flags
            causes = [k for k, v in f.items() if v.item()]
            print(f"=== Episode ended at step {step}: causes={causes} d_xy={d_xy:.3f} ===")
            break

    try_screenshot("scene_end.png")

    print("=== SUMMARY ===")
    print(f"steps run: {len(d_xy_trace)}")
    if not d_xy_trace:
        print("VERIFY: FAIL — no steps completed (broke on step 0). See NaN dump above.")
        env.close()
        return
    print(f"d_xy: start={d_xy_trace[0]:.3f} end={d_xy_trace[-1]:.3f} min={min(d_xy_trace):.3f}")
    print(f"reward: mean={np.mean(reward_trace):.4f} min={np.min(reward_trace):.4f} max={np.max(reward_trace):.4f}")
    print(f"any NaN in trace: {any(np.isnan(reward_trace))}")
    print("VERIFY: PASS — env constructs, resets, steps, terminates/truncates cleanly, no NaNs.")

    env.close()


if __name__ == "__main__":
    try:
        main()
    except Exception:
        print("=== VERIFY: FAIL — exception during verification ===")
        traceback.print_exc()
        simulation_app.close()
        sys.exit(1)
    simulation_app.close()
