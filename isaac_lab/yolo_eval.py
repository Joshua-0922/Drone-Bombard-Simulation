"""Eval-only variant of Isaac-DroneBombard-Direct-v0: real TiledCamera +
YOLOv8 (``drone_bombard_best.pt``) in place of the analytic vision model.

Two purposes:
  --calibrate   sweep slant range 3-15 m x off-nadir angle 0-40 deg, log
                real YOLO pixel-error/conf/dropout binned by (range, angle)
                to recalibrate DroneBombardVisionCfg. Re-run at every
                curriculum phase transition (spawn-range change,
                success_radius tightening, any future moving-target phase)
                per the migration plan's vision recalibration policy.
  --eval        final policy evaluation with YOLO in the loop (num_envs<=8).

Both modes also log diagnostic back-projection position error (YOLO uv +
intrinsics + drone pose -> estimated target position vs ground truth) for a
future vision-only variant. The current policy does NOT consume this
estimate (obs[12:14] are ground-truth-derived, exactly as in Gazebo) — see
the vision-pipeline design note in drone_bombard_env.py's module docstring.
Concrete review triggers for these logs: (1) --eval success rate falling
>10pp below the analytic-vision eval at the same checkpoint, (2) before
training any vision-only observation variant.
"""

import argparse

from isaaclab.app import AppLauncher

parser = argparse.ArgumentParser(description="YOLO-in-the-loop eval / vision calibration.")
parser.add_argument("--task", type=str, default="Isaac-DroneBombard-Direct-v0")
parser.add_argument("--num_envs", type=int, default=8)
parser.add_argument("--calibrate", action="store_true")
parser.add_argument("--eval", action="store_true")
parser.add_argument("--policy", type=str, default=None)
parser.add_argument("--yolo-weights", type=str, default="/workspace/drone-bombard/drone_bombard_best.pt")
parser.add_argument("--marker-texture", type=str,
                     default="/workspace/drone-bombard/gazebo_models/worlds/x_marker_world.sdf")
parser.add_argument("--out-csv", type=str, default="/workspace/logs/isaac_lab/yolo_calibration.csv")
parser.add_argument("--range-bins", type=int, default=5, help="slant range 3-15m sweep bins")
parser.add_argument("--angle-bins", type=int, default=5, help="off-nadir 0-40deg sweep bins")
AppLauncher.add_app_launcher_args(parser)
args_cli = parser.parse_args()

app_launcher = AppLauncher(args_cli)
simulation_app = app_launcher.app

import csv
import math

import torch
import gymnasium as gym

from isaaclab.sensors import TiledCamera, TiledCameraCfg
from isaaclab.utils.math import euler_xyz_from_quat
from isaaclab_tasks.utils import parse_env_cfg

import drone_bombard  # noqa: F401
from drone_bombard.drone_bombard_env import project_target_pinhole


def _build_camera_cfg(vc):
    return TiledCameraCfg(
        prim_path="/World/envs/env_.*/Robot/down_camera",
        offset=TiledCameraCfg.OffsetCfg(pos=(0.0, 0.0, -0.02), rot=(0.7071, 0.0, 0.7071, 0.0), convention="ros"),
        data_types=["rgb"],
        width=vc.img_w,
        height=vc.img_h,
        spawn=None,
    )


def _run_yolo(model, rgb_batch):
    """rgb_batch: [N, H, W, 3] uint8 tensor. Returns (u_px, v_px, conf, detected) each [N]."""
    import numpy as np

    n = rgb_batch.shape[0]
    u = torch.zeros(n)
    v = torch.zeros(n)
    conf = torch.zeros(n)
    detected = torch.zeros(n, dtype=torch.bool)
    imgs = [rgb_batch[i].cpu().numpy() for i in range(n)]
    results = model.predict(imgs, verbose=False)
    for i, r in enumerate(results):
        if len(r.boxes) == 0:
            continue
        best = int(r.boxes.conf.argmax())
        box = r.boxes.xywh[best]
        u[i] = float(box[0])
        v[i] = float(box[1])
        conf[i] = float(r.boxes.conf[best])
        detected[i] = True
    return u, v, conf, detected


def run_calibrate(env, model, out_csv, range_bins, angle_bins):
    vc = env.unwrapped.cfg.vision
    rows = []
    ranges = torch.linspace(3.0, 15.0, range_bins)
    angles = torch.linspace(0.0, math.radians(40.0), angle_bins)

    for r in ranges.tolist():
        for a in angles.tolist():
            obs, _ = env.reset()
            n = env.unwrapped.num_envs
            device = env.unwrapped.device

            target_xy = env.unwrapped._target_xy
            spawn_xy = target_xy + torch.stack([
                r * math.sin(a) * torch.ones(n, device=device),
                torch.zeros(n, device=device),
            ], dim=-1)
            spawn_alt = r * math.cos(a)
            root = env.unwrapped._robot.data.root_state_w.clone()
            root[:, 0:2] = spawn_xy + env.unwrapped.scene.env_origins[:, :2]
            root[:, 2] = spawn_alt
            env.unwrapped._robot.write_root_pose_to_sim(root[:, :7])

            for _ in range(5):
                env.step(torch.zeros(n, 4, device=device))

            camera: TiledCamera = env.unwrapped.scene["down_camera"]
            rgb = camera.data.output["rgb"]
            u_yolo, v_yolo, conf_yolo, detected = _run_yolo(model, rgb)

            # env-LOCAL frame: _target_xy is per-env-origin relative, so the
            # drone position must be too (same fix as DroneBombardEnv._update_vision).
            pos_local = env.unwrapped._robot.data.root_pos_w - env.unwrapped.scene.env_origins
            quat_w = env.unwrapped._robot.data.root_quat_w
            u_geo, v_geo, visible = project_target_pinhole(
                pos_local, quat_w, target_xy, vc.fx, vc.fy, vc.cx, vc.cy, vc.img_w, vc.img_h, vc.near_clip, vc.far_clip,
            )

            for i in range(n):
                du = (u_yolo[i] - u_geo[i].cpu()).item() if detected[i] else float("nan")
                dv = (v_yolo[i] - v_geo[i].cpu()).item() if detected[i] else float("nan")
                rows.append([r, math.degrees(a), bool(detected[i]), float(conf_yolo[i]), du, dv])

    with open(out_csv, "w", newline="") as f:
        writer = csv.writer(f)
        writer.writerow(["slant_range_m", "off_nadir_deg", "detected", "conf", "du_px", "dv_px"])
        writer.writerows(rows)
    print(f"[calibrate] wrote {len(rows)} rows to {out_csv}")


def run_eval(env, model, policy_path, episodes=20):
    from rsl_rl.runners import OnPolicyRunner
    from drone_bombard.agents.rsl_rl_ppo_cfg import DroneBombardPPORunnerCfg

    agent_cfg = DroneBombardPPORunnerCfg()
    runner = OnPolicyRunner(env, agent_cfg.to_dict(), log_dir=None, device=agent_cfg.device)
    runner.load(policy_path)
    policy = runner.get_inference_policy(device=env.unwrapped.device)

    obs, _ = env.reset()
    n_done, n_success = 0, 0
    backproj_errors = []
    while n_done < episodes:
        camera: TiledCamera = env.unwrapped.scene["down_camera"]
        rgb = camera.data.output["rgb"]
        u_yolo, v_yolo, conf_yolo, detected = _run_yolo(model, rgb)

        # Overwrite the analytic obs[9:12] with real YOLO detections — this
        # is the only substitution; obs[12:14] stay ground-truth as designed.
        u_norm = torch.where(detected, (u_yolo / 640.0) * 2.0 - 1.0, torch.zeros_like(u_yolo))
        v_norm = torch.where(detected, (v_yolo / 480.0) * 2.0 - 1.0, torch.zeros_like(v_yolo))
        obs[:, 9] = u_norm.to(obs.device)
        obs[:, 10] = v_norm.to(obs.device)
        obs[:, 11] = torch.where(detected, conf_yolo, torch.zeros_like(conf_yolo)).to(obs.device)

        with torch.inference_mode():
            action = policy(obs)
        obs, rew, terminated, truncated, info = env.step(action)
        done = terminated | truncated
        if done.any():
            f = env.unwrapped._done_flags
            n_done += int(done.sum().item())
            n_success += int(f["success"][done].sum().item())

    print(f"[eval] episodes={n_done} success_rate={n_success/max(n_done,1):.2%}")


def main():
    from ultralytics import YOLO

    env_cfg = parse_env_cfg(args_cli.task, num_envs=args_cli.num_envs)
    vc = env_cfg.vision
    env_cfg.scene.down_camera = _build_camera_cfg(vc)

    env = gym.make(args_cli.task, cfg=env_cfg)
    model = YOLO(args_cli.yolo_weights)

    if args_cli.calibrate:
        run_calibrate(env, model, args_cli.out_csv, args_cli.range_bins, args_cli.angle_bins)
    elif args_cli.eval:
        if not args_cli.policy:
            raise SystemExit("--eval requires --policy CKPT")
        run_eval(env, model, args_cli.policy)
    else:
        print("Specify --calibrate or --eval --policy CKPT")

    env.close()


if __name__ == "__main__":
    main()
    simulation_app.close()
