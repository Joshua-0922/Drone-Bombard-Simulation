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
                     default="/workspace/drone-bombard/gazebo_models/x_marker/materials/textures/x_marker.png")
parser.add_argument("--out-csv", type=str, default="/workspace/logs/isaac_lab/yolo_calibration.csv")
parser.add_argument("--range-bins", type=int, default=5, help="slant range sweep bins")
parser.add_argument("--range-max", type=float, default=15.0,
                    help="slant range sweep maximum (m); 27 covers the full "
                         "climb-attractor envelope up to the 25 m altitude ceiling")
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
    import isaaclab.sim as sim_utils

    # spawn=None would require a down_camera prim in the robot USD — the
    # cf2x asset has none, so the sensor prim must be spawned here. Focal
    # length chosen so the USD camera's h_fov matches the analytic pinhole:
    # h_fov = 2*atan(h_aperture / (2*f)), aperture 20.955 mm (USD default).
    import math as _math
    focal = 20.955 / (2.0 * _math.tan(vc.h_fov / 2.0))
    return TiledCameraCfg(
        prim_path="/World/envs/env_.*/Robot/down_camera",
        offset=TiledCameraCfg.OffsetCfg(pos=(0.0, 0.0, -0.02), rot=(0.7071, 0.0, 0.7071, 0.0), convention="ros"),
        data_types=["rgb"],
        width=vc.img_w,
        height=vc.img_h,
        spawn=sim_utils.PinholeCameraCfg(
            focal_length=focal,
            horizontal_aperture=20.955,
            clipping_range=(vc.near_clip, vc.far_clip),
        ),
    )


def _spawn_target_markers(env, texture_path):
    """One 1.5 x 1.5 m ground quad per env, textured with the Gazebo
    x_marker.png (same physical size as the Gazebo x_marker model.sdf) —
    the visual YOLO was trained on. Pure visuals, no physics. Returns the
    per-env translate ops for _sync_target_markers. Without this the scene
    has NO visible target and --calibrate measures YOLO against a bare
    ground plane (all-miss)."""
    import omni.usd
    from pxr import Gf, Sdf, UsdGeom, UsdShade

    stage = omni.usd.get_context().get_stage()

    mat_path = "/World/Looks/XMarkerMat"
    material = UsdShade.Material.Define(stage, mat_path)
    shader = UsdShade.Shader.Define(stage, mat_path + "/Shader")
    shader.CreateIdAttr("UsdPreviewSurface")
    shader.CreateInput("roughness", Sdf.ValueTypeNames.Float).Set(0.8)
    shader.CreateInput("metallic", Sdf.ValueTypeNames.Float).Set(0.0)
    st_reader = UsdShade.Shader.Define(stage, mat_path + "/stReader")
    st_reader.CreateIdAttr("UsdPrimvarReader_float2")
    st_reader.CreateInput("varname", Sdf.ValueTypeNames.Token).Set("st")
    tex = UsdShade.Shader.Define(stage, mat_path + "/Tex")
    tex.CreateIdAttr("UsdUVTexture")
    tex.CreateInput("file", Sdf.ValueTypeNames.Asset).Set(texture_path)
    tex.CreateInput("st", Sdf.ValueTypeNames.Float2).ConnectToSource(
        st_reader.ConnectableAPI(), "result")
    shader.CreateInput("diffuseColor", Sdf.ValueTypeNames.Color3f).ConnectToSource(
        tex.ConnectableAPI(), "rgb")
    material.CreateSurfaceOutput().ConnectToSource(shader.ConnectableAPI(), "surface")

    half = 0.75  # 1.5 m marker, matches gazebo_models/x_marker/model.sdf
    ops = []
    for i in range(env.unwrapped.num_envs):
        mesh = UsdGeom.Mesh.Define(stage, f"/World/XMarkers/marker_{i}")
        mesh.CreatePointsAttr([(-half, -half, 0), (half, -half, 0),
                               (half, half, 0), (-half, half, 0)])
        mesh.CreateFaceVertexCountsAttr([4])
        mesh.CreateFaceVertexIndicesAttr([0, 1, 2, 3])
        mesh.CreateExtentAttr([(-half, -half, 0), (half, half, 0)])
        st = UsdGeom.PrimvarsAPI(mesh.GetPrim()).CreatePrimvar(
            "st", Sdf.ValueTypeNames.TexCoord2fArray, UsdGeom.Tokens.varying)
        st.Set([(0, 0), (1, 0), (1, 1), (0, 1)])
        UsdShade.MaterialBindingAPI.Apply(mesh.GetPrim()).Bind(material)
        ops.append(UsdGeom.Xformable(mesh).AddTranslateOp())
    return ops


def _sync_target_markers(env, marker_ops):
    """Move each env's marker quad to its CURRENT target position. Call after
    every reset (targets are resampled per episode)."""
    from pxr import Gf

    origins = env.unwrapped.scene.env_origins[:, :2].cpu()
    targets = env.unwrapped._target_xy.cpu()
    for i, op in enumerate(marker_ops):
        op.Set(Gf.Vec3d(float(origins[i, 0] + targets[i, 0]),
                        float(origins[i, 1] + targets[i, 1]), 0.02))


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


def run_calibrate(env, model, marker_ops, out_csv, range_bins, range_max, angle_bins):
    vc = env.unwrapped.cfg.vision
    rows = []
    ranges = torch.linspace(3.0, range_max, range_bins)
    angles = torch.linspace(0.0, math.radians(40.0), angle_bins)

    for r in ranges.tolist():
        for a in angles.tolist():
            obs, _ = env.reset()
            _sync_target_markers(env, marker_ops)
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

            # re-pin pose+velocity every settle step: the bin pose must not
            # drift (hover transients) — terminations are disabled for the
            # calibrate task in main(), so no mid-bin resets either.
            zero_vel = torch.zeros(n, 6, device=device)
            action_dim = env.unwrapped.cfg.action_space  # 6 (vel[0:4] + CCIP residual[4:6])
            for _ in range(5):
                env.unwrapped._robot.write_root_pose_to_sim(root[:, :7])
                env.unwrapped._robot.write_root_velocity_to_sim(zero_vel)
                env.step(torch.zeros(n, action_dim, device=device))

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


def run_eval(env, model, marker_ops, policy_path, episodes=20):
    from rsl_rl.runners import OnPolicyRunner
    from drone_bombard.agents.rsl_rl_ppo_cfg import DroneBombardPPORunnerCfg

    agent_cfg = DroneBombardPPORunnerCfg()
    runner = OnPolicyRunner(env, agent_cfg.to_dict(), log_dir=None, device=agent_cfg.device)
    runner.load(policy_path)
    policy = runner.get_inference_policy(device=env.unwrapped.device)

    obs, _ = env.reset()
    _sync_target_markers(env, marker_ops)
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
            _sync_target_markers(env, marker_ops)  # done envs got fresh targets

    print(f"[eval] episodes={n_done} success_rate={n_success/max(n_done,1):.2%}")


def main():
    from ultralytics import YOLO

    env_cfg = parse_env_cfg(args_cli.task, num_envs=args_cli.num_envs)
    vc = env_cfg.vision

    # DirectRLEnv creates the robot in _setup_scene, AFTER the scene cfg's
    # entities are spawned — a camera injected via env_cfg.scene can never
    # anchor to the robot prim ("Unable to find source prim path" crash).
    # Follow the official direct-env pattern (cartpole_camera_env) instead:
    # create the TiledCamera inside _setup_scene and register it as a sensor.
    cam_cfg = _build_camera_cfg(vc)
    from drone_bombard.drone_bombard_env import DroneBombardEnv

    _orig_setup_scene = DroneBombardEnv._setup_scene

    def _setup_scene_with_camera(self):
        _orig_setup_scene(self)
        self.scene.sensors["down_camera"] = TiledCamera(cam_cfg)

    DroneBombardEnv._setup_scene = _setup_scene_with_camera

    if args_cli.calibrate:
        # teleport sweep: the drone is PLACED at each (range, angle) bin, so
        # the flight-envelope terminations must not fire and reset envs
        # mid-bin (low bins sit below min_altitude, far bins above
        # max_altitude, nadir bins inside success_radius).
        env_cfg.termination.min_altitude = 0.0
        env_cfg.termination.ground_contact_altitude = 0.0
        env_cfg.termination.max_altitude = 1000.0
        env_cfg.reward.success_radius = -1.0

    env = gym.make(args_cli.task, cfg=env_cfg)
    model = YOLO(args_cli.yolo_weights)
    marker_ops = _spawn_target_markers(env, args_cli.marker_texture)

    if args_cli.calibrate:
        run_calibrate(env, model, marker_ops, args_cli.out_csv,
                      args_cli.range_bins, args_cli.range_max, args_cli.angle_bins)
    elif args_cli.eval:
        if not args_cli.policy:
            raise SystemExit("--eval requires --policy CKPT")
        run_eval(env, model, marker_ops, args_cli.policy)
    else:
        print("Specify --calibrate or --eval --policy CKPT")

    env.close()


if __name__ == "__main__":
    main()
    simulation_app.close()
