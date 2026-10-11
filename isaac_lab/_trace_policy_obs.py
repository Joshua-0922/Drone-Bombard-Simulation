"""Record one deterministic Isaac episode (state, 26-ch observation, raw action) for parity checks of
the ROS deployment node (ros2_ws/src/drop_policy). Handoff fixed (heading 0, 4 m/s, 10 m, marker 20 m
ahead -- the disk offset is still drawn), sensor/actuator noise off, model error off.

  isaaclab.sh -p _trace_policy_obs.py --policy L0.pt --out /tmp/isaac_trace.npz [--controller px4]
"""
import argparse
from isaaclab.app import AppLauncher
p = argparse.ArgumentParser()
p.add_argument("--policy", required=True); p.add_argument("--out", required=True)
p.add_argument("--controller", default="p"); p.add_argument("--steps", type=int, default=60)
p.add_argument("--seed", type=int, default=1)
p.add_argument("--disk", type=float, default=None, help="marker_disk_radius override (0 = marker exactly ahead)")
AppLauncher.add_app_launcher_args(p)
a = p.parse_args(); a.headless = True
app = AppLauncher(a).app

import numpy as np, torch, gymnasium as gym
from isaaclab_tasks.utils import parse_env_cfg
import drone_bombard  # noqa: F401
cfg = parse_env_cfg("Isaac-DroneBombard-Task-v0", num_envs=1)
cfg.seed = a.seed
cfg.handoff.randomize = False
cfg.dyn_dr.enabled = False
cfg.model_err.scale = 0.0
cfg.residual.enabled = False
cfg.controller.mode = a.controller
if a.disk is not None:
    cfg.handoff.marker_disk_radius = a.disk
env = gym.make("Isaac-DroneBombard-Task-v0", cfg=cfg)
u = env.unwrapped
sd = torch.load(a.policy, map_location="cpu", weights_only=False)["model_state_dict"]
net = torch.nn.Sequential(torch.nn.Linear(26, 256), torch.nn.ELU(), torch.nn.Linear(256, 256), torch.nn.ELU(), torch.nn.Linear(256, 7)).to(u.device)
net.load_state_dict({k[6:]: v for k, v in sd.items() if k.startswith("actor.")})
obs, _ = env.reset()
rec = {k: [] for k in ("obs", "act", "pos", "vel", "quat", "ang", "target", "prev")}
for t in range(a.steps):
    o = obs["policy"]
    with torch.no_grad():
        act = net(o)
    d = u._robot.data
    rec["obs"].append(o[0].cpu().numpy()); rec["act"].append(act[0].cpu().numpy())
    rec["pos"].append((d.root_pos_w - u.scene.env_origins)[0].cpu().numpy())
    rec["vel"].append(d.root_lin_vel_w[0].cpu().numpy()); rec["quat"].append(d.root_quat_w[0].cpu().numpy())
    rec["ang"].append(d.root_ang_vel_b[0].cpu().numpy()); rec["target"].append(u._target_xy[0].cpu().numpy())
    rec["prev"].append(u._prev_action[0, :4].cpu().numpy())
    obs, r, term, trunc, _ = env.step(act)
    if bool(term[0] | trunc[0]):
        print("episode ended at step", t); break
np.savez(a.out, **{k: np.array(v) for k, v in rec.items()})
print("saved", a.out, len(rec["obs"]), "steps; target", rec["target"][0], "start pos", rec["pos"][0])
env.close(); app.close()
