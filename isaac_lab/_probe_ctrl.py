"""Closed-loop sanity probe: zero velocity command from the cruise handoff.
Usage: isaaclab.sh -p _probe_ctrl.py --headless --mode px4|p [--motor 0|1] [--steps 150]"""
import argparse
from isaaclab.app import AppLauncher
ap = argparse.ArgumentParser()
ap.add_argument("--mode", default="px4"); ap.add_argument("--motor", type=int, default=1)
ap.add_argument("--steps", type=int, default=150); ap.add_argument("--quiet_handoff", type=int, default=0); ap.add_argument("--ang_lim", type=float, default=0); ap.add_argument("--k_rate", type=float, nargs=3, default=None); ap.add_argument("--att", type=float, default=0); ap.add_argument("--rand", type=int, default=0); ap.add_argument("--yaw0", type=int, default=0); ap.add_argument("--dp", type=float, default=-1); ap.add_argument("--tau", type=float, default=0); ap.add_argument("--pk", type=float, nargs=4, default=None); ap.add_argument("--no_dr", type=int, default=0); ap.add_argument("--num_envs", type=int, default=256)
AppLauncher.add_app_launcher_args(ap)
args = ap.parse_args()
app = AppLauncher(args).app
import math, torch, gymnasium as gym
import drone_bombard  # noqa
from drone_bombard.task_env import DroneBombardTaskCfg
cfg = DroneBombardTaskCfg()
cfg.scene.num_envs = args.num_envs
cfg.residual.enabled = False
cfg.controller.mode = args.mode
cfg.controller.motor_model = bool(args.motor)
if args.quiet_handoff:
    cfg.handoff.attitude_std_deg = 0.0; cfg.handoff.ang_vel_std = 0.0
if args.dp >= 0:
    cfg.controller.rate_d_over_p = (args.dp, args.dp, 0.0)
if args.tau:
    cfg.controller.motor_tau_up_s = args.tau
if args.pk:
    cfg.controller.px4_k_rate = tuple(args.pk[:3]); cfg.controller.px4_k_att_rp = args.pk[3]
if args.yaw0:
    cfg.action.yaw_scale = 0.0
if args.no_dr:
    cfg.dyn_dr.enabled = False
if args.k_rate:
    cfg.controller.k_rate = tuple(args.k_rate)
if args.att:
    cfg.controller.k_att_rp = args.att
if args.ang_lim:
    cfg.termination.limit_ang_vel = args.ang_lim
env = gym.make("Isaac-DroneBombard-Task-v0", cfg=cfg).unwrapped
env.reset()
N = env.num_envs
a = torch.zeros(N, env.cfg.action_space, device=env.device)
done_steps = torch.full((N,), -1, device=env.device)
tilt_max = torch.zeros(N, device=env.device); w_max = torch.zeros(N, device=env.device)
agg = {}
for k in range(args.steps):
    if args.rand:
        a = (torch.rand_like(a) * 2 - 1)
    _, _, term, trunc, ex = env.step(a)
    for kk, vv in (ex.get("log", {}) or {}).items():
        if "ermin" in kk or "Episode_Term" in kk:
            agg[kk] = agg.get(kk, 0.0) + float(vv) * float((term | trunc).sum())
    q = env._robot.data.root_quat_w
    w, x, y, z = q.unbind(-1)
    tilt = torch.acos((1 - 2 * (x * x + y * y)).clamp(-1, 1))
    tilt_max = torch.maximum(tilt_max, tilt); w_max = torch.maximum(w_max, env._robot.data.root_ang_vel_b.norm(dim=-1))
    d = (term | trunc) & (done_steps < 0); done_steps[d] = k
    if k < 4:
        wn = env._robot.data.root_ang_vel_b.norm(dim=-1)
        pass  # print(f"STEP{k} |w| p50/p95/max={wn.median():.2f}/{wn.quantile(.95):.2f}/{wn.max():.2f} tilt p95={math.degrees(tilt.quantile(.95)):.1f} done={int((term|trunc).sum())} vcmd0={env._vel_cmd[0,:3].tolist()}")
v = env._robot.data.root_lin_vel_w
print(f"RESULT pk={cfg.controller.px4_k_rate}/{cfg.controller.px4_k_att_rp} dp={cfg.controller.rate_d_over_p[0]} mode={args.mode} motor={args.motor} T/W={env._max_thrust/(env._ctrl_mass_nominal*9.81):.2f} "
      f"terminated_frac={(done_steps>=0).float().mean():.3f} first_done_p50={done_steps[done_steps>=0].float().median().item() if (done_steps>=0).any() else -1:.0f} "
      f"tilt_max_p50/p95={math.degrees(tilt_max.median()):.1f}/{math.degrees(tilt_max.quantile(0.95)):.1f}deg "
      f"angvel_max_p95={w_max.quantile(0.95):.2f}rad/s final|v|_p50={v.norm(dim=-1).median():.2f} "
      f"motorT_minmax={env._motor_T.min():.2f}/{env._motor_T.max():.2f}N Tmax/4={float(env._max_thrust)/4:.2f}N")
print('TERMS', {k: round(v, 1) for k, v in agg.items() if v})
env.close(); app.close()
