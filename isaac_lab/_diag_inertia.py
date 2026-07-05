"""_diag_inertia.py — measure the SOLVER's actual rotational inertia.

Motivation (2026-07-05 probe forensics): re-evaluating the exp013_v2 policy on
the spawn-authored plant exploded bad_attitude 0.5% -> 68%, and a prop-spin
A/B proved the rotor fix innocent. Standing hypothesis: the old runtime
``set_masses()`` path silently RESCALED the solver inertia with the mass ratio
(0.025 -> 2.17 kg, x86.8) while the view cache kept reporting the parse-time
1.66e-5 — i.e. the exp_013 plant really had I_eff ~ 1.44e-3 and its rate loop
was ~87x under-torqued. This script measures I directly: pure body-frame
torque, free fall, I_est = tau / alpha from the ang-vel ramp.

Phases (one process, current spawn-authored code):
  P1  as-built plant                     -> expect 1.66e-5 if authoring landed
  P2  after view.set_masses(body->0.025) -> if I rescales DOWN with mass, the
                                            runtime-set_masses inertia-rescale
                                            hypothesis is CONFIRMED
  P3  after view.set_masses(body->2.17)  -> restore, symmetric check
  P4  after view.set_inertias(x500 diag) -> expect NO change (known no-op)

Run (inside isaac-verify container):
  ./isaaclab.sh -p /workspace/drone-bombard/isaac_lab/_diag_inertia.py --headless
"""

import argparse

from isaaclab.app import AppLauncher

parser = argparse.ArgumentParser(description="Solver inertia measurement probe.")
AppLauncher.add_app_launcher_args(parser)
args_cli = parser.parse_args()

app_launcher = AppLauncher(args_cli)
simulation_app = app_launcher.app

import torch

import gymnasium as gym
from isaaclab_tasks.utils import parse_env_cfg

import drone_bombard  # noqa: F401

TAU = 5e-5          # N*m about body x — alpha=3 rad/s^2 at I=1.66e-5, 0.035 at 1.44e-3
N_TORQUE = 25       # substeps with torque applied (0.25 s)
N_SETTLE = 20       # substeps to coast between phases
DT = 0.01


def zero_state(env):
    u = env.unwrapped
    root = u._robot.data.default_root_state.clone()
    root[:, 0:3] = u.scene.env_origins + torch.tensor([0.0, 0.0, 10.0], device=u.device)
    u._robot.write_root_pose_to_sim(root[:, :7])
    u._robot.write_root_velocity_to_sim(torch.zeros_like(root[:, 7:13]))


def set_wrench(env, tau_x):
    u = env.unwrapped
    forces = torch.zeros(u.num_envs, 1, 3, device=u.device)
    torques = torch.zeros(u.num_envs, 1, 3, device=u.device)
    torques[:, 0, 0] = tau_x
    u._robot.permanent_wrench_composer.set_forces_and_torques(
        body_ids=u._body_id, forces=forces, torques=torques
    )


def measure(env, label):
    u = env.unwrapped
    zero_state(env)
    set_wrench(env, 0.0)
    for _ in range(N_SETTLE):
        u.scene.write_data_to_sim()
        u.sim.step(render=False)
        u.scene.update(DT)
    set_wrench(env, TAU)
    w = []
    for _ in range(N_TORQUE):
        u.scene.write_data_to_sim()
        u.sim.step(render=False)
        u.scene.update(DT)
        w.append(float(u._robot.data.root_ang_vel_b[0, 0]))
    set_wrench(env, 0.0)
    # linear fit omega(t) = alpha*t on the ramp
    t = torch.arange(1, N_TORQUE + 1, dtype=torch.float64) * DT
    wt = torch.tensor(w, dtype=torch.float64)
    alpha = float((t * wt).sum() / (t * t).sum())
    i_est = TAU / alpha if alpha != 0 else float("inf")
    bidx = u._body_id[0] if isinstance(u._body_id, (list, tuple)) else int(u._body_id)
    view = u._robot.root_physx_view
    m_view = float(view.get_masses()[0, bidx])
    i_view = float(view.get_inertias()[0, bidx].reshape(3, 3)[0, 0])
    print(f"[P {label}] alpha={alpha:+.5f} rad/s^2  I_est={i_est:.3e}  "
          f"view: m={m_view:.4f} Ixx={i_view:.3e}  w_end={w[-1]:+.5f}")
    return i_est


def main():
    env_cfg = parse_env_cfg("Isaac-DroneBombard-Direct-v0", num_envs=1)
    env = gym.make("Isaac-DroneBombard-Direct-v0", cfg=env_cfg)
    env.reset()
    u = env.unwrapped
    view = u._robot.root_physx_view
    bidx = u._body_id[0] if isinstance(u._body_id, (list, tuple)) else int(u._body_id)
    idx = torch.arange(u.num_envs)

    i1 = measure(env, "1 as-built (authored mass=2.17, I=1.66e-5)")

    m = view.get_masses().clone(); m[:, bidx] = 0.025
    view.set_masses(m, idx)
    i2 = measure(env, "2 after set_masses(body->0.025)")

    m = view.get_masses().clone(); m[:, bidx] = 2.17
    view.set_masses(m, idx)
    i3 = measure(env, "3 after set_masses(body->2.17) restore")

    iner = view.get_inertias().clone()
    iner[:, bidx, :] = torch.tensor([0.0217, 0, 0, 0, 0.0217, 0, 0, 0, 0.04], dtype=iner.dtype)
    view.set_inertias(iner, idx)
    i4 = measure(env, "4 after set_inertias(x500 0.0217)")

    print("=" * 70)
    print(f"[VERDICT] I1={i1:.3e} I2={i2:.3e} I3={i3:.3e} I4={i4:.3e}")
    print(f"  rescale-with-mass hypothesis: {'CONFIRMED' if i2 < i1 / 10 else 'REFUTED'} "
          f"(I2/I1={i2/i1:.3f}, mass ratio 0.0115)")
    print(f"  set_inertias propagates: {'YES' if abs(i4 - i3) / i3 > 0.5 else 'NO'} "
          f"(I4/I3={i4/i3:.3f})")
    env.close()


if __name__ == "__main__":
    main()
    simulation_app.close()
