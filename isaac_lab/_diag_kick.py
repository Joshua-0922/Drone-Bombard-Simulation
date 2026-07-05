"""_diag_kick.py — LIVE isolation of the reset velocity-kick mechanism.

HISTORICAL (2026-07-04 forensics): this script diagnosed the runtime
``set_masses()`` override that ``DroneBombardEnv.__init__`` used to call via
``_apply_body_mass_override``. That override was replaced by spawn-time USD
authoring (``_author_body_mass_props``), so the monkeypatch modes (t3/t4m/t4i)
now patch a method nobody calls and are inert. Running ``--mode combo``
against the fixed env is still meaningful as a regression check: every phase
should show dvz ~= 0 at ssr=1. Full findings: notes/experiments/exp_013 §4d.

Hypothesis H1: during the reset-flush physics substep(s) the PhysX solver
integrates our applied body wrench against the NATIVE body mass (~0.027 kg)
instead of the overridden 2.173 kg, while hover-sized thrust (~21.3 N,
gravity comp = m_ctrl*9.81) is active. Prediction dv = F*dt/m_native ~= 7.9 m/s.

Modes (argparse --mode):
  combo : one process, several phases:
            t1m  - baseline kick, manual env.reset() path (2 cycles)
            t1a  - baseline kick, auto-reset-inside-step() path (2 cycles)
            t2   - wrench zeroed at reset + thrust suppressed for the whole
                   first post-reset policy step (10 substeps)
            t6   - re-write root velocity after the 1st post-reset substep
  t3    : same-value override at init (set_masses/set_inertias with UNCHANGED
          native values) — does the CALL itself cause the kick?
  t4m   : mass-only override (set_masses only, no set_inertias)
  t4i   : inertia-only override (set_inertias only, no set_masses)

Per-substep log line (velocity is read at substep ENTRY, i.e. it reflects the
integration result of the PREVIOUS substep; the thrust on the same line is
what will be applied in THIS substep):
  [phase] k=<global substep> ssr=<substeps since last reset>
  v=(vx,vy,vz) dvz alt F_thr(applied this substep) |w| m_view

Run (inside isaac-verify container):
  ./isaaclab.sh -p /workspace/drone-bombard/isaac_lab/_diag_kick.py --headless --mode combo
"""

import argparse

from isaaclab.app import AppLauncher

parser = argparse.ArgumentParser(description="Reset velocity-kick mechanism diagnostic.")
parser.add_argument("--mode", type=str, default="combo",
                    choices=["combo", "t3", "t4m", "t4i", "noreset", "t2first"])
parser.add_argument("--steps_per_cycle", type=int, default=3,
                    help="policy steps logged after each reset")
AppLauncher.add_app_launcher_args(parser)
args_cli = parser.parse_args()
args_cli.enable_cameras = True  # env constructs most reliably this way (see verify_one_episode.py)

app_launcher = AppLauncher(args_cli)
simulation_app = app_launcher.app

import os
import sys
import traceback
from types import SimpleNamespace

import torch

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))

import drone_bombard  # noqa: F401  (registers the task / package import path)
from drone_bombard.drone_bombard_env import DroneBombardEnv, DroneBombardEnvCfg

DT = 0.01  # physics dt
G = 9.81


# ---------------------------------------------------------------------------
# __init__-time override patches (must be installed on the CLASS before
# construction because _apply_body_mass_override runs inside __init__).
# ---------------------------------------------------------------------------
def install_override_patch(mode: str):
    if mode == "t3":
        def _override_same(self):
            view = self._robot.root_physx_view
            masses = view.get_masses().clone()
            inertias = view.get_inertias().clone()
            idx = torch.arange(self.num_envs)
            view.set_masses(masses, idx)
            view.set_inertias(inertias, idx)
            print(f"[DIAG t3] wrote back UNCHANGED masses (per-link={masses[0].tolist()}, "
                  f"sum={float(masses[0].sum()):.6f} kg) and unchanged inertias")
        DroneBombardEnv._apply_body_mass_override = _override_same
    elif mode == "t4m":
        def _override_mass_only(self):
            asset = self.cfg.asset
            view = self._robot.root_physx_view
            masses = view.get_masses().clone()
            bidx = self._body_id[0] if isinstance(self._body_id, (list, tuple)) else int(self._body_id)
            print(f"[DIAG t4m] native masses per-link={masses[0].tolist()} sum={float(masses[0].sum()):.6f}")
            masses[:, bidx] = asset.loaded_mass
            idx = torch.arange(self.num_envs)
            view.set_masses(masses, idx)
            print(f"[DIAG t4m] set_masses ONLY (body -> {asset.loaded_mass} kg); set_inertias SKIPPED")
        DroneBombardEnv._apply_body_mass_override = _override_mass_only
    elif mode == "t4i":
        def _override_inertia_only(self):
            asset = self.cfg.asset
            view = self._robot.root_physx_view
            inertias = view.get_inertias().clone()
            bidx = self._body_id[0] if isinstance(self._body_id, (list, tuple)) else int(self._body_id)
            ixx, iyy, izz = asset.inertia_diag
            inertias[:, bidx, :] = torch.tensor(
                [ixx, 0.0, 0.0, 0.0, iyy, 0.0, 0.0, 0.0, izz], dtype=inertias.dtype
            )
            idx = torch.arange(self.num_envs)
            view.set_inertias(inertias, idx)
            print(f"[DIAG t4i] set_inertias ONLY (body -> diag({ixx},{iyy},{izz})); set_masses SKIPPED "
                  f"(masses stay native, sum={float(view.get_masses()[0].sum()):.6f} kg)")
        DroneBombardEnv._apply_body_mass_override = _override_inertia_only
    # "combo" keeps the stock override (full set_masses + set_inertias)


# ---------------------------------------------------------------------------
def fmt3(t):
    return f"({float(t[0]):+8.3f},{float(t[1]):+8.3f},{float(t[2]):+8.3f})"


def main():
    install_override_patch(args_cli.mode)

    cfg = DroneBombardEnvCfg()
    cfg.scene.num_envs = 1
    cfg.sim.device = "cuda:0"
    # deterministic spawn velocity: anything nonzero after reset is INJECTED, not sampled
    cfg.reset.init_vel_std = 0.0

    env = DroneBombardEnv(cfg)
    robot = env._robot
    view = robot.root_physx_view
    composer = robot.permanent_wrench_composer
    device = env.device

    S = SimpleNamespace(
        phase="init", k=0, ssr=10 ** 9, log_on=False,
        prev_vz=None, prev_thrust=None,
        t2_skip=0, t6_pending=False, t6_vel=None,
        resync_k=-1,
        jumps=[],  # (phase, k, ssr, dvz, F_prev, m_eff)
    )
    bidx = env._body_id[0] if isinstance(env._body_id, (list, tuple)) else int(env._body_id)

    print(f"[DIAG] mode={args_cli.mode} ctrl_mass={env._ctrl_mass:.4f} kg "
          f"max_thrust={env._max_thrust:.3f} N hover_thrust={env._ctrl_mass * G:.3f} N "
          f"view_masses={view.get_masses()[0].tolist()}")

    # -- instrument write_root_velocity_to_sim ------------------------------
    _orig_wrv = robot.write_root_velocity_to_sim

    def _wrv_wrapper(root_velocity, env_ids=None):
        pre = view.get_root_velocities()[0].clone()
        _orig_wrv(root_velocity, env_ids)
        post = view.get_root_velocities()[0].clone()
        S.t6_vel = root_velocity.clone()
        print(f"    [WRV k={S.k}] written_lin={fmt3(root_velocity[0, :3])} "
              f"physx_pre_lin={fmt3(pre[:3])} physx_post_lin={fmt3(post[:3])} "
              f"physx_post_ang={fmt3(post[3:])}")

    robot.write_root_velocity_to_sim = _wrv_wrapper

    # -- instrument _reset_idx ----------------------------------------------
    _orig_reset_idx = env._reset_idx

    def _reset_wrapper(env_ids):
        vpre = view.get_root_velocities()[0].clone()
        m_pre = view.get_masses()[0].sum()
        print(f"  [RESET phase={S.phase} k={S.k}] BEFORE _reset_idx: "
              f"physx_lin_vel={fmt3(vpre[:3])} m_view={float(m_pre):.4f} "
              f"composer_active={composer.active}")
        _orig_reset_idx(env_ids)
        vpost = view.get_root_velocities()[0].clone()
        m_post = view.get_masses()[0].sum()
        print(f"  [RESET k={S.k}] AFTER  _reset_idx: physx_lin_vel={fmt3(vpost[:3])} "
              f"m_view={float(m_post):.4f} composer_active={composer.active} "
              f"pos_z={float(robot.data.root_pos_w[0, 2]):.3f}")
        S.ssr = 0
        S.prev_vz = None
        S.prev_thrust = None
        if S.phase in ("t2", "t2first"):
            zero = torch.zeros(env.num_envs, 1, 3, device=device)
            composer.set_forces_and_torques(forces=zero, torques=zero, body_ids=env._body_id)
            S.t2_skip = env.cfg.decimation
            print(f"  [T2] wrench explicitly zeroed at reset; thrust suppressed for next {S.t2_skip} substeps")
        if S.phase == "t6":
            S.t6_pending = True

    env._reset_idx = _reset_wrapper

    # -- instrument _apply_action (runs once per physics substep) -----------
    _orig_apply = env._apply_action

    def _apply_wrapper():
        ssr = S.ssr
        if S.phase == "t6" and ssr == 1 and S.t6_pending:
            S.t6_pending = False
            print(f"    [T6 k={S.k}] re-writing reset root velocity after 1st post-reset substep")
            robot.write_root_velocity_to_sim(S.t6_vel.clone(), None)
        if S.phase == "resync":
            if S.k == S.resync_k:
                m = view.get_masses().clone()
                m[:, bidx] = 1.0865  # half the override mass; controller still sized for 2.1732
                view.set_masses(m, torch.arange(env.num_envs))
                print(f"    [RESYNC k={S.k}] set_masses body -> 1.0865 kg mid-hover "
                      f"(controller thrust still sized for 2.1732 kg; expected steady a=+9.81 m/s^2 "
                      f"=> dvz=+0.098/substep once the solver sees it)")
            elif S.k == S.resync_k + 5:
                m = view.get_masses().clone()
                m[:, bidx] = 2.17
                view.set_masses(m, torch.arange(env.num_envs))
                print(f"    [RESYNC k={S.k}] set_masses body -> 2.17 kg (restore)")
        if (S.phase == "t2" or S.phase == "t2first") and S.t2_skip > 0:
            S.t2_skip -= 1
            env._physics_tick += 1  # keep LPF tick bookkeeping aligned with stock code
            zero = torch.zeros(env.num_envs, 1, 3, device=device)
            composer.set_forces_and_torques(forces=zero, torques=zero, body_ids=env._body_id)
        else:
            _orig_apply()
        # thrust that will be applied during THIS substep's sim.step
        thrust = float(composer.composed_force_as_torch.reshape(-1, 3)[:, 2].sum())
        v = robot.data.root_lin_vel_w[0]
        w = robot.data.root_ang_vel_b[0]
        alt = float(robot.data.root_pos_w[0, 2])
        m_view = float(view.get_masses()[0].sum())
        vz = float(v[2])
        line = (f"[{S.phase}] k={S.k:4d} ssr={ssr if ssr < 10**8 else -1:4d} v={fmt3(v)} ")
        if S.prev_vz is not None:
            dvz = vz - S.prev_vz
            line += f"dvz={dvz:+8.3f} "
            if abs(dvz) > 0.5 and S.prev_thrust is not None and abs(dvz + G * DT) > 1e-6:
                m_eff = S.prev_thrust * DT / (dvz + G * DT)
                line += f"<<JUMP m_eff={S.prev_thrust:.2f}*{DT}/({dvz:+.3f}+{G * DT:.4f})={m_eff:.5f}kg "
                S.jumps.append((S.phase, S.k, ssr, dvz, S.prev_thrust, m_eff))
        else:
            line += "dvz=  (n/a) "
        line += f"alt={alt:7.3f} F_thr={thrust:7.3f} |w|={float(torch.linalg.norm(w)):6.3f} m_view={m_view:.4f}"
        if S.log_on:
            print(line)
        S.prev_vz = vz
        S.prev_thrust = thrust
        S.k += 1
        S.ssr += 1

    env._apply_action = _apply_wrapper

    # -- drivers -------------------------------------------------------------
    zero_action = torch.zeros(env.num_envs, env.cfg.action_space, device=device)

    def run_steps(n, label):
        for i in range(n):
            obs, rew, term, trunc, extras = env.step(zero_action)
            if bool(term[0]) or bool(trunc[0]):
                flags = [kk for kk, vv in env._done_flags.items() if bool(vv[0])]
                print(f"  [{label}] policy step {i}: episode ENDED naturally flags={flags} "
                      f"(auto-reset already ran inside step())")

    def force_auto_reset_next_step():
        # make the NEXT step() hit the time_out branch -> _reset_idx runs INSIDE step()
        env.episode_length_buf[:] = env.max_episode_length - 1

    n_log = args_cli.steps_per_cycle

    if args_cli.mode == "combo":
        # ---- phase t1m: manual env.reset() path (reset() -> write_data_to_sim + sim.forward)
        for cyc in range(2):
            S.phase = "t1m"
            S.log_on = True
            print(f"\n===== PHASE t1m cycle {cyc}: manual env.reset(), then {n_log} zero-action steps =====")
            env.reset()
            run_steps(n_log, "t1m")
            S.log_on = False

        # ---- phase t1a: auto-reset inside step() (production path)
        for cyc in range(2):
            S.phase = "t1a"
            print(f"\n===== PHASE t1a cycle {cyc}: forced timeout -> auto-reset inside step() =====")
            env.reset()          # clean starting point (unlogged)
            run_steps(2, "t1a")  # let the post-reset transient settle (unlogged)
            force_auto_reset_next_step()
            S.log_on = True
            print("--- step T (substeps below are PRE-reset; _reset_idx fires at the END of this step) ---")
            run_steps(1, "t1a")  # this step ends with the auto-reset
            print(f"--- steps T+1..T+{n_log} (post-auto-reset substeps) ---")
            run_steps(n_log, "t1a")
            S.log_on = False

        # ---- phase t2: wrench-zeroed reset
        S.phase = "t2"
        print("\n===== PHASE t2: auto-reset with wrench zeroed + thrust suppressed for 1st policy step =====")
        env.reset()
        run_steps(2, "t2")
        force_auto_reset_next_step()
        S.log_on = True
        print("--- step T (pre-reset substeps; reset at end of step) ---")
        run_steps(1, "t2")
        print("--- post-reset: substeps 0..9 have ZERO wrench; thrust resumes at ssr=10 ---")
        run_steps(n_log + 1, "t2")
        S.log_on = False

        # ---- phase t6: re-write velocity after 1st post-reset substep
        S.phase = "t6"
        print("\n===== PHASE t6: auto-reset, velocity re-written after 1st post-reset substep =====")
        env.reset()
        run_steps(2, "t6")
        force_auto_reset_next_step()
        S.log_on = True
        print("--- step T (pre-reset substeps; reset at end of step) ---")
        run_steps(1, "t6")
        print("--- post-reset substeps (velocity re-write occurs at ssr=1) ---")
        run_steps(n_log, "t6")
        S.log_on = False
    elif args_cli.mode == "noreset":
        # ---- phase nr: NO reset at all. First env.step() of the process directly.
        # If the kick appears here, the teleport/reset writes are NOT required —
        # the transient belongs to the first physics integration after the
        # __init__-time set_masses call.
        S.phase = "nr"
        S.log_on = True
        print("\n===== PHASE nr: NO env.reset() — stepping directly from the default spawn state =====")
        print(f"(default root state z={float(robot.data.root_pos_w[0, 2]):.3f}; wrench is armed by the "
              f"first _apply_action, same as post-reset)")
        run_steps(2, "nr")
        S.log_on = False

        # ---- phase resync: mid-hover second set_masses with a CHANGED value.
        # Measures the latency with which the solver picks up set_masses:
        # dvz on the very next substep tells whether the first integration
        # after the call uses the OLD or NEW mass.
        S.phase = "settle"
        env.reset()
        run_steps(3, "settle")  # restore clean hover (unlogged)
        S.phase = "resync"
        S.resync_k = S.k + 12
        S.log_on = True
        print(f"\n===== PHASE resync: mid-hover set_masses 2.17 -> 1.0865 at k={S.resync_k}, "
              f"restore at k={S.resync_k + 5} =====")
        run_steps(3, "resync")
        S.log_on = False

    elif args_cli.mode == "t2first":
        # ---- wrench-zeroed FIRST reset + first policy step of the process.
        # The one-and-only kick window (launch combo showed it fires only on the
        # process's first physics substep) is crossed with ZERO applied wrench.
        # H1 prediction: no kick at all — pure free fall (dvz = -g*dt = -0.098),
        # and after thrust resumes at ssr=10 the solver mass is already synced
        # (flushed by the zero-force sim.steps), so no jump then either.
        # If instead the ~8 m/s jump still appears with F=0 -> solver-internal
        # impulse, H1 refuted. If a jump appears right after thrust resumes
        # (m_eff ~ native) -> desync persists until first NONZERO force.
        S.phase = "t2first"
        S.log_on = True
        print("\n===== PHASE t2first: FIRST reset+step of the process with wrench zeroed "
              "for substeps 0..9, thrust resumes at ssr=10 =====")
        env.reset()
        run_steps(3, "t2first")
        S.log_on = False

        # follow-up ordinary reset cycle (should be clean)
        S.phase = "t2f_after"
        print("\n===== PHASE t2f_after: ordinary manual reset afterwards =====")
        S.log_on = True
        env.reset()
        run_steps(2, "t2f_after")
        S.log_on = False

    else:
        # single-mode runs (t3 / t4m / t4i): manual + auto reset cycles
        S.phase = args_cli.mode + "m"
        S.log_on = True
        print(f"\n===== PHASE {S.phase}: manual env.reset(), then {n_log} zero-action steps =====")
        env.reset()
        run_steps(n_log, S.phase)
        S.log_on = False

        S.phase = args_cli.mode + "a"
        print(f"\n===== PHASE {S.phase}: forced timeout -> auto-reset inside step() =====")
        env.reset()
        run_steps(2, S.phase)
        force_auto_reset_next_step()
        S.log_on = True
        print("--- step T (pre-reset substeps; reset at end of step) ---")
        run_steps(1, S.phase)
        print("--- post-reset substeps ---")
        run_steps(n_log, S.phase)
        S.log_on = False

    print("\n===== JUMP SUMMARY (|dvz| > 0.5 m/s between consecutive substeps) =====")
    if not S.jumps:
        print("NO velocity jumps recorded.")
    for phase, k, ssr, dvz, f_prev, m_eff in S.jumps:
        print(f"  phase={phase} k={k} ssr={ssr} dvz={dvz:+.3f} m/s F_prev={f_prev:.3f} N "
              f"m_eff={m_eff:.5f} kg  (native~0.027-0.033, override=2.173)")

    env.close()


if __name__ == "__main__":
    try:
        main()
    except Exception:
        print("=== DIAG: FAIL — exception ===")
        traceback.print_exc()
        simulation_app.close()
        sys.exit(1)
    simulation_app.close()
