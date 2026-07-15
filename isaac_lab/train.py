"""Train Isaac-DroneBombard-Direct-v0 with rsl_rl PPO — phased curriculum.

Mirrors Isaac Lab v2.3.2's stock rsl_rl train script API
(RslRlVecEnvWrapper(clip_actions=...), OnPolicyRunner, single learn call)
while building the env cfg directly (the path proven by
verify_one_episode.py) and adding a SIGTERM preempt-save for Spot VMs.

Two modes (see the image's 3-stage curriculum / exp_015 note):

  * Single-phase (``--phase N``): trains one curriculum phase in-process.
    Phase 1 = approach/nominal (stationary target), Phase 2 = CCIP+residual
    with drag/wind domain randomization (stationary target), Phase 3 = moving
    target with lead/intercept. ``--resume`` warm-starts from a checkpoint;
    since the action space is a fixed 6-dim across all phases the rsl_rl
    network is identical phase-to-phase, so a Phase-(N-1) ``model_final.pt``
    loads losslessly into Phase N.

  * Orchestrator (``--phases 1,2,3``): runs each phase sequentially as its own
    subprocess (Isaac Sim is one-sim-per-process), chaining each phase's
    ``model_final.pt`` into the next phase's ``--resume``.

Usage (inside the isaac-lab container, on the L4 Spot VM):
    # full sequential curriculum
    ./isaaclab.sh -p train.py --phases 1,2,3 --headless --num_envs 2048 \\
        --phase_iterations 3000,2000,2000
    # single phase (e.g. resume Phase 2 from a Phase-1 checkpoint)
    ./isaaclab.sh -p train.py --phase 2 --headless --num_envs 2048 \\
        --resume /workspace/logs/.../phase1/model_final.pt
    # tiny smoke
    ./isaaclab.sh -p train.py --phase 1 --headless --num_envs 16 --max_iterations 2
"""

import argparse
import os
import subprocess
import sys
from datetime import datetime

from isaaclab.app import AppLauncher

parser = argparse.ArgumentParser(description="Train Isaac-DroneBombard-Direct-v0 with rsl_rl PPO (phased).")
parser.add_argument("--task", type=str, default="Isaac-DroneBombard-Direct-v0")
parser.add_argument("--num_envs", type=int, default=2048)
parser.add_argument("--max_iterations", type=int, default=None)
parser.add_argument("--seed", type=int, default=42)
parser.add_argument("--resume", type=str, default=None, help="Path to a checkpoint (.pt) to warm-start/resume from.")
parser.add_argument("--log_root", type=str, default="/workspace/logs/isaac_lab/drone_bombard")
parser.add_argument("--run_name", type=str, default="")
parser.add_argument("--wandb_project", type=str, default="drone-bombard-isaac")
parser.add_argument("--logger", type=str, default="wandb", choices=["wandb", "tensorboard"])
# --- phased curriculum ---
parser.add_argument("--phase", type=int, default=None, choices=[1, 2, 3],
                    help="Single curriculum phase to train in-process (1/2/3).")
parser.add_argument("--phases", type=str, default="",
                    help="Comma-separated phase list to run sequentially as subprocesses, e.g. '1,2,3'.")
parser.add_argument("--phase_iterations", type=str, default="",
                    help="Comma-separated max_iterations per phase (aligns with --phases), e.g. '3000,2000,2000'.")
parser.add_argument("--final_out", type=str, default=None,
                    help="If set (single-phase mode), also save the final model to this exact path "
                         "(used by the orchestrator to chain warm-starts).")
parser.add_argument("--w_aim", type=float, default=None,
                    help="Dense CCIP aim-error reward weight (reward.w_aim). Default None keeps the "
                         "cfg value (0.0 = term off, exp_014 Phase-1 reward parity). exp_017 Stage A.")
parser.add_argument("--aim_reward_scale", type=float, default=None,
                    help="tanh knee (metres) for the aim-error reward (reward.aim_reward_scale).")
parser.add_argument("--release_terminal", action="store_true",
                    help="Phase 1 Stage B (exp_018): the scripted CCIP referee's fire event ends the "
                         "episode as success (replaces d_xy proximity success; failure gates unchanged).")
parser.add_argument("--v11_test", action="store_true",
                    help="Isaac-v11 relaxed test env: single integrated phase, fixed marker ahead in "
                         "the cruise direction, policy drop_signal + release envelope, nominal physics. "
                         "Uses DroneBombardV11Cfg + Isaac-DroneBombard-V11-Direct-v0 (ignores --phase).")
parser.add_argument("--v12", action="store_true",
                    help="v11 first expansion: random marker inside a 5 m disk around the 20 m point "
                         "(drone still cruises +X, steers to it). DroneBombardV12Cfg + "
                         "Isaac-DroneBombard-V12-Direct-v0 (ignores --phase).")
AppLauncher.add_app_launcher_args(parser)
args_cli = parser.parse_args()


def _parse_int_list(s: str) -> list[int]:
    return [int(x) for x in s.split(",") if x.strip() != ""]


def run_orchestrator(phases: list[int]) -> int:
    """Run each phase as its own subprocess, chaining model_final.pt -> next
    phase's --resume. Runs BEFORE the Isaac Sim app is launched (this process
    never touches the sim; each child owns its own sim)."""
    phase_iters = _parse_int_list(args_cli.phase_iterations)
    if phase_iters and len(phase_iters) != len(phases):
        print(f"[Orchestrator] ERROR: --phase_iterations has {len(phase_iters)} entries "
              f"but --phases has {len(phases)}.", flush=True)
        return 2

    os.makedirs(args_cli.log_root, exist_ok=True)
    prev_ckpt = args_cli.resume  # allow an external warm-start into the first phase
    base_run = args_cli.run_name or datetime.now().strftime("%Y-%m-%d_%H-%M-%S")

    for i, ph in enumerate(phases):
        final_out = os.path.join(args_cli.log_root, f"{base_run}_phase{ph}_final.pt")
        cmd = [
            sys.executable, os.path.abspath(__file__),
            "--phase", str(ph),
            "--task", args_cli.task,
            "--num_envs", str(args_cli.num_envs),
            "--seed", str(args_cli.seed),
            "--log_root", args_cli.log_root,
            "--run_name", f"{base_run}_phase{ph}",
            "--wandb_project", args_cli.wandb_project,
            "--logger", args_cli.logger,
            "--final_out", final_out,
        ]
        if args_cli.headless:
            cmd.append("--headless")
        if args_cli.device is not None:
            cmd += ["--device", str(args_cli.device)]
        if phase_iters:
            cmd += ["--max_iterations", str(phase_iters[i])]
        elif args_cli.max_iterations is not None:
            cmd += ["--max_iterations", str(args_cli.max_iterations)]
        if args_cli.w_aim is not None:
            cmd += ["--w_aim", str(args_cli.w_aim)]
        if args_cli.aim_reward_scale is not None:
            cmd += ["--aim_reward_scale", str(args_cli.aim_reward_scale)]
        if args_cli.release_terminal:
            cmd += ["--release_terminal"]
        if prev_ckpt:
            cmd += ["--resume", prev_ckpt]

        print(f"\n[Orchestrator] === Phase {ph} ({i + 1}/{len(phases)}) ===", flush=True)
        print(f"[Orchestrator] warm-start: {prev_ckpt or 'from scratch'}", flush=True)
        print(f"[Orchestrator] cmd: {' '.join(cmd)}", flush=True)
        ret = subprocess.call(cmd)
        if ret != 0:
            print(f"[Orchestrator] Phase {ph} failed (exit {ret}); aborting curriculum.", flush=True)
            return ret
        if not os.path.exists(final_out):
            print(f"[Orchestrator] Phase {ph} produced no checkpoint at {final_out}; aborting.", flush=True)
            return 1
        prev_ckpt = final_out  # chain into the next phase's warm-start

    print(f"\n[Orchestrator] Curriculum complete. Final checkpoint: {prev_ckpt}", flush=True)
    return 0


# ---------------------------------------------------------------------------
# Orchestrator mode: dispatch subprocesses and exit BEFORE launching the sim.
# ---------------------------------------------------------------------------
_phases = _parse_int_list(args_cli.phases)
if _phases:
    sys.exit(run_orchestrator(_phases))

# ---------------------------------------------------------------------------
# Single-phase mode: launch the sim and train one phase in-process.
# ---------------------------------------------------------------------------
app_launcher = AppLauncher(args_cli)
simulation_app = app_launcher.app

import gymnasium as gym  # noqa: E402
import torch  # noqa: E402
from rsl_rl.runners import OnPolicyRunner  # noqa: E402

from isaaclab_rl.rsl_rl import RslRlVecEnvWrapper  # noqa: E402

import drone_bombard  # noqa: F401,E402 - registers the task
from drone_bombard.drone_bombard_env import DroneBombardEnvCfg  # noqa: E402
from drone_bombard.v11_env import DroneBombardV11Cfg, DroneBombardV12Cfg  # noqa: E402
from drone_bombard.agents.rsl_rl_ppo_cfg import DroneBombardPPORunnerCfg  # noqa: E402

torch.backends.cuda.matmul.allow_tf32 = True
torch.backends.cudnn.allow_tf32 = True


def main():
    phase = args_cli.phase if args_cli.phase is not None else 1

    # --- env cfg (built directly — the path proven by verify_one_episode.py) ---
    task = args_cli.task
    if args_cli.v12:
        # v11 first expansion: random marker inside a disk (marker_random=True).
        task = "Isaac-DroneBombard-V12-Direct-v0"
        env_cfg = DroneBombardV12Cfg()
    elif args_cli.v11_test:
        # Isaac-v11 relaxed test: separate env class + cfg, single integrated
        # phase (no --phase, no residual/DR/moving/vision — all kept inert).
        task = "Isaac-DroneBombard-V11-Direct-v0"
        env_cfg = DroneBombardV11Cfg()
    else:
        env_cfg = DroneBombardEnvCfg()
        env_cfg.phase = phase  # env __init__ re-derives residual/DR/moving flags from this
        if args_cli.w_aim is not None:
            env_cfg.reward.w_aim = args_cli.w_aim
        if args_cli.aim_reward_scale is not None:
            env_cfg.reward.aim_reward_scale = args_cli.aim_reward_scale
        if args_cli.release_terminal:
            env_cfg.release_terminal = True
    env_cfg.scene.num_envs = args_cli.num_envs
    env_cfg.sim.device = args_cli.device if args_cli.device is not None else "cuda:0"
    env_cfg.seed = args_cli.seed

    # --- agent cfg ---
    agent_cfg: DroneBombardPPORunnerCfg = DroneBombardPPORunnerCfg()
    agent_cfg.seed = args_cli.seed
    if args_cli.max_iterations is not None:
        agent_cfg.max_iterations = args_cli.max_iterations
    agent_cfg.logger = args_cli.logger
    agent_cfg.wandb_project = args_cli.wandb_project
    agent_cfg.run_name = args_cli.run_name if args_cli.run_name else (
        "v12" if args_cli.v12 else "v11" if args_cli.v11_test else f"phase{phase}")

    log_root = os.path.abspath(os.path.join(args_cli.log_root, agent_cfg.experiment_name))
    log_dir = datetime.now().strftime("%Y-%m-%d_%H-%M-%S")
    if agent_cfg.run_name:
        log_dir += f"_{agent_cfg.run_name}"
    log_dir = os.path.join(log_root, log_dir)
    os.makedirs(log_dir, exist_ok=True)
    print(f"[INFO] Phase {phase} — logging experiment to: {log_dir}")

    # --- build env + rsl_rl runner (stock v2.3.2 API) ---
    env = gym.make(task, cfg=env_cfg)
    env = RslRlVecEnvWrapper(env, clip_actions=agent_cfg.clip_actions)
    runner = OnPolicyRunner(env, agent_cfg.to_dict(), log_dir=log_dir, device=agent_cfg.device)

    if args_cli.resume:
        # Warm-start: the 6-dim action space is identical across phases, so a
        # Phase-(N-1) checkpoint loads losslessly into Phase N.
        print(f"[INFO] Warm-starting from checkpoint: {args_cli.resume}")
        runner.load(args_cli.resume)

    # --- Spot-VM preemption: save on SIGTERM before the box dies ---
    import signal

    def _emergency_save(signum, frame):
        path = os.path.join(log_dir, "model_preempt.pt")
        try:
            runner.save(path)
            print(f"[Preempt] Emergency checkpoint saved: {path}", flush=True)
        finally:
            try:
                import wandb
                if wandb.run is not None:
                    wandb.finish()
            except Exception:  # noqa: BLE001
                pass
        sys.exit(0)

    signal.signal(signal.SIGTERM, _emergency_save)

    # --- train (single call; rsl_rl saves every save_interval to log_dir) ---
    runner.learn(num_learning_iterations=agent_cfg.max_iterations, init_at_random_ep_len=True)

    final_path = os.path.join(log_dir, "model_final.pt")
    runner.save(final_path)
    print(f"[Done] Phase {phase} final model saved to {final_path}")
    if args_cli.final_out:
        # Also save to the orchestrator's chained path so the next phase can
        # warm-start from a stable, known location.
        os.makedirs(os.path.dirname(os.path.abspath(args_cli.final_out)), exist_ok=True)
        runner.save(args_cli.final_out)
        print(f"[Done] Phase {phase} final model also saved to {args_cli.final_out}")
    env.close()


if __name__ == "__main__":
    main()
    simulation_app.close()
