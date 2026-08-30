"""Rule-based (NO learning) release baselines — Table 1 arms T0-T3 (P0-6).

These answer the question the paper cannot currently answer: *what does RL
actually buy over classical release logic?* Every arm flies the SAME env, the
SAME release envelope, and is scored by the SAME harness as the learned arms
(``eval_harness.py``), so the rows are directly comparable.

All four share one scripted approach controller (blind cruise -> steer at the
perceived target -> descend into the release altitude band) and differ ONLY in
the release rule:

  T0 ``hover``  — fly over the target, stop, drop. The degenerate vision-drop
                  strategy from the literature (detect, hover-align, release
                  from rest). Should be accurate and SLOW: it exists to make
                  the delivery-time axis mean something.
  T1 ``ccip``   — fire at the first admissible instant, i.e. as soon as the
                  CCIP impact-point error falls inside the release radius.
                  The textbook continuously-computed-impact-point rule.
  T2 ``argmin`` — port of AeroThrow's online release-timing reassessment:
                  propagate the vehicle state over a short horizon, evaluate
                  the predicted impact error at every step of it, and fire when
                  the argmin collapses to NOW. Strongest non-privileged rule.
  T3 ``oracle`` — T2 plus a PRIVILEGED residual: the payload's true fall is
                  integrated forward under the env's own drag ODE with the
                  env's own per-episode wind and ballistic coefficient, and the
                  offset from the nominal CCIP is emitted on the residual
                  action channel — i.e. the correction the learned residual is
                  trying to discover. This is the classical upper bound for the
                  residual mechanism, and it saturates exactly like the learned
                  one when the drift exceeds +-residual_scale.

                  Until 2026-08-26 this drift came from ``ballistic_impact``'s
                  analytic wind term, which assumes the payload adopts the full
                  wind speed instantly. It does not (tau ~ 2.5 s vs a ~1 s
                  fall), so the "oracle" over-corrected ~5x, flew that far
                  upwind of the marker, and scored BELOW the do-nothing hover
                  arm — an upper bound underneath its own baseline. Any Table 1
                  T3 number printed before that date is void.

Fairness notes (state them in the paper):
  * T0-T2 use only quantities the policy also observes: the PERCEIVED
    (reveal-gated, pixel-quantized) target and the vehicle's own state.
  * T3 additionally reads the true wind/drag and is labelled an oracle.
  * The success criterion is the TRUE marker for every arm, as for the policy.
  * No arm gets a widened release envelope: the same gate (impact radius,
    altitude band, speed / vertical-speed / tilt / rate limits, payload
    attached) applies to all, learned and rule-based alike.

Usage (inside isaac-verify), 200 paired episodes per arm:

    /workspace/isaaclab/isaaclab.sh -p baseline_drop.py --arm argmin \\
        --task Isaac-DroneBombard-V20-Direct-v0 --episodes 200 --num_envs 200 \\
        --seed 1000 --out-json /workspace/eval/T2_argmin.json
"""

import argparse

from isaaclab.app import AppLauncher

parser = argparse.ArgumentParser(description="Rule-based release baselines (T0-T3).")
parser.add_argument("--arm", type=str, default="argmin",
                    choices=["hover", "ccip", "argmin", "oracle"],
                    help="Release rule: T0 hover / T1 ccip / T2 argmin / T3 oracle.")
parser.add_argument("--task", type=str, default="Isaac-DroneBombard-Task-v0")
parser.add_argument("--dr_scale", type=float, default=None,
                    help="A-group DR strength. Must match the learned arm being compared against.")
parser.add_argument("--release_10hz", action="store_true",
                    help="Resolve the release on the 10 Hz policy grid instead of at "
                         "physics rate (the ablation arm for release-timing resolution).")
parser.add_argument("--episodes", type=int, default=200)
parser.add_argument("--num_envs", type=int, default=200,
                    help="Paired evaluation needs num_envs >= episodes.")
parser.add_argument("--seed", type=int, default=1000,
                    help="Eval seed. Keep DISJOINT from the training seeds (42/43/44).")
parser.add_argument("--unpaired", action="store_true",
                    help="Score every episode instead of one per env slot (loses pairing).")
parser.add_argument("--out-json", type=str, default=None)
# --- approach controller knobs (shared by every arm) ---
parser.add_argument("--approach_speed", type=float, default=4.0, help="m/s cap on the run-in.")
parser.add_argument("--release_alt", type=float, default=6.0,
                    help="Target altitude for the run-in (must sit inside the envelope band).")
parser.add_argument("--kp_xy", type=float, default=0.6, help="Approach P gain (1/s).")
parser.add_argument("--pass_speed", type=float, default=None,
                    help="Override the flight profile: fly a CONSTANT-SPEED delivery pass at "
                         "this speed instead of whatever the arm defaults to. Defaults are set "
                         "per arm (see FLIGHT PROFILE below); pass 0 to force P-control.")
parser.add_argument("--video", type=str, default=None,
                    help="Record the run to MP4 in this directory (offscreen -- no X display and "
                         "no window needed). Requires --enable_cameras; implies --show. "
                         "Isaac Sim cannot create a Vulkan surface on a TigerVNC X server, so "
                         "offscreen rendering is the reliable way to actually SEE a drop on a "
                         "headless VM. Never use on a measurement run: rendering costs time.")
parser.add_argument("--video_length", type=int, default=1200,
                    help="Frames to record (policy steps). 1200 @ 10 Hz = 120 s of sim time.")
parser.add_argument("--show", action="store_true",
                    help="Viewing mode for the livestream/GUI: turn on the target beacon and "
                         "payload markers and lock a chase camera on the PAYLOAD, so the "
                         "release and the fall read clearly. Mirrors play.py --show. Costs "
                         "render time -- never use it for measurement runs.")
parser.add_argument("--p_control", action="store_true",
                    help="Force the legacy P-control (decelerate-onto-target) profile on every "
                         "arm, reproducing the pre-2026-08-27 baselines.")
parser.add_argument("--release_descent", type=float, default=0.0,
                    metavar="RATE",
                    help="Dive-and-release: hold a constant descent of RATE m/s while above "
                         "--release_alt instead of levelling off, so the arm fires WHILE "
                         "descending. Wind drift scales with t_fall^2 and a descending release "
                         "shortens the fall, so a level baseline is handed a systematically "
                         "longer fall than a policy that dives. Needed to compare geometry-for-"
                         "geometry against L0 (which chose alt 3.53 m, vz -1.17 m/s).")
parser.add_argument("--kp_z", type=float, default=0.8, help="Altitude P gain (1/s).")
parser.add_argument("--horizon", type=int, default=12,
                    help="argmin/oracle: predicted-impact horizon in policy steps.")
parser.add_argument("--hover_radius", type=float, default=0.8,
                    help="hover arm: fire once inside this horizontal distance of the perceived "
                         "target. Cannot be tighter than the perception cell (pixel_cell_k*slant, "
                         "~0.7 m at the release altitude) or the arm never fires at all.")
parser.add_argument("--hover_speed", type=float, default=0.6,
                    help="hover arm: fire only below this horizontal speed (the 'stopped' test).")
parser.add_argument("--marker_dist", type=float, nargs=2, default=None,
                    metavar=("LO", "HI"),
                    help="Override handoff.marker_dist_range. Training draws [18, 22]; pass a "
                         "disjoint band (e.g. 26 30) for the UNSEEN-range evaluation condition.")
# --- moving target (must mirror training) ---
parser.add_argument("--moving_target", action="store_true")
parser.add_argument("--target_motion", type=str, default=None, choices=["gm", "cv", "ca", "ct"])
parser.add_argument("--no_handoff_dr", action="store_true")
parser.add_argument("--no_dyn_dr", action="store_true")
AppLauncher.add_app_launcher_args(parser)
args_cli = parser.parse_args()

app_launcher = AppLauncher(args_cli)
simulation_app = app_launcher.app

import gymnasium as gym  # noqa: E402
import torch  # noqa: E402
from isaaclab_tasks.utils import parse_env_cfg  # noqa: E402

import drone_bombard  # noqa: F401,E402
import eval_harness  # noqa: E402
from drone_bombard.math_utils import (  # noqa: E402
    ballistic_impact,
    integrate_payload_impact,
    predict_impact_nominal,
)


def _perceived(u: torch.nn.Module, pos_xy: torch.Tensor) -> torch.Tensor:
    """The target the POLICY would see: the pixel-quantized / reveal-gated
    estimate when the env models perception, the true marker otherwise."""
    p = getattr(u, "_perceived_target_xy", None)
    if p is None or not bool(u.cfg.perception.pixel_quantize):
        return u._target_xy
    return p


def _predict_series(u, pos_xy, vel_xy, alt, vz, steps, dt):
    """Constant-velocity roll-out of the CCIP prediction over a short horizon.

    The honest analogue of AeroThrow's NMPC-horizon propagation: we have no
    NMPC, so the vehicle is extrapolated at its current velocity (the run-in is
    near-constant-velocity by construction). Returns [N, steps+1] predicted
    impact points and altitudes."""
    out = []
    for k in range(steps + 1):
        t = k * dt
        p_k = pos_xy + vel_xy * t
        a_k = (alt + vz * t).clamp(min=0.05)
        # vz is carried through the roll-out (constant-velocity extrapolation),
        # so the CCIP at each horizon step uses the descent rate the vehicle
        # would actually have there — see math_utils.ballistic_impact HISTORY.
        out.append(u._nominal_impact(p_k, vel_xy, vz, a_k))
    return torch.stack(out, dim=1)  # [N, steps+1, 2]


class BaselineController:
    """Scripted approach + one of the four release rules."""

    def __init__(self, u, arm: str):
        self.u, self.arm = u, arm
        self.dt = u.cfg.sim.dt * u.cfg.decimation
        self.act_dim = int(u.cfg.action_space)
        self.res_scale = float(u.cfg.residual.scale)
        self.has_residual = self.act_dim >= 7 and bool(u.cfg.residual.enabled)
        self.gate_radius = float(u.cfg.release.radius)

        # FLIGHT PROFILE -- part of what each arm IS, not a tuning knob.
        #
        #   T0 hover  -> P-control. Decelerating onto the target and stopping is
        #                the whole point of this arm: it is the degenerate
        #                "place" strategy the delivery-time axis exists to expose.
        #   T1/T2/T3  -> constant-speed pass. These are RELEASE-TIMING rules, and
        #                a timing rule is only meaningful on a trajectory that
        #                actually flies through; the computer picks the instant.
        #
        # Why this is not optional (measured 2026-08-27): under P-control the
        # commanded speed is proportional to the remaining range, so speed and
        # range go to zero TOGETHER and the predicted impact meets the target
        # only overhead at a standstill. Every arm therefore released at walking
        # pace -- T2 at 0.76 m/s against a 4 m/s cruise -- which is a PLACE, not
        # a throw. That matters because every error the learned residual exists
        # to remove (release-delay drift, self-velocity drag, wind carry) scales
        # with release speed and is identically zero at a standstill. Comparing
        # a learned throw against scripted places would beat our own approach
        # controller rather than the published release rule.
        #
        # Note this is NOT a gain-tuning problem: v = kp * range makes both
        # vanish together for ANY kp, so no P gain produces a throw.
        if args_cli.p_control:
            self.pass_speed = 0.0
        elif args_cli.pass_speed is not None:
            self.pass_speed = float(args_cli.pass_speed)
        else:
            self.pass_speed = 0.0 if self.arm == "hover" else float(args_cli.approach_speed)

    # -------- privileged residual (T3 only) --------
    def _oracle_residual(self, pos_xy, vel_xy, alt, vz):
        """Delegates to ``DroneBombardEnv.oracle_impact_residual`` — the single
        definition, shared with ``play.py --oracle_residual``. Keeping a second
        copy here is how the predictor and the plant drifted apart before
        (Rule 30/31)."""
        return self.u.oracle_impact_residual(pos_xy, vel_xy, alt, vz, self.res_scale)

    # -------- release rules --------
    def _want_drop(self, pos_xy, vel_xy, alt, vz, aim_xy, residual):
        u, dc = self.u, self.u.cfg.drop
        speed_xy = torch.linalg.norm(vel_xy, dim=-1)

        def err_of(impact):
            return torch.linalg.norm(impact + residual * self.res_scale - aim_xy, dim=-1)

        if self.arm == "hover":
            # T0: over the target and stopped. CCIP at rest reduces to "am I
            # above it", so this is the classical detect -> hover -> drop.
            d_xy = torch.linalg.norm(aim_xy - pos_xy, dim=-1)
            return ((d_xy <= args_cli.hover_radius) & (speed_xy <= args_cli.hover_speed)
                    & (vz.abs() <= 1.0))

        nominal = u._nominal_impact(pos_xy, vel_xy, vz, alt)
        if self.arm == "ccip":
            # T1: first admissible instant.
            return err_of(nominal) <= self.gate_radius

        # T2/T3: fire when the predicted-impact error is at its horizon minimum
        # NOW (AeroThrow Alg. 1) and that minimum is admissible.
        series = _predict_series(u, pos_xy, vel_xy, alt, vz, args_cli.horizon, self.dt)
        errs = torch.linalg.norm(
            series + (residual * self.res_scale).unsqueeze(1) - aim_xy.unsqueeze(1), dim=-1)
        return (errs.argmin(dim=1) == 0) & (errs[:, 0] <= self.gate_radius)

    # -------- one policy step --------
    def __call__(self, obs, u):
        a = u.cfg.action
        pos = u._robot.data.root_pos_w - u.scene.env_origins
        vel = u._robot.data.root_lin_vel_w
        pos_xy, alt, vel_xy, vz = pos[:, :2], pos[:, 2], vel[:, :2], vel[:, 2]

        aim_xy = _perceived(u, pos_xy)
        detected = getattr(u, "_detected", None)
        if detected is None:
            detected = torch.ones_like(alt, dtype=torch.bool)

        residual = (self._oracle_residual(pos_xy, vel_xy, alt, vz) if self.arm == "oracle"
                    else torch.zeros(u.num_envs, 2, device=u.device))

        # --- approach: P-control toward the target, capped at approach_speed.
        # Before acquisition the marker position is not observable, so hold the
        # handoff velocity (blind cruise) exactly as the policy must.
        steer_xy = aim_xy
        if self.arm == "oracle":
            # A wind-corrected RELEASE RULE is useless if the vehicle still flies
            # straight at the marker: hovering over the marker puts the REAL
            # impact a full drift downwind of it, so the gate (which now tests
            # the real impact) never opens. The oracle therefore also flies the
            # upwind offset — target - drift — so that the real impact and the
            # marker coincide. Both halves are needed; either alone is worse
            # than doing nothing.
            steer_xy = aim_xy - residual * self.res_scale
        to_target = steer_xy - pos_xy
        if self.pass_speed > 0.0:
            # Constant-speed delivery pass: hold speed along the bearing to the
            # aim point and let the release rule pick the instant. This is what a
            # CARP/CCIP pass actually is -- you do not decelerate onto the target,
            # you fly through and the computer picks the moment.
            brg = to_target / torch.linalg.norm(to_target, dim=-1, keepdim=True).clamp(min=1e-6)
            v_des = brg * self.pass_speed
        else:
            v_des = torch.clamp(args_cli.kp_xy * to_target,
                                -args_cli.approach_speed, args_cli.approach_speed)
        blind = ~detected
        if bool(blind.any()):
            v_now = vel_xy / torch.linalg.norm(vel_xy, dim=-1, keepdim=True).clamp(min=1e-6)
            v_des = torch.where(blind.unsqueeze(-1), v_now * args_cli.approach_speed, v_des)
        vz_des = torch.clamp(args_cli.kp_z * (args_cli.release_alt - alt), -2.0, 2.0)
        if args_cli.release_descent > 0.0:
            # Above the target altitude, command the descent rate outright; the P
            # law only takes over as a floor once the band is reached. The arm is
            # therefore still moving down when the release rule fires.
            vz_des = torch.where(alt > args_cli.release_alt,
                                 torch.full_like(alt, -abs(args_cli.release_descent)),
                                 vz_des)

        drop = self._want_drop(pos_xy, vel_xy, alt, vz, aim_xy, residual)

        act = torch.zeros(u.num_envs, self.act_dim, device=u.device)
        act[:, 0] = torch.clamp(v_des[:, 0] / a.vx_scale, -1.0, 1.0)
        act[:, 1] = torch.clamp(v_des[:, 1] / a.vy_scale, -1.0, 1.0)
        act[:, 2] = torch.clamp(vz_des / a.vz_scale, -1.0, 1.0)
        act[:, 3] = 0.0
        if self.act_dim >= 5:
            act[:, 4] = torch.where(drop, torch.ones_like(alt), -torch.ones_like(alt))
        if self.has_residual:
            act[:, 5:7] = residual
        return act


ARM_NAMES = {"hover": "T0_hover_drop", "ccip": "T1_ccip_threshold",
             "argmin": "T2_predictive_argmin", "oracle": "T3_model_oracle_residual"}  # not "wind": drag on the payload's OWN velocity dominates


def main():
    env_cfg = parse_env_cfg(args_cli.task, num_envs=args_cli.num_envs)
    env_cfg.seed = args_cli.seed
    if args_cli.moving_target:
        env_cfg.moving_target_force = True
    if args_cli.target_motion is not None:
        env_cfg.phase_cfg.target_motion_model = args_cli.target_motion
    if args_cli.no_handoff_dr:
        env_cfg.handoff.randomize = False
    if args_cli.marker_dist is not None:
        env_cfg.handoff.marker_dist_range = tuple(args_cli.marker_dist)
    if args_cli.no_dyn_dr:
        env_cfg.dyn_dr.enabled = False
    if args_cli.release_10hz:
        env_cfg.release.decide_at_physics_rate = False
    if args_cli.dr_scale is not None:
        env_cfg.model_err.scale = args_cli.dr_scale
    if args_cli.show or args_cli.video:
        # Same viewing setup play.py uses. The camera tracks the PAYLOAD rather
        # than the drone: while carried it rides underneath (so the drone is in
        # frame anyway), and after release it follows the payload down to the
        # ground, which is the part worth watching.
        env_cfg.show_markers = True
        env_cfg.viewer.origin_type = "asset_root"
        env_cfg.viewer.asset_name = "payload"
        env_cfg.viewer.env_index = 0
        env_cfg.viewer.eye = (-2.5, -2.0, 1.1)
        env_cfg.viewer.lookat = (0.0, 0.0, 0.0)

    env_cfg.__post_init__()

    env = gym.make(args_cli.task, cfg=env_cfg,
                   render_mode="rgb_array" if args_cli.video else None)
    if args_cli.video:
        import os as _os
        _os.makedirs(args_cli.video, exist_ok=True)
        env = gym.wrappers.RecordVideo(
            env, video_folder=args_cli.video, step_trigger=lambda step: step == 0,
            video_length=args_cli.video_length, disable_logger=True)
        print(f"[baseline] recording {args_cli.video_length} frames -> {args_cli.video}", flush=True)
    u = env.unwrapped
    ctrl = BaselineController(u, args_cli.arm)
    print(f"[baseline] arm={ARM_NAMES[args_cli.arm]} task={args_cli.task} seed={args_cli.seed} "
          f"profile={'p_control' if ctrl.pass_speed <= 0.0 else f'pass@{ctrl.pass_speed:.1f}m/s'} "
          f"gate_radius={ctrl.gate_radius:.2f} residual_scale={ctrl.res_scale:.2f} "
          f"{'(PRIVILEGED wind/drag)' if args_cli.arm == 'oracle' else ''}", flush=True)

    eval_harness.run_and_report(
        env, ctrl,
        name=ARM_NAMES[args_cli.arm],
        episodes=args_cli.episodes,
        paired=not args_cli.unpaired,
        meta={"task": args_cli.task, "seed": args_cli.seed, "learned": False,
              "privileged": args_cli.arm == "oracle",
              "approach_speed": args_cli.approach_speed,
              "release_alt": args_cli.release_alt, "horizon": args_cli.horizon,
              "pass_speed": ctrl.pass_speed,
              "flight_profile": "p_control" if ctrl.pass_speed <= 0.0 else "constant_speed_pass",
              "hover_radius": args_cli.hover_radius, "hover_speed": args_cli.hover_speed},
        out_json=args_cli.out_json,
    )
    env.close()


if __name__ == "__main__":
    main()
    simulation_app.close()
