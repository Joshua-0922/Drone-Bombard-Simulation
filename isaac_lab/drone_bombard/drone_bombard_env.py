"""Isaac Lab DirectRLEnv port of the Gazebo+PX4+ROS2 drone-bombard RL task.

Ports the terminal visual-servoing task learned by SAC on Gazebo Harmonic +
PX4 SITL (see ``ros2_ws/src/rl_navigation/rl_navigation/drone_drop_env.py``
and ``ros2_ws/src/rl_navigation/config/hyperparams_v13.yaml``, the v15
baseline) into a single, GPU-vectorized Isaac Lab environment for PPO
(rsl_rl). All obs/action/reward/termination constants below are the
authoritative v13/v15 values — see
``notes/experiments/exp_012_isaac_migration_phase2.md`` for the full parity
table against the Gazebo source.

Design notes (see the migration plan for the full rationale):
  * Policy control rate is 10 Hz (``decimation=10`` @ ``sim.dt=1/100``) to
    match the Gazebo RL loop the v15 reward/LPF tuning assumed.
  * The velocity command LPF is a byte-identical discrete replica of
    ``drone_controller_node._filter_velocity`` (EMA, alpha=0.4, ticked at
    20 Hz = every 5 physics steps). The codebase's own convention labels its
    time constant tau ~= dt*(1/alpha - 1) ~= 75 ms @ 20 Hz (dt=0.05s) — see
    the comment in ``drone_controller_node.py``; this is a labeling
    convention only and does not change the discrete recurrence itself.
  * Vision (obs[9:12]) is an analytic pinhole projection of the ground-truth
    target position for training (YOLO-calibrated noise/dropout/hold);
    ``yolo_eval.py`` swaps in a real YOLOv8 detector for evaluation. Both
    emit the identical raw-pixel-space interface (obs[9:11] = u_norm,v_norm;
    obs[11] = conf) — see the plan's vision-pipeline section.
  * obs[12:14] (rel_dx, rel_dy) are METRIC ENU offsets from ground-truth
    drone/target position, never image-plane, never YOLO-fed. This is
    privileged target-position information (identical privilege to Gazebo,
    which hardcoded the target at ENU (11, 10)); it is not a scale ambiguity.
  * Payload drop is a scripted CCIP-parity metric (ballistic impact-error),
    NOT part of the action space, matching v15. Phase-2 hooks (learned CCIP
    residual, domain randomization) are wired in but inert in Phase 1 — see
    ``mdp/domain_rand.py`` and the ``DropCfg`` / ``ccip_residual`` docstrings
    below; Phase 1 output is bit-identical to a hook-free implementation.
"""

from __future__ import annotations

import math

import torch

import isaaclab.sim as sim_utils
from isaaclab.assets import Articulation, ArticulationCfg
from isaaclab.envs import DirectRLEnv, DirectRLEnvCfg
from isaaclab.scene import InteractiveSceneCfg
from isaaclab.sim import SimulationCfg
from isaaclab.utils import configclass

from isaaclab_assets import CRAZYFLIE_CFG

from .mdp.domain_rand import sample_drag_coefficient, sample_wind, sample_target_velocity
from .math_utils import (
    rate_limit_action,
    lpf_step,
    project_target_pinhole,
    ballistic_impact,
    ccip_residual,
    predict_impact_nominal,
    apply_ccip_residual,
    time_to_fall,
    step_target_velocity,
    impact_terminal_reward,
    lead_prediction_reward,
    compute_reward,
    overshoot_guard,
    stagnation_guard,
    hold_buffer_update,
    quat_apply_inverse_pure,
)

# All action/vision/ballistic/reward/guard math lives in math_utils.py
# (zero isaaclab dependency, unit-tested in isaac_lab/tests/test_math.py).
# This module wires that pure math into the isaaclab DirectRLEnv lifecycle.

# =====================================================================
# Config
# =====================================================================


@configclass
class DroneBombardObsCfg:
    pos_scale: float = 50.0
    vel_scale: float = 15.0
    ang_vel_scale: float = math.pi


@configclass
class DroneBombardActionCfg:
    vx_scale: float = 4.0
    vy_scale: float = 3.0
    vz_scale: float = 3.0
    yaw_scale: float = 1.0
    rate_limit: float = 0.2
    lpf_alpha: float = 0.4
    lpf_tick_period_steps: int = 5  # 100 Hz physics / 5 = 20 Hz LPF tick


@configclass
class DroneBombardRewardCfg:
    w_dist: float = 2.0
    w_heading: float = 0.0
    speed_gate_enabled: bool = True
    w_proximity: float = 0.6
    proximity_radius: float = 2.0
    w_vision_center: float = 1.5
    w_time: float = 0.05
    w_ang_vel: float = 0.15
    w_action_smooth: float = 0.20
    w_vel: float = 0.08
    vel_damp_radius: float = 3.0

    penalty_crash: float = -50.0
    penalty_overspeed: float = -30.0
    penalty_target_lost: float = 0.0
    penalty_out_of_range: float = -30.0
    penalty_max_altitude: float = -30.0
    penalty_stagnation: float = -15.0
    penalty_bad_attitude: float = -30.0
    penalty_overshoot: float = -10.0
    truncation_penalty: float = -15.0
    reward_success: float = 100.0

    success_radius: float = 0.8  # curriculum-tighten -> 0.5 once success_rate is stable


@configclass
class DroneBombardTerminationCfg:
    ground_contact_altitude: float = 0.5
    min_altitude: float = 3.0
    min_altitude_start_step: int = 1
    v_max_safety: float = 20.0
    limit_ang_vel: float = 2.0
    limit_inverted_tilt: float = 1.047  # rad (~60 deg) — matches _cfg_limit_inverted_tilt default;
    # note: the Gazebo yaml's `limit_tilt: 0.26` key is loaded but never
    # read anywhere else in drone_drop_env.py (dead config) — NOT ported.
    max_distance: float = 100.0
    max_altitude: float = 25.0
    overshoot_close_threshold: float = 0.6
    overshoot_margin: float = 2.0
    stagnation_window: int = 150
    stagnation_min_progress: float = 1.0
    stagnation_start_step: int = 50
    overshoot_flythrough_radius: float = 1.2  # non-terminating diagnostic only (see _get_dones)


@configclass
class DroneBombardVisionCfg:
    img_w: int = 640
    img_h: int = 480
    h_fov: float = 1.047  # rad, matches x500_bombard down_camera_sensor SDF
    near_clip: float = 0.1
    far_clip: float = 100.0
    pixel_noise_std: float = 3.0
    conf_lo: float = 0.73
    conf_hi: float = 0.95
    edge_falloff_start: float = 0.8  # center_dist beyond which conf degrades
    dropout_prob: float = 0.05
    hold_frames: int = 10  # @10Hz policy rate = 1s, matches xmarker_detector

    # slant-range conf falloff (exp_014 A2). Real YOLO sees the 1.5 m marker
    # at apparent size ~ 1/slant (Rule 13); the analytic model had NO range
    # dependence — the geometric root of the exp_013 climb-and-farm attractor
    # (offline counterfactual: this curve removes 84-88% of climb-phase
    # vision income while costing finishers ~x0.27). Multiplier:
    #   g(slant) = clamp(1 - (slant - range_conf_full)*range_conf_slope,
    #                    range_conf_floor, 1)
    # i.e. full conf inside 5 m, 0.5 at 10 m, floor 0.1 beyond 14 m.
    # NOTE: real-YOLO calibration (yolo_eval.py --calibrate) must replace
    # these numbers before any sim-to-real conf claim — blocked 2026-07-05 by
    # a container-level TiledCamera/annotator bug (the STOCK camera example
    # crashes identically); the calibrate harness itself is ready (X-marker
    # quads, camera spawn, teleport-sweep hardening, 3-27 m range).
    range_falloff_enabled: bool = True
    range_conf_full: float = 5.0
    range_conf_slope: float = 0.1
    range_conf_floor: float = 0.1

    fx: float = 0.0  # computed in __post_init__
    fy: float = 0.0
    cx: float = 0.0
    cy: float = 0.0

    def __post_init__(self):
        self.cx = self.img_w / 2.0
        self.cy = self.img_h / 2.0
        self.fx = self.cx / math.tan(self.h_fov / 2.0)
        self.fy = self.fx


@configclass
class DroneBombardResetCfg:
    spawn_alt_range: tuple[float, float] = (9.0, 11.0)
    handoff_dist_range: tuple[float, float] = (3.0, 7.0)
    yaw_range: tuple[float, float] = (-math.pi, math.pi)
    init_vel_std: float = 0.3
    target_xy_range: float = 10.0  # target randomized +/- this, around env origin


@configclass
class DroneBombardAssetCfg:
    """Measured x500_bombard SDF values (drone_drop_system/gazebo_models)."""
    drone_mass: float = 2.07  # base 2.0 + 4x0.016076923 rotors
    payload_mass: float = 0.1  # payload_cylinder SDF
    inertia_diag: tuple[float, float, float] = (0.02166666666666667, 0.02166666666666667, 0.04000000000000001)
    # The cf2x body link's parse-time PhysX inertia, kept as a reference
    # number only (it is what get_inertias() returned BEFORE any override).
    # History note (07-05 _diag_inertia measurement): the old runtime
    # set_inertias() DID propagate to the solver — the earlier "never reaches
    # the sim" claim was wrong — so exp_013 actually flew with solver inertia
    # = inertia_diag (0.0217) while the controller sized torques with THIS
    # value, i.e. a ~1300x under-torqued rate loop. Both spawn authoring and
    # the controller now use inertia_diag consistently.
    native_body_inertia_diag: tuple[float, float, float] = (1.6572e-05, 1.6656e-05, 2.9262e-05)
    thrust_to_weight_unloaded: float = 2.0  # fixed airframe/motor property, defined unloaded
    tilt_clamp_deg: float = 35.0

    @property
    def loaded_mass(self) -> float:
        return self.drone_mass + self.payload_mass

    @property
    def max_thrust(self) -> float:
        # T/W is a property of the (unloaded) airframe/motors, not the payload.
        return self.thrust_to_weight_unloaded * self.drone_mass * 9.81


@configclass
class DroneBombardDropCfg:
    gravity: float = 9.81
    release_tolerance: float = 0.2  # metres — was drop_calculator_node `drop_tolerance`
    release_delay: float = 0.1  # seconds — was drop_calculator_node `mechanism_delay`
    residual_enabled: bool = False  # Phase-2 hook; MUST stay False for Phase-1 bit-parity
    payload_mount_offset_z: float = -0.14  # bookkeeping/visual only (analytic payload)


@configclass
class DroneBombardControllerCfg:
    """Cascaded PX4-shaped velocity controller gains — initial values, to be
    calibrated against recorded PX4 SITL step responses (see
    notes/research/isaac_velocity_controller.md)."""
    kp_vel_xy: float = 1.8
    kp_vel_z: float = 4.0
    accel_xy_clamp: float = 8.0
    accel_z_clamp: float = 4.0
    k_att_rp: float = 6.5
    k_att_yaw: float = 4.0
    k_rate: tuple[float, float, float] = (18.0, 18.0, 8.0)


@configclass
class DroneBombardPhaseCfg:
    """Curriculum-phase parameters for the CCIP-residual / domain-randomization
    / moving-target stages (see the image's 3-stage table). Which of these are
    actually *active* is decided by the derived flags on ``DroneBombardEnvCfg``
    (set from ``phase`` in ``__post_init__``); this block only holds the
    magnitudes. All values are initial estimates — calibrate on the L4 VM
    (see the exp_015 note's TODO)."""

    # --- CCIP learned residual (action[4:6]) — Phase 2+ ---
    residual_scale: float = 3.0  # metres; action in [-1,1] -> +/- 3 m impact-point correction

    # --- domain randomization (model mismatch) — Phase 2+ ---
    drag_max: float = 0.15  # U[0, drag_max] first-order drag coefficient
    wind_std: float = 1.5   # m/s, N(0, wind_std) per horizontal axis

    # --- payload release trigger ---
    release_tolerance: float = 0.5   # m; release when |predicted_impact - aim| <= tol
    min_release_altitude: float = 1.0  # m; do not release below this altitude
    # RESERVED (not yet wired): a Phase-3 time-to-intercept gate. The current
    # release trigger fires on aim-error alone (see _evaluate_release); this
    # knob is kept for a future tau-based gate and documented in the exp_015
    # note's tuning TODO.
    intercept_tau: float = 0.5  # s

    # --- terminal impact reward (Phase 2+) ---
    w_impact: float = 100.0
    impact_reward_scale: float = 2.0  # m; exp(-real_err/scale)
    success_impact_radius: float = 0.8  # m; release with real_err <= this counts as success
    no_release_penalty: float = -20.0  # applied if the episode times out without releasing

    # --- moving target (Phase 3) ---
    target_init_speed: float = 2.0   # m/s, initial |target velocity| ~ U[0, this]
    target_vel_theta: float = 0.3    # Gauss-Markov mean-reversion rate (1/s)
    target_vel_sigma: float = 0.5    # Gauss-Markov volatility

    # --- lead-prediction reward (Phase 3) ---
    w_lead: float = 10.0
    lead_reward_scale: float = 3.0  # m


@configclass
class DroneBombardEnvCfg(DirectRLEnvCfg):
    decimation = 10
    episode_length_s = 30.0  # -> 300 policy steps @ 10 Hz, matches max_steps=300
    # Action is 6-dim across ALL phases so the rsl_rl network architecture is
    # identical phase-to-phase and ``runner.load()`` warm-start is lossless:
    #   action[0:4] = ENU velocity setpoint (vx, vy, vz, yaw) — always active
    #   action[4:6] = CCIP impact-point residual (delta_x, delta_y) — Phase 2+
    # In Phase 1 the env ignores action[4:6] entirely (residual_enabled=False),
    # so behavior is identical to the validated exp_014 4-dim approach task.
    action_space = 6
    observation_space = 14
    state_space = 0

    # Curriculum phase (1=approach/nominal, 2=CCIP+residual/stationary,
    # 3=moving target). The derived flags below are set from this in
    # __post_init__; do not set them directly.
    phase: int = 1
    residual_enabled: bool = False
    dr_enabled: bool = False
    moving_target_enabled: bool = False
    release_enabled: bool = False

    sim: SimulationCfg = SimulationCfg(dt=1.0 / 100.0, render_interval=decimation)

    scene: InteractiveSceneCfg = InteractiveSceneCfg(num_envs=2048, env_spacing=16.0, replicate_physics=True)

    # init_state override: CRAZYFLIE_CFG ships joint_vel = +-200 rad/s on the
    # four prop joints (cosmetic spinning rotors in the stock quadcopter demo).
    # With enable_gyroscopic_forces=True those spins are real dynamics, and
    # _reset_idx writes default_joint_vel back on EVERY reset — re-injecting
    # the spin each episode despite the "reset rotor joints" intent. Zeroing
    # at the source fixes both the initial spawn and every reset.
    robot_cfg: ArticulationCfg = CRAZYFLIE_CFG.replace(
        prim_path="/World/envs/env_.*/Robot",
        init_state=CRAZYFLIE_CFG.init_state.replace(joint_vel={".*": 0.0}),
    )

    obs: DroneBombardObsCfg = DroneBombardObsCfg()
    action: DroneBombardActionCfg = DroneBombardActionCfg()
    reward: DroneBombardRewardCfg = DroneBombardRewardCfg()
    termination: DroneBombardTerminationCfg = DroneBombardTerminationCfg()
    vision: DroneBombardVisionCfg = DroneBombardVisionCfg()
    reset: DroneBombardResetCfg = DroneBombardResetCfg()
    asset: DroneBombardAssetCfg = DroneBombardAssetCfg()
    drop: DroneBombardDropCfg = DroneBombardDropCfg()
    controller: DroneBombardControllerCfg = DroneBombardControllerCfg()
    phase_cfg: DroneBombardPhaseCfg = DroneBombardPhaseCfg()

    # Visualization markers (payload cylinder under the drone + target X on the
    # ground). Off by default so training pays no marker cost; turn on for
    # recording/eval to show the drone carrying a payload toward the target.
    show_markers: bool = False

    def __post_init__(self):
        # DirectRLEnvCfg (via its own __post_init__ chain) has no required
        # post-init on this version, but call super defensively in case a
        # future isaaclab version adds one.
        parent_post = getattr(super(), "__post_init__", None)
        if callable(parent_post):
            parent_post()
        # Derive the per-phase feature flags from the single ``phase`` knob so
        # every call site (env + train.py) toggles behavior consistently.
        self.residual_enabled = self.phase >= 2
        self.dr_enabled = self.phase >= 2
        self.release_enabled = self.phase >= 2
        self.moving_target_enabled = self.phase >= 3


# =====================================================================
# Env
# =====================================================================


class DroneBombardEnv(DirectRLEnv):
    cfg: DroneBombardEnvCfg

    def __init__(self, cfg: DroneBombardEnvCfg, render_mode: str | None = None, **kwargs):
        super().__init__(cfg, render_mode, **kwargs)

        # Re-derive the per-phase flags from cfg.phase here too: train.py sets
        # ``env_cfg.phase`` AFTER the cfg is constructed (so cfg.__post_init__
        # already ran with the default phase=1). Recomputing on the stored cfg
        # makes every ``self.cfg.<flag>`` read below authoritative.
        self.cfg.residual_enabled = self.cfg.phase >= 2
        self.cfg.dr_enabled = self.cfg.phase >= 2
        self.cfg.release_enabled = self.cfg.phase >= 2
        self.cfg.moving_target_enabled = self.cfg.phase >= 3

        N, device = self.num_envs, self.device
        # 6-dim action pipeline: [0:4] velocity setpoint, [4:6] CCIP residual.
        self._prev_action = torch.zeros(N, 6, device=device)
        self._v_filt = torch.zeros(N, 4, device=device)  # velocity LPF only tracks the 4 vel dims
        self._lpf_snap = torch.ones(N, dtype=torch.bool, device=device)
        self._physics_tick = 0
        self._residual_action = torch.zeros(N, 2, device=device)  # latest action[4:6]

        self._target_xy = torch.zeros(N, 2, device=device)
        self._target_vel_xy = torch.zeros(N, 2, device=device)  # Phase 3 moving target
        self._d_xy_prev = torch.zeros(N, device=device)
        self._d_xy_min = torch.full((N,), float("inf"), device=device)
        self._d_xy_history = torch.zeros(N, self.max_episode_length, device=device)

        self._held_uv_conf = torch.zeros(N, 3, device=device)
        self._hold_remaining = torch.zeros(N, dtype=torch.long, device=device)

        self._payload_attached = torch.ones(N, dtype=torch.bool, device=device)
        self._drag_coef = torch.zeros(N, device=device)
        self._wind_xy = torch.zeros(N, 2, device=device)

        # Payload-release state (Phase 2+). ``_released`` latches True once the
        # drop is triggered; ``_release_impact_err`` holds the real (DR-physics)
        # miss distance and ``_release_aim_xy`` the point the policy aimed at
        # (for the Phase-3 lead-prediction reward). ``_just_released`` flags the
        # single step the release fires on (for the terminal reward).
        self._released = torch.zeros(N, dtype=torch.bool, device=device)
        self._just_released = torch.zeros(N, dtype=torch.bool, device=device)
        self._release_impact_err = torch.zeros(N, device=device)
        self._release_aim_xy = torch.zeros(N, 2, device=device)
        self._release_target_xy = torch.zeros(N, 2, device=device)

        self._episode_sums = {
            k: torch.zeros(N, device=device)
            for k in ("rew_ctrl", "rew_dist", "rew_orient", "rew_proximity", "rew_vision", "rew_vel")
        }
        # Per-episode sum of the per-step fraction of RAW policy action
        # components outside [-1, 1] (measured before the env clips). The
        # clip hides mean-saturation from every downstream signal — exp_013
        # forensics measured raw |a|=2.6 with 77% of components beyond the
        # rail even at sigma=0, invisible in any logged metric until now.
        self._action_sat_sum = torch.zeros(N, device=device)

        self._robot: Articulation = self.scene["robot"]

        # Body index for wrench application (Crazyflie's main link is "body").
        self._body_id = self._robot.find_bodies("body")[0]

        # Mass/inertia are authored at SPAWN time on the USD body prim (see
        # _author_body_mass_props in _setup_scene) — there is no runtime
        # set_masses() override anymore. The old runtime path left one physics
        # substep in which the solver still integrated the native 25 g body
        # against a hover wrench sized for 2.17 kg: a one-off +8.4 m/s vertical
        # kick on the process's first sim step (exp_013 §4d forensics).
        # get_inertias/get_masses here read back what USD authored, so the
        # controller and the solver agree from substep 0.
        #
        # Rate-loop invariant: torque = I_ctrl*(k_rate*rate_err), closed loop
        # dw/dt = (I_ctrl/I_body)*k_rate*rate_err — design behavior only when
        # I_ctrl == I_body, hence I_ctrl is READ from the sim, not taken from
        # cfg. History (07-05 _diag_inertia): exp_013's runtime set_inertias
        # DID reach the solver (I_body=0.0217) while this read — taken BEFORE
        # the override — returned the parse-time 1.66e-5, so the rate loop ran
        # ~1300x under-torqued (sluggish attitude, wobble). With spawn
        # authoring the read is deferred-consumption-free and always matches.
        bidx = self._body_id[0] if isinstance(self._body_id, (list, tuple)) else int(self._body_id)
        body_I = self._robot.root_physx_view.get_inertias()[0, bidx].reshape(-1).clone()
        self._inertia_diag = torch.stack([body_I[0], body_I[4], body_I[8]]).to(self.device)

        self._ctrl_mass = float(self._robot.root_physx_view.get_masses()[0].sum())
        if self._ctrl_mass < 1.0:
            raise RuntimeError(
                f"spawn-time mass authoring did not reach PhysX (total mass "
                f"{self._ctrl_mass:.4f} kg — expected ~{self.cfg.asset.loaded_mass:.2f} kg). "
                "The controller would fly the native 28 g Crazyflie shell; aborting."
            )
        self._max_thrust = self.cfg.asset.thrust_to_weight_unloaded * self._ctrl_mass * 9.81
        self._k_rate = torch.tensor(self.cfg.controller.k_rate, device=self.device)
        print(f"[DroneBombardEnv] control mass={self._ctrl_mass:.3f}kg max_thrust={self._max_thrust:.2f}N "
              f"inertia_diag={self._inertia_diag.detach().cpu().numpy()} body_id={self._body_id}")

        self._setup_markers()

    def _setup_markers(self):
        """Optional visualization: payload cylinder (measured payload_cylinder
        SDF dims) riding under the drone + an X-marker plate on the ground at
        the target. Visual-only (VisualizationMarkers) — no physics, no cost
        unless cfg.show_markers is True. Used for recording/eval."""
        self._payload_marker = None
        self._target_marker = None
        if not self.cfg.show_markers:
            return
        from isaaclab.markers import VisualizationMarkers, VisualizationMarkersCfg

        self._payload_marker = VisualizationMarkers(VisualizationMarkersCfg(
            prim_path="/Visuals/Payload",
            markers={"payload": sim_utils.CylinderCfg(
                radius=0.05, height=0.06, axis="Z",
                visual_material=sim_utils.PreviewSurfaceCfg(diffuse_color=(0.95, 0.45, 0.05)),
            )},
        ))
        self._target_marker = VisualizationMarkers(VisualizationMarkersCfg(
            prim_path="/Visuals/Target",
            markers={"target": sim_utils.CylinderCfg(
                radius=0.8, height=0.02, axis="Z",
                visual_material=sim_utils.PreviewSurfaceCfg(diffuse_color=(0.9, 0.1, 0.1)),
            )},
        ))

    def _update_markers(self):
        if self._payload_marker is None:
            return
        pos_w = self._robot.data.root_pos_w
        # Only auto-follow the drone while the payload is attached. Once
        # released (_payload_attached False, e.g. by record_episode.py's
        # drop animation), external code drives the marker directly via
        # self._payload_marker.visualize(...) and this call is a no-op for
        # it — we skip re-snapping it to the drone every step.
        if bool(self._payload_attached[0]):
            payload_pos = pos_w.clone()
            payload_pos[:, 2] += self.cfg.drop.payload_mount_offset_z  # ride under the drone
            self._payload_marker.visualize(translations=payload_pos)
        target_pos = torch.zeros_like(pos_w)
        target_pos[:, :2] = self._target_xy + self.scene.env_origins[:, :2]
        target_pos[:, 2] = 0.01
        self._target_marker.visualize(translations=target_pos)

    def _author_body_mass_props(self):
        """Author the x500 mass + the measured native inertia on the body link
        at SPAWN time via UsdPhysics.MassAPI — the replacement for the former
        runtime ``root_physx_view.set_masses()`` override. PhysX parses
        USD-authored values before the first substep, so there is no window in
        which the solver integrates a wrench sized for 2.17 kg against the
        native 25 g shell (the once-per-process +8.4 m/s kick, exp_013 §4d).
        Runs in ``_setup_scene`` on the source env's prim, before
        ``clone_environments`` — physics replication propagates the authored
        values to every env. Inertia is the measured x500 ``inertia_diag`` —
        the SAME solver inertia exp_013 flew (its runtime set_inertias DID
        propagate, per the 07-05 ``_diag_inertia`` measurement); what changes
        vs exp_013 is that the controller's ``get_inertias`` read now returns
        this value too, so the rate loop applies design torque instead of
        being ~1300x under-torqued."""
        from pxr import Gf, UsdPhysics

        from isaaclab.sim.utils import find_matching_prims

        asset = self.cfg.asset
        body_prims = find_matching_prims(self.cfg.robot_cfg.prim_path + "/body")
        if not body_prims:
            raise RuntimeError(
                f"spawn-time mass authoring: no 'body' prim matched "
                f"{self.cfg.robot_cfg.prim_path}/body"
            )
        for prim in body_prims:
            mass_api = UsdPhysics.MassAPI.Apply(prim)
            mass_api.CreateMassAttr().Set(asset.loaded_mass)
            mass_api.CreateDiagonalInertiaAttr().Set(Gf.Vec3f(*asset.inertia_diag))
            mass_api.CreatePrincipalAxesAttr().Set(Gf.Quatf(1.0, 0.0, 0.0, 0.0))

    # ------------------------------------------------------------------
    def _setup_scene(self):
        self._robot = Articulation(self.cfg.robot_cfg)
        self.scene.articulations["robot"] = self._robot
        self._author_body_mass_props()

        spawn_ground = sim_utils.GroundPlaneCfg()
        spawn_ground.func("/World/ground", spawn_ground)
        light_cfg = sim_utils.DomeLightCfg(intensity=2000.0, color=(0.9, 0.9, 0.9))
        light_cfg.func("/World/Light", light_cfg)

        self.scene.clone_environments(copy_from_source=False)
        if self.device == "cpu":
            self.scene.filter_collisions(global_prim_paths=[])

    # ------------------------------------------------------------------
    # Action pipeline
    # ------------------------------------------------------------------
    def _pre_physics_step(self, actions: torch.Tensor):
        self._action_sat_sum += (actions.abs() > 1.0).float().mean(dim=-1)
        clipped = torch.clamp(actions, -1.0, 1.0)

        # Split the 6-dim action: [0:4] velocity setpoint, [4:6] CCIP residual.
        vel_clipped = clipped[:, :4]
        residual_clipped = clipped[:, 4:6]

        # Velocity dims: rate-limit against the previous velocity action.
        prev_vel = self._prev_action[:, :4]
        limited_vel = rate_limit_action(vel_clipped, prev_vel, self.cfg.action.rate_limit)

        # Smoothness penalty stays on the 4 velocity dims (v13/v15 parity — the
        # residual is an instantaneous aim correction, not rate-limited).
        self._delta_action = limited_vel - prev_vel

        # Residual is used only in Phase 2+; zero it out otherwise so a Phase-1
        # policy's spurious [4:6] outputs never touch the impact prediction.
        self._residual_action = residual_clipped if self.cfg.residual_enabled else torch.zeros_like(residual_clipped)

        self._prev_action = torch.cat([limited_vel, self._residual_action], dim=-1)

        a = self.cfg.action
        self._vel_cmd = torch.stack([
            limited_vel[:, 0] * a.vx_scale,
            limited_vel[:, 1] * a.vy_scale,
            limited_vel[:, 2] * a.vz_scale,
            limited_vel[:, 3] * a.yaw_scale,
        ], dim=-1)
        self._physics_tick = 0

    def _apply_action(self):
        # Runs once per physics step (100 Hz); LPF ticks every 5th call (20 Hz).
        if self._physics_tick % self.cfg.action.lpf_tick_period_steps == 0:
            self._v_filt = lpf_step(self._vel_cmd, self._v_filt, self.cfg.action.lpf_alpha, self._lpf_snap)
            self._lpf_snap[:] = False
        self._physics_tick += 1

        self._run_velocity_controller(self._v_filt)

    def _run_velocity_controller(self, v_filt: torch.Tensor):
        """Cascaded PX4-shaped velocity -> wrench controller (velocity P ->
        desired thrust vector -> attitude P -> rate P -> body torque). Thrust
        is applied along body +Z (rotors can only push "up" relative to the
        airframe); the body tilts to redirect it. See
        DroneBombardControllerCfg / notes/research/isaac_velocity_controller.md
        for gain provenance and calibration status.

        Force/torque are applied in the BODY frame via the articulation's
        permanent wrench composer (the API the stock Isaac Lab quadcopter env
        uses on this version)."""
        from isaaclab.utils.math import matrix_from_quat

        c = self.cfg.controller

        vel_w = self._robot.data.root_lin_vel_w
        quat_w = self._robot.data.root_quat_w
        ang_vel_b = self._robot.data.root_ang_vel_b

        vel_err_xy = v_filt[:, :2] - vel_w[:, :2]
        vel_err_z = v_filt[:, 2] - vel_w[:, 2]
        accel_des_xy = torch.clamp(c.kp_vel_xy * vel_err_xy, min=-c.accel_xy_clamp, max=c.accel_xy_clamp)
        accel_des_z = torch.clamp(c.kp_vel_z * vel_err_z, min=-c.accel_z_clamp, max=c.accel_z_clamp)

        # desired total force in WORLD frame: mass*(accel_cmd + gravity comp)
        f_des = torch.zeros_like(vel_w)
        f_des[:, 0] = self._ctrl_mass * accel_des_xy[:, 0]
        f_des[:, 1] = self._ctrl_mass * accel_des_xy[:, 1]
        f_des[:, 2] = self._ctrl_mass * (accel_des_z + 9.81)

        f_norm = torch.linalg.norm(f_des, dim=-1, keepdim=True).clamp(min=1e-6)
        thrust_dir_w = f_des / f_norm  # desired body-up direction (world frame)

        # tilt clamp: cap how far the desired thrust axis can lean from vertical
        tilt_limit = math.radians(self.cfg.asset.tilt_clamp_deg)
        max_horiz_ratio = math.tan(tilt_limit)
        horiz = thrust_dir_w[:, :2]
        horiz_norm = torch.linalg.norm(horiz, dim=-1, keepdim=True).clamp(min=1e-6)
        horiz_clamped = torch.where(
            horiz_norm > max_horiz_ratio, horiz / horiz_norm * max_horiz_ratio, horiz
        )
        thrust_dir_w = torch.cat([horiz_clamped, thrust_dir_w[:, 2:3].clamp(min=1e-3)], dim=-1)
        thrust_dir_w = thrust_dir_w / torch.linalg.norm(thrust_dir_w, dim=-1, keepdim=True).clamp(min=1e-6)

        body_z_w = matrix_from_quat(quat_w)[:, :, 2]  # current body-up in world
        # rotor thrust magnitude = projection of desired force onto current body-up
        thrust_mag = torch.clamp((f_des * body_z_w).sum(dim=-1), min=0.0, max=self._max_thrust)

        # attitude error: rotate current body-up toward desired thrust dir.
        # axis*angle is a WORLD-frame rotation vector; convert to BODY frame
        # because the rate loop and torque act in the body frame.
        cos_err = (thrust_dir_w * body_z_w).sum(dim=-1).clamp(-1.0, 1.0)
        axis_w = torch.linalg.cross(body_z_w, thrust_dir_w, dim=-1)
        axis_norm = torch.linalg.norm(axis_w, dim=-1, keepdim=True).clamp(min=1e-8)
        rot_err_w = (axis_w / axis_norm) * torch.acos(cos_err).unsqueeze(-1)
        rot_err_b = quat_apply_inverse_pure(quat_w, rot_err_w)

        rate_sp = torch.zeros_like(ang_vel_b)
        rate_sp[:, 0] = c.k_att_rp * rot_err_b[:, 0]
        rate_sp[:, 1] = c.k_att_rp * rot_err_b[:, 1]
        # yaw-rate is commanded directly (body z); roll/pitch attitude-controlled
        rate_sp[:, 2] = v_filt[:, 3]

        # rate loop outputs a desired ANGULAR ACCELERATION (k_rate has units
        # 1/s); the applied torque is inertia * angular_accel (tau = I*alpha).
        # Skipping the inertia term makes torque ~1/I too large — for this
        # airframe's ~0.022 kg.m^2 that is a ~46x over-torque that instantly
        # spins the drone past the attitude limits.
        rate_err = rate_sp - ang_vel_b
        ang_accel_des = self._k_rate * rate_err
        torque_b = self._inertia_diag * ang_accel_des

        forces_b = torch.zeros(self.num_envs, 1, 3, device=self.device)
        forces_b[:, 0, 2] = thrust_mag  # body +Z
        torques_b = torch.zeros(self.num_envs, 1, 3, device=self.device)
        torques_b[:, 0, :] = torque_b
        # final safety: never inject a non-finite wrench into physics (a
        # transient degenerate controller state must not NaN the whole sim)
        forces_b = torch.nan_to_num(forces_b)
        torques_b = torch.nan_to_num(torques_b)
        self._robot.permanent_wrench_composer.set_forces_and_torques(
            body_ids=self._body_id, forces=forces_b, torques=torques_b
        )

    # ------------------------------------------------------------------
    # Vision
    # ------------------------------------------------------------------
    def _update_vision(self):
        vc = self.cfg.vision
        # env-LOCAL position: _target_xy is sampled relative to each env's
        # origin, so the projection must see the drone in the same frame.
        # Passing root_pos_w (world) here silently killed vision for every
        # env away from the grid center (origins span hundreds of metres at
        # 2048 envs -> target always projected off-frame -> conf==0 for the
        # whole run). num_envs=1 verification never caught it because there
        # the origin is ~0 and local == world.
        pos_local = self._robot.data.root_pos_w - self.scene.env_origins
        quat_w = self._robot.data.root_quat_w

        u_px, v_px, visible = project_target_pinhole(
            pos_local, quat_w, self._target_xy,
            vc.fx, vc.fy, vc.cx, vc.cy, vc.img_w, vc.img_h, vc.near_clip, vc.far_clip,
        )
        u_px = u_px + torch.randn_like(u_px) * vc.pixel_noise_std
        v_px = v_px + torch.randn_like(v_px) * vc.pixel_noise_std

        u_n = (u_px / vc.img_w) * 2.0 - 1.0
        v_n = (v_px / vc.img_h) * 2.0 - 1.0
        center_dist = torch.sqrt(u_n * u_n + v_n * v_n)

        edge_factor = torch.clamp(1.0 - (center_dist - vc.edge_falloff_start).clamp(min=0.0), min=0.1, max=1.0)
        conf_base = vc.conf_lo + (vc.conf_hi - vc.conf_lo) * torch.rand_like(center_dist)
        conf = conf_base * edge_factor

        if vc.range_falloff_enabled:
            # slant range drone -> target (target on the ground, z=0)
            rel_xy = self._target_xy - pos_local[:, :2]
            slant = torch.sqrt((rel_xy * rel_xy).sum(dim=-1) + pos_local[:, 2] ** 2)
            range_factor = torch.clamp(
                1.0 - (slant - vc.range_conf_full).clamp(min=0.0) * vc.range_conf_slope,
                min=vc.range_conf_floor, max=1.0,
            )
            conf = conf * range_factor

        dropout = torch.rand_like(conf) < (vc.dropout_prob / edge_factor.clamp(min=0.1))
        detected = visible & ~dropout

        fresh_uv_conf = torch.stack([u_px, v_px, conf], dim=-1)
        u_out, v_out, conf_out, self._held_uv_conf, self._hold_remaining = hold_buffer_update(
            detected, fresh_uv_conf, self._held_uv_conf, self._hold_remaining, vc.hold_frames,
        )
        return u_out, v_out, conf_out

    # ------------------------------------------------------------------
    # Observations
    # ------------------------------------------------------------------
    def _get_observations(self) -> dict:
        self._update_markers()

        oc = self.cfg.obs
        pos = self._robot.data.root_pos_w - self.scene.env_origins
        vel = self._robot.data.root_lin_vel_w
        ang = self._robot.data.root_ang_vel_b

        pos_n = torch.clamp(pos / oc.pos_scale, -1.0, 1.0)
        vel_n = torch.clamp(vel / oc.vel_scale, -1.0, 1.0)
        ang_n = torch.clamp(ang / oc.ang_vel_scale, -1.0, 1.0)

        u_px, v_px, conf = self._update_vision()
        vc = self.cfg.vision
        u_norm = torch.where(conf > 0, (u_px / vc.img_w) * 2.0 - 1.0, torch.zeros_like(u_px))
        v_norm = torch.where(conf > 0, (v_px / vc.img_h) * 2.0 - 1.0, torch.zeros_like(v_px))
        conf_clamped = torch.clamp(conf, 0.0, 1.0)

        rel_dx = torch.clamp((pos[:, 0] - self._target_xy[:, 0]) / oc.pos_scale, -1.0, 1.0)
        rel_dy = torch.clamp((pos[:, 1] - self._target_xy[:, 1]) / oc.pos_scale, -1.0, 1.0)

        self._last_vision = (u_norm, v_norm, conf_clamped)

        obs = torch.cat([
            pos_n, vel_n, ang_n,
            u_norm.unsqueeze(-1), v_norm.unsqueeze(-1), conf_clamped.unsqueeze(-1),
            rel_dx.unsqueeze(-1), rel_dy.unsqueeze(-1),
        ], dim=-1)
        obs = torch.nan_to_num(obs, nan=0.0)
        return {"policy": obs}

    def _current_d_xy(self) -> torch.Tensor:
        pos = self._robot.data.root_pos_w - self.scene.env_origins
        return torch.linalg.norm(pos[:, :2] - self._target_xy, dim=-1)

    # ------------------------------------------------------------------
    # Phase 2/3 dynamics: moving target + payload release (once per policy step)
    # ------------------------------------------------------------------
    def _advance_phase_dynamics(self):
        """Run once per policy step, at the top of ``_get_dones`` (which the
        DirectRLEnv contract guarantees fires before ``_get_rewards``). Moves
        the target (Phase 3) and evaluates the payload-release trigger
        (Phase 2+). No-op in Phase 1."""
        self._just_released = torch.zeros(self.num_envs, dtype=torch.bool, device=self.device)

        if self.cfg.moving_target_enabled:
            dt = self.cfg.sim.dt * self.cfg.decimation  # policy-step period (0.1 s @ 10 Hz)
            pc = self.cfg.phase_cfg
            noise = torch.randn_like(self._target_vel_xy)
            self._target_vel_xy = step_target_velocity(
                self._target_vel_xy, pc.target_vel_theta, pc.target_vel_sigma, dt, noise
            )
            self._target_xy = self._target_xy + self._target_vel_xy * dt

        if self.cfg.release_enabled:
            self._evaluate_release()

    def _evaluate_release(self):
        """Evaluate the scripted CCIP release trigger for the envs that have
        not yet dropped. The on-board predictor uses the NOMINAL ballistic
        model (no drag/wind) plus the policy's learned residual; the real drop
        uses the domain-randomized physics, so ``_release_impact_err`` is the
        true miss the terminal reward is shaped on.

        Phase 2 (stationary): aim at the target; the residual must offset the
        DR drag/wind so the real payload lands on it. Phase 3 (moving): aim at
        the analytic lead point (target position at predicted impact time); the
        residual + lead reward drive prediction accuracy. See the exp_015 note
        for the documented follow-up (adding target velocity to the obs so an
        MLP policy can learn lead itself)."""
        pc = self.cfg.phase_cfg
        dc = self.cfg.drop
        pos = self._robot.data.root_pos_w - self.scene.env_origins
        vel = self._robot.data.root_lin_vel_w
        pos_xy = pos[:, :2]
        vel_xy = vel[:, :2]
        altitude = pos[:, 2]

        # On-board prediction: nominal CCIP + learned residual (metres).
        pred_impact = predict_impact_nominal(pos_xy, vel_xy, altitude, dc.release_delay, dc.gravity)
        pred_impact = apply_ccip_residual(pred_impact, self._residual_action, pc.residual_scale)

        # Aim point = target position at the predicted impact time (lead).
        t_impact = time_to_fall(altitude, dc.gravity) + dc.release_delay
        target_at_impact = self._target_xy + self._target_vel_xy * t_impact.unsqueeze(-1)

        aim_err = torch.linalg.norm(pred_impact - target_at_impact, dim=-1)
        fire = (
            (~self._released)
            & (altitude > pc.min_release_altitude)
            & (aim_err <= pc.release_tolerance)
        )
        if not bool(torch.any(fire)):
            return

        # Real drop under domain-randomized physics -> true impact error.
        real_impact = ballistic_impact(
            pos_xy, vel_xy, altitude, dc.release_delay, dc.gravity, self._drag_coef, self._wind_xy
        )
        real_err = torch.linalg.norm(real_impact - target_at_impact, dim=-1)

        self._just_released = fire
        self._released = self._released | fire
        self._release_impact_err = torch.where(fire, real_err, self._release_impact_err)
        self._release_aim_xy = torch.where(fire.unsqueeze(-1), pred_impact, self._release_aim_xy)
        self._release_target_xy = torch.where(fire.unsqueeze(-1), target_at_impact, self._release_target_xy)

    # ------------------------------------------------------------------
    # Dones (computed + cached BEFORE rewards, per DirectRLEnv contract)
    # ------------------------------------------------------------------
    def _get_dones(self) -> tuple[torch.Tensor, torch.Tensor]:
        # Phase 2/3: move the target and evaluate the release trigger first, so
        # the release outcome is available to both dones and rewards this step.
        self._advance_phase_dynamics()

        tc = self.cfg.termination
        rc = self.cfg.reward

        pos = self._robot.data.root_pos_w - self.scene.env_origins
        vel = self._robot.data.root_lin_vel_w
        ang = self._robot.data.root_ang_vel_b
        altitude = pos[:, 2]
        speed = torch.linalg.norm(vel, dim=-1)

        from isaaclab.utils.math import euler_xyz_from_quat
        roll, pitch, _ = euler_xyz_from_quat(self._robot.data.root_quat_w)
        roll = torch.atan2(torch.sin(roll), torch.cos(roll))
        pitch = torch.atan2(torch.sin(pitch), torch.cos(pitch))

        d_xy = self._current_d_xy()
        step_in_ep = self.episode_length_buf

        crash = (altitude < tc.ground_contact_altitude) | (
            (step_in_ep > tc.min_altitude_start_step) & (altitude < tc.min_altitude)
        )
        overspeed = speed > tc.v_max_safety
        bad_ang_vel = torch.linalg.norm(ang, dim=-1) > tc.limit_ang_vel
        inverted = (roll.abs() > tc.limit_inverted_tilt) | (pitch.abs() > tc.limit_inverted_tilt)
        bad_attitude = bad_ang_vel | inverted
        out_of_range = d_xy > tc.max_distance
        max_alt = altitude > tc.max_altitude

        self._d_xy_min = torch.minimum(self._d_xy_min, d_xy)
        overshoot = overshoot_guard(d_xy, self._d_xy_min, tc.overshoot_close_threshold, tc.overshoot_margin)
        # non-terminating diagnostic (arm radius > success_radius, so it is
        # reachable independent of the terminating overshoot guard's dormancy
        # at success_radius=0.8; zero reward/termination effect)
        self._overshoot_flythrough = overshoot_guard(
            d_xy, self._d_xy_min, tc.overshoot_flythrough_radius, tc.overshoot_margin
        )

        idx = step_in_ep.clamp(max=self._d_xy_history.shape[1] - 1)
        self._d_xy_history.scatter_(1, idx.unsqueeze(-1), d_xy.unsqueeze(-1))
        past_idx = (step_in_ep - tc.stagnation_window).clamp(min=0)
        past_d_xy = self._d_xy_history.gather(1, past_idx.unsqueeze(-1)).squeeze(-1)
        stagnation = stagnation_guard(d_xy, past_d_xy, step_in_ep, tc.stagnation_window, tc.stagnation_min_progress)

        pc = self.cfg.phase_cfg
        if self.cfg.release_enabled:
            # Phase 2/3: success is a payload release with a small true impact
            # error. ANY release ends the episode (the payload is spent);
            # proximity alone no longer terminates (the drone must linger and
            # drop, not just fly close).
            success = self._just_released & (self._release_impact_err <= pc.success_impact_radius)
            released_this_step = self._just_released
        else:
            # Phase 1: proximity success (validated exp_014 approach task).
            success = d_xy <= rc.success_radius
            released_this_step = torch.zeros_like(success)

        failure = crash | overspeed | bad_attitude | out_of_range | max_alt | overshoot | stagnation
        terminated = success | failure | released_this_step
        time_out = step_in_ep >= (self.max_episode_length - 1)

        self._done_flags = {
            "success": success, "crash": crash, "overspeed": overspeed,
            "bad_attitude": bad_attitude, "out_of_range": out_of_range,
            "max_altitude": max_alt, "overshoot": overshoot, "stagnation": stagnation,
            "released": released_this_step,
            "release_miss": released_this_step & ~success,
            "timeout": time_out & ~terminated,
        }
        return terminated, time_out & ~terminated

    # ------------------------------------------------------------------
    # Rewards
    # ------------------------------------------------------------------
    def _get_rewards(self) -> torch.Tensor:
        rc = self.cfg.reward
        pos = self._robot.data.root_pos_w - self.scene.env_origins
        vel = self._robot.data.root_lin_vel_w
        ang = self._robot.data.root_ang_vel_b
        d_xy = self._current_d_xy()
        u_norm, v_norm, conf = self._last_vision

        reward, breakdown = compute_reward(
            d_xy, self._d_xy_prev, pos[:, :2], self._target_xy, vel[:, :2], ang,
            self._delta_action, u_norm, v_norm, conf, rc,
        )

        f = self._done_flags
        reward = reward + f["crash"].float() * rc.penalty_crash
        reward = reward + f["overspeed"].float() * rc.penalty_overspeed
        reward = reward + f["bad_attitude"].float() * rc.penalty_bad_attitude
        reward = reward + f["out_of_range"].float() * rc.penalty_out_of_range
        reward = reward + f["max_altitude"].float() * rc.penalty_max_altitude
        reward = reward + f["overshoot"].float() * rc.penalty_overshoot
        reward = reward + f["stagnation"].float() * rc.penalty_stagnation
        reward = reward + f["timeout"].float() * rc.truncation_penalty

        pc = self.cfg.phase_cfg
        if self.cfg.release_enabled:
            # Phase 2/3 terminal reward is the true (DR-physics) impact error at
            # release, NOT the proximity success bonus. exp(-real_err/scale)
            # saturates to w_impact for a perfect hit and decays smoothly.
            released = f["released"].float()
            reward = reward + released * impact_terminal_reward(
                self._release_impact_err, pc.w_impact, pc.impact_reward_scale
            )
            if self.cfg.moving_target_enabled:
                # Phase 3: also reward correctly predicting the target's future
                # (lead) position at impact time.
                reward = reward + released * lead_prediction_reward(
                    self._release_aim_xy, self._release_target_xy, pc.w_lead, pc.lead_reward_scale
                )
            # Penalize timing out without ever dropping the payload.
            reward = reward + f["timeout"].float() * pc.no_release_penalty
        else:
            # Phase 1: proximity success bonus (validated exp_014 approach task).
            reward = reward + f["success"].float() * rc.reward_success

        conf_zero = conf <= 0.0
        reward = reward + conf_zero.float() * rc.penalty_target_lost  # 0.0 in v13/v15, kept for parity

        self._d_xy_prev = d_xy
        for k, v in breakdown.items():
            self._episode_sums[k] += v

        return reward

    # ------------------------------------------------------------------
    # Reset
    # ------------------------------------------------------------------
    def _reset_idx(self, env_ids: torch.Tensor):
        if env_ids is None or len(env_ids) == self.num_envs:
            env_ids = self._robot._ALL_INDICES

        # Snapshot everything needed to log the JUST-ENDED episode's final
        # state BEFORE any of it is overwritten below (teleport, buffer
        # zeroing) — logging after those mutations would silently report the
        # new episode's spawn state instead of the terminated episode's
        # outcome (e.g. "drop impact error" computed from the fresh spawn
        # position, or reward sums that were already zeroed).
        final_snapshot = self._snapshot_final_episode_state(env_ids)

        self._robot.reset(env_ids)
        super()._reset_idx(env_ids)

        n = len(env_ids)
        device = self.device
        rc = self.cfg.reset

        target_xy = (torch.rand(n, 2, device=device) * 2.0 - 1.0) * rc.target_xy_range
        self._target_xy[env_ids] = target_xy

        radius = rc.handoff_dist_range[0] + torch.rand(n, device=device) * (
            rc.handoff_dist_range[1] - rc.handoff_dist_range[0]
        )
        theta = torch.rand(n, device=device) * 2.0 * math.pi
        spawn_xy = target_xy + torch.stack([radius * torch.cos(theta), radius * torch.sin(theta)], dim=-1)
        spawn_alt = rc.spawn_alt_range[0] + torch.rand(n, device=device) * (
            rc.spawn_alt_range[1] - rc.spawn_alt_range[0]
        )
        yaw = rc.yaw_range[0] + torch.rand(n, device=device) * (rc.yaw_range[1] - rc.yaw_range[0])

        default_root = self._robot.data.default_root_state[env_ids].clone()
        default_root[:, 0:2] = spawn_xy + self.scene.env_origins[env_ids, :2]
        default_root[:, 2] = spawn_alt
        half_yaw = yaw / 2.0
        default_root[:, 3] = torch.cos(half_yaw)
        default_root[:, 4] = 0.0
        default_root[:, 5] = 0.0
        default_root[:, 6] = torch.sin(half_yaw)
        default_root[:, 7:10] = torch.randn(n, 3, device=device) * rc.init_vel_std
        default_root[:, 10:13] = 0.0

        self._robot.write_root_pose_to_sim(default_root[:, :7], env_ids)
        self._robot.write_root_velocity_to_sim(default_root[:, 7:13], env_ids)
        # reset rotor joints (Crazyflie articulation has 4 prop joints).
        # default_joint_vel is all-zero via the init_state override in
        # DroneBombardEnvCfg.robot_cfg — the stock CRAZYFLIE_CFG defaults are
        # +-200 rad/s prop spin, which this write used to re-inject every
        # reset with gyroscopic forces enabled.
        joint_pos = self._robot.data.default_joint_pos[env_ids]
        joint_vel = self._robot.data.default_joint_vel[env_ids]
        self._robot.write_joint_state_to_sim(joint_pos, joint_vel, None, env_ids)

        self._prev_action[env_ids] = 0.0
        self._v_filt[env_ids] = 0.0
        self._lpf_snap[env_ids] = True
        self._residual_action[env_ids] = 0.0
        self._d_xy_min[env_ids] = float("inf")
        self._d_xy_history[env_ids] = 0.0
        self._held_uv_conf[env_ids] = 0.0
        self._hold_remaining[env_ids] = 0
        self._payload_attached[env_ids] = True

        # Payload-release state (Phase 2+).
        self._released[env_ids] = False
        self._just_released[env_ids] = False
        self._release_impact_err[env_ids] = 0.0
        self._release_aim_xy[env_ids] = 0.0
        self._release_target_xy[env_ids] = 0.0

        self._action_sat_sum[env_ids] = 0.0

        # Domain randomization (model mismatch) is active only in Phase 2+;
        # the moving-target velocity only in Phase 3. Off -> identity (zero),
        # so Phase 1 reduces to the drag-free/wind-free stationary-target task.
        pc = self.cfg.phase_cfg
        self._drag_coef[env_ids] = sample_drag_coefficient(pc, n, device, self.cfg.dr_enabled)
        self._wind_xy[env_ids] = sample_wind(pc, n, device, self.cfg.dr_enabled)
        self._target_vel_xy[env_ids] = sample_target_velocity(pc, n, device, self.cfg.moving_target_enabled)

        d_xy0 = torch.linalg.norm(spawn_xy - target_xy, dim=-1)
        self._d_xy_prev[env_ids] = d_xy0

        for k in self._episode_sums:
            self._episode_sums[k][env_ids] = 0.0

        self._log_reset_extras(env_ids, final_snapshot)

    def _snapshot_final_episode_state(self, env_ids: torch.Tensor) -> dict | None:
        """Capture everything ``_log_reset_extras`` needs from the
        just-ended episode, before ``_reset_idx`` overwrites any of it
        (teleport, buffer zeroing). Returns None on the very first reset
        (env creation, before ``_get_dones`` has ever run)."""
        if not hasattr(self, "_done_flags"):
            return None
        pos = self._robot.data.root_pos_w[env_ids] - self.scene.env_origins[env_ids]
        vel = self._robot.data.root_lin_vel_w[env_ids]
        return {
            "done_flags": {k: v[env_ids].clone() for k, v in self._done_flags.items()},
            "overshoot_flythrough": self._overshoot_flythrough[env_ids].clone(),
            "action_sat_frac": (
                self._action_sat_sum[env_ids]
                / self.episode_length_buf[env_ids].clamp(min=1).float()
            ).clone(),
            "final_d_xy": self._d_xy_prev[env_ids].clone(),
            "d_xy_min": self._d_xy_min[env_ids].clone(),
            "episode_sums": {k: v[env_ids].clone() for k, v in self._episode_sums.items()},
            "final_pos_xy": pos[:, :2].clone(),
            "final_vel_xy": vel[:, :2].clone(),
            "final_alt": pos[:, 2].clone(),
            "target_xy": self._target_xy[env_ids].clone(),
            "drag_coef": self._drag_coef[env_ids].clone(),
            "wind_xy": self._wind_xy[env_ids].clone(),
            "released": self._released[env_ids].clone(),
            "release_impact_err": self._release_impact_err[env_ids].clone(),
            "release_aim_xy": self._release_aim_xy[env_ids].clone(),
            "release_target_xy": self._release_target_xy[env_ids].clone(),
        }

    def _predicted_impact_from_snapshot(self, snap: dict) -> torch.Tensor:
        """CCIP impact-point prediction with the Phase-2 residual slot,
        evaluated from a pre-reset state snapshot (see
        ``_snapshot_final_episode_state``) so it reflects the terminated
        episode's actual final position/velocity. In Phase 1
        (``residual_enabled=False``) the residual addition is skipped
        entirely — pure passthrough, bit-identical to a hook-free
        implementation."""
        dc = self.cfg.drop
        impact = ballistic_impact(
            snap["final_pos_xy"], snap["final_vel_xy"], snap["final_alt"],
            dc.release_delay, dc.gravity, snap["drag_coef"], snap["wind_xy"],
        )
        if dc.residual_enabled:
            # Phase-2 only: the residual conditions on the terminal
            # observation, which is no longer reconstructable post-reset
            # from this snapshot alone — left as a Phase-2 TODO alongside
            # enabling the flag itself.
            impact = impact + ccip_residual(snap["final_pos_xy"])
        return impact

    def _log_reset_extras(self, env_ids: torch.Tensor, snap: dict | None):
        if snap is None:
            return
        f = snap["done_flags"]
        log = self.extras.setdefault("log", {})
        for cause in ("success", "crash", "overspeed", "bad_attitude", "out_of_range", "max_altitude", "overshoot", "stagnation", "timeout"):
            log[f"Episode_Termination/{cause}"] = f[cause].float().mean().item()
        log["Episode_Diag/overshoot_flythrough"] = snap["overshoot_flythrough"].float().mean().item()
        log["Episode_Termination/timeout_near_miss"] = (
            f["timeout"] & (snap["final_d_xy"] < 2.0)
        ).float().mean().item()

        log["Episode_Metric/action_sat_frac"] = snap["action_sat_frac"].mean().item()

        if self.cfg.release_enabled:
            # Phase 2/3: report the REAL (DR-physics) impact error measured at
            # the actual release event, plus how often the payload was dropped.
            released = snap["released"]
            log["Episode_Metric/release_rate"] = released.float().mean().item()
            if bool(released.any()):
                err_released = snap["release_impact_err"][released]
                log["Episode_Metric/drop_impact_error_m"] = err_released.mean().item()
                if self.cfg.moving_target_enabled:
                    lead_err = torch.linalg.norm(
                        snap["release_aim_xy"][released] - snap["release_target_xy"][released], dim=-1
                    )
                    log["Episode_Metric/lead_error_m"] = lead_err.mean().item()
        else:
            # Phase 1: the analytic CCIP prediction from the terminal state
            # (no real release event in the approach task).
            impact = self._predicted_impact_from_snapshot(snap)
            impact_error = torch.linalg.norm(impact - snap["target_xy"], dim=-1)
            log["Episode_Metric/drop_impact_error_m"] = impact_error.mean().item()

        d_xy_min = snap["d_xy_min"]
        log["Episode_Metric/d_xy_min"] = d_xy_min[torch.isfinite(d_xy_min)].mean().item() if torch.isfinite(d_xy_min).any() else 0.0
        for k, v in snap["episode_sums"].items():
            log[f"Episode_Reward/{k}"] = v.mean().item()
