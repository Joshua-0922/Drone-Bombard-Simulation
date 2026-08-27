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
from isaaclab.assets import Articulation, ArticulationCfg, RigidObject, RigidObjectCfg
from isaaclab.envs import DirectRLEnv, DirectRLEnvCfg
from isaaclab.scene import InteractiveSceneCfg
from isaaclab.sim import SimulationCfg
from isaaclab.utils import configclass

from isaaclab_assets import CRAZYFLIE_CFG

from .mdp.domain_rand import (
    sample_drag_coefficient,
    sample_wind,
    sample_target_velocity,
    sample_target_accel,
    sample_target_turn_rate,
    sample_scale,
    sample_bias,
)
from .math_utils import (
    rate_limit_action,
    lpf_step,
    project_target_pinhole,
    ballistic_impact,
    ccip_residual,
    predict_impact_nominal,
    apply_ccip_residual,
    time_to_fall,
    step_target_motion,
    impact_terminal_reward,
    lead_prediction_reward,
    aim_error_reward,
    compute_reward,
    overshoot_guard,
    stagnation_guard,
    hold_buffer_update,
    quat_apply_inverse_pure,
    pixel_to_ground_xy,
    singer_transition_matrix,
    singer_process_noise,
    kf_predict,
    kf_update_position,
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

    # Dense CCIP aim-error shaping (exp_017 Stage A). Every policy step:
    #   w_aim * (1 - tanh(aim_err / aim_reward_scale))
    # where aim_err is the release referee's |predicted_impact - aim_point|
    # (nominal ballistics — see _advance_phase_dynamics). Layered ON TOP of
    # w_dist (coarse "get close") as the fine "aim accurately" signal; touches
    # no termination/success gate. Default w_aim=0.0 keeps the term exactly
    # off — the validated exp_014 Phase-1 reward is unchanged unless train.py
    # opts in via --w_aim.
    w_aim: float = 0.0
    aim_reward_scale: float = 0.5  # metres (tanh knee; reward ~0 beyond ~1.5 m)


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
class DroneBombardDynDRCfg:
    """P0 dynamics/sensing domain randomization — the axes the wind/drag DR did
    NOT cover (see notes/research/paper_research_plan.md §5 P0-4).

    Design rule for every knob here: randomize what a real deployment cannot
    measure exactly, WITHOUT touching PhysX properties at runtime (Rule 19 —
    a runtime physics override that the controller does not see is exactly the
    exp_013 plant bug). Two equivalences make that possible:

    * ``ctrl_mass_rel`` randomizes the controller's mass BELIEF, not the plant
      mass. For the thrust-sizing loop the two are the same disturbance: with
      true mass m and belief m(1+d) the closed loop realises
      ``a = (1+d)*a_des + d*g``, identical in form to flying an unknown mass
      with a fixed belief. The plant stays exactly as authored at spawn.
    * ``payload_bc_rel`` randomizes the released payload's drag coefficient,
      which is EXACTLY equivalent to randomizing its mass: free flight obeys
      ``dv/dt = g + (k/m)|v_air|v_air``, so only the ballistic coefficient k/m
      enters the trajectory. Again no PhysX write.

    Sensor/actuator noise follows the two-timescale model (per-step white noise
    + per-EPISODE constant bias) — the bias term is the one that forces
    robustness to calibration offsets, since white noise averages out within an
    episode. Defaults are the Learning-to-Throw values (obs 0.01/0.001, act
    0.025/0.02 on the normalized vectors).

    ``enabled=False`` (default) makes every sampler return its identity WITHOUT
    drawing random numbers, so v11-v19 stay bit-identical."""

    enabled: bool = False

    # --- plant / controller mismatch (multiplicative, U[1-rel, 1+rel]) ---
    ctrl_mass_rel: float = 0.05        # controller mass belief vs the authored mass
    kp_vel_rel: float = 0.10           # outer loop: velocity P gains
    k_att_rate_rel: float = 0.10       # inner loop: attitude P + rate P gains
    payload_bc_rel: float = 0.20       # payload ballistic coefficient (== payload mass)

    # --- two-timescale observation noise (on the NORMALIZED obs vector) ---
    obs_noise_std: float = 0.01        # per policy step
    obs_bias_std: float = 0.001        # per episode, constant

    # --- two-timescale action noise (velocity dims [0:4] only) ---
    # Deliberately NOT applied to the drop-signal dim or the CCIP residual dims:
    # jittering the release decision or the guidance correction would confound
    # the two mechanisms the paper measures. This is the control channel.
    act_noise_std: float = 0.025       # per policy step
    act_bias_std: float = 0.02         # per episode, constant


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
    # Motion model for the target ("X marker"), selected via train.py
    # --target_motion:
    #   "gm" (default) — Gauss-Markov/OU velocity walk (legacy Phase-3
    #                    behavior, bit-identical to before this knob existed)
    #   "cv"          — constant velocity (straight line)
    #   "ca"          — constant acceleration (per-episode sampled accel)
    #   "ct"          — coordinated turn (per-episode signed turn rate,
    #                    |v| preserved, exact arc integration)
    target_motion_model: str = "gm"
    target_init_speed: float = 2.0   # m/s, initial |target velocity| ~ U[0, this]
    target_vel_theta: float = 0.3    # Gauss-Markov mean-reversion rate (1/s)
    target_vel_sigma: float = 0.5    # Gauss-Markov volatility
    target_accel_max: float = 0.5    # m/s^2, CA model: |a| ~ U[0, this]
    target_omega_range: tuple[float, float] = (0.2, 0.6)  # rad/s, CT model: |w| ~ U[lo, hi], random sign

    # --- lead-prediction reward (Phase 3) ---
    w_lead: float = 10.0
    lead_reward_scale: float = 3.0  # m


@configclass
class DroneBombardTrackerCfg:
    """Singer (Gauss-Markov acceleration) Kalman-filter target tracker.

    State per env: ``[x, y, vx, vy, ax, ay]`` with the acceleration a
    first-order Gauss-Markov process (correlation time ``tau``, steady-state
    std ``sigma_a``) — the classic Singer maneuvering-target model, which
    tracks CV targets (a ~= 0), CA targets (slowly-varying a) and
    approximates CT turns. Measurements are the YOLO-pipeline detections
    back-projected to the ground plane (``pixel_to_ground_xy``), applied only
    on FRESH detections (hold-buffer repeats are stale copies, not new
    information) with range-dependent measurement noise
    ``sigma = max(pixel_noise_std * slant / fx, meas_std_floor)``.

    The tracker is purely observational: it feeds the policy observation
    (``DroneBombardEnvCfg.target_kf_obs``) and diagnostics, and never touches
    the privileged referee/reward quantities."""

    tau: float = 1.0        # s, Gauss-Markov acceleration correlation time
    sigma_a: float = 1.0    # m/s^2, steady-state maneuver-acceleration std
    meas_std_floor: float = 0.15  # m, lower bound on the ground-plane measurement std
    # initial covariance (applied when the filter (re)initializes on the
    # first fresh detection of an episode)
    init_pos_var: float = 1.0    # m^2
    init_vel_var: float = 4.0    # (m/s)^2
    init_acc_var: float = 1.0    # (m/s^2)^2
    acc_obs_scale: float = 3.0   # m/s^2, normalization for the accel obs dims


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

    # Force the moving target ON regardless of phase (train.py
    # --moving_target). Lets Phase-1/2 tasks face a moving X marker without
    # dragging in the Phase-3 residual/DR/release machinery. Independent of
    # ``phase`` like release_terminal; default False = phase-derived only.
    moving_target_force: bool = False

    # Singer/Gauss-Markov Kalman target tracker in the observation (train.py
    # --target_kf). When True the observation grows 14 -> 21 with the
    # tracker's estimate (see _get_observations); checkpoints trained without
    # it CANNOT be warm-started into it (input layer differs) — fresh start.
    target_kf_obs: bool = False
    tracker: DroneBombardTrackerCfg = DroneBombardTrackerCfg()

    # --- airframe wind disturbance (v15 hook; INERT unless enabled) ---
    # Until this is on, ``_wind_xy`` only ever displaced the PAYLOAD's ballistic
    # impact — the drone itself flew in perfectly still air, so a policy could
    # beat the wind purely by repositioning. With it on, the relative airflow
    # exerts quadratic drag on the airframe and the controller must fight it.
    # Not phase-derived (unlike the flags above), so __post_init__ leaves it be;
    # default False keeps v11-v14 and the base phases bit-identical.
    wind_force_enabled: bool = False
    wind_drag_k: float = 0.06  # N per (m/s)^2 = 0.5*rho*Cd*A (~0.1 m^2 frontal-area quad)

    # --- physical payload drop (v16 hook; INERT unless enabled) ---
    # Until this is on, the "payload" is an analytic ballistic point and a visual
    # cylinder — nothing physically falls. With it on, a real RigidObject is
    # carried under the drone, released on the drop event, and falls under gravity
    # + wind drag; the LANDING point (not a formula) drives the outcome — the
    # thing that will actually matter once the wind is time-varying (the falling
    # payload's trajectory then genuinely bends mid-air).
    payload_physics_enabled: bool = False
    payload_phys_mass: float = 0.1
    payload_phys_radius: float = 0.05
    payload_phys_height: float = 0.06
    payload_phys_drag_k: float = 0.005  # N per (m/s)^2 = 0.5*rho*Cd*A for the ~0.008 m^2 payload
    payload_mount_z: float = -0.14     # carry offset below the drone body (m)
    payload_ground_z: float = 0.0      # landing plane (local frame)
    # Height above the plane at which the impact is latched. MUST exceed the
    # payload's resting height, or the latch can never fire: a cylinder at rest
    # on the ground plane sits at half its height (0.03 m here) and the solver
    # prevents penetration, so a `z <= 0.0` test only ever fires when a payload
    # happens to tunnel through in one substep. Measured 2026-08-03: with the
    # 0.0 test, 0/32 forced releases latched and every payload came to rest at
    # exactly 0.0300 m; in evaluation this showed up as episodes with
    # released=True that ran to the 30 s timeout (17/200 even on v19-on-v19).
    # 0.10 m is the value exp_019 validated the analytic-vs-measured impact
    # parity at (|delta| <= 0.021 m).
    payload_land_eps: float = 0.10

    # Stage B (exp_018): scripted-release-as-terminal for Phase 1. When True,
    # the Phase-1 CCIP referee's fire event ENDS the episode as the success
    # condition (replacing d_xy<=success_radius proximity success) — the drone
    # may loiter near the target, still collecting the dense per-step rewards,
    # until it aims well enough to release or a failure gate/timeout fires.
    # All failure gates (crash/overspeed/bad_attitude/out_of_range/
    # max_altitude/overshoot/stagnation) and the timeout path are UNTOUCHED.
    # Deliberately independent of ``phase`` (NOT derived in __post_init__):
    # Phase 2+ has its own release termination via release_enabled, and this
    # knob must not drag in the Phase-2 residual/DR machinery. Default False
    # = the exp_014/exp_017 legacy proximity task, bit-identical.
    release_terminal: bool = False

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
    dyn_dr: DroneBombardDynDRCfg = DroneBombardDynDRCfg()

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
        self.moving_target_enabled = (self.phase >= 3) or self.moving_target_force
        if self.target_kf_obs:
            self.observation_space = 21  # 14 base + 7 tracker dims


# =====================================================================
# Env
# =====================================================================


class DroneBombardEnv(DirectRLEnv):
    cfg: DroneBombardEnvCfg

    def __init__(self, cfg: DroneBombardEnvCfg, render_mode: str | None = None, **kwargs):
        # The observation space is consumed inside super().__init__ (space
        # construction), and train.py sets cfg.target_kf_obs AFTER the cfg's
        # __post_init__ already ran — so the tracker-obs resize must happen
        # here, before the DirectRLEnv machinery reads it.
        if cfg.target_kf_obs:
            cfg.observation_space = 21  # 14 base + 7 tracker dims
        super().__init__(cfg, render_mode, **kwargs)

        # Re-derive the per-phase flags from cfg.phase here too: train.py sets
        # ``env_cfg.phase`` AFTER the cfg is constructed (so cfg.__post_init__
        # already ran with the default phase=1). Recomputing on the stored cfg
        # makes every ``self.cfg.<flag>`` read below authoritative.
        self.cfg.residual_enabled = self.cfg.phase >= 2
        self.cfg.dr_enabled = self.cfg.phase >= 2
        self.cfg.release_enabled = self.cfg.phase >= 2
        self.cfg.moving_target_enabled = (self.cfg.phase >= 3) or self.cfg.moving_target_force

        N, device = self.num_envs, self.device
        # 6-dim action pipeline: [0:4] velocity setpoint, [4:6] CCIP residual.
        self._prev_action = torch.zeros(N, 6, device=device)
        self._v_filt = torch.zeros(N, 4, device=device)  # velocity LPF only tracks the 4 vel dims
        self._lpf_snap = torch.ones(N, dtype=torch.bool, device=device)
        self._physics_tick = 0
        self._residual_action = torch.zeros(N, 2, device=device)  # latest action[4:6]

        self._target_xy = torch.zeros(N, 2, device=device)
        self._target_vel_xy = torch.zeros(N, 2, device=device)  # Phase 3 moving target
        # Per-episode motion-model parameters (constant within an episode,
        # resampled in _reset_idx; read only by their own model — CA reads
        # _target_acc_xy, CT reads _target_omega, CV/GM read neither).
        self._target_acc_xy = torch.zeros(N, 2, device=device)
        self._target_omega = torch.zeros(N, device=device)

        # Singer-KF target tracker state (allocated always — negligible
        # memory — but stepped only when cfg.target_kf_obs is on).
        # _kf_x: [x, y, vx, vy, ax, ay] in env-local ENU; _kf_valid latches
        # True at the first fresh detection of the episode (until then the
        # tracker obs dims read zero with validity 0).
        self._kf_x = torch.zeros(N, 6, device=device)
        self._kf_P = torch.zeros(N, 6, 6, device=device)
        self._kf_valid = torch.zeros(N, dtype=torch.bool, device=device)
        tr = self.cfg.tracker
        policy_dt = self.cfg.sim.dt * self.cfg.decimation
        self._kf_F = singer_transition_matrix(policy_dt, tr.tau, device=device)
        self._kf_Q = singer_process_noise(policy_dt, tr.tau, tr.sigma_a, device=device)
        self._kf_P0 = torch.diag(torch.tensor(
            [tr.init_pos_var, tr.init_pos_var, tr.init_vel_var,
             tr.init_vel_var, tr.init_acc_var, tr.init_acc_var], device=device))
        # fresh-detection info stashed by _update_vision for the tracker
        # (raw noisy pixels BEFORE the hold buffer — hold repeats are stale)
        self._fresh_detected = torch.zeros(N, dtype=torch.bool, device=device)
        self._fresh_u_px = torch.zeros(N, device=device)
        self._fresh_v_px = torch.zeros(N, device=device)
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

        # v16 physical-payload state (buffers always exist; only used when the
        # cfg hook is on). _payload_free: released and falling (physics active);
        # _payload_landed: has touched the ground; _payload_impact_xy: the actual
        # (local-frame) landing point that drives the terminal reward.
        self._payload_free = torch.zeros(N, dtype=torch.bool, device=device)
        self._payload_landed = torch.zeros(N, dtype=torch.bool, device=device)
        self._payload_impact_xy = torch.zeros(N, 2, device=device)
        # Per-episode minimum CCIP aim error |predicted_impact - target| (all
        # phases). Diagnoses 10 Hz trigger discretization: at approach speed v
        # the predicted impact point sweeps ~v*0.1 m per policy step, so the
        # release window can be crossed between two evaluations — release_rate
        # then under-reports while this stays just above the tolerance.
        self._aim_err_min = torch.full((N,), float("inf"), device=device)
        # Feasible release window: how many policy steps of THIS episode a
        # release would have been admissible (the release envelope open, payload
        # still carried). A wide window = a timing-robust approach; a 1-step
        # window means the whole outcome hinges on hitting one control tick.
        # Incremented wherever the fire condition is evaluated.
        self._window_steps = torch.zeros(N, device=device)
        # This step's CCIP aim error, written by _advance_phase_dynamics at
        # the top of _get_dones — the same dones-before-rewards DirectRLEnv
        # contract _done_flags relies on — and consumed by _get_rewards for
        # the dense aim shaping (w_aim). inf until the first evaluation, which
        # aim_error_reward maps to exactly 0.
        self._aim_err_last = torch.full((N,), float("inf"), device=device)

        self._episode_sums = {
            k: torch.zeros(N, device=device)
            for k in ("rew_ctrl", "rew_dist", "rew_orient", "rew_proximity", "rew_vision", "rew_vel", "rew_aim")
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

        self._ctrl_mass_nominal = float(self._robot.root_physx_view.get_masses()[0].sum())
        if self._ctrl_mass_nominal < 1.0:
            raise RuntimeError(
                f"spawn-time mass authoring did not reach PhysX (total mass "
                f"{self._ctrl_mass_nominal:.4f} kg — expected ~{self.cfg.asset.loaded_mass:.2f} kg). "
                "The controller would fly the native 28 g Crazyflie shell; aborting."
            )
        # max_thrust stays a SCALAR: thrust-to-weight is a physical airframe/motor
        # property, not something the controller can be wrong about. Only the mass
        # BELIEF (and the P gains) are randomized — see DroneBombardDynDRCfg.
        self._max_thrust = self.cfg.asset.thrust_to_weight_unloaded * self._ctrl_mass_nominal * 9.81
        self._k_rate = torch.tensor(self.cfg.controller.k_rate, device=self.device)

        # --- per-env dynamics-DR state (identity until cfg.dyn_dr.enabled) ---
        # _ctrl_mass is per-env so the controller can fly a WRONG mass belief; the
        # authored plant mass is untouched (Rule 19: never override PhysX props at
        # runtime). All of these are resampled in _reset_idx.
        self._ctrl_mass = torch.full((N,), self._ctrl_mass_nominal, device=device)
        self._kp_vel_scale = torch.ones(N, device=device)
        self._k_att_rate_scale = torch.ones(N, device=device)
        self._payload_bc_scale = torch.ones(N, device=device)
        self._obs_bias = torch.zeros(N, int(self.cfg.observation_space), device=device)
        self._act_bias = torch.zeros(N, 4, device=device)
        print(f"[DroneBombardEnv] control mass={self._ctrl_mass_nominal:.3f}kg max_thrust={self._max_thrust:.2f}N "
              f"inertia_diag={self._inertia_diag.detach().cpu().numpy()} body_id={self._body_id} "
              f"dyn_dr={'ON' if self.cfg.dyn_dr.enabled else 'off'}")

        self._setup_markers()

    def _setup_markers(self):
        """Optional visualization: payload cylinder (measured payload_cylinder
        SDF dims) riding under the drone + an X-marker plate on the ground at
        the target. Visual-only (VisualizationMarkers) — no physics, no cost
        unless cfg.show_markers is True. Used for recording/eval."""
        self._payload_marker = None
        self._target_marker = None
        self._target_pole = None
        self._payload_hl = None
        if not self.cfg.show_markers:
            return
        from isaaclab.markers import VisualizationMarkers, VisualizationMarkersCfg

        # Visual payload marker ONLY for the analytic drop (v11-v18) — when there
        # is a real physics payload (v16/v19) the RigidObject is already visible,
        # so this marker would just duplicate it (two cylinders under the drone).
        if not self.cfg.payload_physics_enabled:
            self._payload_marker = VisualizationMarkers(VisualizationMarkersCfg(
                prim_path="/Visuals/Payload",
                markers={"payload": sim_utils.CylinderCfg(
                    radius=0.05, height=0.06, axis="Z",
                    visual_material=sim_utils.PreviewSurfaceCfg(diffuse_color=(0.95, 0.45, 0.05)),
                )},
            ))
        # For the physics payload (v16/v19) we do NOT add a highlight marker: the
        # real RigidObject is shown at its true size so the drop reads honestly
        # (an oversized sphere misrepresents the object). _payload_hl stays None.
        # Ground disc = the success zone; a tall bright pole makes the target
        # visible from the side (a flat ground disc foreshortens to nothing in the
        # chase camera). Both are visual-only, gated by show_markers.
        self._target_marker = VisualizationMarkers(VisualizationMarkersCfg(
            prim_path="/Visuals/Target",
            markers={"target": sim_utils.CylinderCfg(
                radius=1.0, height=0.02, axis="Z",
                visual_material=sim_utils.PreviewSurfaceCfg(diffuse_color=(0.9, 0.1, 0.1)),
            )},
        ))
        self._target_pole = VisualizationMarkers(VisualizationMarkersCfg(
            prim_path="/Visuals/TargetPole",
            markers={"pole": sim_utils.CylinderCfg(
                radius=0.12, height=6.0, axis="Z",
                visual_material=sim_utils.PreviewSurfaceCfg(
                    diffuse_color=(1.0, 0.15, 0.7), emissive_color=(0.7, 0.05, 0.4)),
            )},
        ))

    def _update_markers(self):
        if self._target_marker is None:  # show_markers off
            return
        pos_w = self._robot.data.root_pos_w
        # Visual payload marker (analytic drop only): ride under the drone while
        # attached. Skipped entirely when a physics payload exists (marker is None).
        if self._payload_marker is not None and bool(self._payload_attached[0]):
            payload_pos = pos_w.clone()
            payload_pos[:, 2] += self.cfg.drop.payload_mount_offset_z  # ride under the drone
            self._payload_marker.visualize(translations=payload_pos)
        # physics-payload highlight: tracks the real RigidObject (carried under the
        # drone, then falls on release) so you can actually see it separate + land.
        if self._payload_hl is not None:
            self._payload_hl.visualize(translations=self._payload.data.root_pos_w.clone())
        target_pos = torch.zeros_like(pos_w)
        target_pos[:, :2] = self._target_xy + self.scene.env_origins[:, :2]
        target_pos[:, 2] = 0.01
        self._target_marker.visualize(translations=target_pos)
        if self._target_pole is not None:
            pole_pos = target_pos.clone()
            pole_pos[:, 2] = 3.0  # h=6 cylinder, centre at 3 -> base sits on the ground
            self._target_pole.visualize(translations=pole_pos)

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

        if self.cfg.payload_physics_enabled:
            pc = self.cfg
            payload_cfg = RigidObjectCfg(
                prim_path="/World/envs/env_.*/Payload",
                spawn=sim_utils.CylinderCfg(
                    radius=pc.payload_phys_radius, height=pc.payload_phys_height, axis="Z",
                    rigid_props=sim_utils.RigidBodyPropertiesCfg(),
                    mass_props=sim_utils.MassPropertiesCfg(mass=pc.payload_phys_mass),
                    collision_props=sim_utils.CollisionPropertiesCfg(),
                    visual_material=sim_utils.PreviewSurfaceCfg(diffuse_color=(0.95, 0.45, 0.05)),
                ),
                init_state=RigidObjectCfg.InitialStateCfg(pos=(0.0, 0.0, 10.0)),
            )
            self._payload = RigidObject(payload_cfg)
            self.scene.rigid_objects["payload"] = self._payload

        spawn_ground = sim_utils.GroundPlaneCfg()
        spawn_ground.func("/World/ground", spawn_ground)
        light_cfg = sim_utils.DomeLightCfg(intensity=2000.0, color=(0.9, 0.9, 0.9))
        light_cfg.func("/World/Light", light_cfg)

        if self.cfg.show_markers:
            # Keep the grid (gives motion/position reference); add a tilted sun so
            # the drone/payload cast shadows on it -> a depth cue for the drop.
            sun = sim_utils.DistantLightCfg(intensity=1800.0, angle=1.0, color=(1.0, 0.97, 0.92))
            sun.func("/World/Sun", sun, orientation=(0.94, 0.0, -0.34, 0.0))

        self.scene.clone_environments(copy_from_source=False)
        if self.device == "cpu":
            self.scene.filter_collisions(global_prim_paths=[])

    # ------------------------------------------------------------------
    # Sensing / actuation noise (two-timescale: per-step white + per-episode bias)
    # ------------------------------------------------------------------
    def _perturb_obs(self, obs: torch.Tensor) -> torch.Tensor:
        """Add the per-episode observation bias + per-step observation noise.

        Applied to the WHOLE normalized observation vector, including the binary
        flag channels (payload_attached / detected) and the prev-action echo. At
        the default magnitudes (bias 1e-3, noise 1e-2) a flag reads 0.99 vs 0.01
        — unambiguous — so the alternative (a per-version index mask that breaks
        every time the obs layout changes) buys nothing.

        Identity passthrough when dyn_dr is off, so this is safe to call from any
        observation builder."""
        dd = self.cfg.dyn_dr
        if not dd.enabled:
            return obs
        if obs.shape[-1] != self._obs_bias.shape[-1]:
            raise RuntimeError(
                f"observation width {obs.shape[-1]} != cfg.observation_space "
                f"{self._obs_bias.shape[-1]} — the dyn_dr obs bias buffer was "
                "allocated from the cfg; fix the cfg before enabling dyn_dr."
            )
        out = obs + self._obs_bias
        if dd.obs_noise_std > 0.0:
            out = out + torch.randn_like(out) * dd.obs_noise_std
        return out

    def _perturb_actions(self, actions: torch.Tensor) -> torch.Tensor:
        """Add the per-episode actuation bias + per-step actuation noise to the
        VELOCITY dims [0:4] only (see DroneBombardDynDRCfg for why the drop
        signal and the CCIP residual are deliberately left clean). Runs BEFORE
        the saturation accounting and the [-1, 1] clip, i.e. the noise can push
        a command over the rail exactly like a real actuator would."""
        dd = self.cfg.dyn_dr
        if not dd.enabled:
            return actions
        pert = self._act_bias
        if dd.act_noise_std > 0.0:
            pert = pert + torch.randn_like(pert) * dd.act_noise_std
        out = actions.clone()
        out[:, :4] = out[:, :4] + pert
        return out

    # ------------------------------------------------------------------
    # Action pipeline
    # ------------------------------------------------------------------
    def _pre_physics_step(self, actions: torch.Tensor):
        actions = self._perturb_actions(actions)
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

        if self.cfg.payload_physics_enabled:
            self._step_payload_physics()

    @property
    def payload_bc_over_m(self) -> torch.Tensor:
        """Per-env ballistic coefficient k/m of the released payload, in 1/m.

        Free flight depends on the drag constant and the mass ONLY through this
        ratio, so this is the single physical parameter of the payload's fall
        and the single number a predictor needs. ``_payload_bc_scale`` (1.0
        unless dyn_dr) randomizes it, which is equivalent to randomizing the
        payload mass without writing to PhysX (Rule 19).

        Read this from any impact predictor (see
        ``math_utils.integrate_payload_impact``); do not rebuild it from
        ``cfg.payload_phys_drag_k / cfg.payload_phys_mass``, or the predictor
        and the plant can drift apart the way the CCIP vz term did.
        """
        return (self.cfg.payload_phys_drag_k * self._payload_bc_scale) / self.cfg.payload_phys_mass

    def _step_payload_physics(self):
        """v16: carry the payload under the drone until release, then let it fall
        under gravity + wind drag and record where it lands. Runs at physics rate
        (100 Hz) from _apply_action; gated by cfg.payload_physics_enabled.

        _payload_free is flipped True by the env's release logic on the drop step.
        """
        origins = self.scene.env_origins

        # carried (attached, not yet released): kinematically ride the drone.
        carried = self._payload_attached & ~self._payload_free
        if carried.any():
            idx = carried.nonzero(as_tuple=False).squeeze(-1)
            root = self._robot.data.root_state_w[idx]  # pos(3) quat(4) linvel(3) angvel(3)
            pose = root[:, :7].clone()
            pose[:, 2] = pose[:, 2] + self.cfg.payload_mount_z
            vel = torch.zeros((idx.numel(), 6), device=self.device)
            vel[:, :3] = root[:, 7:10]  # inherit the drone's linear velocity
            self._payload.write_root_pose_to_sim(pose, idx)
            self._payload.write_root_velocity_to_sim(vel, idx)

        # free (released, still airborne): wind drag on the relative airflow.
        active = self._payload_free & ~self._payload_landed
        forces = torch.zeros(self.num_envs, 1, 3, device=self.device)
        if active.any():
            v = self._payload.data.root_lin_vel_w
            v_air = torch.stack([self._wind_xy[:, 0] - v[:, 0],
                                 self._wind_xy[:, 1] - v[:, 1],
                                 -v[:, 2]], dim=-1)
            speed = torch.linalg.norm(v_air, dim=-1, keepdim=True)
            # _payload_bc_scale (1.0 unless dyn_dr) randomizes the ballistic
            # coefficient k/m — equivalent to randomizing the payload MASS, since
            # free flight depends on the pair only through this ratio. No PhysX
            # mass write, so the authored plant stays exactly as spawned.
            # k/m is the ONLY free-flight parameter, so it is defined once in
            # payload_bc_over_m and multiplied back up to a force here. Any
            # predictor that wants the real trajectory must read that property
            # rather than re-deriving it from cfg (Rule 30).
            f = (self.payload_bc_over_m.unsqueeze(-1) * self.cfg.payload_phys_mass) * speed * v_air
            forces[:, 0, :] = torch.where(active.unsqueeze(-1), f, torch.zeros_like(f))
        # is_global=True: `f` above is assembled from world-frame wind and
        # world-frame payload velocity, so it is a world-frame vector. The API
        # defaults to is_global=False, i.e. "interpret this in the body's link
        # frame" — which silently rotated the drag by the payload's attitude
        # (inherited from a drone tilting up to 35 deg). Fixed 2026-08-26.
        self._payload.set_external_force_and_torque(forces, torch.zeros_like(forces), is_global=True)
        self._payload.write_data_to_sim()

        # landing: local-frame z at/below the ground plane.
        if active.any():
            z_local = self._payload.data.root_pos_w[:, 2] - origins[:, 2]
            newly = active & (z_local <= self.cfg.payload_ground_z + self.cfg.payload_land_eps)
            if newly.any():
                self._payload_impact_xy[newly] = (
                    self._payload.data.root_pos_w[newly, :2] - origins[newly, :2])
                self._payload_landed[newly] = True
                idx = newly.nonzero(as_tuple=False).squeeze(-1)
                self._payload.write_root_velocity_to_sim(
                    torch.zeros((idx.numel(), 6), device=self.device), idx)

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

        # Per-env gain scales (all 1.0 unless cfg.dyn_dr.enabled) model a
        # controller that is not perfectly tuned for THIS airframe.
        kp_vel = self._kp_vel_scale.unsqueeze(-1)

        vel_err_xy = v_filt[:, :2] - vel_w[:, :2]
        vel_err_z = v_filt[:, 2] - vel_w[:, 2]
        accel_des_xy = torch.clamp(c.kp_vel_xy * kp_vel * vel_err_xy, min=-c.accel_xy_clamp, max=c.accel_xy_clamp)
        accel_des_z = torch.clamp(
            c.kp_vel_z * self._kp_vel_scale * vel_err_z, min=-c.accel_z_clamp, max=c.accel_z_clamp)

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
        rate_sp[:, 0] = c.k_att_rp * self._k_att_rate_scale * rot_err_b[:, 0]
        rate_sp[:, 1] = c.k_att_rp * self._k_att_rate_scale * rot_err_b[:, 1]
        # yaw-rate is commanded directly (body z); roll/pitch attitude-controlled
        rate_sp[:, 2] = v_filt[:, 3]

        # rate loop outputs a desired ANGULAR ACCELERATION (k_rate has units
        # 1/s); the applied torque is inertia * angular_accel (tau = I*alpha).
        # Skipping the inertia term makes torque ~1/I too large — for this
        # airframe's ~0.022 kg.m^2 that is a ~46x over-torque that instantly
        # spins the drone past the attitude limits.
        rate_err = rate_sp - ang_vel_b
        ang_accel_des = self._k_rate * self._k_att_rate_scale.unsqueeze(-1) * rate_err
        torque_b = self._inertia_diag * ang_accel_des

        forces_b = torch.zeros(self.num_envs, 1, 3, device=self.device)
        forces_b[:, 0, 2] = thrust_mag  # body +Z

        # --- airframe wind disturbance (inert unless cfg.wind_force_enabled) ---
        # Quadratic drag from the RELATIVE airflow (air velocity as seen by the
        # drone), applied in the world frame then rotated into the body frame to
        # join the thrust. This is what makes the wind physically push the
        # airframe, so holding position costs a tilt of ~atan(F_wind/(m*g))
        # instead of being free.
        if self.cfg.wind_force_enabled:
            v_air_w = torch.zeros_like(vel_w)
            v_air_w[:, :2] = self._wind_xy - vel_w[:, :2]
            air_speed = torch.linalg.norm(v_air_w, dim=-1, keepdim=True)
            f_wind_w = self.cfg.wind_drag_k * air_speed * v_air_w
            forces_b[:, 0, :] += quat_apply_inverse_pure(quat_w, f_wind_w)

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

        # Stash the FRESH detection (raw noisy pixel, pre-hold-buffer) for the
        # Singer-KF target tracker: hold-buffer repeats are stale copies of an
        # old measurement, so feeding them to the filter would double-count.
        self._fresh_detected = detected
        self._fresh_u_px = u_px
        self._fresh_v_px = v_px

        fresh_uv_conf = torch.stack([u_px, v_px, conf], dim=-1)
        u_out, v_out, conf_out, self._held_uv_conf, self._hold_remaining = hold_buffer_update(
            detected, fresh_uv_conf, self._held_uv_conf, self._hold_remaining, vc.hold_frames,
        )
        return u_out, v_out, conf_out

    # ------------------------------------------------------------------
    # Target tracker (Singer/Gauss-Markov KF over YOLO detections)
    # ------------------------------------------------------------------
    def _update_target_tracker(self, pos_local: torch.Tensor, quat_w: torch.Tensor):
        """One tracker tick, run every policy step (from _get_observations,
        right after _update_vision): Singer-KF predict, then a measurement
        update on envs with a FRESH detection this step. The measurement is
        the noisy detection pixel back-projected to the ground plane — the
        same pixel the observation shows, so the tracker sees exactly what a
        real YOLO+camera stack would. Envs without a fresh detection coast on
        the prediction (covariance grows), which is precisely what lets the
        policy keep an aim estimate through dropout/occlusion."""
        tr = self.cfg.tracker
        vc = self.cfg.vision

        # predict (all initialized envs)
        x_pred, P_pred = kf_predict(self._kf_x, self._kf_P, self._kf_F, self._kf_Q)
        init_mask = self._kf_valid
        self._kf_x = torch.where(init_mask.unsqueeze(-1), x_pred, self._kf_x)
        self._kf_P = torch.where(init_mask.unsqueeze(-1).unsqueeze(-1), P_pred, self._kf_P)

        # measurement: fresh detections back-projected to the ground plane
        z_xy, backproj_ok = pixel_to_ground_xy(
            self._fresh_u_px, self._fresh_v_px, pos_local, quat_w,
            vc.fx, vc.fy, vc.cx, vc.cy,
        )
        meas = self._fresh_detected & backproj_ok

        # range-dependent measurement noise: one pixel of noise spans
        # ~ slant/fx metres on the ground (near-nadir), floored so a
        # point-blank detection never claims impossible precision.
        rel = z_xy - pos_local[:, :2]
        slant = torch.sqrt((rel * rel).sum(dim=-1) + pos_local[:, 2] ** 2)
        sigma = (vc.pixel_noise_std * slant / vc.fx).clamp(min=tr.meas_std_floor)
        r_var = sigma * sigma

        # first fresh detection of the episode initializes the filter
        init_now = meas & ~self._kf_valid
        if bool(init_now.any()):
            x0 = torch.zeros_like(self._kf_x)
            x0[:, 0:2] = z_xy
            self._kf_x = torch.where(init_now.unsqueeze(-1), x0, self._kf_x)
            self._kf_P = torch.where(
                init_now.unsqueeze(-1).unsqueeze(-1),
                self._kf_P0.expand_as(self._kf_P), self._kf_P)
            self._kf_valid = self._kf_valid | init_now

        # standard update on already-initialized envs with a fresh detection
        upd = meas & ~init_now & self._kf_valid
        if bool(upd.any()):
            x_new, P_new = kf_update_position(self._kf_x, self._kf_P, z_xy, r_var)
            self._kf_x = torch.where(upd.unsqueeze(-1), x_new, self._kf_x)
            self._kf_P = torch.where(upd.unsqueeze(-1).unsqueeze(-1), P_new, self._kf_P)

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

        if self.cfg.target_kf_obs:
            # Singer-KF tracker estimate (obs[14:21]): relative estimated
            # target position (2), estimated target velocity (2), estimated
            # target acceleration (2), validity (1). Same normalization
            # conventions as the base dims (rel = drone - target, pos_scale/
            # vel_scale); all-zero until the first fresh detection, so a
            # blind policy sees an explicit "no track" signal rather than a
            # stale estimate.
            self._update_target_tracker(pos, quat_w=self._robot.data.root_quat_w)
            tr = self.cfg.tracker
            valid = self._kf_valid.unsqueeze(-1).float()
            kf_rel = torch.clamp(
                (pos[:, 0:2] - self._kf_x[:, 0:2]) / oc.pos_scale, -1.0, 1.0) * valid
            kf_vel = torch.clamp(self._kf_x[:, 2:4] / oc.vel_scale, -1.0, 1.0) * valid
            kf_acc = torch.clamp(self._kf_x[:, 4:6] / tr.acc_obs_scale, -1.0, 1.0) * valid
            obs = torch.cat([obs, kf_rel, kf_vel, kf_acc, valid], dim=-1)

        obs = torch.nan_to_num(obs, nan=0.0)
        return {"policy": self._perturb_obs(obs)}

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
        (Phase 2+). In Phase 1 it runs the scripted CCIP release referee —
        metric-only by default; terminal when ``cfg.release_terminal`` is set
        (Stage B, exp_018)."""
        self._just_released = torch.zeros(self.num_envs, dtype=torch.bool, device=self.device)

        if self.cfg.moving_target_enabled:
            dt = self.cfg.sim.dt * self.cfg.decimation  # policy-step period (0.1 s @ 10 Hz)
            pc = self.cfg.phase_cfg
            self._target_xy, self._target_vel_xy = step_target_motion(
                self._target_xy, self._target_vel_xy,
                self._target_acc_xy, self._target_omega, dt,
                pc.target_motion_model,
                theta=pc.target_vel_theta, sigma=pc.target_vel_sigma,
            )

        if self.cfg.release_enabled:
            self._evaluate_release()
        else:
            self._evaluate_scripted_release_metric()

    def _evaluate_scripted_release_metric(self):
        """Phase-1 scripted CCIP referee (v15 ``drop_calculator_node`` parity).
        Latches ``_released`` / ``_release_impact_err`` at the first step the
        nominal predicted impact point comes within ``drop.release_tolerance``
        (0.2 m) of the target, i.e. where the Gazebo referee would actually
        have dropped the payload.

        Two modes (cfg.release_terminal):
          * False (default): METRIC ONLY — zero reward/termination effect,
            bit-identical to the exp_014/exp_017 proximity task.
          * True (Stage B, exp_018): the fire event also raises
            ``_just_released``, which ``_get_dones`` turns into the terminal
            success condition (release-as-terminal-event).

        DELIBERATE (reward-hacking guard): ``aim_err`` here — the quantity
        both this trigger and the dense aim reward (``_aim_err_last`` ->
        ``rew_aim``) consume — is computed from ``predict_impact_nominal``
        ONLY, i.e. the physically-grounded ballistic prediction from the
        drone's actual position/velocity. It must NEVER include the learned
        residual or any other unconstrained policy output the policy could
        shift without flying differently (see the matching guard in
        ``_evaluate_release``).

        Rationale (exp_014 forensics): Phase-1 ``drop_impact_error_m`` used to
        be computed from the SUCCESS-termination state (d_xy <= 0.8), where
        the policy still carries ~3 m/s of approach velocity — the ballistic
        prediction then adds v*(t_fall+delay) ~ 4.6 m of carry, decoupling the
        metric from the 100% d_xy success. The release decision was never
        supposed to fire on raw d_xy; it fires on CCIP aim error (see
        research/ccip_release_decoupling). In metric-only mode (the default)
        ``_just_released`` is deliberately NOT set — that flag feeds the
        termination path, which stays untouched unless ``cfg.release_terminal``
        opts in (see the Two modes paragraph above)."""
        pc = self.cfg.phase_cfg
        dc = self.cfg.drop
        pos = self._robot.data.root_pos_w - self.scene.env_origins
        vel = self._robot.data.root_lin_vel_w
        pos_xy = pos[:, :2]
        vel_xy = vel[:, :2]
        altitude = pos[:, 2]

        pred_impact = predict_impact_nominal(pos_xy, vel_xy, vel[:, 2], altitude, dc.release_delay, dc.gravity)
        aim_err = torch.linalg.norm(pred_impact - self._target_xy, dim=-1)
        self._aim_err_min = torch.minimum(self._aim_err_min, aim_err)
        self._aim_err_last = aim_err

        fire = (
            (~self._released)
            & (altitude > pc.min_release_altitude)
            & (aim_err <= dc.release_tolerance)
        )
        # feasible release window (see _window_steps): fire already
        # excludes already-released envs, so this is exactly 'admissible now'.
        self._window_steps += fire.float()
        if self.cfg.release_terminal:
            # Stage B: the fire event terminates the episode via _get_dones.
            # (_advance_phase_dynamics zeroed _just_released at the top of
            # this step; in metric-only mode the flag is left untouched so
            # the legacy proximity task stays bit-identical.)
            self._just_released = fire
        if not bool(torch.any(fire)):
            return

        # Real impact under episode physics. Phase 1 has zero drag/wind, so
        # this equals the nominal prediction — kept as ballistic_impact for
        # semantic parity with the Phase-2 path (impact_err is always the
        # real-physics miss).
        real_impact = ballistic_impact(
            pos_xy, vel_xy, vel[:, 2], altitude, dc.release_delay, dc.gravity,
            self._drag_coef, self._wind_xy,
        )
        real_err = torch.linalg.norm(real_impact - self._target_xy, dim=-1)

        self._released = self._released | fire
        self._release_impact_err = torch.where(fire, real_err, self._release_impact_err)
        self._release_aim_xy = torch.where(fire.unsqueeze(-1), pred_impact, self._release_aim_xy)
        self._release_target_xy = torch.where(fire.unsqueeze(-1), self._target_xy, self._release_target_xy)

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
        pred_nominal = predict_impact_nominal(pos_xy, vel_xy, vel[:, 2], altitude, dc.release_delay, dc.gravity)
        pred_impact = apply_ccip_residual(pred_nominal, self._residual_action, pc.residual_scale)

        # Aim point = target position at the predicted impact time (lead).
        t_impact = time_to_fall(altitude, vel[:, 2], dc.gravity) + dc.release_delay
        target_at_impact = self._target_xy + self._target_vel_xy * t_impact.unsqueeze(-1)

        aim_err = torch.linalg.norm(pred_impact - target_at_impact, dim=-1)
        self._aim_err_min = torch.minimum(self._aim_err_min, aim_err)
        # DELIBERATE (reward-hacking guard, exp_018): the dense aim reward
        # (_aim_err_last -> rew_aim) is fed the NOMINAL-only error — never the
        # residual-corrected one. The residual is an unconstrained policy
        # output; paying reward on a quantity the policy can shift by
        # +/-residual_scale without flying differently would let it farm
        # rew_aim while flying anywhere. The release TRIGGER (fire) stays
        # residual-inclusive by design — biasing the release decision is the
        # residual's job — and aim_err_min keeps diagnosing that trigger
        # quantity. The lead target is env-computed (not policy-controllable),
        # so it remains part of the reward quantity.
        self._aim_err_last = torch.linalg.norm(pred_nominal - target_at_impact, dim=-1)
        fire = (
            (~self._released)
            & (altitude > pc.min_release_altitude)
            & (aim_err <= pc.release_tolerance)
        )
        # feasible release window (see _window_steps): fire already
        # excludes already-released envs, so this is exactly 'admissible now'.
        self._window_steps += fire.float()
        if not bool(torch.any(fire)):
            return

        # Real drop under domain-randomized physics -> true impact error.
        real_impact = ballistic_impact(
            pos_xy, vel_xy, vel[:, 2], altitude, dc.release_delay, dc.gravity,
            self._drag_coef, self._wind_xy,
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
            # clone: _reset_idx zeroes _just_released in-place for done envs
            # BEFORE step() returns — an alias here would silently flip the
            # cached _done_flags entries that post-step readers (play.py
            # cause counting) consume.
            released_this_step = self._just_released.clone()
        elif self.cfg.release_terminal:
            # Stage B (exp_018): Phase-1 scripted release as the terminal
            # success event. The payload is spent on release, so ANY referee
            # fire ends the episode; under nominal ballistics (zero drag/
            # wind) the real impact equals the aim point, so a fire
            # (aim_err <= 0.2 m) IS a successful drop. Proximity no longer
            # terminates — the drone may loiter near the target, still paid
            # by the dense terms, until it aims well enough to release. The
            # failure gates computed above are shared and untouched (the
            # stagnation guard doubles as the loiter cap: ~150 steps of no
            # d_xy progress after arrival still truncates).
            # clone (NOT an alias): _reset_idx sets _just_released[env_ids] =
            # False in-place before step() returns, which would zero the
            # cached _done_flags["success"]/["released"] for exactly the
            # fired envs — post-step readers (play.py) would then count every
            # release-success as no-cause. wandb was immune (snapshot clones
            # pre-reset) but the eval harness was not.
            success = self._just_released.clone()
            released_this_step = success
        else:
            # Phase 1 legacy: proximity success (validated exp_014 approach task).
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

        # Dense CCIP aim shaping (all phases; exact zeros when w_aim == 0,
        # the default — Phase-1 reward parity is preserved unless train.py
        # opts in). _aim_err_last is this step's NOMINAL-only CCIP error —
        # both referee paths stash |predict_impact_nominal - aim_point|,
        # never a residual-corrected value (see the reward-hacking guard
        # comments in _evaluate_scripted_release_metric/_evaluate_release).
        rew_aim = aim_error_reward(self._aim_err_last, rc.w_aim, rc.aim_reward_scale)
        reward = reward + rew_aim
        self._episode_sums["rew_aim"] += rew_aim

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

        # v16 physical-payload flags. Position needs no reset: _step_payload_physics
        # re-seats a carried payload under the drone on the next physics tick.
        self._payload_free[env_ids] = False
        self._payload_landed[env_ids] = False
        self._payload_impact_xy[env_ids] = 0.0
        self._release_target_xy[env_ids] = 0.0
        self._aim_err_min[env_ids] = float("inf")
        self._aim_err_last[env_ids] = float("inf")
        self._window_steps[env_ids] = 0.0

        self._action_sat_sum[env_ids] = 0.0

        # Domain randomization (model mismatch) is active only in Phase 2+;
        # the moving-target velocity only in Phase 3. Off -> identity (zero),
        # so Phase 1 reduces to the drag-free/wind-free stationary-target task.
        pc = self.cfg.phase_cfg
        self._drag_coef[env_ids] = sample_drag_coefficient(pc, n, device, self.cfg.dr_enabled)
        self._wind_xy[env_ids] = sample_wind(pc, n, device, self.cfg.dr_enabled)
        moving = self.cfg.moving_target_enabled
        self._target_vel_xy[env_ids] = sample_target_velocity(pc, n, device, moving)
        # Per-episode motion-model parameters. Sampled only for the model
        # that reads them (others get the zero identity) so cross-model runs
        # consume identical RNG streams for the shared samplers above.
        self._target_acc_xy[env_ids] = sample_target_accel(
            pc, n, device, moving and pc.target_motion_model == "ca")
        self._target_omega[env_ids] = sample_target_turn_rate(
            pc, n, device, moving and pc.target_motion_model == "ct")

        # P0 dynamics/sensing DR (plant-controller mismatch + two-timescale
        # noise). Every sampler is identity-without-RNG when dyn_dr is off, so
        # the pre-P0 versions keep their exact random streams.
        dd = self.cfg.dyn_dr
        on = dd.enabled
        self._ctrl_mass[env_ids] = self._ctrl_mass_nominal * sample_scale(n, device, dd.ctrl_mass_rel, on)
        self._kp_vel_scale[env_ids] = sample_scale(n, device, dd.kp_vel_rel, on)
        self._k_att_rate_scale[env_ids] = sample_scale(n, device, dd.k_att_rate_rel, on)
        self._payload_bc_scale[env_ids] = sample_scale(n, device, dd.payload_bc_rel, on)
        self._obs_bias[env_ids] = sample_bias(n, self._obs_bias.shape[-1], device, dd.obs_bias_std, on)
        self._act_bias[env_ids] = sample_bias(n, 4, device, dd.act_bias_std, on)

        # Target tracker: fully forget the previous episode's track (the
        # target teleported at reset — coasting the old state through would
        # feed the policy a confidently wrong estimate until re-detection).
        self._kf_x[env_ids] = 0.0
        self._kf_P[env_ids] = 0.0
        self._kf_valid[env_ids] = False
        self._fresh_detected[env_ids] = False

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
            # ENU vertical velocity — required by the full-vz CCIP closed form
            # (see math_utils.ballistic_impact HISTORY). Snapshotted alongside
            # the horizontal pair so the post-reset prediction reflects the
            # terminated episode's real descent rate, not an assumed hover.
            "final_vel_z": vel[:, 2].clone(),
            "final_alt": pos[:, 2].clone(),
            "target_xy": self._target_xy[env_ids].clone(),
            "drag_coef": self._drag_coef[env_ids].clone(),
            "wind_xy": self._wind_xy[env_ids].clone(),
            "released": self._released[env_ids].clone(),
            "release_impact_err": self._release_impact_err[env_ids].clone(),
            "release_aim_xy": self._release_aim_xy[env_ids].clone(),
            "release_target_xy": self._release_target_xy[env_ids].clone(),
            "aim_err_min": self._aim_err_min[env_ids].clone(),
            "target_vel_xy": self._target_vel_xy[env_ids].clone(),
            "kf_valid": self._kf_valid[env_ids].clone(),
            "kf_est": self._kf_x[env_ids].clone(),
            # Delivery time = episode start -> the event that scores the drop.
            # Both drop mechanisms END the episode on that event (analytic:
            # release-terminal; physical: payload landing), so the terminated
            # episode's step count IS the delivery time — no extra state needed.
            # This is the agility metric ("throw duration" in the aerial-throwing
            # literature); without it a hover-then-drop policy looks optimal.
            "deliver_time_s": (
                self.episode_length_buf[env_ids].float() * self.cfg.sim.dt * self.cfg.decimation
            ).clone(),
            "delivered": (self._released[env_ids] | self._payload_landed[env_ids]).clone(),
            "landed": self._payload_landed[env_ids].clone(),
            "feasible_window_s": (
                self._window_steps[env_ids] * self.cfg.sim.dt * self.cfg.decimation
            ).clone(),
            # Which global env slots this snapshot batch belongs to — eval
            # harnesses need it to attribute episodes to a fixed slot (paired
            # evaluation), since a reset batch is only the envs that just ended.
            "env_ids": env_ids.clone(),
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
            snap["final_pos_xy"], snap["final_vel_xy"], snap["final_vel_z"], snap["final_alt"],
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
        # Expose the per-episode terminal arrays to eval harnesses (play.py
        # accumulates distributions from this; the wandb log below only
        # carries batch means).
        self._last_final_snapshot = snap
        f = snap["done_flags"]
        log = self.extras.setdefault("log", {})
        for cause in ("success", "crash", "overspeed", "bad_attitude", "out_of_range", "max_altitude", "overshoot", "stagnation", "timeout"):
            log[f"Episode_Termination/{cause}"] = f[cause].float().mean().item()
        log["Episode_Diag/overshoot_flythrough"] = snap["overshoot_flythrough"].float().mean().item()
        log["Episode_Termination/timeout_near_miss"] = (
            f["timeout"] & (snap["final_d_xy"] < 2.0)
        ).float().mean().item()

        log["Episode_Metric/action_sat_frac"] = snap["action_sat_frac"].mean().item()

        delivered = snap["delivered"]
        if bool(delivered.any()):
            log["Episode_Metric/deliver_time_s"] = snap["deliver_time_s"][delivered].mean().item()
        log["Episode_Metric/feasible_window_s"] = snap["feasible_window_s"].mean().item()

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
            # Phase 1: scripted CCIP referee (v15 drop_calculator parity —
            # see _evaluate_scripted_release_metric). drop_impact_error_m is
            # the miss at the RELEASE event; the legacy terminal-instant
            # blind-drop prediction moves to drop_impact_error_terminal_m
            # (it reads ~v*(t_fall+delay) of approach-velocity carry at the
            # d_xy-success snapshot, NOT drop accuracy — exp_014's 4.59 m).
            released = snap["released"]
            log["Episode_Metric/release_rate"] = released.float().mean().item()
            if bool(released.any()):
                log["Episode_Metric/drop_impact_error_m"] = (
                    snap["release_impact_err"][released].mean().item()
                )
            impact = self._predicted_impact_from_snapshot(snap)
            impact_error = torch.linalg.norm(impact - snap["target_xy"], dim=-1)
            log["Episode_Metric/drop_impact_error_terminal_m"] = impact_error.mean().item()

        if self.cfg.target_kf_obs:
            # Tracker quality at episode end: how far the Singer-KF estimate
            # was from the true target state (position/velocity), over the
            # envs that ever acquired a track this episode.
            kv = snap["kf_valid"]
            log["Episode_Metric/kf_track_rate"] = kv.float().mean().item()
            if bool(kv.any()):
                kf_pos_err = torch.linalg.norm(
                    snap["kf_est"][kv, 0:2] - snap["target_xy"][kv], dim=-1)
                kf_vel_err = torch.linalg.norm(
                    snap["kf_est"][kv, 2:4] - snap["target_vel_xy"][kv], dim=-1)
                log["Episode_Metric/kf_pos_err_m"] = kf_pos_err.mean().item()
                log["Episode_Metric/kf_vel_err_mps"] = kf_vel_err.mean().item()

        aim_min = snap["aim_err_min"]
        finite_aim = torch.isfinite(aim_min)
        log["Episode_Metric/aim_err_min_m"] = (
            aim_min[finite_aim].mean().item() if bool(finite_aim.any()) else 0.0
        )
        log["Episode_Metric/final_speed_xy"] = (
            torch.linalg.norm(snap["final_vel_xy"], dim=-1).mean().item()
        )

        d_xy_min = snap["d_xy_min"]
        log["Episode_Metric/d_xy_min"] = d_xy_min[torch.isfinite(d_xy_min)].mean().item() if torch.isfinite(d_xy_min).any() else 0.0
        for k, v in snap["episode_sums"].items():
            log[f"Episode_Reward/{k}"] = v.mean().item()
