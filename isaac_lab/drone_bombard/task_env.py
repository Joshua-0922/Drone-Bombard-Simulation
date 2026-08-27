"""The payload-delivery task, as ONE environment.

This replaces the ``v11_env.py`` chain (ten cfg classes, seven env classes,
``V11Cfg -> V12 -> ... -> V20``) that grew one subclass per experiment between
2026-06 and 2026-08. That chain served its purpose -- each link isolated one
change so a checkpoint could warm-start across it -- but it left three problems
that outlived the experiments:

1. **No field had one value.** ``v14_wind_std`` was defined at 1.0 in v14,
   overridden to 2.0 in v15, to 1.5 in v18. Answering "what is the wind right
   now" meant walking the inheritance backwards.
2. **Domain randomization was scattered across three switches** in two files,
   and one of them (``dyn_dr``) covered both model-error and sensor-noise axes,
   so the paper's DR_SCALE sweep could not change one without the other.
3. **Two scoring paths coexisted** -- an analytic ``ballistic_impact`` referee
   from the pre-payload era, and the real PhysX landing that superseded it. The
   dead one kept feeding a phantom observation channel.

``research_architecture.md`` §0 discarded every checkpoint from that lineage, so
the structure that produced them had no reason to survive either. What is kept:
the physics (kinematic weld payload, spawn-time mass authoring, equivalence-
transform DR), the release envelope, the reward shape, and every rule learned
along the way. What is gone: version prefixes, inheritance, and the analytic
referee.

Division of labour with ``drone_bombard_env.py``:

* the base env owns the **plant** -- rigid bodies, controller, payload physics,
  release latency, ALL domain randomization sampling, and the single nominal
  CCIP entry point ``_nominal_impact``.
* this file owns the **task** -- the cruise-handoff scenario, perception gating,
  the release decision, the reward, the observation vector, and termination.

This env never samples a random physical parameter; it only reads them.
"""

from __future__ import annotations

import math

import torch

from isaaclab.utils import configclass
from isaaclab.utils.math import euler_xyz_from_quat, quat_from_euler_xyz

from .drone_bombard_env import DroneBombardDynDRCfg, DroneBombardEnv, DroneBombardEnvCfg
from .math_utils import apply_ccip_residual, rate_limit_action, release_gate, time_to_fall
from .mdp.domain_rand import sample_uniform


# =====================================================================
# Config
# =====================================================================


@configclass
class HandoffCfg:
    """B-GROUP randomization — scenario diversity. The state the drone is handed
    at the start of the episode.

    The scenario is a HANDOFF, not a takeoff: the drone arrives already cruising,
    the marker is somewhere ahead, and the drone must spot it while moving and
    turn onto it. Starting from rest would make the first seconds a climb-out
    that has nothing to do with what the paper measures, and would let a policy
    reach the marker with near-zero speed for free -- which is exactly the
    hover-drop degenerate strategy the delivery-time axis exists to expose.

    Not scaled by DR_SCALE: this group is held fixed across the sweep so the only
    thing changing is the model error (see ``DroneBombardModelErrorCfg``).
    """

    randomize: bool = True
    """False pins every axis to its nominal (``speed_nom``/``alt_nom``/heading 0)
    and draws no RNG, reproducing the fixed handoff the v11-v19 lineage used."""

    speed_range: tuple[float, float] = (2.0, 6.0)
    speed_nom: float = 4.0
    alt_range: tuple[float, float] = (8.0, 12.0)
    alt_nom: float = 10.0

    heading_range_deg: tuple[float, float] = (-30.0, 30.0)
    """World-frame cruise heading.

    ⚠️ CAPPED AT ±30° ON PURPOSE. The observation is still world-frame (target
    offset, velocity, CCIP error are all in world ENU), so a heading drawn from
    the full circle is not a robustness burden -- it is a DIFFERENT TASK in every
    rotation. exp_024 measured the consequence: a policy at 100% on the fixed
    heading fell to 8.5% when the heading was opened to ±180°, and never
    recovered. Widening this is only safe together with a heading-invariant
    observation frame (architecture doc §5.1). Until then, ±30° gives the policy
    a real spread without asking it to solve the task in every rotation.
    """

    lateral_offset_max: float = 3.0   # m, spawn offset perpendicular to the cruise axis
    along_offset_max: float = 2.0     # m, spawn offset along the cruise axis
    vel_noise_std: float = 0.3        # m/s on all three velocity axes
    attitude_std_deg: float = 5.0     # roll/pitch at handoff
    ang_vel_std: float = 0.2          # rad/s body rates at handoff

    marker_dist_range: tuple[float, float] = (18.0, 22.0)
    """Along-track distance from spawn to the marker. Randomizing this is what
    makes "target range" a generalization axis at all -- it was pinned at exactly
    20.0 m for the whole v11-v20 lineage, so any claim about extrapolating to
    unseen ranges was untestable. Evaluate outside this band for the unseen
    condition."""

    marker_disk_radius: float = 5.0
    """The marker is placed uniformly in a disk of this radius about the
    along-track point, so it is generally NOT on the cruise axis and the drone
    has to steer laterally after acquiring it."""


@configclass
class ReleaseEnvelopeCfg:
    """The flight conditions under which a drop command is honoured.

    This is a safety/feasibility envelope, not a reward: a drop requested outside
    it simply does not fire. It is what makes "release rate" and "feasible
    release window" meaningful metrics, and it applies identically to learned and
    scripted arms so Table 1 rows stay comparable.
    """

    radius: float = 1.5
    """Predicted-impact error (m) admitting a release. Wider than
    ``success_radius`` on purpose: with a randomly-initialized residual the
    prediction is corrupted early in training, and a gate as tight as the success
    criterion produced a deadlock where the drone never dropped, never landed,
    and therefore never got a learning signal at all (v18). Success is still
    judged at ``success_radius``."""

    alt_min: float = 3.0
    alt_max: float = 8.0
    max_speed: float = 5.0
    max_vz: float = 3.0
    max_tilt: float = 0.35      # rad (20°) — the real constraint on the release path
    max_ang_vel: float = 4.0    # rad/s

    decide_at_physics_rate: bool = True
    """Resolve the release INSTANT at 100 Hz instead of on the 10 Hz policy grid.

    The policy (or a scripted arm) still decides at its own rate whether to
    commit -- ``action[4]`` is a pickle-and-hold, constant across the decimation
    window. What moves to physics rate is the timing solver: on every sub-step
    the envelope is re-tested and the drop fires when the CCIP solution stops
    closing on the target (``d_impact`` crosses its minimum). This is what a real
    CCIP release does -- the pilot holds the pickle, the computer picks the
    instant.

    Why it matters here: exp_025 measured a 0.57 m CEP50 at DR_SCALE 0 with a
    true target and a 0.010 m ballistic model error, i.e. essentially all of it
    was control and release TIMING -- at 4 m/s a 10 Hz decision grid moves 0.4 m
    between the instants the drone is allowed to choose from. No residual can
    recover that, because it is not a modelling error.

    ⚠️ Turning this on changes what the scripted baselines mean. T1 (naive
    threshold) inherits the fine-grained timing it did not have, so it moves
    toward T2 (AeroThrow's horizon argmin, which is the same criterion sampled
    at 10 Hz). That is a result, not a defect -- but Table 1 must be re-measured,
    and False here is the ablation arm that reproduces the 10 Hz grid exactly.
    """


@configclass
class ResidualCfg:
    """The learned impact-space correction -- the paper's mechanism.

    The policy emits a 2-D vector on ``action[5:7]``; it is scaled by ``scale``
    and ADDED TO THE PREDICTED IMPACT POINT, not to the control command. That
    injection point is the contribution: the analytic model stays in the loop and
    the network only supplies what the model cannot know.
    """

    enabled: bool = True
    """False is the control arm ("does the residual contribute at all")."""

    scale: float = 2.0
    """Metres of authority per axis. Sized 2026-08-27 against the measured drift
    across the DR_SCALE sweep: at scale 1.5 the per-axis p90 drift is 0.72 m and
    only 0.1% of releases would saturate ±2 m. Raising this is not free -- a
    large authority lets an untrained residual corrupt the gate badly enough to
    stall the bootstrap (the v18 deadlock, which was fixed by lowering it from
    3.0)."""


@configclass
class PerceptionCfg:
    """How the drone comes to know where the marker is.

    The reveal gate stays on in every configuration: the drone cruises blind,
    acquires the marker within ``reveal_radius``, and only then can aim. That is
    the scenario. ``pixel_quantize`` controls only how NOISY the position is once
    acquired.
    """

    reveal_radius: float = 7.0
    """Horizontal distance at which the marker becomes visible. With the default
    handoff (marker ~20 m ahead) this gives roughly 13 m ≈ 3.3 s of blind cruise
    before acquisition."""

    pixel_quantize: bool = False
    """Quantize the acquired marker position to a camera cell, ``cell = k *
    slant_range``.

    OFF for the main experiments. This is a measurability decision, not a
    convenience: at release altitude the cell is ~0.65 m, so quantization alone
    contributes ~0.28 m of aiming error -- LARGER than the 0.17 m of ballistic
    model error the residual exists to remove. With it on, the effect under study
    is buried in perception noise. Turn it on for the vision-extension results
    and report them separately."""

    pixel_cell_k: float = 0.12


@configclass
class TaskRewardCfg:
    """Reward weights. Per-STEP terms are marked; they scale with the control
    rate, so changing ``decimation`` without rescaling these silently changes the
    objective (the loiter penalty is quadratic in episode length and would move
    by the square)."""

    w_progress: float = 3.0
    """Telescoping approach reward, ``w_progress * (d_prev - d)``. Totals
    ``w_progress * d_0`` over an episode regardless of route, so it cannot be
    farmed by oscillating.

    Raised from 1.0 alongside the 100x increase in ``w_time``: the approach guide
    has to stay visible against the per-step time cost during the bootstrap phase,
    when the policy cannot yet hit anything and progress is the only dense signal
    it can act on. Closing 20 m now pays 60 against ~80 of accumulated time cost."""
    w_ccip: float = 0.5            # aim quality (potential-based, see below)
    k_ccip: float = 1.0
    w_ang_vel: float = 0.05        # per step
    w_tilt: float = 0.05           # per step
    w_action_smooth: float = 0.05  # per step
    w_time: float = 1.0            # per step -> 10.0 per second at 10 Hz
    """⭐ Per-step cost of still carrying the payload. THE knob that sets where on
    the speed-accuracy tradeoff the policy is asked to operate.

    Calibrated 2026-08-27 against the measured scripted arms rather than guessed.
    Evaluating the terminal reward on their real landing distributions (n=128,
    DR_SCALE 1.0) showed the reward PREFERRED hover-drop -- exactly the
    degeneration the paper exists to argue against:

        arm        terminal   time   loiter     NET     err    t
        T0 hover      191.6   1.05    10.47   180.0   0.305  10.50 s
        T2 argmin     166.8   0.88     3.59   162.3   0.353   8.84 s

    T0 buys 0.048 m of accuracy (zero horizontal velocity removes the ballistic
    dispersion entirely) for 1.66 s, and at the old 0.01 the reward valued that
    second at 0.0002 m. Time was free, so the optimum was to stop and hover --
    which is precisely what AeroThrow's own trajectory relaxation converges to
    (their §V-A: "shifting from a throw toward a place").

    1.0/step = 10/s makes the reward approximately INDIFFERENT between the two
    operating points (T0 86.1 vs T2 84.8 net). That is deliberate: it neither
    pushes the policy to hover nor forces it to rush, so the operating point is
    LEARNED rather than imposed -- which is the honest way to then report where
    it landed. Sweeping this weight traces our own speed-accuracy Pareto front,
    the direct counterpart to the baseline front in the related work.

    ⚠️ Per-STEP, so it scales with the control rate. Changing ``decimation``
    without rescaling silently changes the objective."""
    gate_reward: float = 0.05      # per step while a release is admissible
    drop_signal_reward: float = 1.0
    undetected_penalty: float = -0.2   # per step while the marker is not acquired

    potential_shaping: bool = True
    """Reward the CHANGE in aim quality rather than its level.

    A standing ``w_ccip * exp(-k*d)`` pays a guaranteed reward every step for
    merely HOLDING aim, which made hovering-while-aimed out-earn the noisy
    business of actually dropping -- a policy collapsed to release rate 0 with
    perfect aim. Telescoping the term removes the standing income."""

    w_loiter: float = 0.02
    """Escalating penalty, charged as ``w_loiter * (consecutive steps spent
    inside the release envelope without dropping)``. The companion to
    ``potential_shaping``: once the gate is open the cost of not releasing grows
    every step, so the only way out is to release."""

    reward_success: float = 300.0
    success_bonus: float = 100.0
    k_landing: float = 2.0
    """Terminal landing reward is ``reward_success * exp(-k_landing * err)`` plus
    ``success_bonus`` for crossing the success radius. Continuous rather than
    flat-inside-the-circle: a flat reward paid a 0.1 m hit and a 0.9 m hit
    identically, so accuracy plateaued with no gradient to tighten."""

    success_radius: float = 0.5
    """Tightened from 1.0 m on 2026-08-27. At 1.0 m the binary success rate was
    saturated -- T2 and T3 differed by 0.14 m of CEP50 while both scored ~80% --
    so the headline metric could not see the effect under study. 0.5 m sits just
    under the measured CEP50 floor, which is where the rate is most sensitive.
    Must be identical for every arm in Table 1; ``eval_harness`` reads it from
    here for exactly that reason."""

    no_drop_penalty: float = -30.0
    failure_penalty: float = -200.0
    """ONE penalty for EVERY failure termination -- crash, overspeed, bad
    attitude, out of range, max altitude.

    Deliberately a single knob instead of one per cause. The 2026-08-27 dry-run
    found the per-cause version had a hole: ``bad_attitude``, ``overspeed`` and
    ``max_altitude`` all TERMINATED the episode, but only ``crash``,
    ``out_of_range`` and ``timeout`` were charged. With ``w_time`` at 1.0/step
    that made tumbling a FREE EXIT worth 1.0 for every step avoided, and the
    policy took it -- ``bad_attitude`` reached 100% of terminations with
    ``rew_failure`` sitting at exactly 0.0 while the return fell monotonically.
    A single mask over the union cannot grow that hole again.

    Magnitude (-200, up from -50/-30) follows from ``w_time``, not from
    independent tuning: **ending the episode by failing must never beat
    completing it badly.** At 10/s a crash at t=1 s costs only 10 s of
    accumulated time, while the worst SUCCESSFUL delivery (terminal ~0, released
    at ~8 s) nets about -80, so the old -50 made an early crash (-60) *better*
    than delivering badly. -200 puts the worst failure below the worst success at
    every release time in the envelope."""


@configclass
class DroneBombardTaskCfg(DroneBombardEnvCfg):
    """The delivery task. One config -- no version chain, no inherited overrides.

    Domain randomization is NOT configured here. The A group (model error, the
    only group DR_SCALE scales) and the C group (sensor/actuator) both live on
    the base cfg as ``model_err`` and ``dyn_dr``; the B group is ``handoff``
    below. Keeping the physical randomization on the base class is what
    guarantees there is exactly one wind sampler in the codebase.
    """

    handoff: HandoffCfg = HandoffCfg()
    release: ReleaseEnvelopeCfg = ReleaseEnvelopeCfg()
    residual: ResidualCfg = ResidualCfg()
    perception: PerceptionCfg = PerceptionCfg()
    task_reward: TaskRewardCfg = TaskRewardCfg()

    dyn_dr: DroneBombardDynDRCfg = DroneBombardDynDRCfg(enabled=True)
    """C-GROUP randomization, ON by default.

    Explicitly re-declared here rather than inherited: the base default is
    ``enabled=False`` so the pre-P0 lineage stayed bit-identical, and silently
    inheriting that would have trained this task on a perfect plant with perfect
    sensors while the config still advertised a C group. (It did, until an
    in-sim orthogonality check caught the gain spread sitting at exactly 0.)

    Held FIXED across the DR_SCALE sweep -- ``model_err.scale`` must not touch
    it, which is the property that makes the sweep attributable."""

    payload_physics_enabled: bool = True
    """The payload is a real rigid body that falls, is blown by the wind, and
    lands. The episode is scored on where it ACTUALLY landed. There is no
    analytic scoring path in this env -- ``ballistic_impact`` is a predictor
    here and nothing else."""

    wind_force_enabled: bool = True
    """The wind also pushes the airframe, quadratically in relative airflow.
    This is what makes wind partially INFERABLE: the drone can read it off its
    own tracking error during the approach. Without it, an unobserved wind would
    be pure unlearnable noise and the ``observe_wind`` ablation would be
    meaningless."""

    action_space: int = 7   # [0:4] velocity + yaw rate, [4] drop signal, [5:7] impact residual
    episode_length_s: float = 30.0

    def __post_init__(self):
        super().__post_init__()
        # Observation width depends on whether the wind is exposed, so it has to
        # be derived rather than declared. The base env allocates the per-episode
        # observation-bias buffer from this number and hard-raises on a mismatch,
        # which is the guardrail that catches a hand-edited width.
        self.observation_space = 25 + (2 if self.model_err.observe_wind else 0)


# =====================================================================
# Env
# =====================================================================


class DroneBombardTaskEnv(DroneBombardEnv):
    """Cruise in, acquire the marker, aim with CCIP + learned residual, release
    inside the envelope, and be scored on where the payload really lands."""

    cfg: DroneBombardTaskCfg

    def __init__(self, cfg: DroneBombardTaskCfg, render_mode: str | None = None, **kwargs):
        super().__init__(cfg, render_mode, **kwargs)
        N, device = self.num_envs, self.device
        self._wants_drop = torch.zeros(N, dtype=torch.bool, device=device)
        self._residual_action = torch.zeros(N, 2, device=device)
        self._d_impact = torch.zeros(N, device=device)
        self._d_impact_prev = torch.zeros(N, device=device)
        self._gate_open = torch.zeros(N, dtype=torch.bool, device=device)
        self._gate_steps = torch.zeros(N, device=device)
        # Physics-rate release: the previous SUB-STEP's aim error (for the
        # crossing test) and the fire mask accumulated across the window.
        self._d_impact_sub_prev = torch.full((N,), float("inf"), device=device)
        self._fired_in_window = torch.zeros(N, dtype=torch.bool, device=device)
        self._perceived_target_xy = torch.zeros(N, 2, device=device)
        self._detected = torch.zeros(N, dtype=torch.bool, device=device)
        self._cruise_unit = torch.zeros(N, 2, device=device)
        self._cruise_unit[:, 0] = 1.0

        # Per-component reward accounting. The base env's ``_episode_sums`` keys
        # belong to its own Phase-1 reward and stay identically zero here, which
        # means a multi-day run would log nothing about WHY the return moved.
        # Registering this task's terms into the same dict reuses the base env's
        # reset and snapshot plumbing (``_reset_idx`` zeroes every key,
        # ``_snapshot`` emits them as ``Episode_Reward/*``).
        for k in ("rew_progress", "rew_aim_pot", "rew_gate", "rew_drop_signal",
                  "rew_undetected", "rew_time", "rew_loiter", "rew_smooth",
                  "rew_attitude", "rew_landing", "rew_failure"):
            self._episode_sums[k] = torch.zeros(N, device=device)

    # ------------------------------------------------------------------
    # helpers
    # ------------------------------------------------------------------
    def _kinematics(self):
        pos = self._robot.data.root_pos_w - self.scene.env_origins
        vel = self._robot.data.root_lin_vel_w
        ang = self._robot.data.root_ang_vel_b
        roll, pitch, yaw = euler_xyz_from_quat(self._robot.data.root_quat_w)
        wrap = lambda a: torch.atan2(torch.sin(a), torch.cos(a))  # noqa: E731
        return pos, vel, ang, wrap(roll), wrap(pitch), wrap(yaw)

    def _perceive(self, pos_xy: torch.Tensor, altitude: torch.Tensor) -> torch.Tensor:
        """Where the drone THINKS the marker is. Optionally quantized to a camera
        cell whose size grows with slant range -- a pixel subtends more ground
        the further away and the higher you are."""
        if not self.cfg.perception.pixel_quantize:
            return self._target_xy
        rel = self._target_xy - pos_xy
        slant = torch.sqrt((rel * rel).sum(dim=-1) + altitude * altitude)
        cell = (self.cfg.perception.pixel_cell_k * slant).clamp(min=0.05).unsqueeze(-1)
        return pos_xy + (torch.floor(rel / cell) + 0.5) * cell

    def _ccip(self, pos: torch.Tensor, vel: torch.Tensor):
        """Aiming solution: nominal ballistic prediction, corrected by the learned
        residual, measured against the PERCEIVED marker.

        Both approximations are deliberate and both are what the policy actually
        has: it cannot see the true marker, and it cannot compute the true impact
        point. Scoring, by contrast, uses the true marker and the real landing --
        that asymmetry is the whole experiment.
        """
        pos_xy, altitude, vel_xy, vel_z = pos[:, :2], pos[:, 2], vel[:, :2], vel[:, 2]
        perceived = self._perceive(pos_xy, altitude)
        self._perceived_target_xy = perceived

        impact = self._nominal_impact(pos_xy, vel_xy, vel_z, altitude)
        if self.cfg.residual.enabled:
            impact = apply_ccip_residual(impact, self._residual_action, self.cfg.residual.scale)

        ccip_err = impact - perceived
        return ccip_err, torch.linalg.norm(ccip_err, dim=-1), time_to_fall(
            altitude, vel_z, self.cfg.drop.gravity)

    def _current_d_xy(self) -> torch.Tensor:
        pos_xy = self._robot.data.root_pos_w[:, :2] - self.scene.env_origins[:, :2]
        return torch.linalg.norm(self._target_xy - pos_xy, dim=-1)

    # ------------------------------------------------------------------
    # Action
    # ------------------------------------------------------------------
    def _pre_physics_step(self, actions: torch.Tensor):
        """Split the 7-D action. Not delegated to the base env, whose split is
        for its own 6-D layout ([0:4] velocity, [4:6] residual) and would read
        the drop signal as half of the residual.

        Actuation noise is applied to the velocity dims ONLY. Jittering the drop
        decision or the impact residual would blur the two mechanisms the paper
        is trying to measure into the sensor-noise axis.
        """
        actions = self._perturb_actions(actions)
        self._fired_in_window[:] = False
        self._action_sat_sum += (actions.abs() > 1.0).float().mean(dim=-1)
        clipped = torch.clamp(actions, -1.0, 1.0)

        prev_vel = self._prev_action[:, :4]
        limited_vel = rate_limit_action(clipped[:, :4], prev_vel, self.cfg.action.rate_limit)
        # Smoothness is scored on the velocity command only: the residual is an
        # instantaneous aim correction, not something the airframe has to slew.
        self._delta_action = limited_vel - prev_vel

        self._wants_drop = clipped[:, 4] > 0.5
        self._residual_action = (clipped[:, 5:7] if self.cfg.residual.enabled
                                 else torch.zeros_like(clipped[:, 5:7]))
        self._prev_action = torch.cat([limited_vel, self._residual_action], dim=-1)

        a = self.cfg.action
        self._vel_cmd = torch.stack([
            limited_vel[:, 0] * a.vx_scale,
            limited_vel[:, 1] * a.vy_scale,
            limited_vel[:, 2] * a.vz_scale,
            limited_vel[:, 3] * a.yaw_scale,
        ], dim=-1)

    # ------------------------------------------------------------------
    # Release timing, resolved at physics rate
    # ------------------------------------------------------------------
    def _apply_action(self):
        # Before the base env's controller + payload sub-step, so a drop armed
        # here starts its release-latency countdown on THIS sub-step rather than
        # the next one (the countdown lives in ``_step_payload_physics``).
        if self.cfg.release.decide_at_physics_rate:
            self._resolve_release()
        super()._apply_action()

    def _resolve_release(self):
        """Fire on the sub-step where the aiming solution crosses the target.

        ``_wants_drop`` is held constant across the decimation window, so this is
        pickle-and-hold: the commit is the policy's, the instant is the solver's.
        The crossing test (``d_impact`` no longer decreasing) handles both
        orderings without a special case -- pickled early, it waits for the
        crossing; pickled late, the aim is already worsening and it fires at
        once.

        If the crossing falls outside the envelope (typically below
        ``alt_min``), nothing fires and the episode takes ``no_drop_penalty``.
        That is the intended signal: the pass was set up wrong, and repairing it
        is the policy's job, not the release logic's.
        """
        rc = self.cfg.release
        pos, vel, ang, roll, pitch, _ = self._kinematics()
        _, d_impact, _ = self._ccip(pos, vel)

        gate = release_gate(
            d_impact, pos[:, 2], torch.linalg.norm(vel[:, :2], dim=-1), vel[:, 2],
            roll, pitch, torch.linalg.norm(ang, dim=-1), self._payload_attached.float(),
            rc.radius, rc.alt_min, rc.alt_max,
            rc.max_speed, rc.max_vz, rc.max_tilt, rc.max_ang_vel,
        )
        # Seeded to +inf at reset, so the first sub-step of an episode can never
        # read as a crossing.
        crossed = d_impact >= self._d_impact_sub_prev
        fire = self._wants_drop & gate & crossed & (~self._released)

        self.request_release(fire)
        self._fired_in_window = self._fired_in_window | fire
        self._released = self._released | fire
        self._d_impact_sub_prev = d_impact

    # ------------------------------------------------------------------
    # Observation
    # ------------------------------------------------------------------
    def _get_observations(self) -> dict:
        self._update_markers()
        cfg = self.cfg
        pos, vel, ang, roll, pitch, yaw = self._kinematics()
        altitude = pos[:, 2]
        ccip_err, d_impact, t_f = self._ccip(pos, vel)
        self._d_impact = d_impact

        det = self._current_d_xy() <= cfg.perception.reveal_radius
        self._detected = det
        m = det.float().unsqueeze(-1)

        rel = self._perceived_target_xy - pos[:, :2]
        c = lambda x, s: torch.clamp(x / s, -1.0, 1.0)  # noqa: E731

        # Marker-dependent channels are zeroed until acquisition so a blind
        # policy sees an explicit "no target" rather than a stale or leaked one.
        marker_dep = torch.stack([
            c(rel[:, 0], 20.0), c(rel[:, 1], 20.0),
            c(ccip_err[:, 0], 10.0), c(ccip_err[:, 1], 10.0),
            torch.clamp(d_impact / 10.0, 0.0, 1.0),
        ], dim=-1) * m

        own = torch.stack([
            c(0.0 - altitude, 20.0),
            c(vel[:, 0], 10.0), c(vel[:, 1], 10.0), c(vel[:, 2], 10.0),
            torch.clamp(roll / math.pi, -1.0, 1.0), torch.clamp(pitch / math.pi, -1.0, 1.0),
            torch.sin(yaw), torch.cos(yaw),
            c(ang[:, 0], math.pi), c(ang[:, 1], math.pi), c(ang[:, 2], math.pi),
            torch.clamp(t_f / 5.0, 0.0, 1.0),
            torch.clamp(torch.linalg.norm(vel[:, :2], dim=-1) / 10.0, 0.0, 1.0),
            torch.clamp(altitude / 20.0, 0.0, 1.0),
            self._payload_attached.float(),
        ], dim=-1)

        parts = [marker_dep, own, self._prev_action[:, :4], det.float().unsqueeze(-1)]

        if cfg.model_err.observe_wind:
            # Ablation arm only. Masked by detection like the other privileged-ish
            # channels, so it cannot be used as a covert "am I close yet" signal.
            parts.insert(2, c(self._wind_xy, cfg.obs.wind_scale) * m)

        obs = torch.nan_to_num(torch.cat(parts, dim=-1), nan=0.0)
        return {"policy": self._perturb_obs(obs)}

    # ------------------------------------------------------------------
    # Reset: cruise handoff
    # ------------------------------------------------------------------
    def _reset_idx(self, env_ids: torch.Tensor):
        super()._reset_idx(env_ids)  # buffers, DR sampling, spawn mass, episode logging
        if env_ids is None or len(env_ids) == self.num_envs:
            env_ids = self._robot._ALL_INDICES
        n, device = len(env_ids), self.device
        hc = self.cfg.handoff

        if hc.randomize:
            cruise_yaw = sample_uniform(math.radians(hc.heading_range_deg[0]),
                                        math.radians(hc.heading_range_deg[1]), n, device)
            cruise_speed = sample_uniform(*hc.speed_range, n, device)
            spawn_alt = sample_uniform(*hc.alt_range, n, device)
            marker_dist = sample_uniform(*hc.marker_dist_range, n, device)
        else:
            cruise_yaw = torch.zeros(n, device=device)
            cruise_speed = torch.full((n,), hc.speed_nom, device=device)
            spawn_alt = torch.full((n,), hc.alt_nom, device=device)
            marker_dist = torch.full((n,), sum(hc.marker_dist_range) / 2.0, device=device)

        cruise_unit = torch.stack([torch.cos(cruise_yaw), torch.sin(cruise_yaw)], dim=-1)
        cruise_perp = torch.stack([-cruise_unit[:, 1], cruise_unit[:, 0]], dim=-1)
        self._cruise_unit[env_ids] = cruise_unit

        # Marker: along-track distance ahead, then an area-uniform disk offset so
        # it is generally off the cruise axis and must be steered to.
        r = hc.marker_disk_radius * torch.sqrt(torch.rand(n, device=device))
        th = torch.rand(n, device=device) * (2.0 * math.pi)
        marker = (cruise_unit * marker_dist.unsqueeze(-1)
                  + torch.stack([r * torch.cos(th), r * torch.sin(th)], dim=-1))
        self._target_xy[env_ids] = marker

        # Spawn offset expressed in the CRUISE frame, so lateral/along keep their
        # meaning at any heading.
        spawn_xy = torch.zeros(n, 2, device=device)
        if hc.randomize and (hc.lateral_offset_max > 0.0 or hc.along_offset_max > 0.0):
            lat = (torch.rand(n, device=device) * 2.0 - 1.0) * hc.lateral_offset_max
            along = (torch.rand(n, device=device) * 2.0 - 1.0) * hc.along_offset_max
            spawn_xy = cruise_perp * lat.unsqueeze(-1) + cruise_unit * along.unsqueeze(-1)

        root = torch.zeros(n, 13, device=device)
        root[:, :2] = spawn_xy + self.scene.env_origins[env_ids, :2]
        root[:, 2] = spawn_alt + self.scene.env_origins[env_ids, 2]
        if hc.randomize and hc.attitude_std_deg > 0.0:
            att = math.radians(hc.attitude_std_deg)
            roll0 = torch.randn(n, device=device) * att
            pitch0 = torch.randn(n, device=device) * att
        else:
            roll0 = torch.zeros(n, device=device)
            pitch0 = torch.zeros(n, device=device)
        root[:, 3:7] = quat_from_euler_xyz(roll0, pitch0, cruise_yaw)
        root[:, 7:9] = cruise_unit * cruise_speed.unsqueeze(-1)
        if hc.randomize and hc.vel_noise_std > 0.0:
            root[:, 7:10] = root[:, 7:10] + torch.randn(n, 3, device=device) * hc.vel_noise_std
        if hc.randomize and hc.ang_vel_std > 0.0:
            root[:, 10:13] = torch.randn(n, 3, device=device) * hc.ang_vel_std
        self._robot.write_root_pose_to_sim(root[:, :7], env_ids)
        self._robot.write_root_velocity_to_sim(root[:, 7:13], env_ids)

        d0 = torch.linalg.norm(marker - spawn_xy, dim=-1)
        self._d_xy_prev[env_ids] = d0
        self._d_impact_prev[env_ids] = d0
        self._wants_drop[env_ids] = False
        self._residual_action[env_ids] = 0.0
        self._gate_steps[env_ids] = 0.0
        self._d_impact_sub_prev[env_ids] = float("inf")
        self._fired_in_window[env_ids] = False

        # Seed the controller AT the cruise setpoint. Without this the step-1
        # command is whatever the untrained policy emits, producing a
        # cruise-speed velocity error and a corrective tilt violent enough to
        # trip the attitude termination on the first step of every episode.
        a = self.cfg.action
        cruise_v = cruise_unit * cruise_speed.unsqueeze(-1)
        self._prev_action[env_ids, 0] = torch.clamp(cruise_v[:, 0] / a.vx_scale, -1.0, 1.0)
        self._prev_action[env_ids, 1] = torch.clamp(cruise_v[:, 1] / a.vy_scale, -1.0, 1.0)
        self._prev_action[env_ids, 2:] = 0.0
        self._v_filt[env_ids, 0] = cruise_v[:, 0]
        self._v_filt[env_ids, 1] = cruise_v[:, 1]
        self._v_filt[env_ids, 2:] = 0.0

    # ------------------------------------------------------------------
    # Termination: the episode ends when the payload lands
    # ------------------------------------------------------------------
    def _get_dones(self) -> tuple[torch.Tensor, torch.Tensor]:
        cfg = self.cfg
        tc, rc = cfg.termination, cfg.release
        pos, vel, ang, roll, pitch, _ = self._kinematics()
        altitude = pos[:, 2]
        speed = torch.linalg.norm(vel, dim=-1)
        ang_norm = torch.linalg.norm(ang, dim=-1)

        _, d_impact, _ = self._ccip(pos, vel)
        self._d_impact = d_impact
        self._aim_err_min = torch.minimum(self._aim_err_min, d_impact)

        gate = release_gate(
            d_impact, altitude, torch.linalg.norm(vel[:, :2], dim=-1), vel[:, 2],
            roll, pitch, ang_norm, self._payload_attached.float(),
            rc.radius, rc.alt_min, rc.alt_max,
            rc.max_speed, rc.max_vz, rc.max_tilt, rc.max_ang_vel,
        )
        self._gate_open = gate
        self._window_steps += (gate & (~self._released)).float()

        # The payload does NOT detach at the command. The plant holds it for
        # this episode's release latency (base env ``request_release`` /
        # ``_step_release_latency``). ``_released`` means "the command has been
        # issued"; ``_payload_free`` means "it is actually falling".
        if rc.decide_at_physics_rate:
            # Already armed inside the window by ``_resolve_release``; just
            # collect what fired so the reward and the flags see it.
            fire = self._fired_in_window
        else:
            fire = self._wants_drop & gate & (~self._released)
            self.request_release(fire)
            self._released = self._released | fire
        self._just_released = fire.clone()

        d_xy = self._current_d_xy()
        self._d_xy_min = torch.minimum(self._d_xy_min, d_xy)
        self._detected = d_xy <= cfg.perception.reveal_radius
        self._overshoot_flythrough = torch.zeros_like(gate)

        # Scored on the REAL landing point against the TRUE marker.
        landed = self._payload_landed
        impact_err = torch.linalg.norm(self._payload_impact_xy - self._target_xy, dim=-1)
        self._release_impact_err = torch.where(landed, impact_err, self._release_impact_err)

        step = self.episode_length_buf
        # Failure modes are suppressed once the payload is away: the drone is
        # allowed to descend or drift while the episode waits for the landing,
        # and penalizing that would be scoring flight it no longer needs to do.
        crash = (altitude < tc.ground_contact_altitude) | (
            (step > tc.min_altitude_start_step) & (altitude < tc.min_altitude))
        inverted = (roll.abs() > tc.limit_inverted_tilt) | (pitch.abs() > tc.limit_inverted_tilt)
        bad_attitude = (ang_norm > tc.limit_ang_vel) | inverted
        overspeed = speed > tc.v_max_safety
        out_of_range = d_xy > tc.max_distance
        max_alt = altitude > tc.max_altitude
        alive = ~self._released
        failure = (crash | overspeed | bad_attitude | out_of_range | max_alt) & alive

        success = landed & (impact_err <= cfg.task_reward.success_radius)
        terminated = landed | failure
        time_out = step >= (self.max_episode_length - 1)

        z = torch.zeros_like(gate)
        self._done_flags = {
            "success": success,
            "crash": crash & alive, "overspeed": overspeed & alive,
            "bad_attitude": bad_attitude & alive, "out_of_range": out_of_range & alive,
            "max_altitude": max_alt & alive, "overshoot": z, "stagnation": z,
            "released": self._released.clone(), "just_released": self._just_released.clone(),
            "landed": landed,
            "release_miss": landed & ~success,
            "timeout": time_out & ~terminated,
            "gate_open": gate, "wants_drop": self._wants_drop,
        }
        return terminated, time_out & ~terminated

    # ------------------------------------------------------------------
    # Reward
    # ------------------------------------------------------------------
    def _get_rewards(self) -> torch.Tensor:
        cfg = self.cfg
        rw = cfg.task_reward
        _, _, ang, roll, pitch, _ = self._kinematics()
        f = self._done_flags
        d_impact = self._d_impact
        d_xy = self._current_d_xy()
        det = (d_xy <= cfg.perception.reveal_radius).float()
        # Shaping stops at release: the remaining flight is the payload's, and
        # paying the drone for how it flies during the fall would reward things
        # that cannot affect the outcome.
        flying = (~self._released).float()

        acc = self._episode_sums
        attitude = -(rw.w_ang_vel * (ang * ang).sum(dim=-1)
                     + rw.w_tilt * (roll * roll + pitch * pitch)) * flying
        smooth = -rw.w_action_smooth * (self._delta_action * self._delta_action).sum(dim=-1) * flying
        time_cost = -rw.w_time * flying
        undetected = (1.0 - det) * rw.undetected_penalty * flying
        acc["rew_attitude"] += attitude
        acc["rew_smooth"] += smooth
        acc["rew_time"] += time_cost
        acc["rew_undetected"] += undetected
        r = attitude + smooth + time_cost + undetected

        progress = rw.w_progress * (self._d_xy_prev - d_xy)
        if rw.potential_shaping:
            aim = rw.w_ccip * (torch.exp(-rw.k_ccip * d_impact)
                               - torch.exp(-rw.k_ccip * self._d_impact_prev))
        else:
            aim = rw.w_ccip * torch.exp(-rw.k_ccip * d_impact)
        # Progress is NOT gated on detection. It was, and the dry-run showed the
        # consequence: ``rew_progress`` sat at exactly 0.0 for the whole run,
        # because the drone has to approach in order to detect and the approach
        # signal only switched on after detecting. Chicken and egg, leaving the
        # policy nothing but per-step costs to act on.
        #
        # Using the true range in the REWARD while the marker is still unobserved
        # is legitimate -- shaping may use privileged quantities, only the
        # OBSERVATION has to stay honest, and the observation keeps its detection
        # mask. What the policy learns from this is "cruise forward", which is
        # exactly the intended blind-cruise behaviour.
        prog_t = flying * progress
        aim_t = det * flying * aim
        gate_t = det * flying * f["gate_open"].float() * rw.gate_reward
        acc["rew_progress"] += prog_t
        acc["rew_aim_pot"] += aim_t
        acc["rew_gate"] += gate_t
        r = r + prog_t + aim_t + gate_t
        # Paid on the release EVENT. Under pickle-and-hold the old
        # ``wants_drop & gate_open`` form would pay every step the button is
        # held inside the envelope -- a standing income of exactly the kind
        # ``potential_shaping`` exists to remove, and one a policy can farm by
        # holding the pickle where the crossing never comes.
        drop_t = f["just_released"].float() * rw.drop_signal_reward
        acc["rew_drop_signal"] += drop_t
        r = r + drop_t

        in_envelope = f["gate_open"] & (~self._released)
        self._gate_steps = torch.where(in_envelope, self._gate_steps + 1.0,
                                       torch.zeros_like(self._gate_steps))
        loiter_t = -flying * rw.w_loiter * self._gate_steps
        acc["rew_loiter"] += loiter_t
        r = r + loiter_t

        err = self._release_impact_err
        landing = (rw.reward_success * torch.exp(-rw.k_landing * err)
                   + (err <= rw.success_radius).float() * rw.success_bonus)
        landing_t = f["landed"].float() * landing
        # Union, not a per-cause sum: every way of ending an episode without
        # delivering costs the same, so no termination path can go uncharged.
        any_failure = (f["crash"] | f["overspeed"] | f["bad_attitude"]
                       | f["out_of_range"] | f["max_altitude"])
        failure_t = (any_failure.float() * rw.failure_penalty
                     + (f["timeout"] & ~self._released).float() * rw.no_drop_penalty)
        acc["rew_landing"] += landing_t
        acc["rew_failure"] += failure_t
        r = r + landing_t + failure_t

        self._d_xy_prev = d_xy
        self._d_impact_prev = d_impact
        return torch.nan_to_num(r, nan=0.0)
