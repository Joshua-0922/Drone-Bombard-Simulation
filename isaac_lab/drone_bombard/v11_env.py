"""Isaac-v11 (relaxed test) environment — doc 53 spec, minimal scenario.

A single, integrated (NO curriculum) PPO task that layers the doc-53 "Isaac v11"
model onto the existing ``DroneBombardEnv`` by subclassing it: the scene,
cascaded velocity controller, physics, and spawn-time mass authoring are
INHERITED unchanged. Only the pieces v11-test redefines are overridden:

  * ``_pre_physics_step`` — action[4] = policy drop_signal (doc 53 §4), not the
    Phase-2 CCIP residual; action[5] reserved/ignored.
  * ``_get_observations`` — 24-D obs (doc 53 §3): marker-relative pos, ENU vel,
    roll/pitch, yaw sin/cos, ang_vel, CCIP error vector, d_impact, t_f, speed,
    altitude, payload_attached, prev action. NO vision channels.
  * ``_reset_idx`` — the "no random marker spawn" modification: instead of a
    random target + random handoff, the marker is FIXED a set distance ahead
    along a fixed cruise direction and the drone spawns at the origin already
    cruising toward it at ``cruise_speed`` (a fixed handoff geometry).
  * ``_get_dones`` / ``_get_rewards`` — policy drop_signal + release-envelope
    gate (doc 53 §6), payload-landing-outcome terminal (doc 53 §7-8). A drop
    intent with the gate closed is a NO-OP with NO penalty (Gazebo v7 lesson).

DELIBERATELY suppressed-but-kept (expand later by flipping flags — nothing is
deleted): domain randomization (``dr_enabled`` False -> drag/wind samplers
return zero), CCIP residual (math + cfg present, action slot [5] reserved),
moving target (``moving_target_enabled`` False), vision (``_update_vision``
inherited but not read into obs), phase curriculum (single fixed config).
Wind / domain randomization is intentionally deferred to a later expansion.
"""

from __future__ import annotations

import math

import torch

from isaaclab.utils import configclass
from isaaclab.utils.math import euler_xyz_from_quat, quat_from_euler_xyz

from .drone_bombard_env import DroneBombardDynDRCfg, DroneBombardEnv, DroneBombardEnvCfg
from .mdp.domain_rand import sample_uniform
from .math_utils import (
    predict_impact_nominal,
    time_to_fall,
    ballistic_impact,
    release_gate,
    apply_ccip_residual,
    step_target_motion,
)


@configclass
class DroneBombardHandoffCfg:
    """P0 handoff randomization for the v-track (notes/research/paper_research_plan.md §5 P0-3).

    v11-v19 hand the policy an IDENTICAL initial condition every episode: spawn
    at the env origin, exactly 10 m up, exactly 4 m/s along +X, perfectly level,
    zero body rates. Only the marker moves (inside a 5 m disk). That makes every
    generalization claim untestable — a policy can memorize one approach.

    Turning this on samples the whole handoff per episode instead. The marker
    stays ``marker_dist`` ahead ALONG THE SAMPLED HEADING (plus its own disk
    offset), so the nominal geometry is preserved while its world-frame
    orientation, the closing speed, the altitude and the entry offset all vary.

    NOTE on ``heading_range_deg``: the v-track observation is world-frame
    (rel_x/rel_y, velocities, ccip_err, wind), so a random cruise heading makes
    the policy solve the task in every rotation rather than only along +X. That
    is a real (and intended) generalization burden — if it proves to be the
    binding constraint, the fix is a heading-invariant observation frame, which
    is an obs-layout change and therefore NOT part of P0.

    ``enabled=False`` (default) reproduces the fixed v11-v19 handoff exactly and
    draws no random numbers."""

    enabled: bool = False

    speed_range: tuple[float, float] = (2.0, 6.0)          # m/s at handoff
    heading_range_deg: tuple[float, float] = (-180.0, 180.0)  # world cruise heading
    alt_range: tuple[float, float] = (8.0, 12.0)           # m
    lateral_offset_max: float = 3.0   # m, spawn offset perpendicular to the cruise axis
    along_offset_max: float = 2.0     # m, spawn offset along the cruise axis (range jitter)
    vel_noise_std: float = 0.3        # m/s, added to all 3 velocity axes
    attitude_std_deg: float = 5.0     # roll/pitch at handoff
    ang_vel_std: float = 0.2          # rad/s, body rates at handoff


@configclass
class DroneBombardV11Cfg(DroneBombardEnvCfg):
    # 24-D obs (doc 53 §3) replaces the 14-D vision obs. Action stays 6-D
    # ([0:4] vel, [4] drop_signal, [5] reserved). phase stays 1 so the inherited
    # DR/residual/moving/release flags all derive to False (kept inert), and the
    # Phase-1 logging branch in _log_reset_extras applies.
    observation_space = 24
    action_space = 6

    v11_test: bool = True

    # --- fixed-marker cruise scenario (the "no random spawn" modification) ---
    marker_dist: float = 20.0      # marker fixed this far ahead of spawn (m)
    cruise_speed: float = 4.0      # initial forward speed at handoff (m/s) — real cruise
    cruise_dir_deg: float = 0.0    # cruise heading in world frame (deg). 0 = +x (East)
    v11_spawn_alt: float = 10.0    # fixed spawn altitude (m)

    # --- v12 expansion: random marker inside a disk (toggle; v11 keeps False) ---
    # False -> fixed marker marker_dist ahead (v11). True -> marker center sampled
    # (area-uniform) inside a disk of radius marker_spawn_radius around that point.
    marker_random: bool = False
    marker_spawn_radius: float = 5.0

    # --- P0 handoff randomization (inert unless handoff.enabled; see v20) ---
    handoff: DroneBombardHandoffCfg = DroneBombardHandoffCfg()

    # --- release envelope (doc 53 §6) ---
    release_radius: float = 1.0
    release_alt_min: float = 3.0
    release_alt_max: float = 8.0
    release_max_speed: float = 5.0
    release_max_vz: float = 3.0
    release_max_tilt: float = 0.35
    release_max_ang_vel: float = 4.0

    # --- v11 reward (doc 53 §8) ---
    v11_w_progress: float = 1.0
    v11_w_ccip: float = 0.5
    v11_k_ccip: float = 1.0
    v11_w_ang_vel: float = 0.05
    v11_w_tilt: float = 0.05
    v11_w_action_smooth: float = 0.05
    v11_w_time: float = 0.01
    v11_gate_reward: float = 0.05
    v11_drop_signal_reward: float = 1.0
    v11_reward_success: float = 300.0
    v11_k_landing: float = 1.0
    v11_success_radius: float = 1.0
    v11_no_drop_penalty: float = -30.0
    v11_crash_penalty: float = -50.0
    v11_out_of_range_penalty: float = -30.0


@configclass
class DroneBombardV12Cfg(DroneBombardV11Cfg):
    """v12 = first expansion of v11: random marker inside a 5 m disk around the
    fixed 20 m point (method A — drone still cruises +X and steers to it).
    Everything else (drop_signal, release envelope, nominal physics, inert
    DR/residual/moving/vision hooks) is identical to v11."""
    marker_random: bool = True
    marker_spawn_radius: float = 5.0


@configclass
class DroneBombardV13Cfg(DroneBombardV12Cfg):
    """v13 = v12 + partial observability ("reveal"). The drone cruises +X BLIND
    (marker position masked from the obs) until it comes within reveal_radius
    (HORIZONTAL d_xy) of the randomly-spawned marker; only then is the marker
    position revealed. NOT latched — if it later leaves the radius the info is cut
    off again, and a per-step penalty applies whenever the marker is not detected
    (forcing the drone to acquire AND keep contact). A 25th obs channel carries the
    detected flag; marker-dependent rewards are gated on detection so the reward
    never leaks the hidden marker position."""
    observation_space = 25            # 24 + detected flag
    reveal_radius: float = 7.0        # horizontal d_xy (m) within which the marker is seen
    v13_undetected_penalty: float = -0.2  # per-step reward while marker not detected


@configclass
class DroneBombardV14Cfg(DroneBombardV12Cfg):
    """v14 = v12 + domain randomization + learned CCIP residual (Stage A).

    Built on v12 (perfect target obs, random marker) — NOT on v13 — so the
    residual-learning problem is isolated from the perception problem.

    With DR on, per-episode wind/drag make the ACTUAL ballistic impact
    (``ballistic_impact(..., _drag_coef, _wind_xy)``, used for the landing
    outcome) drift away from the NOMINAL CCIP prediction (``predict_impact_nominal``,
    drag/wind-free) that the policy aims with. The policy closes that gap with a
    learned 2-D impact-point residual on ``action[5:7]`` (+/- residual_scale m),
    applied to the nominal prediction — so the gate/aim use the CORRECTED impact
    while the reward still scores the REAL one.

    Stage A resolves the "wind trap" (an unobservable per-env bias cannot be
    corrected by a residual) by putting wind/drag directly in the obs. Stage B
    (v15) removes them and must infer the bias from motion history.
    """
    observation_space = 27            # 24 (v12) + wind_xy (2) + drag (1)
    action_space = 7                  # [0:4] vel, [4] drop_signal, [5:7] CCIP residual

    v14_dr: bool = True               # sample per-episode wind/drag
    v14_residual: bool = True         # apply the policy residual (False = control group)
    v14_residual_scale: float = 3.0   # metres; action[5:7] in [-1,1] -> +/- 3 m correction

    # DR strength — start milder than the base phase_cfg defaults (1.5 / 0.15).
    v14_wind_std: float = 1.0         # m/s, N(0, std) per horizontal axis
    v14_drag_max: float = 0.15        # U[0, drag_max] first-order drag coefficient

    # obs normalisation for the newly observable bias channels
    v14_wind_obs_scale: float = 5.0
    v14_drag_obs_scale: float = 0.2


@configclass
class DroneBombardV15Cfg(DroneBombardV14Cfg):
    """v15 = v14 + the wind physically pushes the DRONE.

    In v14 the sampled wind only displaced the payload's ballistic impact — the
    airframe flew in still air, so the policy could beat the wind by simply
    repositioning (release lower/slower), which is why the residual-off control
    still reached 0.82 m. Here the relative airflow also exerts quadratic drag on
    the airframe (base hook ``wind_force_enabled``), so holding a position now
    costs a tilt and the wind genuinely disturbs the flight.

    Same env class as v14 (the force lives in the base controller, cfg-gated).
    """
    wind_force_enabled: bool = True
    # N(0, 2.0) per axis, magnitude-capped at 5 m/s. Chosen so the payload's
    # wind drift stays within the CCIP residual's +/-3 m authority: at 4.0 the
    # drift reached 3.7-7.7 m and saturated the residual (impact plateaued ~3 m,
    # v15 first run). At 2.0 the typical drift is ~2 m (< 3 m) so the residual can
    # actually close it, while the wind still tilts the airframe a meaningful
    # ~1-2 deg. obs scale matched to the smaller wind so it uses the [-1,1] range.
    v14_wind_std: float = 2.0
    v14_wind_obs_scale: float = 6.0
    v14_wind_max: float = 5.0  # 0 disables the magnitude cap


@configclass
class DroneBombardV16Cfg(DroneBombardV12Cfg):
    """v16 = v12 + a REAL physics payload that is carried, dropped, and LANDS.

    Built on v12 (perfect target obs, random marker, nominal physics) to isolate
    the physical-drop mechanism from wind-on-airframe / residual. Until now the
    "drop" was analytic — a formula (``ballistic_impact``) placed the impact and
    the episode ended at the drop instant. Here a real RigidObject falls under
    gravity + wind drag and the episode ends when IT lands; success and the
    terminal reward are scored from the actual landing point. After release the
    drone hovers (option A) while the payload falls. The analytic CCIP still
    drives the obs and the release gate (aim guidance) — "predict, then verify".
    """
    payload_physics_enabled: bool = True


@configclass
class DroneBombardV17Cfg(DroneBombardV13Cfg):
    """v17 = v13 + PIXEL-QUANTIZED vision (harder perception).

    Inside the reveal radius the marker position is no longer exact: it is snapped
    to the CENTRE of the pixel cell it falls in, and the cell size grows with the
    slant range (coarse when far, fine when near). So on first detection the drone
    only knows "roughly over there" (cell ~1 m at 7 m > success radius) and heads
    for the fuzzy cell centre; as it closes in the cell shrinks and the estimate
    sharpens toward the true position, so it must approach to drop precisely. The
    reveal gate + undetected penalty are inherited from v13; the policy perceives
    the quantized target (obs + CCIP + gate) while success is judged on the TRUE
    landing. Toggle with pixel_vision_enabled (False == v13 exact reveal).
    """
    pixel_vision_enabled: bool = True
    pixel_cell_k: float = 0.15   # pixel cell size (m) = k * slant range (drone->target)


@configclass
class DroneBombardV18Cfg(DroneBombardV15Cfg):
    """v18 = STAGED INTEGRATION #1: perception (v17) + physics (v14/v15).

    Merges the two branches that until now grew separately:
      * perception (v11->v12->v13->v17): random marker, blind cruise + reveal at
        7 m + undetected penalty + detected flag, and PIXEL-QUANTIZED position
        (coarse far, fine near).
      * physics (v11->v12->v14->v15): wind/drag domain randomization, a learned
        CCIP residual on action[5:7], and the wind physically pushing the airframe.

    So the drone must acquire a target it can't see until close, read only a fuzzy
    pixel estimate of it, fight the wind on the airframe, and learn a residual to
    correct the wind-blown ballistics — all at once. Drop stays ANALYTIC (physical
    payload = staged integration #3, v16). obs = 28-D (v11 24 + wind_xy 2 + drag 1
    + detected 1), action = 7-D (residual). All the underlying toggles stay live so
    difficulty can be dialed back for debugging.
    """
    observation_space = 28   # 24 + wind_xy(2) + drag(1) + detected(1)
    # perception knobs (from v13/v17); physics knobs inherited from V15Cfg
    reveal_radius: float = 7.0
    v13_undetected_penalty: float = -0.2
    pixel_vision_enabled: bool = True
    pixel_cell_k: float = 0.12          # slightly sharper than 0.15 (eases the gate)

    # --- integration bootstrap tuning (fixes the v18 deadlock) ---
    # The first v18 run stalled at release_rate 0: early RANDOM residual (+/-3 m)
    # kept |nominal+residual - perceived_target| above the 1 m release gate, so the
    # drone never dropped and never got a learning signal (confirmed: residual OFF
    # -> release 100%). Widening the gate lets a drop happen despite the random
    # residual (success is still judged at success_radius=1.0), a smaller residual
    # scale shrinks the early corruption, and milder wind shrinks the drift the
    # residual must cover.
    release_radius: float = 1.5         # was 1.0 — bootstrap the drop through the gate
    v14_residual_scale: float = 2.0     # was 3.0 — less early gate corruption
    v14_wind_std: float = 1.5           # was 2.0 — smaller drift to correct


@configclass
class DroneBombardV19Cfg(DroneBombardV18Cfg):
    """v19 = staged integration #3: v18 (perception + physics) + REAL physical
    payload drop (v16). The payload is now a RigidObject that actually falls — in
    the DR wind, so its trajectory bends — and lands; the episode ends on the
    landing and success/reward use the REAL landing point (not the analytic
    formula). Same 28-D obs / 7-D action as v18, so a v18 checkpoint warm-starts
    it cleanly (fly/aim/release policy transfers; only post-release dynamics
    change). Keeps v18's eased params for the same gate-deadlock reason."""
    payload_physics_enabled: bool = True

    # --- anti-"hover forever" reward fixes (v19 collapse fix) ---
    # The iter-499 checkpoint collapsed to release_rate 0 (aim still perfect, drop
    # never fired): the standing CCIP aim reward (w_ccip*exp(-k*d_impact)) paid a
    # guaranteed +w_ccip every step for holding aim, so hovering-without-dropping
    # out-earned the noisy physical drop. Fix A makes that shaping potential-based
    # (rewards only IMPROVING aim, zero for holding it). Fix B adds an escalating
    # penalty for loitering IN the release envelope without dropping, so once the
    # gate is open the only way out of the growing cost is to actually release.
    ccip_potential_shaping: bool = True   # A: telescoping aim shaping (no standing reward)
    v19_w_loiter: float = 0.02            # B: per-step penalty * steps-in-envelope-not-dropped

    # --- precision push (v19 accuracy fix) ---
    # The original landing reward was FLAT (reward_success) everywhere inside
    # success_radius, so a 0.1 m hit and a 0.9 m hit paid identically -> the policy
    # had no gradient to land tighter and med_err plateaued ~0.56 m. Make it a
    # continuous exp that keeps pulling toward 0 m even inside the zone, plus a
    # discrete bonus for crossing the success threshold (keeps the "get inside"
    # signal). Warm-start the existing best model into this to sharpen accuracy.
    precise_landing_reward: bool = True   # continuous (vs flat-inside) landing reward
    v19_k_landing: float = 2.0            # steeper falloff -> strong sub-metre gradient
    v19_success_bonus: float = 100.0      # discrete bump for err <= success_radius


@configclass
class DroneBombardV20Cfg(DroneBombardV19Cfg):
    """v20 = v19 + the P0 generalization pass: randomized handoff + dynamics/
    sensing domain randomization (notes/research/paper_research_plan.md §5 P0).

    Nothing about the task, the reward, the observation layout or the action
    layout changes — obs stays 28-D and action 7-D, so a v19 checkpoint
    warm-starts losslessly and the two are directly comparable. What changes is
    the DISTRIBUTION the policy is trained and evaluated on:

      * handoff: cruise heading / speed / altitude / entry offset / attitude /
        body rates are sampled per episode instead of being one fixed condition.
      * dyn_dr: controller mass belief +-5%, velocity-loop gains +-10%,
        attitude+rate-loop gains +-10%, payload ballistic coefficient (== payload
        mass) +-20%, plus per-step AND per-episode-bias noise on observations and
        on the velocity commands.

    Wind/drag DR is inherited from v18/v19 unchanged (wind_std 1.5, drag U[0,0.15],
    wind pushing the airframe).

    NOT this class: the "does the curriculum matter" ablation, which is v19 with
    the same cfg trained fresh instead of warm-started — a RUN configuration, not
    an env variant."""

    handoff: DroneBombardHandoffCfg = DroneBombardHandoffCfg(enabled=True)
    dyn_dr: DroneBombardDynDRCfg = DroneBombardDynDRCfg(enabled=True)


class DroneBombardV11Env(DroneBombardEnv):
    cfg: DroneBombardV11Cfg

    def __init__(self, cfg: DroneBombardV11Cfg, render_mode: str | None = None, **kwargs):
        super().__init__(cfg, render_mode, **kwargs)
        N, device = self.num_envs, self.device
        self._wants_drop = torch.zeros(N, dtype=torch.bool, device=device)
        self._d_impact = torch.zeros(N, device=device)
        self._d_impact_prev = torch.zeros(N, device=device)
        self._gate_open = torch.zeros(N, dtype=torch.bool, device=device)
        ang = math.radians(cfg.cruise_dir_deg)
        self._cruise_unit = torch.tensor([math.cos(ang), math.sin(ang)], device=device)
        self._cruise_yaw = float(ang)

    # ------------------------------------------------------------------
    # Action: [0:4] velocity (reuse parent pipeline), [4] drop_signal, [5] ignore
    # ------------------------------------------------------------------
    def _pre_physics_step(self, actions: torch.Tensor):
        from .math_utils import rate_limit_action

        # Actuation noise on the velocity dims only (identity unless dyn_dr);
        # the drop signal [4] and the CCIP residual [5:7] stay clean so the two
        # mechanisms under study are not confounded — see DroneBombardDynDRCfg.
        actions = self._perturb_actions(actions)
        self._action_sat_sum += (actions.abs() > 1.0).float().mean(dim=-1)
        clipped = torch.clamp(actions, -1.0, 1.0)
        vel_clipped = clipped[:, :4]
        prev_vel = self._prev_action[:, :4]
        limited_vel = rate_limit_action(vel_clipped, prev_vel, self.cfg.action.rate_limit)
        self._delta_action = limited_vel - prev_vel
        self._wants_drop = clipped[:, 4] > 0.5  # policy drop intent
        # store vel in prev_action[:, :4] (obs prev-action channels); [4:6] unused
        self._prev_action = torch.cat([limited_vel, torch.zeros_like(clipped[:, 4:6])], dim=-1)
        a = self.cfg.action
        self._vel_cmd = torch.stack([
            limited_vel[:, 0] * a.vx_scale,
            limited_vel[:, 1] * a.vy_scale,
            limited_vel[:, 2] * a.vz_scale,
            limited_vel[:, 3] * a.yaw_scale,
        ], dim=-1)
        self._physics_tick = 0

    # ------------------------------------------------------------------
    # helpers
    # ------------------------------------------------------------------
    def _kinematics(self):
        pos = self._robot.data.root_pos_w - self.scene.env_origins
        vel = self._robot.data.root_lin_vel_w
        ang = self._robot.data.root_ang_vel_b
        roll, pitch, yaw = euler_xyz_from_quat(self._robot.data.root_quat_w)
        roll = torch.atan2(torch.sin(roll), torch.cos(roll))
        pitch = torch.atan2(torch.sin(pitch), torch.cos(pitch))
        yaw = torch.atan2(torch.sin(yaw), torch.cos(yaw))
        return pos, vel, ang, roll, pitch, yaw

    def _ccip(self, pos, vel):
        dc = self.cfg.drop
        pos_xy, altitude, vel_xy = pos[:, :2], pos[:, 2], vel[:, :2]
        impact = predict_impact_nominal(pos_xy, vel_xy, altitude, dc.release_delay, dc.gravity)
        ccip_err = impact - self._target_xy
        d_impact = torch.linalg.norm(ccip_err, dim=-1)
        t_f = time_to_fall(altitude, dc.gravity)
        return ccip_err, d_impact, t_f

    def _step_moving_target(self):
        """Advance the target (X marker) once per policy step when the moving
        target is on (``--moving_target`` -> cfg.moving_target_force; the
        v-track keeps phase=1). Same integration point as the base env's
        ``_advance_phase_dynamics``: the top of ``_get_dones``, so the gate,
        rewards, and the following observation all see the same target state.
        Initial velocity / accel / turn rate come from the base ``_reset_idx``
        samplers; obs stays unchanged (the marker channels just move), so
        stationary-target checkpoints warm-start losslessly."""
        if not self.cfg.moving_target_enabled:
            return
        dt = self.cfg.sim.dt * self.cfg.decimation
        pc = self.cfg.phase_cfg
        self._target_xy, self._target_vel_xy = step_target_motion(
            self._target_xy, self._target_vel_xy,
            self._target_acc_xy, self._target_omega, dt,
            pc.target_motion_model,
            theta=pc.target_vel_theta, sigma=pc.target_vel_sigma,
        )

    # ------------------------------------------------------------------
    # Observation: 24-D (doc 53 §3), no vision
    # ------------------------------------------------------------------
    def _get_observations(self) -> dict:
        self._update_markers()
        pos, vel, ang, roll, pitch, yaw = self._kinematics()
        altitude = pos[:, 2]
        vel_xy = vel[:, :2]
        ccip_err, d_impact, t_f = self._ccip(pos, vel)
        speed_xy = torch.linalg.norm(vel_xy, dim=-1)
        self._d_impact = d_impact  # cache for reward

        rel_x = self._target_xy[:, 0] - pos[:, 0]
        rel_y = self._target_xy[:, 1] - pos[:, 1]
        rel_z = 0.0 - altitude

        def c(x, s):
            return torch.clamp(x / s, -1.0, 1.0)

        obs = torch.stack([
            c(rel_x, 20.0), c(rel_y, 20.0), c(rel_z, 20.0),
            c(vel[:, 0], 10.0), c(vel[:, 1], 10.0), c(vel[:, 2], 10.0),
            torch.clamp(roll / math.pi, -1.0, 1.0), torch.clamp(pitch / math.pi, -1.0, 1.0),
            torch.sin(yaw), torch.cos(yaw),
            c(ang[:, 0], math.pi), c(ang[:, 1], math.pi), c(ang[:, 2], math.pi),
            c(ccip_err[:, 0], 10.0), c(ccip_err[:, 1], 10.0),
            torch.clamp(d_impact / 10.0, 0.0, 1.0),
            torch.clamp(t_f / 5.0, 0.0, 1.0),
            torch.clamp(speed_xy / 10.0, 0.0, 1.0),
            torch.clamp(altitude / 20.0, 0.0, 1.0),
            self._payload_attached.float(),
            self._prev_action[:, 0], self._prev_action[:, 1],
            self._prev_action[:, 2], self._prev_action[:, 3],
        ], dim=-1)
        obs = torch.nan_to_num(obs, nan=0.0)
        return {"policy": obs}

    # ------------------------------------------------------------------
    # Reset: fixed marker ahead in cruise direction + cruise-speed handoff
    # ------------------------------------------------------------------
    def _reset_idx(self, env_ids: torch.Tensor):
        # Reuse ALL of the parent's machinery (buffer zeroing, DR samplers that
        # return zero in this config, spawn-time mass, per-episode logging of
        # the just-ended episode). Then overwrite the scenario placement.
        super()._reset_idx(env_ids)

        if env_ids is None or len(env_ids) == self.num_envs:
            env_ids = self._robot._ALL_INDICES
        n = len(env_ids)
        device = self.device

        # --- handoff geometry: fixed (v11-v19) or sampled per episode (P0) ---
        # Everything downstream reads the per-env tensors below, so the two paths
        # share one code path. Disabled -> constants, and sample_uniform draws no
        # RNG for a collapsed range, keeping v11-v19 streams bit-identical.
        hc = self.cfg.handoff
        if hc.enabled:
            cruise_yaw = sample_uniform(
                math.radians(hc.heading_range_deg[0]), math.radians(hc.heading_range_deg[1]), n, device)
            cruise_speed = sample_uniform(hc.speed_range[0], hc.speed_range[1], n, device)
            spawn_alt = sample_uniform(hc.alt_range[0], hc.alt_range[1], n, device)
        else:
            cruise_yaw = torch.full((n,), self._cruise_yaw, device=device)
            cruise_speed = torch.full((n,), self.cfg.cruise_speed, device=device)
            spawn_alt = torch.full((n,), self.cfg.v11_spawn_alt, device=device)
        cruise_unit = torch.stack([torch.cos(cruise_yaw), torch.sin(cruise_yaw)], dim=-1)  # [n,2]
        cruise_perp = torch.stack([-cruise_unit[:, 1], cruise_unit[:, 0]], dim=-1)

        # Marker center: marker_dist ahead along the (per-env) cruise direction.
        center = cruise_unit * self.cfg.marker_dist
        if self.cfg.marker_random:
            # v12: sample the marker center (area-uniform) inside a disk of radius
            # marker_spawn_radius around that point. The drone still spawns cruising
            # along the cruise direction, so it must steer to the off-axis marker.
            r = self.cfg.marker_spawn_radius * torch.sqrt(torch.rand(n, device=device))
            theta = torch.rand(n, device=device) * (2.0 * math.pi)
            offset = torch.stack([r * torch.cos(theta), r * torch.sin(theta)], dim=-1)
            marker = center + offset
        else:
            marker = center
        self._target_xy[env_ids] = marker

        # Spawn offset in the CRUISE frame (zero unless handoff DR is on): a
        # lateral component moves the entry off the marker bearing, an along
        # component jitters the effective approach range.
        spawn_xy = torch.zeros(n, 2, device=device)
        if hc.enabled and (hc.lateral_offset_max > 0.0 or hc.along_offset_max > 0.0):
            lat = (torch.rand(n, device=device) * 2.0 - 1.0) * hc.lateral_offset_max
            along = (torch.rand(n, device=device) * 2.0 - 1.0) * hc.along_offset_max
            spawn_xy = cruise_perp * lat.unsqueeze(-1) + cruise_unit * along.unsqueeze(-1)

        root = self._robot.data.default_root_state[env_ids].clone()
        root[:, 0:2] = self.scene.env_origins[env_ids, :2] + spawn_xy
        root[:, 2] = spawn_alt
        if hc.enabled and hc.attitude_std_deg > 0.0:
            att_std = math.radians(hc.attitude_std_deg)
            roll0 = torch.randn(n, device=device) * att_std
            pitch0 = torch.randn(n, device=device) * att_std
        else:
            roll0 = torch.zeros(n, device=device)
            pitch0 = torch.zeros(n, device=device)
        # quat_from_euler_xyz with roll=pitch=0 reduces exactly to the old
        # (cos(yaw/2), 0, 0, sin(yaw/2)) construction, so the disabled path is
        # unchanged.
        root[:, 3:7] = quat_from_euler_xyz(roll0, pitch0, cruise_yaw)
        root[:, 7:9] = cruise_unit * cruise_speed.unsqueeze(-1)
        root[:, 9] = 0.0
        root[:, 10:13] = 0.0
        if hc.enabled and hc.vel_noise_std > 0.0:
            root[:, 7:10] = root[:, 7:10] + torch.randn(n, 3, device=device) * hc.vel_noise_std
        if hc.enabled and hc.ang_vel_std > 0.0:
            root[:, 10:13] = torch.randn(n, 3, device=device) * hc.ang_vel_std
        self._robot.write_root_pose_to_sim(root[:, :7], env_ids)
        self._robot.write_root_velocity_to_sim(root[:, 7:13], env_ids)

        self._wants_drop[env_ids] = False
        # distance from the (possibly offset) spawn to the marker.
        d0 = torch.linalg.norm(marker - spawn_xy, dim=-1)
        self._d_xy_prev[env_ids] = d0
        self._d_impact_prev[env_ids] = d0

        # --- cruise handoff: seed the controller at the cruise setpoint ---
        # The drone spawns already moving at cruise_speed. super() zeroed the
        # velocity LPF (_v_filt) and _prev_action, so without this the step-1
        # controller setpoint would be a random policy command -> a ~cruise_speed
        # velocity error -> an aggressive corrective tilt that trips bad_attitude
        # and kills the episode on step 1. Seeding _prev_action (so the
        # rate-limited step-1 command stays near cruise) and _v_filt (the actual
        # world-frame setpoint the snap re-applies) makes the handoff smooth:
        # "receive a drone already cruising, hold unless commanded otherwise".
        a = self.cfg.action
        cruise_vx = cruise_unit[:, 0] * cruise_speed
        cruise_vy = cruise_unit[:, 1] * cruise_speed
        self._prev_action[env_ids, 0] = torch.clamp(cruise_vx / a.vx_scale, -1.0, 1.0)
        self._prev_action[env_ids, 1] = torch.clamp(cruise_vy / a.vy_scale, -1.0, 1.0)
        self._prev_action[env_ids, 2] = 0.0
        self._prev_action[env_ids, 3] = 0.0
        self._prev_action[env_ids, 4:6] = 0.0
        self._v_filt[env_ids, 0] = cruise_vx
        self._v_filt[env_ids, 1] = cruise_vy
        self._v_filt[env_ids, 2] = 0.0
        self._v_filt[env_ids, 3] = 0.0

    # ------------------------------------------------------------------
    # Dones: drop_signal + release-envelope -> release-terminal + landing
    # ------------------------------------------------------------------
    def _get_dones(self) -> tuple[torch.Tensor, torch.Tensor]:
        self._step_moving_target()
        cfg = self.cfg
        tc = cfg.termination
        dc = cfg.drop
        pos, vel, ang, roll, pitch, _ = self._kinematics()
        altitude = pos[:, 2]
        pos_xy, vel_xy = pos[:, :2], vel[:, :2]
        speed = torch.linalg.norm(vel, dim=-1)
        speed_xy = torch.linalg.norm(vel_xy, dim=-1)
        ang_norm = torch.linalg.norm(ang, dim=-1)

        _, d_impact, _ = self._ccip(pos, vel)
        self._d_impact = d_impact
        self._aim_err_min = torch.minimum(self._aim_err_min, d_impact)

        gate = release_gate(
            d_impact, altitude, speed_xy, vel[:, 2], roll, pitch, ang_norm,
            self._payload_attached.float(),
            cfg.release_radius, cfg.release_alt_min, cfg.release_alt_max,
            cfg.release_max_speed, cfg.release_max_vz, cfg.release_max_tilt, cfg.release_max_ang_vel,
        )
        self._gate_open = gate
        fire = self._wants_drop & gate & (~self._released)

        # Real payload landing (nominal physics: drag/wind == 0 in this config,
        # so this equals the nominal CCIP prediction — but computed via the same
        # ballistic_impact so enabling DR later flows through unchanged).
        real_impact = ballistic_impact(
            pos_xy, vel_xy, altitude, dc.release_delay, dc.gravity, self._drag_coef, self._wind_xy
        )
        landing_err = torch.linalg.norm(real_impact - self._target_xy, dim=-1)

        self._just_released = fire.clone()
        self._released = self._released | fire
        self._release_impact_err = torch.where(fire, landing_err, self._release_impact_err)
        self._release_aim_xy = torch.where(fire.unsqueeze(-1), real_impact, self._release_aim_xy)
        self._release_target_xy = torch.where(fire.unsqueeze(-1), self._target_xy, self._release_target_xy)
        self._payload_attached = self._payload_attached & (~fire)  # release detaches the payload

        d_xy = self._current_d_xy()
        self._d_xy_min = torch.minimum(self._d_xy_min, d_xy)
        # v11 has no overshoot/stagnation guards; keep the flag the snapshot reads.
        self._overshoot_flythrough = torch.zeros_like(gate)

        step = self.episode_length_buf
        crash = (altitude < tc.ground_contact_altitude) | (
            (step > tc.min_altitude_start_step) & (altitude < tc.min_altitude)
        )
        overspeed = speed > tc.v_max_safety
        inverted = (roll.abs() > tc.limit_inverted_tilt) | (pitch.abs() > tc.limit_inverted_tilt)
        bad_attitude = (ang_norm > tc.limit_ang_vel) | inverted
        out_of_range = d_xy > tc.max_distance
        max_alt = altitude > tc.max_altitude

        success = self._just_released & (self._release_impact_err <= cfg.v11_success_radius)
        released_this = self._just_released.clone()
        failure = crash | overspeed | bad_attitude | out_of_range | max_alt
        terminated = success | failure | released_this
        time_out = step >= (self.max_episode_length - 1)

        z = torch.zeros_like(gate)
        self._done_flags = {
            "success": success, "crash": crash, "overspeed": overspeed,
            "bad_attitude": bad_attitude, "out_of_range": out_of_range,
            "max_altitude": max_alt, "overshoot": z, "stagnation": z,
            "released": released_this, "release_miss": released_this & ~success,
            "timeout": time_out & ~terminated,
            "gate_open": gate, "wants_drop": self._wants_drop,
        }
        return terminated, time_out & ~terminated

    # ------------------------------------------------------------------
    # Rewards: doc 53 §8 (progress + CCIP + stability + smooth + time + gate +
    # drop_signal + landing terminal + no-drop timeout; no invalid-drop penalty)
    # ------------------------------------------------------------------
    def _get_rewards(self) -> torch.Tensor:
        cfg = self.cfg
        _, _, ang, roll, pitch, _ = self._kinematics()
        f = self._done_flags
        d_impact = self._d_impact
        d_xy = self._current_d_xy()

        r = cfg.v11_w_progress * (self._d_xy_prev - d_xy)          # approach progress
        r = r + cfg.v11_w_ccip * torch.exp(-cfg.v11_k_ccip * d_impact)  # CCIP aim shaping
        r = r - cfg.v11_w_ang_vel * (ang * ang).sum(dim=-1)
        r = r - cfg.v11_w_tilt * (roll * roll + pitch * pitch)
        r = r - cfg.v11_w_action_smooth * (self._delta_action * self._delta_action).sum(dim=-1)
        r = r - cfg.v11_w_time

        r = r + f["gate_open"].float() * cfg.v11_gate_reward
        r = r + (self._wants_drop & f["gate_open"]).float() * cfg.v11_drop_signal_reward

        landing_err = self._release_impact_err
        landing_reward = torch.where(
            landing_err <= cfg.v11_success_radius,
            torch.full_like(landing_err, cfg.v11_reward_success),
            cfg.v11_reward_success * torch.exp(-cfg.v11_k_landing * landing_err),
        )
        r = r + f["released"].float() * landing_reward

        r = r + f["crash"].float() * cfg.v11_crash_penalty
        r = r + f["out_of_range"].float() * cfg.v11_out_of_range_penalty
        r = r + f["timeout"].float() * cfg.v11_no_drop_penalty  # timed out without releasing

        self._d_xy_prev = d_xy
        self._d_impact_prev = d_impact
        return torch.nan_to_num(r, nan=0.0)


class DroneBombardV14Env(DroneBombardV11Env):
    """v14: domain randomization + learned CCIP residual on top of v12.

    Four small hooks on the v11 pipeline:
      * ``_reset_idx``  — sample per-episode wind/drag (the parent zeroes them,
        since the phase-derived ``dr_enabled`` stays False for the v11 line).
      * ``_pre_physics_step`` — capture ``action[5:7]`` as the residual.
      * ``_ccip`` — ADD the residual to the nominal impact prediction. This is the
        single choke point: the corrected impact then flows into the obs
        (ccip_err/d_impact), the release gate, and the CCIP shaping reward, while
        ``_get_dones`` still scores the REAL drag/wind ballistic impact.
      * ``_get_observations`` — append the observable bias (wind_xy, drag).
    """
    cfg: DroneBombardV14Cfg

    def _reset_idx(self, env_ids: torch.Tensor):
        super()._reset_idx(env_ids)  # v11 marker/cruise-seed; base zeroes wind/drag
        if env_ids is None or len(env_ids) == self.num_envs:
            env_ids = self._robot._ALL_INDICES
        n = len(env_ids)
        cfg, device = self.cfg, self.device
        if cfg.v14_dr:
            wind = torch.randn(n, 2, device=device) * cfg.v14_wind_std
            wmax = getattr(cfg, "v14_wind_max", 0.0) or 0.0
            if wmax > 0.0:
                mag = torch.linalg.norm(wind, dim=-1, keepdim=True)
                wind = wind * (wmax / mag.clamp(min=wmax))  # scale down only if |wind| > wmax
            self._wind_xy[env_ids] = wind
            self._drag_coef[env_ids] = torch.rand(n, device=device) * cfg.v14_drag_max
        self._residual_action[env_ids] = 0.0

    def _pre_physics_step(self, actions: torch.Tensor):
        super()._pre_physics_step(actions)  # [0:4] vel, [4] drop_signal
        self._residual_action = torch.clamp(actions[:, 5:7], -1.0, 1.0)

    def _ccip(self, pos: torch.Tensor, vel: torch.Tensor):
        from .math_utils import apply_ccip_residual, predict_impact_nominal, time_to_fall

        dc = self.cfg.drop
        pos_xy, altitude, vel_xy = pos[:, :2], pos[:, 2], vel[:, :2]
        impact = predict_impact_nominal(pos_xy, vel_xy, altitude, dc.release_delay, dc.gravity)
        if self.cfg.v14_residual:
            # learned model-uncertainty correction: nominal + residual ~= real impact
            impact = apply_ccip_residual(impact, self._residual_action, self.cfg.v14_residual_scale)
        ccip_err = impact - self._target_xy
        d_impact = torch.linalg.norm(ccip_err, dim=-1)
        t_f = time_to_fall(altitude, dc.gravity)
        return ccip_err, d_impact, t_f

    def _get_observations(self) -> dict:
        obs24 = super()._get_observations()["policy"]
        cfg = self.cfg
        wind_n = torch.clamp(self._wind_xy / cfg.v14_wind_obs_scale, -1.0, 1.0)
        drag_n = torch.clamp(self._drag_coef / cfg.v14_drag_obs_scale, 0.0, 1.0).unsqueeze(-1)
        obs = torch.cat([obs24, wind_n, drag_n], dim=-1)  # 27-D
        return {"policy": torch.nan_to_num(obs, nan=0.0)}


class DroneBombardV16Env(DroneBombardV11Env):
    """v16: physical payload drop with a LAND-terminal episode.

    The drop event releases the real payload (base ``_step_payload_physics`` then
    flies it under gravity + wind drag) and hovers the drone in place; the episode
    ends when the PAYLOAD lands, and success / terminal reward are scored from the
    actual landing point rather than the analytic ballistic formula.
    """
    cfg: DroneBombardV16Cfg

    def _pre_physics_step(self, actions: torch.Tensor):
        super()._pre_physics_step(actions)  # [0:4] vel, [4] drop_signal
        # option A: once released the drone's job is done — hover in place so its
        # post-release flight can't confound the payload's fall. (_released reflects
        # last step; the one-step lag is harmless.)
        if self._released.any():
            held = self._released
            self._vel_cmd[held] = 0.0
            self._wants_drop[held] = False

    def _get_dones(self) -> tuple[torch.Tensor, torch.Tensor]:
        self._step_moving_target()
        cfg = self.cfg
        tc = cfg.termination
        pos, vel, ang, roll, pitch, _ = self._kinematics()
        altitude = pos[:, 2]
        speed = torch.linalg.norm(vel, dim=-1)
        speed_xy = torch.linalg.norm(vel[:, :2], dim=-1)
        ang_norm = torch.linalg.norm(ang, dim=-1)

        _, d_impact, _ = self._ccip(pos, vel)
        self._d_impact = d_impact
        self._aim_err_min = torch.minimum(self._aim_err_min, d_impact)

        gate = release_gate(
            d_impact, altitude, speed_xy, vel[:, 2], roll, pitch, ang_norm,
            self._payload_attached.float(),
            cfg.release_radius, cfg.release_alt_min, cfg.release_alt_max,
            cfg.release_max_speed, cfg.release_max_vz, cfg.release_max_tilt, cfg.release_max_ang_vel,
        )
        self._gate_open = gate
        fire = self._wants_drop & gate & (~self._released)

        # release: detach + hand to physics (falls next tick with inherited
        # velocity). NOT terminal — we wait for the real landing.
        self._just_released = fire.clone()
        self._released = self._released | fire
        self._payload_attached = self._payload_attached & (~fire)
        self._payload_free = self._payload_free | fire

        d_xy = self._current_d_xy()
        self._d_xy_min = torch.minimum(self._d_xy_min, d_xy)
        self._overshoot_flythrough = torch.zeros_like(gate)

        # real landing outcome (impact xy set by _step_payload_physics on contact).
        landed = self._payload_landed
        impact_err = torch.linalg.norm(self._payload_impact_xy - self._target_xy, dim=-1)
        self._release_impact_err = torch.where(landed, impact_err, self._release_impact_err)

        step = self.episode_length_buf
        crash = (altitude < tc.ground_contact_altitude) | (
            (step > tc.min_altitude_start_step) & (altitude < tc.min_altitude) & (~self._released)
        )
        overspeed = speed > tc.v_max_safety
        inverted = (roll.abs() > tc.limit_inverted_tilt) | (pitch.abs() > tc.limit_inverted_tilt)
        bad_attitude = (ang_norm > tc.limit_ang_vel) | inverted
        out_of_range = d_xy > tc.max_distance
        max_alt = altitude > tc.max_altitude

        # after release the drone hovers to spectate the fall — its flight failures
        # no longer matter, so gate them on ~released (also avoids the freeze
        # transient tripping bad_attitude and pre-empting the landing).
        failure = (crash | overspeed | bad_attitude | out_of_range | max_alt) & (~self._released)
        success = landed & (impact_err <= cfg.v11_success_radius)
        terminated = landed | failure
        time_out = step >= (self.max_episode_length - 1)

        z = torch.zeros_like(gate)
        self._done_flags = {
            "success": success, "crash": crash & (~self._released),
            "overspeed": overspeed & (~self._released),
            "bad_attitude": bad_attitude & (~self._released),
            "out_of_range": out_of_range & (~self._released),
            "max_altitude": max_alt & (~self._released), "overshoot": z, "stagnation": z,
            "released": self._released.clone(), "landed": landed,
            "release_miss": landed & ~success,
            "timeout": time_out & ~terminated,
            "gate_open": gate, "wants_drop": self._wants_drop,
        }
        return terminated, time_out & ~terminated

    def _get_rewards(self) -> torch.Tensor:
        cfg = self.cfg
        _, _, ang, roll, pitch, _ = self._kinematics()
        f = self._done_flags
        d_impact = self._d_impact
        d_xy = self._current_d_xy()

        # flight shaping active only while still carrying (pre-release); once
        # released the drone just hovers, so freeze the shaping to avoid rewarding
        # the spectating phase.
        flying = (~self._released).float()
        r = flying * cfg.v11_w_progress * (self._d_xy_prev - d_xy)
        r = r + flying * cfg.v11_w_ccip * torch.exp(-cfg.v11_k_ccip * d_impact)
        r = r - cfg.v11_w_ang_vel * (ang * ang).sum(dim=-1) * flying
        r = r - cfg.v11_w_tilt * (roll * roll + pitch * pitch) * flying
        r = r - cfg.v11_w_action_smooth * (self._delta_action * self._delta_action).sum(dim=-1) * flying
        r = r - cfg.v11_w_time * flying

        r = r + f["gate_open"].float() * cfg.v11_gate_reward * flying
        r = r + self._just_released.float() * cfg.v11_drop_signal_reward

        # terminal reward on the REAL landing (not the analytic release).
        landing_err = self._release_impact_err
        landing_reward = torch.where(
            landing_err <= cfg.v11_success_radius,
            torch.full_like(landing_err, cfg.v11_reward_success),
            cfg.v11_reward_success * torch.exp(-cfg.v11_k_landing * landing_err),
        )
        r = r + f["landed"].float() * landing_reward

        r = r + f["crash"].float() * cfg.v11_crash_penalty
        r = r + f["out_of_range"].float() * cfg.v11_out_of_range_penalty
        # never-dropped timeout penalty (only if it truly never released)
        r = r + (f["timeout"] & ~self._released).float() * cfg.v11_no_drop_penalty

        self._d_xy_prev = d_xy
        self._d_impact_prev = d_impact
        return torch.nan_to_num(r, nan=0.0)


class DroneBombardV13Env(DroneBombardV11Env):
    """v13: partial-observability "reveal" on top of v12's random marker.

    Blind +X cruise until the drone is within cfg.reveal_radius (horizontal) of
    the marker; only then are the marker-relative obs channels un-masked. NOT
    latched (leaving the radius re-masks them), with a per-step penalty whenever
    the marker is undetected. A 25th obs channel is the detected flag, and every
    marker-dependent reward term is gated on detection so the reward can never
    leak the hidden marker position.
    """
    cfg: DroneBombardV13Cfg

    def __init__(self, cfg: DroneBombardV13Cfg, render_mode: str | None = None, **kwargs):
        super().__init__(cfg, render_mode, **kwargs)
        self._detected = torch.zeros(self.num_envs, dtype=torch.bool, device=self.device)

    def _get_observations(self) -> dict:
        # Reuse v11's 24-D obs, then mask the marker-dependent channels when the
        # marker is not detected and append the detected flag (-> 25-D).
        obs24 = super()._get_observations()["policy"]
        det = self._current_d_xy() <= self.cfg.reveal_radius
        self._detected = det
        m = det.float().unsqueeze(-1)
        mask = torch.ones_like(obs24)
        # channels: 0,1 = rel_x/rel_y ; 13,14 = ccip_err x/y ; 15 = d_impact
        mask[:, [0, 1, 13, 14, 15]] = m
        obs = torch.cat([obs24 * mask, m], dim=-1)
        return {"policy": obs}

    def _get_rewards(self) -> torch.Tensor:
        cfg = self.cfg
        _, _, ang, roll, pitch, _ = self._kinematics()
        f = self._done_flags
        d_impact = self._d_impact
        d_xy = self._current_d_xy()
        det = (d_xy <= cfg.reveal_radius).float()  # marker detected this step?

        # --- marker-independent shaping (always active) ---
        r = -cfg.v11_w_ang_vel * (ang * ang).sum(dim=-1)
        r = r - cfg.v11_w_tilt * (roll * roll + pitch * pitch)
        r = r - cfg.v11_w_action_smooth * (self._delta_action * self._delta_action).sum(dim=-1)
        r = r - cfg.v11_w_time
        # per-step penalty whenever the marker is NOT detected (acquire + keep contact)
        r = r + (1.0 - det) * cfg.v13_undetected_penalty

        # --- marker-dependent shaping, GATED on detection (no position leak) ---
        progress = cfg.v11_w_progress * (self._d_xy_prev - d_xy)
        ccip = cfg.v11_w_ccip * torch.exp(-cfg.v11_k_ccip * d_impact)
        r = r + det * (progress + ccip)
        r = r + det * f["gate_open"].float() * cfg.v11_gate_reward
        r = r + det * (self._wants_drop & f["gate_open"]).float() * cfg.v11_drop_signal_reward

        # landing terminal (release fires only inside the envelope => detected)
        landing_err = self._release_impact_err
        landing_reward = torch.where(
            landing_err <= cfg.v11_success_radius,
            torch.full_like(landing_err, cfg.v11_reward_success),
            cfg.v11_reward_success * torch.exp(-cfg.v11_k_landing * landing_err),
        )
        r = r + f["released"].float() * landing_reward

        r = r + f["crash"].float() * cfg.v11_crash_penalty
        r = r + f["out_of_range"].float() * cfg.v11_out_of_range_penalty
        r = r + f["timeout"].float() * cfg.v11_no_drop_penalty

        self._d_xy_prev = d_xy
        self._d_impact_prev = d_impact
        return torch.nan_to_num(r, nan=0.0)


class DroneBombardV17Env(DroneBombardV13Env):
    """v17: pixel-quantized vision on top of v13's reveal.

    The revealed marker position is snapped to the centre of a pixel cell whose
    size = pixel_cell_k * slant_range, so it is coarse far away and sharpens as the
    drone approaches. The policy perceives only this quantized target (obs, CCIP,
    release gate); the reveal gate + undetected penalty are v13's, and success is
    still judged against the TRUE marker (a fuzzy aim that must be refined by
    closing in). pixel_vision_enabled=False falls back to v13 exact reveal.
    """
    cfg: DroneBombardV17Cfg

    def __init__(self, cfg: DroneBombardV17Cfg, render_mode: str | None = None, **kwargs):
        super().__init__(cfg, render_mode, **kwargs)
        self._perceived_target_xy = torch.zeros(self.num_envs, 2, device=self.device)

    def _quantize_target(self, pos_xy: torch.Tensor, altitude: torch.Tensor) -> torch.Tensor:
        # snap the true camera-relative target to the centre of its pixel cell;
        # cell size grows with slant range (fine near, coarse far).
        rel = self._target_xy - pos_xy
        slant = torch.sqrt((rel * rel).sum(dim=-1) + altitude * altitude)
        cell = (self.cfg.pixel_cell_k * slant).clamp(min=0.05).unsqueeze(-1)
        rel_q = (torch.floor(rel / cell) + 0.5) * cell
        return pos_xy + rel_q

    def _ccip(self, pos: torch.Tensor, vel: torch.Tensor):
        if not self.cfg.pixel_vision_enabled:
            return super()._ccip(pos, vel)
        # CCIP against the PERCEIVED (quantized) target — so obs, the release gate,
        # and the CCIP shaping reward all use the fuzzy position, not the truth.
        perceived = self._quantize_target(pos[:, :2], pos[:, 2])
        self._perceived_target_xy = perceived
        dc = self.cfg.drop
        impact = predict_impact_nominal(pos[:, :2], vel[:, :2], pos[:, 2], dc.release_delay, dc.gravity)
        ccip_err = impact - perceived
        d_impact = torch.linalg.norm(ccip_err, dim=-1)
        t_f = time_to_fall(pos[:, 2], dc.gravity)
        return ccip_err, d_impact, t_f

    def _get_observations(self) -> dict:
        out = super()._get_observations()  # v13 obs; CCIP channels already perceived via _ccip
        if not self.cfg.pixel_vision_enabled:
            return out
        # v11 built the rel_x/rel_y channels (0,1) from the TRUE target — overwrite
        # them with the perceived (quantized) offset, keeping v13's detected mask.
        obs = out["policy"].clone()
        pos = self._robot.data.root_pos_w - self.scene.env_origins
        perceived = self._perceived_target_xy  # set by _ccip during super()
        det = self._detected.float()
        obs[:, 0] = torch.clamp((perceived[:, 0] - pos[:, 0]) / 20.0, -1.0, 1.0) * det
        obs[:, 1] = torch.clamp((perceived[:, 1] - pos[:, 1]) / 20.0, -1.0, 1.0) * det
        return {"policy": torch.nan_to_num(obs, nan=0.0)}


class DroneBombardV18Env(DroneBombardV14Env):
    """v18: perception (v17) + physics (v14/v15) integrated on one env.

    Inherits the v14 physics pipeline (per-episode wind/drag DR in _reset_idx, the
    action[5:7] residual captured in _pre_physics_step, the wind/drag obs channels,
    and — via the base cfg hook wind_force_enabled — the wind pushing the airframe).
    Adds the v13/v17 perception: reveal gate + undetected penalty + detected flag,
    and a PIXEL-QUANTIZED target. _ccip composes both worlds: the CCIP is aimed at
    the pixel-quantized (perceived) target using the residual-corrected ballistic
    prediction. Drop stays analytic (release-terminal); success is the real
    (DR-physics) landing vs the TRUE marker.
    """
    cfg: DroneBombardV18Cfg

    def __init__(self, cfg: DroneBombardV18Cfg, render_mode: str | None = None, **kwargs):
        super().__init__(cfg, render_mode, **kwargs)
        self._perceived_target_xy = torch.zeros(self.num_envs, 2, device=self.device)

    def _quantize_target(self, pos_xy: torch.Tensor, altitude: torch.Tensor) -> torch.Tensor:
        rel = self._target_xy - pos_xy
        slant = torch.sqrt((rel * rel).sum(dim=-1) + altitude * altitude)
        cell = (self.cfg.pixel_cell_k * slant).clamp(min=0.05).unsqueeze(-1)
        rel_q = (torch.floor(rel / cell) + 0.5) * cell
        return pos_xy + rel_q

    def _ccip(self, pos: torch.Tensor, vel: torch.Tensor):
        # perception (v17) + physics (v14): CCIP of the residual-corrected ballistic
        # prediction relative to the pixel-quantized (perceived) target.
        if self.cfg.pixel_vision_enabled:
            perceived = self._quantize_target(pos[:, :2], pos[:, 2])
        else:
            perceived = self._target_xy
        self._perceived_target_xy = perceived
        dc = self.cfg.drop
        impact = predict_impact_nominal(pos[:, :2], vel[:, :2], pos[:, 2], dc.release_delay, dc.gravity)
        if self.cfg.v14_residual:
            impact = apply_ccip_residual(impact, self._residual_action, self.cfg.v14_residual_scale)
        ccip_err = impact - perceived
        d_impact = torch.linalg.norm(ccip_err, dim=-1)
        t_f = time_to_fall(pos[:, 2], dc.gravity)
        return ccip_err, d_impact, t_f

    def _get_observations(self) -> dict:
        # v14 obs (27-D: v11 24 + wind/drag; CCIP channels already perceived+residual
        # via _ccip). Overwrite rel with the perceived offset, mask marker channels
        # when undetected, append the detected flag -> 28-D.
        out = super()._get_observations()
        obs = out["policy"].clone()
        pos = self._robot.data.root_pos_w - self.scene.env_origins
        det = self._current_d_xy() <= self.cfg.reveal_radius
        self._detected = det
        m = det.float()
        if self.cfg.pixel_vision_enabled:
            perceived = self._perceived_target_xy
            obs[:, 0] = torch.clamp((perceived[:, 0] - pos[:, 0]) / 20.0, -1.0, 1.0)
            obs[:, 1] = torch.clamp((perceived[:, 1] - pos[:, 1]) / 20.0, -1.0, 1.0)
        obs[:, [0, 1, 13, 14, 15]] = obs[:, [0, 1, 13, 14, 15]] * m.unsqueeze(-1)  # reveal mask
        obs = torch.cat([obs, m.unsqueeze(-1)], dim=-1)  # detected flag -> 28-D
        return {"policy": torch.nan_to_num(obs, nan=0.0)}

    def _get_rewards(self) -> torch.Tensor:
        # v13 reward: marker-independent shaping always; marker-dependent terms and
        # the drop reward gated on detection; per-step undetected penalty. Landing
        # reward uses _release_impact_err (the real DR-physics miss set in _get_dones).
        cfg = self.cfg
        _, _, ang, roll, pitch, _ = self._kinematics()
        f = self._done_flags
        d_impact = self._d_impact
        d_xy = self._current_d_xy()
        det = (d_xy <= cfg.reveal_radius).float()

        r = -cfg.v11_w_ang_vel * (ang * ang).sum(dim=-1)
        r = r - cfg.v11_w_tilt * (roll * roll + pitch * pitch)
        r = r - cfg.v11_w_action_smooth * (self._delta_action * self._delta_action).sum(dim=-1)
        r = r - cfg.v11_w_time
        r = r + (1.0 - det) * cfg.v13_undetected_penalty

        progress = cfg.v11_w_progress * (self._d_xy_prev - d_xy)
        ccip = cfg.v11_w_ccip * torch.exp(-cfg.v11_k_ccip * d_impact)
        r = r + det * (progress + ccip)
        r = r + det * f["gate_open"].float() * cfg.v11_gate_reward
        r = r + det * (self._wants_drop & f["gate_open"]).float() * cfg.v11_drop_signal_reward

        landing_err = self._release_impact_err
        landing_reward = torch.where(
            landing_err <= cfg.v11_success_radius,
            torch.full_like(landing_err, cfg.v11_reward_success),
            cfg.v11_reward_success * torch.exp(-cfg.v11_k_landing * landing_err),
        )
        r = r + f["released"].float() * landing_reward

        r = r + f["crash"].float() * cfg.v11_crash_penalty
        r = r + f["out_of_range"].float() * cfg.v11_out_of_range_penalty
        r = r + f["timeout"].float() * cfg.v11_no_drop_penalty

        self._d_xy_prev = d_xy
        self._d_impact_prev = d_impact
        return torch.nan_to_num(r, nan=0.0)


class DroneBombardV19Env(DroneBombardV18Env):
    """v19: v18 perception+physics with a REAL physical payload drop (v16).

    Keeps v18's _ccip (pixel-quantized perceived target + residual-corrected
    prediction) and 28-D obs. Swaps v18's analytic release-terminal for v16's
    land-terminal physical drop: the drop event hands a RigidObject to physics
    (it falls in the DR wind), the drone hovers, and the episode ends when the
    payload LANDS — success/reward from the real landing. Reward = v13 perception
    (undetected penalty + detection gating, frozen post-release) + v16 land-terminal
    landing reward.
    """
    cfg: DroneBombardV19Cfg

    def __init__(self, cfg: DroneBombardV19Cfg, render_mode: str | None = None, **kwargs):
        super().__init__(cfg, render_mode, **kwargs)
        if not hasattr(self, "_perceived_target_xy"):
            self._perceived_target_xy = torch.zeros(self.num_envs, 2, device=self.device)
        # Fix B: consecutive policy steps spent IN the release envelope without
        # dropping. Self-heals to 0 whenever the gate is not open or after release
        # (drone spawns far each episode -> gate closed -> counter zeroed on step 1).
        self._gate_steps = torch.zeros(self.num_envs, device=self.device)

    def _pre_physics_step(self, actions: torch.Tensor):
        super()._pre_physics_step(actions)  # v18->v14: vel + drop_signal + residual capture
        # v16 option A: hover the drone after release while the payload falls.
        if self._released.any():
            held = self._released
            self._vel_cmd[held] = 0.0
            self._wants_drop[held] = False

    def _get_observations(self) -> dict:
        # v18's 28-D builder, then the two-timescale sensing noise. Applied at
        # the OUTERMOST point on purpose: v17/v18 overwrite obs channels 0/1
        # with the perceived target AFTER calling super(), so noise injected
        # deeper in the chain would be partially overwritten.
        out = super()._get_observations()
        return {"policy": self._perturb_obs(out["policy"])}

    def _get_dones(self) -> tuple[torch.Tensor, torch.Tensor]:
        self._step_moving_target()
        cfg = self.cfg
        tc = cfg.termination
        pos, vel, ang, roll, pitch, _ = self._kinematics()
        altitude = pos[:, 2]
        speed = torch.linalg.norm(vel, dim=-1)
        speed_xy = torch.linalg.norm(vel[:, :2], dim=-1)
        ang_norm = torch.linalg.norm(ang, dim=-1)

        _, d_impact, _ = self._ccip(pos, vel)  # v18: perceived (pixel) + residual
        self._d_impact = d_impact
        self._aim_err_min = torch.minimum(self._aim_err_min, d_impact)

        gate = release_gate(
            d_impact, altitude, speed_xy, vel[:, 2], roll, pitch, ang_norm,
            self._payload_attached.float(),
            cfg.release_radius, cfg.release_alt_min, cfg.release_alt_max,
            cfg.release_max_speed, cfg.release_max_vz, cfg.release_max_tilt, cfg.release_max_ang_vel,
        )
        self._gate_open = gate
        fire = self._wants_drop & gate & (~self._released)

        # physical release: detach + fall (NOT terminal — wait for the landing)
        self._just_released = fire.clone()
        self._released = self._released | fire
        self._payload_attached = self._payload_attached & (~fire)
        self._payload_free = self._payload_free | fire

        d_xy = self._current_d_xy()
        self._d_xy_min = torch.minimum(self._d_xy_min, d_xy)
        self._detected = d_xy <= cfg.reveal_radius
        self._overshoot_flythrough = torch.zeros_like(gate)

        landed = self._payload_landed
        impact_err = torch.linalg.norm(self._payload_impact_xy - self._target_xy, dim=-1)
        self._release_impact_err = torch.where(landed, impact_err, self._release_impact_err)

        step = self.episode_length_buf
        crash = (altitude < tc.ground_contact_altitude) | (
            (step > tc.min_altitude_start_step) & (altitude < tc.min_altitude) & (~self._released)
        )
        overspeed = speed > tc.v_max_safety
        inverted = (roll.abs() > tc.limit_inverted_tilt) | (pitch.abs() > tc.limit_inverted_tilt)
        bad_attitude = (ang_norm > tc.limit_ang_vel) | inverted
        out_of_range = d_xy > tc.max_distance
        max_alt = altitude > tc.max_altitude
        failure = (crash | overspeed | bad_attitude | out_of_range | max_alt) & (~self._released)
        success = landed & (impact_err <= cfg.v11_success_radius)
        terminated = landed | failure
        time_out = step >= (self.max_episode_length - 1)

        z = torch.zeros_like(gate)
        self._done_flags = {
            "success": success, "crash": crash & (~self._released),
            "overspeed": overspeed & (~self._released),
            "bad_attitude": bad_attitude & (~self._released),
            "out_of_range": out_of_range & (~self._released),
            "max_altitude": max_alt & (~self._released), "overshoot": z, "stagnation": z,
            "released": self._released.clone(), "landed": landed,
            "release_miss": landed & ~success,
            "timeout": time_out & ~terminated,
            "gate_open": gate, "wants_drop": self._wants_drop,
        }
        return terminated, time_out & ~terminated

    def _get_rewards(self) -> torch.Tensor:
        cfg = self.cfg
        _, _, ang, roll, pitch, _ = self._kinematics()
        f = self._done_flags
        d_impact = self._d_impact
        d_xy = self._current_d_xy()
        det = (d_xy <= cfg.reveal_radius).float()
        flying = (~self._released).float()  # freeze shaping after release (spectate the fall)

        r = -cfg.v11_w_ang_vel * (ang * ang).sum(dim=-1) * flying
        r = r - cfg.v11_w_tilt * (roll * roll + pitch * pitch) * flying
        r = r - cfg.v11_w_action_smooth * (self._delta_action * self._delta_action).sum(dim=-1) * flying
        r = r - cfg.v11_w_time * flying
        r = r + (1.0 - det) * cfg.v13_undetected_penalty * flying

        progress = cfg.v11_w_progress * (self._d_xy_prev - d_xy)
        if cfg.ccip_potential_shaping:
            # Fix A: potential-based aim shaping. Reward the CHANGE in aim quality
            # (telescopes to a fixed total over the episode), so HOLDING aim without
            # dropping earns ~0 — this removes the standing +w_ccip/step that made
            # "hover perfectly aimed forever" out-earn the noisy physical drop.
            ccip = cfg.v11_w_ccip * (torch.exp(-cfg.v11_k_ccip * d_impact)
                                     - torch.exp(-cfg.v11_k_ccip * self._d_impact_prev))
        else:
            ccip = cfg.v11_w_ccip * torch.exp(-cfg.v11_k_ccip * d_impact)
        r = r + det * flying * (progress + ccip)
        r = r + det * flying * f["gate_open"].float() * cfg.v11_gate_reward
        r = r + det * flying * (self._wants_drop & f["gate_open"]).float() * cfg.v11_drop_signal_reward

        # Fix B: escalating loiter penalty. Count consecutive steps spent in the
        # release envelope without dropping and charge w_loiter * that count, so
        # sitting aimed-but-not-releasing costs more every step -> the only way to
        # stop the growing penalty is to actually release.
        in_env = f["gate_open"] & (~self._released)
        self._gate_steps = torch.where(
            in_env, self._gate_steps + 1.0, torch.zeros_like(self._gate_steps))
        r = r - flying * cfg.v19_w_loiter * self._gate_steps

        # terminal reward on the REAL payload landing
        landing_err = self._release_impact_err
        if cfg.precise_landing_reward:
            # Continuous precision reward: exp keeps pulling toward 0 m even INSIDE
            # the success zone (a 0.1 m hit out-earns a 0.9 m hit), + a discrete
            # bonus for crossing the success threshold. Removes the flat-inside cap
            # that plateaued med_err. reward_success is the exp peak at err=0.
            landing_reward = (
                cfg.v11_reward_success * torch.exp(-cfg.v19_k_landing * landing_err)
                + (landing_err <= cfg.v11_success_radius).float() * cfg.v19_success_bonus
            )
        else:
            landing_reward = torch.where(
                landing_err <= cfg.v11_success_radius,
                torch.full_like(landing_err, cfg.v11_reward_success),
                cfg.v11_reward_success * torch.exp(-cfg.v11_k_landing * landing_err),
            )
        r = r + f["landed"].float() * landing_reward

        r = r + f["crash"].float() * cfg.v11_crash_penalty
        r = r + f["out_of_range"].float() * cfg.v11_out_of_range_penalty
        r = r + (f["timeout"] & ~self._released).float() * cfg.v11_no_drop_penalty

        self._d_xy_prev = d_xy
        self._d_impact_prev = d_impact
        return torch.nan_to_num(r, nan=0.0)
