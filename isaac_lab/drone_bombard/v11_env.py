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
from isaaclab.utils.math import euler_xyz_from_quat

from .drone_bombard_env import DroneBombardEnv, DroneBombardEnvCfg
from .math_utils import (
    predict_impact_nominal,
    time_to_fall,
    ballistic_impact,
    release_gate,
)


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

        # Marker center: marker_dist ahead along the cruise direction.
        center = (self._cruise_unit * self.cfg.marker_dist).unsqueeze(0).expand(n, 2)
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

        root = self._robot.data.default_root_state[env_ids].clone()
        root[:, 0:2] = self.scene.env_origins[env_ids, :2]  # spawn at env origin (local 0,0)
        root[:, 2] = self.cfg.v11_spawn_alt
        half = self._cruise_yaw / 2.0
        root[:, 3] = math.cos(half)
        root[:, 4] = 0.0
        root[:, 5] = 0.0
        root[:, 6] = math.sin(half)
        root[:, 7:9] = (self._cruise_unit * self.cfg.cruise_speed).unsqueeze(0).expand(n, 2)
        root[:, 9] = 0.0
        root[:, 10:13] = 0.0
        self._robot.write_root_pose_to_sim(root[:, :7], env_ids)
        self._robot.write_root_velocity_to_sim(root[:, 7:13], env_ids)

        self._wants_drop[env_ids] = False
        # spawn is at local (0,0), so distance to marker = |marker| (per env).
        d0 = torch.linalg.norm(marker, dim=-1)
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
        cruise_vx = self._cruise_unit[0] * self.cfg.cruise_speed
        cruise_vy = self._cruise_unit[1] * self.cfg.cruise_speed
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
