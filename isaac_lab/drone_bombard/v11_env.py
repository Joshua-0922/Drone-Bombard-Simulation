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

        marker = (self._cruise_unit * self.cfg.marker_dist).unsqueeze(0).expand(n, 2)
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
        d0 = torch.full((n,), float(self.cfg.marker_dist), device=device)  # spawn is marker_dist from marker
        self._d_xy_prev[env_ids] = d0
        self._d_impact_prev[env_ids] = d0

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
