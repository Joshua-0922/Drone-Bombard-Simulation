"""PD Velocity Controller 액션 텀.

5D 액션 → velocity command 디코딩 후 PD 컨트롤러로 외력 인가.

액션 스케일:
  action[0] * 15 = vx 목표속도 (m/s)
  action[1] *  5 = vy 목표속도 (m/s)
  action[2] *  3 = vz 목표속도 (m/s)
  action[3] *  1 = yaw_rate 목표각속도 (rad/s)
  action[4]      = 수동 투하 트리거 (>0 → 투하; CCIP 자동투하가 우선)

PD 제어:
  F = Kp*(v_des - v_cur) + Kd*a_cur  →  set_external_force_and_torque()
  호버링 오프셋: F_hover = m * g = 2.0 * 9.81 = 19.62 N (Z 축)
"""

from __future__ import annotations

from dataclasses import dataclass

import torch
from isaaclab.envs import ManagerBasedRLEnv
from isaaclab.managers import ActionTerm, ActionTermCfg
from isaaclab.utils import configclass

_G = 9.81
_DRONE_MASS = 2.0   # kg — X500 기준; ArticulationCfg 의 mass 와 일치해야 함
_F_HOVER = _DRONE_MASS * _G  # 19.62 N
_MAX_TOTAL_THRUST = 34.0    # N — 4 motors × 8.5 N (x500 nominal)
_MAX_FORCE_XY = 25.0        # N — horizontal PD force clip
_MAX_FORCE_Z_DELTA = 15.0   # N — vertical PD delta clip (around hover)


@configclass
class PdVelocityControllerCfg(ActionTermCfg):
    """PD velocity controller 설정."""

    class_type: type = None  # 아래 PdVelocityController 를 동적 할당

    asset_name: str = "drone"

    # PD 게인
    kp_xy: float = 3.0
    kd_xy: float = 1.0
    kp_z: float = 5.0
    kd_z: float = 2.0

    # 액션 스케일
    scale_vx: float = 15.0
    scale_vy: float = 5.0
    scale_vz: float = 3.0
    scale_yaw: float = 1.0


class PdVelocityController(ActionTerm):
    """5D velocity command → PD 외력 기반 드론 제어."""

    cfg: PdVelocityControllerCfg

    def __init__(self, cfg: PdVelocityControllerCfg, env: ManagerBasedRLEnv):
        super().__init__(cfg, env)
        self._drone = env.scene[cfg.asset_name]
        self._prev_vel = torch.zeros(env.num_envs, 3, device=env.device)
        self._raw_actions = torch.zeros(env.num_envs, 5, device=env.device)
        self._processed_actions = torch.zeros(env.num_envs, 4, device=env.device)

        # 드론 루트 body index
        self._body_idx, _ = self._drone.find_bodies(".*")  # 모든 body; 루트 = 0

    # ------------------------------------------------------------------
    # ActionTerm 인터페이스
    # ------------------------------------------------------------------

    @property
    def action_dim(self) -> int:
        return 5

    @property
    def raw_actions(self) -> torch.Tensor:
        return self._raw_actions

    @property
    def processed_actions(self) -> torch.Tensor:
        return self._processed_actions

    def process_actions(self, actions: torch.Tensor):
        """raw actions 저장 및 velocity command 디코딩."""
        self._raw_actions[:] = actions.clamp(-1.0, 1.0)

        cfg = self.cfg
        vx = self._raw_actions[:, 0] * cfg.scale_vx
        vy = self._raw_actions[:, 1] * cfg.scale_vy
        vz = self._raw_actions[:, 2] * cfg.scale_vz
        yaw = self._raw_actions[:, 3] * cfg.scale_yaw

        self._processed_actions[:, 0] = vx
        self._processed_actions[:, 1] = vy
        self._processed_actions[:, 2] = vz
        self._processed_actions[:, 3] = yaw

    def apply_actions(self):
        """PD 제어: 목표속도 vs 현재속도 → 외력 계산 후 PhysX 에 인가."""
        vel_cur = self._drone.data.root_lin_vel_w   # [N, 3]
        ang_cur = self._drone.data.root_ang_vel_w   # [N, 3]

        v_des = self._processed_actions[:, :3]      # [N, 3]
        yaw_des = self._processed_actions[:, 3]     # [N]

        cfg = self.cfg
        dt = self._env.physics_dt

        # 속도 오차
        vel_err = v_des - vel_cur  # [N, 3]

        # 가속도 추정 (후진 차분)
        acc_cur = (vel_cur - self._prev_vel) / dt
        self._prev_vel[:] = vel_cur

        # PD 제어력 계산 (클리핑 — x500 추력 한계)
        f_xy = cfg.kp_xy * vel_err[:, :2] + cfg.kd_xy * acc_cur[:, :2]
        f_xy = torch.clamp(f_xy, -_MAX_FORCE_XY, _MAX_FORCE_XY)
        f_z = cfg.kp_z * vel_err[:, 2] + cfg.kd_z * acc_cur[:, 2]
        f_z = torch.clamp(f_z, -_MAX_FORCE_Z_DELTA, _MAX_FORCE_Z_DELTA)

        # Z 축: 호버링 피드포워드 + PD
        forces = torch.zeros(self._env.num_envs, len(self._body_idx), 3, device=self._env.device)
        forces[:, 0, 0] = f_xy[:, 0]
        forces[:, 0, 1] = f_xy[:, 1]
        forces[:, 0, 2] = (_F_HOVER + f_z).clamp(0.0, _MAX_TOTAL_THRUST)

        # Yaw 토크 (Z 축 회전)
        yaw_err = yaw_des - ang_cur[:, 2]
        torques = torch.zeros_like(forces)
        torques[:, 0, 2] = 1.5 * yaw_err  # 단순 P 제어

        self._drone.set_external_force_and_torque(
            forces=forces,
            torques=torques,
            body_ids=self._body_idx,
        )

    def reset(self, env_ids: torch.Tensor | None = None):
        """에피소드 리셋 시 속도 메모리 초기화."""
        if env_ids is None:
            self._prev_vel.zero_()
        else:
            self._prev_vel[env_ids] = 0.0


# 설정 클래스에 class_type 동적 연결
PdVelocityControllerCfg.class_type = PdVelocityController
