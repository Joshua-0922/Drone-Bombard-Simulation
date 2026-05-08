"""17D 관측 텐서 함수.

모든 함수는 Isaac Lab ManagerBasedRLEnv 를 첫 번째 인자로 받는다.
반환값은 [num_envs, dim] float32 텐서.

인덱스 매핑:
  0-2  : Position ENU / 50
  3-5  : Velocity ENU / 15
  6-8  : Angular velocity / π
  9-11 : u, v, confidence  (use_vision=False → 0, 0, 1)
  12   : payload_attached (bool → float)
  13-14: relative x, y to target / 50
  15   : CCIP d_impact / 50
  16   : CCIP t_f / 10
"""

from __future__ import annotations

import math
import torch
from isaaclab.envs import ManagerBasedRLEnv

# 스케일 상수 (Gazebo 구현과 동일)
_POS_SCALE = 50.0
_VEL_SCALE = 15.0
_ANG_VEL_SCALE = math.pi
_G = 9.81


def _get_drone(env: ManagerBasedRLEnv):
    return env.scene["drone"]


def _get_payload(env: ManagerBasedRLEnv):
    return env.scene["payload"]


# ---------------------------------------------------------------------------
# CCIP 예측기 (벡터화)
# ---------------------------------------------------------------------------

def _ccip_predict(pos: torch.Tensor, vel: torch.Tensor) -> tuple[torch.Tensor, torch.Tensor]:
    """페이로드 투하 지점 예측 (자유낙하 운동학, 벡터화).

    Args:
        pos: [N, 3] 드론 위치 (ENU, m)
        vel: [N, 3] 드론 속도 (ENU, m/s)

    Returns:
        d_impact: [N] 목표점까지 예측 거리 (m)
        t_f:      [N] 낙하 시간 (s, 최대 10 s 클램프)
    """
    z = pos[:, 2].clamp(min=0.0)
    vz = vel[:, 2]

    # 이차 방정식: 0.5*g*t^2 - vz*t - z = 0 → t = (vz + sqrt(vz^2 + 2*g*z)) / g
    discriminant = vz * vz + 2.0 * _G * z
    t_f = (vz + torch.sqrt(discriminant.clamp(min=0.0))) / _G
    t_f = t_f.clamp(min=0.0, max=10.0)

    x_p = pos[:, 0] + vel[:, 0] * t_f
    y_p = pos[:, 1] + vel[:, 1] * t_f

    # target_pos: [N, 2] 텐서, env 에 저장됨
    target = env_target_pos(None)  # placeholder; 실제 호출은 아래 함수에서
    return t_f, x_p, y_p


def _ccip_predict_with_env(
    env: ManagerBasedRLEnv,
    pos: torch.Tensor,
    vel: torch.Tensor,
) -> tuple[torch.Tensor, torch.Tensor]:
    """env.target_pos 를 참조하는 CCIP 예측기."""
    z = pos[:, 2].clamp(min=0.0)
    vz = vel[:, 2]

    discriminant = (vz * vz + 2.0 * _G * z).clamp(min=0.0)
    t_f = (vz + torch.sqrt(discriminant)) / _G
    t_f = t_f.clamp(min=0.0, max=10.0)

    x_p = pos[:, 0] + vel[:, 0] * t_f
    y_p = pos[:, 1] + vel[:, 1] * t_f

    tgt = env.target_pos  # [N, 2]
    dx = x_p - tgt[:, 0]
    dy = y_p - tgt[:, 1]
    d_impact = torch.sqrt(dx * dx + dy * dy)

    return d_impact, t_f


# ---------------------------------------------------------------------------
# 관측 함수
# ---------------------------------------------------------------------------

def position_enu(env: ManagerBasedRLEnv) -> torch.Tensor:
    """드론 위치 ENU, / 50, 클램프 [-1, 1]. Shape: [N, 3]."""
    drone = _get_drone(env)
    pos = drone.data.root_pos_w  # [N, 3]
    return (pos / _POS_SCALE).clamp(-1.0, 1.0)


def velocity_enu(env: ManagerBasedRLEnv) -> torch.Tensor:
    """드론 속도 ENU, / 15, 클램프 [-1, 1]. Shape: [N, 3]."""
    drone = _get_drone(env)
    vel = drone.data.root_lin_vel_w  # [N, 3]
    return (vel / _VEL_SCALE).clamp(-1.0, 1.0)


def angular_velocity(env: ManagerBasedRLEnv) -> torch.Tensor:
    """드론 각속도 body frame, / π, 클램프 [-1, 1]. Shape: [N, 3]."""
    drone = _get_drone(env)
    ang = drone.data.root_ang_vel_w  # [N, 3]
    return (ang / _ANG_VEL_SCALE).clamp(-1.0, 1.0)


def vision_placeholder(env: ManagerBasedRLEnv) -> torch.Tensor:
    """비전 플레이스홀더 (use_vision=False).

    u=0, v=0, confidence=1.0 (타겟 항상 보임 가정).
    커리큘럼 Stage 3 에서 YOLO 탐지값으로 교체.
    Shape: [N, 3]
    """
    n = env.num_envs
    device = env.device
    result = torch.zeros(n, 3, device=device, dtype=torch.float32)
    result[:, 2] = 1.0  # confidence = 1.0
    return result


def payload_attached_flag(env: ManagerBasedRLEnv) -> torch.Tensor:
    """페이로드 부착 여부 (1.0 = 부착, 0.0 = 투하). Shape: [N, 1]."""
    return env.payload_attached.float().unsqueeze(-1)  # [N, 1]


def relative_target_pos(env: ManagerBasedRLEnv) -> torch.Tensor:
    """드론 → 타겟 상대 위치 XY, / 50, 클램프 [-1, 1]. Shape: [N, 2]."""
    drone = _get_drone(env)
    pos = drone.data.root_pos_w[:, :2]  # [N, 2]
    rel = (pos - env.target_pos) / _POS_SCALE  # [N, 2]
    return rel.clamp(-1.0, 1.0)


def ccip_impact_distance(env: ManagerBasedRLEnv) -> torch.Tensor:
    """CCIP 예측 투하 오차 거리 / 50, 클램프 [0, 1]. Shape: [N, 1]."""
    drone = _get_drone(env)
    pos = drone.data.root_pos_w
    vel = drone.data.root_lin_vel_w
    d_impact, _ = _ccip_predict_with_env(env, pos, vel)
    return (d_impact / _POS_SCALE).clamp(0.0, 1.0).unsqueeze(-1)


def ccip_time_of_flight(env: ManagerBasedRLEnv) -> torch.Tensor:
    """CCIP 예측 낙하 시간 / 10, 클램프 [0, 1]. Shape: [N, 1]."""
    drone = _get_drone(env)
    pos = drone.data.root_pos_w
    vel = drone.data.root_lin_vel_w
    _, t_f = _ccip_predict_with_env(env, pos, vel)
    return (t_f / 10.0).clamp(0.0, 1.0).unsqueeze(-1)


# 외부 접근용 헬퍼
def env_target_pos(env):
    """env.target_pos 참조 헬퍼 (타입 힌트용)."""
    return env.target_pos  # [N, 2]
