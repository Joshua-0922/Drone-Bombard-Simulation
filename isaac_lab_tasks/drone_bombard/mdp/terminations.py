"""종료 조건 함수.

모든 함수는 [N] bool 텐서를 반환한다.

종료 조건:
  - time_out        : 500 steps 초과
  - payload_landed  : payload z <= 0.04 m && dropped
  - physics_glitch  : d_xy > 500 m 또는 nan/inf
"""

from __future__ import annotations

import torch
from isaaclab.envs import ManagerBasedRLEnv
from isaaclab.managers import TerminationTermCfg


def time_out(env: ManagerBasedRLEnv) -> torch.Tensor:
    """에피소드 최대 길이 초과 → 잘림(truncation).

    ManagerBasedRLEnvCfg 의 time_out=True 로 설정하면
    terminated=False, truncated=True 로 처리됨.
    """
    return env.episode_length_buf >= env.max_episode_length


def payload_landed(
    env: ManagerBasedRLEnv,
    land_threshold: float = 0.04,
) -> torch.Tensor:
    """페이로드가 지면에 착지 (z <= land_threshold m) 이고 투하된 상태."""
    payload = env.scene["payload"]
    payload_z = payload.data.root_pos_w[:, 2]
    return env.dropped & (payload_z <= land_threshold)


def physics_glitch(
    env: ManagerBasedRLEnv,
    max_dist: float = 500.0,
) -> torch.Tensor:
    """물리 폭발 감지: 드론-타겟 거리 > max_dist 또는 nan/inf.

    Gazebo ODE 폭발과 동일한 안전장치.
    nan/inf 는 PhysX TGS solver 에서 드물지만 극단적 force 인가 시 발생 가능.
    """
    pos = env.scene["drone"].data.root_pos_w[:, :2]
    diff = pos - env.target_pos
    d_xy = torch.norm(diff, dim=-1)

    is_glitch = ~torch.isfinite(d_xy) | (d_xy > max_dist)
    return is_glitch
