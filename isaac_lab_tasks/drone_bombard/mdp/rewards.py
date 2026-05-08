"""4-Layer 계층적 보상 함수.

Layer 1 — Safety:     crash(-10), overspeed(-8)
Layer 2 — Stability:  -0.05 - 0.05*||ω||² - 0.05*||Δa||²
Layer 3 — Approach:   1.0*(d_prev - d_xy) + 1.0*cos(heading)*speed_gate
Layer 4 — Drop:       50*exp(-5*d_error) + 100 jackpot(d<=0.1m)

각 함수는 [N] float32 텐서를 반환한다.
"""

from __future__ import annotations

import math
import torch
from isaaclab.envs import ManagerBasedRLEnv

_G = 9.81


def _get_drone(env: ManagerBasedRLEnv):
    return env.scene["drone"]


def _compute_d_xy(env: ManagerBasedRLEnv) -> torch.Tensor:
    """드론 XY 위치 → 타겟까지 수평 거리 [N]."""
    pos = _get_drone(env).data.root_pos_w[:, :2]
    diff = pos - env.target_pos
    return torch.norm(diff, dim=-1)


# ---------------------------------------------------------------------------
# Layer 1 — Safety
# ---------------------------------------------------------------------------

def crash_penalty(
    env: ManagerBasedRLEnv,
    penalty: float = -10.0,
    min_altitude: float = 2.0,
    start_step: int = 20,
) -> torch.Tensor:
    """고도 < min_altitude 일 때 패널티 (초기 start_step 스텝은 면제)."""
    alt = _get_drone(env).data.root_pos_w[:, 2]
    is_crash = (alt < min_altitude) & (env.episode_length_buf > start_step)
    return is_crash.float() * penalty


def overspeed_penalty(
    env: ManagerBasedRLEnv,
    penalty: float = -8.0,
    v_max: float = 20.0,
) -> torch.Tensor:
    """속도 > v_max 일 때 패널티."""
    vel = _get_drone(env).data.root_lin_vel_w
    speed = torch.norm(vel, dim=-1)
    return (speed > v_max).float() * penalty


# ---------------------------------------------------------------------------
# Layer 2 — Stability
# ---------------------------------------------------------------------------

def stability_reward(
    env: ManagerBasedRLEnv,
    w_time: float = 0.05,
    w_ang_vel: float = 0.05,
    w_action_smooth: float = 0.05,
) -> torch.Tensor:
    """시간 패널티 + 각속도 패널티 + 액션 스무스니스 패널티.

    R2 = -w_time - w_ang_vel * ||ω||² - w_action_smooth * ||Δa||²
    """
    ang = _get_drone(env).data.root_ang_vel_w  # [N, 3]
    omega_sq = (ang * ang).sum(dim=-1)          # [N]

    # action_prev 는 DroneBombardEnv 가 관리하는 [N, 5] 버퍼
    # 현재 액션은 ActionTerm.process_actions 에서 env.action_prev 에 업데이트됨
    # 여기서는 이전 스텝 대비 변화량 계산
    cur_action = env.action_cur   # [N, 5] — apply_actions 이후 갱신됨
    delta_a = cur_action - env.action_prev  # [N, 5]
    delta_sq = (delta_a * delta_a).sum(dim=-1)  # [N]

    return -w_time - w_ang_vel * omega_sq - w_action_smooth * delta_sq


# ---------------------------------------------------------------------------
# Layer 3 — Approach
# ---------------------------------------------------------------------------

def approach_distance_reward(
    env: ManagerBasedRLEnv,
    w_dist: float = 1.0,
) -> torch.Tensor:
    """2D 거리 감소 보상: w_dist * (d_prev - d_now).

    d_xy_prev 는 DroneBombardEnv 에서 매 step 업데이트.
    """
    d_xy = _compute_d_xy(env)
    reward = w_dist * (env.d_xy_prev - d_xy)
    env.d_xy_prev[:] = d_xy  # 다음 스텝을 위해 업데이트
    return reward


def heading_alignment_reward(
    env: ManagerBasedRLEnv,
    w_heading: float = 1.0,
    speed_gate_min: float = 2.0,
) -> torch.Tensor:
    """헤딩 정렬 보상: cos(드론 속도 방향 ↔ 타겟 방향) × speed_gate.

    speed_gate = min(speed_xy / speed_gate_min, 1.0)
    → 정지 상태에서 보상 파밍 방지 (anti-milking).
    """
    drone = _get_drone(env)
    vel = drone.data.root_lin_vel_w     # [N, 3]
    pos = drone.data.root_pos_w[:, :2]  # [N, 2]

    vx = vel[:, 0]
    vy = vel[:, 1]
    speed_xy = torch.sqrt(vx * vx + vy * vy).clamp(min=1e-6)

    dx = env.target_pos[:, 0] - pos[:, 0]
    dy = env.target_pos[:, 1] - pos[:, 1]
    dist_to_tgt = torch.sqrt(dx * dx + dy * dy).clamp(min=1e-6)

    cos_heading = (vx * dx + vy * dy) / (speed_xy * dist_to_tgt)
    cos_heading = cos_heading.clamp(-1.0, 1.0)

    speed_gate = (speed_xy / speed_gate_min).clamp(0.0, 1.0)

    # 속도가 낮을 때 ( < 0.1 m/s) 헤딩 신호 무효
    valid = speed_xy > 0.1
    return w_heading * cos_heading * speed_gate * valid.float()


# ---------------------------------------------------------------------------
# Layer 4 — Drop
# ---------------------------------------------------------------------------

def drop_accuracy_reward(
    env: ManagerBasedRLEnv,
    w_drop_base: float = 50.0,
    k_precision: float = 5.0,
    jackpot_bonus: float = 100.0,
    jackpot_threshold: float = 0.1,
) -> torch.Tensor:
    """페이로드 착지 miss distance 기반 보상.

    payload_landed (termination) 이 발생한 step 에서만 계산.
    d_error = 착지 위치와 타겟 사이 수평 거리 (m).
    """
    # 이번 step 에 착지한 환경만 보상
    payload = env.scene["payload"]
    payload_pos = payload.data.root_pos_w[:, :2]  # [N, 2]
    diff = payload_pos - env.target_pos
    d_error = torch.norm(diff, dim=-1)  # [N]

    # 착지 발생 & dropped 상태인 환경만 활성화
    landed = env.dropped & (payload.data.root_pos_w[:, 2] <= 0.04)

    base_reward = w_drop_base * torch.exp(-k_precision * d_error)
    jackpot = (d_error <= jackpot_threshold).float() * jackpot_bonus

    return landed.float() * (base_reward + jackpot)


def timeout_no_drop_penalty(
    env: ManagerBasedRLEnv,
    penalty: float = -50.0,
) -> torch.Tensor:
    """에피소드 타임아웃 시 미투하 패널티."""
    timed_out = env.episode_length_buf >= env.max_episode_length
    not_dropped = ~env.dropped
    return (timed_out & not_dropped).float() * penalty
