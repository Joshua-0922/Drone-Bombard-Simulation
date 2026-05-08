"""리셋 / 랜덤화 / 커리큘럼 이벤트 함수.

커리큘럼 4단계:
  Stage 0: 드론-타겟 초기 거리  3 -  8 m  (근거리 정밀 타격)
  Stage 1: 드론-타겟 초기 거리  8 - 20 m  (중거리)
  Stage 2: 드론-타겟 초기 거리 20 - 50 m  (장거리)
  Stage 3: 드론-타겟 초기 거리 20 - 50 m + vision masking (미래 구현)

진급 조건: 최근 N 에피소드 평균 성공률 ≥ threshold.
"""

from __future__ import annotations

import torch
from isaaclab.envs import ManagerBasedRLEnv

from ..assets.payload import PAYLOAD_OFFSET_Z

# 커리큘럼 거리 범위 [min, max] (m)
CURRICULUM_STAGES = [
    (3.0, 8.0),    # Stage 0
    (8.0, 20.0),   # Stage 1
    (20.0, 50.0),  # Stage 2
    (20.0, 50.0),  # Stage 3 (vision masking — 향후 구현)
]

# 커리큘럼 진급 임계값 (최근 N 에피소드 성공률)
CURRICULUM_SUCCESS_THRESHOLD = 0.6
CURRICULUM_EVAL_WINDOW = 100   # 평가 에피소드 수


# ---------------------------------------------------------------------------
# 리셋 이벤트
# ---------------------------------------------------------------------------

def reset_drone_pose(
    env: ManagerBasedRLEnv,
    env_ids: torch.Tensor,
    altitude: float = 5.0,
) -> None:
    """드론을 초기 고도로 텔레포트 (속도 0, 자세 수평).

    페이로드도 드론 마운트 포인트 오프셋으로 함께 초기화.
    """
    drone = env.scene["drone"]
    payload = env.scene["payload"]
    num_reset = len(env_ids)

    # 드론 초기 포즈: 각 env 의 로컬 원점 + altitude
    drone_pos = torch.zeros(num_reset, 3, device=env.device)
    drone_pos[:, 2] = altitude
    drone_rot = torch.zeros(num_reset, 4, device=env.device)
    drone_rot[:, 0] = 1.0  # quaternion w=1 (수평)

    drone_pose = torch.cat([drone_pos, drone_rot], dim=-1)  # [N, 7]
    drone_vel = torch.zeros(num_reset, 6, device=env.device)

    drone.write_root_pose_to_sim(drone_pose, env_ids=env_ids)
    drone.write_root_velocity_to_sim(drone_vel, env_ids=env_ids)

    # 페이로드: 드론 하단 마운트 오프셋
    payload_pos = drone_pos.clone()
    payload_pos[:, 2] += PAYLOAD_OFFSET_Z
    payload_pose = torch.cat([payload_pos, drone_rot], dim=-1)
    payload_vel = torch.zeros(num_reset, 6, device=env.device)

    payload.write_root_pose_to_sim(payload_pose, env_ids=env_ids)
    payload.write_root_velocity_to_sim(payload_vel, env_ids=env_ids)


def reset_episode_buffers(
    env: ManagerBasedRLEnv,
    env_ids: torch.Tensor,
) -> None:
    """에피소드별 커스텀 버퍼 초기화.

    DroneBombardEnv 에 정의된 버퍼:
      - payload_attached [N] bool  → True
      - dropped          [N] bool  → False
      - d_xy_prev        [N] float → 초기 d_xy
      - action_prev      [N, 5]   → 0
      - action_cur       [N, 5]   → 0
    """
    env.payload_attached[env_ids] = True
    env.dropped[env_ids] = False
    env.action_prev[env_ids] = 0.0
    env.action_cur[env_ids] = 0.0

    # d_xy_prev: 초기 드론 위치에서 타겟까지 거리
    pos = env.scene["drone"].data.root_pos_w[env_ids, :2]
    diff = pos - env.target_pos[env_ids]
    env.d_xy_prev[env_ids] = torch.norm(diff, dim=-1)


def randomize_target_position(
    env: ManagerBasedRLEnv,
    env_ids: torch.Tensor,
) -> None:
    """커리큘럼 단계에 따라 타겟 위치 랜덤화.

    타겟은 드론 초기 위치(env 로컬 원점) 기준으로
    커리큘럼 단계 거리 범위 내에서 랜덤 방향으로 배치.
    """
    num_reset = len(env_ids)
    stages = env.curriculum_stage[env_ids]  # [N] int

    # 각 환경의 커리큘럼 단계에 맞는 거리 범위 샘플링
    dist_min = torch.zeros(num_reset, device=env.device)
    dist_max = torch.zeros(num_reset, device=env.device)

    for stage_idx, (d_min, d_max) in enumerate(CURRICULUM_STAGES):
        mask = stages == stage_idx
        dist_min[mask] = d_min
        dist_max[mask] = d_max

    # 균등 분포에서 거리와 방향 샘플링
    dist = dist_min + torch.rand(num_reset, device=env.device) * (dist_max - dist_min)
    angle = torch.rand(num_reset, device=env.device) * 2.0 * 3.141592653589793

    target_x = dist * torch.cos(angle)
    target_y = dist * torch.sin(angle)

    # env.target_pos 업데이트 (env 로컬 좌표)
    env.target_pos[env_ids, 0] = target_x
    env.target_pos[env_ids, 1] = target_y

    # 타겟 마커 시각 위치도 업데이트
    target_obj = env.scene["target"]
    new_pose = target_obj.data.root_pose_w[env_ids].clone()
    # world 좌표로 변환: env origin + local target pos
    env_origins = env.scene.env_origins[env_ids]  # [N, 3]
    new_pose[:, 0] = env_origins[:, 0] + target_x
    new_pose[:, 1] = env_origins[:, 1] + target_y
    new_pose[:, 2] = 0.005
    target_obj.write_root_pose_to_sim(new_pose, env_ids=env_ids)


# ---------------------------------------------------------------------------
# 페이로드 부착/투하 로직 (매 step 호출, DroneBombardEnv._pre_physics_step)
# ---------------------------------------------------------------------------

def update_payload_attachment(env: ManagerBasedRLEnv) -> None:
    """페이로드 부착 상태 유지: 매 physics step 마다 드론 위치에 동기화.

    attached=True 인 환경의 페이로드를 드론 하단에 텔레포트.
    attached=False 인 환경은 자유 낙하 (PhysX 물리 자연 적용).

    DroneBombardEnv._pre_physics_step 에서 직접 호출한다.
    """
    drone = env.scene["drone"]
    payload = env.scene["payload"]
    attached = env.payload_attached  # [N] bool

    if not attached.any():
        return

    # 부착된 환경의 드론 위치에 오프셋 적용
    drone_pos = drone.data.root_pos_w  # [N, 3]
    drone_rot = drone.data.root_quat_w  # [N, 4] w,x,y,z

    payload_pos = drone_pos.clone()
    payload_pos[:, 2] += PAYLOAD_OFFSET_Z

    payload_pose = torch.cat([payload_pos, drone_rot], dim=-1)  # [N, 7]
    drone_vel = drone.data.root_lin_vel_w  # [N, 3]
    payload_vel_6d = torch.cat([drone_vel, drone.data.root_ang_vel_w], dim=-1)

    # 부착된 환경만 업데이트
    attached_ids = attached.nonzero(as_tuple=False).squeeze(-1)
    if len(attached_ids) > 0:
        payload.write_root_pose_to_sim(payload_pose[attached_ids], env_ids=attached_ids)
        payload.write_root_velocity_to_sim(payload_vel_6d[attached_ids], env_ids=attached_ids)


def check_ccip_auto_drop(env: ManagerBasedRLEnv, auto_drop_threshold: float = 0.5) -> None:
    """CCIP 예측 거리 ≤ threshold 인 환경에서 자동 투하 트리거.

    DroneBombardEnv._pre_physics_step 에서 호출.
    """
    from ..mdp.observations import _ccip_predict_with_env

    drone = env.scene["drone"]
    pos = drone.data.root_pos_w
    vel = drone.data.root_lin_vel_w

    d_impact, _ = _ccip_predict_with_env(env, pos, vel)

    # 투하 조건: CCIP 거리 ≤ threshold && 아직 부착 상태
    should_drop = (d_impact <= auto_drop_threshold) & env.payload_attached

    if should_drop.any():
        env.payload_attached[should_drop] = False
        env.dropped[should_drop] = True


# ---------------------------------------------------------------------------
# 커리큘럼 업데이트 (interval 이벤트)
# ---------------------------------------------------------------------------

def update_curriculum(
    env: ManagerBasedRLEnv,
    env_ids: torch.Tensor,
) -> None:
    """성공률 기반 커리큘럼 단계 업데이트.

    env.curriculum_success_window 에 쌓인 성공/실패 기록을 바탕으로
    각 환경의 stage 를 진급시킨다.
    """
    if not hasattr(env, "curriculum_success_window"):
        return

    for idx in env_ids.tolist():
        window = env.curriculum_success_window[idx]
        if len(window) >= CURRICULUM_EVAL_WINDOW:
            success_rate = sum(window) / len(window)
            if success_rate >= CURRICULUM_SUCCESS_THRESHOLD:
                current = env.curriculum_stage[idx].item()
                if current < len(CURRICULUM_STAGES) - 1:
                    env.curriculum_stage[idx] = current + 1
                    window.clear()
