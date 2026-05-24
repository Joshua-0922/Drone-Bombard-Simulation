"""페이로드 부착/투하 메커니즘.

Gazebo DetachableJoint 대체. Isaac Lab 배치 환경에서는 런타임 FixedJoint API가
제한적이므로 기본 모드는 ``kinematic_sync`` (매 step pose/velocity 동기화).

``fixed_joint`` 모드는 PhysX FixedJoint 생성을 시도하며, 실패 시 kinematic_sync 로
폴백한다. 상세: ``isaac_lab_tasks/docs/PAYLOAD_ATTACHMENT.md``.
"""

from __future__ import annotations

from typing import Literal

import torch
from isaaclab.envs import ManagerBasedRLEnv

from ..assets.payload import PAYLOAD_OFFSET_Z

AttachmentMode = Literal["kinematic_sync", "fixed_joint"]


def init_payload_attachment(env: ManagerBasedRLEnv, mode: AttachmentMode = "kinematic_sync") -> None:
    """씬 생성 후 부착 모드 초기화. fixed_joint 실패 시 kinematic_sync 로 폴백."""
    env.payload_attachment_mode = mode
    env._payload_joint_created = False

    if mode == "fixed_joint":
        try:
            _try_create_fixed_joints(env)
            env._payload_joint_created = True
        except Exception as exc:
            env.payload_attachment_mode = "kinematic_sync"
            print(
                f"[payload_attachment] FixedJoint 생성 실패 → kinematic_sync 폴백: {exc}"
            )


def _try_create_fixed_joints(env: ManagerBasedRLEnv) -> None:
    """PhysX FixedJoint 생성 시도 (단일/소수 env 권장).

    대규모 배치(num_envs>64)에서는 USD 조인트 생성 비용이 커서 kinematic_sync 권장.
    """
    from pxr import Gf, UsdPhysics

    stage = env.sim.stage
    for env_id in range(env.num_envs):
        env_ns = f"/World/envs/env_{env_id}"
        drone_path = f"{env_ns}/Drone"
        payload_path = f"{env_ns}/Payload"
        joint_path = f"{env_ns}/PayloadFixedJoint"

        if stage.GetPrimAtPath(joint_path).IsValid():
            continue

        joint = UsdPhysics.FixedJoint.Define(stage, joint_path)
        joint.CreateBody0Rel().SetTargets([drone_path])
        joint.CreateBody1Rel().SetTargets([payload_path])
        joint.CreateLocalPos0Attr().Set(Gf.Vec3f(0.0, 0.0, PAYLOAD_OFFSET_Z))
        joint.CreateLocalPos1Attr().Set(Gf.Vec3f(0.0, 0.0, 0.0))


def _set_payload_gravity(env: ManagerBasedRLEnv, env_ids: torch.Tensor, enabled: bool) -> None:
    """부착 중 중력 비활성화로 드론과 함께 이동 (낙하 방지)."""
    if len(env_ids) == 0:
        return
    payload = env.scene["payload"]
    try:
        view = payload.root_physx_view
        if hasattr(view, "set_disable_gravity"):
            disable = not enabled
            view.set_disable_gravity(disable, indices=env_ids.cpu().tolist())
    except Exception:
        pass


def update_payload_attachment(env: ManagerBasedRLEnv) -> None:
    """부착된 환경의 페이로드를 드론에 동기화."""
    if getattr(env, "payload_attachment_mode", "kinematic_sync") == "fixed_joint" and getattr(
        env, "_payload_joint_created", False
    ):
        return

    drone = env.scene["drone"]
    payload = env.scene["payload"]
    attached = env.payload_attached

    if not attached.any():
        return

    drone_pos = drone.data.root_pos_w
    drone_rot = drone.data.root_quat_w

    payload_pos = drone_pos.clone()
    payload_pos[:, 2] += PAYLOAD_OFFSET_Z
    payload_pose = torch.cat([payload_pos, drone_rot], dim=-1)

    drone_vel = drone.data.root_lin_vel_w
    payload_vel_6d = torch.cat([drone_vel, drone.data.root_ang_vel_w], dim=-1)

    attached_ids = attached.nonzero(as_tuple=False).squeeze(-1)
    if len(attached_ids) > 0:
        payload.write_root_pose_to_sim(payload_pose[attached_ids], env_ids=attached_ids)
        payload.write_root_velocity_to_sim(payload_vel_6d[attached_ids], env_ids=attached_ids)
        _set_payload_gravity(env, attached_ids, enabled=False)


def detach_payload(env: ManagerBasedRLEnv, env_ids: torch.Tensor) -> None:
    """투하: 부착 해제 + 중력 복원."""
    if len(env_ids) == 0:
        return
    env.payload_attached[env_ids] = False
    env.dropped[env_ids] = True
    _set_payload_gravity(env, env_ids, enabled=True)

    if getattr(env, "payload_attachment_mode", "") == "fixed_joint" and getattr(
        env, "_payload_joint_created", False
    ):
        _break_fixed_joints(env, env_ids)


def _break_fixed_joints(env: ManagerBasedRLEnv, env_ids: torch.Tensor) -> None:
    """FixedJoint prim 비활성화 (배치 환경)."""
    try:
        from pxr import Usd

        stage = env.sim.stage
        for idx in env_ids.tolist():
            joint_path = f"/World/envs/env_{idx}/PayloadFixedJoint"
            prim = stage.GetPrimAtPath(joint_path)
            if prim.IsValid():
                prim.SetActive(False)
    except Exception:
        pass


def reattach_payload_on_reset(env: ManagerBasedRLEnv, env_ids: torch.Tensor) -> None:
    """에피소드 리셋 시 조인트/중력 상태 복원."""
    if len(env_ids) == 0:
        return
    env.payload_attached[env_ids] = True
    env.dropped[env_ids] = False
    _set_payload_gravity(env, env_ids, enabled=False)

    if getattr(env, "payload_attachment_mode", "") == "fixed_joint" and getattr(
        env, "_payload_joint_created", False
    ):
        try:
            from pxr import Usd

            stage = env.sim.stage
            for idx in env_ids.tolist():
                joint_path = f"/World/envs/env_{idx}/PayloadFixedJoint"
                prim = stage.GetPrimAtPath(joint_path)
                if prim.IsValid():
                    prim.SetActive(True)
        except Exception:
            pass
