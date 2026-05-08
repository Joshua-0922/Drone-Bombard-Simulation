"""페이로드 RigidObjectCfg 정의.

형상: 납작한 원통 (r=0.03 m, h=0.005 m) — 폭탄 모형
질량: 0.1 kg
초기 위치: 드론 하단 마운트 포인트 (z_drone - 0.15 m)
드롭 전 부착 상태는 매 step 위치를 드론에 동기화하는 방식으로 구현.
"""

from __future__ import annotations

import isaaclab.sim as sim_utils
from isaaclab.assets import RigidObjectCfg

PAYLOAD_CFG = RigidObjectCfg(
    prim_path="{ENV_REGEX_NS}/Payload",
    spawn=sim_utils.CylinderCfg(
        radius=0.03,
        height=0.005,
        rigid_props=sim_utils.RigidBodyPropertiesCfg(
            disable_gravity=False,
            linear_damping=0.01,
            angular_damping=0.01,
            max_depenetration_velocity=5.0,
        ),
        mass_props=sim_utils.MassPropertiesCfg(mass=0.1),
        collision_props=sim_utils.CollisionPropertiesCfg(
            collision_enabled=True,
        ),
        visual_material=sim_utils.PreviewSurfaceCfg(
            diffuse_color=(0.8, 0.2, 0.1),  # 붉은색
        ),
    ),
    init_state=RigidObjectCfg.InitialStateCfg(
        # 드론 초기 위치(5 m) 기준 하단 마운트 오프셋 -0.15 m
        pos=(0.0, 0.0, 4.85),
        rot=(1.0, 0.0, 0.0, 0.0),
        lin_vel=(0.0, 0.0, 0.0),
        ang_vel=(0.0, 0.0, 0.0),
    ),
)
"""페이로드(투하물) 설정. 환경 Scene 에 'payload' 키로 등록한다."""

# 드론 → 페이로드 오프셋 (body frame, ENU)
PAYLOAD_OFFSET_Z = -0.15  # 드론 COM 에서 페이로드 부착점까지 아래 방향 거리 [m]
