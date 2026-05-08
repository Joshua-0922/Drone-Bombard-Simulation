"""목표 지점 X-마커 RigidObjectCfg 정의.

크기: 1.5 m × 1.5 m 십자 마커 (지면 평면 위 시각적 표시물)
물리: 키네마틱(정적) — 충돌 없음, 순수 시각 기준점
위치: events.py 커리큘럼에 의해 에피소드마다 랜덤화.
"""

from __future__ import annotations

import isaaclab.sim as sim_utils
from isaaclab.assets import RigidObjectCfg

TARGET_MARKER_CFG = RigidObjectCfg(
    prim_path="{ENV_REGEX_NS}/Target",
    spawn=sim_utils.CuboidCfg(
        # 얇은 평판으로 X-마커를 표현 (실제 X 형상은 USD 에셋으로 대체 가능)
        size=(1.5, 1.5, 0.01),
        rigid_props=sim_utils.RigidBodyPropertiesCfg(
            disable_gravity=True,
            kinematic_enabled=True,  # 정적 객체 — 물리 해결자 제외
        ),
        mass_props=sim_utils.MassPropertiesCfg(mass=0.0),
        collision_props=sim_utils.CollisionPropertiesCfg(
            collision_enabled=False,
        ),
        visual_material=sim_utils.PreviewSurfaceCfg(
            diffuse_color=(1.0, 0.1, 0.1),  # 빨간 X
        ),
    ),
    init_state=RigidObjectCfg.InitialStateCfg(
        # 초기 위치는 events.py 에서 커리큘럼 단계에 따라 오버라이드됨
        pos=(11.0, 10.0, 0.005),
        rot=(1.0, 0.0, 0.0, 0.0),
    ),
)
"""목표 지점 마커 설정. 환경 Scene 에 'target' 키로 등록한다."""
