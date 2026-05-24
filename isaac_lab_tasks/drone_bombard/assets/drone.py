"""x500 드론 ArticulationCfg 정의.

물리 파라미터:
  - mass: 2.0 kg (x500 기체 + 배터리)
  - arm length: 0.25 m
  - max motor thrust per motor: ~8.5 N (4 motors → max 34 N, hover at ~50%)
  - initial altitude: 5.0 m (TAKEOFF FSM 단계 제거)

USD 소스: Isaac Sim /Isaac/Robots/Crazyflie/cf2x.usd (Path A placeholder).
Path B: SDF→URDF→USD — see isaac_lab_tasks/docs/ASSETS.md

제어: 프로펠러 회전 없음. PD 외력만 (mdp/actions.py, max ~34 N total).
"""

from __future__ import annotations

import isaaclab.sim as sim_utils
from isaaclab.assets import ArticulationCfg

X500_CFG = ArticulationCfg(
    prim_path="{ENV_REGEX_NS}/Drone",
    spawn=sim_utils.UsdFileCfg(
        # Crazyflie (cf2x) 를 베이스 USD 로 사용.
        # 실제 x500 USD 가 있으면 아래 경로를 교체한다.
        usd_path="/Isaac/Robots/Crazyflie/cf2x.usd",
        rigid_props=sim_utils.RigidBodyPropertiesCfg(
            disable_gravity=False,
            retain_accelerations=False,
            linear_damping=0.0,
            angular_damping=0.0,
            max_linear_velocity=100.0,
            max_angular_velocity=100.0,
            max_depenetration_velocity=5.0,
        ),
        mass_props=sim_utils.MassPropertiesCfg(
            # x500 기준 총 중량 2.0 kg
            mass=2.0,
        ),
        articulation_props=sim_utils.ArticulationRootPropertiesCfg(
            enabled_self_collisions=False,
            solver_position_iteration_count=4,
            solver_velocity_iteration_count=0,
            sleep_threshold=0.005,
            stabilization_threshold=0.001,
        ),
    ),
    init_state=ArticulationCfg.InitialStateCfg(
        # 5 m 고도에서 시작 → TAKEOFF 단계 스킵
        pos=(0.0, 0.0, 5.0),
        rot=(1.0, 0.0, 0.0, 0.0),  # w, x, y, z
        lin_vel=(0.0, 0.0, 0.0),
        ang_vel=(0.0, 0.0, 0.0),
    ),
    actuators={},  # 모터 시뮬레이션 없음 — PD 컨트롤러가 외력 직접 인가
)
"""x500 드론 설정. 환경 Scene 에 'drone' 키로 등록한다."""
