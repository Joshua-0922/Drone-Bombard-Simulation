"""DroneDropEnv ManagerBasedRLEnvCfg 정의.

설계 결정:
  - sim dt=0.01 s (100 Hz), decimation=10 → 10 Hz 제어 (Gazebo 와 동일)
  - episode_length_s=50.0 → 500 steps
  - num_envs=4096 (기본값; GPU VRAM 에 따라 조절)
  - 페이로드 부착은 환경 서브클래스(DroneBombardEnv)에서 텐서 직접 관리
"""

from __future__ import annotations

import math
from dataclasses import MISSING

import isaaclab.sim as sim_utils
from isaaclab.assets import ArticulationCfg, RigidObjectCfg
from isaaclab.envs import ManagerBasedRLEnvCfg
from isaaclab.managers import (
    EventTermCfg,
    ObservationGroupCfg,
    ObservationTermCfg,
    RewardTermCfg,
    TerminationTermCfg,
)
from isaaclab.scene import InteractiveSceneCfg
from isaaclab.sim import PhysxCfg, SimulationCfg
from isaaclab.terrains import TerrainImporterCfg
from isaaclab.utils import configclass

from ..assets import PAYLOAD_CFG, TARGET_MARKER_CFG, X500_CFG
from ..mdp import actions as mdp_actions
from ..mdp import events as mdp_events
from ..mdp import observations as mdp_obs
from ..mdp import rewards as mdp_rewards
from ..mdp import terminations as mdp_terms


# ---------------------------------------------------------------------------
# Scene
# ---------------------------------------------------------------------------

@configclass
class DroneBombardSceneCfg(InteractiveSceneCfg):
    """드론 + 페이로드 + 타겟 + 지면으로 구성된 씬."""

    ground = TerrainImporterCfg(
        prim_path="/World/ground",
        terrain_type="plane",
        physics_material=sim_utils.RigidBodyMaterialCfg(
            friction_combine_mode="average",
            restitution_combine_mode="average",
            static_friction=1.0,
            dynamic_friction=1.0,
            restitution=0.0,
        ),
    )

    drone: ArticulationCfg = X500_CFG.replace(
        prim_path="{ENV_REGEX_NS}/Drone"
    )

    payload: RigidObjectCfg = PAYLOAD_CFG.replace(
        prim_path="{ENV_REGEX_NS}/Payload"
    )

    target: RigidObjectCfg = TARGET_MARKER_CFG.replace(
        prim_path="{ENV_REGEX_NS}/Target"
    )

    # 씬 내 환경 간격 (num_envs × env_spacing 격자)
    # 4096 envs 에서는 10 m 간격으로 ~640 m × 640 m 그리드
    env_spacing: float = 10.0


# ---------------------------------------------------------------------------
# MDP: Observations
# ---------------------------------------------------------------------------

@configclass
class ObservationsCfg:
    """17D 관측 공간 정의."""

    @configclass
    class PolicyCfg(ObservationGroupCfg):
        """Policy 관측 그룹 (17D)."""

        # 0-2: Position ENU / 50
        pos_enu = ObservationTermCfg(func=mdp_obs.position_enu)
        # 3-5: Velocity ENU / 15
        vel_enu = ObservationTermCfg(func=mdp_obs.velocity_enu)
        # 6-8: Angular velocity / π
        ang_vel = ObservationTermCfg(func=mdp_obs.angular_velocity)
        # 9-11: u, v, confidence (use_vision=False → 0, 0, 1)
        vision = ObservationTermCfg(func=mdp_obs.vision_placeholder)
        # 12: payload_attached flag
        payload_attached = ObservationTermCfg(func=mdp_obs.payload_attached_flag)
        # 13-14: relative x, y to target / 50
        rel_target = ObservationTermCfg(func=mdp_obs.relative_target_pos)
        # 15: CCIP d_impact / 50
        ccip_dist = ObservationTermCfg(func=mdp_obs.ccip_impact_distance)
        # 16: CCIP t_f / 10
        ccip_time = ObservationTermCfg(func=mdp_obs.ccip_time_of_flight)

        enable_corruption: bool = False
        concatenate_terms: bool = True

    policy: PolicyCfg = PolicyCfg()


# ---------------------------------------------------------------------------
# MDP: Actions
# ---------------------------------------------------------------------------

@configclass
class ActionsCfg:
    """5D velocity command → PD 컨트롤러 외력 인가."""

    drone_velocity = mdp_actions.PdVelocityControllerCfg(
        asset_name="drone",
        kp_xy=3.0,
        kd_xy=1.0,
        kp_z=5.0,
        kd_z=2.0,
        scale_vx=15.0,
        scale_vy=5.0,
        scale_vz=3.0,
        scale_yaw=1.0,
    )


# ---------------------------------------------------------------------------
# MDP: Rewards
# ---------------------------------------------------------------------------

@configclass
class RewardsCfg:
    """4-Layer 계층적 보상 함수."""

    # Layer 1 — Safety
    crash_penalty = RewardTermCfg(
        func=mdp_rewards.crash_penalty,
        weight=1.0,
        params={"penalty": -10.0, "min_altitude": 2.0, "start_step": 20},
    )
    overspeed_penalty = RewardTermCfg(
        func=mdp_rewards.overspeed_penalty,
        weight=1.0,
        params={"penalty": -8.0, "v_max": 20.0},
    )

    # Layer 2 — Stability (+ energy penalty)
    stability = RewardTermCfg(
        func=mdp_rewards.stability_reward,
        weight=1.0,
        params={"w_time": 0.01, "w_ang_vel": 0.05, "w_action_smooth": 0.05},
    )
    energy = RewardTermCfg(
        func=mdp_rewards.energy_penalty,
        weight=1.0,
        params={"w_energy": 0.02},
    )

    # Layer 3 — Approach
    approach_distance = RewardTermCfg(
        func=mdp_rewards.approach_distance_reward,
        weight=1.0,
        params={"w_dist": 1.0},
    )
    heading_alignment = RewardTermCfg(
        func=mdp_rewards.heading_alignment_reward,
        weight=1.0,
        params={"w_heading": 1.0, "speed_gate_min": 2.0},
    )

    # Layer 4 — Drop (페이로드 착지 후 계산)
    drop_accuracy = RewardTermCfg(
        func=mdp_rewards.drop_accuracy_reward,
        weight=1.0,
        params={
            "w_drop_base": 50.0,
            "k_precision": 5.0,
            "jackpot_bonus": 100.0,
            "jackpot_threshold": 0.1,
        },
    )
    drop_instability = RewardTermCfg(
        func=mdp_rewards.drop_instability_penalty,
        weight=1.0,
        params={"penalty": -50.0, "limit_ang_vel": 2.0, "limit_tilt": 0.26},
    )

    # 물리 폭발 패널티
    physics_glitch_penalty = RewardTermCfg(
        func=mdp_rewards.physics_glitch_penalty,
        weight=1.0,
        params={"penalty": -100.0, "max_dist": 500.0},
    )

    # 타임아웃 미투하 패널티
    timeout_no_drop = RewardTermCfg(
        func=mdp_rewards.timeout_no_drop_penalty,
        weight=1.0,
        params={"penalty": -50.0},
    )


# ---------------------------------------------------------------------------
# MDP: Terminations
# ---------------------------------------------------------------------------

@configclass
class TerminationsCfg:
    """종료 조건."""

    # 에피소드 타임아웃 (500 steps)
    time_out = TerminationTermCfg(func=mdp_terms.time_out, time_out=True)

    # 페이로드 착지 감지 (z <= 0.04 m && dropped)
    payload_landed = TerminationTermCfg(
        func=mdp_terms.payload_landed,
        params={"land_threshold": 0.04},
    )

    # 물리 폭발 감지 (d_xy > 500 m or !isfinite)
    physics_glitch = TerminationTermCfg(
        func=mdp_terms.physics_glitch,
        params={"max_dist": 500.0},
    )


# ---------------------------------------------------------------------------
# MDP: Events
# ---------------------------------------------------------------------------

@configclass
class EventsCfg:
    """리셋 / 랜덤화 / 커리큘럼 이벤트."""

    # 에피소드 리셋: 드론 + 페이로드 위치/속도 초기화
    reset_drone = EventTermCfg(
        func=mdp_events.reset_drone_pose,
        mode="reset",
        params={"altitude": 5.0},
    )

    # 타겟 위치 커리큘럼 랜덤화
    randomize_target = EventTermCfg(
        func=mdp_events.randomize_target_position,
        mode="reset",
    )

    # 에피소드 버퍼 리셋 (d_xy_prev, action_prev, dropped, payload_attached)
    reset_episode_buffers = EventTermCfg(
        func=mdp_events.reset_episode_buffers,
        mode="reset",
    )

    # 커리큘럼 업데이트 (매 N 에피소드마다)
    curriculum_update = EventTermCfg(
        func=mdp_events.update_curriculum,
        mode="interval",
        interval_range_s=(50.0, 50.0),  # 매 50 s 마다 체크
    )


# ---------------------------------------------------------------------------
# Main Env Config
# ---------------------------------------------------------------------------

@configclass
class DroneBombardEnvCfg(ManagerBasedRLEnvCfg):
    """드론 폭격 RL 환경 설정."""

    # 페이로드 부착: kinematic_sync (기본) | fixed_joint (소수 env smoke)
    payload_attachment_mode: str = "kinematic_sync"
    auto_drop_threshold: float = 0.5

    scene: DroneBombardSceneCfg = DroneBombardSceneCfg(num_envs=4096, env_spacing=10.0)
    observations: ObservationsCfg = ObservationsCfg()
    actions: ActionsCfg = ActionsCfg()
    rewards: RewardsCfg = RewardsCfg()
    terminations: TerminationsCfg = TerminationsCfg()
    events: EventsCfg = EventsCfg()

    def __post_init__(self):
        """시뮬레이션 파라미터 초기화."""
        super().__post_init__()

        # 100 Hz 물리 시뮬레이션
        self.sim.dt = 0.01
        # 10 Hz 정책 제어 (decimation=10 → 100/10)
        self.decimation = 10
        # 에피소드 길이: 50 s = 500 steps @ 10 Hz
        self.episode_length_s = 50.0

        # PhysX 설정
        self.sim.physx = PhysxCfg(
            solver_type=1,  # TGS solver
            max_position_iteration_count=4,
            max_velocity_iteration_count=0,
            enable_ccd=False,
            bounce_threshold_velocity=0.5,
            friction_offset_threshold=0.01,
            friction_correlation_distance=0.025,
        )

        # 관측 공간 정규화 범위
        self.sim.render_interval = self.decimation


@configclass
class DroneBombardEnvCfg_PLAY(DroneBombardEnvCfg):
    """시각화 / 평가용 설정 (소수 환경, 렌더링 활성화)."""

    def __post_init__(self):
        super().__post_init__()
        self.scene.num_envs = 4
        self.scene.env_spacing = 15.0
        self.episode_length_s = 100.0
