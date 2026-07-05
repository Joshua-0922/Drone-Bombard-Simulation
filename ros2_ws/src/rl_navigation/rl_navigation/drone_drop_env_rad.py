"""RAD v1 — Gymnasium environment for drone Approach + Drop RL training.

RAD = Relative + Approach (Phase 1) + Drop (Phase 2). v8/v9a 와 완전 다른 framework.
참조: local/design/rad_v1_design.md (single source of truth).

Phase 1 (Approach): sphere 진입 (d² ≤ 20.5) + 7 final state 조건 → terminal +20~+120
Phase 2 (Drop):     Phase 1 종단 → auto drop (d_impact ≤ 1m) + 정밀 landing
obs 14d:            yaw-only body frame 상대좌표
"""

import math
import queue
import os
import shutil
import signal
import subprocess
import threading
import time
from collections import deque

import gymnasium as gym
import numpy as np
import rclpy
import yaml
from geometry_msgs.msg import Point, Twist
from nav_msgs.msg import Odometry
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy
from std_msgs.msg import Bool, Empty, String

try:
    from px4_msgs.msg import (
        VehicleLocalPosition,
        VehicleAngularVelocity,
        VehicleAttitude,
        VehicleCommand,
    )
    _PX4_AVAILABLE = True
except ImportError:
    _PX4_AVAILABLE = False

# ---------------------------------------------------------------------------
# Module-level scaling constants (RAD v1; overridable via yaml)
# ---------------------------------------------------------------------------
POS_SCALE = 10.0          # RAD: v8 5 → 10 (RAD 좌표 √50 ≈ 7m 대응)
VEL_SCALE = 15.0          # v8 그대로
ANG_VEL_SCALE = math.pi   # v8 그대로

TARGET_ENU_X = 4.0        # RAD: target = (4, 3, 0) 지면 marker
TARGET_ENU_Y = 3.0
TARGET_ENU_Z = 0.0        # 지면 (drop_calculator 와 정합)

# z reward (Hann) 의 reference (w_dist 거리 계산용)
Z_REWARD_REFERENCE = 4.0  # w_dist 의 (target_x, target_y, 4) reference


def _load_config(config_path):
    """Load hyperparams yaml and return the parsed dict."""
    with open(config_path, 'r') as f:
        return yaml.safe_load(f)


class _RLBridgeNode(Node):
    """Background ROS2 node owning all pub/sub for the RL environment."""

    def __init__(self, state_lock, obs_ready_event,
                 drop_error_event, drop_error_queue,
                 px4_topic_prefix='', instance_id=0):
        super().__init__('drone_drop_rl_bridge')
        self._instance_id = instance_id

        self._lock = state_lock
        self._obs_ready = obs_ready_event
        self._drop_error_event = drop_error_event
        self._drop_error_queue = drop_error_queue

        # --- Shared state (protected by _lock) ---
        self.pos_enu = np.zeros(3, dtype=np.float32)
        self.vel_enu = np.zeros(3, dtype=np.float32)
        self.ang_vel = np.zeros(3, dtype=np.float32)
        self.roll = 0.0
        self.pitch = 0.0
        self.yaw_ned = 0.0   # RAD: PX4 NED yaw (radian). _get_obs 가 ENU 로 변환
        self.pixel_coords = np.zeros(3, dtype=np.float32)   # u, v, conf
        self.mission_state = 'IDLE'
        self.payload_attached = True

        # PX4 -i N publishes to /px4_N/fmu/out/* topics on the same
        # ROS_DOMAIN_ID.  Instance 0 (no -i flag) uses /fmu/out/*.
        pfx = px4_topic_prefix   # e.g. '' or '/px4_1'

        # --- QoS for PX4 topics ---
        px4_qos = QoSProfile(
            reliability=QoSReliabilityPolicy.BEST_EFFORT,
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=1,
        )

        # --- Subscribers ---
        if _PX4_AVAILABLE:
            self.create_subscription(
                VehicleLocalPosition,
                f'{pfx}/fmu/out/vehicle_local_position',
                self._on_local_pos,
                px4_qos,
            )
            self.create_subscription(
                VehicleAngularVelocity,
                f'{pfx}/fmu/out/vehicle_angular_velocity',
                self._on_ang_vel,
                px4_qos,
            )
            self.create_subscription(
                VehicleAttitude,
                f'{pfx}/fmu/out/vehicle_attitude',
                self._on_attitude,
                px4_qos,
            )
        else:
            self.get_logger().warning(
                'px4_msgs not available; position/velocity/attitude obs will be zeros.')

        self.create_subscription(
            Point, '/target/pixel_coords', self._on_pixel_coords, 10)
        # Issue #028 옵션 5: mission_state QoS default (VOLATILE + RELIABLE + KEEP_LAST 10).
        # TRANSIENT_LOCAL 폐기 — mission_manager 재시작 시 latched message stale + discovery
        # cache 문제 노출. mission_manager 는 control_loop @10Hz publish 이라 latching 불필요.
        # Publisher 도 default QoS 사용 (mission_manager_rad_node.py 도 변경).
        self._mission_state_qos = 10  # rclpy default (int depth)
        # Option 1a subscription handle 유지 (destroy/recreate escalation 여지)
        self.mission_state_sub = self.create_subscription(
            String, '/mission/state', self._on_mission_state, self._mission_state_qos)
        self.create_subscription(
            Odometry, '/drone/payload/position', self._on_payload_pos, 10)
        self.create_subscription(
            Bool, '/drone/payload/drop_cmd_raw', self._on_drop_cmd_raw, 10)

        # Drop error (published by drop_calculator after impact; used for logging)
        from std_msgs.msg import Float32
        self.create_subscription(
            Float32, '/rl/drop_error', self._on_drop_error, 10)

        # --- Publishers ---
        self.vel_pub = self.create_publisher(Twist, '/drone/cmd/velocity', 10)
        self.detach_pub = self.create_publisher(
            Empty, '/payload/drop_cmd', 10)
        self.drop_raw_pub = self.create_publisher(
            Bool, '/drone/payload/drop_cmd_raw', 10)
        # Issue #022 옵션 C — re-attach DetachableJoint without killing infra.
        self.attach_pub = self.create_publisher(
            Empty, '/payload/attach_cmd', 10)

        # VehicleCommand publisher — used to disarm PX4 between episodes.
        # Must share the same QoS as drone_controller's publisher so PX4
        # receives the command.
        if _PX4_AVAILABLE:
            vc_qos = QoSProfile(
                reliability=QoSReliabilityPolicy.BEST_EFFORT,
                history=QoSHistoryPolicy.KEEP_LAST,
                depth=1,
            )
            self.vehicle_cmd_pub = self.create_publisher(
                VehicleCommand, f'{pfx}/fmu/in/vehicle_command', vc_qos)
        else:
            self.vehicle_cmd_pub = None

    # ------------------------------------------------------------------
    # Subscriber callbacks
    # ------------------------------------------------------------------

    # Maximum plausible position magnitude (metres). The mission world spans
    # ~150 m; anything beyond 1 000 m is a Gazebo/EKF physics glitch.
    _POS_PLAUSIBLE_MAX = 1000.0

    def _on_local_pos(self, msg):
        """Convert NED to ENU and update shared state.

        Two-layer guard:
        1. NaN guard: PX4 EKF can output NaN during DDS time-sync resets.
        2. Magnitude guard: rejects finite but physically impossible values
           (e.g. 1.98e11 m from a Gazebo ODE explosion). Retains last-known-
           good position so neither the observation nor _compute_d_xy ever
           receives a coordinate explosion.
        """
        def _valid_pos(v):
            return math.isfinite(v) and abs(v) < self._POS_PLAUSIBLE_MAX

        with self._lock:
            # NED→ENU: East=Y_ned, North=X_ned, Up=−Z_ned
            self.pos_enu[0] = msg.y if _valid_pos(msg.y) else self.pos_enu[0]
            self.pos_enu[1] = msg.x if _valid_pos(msg.x) else self.pos_enu[1]
            self.pos_enu[2] = -msg.z if _valid_pos(msg.z) else self.pos_enu[2]
            self.vel_enu[0] = msg.vy if math.isfinite(msg.vy) else 0.0
            self.vel_enu[1] = msg.vx if math.isfinite(msg.vx) else 0.0
            self.vel_enu[2] = -msg.vz if math.isfinite(msg.vz) else 0.0
        self._obs_ready.set()

    def _on_ang_vel(self, msg):
        with self._lock:
            self.ang_vel[0] = msg.xyz[0] if math.isfinite(msg.xyz[0]) else 0.0
            self.ang_vel[1] = msg.xyz[1] if math.isfinite(msg.xyz[1]) else 0.0
            self.ang_vel[2] = msg.xyz[2] if math.isfinite(msg.xyz[2]) else 0.0

    def _on_attitude(self, msg):
        """Extract roll, pitch, yaw from PX4 quaternion (NED frame, FRD body).

        PX4 VehicleAttitude q = [w, x, y, z]. ZYX Euler extraction.
        RAD: yaw 도 추출 (NED → ENU 변환은 _get_obs 에서 처리).
        """
        q = msg.q
        w, x, y, z = float(q[0]), float(q[1]), float(q[2]), float(q[3])

        sinr_cosp = 2.0 * (w * x + y * z)
        cosr_cosp = 1.0 - 2.0 * (x * x + y * y)
        roll = math.atan2(sinr_cosp, cosr_cosp)

        sinp = 2.0 * (w * y - z * x)
        sinp = max(-1.0, min(1.0, sinp))
        pitch = math.asin(sinp)

        # Yaw (NED frame, +Z down 회전): around NED z-axis
        siny_cosp = 2.0 * (w * z + x * y)
        cosy_cosp = 1.0 - 2.0 * (y * y + z * z)
        yaw_ned = math.atan2(siny_cosp, cosy_cosp)

        with self._lock:
            self.roll = roll
            self.pitch = pitch
            # NED yaw 그대로 저장. _get_obs 에서 ENU yaw 로 변환.
            self.yaw_ned = yaw_ned

    def _on_pixel_coords(self, msg):
        with self._lock:
            self.pixel_coords[0] = msg.x   # u
            self.pixel_coords[1] = msg.y   # v
            self.pixel_coords[2] = msg.z   # confidence

    def _on_mission_state(self, msg):
        with self._lock:
            self.mission_state = msg.data

    def _on_payload_pos(self, msg):
        """Track payload position (reserved for future extensions)."""
        pass

    def _on_drop_cmd_raw(self, msg):
        if not msg.data:   # False = drop event (inverted semantics)
            with self._lock:
                self.payload_attached = False

    def _on_drop_error(self, msg):
        """Called by drop_calculator after payload impacts ground (logging only)."""
        error = float(msg.data)
        try:
            self._drop_error_queue.put_nowait(error)
        except queue.Full:
            pass
        self._drop_error_event.set()

    def recreate_mission_state_sub(self):
        """Issue #028 옵션 1a: mission_state subscription destroy + recreate.

        Root cause: env subscriber 는 program lifetime 유지 → 매 mission_manager
        재시작 시 이전 publisher 의 dead endpoint entry 가 discovery cache 에 stuck.
        DDS 가 dead entry cleanup 하는 데 heartbeat timeout (30s+) → 그 전 새
        publisher launch 되면 subscriber 가 dead entry 로 route → publish 놓침 →
        deterministic multi-retry fail (관찰: 15번 연속 fail).

        Fix: 매 reset 시 subscription destroy → RMW layer 의 discovery cache wipe.
        Recreate → fresh discovery cycle → 살아있는 publisher 만 발견.
        """
        try:
            self.destroy_subscription(self.mission_state_sub)
        except Exception as e:
            self.get_logger().warn(f'destroy_subscription failed: {e}')
        with self._lock:
            self.mission_state = 'IDLE'
        self.mission_state_sub = self.create_subscription(
            String, '/mission/state', self._on_mission_state, self._mission_state_qos)

    # ------------------------------------------------------------------
    # Action helpers
    # ------------------------------------------------------------------

    def publish_velocity(self, vx, vy, vz, yaw_rate):
        """Publish ENU velocity command."""
        twist = Twist()
        twist.linear.x = float(vx)
        twist.linear.y = float(vy)
        twist.linear.z = float(vz)
        twist.angular.z = float(yaw_rate)
        self.vel_pub.publish(twist)

    def publish_drop(self):
        """Trigger payload drop via DetachableJoint detach.

        Payload velocity is preserved by Gazebo physics on detach
        (verified by minimal_test mtest2: 101.5% transfer ratio).
        """
        self.detach_pub.publish(Empty())
        drop_msg = Bool()
        drop_msg.data = False   # False = drop event (inverted semantics)
        self.drop_raw_pub.publish(drop_msg)

    def disarm_px4(self):
        """Send DISARM command directly to PX4.

        Called during episode reset so PX4 is disarmed before the new
        drone_controller starts.  This ensures the drone_controller 5 s EKF
        warmup runs on a freshly-armed vehicle (not a carry-over armed state),
        giving the EKF time to reconverge after the set_pose teleport.

        Without this, PX4 stays armed across episodes; the new
        drone_controller skips the warmup (already_armed && in_offboard →
        return immediately) and records arm_ned_z at a post-teleport EKF
        transient (e.g. -267 m), causing threshold = -4.75 to fire before
        the drone has physically climbed.
        """
        if self.vehicle_cmd_pub is None:
            return
        msg = VehicleCommand()
        msg.timestamp = int(self.get_clock().now().nanoseconds / 1000)
        msg.command = VehicleCommand.VEHICLE_CMD_COMPONENT_ARM_DISARM
        msg.param1 = 0.0   # 0 = disarm
        msg.param2 = 21196.0   # magic value required for force-disarm in OFFBOARD
        msg.target_system = 1
        msg.target_component = 1
        msg.source_system = 1
        msg.source_component = 1
        msg.from_external = True
        self.vehicle_cmd_pub.publish(msg)


# ---------------------------------------------------------------------------
# Main Environment
# ---------------------------------------------------------------------------

class DroneDropEnvRAD(gym.Env):
    """Gymnasium env: SAC trains a fly-by precision-drop policy.

    Observation: Box(17,) float32
      [0-14] same as before (pos, vel, ang_vel, vision, attached, rel_target)
      [15]   d_impact / 50m (CCIP predicted miss distance, normalised)
      [16]   t_f / 10s     (CCIP time-of-flight, normalised, clamped at 10s)
    Action:      Box(5,)  float32 in [-1, 1]
      [0] vx command (scaled by action_vx_scale)
      [1] vy command (scaled by action_vy_scale)
      [2] vz command (scaled by action_vz_scale)
      [3] yaw_rate command (scaled by action_yaw_scale)
      [4] manual drop trigger (> 0 fires drop; auto-drop overrides via kinematics)
    """

    metadata = {'render_modes': []}

    # Safety limits for Layer 1 (not in yaml — physical hard stops)
    _V_MAX_SAFETY = 20.0   # m/s horizontal+vertical magnitude

    def __init__(self, config_path=None, instance_id=0):
        super().__init__()

        # --- Instance isolation ---
        self._instance_id = instance_id
        # GZ_PARTITION removed — breaks PX4 lockstep; shared Gazebo instead
        self._uxrce_port = 8888 + instance_id
        # PX4 -i N publishes to /px4_N/fmu/* topics; instance 0 uses /fmu/*
        self._px4_ns = f'/px4_{instance_id}' if instance_id > 0 else ''
        # Pre-spawn drone in world SDF + PX4 connects via PX4_GZ_MODEL_NAME.
        # Issue #014: <include merge="true"> breaks DetachableJoint in gz-sim8.
        # Inline x500_bombard model is pre-spawned in world SDF so drone+payload
        # load simultaneously → DetachableJoint forms joint correctly.
        # PX4 uses PX4_GZ_MODEL_NAME (no spawn, connects to existing model).
        self._gz_model_name = f'x500_bombard_{instance_id}'        # name in world SDF <include>
        self._model_name = f'x500_bombard_{instance_id}'           # Gazebo entity name
        self._drop_topic = 'x500_bombard'                          # drop gz-topic prefix
        self._payload_name = f'payload_{instance_id}'       # Paired payload model name

        # Set ROS_DOMAIN_ID before rclpy.init() so DDS uses the right domain
        os.environ['ROS_DOMAIN_ID'] = str(instance_id)

        # --- Load config (RAD: 추가로 cfg_training, cfg_cruise, cfg_phase1, cfg_phase2) ---
        cfg_env = {}
        cfg_reward = {}
        cfg_training = {}
        cfg_cruise = {}
        cfg_phase1 = {}
        cfg_phase2 = {}
        if config_path is not None:
            cfg = _load_config(config_path)
            cfg_env = cfg.get('environment', {})
            cfg_reward = cfg.get('reward', {})
            cfg_training = cfg.get('training', {})
            cfg_cruise = cfg.get('cruise', {})
            cfg_phase1 = cfg.get('phase1', {})
            cfg_phase2 = cfg.get('phase2', {})

        # --- RAD: Phase 분기 ---
        # "phase1" or "phase2". train_sac_rad 가 yaml override 가능.
        self._cfg_phase = cfg_training.get('phase', 'phase1')
        assert self._cfg_phase in ('phase1', 'phase2'), \
            f"Invalid phase: {self._cfg_phase}"

        # --- Environment constants ---
        self._cfg_target_x = cfg_env.get('target_enu_x', TARGET_ENU_X)
        # Method A: target Y shifts by 150m per instance
        self._cfg_target_y = cfg_env.get('target_enu_y', TARGET_ENU_Y) + instance_id * 150.0
        self._cfg_target_z = cfg_env.get('target_enu_z', TARGET_ENU_Z)   # RAD: 0.0 (지면)
        # RAD: 폐기 (v10a stage1 임시값). 0 으로 두면 dead.
        self._cfg_stage1_R = cfg_env.get('stage1_R', 0.0)
        self._cfg_stage1_reach_bonus = cfg_env.get('stage1_reach_bonus', 0.0)
        self._cfg_stage1_only = cfg_env.get('stage1_only', False)
        self._cfg_pos_scale = cfg_env.get('pos_scale', POS_SCALE)
        self._cfg_vel_scale = cfg_env.get('vel_scale', VEL_SCALE)
        self._cfg_ang_vel_scale = cfg_env.get('ang_vel_scale', ANG_VEL_SCALE)
        self._cfg_action_vx_scale = cfg_env.get('action_vx_scale', 3.0)
        self._cfg_action_vy_scale = cfg_env.get('action_vy_scale', 3.0)
        self._cfg_action_vz_scale = cfg_env.get('action_vz_scale', 3.0)
        self._cfg_action_yaw_scale = cfg_env.get('action_yaw_scale', 1.0)
        # RAD: Phase 별 action_rate_limit
        if self._cfg_phase == 'phase1':
            self._cfg_action_rate_limit = cfg_env.get('action_rate_limit_phase1', 0.2)
        else:  # phase2
            self._cfg_action_rate_limit = cfg_env.get('action_rate_limit_phase2', 0.15)
        # RAD: max_steps 분기. Phase 1 = 800 (RL step 1~800). Phase 2 = 200 (sphere 진입 후).
        if self._cfg_phase == 'phase1':
            self._cfg_max_steps = cfg_env.get('max_steps', 800)
        else:
            self._cfg_max_steps = cfg_env.get('phase2_max_steps', 200)
        self._cfg_min_altitude = cfg_env.get('min_altitude', 0.5)
        self._cfg_min_alt_start = cfg_env.get('min_altitude_start_step', 1)
        self._cfg_ground_contact_alt = cfg_env.get('ground_contact_altitude', 0.5)
        self._cfg_max_distance = cfg_env.get('max_distance', 15.0)
        self._cfg_max_altitude = cfg_env.get('max_altitude', 50.0)
        self._cfg_obs_wait = cfg_env.get('obs_wait_timeout', 0.02)
        self._cfg_cruise_timeout = cfg_env.get('cruise_poll_timeout', 60.0)
        self._cfg_sim_speed_factor = int(cfg_env.get('sim_speed_factor', 1))
        self._cfg_use_vision = cfg_env.get('use_vision', False)

        # --- RAD: cruise/spawn yaw (mission_manager_rad 가 실제 처리, env 는 logging 만) ---
        self._cfg_cruise_target_speed = cfg_cruise.get('target_speed', 1.0)
        self._cfg_spawn_yaw_random_enabled = cfg_cruise.get(
            'spawn_yaw_random_enabled', True)
        self._cfg_spawn_yaw_relative_range = cfg_cruise.get(
            'spawn_yaw_relative_range', [-1.5708, 1.5708])

        # --- RAD: Phase 1 cfg ---
        self._cfg_phase1_switch_d_sq = cfg_phase1.get('switch_d_sq', 20.5)
        self._cfg_phase1_terminal_floor = cfg_phase1.get('terminal_floor', 20.0)
        self._cfg_phase1_terminal_w_each = cfg_phase1.get('terminal_w_each', 50.0 / 7.0)
        self._cfg_phase1_terminal_complete_bonus = cfg_phase1.get(
            'terminal_complete_bonus', 50.0)
        # 7 final state 조건 임계값
        self._cfg_p1_C1_z_min = cfg_phase1.get('C1_z_min', 3.0)
        self._cfg_p1_C1_z_max = cfg_phase1.get('C1_z_max', 5.0)
        self._cfg_p1_C2_v_xy_max = cfg_phase1.get('C2_v_xy_max', 4.0)
        self._cfg_p1_C3_v_z_max = cfg_phase1.get('C3_v_z_max', 2.0)
        self._cfg_p1_C4_tilt_max = cfg_phase1.get('C4_tilt_max', 0.26)
        self._cfg_p1_C5_omega_max = cfg_phase1.get('C5_omega_max', 2.0)
        self._cfg_p1_C6_yaw_err_max = cfg_phase1.get('C6_yaw_err_max', 1.047)
        self._cfg_p1_C7_v_xy_min = cfg_phase1.get('C7_v_xy_min', 0.3)

        # === Curriculum config (Issue #028 v2: advance 강화 + regression) ===
        self._cfg_curriculum_enabled = cfg_phase1.get('curriculum_enabled', False)
        self._cfg_curriculum_window_size = int(cfg_phase1.get('curriculum_window_size', 10000))
        self._cfg_curriculum_success_threshold = cfg_phase1.get(
            'curriculum_success_threshold', 0.95)
        self._cfg_curriculum_min_stage_steps = int(cfg_phase1.get(
            'curriculum_min_stage_steps', 10000))
        self._cfg_curriculum_stages = cfg_phase1.get('curriculum_stages', [])
        # Regression config
        self._cfg_curriculum_regression_enabled = cfg_phase1.get(
            'curriculum_regression_enabled', False)
        self._cfg_curriculum_regression_threshold = cfg_phase1.get(
            'curriculum_regression_threshold', 0.3)
        self._cfg_curriculum_regression_min_steps = int(cfg_phase1.get(
            'curriculum_regression_min_steps', 5000))
        self._cfg_curriculum_regression_cooldown = int(cfg_phase1.get(
            'curriculum_regression_cooldown', 20000))
        # Current stage state
        self._curriculum_stage_idx = 0                     # 0-indexed
        self._curriculum_stage_step_start = 0              # total_step when this stage started
        # deque maxlen 은 window / avg_ep_len 근사. window=10000, avg_ep~200 → 50. 여유 200.
        self._curriculum_ep_history = deque(maxlen=200)    # (total_step_at_end, success_bool)
        self._curriculum_last_regression_step = -10**9     # -∞ (cooldown 초기 무효)
        self._total_step_count = 0                         # Global step counter (never reset)
        # Debug print at init
        print(f'[CURRICULUM_INIT] id={id(self)} enabled={self._cfg_curriculum_enabled}, '
              f'stages={len(self._cfg_curriculum_stages)}, '
              f'window={self._cfg_curriculum_window_size}, '
              f'threshold={self._cfg_curriculum_success_threshold}, '
              f'min_stage_steps={self._cfg_curriculum_min_stage_steps}, '
              f'regression_enabled={self._cfg_curriculum_regression_enabled}, '
              f'regression_threshold={self._cfg_curriculum_regression_threshold}, '
              f'regression_min_steps={self._cfg_curriculum_regression_min_steps}, '
              f'regression_cooldown={self._cfg_curriculum_regression_cooldown}', flush=True)
        if self._cfg_curriculum_stages:
            print(f'[CURRICULUM_INIT] stage0 = {self._cfg_curriculum_stages[0]}', flush=True)

        # --- RAD: Phase 2 cfg ---
        self._cfg_phase2_sphere_escape_d_sq = cfg_phase2.get('sphere_escape_d_sq', 22.0)
        self._cfg_phase2_sphere_escape_penalty = cfg_phase2.get(
            'sphere_escape_penalty', -30.0)

        # --- Reward constants (RAD: phase 별 분기 포함) ---
        r = cfg_reward
        self._cfg_g = r.get('g', 9.81)
        self._cfg_auto_drop_threshold = r.get('auto_drop_threshold', 1.0)  # RAD: 2 → 1
        self._cfg_random_drop_start_step = r.get('random_drop_start_step', 600)
        self._cfg_random_drop_prob = r.get('random_drop_prob', 0.0)
        self._cfg_k1_potential = r.get('k1_potential', 1.0)
        self._cfg_k2_precision = r.get('k2_precision', 0.2)
        self._cfg_w_dist = r.get('w_dist', 1.0)
        self._cfg_w_heading = r.get('w_heading', 0.7)
        self._cfg_w_distance_penalty = r.get('w_distance_penalty', 0.0)

        # --- RAD: phase 별 per-step penalty ---
        if self._cfg_phase == 'phase1':
            self._cfg_w_time = r.get('w_time_phase1', -0.05)
            self._cfg_w_ang_vel = r.get('w_ang_vel_phase1', 0.05)
            self._cfg_w_action_smooth = r.get('w_action_smooth_phase1', 0.05)
            self._cfg_w_impact = r.get('w_impact_phase1', 0.0)
            self._cfg_k_impact = r.get('k_impact_phase1', 0.05)
        else:  # phase2
            self._cfg_w_time = r.get('w_time_phase2', -0.1)
            self._cfg_w_ang_vel = r.get('w_ang_vel_phase2', 0.1)
            self._cfg_w_action_smooth = r.get('w_action_smooth_phase2', 0.1)
            self._cfg_w_impact = r.get('w_impact_phase2', 1.0)
            self._cfg_k_impact = r.get('k_impact_phase2', 0.1)

        # --- RAD 신규: z Hann reward ---
        self._cfg_w_z = r.get('w_z', 0.3)
        self._cfg_z_target = r.get('z_target', 4.0)
        self._cfg_z_half_range = r.get('z_half_range', 3.5)

        self._cfg_w_drop_base = r.get('w_drop_base', 100.0)
        self._cfg_r_success_jackpot = r.get('r_success_jackpot', 50.0)
        self._cfg_success_threshold = r.get('success_threshold', 1.0)  # RAD: 2 → 1
        self._cfg_jackpot_threshold = r.get('jackpot_threshold', 0.3)
        self._cfg_penalty_instability = r.get('penalty_instability', 50.0)
        self._cfg_limit_ang_vel = r.get('limit_ang_vel', 10.0)
        self._cfg_limit_tilt = r.get('limit_tilt', 0.26)
        self._cfg_limit_inverted_tilt = r.get('limit_inverted_tilt', 1.047)
        self._cfg_penalty_bad_attitude = r.get('penalty_bad_attitude', -30.0)
        self._cfg_drop_attempt_bonus = r.get('drop_attempt_bonus', 30.0)
        self._cfg_k_drop_proximity = r.get('k_drop_proximity', 0.1)  # RAD: 0.4 → 0.1
        self._cfg_truncation_penalty = r.get('truncation_penalty', -15.0)
        self._cfg_penalty_crash = r.get('penalty_crash', -50.0)
        self._cfg_penalty_overspeed = r.get('penalty_overspeed', -30.0)
        self._cfg_penalty_target_lost = r.get('penalty_target_lost', -10.0)
        self._cfg_penalty_out_of_range = r.get('penalty_out_of_range', -30.0)
        self._cfg_penalty_max_altitude = r.get('penalty_max_altitude', -15.0)
        self._cfg_hover_speed_threshold = r.get('hover_speed_threshold', 1.0)
        self._cfg_hover_consecutive_threshold = int(r.get('hover_consecutive_threshold', 150))
        self._cfg_drop_angaccel_penalty_scale = r.get('drop_angaccel_penalty_scale', 0.5)
        self._cfg_drop_angaccel_window_n = int(r.get('drop_angaccel_window_n', 5))
        self._ang_vel_history = deque(maxlen=self._cfg_drop_angaccel_window_n + 1)
        self._cfg_penalty_hover = r.get('penalty_hover', -30.0)
        self._cfg_hover_truncate_enabled = bool(r.get('hover_truncate_enabled', True))
        self._cfg_hover_drop_block_threshold = r.get('hover_drop_block_threshold', 0.0)
        self._cfg_invalid_drop_threshold = r.get('invalid_drop_threshold', 95.0)
        self._cfg_invalid_drop_penalty = r.get('invalid_drop_penalty', 0.0)
        self._cfg_alt_penalty_max = r.get('alt_penalty_max', 0.0)  # RAD: 폐기
        self._cfg_alt_penalty_mid = r.get('alt_penalty_mid', 0.0)
        self._cfg_alt_penalty_k = r.get('alt_penalty_k', 0.0)
        self._cfg_w_prediction = r.get('w_prediction', 0.0)
        self._cfg_k_prediction = r.get('k_prediction', 0.1)
        self._cfg_drop_wait_timeout = r.get('drop_wait_timeout', 3.0)
        self._cfg_speed_gate = r.get('speed_gate_enabled', True)

        # --- RAD obs (14d, yaw-only body frame) ---
        # [0:3]   = (Δx_b, Δy_b, Δz_world) / POS_SCALE     # target - drone, body frame
        # [3:6]   = (vx_b, vy_b, vz_world) / VEL_SCALE     # drone vel, body frame
        # [6:9]   = (ω) / π                                # body ang_vel
        # [9:11]  = (roll, pitch) / π
        # [11]    = payload_attached                       # Phase 1 dead = 1.0
        # [12]    = clip(d_impact / POS_SCALE, 0, 1)       # Phase 1 dead = 0
        # [13]    = clip(t_f / 10, 0, 1)                   # Phase 1 dead = 0
        self.observation_space = gym.spaces.Box(
            low=-1.0, high=1.0, shape=(14,), dtype=np.float32)
        # action 5d 그대로 (action[4] drop dead in both phases)
        self.action_space = gym.spaces.Box(
            low=-1.0, high=1.0, shape=(5,), dtype=np.float32)

        # --- Threading primitives ---
        self._state_lock = threading.Lock()
        self._obs_ready = threading.Event()
        self._drop_error_event = threading.Event()
        self._drop_error_queue = queue.Queue(maxsize=1)

        # --- Episode state ---
        self._step_count = 0
        self.dropped = False
        self._episode_reward = 0.0
        self._episode_procs = []      # Popen handles for episode nodes
        self.d_xy_prev = 0.0          # 2D horizontal distance to target at previous step
        self.action_prev = np.zeros(5, dtype=np.float32)
        # Round 5: hover 감지 (연속 정체 step 추적)
        self._consecutive_still = 0
        self._max_consecutive_still = 0

        # --- Infra state ---
        self._infra_procs = []        # Popen handles for infra processes

        # --- Reset recursion guard ---
        self._reset_depth = 0

        # --- Diagnostic (Round 7 leak investigation) ---
        # Tracks velocity at end-of-episode (pre-reset) vs at CRUISE (post-reset).
        # If pre_v stays large but post_v ≈ 0 → PX4 EKF absorbs the discontinuity.
        # If post_v grows over many resets → residual velocity accumulation hypothesis.
        self._reset_count = 0
        self._reset_diag = {}

        # --- 2차 처방: 누적 fast-path reset 방지 (periodic forced full restart) ---
        # drop 없이 fast-path teleport reset 이 N회 연속되면 강제 _kill_infra/_start_infra.
        # 누적 가설이 사실이면 cap 전에 청소; 가설이 틀려도 비용은 ~4% slowdown 뿐.
        self._consecutive_fast_resets = 0
        self._cfg_max_consecutive_fast_resets = int(cfg_env.get(
            'max_consecutive_fast_resets', 50))   # RAD: 100 → 50
        # Issue #022 옵션 C 안전장치 B: safe drop path 누적 카운터.
        # 50 회 누적 시 강제 _kill_infra (Gazebo leak 차단).
        self._safe_drop_count = 0
        self._safe_drop_count_max = 50

        # --- Start ROS2 ---
        if not rclpy.ok():
            rclpy.init()
        self._node = _RLBridgeNode(
            self._state_lock,
            self._obs_ready,
            self._drop_error_event,
            self._drop_error_queue,
            px4_topic_prefix=self._px4_ns,
            instance_id=self._instance_id,
        )
        self._spin_thread = threading.Thread(
            target=self._spin_loop, daemon=True, name='Thread-2 (spin)')
        self._spin_thread.start()

        # --- Self-managed infrastructure ---
        self._start_infra()

    # ------------------------------------------------------------------
    # Gymnasium API
    # ------------------------------------------------------------------

    def reset(self, *, seed=None, options=None):
        """Reset simulation and return initial observation."""
        super().reset(seed=seed)

        # FPS-breakdown probe (Issue #022): record per-phase wall time.
        _probe_t0 = time.monotonic()
        _probe = {
            'kill_ep': 0.0, 'kill_infra': 0.0, 'start_infra': 0.0,
            'gz_reset': 0.0, 'start_ep': 0.0, 'cruise': 0.0,
            'cruise_retries': 0,
        }

        # Guard against infinite recursion when CRUISE is permanently unreachable
        # (e.g. spin thread died). Allow at most 5 nested retries total.
        # RAD: 2 → 5 (552 ep 시점 IDLE stuck 의 long-tail recovery 위해 증가)
        self._reset_depth += 1
        if self._reset_depth > 5:
            self._reset_depth = 0
            raise RuntimeError(
                '[RL Env] reset() called recursively more than 5 times. '
                'Spin thread may be dead or infra is unrecoverable. Aborting.')

        # 1. Clear episode state
        # Save dropped state BEFORE clearing — used below to decide infra restart path.
        _prev_dropped = self.dropped

        # Round 7 diagnostic: capture velocity at END of previous episode
        # (this is the residual velocity that set_pose CANNOT reset).
        with self._state_lock:
            _pre_reset_v = float(np.linalg.norm(self._node.vel_enu))
            _pre_reset_ang_v = float(np.linalg.norm(self._node.ang_vel))

        self._step_count = 0
        self.dropped = False
        self._episode_reward = 0.0
        self._obs_ready.clear()
        self._drop_error_event.clear()
        self.action_prev = np.zeros(5, dtype=np.float32)
        # Round 5: hover 감지 초기화
        self._consecutive_still = 0
        self._max_consecutive_still = 0

        # Drain drop-error queue
        while not self._drop_error_queue.empty():
            try:
                self._drop_error_queue.get_nowait()
            except queue.Empty:
                break

        with self._state_lock:
            self._node.payload_attached = True
            self._node.mission_state = 'IDLE'  # avoid stale CRUISE from last episode
            # Round 4 fix: stale pos/vel 초기화 — reset 후 PX4 callback 갱신 전 step 1에서
            # 이전 episode 위치를 읽는 reset 버그 방지.
            # x, y는 (0, 0) 으로 마킹 (auto_drop은 d_impact≤3m에서 발동 — 14.87m면 안전).
            # z는 5m로 마킹 (Round 4: 이전 0이 ground_contact_alt 0.5m 위반 → step 1 crash 유발).
            # 진짜 값은 PX4 callback이 곧 덮어씀.
            self._node.pos_enu = np.array([0.0, 0.0, 5.0], dtype=np.float32)
            self._node.vel_enu = np.zeros(3, dtype=np.float32)
            self._node.ang_vel = np.zeros(3, dtype=np.float32)
            self._node.roll = 0.0
            self._node.pitch = 0.0
            self._node.pixel_coords = np.zeros(3, dtype=np.float32)

        # Ensure spin thread is alive before proceeding
        if not self._spin_thread.is_alive():
            self._node.get_logger().error(
                '[RL Env] Spin thread is dead — restarting...')
            self._spin_thread = threading.Thread(
                target=self._spin_loop, daemon=True, name='Thread-2 (spin)')
            self._spin_thread.start()
            time.sleep(1.0)

        # 2. Kill previous episode processes.
        _t = time.monotonic(); self._kill_episode(); _probe['kill_ep'] = time.monotonic() - _t

        # 3. Reset drone + payload to spawn position.
        #    If payload was dropped this episode, DetachableJoint is gone.
        #    _gz_world_reset(model_only) caused ODE AABB integer overflow crash
        #    because residual payload velocity after drop destabilises physics.
        #    Fix: full infra restart (kill Gazebo+PX4 and restart) — DetachableJoint
        #    is re-created cleanly from SDF. Slower (~30s) but crash-free.
        #    If no drop occurred, the faster teleport path is sufficient.
        #
        # Round 7 (2차 처방): drop 없이 fast-path 가 N회 연속이면 강제 full restart.
        # Round 3/6 v2/7 의 gz model --list timeout crash 가설:
        #   set_pose 가 velocity reset 못함 → 누적 → Gazebo deadlock.
        # 가설 검증 무관하게 누적 자체 차단.
        _forced_restart = (
            self._consecutive_fast_resets
            >= self._cfg_max_consecutive_fast_resets)

        if _prev_dropped or _forced_restart:
            # v7 (D1): 옵션 C safe drop path 비활성.
            # 이유: DetachableJoint plugin 의 reattach 후 detach 가 silent fail.
            # → safe path 의 attach 가 작동해도 다음 detach 무동작 (invalid 50-70%).
            # 매 drop 마다 _kill_infra + _start_infra (38s) 로 fresh SDF attach 보장.
            self._obs_ready.clear()
            _t = time.monotonic(); self._kill_infra(); _probe['kill_infra'] = time.monotonic() - _t
            _t = time.monotonic(); self._start_infra(); _probe['start_infra'] = time.monotonic() - _t
            print(f'[GZ_SERVER_READY] reset_count={self._reset_count + 1}', flush=True)  # GUI relaunch trigger
            self._safe_drop_count = 0
            self._consecutive_fast_resets = 0
            if _forced_restart and rclpy.ok():
                self._node.get_logger().info(
                    f'[RL Env] Forced full restart after'
                    f' {self._cfg_max_consecutive_fast_resets}'
                    ' consecutive fast resets (no drop)')
        else:
            _t = time.monotonic(); self._gz_reset_poses(); _probe['gz_reset'] = time.monotonic() - _t
            self._consecutive_fast_resets += 1

        # Issue #028 옵션 1a: fresh mission_state subscription BEFORE new mission_manager launch.
        # → 이전 publisher 의 dead endpoint entry (discovery cache 안) 제거 → 새 publisher fresh discover.
        self._node.recreate_mission_state_sub()

        # 4. Start fresh episode processes
        _t = time.monotonic(); self._start_episode(); _probe['start_ep'] = time.monotonic() - _t

        # 6. Wait for CRUISE state (blocks until takeoff + climb complete).
        #    Tiered recovery on timeout:
        #      Attempt 1: episode-only restart if Gazebo+PX4 still healthy (fast, ~5s).
        #                 Covers most cases: drone_controller/mission_manager issue.
        #      Attempt 2+: full infra restart (kills Gazebo+PX4, re-spawns fresh, ~80s).
        #                  Used when infra itself is unhealthy or fast path failed.
        #    Retries up to 3 times before giving up; this episode is never
        #    returned to SB3 so the replay buffer stays clean.
        for _cruise_attempt in range(3):
            _t = time.monotonic()
            self._wait_for_cruise()
            _probe['cruise'] += time.monotonic() - _t
            if _cruise_attempt > 0:
                _probe['cruise_retries'] = _cruise_attempt
            # RAD: mission_manager_rad 의 state machine 은 TAKEOFF → YAW_INIT →
            # CRUISE → HANDOFF → TRACKING. RL 정책은 TRACKING 시점부터 control.
            with self._state_lock:
                _reached_cruise = (self._node.mission_state == 'TRACKING')
            if _reached_cruise:
                break

            # Tiered recovery decision
            _try_fast_path = (
                _cruise_attempt == 0
                and self._check_infra_healthy(self._instance_id)
            )

            if _try_fast_path:
                self._node.get_logger().warn(
                    f'[RL Env] CRUISE timeout (attempt {_cruise_attempt + 1}/3)'
                    ' — episode-only restart (infra healthy, fast path)')
                self._kill_episode()
                # Issue #028 옵션 1a: recreate subscription (dead publisher entry 제거)
                self._node.recreate_mission_state_sub()
                self._gz_reset_poses()   # teleport drone+payload to spawn
                self._start_episode()
            else:
                self._node.get_logger().warn(
                    f'[RL Env] CRUISE timeout (attempt {_cruise_attempt + 1}/3)'
                    ' — full infra restart')
                self._kill_episode()
                self._kill_infra()
                self._obs_ready.clear()  # ensure _start_infra() waits for fresh PX4 data
                # Issue #028 옵션 1a: recreate subscription (dead publisher entry 제거)
                self._node.recreate_mission_state_sub()
                self._start_infra()
                print(f'[GZ_SERVER_READY] reset_count={self._reset_count + 1} (retry)', flush=True)
                self._start_episode()
        else:
            # All 3 attempts failed — attempt one full reset from scratch.
            # _reset_depth guard above prevents infinite recursion.
            self._node.get_logger().error(
                '[RL Env] CRUISE timeout after 3 attempts — forcing full reset')
            return self.reset(seed=seed, options=options)

        self._reset_depth = 0  # success — clear depth counter

        # 7. Seed d_xy_prev from the initial post-cruise (TRACKING) position.
        # RAD: w_dist 의 거리 계산이 d_reward (3D, target_z=4 reference) 이므로
        # seed 도 d_reward 로. 변수 명은 v8 호환 위해 d_xy_prev 유지.
        with self._state_lock:
            pos = self._node.pos_enu.copy()
            _post_cruise_v = float(np.linalg.norm(self._node.vel_enu))
            _post_cruise_ang_v = float(np.linalg.norm(self._node.ang_vel))
        self.d_xy_prev = self._compute_d_reward(pos)
        # v9a: ang_vel history reset (drop 시점 ang_accel penalty 추적용)
        self._ang_vel_history.clear()

        # Round 7 diagnostic: store reset diag for callback to read
        self._reset_count += 1
        self._reset_diag = {
            'pre_reset_v': _pre_reset_v,
            'pre_reset_ang_v': _pre_reset_ang_v,
            'post_cruise_v': _post_cruise_v,
            'post_cruise_ang_v': _post_cruise_ang_v,
            'reset_idx': self._reset_count,
            'prev_dropped': bool(_prev_dropped),
            # 2차 처방 + 1차 처방 진단 — 다음 crash 시 직전 100 sample 으로
            # 가설 검증 가능하도록 추가 기록.
            'used_full_restart': bool(_prev_dropped or _forced_restart),
            'forced_restart_triggered': bool(_forced_restart),
            'consecutive_fast_resets': self._consecutive_fast_resets,
            'cruise_timeout_attempts': _cruise_attempt,
        }

        obs = self._get_obs()

        # FPS-breakdown probe (Issue #022): write per-reset breakdown.
        _probe_total = time.monotonic() - _probe_t0
        try:
            with open('/tmp/fps_breakdown.csv', 'a') as _f:
                if self._reset_count == 1:
                    _f.write('ts,inst,ep_idx,path,total,kill_ep,kill_infra,start_infra,gz_reset,start_ep,cruise,cruise_retries\n')
                _path = 'drop' if _prev_dropped else ('forced' if _forced_restart else 'fast')
                _f.write(f"{time.time():.3f},{self._instance_id},{self._reset_count},{_path},"
                         f"{_probe_total:.3f},{_probe['kill_ep']:.3f},{_probe['kill_infra']:.3f},"
                         f"{_probe['start_infra']:.3f},{_probe['gz_reset']:.3f},"
                         f"{_probe['start_ep']:.3f},{_probe['cruise']:.3f},"
                         f"{_probe['cruise_retries']}\n")
        except Exception:
            pass

        return obs, {}

    def step(self, action):
        """Apply action, advance one control step, return (obs, reward, term, trunc, info)."""
        self._step_count += 1
        self._total_step_count += 1  # Global counter — never reset (curriculum 등 사용)

        # --- P2 (junsang_v4): action rate limit (가속도 hard clip) ---
        # action_prev 는 reset() 에서 zeros 로 초기화됨. step 당 |Δa| ≤ rate_limit.
        action = np.clip(
            np.asarray(action, dtype=np.float32),
            self.action_prev - self._cfg_action_rate_limit,
            self.action_prev + self._cfg_action_rate_limit,
        )

        # --- Decode and apply velocity command ---
        vx = float(action[0]) * self._cfg_action_vx_scale
        vy = float(action[1]) * self._cfg_action_vy_scale
        vz = float(action[2]) * self._cfg_action_vz_scale
        yaw_rate = float(action[3]) * self._cfg_action_yaw_scale
        self._node.publish_velocity(vx, vy, vz, yaw_rate)

        # --- Wait for next state observation ---
        self._obs_ready.clear()
        self._obs_ready.wait(timeout=self._cfg_obs_wait)

        # --- Snapshot raw state (protected copy) ---
        with self._state_lock:
            pos = self._node.pos_enu.copy()
            vel = self._node.vel_enu.copy()
            ang = self._node.ang_vel.copy()
            roll = self._node.roll
            pitch = self._node.pitch
            pix = self._node.pixel_coords.copy()

        # v9a 처방 1: 매 step ang_vel history 에 push (drop 시점 max ang_accel penalty 추적용)
        self._ang_vel_history.append(ang.copy())

        # === v6 옵션 6: 첫 step 시점 위치 저장 (정책 시작 시점 fact 확인용) ===
        if self._step_count == 1:
            _dx = float(pos[0]) - self._cfg_target_x
            _dy = float(pos[1]) - self._cfg_target_y
            _dz = float(pos[2]) - 4.0
            self._initial_target_dist_3d = math.sqrt(_dx*_dx + _dy*_dy + _dz*_dz)
            self._initial_target_dist_xy = math.sqrt(_dx*_dx + _dy*_dy)
            self._initial_pos_x = float(pos[0])
            self._initial_pos_y = float(pos[1])
            self._initial_pos_z = float(pos[2])
            self._initial_speed_xy = math.sqrt(vel[0]**2 + vel[1]**2)

        # --- 2D horizontal distance + CCIP predicted impact distance ---
        d_xy = self._compute_d_xy(pos)
        # v10a stage1: 3D distance to (target_x, target_y, target_z=5 호버)
        d_3d = self._compute_d_3d(pos)
        _, _, t_f, d_impact = self._predict_impact_point(pos, vel)

        # Round 5: Hover 추적 (속도 기준 연속 정체)
        speed_xy_now = math.sqrt(vel[0]**2 + vel[1]**2)
        if speed_xy_now < self._cfg_hover_speed_threshold:
            self._consecutive_still += 1
        else:
            self._consecutive_still = 0
        if self._consecutive_still > self._max_consecutive_still:
            self._max_consecutive_still = self._consecutive_still

        # --- Physics explosion guard ---
        # Catches two failure modes:
        #   a) non-finite d_xy (NaN/Inf slipping past _on_local_pos guards)
        #   b) finite but implausible d_xy (> 500 m — magnitude clamp in
        #      _on_local_pos uses 1 000 m so a residual window still exists)
        # Terminate immediately; note: 'd_xy' key intentionally omitted so
        # WandbMetricsCallback does NOT average the glitch value into
        # env/mean_d_xy. Use 'glitch_d_xy' for one-off diagnostics only.
        if not math.isfinite(d_xy) or d_xy > 500.0:
            reward = -100.0
            self._episode_reward += reward
            return self._get_obs(), reward, True, False, {
                'physics_glitch': True,
                'glitch_d_xy': float(d_xy),
                'episode_reward': self._episode_reward,
                'rew_ctrl': 0.0, 'rew_dist': 0.0,
                'rew_orient': 0.0, 'rew_drop': 0.0,
            }

        terminated = False
        truncated = False
        info = {}

        # --- RAD: Drop decision is PHASE-DEPENDENT ---
        # Phase 1: drop 완전 비활성 (sphere 진입까지 approach 만 학습)
        # Phase 2: auto_drop d_impact ≤ 1.0m (manual + random + hover_block 모두 비활성)
        random_drop = (self._step_count >= self._cfg_random_drop_start_step
                       and np.random.random() < self._cfg_random_drop_prob)
        _hover_drop_block = (math.sqrt(vel[0]**2 + vel[1]**2)
                             < self._cfg_hover_drop_block_threshold)

        # RAD: Phase 1 = drop 비활성. Phase 2 = 기존 logic.
        drop_allowed = (self._cfg_phase == 'phase2')
        if (drop_allowed
                and (random_drop or d_impact <= self._cfg_auto_drop_threshold)
                and not self.dropped and not _hover_drop_block):
            # ============================================================
            # Layer 4: Terminal drop accuracy reward (ACTUAL physics result)
            # Issue #014 해결: inline SDF + pre-spawn + PX4_GZ_MODEL_NAME
            # → DetachableJoint 정상 작동 → 실제 payload 낙하.
            # ============================================================
            self._node.publish_drop()
            self.dropped = True

            got_result = self._drop_error_event.wait(
                timeout=self._cfg_drop_wait_timeout)
            actual_error = 99.0
            if got_result:
                try:
                    actual_error = self._drop_error_queue.get_nowait()
                except queue.Empty:
                    pass
            d_error = actual_error

            # Drop attempt bonus: scaled by proximity to target.
            # Dropping far away gives ~0, dropping close gives full bonus.
            reward = self._cfg_drop_attempt_bonus * math.exp(
                -self._cfg_k_drop_proximity * d_xy)

            # Base precision reward: w_drop_base * exp(-k2 * d_error)
            reward += self._cfg_w_drop_base * math.exp(
                -self._cfg_k2_precision * d_error)

            # Prediction bonus: reward CCIP accuracy (Issue #010)
            prediction_gap = abs(d_impact - d_error)
            reward += self._cfg_w_prediction * math.exp(
                -self._cfg_k_prediction * prediction_gap)
            info['prediction_gap'] = prediction_gap

            # Jackpot: bonus for high-precision drop
            if d_error <= self._cfg_jackpot_threshold:
                reward += self._cfg_r_success_jackpot
                info['jackpot'] = True

            # 고도 페널티 (Round 3): sigmoid bounded — 지수 폐기 (-6.77e9 폭주 방지)
            # penalty = -alt_penalty_max * sigmoid(k * (alt - mid))
            alt_at_drop = float(pos[2])
            sig = 1.0 / (1.0 + math.exp(
                -self._cfg_alt_penalty_k * (alt_at_drop - self._cfg_alt_penalty_mid)))
            alt_penalty = self._cfg_alt_penalty_max * sig
            reward -= alt_penalty
            info['altitude_penalty'] = alt_penalty

            # Instability penalty: large angular velocity or excessive tilt
            omega_mag = float(np.linalg.norm(ang))
            if (omega_mag > self._cfg_limit_ang_vel
                    or abs(roll) > self._cfg_limit_tilt
                    or abs(pitch) > self._cfg_limit_tilt):
                reward -= self._cfg_penalty_instability
                info['instability_penalty'] = True

            # v9a 처방 1: drop 시점 직전 N step 의 max angular acceleration penalty
            # 측정 방법 E: deque 의 인접 step 간 diff magnitude max
            # 사용자 의도: toss 의 pitch back (= 각속도의 갑작스러운 변화) 부드럽게 유도
            max_ang_accel = 0.0
            if (self._cfg_drop_angaccel_penalty_scale > 0
                    and len(self._ang_vel_history) >= 2):
                hist = list(self._ang_vel_history)
                for i in range(1, len(hist)):
                    accel = float(np.linalg.norm(hist[i] - hist[i-1]))
                    if accel > max_ang_accel:
                        max_ang_accel = accel
                reward -= self._cfg_drop_angaccel_penalty_scale * max_ang_accel
            info['drop_max_ang_accel'] = max_ang_accel

            # === C: invalid drop 페널티 (v6 처방, drop_calculator timeout 회피 학습) ===
            # drop_error 가 임계 초과 (예: 50m+) 이면 default 99m 또는 매우 멀리.
            # 명시적 페널티로 정책이 invalid drop 회피 학습.
            if d_error > self._cfg_invalid_drop_threshold:
                reward -= self._cfg_invalid_drop_penalty
                info['invalid_drop_penalty'] = True

            info['drop_error_actual_m'] = d_error
            info['is_success'] = bool(d_error <= self._cfg_success_threshold)
            info['drop_trigger'] = 'random' if random_drop else 'auto'
            info['layer4_reward'] = reward
            # Reward component keys (consistent schema with non-terminal steps)
            info['rew_drop'] = reward      # full Layer 4 reward
            info['rew_ctrl'] = 0.0
            info['rew_dist'] = 0.0
            info['rew_orient'] = 0.0
            info['d_xy'] = d_xy            # drone was at this distance when dropped
            info['d_impact'] = d_impact    # CCIP predicted distance that triggered drop
            terminated = True

        else:
            # ============================================================
            # Layers 1–3: Per-step reward for non-terminal steps
            # ============================================================
            reward, terminated, info = self._compute_reward(
                pos, vel, ang, pix, d_xy, d_impact, action)

            # --- P11/P9 (junsang_v4): step 별 safety 검사 + truncated 결정 ---
            # 우선순위: crash > overspeed > ang_vel > inverted
            # crash/overspeed 는 _compute_reward 안에서 이미 penalty add + info['crash'/'overspeed']
            # 표기 — 여기선 truncate_reason 만 결정 (truncate=True 트리거).
            truncate_reason = None
            if info.get('crash'):
                truncated = True
                truncate_reason = 'crash'
            elif info.get('overspeed'):
                truncated = True
                truncate_reason = 'overspeed'
            elif np.linalg.norm(ang) > self._cfg_limit_ang_vel:
                info['bad_attitude'] = 'ang_vel'
                reward += self._cfg_penalty_bad_attitude
                truncated = True
                truncate_reason = 'ang_vel'
            elif (abs(roll) > self._cfg_limit_inverted_tilt
                  or abs(pitch) > self._cfg_limit_inverted_tilt):
                info['bad_attitude'] = 'inverted'
                reward += self._cfg_penalty_bad_attitude
                truncated = True
                truncate_reason = 'inverted'

            # Curriculum-aware tilt limit (stage 별 완화, inverted 아닌 경우만)
            if not truncated:
                _stage_limit_tilt = self._get_stage_limit_tilt()
                if (abs(roll) > _stage_limit_tilt
                        or abs(pitch) > _stage_limit_tilt):
                    info['bad_attitude'] = 'tilt'
                    reward += self._cfg_penalty_bad_attitude
                    truncated = True
                    truncate_reason = 'tilt'

            # Curriculum-aware max_distance (stage 별 완화)
            _stage_max_distance = self._get_stage_max_distance()
            if not truncated and d_xy > _stage_max_distance:
                reward += self._cfg_penalty_out_of_range
                truncated = True
                truncate_reason = 'out_of_range'

            if not truncated and pos[2] > self._cfg_max_altitude:
                reward += self._cfg_penalty_max_altitude
                truncated = True
                truncate_reason = 'max_altitude'

            if truncate_reason:
                info['truncate_reason'] = truncate_reason

        # --- RAD Phase 1: switch sphere 진입 검사 + terminal reward (Curriculum-aware) ---
        # sphere 진입 (d_switch² ≤ stage_switch_d_sq) 시 active conditions 검사 후 ep 종료.
        # Curriculum: stage 별 switch_d_sq 사용, active_conditions 만 검사.
        # Success = sphere 진입 AND active conditions 모두 만족.
        if (not terminated and not truncated
                and self._cfg_phase == 'phase1'):
            d_switch_sq = self._compute_d_switch_sq(pos)
            stage_switch_d_sq = self._get_stage_switch_d_sq()
            if d_switch_sq <= stage_switch_d_sq:
                # yaw_err 계산
                with self._state_lock:
                    yaw_ned = float(self._node.yaw_ned)
                yaw_enu = math.pi / 2.0 - yaw_ned
                dx_w = self._cfg_target_x - float(pos[0])
                dy_w = self._cfg_target_y - float(pos[1])
                target_dir_yaw = math.atan2(dy_w, dx_w)
                yaw_err_rad = target_dir_yaw - yaw_enu
                yaw_err_rad = math.atan2(math.sin(yaw_err_rad), math.cos(yaw_err_rad))

                # 항상 7 조건 검사 (log 용) — active_conditions 만 terminal 에 반영
                satisfied_all = self._check_phase1_final_state(pos, vel, ang, yaw_err_rad)
                active_cond = set(self._get_stage_active_conditions())
                # active 만 sum. Non-active 는 자동 pass 로 취급 (terminal reward 계산에서).
                # Curriculum stage 별로 terminal reward = floor + Σ(w × active_sat) + jackpot × all_active_sat
                satisfied_active = [
                    satisfied_all[i] if f'C{i+1}' in active_cond else True
                    for i in range(7)
                ]
                terminal_reward, all_active_sat = self._compute_phase1_terminal_reward(
                    satisfied_active)
                reward += terminal_reward
                terminated = True

                info['phase1_sphere_entered'] = True
                info['phase1_d_switch_sq'] = float(d_switch_sq)
                info['phase1_success'] = bool(all_active_sat)
                info['phase1_C_satisfied'] = satisfied_all
                info['phase1_n_satisfied'] = sum(satisfied_all)
                info['phase1_terminal_reward'] = float(terminal_reward)
                info['phase1_yaw_err_rad'] = float(yaw_err_rad)
                # Curriculum info
                info['curriculum_stage_idx'] = self._curriculum_stage_idx
                info['curriculum_active_conditions'] = list(active_cond)
                info['curriculum_stage_switch_d_sq'] = float(stage_switch_d_sq)
                # Success metric — curriculum 의 active conditions 기준
                info['is_success'] = bool(all_active_sat)
                info['rew_drop'] = float(terminal_reward)

        # --- RAD Phase 2: sphere escape crash 검사 (매 step) ---
        if (not terminated and not truncated
                and self._cfg_phase == 'phase2'):
            d_switch_sq = self._compute_d_switch_sq(pos)
            if d_switch_sq > self._cfg_phase2_sphere_escape_d_sq:
                reward += self._cfg_phase2_sphere_escape_penalty
                truncated = True
                info['truncate_reason'] = 'sphere_escape'
                info['phase2_d_switch_sq_at_escape'] = float(d_switch_sq)

        # --- Truncation on step limit (timeout) ---
        if not terminated and not truncated and self._step_count >= self._cfg_max_steps:
            truncated = True
            if not self.dropped:
                # P8 (junsang_v4): yaml truncation_penalty 사용 (이전 hardcoded -80)
                reward += self._cfg_truncation_penalty
            info['truncate_reason'] = 'timeout'

        # --- v5 안전장치 δ: hover 강제 truncate ---
        # _consecutive_still 가 threshold 도달 즉시 episode 종료.
        # (기존 episode-end 페널티 로직과 합쳐져 페널티는 아래에서 한 번에 부여)
        if (not terminated and not truncated
                and self._cfg_hover_truncate_enabled
                and self._consecutive_still > self._cfg_hover_consecutive_threshold):
            truncated = True
            info['truncate_reason'] = 'hover_timeout'

        # Round 5: Hover 페널티 — episode 종료 시 (drop 제외)
        # 연속 정체 step이 임계 초과 시 한 번에 -15
        # Drop 시 (terminated) 제외 — drop을 위한 정밀 hover는 정당
        if truncated and not terminated and not self.dropped:
            if self._max_consecutive_still > self._cfg_hover_consecutive_threshold:
                reward += self._cfg_penalty_hover
                info['hover_penalty'] = True
                info['max_consecutive_still'] = self._max_consecutive_still

        # --- Update action memory ---
        self.action_prev = np.array(action, dtype=np.float32)

        # === Curriculum: track ep result + auto-progress ===
        if self._cfg_curriculum_enabled and (terminated or truncated):
            self._curriculum_ep_history.append(
                (self._total_step_count, bool(info.get('is_success', False))))
            self._curriculum_check_progress(info)

        # Round 3: Hard cap + warning — outlier 폭주 차단 (방어 안전망)
        # 이론 최대: drop bonus 200, max penalty ~100. 범위 밖이면 버그.
        REWARD_HARD_MAX = 300.0
        REWARD_HARD_MIN = -200.0
        if reward > REWARD_HARD_MAX or reward < REWARD_HARD_MIN:
            print(f'[WARN] reward out of range: {reward:.2f} at step '
                  f'{self._step_count} (info={info.get("truncate_reason", "")})')
            reward = float(np.clip(reward, REWARD_HARD_MIN, REWARD_HARD_MAX))

        obs = self._get_obs()
        self._episode_reward += reward
        if terminated or truncated:
            info['episode_reward'] = self._episode_reward
            # === Issue #028 diag: terminal type 분류 (H1 vs H3 vs H5 구별) ===
            if info.get('phase1_sphere_entered'):
                tt = 'entry_success' if info.get('phase1_success') else 'entry_partial_fail'
            elif info.get('crash'):
                tt = 'crash'
            elif info.get('bad_attitude'):
                tt = f'bad_att_{info["bad_attitude"]}'
            elif 'truncate_reason' in info:
                tt = f'trunc_{info["truncate_reason"]}'
            else:
                tt = 'unknown'
            info['terminal_type'] = tt
            info['ep_reward_sum'] = float(self._episode_reward)
            info['ep_len'] = int(self._step_count)
            # v6 옵션 6: 초기 위치/속도 (정책 시작 시점 fact)
            if hasattr(self, '_initial_target_dist_3d'):
                info['initial_target_dist_3d'] = float(self._initial_target_dist_3d)
                info['initial_target_dist_xy'] = float(self._initial_target_dist_xy)
                info['initial_pos_x'] = float(self._initial_pos_x)
                info['initial_pos_y'] = float(self._initial_pos_y)
                info['initial_pos_z'] = float(self._initial_pos_z)
                info['initial_speed_xy'] = float(self._initial_speed_xy)

        return obs, reward, terminated, truncated, info

    def close(self):
        """Shutdown episode nodes, infrastructure, and ROS2."""
        self._kill_episode()
        self._kill_infra()
        if rclpy.ok():
            self._node.destroy_node()
            rclpy.shutdown()

    def reset_phase2_state(self):
        """RAD: Phase 1 rollout 종료 후 Phase 2 step 카운트 reset (env.reset() 안 함).

        train_sac_rad 의 Phase 2 학습 wrapper 가 매 ep 시작 시:
          1. env.reset() — cruise 완료까지
          2. env._cfg_phase = 'phase1' 일시 변경
          3. Phase 1 정책으로 sphere 진입까지 rollout (env.step() 반복)
          4. sphere 진입 (phase1_sphere_entered=True) → env.step() terminate
          5. env._cfg_phase = 'phase2' 복원
          6. env.reset_phase2_state() — Phase 2 ep state reset
          7. wrapper.reset() 가 새 obs 반환 → SB3 train loop 가 Phase 2 정책 control
        """
        self._step_count = 0
        self._episode_reward = 0.0
        self.dropped = False
        self._consecutive_still = 0
        self._max_consecutive_still = 0
        self._ang_vel_history.clear()
        with self._state_lock:
            pos = self._node.pos_enu.copy()
        # d_xy_prev seed (sphere 진입 시점의 d_reward)
        self.d_xy_prev = self._compute_d_reward(pos)
        self.action_prev = np.zeros(5, dtype=np.float32)

    # ------------------------------------------------------------------
    # Kinematic predictor — AeroThrow projectile physics
    # ------------------------------------------------------------------

    def _predict_impact_point(self, pos, vel):
        """Predict where the payload will land if released right now.

        Solves the 1-D kinematic equation for time-of-flight t_f:
            z(t) = z_0 + vz_0 * t_f - 0.5 * g * t_f^2 = 0   (target at z = 0)

        Rearranged as the quadratic:
            0.5*g * t_f^2  -  vz_0 * t_f  -  z_0  =  0

        Discriminant:  D = vz_0^2 + 2*g*z_0  (always >= 0 when z_0 >= 0)

        Positive physical root:
            t_f = (vz_0 + sqrt(D)) / g

        Args:
            pos: ENU position [x, y, z] in metres (z = altitude above ground)
            vel: ENU velocity [vx, vy, vz] in m/s

        Returns:
            x_p (float):      predicted impact x (metres, ENU)
            y_p (float):      predicted impact y (metres, ENU)
            t_f (float):      time of flight (seconds)
            d_impact (float): horizontal miss distance to target (metres)
        """
        x, y, z = float(pos[0]), float(pos[1]), float(pos[2])
        vx, vy, vz = float(vel[0]), float(vel[1]), float(vel[2])
        g = self._cfg_g

        if z <= 0.0:
            # Already at/below ground — impact is the current position
            t_f = 0.0
        else:
            # D = vz^2 + 2*g*z >= 0 guaranteed when z > 0
            discriminant = vz * vz + 2.0 * g * z
            t_f = (vz + math.sqrt(discriminant)) / g
            if t_f < 0.0:
                t_f = 0.0   # numerical guard (should not occur for z > 0)
            t_f = min(t_f, 10.0)  # clamp: at vz≈0, z=490m → t_f≈10s; beyond this obs diverges

        x_p = x + vx * t_f
        y_p = y + vy * t_f
        dx = x_p - self._cfg_target_x
        dy = y_p - self._cfg_target_y
        d_impact = math.sqrt(dx * dx + dy * dy)

        return x_p, y_p, t_f, d_impact

    def _compute_d_3d(self, pos):
        """v10a stage1: 3D Euclidean distance to (target_x, target_y, target_z).

        target_z = 5.0 (호버링 z) by default. 단계 1 의 호버 도달 검사용.
        """
        dx = float(pos[0]) - self._cfg_target_x
        dy = float(pos[1]) - self._cfg_target_y
        dz = float(pos[2]) - self._cfg_target_z
        return math.sqrt(dx * dx + dy * dy + dz * dz)

    def _compute_d_xy(self, pos):
        """2D horizontal distance from drone to target (kept for logging only)."""
        dx = float(pos[0]) - self._cfg_target_x
        dy = float(pos[1]) - self._cfg_target_y
        return math.sqrt(dx * dx + dy * dy)

    def _compute_d_reward(self, pos):
        """RAD: w_dist 의 거리 계산. reference = (target_x, target_y, Z_REWARD_REFERENCE=4).

        d_reward = √((x_drone − target_x)² + (y_drone − target_y)² + (z_drone − 4)²)

        z=4 가 reward maximum 위치 (drone 이 최소 안전 고도 위 1m 에서 drop 유도).
        """
        dx = float(pos[0]) - self._cfg_target_x
        dy = float(pos[1]) - self._cfg_target_y
        dz = float(pos[2]) - Z_REWARD_REFERENCE
        return math.sqrt(dx * dx + dy * dy + dz * dz)

    def _compute_d_switch_sq(self, pos):
        """RAD: switch sphere (Phase 1 → Phase 2 trigger 또는 Phase 2 escape).

        d_switch² = (x − target_x)² + (y − target_y)² + (z − target_z)²
        target_z = 0 (지면 marker). target 까지의 실제 3D 거리 제곱.
        """
        dx = float(pos[0]) - self._cfg_target_x
        dy = float(pos[1]) - self._cfg_target_y
        dz = float(pos[2]) - self._cfg_target_z   # target_z = 0 (RAD)
        return dx * dx + dy * dy + dz * dz

    # === Curriculum helpers (Issue #028 fix) ===
    def _get_current_stage_cfg(self):
        """Return current curriculum stage config dict. If disabled/no stages, return None (use design defaults)."""
        if (not self._cfg_curriculum_enabled
                or not self._cfg_curriculum_stages
                or self._curriculum_stage_idx >= len(self._cfg_curriculum_stages)):
            return None
        return self._cfg_curriculum_stages[self._curriculum_stage_idx]

    def _get_stage_switch_d_sq(self):
        """Current stage 의 sphere d² threshold. Design default = self._cfg_phase1_switch_d_sq."""
        stage = self._get_current_stage_cfg()
        if stage is None:
            return self._cfg_phase1_switch_d_sq
        return float(stage.get('switch_d_sq', self._cfg_phase1_switch_d_sq))

    def _get_stage_active_conditions(self):
        """Current stage 의 active final state conditions. list of str ('C1', 'C4' etc).
        Design default = ['C1', 'C2', 'C3', 'C4', 'C5', 'C6', 'C7']."""
        stage = self._get_current_stage_cfg()
        if stage is None:
            return ['C1', 'C2', 'C3', 'C4', 'C5', 'C6', 'C7']
        return list(stage.get('active_conditions', []))

    def _get_stage_z_min(self):
        stage = self._get_current_stage_cfg()
        if stage is None:
            return self._cfg_ground_contact_alt
        return float(stage.get('z_min', self._cfg_ground_contact_alt))

    def _get_stage_max_distance(self):
        stage = self._get_current_stage_cfg()
        if stage is None:
            return self._cfg_max_distance
        return float(stage.get('max_distance', self._cfg_max_distance))

    def _get_stage_limit_tilt(self):
        stage = self._get_current_stage_cfg()
        if stage is None:
            return self._cfg_limit_tilt
        return float(stage.get('limit_tilt', self._cfg_limit_tilt))

    def _curriculum_check_progress(self, info=None):
        """매 ep 종료 후 호출. window success rate 확인 후 stage 진행 결정.

        진행 조건:
          1. 현재 stage 가 마지막 stage 아님
          2. current stage 에서 진행한 step >= curriculum_min_stage_steps
          3. window 안의 success rate >= curriculum_success_threshold
        """
        if not self._cfg_curriculum_enabled:
            return
        # 마지막 stage 이면 진행 안 함
        if self._curriculum_stage_idx >= len(self._cfg_curriculum_stages) - 1:
            return
        # Min stage steps 확보 (total step 기준)
        steps_in_stage = self._total_step_count - self._curriculum_stage_step_start
        if steps_in_stage < self._cfg_curriculum_min_stage_steps:
            return
        # Window 안의 ep 만 계산 (total step 기준)
        window_start_step = self._total_step_count - self._cfg_curriculum_window_size
        in_window = [s for (step, s) in self._curriculum_ep_history if step >= window_start_step]
        # DEBUG every 1000 total_steps
        if self._total_step_count % 1000 < 100:
            print(f'[CURRICULUM_CHECK] total={self._total_step_count} '
                  f'stage={self._curriculum_stage_idx} '
                  f'in_window={len(in_window)} '
                  f'history_len={len(self._curriculum_ep_history)}', flush=True)
        if len(in_window) < 10:
            return  # window 안 ep 수 부족 (통계 신뢰도 ↓)
        success_rate = sum(in_window) / len(in_window)
        if success_rate >= self._cfg_curriculum_success_threshold:
            old_stage_name = self._cfg_curriculum_stages[self._curriculum_stage_idx].get(
                'name', f'stage{self._curriculum_stage_idx}')
            self._curriculum_stage_idx += 1
            new_stage_name = self._cfg_curriculum_stages[self._curriculum_stage_idx].get(
                'name', f'stage{self._curriculum_stage_idx}')
            self._curriculum_stage_step_start = self._total_step_count
            self._curriculum_ep_history.clear()
            if rclpy.ok():
                self._node.get_logger().info(
                    f'[CURRICULUM] Advance: {old_stage_name} → {new_stage_name} '
                    f'(total_step={self._total_step_count}, window_success={success_rate:.3f})')
            print(f'[CURRICULUM_ADVANCE] total_step={self._total_step_count} '
                  f'stage={self._curriculum_stage_idx} name={new_stage_name} '
                  f'window_success={success_rate:.3f}', flush=True)
            if info is not None:
                info['curriculum_advanced'] = True
                info['curriculum_new_stage'] = self._curriculum_stage_idx
            return

        # === Regression (뒤로 돌아가기) ===
        if not self._cfg_curriculum_regression_enabled:
            return
        # First stage 는 rollback 불가
        if self._curriculum_stage_idx <= 0:
            return
        # 진입 후 최소 시간 확보 (정책 적응 시간)
        if steps_in_stage < self._cfg_curriculum_regression_min_steps:
            return
        # Cooldown (이전 regression 후 최소 시간)
        steps_since_regression = self._total_step_count - self._curriculum_last_regression_step
        if steps_since_regression < self._cfg_curriculum_regression_cooldown:
            return
        # Success rate 낮으면 regression
        if success_rate < self._cfg_curriculum_regression_threshold:
            old_stage_name = self._cfg_curriculum_stages[self._curriculum_stage_idx].get(
                'name', f'stage{self._curriculum_stage_idx}')
            self._curriculum_stage_idx -= 1
            new_stage_name = self._cfg_curriculum_stages[self._curriculum_stage_idx].get(
                'name', f'stage{self._curriculum_stage_idx}')
            self._curriculum_stage_step_start = self._total_step_count
            self._curriculum_last_regression_step = self._total_step_count
            self._curriculum_ep_history.clear()
            if rclpy.ok():
                self._node.get_logger().info(
                    f'[CURRICULUM] Regress: {old_stage_name} → {new_stage_name} '
                    f'(total_step={self._total_step_count}, window_success={success_rate:.3f})')
            print(f'[CURRICULUM_REGRESS] total_step={self._total_step_count} '
                  f'stage={self._curriculum_stage_idx} name={new_stage_name} '
                  f'window_success={success_rate:.3f}', flush=True)
            if info is not None:
                info['curriculum_regressed'] = True
                info['curriculum_new_stage'] = self._curriculum_stage_idx

    def _compute_z_hann_reward(self, z):
        """RAD: z reward (Hann raised cosine, z=4 max, [0.5, 7.5] 외 = 0).

        r_z = w_z × 0.5 × (1 + cos(π × (z − 4) / 3.5))   for z ∈ [0.5, 7.5]
            = 0                                            otherwise

        gradient 학습 친화적 (미분 가능 + 양끝 정확히 0).
        """
        if self._cfg_w_z <= 0.0:
            return 0.0
        z_offset = z - self._cfg_z_target
        if abs(z_offset) > self._cfg_z_half_range:
            return 0.0
        return self._cfg_w_z * 0.5 * (1.0 + math.cos(
            math.pi * z_offset / self._cfg_z_half_range))

    def _check_phase1_final_state(self, pos, vel, ang, yaw_err_rad):
        """RAD: Phase 1 final state 7 조건 검사 (sphere 진입 시점).

        Returns:
            satisfied (list[bool]): 길이 7, [C1, C2, C3, C4, C5, C6, C7]
        """
        v_xy = math.sqrt(float(vel[0])**2 + float(vel[1])**2)
        v_z = abs(float(vel[2]))
        # tilt = max(|roll|, |pitch|)
        with self._state_lock:
            tilt = max(abs(self._node.roll), abs(self._node.pitch))
        omega = float(np.linalg.norm(ang))
        z = float(pos[2])

        return [
            self._cfg_p1_C1_z_min <= z <= self._cfg_p1_C1_z_max,   # C1
            v_xy <= self._cfg_p1_C2_v_xy_max,                       # C2
            v_z <= self._cfg_p1_C3_v_z_max,                         # C3
            tilt <= self._cfg_p1_C4_tilt_max,                       # C4
            omega <= self._cfg_p1_C5_omega_max,                     # C5
            abs(yaw_err_rad) <= self._cfg_p1_C6_yaw_err_max,        # C6
            v_xy >= self._cfg_p1_C7_v_xy_min,                       # C7
        ]

    def _compute_phase1_terminal_reward(self, satisfied):
        """RAD: Phase 1 terminal reward 계산.

        reward = floor + Σ(w_i × satisfied_i) + complete_bonus × ∏(satisfied_i)

        모두 만족: floor 20 + 50 + 50 = 120
        6/7 만족: floor 20 + 50 × 6/7 ≈ 62.9 (jackpot 미발동)
        0/7 만족: floor 20 (sphere 진입 자체 보상)
        """
        n_satisfied = sum(satisfied)
        all_satisfied = (n_satisfied == 7)

        reward = self._cfg_phase1_terminal_floor
        reward += self._cfg_phase1_terminal_w_each * n_satisfied
        if all_satisfied:
            reward += self._cfg_phase1_terminal_complete_bonus
        return reward, all_satisfied

    # ------------------------------------------------------------------
    # Reward computation — Layers 1, 2, 3 (non-terminal steps)
    # ------------------------------------------------------------------

    def _compute_reward(self, pos, vel, ang, pix, d_xy, d_impact, action):
        """RAD per-step reward computation.

        Layer 1: Safety penalty (crash, overspeed, target lost)
        Layer 2: w_time (phase 별), w_ang_vel (phase 별), w_action_smooth (phase 별)
        Layer 3: w_dist (3D, d_reward=√(dx²+dy²+(z−4)²)) + w_heading (수평만) +
                 w_impact (phase 별, Phase 1 dead) + z Hann

        Args:
            d_xy:     2D horizontal distance to target (logging only)
            d_impact: CCIP predicted miss distance (from _predict_impact_point)

        Returns:
            reward (float), terminated (bool), info (dict)
        """
        info = {}
        reward = 0.0

        # ----------------------------------------------------------------
        # Layer 1 — Safety
        # ----------------------------------------------------------------
        altitude = float(pos[2])
        speed = float(np.linalg.norm(vel))
        conf = float(pix[2])

        # Curriculum-aware z_min (stage 별 완화)
        _stage_z_min = self._get_stage_z_min()
        if altitude < _stage_z_min:
            info['crash'] = True
            reward += self._cfg_penalty_crash
        elif self._step_count > self._cfg_min_alt_start and altitude < self._cfg_min_altitude:
            info['crash'] = True
            reward += self._cfg_penalty_crash

        if speed > self._V_MAX_SAFETY:
            info['overspeed'] = True
            reward += self._cfg_penalty_overspeed

        if self._cfg_use_vision and conf == 0.0:
            info['target_lost'] = True
            reward += self._cfg_penalty_target_lost

        # ----------------------------------------------------------------
        # Layer 2 — Stability (RAD: w_time/w_ang_vel/w_action_smooth phase 별)
        # ----------------------------------------------------------------
        omega_sq = float(np.dot(ang, ang))

        action_arr = np.asarray(action, dtype=np.float32)
        delta_action = action_arr - self.action_prev
        action_smooth_sq = float(np.dot(delta_action, delta_action))

        # w_time 은 cfg 값이 음수 (-0.05 또는 -0.1) 이므로 그대로 더함
        r2 = (
            self._cfg_w_time
            - self._cfg_w_ang_vel * omega_sq
            - self._cfg_w_action_smooth * action_smooth_sq
        )

        # ----------------------------------------------------------------
        # Layer 3 — Approach (RAD: d_reward 3D + z Hann)
        # ----------------------------------------------------------------
        # RAD: w_dist 의 거리 = d_reward (3D, (target_x, target_y, 4))
        d_reward = self._compute_d_reward(pos)
        r3_dist = self._cfg_w_dist * (self.d_xy_prev - d_reward)
        # d_xy_prev 의 이름은 v8 호환 위해 유지, 실제로는 d_reward 의 prev 값
        self.d_xy_prev = d_reward

        # Heading alignment (수평만, v8 그대로)
        vx_2d, vy_2d = float(vel[0]), float(vel[1])
        speed_xy = math.sqrt(vx_2d * vx_2d + vy_2d * vy_2d)
        if speed_xy > 0.1:
            dx_to_target = self._cfg_target_x - float(pos[0])
            dy_to_target = self._cfg_target_y - float(pos[1])
            dist_to_target = math.sqrt(dx_to_target ** 2 + dy_to_target ** 2)
            if dist_to_target > 0.01:
                cos_heading = (
                    (vx_2d * dx_to_target + vy_2d * dy_to_target)
                    / (speed_xy * dist_to_target)
                )
            else:
                cos_heading = 1.0
        else:
            cos_heading = 0.0

        speed_gate = min(speed_xy / 2.0, 1.0) if self._cfg_speed_gate else 1.0
        r3_orient = self._cfg_w_heading * cos_heading * speed_gate

        # w_impact (Phase 1 = 0, Phase 2 = 1.0)
        r3_impact = self._cfg_w_impact * math.exp(-self._cfg_k_impact * d_impact)

        # RAD 신규: z Hann reward (Phase 1 + Phase 2 모두 활성)
        r3_z_hann = self._compute_z_hann_reward(altitude)

        # distance_penalty (v8: 0 dead)
        r3_distance_penalty = -self._cfg_w_distance_penalty * d_xy / 50.0

        r3 = r3_dist + r3_orient + r3_impact + r3_z_hann + r3_distance_penalty

        reward += r2 + r3
        info['r2'] = r2
        info['r3'] = r3
        info['d_xy'] = d_xy
        info['d_reward'] = d_reward
        info['d_impact'] = d_impact
        info['cos_heading'] = cos_heading
        info['rew_ctrl'] = r2
        info['rew_dist'] = r3_dist
        info['rew_orient'] = r3_orient
        info['rew_impact'] = r3_impact
        info['rew_z_hann'] = r3_z_hann
        info['rew_drop'] = 0.0

        return reward, False, info

    # ------------------------------------------------------------------
    # Observation builder
    # ------------------------------------------------------------------

    def _get_obs(self):
        """RAD: 14d observation in yaw-only body frame.

        Yaw-only body frame:
          - drone yaw 만큼 회전, pitch/roll 은 별도 obs
          - drone heading (+x_body) 기준 target 의 상대 위치, drone velocity 표현
          - yaw 절대값 obs 제외 → 정책이 yaw-invariant 학습

        ENU yaw 계산:
          PX4 NED yaw = z-axis 회전 (NED 기준). ENU yaw 변환:
            yaw_enu = π/2 - yaw_ned   (modulo 2π)
          (NED north = ENU +Y, NED east = ENU +X)

        Body frame 변환 (수평만):
          [Δx_b]   [ cos(yaw_enu)  sin(yaw_enu)] [Δx_w]
          [Δy_b] = [-sin(yaw_enu)  cos(yaw_enu)] [Δy_w]
          Δz_world (회전 안 함)
        """
        with self._state_lock:
            pos = self._node.pos_enu.copy()
            vel = self._node.vel_enu.copy()
            ang = self._node.ang_vel.copy()
            attached = float(self._node.payload_attached)
            roll = float(self._node.roll)
            pitch = float(self._node.pitch)
            yaw_ned = float(self._node.yaw_ned)

        # NaN guard
        pos_clean = np.nan_to_num(pos, nan=0.0)
        vel_clean = np.nan_to_num(vel, nan=0.0)
        ang_clean = np.nan_to_num(ang, nan=0.0)

        # ENU yaw 변환 (NED yaw → ENU yaw)
        # NED frame: x=North, y=East, z=Down. ENU: x=East, y=North, z=Up.
        # NED yaw 가 +X_ned (North) 향함 → ENU yaw 로 변환: yaw_enu = π/2 - yaw_ned
        yaw_enu = math.pi / 2.0 - yaw_ned

        # 상대 위치 (target - drone) — yaw-only body frame
        dx_w = self._cfg_target_x - pos_clean[0]
        dy_w = self._cfg_target_y - pos_clean[1]
        dz_w = self._cfg_target_z - pos_clean[2]   # target_z = 0 (지면)

        cos_yaw = math.cos(yaw_enu)
        sin_yaw = math.sin(yaw_enu)
        dx_b = cos_yaw * dx_w + sin_yaw * dy_w
        dy_b = -sin_yaw * dx_w + cos_yaw * dy_w
        # dz_w 회전 안 함

        # 상대 속도 (drone vel) — yaw-only body frame
        vx_b = cos_yaw * vel_clean[0] + sin_yaw * vel_clean[1]
        vy_b = -sin_yaw * vel_clean[0] + cos_yaw * vel_clean[1]
        vz_w = vel_clean[2]

        # 정규화 + clipping
        pos_scale = self._cfg_pos_scale
        vel_scale = self._cfg_vel_scale
        ang_vel_scale = self._cfg_ang_vel_scale
        rel_pos_b = np.array([dx_b / pos_scale, dy_b / pos_scale, dz_w / pos_scale])
        vel_b = np.array([vx_b / vel_scale, vy_b / vel_scale, vz_w / vel_scale])
        ang_n = ang_clean / ang_vel_scale

        # CCIP (Phase 1 dead, Phase 2 active — 단 식은 동일)
        _, _, t_f_obs, d_impact_obs = self._predict_impact_point(pos_clean, vel_clean)
        obs_d_impact = float(np.clip(d_impact_obs / pos_scale, 0.0, 1.0))
        obs_t_f = float(np.clip(t_f_obs / 10.0, 0.0, 1.0))

        # Phase 1 dead: payload_attached, d_impact, t_f 의 값은 사용하지만
        # 정책이 Phase 1 학습 시 거의 일정 (attached=1, d_impact=0, t_f=0).
        # Phase 2 warm start init 시 같은 dim 의 obs.

        obs = np.array([
            np.clip(rel_pos_b[0], -1.0, 1.0),   # 0  Δx_b (body frame)
            np.clip(rel_pos_b[1], -1.0, 1.0),   # 1  Δy_b
            np.clip(rel_pos_b[2], -1.0, 1.0),   # 2  Δz_world / POS_SCALE
            np.clip(vel_b[0], -1.0, 1.0),       # 3  vx_b
            np.clip(vel_b[1], -1.0, 1.0),       # 4  vy_b
            np.clip(vel_b[2], -1.0, 1.0),       # 5  vz_world / VEL_SCALE
            np.clip(ang_n[0], -1.0, 1.0),       # 6  ωx
            np.clip(ang_n[1], -1.0, 1.0),       # 7  ωy
            np.clip(ang_n[2], -1.0, 1.0),       # 8  ωz
            np.clip(roll / math.pi, -1.0, 1.0), # 9  roll / π
            np.clip(pitch / math.pi, -1.0, 1.0),# 10 pitch / π
            attached,                            # 11 payload_attached (Phase 1: 1.0)
            obs_d_impact,                        # 12 d_impact / POS_SCALE (Phase 1: 0)
            obs_t_f,                             # 13 t_f / 10 (Phase 1: 0)
        ], dtype=np.float32)
        return obs

    # ------------------------------------------------------------------
    # Infrastructure lifecycle (self-managed per instance)
    # ------------------------------------------------------------------

    # Shared Gazebo flag file — instance 0 creates it after Gazebo is ready;
    # instances > 0 wait for it before starting PX4.
    _GZ_READY_FLAG = '/tmp/drone_env_gz_ready'

    def _check_infra_healthy(self, iid):
        """Return True if Gazebo + PX4 + drone model are already running.

        Used by _start_infra() to decide whether to reuse existing infra
        (avoiding the PX4 sensor re-init delay that causes preflight fails).

        Round 7 resilience: if `gz model --list` times out, Gazebo is
        unresponsive → return False so caller triggers full restart instead
        of raising. Previously the exception propagated up and killed training
        (4 historical crashes: 2026-05-28, 5-30, 5-31, 6-2).
        """
        gz_up = subprocess.run(
            ['pgrep', '-f', 'gz sim'], capture_output=True).returncode == 0
        px4_up = subprocess.run(
            ['pgrep', '-f', 'bin/px4'], capture_output=True).returncode == 0
        if not (gz_up and px4_up):
            return False
        # Check drone model exists in Gazebo (pre-spawned in world SDF)
        try:
            result = subprocess.run(
                ['gz', 'model', '--list'],
                capture_output=True, text=True, timeout=5.0)
        except subprocess.TimeoutExpired:
            if rclpy.ok():
                self._node.get_logger().warning(
                    '[RL Env] gz model --list timed out — infra treated as'
                    ' unhealthy (triggers full restart)')
            return False
        return self._model_name in result.stdout

    def _spin_loop(self):
        """Resilient spin loop: catches exceptions and restarts rclpy.spin.

        rclpy.spin() can throw if the underlying DDS/RMW layer encounters an
        error (e.g. Gazebo crashes, network issues). Running it bare in a
        thread means the thread silently dies and no more ROS2 callbacks are
        processed, causing mission_state to freeze and CRUISE timeout loops.

        This wrapper catches those exceptions and re-enters spin so the thread
        stays alive as long as the node and rclpy context are healthy.
        """
        while rclpy.ok():
            try:
                rclpy.spin(self._node)
                break  # spin() returned cleanly (node was destroyed)
            except Exception as exc:  # noqa: BLE001
                if rclpy.ok():
                    try:
                        self._node.get_logger().error(
                            f'[RL Env] Spin thread exception: {exc!r} — restarting spin')
                    except Exception:  # noqa: BLE001
                        pass
                    time.sleep(0.5)
                else:
                    break

    def _start_infra(self):
        """Launch shared Gazebo (instance 0 only), per-instance MicroXRCEAgent
        and PX4 SITL.

        All instances share ONE Gazebo process (no GZ_PARTITION — it breaks
        PX4 lockstep with Gazebo Harmonic). Each PX4 spawns its own model
        at a unique Y-offset position. ROS_DOMAIN_ID isolates ROS2 topics.
        """
        iid = self._instance_id
        infra_env = os.environ.copy()
        infra_env['ROS_DOMAIN_ID'] = str(iid)
        infra_env['XDG_RUNTIME_DIR'] = '/tmp/runtime-root'
        # No GZ_PARTITION — shared Gazebo for all instances

        models_dir = '/workspace/gazebo_models'
        px4_dir = '/opt/PX4-Autopilot'
        gz_resource_path = ':'.join([
            models_dir,
            f'{px4_dir}/Tools/simulation/gz/models',
            f'{px4_dir}/Tools/simulation/gz/worlds',
        ])
        infra_env['GZ_SIM_RESOURCE_PATH'] = gz_resource_path
        os.makedirs('/tmp/runtime-root', exist_ok=True)

        self._node.get_logger().info(
            f'[Instance {iid}] Starting infra (port={self._uxrce_port})...')

        # 0a. Always kill orphaned episode nodes from any previous run.
        #    These are NOT tracked by the current Python instance's
        #    _episode_procs list so _kill_episode() cannot reach them.
        #    Multiple drone_controllers compete for PX4 arm/disarm and
        #    OFFBOARD setpoints, breaking every episode.
        for pattern in ['mission_manager_node', 'drone_controller.*controller',
                        'drop_calculator.*calculator']:
            subprocess.run(['pkill', '-f', pattern],
                           capture_output=True, check=False)

        # 0b. Check if existing Gazebo + PX4 + MicroXRCEAgent are healthy.
        #    If healthy (drone model exists in Gazebo), REUSE them.
        #    NEVER kill and restart PX4 if it's already running:
        #      - PX4 sensor init takes 5-10 s (GPS, IMU, Mag via Gazebo topics).
        #      - Killing PX4 after it has initialized sensors forces a fresh
        #        init → EKF "missing data" / "Compass missing" preflight fails
        #        → arm refused for 5-10 s → CRUISE timeout on every episode.
        #    Reusing avoids this by keeping the warm, sensor-initialized PX4.
        if self._check_infra_healthy(iid):
            self._node.get_logger().info(
                f'[Instance {iid}] Existing Gazebo+PX4 healthy — reusing.')
            # Not started by us → don't kill in _kill_infra
            self._infra_procs = []
            # Ensure MicroXRCEAgent is running on our port
            uxrce_up = subprocess.run(
                ['pgrep', '-f', f'MicroXRCEAgent.*{self._uxrce_port}'],
                capture_output=True).returncode == 0
            if not uxrce_up:
                uxrce = subprocess.Popen(
                    ['MicroXRCEAgent', 'udp4', '-p', str(self._uxrce_port)],
                    env=infra_env,
                    stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL,
                    preexec_fn=os.setsid)
                self._infra_procs.append(uxrce)
                time.sleep(3.0)
            self._node.get_logger().info(
                f'[Instance {iid}] Waiting for PX4 position data...')
            deadline = time.time() + 90.0
            while not self._obs_ready.is_set() and time.time() < deadline:
                time.sleep(1.0)
            if self._obs_ready.is_set():
                self._node.get_logger().info(
                    f'[Instance {iid}] PX4 ready — infra up (reused)')
            else:
                self._node.get_logger().error(
                    f'[Instance {iid}] Timed out on reused infra — '
                    f'falling through to fresh start')
                # Fall through to fresh start below (reset obs_ready first)
                self._obs_ready.clear()
            if self._obs_ready.is_set():
                return

        # Infra not healthy or timed out — kill everything and start fresh.
        # Kill MicroXRCEAgent on our port, Gazebo (if iid==0), bridge, PX4.
        # PX4 must be killed here because Gazebo is being restarted; PX4
        # without Gazebo is unhealthy anyway.
        self._node.get_logger().info(
            f'[Instance {iid}] No healthy infra found — starting fresh.')
        if iid == 0:
            for pattern in ['gz sim', 'parameter_bridge']:
                subprocess.run(['pkill', '-f', pattern],
                               capture_output=True, check=False)
        subprocess.run(
            ['pkill', '-f', f'MicroXRCEAgent.*{self._uxrce_port}'],
            capture_output=True, check=False)
        subprocess.run(['pkill', '-f', 'bin/px4'],
                       capture_output=True, check=False)
        for f_glob in ['/dev/shm/fastrtps_*', f'/tmp/px4_lock-{iid}',
                       f'/tmp/px4-sock-{iid}']:
            subprocess.run(['rm', '-f', f_glob],
                           capture_output=True, check=False)
        time.sleep(2.0)

        # 1. MicroXRCEAgent (per-instance, unique port)
        uxrce = subprocess.Popen(
            ['MicroXRCEAgent', 'udp4', '-p', str(self._uxrce_port)],
            env=infra_env,
            stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL,
            preexec_fn=os.setsid,
        )

        # 2. Shared Gazebo (instance 0 only) + per-instance ros_gz_bridge
        #    Method A: each instance runs its own bridge with instance-specific
        #    topic mappings (payload_N odometry, x500_N drop command).
        gz = None
        if iid == 0:
            # Remove stale ready flag
            try:
                os.remove(self._GZ_READY_FLAG)
            except FileNotFoundError:
                pass

            worlds_dir = os.path.join(models_dir, 'worlds')
            # Read base world SDF and inject drone pre-spawn.
            # Issue #014: drone must be pre-spawned so DetachableJoint
            # forms joint with payload before physics starts.
            base_sdf_path = os.path.join(worlds_dir, 'x_marker_world.sdf')
            with open(base_sdf_path, 'r') as _f:
                _sdf = _f.read()

            # Inject drone include before </world> tag
            iid_y = iid * 150
            drone_include = (
                f'\n    <!-- Pre-spawned drone (injected by drone_drop_env.py) -->\n'
                f'    <include>\n'
                f'      <uri>model://x500_bombard</uri>\n'
                f'      <name>{self._gz_model_name}</name>\n'
                f'      <pose>0 {iid_y} 0.24 0 0 0</pose>\n'
                f'    </include>\n'
            )
            _sdf = _sdf.replace('</world>', drone_include + '  </world>')

            if self._cfg_sim_speed_factor != 1:
                _sdf = _sdf.replace(
                    '<real_time_factor>1</real_time_factor>',
                    f'<real_time_factor>{self._cfg_sim_speed_factor}</real_time_factor>')

            world_sdf = f'/tmp/x_marker_world_rl_{iid}.sdf'
            with open(world_sdf, 'w') as _f:
                _f.write(_sdf)
            gz_log = open(f'/tmp/gz_{iid}.log', 'w')
            gz = subprocess.Popen(
                ['gz', 'sim', '-s', world_sdf],
                env=infra_env,
                stdout=gz_log, stderr=subprocess.STDOUT,
                preexec_fn=os.setsid,
            )
            time.sleep(10)

            # Signal other instances that Gazebo is ready
            with open(self._GZ_READY_FLAG, 'w') as f:
                f.write('ready')
        else:
            # Wait for instance 0 to start Gazebo
            deadline = time.time() + 120.0
            while not os.path.exists(self._GZ_READY_FLAG):
                if time.time() > deadline:
                    raise RuntimeError(
                        f'Instance {iid}: Gazebo ready flag not found after 120s')
                time.sleep(1.0)
            # Brief extra wait for Gazebo services to stabilize
            time.sleep(2)

        # Per-instance ros_gz_bridge: maps instance-specific gz topics to generic
        # ROS2 topics in this instance's domain. Each instance needs its own bridge
        # to isolate payload_N odometry and x500_N drop command from other instances.
        bridge_cfg_path = f'/tmp/ros_gz_bridge_{iid}.yaml'
        bridge_cfg_content = (
            f'- ros_topic_name: /clock\n'
            f'  gz_topic_name: /clock\n'
            f'  ros_type_name: rosgraph_msgs/msg/Clock\n'
            f'  gz_type_name: gz.msgs.Clock\n'
            f'  direction: GZ_TO_ROS\n'
            f'\n'
            f'- ros_topic_name: /drone/payload/position\n'
            f'  gz_topic_name: /model/{self._payload_name}/odometry\n'
            f'  ros_type_name: nav_msgs/msg/Odometry\n'
            f'  gz_type_name: gz.msgs.Odometry\n'
            f'  direction: GZ_TO_ROS\n'
            f'\n'
            f'- ros_topic_name: /payload/drop_cmd\n'
            f'  gz_topic_name: /{self._drop_topic}/drop\n'
            f'  ros_type_name: std_msgs/msg/Empty\n'
            f'  gz_type_name: gz.msgs.Empty\n'
            f'  direction: ROS_TO_GZ\n'
            f'\n'
            f'- ros_topic_name: /payload/attach_cmd\n'
            f'  gz_topic_name: /model/{self._gz_model_name}/detachable_joint/attach\n'
            f'  ros_type_name: std_msgs/msg/Empty\n'
            f'  gz_type_name: gz.msgs.Empty\n'
            f'  direction: ROS_TO_GZ\n'
        )
        with open(bridge_cfg_path, 'w') as f:
            f.write(bridge_cfg_content)
        bridge = subprocess.Popen(
            ['ros2', 'run', 'ros_gz_bridge', 'parameter_bridge',
             '--ros-args', '-p', f'config_file:={bridge_cfg_path}'],
            env=infra_env,
            stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL,
            preexec_fn=os.setsid,
        )
        time.sleep(5)

        # 3. PX4 SITL — connect to pre-spawned drone model via PX4_GZ_MODEL_NAME.
        #    Issue #014: drone is pre-spawned in world SDF (injected above)
        #    so DetachableJoint forms joint with payload at load time.
        #    PX4 connects to existing model instead of spawning a new one.
        px4_env = infra_env.copy()
        px4_env.update({
            'PX4_GZ_STANDALONE': '1',
            'PX4_GZ_WORLD': 'x_marker_world',
            'PX4_GZ_MODEL_NAME': self._gz_model_name,  # connect to pre-spawned model
            'PX4_SIM_SPEED_FACTOR': str(self._cfg_sim_speed_factor),
            'PX4_UXRCE_DDS_PORT': str(self._uxrce_port),
        })
        px4_bridge_dir = (
            f'{px4_dir}/build/px4_sitl_default/src/modules/simulation/gz_bridge')
        px4_bin = f'{px4_dir}/build/px4_sitl_default/bin/px4'
        # Redirect PX4 stdout+stderr to /dev/null — the pxh interactive shell
        # spams ~100 KB/s growing to GB-scale, causing I/O overhead that
        # slows PX4's main loop.
        px4_log = open(os.devnull, 'w')
        # Only pass -i for instances > 0; instance 0 uses default.
        # Copy calibrated parameters.bson to instance rootfs so instance > 0
        # gets COM_ARM_WO_GPS, EKF2_MAG_TYPE etc. from the airframe.
        px4_cmd = [px4_bin]
        if iid > 0:
            px4_cmd.extend(['-i', str(iid)])
            rootfs = f'{px4_dir}/build/px4_sitl_default/rootfs'
            inst_dir = os.path.join(rootfs, str(iid))
            os.makedirs(inst_dir, exist_ok=True)
            shutil.copy2(os.path.join(rootfs, 'parameters.bson'), inst_dir)
        px4 = subprocess.Popen(
            px4_cmd,
            cwd=px4_bridge_dir,
            env=px4_env,
            stdout=px4_log, stderr=subprocess.STDOUT,
            preexec_fn=os.setsid,
        )

        self._infra_procs = [p for p in [uxrce, gz, bridge, px4] if p is not None]
        # Note: bridge is always non-None (per-instance); gz is None for iid > 0

        # Unpause Gazebo — must happen after PX4 process starts so PX4's
        # gz_bridge can connect to the pre-spawned model. PX4 needs sim time
        # to flow for sensor initialization. Short sleep ensures PX4 has
        # connected before physics starts; PX4 motor controller prevents
        # drone from falling immediately.
        if iid == 0:
            time.sleep(3)
            subprocess.run(
                ['gz', 'service', '-s', '/world/x_marker_world/control',
                 '--reqtype', 'gz.msgs.WorldControl',
                 '--reptype', 'gz.msgs.Boolean',
                 '--timeout', '5000',
                 '--req', 'pause: false'],
                env=infra_env, capture_output=True, timeout=10)

        # Wait for PX4 readiness (obs_ready set by _on_local_pos callback)
        self._node.get_logger().info(
            f'[Instance {iid}] Waiting for PX4 position data...')
        deadline = time.time() + 90.0
        while not self._obs_ready.is_set() and time.time() < deadline:
            time.sleep(1.0)
        if not self._obs_ready.is_set():
            self._node.get_logger().error(
                f'[Instance {iid}] Timed out waiting for PX4 readiness (90s)')
        else:
            self._node.get_logger().info(
                f'[Instance {iid}] PX4 ready — infra up')

    def _kill_infra(self):
        """Kill all infrastructure processes.

        When infra was reused (self._infra_procs == []), there are no tracked
        handles, so we fall back to pkill to ensure Gazebo+PX4 are actually
        terminated before the next _start_infra() call.
        """
        iid = self._instance_id

        # Kill tracked procs first (if any)
        for proc in reversed(self._infra_procs):
            if proc.poll() is None:
                try:
                    os.killpg(os.getpgid(proc.pid), signal.SIGTERM)
                    # Phase 1 redux v2: 5s → 2s.
                    # drop 빈도 높을 때 _kill_infra 가 자주 호출되어 fps 폭락 원인.
                    proc.wait(timeout=2)
                except (ProcessLookupError, subprocess.TimeoutExpired):
                    try:
                        os.killpg(os.getpgid(proc.pid), signal.SIGKILL)
                    except ProcessLookupError:
                        pass
        self._infra_procs = []

        # Always pkill by name to catch reused (untracked) processes.
        # This ensures Gazebo+PX4 are truly dead before the next fresh start,
        # even when _start_infra() previously took the "reusing" path.
        if iid == 0:
            for pattern in ['gz sim', 'parameter_bridge']:
                subprocess.run(['pkill', '-f', pattern],
                               capture_output=True, check=False)
        subprocess.run(
            ['pkill', '-f', f'MicroXRCEAgent.*{self._uxrce_port}'],
            capture_output=True, check=False)
        subprocess.run(['pkill', '-f', 'bin/px4'],
                       capture_output=True, check=False)

        # Clean PX4 lock/socket files for this instance
        for path in [f'/tmp/px4_lock-{iid}', f'/tmp/px4-sock-{iid}']:
            try:
                os.remove(path)
            except FileNotFoundError:
                pass
        # Allow processes time to fully terminate before next start.
        # RAD: 2s → 3s (PX4/GZ cleanup 더 널널, race condition margin ↑)
        time.sleep(3.0)

    # ------------------------------------------------------------------
    # Episode lifecycle helpers
    # ------------------------------------------------------------------

    def _kill_episode(self):
        """Terminate episode-layer nodes via process-group kill.

        Each node is launched in its own process group (preexec_fn=os.setsid),
        so SIGTERM cascades to all children without affecting other instances.
        No global pkill — safe for multi-instance parallel training.
        """
        for proc in self._episode_procs:
            if proc.poll() is None:
                try:
                    os.killpg(os.getpgid(proc.pid), signal.SIGTERM)
                    # Round 8 (Phase 2 speedup): 5s → 2s.
                    # 5s timeout 이 짧은 episode + 자주 reset 시 누적 오버헤드 큼.
                    # 2s 도 정상 SIGTERM 응답엔 충분; 안 끝나면 SIGKILL escalation.
                    proc.wait(timeout=2)
                except (ProcessLookupError, subprocess.TimeoutExpired):
                    try:
                        os.killpg(os.getpgid(proc.pid), signal.SIGKILL)
                    except ProcessLookupError:
                        pass
        self._episode_procs = []

        # Brief pause to let DDS participants deregister
        time.sleep(0.5)

    def _start_episode(self):
        """Launch fresh episode-layer nodes (mission_manager, drone_controller,
        drop_calculator) with instance-specific ROS_DOMAIN_ID.

        For PX4 instances > 0, drone_controller topics are remapped to the
        PX4 UXRCE DDS namespace (px4_N).
        """
        ep_env = os.environ.copy()
        ep_env['ROS_DOMAIN_ID'] = str(self._instance_id)

        self._episode_procs = []

        # Log file for episode node output (debug)
        iid = self._instance_id
        ep_log = open(f'/tmp/episode_{iid}.log', 'w')

        # RAD: mission_manager_rad (spawn yaw 랜덤 + cruise 1m/s 가속)
        # 변경 사항 (v8 vs RAD):
        #   v8:  mission_manager_node (CRUISE 자동 이동)
        #   RAD: mission_manager_rad  (YAW_INIT → CRUISE head 1m/s → HANDOFF → TRACKING)
        proc = subprocess.Popen(
            ['ros2', 'run', 'mission_manager', 'mission_manager_rad',
             '--ros-args',
             '-p', f'target_enu_x:={self._cfg_target_x}',
             '-p', f'target_enu_y:={self._cfg_target_y}',
             '-p', f'cruise_target_speed:={self._cfg_cruise_target_speed}',
             '-p', f'spawn_yaw_random_enabled:={str(self._cfg_spawn_yaw_random_enabled).lower()}'],
            env=ep_env, stdout=ep_log, stderr=subprocess.STDOUT,
            preexec_fn=os.setsid,
        )
        self._episode_procs.append(proc)

        # drone_controller — PX4 -i N publishes to /px4_N/fmu/* topics,
        # so controller needs topic remapping for instances > 0
        ctrl_cmd = ['ros2', 'run', 'drone_controller', 'controller']
        if iid > 0:
            ns = self._px4_ns   # e.g. '/px4_1'
            ctrl_cmd.extend([
                '--ros-args',
                '-r', f'/fmu/in/offboard_control_mode:={ns}/fmu/in/offboard_control_mode',
                '-r', f'/fmu/in/trajectory_setpoint:={ns}/fmu/in/trajectory_setpoint',
                '-r', f'/fmu/in/vehicle_command:={ns}/fmu/in/vehicle_command',
                '-r', f'/fmu/out/vehicle_status:={ns}/fmu/out/vehicle_status',
            ])
        ctrl_log = open(f'/tmp/ctrl_{iid}.log', 'w')
        proc = subprocess.Popen(
            ctrl_cmd,
            env=ep_env, stdout=ctrl_log, stderr=subprocess.STDOUT,
            preexec_fn=os.setsid,
        )
        self._episode_procs.append(proc)

        # drop_calculator (no PX4 topics) — use per-instance target coordinates
        # Phase 1 redux fix: x_marker_x 도 cfg 사용 (이전엔 11.0 hardcoded).
        proc = subprocess.Popen(
            ['ros2', 'run', 'drop_calculator', 'calculator',
             '--ros-args',
             '-p', f'x_marker_x:={self._cfg_target_x}',
             '-p', f'x_marker_y:={self._cfg_target_y}'],   # _cfg_target_y = cfg + iid * 150.0
            env=ep_env, stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL,
            preexec_fn=os.setsid,
        )
        self._episode_procs.append(proc)

    def _gz_reset_poses(self):
        """Teleport drone and payload to upright spawn positions between episodes.

        Root cause of CRUISE timeout recurrence: after each episode the drone
        may crash/flip (roll=π).  Motors on an inverted drone push DOWN, so
        TAKEOFF never reaches altitude → perpetual CRUISE timeouts.

        fix: use set_pose/blocking to snap both models to spawn before the new
        drone_controller starts its 5 s EKF warmup.  0.5 s settle is enough
        because EKF reconvergence happens during the warmup, not here.

        If the payload was previously detached (dropped), it just sits freely
        at spawn; no DetachableJoint impulse because the joint is already gone.
        If the payload is still attached, both models align at the correct
        relative offset (z_drone=0, z_payload=0.14) → zero constraint error.
        """
        iid = self._instance_id
        model_y = iid * 150   # Method A: 150m Y-offset per instance
        gz_world = 'x_marker_world'

        for name, x, y, z in [
            (self._model_name,   0.0, float(model_y), 0.0),    # drone on ground
            (self._payload_name, 0.0, float(model_y), 0.14),   # payload at mount height
        ]:
            subprocess.run(
                [
                    'gz', 'service',
                    '-s', f'/world/{gz_world}/set_pose/blocking',
                    '--reqtype', 'gz.msgs.Pose',
                    '--reptype', 'gz.msgs.Boolean',
                    '--timeout', '2000',
                    '--req', (
                        f'name: "{name}", '
                        f'position: {{x: {x}, y: {y}, z: {z}}}, '
                        'orientation: {x: 0.0, y: 0.0, z: 0.0, w: 1.0}'
                    ),
                ],
                capture_output=True,
                timeout=3.0,
            )

        # Brief settle: Gazebo physics must register pose + velocity reset
        # before drone_controller arms (5 s EKF warmup gives additional margin).
        time.sleep(0.5)

    def _gz_world_reset(self):
        """model_only world reset — snaps all entities to SDF spawn positions.

        Resets drone to ground (~0.24m) and payload to 0.14m; re-attaches
        DetachableJoint. Uses model_only (not all) to avoid dartsim crash when
        DetachableJoint payload has been previously detached.
        """
        try:
            subprocess.run(
                [
                    'gz', 'service',
                    '-s', '/world/x_marker_world/control',
                    '--reqtype', 'gz.msgs.WorldControl',
                    '--reptype', 'gz.msgs.Boolean',
                    '--timeout', '3000',
                    '--req', 'reset: {model_only: true}',
                ],
                timeout=5.0,
                check=False,
            )
        except subprocess.TimeoutExpired:
            if rclpy.ok():
                self._node.get_logger().warning('gz world reset timed out')
        # Settle: physics registers joint re-attachment AND PX4 EKF absorbs the
        # pose snap.  5 s here + 2 s arm delay (40 ticks x 20 Hz in drone_controller)
        # = 7 s total before arming — increased from 3s to prevent next episode
        # starting before DetachableJoint is fully re-attached.
        time.sleep(5.0)

    def _wait_for_cruise(self):
        """RAD: Poll /mission/state until TRACKING (RL handoff complete) or timeout.

        mission_manager_rad 의 state machine: TAKEOFF → YAW_INIT → CRUISE → HANDOFF
        → TRACKING. RL 정책은 TRACKING 시점부터 control 인수.
        """
        deadline = time.time() + self._cfg_cruise_timeout
        while time.time() < deadline:
            if not self._spin_thread.is_alive():
                if rclpy.ok():
                    self._node.get_logger().error(
                        '[RL Env] Spin thread died during _wait_for_cruise — aborting wait')
                return
            with self._state_lock:
                state = self._node.mission_state
            if state == 'TRACKING':
                return
            time.sleep(0.5)
        if rclpy.ok():
            self._node.get_logger().warning(
                f'Timed out waiting for TRACKING state (got: {state})')
