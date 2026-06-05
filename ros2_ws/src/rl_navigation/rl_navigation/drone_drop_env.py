"""Gymnasium environment for drone fly-by drop RL training.

Reward structure — 4-Layer Hierarchical:
  Layer 1: Safety    — penalty on crash / overspeed / target lost (no hard termination)
  Layer 2: Stability — per-step time, angular-velocity and action-smoothness penalties
  Layer 3: Approach  — 2D distance gradient + heading alignment (no kinematic prediction)
  Layer 4: Terminal  — ACTUAL physics drop error (wait for drop_calculator result) + jackpot

Method A (1-World-4-Payload): 4 drones/payloads pre-spawned in shared Gazebo world at
150m Y-offsets. PX4 connects via PX4_GZ_MODEL_NAME. Each instance runs its own
ros_gz_bridge. Drop accuracy based on real Gazebo physics, NOT kinematic prediction.
"""

import math
import queue
import os
import shutil
import signal
import subprocess
import threading
import time

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
# Module-level scaling constants (used in _get_obs; also mirrored in yaml)
# ---------------------------------------------------------------------------
POS_SCALE = 50.0
VEL_SCALE = 15.0
ANG_VEL_SCALE = math.pi

TARGET_ENU_X = 11.0   # East  — overridable via yaml
TARGET_ENU_Y = 10.0   # North — overridable via yaml


def _load_config(config_path):
    """Load hyperparams yaml and return the parsed dict."""
    with open(config_path, 'r') as f:
        return yaml.safe_load(f)


class _RLBridgeNode(Node):
    """Background ROS2 node owning all pub/sub for the RL environment."""

    def __init__(self, state_lock, obs_ready_event,
                 drop_error_event, drop_error_queue,
                 px4_topic_prefix=''):
        super().__init__('drone_drop_rl_bridge')

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
        self.create_subscription(
            String, '/mission/state', self._on_mission_state, 10)
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
        """Extract roll and pitch from PX4 quaternion (NED frame, FRD body).

        PX4 VehicleAttitude uses q = [w, x, y, z].
        Standard Euler extraction (aerospace / ZYX convention):
          roll  = atan2(2(wX + yZ),  1 − 2(x² + y²))
          pitch = asin(clamp(2(wY − zX), −1, 1))
        """
        q = msg.q   # float32[4]: [w, x, y, z]
        w, x, y, z = float(q[0]), float(q[1]), float(q[2]), float(q[3])

        sinr_cosp = 2.0 * (w * x + y * z)
        cosr_cosp = 1.0 - 2.0 * (x * x + y * y)
        roll = math.atan2(sinr_cosp, cosr_cosp)

        sinp = 2.0 * (w * y - z * x)
        sinp = max(-1.0, min(1.0, sinp))   # clamp for numerical safety
        pitch = math.asin(sinp)

        with self._lock:
            self.roll = roll
            self.pitch = pitch

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
        """Fire payload drop commands on both ROS2 and gz-transport channels."""
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

class DroneDropEnv(gym.Env):
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

        # --- Load config ---
        cfg_env = {}
        cfg_reward = {}
        if config_path is not None:
            cfg = _load_config(config_path)
            cfg_env = cfg.get('environment', {})
            cfg_reward = cfg.get('reward', {})

        # --- Environment constants ---
        self._cfg_target_x = cfg_env.get('target_enu_x', TARGET_ENU_X)
        # Method A: target Y shifts by 150m per instance to align with pre-spawned x_marker_N
        self._cfg_target_y = cfg_env.get('target_enu_y', TARGET_ENU_Y) + instance_id * 150.0
        self._cfg_pos_scale = cfg_env.get('pos_scale', POS_SCALE)
        self._cfg_vel_scale = cfg_env.get('vel_scale', VEL_SCALE)
        self._cfg_ang_vel_scale = cfg_env.get('ang_vel_scale', ANG_VEL_SCALE)
        self._cfg_action_vx_scale = cfg_env.get('action_vx_scale', 15.0)
        self._cfg_action_vy_scale = cfg_env.get('action_vy_scale', 5.0)
        self._cfg_action_vz_scale = cfg_env.get('action_vz_scale', 3.0)
        self._cfg_action_yaw_scale = cfg_env.get('action_yaw_scale', 1.0)
        # P2 (junsang_v4): action 변화량 hard clip (가속도 제한)
        self._cfg_action_rate_limit = cfg_env.get('action_rate_limit', 0.2)
        self._cfg_max_steps = cfg_env.get('max_steps', 800)
        self._cfg_min_altitude = cfg_env.get('min_altitude', 2.0)
        self._cfg_min_alt_start = cfg_env.get('min_altitude_start_step', 1)
        self._cfg_ground_contact_alt = cfg_env.get('ground_contact_altitude', 0.5)
        self._cfg_max_distance = cfg_env.get('max_distance', 100.0)
        self._cfg_max_altitude = cfg_env.get('max_altitude', 50.0)
        self._cfg_obs_wait = cfg_env.get('obs_wait_timeout', 0.15)
        self._cfg_cruise_timeout = cfg_env.get('cruise_poll_timeout', 60.0)
        self._cfg_sim_speed_factor = int(cfg_env.get('sim_speed_factor', 1))
        # use_vision=False: skip camera termination + synthesise conf=1.0 in obs
        self._cfg_use_vision = cfg_env.get('use_vision', True)

        # --- Reward constants (4-Layer Hierarchical) ---
        r = cfg_reward
        self._cfg_g = r.get('g', 9.81)
        self._cfg_auto_drop_threshold = r.get('auto_drop_threshold', 3.0)
        self._cfg_random_drop_start_step = r.get('random_drop_start_step', 600)
        self._cfg_random_drop_prob = r.get('random_drop_prob', 0.005)
        self._cfg_k1_potential = r.get('k1_potential', 1.0)
        self._cfg_k2_precision = r.get('k2_precision', 0.2)
        self._cfg_w_dist = r.get('w_dist', 1.0)
        self._cfg_w_heading = r.get('w_heading', 0.7)
        self._cfg_w_distance_penalty = r.get('w_distance_penalty', 0.0)
        self._cfg_w_time = r.get('w_time', 0.01)
        self._cfg_w_ang_vel = r.get('w_ang_vel', 0.05)
        self._cfg_w_action_smooth = r.get('w_action_smooth', 0.05)
        self._cfg_w_drop_base = r.get('w_drop_base', 100.0)
        self._cfg_r_success_jackpot = r.get('r_success_jackpot', 50.0)
        self._cfg_success_threshold = r.get('success_threshold', 5.0)
        self._cfg_jackpot_threshold = r.get('jackpot_threshold', 0.1)
        self._cfg_penalty_instability = r.get('penalty_instability', 50.0)
        self._cfg_limit_ang_vel = r.get('limit_ang_vel', 2.0)
        self._cfg_limit_tilt = r.get('limit_tilt', 0.26)
        self._cfg_limit_inverted_tilt = r.get('limit_inverted_tilt', 1.047)
        self._cfg_penalty_bad_attitude = r.get('penalty_bad_attitude', -30.0)
        self._cfg_drop_attempt_bonus = r.get('drop_attempt_bonus', 30.0)
        self._cfg_k_drop_proximity = r.get('k_drop_proximity', 0.15)
        self._cfg_truncation_penalty = r.get('truncation_penalty', -15.0)
        self._cfg_penalty_crash = r.get('penalty_crash', -50.0)
        self._cfg_penalty_overspeed = r.get('penalty_overspeed', -30.0)
        self._cfg_penalty_target_lost = r.get('penalty_target_lost', -10.0)
        self._cfg_penalty_out_of_range = r.get('penalty_out_of_range', -30.0)
        self._cfg_penalty_max_altitude = r.get('penalty_max_altitude', -15.0)
        # Round 5: Hover 감지 (속도 기준 연속 정체)
        self._cfg_hover_speed_threshold = r.get('hover_speed_threshold', 1.0)
        self._cfg_hover_consecutive_threshold = int(r.get('hover_consecutive_threshold', 200))
        self._cfg_penalty_hover = r.get('penalty_hover', -15.0)
        # Drop 시점 고도 페널티 (Round 3): sigmoid bounded
        self._cfg_alt_penalty_max = r.get('alt_penalty_max', 50.0)
        self._cfg_alt_penalty_mid = r.get('alt_penalty_mid', 30.0)
        self._cfg_alt_penalty_k = r.get('alt_penalty_k', 0.15)
        self._cfg_w_prediction = r.get('w_prediction', 20.0)
        self._cfg_k_prediction = r.get('k_prediction', 0.1)
        # Layer 4: time to wait for actual physics landing result from drop_calculator
        self._cfg_drop_wait_timeout = r.get('drop_wait_timeout', 10.0)
        # Ablation flag: disable speed gate to reproduce Spiral Milking reward hack
        self._cfg_speed_gate = r.get('speed_gate_enabled', True)
        # d_impact shaping (Layer 3 addition): reward for reducing CCIP predicted miss distance
        self._cfg_w_impact = r.get('w_impact', 0.0)
        self._cfg_k_impact = r.get('k_impact', 0.05)

        # obs[0-14]: pos(3)+vel(3)+ang_vel(3)+vision(3)+attached(1)+rel_target(2)
        # obs[15]:   d_impact / pos_scale  (CCIP predicted miss distance)
        # obs[16]:   t_f / 10.0            (CCIP time-of-flight, clamped at 10s)
        self.observation_space = gym.spaces.Box(
            low=-1.0, high=1.0, shape=(17,), dtype=np.float32)
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
            'max_consecutive_fast_resets', 100))

        # --- Start ROS2 ---
        if not rclpy.ok():
            rclpy.init()
        self._node = _RLBridgeNode(
            self._state_lock,
            self._obs_ready,
            self._drop_error_event,
            self._drop_error_queue,
            px4_topic_prefix=self._px4_ns,
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

        # Guard against infinite recursion when CRUISE is permanently unreachable
        # (e.g. spin thread died). Allow at most 2 nested retries total.
        self._reset_depth += 1
        if self._reset_depth > 2:
            self._reset_depth = 0
            raise RuntimeError(
                '[RL Env] reset() called recursively more than 2 times. '
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
        self._kill_episode()

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
            self._obs_ready.clear()  # ensure _start_infra() waits for fresh PX4 data
            self._kill_infra()
            self._start_infra()
            self._consecutive_fast_resets = 0
            if _forced_restart and rclpy.ok():
                self._node.get_logger().info(
                    f'[RL Env] Forced full restart after'
                    f' {self._cfg_max_consecutive_fast_resets}'
                    ' consecutive fast resets (no drop)')
        else:
            self._gz_reset_poses()   # fast teleport (joint still intact)
            self._consecutive_fast_resets += 1

        # 4. Start fresh episode processes
        self._start_episode()

        # 6. Wait for CRUISE state (blocks until takeoff + climb complete).
        #    Tiered recovery on timeout:
        #      Attempt 1: episode-only restart if Gazebo+PX4 still healthy (fast, ~5s).
        #                 Covers most cases: drone_controller/mission_manager issue.
        #      Attempt 2+: full infra restart (kills Gazebo+PX4, re-spawns fresh, ~80s).
        #                  Used when infra itself is unhealthy or fast path failed.
        #    Retries up to 3 times before giving up; this episode is never
        #    returned to SB3 so the replay buffer stays clean.
        for _cruise_attempt in range(3):
            self._wait_for_cruise()
            with self._state_lock:
                _reached_cruise = (self._node.mission_state == 'CRUISE')
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
                with self._state_lock:
                    self._node.mission_state = 'IDLE'
                self._gz_reset_poses()   # teleport drone+payload to spawn
                self._start_episode()
            else:
                self._node.get_logger().warn(
                    f'[RL Env] CRUISE timeout (attempt {_cruise_attempt + 1}/3)'
                    ' — full infra restart')
                self._kill_episode()
                self._kill_infra()
                self._obs_ready.clear()  # ensure _start_infra() waits for fresh PX4 data
                with self._state_lock:
                    self._node.mission_state = 'IDLE'
                self._start_infra()
                self._start_episode()
        else:
            # All 3 attempts failed — attempt one full reset from scratch.
            # _reset_depth guard above prevents infinite recursion.
            self._node.get_logger().error(
                '[RL Env] CRUISE timeout after 3 attempts — forcing full reset')
            return self.reset(seed=seed, options=options)

        self._reset_depth = 0  # success — clear depth counter

        # 7. Seed d_xy_prev from the initial CRUISE position (no kinematic prediction)
        with self._state_lock:
            pos = self._node.pos_enu.copy()
            # Round 7 diagnostic: capture velocity at CRUISE (post-reset stabilized).
            _post_cruise_v = float(np.linalg.norm(self._node.vel_enu))
            _post_cruise_ang_v = float(np.linalg.norm(self._node.ang_vel))
        self.d_xy_prev = self._compute_d_xy(pos)

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
        return obs, {}

    def step(self, action):
        """Apply action, advance one control step, return (obs, reward, term, trunc, info)."""
        self._step_count += 1

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

        # --- 2D horizontal distance + CCIP predicted impact distance ---
        d_xy = self._compute_d_xy(pos)
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

        # --- Drop decision: auto + random exploration ---
        # action[4] ignored (manual drop disabled — causes "drop ASAP" exploit).
        # auto_drop: d_impact ≤ threshold → precision drop near target.
        # random_drop: step ≥ start, 0.5% per step → diverse drop experience
        #              at various d_xy. Agent can't control → learns to fly only.
        random_drop = (self._step_count >= self._cfg_random_drop_start_step
                       and np.random.random() < self._cfg_random_drop_prob)
        if (random_drop or d_impact <= self._cfg_auto_drop_threshold) and not self.dropped:
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

            if not truncated and d_xy > self._cfg_max_distance:
                reward += self._cfg_penalty_out_of_range
                truncated = True
                truncate_reason = 'out_of_range'

            if not truncated and pos[2] > self._cfg_max_altitude:
                reward += self._cfg_penalty_max_altitude
                truncated = True
                truncate_reason = 'max_altitude'

            if truncate_reason:
                info['truncate_reason'] = truncate_reason

        # --- Truncation on step limit (timeout) ---
        if not terminated and not truncated and self._step_count >= self._cfg_max_steps:
            truncated = True
            if not self.dropped:
                # P8 (junsang_v4): yaml truncation_penalty 사용 (이전 hardcoded -80)
                reward += self._cfg_truncation_penalty
            info['truncate_reason'] = 'timeout'

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

        return obs, reward, terminated, truncated, info

    def close(self):
        """Shutdown episode nodes, infrastructure, and ROS2."""
        self._kill_episode()
        self._kill_infra()
        if rclpy.ok():
            self._node.destroy_node()
            rclpy.shutdown()

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

    def _compute_d_xy(self, pos):
        """2D horizontal distance from drone to target (no kinematic prediction).

        Used for Layer 3 approach gradient and Layer 4 auto-drop trigger in
        Method A. Replaces _predict_impact_point for per-step distance signal.

        Args:
            pos: ENU position [x, y, z] in metres

        Returns:
            d_xy (float): horizontal Euclidean distance to target (metres)
        """
        dx = float(pos[0]) - self._cfg_target_x
        dy = float(pos[1]) - self._cfg_target_y
        return math.sqrt(dx * dx + dy * dy)

    # ------------------------------------------------------------------
    # Reward computation — Layers 1, 2, 3 (non-terminal steps)
    # ------------------------------------------------------------------

    def _compute_reward(self, pos, vel, ang, pix, d_xy, d_impact, action):
        """Compute per-step reward for non-terminal steps.

        Layer 1 — Safety:     penalty on crash / overspeed / target lost (no termination)
        Layer 2 — Stability:  time + angular-velocity + action-smoothness penalties
        Layer 3 — Approach:   2D distance gradient + heading alignment reward

        Args:
            d_xy:     2D horizontal distance to target (drone position, no kinematics)
            d_impact: CCIP predicted miss distance (from _predict_impact_point)

        Returns:
            reward (float), terminated (bool), info (dict)
        """
        info = {}
        reward = 0.0

        # ----------------------------------------------------------------
        # Layer 1 — Safety (penalties only; episode continues)
        # ----------------------------------------------------------------
        altitude = float(pos[2])
        speed = float(np.linalg.norm(vel))
        conf = float(pix[2])   # vision confidence; 0.0 = no detection

        if altitude < self._cfg_ground_contact_alt:
            info['crash'] = True
            reward += self._cfg_penalty_crash
        elif self._step_count > self._cfg_min_alt_start and altitude < self._cfg_min_altitude:
            info['crash'] = True
            reward += self._cfg_penalty_crash

        if speed > self._V_MAX_SAFETY:
            info['overspeed'] = True
            reward += self._cfg_penalty_overspeed

        # Skip vision-based penalty when running without camera sensor.
        # use_vision=False means pixel_coords will always be zero (no detector).
        if self._cfg_use_vision and conf == 0.0:
            info['target_lost'] = True
            reward += self._cfg_penalty_target_lost

        # ----------------------------------------------------------------
        # Layer 2 — Efficiency / Stability
        # R2 = -w_time - w_ang_vel * ||omega||^2 - w_action_smooth * ||Δa||^2
        # ----------------------------------------------------------------
        omega_sq = float(np.dot(ang, ang))   # ||omega||^2

        action_arr = np.asarray(action, dtype=np.float32)
        delta_action = action_arr - self.action_prev
        action_smooth_sq = float(np.dot(delta_action, delta_action))

        r2 = (
            -0.05
            - self._cfg_w_ang_vel * omega_sq
            - self._cfg_w_action_smooth * action_smooth_sq
        )

        # ----------------------------------------------------------------
        # Layer 3 — Approach
        # R3 = w_dist * (d_prev - d_now)           ← linear distance reward
        #      + w_heading * cos(angle between drone heading and bearing to target)
        #
        # NOTE: Previously used exp(-k1*d) potential which saturates to ~0 for
        # d > 10 m (with k1=1.0, exp(-45) ≈ 6e-20 in float64 — machine zero).
        # Linear reward gives a nonzero gradient at ANY distance.
        # k1_potential is retained in config for reference but unused here.
        # ----------------------------------------------------------------

        # Distance gradient (positive when moving toward target, any distance)
        r3_dist = self._cfg_w_dist * (self.d_xy_prev - d_xy)

        # Heading alignment: cos(delta_yaw_to_target)
        # Use 2-D velocity direction as drone heading proxy.
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
                cos_heading = 1.0   # already over target
        else:
            cos_heading = 0.0   # hovering — no heading signal

        # Speed-gated orientation reward: scales from 0 at hover to full at ≥2 m/s.
        # Prevents farming cos_heading by crawling/hovering (anti-milking).
        # speed_gate_enabled=false reproduces Spiral Milking reward hack (ablation).
        speed_gate = min(speed_xy / 2.0, 1.0) if self._cfg_speed_gate else 1.0
        r3_orient = self._cfg_w_heading * cos_heading * speed_gate

        # d_impact shaping: exp(-k_impact * d_impact) gives nonzero gradient at any distance.
        # Rewards the agent for achieving a trajectory where releasing now would land near target.
        r3_impact = self._cfg_w_impact * math.exp(-self._cfg_k_impact * d_impact)

        # Round 4 (A+C): 거리 비례 per-step 페널티 (hover 차단)
        # 멀리 있을수록 매 step 비용 부과 — hover 수익 차단
        # penalty = -w * d_xy / 50  (d_xy=15m → -0.009/step)
        r3_distance_penalty = -self._cfg_w_distance_penalty * d_xy / 50.0

        r3 = r3_dist + r3_orient + r3_impact + r3_distance_penalty

        # Advance d_xy_prev for next step
        self.d_xy_prev = d_xy

        reward += r2 + r3
        info['r2'] = r2
        info['r3'] = r3
        info['d_xy'] = d_xy
        info['d_impact'] = d_impact    # CCIP predicted miss distance (WandB: env/mean_d_impact)
        info['cos_heading'] = cos_heading
        # Split reward components for per-rollout WandB monitoring
        info['rew_ctrl'] = r2
        info['rew_dist'] = r3_dist
        info['rew_orient'] = r3_orient
        info['rew_impact'] = r3_impact
        info['rew_drop'] = 0.0

        return reward, False, info

    # ------------------------------------------------------------------
    # Observation builder
    # ------------------------------------------------------------------

    def _get_obs(self):
        with self._state_lock:
            pos = self._node.pos_enu.copy()
            vel = self._node.vel_enu.copy()
            ang = self._node.ang_vel.copy()
            pix = self._node.pixel_coords.copy()
            attached = float(self._node.payload_attached)

        pos_n = np.nan_to_num(pos / POS_SCALE, nan=0.0)
        vel_n = np.nan_to_num(vel / VEL_SCALE, nan=0.0)
        ang_n = np.nan_to_num(ang / ANG_VEL_SCALE, nan=0.0)

        # Pixel coordinates normalised to [-1, 1].
        # When use_vision=False (camera disabled), synthesise conf=1.0 so
        # downstream reward layers and SB3 normalisation see a valid signal;
        # u_norm/v_norm remain 0.0 (centred) since no pixel data is available.
        if self._cfg_use_vision:
            u_norm = (pix[0] / 640.0) * 2.0 - 1.0 if pix[2] > 0 else 0.0
            v_norm = (pix[1] / 480.0) * 2.0 - 1.0 if pix[2] > 0 else 0.0
            conf = float(np.clip(pix[2], 0.0, 1.0))
        else:
            u_norm = 0.0
            v_norm = 0.0
            conf = 1.0   # synthetic: "target always visible" via kinematics

        pos_clean = np.nan_to_num(pos, nan=0.0)
        vel_clean = np.nan_to_num(vel, nan=0.0)
        rel_dx = (pos_clean[0] - self._cfg_target_x) / POS_SCALE
        rel_dy = (pos_clean[1] - self._cfg_target_y) / POS_SCALE

        # CCIP features: predicted miss distance and time-of-flight
        # t_f is clamped at 10s in _predict_impact_point; d_impact normalised by POS_SCALE.
        _, _, t_f_obs, d_impact_obs = self._predict_impact_point(pos_clean, vel_clean)
        obs_d_impact = float(np.clip(d_impact_obs / self._cfg_pos_scale, 0.0, 1.0))
        obs_t_f = float(np.clip(t_f_obs / 10.0, 0.0, 1.0))

        obs = np.array([
            *np.clip(pos_n, -1.0, 1.0),    # 0-2  world position (ENU, normalised)
            *np.clip(vel_n, -1.0, 1.0),    # 3-5  world velocity (ENU, normalised)
            *np.clip(ang_n, -1.0, 1.0),    # 6-8  angular velocity (body, normalised)
            u_norm, v_norm, conf,           # 9-11 vision: pixel u, v, confidence
            attached,                       # 12   payload attached flag
            np.clip(rel_dx, -1.0, 1.0),    # 13   relative x to target (normalised)
            np.clip(rel_dy, -1.0, 1.0),    # 14   relative y to target (normalised)
            obs_d_impact,                  # 15   CCIP miss dist / 50m  (0=on target)
            obs_t_f,                       # 16   CCIP t_f / 10s        (0=drop now)
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
        # Allow processes time to fully terminate before next start
        time.sleep(2.0)

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

        # mission_manager (no PX4 topics — only /drone/cmd/* and /mission/state)
        proc = subprocess.Popen(
            ['ros2', 'run', 'mission_manager', 'mission_manager_node',
             '--ros-args', '-p', 'rl_mode:=true'],
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
        """Poll /mission/state until CRUISE or timeout."""
        deadline = time.time() + self._cfg_cruise_timeout
        while time.time() < deadline:
            if not self._spin_thread.is_alive():
                if rclpy.ok():
                    self._node.get_logger().error(
                        '[RL Env] Spin thread died during _wait_for_cruise — aborting wait')
                return  # caller will detect mission_state != 'CRUISE' and retry
            with self._state_lock:
                state = self._node.mission_state
            if state == 'CRUISE':
                return
            time.sleep(0.5)
        if rclpy.ok():
            self._node.get_logger().warning(
                f'Timed out waiting for CRUISE state (got: {state})')
