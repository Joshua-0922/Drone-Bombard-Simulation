"""Gymnasium environment for drone X-marker navigation RL training.

Reward structure — 3-Layer Hierarchical:
  Layer 1: Safety    — penalty on crash / overspeed / target lost (no hard termination)
  Layer 2: Stability — per-step time, angular-velocity and action-smoothness penalties
  Layer 3: Navigation — GPS distance gradient + heading alignment + vision centering + proximity

Goal: train drone to fly as close as possible to the X marker using YOLO camera detection.
Drop/payload functionality is removed; camera (YOLO) must be enabled (use_vision: true).
"""

import math
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
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy
from std_msgs.msg import String

try:
    from px4_msgs.msg import (
        VehicleLocalPosition,
        VehicleAngularVelocity,
        VehicleAttitude,
        VehicleCommand,
        VehicleStatus,
        EstimatorStatusFlags,
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

TARGET_ENU_X = 11.0   # MEASURED 2026-06-14: PX4 EKF East = +Gazebo_East (NO reversal;
                       # prior -Gazebo_East assumption was a misdiagnosis). pos_enu[0]
                       # = PX4 East = Gazebo East, so marker East=11 → target +11.
TARGET_ENU_Y = 10.0   # PX4 North = Gazebo North — unchanged


def _load_config(config_path):
    """Load hyperparams yaml and return the parsed dict."""
    with open(config_path, 'r') as f:
        return yaml.safe_load(f)


class _RLBridgeNode(Node):
    """Background ROS2 node owning all pub/sub for the RL environment."""

    def __init__(self, state_lock, obs_ready_event,
                 px4_topic_prefix=''):
        super().__init__('drone_drop_rl_bridge')

        self._lock = state_lock
        self._obs_ready = obs_ready_event

        # --- Shared state (protected by _lock) ---
        self.pos_enu = np.zeros(3, dtype=np.float32)
        self.vel_enu = np.zeros(3, dtype=np.float32)
        self.ang_vel = np.zeros(3, dtype=np.float32)
        self.roll = 0.0
        self.pitch = 0.0
        self.pixel_coords = np.zeros(3, dtype=np.float32)   # u, v, conf
        self.mission_state = 'IDLE'
        self.ekf_in_air = True   # assume airborne until EKF says otherwise
        self.armed = False       # PX4 arming_state == ARMED (for #4 early bail)

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
            self.create_subscription(
                EstimatorStatusFlags,
                f'{pfx}/fmu/out/estimator_status_flags',
                self._on_estimator_flags,
                px4_qos,
            )
            self.create_subscription(
                VehicleStatus,
                f'{pfx}/fmu/out/vehicle_status',
                self._on_vehicle_status,
                px4_qos,
            )
        else:
            self.get_logger().warning(
                'px4_msgs not available; position/velocity/attitude obs will be zeros.')

        self.create_subscription(
            Point, '/target/pixel_coords', self._on_pixel_coords, 10)
        self.create_subscription(
            String, '/mission/state', self._on_mission_state, 10)

        # --- Publishers ---
        self.vel_pub = self.create_publisher(Twist, '/drone/cmd/velocity', 10)

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

    def _on_estimator_flags(self, msg):
        with self._lock:
            self.ekf_in_air = msg.cs_in_air

    def _on_vehicle_status(self, msg):
        with self._lock:
            self.armed = (msg.arming_state == 2)   # ARMING_STATE_ARMED

    # ------------------------------------------------------------------
    # Action helpers
    # ------------------------------------------------------------------

    def publish_velocity(self, vx, vy, vz, yaw_rate):
        """Publish a velocity Twist to /drone/cmd/velocity (drone_controller forwards to PX4)."""
        msg = Twist()
        msg.linear.x = float(vx)
        msg.linear.y = float(vy)
        msg.linear.z = float(vz)
        msg.angular.z = float(yaw_rate)
        self.vel_pub.publish(msg)

    def send_disarm(self):
        """Force-disarm PX4 between episodes.

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
    """Gymnasium env: SAC trains drone to fly as close as possible to X marker.

    Observation: Box(14,) float32
      [0-2]  pos_enu / pos_scale         (world position, normalised)
      [3-5]  vel_enu / vel_scale         (world velocity, normalised)
      [6-8]  ang_vel / ang_vel_scale     (body angular velocity, normalised)
      [9-11] vision: u_norm, v_norm, conf (YOLO pixel coords + confidence)
      [12-13] rel_dx, rel_dy / pos_scale (relative position to target, normalised)
    Action: Box(4,) float32 in [-1, 1]
      [0] vx command (scaled by action_vx_scale)
      [1] vy command (scaled by action_vy_scale)
      [2] vz command (scaled by action_vz_scale)
      [3] yaw_rate command (scaled by action_yaw_scale)
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
        self._cfg_max_altitude = cfg_env.get('max_altitude', 25.0)
        self._cfg_stagnation_window = int(cfg_env.get('stagnation_window', 200))
        self._cfg_stagnation_min_progress = cfg_env.get('stagnation_min_progress', 2.0)
        self._cfg_stagnation_start_step = int(cfg_env.get('stagnation_start_step', 200))
        self._cfg_obs_wait = cfg_env.get('obs_wait_timeout', 0.15)
        self._cfg_cruise_timeout = cfg_env.get('cruise_poll_timeout', 60.0)
        # #4: if PX4 has not armed within this window the takeoff is stuck on an
        # arm rejection (stale EKF after teleport). Bail to a full infra restart
        # immediately instead of waiting out the full cruise_poll_timeout.
        self._cfg_arm_bail_timeout = cfg_env.get('arm_bail_timeout', 10.0)
        self._cfg_sim_speed_factor = int(cfg_env.get('sim_speed_factor', 1))
        # use_vision=False: skip camera termination + synthesise conf=1.0 in obs
        self._cfg_use_vision = cfg_env.get('use_vision', True)
        self._cfg_detection_hold_frames = cfg_env.get('detection_hold_frames', 10)
        # How long to wait (s) for the drone to detect the marker (CRUISE→TRACKING transition)
        # after takeoff before giving up and resetting the episode.
        self._cfg_tracking_wait_timeout = cfg_env.get('tracking_wait_timeout', 120.0)

        # --- Reward constants (3-Layer Hierarchical) ---
        r = cfg_reward
        self._cfg_w_dist = r.get('w_dist', 1.0)
        self._cfg_w_heading = r.get('w_heading', 0.7)
        self._cfg_w_proximity = r.get('w_proximity', 0.0)
        self._cfg_proximity_radius = r.get('proximity_radius', 5.0)
        self._cfg_w_time = r.get('w_time', 0.01)
        self._cfg_w_ang_vel = r.get('w_ang_vel', 0.05)
        self._cfg_w_action_smooth = r.get('w_action_smooth', 0.05)
        self._cfg_w_vision_center = r.get('w_vision_center', 1.0)
        self._cfg_limit_ang_vel = r.get('limit_ang_vel', 2.0)
        self._cfg_limit_tilt = r.get('limit_tilt', 0.26)
        self._cfg_limit_inverted_tilt = r.get('limit_inverted_tilt', 1.047)
        self._cfg_penalty_bad_attitude = r.get('penalty_bad_attitude', -30.0)
        self._cfg_truncation_penalty = r.get('truncation_penalty', -15.0)
        self._cfg_penalty_crash = r.get('penalty_crash', -50.0)
        self._cfg_penalty_overspeed = r.get('penalty_overspeed', -30.0)
        self._cfg_penalty_target_lost = r.get('penalty_target_lost', -10.0)
        self._cfg_penalty_out_of_range = r.get('penalty_out_of_range', -30.0)
        self._cfg_penalty_max_altitude = r.get('penalty_max_altitude', -30.0)
        self._cfg_penalty_stagnation = r.get('penalty_stagnation', -15.0)
        self._cfg_success_radius = r.get('success_radius', 0.5)
        self._cfg_reward_success = r.get('reward_success', 100.0)
        self._cfg_overshoot_close_threshold = r.get('overshoot_close_threshold', 1.5)
        self._cfg_overshoot_margin = r.get('overshoot_margin', 1.5)
        self._cfg_penalty_overshoot = r.get('penalty_overshoot', -20.0)
        self._cfg_speed_gate = r.get('speed_gate_enabled', True)

        # obs[0-2]:  pos (ENU, normalised)
        # obs[3-5]:  vel (ENU, normalised)
        # obs[6-8]:  ang_vel (body, normalised)
        # obs[9-11]: vision u, v, conf
        # obs[12-13]: rel_dx, rel_dy (relative to target, normalised)
        self.observation_space = gym.spaces.Box(
            low=-1.0, high=1.0, shape=(14,), dtype=np.float32)
        self.action_space = gym.spaces.Box(
            low=-1.0, high=1.0, shape=(4,), dtype=np.float32)

        # --- Threading primitives ---
        self._state_lock = threading.Lock()
        self._obs_ready = threading.Event()

        # --- Episode state ---
        self._step_count = 0
        self._episode_reward = 0.0
        self._episode_procs = []      # Popen handles for episode nodes
        self.d_xy_prev = 0.0          # 2D horizontal distance to target at previous step
        self.action_prev = np.zeros(4, dtype=np.float32)
        self._d_xy_history = {}       # step→d_xy for stagnation detection
        self._rl_detection_printed = False  # printed once per episode on first RL-phase detection

        # --- Infra state ---
        self._infra_procs = []        # Popen handles for infra processes

        # --- Reset recursion guard ---
        self._reset_depth = 0
        self._needs_infra_restart = False  # set by step() on EKF drift; triggers fast infra restart in reset()

        # --- Start ROS2 ---
        if not rclpy.ok():
            rclpy.init()
        self._node = _RLBridgeNode(
            self._state_lock,
            self._obs_ready,
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

        # Guard against infinite recursion when CRUISE/TRACKING is permanently unreachable.
        # Allow up to 5 nested retries (each tracking timeout burns 120s, so 5 × 120s = 10 min
        # before giving up — enough for EKF to settle after a difficult reset).
        self._reset_depth += 1
        if self._reset_depth > 5:
            self._reset_depth = 0
            raise RuntimeError(
                '[RL Env] reset() called recursively more than 5 times. '
                'Spin thread may be dead or infra is unrecoverable. Aborting.')

        # EKF drift fast path: skip the teleport+CRUISE retry cycle and go straight to
        # a full infra restart.  EKF cannot reconverge from a large position delta via
        # gz_reset_poses alone; killing Gazebo+PX4 is the only reliable fix.
        if self._needs_infra_restart:
            self._needs_infra_restart = False
            self._node.get_logger().warn(
                '[RL Env] EKF drift flag set — skipping CRUISE retries, doing full infra restart.')
            self._kill_episode()
            self._kill_infra()
            self._obs_ready.clear()
            with self._state_lock:
                self._node.mission_state = 'IDLE'
            self._start_infra()
            return self.reset(seed=seed, options=options)

        # 1. Clear episode state
        self._step_count = 0
        self._episode_reward = 0.0
        self._d_xy_history = {}
        self._d_xy_min = float('inf')
        self._obs_ready.clear()
        self.action_prev = np.zeros(4, dtype=np.float32)

        with self._state_lock:
            self._node.mission_state = 'IDLE'  # avoid stale CRUISE from last episode

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

        # 2b. Force-disarm PX4 so the new drone_controller's 5s EKF warmup runs
        #     from a freshly-armed state rather than carrying over armed+offboard.
        self._node.send_disarm()
        time.sleep(0.2)

        # 3. Reset drone to spawn position (fast teleport path — payload always attached).
        self._gz_reset_poses()

        # 3b. Wait for PX4 EKF to reconverge to ground position after teleport.
        #     EKF2 carries over cs_in_air=True from the previous episode after
        #     teleport; PX4 refuses to arm (pre_flight_checks_pass=False) while
        #     cs_in_air is True.  Actively poll until EKF reports cs_in_air=False
        #     instead of using a fixed sleep, so fast resets don't waste time and
        #     slow resets (large position delta) don't time out prematurely.
        _ekf_settle_deadline = time.time() + 30.0
        while time.time() < _ekf_settle_deadline:
            with self._state_lock:
                _in_air = self._node.ekf_in_air
            if not _in_air:
                break
            time.sleep(0.5)
        else:
            self._node.get_logger().warn(
                '[RL Env] EKF still reports cs_in_air=True after 30 s — '
                'proceeding anyway; arming may be slow.'
            )

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
        # v9c-#1: fast-path (episode-only restart) REMOVED. It recovered only
        # ~23% of CRUISE timeouts — the stale EKF after a teleport-only reset
        # makes PX4 refuse to arm, so the drone never leaves TAKEOFF — and on the
        # ~77% of failures it wasted a full cruise_poll_timeout before falling
        # back to a full restart anyway. Going straight to a full infra restart
        # on every CRUISE timeout is faster on stuck episodes; a fresh PX4
        # reliably reconverges the EKF at (0,0,0) so arming succeeds first try.
        for _cruise_attempt in range(3):
            self._wait_for_cruise()
            with self._state_lock:
                _reached_cruise = (self._node.mission_state == 'CRUISE')
            if _reached_cruise:
                break

            self._node.get_logger().warn(
                f'[RL Env] CRUISE timeout (attempt {_cruise_attempt + 1}/3)'
                ' — full infra restart (fast path removed)')
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

        # 7. Wait for TRACKING state — mission_manager transitions CRUISE→TRACKING on
        #    first YOLO detection.  RL episode begins only once the marker is visible.
        self._wait_for_tracking()
        with self._state_lock:
            _reached_tracking = (self._node.mission_state == 'TRACKING')
        if not _reached_tracking:
            self._node.get_logger().warn(
                '[RL Env] TRACKING timeout — drone did not detect marker. '
                'Full infra restart to get clean EKF, then retry.')
            # After TRACKING timeout the drone may have been cruising far from spawn.
            # EKF cannot reconverge via gz_reset_poses+sleep alone from a large position
            # delta (10+ m).  Kill and respawn Gazebo+PX4 so EKF starts fresh at (0,0,0),
            # avoiding the CRUISE-timeout cascade that follows a recursive reset() without
            # a clean EKF.
            self._kill_episode()
            self._kill_infra()
            self._obs_ready.clear()
            with self._state_lock:
                self._node.mission_state = 'IDLE'
            self._start_infra()
            return self.reset(seed=seed, options=options)

        # 8. Seed d_xy_prev from the TRACKING position (drone has already cruised to marker).
        with self._state_lock:
            pos = self._node.pos_enu.copy()
        self.d_xy_prev = self._compute_d_xy(pos)

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

        # --- 2D horizontal distance ---
        d_xy = self._compute_d_xy(pos)

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
                'rew_ctrl': 0.0, 'rew_dist': 0.0, 'rew_orient': 0.0,
            }

        # EKF drift guard: normal TRACKING entries are 3.2–3.8m; ≥5m means EKF
        # diverged during CRUISE (physically elsewhere, camera will never see marker).
        if self._step_count == 1 and d_xy > 5.0:
            self._node.get_logger().warn(
                f'[EKF Drift] d_xy={d_xy:.1f}m > 5.0m at step 1 — truncating drifted episode.')
            reward = self._cfg_truncation_penalty
            self._episode_reward += reward
            self._needs_infra_restart = True  # EKF won't recover without a full infra restart
            return self._get_obs(), reward, False, True, {
                'truncate_reason': 'ekf_drift',
                'episode_reward': self._episode_reward,
                'rew_ctrl': 0.0, 'rew_dist': 0.0, 'rew_orient': 0.0,
            }

        terminated = False
        truncated = False
        info = {}

        # Print once per episode when RL policy first sees the marker
        if not self._rl_detection_printed and float(pix[2]) > 0.0:
            self._rl_detection_printed = True
            print(f'[YOLO] Marker visible in RL episode (step {self._step_count}) | '
                  f'ENU: ({pos[0]:.1f}, {pos[1]:.1f}, {pos[2]:.1f}) | '
                  f'd_xy: {d_xy:.1f} m | conf: {pix[2]:.2f}', flush=True)

        # ============================================================
        # Layers 1–3: Per-step reward
        # ============================================================
        reward, terminated, info = self._compute_reward(
            pos, vel, ang, pix, d_xy, action)

        # --- Update closest approach tracker ---
        self._d_xy_min = min(self._d_xy_min, d_xy)

        # --- Option A: Success — drone directly overhead ---
        if not terminated and d_xy <= self._cfg_success_radius:
            reward += self._cfg_reward_success
            terminated = True
            info['success'] = True
            info['truncate_reason'] = 'success'
            self._node.get_logger().info(
                f'[RL] SUCCESS: d_xy={d_xy:.2f}m <= {self._cfg_success_radius}m at step {self._step_count}')

        # --- Safety truncation decisions ---
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

        if not truncated and not terminated:
            self._d_xy_history[self._step_count] = d_xy
            if self._step_count >= self._cfg_stagnation_start_step:
                past_step = self._step_count - self._cfg_stagnation_window
                if past_step in self._d_xy_history:
                    past_d_xy = self._d_xy_history[past_step]
                    if past_d_xy - d_xy < self._cfg_stagnation_min_progress:
                        reward += self._cfg_penalty_stagnation
                        truncated = True
                        truncate_reason = 'stagnation'

        # --- Option B: Overshoot — drone was close but flew past ---
        # Fires when the drone got within overshoot_close_threshold metres at some
        # point, then retreated more than overshoot_margin beyond that closest point.
        if not truncated and not terminated:
            if (self._d_xy_min < self._cfg_overshoot_close_threshold
                    and d_xy > self._d_xy_min + self._cfg_overshoot_margin):
                reward += self._cfg_penalty_overshoot
                truncated = True
                truncate_reason = 'overshoot'

        if truncate_reason:
            info['truncate_reason'] = truncate_reason

        # --- Truncation on step limit (timeout) ---
        if not terminated and not truncated and self._step_count >= self._cfg_max_steps:
            truncated = True
            reward += self._cfg_truncation_penalty
            info['truncate_reason'] = 'timeout'

        # --- Update action memory ---
        self.action_prev = np.array(action, dtype=np.float32)

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

    def _compute_d_xy(self, pos):
        """2D horizontal distance from drone to target.

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

    def _compute_reward(self, pos, vel, ang, pix, d_xy, action):
        """Compute per-step reward.

        Layer 1 — Safety:     penalty on crash / overspeed / target lost
        Layer 2 — Stability:  time + angular-velocity + action-smoothness penalties
        Layer 3 — Navigation: GPS distance gradient + heading alignment + vision centering

        Vision centering: reward proportional to how close the X marker is to
        the camera centre.  max(0, 1 - sqrt(u_norm^2 + v_norm^2)) gives 1.0
        when X is centred and 0 at the frame corners.  Scaled by conf so a
        missing detection contributes nothing.

        Args:
            d_xy: 2D horizontal distance to target (metres)

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
        if self._cfg_use_vision and conf == 0.0:
            info['target_lost'] = True
            reward += self._cfg_penalty_target_lost

        # ----------------------------------------------------------------
        # Layer 2 — Efficiency / Stability
        # R2 = -w_time - w_ang_vel * ||omega||^2 - w_action_smooth * ||Δa||^2
        # ----------------------------------------------------------------
        omega_sq = float(np.dot(ang, ang))

        action_arr = np.asarray(action, dtype=np.float32)
        delta_action = action_arr - self.action_prev
        action_smooth_sq = float(np.dot(delta_action, delta_action))

        r2 = (
            -self._cfg_w_time
            - self._cfg_w_ang_vel * omega_sq
            - self._cfg_w_action_smooth * action_smooth_sq
        )

        # ----------------------------------------------------------------
        # Layer 3 — Navigation + Vision Centering
        # R3 = w_dist * (d_prev - d_now)              ← GPS distance gradient
        #      + w_heading * cos_heading * speed_gate  ← heading alignment (0 when w_heading=0)
        #      + w_proximity * max(0, 1 - d_xy/r)     ← dense proximity bonus
        #      + w_vision_center * centering * conf    ← YOLO centering reward
        # ----------------------------------------------------------------

        # Distance gradient (positive when moving toward target)
        r3_dist = self._cfg_w_dist * (self.d_xy_prev - d_xy)

        # Heading alignment: cos(angle between drone velocity and bearing to target)
        vx_2d, vy_2d = float(vel[0]), float(vel[1])
        speed_xy = math.sqrt(vx_2d * vx_2d + vy_2d * vy_2d)
        if speed_xy > 0.1:
            dx_to_target = self._cfg_target_x - float(pos[0])
            dy_to_target = self._cfg_target_y - float(pos[1])
            dist_to_target = math.sqrt(dx_to_target ** 2 + dy_to_target ** 2)
            cos_heading = (
                (vx_2d * dx_to_target + vy_2d * dy_to_target)
                / (speed_xy * dist_to_target)
            ) if dist_to_target > 0.01 else 1.0
        else:
            cos_heading = 0.0

        speed_gate = min(speed_xy / 2.0, 1.0) if self._cfg_speed_gate else 1.0
        r3_orient = self._cfg_w_heading * cos_heading * speed_gate

        # Vision centering: reward X marker being close to camera centre.
        # Gated by proximity so hovering far away yields near-zero reward;
        # the drone must approach before centering pays off.
        # u_norm, v_norm ∈ [-1,1] where 0 = centre of frame.
        if self._cfg_use_vision and conf > 0.0:
            u_n = (pix[0] / 640.0) * 2.0 - 1.0
            v_n = (pix[1] / 480.0) * 2.0 - 1.0
            center_dist = math.sqrt(u_n * u_n + v_n * v_n)   # 0 at centre, √2 at corner
            proximity_factor = max(0.0, 1.0 - d_xy / 30.0)   # 0 at ≥30 m, 1 at 0 m
            r3_vision = self._cfg_w_vision_center * max(0.0, 1.0 - center_dist) * conf * proximity_factor
        else:
            r3_vision = 0.0

        r3_proximity = self._cfg_w_proximity * max(0.0, 1.0 - d_xy / self._cfg_proximity_radius)

        r3 = r3_dist + r3_orient + r3_proximity + r3_vision

        # Advance d_xy_prev for next step
        self.d_xy_prev = d_xy

        reward += r2 + r3
        info['r2'] = r2
        info['r3'] = r3
        info['d_xy'] = d_xy
        info['cos_heading'] = cos_heading
        info['rew_ctrl'] = r2
        info['rew_dist'] = r3_dist
        info['rew_orient'] = r3_orient
        info['rew_proximity'] = r3_proximity
        info['rew_vision'] = r3_vision

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

        pos_n = np.nan_to_num(pos / POS_SCALE, nan=0.0)
        vel_n = np.nan_to_num(vel / VEL_SCALE, nan=0.0)
        ang_n = np.nan_to_num(ang / ANG_VEL_SCALE, nan=0.0)

        if self._cfg_use_vision:
            u_norm = (pix[0] / 640.0) * 2.0 - 1.0 if pix[2] > 0 else 0.0
            v_norm = (pix[1] / 480.0) * 2.0 - 1.0 if pix[2] > 0 else 0.0
            conf = float(np.clip(pix[2], 0.0, 1.0))
        else:
            u_norm = 0.0
            v_norm = 0.0
            conf = 1.0   # synthetic: "target always visible"

        pos_clean = np.nan_to_num(pos, nan=0.0)
        rel_dx = (pos_clean[0] - self._cfg_target_x) / POS_SCALE
        rel_dy = (pos_clean[1] - self._cfg_target_y) / POS_SCALE

        obs = np.array([
            *np.clip(pos_n, -1.0, 1.0),    # 0-2   world position (ENU, normalised)
            *np.clip(vel_n, -1.0, 1.0),    # 3-5   world velocity (ENU, normalised)
            *np.clip(ang_n, -1.0, 1.0),    # 6-8   angular velocity (body, normalised)
            u_norm, v_norm, conf,           # 9-11  vision: pixel u, v, confidence
            np.clip(rel_dx, -1.0, 1.0),    # 12    relative x to target (normalised)
            np.clip(rel_dy, -1.0, 1.0),    # 13    relative y to target (normalised)
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
        """
        gz_up = subprocess.run(
            ['pgrep', '-f', 'gz sim'], capture_output=True).returncode == 0
        px4_up = subprocess.run(
            ['pgrep', '-f', 'bin/px4'], capture_output=True).returncode == 0
        if not (gz_up and px4_up):
            return False
        # Check drone model exists in Gazebo (pre-spawned in world SDF).
        # Catch TimeoutExpired: Gazebo can become sluggish after long runs and
        # hang on this call.  Treat as unhealthy so the recovery path takes over
        # instead of crashing the entire training process.
        try:
            result = subprocess.run(
                ['gz', 'model', '--list'],
                capture_output=True, text=True, timeout=5.0)
            return self._model_name in result.stdout
        except subprocess.TimeoutExpired:
            if rclpy.ok():
                self._node.get_logger().warning(
                    '[RL Env] gz model --list timed out — treating infra as unhealthy')
            return False
        except Exception as exc:  # noqa: BLE001
            if rclpy.ok():
                self._node.get_logger().warning(
                    f'[RL Env] _check_infra_healthy error: {exc!r} — treating as unhealthy')
            return False

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
        if self._cfg_use_vision:
            bridge_cfg_content += (
                f'\n'
                f'- ros_topic_name: /camera/rgb/image_raw\n'
                f'  gz_topic_name: /{self._drop_topic}/down_camera/image_raw\n'
                f'  ros_type_name: sensor_msgs/msg/Image\n'
                f'  gz_type_name: gz.msgs.Image\n'
                f'  direction: GZ_TO_ROS\n'
                f'\n'
                f'- ros_topic_name: /camera/rgb/camera_info\n'
                f'  gz_topic_name: /{self._drop_topic}/down_camera/camera_info\n'
                f'  ros_type_name: sensor_msgs/msg/CameraInfo\n'
                f'  gz_type_name: gz.msgs.CameraInfo\n'
                f'  direction: GZ_TO_ROS\n'
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

        # 2b. YOLO vision node — start alongside PX4 init so model loads during
        #     the ~30s PX4 startup, ready by the time the drone takes off.
        yolo_proc = None
        if self._cfg_use_vision:
            yolo_log = open(f'/tmp/yolo_{iid}.log', 'w')
            yolo_proc = subprocess.Popen(
                ['ros2', 'run', 'vision_detection', 'xmarker_detector',
                 '--ros-args',
                 '-p', 'inference_device:=cuda',
                 '-p', 'inference_rate:=10.0',
                 '-p', f'detection_hold_frames:={self._cfg_detection_hold_frames}'],
                env=infra_env,
                stdout=yolo_log, stderr=subprocess.STDOUT,
                preexec_fn=os.setsid,
            )

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

        self._infra_procs = [p for p in [uxrce, gz, bridge, yolo_proc, px4] if p is not None]
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
                    proc.wait(timeout=5)
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
                    proc.wait(timeout=5)
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
        proc = subprocess.Popen(
            ['ros2', 'run', 'drop_calculator', 'calculator',
             '--ros-args',
             '-p', 'x_marker_x:=11.0',
             '-p', f'x_marker_y:={self._cfg_target_y}'],   # 10.0 + iid * 150.0
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
        """Poll /mission/state until CRUISE or timeout.

        #4: also bail early if PX4 never arms within arm_bail_timeout. ~28% of
        takeoffs get stuck because PX4 rejects the arm (stale EKF after the
        teleport reset) and the drone never leaves the ground; waiting out the
        full cruise_poll_timeout on those is pure wasted wall-clock. A fresh
        infra restart (which the caller does on bail) reliably re-arms.
        """
        deadline = time.time() + self._cfg_cruise_timeout
        arm_deadline = time.time() + self._cfg_arm_bail_timeout
        # Reset baseline: only a fresh ARMED status from this episode's PX4
        # should count (avoids a stale armed=True carrying over from the prior
        # episode and suppressing the bail).
        with self._state_lock:
            self._node.armed = False
        armed_seen = False
        state = 'IDLE'
        while time.time() < deadline:
            if not self._spin_thread.is_alive():
                if rclpy.ok():
                    self._node.get_logger().error(
                        '[RL Env] Spin thread died during _wait_for_cruise — aborting wait')
                return  # caller will detect mission_state != 'CRUISE' and retry
            with self._state_lock:
                state = self._node.mission_state
                armed = self._node.armed
            if state == 'CRUISE':
                return
            if armed:
                armed_seen = True
            if not armed_seen and time.time() > arm_deadline:
                if rclpy.ok():
                    self._node.get_logger().warning(
                        f'[RL Env] PX4 not armed after {self._cfg_arm_bail_timeout:.0f}s '
                        '— arm-reject suspected; bailing early for infra restart.')
                return  # caller detects mission_state != 'CRUISE' and restarts
            time.sleep(0.5)
        if rclpy.ok():
            self._node.get_logger().warning(
                f'Timed out waiting for CRUISE state (got: {state})')

    def _wait_for_tracking(self):
        """Poll /mission/state until TRACKING or timeout.

        Called after CRUISE is confirmed.  The mission_manager transitions
        CRUISE→TRACKING automatically on the first YOLO detection, so this
        simply waits for that to happen before handing control to the RL policy.

        When TRACKING is first seen, immediately publish a zero-velocity hover
        so the drone holds position while reset() finishes — otherwise the drone
        drifts on its cruise momentum, loses the marker within ~1s, and reverts
        to CRUISE before the RL episode can start.
        """
        deadline = time.time() + self._cfg_tracking_wait_timeout
        state = 'CRUISE'
        while time.time() < deadline:
            if not self._spin_thread.is_alive():
                if rclpy.ok():
                    self._node.get_logger().error(
                        '[RL Env] Spin thread died during _wait_for_tracking — aborting wait')
                return
            with self._state_lock:
                state = self._node.mission_state
            if state == 'TRACKING':
                # Hold position so the drone doesn't drift off the marker before
                # the RL policy takes over.
                self._node.publish_velocity(0.0, 0.0, 0.0, 0.0)
                with self._state_lock:
                    pos = self._node.pos_enu.copy()
                pix = self._node.pixel_coords.copy()
                d_xy = self._compute_d_xy(pos)
                print(f'[YOLO] CRUISE→TRACKING | '
                      f'ENU:({pos[0]:.1f},{pos[1]:.1f},{pos[2]:.1f}) | '
                      f'd_xy:{d_xy:.1f}m | '
                      f'bbox_cx:{pix[0]:.0f} bbox_cy:{pix[1]:.0f} conf:{pix[2]:.2f} '
                      f'(img center=320,240  cy>350=fwd cam  cy~240=down cam)',
                      flush=True)
                return
            time.sleep(0.5)
        if rclpy.ok():
            self._node.get_logger().warning(
                f'Timed out waiting for TRACKING state (got: {state})')
