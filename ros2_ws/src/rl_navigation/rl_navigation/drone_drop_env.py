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

    def _on_local_pos(self, msg):
        """Convert NED to ENU and update shared state.

        NaN guard: PX4 EKF can output NaN velocity during DDS time-sync resets
        (RTF>1). Replace NaN with 0.0 to prevent observation/reward corruption.
        """
        with self._lock:
            # NED→ENU: East=Y_ned, North=X_ned, Up=−Z_ned
            self.pos_enu[0] = msg.y if math.isfinite(msg.y) else self.pos_enu[0]
            self.pos_enu[1] = msg.x if math.isfinite(msg.x) else self.pos_enu[1]
            self.pos_enu[2] = -msg.z if math.isfinite(msg.z) else self.pos_enu[2]
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

    Observation: Box(15,) float32
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
        # Method A: per-instance model names.
        # PX4_SIM_MODEL=gz_x500_bombard_rN → rcS matches airframe 4016_gz_x500_bombard_r0.
        # PX4 gz_bridge strips "gz_" prefix → spawns model "x500_bombard_rN" in Gazebo,
        # then appends "_N" (instance suffix) → final Gazebo entity name: x500_bombard_rN_N.
        # Payloads are pre-spawned as payload_N.
        # Drop topic is /x500_N/drop (hardcoded in DetachableJoint SDF).
        self._px4_sim_model = f'gz_x500_bombard_r{instance_id}'   # PX4_SIM_MODEL env var
        self._model_name = f'x500_bombard_r{instance_id}_{instance_id}'  # Gazebo entity name
        self._drop_topic = f'x500_{instance_id}'                   # drop gz-topic prefix
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
        self._cfg_max_steps = cfg_env.get('max_steps', 500)
        self._cfg_min_altitude = cfg_env.get('min_altitude', 2.0)
        self._cfg_min_alt_start = cfg_env.get('min_altitude_start_step', 20)
        self._cfg_obs_wait = cfg_env.get('obs_wait_timeout', 0.15)
        self._cfg_cruise_timeout = cfg_env.get('cruise_poll_timeout', 60.0)
        # use_vision=False: skip camera termination + synthesise conf=1.0 in obs
        self._cfg_use_vision = cfg_env.get('use_vision', True)

        # --- Reward constants (4-Layer Hierarchical) ---
        r = cfg_reward
        self._cfg_g = r.get('g', 9.81)
        self._cfg_auto_drop_threshold = r.get('auto_drop_threshold', 0.5)
        self._cfg_k1_potential = r.get('k1_potential', 1.0)
        self._cfg_k2_precision = r.get('k2_precision', 5.0)
        self._cfg_w_dist = r.get('w_dist', 10.0)
        self._cfg_w_heading = r.get('w_heading', 1.0)
        self._cfg_w_time = r.get('w_time', 0.01)
        self._cfg_w_ang_vel = r.get('w_ang_vel', 0.05)
        self._cfg_w_action_smooth = r.get('w_action_smooth', 0.05)
        self._cfg_w_drop_base = r.get('w_drop_base', 50.0)
        self._cfg_r_success_jackpot = r.get('r_success_jackpot', 100.0)
        self._cfg_success_threshold = r.get('success_threshold', 0.1)
        self._cfg_penalty_instability = r.get('penalty_instability', 50.0)
        self._cfg_limit_ang_vel = r.get('limit_ang_vel', 2.0)
        self._cfg_limit_tilt = r.get('limit_tilt', 0.26)
        # Layer 1 penalties (no hard termination)
        self._cfg_penalty_crash = r.get('penalty_crash', -10.0)
        self._cfg_penalty_overspeed = r.get('penalty_overspeed', -8.0)
        self._cfg_penalty_target_lost = r.get('penalty_target_lost', -10.0)
        # Layer 4: time to wait for actual physics landing result from drop_calculator
        self._cfg_drop_wait_timeout = r.get('drop_wait_timeout', 10.0)

        self.observation_space = gym.spaces.Box(
            low=-1.0, high=1.0, shape=(15,), dtype=np.float32)
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

        # --- Infra state ---
        self._infra_procs = []        # Popen handles for infra processes

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
            target=rclpy.spin, args=(self._node,), daemon=True)
        self._spin_thread.start()

        # --- Self-managed infrastructure ---
        self._start_infra()

    # ------------------------------------------------------------------
    # Gymnasium API
    # ------------------------------------------------------------------

    def reset(self, *, seed=None, options=None):
        """Reset simulation and return initial observation."""
        super().reset(seed=seed)

        # 1. Clear episode state
        self._step_count = 0
        self.dropped = False
        self._episode_reward = 0.0
        self._obs_ready.clear()
        self._drop_error_event.clear()
        self.action_prev = np.zeros(5, dtype=np.float32)

        # Drain drop-error queue
        while not self._drop_error_queue.empty():
            try:
                self._drop_error_queue.get_nowait()
            except queue.Empty:
                break

        with self._state_lock:
            self._node.payload_attached = True
            self._node.mission_state = 'IDLE'  # avoid stale CRUISE from last episode

        # 2. Kill previous episode processes.
        self._kill_episode()

        # 3. Teleport drone + payload back to upright spawn position.
        #    Root cause: PX4 auto-arms during SAC init (~70s) and flies
        #    without an OFFBOARD position controller → drone drifts and
        #    flips (roll=π).  An inverted drone pushes DOWN with props so
        #    TAKEOFF never reaches altitude → perpetual CRUISE timeouts.
        #    Teleporting before each episode resets to known-good state.
        #    EKF settles during the 5s drone_controller warmup + episode
        #    node startup (~2s), so arm_ned_z is valid by arming time.
        self._gz_reset_poses()

        # 4. Start fresh episode processes
        self._start_episode()

        # 6. Wait for CRUISE state (blocks until takeoff + climb complete)
        self._wait_for_cruise()

        # 7. Seed d_xy_prev from the initial CRUISE position (no kinematic prediction)
        with self._state_lock:
            pos = self._node.pos_enu.copy()
        self.d_xy_prev = self._compute_d_xy(pos)

        obs = self._get_obs()
        return obs, {}

    def step(self, action):
        """Apply action, advance one control step, return (obs, reward, term, trunc, info)."""
        self._step_count += 1

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

        # --- 2D horizontal distance to target (no kinematic prediction) ---
        d_xy = self._compute_d_xy(pos)

        # --- Physics explosion guard ---
        # d_xy > 500 m means Gazebo had a coordinate explosion (physics glitch).
        # Terminate immediately with a large penalty so no corrupted transition
        # enters the replay buffer beyond this step.
        if d_xy > 500.0:
            reward = -100.0
            self._episode_reward += reward
            return self._get_obs(), reward, True, False, {
                'physics_glitch': True,
                'd_xy': d_xy,
                'episode_reward': self._episode_reward,
                'rew_ctrl': 0.0, 'rew_dist': 0.0,
                'rew_orient': 0.0, 'rew_drop': 0.0,
            }

        terminated = False
        truncated = False
        info = {}

        # --- Drop decision: auto-drop when drone is directly over target ---
        # Phase 1 curriculum: manual drop disabled. action[4] is kept as a
        # dummy dimension (always ignored) for checkpoint compatibility.
        # Phase 2 will re-enable once policy navigates to target reliably.
        # manual_drop = float(action[4]) > 0.0
        if d_xy <= self._cfg_auto_drop_threshold and not self.dropped:
            # ============================================================
            # Layer 4: Terminal drop accuracy reward (ACTUAL physics result)
            # ============================================================
            self._node.publish_drop()
            self.dropped = True

            # Wait for actual physics-based landing result from drop_calculator.
            # drop_calculator tracks /drone/payload/position until z <= 0.04m,
            # then publishes actual miss distance to /rl/drop_error.
            # At 5m altitude, payload falls ~1s; 10s timeout gives ample margin.
            got_result = self._drop_error_event.wait(
                timeout=self._cfg_drop_wait_timeout)
            actual_error = 99.0   # penalty if timeout (no landing detected)
            if got_result:
                try:
                    actual_error = self._drop_error_queue.get_nowait()
                except queue.Empty:
                    pass  # race: event set but queue empty → keep 99.0

            d_error = actual_error   # real Gazebo physics result, NOT kinematic

            # Base precision reward: w_drop_base * exp(-k2 * d_error)
            reward = self._cfg_w_drop_base * math.exp(
                -self._cfg_k2_precision * d_error)

            # Jackpot: bonus for high-precision drop
            if d_error <= self._cfg_success_threshold:
                reward += self._cfg_r_success_jackpot
                info['jackpot'] = True

            # Instability penalty: large angular velocity or excessive tilt
            omega_mag = float(np.linalg.norm(ang))
            if (omega_mag > self._cfg_limit_ang_vel
                    or abs(roll) > self._cfg_limit_tilt
                    or abs(pitch) > self._cfg_limit_tilt):
                reward -= self._cfg_penalty_instability
                info['instability_penalty'] = True

            info['drop_error_actual_m'] = d_error
            info['is_success'] = bool(d_error <= 0.5)
            info['layer4_reward'] = reward
            # Reward component keys (consistent schema with non-terminal steps)
            info['rew_drop'] = reward      # full Layer 4 reward
            info['rew_ctrl'] = 0.0
            info['rew_dist'] = 0.0
            info['rew_orient'] = 0.0
            info['d_xy'] = d_xy            # drone was at this distance when dropped
            terminated = True

        else:
            # ============================================================
            # Layers 1–3: Per-step reward for non-terminal steps
            # ============================================================
            reward, terminated, info = self._compute_reward(
                pos, vel, ang, pix, d_xy, action)

        # --- Truncation on step limit ---
        if self._step_count >= self._cfg_max_steps:
            truncated = True

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

    def _compute_reward(self, pos, vel, ang, pix, d_xy, action):
        """Compute per-step reward for non-terminal steps.

        Layer 1 — Safety:     penalty on crash / overspeed / target lost (no termination)
        Layer 2 — Stability:  time + angular-velocity + action-smoothness penalties
        Layer 3 — Approach:   2D distance gradient + heading alignment reward

        Args:
            d_xy: 2D horizontal distance to target (from _compute_d_xy, no kinematics)

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

        if self._step_count > self._cfg_min_alt_start and altitude < self._cfg_min_altitude:
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
            -self._cfg_w_time
            - self._cfg_w_ang_vel * omega_sq
            - self._cfg_w_action_smooth * action_smooth_sq
        )

        # ----------------------------------------------------------------
        # Layer 3 — Approach
        # R3 = w_dist * (exp(-k1 * d_now) - exp(-k1 * d_prev))
        #      + w_heading * cos(angle between drone heading and bearing to target)
        # ----------------------------------------------------------------

        # Distance gradient (positive when d_xy decreases toward target)
        r3_dist = self._cfg_w_dist * (
            math.exp(-self._cfg_k1_potential * d_xy)
            - math.exp(-self._cfg_k1_potential * self.d_xy_prev)
        )

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

        r3 = r3_dist + self._cfg_w_heading * cos_heading

        # Advance d_xy_prev for next step
        self.d_xy_prev = d_xy

        reward += r2 + r3
        info['r2'] = r2
        info['r3'] = r3
        info['d_xy'] = d_xy
        info['cos_heading'] = cos_heading
        # Split reward components for per-rollout WandB monitoring
        info['rew_ctrl'] = r2
        info['rew_dist'] = r3_dist
        info['rew_orient'] = self._cfg_w_heading * cos_heading
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
        rel_dx = (pos_clean[0] - self._cfg_target_x) / POS_SCALE
        rel_dy = (pos_clean[1] - self._cfg_target_y) / POS_SCALE

        obs = np.array([
            *np.clip(pos_n, -1.0, 1.0),    # 0-2  world position (ENU, normalised)
            *np.clip(vel_n, -1.0, 1.0),    # 3-5  world velocity (ENU, normalised)
            *np.clip(ang_n, -1.0, 1.0),    # 6-8  angular velocity (body, normalised)
            u_norm, v_norm, conf,           # 9-11 vision: pixel u, v, confidence
            attached,                       # 12   payload attached flag
            np.clip(rel_dx, -1.0, 1.0),    # 13   relative x to target (normalised)
            np.clip(rel_dy, -1.0, 1.0),    # 14   relative y to target (normalised)
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
        # Check drone model exists in Gazebo (pre-spawned in world SDF)
        result = subprocess.run(
            ['gz', 'model', '--list'],
            capture_output=True, text=True, timeout=5.0)
        return self._model_name in result.stdout

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
            gz_log = open(f'/tmp/gz_{iid}.log', 'w')
            gz = subprocess.Popen(
                ['gz', 'sim', '-r', '-s',
                 os.path.join(worlds_dir, 'x_marker_world.sdf')],
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

        # 3. PX4 SITL — spawn drone model dynamically (not pre-spawned in world SDF).
        #    Method A: payloads are pre-spawned but DRONES are spawned by PX4.
        #    Pre-spawning all 4 drones causes ODE crash at physics step 1 (all
        #    motor plugins activate simultaneously). PX4_SIM_MODEL spawn inserts
        #    the drone after Gazebo is running — proven stable in Phase 1.5.
        iid_y = self._instance_id * 150
        px4_env = infra_env.copy()
        px4_env.update({
            'PX4_GZ_STANDALONE': '1',
            'PX4_GZ_WORLD': 'x_marker_world',
            'PX4_SIM_MODEL': self._px4_sim_model,  # gz_x500_bombard_rN → spawns x500_bombard_rN_N
            'PX4_GZ_MODEL_POSE': f'0,{iid_y},0.5,0,0,0',  # 0.5m above ground to prevent geometry overlap on spawn
            'PX4_SIM_SPEED_FACTOR': '1',
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
        """Kill all infrastructure processes in reverse order."""
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

        # Clean PX4 lock/socket files for this instance
        iid = self._instance_id
        for path in [f'/tmp/px4_lock-{iid}', f'/tmp/px4-sock-{iid}']:
            try:
                os.remove(path)
            except FileNotFoundError:
                pass

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
            ['ros2', 'run', 'mission_manager', 'mission_manager_node'],
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
        # pose snap.  3 s here + 2 s arm delay (40 ticks x 20 Hz in drone_controller)
        # = 5 s total before arming — sufficient for EKF to reconverge after the
        # world-reset position discontinuity.
        time.sleep(3.0)

    def _wait_for_cruise(self):
        """Poll /mission/state until CRUISE or timeout."""
        deadline = time.time() + self._cfg_cruise_timeout
        while time.time() < deadline:
            with self._state_lock:
                state = self._node.mission_state
            if state == 'CRUISE':
                return
            time.sleep(0.5)
        if rclpy.ok():
            self._node.get_logger().warning(
                f'Timed out waiting for CRUISE state (got: {state})')
