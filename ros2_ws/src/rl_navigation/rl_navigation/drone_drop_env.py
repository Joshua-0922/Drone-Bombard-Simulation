"""Gymnasium environment for drone fly-by drop RL training.

Reward structure — 4-Layer Hierarchical:
  Layer 1: Safety    — penalty on crash / overspeed / target lost (no hard termination)
  Layer 2: Stability — per-step time, angular-velocity and action-smoothness penalties
  Layer 3: Approach  — predicted-impact distance gradient + heading alignment
  Layer 4: Terminal  — kinematic drop accuracy + jackpot + instability penalty
"""

import math
import queue
import os
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
                 drop_error_event, drop_error_queue):
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
                '/fmu/out/vehicle_local_position',
                self._on_local_pos,
                px4_qos,
            )
            self.create_subscription(
                VehicleAngularVelocity,
                '/fmu/out/vehicle_angular_velocity',
                self._on_ang_vel,
                px4_qos,
            )
            self.create_subscription(
                VehicleAttitude,
                '/fmu/out/vehicle_attitude',
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

    # ------------------------------------------------------------------
    # Subscriber callbacks
    # ------------------------------------------------------------------

    def _on_local_pos(self, msg):
        """Convert NED to ENU and update shared state."""
        with self._lock:
            # NED→ENU: East=Y_ned, North=X_ned, Up=−Z_ned
            self.pos_enu[0] = msg.y
            self.pos_enu[1] = msg.x
            self.pos_enu[2] = -msg.z
            self.vel_enu[0] = msg.vy
            self.vel_enu[1] = msg.vx
            self.vel_enu[2] = -msg.vz
        self._obs_ready.set()

    def _on_ang_vel(self, msg):
        with self._lock:
            self.ang_vel[0] = msg.xyz[0]
            self.ang_vel[1] = msg.xyz[1]
            self.ang_vel[2] = msg.xyz[2]

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

    def __init__(self, config_path=None):
        super().__init__()

        # --- Load config ---
        cfg_env = {}
        cfg_reward = {}
        if config_path is not None:
            cfg = _load_config(config_path)
            cfg_env = cfg.get('environment', {})
            cfg_reward = cfg.get('reward', {})

        # --- Environment constants ---
        self._cfg_target_x = cfg_env.get('target_enu_x', TARGET_ENU_X)
        self._cfg_target_y = cfg_env.get('target_enu_y', TARGET_ENU_Y)
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
        self._episode_proc = None     # Popen handle for episode.launch.py
        self.d_impact_prev = 0.0      # predicted impact distance at previous step
        self.action_prev = np.zeros(5, dtype=np.float32)

        # --- Start ROS2 ---
        if not rclpy.ok():
            rclpy.init()
        self._node = _RLBridgeNode(
            self._state_lock,
            self._obs_ready,
            self._drop_error_event,
            self._drop_error_queue,
        )
        self._spin_thread = threading.Thread(
            target=rclpy.spin, args=(self._node,), daemon=True)
        self._spin_thread.start()

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

        # 2. Kill previous episode processes
        self._kill_episode()

        # 3. Reset Gazebo world (restores all poses + DetachableJoint)
        self._gz_world_reset()

        # 4. Start fresh episode processes
        self._start_episode()

        # 5. Wait for CRUISE state (blocks until takeoff + climb complete)
        self._wait_for_cruise()

        # 6. Seed d_impact_prev from the initial state
        with self._state_lock:
            pos = self._node.pos_enu.copy()
            vel = self._node.vel_enu.copy()
        _, _, _, self.d_impact_prev = self._predict_impact_point(pos, vel)

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

        # --- Kinematic predictor (called every step) ---
        _, _, _, d_impact = self._predict_impact_point(pos, vel)

        terminated = False
        truncated = False
        info = {}

        # --- Drop decision: auto-drop by threshold OR manual policy trigger ---
        manual_drop = float(action[4]) > 0.0
        if (d_impact <= self._cfg_auto_drop_threshold or manual_drop) and not self.dropped:
            # ============================================================
            # Layer 4: Terminal drop accuracy reward
            # ============================================================
            self._node.publish_drop()
            self.dropped = True

            d_error = d_impact   # predicted miss distance at drop moment

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

            info['drop_error_predicted_m'] = d_error
            info['layer4_reward'] = reward
            terminated = True

        else:
            # ============================================================
            # Layers 1–3: Per-step reward for non-terminal steps
            # ============================================================
            reward, terminated, info = self._compute_reward(
                pos, vel, ang, pix, d_impact, action)

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
        """Shutdown ROS2 node and executor."""
        self._kill_episode()
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

    # ------------------------------------------------------------------
    # Reward computation — Layers 1, 2, 3 (non-terminal steps)
    # ------------------------------------------------------------------

    def _compute_reward(self, pos, vel, ang, pix, d_impact, action):
        """Compute per-step reward for non-terminal steps.

        Layer 1 — Safety:     penalty on crash / overspeed / target lost (no termination)
        Layer 2 — Stability:  time + angular-velocity + action-smoothness penalties
        Layer 3 — Approach:   predicted-impact gradient + heading alignment reward

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

        # Distance gradient (positive when d_impact decreases)
        r3_dist = self._cfg_w_dist * (
            math.exp(-self._cfg_k1_potential * d_impact)
            - math.exp(-self._cfg_k1_potential * self.d_impact_prev)
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

        # Advance d_impact_prev for next step
        self.d_impact_prev = d_impact

        reward += r2 + r3
        info['r2'] = r2
        info['r3'] = r3
        info['d_impact'] = d_impact
        info['cos_heading'] = cos_heading

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

        pos_n = pos / POS_SCALE
        vel_n = vel / VEL_SCALE
        ang_n = ang / ANG_VEL_SCALE

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

        rel_dx = (pos[0] - self._cfg_target_x) / POS_SCALE
        rel_dy = (pos[1] - self._cfg_target_y) / POS_SCALE

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
    # Episode lifecycle helpers
    # ------------------------------------------------------------------

    def _kill_episode(self):
        """Terminate the episode-layer process group (mission nodes only).

        PX4 SITL is persistent in infra.launch.py and is NOT killed here.
        """
        if self._episode_proc is not None and self._episode_proc.poll() is None:
            try:
                os.killpg(os.getpgid(self._episode_proc.pid), signal.SIGTERM)
                self._episode_proc.wait(timeout=5)
            except (ProcessLookupError, subprocess.TimeoutExpired):
                try:
                    os.killpg(os.getpgid(self._episode_proc.pid), signal.SIGKILL)
                except ProcessLookupError:
                    pass
        # Brief pause to let DDS participants deregister (reduced from 1.5 s)
        time.sleep(0.5)

    def _start_episode(self):
        """Launch a fresh episode-layer process group (mission nodes only).

        PX4 SITL is persistent in infra.launch.py and is already running.
        """
        self._episode_proc = subprocess.Popen(
            ['ros2', 'launch', 'mission_manager', 'episode.launch.py',
             'rl_mode:=true'],
            stdout=subprocess.DEVNULL,
            stderr=subprocess.DEVNULL,
            preexec_fn=os.setsid,   # own process group → clean SIGTERM cascade
        )

    def _gz_world_reset(self):
        """Call gz service to reset the simulation world.

        After reset, all entity poses (including the drone) snap back to their
        spawn positions. PX4 SITL (persistent in infra) sees this as a sudden
        position jump in its EKF. The 3-second sleep gives PX4 time to absorb
        the snap and stabilise before mission nodes re-arm the drone.
        """
        try:
            subprocess.run(
                [
                    'gz', 'service',
                    '-s', '/world/x_marker_world/control',
                    '--reqtype', 'gz.msgs.WorldControl',
                    '--reptype', 'gz.msgs.Boolean',
                    '--timeout', '3000',
                    '--req', 'reset: {all: true}',
                ],
                timeout=5.0,
                check=False,
            )
        except subprocess.TimeoutExpired:
            if rclpy.ok():
                self._node.get_logger().warning('gz world reset timed out')
        # Allow PX4 EKF to stabilise after the pose snap before mission nodes start
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
