"""Gymnasium environment for drone fly-by drop RL training."""

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
    from px4_msgs.msg import VehicleLocalPosition, VehicleAngularVelocity
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
        else:
            self.get_logger().warning(
                'px4_msgs not available; position/velocity obs will be zeros.')

        self.create_subscription(
            Point, '/target/pixel_coords', self._on_pixel_coords, 10)
        self.create_subscription(
            String, '/mission/state', self._on_mission_state, 10)
        self.create_subscription(
            Odometry, '/drone/payload/position', self._on_payload_pos, 10)
        self.create_subscription(
            Bool, '/drone/payload/drop_cmd_raw', self._on_drop_cmd_raw, 10)

        # Drop error (published by drop_calculator after impact)
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
            # NED->ENU: East=Y_ned, North=X_ned, Up=-Z_ned
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

    def _on_pixel_coords(self, msg):
        with self._lock:
            self.pixel_coords[0] = msg.x   # u
            self.pixel_coords[1] = msg.y   # v
            self.pixel_coords[2] = msg.z   # confidence

    def _on_mission_state(self, msg):
        with self._lock:
            self.mission_state = msg.data

    def _on_payload_pos(self, msg):
        """Track payload position (used for future extensions)."""
        pass

    def _on_drop_cmd_raw(self, msg):
        if not msg.data:   # False = drop event
            with self._lock:
                self.payload_attached = False

    def _on_drop_error(self, msg):
        """Called by drop_calculator after payload impacts ground."""
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
        """Fire payload drop commands."""
        self.detach_pub.publish(Empty())
        drop_msg = Bool()
        drop_msg.data = False   # False = drop event (inverted semantics)
        self.drop_raw_pub.publish(drop_msg)


# ---------------------------------------------------------------------------
# Main Environment
# ---------------------------------------------------------------------------

class DroneDropEnv(gym.Env):
    """Gymnasium env: SAC trains a fighter-jet fly-by drop policy.

    Observation: Box(15,) float32
    Action:      Box(5,)  float32 in [-1, 1]
    """

    metadata = {'render_modes': []}

    def __init__(self, config_path=None):
        super().__init__()

        # --- Load config ---
        cfg_env = {}
        cfg_reward = {}
        if config_path is not None:
            cfg = _load_config(config_path)
            cfg_env = cfg.get('environment', {})
            cfg_reward = cfg.get('reward', {})

        # Environment constants (yaml overrides module-level defaults)
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

        # Reward constants (used in step() for terminal accuracy reward)
        self._cfg_accuracy_reward_scale = cfg_reward.get(
            'accuracy_reward_scale', 2.0)

        self.observation_space = gym.spaces.Box(
            low=-1.0, high=1.0, shape=(15,), dtype=np.float32)
        self.action_space = gym.spaces.Box(
            low=-1.0, high=1.0, shape=(5,), dtype=np.float32)

        # Threading primitives
        self._state_lock = threading.Lock()
        self._obs_ready = threading.Event()
        self._drop_error_event = threading.Event()
        self._drop_error_queue = queue.Queue(maxsize=1)

        # Episode state
        self._step_count = 0
        self._has_dropped = False
        self._drop_reward_pending = 0.0
        self._episode_reward = 0.0
        self._episode_proc = None   # Popen handle for episode.launch.py

        # Start ROS2
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
        self._has_dropped = False
        self._drop_reward_pending = 0.0
        self._episode_reward = 0.0
        self._obs_ready.clear()
        self._drop_error_event.clear()
        # Drain queue
        while not self._drop_error_queue.empty():
            try:
                self._drop_error_queue.get_nowait()
            except queue.Empty:
                break
        with self._state_lock:
            self._node.payload_attached = True
            # Prevent _wait_for_cruise from seeing a stale CRUISE from last episode
            self._node.mission_state = 'IDLE'

        # 2. Kill previous episode processes (PX4 + mission nodes)
        self._kill_episode()

        # 3. Reset Gazebo world (restores all model poses + DetachableJoint)
        self._gz_world_reset()

        # 4. Start fresh episode processes
        self._start_episode()

        # 5. Wait for CRUISE state (blocks until takeoff + climb complete)
        self._wait_for_cruise()

        # 6. Return initial observation
        obs = self._get_obs()
        return obs, {}

    def step(self, action):
        """Apply action, advance one control step, return (obs, reward, term, trunc, info)."""
        self._step_count += 1

        # --- Decode action ---
        vx = float(action[0]) * self._cfg_action_vx_scale
        vy = float(action[1]) * self._cfg_action_vy_scale
        vz = float(action[2]) * self._cfg_action_vz_scale
        yaw_rate = float(action[3]) * self._cfg_action_yaw_scale
        drop_trigger = float(action[4])

        # --- Execute velocity command ---
        self._node.publish_velocity(vx, vy, vz, yaw_rate)

        # --- Drop logic (once per episode) ---
        drop_fired_this_step = False
        if drop_trigger > 0.0 and not self._has_dropped:
            self._node.publish_drop()
            self._has_dropped = True
            drop_fired_this_step = True

        # --- Wait for next observation ---
        self._obs_ready.clear()
        self._obs_ready.wait(timeout=self._cfg_obs_wait)
        obs = self._get_obs()

        # --- Reward ---
        reward = self._compute_reward(obs, drop_fired_this_step)

        # --- Termination conditions ---
        terminated = False
        truncated = False
        info = {}

        # Wait for drop_error if we have dropped
        if self._has_dropped:
            got_error = self._drop_error_event.wait(timeout=self._cfg_obs_wait)
            if got_error:
                try:
                    drop_error = self._drop_error_queue.get_nowait()
                    accuracy_reward = -drop_error * self._cfg_accuracy_reward_scale
                    reward += accuracy_reward
                    info['drop_error_m'] = drop_error
                    info['accuracy_reward'] = accuracy_reward
                except queue.Empty:
                    pass
                terminated = True

        # Crash detection
        altitude = float(obs[2]) * self._cfg_pos_scale
        if self._step_count > self._cfg_min_alt_start and altitude < self._cfg_min_altitude:
            terminated = True
            info['crash'] = True

        # Step limit
        if self._step_count >= self._cfg_max_steps:
            truncated = True

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

        # Pixel coordinates normalised to [-1, 1]
        u_norm = (pix[0] / 640.0) * 2.0 - 1.0 if pix[2] > 0 else 0.0
        v_norm = (pix[1] / 480.0) * 2.0 - 1.0 if pix[2] > 0 else 0.0
        conf = float(np.clip(pix[2], 0.0, 1.0))

        rel_dx = (pos[0] - self._cfg_target_x) / POS_SCALE
        rel_dy = (pos[1] - self._cfg_target_y) / POS_SCALE

        obs = np.array([
            *np.clip(pos_n, -1.0, 1.0),    # 0-2
            *np.clip(vel_n, -1.0, 1.0),    # 3-5
            *np.clip(ang_n, -1.0, 1.0),    # 6-8
            u_norm, v_norm, conf,           # 9-11
            attached,                       # 12
            np.clip(rel_dx, -1.0, 1.0),    # 13
            np.clip(rel_dy, -1.0, 1.0),    # 14
        ], dtype=np.float32)
        return obs

    # ------------------------------------------------------------------
    # Reward computation
    # ------------------------------------------------------------------

    def _compute_reward(self, obs, drop_fired_this_step):
        """TODO: implement per-step reward function."""
        return 0.0

    # ------------------------------------------------------------------
    # Helpers
    # ------------------------------------------------------------------

    def _kill_episode(self):
        """Terminate the episode-layer process group (PX4 + mission nodes)."""
        if self._episode_proc is not None and self._episode_proc.poll() is None:
            try:
                os.killpg(os.getpgid(self._episode_proc.pid), signal.SIGTERM)
                self._episode_proc.wait(timeout=5)
            except (ProcessLookupError, subprocess.TimeoutExpired):
                try:
                    os.killpg(os.getpgid(self._episode_proc.pid), signal.SIGKILL)
                except ProcessLookupError:
                    pass
        # Brief pause to let DDS participants deregister
        time.sleep(1.5)

    def _start_episode(self):
        """Launch a fresh episode-layer process group (PX4 + mission nodes)."""
        self._episode_proc = subprocess.Popen(
            ['ros2', 'launch', 'mission_manager', 'episode.launch.py',
             'rl_mode:=true'],
            stdout=subprocess.DEVNULL,
            stderr=subprocess.DEVNULL,
            preexec_fn=os.setsid,   # own process group -> clean kill
        )

    def _gz_world_reset(self):
        """Call gz service to reset the simulation world."""
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

    def _wait_for_cruise(self):
        """Poll mission_state until CRUISE or timeout."""
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
