#!/usr/bin/env python3
"""mission_manager_rad — RAD framework 의 mission manager.

기존 mission_manager 와 차이:
  1. Spawn yaw 랜덤화 (uniform [-π/2, +π/2] relative to drone→target vector)
  2. Cruise = head 방향 1 m/s 까지 가속 (기존 자동 이동 폐기)
  3. Yaw 고정 유지 (cruise 도중 회전 안 함)

State machine:
  TAKEOFF (climb to 5m hover)
    ↓
  YAW_INIT (랜덤 yaw 설정)
    ↓
  CRUISE (head 방향 1 m/s 까지 가속)
    ↓
  HANDOFF (RL 정책으로 control 인수)
    ↓
  TRACK (RL 정책 control, mission_manager 는 state 만 publish)
    ↓
  DROP (drop 발동 시 hover)
"""
import math
import random
import time

import rclpy
from geometry_msgs.msg import Twist, Vector3
from px4_msgs.msg import VehicleLocalPosition, VehicleStatus
from rclpy.node import Node
from rclpy.qos import (
    DurabilityPolicy,
    HistoryPolicy,
    QoSProfile,
    ReliabilityPolicy,
)
from std_msgs.msg import Bool, String

# --- 상태 상수 ---
STATE_TAKEOFF = "TAKEOFF"
STATE_YAW_INIT = "YAW_INIT"
STATE_CRUISE = "CRUISE"
STATE_HANDOFF = "HANDOFF"      # 잠시 HANDOFF (RL 정책 인수 시점)
STATE_TRACK = "TRACKING"
STATE_DROP = "DROP"

# --- Target 위치 (지면 marker, RAD design) ---
TARGET_ENU_X = 4.0
TARGET_ENU_Y = 3.0
TARGET_ALT = 5.0                   # 호버 고도

# --- Cruise ---
CRUISE_TARGET_SPEED = 1.0          # m/s
YAW_RELATIVE_RANGE = math.pi / 2   # ±90°


class MissionManagerRADNode(Node):
    def __init__(self):
        super().__init__('mission_manager_rad_node')

        # --- Parameters (overridable via launch) ---
        self.declare_parameter('target_enu_x', TARGET_ENU_X)
        self.declare_parameter('target_enu_y', TARGET_ENU_Y)
        self.declare_parameter('target_altitude', TARGET_ALT)
        self.declare_parameter('cruise_target_speed', CRUISE_TARGET_SPEED)
        self.declare_parameter('yaw_relative_range', YAW_RELATIVE_RANGE)
        self.declare_parameter('spawn_yaw_random_enabled', True)

        self.target_enu_x = float(self.get_parameter('target_enu_x').value)
        self.target_enu_y = float(self.get_parameter('target_enu_y').value)
        self.target_altitude = float(self.get_parameter('target_altitude').value)
        self.cruise_target_speed = float(self.get_parameter('cruise_target_speed').value)
        self.yaw_relative_range = float(self.get_parameter('yaw_relative_range').value)
        self.spawn_yaw_random_enabled = bool(
            self.get_parameter('spawn_yaw_random_enabled').value)

        # --- Publishers ---
        self.pos_pub = self.create_publisher(Vector3, '/drone/cmd/position', 10)
        self.vel_pub = self.create_publisher(Twist, '/drone/cmd/velocity', 10)
        self.yaw_pub = self.create_publisher(Vector3, '/drone/cmd/yaw_setpoint', 10)
        # Issue #028 옵션 5: mission_state QoS default (VOLATILE + RELIABLE + KEEP_LAST 10).
        # TRANSIENT_LOCAL 폐기 — 재시작 시 latched stale 문제. control_loop @10Hz publish
        # 이라 latching 불필요.
        self.state_pub = self.create_publisher(String, '/mission/state', 10)
        self.handoff_pub = self.create_publisher(Bool, '/mission/handoff_to_rl', 10)

        # --- Subscribers ---
        self.drop_sub = self.create_subscription(
            Bool, '/drone/payload/drop_cmd_raw', self.drop_callback, 10)

        qos = QoSProfile(reliability=ReliabilityPolicy.BEST_EFFORT, depth=1)
        self.local_pos_sub = self.create_subscription(
            VehicleLocalPosition, '/fmu/out/vehicle_local_position',
            self.local_pos_callback, qos)
        self.status_sub = self.create_subscription(
            VehicleStatus, '/fmu/out/vehicle_status',
            self.vehicle_status_callback, qos)

        # --- 내부 변수 ---
        self.state = STATE_TAKEOFF
        self.current_pos = [0.0, 0.0, 0.0]   # NED
        self.current_vel = [0.0, 0.0, 0.0]   # NED
        self.px4_armed = False
        self.armed_stamp = None
        self.arm_ned_z = None
        self.on_ground_at_arm = None
        self.altitude_hold_ticks = 0

        # Spawn yaw (랜덤 선택, YAW_INIT 단계에서 결정)
        self.spawn_yaw = 0.0                  # ENU yaw (radian)
        self.cruise_target_x = 0.0            # cruise hover XY
        self.cruise_target_y = 0.0
        # RAD: cruise timeout (Issue #028) — EKF stale 로 speed_xy 도달 못 해도 강제 HANDOFF
        self.cruise_start_time = None         # _step_cruise 첫 진입 시 set

        # --- Main Loop (10 Hz) ---
        self.timer = self.create_timer(0.1, self.control_loop)
        self.get_logger().info(
            f"MissionManagerRAD started — target=({self.target_enu_x:.1f}, "
            f"{self.target_enu_y:.1f}, 0.0), cruise_speed={self.cruise_target_speed:.2f}"
        )

    # =================================================================
    # Callbacks
    # =================================================================

    def local_pos_callback(self, msg):
        self.current_pos = [msg.x, msg.y, msg.z]      # NED
        self.current_vel = [msg.vx, msg.vy, msg.vz]   # NED

    def vehicle_status_callback(self, msg):
        if not self.px4_armed and msg.arming_state == 2:
            self.arm_ned_z = self.current_pos[2]
            self.on_ground_at_arm = True
            self.armed_stamp = time.time()
            self.get_logger().info(
                f"PX4 armed at NED z={self.arm_ned_z:.2f} — TAKEOFF.")
        self.px4_armed = (msg.arming_state == 2)

    def drop_callback(self, msg):
        if not msg.data:
            self.get_logger().warn(">>> DROP CONFIRMED! <<<")
            self.state = STATE_DROP

    # =================================================================
    # Main Loop
    # =================================================================

    def control_loop(self):
        # Publish state
        state_msg = String()
        state_msg.data = self.state
        self.state_pub.publish(state_msg)

        # --- FSM ---
        if self.state == STATE_TAKEOFF:
            self._step_takeoff()
        elif self.state == STATE_YAW_INIT:
            self._step_yaw_init()
        elif self.state == STATE_CRUISE:
            self._step_cruise()
        elif self.state == STATE_HANDOFF:
            self._step_handoff()
        elif self.state == STATE_TRACK:
            # RL 정책이 /drone/cmd/velocity 담당. 우리는 state publish 만.
            pass
        elif self.state == STATE_DROP:
            # Hover
            self.send_velocity_cmd(0.0, 0.0, 0.0, 0.0)

    # ---- TAKEOFF ----
    def _step_takeoff(self):
        if not self.px4_armed:
            return
        if self.on_ground_at_arm is False:
            return
        self.send_position_cmd(0.0, 0.0, self.target_altitude)

        min_armed_secs = 0.3
        armed_long_enough = (
            self.armed_stamp is not None
            and (time.time() - self.armed_stamp) >= min_armed_secs
        )
        arm_z = self.arm_ned_z if self.arm_ned_z is not None else 0.0
        altitude_threshold = max(arm_z, 0.0) - (self.target_altitude * 0.95)
        if armed_long_enough and self.current_pos[2] <= altitude_threshold:
            self.altitude_hold_ticks += 1
        else:
            self.altitude_hold_ticks = 0
        if self.altitude_hold_ticks >= 2:
            self.get_logger().info("Altitude Reached → YAW_INIT")
            self.state = STATE_YAW_INIT
            self.altitude_hold_ticks = 0
            self.cruise_target_x = self.current_pos[0]   # NED x
            self.cruise_target_y = self.current_pos[1]   # NED y

    # ---- YAW_INIT ----
    def _step_yaw_init(self):
        """Spawn yaw 랜덤 결정 + yaw setpoint publish + 다음 cruise tick 까지 hover.

        yaw 의 의미:
          drone heading (정면) 의 ENU yaw (radian). 0 = East, π/2 = North.

        Random 절차:
          1. drone → target vector 의 ENU yaw 계산:
               base_yaw = atan2(target_y − drone_y, target_x − drone_x)
          2. Random offset ∈ uniform[-π/2, +π/2]
          3. spawn_yaw = base_yaw + offset
        """
        # drone_x, drone_y 는 ENU (NED → ENU 변환: ENU.x = NED.y, ENU.y = NED.x)
        drone_enu_x = self.current_pos[1]   # NED y → ENU x
        drone_enu_y = self.current_pos[0]   # NED x → ENU y

        base_yaw = math.atan2(
            self.target_enu_y - drone_enu_y,
            self.target_enu_x - drone_enu_x,
        )

        if self.spawn_yaw_random_enabled:
            offset = random.uniform(-self.yaw_relative_range, self.yaw_relative_range)
            self.spawn_yaw = base_yaw + offset
        else:
            self.spawn_yaw = base_yaw   # 정면 (랜덤 비활성)

        # Wrap to [-π, π]
        self.spawn_yaw = math.atan2(math.sin(self.spawn_yaw), math.cos(self.spawn_yaw))

        self.get_logger().info(
            f"Spawn yaw set: base={math.degrees(base_yaw):.1f}°, "
            f"final={math.degrees(self.spawn_yaw):.1f}° → CRUISE"
        )

        # Publish yaw setpoint (drone_controller 가 yaw control)
        yaw_msg = Vector3()
        yaw_msg.x = 0.0
        yaw_msg.y = 0.0
        yaw_msg.z = float(self.spawn_yaw)
        self.yaw_pub.publish(yaw_msg)

        self.state = STATE_CRUISE

    # ---- CRUISE ----
    def _step_cruise(self):
        """Head 방향 1 m/s 까지 가속. yaw 고정.

        speed_xy = sqrt(vx² + vy²) (ENU)
        target 도달 시 (≥ 1.0 m/s) → HANDOFF
        """
        # ENU velocity (NED → ENU: ENU.vx = NED.vy, ENU.vy = NED.vx)
        enu_vx = self.current_vel[1]
        enu_vy = self.current_vel[0]
        speed_xy = math.sqrt(enu_vx * enu_vx + enu_vy * enu_vy)

        # Cruise command: forward velocity in body frame = (1.0, 0, 0)
        # ENU velocity = R(yaw) · [1, 0]  →  vx_enu = cos(yaw), vy_enu = sin(yaw)
        target_vx_enu = self.cruise_target_speed * math.cos(self.spawn_yaw)
        target_vy_enu = self.cruise_target_speed * math.sin(self.spawn_yaw)

        # NED 변환: NED.vx = ENU.vy, NED.vy = ENU.vx
        target_vx_ned = target_vy_enu
        target_vy_ned = target_vx_enu

        self.send_velocity_cmd(target_vx_ned, target_vy_ned, 0.0, 0.0)

        # Cruise timeout 초기화 (첫 진입 시)
        if self.cruise_start_time is None:
            self.cruise_start_time = time.time()

        if speed_xy >= self.cruise_target_speed * 0.95:
            self.get_logger().info(
                f"Cruise speed reached ({speed_xy:.2f} m/s) → HANDOFF")
            self.state = STATE_HANDOFF
            self.cruise_start_time = None
        elif time.time() - self.cruise_start_time > 10.0:
            # Issue #028 fix: EKF stale 로 가속 못 해도 10s 후 강제 HANDOFF.
            # RL 정책이 control 받음 — bad init state 면 ep 빨리 truncate (학습 신호).
            self.get_logger().warn(
                f"Cruise timeout (10s, speed_xy={speed_xy:.2f}) — forcing HANDOFF")
            self.state = STATE_HANDOFF
            self.cruise_start_time = None

    # ---- HANDOFF ----
    def _step_handoff(self):
        """Single tick: RL 정책 control 인수 신호 publish + TRACK 전환."""
        handoff_msg = Bool()
        handoff_msg.data = True
        self.handoff_pub.publish(handoff_msg)
        self.get_logger().info("Handoff to RL → TRACKING")
        self.state = STATE_TRACK

    # =================================================================
    # Helpers
    # =================================================================

    def send_position_cmd(self, x, y, z):
        msg = Vector3()
        msg.x = float(x)
        msg.y = float(y)
        msg.z = float(z)
        self.pos_pub.publish(msg)

    def send_velocity_cmd(self, vx, vy, vz, yaw_rate):
        msg = Twist()
        msg.linear.x = float(vx)
        msg.linear.y = float(vy)
        msg.linear.z = float(vz)
        msg.angular.z = float(yaw_rate)
        self.vel_pub.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = MissionManagerRADNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
