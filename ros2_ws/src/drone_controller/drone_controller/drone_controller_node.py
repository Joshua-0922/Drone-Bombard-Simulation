import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy

# ROS 2 메시지
from geometry_msgs.msg import Twist, Vector3
# PX4 메시지
from px4_msgs.msg import OffboardControlMode, TrajectorySetpoint, VehicleCommand, VehicleStatus

class DroneControllerNode(Node):
    def __init__(self):
        super().__init__('drone_controller_node')

        # --- [1] QoS 설정 ---
        # Publishers → PX4: BEST_EFFORT is sufficient; TRANSIENT_LOCAL is compatible
        pub_qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            history=HistoryPolicy.KEEP_LAST,
            depth=1
        )
        # Subscribers ← PX4: PX4 publishes VOLATILE; using TRANSIENT_LOCAL here
        # causes a silent DDS incompatibility and no messages are ever received.
        sub_qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE,
            history=HistoryPolicy.KEEP_LAST,
            depth=1
        )

        # --- [2] Publishers (To PX4) ---
        self.offboard_control_mode_pub = self.create_publisher(
            OffboardControlMode, '/fmu/in/offboard_control_mode', pub_qos)
        self.trajectory_setpoint_pub = self.create_publisher(
            TrajectorySetpoint, '/fmu/in/trajectory_setpoint', pub_qos)
        self.vehicle_command_pub = self.create_publisher(
            VehicleCommand, '/fmu/in/vehicle_command', pub_qos)

        # --- [3] Subscribers (From Brains) ---
        # A. 위치 제어 (순항 모드)
        self.pos_sub = self.create_subscription(
            Vector3, '/drone/cmd/position', self.position_command_callback, 10)

        # B. 속도 제어 (추적 모드)
        self.vel_sub = self.create_subscription(
            Twist, '/drone/cmd/velocity', self.velocity_command_callback, 10)

        self.status_sub = self.create_subscription(
            VehicleStatus, '/fmu/out/vehicle_status',
            self.vehicle_status_callback, sub_qos)

        # --- [4] 내부 변수 ---
        self.control_mode = "POSITION"

        # 초기 고도 설정: 10m (NED 좌표계이므로 -10.0)
        self.target_pos = [0.0, 0.0, -10.0]
        self.target_vel = [0.0, 0.0, 0.0, 0.0]  # vx, vy, vz, yaw_speed

        self.vehicle_status = VehicleStatus()
        self.offboard_set_counter = 0
        self.px4_connected = False  # True after first vehicle_status received

        # --- [5] Main Loop (20Hz) ---
        self.timer = self.create_timer(0.05, self.cmdloop_callback)
        self.get_logger().info("Drone Controller Node Started! Hovering at 10m.")

    # ... (Callback 함수들은 기존과 동일, 생략 가능하지만 복붙 편의를 위해 유지) ...

    def position_command_callback(self, msg):
        self.control_mode = "POSITION"
        # ROS(ENU) -> PX4(NED): x->N, y->-E, z->-D
        self.target_pos = [msg.x, -msg.y, -msg.z] 

    def velocity_command_callback(self, msg):
        self.control_mode = "VELOCITY"
        # ROS(ENU) -> PX4(NED): vx->vx, vy->-vy, vz->-vz, yaw->-yaw
        self.target_vel = [msg.linear.x, -msg.linear.y, -msg.linear.z, -msg.angular.z]

    def vehicle_status_callback(self, msg):
        if not self.px4_connected:
            self.get_logger().info("PX4 connected — vehicle_status received.")
        self.px4_connected = True
        self.vehicle_status = msg

    def cmdloop_callback(self):
        self.publish_offboard_control_mode()

        if self.control_mode == "POSITION":
            self.publish_position_setpoint(self.target_pos)
        elif self.control_mode == "VELOCITY":
            self.publish_velocity_setpoint(self.target_vel)

        self.offboard_set_counter += 1

        # Wait until PX4 vehicle_status is actually received before arming.
        # Sending arm/offboard commands before PX4 is ready silently fails.
        if not self.px4_connected:
            return

        already_armed = (self.vehicle_status.arming_state == 2)   # ARMED
        in_offboard = (self.vehicle_status.nav_state == 14)        # OFFBOARD

        if already_armed and in_offboard:
            return  # Nothing to do

        # Retry arm every 2 s (40 ticks at 20 Hz) until confirmed.
        # Retry OFFBOARD mode every 0.5 s (10 ticks) — more aggressively than arm
        # because DDS time-sync disruptions can cause brief OFFBOARD mode loss, and
        # waiting 2 s between retries creates a race condition with the ~2 s sync cycle.
        if self.offboard_set_counter % 40 == 0:
            if not already_armed:
                self.arm()
        if self.offboard_set_counter % 10 == 0:
            if not in_offboard:
                self.engage_offboard_mode()

    def publish_offboard_control_mode(self):
        msg = OffboardControlMode()
        msg.timestamp = int(self.get_clock().now().nanoseconds / 1000)
        msg.position = True
        msg.velocity = True
        msg.acceleration = False
        self.offboard_control_mode_pub.publish(msg)

    def publish_position_setpoint(self, pos_ned):
        msg = TrajectorySetpoint()
        msg.timestamp = int(self.get_clock().now().nanoseconds / 1000)
        msg.position = [float(pos_ned[0]), float(pos_ned[1]), float(pos_ned[2])]
        msg.velocity = [float('nan'), float('nan'), float('nan')]
        msg.yaw = float('nan')
        self.trajectory_setpoint_pub.publish(msg)

    def publish_velocity_setpoint(self, vel_ned):
        msg = TrajectorySetpoint()
        msg.timestamp = int(self.get_clock().now().nanoseconds / 1000)
        msg.position = [float('nan'), float('nan'), float('nan')]
        msg.velocity = [float(vel_ned[0]), float(vel_ned[1]), float(vel_ned[2])]
        msg.yawspeed = float(vel_ned[3])
        self.trajectory_setpoint_pub.publish(msg)

    def engage_offboard_mode(self):
        self.publish_vehicle_command(VehicleCommand.VEHICLE_CMD_DO_SET_MODE, param1=1.0, param2=6.0)
        self.get_logger().info("--> Switching to OFFBOARD Mode")

    def arm(self):
        self.publish_vehicle_command(VehicleCommand.VEHICLE_CMD_COMPONENT_ARM_DISARM, param1=1.0)
        self.get_logger().info("--> Arming Drone")

    def publish_vehicle_command(self, command, **params):
        msg = VehicleCommand()
        msg.timestamp = int(self.get_clock().now().nanoseconds / 1000)
        msg.command = command
        msg.param1 = params.get("param1", 0.0)
        msg.param2 = params.get("param2", 0.0)
        msg.target_system = 1
        msg.target_component = 1
        msg.source_system = 1
        msg.source_component = 1
        msg.from_external = True
        self.vehicle_command_pub.publish(msg)

def main(args=None):
    rclpy.init(args=args)
    node = DroneControllerNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()