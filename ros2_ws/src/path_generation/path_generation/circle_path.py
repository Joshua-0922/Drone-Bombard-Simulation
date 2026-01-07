import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy
from px4_msgs.msg import OffboardControlMode, TrajectorySetpoint, VehicleCommand, VehicleStatus
import math

class CircleFlightNode(Node):
    def __init__(self):
        super().__init__('circle_flight_node')

        # --- 1. 통신 품질 설정 (QoS) ---
        qos_profile = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            history=HistoryPolicy.KEEP_LAST,
            depth=1
        )

        # --- 2. 퍼블리셔 (명령 보내기) ---
        self.offboard_control_mode_publisher = self.create_publisher(
            OffboardControlMode, '/fmu/in/offboard_control_mode', qos_profile)
        self.trajectory_setpoint_publisher = self.create_publisher(
            TrajectorySetpoint, '/fmu/in/trajectory_setpoint', qos_profile)
        self.vehicle_command_publisher = self.create_publisher(
            VehicleCommand, '/fmu/in/vehicle_command', qos_profile)

        # --- 3. 서브스크라이버 (상태 듣기) ---
        self.vehicle_status_subscriber = self.create_subscription(
            VehicleStatus, '/fmu/out/vehicle_status', self.vehicle_status_callback, qos_profile)

        # --- 4. 변수 초기화 ---
        self.vehicle_status = VehicleStatus()
        self.theta = 0.0          # 현재 각도
        self.radius = 5.0         # 반지름 5m
        self.altitude = -10.0     # 고도 10m (NED 좌표계: 위쪽이 -)
        self.omega = 0.5          # 회전 속도 (rad/s)
        self.counter = 0          # 시간 카운터

        # --- 5. 타이머 실행 (0.1초마다 반복 -> 10Hz) ---
        self.timer = self.create_timer(0.1, self.timer_callback)

    def vehicle_status_callback(self, msg):
        """드론 상태 업데이트"""
        self.vehicle_status = msg

    def publish_offboard_control_mode(self):
        """Offboard 모드 유지를 위한 심박 신호"""
        msg = OffboardControlMode()
        msg.timestamp = int(self.get_clock().now().nanoseconds / 1000)
        msg.position = True      # 위치 제어 모드 사용
        msg.velocity = False
        msg.acceleration = False
        self.offboard_control_mode_publisher.publish(msg)

    def publish_vehicle_command(self, command, param1=0.0, param2=0.0):
        """드론에게 시스템 명령(시동, 모드변경) 전송"""
        msg = VehicleCommand()
        msg.param1 = param1
        msg.param2 = param2
        msg.command = command
        msg.target_system = 1
        msg.target_component = 1
        msg.source_system = 1
        msg.source_component = 1
        msg.from_external = True
        msg.timestamp = int(self.get_clock().now().nanoseconds / 1000)
        self.vehicle_command_publisher.publish(msg)

    def arm(self):
        """시동 걸기 (Arming)"""
        self.publish_vehicle_command(VehicleCommand.VEHICLE_CMD_COMPONENT_ARM_DISARM, 1.0)
        self.get_logger().info("Arming command sent")

    def engage_offboard_mode(self):
        """Offboard 모드 진입"""
        self.publish_vehicle_command(
            VehicleCommand.VEHICLE_CMD_DO_SET_MODE, 1.0, 6.0)
        self.get_logger().info("Switching to Offboard mode")

    def timer_callback(self):
        """0.1초마다 실행되는 메인 루프"""
        # 1. Offboard 모드 신호는 무조건 계속 보내야 함 (안 보내면 끊김)
        self.publish_offboard_control_mode()

        # 2. 시작 후 1초(10번) 뒤에 -> Offboard 모드 변경 + 시동 걸기
        if self.counter == 10:
            self.engage_offboard_mode()
            self.arm()

        # 3. 원운동 좌표 계산 (x = r*cos, y = r*sin)
        x = self.radius * math.cos(self.theta)
        y = self.radius * math.sin(self.theta)

        # 4. 목표 위치 메시지 생성
        traj_msg = TrajectorySetpoint()
        traj_msg.timestamp = int(self.get_clock().now().nanoseconds / 1000)
        # NED 좌표계: [North, East, Down]
        traj_msg.position = [x, y, self.altitude] 
        traj_msg.yaw = float('nan') # Yaw는 자동 제어

        # 5. 메시지 발행 (드론아 이리로 가라!)
        self.trajectory_setpoint_publisher.publish(traj_msg)

        # 6. 다음 스텝을 위해 각도 증가 (이륙 후 충분한 시간 뒤부터 회전)
        if self.counter > 100: # 약 10초 뒤부터 회전 시작
            self.theta += self.omega * 0.1

        self.counter += 1

def main(args=None):
    rclpy.init(args=args)
    node = CircleFlightNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
