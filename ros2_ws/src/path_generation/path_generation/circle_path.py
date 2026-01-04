import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy

# PX4 통신을 위한 메시지 타입들 (빌드된 px4_msgs가 있어야 함)
from px4_msgs.msg import OffboardControlMode, TrajectorySetpoint, VehicleCommand, VehicleStatus

import math
import numpy as np

    def __init__(self):
        super().__init__('circle_flight_node')

        # QoS 설정 (PX4와 통신할 때 필수 - Best Effort 방식)
        qos_profile = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            history=HistoryPolicy.KEEP_LAST,
            depth=1
        )

        # 1. 퍼블리셔 (명령 보내기)
        self.offboard_control_mode_publisher = self.create_publisher(
            OffboardControlMode, '/fmu/in/offboard_control_mode', qos_profile)
        self.trajectory_setpoint_publisher = self.create_publisher(
            TrajectorySetpoint, '/fmu/in/trajectory_setpoint', qos_profile)
        self.vehicle_command_publisher = self.create_publisher(
            VehicleCommand, '/fmu/in/vehicle_command', qos_profile)

        # 2. 서브스크라이버 (상태 듣기)
        self.vehicle_status_subscriber = self.create_subscription(
            VehicleStatus, '/fmu/out/vehicle_status', self.vehicle_status_callback, qos_profile)

        # 변수 초기화
        self.vehicle_status = VehicleStatus()
        self.theta = 0.0          # 회전 각도
        self.radius = 5.0         # 반지름 5m
        self.altitude = -10.0     # 고도 10m (NED 좌표계이므로 -)
        self.omega = 0.5          # 각속도 (rad/s) - 속도 조절
        self.takeoff_height = -10.0
        self.counter = 0

        # 3. 타이머 (0.1초마다 실행 -> 10Hz)
        self.timer = self.create_timer(0.1, self.timer_callback)

    def vehicle_status_callback(self, msg):
        """드론의 현재 상태(시동 여부, 비행 모드 등)를 업데이트"""
        self.vehicle_status = msg

    def publish_offboard_control_mode(self):
        """Offboard 모드 유지를 위한 심박 신호(Heartbeat)"""
        msg = OffboardControlMode()
        msg.timestamp = int(self.get_clock().now().nanoseconds / 1000)
        msg.position = True      # 위치 제어 사용
        msg.velocity = False
        msg.acceleration = False
        self.offboard_control_mode_publisher.publish(msg)

    def publish_vehicle_command(self, command, param1=0.0, param2=0.0):
        """드론에게 시스템 명령(시동, 모드 변경 등) 전송"""
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
        """시동 걸기"""
        self.publish_vehicle_command(VehicleCommand.VEHICLE_CMD_COMPONENT_ARM_DISARM, 1.0)
        self.get_logger().info("Arm command sent")

    def engage_offboard_mode(self):
        """Offboard 모드로 전환"""
        self.publish_vehicle_command(
            VehicleCommand.VEHICLE_CMD_DO_SET_MODE, 1.0, 6.0)
        self.get_logger().info("Switching to Offboard mode")

    def timer_callback(self):
        # 1. Offboard 모드 유지를 위해 계속 신호를 보내야 함 (필수)
        self.publish_offboard_control_mode()

        # 2. 시동 및 모드 전환 로직 (초반 10번 정도 신호 보낸 후 시도)
        if self.counter == 10:
            self.engage_offboard_mode()
            self.arm()
        
        # 3. 원운동 좌표 계산
        # x = r * cos(theta)
        # y = r * sin(theta)
        x = self.radius * math.cos(self.theta)
        y = self.radius * math.sin(self.theta)
        
        # 4. Trajectory Setpoint 메시지 생성
        traj_msg = TrajectorySetpoint()
        traj_msg.timestamp = int(self.get_clock().now().nanoseconds / 1000)
        
        # NED 좌표계: [North, East, Down]
        # (0, 0)을 중심으로 반지름 5m 원을 그림
        traj_msg.position = [x, y, self.altitude] 
        traj_msg.yaw = Float.NaN # (혹은 진행 방향을 보게 하려면 theta + 90도)

        # 5. 메시지 발행
        self.trajectory_setpoint_publisher.publish(traj_msg)

        # 6. 각도 업데이트 (다음 스텝을 위해)
        if self.counter > 100: # 충분히 이륙한 후 회전 시작 (약 10초 뒤)
             self.theta += self.omega * 0.1  # theta = omega * dt

        self.counter += 1

def main(args=None):
    rclpy.init(args=args)
    node = CircleFlightNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()