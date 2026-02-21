#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Point, Twist
from std_msgs.msg import String, Bool
import os

class RLNavigationNode(Node):
    def __init__(self):
        super().__init__('rl_navigation_node')

        self.mission_state = "IDLE"
        self.target_visible = False
        self.has_dropped = False

        # --- Subscribers ---
        self.state_sub = self.create_subscription(String, '/mission/state', self.state_callback, 10)
        self.vision_sub = self.create_subscription(Point, '/target/pixel_coords', self.vision_callback, 10)
        
        # --- Publishers ---
        self.vel_pub = self.create_publisher(Twist, '/drone/cmd/velocity', 10)
        # Drop Calculator 노드가 이 토픽이 False가 되는 순간을 감지하여 점수를 계산합니다.
        self.drop_pub = self.create_publisher(Bool, '/drone/payload/drop_cmd_raw', 10)
        
        self.get_logger().info("🔌 Initializing Link Attacher: Welding Payload...")
        
        # [초기 설정] 시작하자마자 부착 (Type은 확인하신 대로 Attach 사용)
        # 만약 README처럼 경로가 필요하다면 /detach 대신 /link_attacher_node/detach로 수정하세요.
        attach_cmd = (
            "ros2 service call /attach gazebo_ros_link_attacher/srv/Attach "
            "\"{model_name_1: 'iris', link_name_1: 'base_link', "
            "model_name_2: 'payload_cylinder', link_name_2: 'payload_link'}\" &"
        )
        os.system(attach_cmd)
        
        # 초기 상태는 '잡고 있음'(True) 전송
        init_msg = Bool()
        init_msg.data = True
        self.drop_pub.publish(init_msg)

        self.timer = self.create_timer(0.1, self.control_loop)
        self.get_logger().info("RL Navigation Node Ready!")

    def state_callback(self, msg):
        self.mission_state = msg.data

    def vision_callback(self, msg):
        # z값(Confidence)이 0보다 크면 타겟 발견으로 간주
        self.target_visible = msg.z > 0

    def control_loop(self):
        # TRACKING 상태가 아니면 명령을 내리지 않음
        if self.mission_state != "TRACKING":
            return

        # 1. 무조건 전진 명령 (초속 2m)
        forward_vel = Twist()
        forward_vel.linear.x = 2.0  
        self.vel_pub.publish(forward_vel)

        # 2. 투하 로직 (타겟이 보이고 아직 안 떨어뜨렸을 때)
        if self.target_visible and not self.has_dropped:
            self.get_logger().warn("🎯 TARGET DETECTED! Dropping Payload...")

            # [A] 서비스 호출로 물리적 분리 (README 형식을 ROS2 환경에 맞춰 반영)
            # 타입 이름이 srv/Attach인 것에 주의하세요!
            detach_cmd = (
                "ros2 service call /detach gazebo_ros_link_attacher/srv/Attach "
                "\"{model_name_1: 'iris', link_name_1: 'base_link', "
                "model_name_2: 'payload_cylinder', link_name_2: 'payload_link'}\" &"
            )
            os.system(detach_cmd)

            # [B] 토픽 발행으로 계산기(DropCalculator)에게 알림
            drop_msg = Bool()
            drop_msg.data = False # False가 되는 순간이 투하 시점
            self.drop_pub.publish(drop_msg)

            self.has_dropped = True
            self.get_logger().info("✅ Detach command sent and Topic published.")

def main(args=None):
    rclpy.init(args=args)
    node = RLNavigationNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()