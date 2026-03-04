#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Point, Twist
from std_msgs.msg import String, Bool, Empty

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
        
        self.detach_pub = self.create_publisher(Empty, '/payload/drop_cmd', 10)

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

            # Triggers DetachableJoint via ros_gz_bridge → gz /x500_bombard/drop
            self.detach_pub.publish(Empty())

            # Signals drop_calculator and mission_manager via Bool(False)
            drop_msg = Bool()
            drop_msg.data = False
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