#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Point, Twist
from std_msgs.msg import String
import os # 터미널 명령어를 파이썬에서 실행하기 위해 추가

# 🚨 주의: linkattacher_msgs 임포트 문구는 삭제했습니다!

class RLNavigationNode(Node):
    def __init__(self):
        super().__init__('rl_navigation_node')

        self.mission_state = "IDLE"
        self.target_visible = False
        self.has_dropped = False

        # --- Subscribers & Publishers ---
        self.state_sub = self.create_subscription(String, '/mission/state', self.state_callback, 10)
        self.vision_sub = self.create_subscription(Point, '/target/pixel_coords', self.vision_callback, 10)
        self.vel_pub = self.create_publisher(Twist, '/drone/cmd/velocity', 10)
        
        self.get_logger().info("🔌 Initializing Link Attacher: Welding Payload to Drone via OS Command...")
        
        # [해결책] 노드 시작과 동시에 터미널 명령어로 Attach 강제 실행!
        attach_cmd = "ros2 service call /ATTACH_LINK linkattacher_msgs/srv/AttachLink \"{model1_name: 'iris', link1_name: 'base_link', model2_name: 'payload_cylinder', link2_name: 'payload_link'}\" &"
        os.system(attach_cmd)

        self.timer = self.create_timer(0.1, self.control_loop)
        self.get_logger().info("Simple Agent Ready! (Non-blocking mode)")

    def state_callback(self, msg):
        self.mission_state = msg.data

    def vision_callback(self, msg):
        if msg.z > 0: 
            self.target_visible = True
        else:
            self.target_visible = False

    def control_loop(self):
        if self.mission_state != "TRACKING":
            return

        # 무조건 전진
        forward_vel = Twist()
        forward_vel.linear.x = 2.0  
        self.vel_pub.publish(forward_vel)

        # 투하 로직 (Detach)
        if self.target_visible and not self.has_dropped:
            self.get_logger().warn("🎯 Target Found! Sending DETACH (DROP) command via OS Command!")
            
            # [해결책] 타겟 발견 시 터미널 명령어로 Detach 강제 실행!
            detach_cmd = "ros2 service call /DETACH_LINK linkattacher_msgs/srv/DetachLink \"{model1_name: 'iris', link1_name: 'base_link', model2_name: 'payload_cylinder', link2_name: 'payload_link'}\" &"
            os.system(detach_cmd)
            
            self.has_dropped = True

        if self.has_dropped:
            self.get_logger().info("Bomb Dropped! Flying away...", throttle_duration_sec=1.0)

def main(args=None):
    rclpy.init(args=args)
    node = RLNavigationNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()