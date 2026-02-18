#!/usr/bin/env python3
import rclpy
from rclpy.node import Node

from geometry_msgs.msg import Point, Twist
from std_msgs.msg import String, Bool

class RLNavigationNode(Node):
    def __init__(self):
        super().__init__('rl_navigation_node')

        self.mission_state = "IDLE"
        self.target_visible = False
        self.has_dropped = False

        # --- Subscribers ---
        # 1. 미션 상태 수신
        self.state_sub = self.create_subscription(
            String, '/mission/state', self.state_callback, 10)
        
        # 2. X 마커 시각 정보 수신
        self.vision_sub = self.create_subscription(
            Point, '/target/pixel_coords', self.vision_callback, 10)

        # --- Publishers ---
        # 1. 속도 제어
        self.vel_pub = self.create_publisher(Twist, '/drone/cmd/velocity', 10)
        
        # 2. 진공 그립퍼 직접 제어 (투하)
        self.vacuum_pub = self.create_publisher(Bool, '/drone/payload/drop_cmd_raw', 10)

        # 10Hz 루프
        self.timer = self.create_timer(0.1, self.control_loop)
        self.get_logger().info("Simple Agent Ready! (Will drop immediately on sight)")

    def state_callback(self, msg):
        self.mission_state = msg.data

    def vision_callback(self, msg):
        if msg.z > 0: # confidence가 0보다 크면 감지된 것
            self.target_visible = True
        else:
            self.target_visible = False

    def control_loop(self):
        # Mission Manager가 제어권을 주지 않았거나 이미 투하했으면 대기
        if self.mission_state != "TRACKING" or self.has_dropped:
            return
        # ---------------------------------------------------------
        # 1. TRACKING 상태 진입 시: 무조건 앞으로 전진 (예: 2.0 m/s)
        # ---------------------------------------------------------
        forward_vel = Twist()
        forward_vel.linear.x = 2.0  # 과장님 시스템에 맞는 전진 속도로 조절하세요
        self.vel_pub.publish(forward_vel)
        # ---------------------------------------------------------
        # 2. 전진 중 타겟이 보이면: 멈추지 않고 즉시 투하!
        # ---------------------------------------------------------
        if self.target_visible:
            self.get_logger().warn("RL Agent: Target Spotted! Dropping Payload NOW while moving!")
            
            # 진공 그립퍼 끄기 (False = Drop)
            msg = Bool()
            msg.data = False

            # 플러그인이 신호를 놓치지 않게 확실하게 퍼블리시
            self.vacuum_pub.publish(msg)
            
            # 투하 완료 처리 (이제 더 이상 투하 신호를 주지 않음)
            self.has_dropped = True
            
            # 주의: 투하 후에도 forward_vel.linear.x = 2.0은 계속 유지되므로 드론은 지나쳐 날아갑니다.

def main(args=None):
    rclpy.init(args=args)
    node = RLNavigationNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()