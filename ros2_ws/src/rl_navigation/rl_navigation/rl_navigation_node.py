#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Point, Twist
from std_msgs.msg import String, Bool
from std_srvs.srv import SetBool   

class RLNavigationNode(Node):
    def __init__(self):
        super().__init__('rl_navigation_node')

        self.mission_state = "IDLE"
        self.target_visible = False
        self.has_dropped = False

        # --- Subscribers ---
        self.state_sub = self.create_subscription(
            String, '/mission/state', self.state_callback, 10)
        
        self.vision_sub = self.create_subscription(
            Point, '/target/pixel_coords', self.vision_callback, 10)

        # --- Publishers ---
        self.vel_pub = self.create_publisher(Twist, '/drone/cmd/velocity', 10)
        
        # --- Service Client ---
        self.vacuum_client = self.create_client(SetBool, '/drone/payload/drop_cmd_raw')
        
        # [주의] 가제보가 켜져있지 않으면 여기서 영원히 대기할 수 있습니다.
        # 테스트 시 반드시 가제보를 먼저 켜주세요.
        while not self.vacuum_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().warn('Waiting for vacuum service...')
        
        self.timer = self.create_timer(0.1, self.control_loop)
        self.get_logger().info("Simple Agent Ready!")

    def state_callback(self, msg):
        self.mission_state = msg.data

    def vision_callback(self, msg):
        if msg.z > 0: 
            self.target_visible = True
        else:
            self.target_visible = False

    def control_loop(self):
        # 1. TRACKING 상태가 아니면 아무것도 안 함 (투하 여부는 여기서 체크하지 않음!)
        if self.mission_state != "TRACKING":
            return

        # ---------------------------------------------------------
        # 2. 기본 동작: 무조건 앞으로 전진 (투하 했든 안 했든 계속 전진)
        # ---------------------------------------------------------
        forward_vel = Twist()
        forward_vel.linear.x = 2.0  
        self.vel_pub.publish(forward_vel)

        # ---------------------------------------------------------
        # 3. 투하 로직 (타겟 보임 + 아직 투하 안 함)
        # ---------------------------------------------------------
        if self.target_visible and not self.has_dropped:
            self.get_logger().warn("RL Agent: Target Spotted! Dropping Payload NOW!")
            
            # 서비스 요청 (Vacuum OFF)
            request = SetBool.Request()
            request.data = False 
            
            # 비동기 호출
            self.vacuum_client.call_async(request)
            
            self.has_dropped = True
        
        # ---------------------------------------------------------
        # 4. 투하 후 상태 로그
        # ---------------------------------------------------------
        if self.has_dropped:
            # 투하 후에도 위에서 forward_vel을 계속 보내고 있으므로 잘 날아갑니다.
            self.get_logger().info("Payload Dropped. Flying away...", throttle_duration_sec=1.0)

def main(args=None):
    rclpy.init(args=args)
    node = RLNavigationNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()