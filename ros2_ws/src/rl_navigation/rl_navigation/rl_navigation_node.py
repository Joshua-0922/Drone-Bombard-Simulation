#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Point, Twist
from std_msgs.msg import String
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
        
        while not self.vacuum_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().warn('Waiting for vacuum service to start...')
        # [수정] 여기서 wait_for_service로 무한 대기하지 않습니다! 
        # 서비스가 없어도 일단 노드는 켜져야 드론이 날아갑니다.
        self.get_logger().info("🔌 Initializing Gripper: Holding Payload...")
        hold_req = SetBool.Request()
        hold_req.data = True  # True = 켜기 (잡기)
        self.vacuum_client.call_async(hold_req)

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
        # 1. TRACKING 상태가 아니면 대기
        if self.mission_state != "TRACKING":
            return

        # ---------------------------------------------------------
        # [중요] 서비스 연결 여부와 상관없이 무조건 전진 명령!
        # ---------------------------------------------------------
        forward_vel = Twist()
        forward_vel.linear.x = 2.0  
        self.vel_pub.publish(forward_vel)

        # ---------------------------------------------------------
        # 2. 투하 로직 (서비스가 살아있을 때만 시도)
        # ---------------------------------------------------------
        if self.target_visible and not self.has_dropped:
            
            # 서비스가 준비되었는지 0.1초만 살짝 체크
            if self.vacuum_client.service_is_ready():
                self.get_logger().warn("🎯 Target Found! Sending DROP command!")
                
                request = SetBool.Request()
                request.data = False 
                self.vacuum_client.call_async(request)
                self.has_dropped = True
            else:
                # 서비스가 안 보여도 멈추지 말고 로그만 띄우고 지나감
                self.get_logger().error("⚠️ Cannot drop! Service not ready. Flying pass...")
                # (옵션) 여기서 has_dropped = True를 안 하면 계속 시도함

        if self.has_dropped:
            self.get_logger().info("Mission Complete. Flying away...", throttle_duration_sec=1.0)

def main(args=None):
    rclpy.init(args=args)
    node = RLNavigationNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()