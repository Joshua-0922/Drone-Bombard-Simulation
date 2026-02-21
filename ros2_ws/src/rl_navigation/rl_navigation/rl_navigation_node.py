#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Point, Twist
from std_msgs.msg import String

# [수정됨] Link Attacher 플러그인의 커스텀 서비스 메시지 임포트
from linkattacher_msgs.srv import AttachLink, DetachLink

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
        
        # --- Service Clients ---
        # [수정됨] Vacuum 클라이언트 대신 Attach/Detach 클라이언트 생성
        self.attach_client = self.create_client(AttachLink, '/ATTACH_LINK')
        self.detach_client = self.create_client(DetachLink, '/DETACH_LINK')
        
        # 서비스 대기
        while not self.attach_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().warn('Waiting for Link Attacher service to start...')
            
        self.get_logger().info("🔌 Initializing Link Attacher: Welding Payload to Drone...")
        
        # [수정됨] 노드 시작과 동시에 드론과 원통을 물리적으로 강제 결합 (Attach)
        attach_req = AttachLink.Request()
        attach_req.model1_name = 'iris'
        attach_req.link1_name = 'base_link'
        attach_req.model2_name = 'payload_cylinder'  # world 파일에 소환한 페이로드 이름
        attach_req.link2_name = 'payload_link'
        self.attach_client.call_async(attach_req)

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
            if self.detach_client.service_is_ready():
                self.get_logger().warn("🎯 Target Found! Sending DETACH (DROP) command!")
                
                # [수정됨] 투하 시 조인트 해제 요청
                detach_req = DetachLink.Request()
                detach_req.model1_name = 'iris'
                detach_req.link1_name = 'base_link'
                detach_req.model2_name = 'payload_cylinder'
                detach_req.link2_name = 'payload_link'
                
                self.detach_client.call_async(detach_req)
                self.has_dropped = True
            else:
                self.get_logger().error("⚠️ Cannot drop! Detach service not ready.")

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