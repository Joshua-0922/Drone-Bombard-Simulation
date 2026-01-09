import rclpy
from rclpy.node import Node
import math

from geometry_msgs.msg import Twist, Point
from nav_msgs.msg import Odometry

class PathGenerationNode(Node):
    def __init__(self):
        super().__init__('path_generation_node')

        # --- 1. 파라미터 선언 (Default 값 설정) ---
        # 나중에 params.yaml에서 덮어씌워짐
        self.declare_parameters(
            namespace='',
            parameters=[
                ('target_altitude', 5.0),
                ('image_width', 640),
                ('image_height', 480),
                ('kp_yaw', 0.001),
                ('kp_altitude', 0.5),
                ('default_speed_x', 1.0),
                ('yaw_error_threshold', 0.05)
            ]
        )

        # --- 2. 상태 변수 soir 초기화 ---
        self.current_altitude = 0.0
        self.current_yaw = 0.0
        self.target_u = -1.0      # -1이면 타겟 없음
        self.target_v = -1.0
        self.last_target_time = 0.0

        # --- 3. Pub/Sub 설정 ---
        # 드론 제어 명령 발행
        self.cmd_vel_pub = self.create_publisher(Twist, '/drone/cmd_vel', 10) # PX4는 /mavros/setpoint_velocity/cmd_vel_unstamped 등 확인 필요

        # YOLO 좌표 구독
        self.target_sub = self.create_subscription(
            Point,
            '/target/pixel_coords',
            self.target_callback,
            10
        )

        # 드론 상태(오도메트리) 구독
        self.odom_sub = self.create_subscription(
            Odometry,
            '/mavros/local_position/odom',
            self.odom_callback,
            10
        )

        # 0.05초(20Hz)마다 제어 루프 실행
        self.timer = self.create_timer(0.05, self.control_loop)
        
        self.get_logger().info("Path Generation Node Initialized with Param Support")

    def target_callback(self, msg):
        self.target_u = msg.x
        self.target_v = msg.y
        self.last_target_time = self.get_clock().now().nanoseconds

    def odom_callback(self, msg):
        # 고도 업데이트
        self.current_altitude = msg.pose.pose.position.z
        
        # 쿼터니언 -> Yaw 변환
        q = msg.pose.pose.orientation
        # 수식: yaw = atan2(2(wz + xy), 1 - 2(y^2 + z^2))
        siny_cosp = 2 * (q.w * q.z + q.x * q.y)
        cosy_cosp = 1 - 2 * (q.y * q.y + q.z * q.z)
        self.current_yaw = math.atan2(siny_cosp, cosy_cosp)

    def control_loop(self):
        # --- 파라미터 실시간 로드 (학습 시 동적 변경 대응) ---
        target_alt = self.get_parameter('target_altitude').value
        w = self.get_parameter('image_width').value
        kp_yaw = self.get_parameter('kp_yaw').value
        kp_alt = self.get_parameter('kp_altitude').value
        speed_x = self.get_parameter('default_speed_x').value
        
        cmd = Twist()
        img_center_u = w / 2.0

        # 1. 타겟이 보이는지 확인 (최근 0.5초 내에 데이터가 있었나?)
        current_time = self.get_clock().now().nanoseconds
        is_target_visible = (current_time - self.last_target_time) < 0.5 * 1e9 and self.target_u >= 0

        # --- 제어 로직 ---
        
        # A. 고도 제어 (항상 수행 - Height Keep)
        alt_error = target_alt - self.current_altitude
        cmd.linear.z = kp_alt * alt_error

        if is_target_visible:
            # 타겟이 보임 -> 정렬 및 접근
            
            # B. Yaw 제어 (u좌표를 중앙으로)
            # 화면 중심보다 타겟이 오른쪽에 있으면(u > center), 
            # 드론을 오른쪽으로 회전(Yaw -)해야 하는지 왼쪽(+)인지 카메라 장착 방식에 따라 다름.
            # 보통 하방 카메라는 드론이 우회전하면 이미지는 왼쪽으로 이동함.
            pixel_error_u = self.target_u - img_center_u
            
            # P 제어: 에러만큼 회전 속도 부여 (부호는 시뮬레이션 보며 조정 필요)
            # 여기서는 (Target - Center) * Gain 으로 설정
            cmd.angular.z = -1.0 * kp_yaw * pixel_error_u 

            # C. 전진 제어 (정렬이 어느 정도 되면 전진)
            # Yaw 에러가 크면 회전만 하고, 작으면 전진하면서 미세 조정
            if abs(pixel_error_u) < (w * 0.2): # 화면 중심 20% 안에 들어오면
                cmd.linear.x = speed_x
            else:
                cmd.linear.x = speed_x * 0.1 # 정렬 중엔 천천히 전진

            self.get_logger().info(f"Tracking.. ErrU:{pixel_error_u:.1f}, Alt:{self.current_altitude:.1f}")

        else:
            # 타겟이 안 보임 -> 호버링하거나 탐색 모드 (여기서는 정지)
            cmd.linear.x = 0.0
            cmd.angular.z = 0.0
            self.get_logger().debug("Searching for target...")

        # 명령 발행
        self.cmd_vel_pub.publish(cmd)

def main(args=None):
    rclpy.init(args=args)
    node = PathGenerationNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()