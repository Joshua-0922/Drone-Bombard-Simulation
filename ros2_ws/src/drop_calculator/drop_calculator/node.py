import rclpy
from rclpy.node import Node
import math
import numpy as np

# 메시지 타입 임포트
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Point
from std_msgs.msg import Bool

class DropCalculatorNode(Node):
    def __init__(self):
        super().__init__('drop_calculator_node')

        # --- 설정 파라미터 (실제 카메라 스펙에 맞게 수정 필요) ---
        self.declare_parameter('camera_fx', 320.0) # 초점거리 x (pixel)
        self.declare_parameter('camera_fy', 320.0) # 초점거리 y (pixel)
        self.declare_parameter('image_width', 640)
        self.declare_parameter('image_height', 480)
        self.declare_parameter('gravity', 9.81)
        self.declare_parameter('drop_tolerance', 0.2) # 투하 허용 오차 (미터)

        # 파라미터 가져오기
        self.fx = self.get_parameter('camera_fx').value
        self.fy = self.get_parameter('camera_fy').value
        self.cx = self.get_parameter('image_width').value / 2.0
        self.cy = self.get_parameter('image_height').value / 2.0
        self.g = self.get_parameter('gravity').value
        self.tolerance = self.get_parameter('drop_tolerance').value

        # --- 상태 변수 ---
        self.current_height = 0.0
        self.current_velocity_x = 0.0 # 드론 전진 속도 (Body frame 기준)
        self.target_pixel_u = -1 # 타겟이 없으면 -1
        self.target_pixel_v = -1
        self.is_ready_to_drop = False # 중복 투하 방지

        # --- Pub/Sub 설정 ---
        # 1. 드론 상태 구독 (위치, 속도)
        self.odom_sub = self.create_subscription(
            Odometry,
            '/drone/odometry', # PX4의 경우 /mavros/local_position/odom 등 확인 필요
            self.odom_callback,
            10
        )

        # 2. 타겟 픽셀 좌표 구독 (YOLO 노드에서 발행한다고 가정)
        # 메시지 타입은 geometry_msgs/Point 사용 (x=u, y=v, z=0)
        self.pixel_sub = self.create_subscription(
            Point,
            '/target/pixel_coords',
            self.pixel_callback,
            10
        )

        # 3. 투하 명령 발행
        self.drop_pub = self.create_publisher(Bool, '/payload/drop_cmd', 10)

        # 계산 루프 (0.1초마다 계산)
        self.timer = self.create_timer(0.1, self.calculate_drop_logic)
        
        self.get_logger().info("Drop Calculator Node Started!")

    def odom_callback(self, msg):
        """드론의 고도와 속도를 업데이트"""
        # ROS 좌표계 (ENU) 기준: Z가 고도
        self.current_height = msg.pose.pose.position.z
        
        # 드론의 전진 속도 구하기
        # 간단히 하기 위해 Global Frame의 속도 벡터 크기를 구하거나,
        # 드론이 x축 방향으로만 간다면 linear.x를 사용
        vx = msg.twist.twist.linear.x
        vy = msg.twist.twist.linear.y
        # 평면 속도 크기 (속도 방향과 타겟 방향이 일치한다고 가정)
        self.current_velocity_x = math.sqrt(vx**2 + vy**2)

    def pixel_callback(self, msg):
        """YOLO에서 검출된 타겟의 픽셀 좌표 업데이트"""
        self.target_pixel_u = msg.x
        self.target_pixel_v = msg.y

    def calculate_drop_logic(self):
        """핵심 탄도학 계산 및 투하 결정"""
        
        # 1. 데이터 유효성 검사
        if self.target_pixel_u < 0 or self.current_height < 0.5:
            # 타겟이 안 보이거나 고도가 너무 낮으면 패스
            return

        # 2. 낙하 시간 계산 (Free fall time)
        # h = 1/2 * g * t^2  => t = sqrt(2h/g)
        t_fall = math.sqrt((2 * self.current_height) / self.g)

        # 3. 탄도 진행 거리 (Ballistic Advance)
        # 물체가 떨어지는 동안 관성에 의해 앞으로 나아가는 거리
        d_advance = self.current_velocity_x * t_fall

        # 4. 현재 타겟까지의 수평 거리 계산 (Image Plane -> World Distance)
        # 가정: 카메라가 수직 하방(Down)을 보고 있음.
        # 이미지 상의 중심(cy)에서 타겟(v)까지의 거리가 실제 바닥 거리와 비례
        # 드론 진행 방향이 이미지의 세로축(Height)과 정렬되어 있다고 가정
        
        # pixel_dy: 이미지 중심에서 타겟까지의 픽셀 거리 (y축 방향)
        # 만약 드론이 전진(화면 위쪽)하고 있다면, 타겟은 화면 중심보다 위(값 작음) 혹은 아래에 위치
        # 여기서는 단순 거리를 구하기 위해 절대값 사용. 방향은 제어 로직에서 맞춘다고 가정.
        pixel_dy = abs(self.target_pixel_v - self.cy)
        
        # Pinhole Camera Model: Z / f = D / pixel_d
        # D_target = Z * (pixel_d / f)
        d_target = self.current_height * (pixel_dy / self.fy)

        # 5. 투하 결정
        # 남은 거리(d_target)와 날아갈 거리(d_advance)의 차이 계산
        error = d_target - d_advance

        self.get_logger().info(
            f"H:{self.current_height:.2f}m, V:{self.current_velocity_x:.2f}m/s | "
            f"Need:{d_advance:.2f}m, Curr:{d_target:.2f}m, Err:{error:.2f}m"
        )

        # 오차가 허용 범위 내에 들어오고, 아직 투하하지 않았다면
        if abs(error) < self.tolerance and not self.is_ready_to_drop:
            self.trigger_drop()

    def trigger_drop(self):
        msg = Bool()
        msg.data = True
        self.drop_pub.publish(msg)
        self.is_ready_to_drop = True # 한 번만 실행
        self.get_logger().warn("!!! DROP COMMAND SENT !!!")

def main(args=None):
    rclpy.init(args=args)
    node = DropCalculatorNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()