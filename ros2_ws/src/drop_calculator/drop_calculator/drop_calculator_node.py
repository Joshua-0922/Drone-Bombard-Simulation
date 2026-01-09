import rclpy
from rclpy.node import Node
import math
import numpy as np # 벡터 연산을 위해 추가하면 좋으나, 여기선 math로 해결

from nav_msgs.msg import Odometry
from geometry_msgs.msg import Point
from std_msgs.msg import Bool

class DropCalculatorNode(Node):
    def __init__(self):
        super().__init__('drop_calculator_node')

        # --- 파라미터 선언 ---
        self.declare_parameters(
            namespace='',
            parameters=[
                ('gravity', 9.81),
                ('camera_fy', 240.0),   # 카메라 초점거리 y (pixel)
                ('image_height', 480),
                ('drop_tolerance', 0.2), # 허용 오차 (m)
                ('mechanism_delay', 0.1) # 투하 장치 지연 시간 (sec)
            ]
        )

        # --- 상태 변수 ---
        self.current_h = 0.0        # 고도 (m)
        self.velocity = [0.0, 0.0, 0.0] # [vx, vy, vz] (m/s) - 3차원 속도
        
        self.target_v = -1.0        # 타겟의 이미지 상 세로 좌표
        self.last_target_time = 0
        self.has_dropped = False

        # --- Pub/Sub ---
        self.drop_pub = self.create_publisher(Bool, '/payload/drop_cmd', 10)
        
        self.target_sub = self.create_subscription(
            Point, 
            '/target/pixel_coords', 
            self.target_callback, 
            10
        )
        
        self.odom_sub = self.create_subscription(
            Odometry,
            '/drone/odometry',
            self.odom_callback,
            10
        )

        # 고속 기동을 고려하여 연산 주기를 50Hz(0.02s)로 설정
        self.timer = self.create_timer(0.02, self.calculate_ballistics)
        
        self.get_logger().info("Advanced 3D Drop Calculator Started")

    def odom_callback(self, msg):
        # 1. 고도 업데이트 (World Frame Z)
        self.current_h = msg.pose.pose.position.z
        
        # 2. 3차원 속도 업데이트 (vx, vy, vz)
        # 주의: PX4/Mavros의 Twist는 보통 Body Frame일 수 있으나, 
        # 여기서는 Global Frame(ENU) 기준 속도라고 가정하고 계산합니다.
        # (만약 Body Frame이라면 Rotation Matrix를 곱해서 변환해야 함)
        q = msg.pose.pose.orientation
        
        # 선속도 가져오기
        vx_local = msg.twist.twist.linear.x
        vy_local = msg.twist.twist.linear.y
        vz_local = msg.twist.twist.linear.z # 상승(+)/하강(-) 속도
        
        # 간단한 구현을 위해 드론이 거의 수평 비행 중이라고 가정하거나,
        # Odometry가 Global Frame 속도를 준다고 가정 (nav_msgs/Odometry의 child_frame_id 확인 필요)
        # 여기서는 편의상 입력된 속도를 바로 사용합니다.
        self.velocity = [vx_local, vy_local, vz_local]

    def target_callback(self, msg):
        self.target_v = msg.y
        self.last_target_time = self.get_clock().now().nanoseconds

    def calculate_ballistics(self):
        if self.has_dropped: return

        # 타겟 데이터 유효성 검사 (0.5초 timeout)
        now = self.get_clock().now().nanoseconds
        if (now - self.last_target_time) > 0.5 * 1e9: return

        # --- 파라미터 로드 ---
        g = self.get_parameter('gravity').value
        fy = self.get_parameter('camera_fy').value
        h_res = self.get_parameter('image_height').value
        cy = h_res / 2.0
        tol = self.get_parameter('drop_tolerance').value
        delay = self.get_parameter('mechanism_delay').value

        # 변수 가독성 확보
        H = self.current_h
        Vx, Vy, Vz = self.velocity # Vz가 핵심

        if H < 0.5: return # 너무 낮으면 계산 스킵

        # ---------------------------------------------------------
        # [Step 1] 3차원 낙하 시간 계산 (Quadratic Formula)
        # 방정식: H + Vz*t - 0.5*g*t^2 = 0
        # 근의 공식: t = (Vz + sqrt(Vz^2 + 2*g*H)) / g
        # ---------------------------------------------------------
        discriminant = Vz**2 + 2 * g * H
        if discriminant < 0:
            return # 수학적 오류 (땅속에 있는 경우 등)
            
        t_impact = (Vz + math.sqrt(discriminant)) / g
        
        # ---------------------------------------------------------
        # [Step 2] 물체 도달 거리 예측 (Prediction)
        # 메커니즘 딜레이(delay) 동안은 드론과 같이 움직이고, 
        # 그 후 t_impact 동안은 관성으로 날아감.
        # ---------------------------------------------------------
        
        # 수평 속력 (Horizontal Speed)
        v_horizontal = math.sqrt(Vx**2 + Vy**2)
        
        # 총 비행 시간 (딜레이 포함) 동안 이동할 수평 거리
        # (공기 저항 무시 가정)
        flight_time_total = delay + t_impact
        dist_advance = v_horizontal * flight_time_total

        # ---------------------------------------------------------
        # [Step 3] 현재 타겟까지의 실제 수평 거리 (Ground Truth)
        # 높이(H)가 계속 변하므로 매 순간 H를 대입해서 거리를 구함
        # ---------------------------------------------------------
        pixel_diff = cy - self.target_v # 이미지 중심과 타겟의 거리
        
        # 타겟이 드론 뒤로 넘어갔으면(화면 아래쪽) 패스
        if pixel_diff <= 0: return 

        # Ray Casting (Pinhole Model)
        # H가 변하더라도 현재 H에 비례하여 거리가 계산됨
        dist_target = (H * pixel_diff) / fy

        # ---------------------------------------------------------
        # [Step 4] 판별
        # ---------------------------------------------------------
        error = abs(dist_target - dist_advance)
        
        # 디버깅 로그 (필요시 주석 해제)
        self.get_logger().info(
             f"H:{H:.1f}m, Vz:{Vz:.1f}m/s | T_fall:{t_impact:.2f}s | "
             f"Adv:{dist_advance:.2f}m vs Tgt:{dist_target:.2f}m | Err:{error:.2f}"
        )

        if error < tol:
            self.trigger_drop()

    def trigger_drop(self):
        msg = Bool()
        msg.data = True
        self.drop_pub.publish(msg)
        self.has_dropped = True
        self.get_logger().warn(f"!!! DROP EXECITED !!! (Alt: {self.current_h:.2f}m)")

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