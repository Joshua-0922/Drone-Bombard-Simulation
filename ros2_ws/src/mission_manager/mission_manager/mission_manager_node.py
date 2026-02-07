#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy

from geometry_msgs.msg import Point, Vector3, Twist
from std_msgs.msg import String, Bool
from px4_msgs.msg import VehicleLocalPosition

import math
import time

# --- 상태 상수 정의 ---
STATE_TAKEOFF = "TAKEOFF"       # 이륙 단계
STATE_CRUISE  = "CRUISE"        # 순항 (직진 탐색)
STATE_TRACK   = "TRACKING"      # 요격 (추적 및 접근)
STATE_DROP    = "DROP"          # 투하 완료
STATE_RETURN  = "RETURN"        # 복귀 (옵션)

class MissionManagerNode(Node):
    def __init__(self):
        super().__init__('mission_manager_node')

        # --- [1] 파라미터 ---
        self.target_altitude = 10.0   # 순항 고도 10m
        
        # (1, 1) 방향으로 움직이게 설정
        self.cruise_speed_x = 1.0     # 북쪽(X) 속도 (m/s)
        self.cruise_speed_y = -1.0     # 동쪽(Y) 속도 (m/s)
        
        # --- [2] Publishers ---
        self.pos_pub = self.create_publisher(Vector3, '/drone/cmd/position', 10)
        self.vel_pub = self.create_publisher(Twist, '/drone/cmd/velocity', 10)
        self.state_pub = self.create_publisher(String, '/mission/state', 10)

        # --- [3] Subscribers ---
        self.vision_sub = self.create_subscription(
            Point, '/target/pixel_coords', self.vision_callback, 10)
        
        self.drop_sub = self.create_subscription(
            Bool, '/payload/drop_cmd', self.drop_callback, 10)

        qos_profile = QoSProfile(reliability=ReliabilityPolicy.BEST_EFFORT, depth=1)
        self.local_pos_sub = self.create_subscription(
            VehicleLocalPosition, '/fmu/out/vehicle_local_position', self.local_pos_callback, qos_profile)

        # --- [4] 내부 변수 ---
        self.state = STATE_TAKEOFF
        self.current_pos = [0.0, 0.0, 0.0] # [x, y, z] (NED)
        
        # 순항 목표 지점 (초기화는 TAKEOFF 완료 시점에 함)
        self.cruise_target_x = 0.0
        self.cruise_target_y = 0.0
        
        # 타겟 정보
        self.target_visible = False
        self.target_u = -1.0
        self.target_v = -1.0
        self.last_detection_time = 0

        # --- [5] Main Loop (10Hz) ---
        self.timer = self.create_timer(0.1, self.control_loop)
        self.get_logger().info("Mission Manager Started! State: TAKEOFF")

    # =================================================================
    # Callbacks
    # =================================================================
    
    def local_pos_callback(self, msg):
        self.current_pos = [msg.x, msg.y, msg.z]

    def vision_callback(self, msg):
        # 타겟 발견 (msg.z가 0이면 감지 안 된 것, 0보다 크면 Confidence)
        # vision 노드에서 z=0.0으로 보내면 감지 안 된 것으로 처리
        if msg.z > 0.0: 
            self.target_visible = True
            self.target_u = msg.x
            self.target_v = msg.y
            self.last_detection_time = time.time()

    def drop_callback(self, msg):
        if msg.data:
            self.get_logger().warn(">>> DROP CONFIRMED! <<<")
            self.state = STATE_DROP

    # =================================================================
    # Main State Machine Loop
    # =================================================================

    def control_loop(self):
        # 상태 메시지 발행
        state_msg = String()
        state_msg.data = self.state
        self.state_pub.publish(state_msg)

        # 타겟 소실 처리 (1초 타임아웃)
        if time.time() - self.last_detection_time > 1.0:
            self.target_visible = False

        # --- FSM Logic ---
        
        if self.state == STATE_TAKEOFF:
            self.send_position_cmd(0.0, 0.0, self.target_altitude)

            # 고도 도달 체크 (NED 좌표계 주의: 위쪽이 -z)
            # 목표 -10m, 현재 -9.5m 이하이면 도달로 판단
            if self.current_pos[2] <= -(self.target_altitude * 0.95):
                self.get_logger().info("Altitude Reached! Switching to CRUISE mode.")
                self.state = STATE_CRUISE
                
                # [중요] 순항 시작점을 현재 위치로 초기화해야 튐 현상 방지
                self.cruise_target_x = self.current_pos[0]
                self.cruise_target_y = self.current_pos[1]

        elif self.state == STATE_CRUISE:
            # 타겟 발견 시
            if self.target_visible:
                self.get_logger().warn("TARGET DETECTED! Engaging Intercept Mode.")
                self.state = STATE_TRACK
                return

            # (1, 1) 방향으로 목표점 갱신
            self.cruise_target_x += (self.cruise_speed_x * 0.1)
            self.cruise_target_y += (self.cruise_speed_y * 0.1)
            
            self.send_position_cmd(self.cruise_target_x, self.cruise_target_y, self.target_altitude)

        elif self.state == STATE_TRACK:
            # 타겟 놓치면 다시 순항
            if not self.target_visible:
                self.get_logger().info("Target Lost... Resuming Search.")
                self.state = STATE_CRUISE
                self.cruise_target_x = self.current_pos[0]
                self.cruise_target_y = self.current_pos[1]
                return
            
            # 추적 (Visual Servoing)
            self.run_visual_servoing()

        elif self.state == STATE_DROP:
            # 정지 (Hover)
            self.send_velocity_cmd(0.0, 0.0, 0.0, 0.0)

    # =================================================================
    # Helper Functions
    # =================================================================

    def send_position_cmd(self, x, y, z):
        msg = Vector3()
        msg.x = float(x)
        msg.y = float(y)
        msg.z = float(z)
        self.pos_pub.publish(msg)

    def send_velocity_cmd(self, vx, vy, vz, yaw_rate):
        msg = Twist()
        msg.linear.x = float(vx)
        msg.linear.y = float(vy)
        msg.linear.z = float(vz)
        msg.angular.z = float(yaw_rate)
        self.vel_pub.publish(msg)

    def run_visual_servoing(self):
        # 영상 중심 (640x480 기준)
        cx, cy = 320, 240
        
        # P 게인 (반응 속도 조절)
        kp_x = 0.01  # 전후 이동 게인
        kp_y = 0.01  # 좌우 이동 게인

        # 에러 계산
        # 타겟이 화면 아래(y > 240) -> 드론은 전진해야 함 (+x 속도)
        error_x = (self.target_v - cy) 
        
        # 타겟이 화면 오른쪽(x > 320) -> 드론은 오른쪽으로 가야 함 (-y 속도? ROS 좌표계 확인 필요)
        # ROS 좌표계: x(전), y(좌), z(상)
        # 화면 오른쪽 = 드론 우측 = -y 방향
        error_y = (self.target_u - cx)

        cmd_vx = kp_x * error_x
        cmd_vy = -kp_y * error_y # 부호 반대 주의 (화면 우측은 y좌표 증가, 드론 우측은 y속도 감소)

        # 속도 제한 (너무 빠르면 위험)
        cmd_vx = max(min(cmd_vx, 2.0), -2.0)
        cmd_vy = max(min(cmd_vy, 2.0), -2.0)

        self.send_velocity_cmd(cmd_vx, cmd_vy, 0.0, 0.0)


def main(args=None):
    rclpy.init(args=args)
    node = MissionManagerNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()