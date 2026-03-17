#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy

from geometry_msgs.msg import Point, Vector3, Twist
from std_msgs.msg import String, Bool
from px4_msgs.msg import VehicleLocalPosition, VehicleStatus

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
        self.target_altitude = 5.0    # 순항 고도 5m (RL: reduced for faster reset)
        
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
            Bool, '/drone/payload/drop_cmd_raw', self.drop_callback, 10)

        qos_profile = QoSProfile(reliability=ReliabilityPolicy.BEST_EFFORT, depth=1)
        self.local_pos_sub = self.create_subscription(
            VehicleLocalPosition, '/fmu/out/vehicle_local_position', self.local_pos_callback, qos_profile)

        self.px4_armed = False
        self.armed_stamp = None          # wall-clock time when first armed
        self.status_sub = self.create_subscription(
            VehicleStatus, '/fmu/out/vehicle_status',
            self.vehicle_status_callback, qos_profile)

        # --- [4] 내부 변수 ---
        self.state = STATE_TAKEOFF
        self.current_pos = [0.0, 0.0, 0.0] # [x, y, z] (NED)
        self.altitude_hold_ticks = 0     # consecutive ticks at target altitude
        self.on_ground_at_arm = None     # True = on ground when armed; False = floating
        
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

    def vehicle_status_callback(self, msg):
        if not self.px4_armed and msg.arming_state == 2:  # ARMING_STATE_ARMED
            # NED z ≈ 0 on ground; < -1.0 means drone is already > 1 m airborne.
            self.on_ground_at_arm = (self.current_pos[2] > -1.0)
            if self.on_ground_at_arm:
                self.get_logger().info('PX4 armed — starting TAKEOFF sequence.')
            else:
                self.get_logger().error(
                    f'GROUND CHECK FAILED: drone NED z={self.current_pos[2]:.2f} m '
                    f'at arming (> 1 m airborne). Spawn or DetachableJoint issue. '
                    f'TAKEOFF suppressed.')
            self.armed_stamp = time.time()
        self.px4_armed = (msg.arming_state == 2)

    def vision_callback(self, msg):
        # Ignore detections during TAKEOFF/DROP — only update tracking state
        # when the FSM is actually in a state that uses it.
        if self.state not in (STATE_CRUISE, STATE_TRACK):
            return
        if msg.z > 0.0:
            self.target_visible = True
            self.target_u = msg.x
            self.target_v = msg.y
            self.last_detection_time = time.time()

    def drop_callback(self, msg):
        if not msg.data:
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
            if not self.px4_armed:
                return
            # Ground check: refuse to climb if drone was already airborne when armed.
            if self.on_ground_at_arm is False:
                return
            self.send_position_cmd(0.0, 0.0, self.target_altitude)

            # Altitude check (NED: up = negative-z).
            # 1 s EKF settle guard after arming (reduced from 5 s; saves ~4 s per RL episode).
            # Requires 5 consecutive ticks (~0.5 s) at target altitude to prevent
            # spurious transitions from a single noisy reading.
            min_armed_secs = 1.0
            armed_long_enough = (
                self.armed_stamp is not None
                and (time.time() - self.armed_stamp) >= min_armed_secs
            )
            if armed_long_enough and self.current_pos[2] <= -(self.target_altitude * 0.95):
                self.altitude_hold_ticks += 1
            else:
                self.altitude_hold_ticks = 0
            if self.altitude_hold_ticks >= 5:
                self.get_logger().info("Altitude Reached! Switching to CRUISE mode.")
                self.state = STATE_CRUISE
                self.altitude_hold_ticks = 0
                
                # [중요] 순항 시작점을 현재 위치로 초기화해야 튐 현상 방지
                self.cruise_target_x = self.current_pos[0]
                self.cruise_target_y = self.current_pos[1]
                # Clear any stale detections accumulated during TAKEOFF
                self.target_visible = False
                self.last_detection_time = 0

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
            
            # 추적은 rl_navigation 노드가 /drone/cmd/velocity로 담당
            pass

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