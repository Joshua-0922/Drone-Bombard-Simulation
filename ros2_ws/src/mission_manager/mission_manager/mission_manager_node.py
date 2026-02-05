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
        self.cruise_speed_x = 1.0     # 순항 속도 (m/s)
        self.search_direction = 'X'   # X축(북쪽)으로 직진
        
        # --- [2] Publishers ---
        # 1. 위치 명령 (To DroneController - 이륙/순항용)
        self.pos_pub = self.create_publisher(Vector3, '/drone/cmd/position', 10)
        
        # 2. 속도 명령 (To DroneController - 추적용)
        # *원래는 RL 노드가 보내야 하지만, 아직 없으므로 여기서 임시 수행*
        self.vel_pub = self.create_publisher(Twist, '/drone/cmd/velocity', 10)
        
        # 3. 현재 미션 상태 알림 (디버깅용)
        self.state_pub = self.create_publisher(String, '/mission/state', 10)

        # --- [3] Subscribers ---
        # 1. 시각 정보 (X 마커 픽셀 좌표)
        self.vision_sub = self.create_subscription(
            Point, '/target/pixel_coords', self.vision_callback, 10)
        
        # 2. 투하 완료 신호
        self.drop_sub = self.create_subscription(
            Bool, '/payload/drop_cmd', self.drop_callback, 10)

        # 3. 고도 정보 (이륙 완료 판단용)
        qos_profile = QoSProfile(reliability=ReliabilityPolicy.BEST_EFFORT, depth=1)
        self.local_pos_sub = self.create_subscription(
            VehicleLocalPosition, '/fmu/out/vehicle_local_position', self.local_pos_callback, qos_profile)

        # --- [4] 내부 변수 ---
        self.state = STATE_TAKEOFF
        self.current_pos = [0.0, 0.0, 0.0] # [x, y, z] (NED)
        self.start_time = time.time()
        
        # 순항 목표 지점
        self.cruise_target_x = 0.0
        
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
        # PX4는 NED 좌표계 사용 (z가 음수여야 하늘)
        self.current_pos = [msg.x, msg.y, msg.z]

    def vision_callback(self, msg):
        # 타겟 발견 시 데이터 업데이트
        if msg.z > 0.5: # Confidence가 0.5 이상일 때만 (msg.z를 confidence로 활용 가정)
            self.target_visible = True
            self.target_u = msg.x # 화면 가로 (Pixel)
            self.target_v = msg.y # 화면 세로 (Pixel)
            self.last_detection_time = time.time()
        else:
            self.target_visible = False

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

        # 타겟 소실 처리 (1초 이상 안 보이면 놓친 것)
        if time.time() - self.last_detection_time > 1.0:
            self.target_visible = False

        # --- FSM (Finite State Machine) ---
        
        if self.state == STATE_TAKEOFF:
            # 10m 상공으로 이륙 명령
            # drone_controller는 ROS좌표(z=+)를 받음
            self.send_position_cmd(0.0, 0.0, self.target_altitude)

            # 고도가 95% 도달하면 CRUISE 모드로 전환
            # NED 좌표계에서 고도는 음수임 (예: -10m)
            if self.current_pos[2] <= -(self.target_altitude * 0.95):
                self.get_logger().info("Altitude Reached! Switching to CRUISE mode.")
                self.state = STATE_CRUISE
                self.cruise_target_x = self.current_pos[0] # 현재 위치에서 시작

        elif self.state == STATE_CRUISE:
            # --- [이벤트] 타겟 발견 시 추적 모드로 즉시 전환 (Intercept) ---
            if self.target_visible:
                self.get_logger().warn("TARGET DETECTED! Engaging Intercept Mode.")
                self.state = STATE_TRACK
                return

            # --- [행동] 앞으로 직진 (North) ---
            # 0.1초마다 조금씩 목표 지점을 갱신하여 앞으로 밀고 나감
            self.cruise_target_x += (self.cruise_speed_x * 0.1)
            
            # ROS 좌표계 (x=앞, y=왼쪽, z=위)
            self.send_position_cmd(self.cruise_target_x, 0.0, self.target_altitude)

        elif self.state == STATE_TRACK:
            # --- [이벤트] 타겟을 놓쳤다면? ---
            if not self.target_visible:
                self.get_logger().info("Target Lost... Resuming Search.")
                self.state = STATE_CRUISE
                # 다시 현재 위치부터 순항 시작
                self.cruise_target_x = self.current_pos[0] 
                return
            
            # --- [이벤트] 투하 완료? ---
            # (drop_callback에서 처리됨)

            # --- [행동] Visual Servoing (임시 추적기) ---
            # 나중에 rl_navigator 노드가 생기면 이 부분은 삭제하고,
            # mission_manager는 아무것도 안 보내면 됩니다. (RL 노드가 보낼 테니까)
            self.run_visual_servoing()

        elif self.state == STATE_DROP:
            # 투하 후 행동: 제자리 호버링 또는 복귀
            self.send_velocity_cmd(0.0, 0.0, 0.0, 0.0) # 정지
            # 복귀 코드를 넣으려면 STATE_RETURN으로 전환

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
        """
        [임시 기능] RL Navigator가 없을 때 타겟을 추적하는 P-제어기.
        이미지 중심(320, 240)에 타겟이 오도록 속도를 조절함.
        """
        img_w, img_h = 640, 480
        cx, cy = img_w / 2, img_h / 2
        
        # 1. 에러 계산 (픽셀 단위)
        # 카메라 좌표계: x(우), y(하)
        # 드론 좌표계: x(전), y(좌), z(상)
        
        err_u = -(self.target_u - cx) # 화면 오른쪽(+)에 있으면 드론은 오른쪽(-y)으로 가야 함?? 
                                      # drone_controller는 ROS좌표(좌측+) -> 화면 우측은 ROS -y
        err_v = -(self.target_v - cy) # 화면 아래(+)에 있으면 드론은 뒤로(-x) 가야 함? 
                                      # 아니지, 화면 아래에 있다는 건 타겟이 덜 온 것(앞에 있음) -> 전진(+x)
        
        # 좌표계 정리 (하방 카메라 기준, 드론 헤딩이 위쪽일 때)
        # - 타겟이 화면 위쪽(v 작음) -> 드론이 더 전진해야 함 (+x)
        # - 타겟이 화면 오른쪽(u 큼) -> 드론이 오른쪽으로 가야 함 (-y)
        
        cmd_x = 0.0
        cmd_y = 0.0
        
        # P 게인 (튜닝 필요)
        kp_x = 0.005
        kp_y = 0.005
        
        # 화면 Y축 오차 -> 드론 X축 속도 (전후)
        # 타겟이 화면 중심보다 위에 있으면(v < cy), 에러 양수 -> 전진
        cmd_x = kp_x * (cy - self.target_v)
        
        # 화면 X축 오차 -> 드론 Y축 속도 (좌우)
        # 타겟이 화면 중심보다 오른쪽에 있으면(u > cx), 에러 음수 -> 우이동(ROS -y)
        cmd_y = kp_y * (cx - self.target_u)

        # Yaw 제어 (옵션: 항상 머리를 타겟 쪽으로 돌리려면 사용)
        cmd_yaw = 0.0 
        
        # 접근 속도 제한
        cmd_x = max(min(cmd_x, 2.0), -2.0)
        cmd_y = max(min(cmd_y, 2.0), -2.0)

        # 명령 전송
        self.send_velocity_cmd(cmd_x, cmd_y, 0.0, cmd_yaw)


def main(args=None):
    rclpy.init(args=args)
    node = MissionManagerNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()