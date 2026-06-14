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

        # Cruise heading toward the marker (NE). drone_controller negates the y of
        # /drone/cmd/position (PX4_East = -cmd.y), so a NEGATIVE cruise_speed_y
        # drives the drone toward +Gazebo East (the marker side). North uses +x.
        self.cruise_speed_x = 1.0     # +North velocity (m/s) → toward marker North=10
        self.cruise_speed_y = -1.0    # -cmd.y → +East via controller negation → toward marker East=11

        self.rl_mode = self.declare_parameter('rl_mode', False).value
        # Number of consecutive 10 Hz ticks with target visible before
        # CRUISE → TRACKING transition fires (0.5 s of stable detection).
        # Proximity trigger is the primary TRACKING mechanism; this is secondary.
        self._stable_detect_ticks = 5
        self._stable_detect_count = 0
        # Proximity fallback: transition to TRACKING when within this distance
        # of target (NED metres), regardless of YOLO.  Guards against camera
        # misconfiguration causing YOLO to never (or always) fire.
        # Target in PX4 NED (actual frame, as reported by VehicleLocalPosition).
        # MEASURED 2026-06-14: PX4 EKF East = +Gazebo_East (NO reversal — the prior
        # "EKF East = -Gazebo_East" assumption was a misdiagnosis). Marker at Gazebo
        # ENU (East=11, North=10) → PX4 NED actual (North=10, East=+11). Proximity
        # compares current_pos (actual NED) to this, so East target is +11.
        _t_ned_x = self.declare_parameter('target_ned_x', 10.0).value
        _t_ned_y = self.declare_parameter('target_ned_y', 11.0).value
        self._target_ned = [_t_ned_x, _t_ned_y]
        # 1.5 m: hold/fallback distance. With the camera now pointing straight down
        # (model.sdf pitch=90deg), the marker centers in-frame as the drone approaches,
        # so YOLO-primary should lock during the approach; this 1.5 m hold is the
        # reachable fallback point (0.7 m was unreachable -> TRACKING timeouts).
        self._proximity_threshold = self.declare_parameter('proximity_m', 1.5).value
        # Proximity is now a FALLBACK ONLY. YOLO stable-detection is the PRIMARY
        # TRACKING trigger (RL takes over when the camera locks onto the marker).
        # If the drone sits within proximity_m of the marker for this many seconds
        # WITHOUT a YOLO lock, fall back to proximity so the episode isn't wasted.
        self._proximity_fallback_secs = self.declare_parameter('proximity_fallback_secs', 4.0).value
        self._proximity_in_range_since = 0.0
        # Mandatory cruise gate: ignore ALL detection/proximity triggers for this
        # many seconds after entering CRUISE.  Prevents spurious TRACKING from
        # stale /target/pixel_coords data or brief YOLO false-positives at startup.
        # At cruise_speed (1,1) NED @ 1 m/s, 8 s ≈ 11 m toward the marker.
        self._cruise_gate_secs = self.declare_parameter('cruise_gate_secs', 2.0).value
        self._cruise_enter_time = 0.0

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
        self.arm_ned_z = None            # NED z at arming — TAKEOFF uses relative check
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
            # Record the NED z at arm time for relative altitude check.
            # This makes TAKEOFF robust regardless of where PX4's EKF home
            # is set (e.g. after a world reset where home may be at cruise altitude).
            self.arm_ned_z = self.current_pos[2]
            self.on_ground_at_arm = True  # always proceed; relative check handles safety
            self.armed_stamp = time.time()
            self.get_logger().info(
                f'PX4 armed at NED z={self.arm_ned_z:.2f} m — starting TAKEOFF sequence.')
        self.px4_armed = (msg.arming_state == 2)

    def vision_callback(self, msg):
        # Ignore detections during TAKEOFF/DROP — only update tracking state
        # when the FSM is actually in a state that uses it.
        if self.state not in (STATE_CRUISE, STATE_TRACK):
            return
        if msg.z > 0.0:
            # Spatial filter: reject detections far from image center (640×480).
            # Real marker (directly below at cruise altitude) appears within ~200px
            # of center; edge-of-frame detections are false positives from distant
            # ground patterns that happen to look X-like.
            dx = msg.x - 320.0
            dy = msg.y - 240.0
            pixel_dist = math.sqrt(dx * dx + dy * dy)
            if pixel_dist > 200.0:
                return
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

            # Altitude check — EKF-home-aware relative check.
            #
            # Problem: PX4 SITL EKF home can drift above physical ground over many
            # episode resets (drone arms mid-descent during AUTO.LAND → EKF home set
            # at, e.g., +1.44 m ENU instead of 0 m). With home at +1.44 m, the drone
            # commanded to 5 m ENU sits at NED z = -(5-1.44) = -3.56, which fails the
            # old absolute check z ≤ -4.75.
            #
            # Fix: use arm_ned_z to handle both cases:
            #   • On-ground arm (arm_ned_z > 0, EKF home above physical ground):
            #       target = arm_ned_z - target_altitude*0.95  (climb 4.75 m above arm)
            #   • Mid-air arm  (arm_ned_z ≤ 0, drone already at altitude after reset):
            #       target = -target_altitude*0.95              (absolute ≤ -4.75)
            #   Combined: threshold = max(arm_ned_z, 0) - target_altitude*0.95
            # This is equivalent to: climb at least target_altitude*0.95 meters above
            # the physical arm point, regardless of EKF home drift.
            # Requires 0.3 s EKF settle after arming and 2 consecutive ticks (~0.2 s).
            min_armed_secs = 0.3
            armed_long_enough = (
                self.armed_stamp is not None
                and (time.time() - self.armed_stamp) >= min_armed_secs
            )
            arm_z = self.arm_ned_z if self.arm_ned_z is not None else 0.0
            altitude_threshold = max(arm_z, 0.0) - (self.target_altitude * 0.95)
            if (armed_long_enough
                    and self.current_pos[2] <= altitude_threshold):
                self.altitude_hold_ticks += 1
            else:
                self.altitude_hold_ticks = 0
            if self.altitude_hold_ticks >= 2:
                self.get_logger().info("Altitude Reached! Switching to CRUISE mode.")
                self.state = STATE_CRUISE
                self.altitude_hold_ticks = 0
                
                # [중요] 순항 시작점을 현재 위치로 초기화해야 튐 현상 방지
                self.cruise_target_x = self.current_pos[0]
                self.cruise_target_y = self.current_pos[1]
                # Clear any stale detections accumulated during TAKEOFF
                self.target_visible = False
                self.last_detection_time = 0
                self._cruise_enter_time = time.time()
                self._proximity_in_range_since = 0.0

        elif self.state == STATE_CRUISE:
            # Mandatory cruise gate — keep flying toward (1,1) for the first N seconds
            # regardless of YOLO detections or proximity.  Prevents spurious TRACKING
            # from stale topic data or brief false-positives at infra startup.
            gate_elapsed = time.time() - self._cruise_enter_time
            if gate_elapsed < self._cruise_gate_secs:
                self._stable_detect_count = 0
                self.cruise_target_x += (self.cruise_speed_x * 0.1)
                self.cruise_target_y += (self.cruise_speed_y * 0.1)
                self.send_position_cmd(
                    self.cruise_target_x, self.cruise_target_y, self.target_altitude)
                if int(gate_elapsed) != int(gate_elapsed - 0.1):
                    self.get_logger().info(
                        f'[CRUISE GATE] {gate_elapsed:.1f}/{self._cruise_gate_secs:.0f}s '
                        f'— target NED ({self.cruise_target_x:.1f}, {self.cruise_target_y:.1f})')
                return

            # === PRIMARY trigger: YOLO stable detection ===
            # RL takes over the moment the camera locks stably onto the marker.
            if self.target_visible:
                self._stable_detect_count += 1
                if self._stable_detect_count >= self._stable_detect_ticks:
                    self.get_logger().warn(
                        f"TARGET STABLY DETECTED ({self._stable_detect_count} ticks)! "
                        "Switching to TRACKING (YOLO primary).")
                    self.state = STATE_TRACK
                    self._stable_detect_count = 0
                    self._proximity_in_range_since = 0.0
                    return
                # Marker in view but not yet stable — hold while locking on.
                self.send_position_cmd(
                    self.cruise_target_x, self.cruise_target_y, self.target_altitude)
                return

            # No detection this tick — reset the stable-lock counter.
            self._stable_detect_count = 0

            # === FALLBACK trigger: proximity (only if YOLO fails to lock) ===
            # If the drone reaches the marker but YOLO never locks on, HOLD over
            # the marker (don't overshoot) and fall back to proximity after
            # proximity_fallback_secs so the episode isn't wasted on a YOLO miss.
            dx = self.current_pos[0] - self._target_ned[0]
            dy = self.current_pos[1] - self._target_ned[1]
            d_xy = math.sqrt(dx * dx + dy * dy)
            if d_xy <= self._proximity_threshold:
                if self._proximity_in_range_since == 0.0:
                    self._proximity_in_range_since = time.time()
                elif (time.time() - self._proximity_in_range_since
                        >= self._proximity_fallback_secs):
                    self.get_logger().warn(
                        f"PROXIMITY FALLBACK: in range {self._proximity_fallback_secs:.0f}s "
                        f"without YOLO lock (d_xy={d_xy:.1f} m). Switching to TRACKING.")
                    self.state = STATE_TRACK
                    self._stable_detect_count = 0
                    self._proximity_in_range_since = 0.0
                    return
                # Hold over the marker while giving YOLO more time to lock.
                self.send_position_cmd(
                    self.cruise_target_x, self.cruise_target_y, self.target_altitude)
            else:
                # Not yet in range — keep cruising toward the marker.
                self._proximity_in_range_since = 0.0
                self.cruise_target_x += (self.cruise_speed_x * 0.1)
                self.cruise_target_y += (self.cruise_speed_y * 0.1)
                self.send_position_cmd(
                    self.cruise_target_x, self.cruise_target_y, self.target_altitude)

        elif self.state == STATE_TRACK:
            if not self.target_visible:
                if self.rl_mode:
                    # In RL mode the env manages episode termination.
                    # Do NOT revert to CRUISE here — that would publish
                    # position commands that conflict with RL velocity commands.
                    pass
                else:
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