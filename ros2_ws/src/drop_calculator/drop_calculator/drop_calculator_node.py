#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
import math

from std_msgs.msg import Bool, Float32
from gazebo_msgs.msg import ModelStates, LinkStates

class DropCalculatorNode(Node):
    def __init__(self):
        super().__init__('drop_calculator_node')

        # --- 상태 변수 ---
        self.marker_x = 0.0
        self.marker_y = 0.0
        self.is_falling = False  # 투하 중인지 여부
        self.has_scored = False  # 점수 채점 완료 여부

        # --- Subscribers (가제보 훔쳐보기) ---
        # 1. 'X' 마커 위치 찾기용
        self.model_sub = self.create_subscription(
            ModelStates, '/gazebo/model_states', self.model_state_callback, 10)
        
        # 2. 페이로드 낙하 추적용
        self.link_sub = self.create_subscription(
            LinkStates, '/gazebo/link_states', self.link_state_callback, 10)

        # 3. RL 에이전트의 투하 명령 감지
        # (진공 그립퍼 토픽을 직접 감시하여 False(끄기)가 될 때 낙하 시작으로 간주)
        self.drop_cmd_sub = self.create_subscription(
            Bool, '/drone/payload/drop_cmd_raw', self.drop_cmd_callback, 10)

        # --- Publisher (점수 통보) ---
        self.reward_pub = self.create_publisher(Float32, '/rl/drop_error', 10)

        self.get_logger().info("God-Mode Referee (Drop Calculator) Started!")

    def model_state_callback(self, msg):
        try:
            # 리스트에서 'x_marker_2'의 인덱스 찾기
            idx = msg.name.index('x_marker_2')
            self.marker_x = msg.pose[idx].position.x
            self.marker_y = msg.pose[idx].position.y
        except ValueError:
            pass # 아직 스폰되지 않음

    def drop_cmd_callback(self, msg):
        # msg.data == False 이면 진공 그립퍼가 꺼짐 -> 투하 시작!
        if not msg.data and not self.is_falling and not self.has_scored:
            self.get_logger().warn("Referee: Drop detected! Tracking payload...")
            self.is_falling = True

    def link_state_callback(self, msg):
        if not self.is_falling:
            return

        try:
            # 드론 이름이 바뀌어도 찾을 수 있도록 'payload_link'가 포함된 이름 찾기
            payload_idx = next(i for i, name in enumerate(msg.name) if 'payload_link' in name)
            
            payload_x = msg.pose[payload_idx].position.x
            payload_y = msg.pose[payload_idx].position.y
            payload_z = msg.pose[payload_idx].position.z

            # 페이로드 구체의 반지름이 0.03m이므로, z가 0.04m 이하로 떨어지면 충돌로 간주
            if payload_z <= 0.04:
                self.is_falling = False
                self.has_scored = True
                
                # 피타고라스 정리로 오차(m) 계산
                error = math.sqrt((payload_x - self.marker_x)**2 + (payload_y - self.marker_y)**2)
                
                self.get_logger().info(f"--- IMPACT! ---")
                self.get_logger().info(f"Target: ({self.marker_x:.2f}, {self.marker_y:.2f})")
                self.get_logger().info(f"Impact: ({payload_x:.2f}, {payload_y:.2f})")
                self.get_logger().error(f">>> FINAL ERROR: {error:.2f} meters <<<")

                # 결과 퍼블리시
                self.reward_pub.publish(Float32(data=error))
                
        except StopIteration:
            pass # payload_link를 찾을 수 없음

def main(args=None):
    rclpy.init(args=args)
    node = DropCalculatorNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()