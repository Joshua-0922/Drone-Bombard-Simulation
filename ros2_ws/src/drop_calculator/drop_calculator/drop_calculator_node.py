#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
import math

from std_msgs.msg import Bool, Float32
from nav_msgs.msg import Odometry


class DropCalculatorNode(Node):
    def __init__(self):
        super().__init__('drop_calculator_node')

        self.declare_parameter('x_marker_x', 11.0)   # from x_marker_world.sdf pose
        self.declare_parameter('x_marker_y', 10.0)

        self.marker_x = self.get_parameter('x_marker_x').value
        self.marker_y = self.get_parameter('x_marker_y').value

        self.is_falling = False
        self.has_scored = False
        self.payload_x = 0.0
        self.payload_y = 0.0
        self.payload_z = 1.0

        # Payload pose from OdometryPublisher → ros_gz_bridge → /drone/payload/position
        self.odom_sub = self.create_subscription(
            Odometry, '/drone/payload/position', self.payload_odom_callback, 10)

        # Drop trigger: rl_navigation publishes Bool(False) at drop
        self.drop_cmd_sub = self.create_subscription(
            Bool, '/drone/payload/drop_cmd_raw', self.drop_cmd_callback, 10)

        self.reward_pub = self.create_publisher(Float32, '/rl/drop_error', 10)
        self.timer = self.create_timer(0.05, self.check_impact)

        self.get_logger().info(
            f"Drop Calculator started. Target: ({self.marker_x}, {self.marker_y})")

    def payload_odom_callback(self, msg):
        self.payload_x = msg.pose.pose.position.x
        self.payload_y = msg.pose.pose.position.y
        self.payload_z = msg.pose.pose.position.z

    def drop_cmd_callback(self, msg):
        if not msg.data and not self.is_falling and not self.has_scored:
            self.get_logger().warn("Referee: Drop detected! Tracking payload...")
            self.is_falling = True

    def check_impact(self):
        if not self.is_falling:
            return
        if self.payload_z <= 0.04:   # cylinder radius=0.03m → ground contact
            self.is_falling = False
            self.has_scored = True
            error = math.sqrt(
                (self.payload_x - self.marker_x) ** 2 +
                (self.payload_y - self.marker_y) ** 2)
            self.get_logger().info(f"--- IMPACT! Target:({self.marker_x:.2f},{self.marker_y:.2f}) "
                                   f"Impact:({self.payload_x:.2f},{self.payload_y:.2f})")
            self.get_logger().error(f">>> FINAL ERROR: {error:.2f} meters <<<")
            self.reward_pub.publish(Float32(data=error))


def main(args=None):
    rclpy.init(args=args)
    node = DropCalculatorNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
