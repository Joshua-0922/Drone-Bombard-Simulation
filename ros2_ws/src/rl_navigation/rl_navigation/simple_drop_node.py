#!/usr/bin/env python3
"""simple_drop_node — test node: 2 m/s forward in TRACKING; drop when confidence > 0.5."""
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Point, Twist
from std_msgs.msg import String, Bool, Empty

CONFIDENCE_THRESHOLD = 0.5
FORWARD_SPEED = 2.0  # m/s


class SimpleDropNode(Node):
    def __init__(self):
        super().__init__('simple_drop_node')
        self.mission_state = 'IDLE'
        self.has_dropped = False

        self.state_sub = self.create_subscription(
            String, '/mission/state', self._state_cb, 10)
        self.vision_sub = self.create_subscription(
            Point, '/target/pixel_coords', self._vision_cb, 10)

        self.vel_pub = self.create_publisher(Twist, '/drone/cmd/velocity', 10)
        self.detach_pub = self.create_publisher(Empty, '/payload/drop_cmd', 10)
        self.drop_raw_pub = self.create_publisher(Bool, '/drone/payload/drop_cmd_raw', 10)

        self.create_timer(0.1, self._loop)
        self.get_logger().info(
            'SimpleDropNode ready (confidence threshold=%.1f).' % CONFIDENCE_THRESHOLD)

    def _state_cb(self, msg):
        self.mission_state = msg.data

    def _vision_cb(self, msg):
        if self.mission_state != 'TRACKING' or self.has_dropped:
            return
        if msg.z > CONFIDENCE_THRESHOLD:
            self.get_logger().warn(
                'Target confidence %.2f > %.1f — dropping payload.' % (msg.z, CONFIDENCE_THRESHOLD))
            self.detach_pub.publish(Empty())
            drop_msg = Bool()
            drop_msg.data = False  # False = drop event (inverted semantics for drop_calculator)
            self.drop_raw_pub.publish(drop_msg)
            self.has_dropped = True
            self.get_logger().info('Drop command sent.')

    def _loop(self):
        if self.mission_state != 'TRACKING':
            return
        cmd = Twist()
        cmd.linear.x = FORWARD_SPEED
        self.vel_pub.publish(cmd)


def main(args=None):
    rclpy.init(args=args)
    node = SimpleDropNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
