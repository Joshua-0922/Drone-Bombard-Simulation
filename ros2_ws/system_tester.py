import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Point, Twist
from nav_msgs.msg import Odometry
from std_msgs.msg import Bool
import time

class SystemTester(Node):
    def __init__(self):
        super().__init__('system_tester')

        # 1. [Publish] ?? ??? ?? (YOLO ?? + FC ??)
        self.target_pub = self.create_publisher(Point, '/target/pixel_coords', 10)
        self.odom_pub = self.create_publisher(Odometry, '/drone/odometry', 10)

        # 2. [Subscribe] ?? ??? ?? (? ??? ? ????? ??)
        self.create_subscription(Twist, '/drone/cmd_vel', self.cmd_vel_callback, 10)
        self.create_subscription(Bool, '/payload/drop_cmd', self.drop_callback, 10)

        # 0.1??? ??? ?? (10Hz)
        self.timer = self.create_timer(0.1, self.timer_callback)
        self.start_time = time.time()

        print("=== SYSTEM TESTER STARTED ===")
        print("Sending Fake Target (u=400, v=300) & Fake Altitude (20m)...")

    def timer_callback(self):
        # --- A. ?? ?? ??? ?? (?? ???? ?? ???) ---
        target_msg = Point()
        target_msg.x = 400.0  # u (?? ??)
        target_msg.y = 300.0  # v (?? ??)
        target_msg.z = 0.0    # ?? ? ?
        self.target_pub.publish(target_msg)

        # --- B. ?? ?? ?? ?? (?? 20m, ?? ?? 5m/s) ---
        odom_msg = Odometry()
        odom_msg.header.stamp = self.get_clock().now().to_msg()
        odom_msg.header.frame_id = "map"
        
        # ?? (?? 20m)
        odom_msg.pose.pose.position.z = 20.0 
        # ?? (x??? 5m/s ?? ?)
        odom_msg.twist.twist.linear.x = 5.0  
        odom_msg.twist.twist.linear.z = -0.1 # ?? ?? ?
        
        self.odom_pub.publish(odom_msg)

    def cmd_vel_callback(self, msg):
        # path_generation? ?? ?? ?? ??
        self.get_logger().info(f"[PathGen] Vel Command -> Linear X: {msg.linear.x:.2f}, Angular Z: {msg.angular.z:.2f}")

    def drop_callback(self, msg):
        # drop_calculator? ?? ?? ?? ??
        if msg.data:
            self.get_logger().warn(f"[DropCalc] ?? DROP COMMAND RECEIVED! ??")
        else:
            # ?? ??? ??? ? ??? ?? ?? ??
            # self.get_logger().info(f"[DropCalc] Holding payload...")
            pass

def main(args=None):
    rclpy.init(args=args)
    node = SystemTester()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
