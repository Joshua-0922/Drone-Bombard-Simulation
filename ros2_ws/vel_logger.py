"""Log the velocity setpoint PX4 receives, to measure control smoothness (wobble).

Subscribes to /fmu/in/trajectory_setpoint (the post-LPF command the controller
sends to PX4) and appends `t,vx,vy,vz` to a CSV. Runs for DURATION seconds
(argv[1]); output path is argv[2]. Only rows with finite velocity (VELOCITY /
RL control mode) are written — position-mode setpoints carry NaN velocity and
are skipped.

Usage: python3 vel_logger.py <duration_s> <out.csv>
"""
import math
import sys
import time

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy, HistoryPolicy
from px4_msgs.msg import TrajectorySetpoint

DURATION = float(sys.argv[1]) if len(sys.argv) > 1 else 120.0
OUT = sys.argv[2] if len(sys.argv) > 2 else '/tmp/vel_log.csv'


class VelLogger(Node):
    def __init__(self, fh):
        super().__init__('vel_logger')
        self.fh = fh
        self.n = 0
        qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            history=HistoryPolicy.KEEP_LAST, depth=10)
        self.create_subscription(
            TrajectorySetpoint, '/fmu/in/trajectory_setpoint', self.cb, qos)

    def cb(self, msg):
        vx, vy, vz = msg.velocity[0], msg.velocity[1], msg.velocity[2]
        if any(math.isnan(v) for v in (vx, vy, vz)):
            return  # position-mode setpoint (cruise) — not RL velocity control
        self.fh.write(f'{time.time():.4f},{vx:.5f},{vy:.5f},{vz:.5f}\n')
        self.n += 1


def main():
    rclpy.init()
    with open(OUT, 'w') as fh:
        fh.write('t,vx,vy,vz\n')
        node = VelLogger(fh)
        end = time.time() + DURATION
        while rclpy.ok() and time.time() < end:
            rclpy.spin_once(node, timeout_sec=0.1)
        node.get_logger().info(f'wrote {node.n} velocity rows -> {OUT}')
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
    print(f'VEL_LOGGER DONE rows={node.n} out={OUT}', flush=True)


if __name__ == '__main__':
    main()
