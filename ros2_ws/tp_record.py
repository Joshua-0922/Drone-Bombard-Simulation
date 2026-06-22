"""Record a ROS sensor_msgs/Image topic to MP4 (duration-bounded, clean finalize).

Usage: python3 tp_record.py TOPIC OUT [DURATION_S] [FPS]
"""
import sys
import time

import cv2
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSReliabilityPolicy
from sensor_msgs.msg import Image
from cv_bridge import CvBridge

TOPIC = sys.argv[1]
OUT = sys.argv[2]
DUR = float(sys.argv[3]) if len(sys.argv) > 3 else 100.0
FPS = float(sys.argv[4]) if len(sys.argv) > 4 else 20.0


class Rec(Node):
    def __init__(self):
        super().__init__('tp_recorder')
        self.bridge = CvBridge()
        self.vw = None
        self.n = 0
        qos = QoSProfile(depth=10)
        qos.reliability = QoSReliabilityPolicy.BEST_EFFORT
        self.create_subscription(Image, TOPIC, self.cb, qos)

    def cb(self, msg):
        try:
            img = self.bridge.imgmsg_to_cv2(msg, 'bgr8')
        except Exception:
            img = self.bridge.imgmsg_to_cv2(msg)
            if img.ndim == 2:
                img = cv2.cvtColor(img, cv2.COLOR_GRAY2BGR)
        if self.vw is None:
            h, w = img.shape[:2]
            self.vw = cv2.VideoWriter(
                OUT, cv2.VideoWriter_fourcc(*'mp4v'), FPS, (w, h))
        self.vw.write(img)
        self.n += 1


rclpy.init()
node = Rec()
end = time.time() + DUR
while rclpy.ok() and time.time() < end:
    rclpy.spin_once(node, timeout_sec=0.1)
if node.vw is not None:
    node.vw.release()
print(f'TP_REC frames={node.n} -> {OUT}', flush=True)
node.destroy_node()
rclpy.shutdown()
