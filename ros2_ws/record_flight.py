"""Record the drone's camera feeds to MP4 while a model drives the flight.

Subscribes to the onboard downward RGB camera and the YOLO-annotated image and
writes each to an .mp4 via OpenCV (no ffmpeg needed). Run on ROS_DOMAIN_ID=0
alongside `ros2 run rl_navigation evaluate`.

Runs for a fixed DURATION (argv[1] seconds) using a spin_once loop, then
finalises the files in normal control flow — does NOT rely on signals, which do
not propagate through rclpy.spin() once the camera publisher goes away (the moov
atom would never get written → unplayable mp4).
"""
import os
import sys
import time

import cv2
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSReliabilityPolicy
from sensor_msgs.msg import Image
from cv_bridge import CvBridge

OUT_DIR = '/workspace/ros2_ws/rl_eval_results'
FPS = 15.0
DURATION = float(sys.argv[1]) if len(sys.argv) > 1 else 150.0
TOPICS = {
    '/camera/rgb/image_raw': os.path.join(OUT_DIR, 'flight_raw.mp4'),
    '/vision/annotated_image': os.path.join(OUT_DIR, 'flight_annotated.mp4'),
}


class Recorder(Node):
    def __init__(self):
        super().__init__('flight_recorder')
        self.bridge = CvBridge()
        self.writers = {}
        self.counts = {t: 0 for t in TOPICS}
        qos = QoSProfile(depth=10)
        qos.reliability = QoSReliabilityPolicy.BEST_EFFORT
        for topic, path in TOPICS.items():
            self.create_subscription(Image, topic, self._cb(topic, path), qos)
            self.get_logger().info(f'subscribed {topic} -> {path}')

    def _cb(self, topic, path):
        def cb(msg):
            try:
                img = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
            except Exception:
                try:
                    img = self.bridge.imgmsg_to_cv2(msg)
                    if img.ndim == 2:
                        img = cv2.cvtColor(img, cv2.COLOR_GRAY2BGR)
                except Exception:
                    return
            h, w = img.shape[:2]
            if topic not in self.writers:
                vw = cv2.VideoWriter(
                    path, cv2.VideoWriter_fourcc(*'mp4v'), FPS, (w, h))
                self.writers[topic] = vw
                self.get_logger().info(f'opened {path} {w}x{h}')
            self.writers[topic].write(img)
            self.counts[topic] += 1
        return cb

    def finalize(self):
        for topic, vw in self.writers.items():
            vw.release()
            self.get_logger().info(f'{topic}: {self.counts[topic]} frames written')


def main():
    rclpy.init()
    node = Recorder()
    node.get_logger().info(f'recording for {DURATION:.0f}s...')
    end = time.time() + DURATION
    try:
        while rclpy.ok() and time.time() < end:
            rclpy.spin_once(node, timeout_sec=0.1)
    except KeyboardInterrupt:
        pass
    finally:
        node.finalize()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
        print('RECORDER DONE', dict(node.counts), flush=True)


if __name__ == '__main__':
    main()
