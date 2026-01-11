#!/usr/bin/env python3
"""X-Marker Detection Node using YOLOv8.

This node subscribes to camera images from a downward-facing depth camera,
runs YOLOv8 inference to detect X-markers, and publishes detection results
including both pixel coordinates and NED world coordinates.
"""

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy

from sensor_msgs.msg import Image, CameraInfo
from geometry_msgs.msg import Point
from std_msgs.msg import Header
from px4_msgs.msg import VehicleLocalPosition

from cv_bridge import CvBridge
import cv2
import numpy as np
import math


class XMarkerDetectorNode(Node):
    """ROS 2 node for X-marker detection using YOLOv8."""

    def __init__(self):
        """Initialize the X-marker detector node."""
        super().__init__('xmarker_detector')

        # Declare parameters (use_sim_time is automatically declared)
        if not self.has_parameter('model_path'):
            self.declare_parameter('model_path',
                                   '/workspace/ros2_ws/yolo_workspace/runs/train/'
                                   'drone_bombard_train2/weights/best.pt')
        if not self.has_parameter('inference_rate'):
            self.declare_parameter('inference_rate', 10.0)  # Hz

        # Get parameters
        model_path = self.get_parameter('model_path').value
        inference_rate = self.get_parameter('inference_rate').value

        # Initialize CV Bridge
        self.bridge = CvBridge()

        # Initialize state variables
        self.latest_rgb = None
        self.latest_depth = None
        self.camera_matrix = None
        self.fx = None
        self.fy = None
        self.cx = None
        self.cy = None
        self.vehicle_position = None
        self.vehicle_heading = 0.0

        # Load YOLO model
        self.get_logger().info(f'Loading YOLO model from: {model_path}')
        try:
            from ultralytics import YOLO
            self.model = YOLO(model_path)
            self.get_logger().info('YOLO model loaded successfully')
        except Exception as e:
            self.get_logger().error(f'Failed to load YOLO model: {e}')
            self.model = None

        # QoS profile for PX4 and camera topics (BEST_EFFORT)
        qos_profile = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            history=HistoryPolicy.KEEP_LAST,
            depth=1
        )

        # QoS profile for publishers (RELIABLE)
        qos_reliable = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            history=HistoryPolicy.KEEP_LAST,
            depth=10
        )

        # Create subscribers
        self.rgb_sub = self.create_subscription(
            Image,
            '/camera/rgb/image_raw',
            self.rgb_callback,
            qos_profile
        )

        self.depth_sub = self.create_subscription(
            Image,
            '/camera/depth/image_raw',
            self.depth_callback,
            qos_profile
        )

        self.camera_info_sub = self.create_subscription(
            CameraInfo,
            '/camera/rgb/camera_info',
            self.camera_info_callback,
            qos_profile
        )

        self.vehicle_pos_sub = self.create_subscription(
            VehicleLocalPosition,
            '/fmu/out/vehicle_local_position',
            self.vehicle_position_callback,
            qos_profile
        )

        # Create publishers
        # Import custom message (will be generated after build)
        try:
            from vision_detection.msg import DetectionResult
            self.detection_pub = self.create_publisher(
                DetectionResult,
                '/vision/detections',
                qos_reliable
            )
        except ImportError:
            self.get_logger().warn(
                'DetectionResult message not found. '
                'Build the package first to generate messages.'
            )
            self.detection_pub = None

        self.annotated_pub = self.create_publisher(
            Image,
            '/vision/annotated_image',
            qos_reliable
        )

        # Create timer for inference at specified rate
        timer_period = 1.0 / inference_rate
        self.timer = self.create_timer(timer_period, self.detect_callback)

        self.get_logger().info(
            f'X-Marker Detector initialized at {inference_rate} Hz'
        )

    def rgb_callback(self, msg):
        """Store latest RGB image."""
        self.latest_rgb = msg

    def depth_callback(self, msg):
        """Store latest depth image."""
        self.latest_depth = msg

    def camera_info_callback(self, msg):
        """Extract camera intrinsics from CameraInfo message."""
        if self.camera_matrix is None:
            self.fx = msg.k[0]
            self.fy = msg.k[4]
            self.cx = msg.k[2]
            self.cy = msg.k[5]
            self.camera_matrix = np.array(msg.k).reshape(3, 3)
            self.get_logger().info(
                f'Camera intrinsics: fx={self.fx:.2f}, fy={self.fy:.2f}, '
                f'cx={self.cx:.2f}, cy={self.cy:.2f}'
            )

    def vehicle_position_callback(self, msg):
        """Store latest vehicle position in NED coordinates."""
        self.vehicle_position = msg
        # Extract heading (yaw) from quaternion or use directly if available
        # PX4 VehicleLocalPosition provides heading in radians
        self.vehicle_heading = msg.heading

    def pixel_to_camera_frame(self, u, v, depth):
        """Convert pixel coordinates to 3D camera frame coordinates.

        Args:
            u: Pixel x-coordinate
            v: Pixel y-coordinate
            depth: Depth value at (u, v) in meters

        Returns:
            tuple: (X_cam, Y_cam, Z_cam) in camera frame
        """
        if self.fx is None or depth <= 0 or np.isnan(depth) or np.isinf(depth):
            return None

        X_cam = (u - self.cx) * depth / self.fx
        Y_cam = (v - self.cy) * depth / self.fy
        Z_cam = depth

        return (X_cam, Y_cam, Z_cam)

    def camera_to_body_frame(self, X_cam, Y_cam, Z_cam):
        """Transform camera frame to body frame.

        Camera mounting: position [0.108, 0, -0.01], rotation [0, 1.5708, 0]
        Camera pitch -90° (pointing down).

        Args:
            X_cam: X coordinate in camera frame
            Y_cam: Y coordinate in camera frame
            Z_cam: Z coordinate in camera frame

        Returns:
            tuple: (X_body, Y_body, Z_body) in body frame
        """
        # Camera rotation: pitch -90 degrees (1.5708 radians)
        # X_cam (forward) → Z_body (down)
        # Y_cam (right) → Y_body (right)
        # Z_cam (down in camera) → -X_body (backward in body)

        X_body = -Z_cam + 0.108  # Camera offset in X
        Y_body = Y_cam
        Z_body = X_cam - 0.01    # Camera offset in Z

        return (X_body, Y_body, Z_body)

    def body_to_ned_frame(self, X_body, Y_body, Z_body):
        """Transform body frame to NED world frame.

        Args:
            X_body: X coordinate in body frame
            Y_body: Y coordinate in body frame
            Z_body: Z coordinate in body frame

        Returns:
            tuple: (X_ned, Y_ned, Z_ned) in NED frame, or None if invalid
        """
        if self.vehicle_position is None:
            return None

        # Vehicle position in NED
        vehicle_x = self.vehicle_position.x
        vehicle_y = self.vehicle_position.y
        vehicle_z = self.vehicle_position.z

        # Rotate by vehicle heading (yaw)
        cos_yaw = math.cos(self.vehicle_heading)
        sin_yaw = math.sin(self.vehicle_heading)

        X_ned = vehicle_x + (X_body * cos_yaw - Y_body * sin_yaw)
        Y_ned = vehicle_y + (X_body * sin_yaw + Y_body * cos_yaw)
        Z_ned = vehicle_z + Z_body

        return (X_ned, Y_ned, Z_ned)

    def detect_callback(self):
        """Run YOLOv8 inference and publish detection results."""
        if self.latest_rgb is None or self.model is None:
            return

        try:
            # Convert ROS Image to OpenCV format
            cv_image = self.bridge.imgmsg_to_cv2(self.latest_rgb, 'bgr8')

            # Run YOLO inference
            results = self.model(cv_image, verbose=False)

            # Process detection results
            detections = results[0].boxes
            detected = False
            confidence = 0.0
            bbox_center_x = 0.0
            bbox_center_y = 0.0
            bbox_width = 0.0
            bbox_height = 0.0
            camera_coords = Point()
            ned_coords = Point()
            ned_valid = False
            depth_value = 0.0

            if len(detections) > 0:
                # Select detection with highest confidence
                best_idx = detections.conf.argmax()
                box = detections.xyxy[best_idx].cpu().numpy()
                confidence = float(detections.conf[best_idx].cpu().numpy())

                # Bounding box coordinates
                x1, y1, x2, y2 = box
                bbox_center_x = float((x1 + x2) / 2)
                bbox_center_y = float((y1 + y2) / 2)
                bbox_width = float(x2 - x1)
                bbox_height = float(y2 - y1)
                detected = True

                # Get depth at bbox center
                if self.latest_depth is not None:
                    try:
                        depth_image = self.bridge.imgmsg_to_cv2(
                            self.latest_depth,
                            desired_encoding='32FC1'
                        )
                        u = int(bbox_center_x)
                        v = int(bbox_center_y)

                        if 0 <= v < depth_image.shape[0] and \
                           0 <= u < depth_image.shape[1]:
                            depth_value = float(depth_image[v, u])

                            # Convert to camera frame
                            cam_coords = self.pixel_to_camera_frame(
                                bbox_center_x,
                                bbox_center_y,
                                depth_value
                            )

                            if cam_coords is not None:
                                camera_coords.x = cam_coords[0]
                                camera_coords.y = cam_coords[1]
                                camera_coords.z = cam_coords[2]

                                # Convert to body frame
                                body_coords = self.camera_to_body_frame(
                                    *cam_coords
                                )

                                # Convert to NED frame
                                ned = self.body_to_ned_frame(*body_coords)

                                if ned is not None:
                                    ned_coords.x = ned[0]
                                    ned_coords.y = ned[1]
                                    ned_coords.z = ned[2]
                                    ned_valid = True

                    except Exception as e:
                        self.get_logger().warn(
                            f'Depth processing error: {e}'
                        )

                # Draw bounding box on image for visualization
                cv2.rectangle(
                    cv_image,
                    (int(x1), int(y1)),
                    (int(x2), int(y2)),
                    (0, 255, 0),
                    2
                )
                label = f'X-marker {confidence:.2f}'
                cv2.putText(
                    cv_image,
                    label,
                    (int(x1), int(y1) - 10),
                    cv2.FONT_HERSHEY_SIMPLEX,
                    0.5,
                    (0, 255, 0),
                    2
                )

                # Add NED coordinates if valid
                if ned_valid:
                    ned_text = (f'NED: ({ned_coords.x:.2f}, '
                                f'{ned_coords.y:.2f}, {ned_coords.z:.2f})')
                    cv2.putText(
                        cv_image,
                        ned_text,
                        (10, 30),
                        cv2.FONT_HERSHEY_SIMPLEX,
                        0.6,
                        (255, 255, 0),
                        2
                    )

            # Publish detection result
            if self.detection_pub is not None:
                from vision_detection.msg import DetectionResult
                detection_msg = DetectionResult()
                detection_msg.header = Header()
                detection_msg.header.stamp = self.get_clock().now().to_msg()
                detection_msg.header.frame_id = 'camera_rgb_optical_frame'
                detection_msg.detected = detected
                detection_msg.confidence = confidence
                detection_msg.bbox_center_x = bbox_center_x
                detection_msg.bbox_center_y = bbox_center_y
                detection_msg.bbox_width = bbox_width
                detection_msg.bbox_height = bbox_height
                detection_msg.camera_coords = camera_coords
                detection_msg.ned_coords = ned_coords
                detection_msg.ned_valid = ned_valid
                detection_msg.depth = depth_value

                self.detection_pub.publish(detection_msg)

            # Publish annotated image
            annotated_msg = self.bridge.cv2_to_imgmsg(cv_image, 'bgr8')
            annotated_msg.header.stamp = self.get_clock().now().to_msg()
            self.annotated_pub.publish(annotated_msg)

        except Exception as e:
            self.get_logger().error(f'Detection error: {e}')


def main(args=None):
    """Run the X-marker detector node."""
    rclpy.init(args=args)
    node = XMarkerDetectorNode()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
