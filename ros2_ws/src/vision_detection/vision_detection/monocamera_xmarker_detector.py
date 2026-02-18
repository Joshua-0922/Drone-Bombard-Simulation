#!/usr/bin/env python3
"""X-Marker Detection Node using YOLOv8 (Mono Camera Version)."""

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy

from sensor_msgs.msg import Image, CameraInfo
from geometry_msgs.msg import Point
from std_msgs.msg import Header, String  # <--- String 추가됨
from px4_msgs.msg import VehicleLocalPosition

from cv_bridge import CvBridge
import cv2
import numpy as np
import math


class XMarkerDetectorNode(Node):
    """ROS 2 node for X-marker detection using YOLOv8 and Altitude estimation."""

    def __init__(self):
        super().__init__('xmarker_detector')

        # 1. 파라미터 설정
        if not self.has_parameter('model_path'):
            self.declare_parameter('model_path',
                                   '/workspace/ros2_ws/yolo_workspace/runs/train/'
                                   'drone_bombard_train2/weights/best.pt')
        if not self.has_parameter('inference_rate'):
            self.declare_parameter('inference_rate', 10.0)  # Hz

        model_path = self.get_parameter('model_path').value
        inference_rate = self.get_parameter('inference_rate').value

        self.bridge = CvBridge()

        # 상태 변수 초기화
        self.latest_rgb = None
        self.camera_matrix = None
        self.fx = None
        self.fy = None
        self.cx = None
        self.cy = None
        self.vehicle_position = None
        self.vehicle_heading = 0.0

        # 2. YOLO 모델 로드
        self.get_logger().info(f'Loading YOLO model from: {model_path}')
        try:
            from ultralytics import YOLO
            self.model = YOLO(model_path)
            self.get_logger().info('YOLO model loaded successfully')
        except Exception as e:
            self.get_logger().error(f'Failed to load YOLO model: {e}')
            self.model = None

        # QoS 프로필 설정
        qos_profile = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE,
            history=HistoryPolicy.KEEP_LAST,
            depth=1
        )

        qos_reliable = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            history=HistoryPolicy.KEEP_LAST,
            depth=10
        )

        # 3. Subscriber 설정
        self.rgb_sub = self.create_subscription(
            Image,
            '/down_camera_sensor/image_raw',  # 실제 토픽 이름 확인 필요!
            self.rgb_callback,
            qos_profile
        )

        self.camera_info_sub = self.create_subscription(
            CameraInfo,
            '/down_camera_sensor/camera_info',
            self.camera_info_callback,
            qos_profile
        )

        self.vehicle_pos_sub = self.create_subscription(
            VehicleLocalPosition,
            '/fmu/out/vehicle_local_position',
            self.vehicle_position_callback,
            qos_profile
        )

        # 4. Publisher 설정
        try:
            from vision_detection.msg import DetectionResult
            self.detection_pub = self.create_publisher(
                DetectionResult,
                '/vision/detections',
                qos_reliable
            )
        except ImportError:
            self.get_logger().warn('DetectionResult msg not found. Build package first.')
            self.detection_pub = None

        self.annotated_pub = self.create_publisher(
            Image,
            '/vision/annotated_image',
            qos_reliable
        )

        self.pixel_coords_pub = self.create_publisher(
            Point,
            '/target/pixel_coords',
            qos_reliable
        )

        # [추가됨] 비전 상태 퍼블리셔
        self.vision_status_pub = self.create_publisher(
            String, 
            '/vision/status', # /mission/state와 헷갈리지 않게 이름 변경
            qos_reliable
        )

        # 타이머 설정
        timer_period = 1.0 / inference_rate
        self.timer = self.create_timer(timer_period, self.detect_callback)

        self.get_logger().info(f'X-Marker Detector (Mono) initialized at {inference_rate} Hz')

    def rgb_callback(self, msg):
        self.latest_rgb = msg
        # [디버깅] 이미지가 들어오고 있는지 확인 (너무 자주 뜨면 주석 처리)
        # self.get_logger().debug('Image received') 

    def camera_info_callback(self, msg):
        if self.camera_matrix is None:
            self.fx = msg.k[0]
            self.fy = msg.k[4]
            self.cx = msg.k[2]
            self.cy = msg.k[5]
            self.camera_matrix = np.array(msg.k).reshape(3, 3)
            self.get_logger().info(f'Camera intrinsics loaded: fx={self.fx:.2f}')

    def vehicle_position_callback(self, msg):
        self.vehicle_position = msg
        self.vehicle_heading = msg.heading

    def pixel_to_camera_frame(self, u, v, depth):
        if self.fx is None or depth <= 0:
            return None
        X_cam = (u - self.cx) * depth / self.fx
        Y_cam = (v - self.cy) * depth / self.fy
        Z_cam = depth 
        return (X_cam, Y_cam, Z_cam)

    def camera_to_body_frame(self, X_cam, Y_cam, Z_cam):
        X_body = -Z_cam + 0.108
        Y_body = Y_cam
        Z_body = X_cam - 0.01
        return (X_body, Y_body, Z_body)

    def body_to_ned_frame(self, X_body, Y_body, Z_body):
        if self.vehicle_position is None:
            return None
        vehicle_x = self.vehicle_position.x
        vehicle_y = self.vehicle_position.y
        vehicle_z = self.vehicle_position.z
        cos_yaw = math.cos(self.vehicle_heading)
        sin_yaw = math.sin(self.vehicle_heading)
        X_ned = vehicle_x + (X_body * cos_yaw - Y_body * sin_yaw)
        Y_ned = vehicle_y + (X_body * sin_yaw + Y_body * cos_yaw)
        Z_ned = vehicle_z + Z_body
        return (X_ned, Y_ned, Z_ned)

    def detect_callback(self):
        # [디버깅] 이미지가 없는 경우 로그 출력 (5초에 한 번만 출력)
        if self.latest_rgb is None:
            self.get_logger().warn('Waiting for image...', throttle_duration_sec=5.0)
            return
        
        if self.model is None:
            return

        try:
            cv_image = self.bridge.imgmsg_to_cv2(self.latest_rgb, 'bgr8')
            results = self.model(cv_image, verbose=False)
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

            # --- 상태 메시지 준비 ---
            status_msg = String()

            if len(detections) > 0:
                best_idx = detections.conf.argmax()
                box = detections.xyxy[best_idx].cpu().numpy()
                confidence = float(detections.conf[best_idx].cpu().numpy())

                # [디버깅] 감지 성공 로그 (1초에 한 번만 출력)
                self.get_logger().info(f'Target Found! Conf: {confidence:.2f}', throttle_duration_sec=1.0)

                x1, y1, x2, y2 = box
                bbox_center_x = float((x1 + x2) / 2)
                bbox_center_y = float((y1 + y2) / 2)
                bbox_width = float(x2 - x1)
                bbox_height = float(y2 - y1)
                detected = True

                # 상태 업데이트: 타겟 찾음
                status_msg.data = "TARGET_LOCKED"

                # ... (좌표 변환 로직은 동일) ...
                if self.vehicle_position is not None:
                    current_altitude = -self.vehicle_position.z
                    if current_altitude > 0.1:
                        depth_value = current_altitude
                        cam_coords = self.pixel_to_camera_frame(bbox_center_x, bbox_center_y, depth_value)
                        if cam_coords is not None:
                            camera_coords.x = cam_coords[0]
                            camera_coords.y = cam_coords[1]
                            camera_coords.z = cam_coords[2]
                            body_coords = self.camera_to_body_frame(*cam_coords)
                            ned = self.body_to_ned_frame(*body_coords)
                            if ned is not None:
                                ned_coords.x = ned[0]
                                ned_coords.y = ned[1]
                                ned_coords.z = ned[2]
                                ned_valid = True
                    else:
                        depth_value = 0.0
                else:
                    self.get_logger().warning('Vehicle pos missing', throttle_duration_sec=2.0)

                # 시각화 그리기
                cv2.rectangle(cv_image, (int(x1), int(y1)), (int(x2), int(y2)), (0, 255, 0), 2)
                label = f'X-marker {confidence:.2f}'
                cv2.putText(cv_image, label, (int(x1), int(y1)-10), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 2)

            else:
                # [디버깅] 감지 실패 로그 (2초에 한 번만 출력 - 너무 시끄럽지 않게)
                self.get_logger().info('Searching... (No Target)', throttle_duration_sec=2.0)
                
                # 상태 업데이트: 찾는 중
                status_msg.data = "SEARCHING"

            # --- 상태 메시지 발행 ---
            self.vision_status_pub.publish(status_msg)

            # 결과 Publish
            if self.detection_pub is not None:
                from vision_detection.msg import DetectionResult
                detection_msg = DetectionResult()
                detection_msg.header = Header()
                detection_msg.header.stamp = self.get_clock().now().to_msg()
                detection_msg.header.frame_id = 'camera_link'
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

            if detected:
                pixel_coords_msg = Point()
                pixel_coords_msg.x = float(bbox_center_x)
                pixel_coords_msg.y = float(bbox_center_y)
                pixel_coords_msg.z = float(confidence)
                self.pixel_coords_pub.publish(pixel_coords_msg)

            annotated_msg = self.bridge.cv2_to_imgmsg(cv_image, 'bgr8')
            annotated_msg.header.stamp = self.get_clock().now().to_msg()
            self.annotated_pub.publish(annotated_msg)

        except Exception as e:
            self.get_logger().error(f'Detection error: {e}')


def main(args=None):
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