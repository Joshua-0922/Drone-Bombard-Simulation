#!/usr/bin/env python3
"""X-Marker Detection Node using YOLOv8 (Mono Camera Version).

This node subscribes to MONO camera images, runs YOLOv8 inference,
and calculates 3D coordinates using the drone's altitude (Barometer/GPS)
instead of a depth camera.
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
        # self.latest_depth = None  <-- 삭제됨 (Mono 카메라 사용)
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

        # 3. Subscriber 설정 (토픽 이름 수정됨)
        # Depth 관련 Subscriber는 삭제했습니다.
        
        self.rgb_sub = self.create_subscription(
            Image,
            '/down_camera_sensor/image_raw',  # <--- 사용자의 실제 토픽 이름 반영
            self.rgb_callback,
            qos_profile
        )

        self.camera_info_sub = self.create_subscription(
            CameraInfo,
            '/down_camera_sensor/camera_info', # <--- Camera Info도 네임스페이스 맞춤
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

        # 타이머 설정
        timer_period = 1.0 / inference_rate
        self.timer = self.create_timer(timer_period, self.detect_callback)

        self.get_logger().info(f'X-Marker Detector (Mono) initialized at {inference_rate} Hz')

    def rgb_callback(self, msg):
        self.latest_rgb = msg

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
        
        # 핀홀 카메라 모델 역투영
        X_cam = (u - self.cx) * depth / self.fx
        Y_cam = (v - self.cy) * depth / self.fy
        Z_cam = depth  # 카메라 좌표계에서 Z는 '앞쪽(깊이)'을 의미

        return (X_cam, Y_cam, Z_cam)

    def camera_to_body_frame(self, X_cam, Y_cam, Z_cam):
        # 카메라 마운트: [0.108, 0, -0.01], Pitch -90도 (아래를 봄)
        # 좌표 변환:
        # Cam Z (앞) -> Body -Z (아래가 아니라 Body 기준 수직 아래...가 아니라)
        # Pitch -90도 회전 행렬 적용 시:
        # Body X (전방) = Cam -Z (카메라 위쪽...?) -> 
        # 직관적 변환:
        # 카메라가 아래를 보고 있음.
        # 카메라의 "오른쪽(X)" -> Body "오른쪽(Y)"
        # 카메라의 "아래쪽(Y)" -> Body "뒤쪽(-X)"
        # 카메라의 "앞쪽(Z, 깊이)" -> Body "아래쪽(Z)"
        
        # 다만 기존 코드의 변환식을 유지합니다 (검증된 값이라 가정)
        # 기존: X_body = -Z_cam + offset, Y_body = Y_cam, Z_body = X_cam - offset
        
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
        if self.latest_rgb is None or self.model is None:
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

            if len(detections) > 0:
                best_idx = detections.conf.argmax()
                box = detections.xyxy[best_idx].cpu().numpy()
                confidence = float(detections.conf[best_idx].cpu().numpy())

                x1, y1, x2, y2 = box
                bbox_center_x = float((x1 + x2) / 2)
                bbox_center_y = float((y1 + y2) / 2)
                bbox_width = float(x2 - x1)
                bbox_height = float(y2 - y1)
                detected = True

                # ==========================================================
                # [수정됨] Depth 이미지가 아닌 드론의 고도(Altitude) 사용
                # ==========================================================
                if self.vehicle_position is not None:
                    # NED 좌표계에서 Z는 아래쪽이 양수. 하늘 위는 음수.
                    # 드론이 공중에 떠있으면 z는 -5.0m 등의 음수 값을 가짐.
                    # 따라서 바닥(0)까지의 거리는 abs(z) 혹은 -z 가 됨.
                    
                    # 지면이 평평(0m)하다고 가정 시:
                    current_altitude = -self.vehicle_position.z
                    
                    # 고도가 너무 낮으면(착륙 상태 등) 계산 방지 (0으로 나누기 방지)
                    if current_altitude > 0.1:
                        depth_value = current_altitude
                        
                        # 카메라 좌표로 변환
                        cam_coords = self.pixel_to_camera_frame(
                            bbox_center_x,
                            bbox_center_y,
                            depth_value
                        )

                        if cam_coords is not None:
                            camera_coords.x = cam_coords[0]
                            camera_coords.y = cam_coords[1]
                            camera_coords.z = cam_coords[2]

                            # Body Frame 변환
                            body_coords = self.camera_to_body_frame(*cam_coords)

                            # NED Frame 변환
                            ned = self.body_to_ned_frame(*body_coords)

                            if ned is not None:
                                ned_coords.x = ned[0]
                                ned_coords.y = ned[1]
                                ned_coords.z = ned[2]
                                ned_valid = True
                    else:
                        # 고도가 너무 낮음
                        depth_value = 0.0
                else:
                    self.get_logger().warning('Vehicle position not received yet!', throttle_duration_sec=2.0)

                # 시각화 (Bounding Box 그리기)
                cv2.rectangle(cv_image, (int(x1), int(y1)), (int(x2), int(y2)), (0, 255, 0), 2)
                label = f'X-marker {confidence:.2f}'
                cv2.putText(cv_image, label, (int(x1), int(y1)-10),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 2)

                if ned_valid:
                    ned_text = f'NED: ({ned_coords.x:.2f}, {ned_coords.y:.2f})'
                    cv2.putText(cv_image, ned_text, (10, 30),
                                cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 0), 2)
                    
                    # 거리(고도) 표시 추가
                    alt_text = f'Alt: {depth_value:.2f}m'
                    cv2.putText(cv_image, alt_text, (10, 60),
                                cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 255), 2)

            # 결과 Publish
            if self.detection_pub is not None:
                from vision_detection.msg import DetectionResult
                detection_msg = DetectionResult()
                detection_msg.header = Header()
                detection_msg.header.stamp = self.get_clock().now().to_msg()
                detection_msg.header.frame_id = 'camera_link' # 적절한 프레임 ID
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
                pixel_coords_msg.z = 0.0
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
