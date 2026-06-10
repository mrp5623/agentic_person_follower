#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image, LaserScan
from vision_msgs.msg import Detection2DArray, Detection2D, ObjectHypothesisWithPose
from cv_bridge import CvBridge
from ultralytics import YOLO
import numpy as np
import math

class PerceptionNode(Node):

    def __init__(self):
        super().__init__('perception_node')

        self.declare_parameters(
            namespace='',
            parameters=[
                ('image_topic', '/oakd/rgb/preview/image_raw'),
                ('scan_topic', '/scan'),
                ('yolo_model', 'yolov8n.pt'),
                ('detection_confidence', 0.5),
                ('lidar_search_angle', 10.0),
            ]
        )

        params = self.get_parameters([
            'image_topic', 'scan_topic', 'yolo_model',
            'detection_confidence', 'lidar_search_angle'
        ])
        image_topic, scan_topic, model_path, \
            self.conf_threshold, self.lidar_search_angle = [p.value for p in params]

        # Camera intrinsics for OAK-D Lite preview stream (250x250)
        self.fx = 194.097
        self.fy = 194.097
        self.cx = 122.441
        self.cy = 124.933

        self.bridge = CvBridge()
        self.model = YOLO(model_path)
        self.latest_scan = None

        self.image_sub = self.create_subscription(
            Image, image_topic, self.image_callback, 10)

        self.scan_sub = self.create_subscription(
            LaserScan, scan_topic, self.scan_callback, 10)

        self.detection_pub = self.create_publisher(
            Detection2DArray, '/person_detections', 10)

        self.get_logger().info('Perception node started')

    def scan_callback(self, msg):
        self.latest_scan = msg

    def image_callback(self, msg):
        if self.latest_scan is None:
            return

        frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        results = self.model(frame, classes=[0], conf=self.conf_threshold, verbose=False)

        detection_array = Detection2DArray()
        detection_array.header = msg.header

        for result in results:
            if len(result.boxes) == 0:
                continue

            # Take only the highest confidence detection
            best_box = max(result.boxes, key=lambda b: float(b.conf[0]))

            x1, y1, x2, y2 = best_box.xyxy[0].tolist()
            cx = (x1 + x2) / 2

            angle_rad = self.compute_heading((cx, (y1 + y2) / 2))

            distance = self.get_lidar_distance(angle_rad)
            if distance is None:
                continue

            detection = Detection2D()
            detection.header = msg.header

            hypothesis = ObjectHypothesisWithPose()
            hypothesis.hypothesis.class_id = 'person'
            hypothesis.hypothesis.score = float(best_box.conf[0])
            detection.results.append(hypothesis)

            detection.bbox.center.position.x = cx
            detection.bbox.center.position.y = (y1 + y2) / 2
            detection.bbox.size_x = float(x2 - x1)
            detection.bbox.size_y = float(y2 - y1)

            detection.bbox.center.theta = angle_rad
            detection.results[0].pose.pose.position.z = float(distance)

            detection_array.detections.append(detection)

        self.detection_pub.publish(detection_array)

    def compute_heading(self, center):
        x_norm = (center[0] - self.cx) / self.fx
        theta_cam = -math.atan(x_norm)
        dx, dy = 0.0635, 0.0381
        x_c = math.cos(theta_cam)
        y_c = math.sin(theta_cam)
        theta_lidar = math.atan2(y_c - dy, x_c - dx)
        return theta_lidar

    def get_lidar_distance(self, angle_rad):
        if self.latest_scan is None:
            return None

        idx = int(math.degrees(angle_rad) * 2 + 270)

        if idx < 0 or idx >= len(self.latest_scan.ranges):
            return None

        start_idx = max(0, idx - 20)
        end_idx = min(len(self.latest_scan.ranges), idx + 20)
        ranges = self.latest_scan.ranges[start_idx:end_idx]

        valid_ranges = [r for r in ranges
                        if not math.isinf(r) and not math.isnan(r)
                        and r >= self.latest_scan.range_min
                        and r <= self.latest_scan.range_max]

        if not valid_ranges:
            return None

        # self.get_logger().info(
        #     f'angle_deg: {math.degrees(angle_rad):.1f}, '
        #     f'idx: {idx}, '
        #     f'distance: {min(valid_ranges):.3f}'
        # )

        return min(valid_ranges)


def main(args=None):
    rclpy.init(args=args)
    node = PerceptionNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()