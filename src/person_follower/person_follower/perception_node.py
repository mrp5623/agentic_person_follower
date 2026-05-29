#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image, LaserScan
from vision_msgs.msg import Detection2DArray, Detection2D, ObjectHypothesisWithPose
from cv_bridge import CvBridge
from ultralytics import YOLO
import numpy as np

class PerceptionNode(Node):

    def __init__(self):
        super().__init__('perception_node')

        # Load parameters
        self.declare_parameter('image_topic', '/oakd/rgb/image_raw')
        self.declare_parameter('scan_topic', '/scan')
        self.declare_parameter('yolo_model', 'yolov8n.pt')
        self.declare_parameter('detection_confidence', 0.5)
        self.declare_parameter('lidar_search_angle', 10.0)

        image_topic = self.get_parameter('image_topic').value
        scan_topic = self.get_parameter('scan_topic').value
        model_path = self.get_parameter('yolo_model').value
        self.conf_threshold = self.get_parameter('detection_confidence').value
        self.lidar_search_angle = self.get_parameter('lidar_search_angle').value

        self.bridge = CvBridge()

        self.model = YOLO(model_path)
        
        self.latest_scan = None

        self.image_sub = self.create_subscription(
            Image,
            image_topic,
            self.image_callback,
            10
        )

        self.scan_sub = self.create_subscription(
            LaserScan,
            scan_topic,
            self.scan_callback,
            10
        )

        self.detection_pub = self.create_publisher(
            Detection2DArray,
            '/person_detections',
            10
        )

        self.get_logger().info('Perception node started')
    
    def scan_callback(self, msg):
        self.latest_scan = msg
    
    def image_callback(self, msg):
        if self.latest_scan is None:
            return

        frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')

        results = self.model(frame, classes=[0], conf=self.conf_threshold, verbose=False)

        self.get_logger().info(f'YOLO results: {len(results[0].boxes)} detections')

        detection_array = Detection2DArray()
        detection_array.header = msg.header

        for result in results:
            for box in result.boxes:
                # Get bounding box center in pixel coordinates
                x1, y1, x2, y2 = box.xyxy[0].tolist()
                cx = (x1 + x2) / 2
                image_width = frame.shape[1]

                # Convert pixel x position to angle relative to robot forward
                # Positive angle = person is to the left
                angle = ((image_width / 2) - cx) / (image_width / 2) * (69.0 / 2)
                angle_rad = np.deg2rad(angle)

                # Get distance from LIDAR
                distance = self.get_lidar_distance(angle_rad)
                if distance is None:
                    continue

                # Build individual detection message
                detection = Detection2D()
                detection.header = msg.header

                hypothesis = ObjectHypothesisWithPose()
                hypothesis.hypothesis.class_id = 'person'
                hypothesis.hypothesis.score = float(box.conf[0])
                detection.results.append(hypothesis)

                detection.bbox.center.position.x = cx
                detection.bbox.center.position.y = (y1 + y2) / 2
                detection.bbox.size_x = float(x2 - x1)
                detection.bbox.size_y = float(y2 - y1)

                # Store distance and angle as z position and theta
                detection.bbox.center.theta = angle_rad
                detection.results[0].pose.pose.position.z = float(distance)

                detection_array.detections.append(detection)

        self.detection_pub.publish(detection_array)

    def get_lidar_distance(self, angle_rad):
        scan = self.latest_scan

        # Convert search angle to number of LIDAR indices
        angle_increment = scan.angle_increment
        search_indices = int(np.deg2rad(self.lidar_search_angle) / angle_increment)

        # Find the index in the scan array that corresponds to our target angle
        # LIDAR scans start at scan.angle_min and increment by scan.angle_increment
        target_index = int((angle_rad - scan.angle_min) / angle_increment)

        # Define search window around target index
        min_index = max(0, target_index - search_indices)
        max_index = min(len(scan.ranges) - 1, target_index + search_indices)

        # Extract ranges in the search window, filtering out invalid readings
        ranges = np.array(scan.ranges[min_index:max_index])
        valid_ranges = ranges[
            (ranges >= scan.range_min) &
            (ranges <= scan.range_max) &
            np.isfinite(ranges)
        ]

        if len(valid_ranges) == 0:
            return None

        # Return closest valid range
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