#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from vision_msgs.msg import Detection2DArray
import numpy as np
import math
import time
from dataclasses import dataclass
from collections import deque
from std_msgs.msg import Float32MultiArray
from ultralytics.trackers.byte_tracker import BYTETracker
from ultralytics.utils import IterableSimpleNamespace
import torch

@dataclass
class AlphaBeta:
    #1-D α-β tracker from English project
    alpha: float = 0.25
    beta: float = 0.25**2 / 2
    x: float = None
    v: float = 0.0
    t: float = None

    def reset(self):
        self.x, self.v, self.t = None, 0.0, None

    def update(self, z: float, t_now: float = None) -> float:
        if t_now is None:
            t_now = time.time()
        if self.x is None or abs(z - self.x) > 2.0:
            self.x, self.v, self.t = z, 0.0, t_now
            return self.x
        dt = max(t_now - self.t, 1e-3)
        x_pred = self.x + self.v * dt
        v_pred = self.v
        r = z - x_pred
        self.x = x_pred + self.alpha * r
        self.v = v_pred + self.beta * r / dt
        self.t = t_now
        return self.x

class TrackerNode(Node):

    def __init__(self):
        super().__init__('tracker_node')

        self.declare_parameter('max_lost_frames', 10)
        self.max_lost_frames = self.get_parameter('max_lost_frames').value

        #BYTETracker upgrade from English SORT
        tracker_args = IterableSimpleNamespace(
            #range for secondary matches
            track_high_thresh=0.5,
            track_low_thresh=0.1,
            #bar for new match
            new_track_thresh=0.5,
            #max age from SORT
            track_buffer=30,
            #mathcing confidence   
            match_thresh=0.8,
            fuse_score=True,
        )
        self.tracker = BYTETracker(tracker_args)

        # Filters — one per measurement type
        self.distance_filter = AlphaBeta(alpha=0.25)

        # Persistent target state
        self.tracked_id = None
        self.frames_without_detection = 0

        # Subscriber
        self.detection_sub = self.create_subscription(
            Detection2DArray,
            '/person_detections',
            self.detection_callback,
            10
        )

        # Publisher — [track_id, filtered_angle, filtered_distance, confidence]
        self.tracked_pub = self.create_publisher(
            Float32MultiArray,
            '/tracked_person',
            10
        )

        self.get_logger().info('Tracker node started')

    def detection_callback(self, msg):
        try:
            out = Float32MultiArray()

            if len(msg.detections) == 0:
                self.frames_without_detection += 1
                if self.frames_without_detection > self.max_lost_frames:
                    self.tracked_id = None
                    self.distance_filter.reset()

                out.data = [-1.0, 0.0, 0.0] #Invalid state
                self.tracked_pub.publish(out)
                return

            self.frames_without_detection = 0

            dets = []
            for detection in msg.detections:
                cx = detection.bbox.center.position.x
                cy = detection.bbox.center.position.y
                w = detection.bbox.size_x
                h = detection.bbox.size_y
                score = detection.results[0].hypothesis.score
                x1 = cx - w / 2
                y1 = cy - h / 2
                x2 = cx + w / 2
                y2 = cy + h / 2
                dets.append([x1, y1, x2, y2, score])

            dets = np.array(dets, dtype=np.float32)

            class MyResults:
                def __init__(self, dets):
                    import torch
                    self.conf = torch.from_numpy(dets[:, 4].astype(np.float32))
                    self.xyxy = torch.from_numpy(dets[:, :4].astype(np.float32))
                    self.cls = torch.zeros(len(dets))
                    self._dets = dets
                    
                    # Convert xyxy to xywh
                    x1, y1, x2, y2 = dets[:, 0], dets[:, 1], dets[:, 2], dets[:, 3]
                    cx = (x1 + x2) / 2
                    cy = (y1 + y2) / 2
                    w = x2 - x1
                    h = y2 - y1
                    xywh = np.stack([cx, cy, w, h], axis=1).astype(np.float32)
                    self.xywh = torch.from_numpy(xywh)

                def __getitem__(self, idx):
                    return MyResults(self._dets[idx.numpy() if hasattr(idx, 'numpy') else idx])

                def __len__(self):
                    return len(self._dets)
                
            #Update tracker
            tracks = self.tracker.update(MyResults(dets), img=np.zeros((250, 250, 3), dtype=np.uint8))

            if len(tracks) == 0:
                out.data = [-1.0, 0.0, 0.0]
                self.tracked_pub.publish(out)
                return

            #Best = prev tracked || biggest
            best = None
            max_area = 0
            for track in tracks:
                x1, y1, x2, y2, track_id = track[:5]
                area = (x2 - x1) * (y2 - y1)
                if self.tracked_id is not None and int(track_id) == int(self.tracked_id):
                    best = track
                    break
                if area > max_area:
                    max_area = area
                    best = track

            if best is None:
                out.data = [-1.0, 0.0, 0.0]
                self.tracked_pub.publish(out)
                return

            self.tracked_id = int(best[4])

            #Match back to the detection closest to the tracked bounding box
            best_cx = (best[0] + best[2]) / 2
            matched_detection = min(
                msg.detections,
                key=lambda d: abs(d.bbox.center.position.x - best_cx)
            )

            #Filter
            raw_angle = matched_detection.bbox.center.theta
            raw_distance = matched_detection.results[0].pose.pose.position.z

            filtered_distance = self.distance_filter.update(raw_distance)

            out.data = [
                float(self.tracked_id),
                float(raw_angle),
                float(filtered_distance),
            ]

            #self.get_logger().info(f'tracked_id: {self.tracked_id}, best_id: {int(best[4])}, angle: {math.degrees(raw_angle):.1f}')
            
            self.tracked_pub.publish(out)
        except Exception as e:
            self.get_logger().error(f'Tracker callback error: {e}')
            import traceback
            self.get_logger().error(traceback.format_exc())

def main(args=None):
    rclpy.init(args=args)
    node = TrackerNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()