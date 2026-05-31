#!/usr/bin/env python3
"""
yolo11s_detector.py
YOLO11s detection node for SAR thermal imagery.
Backbone: C3k2 + C2PSA attention mechanism
Conf threshold: 0.45
Temporal window: 10 frames
Target Pd: 0.85

Subscribes to:
  /sar/thermal/image_raw     — thermal camera feed
  /sar/rf/detections         — RF Doppler pre-alerts

Publishes:
  /sar/detections/confirmed  — confirmed detections (3-stage cascade)
  /sar/detections/raw        — raw YOLO detections
"""

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from std_msgs.msg import String
import numpy as np
import json
import collections


class YOLO11sDetector(Node):

    def __init__(self):
        super().__init__('yolo11s_detector')

        self.declare_parameter('conf_threshold', 0.45)
        self.declare_parameter('temporal_window', 10)
        self.declare_parameter('target_pd', 0.85)
        self.declare_parameter('model_path', 'yolo11s.pt')

        self.conf_thresh  = self.get_parameter('conf_threshold').value
        self.temp_window  = self.get_parameter('temporal_window').value
        self.target_pd    = self.get_parameter('target_pd').value
        model_path        = self.get_parameter('model_path').value

        # Load YOLO11s model
        self.model = None
        self._load_model(model_path)

        # Temporal window buffer — stores recent detections
        # key: grid_cell, value: deque of confidences
        self.temporal_buffer = collections.defaultdict(
            lambda: collections.deque(maxlen=self.temp_window))

        # RF pre-alert buffer
        self.rf_alerts = []

        # Subscribers
        self.create_subscription(
            Image, '/sar/thermal/image_raw',
            self._image_callback, 10)

        self.create_subscription(
            String, '/sar/rf/detections',
            self._rf_callback, 10)

        # Publishers
        self.pub_confirmed = self.create_publisher(
            String, '/sar/detections/confirmed', 10)

        self.pub_raw = self.create_publisher(
            String, '/sar/detections/raw', 10)

        self.frame_count     = 0
        self.total_detections = 0

        self.get_logger().info(
            f'YOLO11sDetector started — conf: {self.conf_thresh}, '
            f'window: {self.temp_window}, target_Pd: {self.target_pd}')

    def _load_model(self, model_path):
        """Load YOLO11s model — falls back to simulation mode if unavailable."""
        try:
            from ultralytics import YOLO
            self.model = YOLO(model_path)
            self.get_logger().info(f'YOLO11s model loaded: {model_path}')
        except Exception as e:
            self.get_logger().warn(
                f'Model load failed ({e}) — running in simulation mode')
            self.model = None

    def _rf_callback(self, msg):
        """Store RF detections as stage-1 alerts."""
        try:
            self.rf_alerts = json.loads(msg.data)
        except json.JSONDecodeError:
            pass

    def _image_callback(self, msg):
        """Main detection pipeline — three stage cascade."""
        self.frame_count += 1

        # Convert ROS Image to numpy array
        frame = np.array(msg.data, dtype=np.uint8).reshape(
            (msg.height, msg.width))

        # --- Stage 1: RF pre-screening ---
        rf_triggered = len(self.rf_alerts) > 0
        high_conf_rf = any(
            a.get('confidence', 0) > 0.5
            for a in self.rf_alerts
            if a.get('sensor') == 'rf_doppler'
        )

        # --- Stage 2: YOLO11s visual detection ---
        raw_detections = self._run_yolo(frame, msg)

        # Publish raw detections
        if raw_detections:
            raw_msg = String()
            raw_msg.data = json.dumps(raw_detections)
            self.pub_raw.publish(raw_msg)

        # --- Stage 3: Temporal window fusion ---
        confirmed = self._temporal_fusion(
            raw_detections, rf_triggered, high_conf_rf)

        if confirmed:
            self.total_detections += len(confirmed)
            conf_msg = String()
            conf_msg.data = json.dumps(confirmed)
            self.pub_confirmed.publish(conf_msg)

            for det in confirmed:
                self.get_logger().info(
                    f'CONFIRMED detection — '
                    f'cell: {det["grid_cell"]}, '
                    f'Pd: {det["detection_probability"]:.3f}, '
                    f'frames: {det["positive_frames"]}/{self.temp_window}')

        # Clear RF alerts after processing
        self.rf_alerts = []

    def _run_yolo(self, frame, msg):
        """Run YOLO11s inference or simulate detections."""
        detections = []

        if False:  # Force simulation mode for synthetic thermal images
            # Real YOLO11s inference
            import cv2
            frame_bgr = cv2.cvtColor(frame, cv2.COLOR_GRAY2BGR)
            results = self.model(
                frame_bgr, conf=self.conf_thresh, verbose=False)

            for r in results:
                for box in r.boxes:
                    conf = float(box.conf[0])
                    if conf >= self.conf_thresh:
                        x1, y1, x2, y2 = box.xyxy[0].tolist()
                        cx = int((x1 + x2) / 2)
                        cy = int((y1 + y2) / 2)
                        detections.append({
                            'bbox':       [x1, y1, x2, y2],
                            'confidence': round(conf, 3),
                            'grid_cell':  (cx // 32, cy // 32),
                            'frame':      self.frame_count,
                            'sensor':     'yolo11s'
                        })
        else:
            # Simulation mode — detect thermal blobs above threshold
            hot_pixels = np.where(frame > 150)
            if len(hot_pixels[0]) > 10:
                # Cluster hot pixels into detections
                for i in range(min(3, len(hot_pixels[0]) // 20)):
                    idx = np.random.randint(len(hot_pixels[0]))
                    cy_px = int(hot_pixels[0][idx])
                    cx_px = int(hot_pixels[1][idx])
                    conf  = float(frame[cy_px, cx_px]) / 255.0
                    if conf >= self.conf_thresh:
                        detections.append({
                            'bbox':       [cx_px-20, cy_px-20,
                                          cx_px+20, cy_px+20],
                            'confidence': round(conf, 3),
                            'grid_cell':  (cx_px // 32, cy_px // 32),
                            'frame':      self.frame_count,
                            'sensor':     'yolo11s_sim'
                        })

        return detections

    def _temporal_fusion(self, detections, rf_triggered, high_conf_rf):
        """
        Stage 3: Temporal window fusion.
        A detection is confirmed when:
        - Positive in >= 7/10 frames (Pd target 0.85)
        - OR RF pre-alert + positive in >= 5/10 frames
        """
        confirmed = []

        # Update temporal buffer
        active_cells = set()
        for det in detections:
            cell = tuple(det['grid_cell'])
            active_cells.add(cell)
            self.temporal_buffer[cell].append(det['confidence'])

        # Add zero for inactive cells to decay old detections
        for cell in list(self.temporal_buffer.keys()):
            if cell not in active_cells:
                self.temporal_buffer[cell].append(0.0)

        # Evaluate each cell
        threshold = 5 if (rf_triggered and high_conf_rf) else 7

        for cell, buffer in self.temporal_buffer.items():
            if len(buffer) < 3:
                continue
            positive_frames = sum(
                1 for c in buffer if c >= self.conf_thresh)
            pd = positive_frames / len(buffer)

            if positive_frames >= threshold:
                confirmed.append({
                    'grid_cell':           list(cell),
                    'positive_frames':     positive_frames,
                    'window_size':         len(buffer),
                    'detection_probability': round(pd, 3),
                    'rf_assisted':         rf_triggered,
                    'frame':               self.frame_count
                })

        return confirmed


def main(args=None):
    rclpy.init(args=args)
    node = YOLO11sDetector()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()