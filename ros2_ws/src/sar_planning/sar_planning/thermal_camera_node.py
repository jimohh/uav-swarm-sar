#!/usr/bin/env python3
"""
thermal_camera_node.py
Simulates a thermal camera feed for SAR detection.
In simulation, generates synthetic thermal images with
randomised heat signatures representing potential victims.
Publishes sensor_msgs/Image on /sar/thermal/image_raw
"""

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from std_msgs.msg import Header
import numpy as np


class ThermalCameraNode(Node):

    def __init__(self):
        super().__init__('thermal_camera_node')

        self.declare_parameter('publish_rate', 10.0)  # Hz
        self.declare_parameter('image_width', 640)
        self.declare_parameter('image_height', 480)
        self.declare_parameter('num_targets', 3)

        self.rate    = self.get_parameter('publish_rate').value
        self.width   = self.get_parameter('image_width').value
        self.height  = self.get_parameter('image_height').value
        self.n_tgts  = self.get_parameter('num_targets').value

        # Publisher
        self.pub = self.create_publisher(
            Image, '/sar/thermal/image_raw', 10)

        # Randomise target positions once
        self.targets = [
            (np.random.randint(50, self.width-50),
             np.random.randint(50, self.height-50))
            for _ in range(self.n_tgts)
        ]

        self.timer = self.create_timer(
            1.0 / self.rate, self._publish_frame)

        self.get_logger().info(
            f'ThermalCameraNode started — {self.n_tgts} synthetic targets')

    def _publish_frame(self):
        # Background: cool scene (low values)
        frame = np.random.randint(20, 60,
            (self.height, self.width), dtype=np.uint8)

        # Add heat signatures at target positions
        for (tx, ty) in self.targets:
            # Add Gaussian heat blob
            for dy in range(-20, 21):
                for dx in range(-20, 21):
                    px, py = tx + dx, ty + dy
                    if 0 <= px < self.width and 0 <= py < self.height:
                        dist = np.sqrt(dx**2 + dy**2)
                        heat = int(180 * np.exp(-dist**2 / (2 * 8**2)))
                        frame[py, px] = min(255,
                            int(frame[py, px]) + heat)

        # Build ROS Image message (mono8)
        msg = Image()
        msg.header = Header()
        msg.header.stamp    = self.get_clock().now().to_msg()
        msg.header.frame_id = 'thermal_camera'
        msg.height   = self.height
        msg.width    = self.width
        msg.encoding = 'mono8'
        msg.step     = self.width
        msg.data     = frame.flatten().tolist()

        self.pub.publish(msg)
        self.get_logger().debug('Published thermal frame')


def main(args=None):
    rclpy.init(args=args)
    node = ThermalCameraNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()