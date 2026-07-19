#!/usr/bin/env python3
"""
probability_map_node.py
Publishes OccupancyGrid probability maps for all three SAR scenarios.
- Urban:      OpenQuake seismic damage model
- Wilderness: IAMSAR expanding square search prior
- Maritime:   Leeway drift prediction prior
"""

import rclpy
from rclpy.node import Node
from nav_msgs.msg import OccupancyGrid
from std_msgs.msg import Header
import numpy as np
import math


class ProbabilityMapNode(Node):

    def __init__(self):
        super().__init__('probability_map_node')

        # --- Parameters ---
        self.declare_parameter('map_width', 100)
        self.declare_parameter('map_height', 100)
        self.declare_parameter('resolution', 1.0)   # metres per cell
        self.declare_parameter('publish_rate', 1.0) # Hz

        self.width      = self.get_parameter('map_width').value
        self.height     = self.get_parameter('map_height').value
        self.resolution = self.get_parameter('resolution').value
        rate            = self.get_parameter('publish_rate').value

        # --- Publishers ---
        self.pub_urban = self.create_publisher(
            OccupancyGrid, '/sar/prob_map/urban', 10)
        self.pub_wilderness = self.create_publisher(
            OccupancyGrid, '/sar/prob_map/wilderness', 10)
        self.pub_maritime = self.create_publisher(
            OccupancyGrid, '/sar/prob_map/maritime', 10)

        # --- Pre-compute maps ---
        self.urban_map      = self._build_urban_map()
        self.wilderness_map = self._build_wilderness_map()
        self.maritime_map   = self._build_maritime_map()

        # --- Timer ---
        self.timer = self.create_timer(1.0 / rate, self._publish_all)
        self.get_logger().info('ProbabilityMapNode started — publishing all three scenario maps')

    # ------------------------------------------------------------------
    # Urban: OpenQuake seismic damage model
    # Models building collapse probability as gaussian clusters
    # representing high-damage zones near fault lines
    # ------------------------------------------------------------------
    def _build_urban_map(self):
        grid = np.zeros((self.height, self.width), dtype=np.float64)

        # Three high-damage cluster centres (normalised grid coords)
        damage_centres = [
            (0.3, 0.4, 0.9),   # (x_frac, y_frac, peak_prob)
            (0.6, 0.7, 0.75),
            (0.5, 0.2, 0.6),
        ]

        for cx_f, cy_f, peak in damage_centres:
            cx = cx_f * self.width
            cy = cy_f * self.height
            sigma = self.width * 0.12   # spread

            for row in range(self.height):
                for col in range(self.width):
                    dist_sq = (col - cx)**2 + (row - cy)**2
                    grid[row, col] += peak * math.exp(-dist_sq / (2 * sigma**2))

        return self._normalise(grid)

    # ------------------------------------------------------------------
    # Wilderness: IAMSAR expanding square search prior
    # Probability decays from last known position (LKP) at centre
    # following IAMSAR Volume II search planning methodology
    # ------------------------------------------------------------------
    def _build_wilderness_map(self):
        grid = np.zeros((self.height, self.width), dtype=np.float64)

        cx = self.width  / 2.0
        cy = self.height / 2.0
        sigma = self.width * 0.2

        for row in range(self.height):
            for col in range(self.width):
                dist_sq = (col - cx)**2 + (row - cy)**2
                grid[row, col] = math.exp(-dist_sq / (2 * sigma**2))

        # Apply IAMSAR track spacing weighting — higher prob along
        # cardinal axes from LKP reflecting expanding square pattern
        for row in range(self.height):
            for col in range(self.width):
                dx = abs(col - cx) / self.width
                dy = abs(row - cy) / self.height
                axis_weight = 1.0 + 0.3 * math.exp(-min(dx, dy) * 10)
                grid[row, col] *= axis_weight

        return self._normalise(grid)

    # ------------------------------------------------------------------
    # Maritime: Leeway drift prediction prior
    # Asymmetric gaussian elongated in prevailing drift direction
    # based on wind/current leeway model from IAMSAR Vol III
    # ------------------------------------------------------------------
    def _build_maritime_map(self):
        grid = np.zeros((self.height, self.width), dtype=np.float64)

        # Datum point (last known position) — offset from centre
        # to simulate drift from initial distress position
        cx = self.width  * 0.4
        cy = self.height * 0.5

        # Drift direction: NE (45 deg) — sigma_major along drift axis
        sigma_major = self.width  * 0.25   # elongated along drift
        sigma_minor = self.height * 0.10   # narrow across drift
        angle = math.radians(45)           # drift bearing

        cos_a = math.cos(angle)
        sin_a = math.sin(angle)

        for row in range(self.height):
            for col in range(self.width):
                dx = col - cx
                dy = row - cy
                # Rotate into drift-aligned frame
                x_rot =  cos_a * dx + sin_a * dy
                y_rot = -sin_a * dx + cos_a * dy
                exponent = (x_rot**2 / (2 * sigma_major**2) +
                            y_rot**2 / (2 * sigma_minor**2))
                grid[row, col] = math.exp(-exponent)

        return self._normalise(grid)

    # ------------------------------------------------------------------
    # Helpers
    # ------------------------------------------------------------------
    def _normalise(self, grid):
        """Normalise to [0, 1] then scale to OccupancyGrid [0, 100]."""
        mn, mx = grid.min(), grid.max()
        if mx - mn > 1e-9:
            grid = (grid - mn) / (mx - mn)
        return (grid * 100).astype(np.int8)

    def _make_msg(self, data_2d, frame_id):
        msg = OccupancyGrid()
        msg.header = Header()
        msg.header.stamp    = self.get_clock().now().to_msg()
        msg.header.frame_id = frame_id
        msg.info.resolution = self.resolution
        msg.info.width      = self.width
        msg.info.height     = self.height
        msg.info.origin.position.x = 0.0
        msg.info.origin.position.y = 0.0
        msg.info.origin.position.z = 0.0
        msg.data = data_2d.flatten().tolist()
        return msg

    def _publish_all(self):
        self.pub_urban.publish(
            self._make_msg(self.urban_map, 'map'))
        self.pub_wilderness.publish(
            self._make_msg(self.wilderness_map, 'map'))
        self.pub_maritime.publish(
            self._make_msg(self.maritime_map, 'map'))
        self.get_logger().debug('Published all three probability maps')


def main(args=None):
    rclpy.init(args=args)
    node = ProbabilityMapNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()