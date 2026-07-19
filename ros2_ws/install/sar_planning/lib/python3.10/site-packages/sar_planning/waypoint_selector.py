#!/usr/bin/env python3
"""
waypoint_selector.py
Reads OccupancyGrid probability maps and publishes the top-N
highest-probability waypoints as navigation targets.
"""

import rclpy
from rclpy.node import Node
from nav_msgs.msg import OccupancyGrid
from geometry_msgs.msg import PoseArray, Pose
import numpy as np


class WaypointSelector(Node):

    def __init__(self):
        super().__init__('waypoint_selector')

        # Parameters
        self.declare_parameter('scenario', 'urban')
        self.declare_parameter('top_n', 5)
        self.declare_parameter('resolution', 1.0)

        self.scenario   = self.get_parameter('scenario').value
        self.top_n      = self.get_parameter('top_n').value
        self.resolution = self.get_parameter('resolution').value

        # Subscribe to the active scenario map
        topic = f'/sar/prob_map/{self.scenario}'
        self.sub = self.create_subscription(
            OccupancyGrid, topic, self._map_callback, 10)

        # Publish selected waypoints
        self.pub_waypoints = self.create_publisher(
            PoseArray, '/sar/waypoints', 10)

        self.get_logger().info(
            f'WaypointSelector ready — scenario: {self.scenario}, '
            f'top_n: {self.top_n}')

    def _map_callback(self, msg):
        width  = msg.info.width
        height = msg.info.height
        data   = np.array(msg.data, dtype=np.int32).reshape((height, width))

        # Get flat indices of top-N cells
        flat_indices = np.argpartition(
            data.flatten(), -self.top_n)[-self.top_n:]
        flat_indices = flat_indices[
            np.argsort(data.flatten()[flat_indices])[::-1]]

        pose_array = PoseArray()
        pose_array.header.stamp    = self.get_clock().now().to_msg()
        pose_array.header.frame_id = 'map'

        for idx in flat_indices:
            row = idx // width
            col = idx  % width
            p = Pose()
            p.position.x = float(col) * self.resolution
            p.position.y = float(row) * self.resolution
            p.position.z = 10.0
            pose_array.poses.append(p)

        self.pub_waypoints.publish(pose_array)
        self.get_logger().debug(
            f'Published {self.top_n} waypoints for {self.scenario}')


def main(args=None):
    rclpy.init(args=args)
    node = WaypointSelector()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()