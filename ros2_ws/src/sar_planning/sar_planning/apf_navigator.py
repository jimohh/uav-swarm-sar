#!/usr/bin/env python3
"""
apf_navigator.py
Artificial Potential Field navigator for SAR UAV.
- Attractive force pulls UAV toward current waypoint
- Repulsive force pushes UAV away from obstacles
- Outputs velocity setpoints to MAVROS
- Switches to next waypoint when within arrival threshold
"""

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from geometry_msgs.msg import PoseArray, TwistStamped, PoseStamped
from nav_msgs.msg import Odometry
from sensor_msgs.msg import LaserScan
import numpy as np
import math


class APFNavigator(Node):

    def __init__(self):
        super().__init__('apf_navigator')

        # --- Parameters ---
        self.declare_parameter('uav_id', 0)
        self.declare_parameter('attractive_gain', 1.5)
        self.declare_parameter('repulsive_gain', 2.0)
        self.declare_parameter('repulsive_radius', 5.0)   # metres
        self.declare_parameter('max_speed', 3.0)           # m/s
        self.declare_parameter('arrival_threshold', 2.0)   # metres
        self.declare_parameter('cruise_altitude', 10.0)    # metres

        uav_id      = self.get_parameter('uav_id').value
        ns          = f'uav{uav_id}'
        self.k_att  = self.get_parameter('attractive_gain').value
        self.k_rep  = self.get_parameter('repulsive_gain').value
        self.r_rep  = self.get_parameter('repulsive_radius').value
        self.v_max  = self.get_parameter('max_speed').value
        self.d_arr  = self.get_parameter('arrival_threshold').value
        self.z_cruise = self.get_parameter('cruise_altitude').value

        # State
        self.pose        = None   # current UAV pose (x, y, z)
        self.waypoints   = []     # list of (x, y, z) tuples
        self.wp_index    = 0      # current target waypoint index
        self.obstacles   = []     # list of (x, y) obstacle positions

        # --- Subscribers ---
        sensor_qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=10,
        )
        self.create_subscription(
            Odometry,
            f'/{ns}/mavros/local_position/odom',
            self._odom_callback, sensor_qos)

        self.create_subscription(
            PoseArray,
            '/sar/waypoints',
            self._waypoints_callback, 10)

        self.create_subscription(
            LaserScan,
            f'/{ns}/scan',
            self._scan_callback, 10)

        # --- Publishers ---
        self.pub_vel = self.create_publisher(
            TwistStamped,
            f'/{ns}/mavros/setpoint_velocity/cmd_vel_unstamped',
            10)

        self.pub_setpoint = self.create_publisher(
            PoseStamped,
            f'/{ns}/mavros/setpoint_position/local',
            10)

        # Control loop at 20 Hz
        self.timer = self.create_timer(0.05, self._control_loop)

        self.get_logger().info(f'APFNavigator started for {ns}')

    # ------------------------------------------------------------------
    # Callbacks
    # ------------------------------------------------------------------
    def _odom_callback(self, msg):
        p = msg.pose.pose.position
        self.pose = np.array([p.x, p.y, p.z])

    def _waypoints_callback(self, msg):
        self.waypoints = [
            np.array([p.position.x, p.position.y, self.z_cruise])
            for p in msg.poses
        ]
        self.wp_index = 0
        self.get_logger().info(
            f'Received {len(self.waypoints)} waypoints')

    def _scan_callback(self, msg):
        """Convert LaserScan ranges to obstacle positions in body frame."""
        self.obstacles = []
        if self.pose is None:
            return
        angle = msg.angle_min
        for r in msg.ranges:
            if msg.range_min < r < msg.range_max:
                ox = self.pose[0] + r * math.cos(angle)
                oy = self.pose[1] + r * math.sin(angle)
                self.obstacles.append(np.array([ox, oy]))
            angle += msg.angle_increment

    # ------------------------------------------------------------------
    # APF core
    # ------------------------------------------------------------------
    def _attractive_force(self, pos, goal):
        """Linear attractive potential toward goal."""
        diff = goal[:2] - pos[:2]
        dist = np.linalg.norm(diff)
        if dist < 1e-6:
            return np.zeros(2)
        return self.k_att * diff / dist * min(dist, self.v_max)

    def _repulsive_force(self, pos):
        """Repulsive potential away from all detected obstacles."""
        force = np.zeros(2)
        for obs in self.obstacles:
            diff = pos[:2] - obs[:2]
            dist = np.linalg.norm(diff)
            if 0 < dist < self.r_rep:
                magnitude = self.k_rep * (1.0/dist - 1.0/self.r_rep) / (dist**2)
                force += magnitude * diff / dist
        # Clip repulsive force to avoid instability
        norm = np.linalg.norm(force)
        if norm > self.v_max:
            force = force / norm * self.v_max
        return force

    def _altitude_control(self, current_z, target_z):
        """Simple P controller for altitude."""
        error = target_z - current_z
        return np.clip(1.5 * error, -1.5, 1.5)

    # ------------------------------------------------------------------
    # Main control loop
    # ------------------------------------------------------------------
    def _control_loop(self):
        if self.pose is None:
            return
        if not self.waypoints or self.wp_index >= len(self.waypoints):
            # No waypoints — hover in place
            self._publish_velocity(0.0, 0.0, 0.0)
            return

        goal = self.waypoints[self.wp_index]

        # Check arrival
        dist_to_goal = np.linalg.norm(goal[:2] - self.pose[:2])
        if dist_to_goal < self.d_arr:
            self.get_logger().info(
                f'Reached waypoint {self.wp_index} — '
                f'distance: {dist_to_goal:.2f}m')
            self.wp_index += 1
            if self.wp_index >= len(self.waypoints):
                self.get_logger().info('All waypoints reached')
                self._publish_velocity(0.0, 0.0, 0.0)
                return

        # Compute APF resultant
        f_att = self._attractive_force(self.pose, goal)
        f_rep = self._repulsive_force(self.pose)
        f_total = f_att + f_rep

        # Normalise total force to max speed
        norm = np.linalg.norm(f_total)
        if norm > self.v_max:
            f_total = f_total / norm * self.v_max

        vz = self._altitude_control(self.pose[2], self.z_cruise)

        self._publish_velocity(f_total[0], f_total[1], vz)

    def _publish_velocity(self, vx, vy, vz):
        msg = TwistStamped()
        msg.header.stamp    = self.get_clock().now().to_msg()
        msg.header.frame_id = 'map'
        msg.twist.linear.x  = float(vx)
        msg.twist.linear.y  = float(vy)
        msg.twist.linear.z  = float(vz)
        self.pub_vel.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = APFNavigator()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()