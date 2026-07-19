#!/usr/bin/env python3
"""
quad_bridge.py — Quadrotor UAV Arming Bridge
=============================================
Handles arming and OFFBOARD mode switching for UAV0 and UAV1.
Mirrors plane_bridge.py's proven self-arming pattern:
  1. Streams setpoints at 20Hz continuously
  2. After 10 setpoints accumulated, requests arm + OFFBOARD
  3. Retries every 3 seconds until both succeed

This bypasses the external ros2 service call approach which
suffered from timing/connection issues.
"""

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy

from geometry_msgs.msg import Twist, PoseStamped
from std_msgs.msg import Bool

try:
    from mavros_msgs.msg import State
    from mavros_msgs.srv import CommandBool, SetMode
    MAVROS_AVAILABLE = True
except ImportError:
    MAVROS_AVAILABLE = False


class QuadBridge(Node):
    """
    Minimal arming bridge for quadrotor UAVs (UAV0, UAV1).
    Streams zero-velocity setpoints until MAVROS is ready,
    then arms and switches to OFFBOARD mode.
    """

    def __init__(self):
        super().__init__('quad_bridge')

        self.declare_parameter('uav_id', 0)
        self.declare_parameter('offboard_rate', 20.0)

        uav_id = self.get_parameter('uav_id').value
        ns = f'/uav{uav_id}'
        rate = self.get_parameter('offboard_rate').value

        sensor_qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST, depth=1)
        reliable_qos = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            history=HistoryPolicy.KEEP_LAST, depth=10)

        # Subscribe to state and pose
        self.px4_state = None
        self.pose = None
        self._offboard_sent = 0
        self._armed = False
        self._offboard_active = False

        if MAVROS_AVAILABLE:
            self.sub_state = self.create_subscription(
                State, f'{ns}/mavros/state',
                self._cb_state, reliable_qos)

        self.sub_pose = self.create_subscription(
            PoseStamped, f'{ns}/mavros/local_position/pose',
            self._cb_pose, sensor_qos)

        # Publisher for setpoints (keeps OFFBOARD mode alive)
        self.pub_vel = self.create_publisher(
            Twist,
            f'{ns}/mavros/setpoint_velocity/cmd_vel_unstamped',
            reliable_qos)

        # Heartbeat
        self.pub_hb = self.create_publisher(
            Bool, f'{ns}/heartbeat', reliable_qos)

        # Service clients
        if MAVROS_AVAILABLE:
            self.arm_client = self.create_client(
                CommandBool, f'{ns}/mavros/cmd/arming')
            self.mode_client = self.create_client(
                SetMode, f'{ns}/mavros/set_mode')

        # Timers
        self.sp_timer = self.create_timer(
            1.0 / rate, self._setpoint_loop)
        self.arm_timer = self.create_timer(3.0, self._arm_and_offboard)
        self.hb_timer = self.create_timer(1.0, self._heartbeat_loop)

        self.get_logger().info(
            f'QuadBridge started — UAV{uav_id}')

    def _cb_state(self, msg):
        self.px4_state = msg

    def _cb_pose(self, msg):
        self.pose = msg

    def _setpoint_loop(self):
        """Stream zero-velocity setpoints to satisfy PX4's OFFBOARD requirement."""
        t = Twist()
        self.pub_vel.publish(t)
        self._offboard_sent += 1

    def _arm_and_offboard(self):
        """Mirror of plane_bridge's proven arming sequence."""
        if not MAVROS_AVAILABLE:
            return
        if self.pose is None:
            return
        if self._offboard_sent < 10:
            return  # wait for setpoints to accumulate

        if self.px4_state is None:
            return

        if not self.px4_state.armed and not self._armed:
            if not self.arm_client.service_is_ready():
                return
            req = CommandBool.Request()
            req.value = True
            self.arm_client.call_async(req)
            self._armed = True
            self.get_logger().info('QuadBridge: arming requested')

        if self.px4_state.mode != 'OFFBOARD':
            if not self.mode_client.service_is_ready():
                return
            req = SetMode.Request()
            req.custom_mode = 'OFFBOARD'
            self.mode_client.call_async(req)
            self.get_logger().info('QuadBridge: OFFBOARD requested')

        if (self.px4_state.armed and
                self.px4_state.mode == 'OFFBOARD'):
            self.arm_timer.cancel()
            self.get_logger().info('QuadBridge: armed and OFFBOARD ✓')

    def _heartbeat_loop(self):
        hb = Bool()
        hb.data = True
        self.pub_hb.publish(hb)


def main(args=None):
    rclpy.init(args=args)
    node = QuadBridge()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()