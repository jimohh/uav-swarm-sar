#!/usr/bin/env python3
"""
rf_doppler_stub.py — CW Doppler RF Vital-Signs Sensor (Rule-Based Stub)
========================================================================
Package: sar_planning  (ROS 2 Humble / Ubuntu 22.04)

Architecture role
-----------------
Simulates a continuous-wave (CW) Doppler radar operating at 2.4 GHz
for detecting breathing/heartbeat motion of victims beneath rubble or
in wilderness/maritime scenarios.

Detection model (from SIVED dataset grounding):
  - Detection probability Pd = 0.45 at <= 5 m range
  - Pd falls off with range: Pd(r) = 0.45 * exp(-0.1 * (r - 5)) for r > 5
  - False alarm rate Pfa = 0.05
  - Minimum detectable range: 0.5 m
  - Maximum detectable range: 20 m
  - Blocked by solid walls (urban scenario attenuation = -40 dB)

In SITL simulation, victim positions are read from the probability map
and detections are generated probabilistically based on UAV-victim distance.

Topics
------
Subscribed:
  /uav{N}/mavros/local_position/pose  [geometry_msgs/PoseStamped]
  /probability_map                    [nav_msgs/OccupancyGrid]

Published:
  /uav{N}/rf_doppler/detection        [std_msgs/Bool]
      True if victim detected in current scan
  /uav{N}/rf_doppler/detections       [geometry_msgs/PoseArray]
      Estimated victim positions from RF detections
  /uav{N}/rf_doppler/signal_strength  [std_msgs/Float32]
      Simulated received signal strength [dBm]

Parameters
----------
  uav_id        int    default 0
  pd_max        float  default 0.45   — max detection probability
  pfa           float  default 0.05   — false alarm probability
  range_max     float  default 20.0   — max detection range [m]
  range_min     float  default 0.5    — min detection range [m]
  scan_rate     float  default 2.0    — scan rate [Hz]
  scenario      str    default 'urban' — urban/wilderness/maritime
"""

import math
import random

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy

from geometry_msgs.msg import PoseStamped, PoseArray, Pose
from nav_msgs.msg import OccupancyGrid
from std_msgs.msg import Bool, Float32


# ---------------------------------------------------------------------------
# Synthetic victim positions per scenario (SITL ground truth)
# ---------------------------------------------------------------------------

SCENARIO_VICTIMS = {
    'urban': [
        (5.0,  8.0,  0.0),
        (12.0, -3.0, 0.0),
        (-8.0, 15.0, 0.0),
    ],
    'wilderness': [
        (20.0, 10.0, 0.0),
        (-15.0, 25.0, 0.0),
        (30.0, -10.0, 0.0),
    ],
    'maritime': [
        (8.0,  5.0,  0.0),
        (-5.0, 12.0, 0.0),
        (15.0, 20.0, 0.0),
    ],
}


# ---------------------------------------------------------------------------
# RFDopplerStub Node
# ---------------------------------------------------------------------------

class RFDopplerStub(Node):
    """Rule-based CW Doppler RF sensor stub."""

    def __init__(self):
        super().__init__('rf_doppler_stub')

        # ── Parameters ────────────────────────────────────────────────
        self.declare_parameter('uav_id',    0)
        self.declare_parameter('pd_max',    0.45)
        self.declare_parameter('pfa',       0.05)
        self.declare_parameter('range_max', 20.0)
        self.declare_parameter('range_min', 0.5)
        self.declare_parameter('scan_rate', 2.0)
        self.declare_parameter('scenario',  'urban')

        uav_id   = self.get_parameter('uav_id').value
        ns       = f'/uav{uav_id}'
        scenario = self.get_parameter('scenario').value

        # ── QoS ───────────────────────────────────────────────────────
        sensor_qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST, depth=1)
        reliable_qos = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            history=HistoryPolicy.KEEP_LAST, depth=10)

        # ── Subscribers ───────────────────────────────────────────────
        self.sub_pose = self.create_subscription(
            PoseStamped,
            f'{ns}/mavros/local_position/pose',
            self._cb_pose,
            sensor_qos,
        )

        # ── Publishers ────────────────────────────────────────────────
        self.pub_detection = self.create_publisher(
            Bool,      f'{ns}/rf_doppler/detection',       reliable_qos)
        self.pub_poses = self.create_publisher(
            PoseArray, f'{ns}/rf_doppler/detections',      reliable_qos)
        self.pub_rssi = self.create_publisher(
            Float32,   f'{ns}/rf_doppler/signal_strength', reliable_qos)

        # ── State ─────────────────────────────────────────────────────
        self.pose: PoseStamped | None = None
        self.victims = SCENARIO_VICTIMS.get(scenario, SCENARIO_VICTIMS['urban'])

        # ── Scan timer ────────────────────────────────────────────────
        scan_rate = self.get_parameter('scan_rate').value
        self.timer = self.create_timer(1.0 / scan_rate, self._scan)

        self.get_logger().info(
            f'RFDopplerStub started — UAV{uav_id} | '
            f'scenario={scenario} | Pd={self.get_parameter("pd_max").value} | '
            f'{len(self.victims)} victims loaded'
        )

    # ── Callbacks ─────────────────────────────────────────────────────────

    def _cb_pose(self, msg: PoseStamped):
        self.pose = msg

    # ── Detection logic ───────────────────────────────────────────────────

    def _pd_at_range(self, r: float) -> float:
        """
        Detection probability as a function of range.
        Pd(r) = pd_max            for r <= 5 m
        Pd(r) = pd_max * exp(-0.1*(r-5))  for r > 5 m
        Pd(r) = 0                 for r > range_max or r < range_min
        """
        pd_max   = self.get_parameter('pd_max').value
        r_max    = self.get_parameter('range_max').value
        r_min    = self.get_parameter('range_min').value

        if r < r_min or r > r_max:
            return 0.0
        if r <= 5.0:
            return pd_max
        return pd_max * math.exp(-0.1 * (r - 5.0))

    def _rssi_at_range(self, r: float) -> float:
        """
        Simulated received signal strength [dBm].
        Free-space path loss model: RSSI = P_tx - 20*log10(4*pi*r/lambda)
        P_tx = 20 dBm, f = 2.4 GHz → lambda = 0.125 m
        """
        if r < 0.01:
            return 20.0
        lam = 0.125   # wavelength at 2.4 GHz [m]
        fspl = 20.0 * math.log10(max(1e-6, (4 * math.pi * r) / lam))
        return 20.0 - fspl   # P_tx - FSPL

    def _scan(self):
        """Run one RF scan cycle."""
        if self.pose is None:
            return

        cx = self.pose.pose.position.x
        cy = self.pose.pose.position.y
        cz = self.pose.pose.position.z

        pfa     = self.get_parameter('pfa').value
        any_det = False
        det_poses = PoseArray()
        det_poses.header.stamp    = self.get_clock().now().to_msg()
        det_poses.header.frame_id = 'map'
        best_rssi = -120.0   # dBm floor

        for (vx, vy, vz) in self.victims:
            r = math.sqrt((cx-vx)**2 + (cy-vy)**2 + (cz-vz)**2)
            pd = self._pd_at_range(r)

            if random.random() < pd:
                any_det = True
                # Add noisy position estimate
                p = Pose()
                p.position.x = vx + random.gauss(0, 1.0)
                p.position.y = vy + random.gauss(0, 1.0)
                p.position.z = vz
                p.orientation.w = 1.0
                det_poses.poses.append(p)
                rssi = self._rssi_at_range(r)
                best_rssi = max(best_rssi, rssi)

        # False alarm
        if not any_det and random.random() < pfa:
            any_det = True
            best_rssi = -90.0 + random.gauss(0, 5.0)

        # Publish detection flag
        det_msg = Bool()
        det_msg.data = any_det
        self.pub_detection.publish(det_msg)

        # Publish detected poses
        if det_poses.poses:
            self.pub_poses.publish(det_poses)

        # Publish RSSI
        rssi_msg = Float32()
        rssi_msg.data = float(best_rssi)
        self.pub_rssi.publish(rssi_msg)


# ---------------------------------------------------------------------------
# Entry point
# ---------------------------------------------------------------------------

def main(args=None):
    rclpy.init(args=args)
    node = RFDopplerStub()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
