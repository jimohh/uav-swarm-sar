#!/usr/bin/env python3
"""
vfh_navigator.py — Mid-scale VFH navigator for UAV swarm SAR
=============================================================
Layer:   Navigation → Mid-scale (VFH)
Package: sar_planning  (ROS 2 Humble / Ubuntu 22.04)

Architecture role
-----------------
Sits between the APF local-reactive layer (apf_navigator) and the
RRT* global planner (rrtstar_planner).  Receives a global waypoint
from RRT*, maintains a polar obstacle histogram from LaserScan /
PointCloud2, selects the lowest-cost valley heading, and publishes
velocity commands at 10 Hz.  When no obstacles are detected the node
passes through the pure geometric heading to the goal.

Topics
------
Subscribed:
  /uav{N}/goal_waypoint        [geometry_msgs/PointStamped]
      Mid-scale goal produced by rrtstar_planner or waypoint_selector
  /uav{N}/mavros/local_position/pose  [geometry_msgs/PoseStamped]
      EKF-fused local position (NED→ENU via MAVROS convention)
  /uav{N}/scan                 [sensor_msgs/LaserScan]  (optional)
      2-D lidar scan; if absent, VFH runs in open-space passthrough mode
  /uav{N}/mavros/obstacle_distance  [sensor_msgs/LaserScan]  (fallback)
      PX4 distance sensor array — used when /scan is absent

Published:
  /uav{N}/vfh/cmd_vel          [geometry_msgs/TwistStamped]
      Desired body-frame velocity forwarded to APF layer for blending
  /uav{N}/vfh/heading          [std_msgs/Float64]
      Selected best heading (radians, ENU)
  /uav{N}/vfh/histogram        [std_msgs/Float32MultiArray]
      Raw polar histogram (debug / thesis visualisation)

Parameters (ROS 2, declared in __init__)
-----------------------------------------
  uav_id           int   default 0      — instance number
  num_sectors      int   default 72     — histogram resolution (5 ° bins)
  safety_dist      float default 1.5    — obstacle safety radius [m]
  max_speed        float default 3.0    — cruise speed [m/s]
  goal_tol         float default 0.5    — goal-reached tolerance [m]
  valley_threshold float default 0.3    — hist magnitude below → free sector
  alpha            float default 0.5    — smoothing: target vs. continuity
  a_coeff          float default 1.0    — obstacle weight a (Borenstein 1991)
  b_coeff          float default 0.1    — distance decay  b (Borenstein 1991)
  h_m              float default 0.2    — VFH+ masked-threshold
  vfh_plus         bool  default True   — use VFH+ cost function

Design notes
------------
VFH (Borenstein & Koren 1991) builds a polar density histogram H_prime
around the UAV.  Each cell c_i in the occupancy grid contributes:

    h(k) += (a - b·d_i)² · cert_i       if sector(c_i) == k

where d_i is the distance to cell i and cert_i is its certainty value.

VFH+ (Ulrich & Borenstein 1998) adds a cost function that jointly
minimises:

    g(k) = μ₁·Δ(k,target) + μ₂·Δ(k,prev_dir) + μ₃·Δ(k,robot_dir)

This node uses a simplified 1-D scan version suited to Gazebo Harmonic
simulation:  the LaserScan replaces the 2-D occupancy grid but the
same histogram accumulation and valley-finding logic applies.

Reference
---------
Borenstein J, Koren Y (1991) "The vector field histogram — fast obstacle
avoidance for mobile robots." IEEE T-RA 7(3):278-288.
Ulrich I, Borenstein J (1998) "VFH+: reliable obstacle avoidance for
fast mobile robots." ICRA 1:1572-1577.
"""

import math
import numpy as np

from builtin_interfaces import msg
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy

from geometry_msgs.msg import TwistStamped, PointStamped, PoseStamped
from sensor_msgs.msg import LaserScan
from std_msgs.msg import Float64, Float32MultiArray, Bool
from builtin_interfaces.msg import Time

# ---------------------------------------------------------------------------
# Helper utilities
# ---------------------------------------------------------------------------

def wrap_to_pi(angle: float) -> float:
    """Wrap angle to (-π, π]."""
    return (angle + math.pi) % (2 * math.pi) - math.pi


def angular_diff(a: float, b: float) -> float:
    """Smallest signed angular difference a - b, wrapped to (-π, π]."""
    return wrap_to_pi(a - b)


# ---------------------------------------------------------------------------
# VFH Navigator Node
# ---------------------------------------------------------------------------

class VFHNavigator(Node):
    """Mid-scale Vector Field Histogram navigator."""

    # Cost weights for VFH+
    MU1 = 5.0   # target alignment
    MU2 = 2.0   # continuity with previous command
    MU3 = 2.0   # robot current heading alignment

    def __init__(self):
        super().__init__('vfh_navigator')

        # ── Parameters ──────────────────────────────────────────────────
        self.declare_parameter('uav_id',           0)
        self.declare_parameter('num_sectors',      72)
        self.declare_parameter('safety_dist',      1.5)
        self.declare_parameter('max_speed',        3.0)
        self.declare_parameter('goal_tol',         0.5)
        self.declare_parameter('valley_threshold', 0.3)
        self.declare_parameter('alpha',            0.5)
        self.declare_parameter('a_coeff',          1.0)
        self.declare_parameter('b_coeff',          0.1)
        self.declare_parameter('h_m',              0.2)
        self.declare_parameter('vfh_plus',         True)

        p = self._params()

        # ── Namespace / topic prefixes ───────────────────────────────────
        ns = f'/uav{p["uav_id"]}'

        # ── QoS ─────────────────────────────────────────────────────────
        sensor_qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
        )
        reliable_qos = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            history=HistoryPolicy.KEEP_LAST,
            depth=10,
        )

        # ── Subscribers ──────────────────────────────────────────────────
        self.sub_goal = self.create_subscription(
            PointStamped,
            f'{ns}/goal_waypoint',
            self._cb_goal,
            reliable_qos,
        )
        self.sub_pose = self.create_subscription(
            PoseStamped,
            f'{ns}/mavros/local_position/pose',
            self._cb_pose,
            sensor_qos,
        )
        self.sub_scan = self.create_subscription(
            LaserScan,
            f'{ns}/scan',
            self._cb_scan,
            sensor_qos,
        )
        # PX4 obstacle-distance fallback (DISTANCE_SENSOR MAVLink msg)
        self.sub_obs = self.create_subscription(
            LaserScan,
            f'{ns}/mavros/obstacle_distance',
            self._cb_scan,       # same callback — same message type
            sensor_qos,
        )

        # ── Publishers ───────────────────────────────────────────────────
        self.pub_cmd   = self.create_publisher(TwistStamped,       f'{ns}/vfh/cmd_vel',   reliable_qos)
        self.pub_hdg   = self.create_publisher(Float64,            f'{ns}/vfh/heading',   reliable_qos)
        self.pub_hist  = self.create_publisher(Float32MultiArray,  f'{ns}/vfh/histogram', reliable_qos)
        self.pub_hb    = self.create_publisher(Bool, f'{ns}/heartbeat', reliable_qos)

        # ── State ────────────────────────────────────────────────────────
        self.goal:       PointStamped | None = None
        self.pose:       PoseStamped  | None = None
        self.scan:       LaserScan    | None = None
        self.prev_hdg:   float = 0.0          # last selected heading [rad]
        self.robot_hdg:  float = 0.0          # current yaw from odometry [rad]

        # ── Main control loop ─────────────────────────────────────────────
        self.timer = self.create_timer(0.1, self._control_loop)   # 10 Hz
        self.hb_timer = self.create_timer(1.0, self._heartbeat_loop)  # 1 Hz

        self.get_logger().info(
            f'VFHNavigator started — UAV {p["uav_id"]} | '
            f'{p["num_sectors"]} sectors | VFH+={p["vfh_plus"]}'
        )

    # ── Parameter convenience ────────────────────────────────────────────

    def _params(self) -> dict:
        return {
            'uav_id':           self.get_parameter('uav_id').value,
            'num_sectors':      self.get_parameter('num_sectors').value,
            'safety_dist':      self.get_parameter('safety_dist').value,
            'max_speed':        self.get_parameter('max_speed').value,
            'goal_tol':         self.get_parameter('goal_tol').value,
            'valley_threshold': self.get_parameter('valley_threshold').value,
            'alpha':            self.get_parameter('alpha').value,
            'a_coeff':          self.get_parameter('a_coeff').value,
            'b_coeff':          self.get_parameter('b_coeff').value,
            'h_m':              self.get_parameter('h_m').value,
            'vfh_plus':         self.get_parameter('vfh_plus').value,
        }

    # ── Callbacks ────────────────────────────────────────────────────────

    def _cb_goal(self, msg: PointStamped):
        self.goal = msg

    def _cb_pose(self, msg: PoseStamped):
        self.pose = msg
        # Extract yaw from quaternion (z-axis rotation)
        q = msg.pose.orientation
        siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
        cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
        self.robot_hdg = math.atan2(siny_cosp, cosy_cosp)

    def _cb_scan(self, msg: LaserScan):
        self.scan = msg

    # ── Core VFH logic ───────────────────────────────────────────────────

    def _build_histogram(self, p: dict) -> np.ndarray:
        """
        Build the polar obstacle density histogram H_prime.

        Each LaserScan range reading is treated as a point obstacle.
        The certainty value is linearly proportional to (1/range).
        Sectors outside the safety distance are left at zero.
        """
        N = p['num_sectors']
        h = np.zeros(N, dtype=np.float32)

        if self.scan is None:
            return h

        scan = self.scan
        a = p['a_coeff']
        b = p['b_coeff']
        num_ranges = len(scan.ranges)

        for i, r in enumerate(scan.ranges):
            if not math.isfinite(r) or r <= 0.0:
                continue
            if r < scan.range_min or r > scan.range_max:
                continue

            # Angle of this beam in the robot frame
            angle_rad = scan.angle_min + i * scan.angle_increment
            # Map to histogram sector index
            sector = int(((angle_rad % (2 * math.pi)) / (2 * math.pi)) * N) % N

            # VFH obstacle magnitude: higher for closer obstacles
            cert = max(0.0, a - b * r)
            h[sector] += cert * cert

        # Smooth histogram with a 3-sector sliding window
        h = np.convolve(h, np.array([0.25, 0.5, 0.25]), mode='same')

        # Normalise to [0, 1]
        max_h = h.max()
        if max_h > 1e-6:
            h /= max_h

        return h

    def _find_valleys(self, h: np.ndarray, threshold: float) -> list[tuple[int, int]]:
        """
        Identify contiguous free (valley) sectors where h[k] < threshold.
        Returns list of (start_sector, end_sector) pairs (inclusive).

        Handles wrap-around correctly: if sectors at both ends of the array
        are free they belong to the same valley (e.g. sectors 70-71, 0-1
        → one valley represented as (70, 1)).
        """
        N = len(h)
        free = h < threshold

        if not np.any(free):
            return []
        if np.all(free):
            return [(0, N - 1)]

        # Linear scan for runs
        valleys = []
        in_v    = False
        v_start = 0

        for i in range(N):
            if free[i] and not in_v:
                v_start = i
                in_v    = True
            elif not free[i] and in_v:
                valleys.append((v_start, i - 1))
                in_v = False
        if in_v:
            valleys.append((v_start, N - 1))

        # Merge head and tail valleys when both ends of the array are free
        if len(valleys) >= 2 and free[0] and free[N - 1]:
            first, last = valleys[0], valleys[-1]
            if first[0] == 0 and last[1] == N - 1:
                valleys = [(last[0], first[1])] + valleys[1:-1]

        return valleys

    def _sector_to_angle(self, sector: int, N: int) -> float:
        """Convert sector index to heading angle [rad] in robot frame."""
        return (sector / N) * 2.0 * math.pi

    def _vfh_select_heading(
        self,
        h: np.ndarray,
        target_hdg: float,
        p: dict,
    ) -> float | None:
        """
        Select best heading using VFH+ cost function.

        Returns the best heading in the world frame [rad], or None if
        every sector is blocked.

        The cost for candidate sector k is:
            g(k) = μ₁·|Δ(k, target)| + μ₂·|Δ(k, prev_hdg)| + μ₃·|Δ(k, robot_hdg)|
        """
        N   = p['num_sectors']
        thr = p['valley_threshold']

        valleys = self._find_valleys(h, thr)

        if not valleys:
            return None

        # Collect candidate sector centres
        candidates = []
        for (s, e) in valleys:
            if s <= e:
                mid = (s + e) // 2
            else:
                # Wraps around
                length = (e + N - s) % N
                mid = (s + length // 2) % N
            candidates.append(mid)

        # VFH+ cost
        best_hdg  = None
        best_cost = float('inf')

        for sector in candidates:
            hdg = self._sector_to_angle(sector, N)   # robot-frame heading

            if p['vfh_plus']:
                # Δ in world frame
                world_hdg = wrap_to_pi(hdg + self.robot_hdg)
                cost = (
                    self.MU1 * abs(angular_diff(world_hdg, target_hdg))
                    + self.MU2 * abs(angular_diff(world_hdg, self.prev_hdg))
                    + self.MU3 * abs(angular_diff(world_hdg, self.robot_hdg))
                )
            else:
                # Original VFH: minimise deviation from target only
                world_hdg = wrap_to_pi(hdg + self.robot_hdg)
                cost = abs(angular_diff(world_hdg, target_hdg))

            if cost < best_cost:
                best_cost = cost
                best_hdg  = world_hdg

        return best_hdg

    # ── Control loop ─────────────────────────────────────────────────────

    def _control_loop(self):
        p = self._params()

        if self.goal is None or self.pose is None:
            return

        # ── Current position ──────────────────────────────────────────
        cx = self.pose.pose.position.x
        cy = self.pose.pose.position.y
        cz = self.pose.pose.position.z

        # ── Goal position ─────────────────────────────────────────────
        gx = self.goal.point.x
        gy = self.goal.point.y
        gz = self.goal.point.z

        # ── Distance / direction to goal ──────────────────────────────
        dx = gx - cx
        dy = gy - cy
        dz = gz - cz
        dist_xy = math.hypot(dx, dy)
        dist_3d = math.sqrt(dx*dx + dy*dy + dz*dz)

        if dist_3d < p['goal_tol']:
            # Publish zero velocity — goal reached
            self._publish_stop(p)
            return

        target_hdg = math.atan2(dy, dx)   # world-frame heading to goal

        # ── Build obstacle histogram ───────────────────────────────────
        h = self._build_histogram(p)

        # ── Publish raw histogram for debugging ────────────────────────
        hist_msg = Float32MultiArray()
        hist_msg.data = h.tolist()
        self.pub_hist.publish(hist_msg)

        # ── Select heading ────────────────────────────────────────────
        best_hdg = self._vfh_select_heading(h, target_hdg, p)

        if best_hdg is None:
            # No free valley — stop and log
            self.get_logger().warn('VFH: all sectors blocked, holding position')
            self._publish_stop(p)
            return

        # ── Smooth selected heading (circular mean — safe across ±π) ─────
        import cmath
        alpha      = p['alpha']
        blended    = alpha * cmath.exp(1j * best_hdg) + (1 - alpha) * cmath.exp(1j * self.prev_hdg)
        smooth_hdg = cmath.phase(blended)   # always in (-π, π]
        self.prev_hdg = smooth_hdg

        # ── Compute velocity command ──────────────────────────────────
        # Speed scales down as UAV approaches goal (slow-down ramp)
        ramp     = min(1.0, dist_xy / 3.0)   # full speed beyond 3 m
        speed    = p['max_speed'] * ramp

        vx = speed * math.cos(smooth_hdg)
        vy = speed * math.sin(smooth_hdg)
        # Vertical: P-controller to match goal altitude
        vz = max(-1.5, min(1.5, 0.7 * dz))

        # ── Publish cmd_vel ───────────────────────────────────────────
        cmd = TwistStamped()
        cmd.header.stamp    = self.get_clock().now().to_msg()
        cmd.header.frame_id = 'map'
        cmd.twist.linear.x  = vx
        cmd.twist.linear.y  = vy
        cmd.twist.linear.z  = vz
        self.pub_cmd.publish(cmd)

        # ── Publish selected heading ──────────────────────────────────
        hdg_msg = Float64()
        hdg_msg.data = smooth_hdg
        self.pub_hdg.publish(hdg_msg)

    def _publish_stop(self, p: dict):
        cmd = TwistStamped()
        cmd.header.stamp    = self.get_clock().now().to_msg()
        cmd.header.frame_id = 'map'
        self.pub_cmd.publish(cmd)
    def _heartbeat_loop(self):
        msg = Bool()
        msg.data = True
        self.pub_hb.publish(msg)


# ---------------------------------------------------------------------------
# Entry point
# ---------------------------------------------------------------------------

def main(args=None):
    rclpy.init(args=args)
    node = VFHNavigator()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
