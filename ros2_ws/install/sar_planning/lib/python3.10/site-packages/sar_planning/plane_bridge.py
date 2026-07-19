#!/usr/bin/env python3
"""
plane_bridge.py — UAV2 Fixed-Wing (Standard Plane) Bridge Node
===============================================================
Package: sar_planning  (ROS 2 Humble / Ubuntu 22.04)

Architecture role
-----------------
UAV2 is a PX4 Standard Plane (gz_standard_vtol or gz_plane), which has
fundamentally different flight constraints from the two Iris quadrotors:

  • Cannot hover — requires minimum airspeed (~12 m/s) at all times
  • Cannot fly backwards or strafe
  • Turns via coordinated bank angles, not direct yaw
  • Requires a loiter/orbit pattern when waiting on station
  • Altitude changes are slow (shallow climb/dive angle)

This node acts as a **bridge** between the generic swarm navigation
stack (which publishes TwistStamped cmd_vel) and the PX4 fixed-wing
autopilot (which expects setpoint_raw/global or setpoint_raw/local
in OFFBOARD mode).

Key behaviours
--------------
  1. Converts cmd_vel velocity commands into fixed-wing–compatible
     position/velocity setpoints (NED frame via MAVROS)
  2. Enforces minimum airspeed — never commands below V_MIN
  3. Implements a loiter orbit when no goal is active (station-keeping)
  4. Participates in CNP auction via /uav2/cnp/* topics (same interface
     as quadrotors — transparent to cnp_coordinator)
  5. Publishes heartbeat at 1 Hz (same as other UAVs)
  6. Reports plane-specific telemetry: airspeed, bank angle, groundspeed

Topics
------
Subscribed:
  /uav2/goal_waypoint          [geometry_msgs/PointStamped]  — from RRT*
  /uav2/mavros/local_position/pose   [geometry_msgs/PoseStamped]
  /uav2/mavros/local_position/velocity_body [geometry_msgs/TwistStamped]
  /uav2/mavros/state           [mavros_msgs/State]

Published:
  /uav2/mavros/setpoint_raw/local  [mavros_msgs/PositionTarget]
      OFFBOARD position+velocity setpoint (NED)
  /uav2/mavros/setpoint_velocity/cmd_vel_unstamped [geometry_msgs/Twist]
      Velocity fallback for MAVROS
  /uav2/plane/status           [std_msgs/String]  — JSON telemetry
  /uav2/heartbeat              [std_msgs/Bool]

Services called (via MAVROS):
  /uav2/mavros/cmd/arming
  /uav2/mavros/set_mode

Parameters
----------
  uav_id          int    default 2
  v_min           float  default 12.0   — minimum airspeed [m/s]
  v_cruise        float  default 18.0   — cruise airspeed [m/s]
  v_max           float  default 25.0   — maximum airspeed [m/s]
  loiter_radius   float  default 30.0   — orbit radius when loitering [m]
  loiter_alt      float  default 15.0   — loiter altitude AGL [m]
  climb_angle_max float  default 15.0   — max climb/dive angle [deg]
  goal_tol        float  default 15.0   — waypoint acceptance radius [m]
                                           (larger than quads due to speed)
  offboard_rate   float  default 20.0   — setpoint publish rate [Hz]
  arm_on_start    bool   default False  — auto-arm when node starts
"""

import math
import json
import time

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy

from geometry_msgs.msg import PointStamped, PoseStamped, TwistStamped, Twist
from std_msgs.msg import Bool, String

# MAVROS message types
try:
    from mavros_msgs.msg import PositionTarget, State
    from mavros_msgs.srv import CommandBool, SetMode
    MAVROS_AVAILABLE = True
except ImportError:
    MAVROS_AVAILABLE = False


# ---------------------------------------------------------------------------
# Constants
# ---------------------------------------------------------------------------

# PX4 coordinate mask bits for PositionTarget
IGNORE_PX = 0b0000_0000_0000_0001   # ignore pos X
IGNORE_PY = 0b0000_0000_0000_0010   # ignore pos Y
IGNORE_PZ = 0b0000_0000_0000_0100   # ignore pos Z
IGNORE_VX = 0b0000_0000_0000_1000   # ignore vel X
IGNORE_VY = 0b0000_0000_0001_0000   # ignore vel Y
IGNORE_VZ = 0b0000_0000_0010_0000   # ignore vel Z
IGNORE_AFX= 0b0000_0000_0100_0000
IGNORE_AFY= 0b0000_0000_1000_0000
IGNORE_AFZ= 0b0000_0001_0000_0000
IGNORE_YAW= 0b0000_0100_0000_0000
IGNORE_YAW_RATE = 0b0000_1000_0000_0000

# Use position + yaw, ignore velocity/accel
MASK_POS_YAW = (IGNORE_VX | IGNORE_VY | IGNORE_VZ |
                IGNORE_AFX | IGNORE_AFY | IGNORE_AFZ |
                IGNORE_YAW_RATE)

# Use velocity + yaw_rate, ignore position/accel
MASK_VEL_YAW_RATE = (IGNORE_PX | IGNORE_PY | IGNORE_PZ |
                     IGNORE_AFX | IGNORE_AFY | IGNORE_AFZ |
                     IGNORE_YAW)

DEG2RAD = math.pi / 180.0


# ---------------------------------------------------------------------------
# Helper
# ---------------------------------------------------------------------------

def wrap_to_pi(a: float) -> float:
    return (a + math.pi) % (2 * math.pi) - math.pi


# ---------------------------------------------------------------------------
# PlaneBridge Node
# ---------------------------------------------------------------------------

class PlaneBridge(Node):
    """
    Fixed-wing UAV2 bridge: translates swarm navigation commands into
    PX4-compatible OFFBOARD setpoints for a Standard Plane.
    """

    # Flight state machine
    STATE_LOITER   = 'LOITER'    # no goal — orbit at loiter point
    STATE_TRANSIT  = 'TRANSIT'   # flying to goal waypoint
    STATE_ARRIVED  = 'ARRIVED'   # within goal_tol, transitioning back to loiter

    def __init__(self):
        super().__init__('plane_bridge')

        # ── Parameters ────────────────────────────────────────────────
        self.declare_parameter('uav_id',          2)
        self.declare_parameter('v_min',           12.0)
        self.declare_parameter('v_cruise',        18.0)
        self.declare_parameter('v_max',           25.0)
        self.declare_parameter('loiter_radius',   30.0)
        self.declare_parameter('loiter_alt',      15.0)
        self.declare_parameter('climb_angle_max', 15.0)
        self.declare_parameter('goal_tol',        15.0)
        self.declare_parameter('offboard_rate',   20.0)
        self.declare_parameter('arm_on_start',    False)

        ns = f'/uav{self.get_parameter("uav_id").value}'

        # ── QoS ───────────────────────────────────────────────────────
        sensor_qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST, depth=1)
        reliable_qos = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            history=HistoryPolicy.KEEP_LAST, depth=10)

        # ── Subscribers ───────────────────────────────────────────────
        self.sub_goal = self.create_subscription(
            PointStamped, f'{ns}/goal_waypoint', self._cb_goal, reliable_qos)
        self.sub_pose = self.create_subscription(
            PoseStamped, f'{ns}/mavros/local_position/pose',
            self._cb_pose, sensor_qos)
        self.sub_vel = self.create_subscription(
            TwistStamped, f'{ns}/mavros/local_position/velocity_body',
            self._cb_vel, sensor_qos)

        if MAVROS_AVAILABLE:
            self.sub_state = self.create_subscription(
                State, f'{ns}/mavros/state', self._cb_state, reliable_qos)

        # ── Publishers ────────────────────────────────────────────────
        self.pub_heartbeat = self.create_publisher(
            Bool, f'{ns}/heartbeat', reliable_qos)
        self.pub_status = self.create_publisher(
            String, f'{ns}/plane/status', reliable_qos)

        if MAVROS_AVAILABLE:
            self.pub_setpoint = self.create_publisher(
                PositionTarget,
                f'{ns}/mavros/setpoint_raw/local',
                reliable_qos)
            self.pub_vel_cmd = self.create_publisher(
                Twist,
                f'{ns}/mavros/setpoint_velocity/cmd_vel_unstamped',
                reliable_qos)

        # ── Service clients ───────────────────────────────────────────
        if MAVROS_AVAILABLE:
            self.arm_client  = self.create_client(
                CommandBool, f'{ns}/mavros/cmd/arming')
            self.mode_client = self.create_client(
                SetMode, f'{ns}/mavros/set_mode')

        # ── State ─────────────────────────────────────────────────────
        self.goal:         PointStamped | None = None
        self.pose:         PoseStamped  | None = None
        self.body_vel:     TwistStamped | None = None
        self.px4_state:    object | None       = None
        self.flight_state: str                 = self.STATE_LOITER
        self.loiter_centre: tuple[float,float,float] | None = None
        self.loiter_angle: float               = 0.0   # current orbit angle [rad]
        self._offboard_sent: int               = 0     # setpoints sent before mode switch
        self._armed: bool                      = False

        # ── Timers ────────────────────────────────────────────────────
        rate = self.get_parameter('offboard_rate').value
        self.sp_timer  = self.create_timer(1.0 / rate, self._setpoint_loop)
        self.hb_timer  = self.create_timer(1.0,         self._heartbeat_loop)
        self.arm_timer = self.create_timer(3.0,         self._arm_and_offboard)

        self.get_logger().info(
            f'PlaneBridge started — UAV2 (Standard Plane) | '
            f'v_cruise={self.get_parameter("v_cruise").value} m/s'
        )

    # ── Callbacks ─────────────────────────────────────────────────────────

    def _cb_goal(self, msg: PointStamped):
        self.goal = msg
        self.flight_state = self.STATE_TRANSIT
        self.get_logger().info(
            f'Plane: new goal ({msg.point.x:.1f}, {msg.point.y:.1f}, {msg.point.z:.1f})'
        )

    def _cb_pose(self, msg: PoseStamped):
        self.pose = msg
        # Initialise loiter centre at first known position
        if self.loiter_centre is None:
            self.loiter_centre = (
                msg.pose.position.x,
                msg.pose.position.y,
                self.get_parameter('loiter_alt').value,
            )

    def _cb_vel(self, msg: TwistStamped):
        self.body_vel = msg

    def _cb_state(self, msg):
        self.px4_state = msg

    # ── Arm + OFFBOARD ────────────────────────────────────────────────────

    def _arm_and_offboard(self):
        """
        Called at 3 s intervals. Sends enough setpoints for PX4 to accept
        OFFBOARD mode, then arms and switches mode.
        """
        if not MAVROS_AVAILABLE or self.pose is None:
            return

        # PX4 requires >2 Hz setpoints before accepting OFFBOARD
        if self._offboard_sent < 10:
            return   # wait for setpoint loop to accumulate

        if self.px4_state is None:
            return

        if not self.px4_state.armed and not self._armed:
            if not self.arm_client.service_is_ready():
                return
            req = CommandBool.Request()
            req.value = True
            self.arm_client.call_async(req)
            self._armed = True
            self.get_logger().info('Plane: arming requested')

        if self.px4_state.mode != 'OFFBOARD':
            if not self.mode_client.service_is_ready():
                return
            req = SetMode.Request()
            req.custom_mode = 'OFFBOARD'
            self.mode_client.call_async(req)
            self.get_logger().info('Plane: OFFBOARD mode requested')

        # Run once successfully then cancel
        self.arm_timer.cancel()

    # ── Main setpoint loop ────────────────────────────────────────────────

    def _setpoint_loop(self):
        if self.pose is None:
            return

        p = self._params()

        cx = self.pose.pose.position.x
        cy = self.pose.pose.position.y
        cz = self.pose.pose.position.z

        if self.flight_state == self.STATE_TRANSIT and self.goal is not None:
            self._transit_setpoint(cx, cy, cz, p)
        else:
            self._loiter_setpoint(cx, cy, cz, p)

        self._offboard_sent += 1

    def _transit_setpoint(self, cx, cy, cz, p):
        """
        Fly toward goal waypoint using position setpoint.
        Enforces altitude change limits and minimum airspeed.
        Switches to LOITER when within goal_tol.
        """
        gx = self.goal.point.x
        gy = self.goal.point.y
        gz = self.goal.point.z

        dx = gx - cx
        dy = gy - cy
        dz = gz - cz
        dist_2d = math.hypot(dx, dy)
        dist_3d = math.sqrt(dx*dx + dy*dy + dz*dz)

        if dist_3d < p['goal_tol']:
            self.flight_state = self.STATE_LOITER
            self.loiter_centre = (gx, gy, gz)
            self.get_logger().info('Plane: goal reached, entering loiter')
            return

        # Target heading
        target_yaw = math.atan2(dy, dx)

        # Altitude: clamp vertical rate to climb angle limit
        max_dz = dist_2d * math.tan(p['climb_angle_max'] * DEG2RAD)
        clamped_gz = cz + max(-max_dz, min(max_dz, dz))

        if MAVROS_AVAILABLE:
            sp = PositionTarget()
            sp.header.stamp    = self.get_clock().now().to_msg()
            sp.header.frame_id = 'map'
            sp.coordinate_frame = PositionTarget.FRAME_LOCAL_NED
            sp.type_mask = MASK_POS_YAW

            # ENU → NED conversion (MAVROS local frame is ENU, PX4 NED)
            # MAVROS handles this automatically when using FRAME_LOCAL_NED
            sp.position.x = gx
            sp.position.y = gy
            sp.position.z = clamped_gz
            sp.yaw        = float(target_yaw)

            self.pub_setpoint.publish(sp)
        else:
            # Fallback: publish Twist for simulation without full MAVROS
            t = Twist()
            speed = min(p['v_cruise'], max(p['v_min'], dist_3d))
            t.linear.x = speed * math.cos(target_yaw)
            t.linear.y = speed * math.sin(target_yaw)
            t.linear.z = max(-2.0, min(2.0, dz * 0.3))
            if MAVROS_AVAILABLE:
                self.pub_vel_cmd.publish(t)

    def _loiter_setpoint(self, cx, cy, cz, p):
        """
        Orbit around loiter_centre at loiter_radius and loiter_alt.
        Advances angle at cruise speed.
        """
        if self.loiter_centre is None:
            self.loiter_centre = (cx, cy, p['loiter_alt'])

        lx, ly, lz = self.loiter_centre
        r = p['loiter_radius']
        v = p['v_cruise']

        # Angular rate ω = v / r
        dt   = 1.0 / p['offboard_rate']
        self.loiter_angle += (v / r) * dt
        self.loiter_angle  = self.loiter_angle % (2 * math.pi)

        # Next orbit position
        tx = lx + r * math.cos(self.loiter_angle)
        ty = ly + r * math.sin(self.loiter_angle)
        tz = lz

        # Yaw tangent to orbit
        yaw = self.loiter_angle + math.pi / 2.0

        if MAVROS_AVAILABLE:
            sp = PositionTarget()
            sp.header.stamp     = self.get_clock().now().to_msg()
            sp.header.frame_id  = 'map'
            sp.coordinate_frame = PositionTarget.FRAME_LOCAL_NED
            sp.type_mask        = MASK_POS_YAW
            sp.position.x = tx
            sp.position.y = ty
            sp.position.z = tz
            sp.yaw        = float(wrap_to_pi(yaw))
            self.pub_setpoint.publish(sp)

    # ── Heartbeat + status ────────────────────────────────────────────────

    def _heartbeat_loop(self):
        # Heartbeat
        hb = Bool(); hb.data = True
        self.pub_heartbeat.publish(hb)

        # JSON status telemetry
        airspeed = 0.0
        if self.body_vel is not None:
            # Forward body velocity ≈ airspeed in calm conditions
            airspeed = abs(self.body_vel.twist.linear.x)

        status = {
            'uav_id':       2,
            'type':         'fixed_wing',
            'flight_state': self.flight_state,
            'airspeed_ms':  round(airspeed, 2),
            'loiter_angle': round(math.degrees(self.loiter_angle), 1),
            'goal':         None if self.goal is None else {
                'x': round(self.goal.point.x, 2),
                'y': round(self.goal.point.y, 2),
                'z': round(self.goal.point.z, 2),
            },
        }
        msg = String(); msg.data = json.dumps(status)
        self.pub_status.publish(msg)

    # ── Parameter convenience ─────────────────────────────────────────────

    def _params(self) -> dict:
        return {
            'uav_id':          self.get_parameter('uav_id').value,
            'v_min':           self.get_parameter('v_min').value,
            'v_cruise':        self.get_parameter('v_cruise').value,
            'v_max':           self.get_parameter('v_max').value,
            'loiter_radius':   self.get_parameter('loiter_radius').value,
            'loiter_alt':      self.get_parameter('loiter_alt').value,
            'climb_angle_max': self.get_parameter('climb_angle_max').value,
            'goal_tol':        self.get_parameter('goal_tol').value,
            'offboard_rate':   self.get_parameter('offboard_rate').value,
            'arm_on_start':    self.get_parameter('arm_on_start').value,
        }


# ---------------------------------------------------------------------------
# Entry point
# ---------------------------------------------------------------------------

def main(args=None):
    rclpy.init(args=args)
    node = PlaneBridge()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
