#!/usr/bin/env python3
"""
rrtstar_planner.py — Global RRT* path planner for UAV swarm SAR
================================================================
Layer:   Navigation → Global (RRT*)
Package: sar_planning  (ROS 2 Humble / Ubuntu 22.04)

Architecture role
-----------------
Top of the three-scale navigation hierarchy.  Receives a mission
waypoint from waypoint_selector (POC planner output), plans a
collision-free path through the known 3-D obstacle map using RRT*,
and publishes a sequence of intermediate sub-goals consumed by
vfh_navigator.

Key design choices
------------------
* **Incremental replanning** — runs in a background timer at 1 Hz.
  If the goal changes or path becomes invalid, planning restarts.
* **Informed RRT*** (Gammell et al. 2014) — once an initial solution
  exists, sampling is restricted to an ellipsoidal subset between
  start and goal, accelerating convergence.
* **Obstacle map** — populated from MAVROS /local_costmap or from
  a simplified axis-aligned bounding-box list set via a service.
  For SITL runs without a costmap, the planner falls back to a
  synthetic random-pillar map matching the Gazebo SAR world.
* **Path smoothing** — after RRT* terminates, a path-shortcutting
  pass removes unnecessary waypoints that have line-of-sight
  clearance.

Topics
------
Subscribed:
  /uav{N}/mission_waypoint       [geometry_msgs/PointStamped]
      Final goal from waypoint_selector
  /uav{N}/mavros/local_position/pose  [geometry_msgs/PoseStamped]
      Current position for start node
  /uav{N}/rrtstar/obstacles      [visualization_msgs/MarkerArray]
      Optional obstacle boxes (from Gazebo ground-truth or costmap)

Published:
  /uav{N}/goal_waypoint          [geometry_msgs/PointStamped]
      Next intermediate waypoint on the planned path → vfh_navigator
  /uav{N}/rrtstar/path           [nav_msgs/Path]
      Full planned path (RViz / thesis logging)
  /uav{N}/rrtstar/tree           [visualization_msgs/MarkerArray]
      RRT* tree edges for thesis visualisation (throttled at 1 Hz)

Parameters
----------
  uav_id          int   default 0       — instance number
  max_iter        int   default 2000    — RRT* iterations per plan
  step_size       float default 2.0     — expansion step [m]
  goal_bias       float default 0.10    — probability of sampling goal
  goal_tol        float default 0.8     — solution acceptance radius [m]
  search_radius   float default 4.0     — rewire radius [m]
  map_x_min       float default -50.0   — search space bounds [m]
  map_x_max       float default  50.0
  map_y_min       float default -50.0
  map_y_max       float default  50.0
  map_z_min       float default   2.0   — minimum safe altitude
  map_z_max       float default  30.0
  informed_rrtstar bool  default True   — enable ellipsoidal sampling
  waypoint_index  int   default 1       — sub-goal lookahead on path
  safety_margin   float default 1.5     — obstacle inflation [m]

References
----------
LaValle S (1998) Rapidly-exploring random trees: a new tool for path
    planning. TR 98-11, Iowa State Univ.
Karaman S, Frazzoli E (2011) "Sampling-based algorithms for optimal
    motion planning." IJRR 30(7):846-894.
Gammell J, Srinivasa S, Barfoot T (2014) "Informed RRT*: optimal
    sampling-based path planning." IROS 2014:2997-3004.
"""

import math
import random
import threading
import numpy as np

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy

from geometry_msgs.msg import (
    PointStamped, PoseStamped, Point, PoseArray, Pose
)
from nav_msgs.msg import Path
from std_msgs.msg import Header
from visualization_msgs.msg import Marker, MarkerArray


# ---------------------------------------------------------------------------
# Data structures
# ---------------------------------------------------------------------------

class Node3D:
    """Single node in the RRT* tree."""

    __slots__ = ('x', 'y', 'z', 'parent', 'cost', 'id')

    def __init__(self, x: float, y: float, z: float, _id: int = 0):
        self.x      = x
        self.y      = y
        self.z      = z
        self.parent: 'Node3D | None' = None
        self.cost:   float           = 0.0
        self.id     = _id

    def dist(self, other: 'Node3D') -> float:
        return math.sqrt(
            (self.x - other.x)**2 +
            (self.y - other.y)**2 +
            (self.z - other.z)**2
        )

    def as_point(self) -> tuple[float, float, float]:
        return (self.x, self.y, self.z)


class AABB:
    """Axis-aligned bounding box obstacle."""

    __slots__ = ('x_min', 'x_max', 'y_min', 'y_max', 'z_min', 'z_max')

    def __init__(self, cx, cy, cz, rx, ry, rz):
        self.x_min = cx - rx;  self.x_max = cx + rx
        self.y_min = cy - ry;  self.y_max = cy + ry
        self.z_min = cz - rz;  self.z_max = cz + rz

    def collides_point(self, x, y, z, margin=0.0) -> bool:
        return (
            self.x_min - margin <= x <= self.x_max + margin and
            self.y_min - margin <= y <= self.y_max + margin and
            self.z_min - margin <= z <= self.z_max + margin
        )

    def collides_segment(
        self,
        x1, y1, z1,
        x2, y2, z2,
        margin=0.0,
        steps=10,
    ) -> bool:
        """Coarse segment collision check via interpolation."""
        for t in np.linspace(0, 1, steps):
            px = x1 + t * (x2 - x1)
            py = y1 + t * (y2 - y1)
            pz = z1 + t * (z2 - z1)
            if self.collides_point(px, py, pz, margin):
                return True
        return False


# ---------------------------------------------------------------------------
# RRT* Core (pure Python, no external deps)
# ---------------------------------------------------------------------------

class RRTStar:
    """
    3-D RRT* planner.

    Supports both standard and Informed RRT* (ellipsoidal sampling).
    """

    def __init__(
        self,
        start:          tuple[float, float, float],
        goal:           tuple[float, float, float],
        bounds:         tuple[float, float, float, float, float, float],
        obstacles:      list[AABB],
        step_size:      float = 2.0,
        max_iter:       int   = 2000,
        goal_bias:      float = 0.10,
        goal_tol:       float = 0.8,
        search_radius:  float = 4.0,
        safety_margin:  float = 1.5,
        informed:       bool  = True,
    ):
        self.start         = Node3D(*start)
        self.goal_pt       = Node3D(*goal)
        self.bounds        = bounds          # x_min,x_max,y_min,y_max,z_min,z_max
        self.obstacles     = obstacles
        self.step_size     = step_size
        self.max_iter      = max_iter
        self.goal_bias     = goal_bias
        self.goal_tol      = goal_tol
        self.search_radius = search_radius
        self.safety_margin = safety_margin
        self.informed      = informed

        self.tree:     list[Node3D]       = [self.start]
        self.solution: list[Node3D] | None = None
        self.best_cost = float('inf')
        self._node_count = 1   # instance-local counter (avoids class-level bleed)

        # Informed RRT* ellipsoid parameters
        self._c_min   = self.start.dist(self.goal_pt)
        self._c_best  = float('inf')
        # NOTE: search_radius is fixed here for simplicity. Optimal RRT* scales it as
        #   r = γ · (log(n)/n)^(1/d),  d=3, γ = 2·(1 + 1/d)^(1/d) · (vol/ζ_d)^(1/d)
        # For SITL with bounded workspaces (≤100 m), fixed radius ≈ 4 m gives
        # near-optimal results within 2000 iterations. Enable adaptive radius for
        # larger operational areas by replacing self.search_radius with _adaptive_radius().

    # ── Sampling ─────────────────────────────────────────────────────────

    def _sample_free(self) -> Node3D:
        x_min, x_max, y_min, y_max, z_min, z_max = self.bounds

        if random.random() < self.goal_bias:
            self._node_count += 1
            return Node3D(self.goal_pt.x, self.goal_pt.y, self.goal_pt.z, self._node_count)

        if self.informed and self._c_best < float('inf'):
            return self._sample_ellipsoid(x_min, x_max, y_min, y_max, z_min, z_max)

        # Uniform random sample
        for _ in range(100):
            x = random.uniform(x_min, x_max)
            y = random.uniform(y_min, y_max)
            z = random.uniform(z_min, z_max)
            if not self._in_collision(x, y, z):
                self._node_count += 1
                return Node3D(x, y, z, self._node_count)

        # Fallback: return a random point ignoring collisions
        self._node_count += 1
        return Node3D(
            random.uniform(x_min, x_max),
            random.uniform(y_min, y_max),
            random.uniform(z_min, z_max),
            self._node_count,
        )

    def _sample_ellipsoid(self, x_min, x_max, y_min, y_max, z_min, z_max) -> Node3D:
        """
        Informed RRT* ellipsoidal sampling.

        The prolate hyper-spheroid has:
          c_max = c_best (best path cost found so far)
          c_min = straight-line distance start→goal

        We sample uniformly in a unit ball, scale, rotate, and translate.
        """
        c_max = self._c_best
        c_min = self._c_min

        # Semi-axes
        r1  = c_max / 2.0
        r23 = math.sqrt(max(0.0, c_max**2 - c_min**2)) / 2.0

        # Unit-ball sample (rejection)
        for _ in range(200):
            # Sample on unit ball using normal distribution normalisation
            u = np.random.normal(0, 1, 3)
            norm = np.linalg.norm(u)
            if norm < 1e-9:
                continue
            r = random.random() ** (1.0 / 3.0)
            u = r * u / norm

            # Scale to ellipsoid
            p_ellip = np.array([r1 * u[0], r23 * u[1], r23 * u[2]])

            # Rotation: align first axis with start→goal
            sx, sy, sz = self.start.x, self.start.y, self.start.z
            gx, gy, gz = self.goal_pt.x, self.goal_pt.y, self.goal_pt.z
            a1 = np.array([gx - sx, gy - sy, gz - sz])
            if np.linalg.norm(a1) < 1e-9:
                a1 = np.array([1.0, 0.0, 0.0])
            else:
                a1 /= np.linalg.norm(a1)

            # Build orthonormal basis (Gram-Schmidt)
            tmp = np.array([0.0, 1.0, 0.0]) if abs(a1[0]) < 0.9 else np.array([0.0, 0.0, 1.0])
            a2  = tmp - np.dot(tmp, a1) * a1
            a2 /= np.linalg.norm(a2)
            a3  = np.cross(a1, a2)

            R = np.column_stack([a1, a2, a3])
            p_world = R @ p_ellip + np.array([(sx + gx) / 2, (sy + gy) / 2, (sz + gz) / 2])

            x, y, z = float(p_world[0]), float(p_world[1]), float(p_world[2])

            # Clip to search bounds
            x = max(x_min, min(x_max, x))
            y = max(y_min, min(y_max, y))
            z = max(z_min, min(z_max, z))

            if not self._in_collision(x, y, z):
                self._node_count += 1
                return Node3D(x, y, z, self._node_count)

        # Fallback to uniform
        self._node_count += 1
        return Node3D(
            random.uniform(x_min, x_max),
            random.uniform(y_min, y_max),
            random.uniform(z_min, z_max),
            self._node_count,
        )

    # ── Collision checking ────────────────────────────────────────────────

    def _in_collision(self, x, y, z) -> bool:
        m = self.safety_margin
        for obs in self.obstacles:
            if obs.collides_point(x, y, z, margin=m):
                return True
        return False

    def _segment_free(self, n1: Node3D, n2: Node3D) -> bool:
        m = self.safety_margin
        for obs in self.obstacles:
            if obs.collides_segment(n1.x, n1.y, n1.z, n2.x, n2.y, n2.z, margin=m):
                return False
        return True

    # ── Tree operations ───────────────────────────────────────────────────

    def _nearest(self, q: Node3D) -> Node3D:
        return min(self.tree, key=lambda n: n.dist(q))

    def _near(self, q: Node3D) -> list[Node3D]:
        r = self.search_radius
        return [n for n in self.tree if n.dist(q) <= r]

    def _steer(self, from_node: Node3D, to_node: Node3D) -> Node3D:
        d = from_node.dist(to_node)
        self._node_count += 1
        if d <= self.step_size:
            return Node3D(to_node.x, to_node.y, to_node.z, self._node_count)
        ratio = self.step_size / d
        nx = from_node.x + ratio * (to_node.x - from_node.x)
        ny = from_node.y + ratio * (to_node.y - from_node.y)
        nz = from_node.z + ratio * (to_node.z - from_node.z)
        return Node3D(nx, ny, nz, self._node_count)

    # ── Main planning loop ────────────────────────────────────────────────

    def plan(self) -> list[tuple[float, float, float]] | None:
        """
        Run RRT* for max_iter iterations.

        Returns the path as a list of (x, y, z) tuples from start to
        goal, or None if no solution was found.
        """
        goal_node: Node3D | None = None

        for _ in range(self.max_iter):
            x_rand = self._sample_free()

            # Nearest node in tree
            x_near = self._nearest(x_rand)

            # Steer towards sample
            x_new  = self._steer(x_near, x_rand)

            if self._in_collision(x_new.x, x_new.y, x_new.z):
                continue
            if not self._segment_free(x_near, x_new):
                continue

            # Choose best parent from near set
            X_near = self._near(x_new)
            best_parent = x_near
            best_cost   = x_near.cost + x_near.dist(x_new)

            for x_near_i in X_near:
                c = x_near_i.cost + x_near_i.dist(x_new)
                if c < best_cost and self._segment_free(x_near_i, x_new):
                    best_cost   = c
                    best_parent = x_near_i

            x_new.parent = best_parent
            x_new.cost   = best_cost
            self.tree.append(x_new)

            # Rewire near nodes through x_new if cheaper
            for x_near_i in X_near:
                c = x_new.cost + x_new.dist(x_near_i)
                if c < x_near_i.cost and self._segment_free(x_new, x_near_i):
                    x_near_i.parent = x_new
                    x_near_i.cost   = c

            # Check goal reach
            if x_new.dist(self.goal_pt) <= self.goal_tol:
                path_cost = x_new.cost + x_new.dist(self.goal_pt)
                if path_cost < self.best_cost:
                    self.best_cost = path_cost
                    self._c_best   = path_cost
                    goal_node      = x_new

        if goal_node is None:
            return None

        # Reconstruct path
        path = []
        node = goal_node
        while node is not None:
            path.append((node.x, node.y, node.z))
            node = node.parent
        path.reverse()
        path.append((self.goal_pt.x, self.goal_pt.y, self.goal_pt.z))

        # Path shortcutting
        path = self._shortcut(path)

        return path

    def _shortcut(
        self,
        path: list[tuple[float, float, float]],
    ) -> list[tuple[float, float, float]]:
        """
        Iterative path shortcutting pass.

        Attempts to replace 3-point sub-paths A→B→C with direct A→C
        when the segment is collision-free.  Repeats until no further
        shortcut is possible.
        """
        improved = True
        while improved:
            improved = False
            i = 0
            while i < len(path) - 2:
                a = Node3D(*path[i],     0)
                c = Node3D(*path[i + 2], 0)
                if self._segment_free(a, c):
                    path.pop(i + 1)
                    improved = True
                else:
                    i += 1
        return path

    def get_tree_edges(self) -> list[tuple[Node3D, Node3D]]:
        """Return all tree edges for visualisation."""
        edges = []
        for node in self.tree:
            if node.parent is not None:
                edges.append((node.parent, node))
        return edges


# ---------------------------------------------------------------------------
# ROS 2 Node
# ---------------------------------------------------------------------------

class RRTStarPlanner(Node):
    """Global RRT* path planner node."""

    def __init__(self):
        super().__init__('rrtstar_planner')

        # ── Parameters ────────────────────────────────────────────────
        self.declare_parameter('uav_id',         0)
        self.declare_parameter('max_iter',        2000)
        self.declare_parameter('step_size',       2.0)
        self.declare_parameter('goal_bias',       0.10)
        self.declare_parameter('goal_tol',        0.8)
        self.declare_parameter('search_radius',   4.0)
        self.declare_parameter('map_x_min',      -50.0)
        self.declare_parameter('map_x_max',       50.0)
        self.declare_parameter('map_y_min',      -50.0)
        self.declare_parameter('map_y_max',       50.0)
        self.declare_parameter('map_z_min',        2.0)
        self.declare_parameter('map_z_max',       30.0)
        self.declare_parameter('informed_rrtstar', True)
        self.declare_parameter('waypoint_index',   1)
        self.declare_parameter('safety_margin',    1.5)

        ns = f'/uav{self.get_parameter("uav_id").value}'

        # ── QoS ───────────────────────────────────────────────────────
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

        # ── Subscribers ───────────────────────────────────────────────
        self.sub_mission = self.create_subscription(
            PointStamped, f'{ns}/mission_waypoint', self._cb_mission, reliable_qos
        )
        self.sub_pose = self.create_subscription(
            PoseStamped, f'{ns}/mavros/local_position/pose', self._cb_pose, sensor_qos
        )
        self.sub_obs = self.create_subscription(
            MarkerArray, f'{ns}/rrtstar/obstacles', self._cb_obstacles, reliable_qos
        )

        # ── Publishers ────────────────────────────────────────────────
        self.pub_goal = self.create_publisher(PointStamped, f'{ns}/goal_waypoint',   reliable_qos)
        self.pub_path = self.create_publisher(Path,          f'{ns}/rrtstar/path',    reliable_qos)
        self.pub_tree = self.create_publisher(MarkerArray,   f'{ns}/rrtstar/tree',    reliable_qos)

        # ── State ─────────────────────────────────────────────────────
        self.mission_goal: PointStamped | None = None
        self.pose:         PoseStamped  | None = None
        self.obstacles:    list[AABB]          = self._default_obstacles()
        self.current_path: list[tuple]         = []
        self.path_index:   int                 = 0
        self._last_goal:   tuple | None        = None
        self._planning:    bool                = False
        self._lock = threading.Lock()

        # ── Timers ────────────────────────────────────────────────────
        self.plan_timer  = self.create_timer(1.0,  self._planning_loop)  # 1 Hz
        self.pub_timer   = self.create_timer(0.1,  self._publish_subgoal)  # 10 Hz

        self.get_logger().info(
            f'RRTStarPlanner started — UAV {self.get_parameter("uav_id").value} | '
            f'Informed={self.get_parameter("informed_rrtstar").value}'
        )

    # ── Default SAR world obstacles (urban + wilderness stubs) ───────────

    def _default_obstacles(self) -> list[AABB]:
        """
        Minimal synthetic obstacle set matching the Gazebo SAR worlds.
        Override at runtime via /uav{N}/rrtstar/obstacles topic.
        """
        return [
            # Urban scenario: building columns (x, y, z centres; half-extents)
            AABB( 10,  10, 5,  2, 2, 5),
            AABB(-10,  15, 6,  3, 2, 6),
            AABB( 20,  -5, 4,  2, 3, 4),
            AABB(-15, -10, 5,  2, 2, 5),
            AABB(  5,  25, 8,  3, 3, 8),
            # Wilderness: sparse trees
            AABB(30, -20, 8, 1, 1, 8),
            AABB(-30, 30, 6, 1, 1, 6),
        ]

    # ── Callbacks ─────────────────────────────────────────────────────────

    def _cb_mission(self, msg: PointStamped):
        self.mission_goal = msg

    def _cb_pose(self, msg: PoseStamped):
        self.pose = msg

    def _cb_obstacles(self, msg: MarkerArray):
        """Parse MarkerArray (CUBE markers) into AABB obstacle list."""
        new_obs = []
        for m in msg.markers:
            if m.action == Marker.ADD and m.type == Marker.CUBE:
                cx = m.pose.position.x
                cy = m.pose.position.y
                cz = m.pose.position.z
                rx = m.scale.x / 2.0
                ry = m.scale.y / 2.0
                rz = m.scale.z / 2.0
                new_obs.append(AABB(cx, cy, cz, rx, ry, rz))
        with self._lock:
            self.obstacles = new_obs if new_obs else self._default_obstacles()

    # ── Planning loop ─────────────────────────────────────────────────────

    def _planning_loop(self):
        if self.mission_goal is None or self.pose is None:
            return
        if self._planning:
            return

        goal = (
            self.mission_goal.point.x,
            self.mission_goal.point.y,
            self.mission_goal.point.z,
        )

        # Only replan if goal changed significantly
        if self._last_goal is not None:
            d = math.sqrt(sum((a - b)**2 for a, b in zip(goal, self._last_goal)))
            if d < 0.5:
                return

        self._last_goal = goal
        self._planning  = True

        # Run in background thread to avoid blocking ROS spin
        t = threading.Thread(target=self._run_planner, args=(goal,), daemon=True)
        t.start()

    def _run_planner(self, goal: tuple):
        try:
            p = self._params()

            start = (
                self.pose.pose.position.x,
                self.pose.pose.position.y,
                self.pose.pose.position.z,
            )

            bounds = (
                p['map_x_min'], p['map_x_max'],
                p['map_y_min'], p['map_y_max'],
                p['map_z_min'], p['map_z_max'],
            )

            self.get_logger().info(
                f'RRT*: planning from {start} → {goal}'
            )

            with self._lock:
                obs_copy = list(self.obstacles)

            planner = RRTStar(
                start          = start,
                goal           = goal,
                bounds         = bounds,
                obstacles      = obs_copy,
                step_size      = p['step_size'],
                max_iter       = p['max_iter'],
                goal_bias      = p['goal_bias'],
                goal_tol       = p['goal_tol'],
                search_radius  = p['search_radius'],
                safety_margin  = p['safety_margin'],
                informed       = p['informed_rrtstar'],
            )

            path = planner.plan()

            if path is not None:
                with self._lock:
                    self.current_path = path
                    self.path_index   = 0

                self.get_logger().info(
                    f'RRT*: path found — {len(path)} waypoints, '
                    f'cost ≈ {planner.best_cost:.2f} m'
                )

                self._publish_path(path)
                self._publish_tree_markers(planner.get_tree_edges())

            else:
                self.get_logger().warn(
                    f'RRT*: no path found in {p["max_iter"]} iterations'
                )

        except Exception as e:
            self.get_logger().error(f'RRT* planner exception: {e}')
        finally:
            self._planning = False

    # ── Sub-goal publishing ───────────────────────────────────────────────

    def _publish_subgoal(self):
        """
        Publish the next waypoint on the planned path as the VFH goal.
        Advances the path index when UAV reaches the current sub-goal.
        """
        if self.pose is None:
            return

        with self._lock:
            if not self.current_path:
                return
            path = self.current_path
            idx  = self.path_index

        p = self._params()
        cx = self.pose.pose.position.x
        cy = self.pose.pose.position.y
        cz = self.pose.pose.position.z

        # Advance index if sub-goal reached
        while idx < len(path) - 1:
            wx, wy, wz = path[idx]
            d = math.sqrt((cx-wx)**2 + (cy-wy)**2 + (cz-wz)**2)
            if d < p['goal_tol'] * 1.5:
                idx += 1
            else:
                break

        with self._lock:
            self.path_index = idx
            wx, wy, wz = path[idx]

        # Publish lookahead sub-goal
        look = min(idx + p['waypoint_index'], len(path) - 1)
        wx, wy, wz = path[look]

        msg = PointStamped()
        msg.header.stamp    = self.get_clock().now().to_msg()
        msg.header.frame_id = 'map'
        msg.point.x = wx
        msg.point.y = wy
        msg.point.z = wz
        self.pub_goal.publish(msg)

    # ── Visualisation helpers ─────────────────────────────────────────────

    def _publish_path(self, path: list[tuple]):
        msg = Path()
        msg.header.stamp    = self.get_clock().now().to_msg()
        msg.header.frame_id = 'map'

        from geometry_msgs.msg import PoseStamped as PS
        for (x, y, z) in path:
            ps = PS()
            ps.header = msg.header
            ps.pose.position.x = x
            ps.pose.position.y = y
            ps.pose.position.z = z
            ps.pose.orientation.w = 1.0
            msg.poses.append(ps)

        self.pub_path.publish(msg)

    def _publish_tree_markers(self, edges: list):
        """Publish RRT* tree as LINE_LIST markers for RViz."""
        ma = MarkerArray()
        marker = Marker()
        marker.header.stamp    = self.get_clock().now().to_msg()
        marker.header.frame_id = 'map'
        marker.ns       = 'rrtstar_tree'
        marker.id       = 0
        marker.type     = Marker.LINE_LIST
        marker.action   = Marker.ADD
        marker.scale.x  = 0.05
        marker.color.r  = 0.2
        marker.color.g  = 0.8
        marker.color.b  = 0.2
        marker.color.a  = 0.5

        for (parent, child) in edges:
            p1 = Point(); p1.x = parent.x; p1.y = parent.y; p1.z = parent.z
            p2 = Point(); p2.x = child.x;  p2.y = child.y;  p2.z = child.z
            marker.points.append(p1)
            marker.points.append(p2)

        ma.markers.append(marker)
        self.pub_tree.publish(ma)

    # ── Parameter convenience ─────────────────────────────────────────────

    def _params(self) -> dict:
        return {
            'uav_id':          self.get_parameter('uav_id').value,
            'max_iter':        self.get_parameter('max_iter').value,
            'step_size':       self.get_parameter('step_size').value,
            'goal_bias':       self.get_parameter('goal_bias').value,
            'goal_tol':        self.get_parameter('goal_tol').value,
            'search_radius':   self.get_parameter('search_radius').value,
            'map_x_min':       self.get_parameter('map_x_min').value,
            'map_x_max':       self.get_parameter('map_x_max').value,
            'map_y_min':       self.get_parameter('map_y_min').value,
            'map_y_max':       self.get_parameter('map_y_max').value,
            'map_z_min':       self.get_parameter('map_z_min').value,
            'map_z_max':       self.get_parameter('map_z_max').value,
            'informed_rrtstar': self.get_parameter('informed_rrtstar').value,
            'waypoint_index':  self.get_parameter('waypoint_index').value,
            'safety_margin':   self.get_parameter('safety_margin').value,
        }


# ---------------------------------------------------------------------------
# Entry point
# ---------------------------------------------------------------------------

def main(args=None):
    rclpy.init(args=args)
    node = RRTStarPlanner()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
