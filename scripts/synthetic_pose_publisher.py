#!/usr/bin/env python3
"""
synthetic_pose_publisher.py  (v2 — corrected per-planner topic contracts)

Standalone planner validation harness — bypasses PX4/Gazebo/MAVROS-arming.

CRITICAL: the three planners do NOT share a topic contract. Verified by
direct source inspection:

  Planner   Goal input                          Pose/odom input                          Velocity output
  -------   -----------------------------------  ----------------------------------------  ---------------------------------------------
  APF       /sar/waypoints      (PoseArray)       /uavN/mavros/local_position/odom (Odom)   /uavN/mavros/setpoint_velocity/cmd_vel_unstamped
  VFH+      /uavN/goal_waypoint (PointStamped)    /uavN/mavros/local_position/pose (Pose)    /uavN/vfh/cmd_vel   <-- NOT MAVROS directly
  RRT*      /uavN/mission_waypoint (PointStamped) /uavN/mavros/local_position/pose (Pose)    /uavN/goal_waypoint <-- feeds VFH+, not a velocity

Also confirmed: obstacle-avoidance behaviour is NOT meaningfully testable
via a fake /uavN/scan for any of the three planners:
  - APF's self.obstacles is always empty in the real system too (no lidar
    on x500/standard_vtol in Gazebo) -> repulsive force is always zero.
  - VFH+ degrades to open-space passthrough with no scan/obstacle_distance,
    which is its normal real-world behaviour here anyway.
  - RRT* does not subscribe to scan at all -- it collision-checks against
    a hardcoded _default_obstacles() AABB list baked into the code.
So obstacle differentiation for RRT* emerges automatically from its own
hardcoded obstacles, not from anything this harness injects. Scenarios
here vary start/goal geometry instead, which is what's actually testable.

Because of the differing contracts, this script runs in one of three
modes (--mode apf | vfh | rrtstar), each wiring up only the topics that
mode's planner actually uses, and closing the loop on that planner's own
real output topic.

For the "full_hierarchy" condition (RRT* + VFH+ together), the runner
script runs --mode vfh (since VFH+'s cmd_vel is what actually results
in motion) while RRT* runs upstream feeding it real goal_waypoint
messages -- RRT*'s own output is separately captured by a --mode rrtstar
run for latency/output confirmation.

Usage
-----
    python3 synthetic_pose_publisher.py --uav_id 0 --mode apf      --scenario direct
    python3 synthetic_pose_publisher.py --uav_id 0 --mode vfh      --scenario direct
    python3 synthetic_pose_publisher.py --uav_id 0 --mode rrtstar  --scenario direct
"""

import argparse
import csv
import math
import os
import time
from datetime import datetime

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy

from geometry_msgs.msg import PoseStamped, PoseArray, Pose, PointStamped, Twist
from nav_msgs.msg import Odometry


# ---------------------------------------------------------------------------
# Scenarios — vary start/goal geometry only (no fake obstacles; see header)
# ---------------------------------------------------------------------------

SCENARIOS = {
    "direct": {
        "start": (0.0, 0.0, 15.0),
        "goal": (60.0, 0.0, 15.0),
    },
    "diagonal": {
        "start": (0.0, 0.0, 15.0),
        "goal": (50.0, 50.0, 20.0),
    },
    "long_range": {
        "start": (0.0, 0.0, 15.0),
        "goal": (120.0, -30.0, 25.0),
    },
}

GOAL_TOLERANCE_M = 3.0
MAX_RUN_TIME_S = 90.0
MAX_SPEED = 8.0
MAX_ACCEL = 4.0
DRAG = 0.05
TICK_HZ = 20.0


def log(uav_id: int, mode: str, msg: str) -> None:
    ts = time.strftime("%H:%M:%S")
    print(f"[{ts}] [uav{uav_id}][{mode}] {msg}", flush=True)


class SyntheticPosePublisher(Node):
    def __init__(self, uav_id: int, mode: str, scenario_name: str, results_dir: str):
        super().__init__(f'synthetic_pose_publisher_uav{uav_id}_{mode}')

        if scenario_name not in SCENARIOS:
            raise ValueError(f"Unknown scenario '{scenario_name}', choices: {list(SCENARIOS)}")
        if mode not in ("apf", "vfh", "rrtstar"):
            raise ValueError(f"Unknown mode '{mode}', choices: apf, vfh, rrtstar")

        self.uav_id = uav_id
        self.mode = mode
        self.scenario_name = scenario_name
        self.scenario = SCENARIOS[scenario_name]
        self.results_dir = results_dir

        ns = f'/uav{uav_id}'

        sensor_qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST, depth=1)
        reliable_qos = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            history=HistoryPolicy.KEEP_LAST, depth=10)

        # -- Pose/odom publishers (always both, harmless if unused) --------
        self.pub_odom = self.create_publisher(Odometry, f'{ns}/mavros/local_position/odom', sensor_qos)
        self.pub_pose = self.create_publisher(PoseStamped, f'{ns}/mavros/local_position/pose', sensor_qos)

        # -- Goal publishers + cmd_vel subscriber, per-mode -----------------
        self.pub_goal_posearray = None
        self.pub_goal_point = None
        self.goal_sent = False

        if mode == "apf":
            self.pub_goal_posearray = self.create_publisher(PoseArray, '/sar/waypoints', reliable_qos)
            self.sub_cmd_vel = self.create_subscription(
                Twist, f'{ns}/mavros/setpoint_velocity/cmd_vel_unstamped',
                self._cb_cmd_vel, reliable_qos)
        elif mode == "vfh":
            self.pub_goal_point = self.create_publisher(PointStamped, f'{ns}/goal_waypoint', reliable_qos)
            self.sub_cmd_vel = self.create_subscription(
                Twist, f'{ns}/vfh/cmd_vel',
                self._cb_cmd_vel, reliable_qos)
        elif mode == "rrtstar":
            self.pub_goal_point = self.create_publisher(PointStamped, f'{ns}/mission_waypoint', reliable_qos)
            # RRT* doesn't output velocity -- it outputs a waypoint that
            # feeds VFH+. We don't integrate physics from this; instead
            # we just confirm it produces a goal_waypoint and log latency.
            self.sub_rrt_output = self.create_subscription(
                PointStamped, f'{ns}/goal_waypoint',
                self._cb_rrt_output, reliable_qos)

        # -- State -----------------------------------------------------------
        sx, sy, sz = self.scenario["start"]
        self.pos = [sx, sy, sz]
        self.vel = [0.0, 0.0, 0.0]
        self.last_cmd = [0.0, 0.0, 0.0]
        self.heading_history = []
        self.trajectory = []
        self.start_time = time.monotonic()
        self.goal_reached = False
        self.goal_reached_time = None
        self.rrt_first_output_time = None
        self.rrt_output_received = False

        gx, gy, gz = self.scenario["goal"]
        self.goal = (gx, gy, gz)

        self.tick_timer = self.create_timer(1.0 / TICK_HZ, self._tick)

        log(uav_id, mode, f"scenario='{scenario_name}' start={self.pos} goal={self.goal}")

    # -- Callbacks -----------------------------------------------------------

    def _cb_cmd_vel(self, msg: Twist):
        self.last_cmd = [msg.linear.x, msg.linear.y, msg.linear.z]

    def _cb_rrt_output(self, msg: PointStamped):
        if not self.rrt_output_received:
            self.rrt_output_received = True
            self.rrt_first_output_time = time.monotonic() - self.start_time
            log(self.uav_id, self.mode,
                f"RRT* produced first goal_waypoint at t={self.rrt_first_output_time:.2f}s: "
                f"({msg.point.x:.1f},{msg.point.y:.1f},{msg.point.z:.1f})")

    # -- Physics -----------------------------------------------------------

    def _integrate(self, dt: float):
        for i in range(3):
            desired = max(-MAX_SPEED, min(MAX_SPEED, self.last_cmd[i]))
            delta = desired - self.vel[i]
            max_delta = MAX_ACCEL * dt
            delta = max(-max_delta, min(max_delta, delta))
            self.vel[i] += delta
            self.vel[i] *= (1.0 - DRAG * dt)
        for i in range(3):
            self.pos[i] += self.vel[i] * dt

    # -- Main tick -----------------------------------------------------------

    def _tick(self):
        elapsed = time.monotonic() - self.start_time
        dt = 1.0 / TICK_HZ
        now = self.get_clock().now().to_msg()

        if not self.goal_sent and elapsed > 2.0:
            self._send_goal()
            self.goal_sent = True

        if self.mode != "rrtstar":
            self._integrate(dt)

        odom = Odometry()
        odom.header.stamp = now
        odom.header.frame_id = 'map'
        odom.child_frame_id = f'uav{self.uav_id}/base_link'
        odom.pose.pose.position.x = self.pos[0]
        odom.pose.pose.position.y = self.pos[1]
        odom.pose.pose.position.z = self.pos[2]
        odom.twist.twist.linear.x = self.vel[0]
        odom.twist.twist.linear.y = self.vel[1]
        odom.twist.twist.linear.z = self.vel[2]
        self.pub_odom.publish(odom)

        pose = PoseStamped()
        pose.header.stamp = now
        pose.header.frame_id = 'map'
        pose.pose.position.x = self.pos[0]
        pose.pose.position.y = self.pos[1]
        pose.pose.position.z = self.pos[2]
        self.pub_pose.publish(pose)

        if abs(self.vel[0]) > 0.05 or abs(self.vel[1]) > 0.05:
            heading = math.atan2(self.vel[1], self.vel[0])
            self.heading_history.append(heading)

        self.trajectory.append((elapsed, self.pos[0], self.pos[1], self.pos[2],
                                 self.vel[0], self.vel[1], self.vel[2]))

        dist_to_goal = math.dist(self.pos, self.goal)
        if not self.goal_reached and self.mode != "rrtstar" and dist_to_goal < GOAL_TOLERANCE_M:
            self.goal_reached = True
            self.goal_reached_time = elapsed
            log(self.uav_id, self.mode, f"GOAL REACHED at t={elapsed:.1f}s")

        if self.mode == "rrtstar":
            done = self.rrt_output_received or elapsed > MAX_RUN_TIME_S
        else:
            done = self.goal_reached or elapsed > MAX_RUN_TIME_S

        if done:
            self._finalize(elapsed)
            self.tick_timer.cancel()
            rclpy.shutdown()

    def _send_goal(self):
        gx, gy, gz = self.goal
        if self.mode == "apf":
            pa = PoseArray()
            pa.header.frame_id = 'map'
            pa.header.stamp = self.get_clock().now().to_msg()
            p = Pose()
            p.position.x, p.position.y, p.position.z = gx, gy, gz
            pa.poses = [p]
            self.pub_goal_posearray.publish(pa)
            log(self.uav_id, self.mode, "published goal PoseArray to /sar/waypoints")
        else:
            pt = PointStamped()
            pt.header.frame_id = 'map'
            pt.header.stamp = self.get_clock().now().to_msg()
            pt.point.x, pt.point.y, pt.point.z = gx, gy, gz
            self.pub_goal_point.publish(pt)
            topic = f'/uav{self.uav_id}/goal_waypoint' if self.mode == 'vfh' else f'/uav{self.uav_id}/mission_waypoint'
            log(self.uav_id, self.mode, f"published goal PointStamped to {topic}")

    # -- Finalize / metrics ---------------------------------------------------

    def _finalize(self, elapsed: float):
        os.makedirs(self.results_dir, exist_ok=True)

        path_length = 0.0
        for i in range(1, len(self.trajectory)):
            _, x0, y0, z0, *_ = self.trajectory[i - 1]
            _, x1, y1, z1, *_ = self.trajectory[i]
            path_length += math.dist((x0, y0, z0), (x1, y1, z1))

        smoothness = 0.0
        if len(self.heading_history) > 1:
            diffs = []
            for i in range(1, len(self.heading_history)):
                d = self.heading_history[i] - self.heading_history[i - 1]
                d = (d + math.pi) % (2 * math.pi) - math.pi
                diffs.append(d)
            smoothness = sum(abs(d) for d in diffs) / len(diffs)

        time_to_goal = self.goal_reached_time if self.goal_reached else None

        ts_str = datetime.now().strftime('%Y%m%d_%H%M%S')
        summary_path = os.path.join(
            self.results_dir, f"{self.mode}_{self.scenario_name}_uav{self.uav_id}_{ts_str}_summary.csv")
        traj_path = os.path.join(
            self.results_dir, f"{self.mode}_{self.scenario_name}_uav{self.uav_id}_{ts_str}_trajectory.csv")

        with open(summary_path, 'w', newline='') as f:
            w = csv.writer(f)
            w.writerow(["mode", "scenario", "uav_id", "goal_reached", "time_to_goal_s",
                        "path_length_m", "path_smoothness_rad",
                        "rrt_output_received", "rrt_first_output_time_s", "run_duration_s"])
            w.writerow([self.mode, self.scenario_name, self.uav_id, self.goal_reached,
                        time_to_goal, round(path_length, 3), round(smoothness, 4),
                        self.rrt_output_received, self.rrt_first_output_time,
                        round(elapsed, 2)])

        with open(traj_path, 'w', newline='') as f:
            w = csv.writer(f)
            w.writerow(["t_s", "x", "y", "z", "vx", "vy", "vz"])
            for row in self.trajectory:
                w.writerow(row)

        log(self.uav_id, self.mode, f"summary written: {summary_path}")
        log(self.uav_id, self.mode,
            f"path_length={path_length:.2f}m smoothness={smoothness:.4f}rad "
            f"goal_reached={self.goal_reached} rrt_output_received={self.rrt_output_received}")


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--uav_id", type=int, required=True)
    parser.add_argument("--mode", type=str, required=True, choices=["apf", "vfh", "rrtstar"])
    parser.add_argument("--scenario", type=str, required=True, choices=list(SCENARIOS))
    parser.add_argument("--results_dir", type=str,
                         default=os.path.expanduser("~/thesis_ws/results/synthetic"))
    args = parser.parse_args()

    rclpy.init()
    node = SyntheticPosePublisher(args.uav_id, args.mode, args.scenario, args.results_dir)
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if rclpy.ok():
            node.destroy_node()
            rclpy.shutdown()


if __name__ == "__main__":
    main()