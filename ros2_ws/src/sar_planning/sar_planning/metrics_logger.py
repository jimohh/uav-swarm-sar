#!/usr/bin/env python3
"""
metrics_logger.py
Logs five metrics per trial to CSV:
  1. coverage_rate        — fraction of search area covered
  2. time_to_detection    — time from start to first confirmed detection
  3. path_efficiency      — ratio of optimal to actual path length
  4. inter_agent_distance — mean distance between active UAVs
  5. fault_recovery_time  — time to recover from injected fault (if any)
"""

import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from nav_msgs.msg import Odometry
import csv
import os
import time
import json
import numpy as np


class MetricsLogger(Node):

    def __init__(self):
        super().__init__('metrics_logger')

        # Parameters
        self.declare_parameter('trial_id', 0)
        self.declare_parameter('scenario', 'urban')
        self.declare_parameter('planner', 'apf')
        self.declare_parameter('trial_duration', 120)  # seconds
        self.declare_parameter('results_dir',
            '/root/thesis_ws/results')

        self.trial_id      = self.get_parameter('trial_id').value
        self.scenario      = self.get_parameter('scenario').value
        self.planner       = self.get_parameter('planner').value
        self.duration      = int(self.get_parameter('trial_duration').value)
        self.results_dir   = self.get_parameter('results_dir').value

        # Metrics state
        self.start_time            = time.time()
        self.first_detection_time  = None
        self.covered_cells         = set()
        self.total_cells           = 100 * 100
        self.uav_positions         = {}
        self.path_lengths          = {0: 0.0, 1: 0.0, 2: 0.0}
        self.prev_positions        = {}
        self.fault_start_time      = None
        self.fault_recovery_time   = None
        self.trial_complete        = False

        # Results file
        os.makedirs(f'{self.results_dir}/{self.scenario}', exist_ok=True)
        self.csv_path = (
            f'{self.results_dir}/{self.scenario}/'
            f'{self.planner}_results.csv'
        )
        self._init_csv()

        # Subscribers
        self.create_subscription(
            String, '/sar/detections/confirmed',
            self._detection_callback, 10)

        self.create_subscription(
            String, '/sar/fault/alert',
            self._fault_alert_callback, 10)

        self.create_subscription(
            String, '/sar/fault/recovery',
            self._fault_recovery_callback, 10)

        for i in range(3):
            self.create_subscription(
                Odometry,
                f'/uav{i}/mavros/local_position/odom',
                lambda msg, uid=i: self._odom_callback(msg, uid),
                10)

        # Trial timer
        self.create_timer(1.0, self._update_metrics)
        self.create_timer(
            self.duration, self._end_trial)

        self.get_logger().info(
            f'MetricsLogger started — trial {self.trial_id}, '
            f'scenario: {self.scenario}, planner: {self.planner}')

    def _init_csv(self):
        """Create CSV with headers if it doesn't exist."""
        if not os.path.exists(self.csv_path):
            with open(self.csv_path, 'w', newline='') as f:
                writer = csv.writer(f)
                writer.writerow([
                    'trial_id', 'scenario', 'planner',
                    'coverage_rate', 'time_to_detection',
                    'path_efficiency', 'inter_agent_distance',
                    'fault_recovery_time', 'trial_duration',
                    'timestamp'
                ])

    def _odom_callback(self, msg, uav_id):
        """Track UAV positions and path lengths."""
        p = msg.pose.pose.position
        pos = np.array([p.x, p.y, p.z])
        self.uav_positions[uav_id] = pos

        # Update path length
        if uav_id in self.prev_positions:
            dist = np.linalg.norm(pos - self.prev_positions[uav_id])
            self.path_lengths[uav_id] += dist

        self.prev_positions[uav_id] = pos.copy()

        # Update coverage
        cell = (int(p.x), int(p.y))
        self.covered_cells.add(cell)

    def _detection_callback(self, msg):
        """Record time to first detection."""
        if self.first_detection_time is None:
            self.first_detection_time = time.time() - self.start_time
            self.get_logger().info(
                f'First detection at {self.first_detection_time:.2f}s')

    def _fault_alert_callback(self, msg):
        """Record fault injection time."""
        if self.fault_start_time is None:
            self.fault_start_time = time.time()

    def _fault_recovery_callback(self, msg):
        """Record fault recovery time."""
        if (self.fault_start_time is not None and
                self.fault_recovery_time is None):
            self.fault_recovery_time = (
                time.time() - self.fault_start_time)
            self.get_logger().info(
                f'Fault recovered in {self.fault_recovery_time:.2f}s')

    def _update_metrics(self):
        """Periodic metrics update."""
        elapsed = time.time() - self.start_time
        coverage = len(self.covered_cells) / self.total_cells
        self.get_logger().debug(
            f'Trial {self.trial_id}: t={elapsed:.0f}s '
            f'coverage={coverage:.3f}')

    def _compute_inter_agent_distance(self):
        """Mean pairwise distance between UAVs."""
        positions = list(self.uav_positions.values())
        if len(positions) < 2:
            return 0.0
        distances = []
        for i in range(len(positions)):
            for j in range(i+1, len(positions)):
                distances.append(
                    np.linalg.norm(positions[i] - positions[j]))
        return float(np.mean(distances)) if distances else 0.0

    def _compute_path_efficiency(self):
        """
        Path efficiency = straight-line distance to coverage centre
        divided by actual path length.
        """
        total_actual = sum(self.path_lengths.values())
        if total_actual < 1e-6:
            return 0.0
        # Optimal path approximation
        coverage_area = len(self.covered_cells)
        optimal = np.sqrt(coverage_area) * 1.0
        return min(1.0, optimal / total_actual)

    def _end_trial(self):
        """Save trial results to CSV."""
        if self.trial_complete:
            return
        self.trial_complete = True

        elapsed = time.time() - self.start_time
        coverage = len(self.covered_cells) / self.total_cells

        row = [
            self.trial_id,
            self.scenario,
            self.planner,
            round(coverage, 4),
            round(self.first_detection_time, 2)
                if self.first_detection_time else -1,
            round(self._compute_path_efficiency(), 4),
            round(self._compute_inter_agent_distance(), 2),
            round(self.fault_recovery_time, 2)
                if self.fault_recovery_time else -1,
            round(elapsed, 2),
            time.strftime('%Y-%m-%d %H:%M:%S'),
        ]

        with open(self.csv_path, 'a', newline='') as f:
            csv.writer(f).writerow(row)

        self.get_logger().info(
            f'Trial {self.trial_id} complete — '
            f'coverage={coverage:.3f}, '
            f'ttd={self.first_detection_time:.2f}s '
            if self.first_detection_time else
            f'Trial {self.trial_id} complete — '
            f'coverage={coverage:.3f}, ttd=None')

        rclpy.shutdown()


def main(args=None):
    rclpy.init(args=args)
    node = MetricsLogger()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, Exception):
        pass
    finally:
        if not node.trial_complete:
            node._end_trial()
        node.destroy_node()


if __name__ == '__main__':
    main()