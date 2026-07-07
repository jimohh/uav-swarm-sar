#!/usr/bin/env python3
"""
experiment_runner.py
Runs a single experimental trial.
Called by run_experiments.sh for each of the 180 trials.

Usage:
  ros2 run sar_planning experiment_runner \
    --ros-args \
    -p trial_id:=0 \
    -p scenario:=urban \
    -p planner:=apf \
    -p trial_duration:=120.0
"""

import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from geometry_msgs.msg import PoseStamped
from mavros_msgs.srv import CommandBool, SetMode
import json
import time
import numpy as np


class ExperimentRunner(Node):

    def __init__(self):
        super().__init__('experiment_runner')

        # Parameters
        self.declare_parameter('trial_id', 0)
        self.declare_parameter('scenario', 'urban')
        self.declare_parameter('planner', 'apf')
        self.declare_parameter('trial_duration', 120)
        self.declare_parameter('num_uavs', 2)

        self.trial_id  = self.get_parameter('trial_id').value
        self.scenario  = self.get_parameter('scenario').value
        self.planner   = self.get_parameter('planner').value
        self.duration  = self.get_parameter('trial_duration').value
        self.num_uavs  = self.get_parameter('num_uavs').value

        # Trial state
        self.trial_started  = False
        self.trial_complete = False
        self.start_time     = None

        # Publishers
        self.pub_scenario = self.create_publisher(
            String, '/sar/experiment/scenario', 10)
        self.pub_planner = self.create_publisher(
            String, '/sar/experiment/planner', 10)
        self.pub_status = self.create_publisher(
            String, '/sar/experiment/status', 10)

        # Setpoint publishers for each UAV
        self.setpoint_pubs = {}
        for i in range(self.num_uavs):
            self.setpoint_pubs[i] = self.create_publisher(
                PoseStamped,
                f'/uav{i}/mavros/setpoint_position/local',
                10)

        # Timer to publish setpoints at 20Hz (required for OFFBOARD)
        self.setpoint_timer = self.create_timer(
            0.05, self._publish_setpoints)

        # Trial start timer
        self.create_timer(2.0, self._start_trial)

        # Trial end timer
        self.create_timer(
            self.duration + 2.0, self._end_trial)

        self.get_logger().info(
            f'ExperimentRunner: trial {self.trial_id}, '
            f'scenario={self.scenario}, planner={self.planner}, '
            f'duration={self.duration}s')

    def _publish_setpoints(self):
        """Publish setpoints to keep OFFBOARD mode active."""
        msg = PoseStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'map'
        msg.pose.position.z = 10.0

        for i, pub in self.setpoint_pubs.items():
            msg.pose.position.x = float(i * 5)
            pub.publish(msg)

    def _start_trial(self):
        """Broadcast trial start to all nodes."""
        if self.trial_started:
            return
        self.trial_started = True
        self.start_time = time.time()

        # Broadcast scenario and planner
        scenario_msg = String()
        scenario_msg.data = self.scenario
        self.pub_scenario.publish(scenario_msg)

        planner_msg = String()
        planner_msg.data = self.planner
        self.pub_planner.publish(planner_msg)

        status = {
            'status':    'started',
            'trial_id':  self.trial_id,
            'scenario':  self.scenario,
            'planner':   self.planner,
            'timestamp': time.time(),
        }
        msg = String()
        msg.data = json.dumps(status)
        self.pub_status.publish(msg)

        self.get_logger().info(
            f'Trial {self.trial_id} started — '
            f'{self.scenario}/{self.planner}')

    def _end_trial(self):
        """Signal trial completion."""
        if self.trial_complete:
            return
        self.trial_complete = True

        elapsed = time.time() - (self.start_time or time.time())

        status = {
            'status':    'complete',
            'trial_id':  self.trial_id,
            'scenario':  self.scenario,
            'planner':   self.planner,
            'duration':  round(elapsed, 2),
            'timestamp': time.time(),
        }
        msg = String()
        msg.data = json.dumps(status)
        self.pub_status.publish(msg)

        self.get_logger().info(
            f'Trial {self.trial_id} complete '
            f'({elapsed:.1f}s)')

        rclpy.shutdown()


def main(args=None):
    rclpy.init(args=args)
    node = ExperimentRunner()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, Exception):
        pass
    finally:
        node.destroy_node()


if __name__ == '__main__':
    main()