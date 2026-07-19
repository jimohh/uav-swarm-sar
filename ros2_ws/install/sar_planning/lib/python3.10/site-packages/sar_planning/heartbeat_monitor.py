#!/usr/bin/env python3
"""
heartbeat_monitor.py
Fault tolerance via heartbeat timeout monitoring.
Monitors all UAV agents and triggers fault recovery
when heartbeat timeout is exceeded.

Topics:
  Subscribes: /uavN/mavros/state     — UAV connection status
              /sar/cnp/task_award    — track task assignments
  Publishes:  /sar/fault/alert       — fault alerts
              /sar/fault/recovery    — recovery commands
"""

import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from mavros_msgs.msg import State
import json
import time


class HeartbeatMonitor(Node):

    def __init__(self):
        super().__init__('heartbeat_monitor')

        # Parameters
        self.declare_parameter('num_uavs', 3)
        self.declare_parameter('heartbeat_timeout', 5.0)
        self.declare_parameter('check_rate', 1.0)

        self.num_uavs   = self.get_parameter('num_uavs').value
        self.hb_timeout = self.get_parameter('heartbeat_timeout').value
        check_rate      = self.get_parameter('check_rate').value

        # UAV state tracking
        self.uav_states = {
            i: {
                'connected':        False,
                'last_heartbeat':   0.0,
                'fault_detected':   False,
                'current_task':     None,
                'recovery_attempts': 0,
            }
            for i in range(self.num_uavs)
        }

        self.fault_history = []

        # --- Publishers ---
        self.pub_alert = self.create_publisher(
            String, '/sar/fault/alert', 10)
        self.pub_recovery = self.create_publisher(
            String, '/sar/fault/recovery', 10)

        # --- Subscribers ---
        for i in range(self.num_uavs):
            self.create_subscription(
                State,
                f'/uav{i}/mavros/state',
                lambda msg, uid=i: self._state_callback(msg, uid),
                10)

        self.create_subscription(
            String, '/sar/cnp/task_award',
            self._task_award_callback, 10)

        # Monitor timer
        self.create_timer(1.0 / check_rate, self._check_heartbeats)

        self.get_logger().info(
            f'HeartbeatMonitor started — '
            f'{self.num_uavs} UAVs, timeout: {self.hb_timeout}s')

    # ------------------------------------------------------------------
    # Callbacks
    # ------------------------------------------------------------------
    def _state_callback(self, msg, uav_id):
        """Update UAV heartbeat timestamp."""
        state = self.uav_states[uav_id]
        state['connected']      = msg.connected
        state['last_heartbeat'] = time.time()

        # Clear fault if reconnected
        if msg.connected and state['fault_detected']:
            state['fault_detected']    = False
            state['recovery_attempts'] = 0
            self.get_logger().info(
                f'UAV{uav_id} fault cleared — reconnected')
            self._publish_recovery(uav_id, 'reconnected')

    def _task_award_callback(self, msg):
        """Track which UAV has which task."""
        try:
            award = json.loads(msg.data)
            uav_id  = award['awarded_to']
            task_id = award['task_id']
            if uav_id in self.uav_states:
                self.uav_states[uav_id]['current_task'] = task_id
        except (json.JSONDecodeError, KeyError):
            pass

    # ------------------------------------------------------------------
    # Heartbeat monitoring
    # ------------------------------------------------------------------
    def _check_heartbeats(self):
        """Check all UAVs for heartbeat timeout."""
        now = time.time()

        for uav_id, state in self.uav_states.items():
            if state['last_heartbeat'] == 0.0:
                continue  # Never received heartbeat yet

            time_since_hb = now - state['last_heartbeat']

            if time_since_hb > self.hb_timeout:
                if not state['fault_detected']:
                    # New fault detected
                    state['fault_detected'] = True
                    state['recovery_attempts'] += 1

                    self.get_logger().warn(
                        f'FAULT: UAV{uav_id} heartbeat timeout '
                        f'({time_since_hb:.1f}s > {self.hb_timeout}s)')

                    self._publish_alert(uav_id, time_since_hb)
                    self._trigger_recovery(uav_id, state)

    def _publish_alert(self, uav_id, time_since_hb):
        """Publish fault alert."""
        alert = {
            'uav_id':         uav_id,
            'fault_type':     'heartbeat_timeout',
            'time_since_hb':  round(time_since_hb, 2),
            'timestamp':      time.time(),
            'current_task':   self.uav_states[uav_id]['current_task'],
        }

        self.fault_history.append(alert)

        msg = String()
        msg.data = json.dumps(alert)
        self.pub_alert.publish(msg)

    def _trigger_recovery(self, uav_id, state):
        """Trigger fault recovery procedure."""
        recovery_action = self._determine_recovery(uav_id, state)

        self.get_logger().info(
            f'Recovery for UAV{uav_id}: {recovery_action}')

        self._publish_recovery(uav_id, recovery_action)

    def _determine_recovery(self, uav_id, state):
        """Determine appropriate recovery action."""
        attempts = state['recovery_attempts']

        if attempts == 1:
            return 'reassign_task'     # reassign task to other UAV
        elif attempts == 2:
            return 'return_to_base'    # command UAV to RTL
        else:
            return 'remove_from_swarm' # exclude from task allocation

    def _publish_recovery(self, uav_id, action):
        """Publish recovery command."""
        recovery = {
            'uav_id':    uav_id,
            'action':    action,
            'timestamp': time.time(),
            'task_to_reassign': self.uav_states[uav_id]['current_task'],
        }

        msg = String()
        msg.data = json.dumps(recovery)
        self.pub_recovery.publish(msg)

    # ------------------------------------------------------------------
    # UAV bidding node
    # ------------------------------------------------------------------


class UAVBidder(Node):
    """
    Each UAV runs this node to participate in CNP auctions.
    Listens for task announcements and submits bids based on
    proximity, capability, and current workload.
    """

    def __init__(self, uav_id, uav_type='quadrotor'):
        super().__init__(f'uav_bidder_{uav_id}')

        self.uav_id   = uav_id
        self.uav_type = uav_type
        self.position = None
        self.current_task = None

        # Subscribe to task announcements
        self.create_subscription(
            String, '/sar/cnp/task_announcement',
            self._announcement_callback, 10)

        # Subscribe to own odometry
        from nav_msgs.msg import Odometry
        self.create_subscription(
            Odometry,
            f'/uav{uav_id}/mavros/local_position/odom',
            self._odom_callback, 10)

        # Subscribe to task awards to track own assignments
        self.create_subscription(
            String, '/sar/cnp/task_award',
            self._award_callback, 10)

        # Publisher for bids
        self.pub_bid = self.create_publisher(
            String, '/sar/cnp/bid', 10)

        self.get_logger().info(
            f'UAVBidder {uav_id} started ({uav_type})')

    def _odom_callback(self, msg):
        p = msg.pose.pose.position
        self.position = [p.x, p.y, p.z]

    def _announcement_callback(self, msg):
        """Evaluate task and submit bid if capable."""
        try:
            task = json.loads(msg.data)
            required = task.get('required_capability', 'any')

            # Check capability match
            if required != 'any' and required != self.uav_type:
                return  # Not capable of this task type

            # Don't bid if already assigned a task
            if self.current_task is not None:
                return

            # Calculate bid score
            score = self._calculate_bid(task)

            bid = {
                'task_id':  task['task_id'],
                'uav_id':   self.uav_id,
                'uav_type': self.uav_type,
                'score':    score,
                'position': self.position or [0, 0, 0],
            }

            msg_out = String()
            msg_out.data = json.dumps(bid)
            self.pub_bid.publish(msg_out)

            self.get_logger().debug(
                f'Bid submitted: {task["task_id"]} score={score:.3f}')

        except (json.JSONDecodeError, KeyError) as e:
            self.get_logger().warn(f'Announcement error: {e}')

    def _calculate_bid(self, task):
        """
        Calculate bid score based on:
        - Proximity to task (closer = higher score)
        - Task priority
        - UAV capability match
        """
        import numpy as np

        task_pos = np.array(task['position'])
        my_pos   = np.array(self.position or [0, 0, 0])

        dist = np.linalg.norm(task_pos - my_pos)

        # Proximity score (inverse distance, normalised)
        proximity_score = 1.0 / (1.0 + dist / 50.0)

        # Priority weighting
        priority_score = task.get('priority', 0.5)

        # Capability bonus
        required = task.get('required_capability', 'any')
        capability_bonus = 0.2 if required == self.uav_type else 0.0

        score = (0.5 * proximity_score +
                 0.3 * priority_score +
                 0.2 * capability_bonus)

        return round(score, 4)

    def _award_callback(self, msg):
        """Track task awards."""
        try:
            award = json.loads(msg.data)
            if award['awarded_to'] == self.uav_id:
                self.current_task = award['task_id']
                self.get_logger().info(
                    f'Task awarded to me: {award["task_id"]}')
        except (json.JSONDecodeError, KeyError):
            pass


def main(args=None):
    rclpy.init(args=args)
    node = HeartbeatMonitor()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()