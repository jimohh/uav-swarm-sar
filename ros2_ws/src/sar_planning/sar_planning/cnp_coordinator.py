#!/usr/bin/env python3
"""
cnp_coordinator.py
Auction-based Contract Net Protocol coordinator.
Implements decentralised task allocation for heterogeneous UAV swarm.
- Ground station coordinator broadcasts task announcements
- UAVs bid based on proximity, battery, and capability
- Coordinator awards tasks to winning bidders
- Supports heterogeneous tasks (quadrotor vs fixed-wing)

Topics:
  Publishes:  /sar/cnp/task_announcement  — broadcast task to all UAVs
              /sar/cnp/task_award         — award task to winning UAV
              /sar/cnp/swarm_status       — overall swarm status
  Subscribes: /sar/cnp/bid               — bids from UAVs
              /uavN/mavros/state          — UAV connection status
              /sar/detections/confirmed   — triggers new task allocation
"""

import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from mavros_msgs.msg import State
import json
import time
import numpy as np
from enum import Enum


class UAVType(Enum):
    QUADROTOR  = 'quadrotor'
    FIXED_WING = 'fixed_wing'


class UAVAgent:
    """Represents a UAV agent in the swarm."""
    def __init__(self, uav_id, uav_type):
        self.uav_id    = uav_id
        self.uav_type  = uav_type
        self.connected = False
        self.position  = np.zeros(3)
        self.last_heartbeat = 0.0
        self.current_task   = None
        self.bid_history    = []


class Task:
    """Represents a SAR task to be allocated."""
    def __init__(self, task_id, task_type, position, priority):
        self.task_id   = task_id
        self.task_type = task_type   # 'search', 'verify', 'track'
        self.position  = position
        self.priority  = priority
        self.assigned_to = None
        self.created_at  = time.time()
        self.bids        = {}


class CNPCoordinator(Node):

    def __init__(self):
        super().__init__('cnp_coordinator')

        # Parameters
        self.declare_parameter('num_uavs', 3)
        self.declare_parameter('scenario', 'urban')
        self.declare_parameter('auction_timeout', 2.0)
        self.declare_parameter('heartbeat_timeout', 5.0)

        self.num_uavs         = self.get_parameter('num_uavs').value
        self.scenario         = self.get_parameter('scenario').value
        self.auction_timeout  = self.get_parameter('auction_timeout').value
        self.hb_timeout       = self.get_parameter('heartbeat_timeout').value

        # UAV registry — heterogeneous swarm
        self.agents = {
            0: UAVAgent(0, UAVType.QUADROTOR),
            1: UAVAgent(1, UAVType.QUADROTOR),
            2: UAVAgent(2, UAVType.FIXED_WING),
        }

        # Task queue
        self.pending_tasks  = {}
        self.active_tasks   = {}
        self.completed_tasks = []
        self.task_counter   = 0

        # Auction state
        self.open_auctions = {}   # task_id -> Task

        # --- Publishers ---
        self.pub_announcement = self.create_publisher(
            String, '/sar/cnp/task_announcement', 10)
        self.pub_award = self.create_publisher(
            String, '/sar/cnp/task_award', 10)
        self.pub_status = self.create_publisher(
            String, '/sar/cnp/swarm_status', 10)

        # --- Subscribers ---
        self.create_subscription(
            String, '/sar/cnp/bid',
            self._bid_callback, 10)

        self.create_subscription(
            String, '/sar/detections/confirmed',
            self._detection_callback, 10)

        # Subscribe to each UAV's MAVROS state
        for i in range(self.num_uavs):
            self.create_subscription(
                State,
                f'/uav{i}/mavros/state',
                lambda msg, uid=i: self._state_callback(msg, uid),
                10)

        # Timers
        self.create_timer(1.0, self._status_publisher)
        self.create_timer(0.5, self._auction_manager)
        self.create_timer(10.0, self._generate_search_tasks)

        self.get_logger().info(
            f'CNPCoordinator started — {self.num_uavs} UAVs, '
            f'scenario: {self.scenario}')

        # Generate initial tasks
        self._generate_search_tasks()

    # ------------------------------------------------------------------
    # Task generation
    # ------------------------------------------------------------------
    def _generate_search_tasks(self):
        """Generate search tasks based on probability map hotspots."""
        # Simulated high-probability search areas per scenario
        search_areas = {
            'urban': [
                (30.0, 40.0, 0.0, 'search', 0.9),
                (60.0, 70.0, 0.0, 'search', 0.75),
                (50.0, 20.0, 0.0, 'verify', 0.6),
            ],
            'wilderness': [
                (50.0, 50.0, 0.0, 'search', 0.85),
                (30.0, 60.0, 0.0, 'search', 0.7),
                (70.0, 40.0, 0.0, 'track',  0.65),
            ],
            'maritime': [
                (40.0, 50.0, 0.0, 'search', 0.9),
                (55.0, 35.0, 0.0, 'verify', 0.8),
                (45.0, 65.0, 0.0, 'search', 0.7),
            ],
        }

        areas = search_areas.get(self.scenario, search_areas['urban'])

        for x, y, z, task_type, priority in areas:
            # Only add if not already in pending/active
            pos = np.array([x, y, z])
            already_exists = any(
                np.linalg.norm(t.position - pos) < 5.0
                for t in list(self.pending_tasks.values()) +
                         list(self.active_tasks.values())
            )
            if not already_exists:
                self._create_task(task_type, pos, priority)

    def _create_task(self, task_type, position, priority):
        """Create and announce a new task."""
        task_id = f'task_{self.task_counter:04d}'
        self.task_counter += 1

        task = Task(task_id, task_type, position, priority)
        self.pending_tasks[task_id] = task

        # Announce task for bidding
        announcement = {
            'task_id':   task_id,
            'task_type': task_type,
            'position':  position.tolist(),
            'priority':  priority,
            'deadline':  time.time() + self.auction_timeout,
            'required_capability': self._required_capability(task_type),
        }

        msg = String()
        msg.data = json.dumps(announcement)
        self.pub_announcement.publish(msg)

        self.open_auctions[task_id] = task

        self.get_logger().info(
            f'Task announced: {task_id} ({task_type}) '
            f'at {position[:2]} priority={priority:.2f}')

    def _required_capability(self, task_type):
        """Determine which UAV types can perform this task."""
        if task_type == 'track':
            return 'fixed_wing'   # fixed-wing better for tracking
        elif task_type == 'verify':
            return 'quadrotor'    # quadrotor better for close inspection
        else:
            return 'any'          # search tasks for any UAV

    # ------------------------------------------------------------------
    # Auction management
    # ------------------------------------------------------------------
    def _auction_manager(self):
        """Check open auctions and award tasks when timeout reached."""
        now = time.time()
        to_close = []

        for task_id, task in self.open_auctions.items():
            deadline = task.created_at + self.auction_timeout

            if now >= deadline:
                if task.bids:
                    self._award_task(task_id, task)
                else:
                    # No bids — reassign to nearest available UAV
                    self._force_assign(task_id, task)
                to_close.append(task_id)

        for task_id in to_close:
            del self.open_auctions[task_id]

    def _award_task(self, task_id, task):
        """Award task to highest bidder."""
        if not task.bids:
            return

        # Select winner — highest bid score
        winner_id = max(task.bids, key=lambda k: task.bids[k]['score'])
        winner_bid = task.bids[winner_id]

        task.assigned_to = winner_id
        self.active_tasks[task_id] = task
        if task_id in self.pending_tasks:
            del self.pending_tasks[task_id]

        # Update agent
        self.agents[winner_id].current_task = task_id

        award = {
            'task_id':    task_id,
            'awarded_to': winner_id,
            'task_type':  task.task_type,
            'position':   task.position.tolist(),
            'bid_score':  winner_bid['score'],
        }

        msg = String()
        msg.data = json.dumps(award)
        self.pub_award.publish(msg)

        self.get_logger().info(
            f'Task {task_id} awarded to UAV{winner_id} '
            f'(score: {winner_bid["score"]:.3f})')

    def _force_assign(self, task_id, task):
        """Force assign task when no bids received."""
        available = [
            uid for uid, agent in self.agents.items()
            if agent.connected and agent.current_task is None
        ]

        if not available:
            self.get_logger().warn(
                f'No available UAVs for task {task_id}')
            return

        # Assign to first available
        winner_id = available[0]
        task.assigned_to = winner_id
        task.bids[winner_id] = {'score': 0.0, 'forced': True}
        self._award_task(task_id, task)

    # ------------------------------------------------------------------
    # Callbacks
    # ------------------------------------------------------------------
    def _bid_callback(self, msg):
        """Process incoming bid from a UAV."""
        try:
            bid = json.loads(msg.data)
            task_id  = bid['task_id']
            uav_id   = bid['uav_id']
            score    = bid['score']

            if task_id in self.open_auctions:
                self.open_auctions[task_id].bids[uav_id] = bid
                self.get_logger().debug(
                    f'Bid received: UAV{uav_id} -> {task_id} '
                    f'score={score:.3f}')
        except (json.JSONDecodeError, KeyError) as e:
            self.get_logger().warn(f'Invalid bid: {e}')

    def _state_callback(self, msg, uav_id):
        """Update UAV connection status."""
        agent = self.agents[uav_id]
        was_connected = agent.connected
        agent.connected = msg.connected
        agent.last_heartbeat = time.time()

        if msg.connected and not was_connected:
            self.get_logger().info(f'UAV{uav_id} connected')
        elif not msg.connected and was_connected:
            self.get_logger().warn(f'UAV{uav_id} disconnected')

    def _detection_callback(self, msg):
        """Create verification task when detection confirmed."""
        try:
            detections = json.loads(msg.data)
            for det in detections:
                cell = det.get('grid_cell', [0, 0])
                pos  = np.array([
                    float(cell[0]) * 1.0,
                    float(cell[1]) * 1.0,
                    0.0
                ])
                self._create_task('verify', pos, 0.95)
        except (json.JSONDecodeError, KeyError):
            pass

    # ------------------------------------------------------------------
    # Status publisher
    # ------------------------------------------------------------------
    def _status_publisher(self):
        """Publish overall swarm status."""
        status = {
            'timestamp':       time.time(),
            'scenario':        self.scenario,
            'agents': {
                str(uid): {
                    'connected':    agent.connected,
                    'type':         agent.uav_type.value,
                    'current_task': agent.current_task,
                }
                for uid, agent in self.agents.items()
            },
            'pending_tasks':   len(self.pending_tasks),
            'active_tasks':    len(self.active_tasks),
            'completed_tasks': len(self.completed_tasks),
            'open_auctions':   len(self.open_auctions),
        }

        msg = String()
        msg.data = json.dumps(status)
        self.pub_status.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = CNPCoordinator()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()