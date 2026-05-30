#!/usr/bin/env python3
"""
planning_stack.launch.py
Launches the full Day 3 planning stack:
  1. ProbabilityMapNode  — publishes all three scenario maps
  2. WaypointSelector    — extracts top-N waypoints from active map
  3. APFNavigator        — navigates UAV via artificial potential fields
  4. EKFNode             — fuses IMU + GPS for clean pose estimates
"""

from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration


def generate_launch_description():

    # --- Launch arguments ---
    scenario_arg = DeclareLaunchArgument(
        'scenario',
        default_value='urban',
        description='SAR scenario: urban | wilderness | maritime')

    uav_ns_arg = DeclareLaunchArgument(
        'uav_ns',
        default_value='uav0',
        description='UAV MAVROS namespace')

    top_n_arg = DeclareLaunchArgument(
        'top_n',
        default_value='5',
        description='Number of waypoints to extract from probability map')

    scenario  = LaunchConfiguration('scenario')
    uav_ns    = LaunchConfiguration('uav_ns')
    top_n     = LaunchConfiguration('top_n')

    # --- Nodes ---

    # 1. Probability map publisher
    prob_map_node = Node(
        package='sar_planning',
        executable='probability_map_node',
        name='probability_map_node',
        output='screen',
        parameters=[{
            'map_width':    100,
            'map_height':   100,
            'resolution':   1.0,
            'publish_rate': 1.0,
        }]
    )

    # 2. Waypoint selector
    waypoint_selector_node = Node(
        package='sar_planning',
        executable='waypoint_selector',
        name='waypoint_selector',
        output='screen',
        parameters=[{
            'scenario':   scenario,
            'top_n':      top_n,
            'resolution': 1.0,
        }]
    )

    # 3. APF navigator
    apf_navigator_node = Node(
        package='sar_planning',
        executable='apf_navigator',
        name='apf_navigator',
        output='screen',
        parameters=[{
            'uav_ns':            uav_ns,
            'attractive_gain':   1.5,
            'repulsive_gain':    2.0,
            'repulsive_radius':  5.0,
            'max_speed':         3.0,
            'arrival_threshold': 2.0,
            'cruise_altitude':   10.0,
        }]
    )

    # 4. EKF node
    ekf_node = Node(
        package='sar_planning',
        executable='ekf_node',
        name='ekf_node',
        output='screen',
        parameters=[{
            'uav_ns':       uav_ns,
            'publish_rate': 50.0,
            'q_pos':        0.01,
            'q_vel':        0.1,
            'q_att':        0.001,
            'r_gps':        2.5,
            'r_imu':        0.01,
        }]
    )

    return LaunchDescription([
        scenario_arg,
        uav_ns_arg,
        top_n_arg,
        prob_map_node,
        waypoint_selector_node,
        apf_navigator_node,
        ekf_node,
    ])