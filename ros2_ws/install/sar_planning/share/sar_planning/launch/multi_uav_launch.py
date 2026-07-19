#!/usr/bin/env python3
"""
multi_uav.launch.py
Spawns heterogeneous UAV swarm:
  - UAV0: PX4 Iris quadrotor  (instance 0)
  - UAV1: PX4 Iris quadrotor  (instance 1)
  - UAV2: PX4 Standard Plane  (instance 2)
Each UAV gets its own MAVROS node with namespaced topics.
"""

from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument, ExecuteProcess
from launch.substitutions import LaunchConfiguration


def generate_launch_description():

    scenario_arg = DeclareLaunchArgument(
        'scenario', default_value='urban',
        description='SAR scenario: urban | wilderness | maritime')

    scenario = LaunchConfiguration('scenario')

    # --- MAVROS node for UAV0 (quadrotor) ---
    mavros_uav0 = Node(
        package='mavros',
        executable='mavros_node',
        namespace='uav0',
        name='mavros',
        output='screen',
        parameters=[{
            'fcu_url':   'udp://:14540@localhost:14550',
            'gcs_url':   '',
            'system_id': 1,
            'component_id': 1,
        }])

    # --- MAVROS node for UAV1 (quadrotor) ---
    mavros_uav1 = Node(
        package='mavros',
        executable='mavros_node',
        namespace='uav1',
        name='mavros',
        output='screen',
        parameters=[{
            'fcu_url':   'udp://:14541@localhost:14560',
            'gcs_url':   '',
            'system_id': 2,
            'component_id': 1,
        }])

    # --- MAVROS node for UAV2 (fixed-wing) ---
    mavros_uav2 = Node(
        package='mavros',
        executable='mavros_node',
        namespace='uav2',
        name='mavros',
        output='screen',
        parameters=[{
            'fcu_url':   'udp://:14542@localhost:14570',
            'gcs_url':   '',
            'system_id': 3,
            'component_id': 1,
        }])

    # --- CNP Coordinator ---
    cnp_coordinator = Node(
        package='sar_planning',
        executable='cnp_coordinator',
        name='cnp_coordinator',
        output='screen',
        parameters=[{
            'num_uavs':          3,
            'scenario':          scenario,
            'auction_timeout':   2.0,
            'heartbeat_timeout': 5.0,
        }])

    # --- Heartbeat monitor ---
    heartbeat_monitor = Node(
        package='sar_planning',
        executable='heartbeat_monitor',
        name='heartbeat_monitor',
        output='screen',
        parameters=[{
            'num_uavs':          3,
            'heartbeat_timeout': 5.0,
            'check_rate':        1.0,
        }])

    # --- Planning stack for each UAV ---
    planning_uav0 = Node(
        package='sar_planning',
        executable='apf_navigator',
        name='apf_navigator_uav0',
        output='screen',
        parameters=[{
            'uav_ns':          'uav0',
            'cruise_altitude': 10.0,
            'max_speed':       3.0,
        }])

    planning_uav1 = Node(
        package='sar_planning',
        executable='apf_navigator',
        name='apf_navigator_uav1',
        output='screen',
        parameters=[{
            'uav_ns':          'uav1',
            'cruise_altitude': 12.0,
            'max_speed':       3.0,
        }])

    planning_uav2 = Node(
        package='sar_planning',
        executable='apf_navigator',
        name='apf_navigator_uav2',
        output='screen',
        parameters=[{
            'uav_ns':          'uav2',
            'cruise_altitude': 50.0,   # fixed-wing flies higher
            'max_speed':       15.0,   # fixed-wing faster
        }])

    return LaunchDescription([
        scenario_arg,
        mavros_uav0,
        mavros_uav1,
        mavros_uav2,
        cnp_coordinator,
        heartbeat_monitor,
        planning_uav0,
        planning_uav1,
        planning_uav2,
    ])