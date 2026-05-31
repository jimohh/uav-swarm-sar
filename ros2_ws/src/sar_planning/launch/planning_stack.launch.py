#!/usr/bin/env python3
"""
planning_stack.launch.py
Full SAR stack — Days 2, 3 and 4:
  1. ProbabilityMapNode   — three scenario maps
  2. WaypointSelector     — top-N waypoint extraction
  3. APFNavigator         — potential field navigation
  4. EKFNode              — IMU + GPS fusion
  5. ThermalCameraNode    — synthetic thermal feed
  6. RFDopplerStub        — CW Doppler RF stage 1
  7. YOLO11sDetector      — visual detection stage 2+3
"""

from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration


def generate_launch_description():

    # --- Launch arguments ---
    scenario_arg = DeclareLaunchArgument(
        'scenario', default_value='urban',
        description='SAR scenario: urban | wilderness | maritime')

    uav_ns_arg = DeclareLaunchArgument(
        'uav_ns', default_value='uav0',
        description='UAV MAVROS namespace')

    top_n_arg = DeclareLaunchArgument(
        'top_n', default_value='5',
        description='Number of waypoints to extract')

    scenario = LaunchConfiguration('scenario')
    uav_ns   = LaunchConfiguration('uav_ns')
    top_n    = LaunchConfiguration('top_n')

    # --- Day 2: Probability maps ---
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
        }])

    # --- Day 3: Planning layer ---
    waypoint_selector_node = Node(
        package='sar_planning',
        executable='waypoint_selector',
        name='waypoint_selector',
        output='screen',
        parameters=[{
            'scenario':   scenario,
            'top_n':      top_n,
            'resolution': 1.0,
        }])

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
        }])

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
        }])

    # --- Day 4: Detection pipeline ---
    thermal_camera_node = Node(
        package='sar_planning',
        executable='thermal_camera_node',
        name='thermal_camera_node',
        output='screen',
        parameters=[{
            'publish_rate': 10.0,
            'image_width':  640,
            'image_height': 480,
            'num_targets':  3,
        }])

    rf_doppler_node = Node(
        package='sar_planning',
        executable='rf_doppler_stub',
        name='rf_doppler_stub',
        output='screen',
        parameters=[{
            'uav_ns':          uav_ns,
            'detection_range': 15.0,
            'false_alarm_rate': 0.05,
            'publish_rate':    5.0,
        }])

    yolo11s_node = Node(
        package='sar_planning',
        executable='yolo11s_detector',
        name='yolo11s_detector',
        output='screen',
        parameters=[{
            'conf_threshold':  0.45,
            'temporal_window': 10,
            'target_pd':       0.85,
            'model_path':      'yolo11s.pt',
        }])

    return LaunchDescription([
        scenario_arg,
        uav_ns_arg,
        top_n_arg,
        prob_map_node,
        waypoint_selector_node,
        apf_navigator_node,
        ekf_node,
        thermal_camera_node,
        rf_doppler_node,
        yolo11s_node,
    ])