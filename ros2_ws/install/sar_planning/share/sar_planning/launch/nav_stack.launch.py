"""
nav_stack.launch.py — Three-scale navigation stack launch for UAV swarm SAR
============================================================================
Launches all three navigation layers for a single UAV:
  1. rrtstar_planner  (global, 1 Hz planning)
  2. vfh_navigator    (mid-scale, 10 Hz)
  3. apf_navigator    (local, 20 Hz — assumed pre-existing)

Usage:
    ros2 launch sar_planning nav_stack.launch.py uav_id:=0
    ros2 launch sar_planning nav_stack.launch.py uav_id:=1
    ros2 launch sar_planning nav_stack.launch.py uav_id:=2
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():

    uav_id_arg = DeclareLaunchArgument(
        'uav_id',
        default_value='0',
        description='UAV instance number (0=quad1, 1=quad2, 2=plane)',
    )

    uav_id = LaunchConfiguration('uav_id')

    rrtstar_node = Node(
        package    = 'sar_planning',
        executable = 'rrtstar_planner',
        name       = ['rrtstar_planner_uav', uav_id],
        parameters = [{
            'uav_id':          uav_id,
            'max_iter':        2000,
            'step_size':       2.0,
            'goal_bias':       0.10,
            'goal_tol':        0.8,
            'search_radius':   4.0,
            'map_x_min':      -50.0,
            'map_x_max':       50.0,
            'map_y_min':      -50.0,
            'map_y_max':       50.0,
            'map_z_min':        2.0,
            'map_z_max':       30.0,
            'informed_rrtstar': True,
            'waypoint_index':   1,
            'safety_margin':    1.5,
        }],
        output = 'screen',
    )

    vfh_node = Node(
        package    = 'sar_planning',
        executable = 'vfh_navigator',
        name       = ['vfh_navigator_uav', uav_id],
        parameters = [{
            'uav_id':           uav_id,
            'num_sectors':      72,
            'safety_dist':      1.5,
            'max_speed':        3.0,
            'goal_tol':         0.5,
            'valley_threshold': 0.3,
            'alpha':            0.5,
            'a_coeff':          1.0,
            'b_coeff':          0.1,
            'vfh_plus':         True,
        }],
        output = 'screen',
    )

    apf_node = Node(
        package    = 'sar_planning',
        executable = 'apf_navigator',
        name       = ['apf_navigator_uav', uav_id],
        parameters = [{
            'uav_id': uav_id,
        }],
        output = 'screen',
    )

    return LaunchDescription([
        uav_id_arg,
        rrtstar_node,
        vfh_node,
        apf_node,
    ])
