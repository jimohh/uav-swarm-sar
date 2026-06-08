from setuptools import setup
import os
from glob import glob

package_name = 'sar_planning'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'),
            glob('launch/*.py')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='maintainer',
    maintainer_email='maintainer@example.com',
    description='SAR planning stack',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'probability_map_node = sar_planning.probability_map_node:main',
            'waypoint_selector    = sar_planning.waypoint_selector:main',
            'apf_navigator        = sar_planning.apf_navigator:main',
            'ekf_node             = sar_planning.ekf_node:main',
            'thermal_camera_node  = sar_planning.thermal_camera_node:main',
            'rf_doppler_stub      = sar_planning.rf_doppler_stub:main',
            'yolo11s_detector     = sar_planning.yolo11s_detector:main',
            'cnp_coordinator      = sar_planning.cnp_coordinator:main',
            'heartbeat_monitor    = sar_planning.heartbeat_monitor:main',
            'metrics_logger       = sar_planning.metrics_logger:main',
            'experiment_runner    = sar_planning.experiment_runner:main',
            'vfh_navigator        = sar_planning.vfh_navigator:main',
            'rrtstar_planner      = sar_planning.rrtstar_planner:main',
            'plane_bridge         = sar_planning.plane_bridge:main',
        ],
    },
)