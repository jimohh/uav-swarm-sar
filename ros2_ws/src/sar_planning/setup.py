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
        ],
    },
)
