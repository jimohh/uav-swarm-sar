# Project context for GitHub Copilot

## Stack
- ROS 2 Humble (on DigitalOcean Droplet, Ubuntu 22.04)
- ROS 2 Jazzy (on WSL2, Ubuntu 24.04)
- Gazebo Harmonic
- PX4 v1.14 SITL
- MAVROS
- Python 3.10

## Architecture
- Swarm: 2x PX4 Iris quadrotors + 1x PX4 Standard Plane
- Planning: POC-based task allocation
- Navigation: APF/VFH/RRT* hierarchy
- Detection: YOLO11s (C3k2+C2PSA) + CW Doppler RF
- Coordination: Auction-based CNP, decentralised

## Conventions
- UAV namespaces: /uav0, /uav1, /uav2
- All nodes use rclpy, ament_python build type
- Headless simulation — no GUI dependencies
- Results saved to ~/thesis_ws/results/{urban,wilderness,maritime}/
