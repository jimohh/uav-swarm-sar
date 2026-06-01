# Project State — End of Day 3

## Environment
- Droplet: root@64.227.35.132 (uav-swarm-sim, LON1, Ubuntu 22.04, 4vCPU/8GB)
- Local: WSL2 Ubuntu, ROS 2 Jazzy, Gazebo Harmonic
- Repo: https://github.com/jimohh/uav-swarm-sar
- ROS on Droplet: Humble

## Completed
- Day 1: PX4 SITL + MAVROS connected (gz_x500, HEADLESS=1, PX4_SIM_SPEED_FACTOR=0.5)
- Day 2: probability_map_node — /sar/prob_map/{urban,wilderness,maritime} at 1Hz
- Day 3: waypoint_selector + apf_navigator + ekf_node — full planning loop verified

## Active topics verified
- /sar/ekf/odom
- /sar/prob_map/urban|wilderness|maritime
- /sar/waypoints
- /uav0/mavros/state (connected: True)

## Key commands
# Start PX4:
cd ~/PX4-Autopilot && export GZ_VERSION=harmonic && export PX4_SIM_SPEED_FACTOR=0.5 && HEADLESS=1 make px4_sitl gz_x500
# Start MAVROS:
source /opt/ros/humble/setup.bash && ros2 run mavros mavros_node --ros-args -p fcu_url:=udp://:14540@localhost:14550 -p system_id:=1 --remap __ns:=/uav0
# Launch planning stack:
cd ~/thesis_ws/uav-swarm-sar/ros2_ws && source /opt/ros/humble/setup.bash && source install/setup.bash && ros2 launch sar_planning planning_stack.launch.py scenario:=urban uav_ns:=uav0

## Next: Day 4
- Integrate YOLO11s on simulated thermal feed
- Implement CW Doppler RF acoustic stub
- Wire three-stage cascade: RF -> YOLO11s -> 10-frame temporal window
- Target: Pd=0.85, conf=0.45

## Architecture reminders
- Detection: YOLO11s (C3k2+C2PSA backbone), NOT YOLOv8
- Coordination: decentralised auction CNP, NOT leader-follower
- Swarm: 2x Iris quadrotors + 1x Standard Plane
- Experimental: 9 conditions x 20 trials = 180 total, ANOVA + Tukey HSD

## Day 4 Status
- thermal_camera_node: VERIFIED (10 Hz)
- yolo11s_detector: VERIFIED (model loads, simulation mode working)
- rf_doppler_stub: packaging issue (main function not found) — fix pending
- Detection cascade architecture: implemented and individually verified

## Day 5 Next
- Spawn 2x Iris quadrotors + 1x Standard Plane
- Implement auction CNP coordinator
- MAVLink mesh communications
- Heartbeat-timeout fault tolerance
