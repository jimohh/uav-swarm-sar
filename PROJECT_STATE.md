# UAV Swarm SAR — Project State
# Last updated: June 7, 2026

## Environment
- Droplet: root@64.227.35.132 (uav-swarm-sim, LON1, Ubuntu 22.04, 4vCPU/8GB)
- Local: WSL2 Ubuntu 24.04, ROS 2 Jazzy, Gazebo Harmonic
- Repo: https://github.com/jimohh/uav-swarm-sar
- ROS on Droplet: Humble
- SSH note: ISP sometimes blocks port 22 — fix: ssh -p 443 root@64.227.35.132 (already configured in sshd_config)

## Completed Days
- Day 1: PX4 SITL + MAVROS connected (gz_x500, HEADLESS=1, PX4_SIM_SPEED_FACTOR=0.5)
- Day 2: probability_map_node — /sar/prob_map/{urban,wilderness,maritime} at 1Hz verified
- Day 3: waypoint_selector + apf_navigator + ekf_node — full planning loop verified
- Day 4: thermal_camera_node (10Hz) + yolo11s_detector (model loaded) — individually verified. rf_doppler_stub has packaging issue (main function not found) — UNRESOLVED
- Day 5: cnp_coordinator + heartbeat_monitor — task announcement/award verified with 2 UAVs
- Day 6 (partial): experiment script written but has sourcing bug — UNRESOLVED

## Key Commands
# Start PX4 UAV0:
cd ~/PX4-Autopilot && export GZ_VERSION=harmonic && export PX4_SIM_SPEED_FACTOR=0.5 && HEADLESS=1 make px4_sitl gz_x500

# Start PX4 UAV1 (direct binary):
mkdir -p /tmp/px4_instance1 && ln -sf /root/PX4-Autopilot/build/px4_sitl_default/etc /tmp/px4_instance1/etc && ln -sf /root/PX4-Autopilot/build/px4_sitl_default/bin /tmp/px4_instance1/bin
cd ~/PX4-Autopilot && export GZ_VERSION=harmonic && export PX4_SIM_SPEED_FACTOR=0.5 && export PX4_GZ_MODEL_POSE="10,0,0,0,0,0" && ./build/px4_sitl_default/bin/px4 -i 1 -s /root/PX4-Autopilot/build/px4_sitl_default/etc/init.d-posix/rcS -w /tmp/px4_instance1

# Start MAVROS UAV0:
source /opt/ros/humble/setup.bash && ros2 run mavros mavros_node --ros-args -p fcu_url:=udp://:14540@localhost:14550 -p system_id:=1 --remap __ns:=/uav0

# Start MAVROS UAV1:
source /opt/ros/humble/setup.bash && ros2 run mavros mavros_node --ros-args -p fcu_url:=udp://:14541@localhost:14560 -p system_id:=2 --remap __ns:=/uav1

# Launch planning stack:
cd ~/thesis_ws/uav-swarm-sar/ros2_ws && source /opt/ros/humble/setup.bash && source install/setup.bash && ros2 launch sar_planning planning_stack.launch.py scenario:=urban uav_ns:=uav0

# Run experiments (smoke test):
cd ~/thesis_ws/uav-swarm-sar && bash scripts/run_experiments.sh 1 60

## Nodes Implemented (all in ros2_ws/src/sar_planning/sar_planning/)
- probability_map_node.py ✓
- waypoint_selector.py ✓
- apf_navigator.py ✓
- ekf_node.py ✓
- thermal_camera_node.py ✓
- rf_doppler_stub.py ✓ (packaging bug unresolved)
- yolo11s_detector.py ✓
- cnp_coordinator.py ✓
- heartbeat_monitor.py ✓
- metrics_logger.py ✓
- experiment_runner.py ✓

## Unresolved Issues
1. rf_doppler_stub: AttributeError module has no attribute 'main' — packaging issue
2. run_experiments.sh: exits silently after trial starts — ROS sourcing causes early exit with set -eo pipefail
3. UAV2 (Standard Plane / fixed-wing) not yet added to experiment script or MAVROS
4. VFH navigator not implemented
5. RRT* planner not implemented

## Revised 7-Day Plan (from June 7)
- Day 1: Implement vfh_navigator.py + rrtstar_planner.py
- Day 2: Add UAV2 (fixed-wing) — PX4 instance 2, MAVROS, CNP routing
- Day 3: Fix rf_doppler_stub packaging + full Day 4 detection pipeline validation
- Day 4: Full 3-agent swarm integration test
- Day 5: Fix experiment script + smoke test (9 trials)
- Day 6: Full 180-trial batch run overnight
- Day 7: ANOVA + Tukey HSD + figures + thesis Chapters 4-5

## Architecture (for reference)
- Swarm: UAV0 (Iris quadrotor) + UAV1 (Iris quadrotor) + UAV2 (Standard Plane)
- Planning: POC-based, time-discounted I(π) = ∫λᵗ(x)·P(x)dx
- Navigation: APF (local) / VFH (mid) / RRT* (global) — three-scale hierarchy
- Detection: YOLO11s (C3k2+C2PSA, conf=0.45, 10-frame window, Pd=0.85) + CW Doppler RF (Pd=0.45)
- Coordination: Decentralised auction CNP — NOT leader-follower
- Experimental: 9 conditions × 20 trials = 180 total, two-factor ANOVA + Tukey HSD α=0.05
- Five metrics: coverage_rate, time_to_detection, path_efficiency, inter_agent_distance, fault_recovery_time

## Disk management
- Droplet disk was at 100% — cleared ~/thesis_ws/logs/* (136GB of PX4 logs)
- Log truncation added to run_trial() in run_experiments.sh to prevent recurrence
- Current disk usage: ~13% after cleanup

## GitHub
- Repo: https://github.com/jimohh/uav-swarm-sar
- Auth: Personal Access Token (PAT) stored via git credential.helper store
- WSL2 alias: droplet → ssh root@64.227.35.132
