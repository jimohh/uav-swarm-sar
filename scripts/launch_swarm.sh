#!/usr/bin/env bash
# =============================================================================
# launch_swarm.sh — Launch full 3-UAV SAR swarm in tmux
# =============================================================================
# Run on the Droplet:
#   chmod +x scripts/launch_swarm.sh
#   ./scripts/launch_swarm.sh
#
# To attach:   tmux attach -t swarm
# To kill all: tmux kill-session -t swarm
# =============================================================================

SESSION="swarm"
PX4="$HOME/PX4-Autopilot"
WS="$HOME/thesis_ws/uav-swarm-sar/ros2_ws"
ROS="source /opt/ros/humble/setup.bash"
WS_SRC="source $WS/install/setup.bash"
PX4_ENV="export GZ_VERSION=harmonic && export PX4_SIM_SPEED_FACTOR=0.5 && export HEADLESS=1"

# Kill any existing session
tmux kill-session -t "$SESSION" 2>/dev/null || true

# Create session — window named at creation to avoid rename issues
tmux new-session -d -s "$SESSION" -n "uav0-px4" -x 220 -y 50

# UAV0 — Iris quadrotor instance 0
tmux send-keys -t "${SESSION}:uav0-px4" \
  "cd $PX4 && $PX4_ENV && make px4_sitl gz_x500" Enter

# UAV1 — Iris quadrotor instance 1
tmux new-window -t "${SESSION}:" -n "uav1-px4"
tmux send-keys -t "${SESSION}:uav1-px4" \
  "sleep 10 && \
   mkdir -p /tmp/px4_instance1 && \
   ln -sf $PX4/build/px4_sitl_default/etc /tmp/px4_instance1/etc && \
   ln -sf $PX4/build/px4_sitl_default/bin /tmp/px4_instance1/bin && \
   cd $PX4 && $PX4_ENV && \
   export PX4_GZ_MODEL_POSE='10,0,0,0,0,0' && \
   ./build/px4_sitl_default/bin/px4 -i 1 \
     -s $PX4/build/px4_sitl_default/etc/init.d-posix/rcS \
     -w /tmp/px4_instance1" Enter

# UAV2 — Standard Plane instance 2
tmux new-window -t "${SESSION}:" -n "uav2-px4"
tmux send-keys -t "${SESSION}:uav2-px4" \
  "sleep 20 && \
   mkdir -p /tmp/px4_instance2 && \
   ln -sf $PX4/build/px4_sitl_default/etc /tmp/px4_instance2/etc && \
   ln -sf $PX4/build/px4_sitl_default/bin /tmp/px4_instance2/bin && \
   cd $PX4 && $PX4_ENV && \
   export PX4_GZ_MODEL=standard_vtol && \
   export PX4_GZ_MODEL_POSE='20,0,0,0,0,0' && \
   ./build/px4_sitl_default/bin/px4 -i 2 \
     -s $PX4/build/px4_sitl_default/etc/init.d-posix/rcS \
     -w /tmp/px4_instance2" Enter

# MAVROS UAV0
tmux new-window -t "${SESSION}:" -n "mavros0"
tmux send-keys -t "${SESSION}:mavros0" \
  "sleep 30 && $ROS && \
   ros2 run mavros mavros_node --ros-args \
     -p fcu_url:=udp://:14540@localhost:14550 \
     -p system_id:=1 \
     --remap __ns:=/uav0" Enter

# MAVROS UAV1
tmux new-window -t "${SESSION}:" -n "mavros1"
tmux send-keys -t "${SESSION}:mavros1" \
  "sleep 32 && $ROS && \
   ros2 run mavros mavros_node --ros-args \
     -p fcu_url:=udp://:14541@localhost:14560 \
     -p system_id:=2 \
     --remap __ns:=/uav1" Enter

# MAVROS UAV2
tmux new-window -t "${SESSION}:" -n "mavros2"
tmux send-keys -t "${SESSION}:mavros2" \
  "sleep 34 && $ROS && \
   ros2 run mavros mavros_node --ros-args \
     -p fcu_url:=udp://:14542@localhost:14570 \
     -p system_id:=3 \
     --remap __ns:=/uav2" Enter

# ROS nodes
tmux new-window -t "${SESSION}:" -n "ros-nodes"
tmux send-keys -t "${SESSION}:ros-nodes" \
  "sleep 40 && $ROS && $WS_SRC && \
   ros2 launch sar_planning nav_stack.launch.py uav_id:=0 & \
   ros2 launch sar_planning nav_stack.launch.py uav_id:=1 & \
   ros2 run sar_planning plane_bridge --ros-args -p uav_id:=2 & \
   ros2 run sar_planning cnp_coordinator & \
   ros2 run sar_planning heartbeat_monitor & \
   ros2 run sar_planning probability_map_node & \
   wait" Enter

# Monitor
tmux new-window -t "${SESSION}:" -n "monitor"
tmux send-keys -t "${SESSION}:monitor" \
  "sleep 45 && $ROS && $WS_SRC && \
   watch -n 2 'ros2 topic list | grep -E \"heartbeat|goal|status\"'" Enter

# Free shell
tmux new-window -t "${SESSION}:" -n "shell"
tmux send-keys -t "${SESSION}:shell" "$ROS && $WS_SRC" Enter

# Go to first window and attach
tmux select-window -t "${SESSION}:uav0-px4"
tmux attach-session -t "${SESSION}"