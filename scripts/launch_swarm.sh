#!/usr/bin/env bash
# =============================================================================
# launch_swarm.sh — Launch full 3-UAV SAR swarm in tmux
# =============================================================================
# Creates one tmux session with named windows for each component.
# Run on the Droplet:
#   chmod +x launch_swarm.sh
#   ./launch_swarm.sh
#
# To attach:   tmux attach -t swarm
# To kill all: tmux kill-session -t swarm
# =============================================================================

set -euo pipefail

SESSION="swarm"
WS="$HOME/thesis_ws/uav-swarm-sar/ros2_ws"
PX4="$HOME/PX4-Autopilot"
ROS_SETUP="/opt/ros/humble/setup.bash"
WS_SETUP="$WS/install/setup.bash"

# PX4 env
PX4_ENV="export GZ_VERSION=harmonic && export PX4_SIM_SPEED_FACTOR=0.5 && export HEADLESS=1"

# Kill existing session if any
tmux kill-session -t "$SESSION" 2>/dev/null || true

echo "Starting tmux session: $SESSION"
tmux new-session -d -s "$SESSION" -x 220 -y 50

# ── Window 0: UAV0 — Iris quadrotor (instance 0) ─────────────────────────
tmux rename-window -t "$SESSION:0" "uav0-px4"
tmux send-keys -t "$SESSION:0" \
    "cd $PX4 && $PX4_ENV && make px4_sitl gz_x500" Enter

# ── Window 1: UAV1 — Iris quadrotor (instance 1) ─────────────────────────
tmux new-window -t "$SESSION" -n "uav1-px4"
tmux send-keys -t "$SESSION:uav1-px4" \
    "sleep 8 && \
     mkdir -p /tmp/px4_instance1 && \
     ln -sf $PX4/build/px4_sitl_default/etc /tmp/px4_instance1/etc && \
     ln -sf $PX4/build/px4_sitl_default/bin /tmp/px4_instance1/bin && \
     cd $PX4 && $PX4_ENV && \
     export PX4_GZ_MODEL_POSE='10,0,0,0,0,0' && \
     ./build/px4_sitl_default/bin/px4 -i 1 \
       -s $PX4/build/px4_sitl_default/etc/init.d-posix/rcS \
       -w /tmp/px4_instance1" Enter

# ── Window 2: UAV2 — Standard Plane (instance 2) ─────────────────────────
tmux new-window -t "$SESSION" -n "uav2-px4"
tmux send-keys -t "$SESSION:uav2-px4" \
    "sleep 16 && \
     mkdir -p /tmp/px4_instance2 && \
     ln -sf $PX4/build/px4_sitl_default/etc /tmp/px4_instance2/etc && \
     ln -sf $PX4/build/px4_sitl_default/bin /tmp/px4_instance2/bin && \
     cd $PX4 && $PX4_ENV && \
     export PX4_GZ_MODEL=standard_vtol && \
     export PX4_GZ_MODEL_POSE='20,0,0,0,0,0' && \
     ./build/px4_sitl_default/bin/px4 -i 2 \
       -s $PX4/build/px4_sitl_default/etc/init.d-posix/rcS \
       -w /tmp/px4_instance2" Enter

# ── Window 3: MAVROS UAV0 ────────────────────────────────────────────────
tmux new-window -t "$SESSION" -n "mavros0"
tmux send-keys -t "$SESSION:mavros0" \
    "sleep 20 && source $ROS_SETUP && \
     ros2 run mavros mavros_node --ros-args \
       -p fcu_url:=udp://:14540@localhost:14550 \
       -p system_id:=1 \
       --remap __ns:=/uav0" Enter

# ── Window 4: MAVROS UAV1 ────────────────────────────────────────────────
tmux new-window -t "$SESSION" -n "mavros1"
tmux send-keys -t "$SESSION:mavros1" \
    "sleep 22 && source $ROS_SETUP && \
     ros2 run mavros mavros_node --ros-args \
       -p fcu_url:=udp://:14541@localhost:14560 \
       -p system_id:=2 \
       --remap __ns:=/uav1" Enter

# ── Window 5: MAVROS UAV2 (plane) ────────────────────────────────────────
tmux new-window -t "$SESSION" -n "mavros2"
tmux send-keys -t "$SESSION:mavros2" \
    "sleep 24 && source $ROS_SETUP && \
     ros2 run mavros mavros_node --ros-args \
       -p fcu_url:=udp://:14542@localhost:14570 \
       -p system_id:=3 \
       --remap __ns:=/uav2" Enter

# ── Window 6: ROS nodes (all 3 UAVs) ────────────────────────────────────
tmux new-window -t "$SESSION" -n "ros-nodes"
tmux send-keys -t "$SESSION:ros-nodes" \
    "sleep 30 && \
     source $ROS_SETUP && source $WS_SETUP && \
     ros2 launch sar_planning nav_stack.launch.py uav_id:=0 &
     ros2 launch sar_planning nav_stack.launch.py uav_id:=1 &
     ros2 run sar_planning plane_bridge --ros-args -p uav_id:=2 &
     ros2 run sar_planning cnp_coordinator &
     ros2 run sar_planning heartbeat_monitor &
     ros2 run sar_planning probability_map_node &
     wait" Enter

# ── Window 7: Monitoring ─────────────────────────────────────────────────
tmux new-window -t "$SESSION" -n "monitor"
tmux send-keys -t "$SESSION:monitor" \
    "sleep 35 && source $ROS_SETUP && source $WS_SETUP && \
     watch -n 2 'ros2 topic list | grep -E \"heartbeat|goal|status\"'" Enter

# ── Window 8: Shell (free terminal) ──────────────────────────────────────
tmux new-window -t "$SESSION" -n "shell"
tmux send-keys -t "$SESSION:shell" \
    "source $ROS_SETUP && source $WS_SETUP" Enter

# Attach
tmux select-window -t "$SESSION:0"
tmux attach-session -t "$SESSION"