#!/usr/bin/env bash
SESSION="swarm"
PX4="$HOME/PX4-Autopilot"
WS="$HOME/thesis_ws/uav-swarm-sar/ros2_ws"

tmux kill-session -t "$SESSION" 2>/dev/null || true

tmux new-session -d -s "$SESSION" -n "uav0-px4" -x 220 -y 50
tmux new-window -t "$SESSION" -n "uav1-px4"
tmux new-window -t "$SESSION" -n "uav2-px4"
tmux new-window -t "$SESSION" -n "mavros0"
tmux new-window -t "$SESSION" -n "mavros1"
tmux new-window -t "$SESSION" -n "mavros2"
tmux new-window -t "$SESSION" -n "ros-nodes"
tmux new-window -t "$SESSION" -n "shell"

tmux send-keys -t "$SESSION:uav0-px4" "cd $PX4 && export GZ_VERSION=harmonic && export PX4_SIM_SPEED_FACTOR=0.5 && export HEADLESS=1 && make px4_sitl gz_x500" Enter

tmux send-keys -t "$SESSION:uav1-px4" "sleep 10 && mkdir -p /tmp/px4_instance1 && ln -sf $PX4/build/px4_sitl_default/etc /tmp/px4_instance1/etc && ln -sf $PX4/build/px4_sitl_default/bin /tmp/px4_instance1/bin && cd $PX4 && export GZ_VERSION=harmonic && export PX4_SIM_SPEED_FACTOR=0.5 && export HEADLESS=1 && export PX4_GZ_MODEL_POSE='10,0,0,0,0,0' && ./build/px4_sitl_default/bin/px4 -i 1 -s $PX4/build/px4_sitl_default/etc/init.d-posix/rcS -w /tmp/px4_instance1" Enter

tmux send-keys -t "$SESSION:uav2-px4" "sleep 20 && mkdir -p /tmp/px4_instance2 && ln -sf $PX4/build/px4_sitl_default/etc /tmp/px4_instance2/etc && ln -sf $PX4/build/px4_sitl_default/bin /tmp/px4_instance2/bin && cd $PX4 && export GZ_VERSION=harmonic && export PX4_SIM_SPEED_FACTOR=0.5 && export HEADLESS=1 && export PX4_GZ_MODEL=standard_vtol && export PX4_GZ_MODEL_POSE='20,0,0,0,0,0' && ./build/px4_sitl_default/bin/px4 -i 2 -s $PX4/build/px4_sitl_default/etc/init.d-posix/rcS -w /tmp/px4_instance2" Enter

tmux send-keys -t "$SESSION:mavros0" "sleep 30 && source /opt/ros/humble/setup.bash && ros2 run mavros mavros_node --ros-args -p fcu_url:=udp://:14540@localhost:14550 -p system_id:=1 --remap __ns:=/uav0" Enter

tmux send-keys -t "$SESSION:mavros1" "sleep 32 && source /opt/ros/humble/setup.bash && ros2 run mavros mavros_node --ros-args -p fcu_url:=udp://:14541@localhost:14560 -p system_id:=2 --remap __ns:=/uav1" Enter

tmux send-keys -t "$SESSION:mavros2" "sleep 34 && source /opt/ros/humble/setup.bash && ros2 run mavros mavros_node --ros-args -p fcu_url:=udp://:14542@localhost:14570 -p system_id:=3 --remap __ns:=/uav2" Enter

tmux send-keys -t "$SESSION:ros-nodes" "sleep 40 && source /opt/ros/humble/setup.bash && source $WS/install/setup.bash && ros2 run sar_planning cnp_coordinator & ros2 run sar_planning heartbeat_monitor & ros2 run sar_planning probability_map_node & wait" Enter

tmux send-keys -t "$SESSION:shell" "source /opt/ros/humble/setup.bash && source $WS/install/setup.bash" Enter

tmux select-window -t "$SESSION:uav0-px4"
tmux attach-session -t "$SESSION"