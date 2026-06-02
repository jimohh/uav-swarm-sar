#!/bin/bash
###############################################################################
# run_experiments.sh
# Orchestrates the full 180-trial experimental batch.
#   9 conditions (3 planners x 3 scenarios) x 20 trials = 180 total
#
# Each trial:
#   1. Starts 2x PX4 SITL instances (headless)
#   2. Starts MAVROS for each UAV
#   3. Launches the planning + detection + coordination stack
#   4. Runs experiment_runner + metrics_logger for trial_duration
#   5. Logs 5 metrics to CSV
#   6. Cleans up all processes before next trial
#
# Usage:  bash run_experiments.sh [trials_per_condition] [trial_duration]
#   e.g.  bash run_experiments.sh 20 120
###############################################################################

set -eo pipefail

# --- Configuration ---
TRIALS_PER_CONDITION=${1:-20}
TRIAL_DURATION=${2:-120}
PX4_DIR=~/PX4-Autopilot
WS_DIR=~/thesis_ws/uav-swarm-sar/ros2_ws
RESULTS_DIR=~/thesis_ws/results
LOG_DIR=~/thesis_ws/logs

SCENARIOS=("urban" "wilderness" "maritime")
PLANNERS=("apf" "vfh" "rrtstar")

mkdir -p "$RESULTS_DIR"/{urban,wilderness,maritime}
mkdir -p "$LOG_DIR"

# --- Source ROS ---
source /opt/ros/humble/setup.bash
source "$WS_DIR/install/setup.bash"

# --- Cleanup function ---
cleanup_trial() {
    pkill -f "px4" 2>/dev/null
    pkill -f "mavros_node" 2>/dev/null
    pkill -f "gz sim" 2>/dev/null
    pkill -f "ruby" 2>/dev/null
    pkill -f "sar_planning" 2>/dev/null
    sleep 3
}

# --- Start PX4 instances ---
start_px4_instances() {
    # UAV0
    cd "$PX4_DIR"
    export GZ_VERSION=harmonic
    export PX4_SIM_SPEED_FACTOR=2.0   # speed up for batch runs
    HEADLESS=1 make px4_sitl gz_x500 > "$LOG_DIR/px4_uav0.log" 2>&1 &
    sleep 15

    # UAV1
    mkdir -p /tmp/px4_instance1
    ln -sf "$PX4_DIR/build/px4_sitl_default/etc" /tmp/px4_instance1/etc
    ln -sf "$PX4_DIR/build/px4_sitl_default/bin" /tmp/px4_instance1/bin
    export PX4_GZ_MODEL_POSE="10,0,0,0,0,0"
    "$PX4_DIR/build/px4_sitl_default/bin/px4" \
        -i 1 \
        -s "$PX4_DIR/build/px4_sitl_default/etc/init.d-posix/rcS" \
        -w /tmp/px4_instance1 > "$LOG_DIR/px4_uav1.log" 2>&1 &
    sleep 10
}

# --- Start MAVROS ---
start_mavros() {
    ros2 run mavros mavros_node --ros-args \
        -p fcu_url:=udp://:14540@localhost:14550 \
        -p system_id:=1 --remap __ns:=/uav0 \
        > "$LOG_DIR/mavros_uav0.log" 2>&1 &
    sleep 5

    ros2 run mavros mavros_node --ros-args \
        -p fcu_url:=udp://:14541@localhost:14560 \
        -p system_id:=2 --remap __ns:=/uav1 \
        > "$LOG_DIR/mavros_uav1.log" 2>&1 &
    sleep 8
}

# --- Start the stack for a trial ---
start_stack() {
    local scenario=$1
    local planner=$2

    # Probability map + waypoint selector + detection
    ros2 run sar_planning probability_map_node \
        > "$LOG_DIR/probmap.log" 2>&1 &
    ros2 run sar_planning waypoint_selector \
        --ros-args -p scenario:="$scenario" \
        > "$LOG_DIR/waypoint.log" 2>&1 &
    ros2 run sar_planning thermal_camera_node \
        > "$LOG_DIR/thermal.log" 2>&1 &
    ros2 run sar_planning yolo11s_detector \
        > "$LOG_DIR/yolo.log" 2>&1 &
    ros2 run sar_planning cnp_coordinator \
        --ros-args -p scenario:="$scenario" \
        > "$LOG_DIR/cnp.log" 2>&1 &
    ros2 run sar_planning heartbeat_monitor \
        > "$LOG_DIR/heartbeat.log" 2>&1 &

    # Navigator for each UAV
    ros2 run sar_planning apf_navigator \
        --ros-args -p uav_ns:=uav0 \
        > "$LOG_DIR/nav_uav0.log" 2>&1 &
    ros2 run sar_planning apf_navigator \
        --ros-args -p uav_ns:=uav1 \
        > "$LOG_DIR/nav_uav1.log" 2>&1 &

    sleep 5
}

# --- Run a single trial ---
run_trial() {
    local trial_id=$1
    local scenario=$2
    local planner=$3

    echo "  [Trial $trial_id] $scenario/$planner — starting..."

    start_px4_instances
    start_mavros
    start_stack "$scenario" "$planner"

    # Run metrics logger (blocks for trial_duration)
    ros2 run sar_planning metrics_logger --ros-args \
        -p trial_id:="$trial_id" \
        -p scenario:="$scenario" \
        -p planner:="$planner" \
        -p trial_duration:="$TRIAL_DURATION" \
        -p results_dir:="$RESULTS_DIR" \
        > "$LOG_DIR/metrics_${scenario}_${planner}_${trial_id}.log" 2>&1 &

    # Run experiment runner
    ros2 run sar_planning experiment_runner --ros-args \
        -p trial_id:="$trial_id" \
        -p scenario:="$scenario" \
        -p planner:="$planner" \
        -p trial_duration:="$TRIAL_DURATION" \
        > "$LOG_DIR/runner_${scenario}_${planner}_${trial_id}.log" 2>&1

    # Wait for trial duration + buffer
    sleep $((TRIAL_DURATION + 10))

    cleanup_trial
    echo "  [Trial $trial_id] $scenario/$planner — done"
}

###############################################################################
# Main experiment loop
###############################################################################
echo "=========================================="
echo "SAR UAV Swarm — Batch Experiment"
echo "Conditions: ${#SCENARIOS[@]} scenarios x ${#PLANNERS[@]} planners"
echo "Trials per condition: $TRIALS_PER_CONDITION"
echo "Trial duration: ${TRIAL_DURATION}s"
echo "Total trials: $((${#SCENARIOS[@]} * ${#PLANNERS[@]} * TRIALS_PER_CONDITION))"
echo "Started: $(date)"
echo "=========================================="

cleanup_trial   # ensure clean start

GLOBAL_TRIAL=0
for scenario in "${SCENARIOS[@]}"; do
    for planner in "${PLANNERS[@]}"; do
        echo ""
        echo "=== Condition: $scenario / $planner ==="
        for ((t=0; t<TRIALS_PER_CONDITION; t++)); do
            run_trial "$GLOBAL_TRIAL" "$scenario" "$planner"
            GLOBAL_TRIAL=$((GLOBAL_TRIAL + 1))
        done
    done
done

echo ""
echo "=========================================="
echo "ALL EXPERIMENTS COMPLETE"
echo "Total trials run: $GLOBAL_TRIAL"
echo "Finished: $(date)"
echo "Results in: $RESULTS_DIR"
echo "=========================================="