#!/bin/bash
###############################################################################
# run_experiments.sh
# Orchestrates the full 180-trial experimental batch.
#   9 conditions (3 planners x 3 scenarios) x 20 trials = 180 total
#
# Each trial:
#   1. Starts 3x PX4 SITL instances (UAV0/UAV1 quads + UAV2 plane, headless)
#   2. Starts MAVROS for each UAV
#   3. Launches the planning + detection + coordination stack
#      matching the $planner condition (apf / vfh / rrtstar)
#   4. Runs experiment_runner + metrics_logger for trial_duration
#   5. Logs 5 metrics to CSV
#   6. Cleans up all processes before next trial
#
# Usage:  bash run_experiments.sh [trials_per_condition] [trial_duration]
#   e.g.  bash run_experiments.sh 20 120
###############################################################################

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
    pkill -f "px4" 2>/dev/null || true
    pkill -f "mavros_node" 2>/dev/null || true
    pkill -f "gz sim" 2>/dev/null || true
    pkill -f "ruby" 2>/dev/null || true
    pkill -f "sar_planning" 2>/dev/null || true
    rm -rf /tmp/px4_instance0 /tmp/px4_instance1 /tmp/px4_instance2 2>/dev/null || true
    sleep 3
}

# --- Start PX4 instances (3 UAVs: 2 quads + 1 plane) ---
start_px4_instances() {
    # UAV0 — Iris quadrotor
    cd "$PX4_DIR"
    export GZ_VERSION=harmonic
    export PX4_SIM_SPEED_FACTOR=2.0
    mkdir -p /tmp/px4_instance0
    ln -sf "$PX4_DIR/build/px4_sitl_default/etc" /tmp/px4_instance0/etc
    ln -sf "$PX4_DIR/build/px4_sitl_default/bin" /tmp/px4_instance0/bin
    export PX4_GZ_MODEL=x500
    export PX4_GZ_MODEL_POSE="0,0,0,0,0,0"
    "$PX4_DIR/build/px4_sitl_default/bin/px4" \
        -i 0 \
        -s "$PX4_DIR/build/px4_sitl_default/etc/init.d-posix/rcS" \
        -w /tmp/px4_instance0 > "$LOG_DIR/px4_uav0.log" 2>&1 &
    sleep 15

    # UAV1 — Iris quadrotor
    mkdir -p /tmp/px4_instance1
    ln -sf "$PX4_DIR/build/px4_sitl_default/etc" /tmp/px4_instance1/etc
    ln -sf "$PX4_DIR/build/px4_sitl_default/bin" /tmp/px4_instance1/bin
    export PX4_GZ_MODEL_POSE="10,0,0,0,0,0"
    "$PX4_DIR/build/px4_sitl_default/bin/px4" \
        -i 1 \
        -s "$PX4_DIR/build/px4_sitl_default/etc/init.d-posix/rcS" \
        -w /tmp/px4_instance1 > "$LOG_DIR/px4_uav1.log" 2>&1 &
    sleep 10

    # UAV2 — Standard Plane (fixed-wing)
    mkdir -p /tmp/px4_instance2
    ln -sf "$PX4_DIR/build/px4_sitl_default/etc" /tmp/px4_instance2/etc
    ln -sf "$PX4_DIR/build/px4_sitl_default/bin" /tmp/px4_instance2/bin
    export PX4_GZ_MODEL=standard_vtol
    export PX4_GZ_MODEL_POSE="20,0,0,0,0,0"
    "$PX4_DIR/build/px4_sitl_default/bin/px4" \
        -i 2 \
        -s "$PX4_DIR/build/px4_sitl_default/etc/init.d-posix/rcS" \
        -w /tmp/px4_instance2 > "$LOG_DIR/px4_uav2.log" 2>&1 &
    sleep 10
    unset PX4_GZ_MODEL
}

# --- Start MAVROS (3 UAVs) ---
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
    sleep 5

    ros2 run mavros mavros_node --ros-args \
        -p fcu_url:=udp://:14542@localhost:14570 \
        -p system_id:=3 --remap __ns:=/uav2 \
        > "$LOG_DIR/mavros_uav2.log" 2>&1 &
    sleep 8
}

# --- Start the stack for a trial — planner-dependent ---
start_stack() {
    local scenario=$1
    local planner=$2

    # Shared: probability map + waypoint selector + detection (all conditions)
    ros2 run sar_planning probability_map_node \
        > "$LOG_DIR/probmap.log" 2>&1 &
    ros2 run sar_planning waypoint_selector \
        --ros-args -p scenario:="$scenario" \
        > "$LOG_DIR/waypoint.log" 2>&1 &
    ros2 run sar_planning thermal_camera_node \
        > "$LOG_DIR/thermal.log" 2>&1 &
    ros2 run sar_planning yolo11s_detector \
        > "$LOG_DIR/yolo.log" 2>&1 &
    ros2 run sar_planning rf_doppler_stub --ros-args -p uav_id:=0 -p scenario:="$scenario" \
        > "$LOG_DIR/rf0.log" 2>&1 &
    ros2 run sar_planning rf_doppler_stub --ros-args -p uav_id:=1 -p scenario:="$scenario" \
        > "$LOG_DIR/rf1.log" 2>&1 &
    ros2 run sar_planning rf_doppler_stub --ros-args -p uav_id:=2 -p scenario:="$scenario" \
        > "$LOG_DIR/rf2.log" 2>&1 &
    ros2 run sar_planning cnp_coordinator \
        --ros-args -p scenario:="$scenario" \
        > "$LOG_DIR/cnp.log" 2>&1 &
    ros2 run sar_planning heartbeat_monitor \
        > "$LOG_DIR/heartbeat.log" 2>&1 &

    # UAV2 (fixed-wing) — same bridge in every condition; it always needs
    # OFFBOARD position setpoints regardless of which quad planner is tested
    ros2 run sar_planning plane_bridge --ros-args -p uav_id:=2 \
        > "$LOG_DIR/plane_bridge.log" 2>&1 &

    # ── Planner-dependent navigation stack for UAV0 / UAV1 ──────────────
    # This is the actual independent variable of the experiment.
    case "$planner" in
        apf)
            # APF-only: local reactive layer, no mid/global planning
            ros2 run sar_planning apf_navigator --ros-args -p uav_id:=0 \
                > "$LOG_DIR/nav_uav0.log" 2>&1 &
            ros2 run sar_planning apf_navigator --ros-args -p uav_id:=1 \
                > "$LOG_DIR/nav_uav1.log" 2>&1 &
            ;;
        vfh)
            # APF (local) + VFH+ (mid-scale)
            ros2 run sar_planning apf_navigator --ros-args -p uav_id:=0 \
                > "$LOG_DIR/nav_uav0.log" 2>&1 &
            ros2 run sar_planning apf_navigator --ros-args -p uav_id:=1 \
                > "$LOG_DIR/nav_uav1.log" 2>&1 &
            ros2 run sar_planning vfh_navigator --ros-args -p uav_id:=0 \
                > "$LOG_DIR/vfh_uav0.log" 2>&1 &
            ros2 run sar_planning vfh_navigator --ros-args -p uav_id:=1 \
                > "$LOG_DIR/vfh_uav1.log" 2>&1 &
            ;;
        rrtstar)
            # Full 3-scale hierarchy: APF + VFH+ + RRT* global planner
            ros2 run sar_planning apf_navigator --ros-args -p uav_id:=0 \
                > "$LOG_DIR/nav_uav0.log" 2>&1 &
            ros2 run sar_planning apf_navigator --ros-args -p uav_id:=1 \
                > "$LOG_DIR/nav_uav1.log" 2>&1 &
            ros2 run sar_planning vfh_navigator --ros-args -p uav_id:=0 \
                > "$LOG_DIR/vfh_uav0.log" 2>&1 &
            ros2 run sar_planning vfh_navigator --ros-args -p uav_id:=1 \
                > "$LOG_DIR/vfh_uav1.log" 2>&1 &
            ros2 run sar_planning rrtstar_planner --ros-args -p uav_id:=0 \
                > "$LOG_DIR/rrt_uav0.log" 2>&1 &
            ros2 run sar_planning rrtstar_planner --ros-args -p uav_id:=1 \
                > "$LOG_DIR/rrt_uav1.log" 2>&1 &
            ;;
        *)
            echo "ERROR: unknown planner '$planner' — skipping trial"
            return 1
            ;;
    esac

    sleep 5
}

# --- Run a single trial ---
run_trial() {
    local trial_id=$1
    local scenario=$2
    local planner=$3

    echo "  [Trial $trial_id] $scenario/$planner — starting..."

    # Clean up any leftover processes from previous trial
    cleanup_trial

    # Truncate logs at start of each trial to prevent disk overflow
    : > "$LOG_DIR/px4_uav0.log"
    : > "$LOG_DIR/px4_uav1.log"
    : > "$LOG_DIR/px4_uav2.log"
    : > "$LOG_DIR/cnp.log"
    : > "$LOG_DIR/yolo.log"
    : > "$LOG_DIR/mavros_uav0.log"
    : > "$LOG_DIR/mavros_uav1.log"
    : > "$LOG_DIR/mavros_uav2.log"

    start_px4_instances
    start_mavros

    if ! start_stack "$scenario" "$planner"; then
        cleanup_trial
        return 1
    fi

    # Run metrics logger in background — it logs for trial_duration
    ros2 run sar_planning metrics_logger --ros-args \
        -p trial_id:="$trial_id" \
        -p scenario:="$scenario" \
        -p planner:="$planner" \
        -p trial_duration:="$TRIAL_DURATION" \
        -p results_dir:="$RESULTS_DIR" \
        > "$LOG_DIR/metrics_${scenario}_${planner}_${trial_id}.log" 2>&1 &

    # Run experiment runner in FOREGROUND — this call already blocks for
    # trial_duration internally, so no extra sleep is needed after it returns.
    ros2 run sar_planning experiment_runner --ros-args \
        -p trial_id:="$trial_id" \
        -p scenario:="$scenario" \
        -p planner:="$planner" \
        -p trial_duration:="$TRIAL_DURATION" \
        > "$LOG_DIR/runner_${scenario}_${planner}_${trial_id}.log" 2>&1

    # Small buffer only — NOT trial_duration again. experiment_runner already
    # consumed trial_duration seconds while running in the foreground above.
    sleep 5

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
echo "Estimated runtime: ~$(( (${#SCENARIOS[@]} * ${#PLANNERS[@]} * TRIALS_PER_CONDITION * (TRIAL_DURATION + 45)) / 3600 )) hours"
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