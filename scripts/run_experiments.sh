#!/bin/bash
###############################################################################
# run_experiments.sh
# Orchestrates the full 180-trial experimental batch.
#   9 conditions (3 planners x 3 scenarios) x 20 trials = 180 total
#
# RESUME SUPPORT: if interrupted, re-running this script skips any trials
# already present in the results CSVs and continues from where it stopped.
#
# Usage:  bash run_experiments.sh [trials_per_condition] [trial_duration]
#   e.g.  bash run_experiments.sh 20 120
###############################################################################

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

source /opt/ros/humble/setup.bash
source "$WS_DIR/install/setup.bash"

# --- Hardened cleanup function ---
# Kills every process from the previous trial AND removes any stale PX4
# lock/rootfs artifacts that cause "PX4 server already running" errors.
cleanup_trial() {
    pkill -9 -f "px4"              2>/dev/null || true
    pkill -9 -f "mavros_node"      2>/dev/null || true
    pkill -9 -f "gz sim"           2>/dev/null || true
    pkill -9 -f "parameter_bridge" 2>/dev/null || true
    pkill -9 -f "ruby"             2>/dev/null || true
    pkill -9 -f "sar_planning"     2>/dev/null || true

    # Remove PX4 SITL working directories — these hold the lock files that
    # cause "PX4 server already running for instance N" on the next start.
    rm -rf /tmp/px4_instance0 /tmp/px4_instance1 /tmp/px4_instance2 2>/dev/null || true
    rm -f  /tmp/px4_lock* /tmp/.px4* 2>/dev/null || true

    # The make-based UAV0 launch also leaves a rootfs dir with its own lock
    rm -rf "$PX4_DIR/build/px4_sitl_default/rootfs/lock" 2>/dev/null || true

    sleep 5   # give the OS time to release ports/sockets before next trial
}

# --- Start PX4 instances (3 UAVs: 2 quads + 1 plane) ---
start_px4_instances() {
    cd "$PX4_DIR"
    export GZ_VERSION=harmonic
    export PX4_SIM_SPEED_FACTOR=2.0
    HEADLESS=1 make px4_sitl gz_x500 > "$LOG_DIR/px4_uav0.log" 2>&1 &
    sleep 15

    mkdir -p /tmp/px4_instance1
    ln -sf "$PX4_DIR/build/px4_sitl_default/etc" /tmp/px4_instance1/etc
    ln -sf "$PX4_DIR/build/px4_sitl_default/bin" /tmp/px4_instance1/bin
    export PX4_GZ_MODEL_POSE="10,0,0,0,0,0"
    "$PX4_DIR/build/px4_sitl_default/bin/px4" \
        -i 1 \
        -s "$PX4_DIR/build/px4_sitl_default/etc/init.d-posix/rcS" \
        -w /tmp/px4_instance1 > "$LOG_DIR/px4_uav1.log" 2>&1 &
    sleep 10

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

    ros2 run sar_planning probability_map_node > "$LOG_DIR/probmap.log" 2>&1 &
    ros2 run sar_planning waypoint_selector --ros-args -p scenario:="$scenario" \
        > "$LOG_DIR/waypoint.log" 2>&1 &
    ros2 run sar_planning thermal_camera_node > "$LOG_DIR/thermal.log" 2>&1 &
    ros2 run sar_planning yolo11s_detector > "$LOG_DIR/yolo.log" 2>&1 &
    ros2 run sar_planning rf_doppler_stub --ros-args -p uav_id:=0 -p scenario:="$scenario" \
        > "$LOG_DIR/rf0.log" 2>&1 &
    ros2 run sar_planning rf_doppler_stub --ros-args -p uav_id:=1 -p scenario:="$scenario" \
        > "$LOG_DIR/rf1.log" 2>&1 &
    ros2 run sar_planning rf_doppler_stub --ros-args -p uav_id:=2 -p scenario:="$scenario" \
        > "$LOG_DIR/rf2.log" 2>&1 &
    ros2 run sar_planning cnp_coordinator --ros-args -p scenario:="$scenario" \
        > "$LOG_DIR/cnp.log" 2>&1 &
    ros2 run sar_planning heartbeat_monitor > "$LOG_DIR/heartbeat.log" 2>&1 &
    ros2 run sar_planning plane_bridge --ros-args -p uav_id:=2 \
        > "$LOG_DIR/plane_bridge.log" 2>&1 &

    case "$planner" in
        apf)
            ros2 run sar_planning apf_navigator --ros-args -p uav_id:=0 > "$LOG_DIR/nav_uav0.log" 2>&1 &
            ros2 run sar_planning apf_navigator --ros-args -p uav_id:=1 > "$LOG_DIR/nav_uav1.log" 2>&1 &
            ;;
        vfh)
            ros2 run sar_planning apf_navigator --ros-args -p uav_id:=0 > "$LOG_DIR/nav_uav0.log" 2>&1 &
            ros2 run sar_planning apf_navigator --ros-args -p uav_id:=1 > "$LOG_DIR/nav_uav1.log" 2>&1 &
            ros2 run sar_planning vfh_navigator --ros-args -p uav_id:=0 > "$LOG_DIR/vfh_uav0.log" 2>&1 &
            ros2 run sar_planning vfh_navigator --ros-args -p uav_id:=1 > "$LOG_DIR/vfh_uav1.log" 2>&1 &
            ;;
        rrtstar)
            ros2 run sar_planning apf_navigator --ros-args -p uav_id:=0 > "$LOG_DIR/nav_uav0.log" 2>&1 &
            ros2 run sar_planning apf_navigator --ros-args -p uav_id:=1 > "$LOG_DIR/nav_uav1.log" 2>&1 &
            ros2 run sar_planning vfh_navigator --ros-args -p uav_id:=0 > "$LOG_DIR/vfh_uav0.log" 2>&1 &
            ros2 run sar_planning vfh_navigator --ros-args -p uav_id:=1 > "$LOG_DIR/vfh_uav1.log" 2>&1 &
            ros2 run sar_planning rrtstar_planner --ros-args -p uav_id:=0 > "$LOG_DIR/rrt_uav0.log" 2>&1 &
            ros2 run sar_planning rrtstar_planner --ros-args -p uav_id:=1 > "$LOG_DIR/rrt_uav1.log" 2>&1 &
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

    cleanup_trial

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

    ros2 run sar_planning metrics_logger --ros-args \
        -p trial_id:="$trial_id" \
        -p scenario:="$scenario" \
        -p planner:="$planner" \
        -p trial_duration:="$TRIAL_DURATION" \
        -p results_dir:="$RESULTS_DIR" \
        > "$LOG_DIR/metrics_${scenario}_${planner}_${trial_id}.log" 2>&1 &

    # HARD TIMEOUT: experiment_runner must never be allowed to hang forever.
    # If it doesn't return within trial_duration + 30s buffer, force-kill it
    # so the batch can move on to the next trial instead of stalling.
    timeout --signal=KILL "$((TRIAL_DURATION + 30))" \
    ros2 run sar_planning experiment_runner --ros-args \
        -p trial_id:="$trial_id" \
        -p scenario:="$scenario" \
        -p planner:="$planner" \
        -p trial_duration:="$TRIAL_DURATION" \
        > "$LOG_DIR/runner_${scenario}_${planner}_${trial_id}.log" 2>&1
    RUNNER_EXIT=$?

    if [ "$RUNNER_EXIT" -eq 137 ] || [ "$RUNNER_EXIT" -eq 124 ]; then
        echo "  [Trial $trial_id] $scenario/$planner — WARNING: experiment_runner timed out and was killed"
    fi

    # Kill metrics_logger explicitly too — it may still be waiting on the
    # same stuck condition that caused experiment_runner to hang.
    pkill -9 -f "metrics_logger" 2>/dev/null || true

    sleep 5
    cleanup_trial
    echo "  [Trial $trial_id] $scenario/$planner — done"
}

# --- Count trials already completed for a condition (resume support) ---
count_completed() {
    local scenario=$1
    local planner=$2
    local csv="$RESULTS_DIR/$scenario/${planner}_results.csv"
    if [ -f "$csv" ]; then
        local lines
        lines=$(wc -l < "$csv")
        echo $(( lines > 0 ? lines - 1 : 0 ))   # subtract header row
    else
        echo 0
    fi
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

cleanup_trial

GLOBAL_TRIAL=0
for scenario in "${SCENARIOS[@]}"; do
    for planner in "${PLANNERS[@]}"; do
        COMPLETED=$(count_completed "$scenario" "$planner")

        if [ "$COMPLETED" -ge "$TRIALS_PER_CONDITION" ]; then
            echo ""
            echo "=== Condition: $scenario / $planner — already complete ($COMPLETED/$TRIALS_PER_CONDITION), skipping ==="
            GLOBAL_TRIAL=$((GLOBAL_TRIAL + TRIALS_PER_CONDITION))
            continue
        fi

        echo ""
        if [ "$COMPLETED" -gt 0 ]; then
            echo "=== Condition: $scenario / $planner — resuming from trial $COMPLETED/$TRIALS_PER_CONDITION ==="
        else
            echo "=== Condition: $scenario / $planner ==="
        fi

        GLOBAL_TRIAL=$((GLOBAL_TRIAL + COMPLETED))
        for ((t=COMPLETED; t<TRIALS_PER_CONDITION; t++)); do
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