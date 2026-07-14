#!/bin/bash
###############################################################################
# run_synthetic_validation.sh  (v2 — corrected per-planner topic wiring)
#
# Standalone planner validation — bypasses PX4/Gazebo/MAVROS-arming entirely.
#
# Three conditions, matching each planner's REAL topic contract:
#   apf            -> synthetic_pose_publisher.py --mode apf
#                      (publishes /sar/waypoints PoseArray, integrates via
#                       /uavN/mavros/setpoint_velocity/cmd_vel_unstamped)
#   apf_vfh        -> apf_navigator + vfh_navigator both running;
#                      synthetic_pose_publisher.py --mode vfh
#                      (publishes /uavN/goal_waypoint, integrates via
#                       /uavN/vfh/cmd_vel -- this is VFH+'s own output,
#                       which is what actually moves the vehicle in this tier)
#   full_hierarchy -> apf_navigator + vfh_navigator + rrtstar_planner running;
#                      synthetic_pose_publisher.py --mode vfh handles physics
#                      (same integration point as apf_vfh, since VFH+ is still
#                       the final velocity source), while a SEPARATE short
#                      --mode rrtstar run confirms RRT* itself is alive and
#                      producing goal_waypoint output, logged separately for
#                      latency reporting (not physics-integrated, since RRT*
#                      does not output a velocity).
#
# Usage: bash run_synthetic_validation.sh
###############################################################################

WS_DIR=~/thesis_ws/uav-swarm-sar/ros2_ws
RESULTS_DIR=~/thesis_ws/results/synthetic
LOG_DIR=~/thesis_ws/logs/synthetic

mkdir -p "$RESULTS_DIR" "$LOG_DIR"

source /opt/ros/humble/setup.bash
source ~/thesis_ws/install/setup.bash
source "$WS_DIR/install/setup.bash"
export PYTHONUNBUFFERED=1

SCENARIOS=("direct" "diagonal" "long_range")
UAV_ID=0

cleanup_nodes() {
    pkill -9 -f "apf_navigator"              2>/dev/null || true
    pkill -9 -f "vfh_navigator"               2>/dev/null || true
    pkill -9 -f "rrtstar_planner"             2>/dev/null || true
    pkill -9 -f "synthetic_pose_publisher.py" 2>/dev/null || true
    sleep 2
}

run_apf() {
    local scenario=$1
    echo ""
    echo "=== apf / $scenario ==="
    cleanup_nodes

    ros2 run sar_planning apf_navigator --ros-args -p uav_id:="$UAV_ID" \
        > "$LOG_DIR/nav_apf_${scenario}.log" 2>&1 &
    sleep 5

    python3 ~/thesis_ws/uav-swarm-sar/scripts/synthetic_pose_publisher.py \
        --uav_id "$UAV_ID" --mode apf --scenario "$scenario" --results_dir "$RESULTS_DIR" \
        2>&1 | tee "$LOG_DIR/synpub_apf_${scenario}.log"

    cleanup_nodes
    echo "=== apf / $scenario — done ==="
}

run_apf_vfh() {
    local scenario=$1
    echo ""
    echo "=== apf_vfh / $scenario ==="
    cleanup_nodes

    ros2 run sar_planning apf_navigator --ros-args -p uav_id:="$UAV_ID" \
        > "$LOG_DIR/nav_apfvfh_${scenario}.log" 2>&1 &
    ros2 run sar_planning vfh_navigator --ros-args -p uav_id:="$UAV_ID" \
        > "$LOG_DIR/vfh_apfvfh_${scenario}.log" 2>&1 &
    sleep 5

    python3 ~/thesis_ws/uav-swarm-sar/scripts/synthetic_pose_publisher.py \
        --uav_id "$UAV_ID" --mode vfh --scenario "$scenario" --results_dir "$RESULTS_DIR" \
        2>&1 | tee "$LOG_DIR/synpub_apfvfh_${scenario}.log"

    cleanup_nodes
    echo "=== apf_vfh / $scenario — done ==="
}

run_full_hierarchy() {
    local scenario=$1
    echo ""
    echo "=== full_hierarchy / $scenario ==="
    cleanup_nodes

    ros2 run sar_planning apf_navigator --ros-args -p uav_id:="$UAV_ID" \
        > "$LOG_DIR/nav_full_${scenario}.log" 2>&1 &
    ros2 run sar_planning vfh_navigator --ros-args -p uav_id:="$UAV_ID" \
        > "$LOG_DIR/vfh_full_${scenario}.log" 2>&1 &
    ros2 run sar_planning rrtstar_planner --ros-args -p uav_id:="$UAV_ID" \
        > "$LOG_DIR/rrt_full_${scenario}.log" 2>&1 &
    sleep 5

    # Physics/motion integration happens via VFH+'s cmd_vel, same as apf_vfh tier
    python3 ~/thesis_ws/uav-swarm-sar/scripts/synthetic_pose_publisher.py \
        --uav_id "$UAV_ID" --mode vfh --scenario "$scenario" --results_dir "$RESULTS_DIR" \
        2>&1 | tee "$LOG_DIR/synpub_full_vfh_${scenario}.log"

    cleanup_nodes

    # Separately confirm RRT* itself produces output (latency-only, not integrated)
    ros2 run sar_planning rrtstar_planner --ros-args -p uav_id:="$UAV_ID" \
        > "$LOG_DIR/rrt_full_standalone_${scenario}.log" 2>&1 &
    sleep 2

    python3 ~/thesis_ws/uav-swarm-sar/scripts/synthetic_pose_publisher.py \
        --uav_id "$UAV_ID" --mode rrtstar --scenario "$scenario" --results_dir "$RESULTS_DIR" \
        2>&1 | tee "$LOG_DIR/synpub_full_rrt_${scenario}.log"

    cleanup_nodes
    echo "=== full_hierarchy / $scenario — done ==="
}

echo "=========================================="
echo "Synthetic Planner Validation (no PX4/Gazebo)"
echo "Conditions: apf, apf_vfh, full_hierarchy"
echo "Scenarios: ${SCENARIOS[*]}"
echo "Started: $(date)"
echo "=========================================="

for scenario in "${SCENARIOS[@]}"; do
    run_apf "$scenario"
    run_apf_vfh "$scenario"
    run_full_hierarchy "$scenario"
done

echo ""
echo "=========================================="
echo "SYNTHETIC VALIDATION COMPLETE"
echo "Results in: $RESULTS_DIR"
echo "Finished: $(date)"
echo "=========================================="
