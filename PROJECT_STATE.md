# UAV Swarm SAR — Project State
# Last updated: June 7, 2026

## Environment
- Droplet: root@64.227.35.132 (uav-swarm-sim, LON1, Ubuntu 22.04, 4vCPU/8GB)
- Local: WSL2 Ubuntu 24.04, ROS 2 Jazzy, Gazebo Harmonic
- Repo: https://github.com/jimohh/uav-swarm-sar
- ROS on Droplet: Humble
- SSH note: ISP sometimes blocks port 22 — fix: ssh -p 443 root@64.227.35.132

## Completed Days
- Day 1: PX4 SITL + MAVROS connected (gz_x500, HEADLESS=1, PX4_SIM_SPEED_FACTOR=0.5)
- Day 2: probability_map_node — /sar/prob_map/{urban,wilderness,maritime} at 1Hz verified
- Day 3: waypoint_selector + apf_navigator + ekf_node — full planning loop verified
- Day 4: thermal_camera_node (10Hz) + yolo11s_detector (model loaded) verified. rf_doppler_stub has packaging issue UNRESOLVED
- Day 5: cnp_coordinator + heartbeat_monitor — task announcement/award verified with 2 UAVs
- Day 6 (partial): experiment script written but has sourcing bug UNRESOLVED

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

## Nodes Implemented
- probability_map_node.py
- waypoint_selector.py
- apf_navigator.py
- ekf_node.py
- thermal_camera_node.py
- rf_doppler_stub.py (packaging bug unresolved)
- yolo11s_detector.py
- cnp_coordinator.py
- heartbeat_monitor.py
- metrics_logger.py
- experiment_runner.py

## Unresolved Issues
1. rf_doppler_stub: AttributeError module has no attribute 'main'
2. run_experiments.sh: exits silently — ROS sourcing causes early exit
3. UAV2 (Standard Plane) not yet added to experiment script or MAVROS
4. VFH navigator not implemented
5. RRT* planner not implemented

## Revised 7-Day Plan (from June 7)
- Day 1: Implement vfh_navigator.py + rrtstar_planner.py
- Day 2: Add UAV2 (fixed-wing) — PX4 instance 2, MAVROS, CNP routing
- Day 3: Fix rf_doppler_stub + full detection pipeline validation
- Day 4: Full 3-agent swarm integration test
- Day 5: Fix experiment script + smoke test (9 trials)
- Day 6: Full 180-trial batch run overnight
- Day 7: ANOVA + Tukey HSD + figures + thesis Chapters 4-5

## Architecture
- Swarm: UAV0 (Iris quadrotor) + UAV1 (Iris quadrotor) + UAV2 (Standard Plane)
- Navigation: APF (local) / VFH (mid) / RRT* (global)
- Detection: YOLO11s (C3k2+C2PSA, conf=0.45, 10-frame window, Pd=0.85) + CW Doppler RF (Pd=0.45)
- Coordination: Decentralised auction CNP — NOT leader-follower
- Experimental: 9 conditions x 20 trials = 180 total, two-factor ANOVA + Tukey HSD a=0.05
- Five metrics: coverage_rate, time_to_detection, path_efficiency, inter_agent_distance, fault_recovery_time

## Disk Management
- Droplet disk cleared — logs removed (136GB). Current usage ~13%
- Log truncation added to run_trial() to prevent recurrence

## GitHub
- Repo: https://github.com/jimohh/uav-swarm-sar
- Auth: PAT stored via git credential.helper store
- WSL2 alias: droplet = ssh root@64.227.35.132

# PROJECT_STATE.md — UAV Swarm SAR Thesis

**Last updated:** 2026-07-08 22:00 UTC

## Purpose & context
Final-year thesis: "Heterogeneous UAV Swarm: Design and Development of a Hybrid
Control Architecture for Distributed Search and Rescue Operations." Comparing
APF / VFH+ / Informed RRT* navigation planners across urban/wilderness/maritime
SAR scenarios using a 3-UAV heterogeneous swarm (2× Iris quad + 1× Standard
Plane) in ROS 2 Humble / Gazebo Harmonic / PX4 SITL on a DigitalOcean Droplet
(IP 64.227.35.132, 4vCPU/8GB, London).

Experimental design: 3 planners × 3 scenarios × 20 trials = 180-trial full
factorial, two-factor ANOVA + Bonferroni-corrected pairwise comparisons, α=0.05.
Five metrics logged per trial: coverage_rate, time_to_detection, path_efficiency,
inter_agent_distance, fault_recovery_time.

## Infrastructure
- **Droplet:** `uav-swarm-sim`, 64.227.35.132, Ubuntu 22.04, London (LON1)
- **Repo:** `github.com/jimohh/uav-swarm-sar` (PUBLIC)
- **WSL2 path:** `~/uav-swarm-sar`
- **Droplet path:** `~/thesis_ws/uav-swarm-sar`
- **ROS workspace:** `~/thesis_ws/uav-swarm-sar/ros2_ws`
- **Results dir:** `~/thesis_ws/results/{urban,wilderness,maritime}/`
- **Analysis dir:** `~/thesis_ws/analysis/`
- **tmux session:** `swarm` with windows: uav0-px4, uav1-px4, uav2-px4,
  mavros0, mavros1, mavros2, ros-nodes, monitor, shell
- **Workflow:** edit in VS Code (WSL2) → git push → git pull on Droplet → rebuild
- **SSH fallback:** port 443 configured in sshd_config if port 22 blocked

## Revised 7-day plan status
- Day 1 (VFH+ & RRT*): ✅ Complete
- Day 2 (UAV2 Standard Plane): ✅ Complete
- Day 3 (rf_doppler fix): ✅ Complete
- Day 4 (3-agent integration test): ✅ Complete
- Day 5 (script fixes): ✅ Complete
- Day 6 (180 trials): 🔄 In progress — 3rd attempt running (smoke test at 120s
  just confirmed real data flowing; full batch about to launch)
- Day 7 (ANOVA + thesis): ⏳ Pending

## Current state — Day 6, launching 3rd (clean) batch run

### Bugs fixed (all confirmed resolved, do NOT reintroduce)

**1. rf_doppler_stub.py was empty**
→ Rewritten from scratch (Day 3)

**2. PX4 stale lock files**
→ `cleanup_trial()` now does `pkill -9` + removes `/tmp/px4_instance*` +
  `rm -rf $PX4_DIR/build/px4_sitl_default/rootfs/lock` before every trial

**3. rclpy.shutdown() race condition (experiment_runner + metrics_logger)**
→ Removed `rclpy.shutdown()` from `_end_trial()` timer callbacks in both files.
  Replaced `rclpy.spin()` in `main()` with manual
  `while rclpy.ok() and not node.trial_complete: rclpy.spin_once(...)` loop.

**4. trial_duration INTEGER vs DOUBLE mismatch**
→ Both `metrics_logger.py` and `experiment_runner.py` now declare
  `trial_duration` as integer default (`120` not `120.0`).

**5. experiment_runner hanging forever (no timeout)**
→ Wrapped `experiment_runner` call in `run_experiments.sh` with
  `timeout --signal=KILL "$((TRIAL_DURATION + 30))"`.
  Also explicitly kills `metrics_logger` after timeout.

**6. Missing UAV2 in experiments**
→ Added PX4 instance 2 (standard_vtol), MAVROS for uav2, and `plane_bridge`
  to `run_experiments.sh`.

**7. Planner never switched between conditions**
→ Added `case "$planner"` block in `start_stack()` launching:
  - apf: apf_navigator only
  - vfh: apf_navigator + vfh_navigator
  - rrtstar: apf_navigator + vfh_navigator + rrtstar_planner

**8. Double-length trials**
→ Removed redundant `sleep $TRIAL_DURATION` after `experiment_runner` already
  blocked for that duration internally.

**9. MAVROS wrong namespace remap**
→ Changed `--remap __ns:=/uav{N}` to `--remap __ns:=/uav{N}/mavros` in both
  `run_experiments.sh` (start_mavros function) and `launch_swarm.sh`.
  This was causing ALL MAVROS topics to publish under `/uav{N}/...` instead of
  the `/uav{N}/mavros/...` path that every Python node subscribes to.

**10. apf_navigator wrong parameter name (critical)**
→ `apf_navigator.py` was declaring `uav_ns` but `run_experiments.sh` was
  passing `uav_id`. Both UAV0 and UAV1 APF navigators silently defaulted to
  `uav0` every trial. Fixed: changed `declare_parameter('uav_ns', 'uav0')` to
  `declare_parameter('uav_id', 0)` and derived `ns = f'uav{uav_id}'`.
  This was causing `inter_agent_distance` and `path_efficiency` to be zero.

**11. QoS mismatch on odometry subscriptions (critical)**
→ MAVROS publishes `/uav{N}/mavros/local_position/odom` with
  QoS=BEST_EFFORT. Both `metrics_logger.py` and `apf_navigator.py` were
  subscribing with default QoS=RELIABLE. In ROS 2/DDS a RELIABLE subscriber
  silently never connects to a BEST_EFFORT publisher — no error, just zero data.
  Fixed: added explicit `QoSProfile(reliability=ReliabilityPolicy.BEST_EFFORT,
  history=HistoryPolicy.KEEP_LAST, depth=10)` to both odometry subscriptions.

**12. tmux window naming/targeting bugs in launch_swarm.sh**
→ Fixed by creating all windows upfront before sending any keys, using
  `-n "window-name"` at creation time instead of separate rename commands.

**13. Resume feature added to run_experiments.sh**
→ `count_completed()` function checks existing CSV row counts before each
  condition; skips already-complete conditions automatically on relaunch.

## Immediate next steps (in new chat)
1. **Confirm smoke test (1 trial × 9 conditions × 120s) shows non-zero
   coverage_rate, path_efficiency, inter_agent_distance** in all 9 CSVs
2. If confirmed: clear all results and launch full 180-trial batch:
```bash
   rm -f ~/thesis_ws/results/urban/*.csv
   rm -f ~/thesis_ws/results/wilderness/*.csv
   rm -f ~/thesis_ws/results/maritime/*.csv
   rm -rf ~/thesis_ws/logs/*
   cd ~/thesis_ws/uav-swarm-sar
   nohup bash scripts/run_experiments.sh 20 120 > ~/thesis_ws/logs/batch_run.log 2>&1 &
   echo $!
```
3. Monitor periodically:
```bash
   tail -5 ~/thesis_ws/logs/batch_run.log
   watch -n 30 'find ~/thesis_ws/results -name "*.csv" -exec tail -1 {} \; -print'
```
4. Once complete: run analysis:
```bash
   python3 scripts/analyse_results.py ~/thesis_ws/results ~/thesis_ws/analysis
```
5. Populate `chapter4_results_skeleton.docx` with real figures and stats

## Key files
- `scripts/run_experiments.sh` — main batch orchestration (resume-capable)
- `scripts/launch_swarm.sh` — interactive manual launch
- `scripts/analyse_results.py` — generates 7 figures + ANOVA + summary CSV
- `scripts/record_demo_bag.sh` — records rosbag for RViz playback on WSL2
- `ros2_ws/src/sar_planning/sar_planning/` — all ROS 2 nodes
- `ros2_ws/src/sar_planning/launch/` — nav_stack.launch.py, rviz_visualization.launch.py
- `ros2_ws/src/sar_planning/rviz/sar_swarm.rviz` — RViz config
- `chapter4_results_skeleton.docx` — thesis Chapter 4 template with placeholders

## Nodes implemented
apf_navigator, vfh_navigator, rrtstar_planner, plane_bridge, rf_doppler_stub,
cnp_coordinator, heartbeat_monitor, metrics_logger, experiment_runner,
probability_map_node, waypoint_selector, thermal_camera_node, yolo11s_detector,
ekf_node

## Analysis outputs (once batch completes)
- `~/thesis_ws/analysis/figures/bar_coverage_rate.png`
- `~/thesis_ws/analysis/figures/bar_time_to_detection.png`
- `~/thesis_ws/analysis/figures/bar_path_efficiency.png`
- `~/thesis_ws/analysis/figures/bar_inter_agent_distance.png`
- `~/thesis_ws/analysis/figures/bar_fault_recovery_time.png`
- `~/thesis_ws/analysis/figures/boxplot_all_metrics.png`
- `~/thesis_ws/analysis/figures/heatmap_performance.png`
- `~/thesis_ws/analysis/summary_stats.csv`
- `~/thesis_ws/analysis/anova_results.txt`


## Session July 11-12 2026 — Major Debugging Findings

### Root causes of non-arming (all confirmed from PX4 source code):

1. MAVROS heartbeat type was ONBOARD_CONTROLLER, not GCS
   → Fixed: heartbeat_mav_type:=GCS in all MAVROS launches

2. PX4_SIM_SPEED_FACTOR env var caused SIH (internal) simulator
   instead of Gazebo sensor pipeline
   → Fixed: PX4_PARAM_SIM_GZ_EN=1 env var

3. No accelerometer auto-calibration at boot (gyro had it, accel didn't)
   → Fixed: commander calibrate accel quick added to rcS

4. Gazebo gz_x500 model has NO magnetometer sensor
   → EKF2 has no heading source → "no heading reference" arming block
   → Fix: switched UAV0/UAV1 to standard_vtol model (has magnetometer)

5. arm_and_offboard() bash service call fires at fixed time before
   EKF2 yaw has converged
   → Chosen fix: MAVSDK mavsdk_arm.py with health.is_armable wait

### Batch data status:
- ALL previous batch data INVALID (quadrotors never armed/flew)
- Must run clean 5-trial batch from scratch once arming is fixed
- coverage_rate was frozen at ~0.001 in all prior runs

### Next action:
- Install MAVSDK, write mavsdk_arm.py, smoke test, full batch
- See summary for exact steps

### quad_bridge.py — abandoned approach
- Written and built successfully but never launched correctly due to
  source path issue in run_experiments.sh
- SUPERSEDED by mavsdk_arm.py approach
- Can be deleted from the repo or left as dead code — does not affect anything