#!/usr/bin/env bash
# Replay a real-vehicle rosbag through fsae_autonomous's SLAM node (cone_landmark_mapper)
# and record its outputs to a new bag, so `rosbag_cone_map --slam` can turn the final
# SLAM map into a track.
#
#   tuner/tools/rosbag_slam_replay.sh <input_bag_dir> <output_bag_dir> [max_range_m]
#
# Needs FSAE_SLAM_WS: a colcon workspace with fsae_interfaces and fsae_slam built from
# fsae_autonomous (feat/SLAM or later), and FSAE_PARAMS: that checkout's
# common/fsae_bringup/config/fsae_params.yaml (the SLAM noise model and mount yaw). See docs/guides/offline_guide.md, "SLAM-built
# maps", for the build, including GTSAM without sudo.
#
# What it runs, all on simulated time from the bag's /clock:
#   cone_landmark_mapper  with FSAE_PARAMS
#   cone_range_relay.py   drops fused cones beyond max_range (default 7 m), see that file
#   ros2 bag record       /fsae/slam/cone_map, /fsae/slam/path, /fsae/slam/odom_path
#   ros2 bag play         /zed/zed_node/odom, /cone_detection_fused (renamed _raw), /tf, /tf_static
# The replay runs in real time, as long as the bag.
set -e
IN=$(realpath "$1"); OUT=$(realpath -m "$2"); RMAX=${3:-7.0}
: "${FSAE_SLAM_WS:?set FSAE_SLAM_WS to the workspace holding fsae_slam}"
HERE=$(cd "$(dirname "$0")" && pwd)
: "${FSAE_PARAMS:?set FSAE_PARAMS to fsae_autonomous/common/fsae_bringup/config/fsae_params.yaml}"
[ -f "$FSAE_PARAMS" ] || { echo "$FSAE_PARAMS not found" >&2; exit 1; }
[ -e "$OUT" ] && { echo "$OUT already exists" >&2; exit 1; }

set +u
source /opt/ros/"${ROS_DISTRO:-jazzy}"/setup.bash
source "$FSAE_SLAM_WS/install/setup.bash"
# GTSAM_LIB: the GTSAM library folder, when GTSAM is unpacked somewhere ld does not search.
export LD_LIBRARY_PATH=${GTSAM_LIB:+$GTSAM_LIB:}$LD_LIBRARY_PATH
# A private domain: the replay must not see, or be seen by, anything else running.
export ROS_DOMAIN_ID=${ROS_DOMAIN_ID:-87}

LOG="$OUT.log"; mkdir -p "$(dirname "$OUT")"; : > "$LOG"
setsid "$FSAE_SLAM_WS/install/fsae_slam/lib/fsae_slam/cone_landmark_mapper" --ros-args \
    --params-file "$FSAE_PARAMS" -p use_sim_time:=true >> "$LOG" 2>&1 &
SLAM=$!
setsid python3 "$HERE/cone_range_relay.py" --ros-args -p max_range:="$RMAX" \
    -p use_sim_time:=true >> "$LOG" 2>&1 &
RELAY=$!
setsid ros2 bag record --use-sim-time -s sqlite3 -o "$OUT" \
    /fsae/slam/cone_map /fsae/slam/path /fsae/slam/odom_path >> "$LOG" 2>&1 &
REC=$!
cleanup() { kill -INT -$REC -$RELAY -$SLAM 2>/dev/null; sleep 3; kill -9 -$REC -$RELAY -$SLAM 2>/dev/null; }
trap cleanup EXIT
sleep 5   # discovery: everything must be up before the first odometry message plays

ros2 bag play "$IN" --clock 100 --disable-keyboard-controls \
    --topics /zed/zed_node/odom /cone_detection_fused /tf /tf_static \
    --remap /cone_detection_fused:=/cone_detection_fused_raw >> "$LOG" 2>&1
sleep 3
kill -INT -$REC
# The recorder writes metadata.yaml only as it shuts down; give it time, and rebuild the
# metadata if it still did not finish (the cone map is read from the .db3 either way).
for _ in $(seq 30); do [ -f "$OUT/metadata.yaml" ] && break; sleep 0.5; done
[ -f "$OUT/metadata.yaml" ] || ros2 bag reindex "$OUT" -s sqlite3 >> "$LOG" 2>&1
grep "kf=" "$LOG" | tail -1 || echo "SLAM made no keyframes: check $LOG" >&2
echo "wrote $OUT (log $LOG)"
