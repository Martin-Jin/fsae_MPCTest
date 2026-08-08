#!/bin/bash
# Language: bash
# Title: One-shot launcher for the open-loop steering system-ID experiment
#
# NOTE: the shebang MUST stay on line 1 — see the same note in
# run_steering_step.sh. Below the header comments the kernel never sees it and
# the script inherits the caller's shell (dash dies on `set -o pipefail`).
#
# Runs the whole experiment from a single terminal:
#   1. starts FSDS on Windows
#   2. starts the fsds_ros2_bridge
#   3. waits for the RPC + odom to actually come up
#   4. runs the steering_sysid sweep in the foreground
#   5. analyses the resulting log
#   6. tears everything down on exit or Ctrl+C
#
# This is a DIAGNOSTIC harness, deliberately separate from launch_all.sh:
# it must NOT start the planning/control stack, because any other node
# publishing to /fsds/control_command would interleave with the sweep's
# commands and corrupt the measurement.
#
# Why the experiment exists: see
#   fsae_MPCTest/docs/planning_control_sync.md
#   -> "MEASURED: the car's yaw response is ~3x weaker than commanded"
#
# Usage:
#   ./run_steering_sysid.sh                       # defaults (25 points, ~6 min)
#   ./run_steering_sysid.sh --quick               # 12 points, ~3 min
#   ./run_steering_sysid.sh --no-sim              # FSDS already running
#   ./run_steering_sysid.sh -p home_radius:=25.0  # pass through to the node

set -o pipefail

# This script itself lives in <mpcTest repo root>/fsds_simulator/ (see
# launch_all.sh's identical convention) -- the actual ROS 2 workspace is the
# separate directory these paths were adapted from (see launch_all.sh's
# HOST_ROS2_DIR); edit HOST_ROS2_DIR below to point at your own workspace.
HOST_ROS2_DIR="/home/Formula-Student-Driverless-Simulator/ros2"
HOST_REPO_ROOT="$(dirname "$HOST_ROS2_DIR")"
SIM_DIR_WIN='C:\Users\Martin\Downloads\fsds-v2.2.0-windows'
SIM_DIR_WSL="/mnt/c/Users/Martin/Downloads/fsds-v2.2.0-windows"
# Matches launch_all.sh's log_dir:=$HOST_REPO_ROOT/fsae_logs -- keeps every
# diagnostic script's logs in the same place instead of the ~/fsae_logs
# default telemetry_logger.py falls back to.
LOG_DIR="$HOST_REPO_ROOT/fsae_logs"
HOST_IP="$(ip route show default | awk '{print $3}')"

START_SIM=true
EXTRA_ARGS=()
for a in "$@"; do
    case "$a" in
        --no-sim) START_SIM=false ;;
        --quick)  EXTRA_ARGS+=(-p 'speeds:=[3.0, 6.0, 10.0, 14.0]'
                               -p 'steer_cmds:=[0.3, 0.6, 1.0]') ;;
        *)        EXTRA_ARGS+=("$a") ;;
    esac
done

cleanup() {
    echo ""
    echo "============================================="
    echo "🛑 Cleaning up..."
    echo "============================================="
    [ -n "$SYSID_PID" ] && kill "$SYSID_PID" 2>/dev/null
    if [ -n "$BRIDGE_PID" ]; then
        echo "Stopping ROS 2 bridge (PID $BRIDGE_PID)..."
        kill "$BRIDGE_PID" 2>/dev/null
        sleep 1
        kill -9 "$BRIDGE_PID" 2>/dev/null
    fi
    pkill -f 'fsds_ros2_bridge' 2>/dev/null
    pkill -f 'steering_sysid'  2>/dev/null
    if [ "$START_SIM" = true ]; then
        echo "Terminating Windows FSDS instances..."
        taskkill.exe /F /T /IM "FSDS.exe"     2>/dev/null
        taskkill.exe /F /T /IM "FSOnline.exe" 2>/dev/null
        taskkill.exe /F /T /IM "Blocks.exe"   2>/dev/null
    fi
    find "$HOST_ROS2_DIR" -maxdepth 1 -type f -name 'core.[0-9]*' -delete 2>/dev/null
    echo "✅ Done."
    exit 0
}
trap cleanup SIGINT SIGTERM

echo "============================================="
echo "🔬 Open-loop steering system-ID"
echo "============================================="

# --- 0. Refuse to run alongside a controller -------------------------------
# Two publishers on /fsds/control_command produce interleaved commands and a
# useless log.  Catch it here rather than after a 6-minute run.
if pgrep -f 'mpc_controller|fsds_bridge|stanley' >/dev/null 2>&1; then
    echo "❌ A controller node is already running (mpc_controller / fsds_bridge /"
    echo "   stanley).  It would fight the sweep for /fsds/control_command."
    echo "   Stop it first — e.g. pkill -f 'sim.launch|mpc_controller|fsds_bridge'"
    exit 1
fi

# --- 1. Simulator -----------------------------------------------------------
if [ "$START_SIM" = true ]; then
    # Duplicate instances both bind :41451; connections then land on the wrong
    # one and hang in CloseWait ("rpc::timeout ... getServerVersion").
    echo "[1/5] Clearing any existing FSDS instances..."
    taskkill.exe /F /T /IM "FSDS.exe"   2>/dev/null
    taskkill.exe /F /T /IM "Blocks.exe" 2>/dev/null
    sleep 2
    if [ -d "$SIM_DIR_WSL" ]; then
        echo "[1/5] Starting FSDS..."
        cmd.exe /c "cd /d $SIM_DIR_WIN && FSDS.exe -windowed -ResX=600 -ResY=500" &
    else
        echo "⚠️  FSDS folder not found at $SIM_DIR_WSL"; exit 1
    fi
else
    echo "[1/5] --no-sim: using the already-running FSDS."
fi

# --- 2. Wait for the RPC server --------------------------------------------
echo "[2/5] Waiting for FSDS RPC at $HOST_IP:41451 ..."
for i in $(seq 1 60); do
    if timeout 2 python3 -c "
import socket,sys
try:
    s=socket.create_connection(('$HOST_IP',41451),timeout=1.5); s.settimeout(1.5)
    s.sendall(b'\x94\x00\x00\xa10\x90')
    sys.exit(0 if s.recv(64) else 1)
except Exception: sys.exit(1)
" 2>/dev/null; then
        echo "      RPC responding after ${i}s."
        break
    fi
    [ "$i" -eq 60 ] && { echo "❌ FSDS RPC never responded. Is the map loaded?"; cleanup; }
    sleep 1
done

# --- 3. Bridge --------------------------------------------------------------
echo "[3/5] Starting fsds_ros2_bridge..."
bash -c "
    source /opt/ros/jazzy/setup.bash && \
    cd '$HOST_ROS2_DIR' && \
    source install/local_setup.bash && \
    ros2 launch fsds_ros2_bridge fsds_ros2_bridge.launch.py host:=$HOST_IP
" > /tmp/sysid_bridge.log 2>&1 &
BRIDGE_PID=$!

echo "[4/5] Waiting for /fsds/testing_only/odom ..."
ODOM_OK=false
for i in $(seq 1 45); do
    if source /opt/ros/jazzy/setup.bash 2>/dev/null && \
       cd "$HOST_ROS2_DIR" && source install/local_setup.bash 2>/dev/null && \
       timeout 4 ros2 topic echo --once /fsds/testing_only/odom >/dev/null 2>&1; then
        ODOM_OK=true; echo "      odom flowing after ${i}s."; break
    fi
    sleep 1
done
if [ "$ODOM_OK" != true ]; then
    echo "❌ odom never arrived. Bridge log:"; tail -20 /tmp/sysid_bridge.log; cleanup
fi

# --- 4. The sweep -----------------------------------------------------------
echo ""
echo "============================================="
echo "[5/5] Running the sweep."
echo "  ⚠️  PRESS GO IN FSDS if it waits for the signal."
echo "  The car circles near its start point (geofenced)."
echo "  Ctrl+C aborts cleanly; the partial log stays valid."
echo "============================================="
echo ""

BEFORE=$(ls -1 "$LOG_DIR"/steering_sysid_*.csv 2>/dev/null | wc -l)

bash -c '
    source /opt/ros/jazzy/setup.bash && \
    cd "$1" && \
    source install/local_setup.bash && \
    shift && \
    ros2 run fsae_control steering_sysid --ros-args "$@"
' _ "$HOST_ROS2_DIR" "${EXTRA_ARGS[@]}"

# --- 5. Analyse -------------------------------------------------------------
NEWEST=$(ls -t "$LOG_DIR"/steering_sysid_*.csv 2>/dev/null | head -1)
AFTER=$(ls -1 "$LOG_DIR"/steering_sysid_*.csv 2>/dev/null | wc -l)
echo ""
echo "============================================="
if [ -n "$NEWEST" ] && [ "$AFTER" -gt "$BEFORE" ]; then
    echo "📊 Analysing $NEWEST"
    echo "============================================="
    ( cd "$HOST_REPO_ROOT/fsae_MPCTest" && \
      python3 -m tuner.steering_sysid_analysis "$NEWEST" )
    echo ""
    echo "Log kept at: $NEWEST"
else
    echo "⚠️  No new log was produced."
fi

cleanup
