#!/bin/bash
# Language: bash
# Title: Rock-Solid Auto-Launch and Cleanup Orchestrator for launch_all.sh

# --- CONFIGURATION ---
CONTAINER_NAME="fsds_ros2_bridge"
WINDOWS_SIM_PATH="/mnt/c/Users/Martin/Downloads/fsds-v2.2.0-windows/FSDS.exe"
HOST_ROS2_DIR="/home/Formula-Student-Driverless-Simulator/ros2"
CONTAINER_ROS2_DIR="/root/Formula-Student-Driverless-Simulator/ros2"
HOST_REPO_ROOT="$(dirname "$HOST_ROS2_DIR")"
CONTAINER_REPO_ROOT="$(dirname "$CONTAINER_ROS2_DIR")"
# This script itself lives in <mpcTest repo root>/fsds_simulator/, so its own
# directory is where gui/simulation.py's RECORDED_TRACK_DIR looks for cone maps.
MPCTEST_CONE_MAPS_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)/cone_maps"

# Precomputed-map toggles for the mpc/mpc_standalone controller (see
# fsae_planning's README.md "Precomputed-map launch args" and sim.launch.py's
# own DeclareLaunchArgument defaults for the full explanation). Set here so
# they don't need to be typed on every launch; both default to matching
# sim.launch.py's own defaults (true).
USE_PRECOMPUTED_SPEED=true
USE_PRECOMPUTED_PATH=true

# Use the host's native ROS 2 install when available; otherwise fall back to Docker.
if command -v ros2 >/dev/null 2>&1 && [ -f "$HOST_ROS2_DIR/install/local_setup.bash" ]; then
    USE_DOCKER=false
else
    USE_DOCKER=true
fi

# Under WSL2 (NAT networking), 127.0.0.1/localhost inside WSL does NOT reach
# the Windows host running FSDS — WSL has its own network namespace. The
# Windows host is reachable via WSL's default gateway instead. This affects
# both this script's own RPC-readiness check below AND fsds_ros2_bridge's
# connection to AirSim (fsds_ros2_bridge.launch.py already supports this via
# the FSDS_HOST_IP env var, but expects it to be set externally — it defaults
# to 'localhost' otherwise, which fails the same way). Computed once here and
# exported so both consumers agree, without requiring a manual `export` step.
# Skipped for the Docker path, where the container's own networking applies.
if [ "$USE_DOCKER" != true ] && [ -z "$FSDS_HOST_IP" ]; then
    export FSDS_HOST_IP="$(ip route show default 2>/dev/null | awk '{print $3; exit}')"
fi

cleanup() {
    echo ""
    echo "============================================="
    echo "🛑 Caught termination signal! Cleaning up..."
    echo "============================================="

    # 1. Terminate the background ROS 2 bridge process
    if [ ! -z "$BRIDGE_PID" ]; then
        echo "Stopping background ROS 2 Bridge (PID: $BRIDGE_PID)..."
        kill "$BRIDGE_PID" 2>/dev/null
    fi

    # 2. Forcefully terminate the Windows visual simulator trees via taskkill
    echo "Forcefully terminating Windows FSDS window instances..."
    taskkill.exe /F /T /IM "FSDS.exe" 2>/dev/null
    taskkill.exe /F /T /IM "FSOnline.exe" 2>/dev/null
    taskkill.exe /F /T /IM "Blocks.exe" 2>/dev/null

    # 3. Clean up core dump files generated in the ROS 2 directory
    if [ "$USE_DOCKER" = true ]; then
        if [ "$(docker inspect -f '{{.State.Running}}' "$CONTAINER_NAME" 2>/dev/null)" = "true" ]; then
            echo "🧹 Sweeping up any generated core dump files inside the container..."
            docker exec "$CONTAINER_NAME" bash -c "find $CONTAINER_ROS2_DIR -maxdepth 1 -type f -name 'core.[0-9]*' -delete" 2>/dev/null
            echo "✅ Core dumps cleared."
        else
            echo "⚠️ Container wasn't running; skipped core dump purge."
        fi
    else
        echo "🧹 Sweeping up any generated core dump files..."
        find "$HOST_ROS2_DIR" -maxdepth 1 -type f -name 'core.[0-9]*' -delete 2>/dev/null
        echo "✅ Core dumps cleared."
    fi

    exit 0
}

# Catch Ctrl+C (SIGINT) and termination signals explicitly
trap cleanup SIGINT SIGTERM

echo "============================================="
echo "🏎️  Launching Formula Student Driverless Stack"
echo "============================================="

if [ "$USE_DOCKER" = true ]; then
    echo "🐳 Native ROS 2 install not found on host; using Docker container $CONTAINER_NAME."

    CONTAINER_STATUS=$(docker inspect -f '{{.State.Running}}' "$CONTAINER_NAME" 2>/dev/null)
    if [ "$CONTAINER_STATUS" != "true" ]; then
        echo "🐳 Docker container is not running. Starting $CONTAINER_NAME..."
        docker start "$CONTAINER_NAME"
        sleep 2
    else
        echo "🐳 Docker container is already running."
    fi
else
    echo "🖥️  Native ROS 2 install found on host; running without Docker."
fi

# 1. Launch Simulator in background
AIRSIM_RPC_PORT=41451
# FSDS_HOST_IP is 'localhost' when running natively on the same machine as
# ROS 2, or the WSL default-gateway IP under WSL2 (see above) — either way
# it's the address FSDS's RPC server is actually reachable at, so the
# readiness check below must probe the same address the bridge will use.
AIRSIM_RPC_HOST="${FSDS_HOST_IP:-localhost}"
AIRSIM_READY_TIMEOUT=120   # seconds — a full competition map's Vulkan/shader/
                           # level-streaming boot can take well over a short
                           # fixed sleep, which would race the bridge against
                           # a simulator that hadn't opened its RPC port yet
                           # ("Failed connecting to RPC server (airsim)").

wait_for_airsim_rpc() {
    echo "⏳ Waiting for FSDS AirSim RPC server on $AIRSIM_RPC_HOST:$AIRSIM_RPC_PORT..."
    local waited=0
    while ! (exec 3<>"/dev/tcp/$AIRSIM_RPC_HOST/$AIRSIM_RPC_PORT") 2>/dev/null; do
        exec 3>&- 2>/dev/null
        sleep 1
        waited=$((waited + 1))
        if [ "$waited" -ge "$AIRSIM_READY_TIMEOUT" ]; then
            echo "⚠️ Timed out after ${AIRSIM_READY_TIMEOUT}s waiting for AirSim RPC — proceeding anyway."
            return 1
        fi
    done
    exec 3>&- 2>/dev/null
    echo "✅ AirSim RPC is up after ${waited}s."
    return 0
}

if [ -d "/mnt/c/Users/Martin/Downloads/fsds-v2.2.0-windows" ]; then
    echo "[1/3] Spinning up Windows Simulator within its home directory..."
    cmd.exe /c "cd /d C:\Users\Martin\Downloads\fsds-v2.2.0-windows && FSDS.exe -windowed -ResX=1280 -ResY=720" &
    wait_for_airsim_rpc
else
    echo "⚠️ Warning: Windows Simulator folder path not found!"
fi

# 2. Rebuild with --symlink-install so edits to src/ take effect immediately,
# without a separate `colcon build` step. Plain `colcon build` COPIES Python
# files into install/ at build time, so an edit to src/ after the last build
# is silently invisible to `ros2 launch` until rebuilt — this bit twice in
# one session (S49: a stale v_max clip; a stale Q_diag[4] weight straight
# after). --symlink-install replaces the copy with a symlink for supported
# files (this workspace's packages are all pure Python + ament_index
# resources, so every affected file qualifies), so src/ IS the running code.
# Safe to run every launch: colcon no-ops packages that are already built
# and up to date.
echo "[1.5/3] Building workspace (--symlink-install)..."
if [ "$USE_DOCKER" = true ]; then
    docker exec "$CONTAINER_NAME" bash -c "
        source /opt/ros/jazzy/setup.bash && \
        cd $CONTAINER_ROS2_DIR && \
        colcon build --symlink-install
    "
else
    bash -c "
        source /opt/ros/jazzy/setup.bash && \
        cd '$HOST_ROS2_DIR' && \
        colcon build --symlink-install
    "
fi

# 2. Launch ROS 2 Bridge in background
echo "[2/3] Initializing fsds_ros2_bridge..."
if [ "$USE_DOCKER" = true ]; then
    docker exec "$CONTAINER_NAME" bash -c "
        source /opt/ros/jazzy/setup.bash && \
        cd $CONTAINER_ROS2_DIR && \
        source install/local_setup.bash && \
        ros2 launch fsds_ros2_bridge fsds_ros2_bridge.launch.py
    " &
else
    bash -c "
        source /opt/ros/jazzy/setup.bash && \
        cd '$HOST_ROS2_DIR' && \
        source install/local_setup.bash && \
        ros2 launch fsds_ros2_bridge fsds_ros2_bridge.launch.py
    " &
fi
BRIDGE_PID=$!
sleep 2

# 3. Launch Planning Stack in the foreground
# sim.launch.py defaults to controller:=mpc_standalone and record_cones:=true,
# so cone recording starts automatically alongside the stack (no separate
# terminal needed).
#
# The precomputed-speed/path toggles (see fsae_planning's README.md
# "Precomputed-map launch args") are set via USE_PRECOMPUTED_SPEED /
# USE_PRECOMPUTED_PATH above, not here.
echo "[3/3] Launching Autonomous Stack (Perception, Planner, Control, Cone Recorder)..."
if [ "$USE_DOCKER" = true ]; then
    # No volume mount ties the container to this repo, so we can't write
    # straight into $MPCTEST_CONE_MAPS_DIR here — falls back to the FSDS
    # repo root; copy the file into fsds_simulator/cone_maps manually to pick
    # it up from Load Recorded Track.
    docker exec -it "$CONTAINER_NAME" bash -c "
        source /opt/ros/jazzy/setup.bash && \
        cd $CONTAINER_ROS2_DIR && \
        source install/local_setup.bash && \
        ros2 launch fsae_bringup sim.launch.py cone_out_path:=$CONTAINER_REPO_ROOT/cone_map.json log_dir:=$CONTAINER_REPO_ROOT/fsae_logs use_precomputed_speed:=$USE_PRECOMPUTED_SPEED use_precomputed_path:=$USE_PRECOMPUTED_PATH
    "
else
    bash -c "
        source /opt/ros/jazzy/setup.bash && \
        cd '$HOST_ROS2_DIR' && \
        source install/local_setup.bash && \
        ros2 launch fsae_bringup sim.launch.py cone_out_path:='$MPCTEST_CONE_MAPS_DIR/cone_map_$(date +%s).json' log_dir:='$HOST_REPO_ROOT/fsae_logs' use_precomputed_speed:=$USE_PRECOMPUTED_SPEED use_precomputed_path:=$USE_PRECOMPUTED_PATH
    "
fi

# Handle manual exit or fallback execution when foreground process drops out cleanly
cleanup
