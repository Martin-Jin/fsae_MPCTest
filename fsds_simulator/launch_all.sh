# Language: bash
# Title: Rock-Solid Auto-Launch and Cleanup Orchestrator for launch_all.sh
#!/bin/bash

# --- CONFIGURATION ---
CONTAINER_NAME="fsds_ros2_bridge"
WINDOWS_SIM_PATH="/mnt/c/Users/Martin/Downloads/fsds-v2.2.0-windows/FSDS.exe"
CONTAINER_ROS2_DIR="/root/Formula-Student-Driverless-Simulator/ros2"

cleanup() {
    echo ""
    echo "============================================="
    echo "🛑 Caught termination signal! Cleaning up..."
    echo "============================================="
    
    # 1. Terminate the background ROS 2 bridge process running in the host shell background
    if [ ! -z "$BRIDGE_PID" ]; then
        echo "Stopping background ROS 2 Bridge (PID: $BRIDGE_PID)..."
        kill "$BRIDGE_PID" 2>/dev/null
    fi

    # 2. Forcefully terminate the Windows visual simulator trees via taskkill
    echo "Forcefully terminating Windows FSDS window instances..."
    taskkill.exe /F /T /IM "FSDS.exe" 2>/dev/null
    taskkill.exe /F /T /IM "FSOnline.exe" 2>/dev/null
    taskkill.exe /F /T /IM "Blocks.exe" 2>/dev/null
    
    # 3. Clean up core dump files generated inside the Docker container's ROS 2 directory
    # Checks if the container is still running before attempting to clear files
    if [ "$(docker inspect -f '{{.State.Running}}' "$CONTAINER_NAME" 2>/dev/null)" = "true" ]; then
        echo "🧹 Sweeping up any generated core dump files inside the container..."
        docker exec "$CONTAINER_NAME" bash -c "find $CONTAINER_ROS2_DIR -maxdepth 1 -type f -name 'core.[0-9]*' -delete" 2>/dev/null
        echo "✅ Core dumps cleared."
    else
        echo "⚠️ Container wasn't running; skipped core dump purge."
    fi

    exit 0
}

# Catch Ctrl+C (SIGINT) and termination signals explicitly
trap cleanup SIGINT SIGTERM

echo "============================================="
echo "🏎️  Launching Formula Student Driverless Stack"
echo "============================================="

# 1. Manage Docker lifecycle
CONTAINER_STATUS=$(docker inspect -f '{{.State.Running}}' "$CONTAINER_NAME" 2>/dev/null)
if [ "$CONTAINER_STATUS" != "true" ]; then
    echo "🐳 Docker container is not running. Starting $CONTAINER_NAME..."
    docker start "$CONTAINER_NAME"
    sleep 2
else
    echo "🐳 Docker container is already running."
fi

# 2. Launch Simulator in background
if [ -d "/mnt/c/Users/Martin/Downloads/fsds-v2.2.0-windows" ]; then
    echo "[1/3] Spinning up Windows Simulator within its home directory..."
    cmd.exe /c "cd /d C:\Users\Martin\Downloads\fsds-v2.2.0-windows && FSDS.exe -windowed -ResX=1280 -ResY=720" &
    sleep 2
else
    echo "⚠️ Warning: Windows Simulator folder path not found!"
fi

# 3. Launch ROS 2 Bridge in background
echo "[2/3] Initializing fsds_ros2_bridge inside container..."
docker exec "$CONTAINER_NAME" bash -c "
    source /opt/ros/jazzy/setup.bash && \
    cd $CONTAINER_ROS2_DIR && \
    source install/local_setup.bash && \
    ros2 launch fsds_ros2_bridge fsds_ros2_bridge.launch.py
" &
BRIDGE_PID=$!
sleep 2

# 4. Launch Planning Stack in the foreground
echo "[3/3] Launching Autonomous Stack (Perception, Planner, Control)..."
docker exec -it "$CONTAINER_NAME" bash -c "
    source /opt/ros/jazzy/setup.bash && \
    cd $CONTAINER_ROS2_DIR && \
    source install/local_setup.bash && \
    ros2 launch fsae_planning launch_planning.py
"

# Handle manual exit or fallback execution when foreground process drops out cleanly
cleanup