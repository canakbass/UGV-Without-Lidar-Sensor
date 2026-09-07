#!/bin/bash
set -e # Exit on error

echo "=========================================="
echo "   TEKNOFEST 2026 UGV - WSL SETUP SCRIPT  "
echo "   (UNIVERSAL VERSION: 22.04 / 24.04)     "
echo "   (FIX V4: FORCE CLEAN BUILD)            "
echo "=========================================="

# 0. Detect OS & Select ROS Distro
ROS_DISTRO="humble" # Default
if [ -f /etc/os-release ]; then
    . /etc/os-release
    echo "[DEBUG] Detected OS: $NAME $VERSION_ID"
    if [[ "$VERSION_ID" == "24.04" ]]; then
        ROS_DISTRO="jazzy"
        echo "[INFO] Ubuntu 24.04 detected. Switching to ROS2 JAZZY."
    elif [[ "$VERSION_ID" == "22.04" ]]; then
        ROS_DISTRO="humble"
        echo "[INFO] Ubuntu 22.04 detected. Using ROS2 HUMBLE."
    else
        echo "[WARN] Unsupported Ubuntu version: $VERSION_ID. Defaulting to Humble (might fail)."
    fi
fi

# 1. Install ROS2 (Skipped for speed if already there)
setup_file="/opt/ros/$ROS_DISTRO/setup.bash"
if [ ! -f "$setup_file" ]; then
    echo "[1/4] Installing ROS2..."
    sudo apt update && sudo apt install "ros-$ROS_DISTRO-desktop" python3-colcon-common-extensions "ros-$ROS_DISTRO-rosbridge-server" -y
    source "$setup_file"
else
    echo "[1/4] ROS2 found. Skipping install."
fi

# 2. Setup Workspace
WS_DIR=~/ika_ws
echo "[2/4] Setting up Workspace at $WS_DIR..."

if [ ! -d "$WS_DIR" ]; then
    mkdir -p "$WS_DIR/src"
fi

# 3. Copy Source Code
WINDOWS_SRC="/mnt/c/Users/1hayr/OneDrive/Desktop/ika yeni/ika_ws/src"

echo "[3/4] Copying project files from Windows..."
cp -r "$WINDOWS_SRC/." "$WS_DIR/src/"

# FIX: Force Cleanup of Build Artifacts
echo "      [CRITICAL] Cleaning old build artifacts..."
rm -rf "$WS_DIR/build" "$WS_DIR/install" "$WS_DIR/log"

# FIX: Ensure resources exist
mkdir -p "$WS_DIR/src/ika_robot/ika_control/resource" && touch "$WS_DIR/src/ika_robot/ika_control/resource/ika_control"
mkdir -p "$WS_DIR/src/ika_robot/ika_vision/resource" && touch "$WS_DIR/src/ika_robot/ika_vision/resource/ika_vision"
mkdir -p "$WS_DIR/src/ika_robot/ika_decision/resource" && touch "$WS_DIR/src/ika_robot/ika_decision/resource/ika_decision"
mkdir -p "$WS_DIR/src/ika_robot/ika_sensors/resource" && touch "$WS_DIR/src/ika_robot/ika_sensors/resource/ika_sensors"
mkdir -p "$WS_DIR/src/ika_robot/ika_bridge/resource" && touch "$WS_DIR/src/ika_robot/ika_bridge/resource/ika_bridge"
mkdir -p "$WS_DIR/src/ika_robot/ika_web_bridge/resource" && touch "$WS_DIR/src/ika_robot/ika_web_bridge/resource/ika_web_bridge"

# 4. Build
echo "[4/4] Building ROS2 Packages using $ROS_DISTRO..."
cd "$WS_DIR"
source "/opt/ros/$ROS_DISTRO/setup.bash"

# Normal build without symlink first to ensure executables are generated correctly
colcon build

# 5. Final Instructions
echo ""
echo "=========================================="
echo "   SETUP COMPLETE! SYSTEM READY.          "
echo "=========================================="
echo ""
echo "To start the simulation, RUN THIS COMMAND:"
echo "------------------------------------------"
echo "source ~/ika_ws/install/setup.bash && ros2 launch ika_bringup system.launch.py"
echo "------------------------------------------"
echo ""
