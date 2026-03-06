#!/bin/bash
set -e

echo "=========================================="
echo "   REPAIR V2: ADDING MISSING SETUP.CFG    "
echo "=========================================="
echo "The missing piece found! We need setup.cfg to tell ROS where to put the executables."

WS_DIR=~/ika_ws

# Function to create setup.cfg
create_cfg() {
    local pkg_name=$1
    echo "  > Fixing $pkg_name..."
    local pkg_path="$WS_DIR/src/ika_robot/$pkg_name"
    
    mkdir -p "$pkg_path"
    
    cat > "$pkg_path/setup.cfg" << EOF
[develop]
script_dir=\$base/lib/$pkg_name
[install]
install_scripts=\$base/lib/$pkg_name
EOF
}

# Apply to all
create_cfg "ika_bridge"
create_cfg "ika_control"
create_cfg "ika_vision"
create_cfg "ika_decision"
create_cfg "ika_web_bridge"
# ika_interfaces and ika_bringup match CMAKE or don't need it, skipping.

# Clean Build
echo "[2/3] Cleaning and Rebuilding (Isolated)..."
cd "$WS_DIR"
rm -rf build install log

if [ -f /opt/ros/jazzy/setup.bash ]; then source /opt/ros/jazzy/setup.bash; else source /opt/ros/humble/setup.bash; fi

# Standard isolated build is safer for paths
colcon build

echo "[3/3] DONE. Launching..."
source install/setup.bash
ros2 launch ika_bringup system.launch.py
