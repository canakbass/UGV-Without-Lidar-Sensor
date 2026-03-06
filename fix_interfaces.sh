#!/bin/bash
set -e
WS_DIR=~/ika_ws

echo "Fixing ika_interfaces resolution for vision node..."
cd "$WS_DIR"

# 1. Add dependency to package.xml if it's missing
PKG_XML="$WS_DIR/src/ika_robot/ika_vision/package.xml"
if ! grep -q "ika_interfaces" "$PKG_XML"; then
    echo "Adding ika_interfaces dependency to vision package..."
    sed -i '/<export>/i \  <depend>ika_interfaces</depend>' "$PKG_XML"
fi

# 2. Rebuild the interfaces package FIRST
echo "Rebuilding Interfaces..."
rm -rf build/ika_interfaces install/ika_interfaces
if [ -f /opt/ros/jazzy/setup.bash ]; then 
    source /opt/ros/jazzy/setup.bash
else 
    source /opt/ros/humble/setup.bash
fi
colcon build --packages-select ika_interfaces

# 3. Source the updated environment
source install/setup.bash

# 4. Rebuild the vision package
echo "Rebuilding Vision Node..."
rm -rf build/ika_vision install/ika_vision
colcon build --packages-select ika_vision

echo "DONE"
