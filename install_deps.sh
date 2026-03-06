#!/bin/bash
set -e

echo "=========================================="
echo "   FINAL STEP: INSTALLING DEPENDENCIES    "
echo "=========================================="

# Detect Distro
if [ -f /opt/ros/jazzy/setup.bash ]; then
    ROS_DISTRO="jazzy"
elif [ -f /opt/ros/humble/setup.bash ]; then
    ROS_DISTRO="humble"
else
    echo "ROS2 not found!"
    exit 1
fi

echo "Detected ROS2 Distro: $ROS_DISTRO"
echo "Installing rosbridge-server..."

sudo apt update
sudo apt install "ros-$ROS_DISTRO-rosbridge-server" -y

echo "=========================================="
echo "   SYSTEM READY FOR LAUNCH                "
echo "=========================================="

source ~/ika_ws/install/setup.bash
ros2 launch ika_bringup system.launch.py
