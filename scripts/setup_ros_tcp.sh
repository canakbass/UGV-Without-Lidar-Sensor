#!/bin/bash
set -e
WS_DIR=~/ika_ws

echo "Installing ROS-TCP-Endpoint in WSL..."
cd "$WS_DIR/src"
if [ ! -d "ROS-TCP-Endpoint" ]; then
    git clone -b ros2 https://github.com/Unity-Technologies/ROS-TCP-Endpoint.git
fi
cd "$WS_DIR"

if [ -f /opt/ros/jazzy/setup.bash ]; then
    source /opt/ros/jazzy/setup.bash
else
    source /opt/ros/humble/setup.bash
fi

echo "Building ROS-TCP-Endpoint..."
colcon build --packages-select ros_tcp_endpoint

echo "Successfully built ROS-TCP-Endpoint."
