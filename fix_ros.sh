#!/bin/bash
set -e
WS_DIR=~/ika_ws

echo "Fixing Launch file to accept mock_mode parameter and fixing ika_vision..."
cat > "$WS_DIR/src/ika_robot/ika_bringup/launch/system.launch.py" << 'INNER_EOF'
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration

def generate_launch_description():
    mock_mode = LaunchConfiguration('mock_mode')
    
    return LaunchDescription([
        DeclareLaunchArgument(
            'mock_mode',
            default_value='true',
            description='Enable mock physics and simulated sensors'
        ),
        
        Node(
            package='ika_bridge',
            executable='serial_bridge',
            name='serial_bridge',
            parameters=[{'mock_mode': mock_mode}]
        ),
        
        Node(
            package='ika_vision',
            executable='detector',
            name='vision_node',
            parameters=[{'mock_mode': mock_mode}]
        ),
        
        Node(
            package='ika_decision',
            executable='state_machine',
            name='state_machine'
        ),
        
        Node(
            package='ika_control',
            executable='acceleration',
            name='acceleration_logic'
        ),
        
        Node(
            package='ika_control',
            executable='navigation',
            name='navigation_logic'
        ),
        
        Node(
            package='ika_web_bridge',
            executable='web_bridge',
            name='web_bridge'
        ),
        
        Node(
            package='rosbridge_server',
            executable='rosbridge_websocket',
            name='rosbridge_websocket'
        ),
    ])
INNER_EOF

echo "Rebuilding ika_vision and ika_bringup..."
cd "$WS_DIR"
# Force a clean rebuild for ika_vision to recreate the entry points and fix dependencies
rm -rf build/ika_vision install/ika_vision
rm -rf build/ika_bringup install/ika_bringup

if [ -f /opt/ros/jazzy/setup.bash ]; then source /opt/ros/jazzy/setup.bash; else source /opt/ros/humble/setup.bash; fi
colcon build --packages-select ika_vision ika_bringup

echo "DONE"
