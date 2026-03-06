#!/bin/bash
set -e

echo "=========================================="
echo "   ULTIMATE REPAIR SYSTEM (SCORCHED EARTH)"
echo "=========================================="
echo "This script deletes the corrupted 'src' and regenerates EVERYTHING from scratch."
echo "This fixes the 'libexec does not exist' error by guaranteeing valid setup.py files."

WS_DIR=~/ika_ws

# 1. CLEANUP (Delete everything to be sure)
echo "[1/4] Wiping old corrupted files..."
rm -rf "$WS_DIR/src/ika_robot"
rm -rf "$WS_DIR/build" "$WS_DIR/install" "$WS_DIR/log"

# 2. REGENERATE PACKAGES

# Function to create a standard python package
create_pkg() {
    local pkg_name=$1
    local script_name=$2
    
    echo "  > Generating $pkg_name..."
    
    local pkg_path="$WS_DIR/src/ika_robot/$pkg_name"
    mkdir -p "$pkg_path/$pkg_name"
    mkdir -p "$pkg_path/resource"
    
    # Resource Marker
    touch "$pkg_path/resource/$pkg_name"
    
    # package.xml
    cat > "$pkg_path/package.xml" << EOF
<?xml version="1.0"?>
<?xml-model href="http://download.ros.org/schema/package_format3.xsd" schematypens="http://www.w3.org/2001/XMLSchema"?>
<package format="3">
  <name>$pkg_name</name>
  <version>0.0.0</version>
  <description>Auto-generated $pkg_name</description>
  <maintainer email="user@todo.todo">user</maintainer>
  <license>TODO</license>
  <depend>rclpy</depend>
  <depend>std_msgs</depend>
  <depend>geometry_msgs</depend>
  <test_depend>ament_copyright</test_depend>
  <test_depend>ament_flake8</test_depend>
  <test_depend>ament_pep257</test_depend>
  <test_depend>python3-pytest</test_depend>
  <export>
    <build_type>ament_python</build_type>
  </export>
</package>
EOF

    # setup.py (CRITICAL: Using find_packages to be robust)
    cat > "$pkg_path/setup.py" << EOF
from setuptools import setup, find_packages
import os
from glob import glob

package_name = '$pkg_name'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='user',
    maintainer_email='user@todo.todo',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            '$script_name = $pkg_name.${script_name}_node:main',
        ],
    },
)
EOF

    # __init__.py
    touch "$pkg_path/$pkg_name/__init__.py"
}

# --- GENERATE ALL PACKAGES ---
create_pkg "ika_bridge" "serial_bridge"
create_pkg "ika_control" "acceleration"
create_pkg "ika_vision" "detector"
create_pkg "ika_decision" "state_machine"
create_pkg "ika_web_bridge" "web_bridge"
create_pkg "ika_sensors" "sensor_dummy" # Dummy for now to satisfy deps

# --- ADD NODE CONTENT ---
echo "  > Injecting Python Code..."

# Serial Bridge Node
cat > "$WS_DIR/src/ika_robot/ika_bridge/ika_bridge/serial_bridge_node.py" << EOF
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from geometry_msgs.msg import Twist

class SerialBridgeNode(Node):
    def __init__(self):
        super().__init__('serial_bridge_node')
        self.get_logger().info('Serial Bridge Started.')
        self.create_subscription(Twist, '/cmd_vel', self.cmd_vel_callback, 10)
        self.pub_state = self.create_publisher(String, '/robot_state', 10)
    def cmd_vel_callback(self, msg): pass

def main(args=None):
    rclpy.init(args=args)
    node = SerialBridgeNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
EOF

# Acceleration Node
cat > "$WS_DIR/src/ika_robot/ika_control/ika_control/acceleration_node.py" << EOF
import rclpy
from rclpy.node import Node
class AccelerationNode(Node):
    def __init__(self):
        super().__init__('acceleration_node')
        self.get_logger().info('Acceleration Node Started')
def main(args=None):
    rclpy.init(args=args)
    node = AccelerationNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
EOF

# Vision Node
cat > "$WS_DIR/src/ika_robot/ika_vision/ika_vision/detector_node.py" << EOF
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32
class VisionNode(Node):
    def __init__(self):
        super().__init__('vision_node')
        self.declare_parameter('mock_mode', True)
        self.get_logger().info('Vision Node Started')
        self.pub_lane_error = self.create_publisher(Float32, '/vision/lane_error', 10)
    def mock_loop(self): pass
def main(args=None):
    rclpy.init(args=args)
    node = VisionNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
EOF

# State Machine Node
cat > "$WS_DIR/src/ika_robot/ika_decision/ika_decision/state_machine_node.py" << EOF
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
class StateMachineNode(Node):
    def __init__(self):
        super().__init__('state_machine_node')
        self.get_logger().info('State Machine Node Started')
        self.pub_state = self.create_publisher(String, '/robot_state', 10)
        self.create_timer(1.0, self.loop)
    def loop(self):
        msg = String()
        msg.data = "IDLE"
        self.pub_state.publish(msg)
def main(args=None):
    rclpy.init(args=args)
    node = StateMachineNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
EOF

# Web Bridge Node
cat > "$WS_DIR/src/ika_robot/ika_web_bridge/ika_web_bridge/web_bridge_node.py" << EOF
import rclpy
from rclpy.node import Node
class WebBridgeNode(Node):
    def __init__(self):
        super().__init__('web_bridge_node')
        self.get_logger().info('Web Bridge Node Started')
def main(args=None):
    rclpy.init(args=args)
    node = WebBridgeNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
EOF

# Dummy Sensor Node
cat > "$WS_DIR/src/ika_robot/ika_sensors/ika_sensors/sensor_dummy_node.py" << EOF
import rclpy
from rclpy.node import Node
class SensorNode(Node):
    def __init__(self):
        super().__init__('sensor_dummy_node')
def main(args=None):
    rclpy.init(args=args)
    node = SensorNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
EOF

# --- ika_bringup (The Manager) ---
echo "  > Generating ika_bringup..."
BRINGUP_PATH="$WS_DIR/src/ika_robot/ika_bringup"
mkdir -p "$BRINGUP_PATH/launch"

# package.xml
cat > "$BRINGUP_PATH/package.xml" << EOF
<?xml version="1.0"?>
<?xml-model href="http://download.ros.org/schema/package_format3.xsd" schematypens="http://www.w3.org/2001/XMLSchema"?>
<package format="3">
  <name>ika_bringup</name>
  <version>0.0.0</version>
  <description>Bringup</description>
  <maintainer email="user@todo.todo">user</maintainer>
  <license>TODO</license>
  <buildtool_depend>ament_cmake</buildtool_depend>
  <exec_depend>ros2launch</exec_depend>
  <exec_depend>ika_control</exec_depend>
  <exec_depend>ika_vision</exec_depend>
  <export>
    <build_type>ament_cmake</build_type>
  </export>
</package>
EOF

# CMakeLists.txt
cat > "$BRINGUP_PATH/CMakeLists.txt" << EOF
cmake_minimum_required(VERSION 3.8)
project(ika_bringup)
find_package(ament_cmake REQUIRED)
install(DIRECTORY launch DESTINATION share/\${PROJECT_NAME})
ament_package()
EOF

# Launch File
cat > "$BRINGUP_PATH/launch/system.launch.py" << EOF
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(package='ika_bridge', executable='serial_bridge', name='serial_bridge'),
        Node(package='ika_vision', executable='detector', name='vision_node'),
        Node(package='ika_decision', executable='state_machine', name='state_machine'),
        Node(package='ika_control', executable='acceleration', name='acceleration_logic'),
        Node(package='ika_web_bridge', executable='web_bridge', name='web_bridge'),
        Node(package='rosbridge_server', executable='rosbridge_websocket', name='rosbridge_websocket'),
    ])
EOF


# ika_interfaces (Mock)
mkdir -p "$WS_DIR/src/ika_robot/ika_interfaces"
cat > "$WS_DIR/src/ika_robot/ika_interfaces/package.xml" << EOF
<?xml version="1.0"?>
<?xml-model href="http://download.ros.org/schema/package_format3.xsd" schematypens="http://www.w3.org/2001/XMLSchema"?>
<package format="3">
  <name>ika_interfaces</name>
  <version>0.0.0</version>
  <description>Interfaces</description>
  <maintainer email="user@todo.todo">user</maintainer>
  <license>TODO</license>
  <buildtool_depend>ament_cmake</buildtool_depend>
  <member_of_group>rosidl_interface_packages</member_of_group>
  <export><build_type>ament_cmake</build_type></export>
</package>
EOF
cat > "$WS_DIR/src/ika_robot/ika_interfaces/CMakeLists.txt" << EOF
cmake_minimum_required(VERSION 3.8)
project(ika_interfaces)
find_package(ament_cmake REQUIRED)
ament_package()
EOF


# 3. BUILD
echo "[3/4] Building (Verbose)..."
cd "$WS_DIR"
if [ -f /opt/ros/jazzy/setup.bash ]; then source /opt/ros/jazzy/setup.bash; else source /opt/ros/humble/setup.bash; fi

# Using --symlink-install is great for dev, but sometimes causes path issues in WSL1/WSL2 cross-fs
# We will TRY without symlink first to guarantee files are physically copied to install/
colcon build --merge-install

echo "[4/4] Verifying Installation..."
if [ -f "$WS_DIR/install/lib/ika_bridge/serial_bridge" ]; then
    echo "SUCCESS: ika_bridge executable FOUND!"
else
    echo "CRITICAL FAILURE: ika_bridge executable NOT FOUND in installed artifacts."
    echo "Debug Info:"
    find "$WS_DIR/install" -name "serial_bridge"
fi

echo ""
echo "Repair Complete. Try running:"
echo "source install/setup.bash && ros2 launch ika_bringup system.launch.py"
