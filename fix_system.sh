#!/bin/bash
set -e

echo "=========================================="
echo "   EMERGENCY REPAIR SYSTEM                "
echo "=========================================="

WS_DIR=~/ika_ws

# 1. Force Create Directory Structure
echo "[1/3] Repairing Source Code Structure..."
mkdir -p "$WS_DIR/src/ika_robot/ika_bridge/ika_bridge"
mkdir -p "$WS_DIR/src/ika_robot/ika_control/ika_control"
mkdir -p "$WS_DIR/src/ika_robot/ika_vision/ika_vision"
mkdir -p "$WS_DIR/src/ika_robot/ika_decision/ika_decision"
mkdir -p "$WS_DIR/src/ika_robot/ika_web_bridge/ika_web_bridge"

# 2. Inject Code Directly (Bypassing Windows Copy)

# --- IKA BRIDGE ---
cat > "$WS_DIR/src/ika_robot/ika_bridge/ika_bridge/__init__.py" << EOF
EOF

cat > "$WS_DIR/src/ika_robot/ika_bridge/ika_bridge/serial_bridge_node.py" << EOF
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from geometry_msgs.msg import Twist

class SerialBridgeNode(Node):
    def __init__(self):
        super().__init__('serial_bridge_node')
        self.declare_parameter('mock_mode', True)
        self.mock_mode = self.get_parameter('mock_mode').value
        self.get_logger().info('Serial Bridge Started.')
        
        self.create_subscription(Twist, '/cmd_vel', self.cmd_vel_callback, 10)
        self.pub_state = self.create_publisher(String, '/robot_state', 10)
        
        if self.mock_mode:
            self.create_timer(1.0, self.mock_loop)

    def cmd_vel_callback(self, msg):
        pass

    def mock_loop(self):
        pass

def main(args=None):
    rclpy.init(args=args)
    node = SerialBridgeNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
EOF

# --- IKA CONTROL ---
cat > "$WS_DIR/src/ika_robot/ika_control/ika_control/__init__.py" << EOF
EOF

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

# --- IKA VISION ---
cat > "$WS_DIR/src/ika_robot/ika_vision/ika_vision/__init__.py" << EOF
EOF

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
        self.create_timer(1.0, self.mock_loop)

    def mock_loop(self):
        msg = Float32()
        msg.data = 0.0
        self.pub_lane_error.publish(msg)

def main(args=None):
    rclpy.init(args=args)
    node = VisionNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
EOF

# --- IKA DECISION ---
cat > "$WS_DIR/src/ika_robot/ika_decision/ika_decision/__init__.py" << EOF
EOF

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

# --- IKA WEB BRIDGE ---
cat > "$WS_DIR/src/ika_robot/ika_web_bridge/ika_web_bridge/__init__.py" << EOF
EOF

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

# 3. Clean and Build
echo "[2/3] Cleaning and Rebuilding..."
cd "$WS_DIR"
rm -rf build install
# Determine Distro
if [ -f /opt/ros/jazzy/setup.bash ]; then
    source /opt/ros/jazzy/setup.bash
else
    source /opt/ros/humble/setup.bash
fi

colcon build

echo "[3/3] DONE. Launching..."
source install/setup.bash
ros2 launch ika_bringup system.launch.py
