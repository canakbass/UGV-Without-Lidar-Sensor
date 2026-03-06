#!/bin/bash
set -e
WS_DIR=~/ika_ws

echo "=========================================="
echo "   FINAL V1 UPDATE (SMART LOGIC)          "
echo "=========================================="
echo "Applying Changes:"
echo "1. Navigation: Ultrasonic PID (Dynamic Offset)"
echo "2. Vision: Stage-Specific Detection (Save CPU)"
echo "3. State Machine: Stage Transitions (Mock)"

# 1. SERIAL BRIDGE (PROTOCOL + PHYSICS)
echo "  > Updating serial_bridge_node.py..."
cat > "$WS_DIR/src/ika_robot/ika_bridge/ika_bridge/serial_bridge_node.py" << EOF
import rclpy
from rclpy.node import Node
from std_msgs.msg import String, Float32MultiArray
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
import math
import struct
import time

class SerialBridgeNode(Node):
    def __init__(self):
        super().__init__('serial_bridge_node')
        self.declare_parameter('mock_mode', True)
        self.mock_mode = self.get_parameter('mock_mode').value
        
        # Rover Params
        self.wheel_base = 0.4 
        
        # Physics State
        self.x = 0.0
        self.y = 0.0 # Lateral position (0 = Center)
        self.theta = 0.0
        self.v = 0.0
        self.w = 0.0
        self.last_time = time.time()
        
        self.left_us = 1.0 
        self.right_us = 1.0 
        
        self.get_logger().info(f'Serial Bridge Started. Mock Physics: {self.mock_mode}')
        self.create_subscription(Twist, '/cmd_vel', self.cmd_vel_callback, 10)
        self.pub_odom = self.create_publisher(Odometry, '/odometry/raw', 10)
        self.pub_sensors = self.create_publisher(Float32MultiArray, '/sensors/ultrasonic', 10)
        
        if self.mock_mode:
            self.create_timer(0.05, self.physics_loop) 

    def cmd_vel_callback(self, msg):
        self.v = msg.linear.x
        self.w = msg.angular.z

    def physics_loop(self):
        now = time.time()
        dt = now - self.last_time
        self.last_time = now
        
        # Simple Motion Model
        self.x += self.v * math.cos(self.theta) * dt
        self.y += self.v * math.sin(self.theta) * dt
        self.theta += self.w * dt
        
        # Simulate Ultrasonic Sensors (Lane Width = 3.0m)
        lane_width = 3.0
        # If robot is perfectly centered (y=0), left=1.5, right=1.5
        # If robot is right (y=-0.5), left=2.0, right=1.0
        self.left_us = (lane_width / 2.0) + self.y 
        self.right_us = (lane_width / 2.0) - self.y
        
        # Publish Odom
        odom = Odometry()
        odom.header.stamp = self.get_clock().now().to_msg()
        odom.header.frame_id = "odom"
        odom.twist.twist.linear.x = self.v
        odom.twist.twist.angular.z = self.w
        odom.pose.pose.position.x = self.x
        odom.pose.pose.position.y = self.y
        self.pub_odom.publish(odom)
        
        # Publish Sensors [Front, Left, Right]
        sens_msg = Float32MultiArray()
        sens_msg.data = [2.0, self.left_us, self.right_us] 
        self.pub_sensors.publish(sens_msg)

def main(args=None):
    rclpy.init(args=args)
    node = SerialBridgeNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
EOF

# 2. NAVIGATION (PID CENTERING WITH OFFSET)
echo "  > Updating navigation_node.py..."
cat > "$WS_DIR/src/ika_robot/ika_control/ika_control/navigation_node.py" << EOF
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from std_msgs.msg import Float32MultiArray, String

class NavigationNode(Node):
    def __init__(self):
        super().__init__('navigation_node')
        
        self.active = False
        self.kp = 0.8 # Proportional Gain
        self.target_offset = 0.0 # 0.0 = Center, -0.3 = Right Hug, +0.3 = Left Hug
        
        self.create_subscription(Float32MultiArray, '/sensors/ultrasonic', self.us_callback, 10)
        self.create_subscription(String, '/robot_state', self.state_callback, 10)
        self.pub_cmd = self.create_publisher(Twist, '/cmd_vel', 10)
        
        self.get_logger().info('Navigation Node (Smart PID) Started.')

    def state_callback(self, msg):
        state = msg.data
        self.active = (state != "IDLE" and state != "MANUAL")
        
        # DYNAMIC OFFSET LOGIC
        if state == "YAN_EGIM" or state == "KAYAR_ENGEL":
            self.target_offset = -0.5 # Hug Right Wall (50cm bias)
            self.get_logger().info(f"State: {state} -> Hugging RIGHT Wall")
        else:
            self.target_offset = 0.0 # Center
            
    def us_callback(self, msg):
        if not self.active: return
        
        # msg.data = [Front, Left, Right]
        left_dist = msg.data[1]
        right_dist = msg.data[2]
        
        # Error = (Left - Right) - Target_Offset
        # If Target_Offset is -0.5 (Right Hug), we want Left to be bigger than Right by 0.5
        current_diff = left_dist - right_dist
        error = current_diff - self.target_offset
        
        # PID Control
        # If error > 0 (Too Left), Turn Right (-)
        # If error < 0 (Too Right), Turn Left (+)
        angular_z = -self.kp * error 
        
        # Clamp
        angular_z = max(-1.0, min(1.0, angular_z))
        
        cmd = Twist()
        cmd.linear.x = 0.5 # Constant forward speed
        cmd.angular.z = angular_z
        
        self.pub_cmd.publish(cmd)

def main(args=None):
    rclpy.init(args=args)
    node = NavigationNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
EOF


# 3. VISION NODE (STAGE AWARE)
echo "  > Updating detector_node.py..."
cat > "$WS_DIR/src/ika_robot/ika_vision/ika_vision/detector_node.py" << EOF
import rclpy
from rclpy.node import Node
from ika_interfaces.msg import SignDetection
from std_msgs.msg import String
import random

class VisionNode(Node):
    def __init__(self):
        super().__init__('vision_node')
        self.current_state = "IDLE"
        
        self.create_subscription(String, '/robot_state', self.state_callback, 10)
        self.pub_sign = self.create_publisher(SignDetection, '/vision/sign', 10)
        self.create_timer(1.0, self.mock_loop)
        
        self.get_logger().info('Vision Node (Smart Detection) Started.')

    def state_callback(self, msg):
        self.current_state = msg.data

    def mock_loop(self):
        # Only detect cones in Stage 5
        if self.current_state == "TRAFIK_KONILERI":
             if random.random() < 0.2:
                 self.get_logger().info("DETECTED: CONE (Processing...)")
                 # Logic for cone avoidance would trigger here
        
        # Always look for signs (Transition Triggers)
        if random.random() < 0.05:
            msg = SignDetection()
            msg.type = "NEXT_STAGE"
            msg.distance = 5.0
            msg.confidence = 0.98
            self.pub_sign.publish(msg)
            self.get_logger().info("DETECTED: SIGN (Next Stage)")

def main(args=None):
    rclpy.init(args=args)
    node = VisionNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
EOF

# 4. STATE MACHINE (MOCK TRANSITIONS)
echo "  > Updating state_machine_node.py..."
cat > "$WS_DIR/src/ika_robot/ika_decision/ika_decision/state_machine_node.py" << EOF
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from ika_interfaces.msg import SignDetection

class StateMachineNode(Node):
    def __init__(self):
        super().__init__('state_machine_node')
        self.state = "IDLE"
        self.stages = ["NORMAL", "TASLI_YOL", "YAN_EGIM", "DIK_ENGEL", "TRAFIK_KONILERI", "KAYAR_ENGEL"]
        self.stage_idx = 0
        
        self.pub_state = self.create_publisher(String, '/robot_state', 10)
        self.create_subscription(SignDetection, '/vision/sign', self.sign_callback, 10)
        self.create_subscription(String, '/user_command', self.cmd_callback, 10)
        
        self.create_timer(0.5, self.loop)
        self.get_logger().info('State Machine Ready.')

    def cmd_callback(self, msg):
        if msg.data == "START_AUTO_NORMAL":
            self.state = "NORMAL"
            self.stage_idx = 0
            self.get_logger().info("Starting Normal Parkour")
        elif msg.data == "STOP":
            self.state = "IDLE"

    def sign_callback(self, msg):
        # If valid sign, move to next stage
        if msg.type == "NEXT_STAGE" and self.state != "IDLE":
            self.stage_idx = (self.stage_idx + 1) % len(self.stages)
            self.state = self.stages[self.stage_idx]
            self.get_logger().info(f"TRANSITION -> {self.state}")

    def loop(self):
        msg = String()
        msg.data = self.state
        self.pub_state.publish(msg)

def main(args=None):
    rclpy.init(args=args)
    node = StateMachineNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
EOF

# 5. REBUILD
echo "  > Rebuilding..."
cd "$WS_DIR"
# Clean specific packages to ensure update
rm -rf build/ika_control install/ika_control
rm -rf build/ika_vision install/ika_vision
rm -rf build/ika_decision install/ika_decision

if [ -f /opt/ros/jazzy/setup.bash ]; then source /opt/ros/jazzy/setup.bash; else source /opt/ros/humble/setup.bash; fi
colcon build

echo "V1 FINAL UPDATE COMPLETE."
echo "Restart the simulation with:"
echo "source install/setup.bash && ros2 launch ika_bringup system.launch.py"
EOF
