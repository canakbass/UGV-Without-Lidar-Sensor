#!/bin/bash
set -e
WS_DIR=~/ika_ws

echo "=========================================="
echo "   UPDATING STAGE 6 (SLIDING OBSTACLE)    "
echo "=========================================="
echo "Strategy: Wait for opening, then DASH!"

# 1. Update Navigation Node (Wait & Dash logic)
cat > "$WS_DIR/src/ika_robot/ika_control/ika_control/navigation_node.py" << 'EOF'
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from std_msgs.msg import Float32MultiArray, String

class NavigationNode(Node):
    def __init__(self):
        super().__init__('navigation_node')
        
        self.active = False
        self.kp = 0.8
        self.target_offset = 0.0
        self.current_state = "IDLE"
        
        self.create_subscription(Float32MultiArray, '/sensors/ultrasonic', self.us_callback, 10)
        self.create_subscription(String, '/robot_state', self.state_callback, 10)
        self.pub_cmd = self.create_publisher(Twist, '/cmd_vel', 10)
        
        self.get_logger().info('Navigation Node (Stage 6 Updated) Started.')

    def state_callback(self, msg):
        self.current_state = msg.data
        self.active = (self.current_state != "IDLE" and self.current_state != "MANUAL")
        
        if self.current_state == "YAN_EGIM":
            self.target_offset = -0.5 # Hug Right Wall
        else:
            self.target_offset = 0.0 # Center

    def us_callback(self, msg):
        if not self.active: return
        
        front_dist = msg.data[0]
        left_dist = msg.data[1]
        right_dist = msg.data[2]
        
        error = (left_dist - right_dist) - self.target_offset
        angular_z = -self.kp * error 
        angular_z = max(-1.0, min(1.0, angular_z))
        
        # Speed Logic
        v = 0.5 # Safe default
        
        if self.current_state == "KAYAR_ENGEL":
            if front_dist < 1.7: # Obstacle is blocking
                v = 0.0
                self.get_logger().warn(f"Sliding Obstacle Blocks Path ({front_dist:.2f}m). WAITING...")
            else: # Path clear
                v = 1.5 # Dash!
                self.get_logger().info(f"Path Clear ({front_dist:.2f}m)! DASHING!")
        
        cmd = Twist()
        cmd.linear.x = v
        cmd.angular.z = angular_z
        self.pub_cmd.publish(cmd)

def main(args=None):
    rclpy.init(args=args)
    node = NavigationNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
EOF

# 2. Update Serial Bridge Node (Mock sliding obstacle)
cat > "$WS_DIR/src/ika_robot/ika_bridge/ika_bridge/serial_bridge_node.py" << 'EOF'
import rclpy
from rclpy.node import Node
from std_msgs.msg import String, Float32MultiArray
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
import math
import time

class SerialBridgeNode(Node):
    def __init__(self):
        super().__init__('serial_bridge_node')
        self.declare_parameter('mock_mode', True)
        self.mock_mode = self.get_parameter('mock_mode').value
        
        self.wheel_base = 0.4 
        self.x = 0.0
        self.y = 0.0
        self.theta = 0.0
        self.v = 0.0
        self.w = 0.0
        self.last_time = time.time()
        
        self.left_us = 1.5 
        self.right_us = 1.5 
        self.current_state = "IDLE"
        
        self.get_logger().info(f'Serial Bridge Started. Mock Physics: {self.mock_mode}')
        self.create_subscription(Twist, '/cmd_vel', self.cmd_vel_callback, 10)
        self.create_subscription(String, '/robot_state', self.state_callback, 10)
        self.pub_odom = self.create_publisher(Odometry, '/odometry/raw', 10)
        self.pub_sensors = self.create_publisher(Float32MultiArray, '/sensors/ultrasonic', 10)
        
        if self.mock_mode:
            self.create_timer(0.05, self.physics_loop) 

    def state_callback(self, msg):
        self.current_state = msg.data

    def cmd_vel_callback(self, msg):
        self.v = msg.linear.x
        self.w = msg.angular.z

    def physics_loop(self):
        now = time.time()
        dt = now - self.last_time
        self.last_time = now
        
        self.x += self.v * math.cos(self.theta) * dt
        self.y += self.v * math.sin(self.theta) * dt
        self.theta += self.w * dt
        
        lane_width = 3.0
        self.left_us = (lane_width / 2.0) + self.y 
        self.right_us = (lane_width / 2.0) - self.y
        
        # Simulate Front US
        front_us = 3.0
        if self.current_state == "KAYAR_ENGEL":
            # Oscillate front distance between 1.0 (blocked) and 3.0 (clear)
            # Cycle takes ~4 seconds (simulating 1m wall moving back and forth)
            cycle = (math.sin(time.time() * 1.5) + 1.0) / 2.0 # 0.0 to 1.0
            if cycle > 0.5:
                front_us = 1.2 # Blocked
            else:
                front_us = 3.0 # Clear
                
        odom = Odometry()
        odom.header.stamp = self.get_clock().now().to_msg()
        odom.header.frame_id = "odom"
        odom.twist.twist.linear.x = self.v
        odom.twist.twist.angular.z = self.w
        odom.pose.pose.position.x = self.x
        odom.pose.pose.position.y = self.y
        self.pub_odom.publish(odom)
        
        sens_msg = Float32MultiArray()
        sens_msg.data = [front_us, self.left_us, self.right_us] 
        self.pub_sensors.publish(sens_msg)

def main(args=None):
    rclpy.init(args=args)
    node = SerialBridgeNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
EOF

# 3. REBUILD
echo "  > Rebuilding Control & Bridge..."
cd "$WS_DIR"
rm -rf build/ika_control install/ika_control build/ika_bridge install/ika_bridge
if [ -f /opt/ros/jazzy/setup.bash ]; then source /opt/ros/jazzy/setup.bash; else source /opt/ros/humble/setup.bash; fi
colcon build --packages-select ika_control ika_bridge

echo "=========================================="
echo "   STAGE 6 UPDATE COMPLETE! 🚦🚀          "
echo "=========================================="
