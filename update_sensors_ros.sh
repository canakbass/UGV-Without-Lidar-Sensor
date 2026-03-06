#!/bin/bash
set -e
WS_DIR=~/ika_ws

echo "Updating ROS 2 Nodes for 7 Ultrasonic Sensors..."

# 1. Update navigation_node.py
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
        
        self.get_logger().info('Navigation Node (7 Sensors) Started.')

    def state_callback(self, msg):
        self.current_state = msg.data
        self.active = (self.current_state != "IDLE" and self.current_state != "MANUAL")
        
        if self.current_state == "YAN_EGIM":
            self.target_offset = -0.5 # Hug Right Wall
        else:
            self.target_offset = 0.0 # Center

    def us_callback(self, msg):
        if not self.active or len(msg.data) < 7: return
        
        # Sensor Layout: 
        # 0:FrontCenter, 1:FrontLeft, 2:FrontRight, 3:CornerFL, 4:CornerFR, 5:CornerRL, 6:CornerRR
        front_center = msg.data[0]
        front_left   = msg.data[1]
        front_right  = msg.data[2]
        
        corner_fl = msg.data[3]
        corner_fr = msg.data[4]
        
        # We use the 45-degree front corners for lane centering instead of direct sides
        error = (corner_fl - corner_fr) - self.target_offset
        angular_z = -self.kp * error 
        angular_z = max(-1.0, min(1.0, angular_z))
        
        # Speed Logic
        v = 0.5 # Safe default
        
        if self.current_state == "KAYAR_ENGEL":
            # For sliding obstacle, check all 3 front sensors to ensure safe passage
            min_front = min(front_center, front_left, front_right)
            if min_front < 1.7: # Obstacle is blocking ANY of the front paths
                v = 0.0
            else: # Path completely clear
                v = 1.5 # Dash!
        
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

# 2. Update serial_bridge_node.py
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
        
        self.current_state = "IDLE"
        
        self.get_logger().info(f'Serial Bridge (7 Sensors) Started. Mock Physics: {self.mock_mode}')
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
        dist_left = (lane_width / 2.0) + self.y 
        dist_right = (lane_width / 2.0) - self.y
        
        # Simulate 7 Sensors
        # 0:FrontCenter, 1:FrontLeft, 2:FrontRight, 3:CornerFL, 4:CornerFR, 5:CornerRL, 6:CornerRR
        fc = 3.0
        if self.current_state == "KAYAR_ENGEL":
            cycle = (math.sin(time.time() * 1.5) + 1.0) / 2.0
            if cycle > 0.5:
                fc = 1.2 # Blocked
            else:
                fc = 3.0 # Clear
                
        fl = fc * 1.1
        fr = fc * 1.1
        
        cfl = dist_left * 1.414 # Assume 45 degree angle -> distance increases by sqrt(2)
        cfr = dist_right * 1.414
        crl = cfl
        crr = cfr
                
        odom = Odometry()
        odom.header.stamp = self.get_clock().now().to_msg()
        odom.header.frame_id = "odom"
        odom.twist.twist.linear.x = self.v
        odom.twist.twist.angular.z = self.w
        odom.pose.pose.position.x = self.x
        odom.pose.pose.position.y = self.y
        self.pub_odom.publish(odom)
        
        sens_msg = Float32MultiArray()
        sens_msg.data = [fc, fl, fr, cfl, cfr, crl, crr] 
        self.pub_sensors.publish(sens_msg)

def main(args=None):
    rclpy.init(args=args)
    node = SerialBridgeNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
EOF

# 3. REBUILD
echo "Rebuilding ROS 2 Workspace..."
cd "$WS_DIR"
rm -rf build/ika_control install/ika_control build/ika_bridge install/ika_bridge

if [ -f /opt/ros/jazzy/setup.bash ]; then source /opt/ros/jazzy/setup.bash; else source /opt/ros/humble/setup.bash; fi
colcon build --packages-select ika_control ika_bridge

echo "Update complete."
