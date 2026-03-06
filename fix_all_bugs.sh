#!/bin/bash
set -e
WS_DIR=~/ika_ws

echo "============================================"
echo "  FIXING ALL ROS 2 BUGS (Final Patch)       "
echo "============================================"

# 1. Fix Acceleration Node (reads /unity/telemetry instead of /odometry/raw)
echo "  > Fixing acceleration_node.py..."
cat > "$WS_DIR/src/ika_robot/ika_control/ika_control/acceleration_node.py" << 'EOF'
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from std_msgs.msg import Float32MultiArray, String

class AccelerationNode(Node):
    def __init__(self):
        super().__init__('acceleration_node')
        
        self.state = "IDLE"
        self.start_dist = None
        self.current_dist = 0.0
        
        self.get_logger().info('Acceleration Protocol Ready (30m Limit).')
        
        self.create_subscription(Float32MultiArray, '/unity/telemetry', self.telemetry_callback, 10)
        self.create_subscription(String, '/user_command', self.command_callback, 10)
        
        self.pub_cmd = self.create_publisher(Twist, '/cmd_vel', 10)
        self.pub_state = self.create_publisher(String, '/robot_state', 10)
        
        self.create_timer(0.1, self.control_loop)

    def command_callback(self, msg):
        if msg.data == "START_AUTO_ACCEL":
            self.state = "RUNNING"
            self.start_dist = None
            self.get_logger().info("MISSION STARTED: ACCELERATION RUN")
        elif msg.data == "STOP":
            self.state = "IDLE"
            self.stop_robot()

    def telemetry_callback(self, msg):
        if len(msg.data) >= 8:
            # [0]:posX, [1]:posY, [2]:posZ, [3]:speed, [4]:yaw, [5]:pitch, [6]:roll, [7]:encoderDist
            encoder_dist = msg.data[7]
            
            if self.state == "RUNNING":
                if self.start_dist is None:
                    self.start_dist = encoder_dist
                self.current_dist = encoder_dist - self.start_dist

    def control_loop(self):
        if self.state != "IDLE":
             msg_state = String()
             msg_state.data = self.state
             self.pub_state.publish(msg_state)
        
        if self.state == "RUNNING":
            if self.current_dist >= 30.0:
                self.get_logger().warn(">>> 30 METERS REACHED! EMERGENCY BRAKE! <<<")
                self.state = "BRAKING"
                self.stop_robot()
            else:
                twist = Twist()
                twist.linear.x = 2.0
                self.pub_cmd.publish(twist)
                
        elif self.state == "BRAKING":
            self.stop_robot()

    def stop_robot(self):
        stop_cmd = Twist()
        stop_cmd.linear.x = 0.0
        self.pub_cmd.publish(stop_cmd)

def main(args=None):
    rclpy.init(args=args)
    node = AccelerationNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
EOF

# 2. Fix Navigation Node (7 sensor indices)
echo "  > Fixing navigation_node.py..."
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
            self.target_offset = -0.5
        else:
            self.target_offset = 0.0

    def us_callback(self, msg):
        if not self.active or len(msg.data) < 7: return
        
        # [0:FC, 1:FL, 2:FR, 3:CFL, 4:CFR, 5:CRL, 6:CRR]
        front_center = msg.data[0]
        front_left   = msg.data[1]
        front_right  = msg.data[2]
        corner_fl    = msg.data[3]
        corner_fr    = msg.data[4]
        
        # PID: 45 derece köşe sensörleriyle yol ortalama
        error = (corner_fl - corner_fr) - self.target_offset
        angular_z = -self.kp * error 
        angular_z = max(-1.0, min(1.0, angular_z))
        
        v = 0.5
        
        if self.current_state == "KAYAR_ENGEL":
            min_front = min(front_center, front_left, front_right)
            if min_front < 1.7:
                v = 0.0
                self.get_logger().warn(f"Obstacle blocks! ({min_front:.2f}m) WAITING...")
            else:
                v = 1.5
                self.get_logger().info(f"Path clear ({min_front:.2f}m)! DASH!")
        
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

# 3. Fix Web Bridge (forward /unity/telemetry to web-readable topics)
echo "  > Fixing web_bridge_node.py..."
cat > "$WS_DIR/src/ika_robot/ika_web_bridge/ika_web_bridge/web_bridge_node.py" << 'EOF'
import rclpy
from rclpy.node import Node
from std_msgs.msg import String, Float32MultiArray
from nav_msgs.msg import Odometry

class WebBridgeNode(Node):
    def __init__(self):
        super().__init__('web_bridge')
        
        # Unity telemetry -> Odometry (for web UI speed display)
        self.create_subscription(Float32MultiArray, '/unity/telemetry', self.telemetry_callback, 10)
        self.pub_odom = self.create_publisher(Odometry, '/odometry/raw', 10)
        
        self.get_logger().info('Web Bridge Node Started')

    def telemetry_callback(self, msg):
        if len(msg.data) < 8: return
        
        odom = Odometry()
        odom.header.frame_id = "odom"
        odom.header.stamp = self.get_clock().now().to_msg()
        odom.pose.pose.position.x = float(msg.data[7])  # encoder distance
        odom.pose.pose.position.y = float(msg.data[0])   # posX
        odom.twist.twist.linear.x = float(msg.data[3])   # speed
        self.pub_odom.publish(odom)

def main(args=None):
    rclpy.init(args=args)
    node = WebBridgeNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
EOF

# 4. Fix Launch File
echo "  > Fixing system.launch.py..."
cat > "$WS_DIR/src/ika_robot/ika_bringup/launch/system.launch.py" << 'EOF'
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
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
            package='ika_vision',
            executable='detector',
            name='vision_node'
        ),
        
        Node(
            package='rosbridge_server',
            executable='rosbridge_websocket',
            name='rosbridge_websocket'
        ),
    ])
EOF

# 5. Rebuild
echo "  > Rebuilding all ROS 2 packages..."
cd "$WS_DIR"
rm -rf build/ika_control install/ika_control
rm -rf build/ika_web_bridge install/ika_web_bridge
rm -rf build/ika_bringup install/ika_bringup

if [ -f /opt/ros/jazzy/setup.bash ]; then source /opt/ros/jazzy/setup.bash; else source /opt/ros/humble/setup.bash; fi
source install/setup.bash 2>/dev/null || true
colcon build --packages-select ika_control ika_web_bridge ika_bringup

echo "============================================"
echo "  ALL BUGS FIXED! ✅                         "
echo "============================================"
