#!/bin/bash
set -e
WS_DIR=~/ika_ws

echo "=========================================="
echo "   UPDATING NODES FOR V1 (MOCK LOGIC)     "
echo "=========================================="

# 1. SERIAL BRIDGE (PHYSICS SIMULATOR)
echo "  > Updating serial_bridge_node.py..."
cat > "$WS_DIR/src/ika_robot/ika_bridge/ika_bridge/serial_bridge_node.py" << EOF
import rclpy
from rclpy.node import Node
from std_msgs.msg import String, Float32
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
import math
import time

class SerialBridgeNode(Node):
    def __init__(self):
        super().__init__('serial_bridge_node')
        self.declare_parameter('mock_mode', True)
        self.mock_mode = self.get_parameter('mock_mode').value
        
        # Physics State
        self.x = 0.0          # Current Position (m)
        self.current_vel = 0.0 # Current Speed (m/s)
        self.target_vel = 0.0  # Desired Speed (m/s)
        self.last_time = time.time()
        
        # Params
        self.max_accel = 2.0  # m/s^2 (Aggressive acceleration)
        self.max_decel = 5.0  # m/s^2 (Strong Braking)
        
        self.get_logger().info(f'Serial Bridge Started. Mock Physics: {self.mock_mode}')
        
        # Subs
        self.create_subscription(Twist, '/cmd_vel', self.cmd_vel_callback, 10)
        
        # Pubs
        self.pub_odom = self.create_publisher(Odometry, '/odometry/raw', 10)
        self.pub_state = self.create_publisher(String, '/robot_state', 10)
        
        if self.mock_mode:
            self.create_timer(0.05, self.physics_loop) # 20Hz Update

    def cmd_vel_callback(self, msg):
        self.target_vel = msg.linear.x
        # Angular ignored for straight line mock

    def physics_loop(self):
        now = time.time()
        dt = now - self.last_time
        self.last_time = now
        
        if dt > 0.1: dt = 0.05 # Clamp large steps
        
        # Acceleration Logic
        error = self.target_vel - self.current_vel
        
        if abs(error) < 0.05:
            self.current_vel = self.target_vel
        else:
            limit = self.max_accel if error > 0 else self.max_decel
            step = limit * dt
            if error > 0:
                self.current_vel += min(step, error)
            else:
                self.current_vel += max(-step, error)
                
        # Integration (Odom)
        self.x += self.current_vel * dt
        
        # Publish Odom
        odom = Odometry()
        odom.header.stamp = self.get_clock().now().to_msg()
        odom.header.frame_id = "odom"
        odom.child_frame_id = "base_link"
        
        # Twist
        odom.twist.twist.linear.x = self.current_vel
        # Pose
        odom.pose.pose.position.x = self.x
        
        self.pub_odom.publish(odom)

def main(args=None):
    rclpy.init(args=args)
    node = SerialBridgeNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
EOF

# 2. ACCELERATION LOGIC (30m SPRINT)
echo "  > Updating acceleration_node.py..."
cat > "$WS_DIR/src/ika_robot/ika_control/ika_control/acceleration_node.py" << EOF
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from std_msgs.msg import String

class AccelerationNode(Node):
    def __init__(self):
        super().__init__('acceleration_node')
        
        self.state = "IDLE" # IDLE, RUNNING, BRAKING, FINISHED
        self.start_x = None
        self.distance_traveled = 0.0
        
        self.get_logger().info('Acceleration Protocol Ready.')
        
        # Subs
        self.create_subscription(Odometry, '/odometry/raw', self.odom_callback, 10)
        self.create_subscription(String, '/user_command', self.command_callback, 10)
        
        # Pubs
        self.pub_cmd = self.create_publisher(Twist, '/cmd_vel', 10)
        self.pub_state = self.create_publisher(String, '/robot_state', 10)
        
        # Control Loop (10Hz)
        self.create_timer(0.1, self.control_loop)

    def command_callback(self, msg):
        if msg.data == "START_AUTO_ACCEL":
            self.state = "RUNNING"
            self.start_x = None # Reset origin
            self.get_logger().info("MISSION STARTED: ACCELERATION RUN")
        elif msg.data == "STOP":
            self.state = "IDLE"
            self.stop_robot()

    def odom_callback(self, msg):
        current_x = msg.pose.pose.position.x
        
        if self.state == "RUNNING":
            if self.start_x is None:
                self.start_x = current_x
            
            self.distance_traveled = current_x - self.start_x
            # self.get_logger().info(f"Distance: {self.distance_traveled:.2f}m")

    def control_loop(self):
        msg_state = String()
        msg_state.data = self.state
        self.pub_state.publish(msg_state)
        
        if self.state == "RUNNING":
            # Check for 30m Limit
            if self.distance_traveled >= 30.0:
                self.get_logger().warn(">>> 30 METERS REACHED! EMERGENCY BRAKE! <<<")
                self.state = "BRAKING"
                self.stop_robot()
            else:
                # Full Speed Ahead
                twist = Twist()
                twist.linear.x = 2.0 # Target 2 m/s
                self.pub_cmd.publish(twist)
                
        elif self.state == "BRAKING":
            # Keep sending stop command for a bit to ensure safety
            self.stop_robot()
            # Transition to FINISHED? Logic needed here.

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


# 3. VISION NODE (SCENARIOS)
echo "  > Updating detector_node.py..."
cat > "$WS_DIR/src/ika_robot/ika_vision/ika_vision/detector_node.py" << EOF
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32, String
import math

class VisionNode(Node):
    def __init__(self):
        super().__init__('vision_node')
        self.declare_parameter('mock_mode', True)
        self.declare_parameter('scenario', 'STRAIGHT') # STRAIGHT, CURVE
        
        self.scenario = self.get_parameter('scenario').value
        self.start_time = self.get_clock().now().nanoseconds / 1e9
        
        self.get_logger().info(f'Vision Node Started. Scenario: {self.scenario}')
        
        self.pub_lane_error = self.create_publisher(Float32, '/vision/lane_error', 10)
        self.create_timer(0.1, self.mock_loop)

    def mock_loop(self):
        t = (self.get_clock().now().nanoseconds / 1e9) - self.start_time
        error = 0.0
        
        if self.scenario == 'CURVE':
             # Simulate a gentle S-curve
             error = 0.5 * math.sin(t * 0.5)
        else:
             # Straight line noise
             import random
             error = random.uniform(-0.05, 0.05)
             
        msg = Float32()
        msg.data = error
        self.pub_lane_error.publish(msg)

def main(args=None):
    rclpy.init(args=args)
    node = VisionNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
EOF

# 4. REBUILD
echo "  > Rebuilding..."
cd "$WS_DIR"
if [ -f /opt/ros/jazzy/setup.bash ]; then source /opt/ros/jazzy/setup.bash; else source /opt/ros/humble/setup.bash; fi
colcon build

echo "V1 UPDATE COMPLETE."
echo "Restart the simulation with:"
echo "source install/setup.bash && ros2 launch ika_bringup system.launch.py"
EOF
