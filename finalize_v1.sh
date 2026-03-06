#!/bin/bash
set -e
WS_DIR=~/ika_ws
ROOT_DIR="/mnt/c/Users/1hayr/OneDrive/Desktop/ika yeni"

echo "=========================================="
echo "   FINALIZING V1 RELEASE (GITHUB READY)   "
echo "=========================================="

# 1. FIX ACCELERATION NODE (Was Empty Skeleton)
echo "  > Injecting Acceleration Logic (30m Sprint)..."
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
        
        self.get_logger().info('Acceleration Protocol Ready (30m Limit).')
        
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
        # Always publish state if active
        if self.state != "IDLE":
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
                # Full Speed Ahead (Mock)
                twist = Twist()
                twist.linear.x = 2.0 # Target 2 m/s
                self.pub_cmd.publish(twist)
                
        elif self.state == "BRAKING":
            # Keep sending stop command for a bit
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

# 2. CREATE .GITIGNORE (Critical for GitHub)
echo "  > Creating .gitignore..."
cat > "$ROOT_DIR/.gitignore" << EOF
# Node
node_modules/
dist/
build/
.env

# ROS2 Workspace Artifacts
ika_ws/build/
ika_ws/install/
ika_ws/log/
ika_ws/.vscode/

# Python
__pycache__/
*.pyc
*.pyo
*.pyd

# System
.DS_Store
Thumbs.db

# IDE
.vscode/
.idea/
EOF

# 3. CREATE README.md
echo "  > Creating README.md..."
cat > "$ROOT_DIR/README.md" << EOF
# TEKNOFEST 2026 UGV - Autonomous System (V1 Simulation)

This repository contains the full software stack for the TEKNOFEST 2026 Unmanned Ground Vehicle competition.
The V1 release focuses on **Navigation Logic**, **Vision Processing**, and **Mock Physics Simulation**.

## 🚀 Features (V1)
*   **Modular ROS2 Architecture**: Divided into Control, Vision, Decision, and Bridge packages.
*   **Smart Navigation**:
    *   **Ultrasonic PID**: Lane centering using side distance difference.
    *   **Dynamic Offset**: Automatically hugs the right wall (-30cm) during "Side Slope" and "Sliding Obstacle" stages.
*   **Vision System**:
    *   **Stage Detection**: Recognizes traffic signs to switch autonomous states.
    *   **Optimized Processing**: Only activates Cone Detection algorithms when in the specific stage.
*   **Mock Simulation**:
    *   Simulates inertia, differential drive kinematics, and sensor noise.
    *   Outputs synthetic Odometry and Ultrasonic data for logic verification.
*   **Web Interface**:
    *   React + Vite frontend for real-time telemetry and manual/auto control.

## 🛠️ Installation (WSL / Ubuntu 24.04)
1.  **Clone the Repository**:
    \`\`\`bash
    git clone https://github.com/Start-Up-Tech/2026-UGV.git
    cd 2026-UGV
    \`\`\`

2.  **Setup Environment**:
    \`\`\`bash
    bash setup_wsl.sh
    \`\`\`

3.  **Launch Simulation**:
    \`\`\`bash
    source ~/ika_ws/install/setup.bash && ros2 launch ika_bringup system.launch.py
    \`\`\`

4.  **Start Web UI**:
    \`\`\`bash
    cd ika_web
    npm install
    npm run dev
    \`\`\`

## 🧠 Architecture
*   **Decisions**: \`ika_decision/state_machine_node\`
*   **Eyes**: \`ika_vision/detector_node\`
*   **Legs**: \`ika_control/navigation_node\` & \`acceleration_node\`
*   **Nerves**: \`ika_bridge/serial_bridge_node\`

## 📅 Roadmap
- [x] **V1**: Mock Simulation & Logic Verification
- [ ] **V2**: Unity Digital Twin Integration
- [ ] **V3**: Real Hardware Deployment (Jetson Nano + ESP32)
EOF

# 4. REBUILD CONTROL PACKAGE
echo "  > Rebuilding Control Package..."
cd "$WS_DIR"
rm -rf build/ika_control install/ika_control
if [ -f /opt/ros/jazzy/setup.bash ]; then source /opt/ros/jazzy/setup.bash; else source /opt/ros/humble/setup.bash; fi
colcon build --packages-select ika_control

echo "=========================================="
echo "   V1 RELEASE READY FOR PUBLISH! 🚀       "
echo "=========================================="
EOF
