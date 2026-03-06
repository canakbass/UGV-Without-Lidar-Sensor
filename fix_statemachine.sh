#!/bin/bash
set -e
WS_DIR=~/ika_ws

echo "============================================"
echo "  FIXING STATE MACHINE + DETECTOR           "
echo "============================================"

# 1. Fix State Machine (was just publishing IDLE forever)
echo "  > Fixing state_machine_node.py..."
cat > "$WS_DIR/src/ika_robot/ika_decision/ika_decision/state_machine_node.py" << 'EOF'
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from ika_interfaces.msg import SignDetection

class StateMachineNode(Node):
    def __init__(self):
        super().__init__('state_machine_node')
        
        self.state = "IDLE"
        self.stages = [
            "NORMAL",           # 1. Normal sürüş
            "TASLI_YOL",        # 2. Taşlı/engebeli yol
            "YAN_EGIM",         # 3. Yan eğim
            "DIK_ENGEL",        # 4. Dik engeller
            "TRAFIK_KONILERI",  # 5. Trafik konileri
            "KAYAR_ENGEL"       # 6. Kayar engel
        ]
        self.stage_idx = 0
        
        self.pub_state = self.create_publisher(String, '/robot_state', 10)
        self.create_subscription(SignDetection, '/vision/sign', self.sign_callback, 10)
        self.create_subscription(String, '/user_command', self.cmd_callback, 10)
        
        self.create_timer(0.5, self.publish_state)
        self.get_logger().info('State Machine Ready. Stages: ' + str(self.stages))

    def cmd_callback(self, msg):
        cmd = msg.data
        self.get_logger().info(f'Command received: {cmd}')
        
        if cmd == "START_AUTO_NORMAL":
            self.state = self.stages[0]
            self.stage_idx = 0
            self.get_logger().info(f'=== AUTOPILOT STARTED === Stage: {self.state}')
        elif cmd == "START_AUTO_ACCEL":
            self.state = "ACCEL_RUN"
            self.get_logger().info('=== ACCELERATION RUN STARTED ===')
        elif cmd == "STOP":
            self.state = "IDLE"
            self.get_logger().info('=== EMERGENCY STOP ===')
        elif cmd == "MANUAL":
            self.state = "MANUAL"

    def sign_callback(self, msg):
        if self.state == "IDLE" or self.state == "MANUAL":
            return
            
        self.get_logger().info(f'Sign detected: type={msg.type}, confidence={msg.confidence:.2f}')
        
        if msg.type == "NEXT_STAGE" and msg.confidence > 0.7:
            old_state = self.state
            self.stage_idx = min(self.stage_idx + 1, len(self.stages) - 1)
            self.state = self.stages[self.stage_idx]
            self.get_logger().warn(f'STAGE TRANSITION: {old_state} -> {self.state}')
            
        elif msg.type == "STOP" and msg.confidence > 0.7:
            self.get_logger().warn('STOP SIGN DETECTED -> STOPPING')
            self.state = "IDLE"
            
        elif msg.type == "FINISH" and msg.confidence > 0.7:
            self.get_logger().warn('FINISH LINE DETECTED -> MISSION COMPLETE')
            self.state = "IDLE"

    def publish_state(self):
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

# 2. Fix Detector Node (was sending wrong sign types, now stage-aware)
echo "  > Fixing detector_node.py..."
cat > "$WS_DIR/src/ika_robot/ika_vision/ika_vision/detector_node.py" << 'EOF'
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from ika_interfaces.msg import SignDetection
import random

class VisionNode(Node):
    def __init__(self):
        super().__init__('vision_node')
        self.current_state = "IDLE"
        
        self.create_subscription(String, '/robot_state', self.state_callback, 10)
        self.pub_sign = self.create_publisher(SignDetection, '/vision/sign', 10)
        
        # Timer: Her 2 saniyede bir kontrol et
        self.create_timer(2.0, self.detection_loop)
        
        self.get_logger().info('Vision Node Started (Unity Camera Mode)')

    def state_callback(self, msg):
        self.current_state = msg.data

    def detection_loop(self):
        if self.current_state == "IDLE" or self.current_state == "MANUAL":
            return
        
        # === GERÇEK MODDA BURASI KAMERA İŞLEME OLACAK ===
        # Şimdilik Unity'den gelen sensör verilerine göre simüle ediyoruz
        # Gerçek sistemde: OpenCV HSV maskeleme + Tesseract OCR / YOLOv8
        
        # Sadece koni etabında koni tespiti yap (CPU tasarrufu)
        if self.current_state == "TRAFIK_KONILERI":
            self.get_logger().info("CONE DETECTION ACTIVE (Stage 5)")
            # Gerçek: Turuncu renk maskeleme + kontur analizi
        
        # Tabela tespiti her zaman aktif ama spam yapma
        # Gerçek sistemde kamera frame'den OCR/YOLO ile tespit edilecek
        # Şimdilik bu fonksiyon boş - tabela tespiti Unity'den gelecek

def main(args=None):
    rclpy.init(args=args)
    node = VisionNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
EOF

# 3. Rebuild
echo "  > Rebuilding..."
cd "$WS_DIR"
rm -rf build/ika_decision install/ika_decision build/ika_vision install/ika_vision

if [ -f /opt/ros/jazzy/setup.bash ]; then source /opt/ros/jazzy/setup.bash; else source /opt/ros/humble/setup.bash; fi
source install/setup.bash 2>/dev/null || true
colcon build --packages-select ika_decision ika_vision

echo "============================================"
echo "  STATE MACHINE + DETECTOR FIXED! ✅         "
echo "============================================"
