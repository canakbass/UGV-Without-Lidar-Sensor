#!/bin/bash
set -e
WS_DIR=~/ika_ws

echo "Updating detector to Unity-side mode..."

cat > "$WS_DIR/src/ika_robot/ika_vision/ika_vision/detector_node.py" << 'PYEOF'
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from ika_interfaces.msg import SignDetection

class VisionNode(Node):
    """
    Vision Node - Sign Detection Relay
    
    Tabela algılama artık Unity tarafında (SignDetector.cs) yapılıyor.
    Bu node sadece /vision/sign topic'ini dinleyip loglama yapıyor.
    Gerçek robotda burası Jetson Nano'da OpenCV/YOLO ile değiştirilecek.
    """
    def __init__(self):
        super().__init__('vision_node')
        
        self.current_state = "IDLE"
        self.detection_count = 0
        
        self.create_subscription(String, '/robot_state', self.state_callback, 10)
        self.create_subscription(SignDetection, '/vision/sign', self.sign_callback, 10)
        
        self.get_logger().info('Vision Node Started (Unity-side detection mode)')
        self.get_logger().info('Sign detection runs in Unity SignDetector.cs')

    def state_callback(self, msg):
        self.current_state = msg.data

    def sign_callback(self, msg):
        self.detection_count += 1
        self.get_logger().info(
            f'[Detection #{self.detection_count}] '
            f'Type: {msg.type}, Confidence: {msg.confidence:.2f}, '
            f'Distance: {msg.distance:.1f}m'
        )

def main(args=None):
    rclpy.init(args=args)
    node = VisionNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
PYEOF

cd "$WS_DIR"
rm -rf build/ika_vision install/ika_vision
if [ -f /opt/ros/jazzy/setup.bash ]; then source /opt/ros/jazzy/setup.bash; else source /opt/ros/humble/setup.bash; fi
source install/setup.bash 2>/dev/null || true
colcon build --packages-select ika_vision

echo "DONE - Vision node updated!"
