import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from geometry_msgs.msg import Twist

class SerialBridgeNode(Node):
    def __init__(self):
        super().__init__('serial_bridge_node')
        self.declare_parameter('mock_mode', True)
        self.mock_mode = self.get_parameter('mock_mode').value
        
        self.get_logger().info(f'Serial Bridge Started. Mock Mode: {self.mock_mode}')
        
        # Subs
        self.create_subscription(Twist, '/cmd_vel', self.cmd_vel_callback, 10)
        
        # Pubs
        self.pub_state = self.create_publisher(String, '/robot_state', 10)
        
        if self.mock_mode:
            self.create_timer(1.0, self.mock_loop)

    def cmd_vel_callback(self, msg):
        pass # Todo: Send to ESP32

    def mock_loop(self):
        pass # Keep alive

def main(args=None):
    rclpy.init(args=args)
    node = SerialBridgeNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
