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

if __name__ == '__main__':
    main()
