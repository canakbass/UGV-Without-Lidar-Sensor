import rclpy
from rclpy.node import Node

class FusionNode(Node):
    def __init__(self):
        super().__init__('fusion_node')
        self.get_logger().info('Fusion Node Started')

def main(args=None):
    rclpy.init(args=args)
    node = FusionNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
