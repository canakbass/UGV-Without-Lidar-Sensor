import rclpy
from rclpy.node import Node

class ControllerNode(Node):
    def __init__(self):
        super().__init__('controller_node')
        self.get_logger().info('Controller Node Started')

def main(args=None):
    rclpy.init(args=args)
    node = ControllerNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
