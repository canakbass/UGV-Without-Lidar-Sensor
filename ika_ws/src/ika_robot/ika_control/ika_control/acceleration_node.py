import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist

class AccelerationNode(Node):
    def __init__(self):
        super().__init__('acceleration_node')
        self.get_logger().info('Acceleration Node Started')

def main(args=None):
    rclpy.init(args=args)
    node = AccelerationNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
