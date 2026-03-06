import rclpy
from rclpy.node import Node
from std_msgs.msg import String

class StateMachineNode(Node):
    def __init__(self):
        super().__init__('state_machine_node')
        self.get_logger().info('State Machine Node Started')
        self.pub_state = self.create_publisher(String, '/robot_state', 10)
        self.create_timer(1.0, self.loop)
        
    def loop(self):
        msg = String()
        msg.data = "IDLE"
        self.pub_state.publish(msg)

def main(args=None):
    rclpy.init(args=args)
    node = StateMachineNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
