import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from std_msgs.msg import Float32MultiArray, String

class NavigationNode(Node):
    def __init__(self):
        super().__init__('navigation_node')
        
        self.active = False
        self.kp = 0.8
        self.target_offset = 0.0
        self.current_state = "IDLE"
        
        self.create_subscription(Float32MultiArray, '/sensors/ultrasonic', self.us_callback, 10)
        self.create_subscription(String, '/robot_state', self.state_callback, 10)
        self.pub_cmd = self.create_publisher(Twist, '/cmd_vel', 10)
        
        self.get_logger().info('Navigation Node (7 Sensors) Started.')

    def state_callback(self, msg):
        self.current_state = msg.data
        self.active = (self.current_state != "IDLE" and self.current_state != "MANUAL")
        
        if self.current_state == "YAN_EGIM":
            self.target_offset = -0.5 # Hug Right Wall
        else:
            self.target_offset = 0.0 # Center

    def us_callback(self, msg):
        if not self.active or len(msg.data) < 7: return
        
        # Sensor Layout: 
        # 0:FrontCenter, 1:FrontLeft, 2:FrontRight, 3:CornerFL, 4:CornerFR, 5:CornerRL, 6:CornerRR
        front_center = msg.data[0]
        front_left   = msg.data[1]
        front_right  = msg.data[2]
        
        corner_fl = msg.data[3]
        corner_fr = msg.data[4]
        
        # We use the 45-degree front corners for lane centering instead of direct sides
        error = (corner_fl - corner_fr) - self.target_offset
        angular_z = -self.kp * error 
        angular_z = max(-1.0, min(1.0, angular_z))
        
        # Speed Logic
        v = 0.5 # Safe default
        
        if self.current_state == "KAYAR_ENGEL":
            # For sliding obstacle, check all 3 front sensors to ensure safe passage
            min_front = min(front_center, front_left, front_right)
            if min_front < 1.7: # Obstacle is blocking ANY of the front paths
                v = 0.0
            else: # Path completely clear
                v = 1.5 # Dash!
        
        cmd = Twist()
        cmd.linear.x = v
        cmd.angular.z = angular_z
        self.pub_cmd.publish(cmd)

def main(args=None):
    rclpy.init(args=args)
    node = NavigationNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()