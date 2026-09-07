import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Imu
from std_msgs.msg import Bool

import math
import time
# import serial # Commented out for mock-first approach, will add later

class SerialNode(Node):
    def __init__(self):
        super().__init__('serial_node')

        self.declare_parameter('mock_mode', True)
        self.mock_mode = self.get_parameter('mock_mode').value
        
        self.get_logger().info(f'Serial Node Started. Mock Mode: {self.mock_mode}')

        # Subscribe to commands
        self.create_subscription(Twist, '/cmd_vel', self.cmd_callback, 10)
        self.create_subscription(Bool, '/system/emergency_brake', self.brake_callback, 10)

        # Publish sensors
        self.pub_odom = self.create_publisher(Odometry, '/odometry/raw', 10)
        self.pub_imu = self.create_publisher(Imu, '/sensors/imu_raw', 10)

        # Mock State
        self.x = 0.0
        self.y = 0.0
        self.th = 0.0
        self.last_time = time.time()
        self.current_v = 0.0
        self.current_w = 0.0

        if self.mock_mode:
            self.timer = self.create_timer(0.05, self.mock_loop) # 20Hz Simulation
        
    def cmd_callback(self, msg):
        # Output to Serial if real
        if not self.mock_mode:
            # write_serial_packet(msg.linear.x, msg.angular.z)
            pass
        else:
            # Update target speeds for simulation
            self.current_v = msg.linear.x
            self.current_w = msg.angular.z

    def brake_callback(self, msg):
        if msg.data:
            self.get_logger().warn('HARD HARD BRAKE COMMAND RECEIVED - Stopping Motors')
            self.current_v = 0.0
            self.current_w = 0.0

    def mock_loop(self):
        # Simulate Robot Kinematics (Differential Drive)
        now = time.time()
        dt = now - self.last_time
        self.last_time = now

        # Simple euler integration
        delta_x = (self.current_v * math.cos(self.th)) * dt
        delta_y = (self.current_v * math.sin(self.th)) * dt
        delta_th = self.current_w * dt

        self.x += delta_x
        self.y += delta_y
        self.th += delta_th

        # Publish Odometry
        odom = Odometry()
        odom.header.stamp = self.get_clock().now().to_msg()
        odom.header.frame_id = "odom"
        odom.pose.pose.position.x = self.x
        odom.pose.pose.position.y = self.y
        
        # Quaternion from Yaw
        odom.pose.pose.orientation.z = math.sin(self.th / 2.0)
        odom.pose.pose.orientation.w = math.cos(self.th / 2.0)

        self.pub_odom.publish(odom)

        # Publish Mock IMU
        imu = Imu()
        imu.header.stamp = self.get_clock().now().to_msg()
        imu.orientation = odom.pose.pose.orientation
        self.pub_imu.publish(imu)

def main(args=None):
    rclpy.init(args=args)
    node = SerialNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
