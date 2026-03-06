import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from std_msgs.msg import Float32, String
from ika_interfaces.msg import SignDetection
# import cv2 # OpenCv - Commented out for initial structure
# from cv_bridge import CvBridge # Commented out

class VisionNode(Node):
    def __init__(self):
        super().__init__('vision_node')
        
        self.declare_parameter('mock_mode', True)
        self.mock_mode = self.get_parameter('mock_mode').value
        
        # Publishers
        self.pub_lane_error = self.create_publisher(Float32, '/vision/lane_error', 10)
        self.pub_sign = self.create_publisher(SignDetection, '/vision/sign', 10)
        
        # Subscribers
        # self.sub_image = self.create_subscription(Image, '/camera/image_raw', self.image_callback, 10)
        
        if self.mock_mode:
            self.timer = self.create_timer(1.0, self.mock_loop)
            self.get_logger().info('Vision Node Started in MOCK MODE')
        else:
            self.get_logger().info('Vision Node Started (Real Camera Mode)')
            # self.bridge = CvBridge()

    def image_callback(self, msg):
        # Real image processing logic here (HSV, Canny, Hough)
        pass

    def mock_loop(self):
        # Simulate varying lane error (Sine wave behavior)
        import math
        import time
        
        t = time.time()
        error = 0.5 * math.sin(t) # Oscillate between -0.5 and 0.5 meters
        
        msg = Float32()
        msg.data = error
        self.pub_lane_error.publish(msg)
        
        # Simulate random sign detection occasionally
        if int(t) % 15 == 0: # Every 15 seconds
            sign_msg = SignDetection()
            sign_msg.type = "TURN_LEFT"
            sign_msg.distance = 2.5
            sign_msg.confidence = 0.95
            self.pub_sign.publish(sign_msg)
            self.get_logger().info('Mock Sign Detected: TURN_LEFT')

def main(args=None):
    rclpy.init(args=args)
    node = VisionNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
