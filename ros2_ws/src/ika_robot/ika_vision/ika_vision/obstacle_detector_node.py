"""
Engel Algılama Node - Koni + Hareketli Duvar

Stage 5 (TRAFIK_KONILERI): Turuncu konileri algıla
  - HSV turuncu filtreleme
  - Konilerin sol/sağ dağılımına göre yön ver

Stage 6 (KAYAR_ENGEL): Beyaz hareketli duvarı algıla
  - Büyük beyaz bölge tespiti
  - Frame-to-frame pozisyon takibi
  - Duvarın hangi tarafta olduğuna göre yön ver
"""

import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import cv2
import numpy as np
import base64
import time

class ObstacleDetector(Node):
    def __init__(self):
        super().__init__('obstacle_detector')
        
        self.current_state = "IDLE"
        self.prev_wall_cx = None
        self.frame_count = 0
        
        self.pub_cone = self.create_publisher(String, '/vision/cone_steer', 10)
        self.pub_wall = self.create_publisher(String, '/vision/wall_steer', 10)
        self.create_subscription(String, '/robot_state', self.state_cb, 10)
        self.create_subscription(String, '/camera/image_base64', self.image_cb, 10)
        
        self.create_timer(10.0, self.stats)
        self.get_logger().info('=== OBSTACLE DETECTOR (Cone + Wall) ===')
    
    def state_cb(self, msg):
        old = self.current_state
        self.current_state = msg.data
        if old != self.current_state:
            self.get_logger().info(f'State: {old} → {self.current_state}')
            self.prev_wall_cx = None
    
    def stats(self):
        self.get_logger().info(f'Obstacle: state={self.current_state}, frames={self.frame_count}')
    
    def image_cb(self, msg):
        if self.current_state not in ["TRAFIK_KONILERI", "KAYAR_ENGEL"]:
            return
        
        try:
            jpeg_bytes = base64.b64decode(msg.data)
            np_arr = np.frombuffer(jpeg_bytes, np.uint8)
            frame = cv2.imdecode(np_arr, cv2.IMREAD_COLOR)
            if frame is None:
                return
            self.frame_count += 1
        except:
            return
        
        if self.current_state == "TRAFIK_KONILERI":
            self.detect_cones(frame)
        elif self.current_state == "KAYAR_ENGEL":
            self.detect_moving_wall(frame)
    
    def detect_cones(self, frame):
        """Turuncu trafik konilerini algıla"""
        hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
        
        # Turuncu renk (koniler H=5-25, yüksek satürasyon)
        mask = cv2.inRange(hsv, np.array([5, 100, 100]), np.array([25, 255, 255]))
        
        k = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (5, 5))
        mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, k)
        mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, k)
        
        contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        
        h, w = frame.shape[:2]
        mid_x = w // 2
        
        left_cones = 0
        right_cones = 0
        left_area = 0
        right_area = 0
        
        for cnt in contours:
            area = cv2.contourArea(cnt)
            if area < 100:
                continue
            M = cv2.moments(cnt)
            if M["m00"] == 0:
                continue
            cx = int(M["m10"] / M["m00"])
            
            if cx < mid_x:
                left_cones += 1
                left_area += area
            else:
                right_cones += 1
                right_area += area
        
        total = left_cones + right_cones
        if total == 0:
            return
        
        # Koniler hangi tarafta daha yoğun → o tarafa GİTME
        if left_area > right_area * 1.3:
            steer = -0.4  # Sola koni var → sağa git
        elif right_area > left_area * 1.3:
            steer = 0.4   # Sağa koni var → sola git
        else:
            steer = 0.0   # Eşit → düz
        
        msg = String()
        msg.data = str(steer)
        self.pub_cone.publish(msg)
        
        if self.frame_count % 10 == 0:
            self.get_logger().info(
                f'CONES: L={left_cones}({left_area}) R={right_cones}({right_area}) → steer={steer:.1f}'
            )
    
    def detect_moving_wall(self, frame):
        """Beyaz hareketli duvarı algıla ve yönünü belirle"""
        hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
        
        # Beyaz bölge (duvar)
        white_mask = cv2.inRange(hsv, np.array([0, 0, 180]), np.array([180, 40, 255]))
        
        # Alt yarıya odaklan (duvar yol seviyesinde)
        h, w = frame.shape[:2]
        white_mask[:h//3, :] = 0  # Üst 1/3'ü görmezden gel (gökyüzü)
        
        k = cv2.getStructuringElement(cv2.MORPH_RECT, (10, 10))
        white_mask = cv2.morphologyEx(white_mask, cv2.MORPH_CLOSE, k)
        
        contours, _ = cv2.findContours(white_mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        
        if not contours:
            self.prev_wall_cx = None
            return
        
        # En büyük beyaz bölge = duvar
        biggest = max(contours, key=cv2.contourArea)
        area = cv2.contourArea(biggest)
        
        if area < 1000:
            self.prev_wall_cx = None
            return
        
        M = cv2.moments(biggest)
        if M["m00"] == 0:
            return
        
        wall_cx = int(M["m10"] / M["m00"])
        mid_x = w // 2
        
        # Duvar nerede? → ters taraftan geç
        if wall_cx < mid_x - 30:
            steer = -0.5  # Duvar solda → sağa git
        elif wall_cx > mid_x + 30:
            steer = 0.5   # Duvar sağda → sola git
        else:
            # Duvar ortada → hangi tarafa kayıyor?
            if self.prev_wall_cx is not None:
                dx = wall_cx - self.prev_wall_cx
                if dx > 3:
                    steer = 0.4  # Duvar sağa gidiyor → sola git (ters)
                elif dx < -3:
                    steer = -0.4  # Duvar sola gidiyor → sağa git
                else:
                    steer = 0.0
            else:
                steer = 0.0
        
        self.prev_wall_cx = wall_cx
        
        msg = String()
        msg.data = str(steer)
        self.pub_wall.publish(msg)
        
        if self.frame_count % 10 == 0:
            self.get_logger().info(
                f'WALL: cx={wall_cx} mid={mid_x} area={area} → steer={steer:.1f}'
            )

def main(args=None):
    rclpy.init(args=args)
    node = ObstacleDetector()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()