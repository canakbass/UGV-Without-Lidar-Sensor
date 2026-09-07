"""
TEKNOFEST UGV - Tabela Algılama

Algılanan 3 tip tabela:
  1. NUMARA (1-10): Kırmızı daire + siyah rakam → NEXT_STAGE
  2. STOP: Kırmızı daire + "STOP" yazısı → STOP
  3. HEDEF: Siyah-beyaz bullseye (kırmızı YOK) → FINISH

Algılama yöntemi: 
  - Hough Circle Transform (daire şekli → bariyerden ayırt eder)
  - İç beyazlık kontrolü (tabela iç kısmı beyaz olmalı)
  - Aspect ratio (STOP geniş metin, rakam dar)
  
Hedef tabelası:
  - SADECE "DIK_EGIM_ATIS" veya "PLATFORM_ATIS" state'inde aktif
  - İç içe siyah daire + kırmızı OLMAMASI gerekir
"""

import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from sensor_msgs.msg import CompressedImage
from ika_interfaces.msg import SignDetection
import cv2
import numpy as np
import time

class VisionDetector(Node):
    def __init__(self):
        super().__init__('vision_node')
        
        self.declare_parameter('camera_source', 'ros_topic')
        self.declare_parameter('local_camera_id', 0)
        self.declare_parameter('cooldown', 4.0)
        
        self.camera_source = self.get_parameter('camera_source').value
        self.cooldown = self.get_parameter('cooldown').value
        
        self.current_state = "IDLE"
        self.last_detection_time = 0
        self.frame_count = 0
        self.detection_count = 0
        
        self.pub_sign = self.create_publisher(SignDetection, '/vision/sign', 10)
        self.create_subscription(String, '/robot_state', self.state_cb, 10)
        
        if self.camera_source == 'ros_topic':
            self.create_subscription(
                CompressedImage, '/camera/compressed', 
                self.image_cb, 10
            )
            self.get_logger().info('Camera: ROS topic /camera/compressed')
        else:
            cam_id = self.get_parameter('local_camera_id').value
            self.cap = cv2.VideoCapture(cam_id)
            self.create_timer(0.2, self.local_camera_loop)
            self.get_logger().info(f'Camera: local /dev/video{cam_id}')
        
        self.get_logger().info('=== VISION DETECTOR v2 (Hough Circle) ===')

    def state_cb(self, msg):
        self.current_state = msg.data

    def image_cb(self, msg):
        if self.current_state in ["IDLE", "MANUAL"]:
            return
        
        now = time.time()
        if now - self.last_detection_time < self.cooldown:
            return
        
        try:
            jpeg_data = bytes(msg.data)
            np_arr = np.frombuffer(jpeg_data, np.uint8)
            frame = cv2.imdecode(np_arr, cv2.IMREAD_COLOR)
            if frame is not None:
                self.frame_count += 1
                self.process_frame(frame)
        except Exception as e:
            self.get_logger().warn(f'Frame error: {e}')
    
    def local_camera_loop(self):
        if self.current_state in ["IDLE", "MANUAL"]:
            return
        if not hasattr(self, 'cap') or not self.cap.isOpened():
            return
        now = time.time()
        if now - self.last_detection_time < self.cooldown:
            return
        ret, frame = self.cap.read()
        if ret:
            self.frame_count += 1
            self.process_frame(frame)

    def process_frame(self, frame):
        # 1. Kırmızı daire tespiti (etap/stop)
        result = self.detect_red_circle_sign(frame)
        if result is not None:
            sign_type, confidence = result
            self.publish_detection(sign_type, confidence)
            self.last_detection_time = time.time()
            return
        
        # 2. Hedef tespiti - SADECE son bölüm state'lerinde
        if self.current_state in ["PLATFORM_ATIS"]:
            if self.detect_target_sign(frame):
                self.publish_detection("FINISH", 0.90)
                self.last_detection_time = time.time()

    def detect_red_circle_sign(self, frame):
        """Kırmızı daire tespiti: Hough Circle + iç analiz"""
        hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
        
        mask1 = cv2.inRange(hsv, np.array([0, 80, 80]), np.array([10, 255, 255]))
        mask2 = cv2.inRange(hsv, np.array([160, 80, 80]), np.array([180, 255, 255]))
        red_mask = mask1 | mask2
        
        kernel = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (5, 5))
        red_mask = cv2.morphologyEx(red_mask, cv2.MORPH_CLOSE, kernel, iterations=2)
        red_mask = cv2.morphologyEx(red_mask, cv2.MORPH_OPEN, kernel, iterations=1)
        
        # Minimum kırmızı kontrolü
        total_red = cv2.countNonZero(red_mask)
        total_pixels = frame.shape[0] * frame.shape[1]
        if (total_red / total_pixels) < 0.005:
            return None
        
        blurred = cv2.GaussianBlur(red_mask, (9, 9), 2)
        circles = cv2.HoughCircles(
            blurred, cv2.HOUGH_GRADIENT,
            dp=1.2, minDist=50,
            param1=50, param2=30,
            minRadius=15, maxRadius=150
        )
        
        if circles is None:
            return None
        
        circles = np.uint16(np.around(circles))
        h, w = frame.shape[:2]
        
        for circle in circles[0]:
            cx, cy, r = int(circle[0]), int(circle[1]), int(circle[2])
            
            if cx-r < 0 or cy-r < 0 or cx+r >= w or cy+r >= h or r < 15:
                continue
            
            # Halka kırmızılık kontrolü
            circle_mask = np.zeros(red_mask.shape, np.uint8)
            cv2.circle(circle_mask, (cx, cy), r, 255, -1)
            inner_mask = np.zeros(red_mask.shape, np.uint8)
            cv2.circle(inner_mask, (cx, cy), int(r * 0.65), 255, -1)
            ring_mask = circle_mask - inner_mask
            
            ring_px = cv2.countNonZero(ring_mask)
            ring_red = cv2.countNonZero(cv2.bitwise_and(red_mask, ring_mask))
            if ring_px == 0 or (ring_red / ring_px) < 0.35:
                continue
            
            # İç beyazlık kontrolü
            ir = int(r * 0.6)
            ix1, iy1 = max(0, cx-ir), max(0, cy-ir)
            ix2, iy2 = min(w, cx+ir), min(h, cy+ir)
            roi = frame[iy1:iy2, ix1:ix2]
            if roi.size == 0:
                continue
            
            hsv_roi = cv2.cvtColor(roi, cv2.COLOR_BGR2HSV)
            white = cv2.inRange(hsv_roi, np.array([0,0,170]), np.array([180,50,255]))
            white_ratio = cv2.countNonZero(white) / (white.shape[0] * white.shape[1])
            
            if white_ratio < 0.20:
                continue  # İç kısım beyaz değil → tabela değil
            
            # Siyah içerik analizi
            gray = cv2.cvtColor(roi, cv2.COLOR_BGR2GRAY)
            _, bw = cv2.threshold(gray, 80, 255, cv2.THRESH_BINARY_INV)
            coords = cv2.findNonZero(bw)
            if coords is None or len(coords) < 10:
                continue
            
            bx, by, bw2, bh2 = cv2.boundingRect(coords)
            if bh2 == 0:
                continue
            
            aspect = bw2 / bh2
            black_fill = cv2.countNonZero(bw) / (bw.shape[0] * bw.shape[1])
            red_ratio = ring_red / ring_px
            
            self.get_logger().info(
                f'CIRCLE: ({cx},{cy}) r={r} '
                f'red={red_ratio:.2f} white={white_ratio:.2f} '
                f'aspect={aspect:.2f} black={black_fill:.2f}'
            )
            
            if black_fill < 0.05:
                continue
            
            # STOP: geniş siyah (4 harf)
            if aspect > 1.5 and black_fill > 0.12:
                self.get_logger().warn(f'>>> STOP (aspect={aspect:.1f}) <<<')
                return ("STOP", 0.95)
            else:
                # Numara (tek karakter)
                self.get_logger().warn(f'>>> NUMBER (aspect={aspect:.1f}) <<<')
                return ("NEXT_STAGE", 0.90)
        
        return None

    def detect_target_sign(self, frame):
        """
        Hedef tabelası: siyah-beyaz bullseye
        ÇOK SIKI kontrol - sadece PLATFORM_ATIS'ta çağrılır
        """
        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        
        # Çok keskin siyah
        _, black_mask = cv2.threshold(gray, 50, 255, cv2.THRESH_BINARY_INV)
        
        kernel = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (7, 7))
        black_mask = cv2.morphologyEx(black_mask, cv2.MORPH_CLOSE, kernel)
        black_mask = cv2.morphologyEx(black_mask, cv2.MORPH_OPEN, kernel)
        
        blurred = cv2.GaussianBlur(black_mask, (9, 9), 2)
        circles = cv2.HoughCircles(
            blurred, cv2.HOUGH_GRADIENT,
            dp=1.2, minDist=20,
            param1=50, param2=40,  # Çok sıkı
            minRadius=25, maxRadius=150
        )
        
        if circles is None or len(circles[0]) < 2:
            return False
        
        circles_list = [(int(c[0]), int(c[1]), int(c[2])) for c in circles[0]]
        
        for i in range(len(circles_list)):
            for j in range(i+1, len(circles_list)):
                ci, cj = circles_list[i], circles_list[j]
                dist = np.sqrt((ci[0]-cj[0])**2 + (ci[1]-cj[1])**2)
                r_diff = abs(ci[2] - cj[2])
                max_r = max(ci[2], cj[2])
                
                # İç içe: merkezler yakın + yarıçaplar farklı
                if dist < max_r * 0.2 and r_diff > max_r * 0.3:
                    # Kırmızı OLMAMALI
                    hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
                    cx, cy = ci[0], ci[1]
                    rr = max_r + 10
                    h, w = frame.shape[:2]
                    rx1, ry1 = max(0,cx-rr), max(0,cy-rr)
                    rx2, ry2 = min(w,cx+rr), min(h,cy+rr)
                    
                    hsv_r = hsv[ry1:ry2, rx1:rx2]
                    if hsv_r.size == 0:
                        continue
                    
                    r1 = cv2.inRange(hsv_r, np.array([0,80,80]), np.array([10,255,255]))
                    r2 = cv2.inRange(hsv_r, np.array([160,80,80]), np.array([180,255,255]))
                    red_pct = (cv2.countNonZero(r1|r2) / (hsv_r.shape[0]*hsv_r.shape[1])) * 100
                    
                    if red_pct > 3:
                        continue  # Kırmızı var → numara tabelası
                    
                    self.get_logger().warn(
                        f'>>> TARGET: concentric at ({cx},{cy}) '
                        f'r={ci[2]},{cj[2]} red={red_pct:.1f}% <<<'
                    )
                    return True
        
        return False

    def publish_detection(self, sign_type, confidence):
        self.detection_count += 1
        msg = SignDetection()
        msg.type = sign_type
        msg.distance = 2.0
        msg.confidence = confidence
        self.pub_sign.publish(msg)
        self.get_logger().info(
            f'[#{self.detection_count}] {sign_type} (conf={confidence:.2f})'
        )

def main(args=None):
    rclpy.init(args=args)
    node = VisionDetector()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()