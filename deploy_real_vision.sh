#!/bin/bash
set -e
WS_DIR=~/ika_ws

echo "============================================"
echo "  REAL VISION DETECTOR (Hough Circle)       "
echo "============================================"

cat > "$WS_DIR/src/ika_robot/ika_vision/ika_vision/detector_node.py" << 'PYEOF'
"""
TEKNOFEST UGV - Gerçek Görüntü İşleme ile Tabela Algılama

Bu kod hem simülasyonda hem gerçek robotta çalışır.

Algılama Mantığı:
  1. Kamera frame'ini al (ROS topic veya yerel kamera)
  2. HSV renk uzayına çevir
  3. Kırmızı maske oluştur
  4. Morfolojik işlemlerle gürültüyü temizle
  5. Hough Circle Transform ile DAIRE bul (bariyerler dikdörtgen, tabelalar daire)
  6. Bulunan dairenin içini analiz et:
     - İç kısımdaki siyah alanın genişlik/yükseklik oranı
     - Geniş (>1.5) → STOP yazısı
     - Dar (<=1.5) → Tek rakam (etap numarası)
  7. Eğer kırmızı daire yoksa → siyah daire ara → HEDEF tabelası

Gerçek Robotta:
  camera_source parametresini "local" yaparak /dev/video0'dan okur.
"""

import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from sensor_msgs.msg import CompressedImage
from ika_interfaces.msg import SignDetection
import cv2
import numpy as np
import time
import base64

class VisionDetector(Node):
    def __init__(self):
        super().__init__('vision_node')
        
        # Parametreler
        self.declare_parameter('camera_source', 'ros_topic')  # 'ros_topic' veya 'local'
        self.declare_parameter('local_camera_id', 0)
        self.declare_parameter('cooldown', 4.0)
        self.declare_parameter('min_circle_radius', 15)
        self.declare_parameter('max_circle_radius', 150)
        self.declare_parameter('stop_aspect_threshold', 1.5)
        
        self.camera_source = self.get_parameter('camera_source').value
        self.cooldown = self.get_parameter('cooldown').value
        self.min_r = self.get_parameter('min_circle_radius').value
        self.max_r = self.get_parameter('max_circle_radius').value
        self.stop_aspect = self.get_parameter('stop_aspect_threshold').value
        
        self.current_state = "IDLE"
        self.last_detection_time = 0
        self.frame_count = 0
        self.detection_count = 0
        
        # Publishers & Subscribers
        self.pub_sign = self.create_publisher(SignDetection, '/vision/sign', 10)
        self.create_subscription(String, '/robot_state', self.state_cb, 10)
        
        if self.camera_source == 'ros_topic':
            self.create_subscription(
                CompressedImage, '/camera/compressed', 
                self.image_cb, 10
            )
            self.get_logger().info('Camera source: ROS topic /camera/compressed')
        else:
            # Gerçek robot: yerel kamera
            cam_id = self.get_parameter('local_camera_id').value
            self.cap = cv2.VideoCapture(cam_id)
            self.create_timer(0.2, self.local_camera_loop)
            self.get_logger().info(f'Camera source: local camera {cam_id}')
        
        self.get_logger().info('=== VISION DETECTOR STARTED (Hough Circle) ===')

    def state_cb(self, msg):
        self.current_state = msg.data

    def image_cb(self, msg):
        """ROS topic'ten gelen compressed image'ı işle"""
        if self.current_state == "IDLE":
            return
        
        now = time.time()
        if now - self.last_detection_time < self.cooldown:
            return
        
        try:
            # Base64 → JPEG → OpenCV frame
            jpeg_data = bytes(msg.data)
            np_arr = np.frombuffer(jpeg_data, np.uint8)
            frame = cv2.imdecode(np_arr, cv2.IMREAD_COLOR)
            
            if frame is not None:
                self.frame_count += 1
                self.process_frame(frame)
        except Exception as e:
            self.get_logger().warn(f'Frame decode error: {e}')
    
    def local_camera_loop(self):
        """Yerel kameradan oku (gerçek robot için)"""
        if self.current_state == "IDLE":
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
        """Ana görüntü işleme pipeline'ı"""
        
        # ═══════════════════════════════════════
        # ADIM 1: Kırmızı daire tespiti (etap/stop tabelası)
        # ═══════════════════════════════════════
        result = self.detect_red_circle_sign(frame)
        if result is not None:
            sign_type, confidence = result
            self.publish_detection(sign_type, confidence)
            self.last_detection_time = time.time()
            return
        
        # ═══════════════════════════════════════
        # ADIM 2: Siyah-beyaz daire tespiti (hedef tabelası)
        # ═══════════════════════════════════════
        if self.detect_target_sign(frame):
            self.publish_detection("FINISH", 0.90)
            self.last_detection_time = time.time()
            return

    def detect_red_circle_sign(self, frame):
        """
        Kırmızı daire tespiti: Hough Circle Transform
        
        Neden Hough Circle? → Bariyerler dikdörtgen, tabelalar daire.
        Renk bazlı algılama bariyerlerle karışır, şekil bazlı karışmaz.
        """
        hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
        
        # Kırmızı maske (HSV'de kırmızı 2 aralıkta)
        mask1 = cv2.inRange(hsv, np.array([0, 80, 80]), np.array([10, 255, 255]))
        mask2 = cv2.inRange(hsv, np.array([160, 80, 80]), np.array([180, 255, 255]))
        red_mask = mask1 | mask2
        
        # Morfolojik temizlik
        kernel = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (5, 5))
        red_mask = cv2.morphologyEx(red_mask, cv2.MORPH_CLOSE, kernel, iterations=2)
        red_mask = cv2.morphologyEx(red_mask, cv2.MORPH_OPEN, kernel, iterations=1)
        
        # Hough Circle Transform - DAİRE BUL
        blurred = cv2.GaussianBlur(red_mask, (9, 9), 2)
        circles = cv2.HoughCircles(
            blurred,
            cv2.HOUGH_GRADIENT,
            dp=1.2,
            minDist=50,
            param1=50,
            param2=25,
            minRadius=self.min_r,
            maxRadius=self.max_r
        )
        
        if circles is None:
            return None
        
        circles = np.uint16(np.around(circles))
        
        for circle in circles[0]:
            cx, cy, r = int(circle[0]), int(circle[1]), int(circle[2])
            
            # Güvenlik: frame sınırları içinde mi?
            h, w = frame.shape[:2]
            x1 = max(0, cx - r)
            y1 = max(0, cy - r)
            x2 = min(w, cx + r)
            y2 = min(h, cy + r)
            
            if (x2 - x1) < 20 or (y2 - y1) < 20:
                continue
            
            # Dairenin gerçekten kırmızı halka olduğunu doğrula
            # Halka bölgesindeki kırmızı piksel oranını kontrol et
            circle_mask = np.zeros(red_mask.shape, dtype=np.uint8)
            cv2.circle(circle_mask, (cx, cy), r, 255, -1)
            inner_mask = np.zeros(red_mask.shape, dtype=np.uint8)
            cv2.circle(inner_mask, (cx, cy), int(r * 0.65), 255, -1)
            ring_mask = circle_mask - inner_mask  # Sadece halka bölgesi
            
            ring_pixels = cv2.countNonZero(ring_mask)
            ring_red = cv2.countNonZero(cv2.bitwise_and(red_mask, ring_mask))
            
            if ring_pixels == 0:
                continue
            
            red_ratio = ring_red / ring_pixels
            
            if red_ratio < 0.3:  # Halkanın en az %30'u kırmızı olmalı
                continue
            
            # ═══ DAİRE DOĞRULANDI! İçini analiz et ═══
            
            # İç bölgeyi krop et (dairenin %60'ı)
            inner_r = int(r * 0.6)
            ix1 = max(0, cx - inner_r)
            iy1 = max(0, cy - inner_r)
            ix2 = min(w, cx + inner_r)
            iy2 = min(h, cy + inner_r)
            
            inner_roi = frame[iy1:iy2, ix1:ix2]
            
            if inner_roi.size == 0:
                continue
            
            # İç kısımdaki siyah içeriği bul
            gray_inner = cv2.cvtColor(inner_roi, cv2.COLOR_BGR2GRAY)
            _, black_thresh = cv2.threshold(gray_inner, 80, 255, cv2.THRESH_BINARY_INV)
            
            # Siyah piksellerin bounding box'ını bul
            coords = cv2.findNonZero(black_thresh)
            
            if coords is None or len(coords) < 10:
                continue
            
            bx, by, bw, bh = cv2.boundingRect(coords)
            
            if bh == 0:
                continue
            
            aspect = bw / bh
            black_fill = cv2.countNonZero(black_thresh) / (black_thresh.shape[0] * black_thresh.shape[1])
            
            self.get_logger().info(
                f'Circle found: center=({cx},{cy}), r={r}, '
                f'red_ratio={red_ratio:.2f}, aspect={aspect:.2f}, '
                f'black_fill={black_fill:.2f}'
            )
            
            # STOP vs SAYI ayrımı
            if aspect > self.stop_aspect and black_fill > 0.15:
                # "STOP" yazısı: 4 harf yan yana → geniş
                self.get_logger().warn(f'>>> STOP DETECTED (aspect={aspect:.2f}) <<<')
                return ("STOP", 0.95)
            elif black_fill > 0.05:
                # Tek rakam: dar
                self.get_logger().warn(f'>>> STAGE NUMBER DETECTED (aspect={aspect:.2f}) <<<')
                return ("NEXT_STAGE", 0.90)
        
        return None

    def detect_target_sign(self, frame):
        """
        Hedef tabelası tespiti: Siyah-beyaz iç içe daireler (bullseye)
        Kırmızı yok, siyah kalın halka + beyaz boşluk + siyah merkez
        """
        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        
        # Siyah bölgeleri bul
        _, black_mask = cv2.threshold(gray, 60, 255, cv2.THRESH_BINARY_INV)
        
        kernel = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (5, 5))
        black_mask = cv2.morphologyEx(black_mask, cv2.MORPH_CLOSE, kernel)
        
        blurred = cv2.GaussianBlur(black_mask, (9, 9), 2)
        
        circles = cv2.HoughCircles(
            blurred,
            cv2.HOUGH_GRADIENT,
            dp=1.2,
            minDist=30,
            param1=50,
            param2=25,
            minRadius=self.min_r,
            maxRadius=self.max_r
        )
        
        if circles is None:
            return False
        
        circles = np.uint16(np.around(circles))
        
        # İç içe 2+ daire bulunursa → hedef tabelası
        if len(circles[0]) >= 2:
            # Dairelerin merkezleri birbirine yakın mı?
            centers = [(int(c[0]), int(c[1])) for c in circles[0]]
            
            for i in range(len(centers)):
                for j in range(i + 1, len(centers)):
                    dist = np.sqrt(
                        (centers[i][0] - centers[j][0])**2 + 
                        (centers[i][1] - centers[j][1])**2
                    )
                    # İç içe daireler: merkezler birbirine çok yakın
                    max_r = max(int(circles[0][i][2]), int(circles[0][j][2]))
                    if dist < max_r * 0.3:
                        self.get_logger().warn('>>> TARGET/FINISH DETECTED <<<')
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
            f'[#{self.detection_count}] Published: {sign_type} '
            f'(conf={confidence:.2f}, frame={self.frame_count})'
        )

def main(args=None):
    rclpy.init(args=args)
    node = VisionDetector()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
PYEOF

cd "$WS_DIR"
rm -rf build/ika_vision install/ika_vision
if [ -f /opt/ros/jazzy/setup.bash ]; then source /opt/ros/jazzy/setup.bash; else source /opt/ros/humble/setup.bash; fi
source install/setup.bash 2>/dev/null || true
colcon build --packages-select ika_vision

echo "============================================"
echo "  REAL VISION DETECTOR DEPLOYED!            "
echo "============================================"
