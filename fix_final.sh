#!/bin/bash
set -e
WS_DIR=~/ika_ws

echo "============================================"
echo "  DETECTOR v5 (Contour) + NAVIGATION NODE   "
echo "============================================"

# ═══════════════════════════════════════
# 1. DETECTOR v5: Kontur tabanlı algılama
# ═══════════════════════════════════════
cat > "$WS_DIR/src/ika_robot/ika_vision/ika_vision/detector_node.py" << 'PYEOF'
"""
TEKNOFEST UGV - Tabela Algılama v5 (Contour)

Hough Circle yerine KONTUR + DAİRESELLİK kullanıyoruz.
Neden? Bariyerler kırmızı dikdörtgen → daireselliği düşük.
        Tabelalar kırmızı daire → daireselliği yüksek.

Algoritma:
  1. HSV → kırmızı maske
  2. Kontur bul
  3. Her kontur için dairesellik hesapla: 4π×Alan/(Çevre²)
     Mükemmel daire = 1.0, dikdörtgen ≈ 0.78, düzensiz < 0.5
  4. Dairesellik > 0.6 ve alan yeterli → tabela adayı
  5. İç analiz: beyazlık + siyah içerik aspect ratio
"""

import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from ika_interfaces.msg import SignDetection
import cv2
import numpy as np
import time
import base64
import math

class VisionDetector(Node):
    def __init__(self):
        super().__init__('vision_node')
        
        self.declare_parameter('camera_source', 'ros_topic')
        self.declare_parameter('local_camera_id', 0)
        self.declare_parameter('cooldown', 3.0)
        self.declare_parameter('min_circularity', 0.55)
        self.declare_parameter('min_area', 400)
        self.declare_parameter('max_area', 50000)
        
        self.camera_source = self.get_parameter('camera_source').value
        self.cooldown = self.get_parameter('cooldown').value
        self.min_circ = self.get_parameter('min_circularity').value
        self.min_area = self.get_parameter('min_area').value
        self.max_area = self.get_parameter('max_area').value
        
        self.current_state = "IDLE"
        self.last_detection_time = 0
        self.frame_count = 0
        self.detection_count = 0
        
        self.pub_sign = self.create_publisher(SignDetection, '/vision/sign', 10)
        self.create_subscription(String, '/robot_state', self.state_cb, 10)
        
        if self.camera_source == 'ros_topic':
            self.create_subscription(String, '/camera/image_base64', self.image_cb, 10)
            self.get_logger().info('Camera: /camera/image_base64')
        else:
            cam_id = self.get_parameter('local_camera_id').value
            self.cap = cv2.VideoCapture(cam_id)
            self.create_timer(0.2, self.local_camera_loop)
            self.get_logger().info(f'Camera: /dev/video{cam_id}')
        
        self.create_timer(10.0, self.print_stats)
        self.get_logger().info('=== VISION DETECTOR v5 (Contour circularity) ===')

    def state_cb(self, msg):
        old = self.current_state
        self.current_state = msg.data
        if old != self.current_state:
            self.get_logger().info(f'State: {old} → {self.current_state}')

    def image_cb(self, msg):
        if self.current_state in ["IDLE", "MANUAL"]:
            return
        now = time.time()
        if now - self.last_detection_time < self.cooldown:
            return
        try:
            jpeg_bytes = base64.b64decode(msg.data)
            np_arr = np.frombuffer(jpeg_bytes, np.uint8)
            frame = cv2.imdecode(np_arr, cv2.IMREAD_COLOR)
            if frame is not None:
                self.frame_count += 1
                if self.frame_count <= 3:
                    h, w = frame.shape[:2]
                    self.get_logger().info(f'Frame #{self.frame_count}: {w}x{h} OK')
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

    def print_stats(self):
        self.get_logger().info(
            f'Stats: state={self.current_state}, '
            f'frames={self.frame_count}, detections={self.detection_count}'
        )

    def process_frame(self, frame):
        result = self.detect_sign_contour(frame)
        if result is not None:
            sign_type, confidence = result
            self.publish_detection(sign_type, confidence)
            self.last_detection_time = time.time()
            return
        
        if self.current_state == "PLATFORM_ATIS":
            if self.detect_target_sign(frame):
                self.publish_detection("FINISH", 0.90)
                self.last_detection_time = time.time()

    def detect_sign_contour(self, frame):
        """Kontur tabanlı kırmızı daire tespiti"""
        hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
        
        # Kırmızı maske
        m1 = cv2.inRange(hsv, np.array([0, 70, 70]), np.array([12, 255, 255]))
        m2 = cv2.inRange(hsv, np.array([160, 70, 70]), np.array([180, 255, 255]))
        red_mask = m1 | m2
        
        # Morfolojik temizlik
        k = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (5, 5))
        red_mask = cv2.morphologyEx(red_mask, cv2.MORPH_CLOSE, k, iterations=2)
        red_mask = cv2.morphologyEx(red_mask, cv2.MORPH_OPEN, k, iterations=1)
        
        # Konturları bul
        contours, _ = cv2.findContours(red_mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        
        h, w = frame.shape[:2]
        
        for cnt in contours:
            area = cv2.contourArea(cnt)
            if area < self.min_area or area > self.max_area:
                continue
            
            perimeter = cv2.arcLength(cnt, True)
            if perimeter == 0:
                continue
            
            # DAİRESELLİK: 4π×Alan/(Çevre²) — daire=1.0, dikdörtgen≈0.78
            circularity = (4 * math.pi * area) / (perimeter * perimeter)
            
            if circularity < self.min_circ:
                continue  # Daire değil (bariyer dikdörtgen)
            
            # Bounding circle
            (cx, cy), radius = cv2.minEnclosingCircle(cnt)
            cx, cy, r = int(cx), int(cy), int(radius)
            
            if r < 10:
                continue
            
            # İç analiz: dairenin merkezindeki bölge
            ir = int(r * 0.6)
            ix1, iy1 = max(0, cx-ir), max(0, cy-ir)
            ix2, iy2 = min(w, cx+ir), min(h, cy+ir)
            roi = frame[iy1:iy2, ix1:ix2]
            
            if roi.size == 0 or roi.shape[0] < 5 or roi.shape[1] < 5:
                continue
            
            # İç beyazlık kontrolü
            hsv_roi = cv2.cvtColor(roi, cv2.COLOR_BGR2HSV)
            white = cv2.inRange(hsv_roi, np.array([0, 0, 160]), np.array([180, 60, 255]))
            wr = cv2.countNonZero(white) / (white.shape[0] * white.shape[1])
            
            if wr < 0.15:
                continue  # İç kısım beyaz değil
            
            # Siyah içerik
            gray = cv2.cvtColor(roi, cv2.COLOR_BGR2GRAY)
            _, bw = cv2.threshold(gray, 80, 255, cv2.THRESH_BINARY_INV)
            coords = cv2.findNonZero(bw)
            if coords is None or len(coords) < 5:
                continue
            
            bx, by, bw2, bh2 = cv2.boundingRect(coords)
            if bh2 == 0:
                continue
            
            aspect = bw2 / bh2
            bf = cv2.countNonZero(bw) / (bw.shape[0] * bw.shape[1])
            
            self.get_logger().info(
                f'SIGN CANDIDATE: ({cx},{cy}) r={r} '
                f'circ={circularity:.2f} white={wr:.2f} '
                f'aspect={aspect:.2f} black={bf:.2f}'
            )
            
            if bf < 0.03:
                continue  # Çok az siyah
            
            # STOP vs NUMARA ayrımı
            if aspect > 1.5 and bf > 0.10:
                self.get_logger().warn(f'>>> STOP (a={aspect:.1f} c={circularity:.2f}) <<<')
                return ("STOP", 0.95)
            else:
                self.get_logger().warn(f'>>> NUMBER (a={aspect:.1f} c={circularity:.2f}) <<<')
                return ("NEXT_STAGE", 0.90)
        
        return None

    def detect_target_sign(self, frame):
        """Hedef: siyah-beyaz bullseye (sadece PLATFORM_ATIS)"""
        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        _, bmask = cv2.threshold(gray, 50, 255, cv2.THRESH_BINARY_INV)
        k = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (7, 7))
        bmask = cv2.morphologyEx(bmask, cv2.MORPH_CLOSE, k)
        bmask = cv2.morphologyEx(bmask, cv2.MORPH_OPEN, k)
        
        contours, _ = cv2.findContours(bmask, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
        
        circles = []
        for cnt in contours:
            area = cv2.contourArea(cnt)
            if area < 500:
                continue
            peri = cv2.arcLength(cnt, True)
            if peri == 0:
                continue
            circ = (4 * math.pi * area) / (peri * peri)
            if circ > 0.65:
                (cx, cy), r = cv2.minEnclosingCircle(cnt)
                circles.append((int(cx), int(cy), int(r)))
        
        # İç içe daireler: merkezler yakın, yarıçaplar farklı
        for i in range(len(circles)):
            for j in range(i+1, len(circles)):
                d = np.sqrt((circles[i][0]-circles[j][0])**2 + (circles[i][1]-circles[j][1])**2)
                mr = max(circles[i][2], circles[j][2])
                rd = abs(circles[i][2] - circles[j][2])
                if d < mr * 0.3 and rd > mr * 0.25:
                    # Kırmızı olmamalı
                    cx, cy = circles[i][0], circles[i][1]
                    rr = mr + 5
                    fh, fw = frame.shape[:2]
                    hr = cv2.cvtColor(frame[max(0,cy-rr):min(fh,cy+rr), max(0,cx-rr):min(fw,cx+rr)], cv2.COLOR_BGR2HSV)
                    if hr.size == 0: continue
                    r1 = cv2.inRange(hr, np.array([0,80,80]), np.array([10,255,255]))
                    r2 = cv2.inRange(hr, np.array([160,80,80]), np.array([180,255,255]))
                    rp = (cv2.countNonZero(r1|r2)/(hr.shape[0]*hr.shape[1]))*100
                    if rp > 5: continue
                    self.get_logger().warn(f'>>> TARGET ({cx},{cy}) <<<')
                    return True
        return False

    def publish_detection(self, sign_type, confidence):
        self.detection_count += 1
        msg = SignDetection()
        msg.type = sign_type
        msg.distance = 2.0
        msg.confidence = confidence
        self.pub_sign.publish(msg)
        self.get_logger().info(f'[#{self.detection_count}] {sign_type} ({confidence:.2f})')

def main(args=None):
    rclpy.init(args=args)
    node = VisionDetector()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
PYEOF

# ═══════════════════════════════════════
# 2. NAVIGATION NODE: Temel sürüş
# ═══════════════════════════════════════
cat > "$WS_DIR/src/ika_robot/ika_control/ika_control/controller_node.py" << 'PYEOF'
"""
TEKNOFEST UGV - Temel Navigasyon Kontrolü

State'e göre araç hareketi:
  IDLE/MANUAL → dur (cmd_vel gönderme)
  BASLA...ENGEBELI_ARAZI → ileri git, PID merkezleme (ultrasonik)
  DIK_EGIM_CIKIS → yavaş ileri
  CIKIS_DURMA → dur
  PLATFORM_ATIS → dur (lazer hizalama)
  DIK_EGIM_INIS → yavaş ileri
  INIS_DURMA → dur
"""

import rclpy
from rclpy.node import Node
from std_msgs.msg import String, Float32MultiArray
from geometry_msgs.msg import Twist
import time

# Her state için hız ayarları
STATE_SPEEDS = {
    "BASLA": 0.5,
    "TASLI_YOL": 0.4,
    "YAN_EGIM": 0.35,
    "DIK_ENGEL": 0.3,
    "TRAFIK_KONILERI": 0.25,
    "KAYAR_ENGEL": 0.3,
    "ENGEBELI_ARAZI": 0.35,
    "DIK_EGIM_CIKIS": 0.3,
    "DIK_EGIM_INIS": 0.25,
}

# Durması gereken state'ler
STOP_STATES = ["IDLE", "MANUAL", "CIKIS_DURMA", "PLATFORM_ATIS", "INIS_DURMA"]

class ControllerNode(Node):
    def __init__(self):
        super().__init__('controller_node')
        
        self.state = "IDLE"
        
        # Ultrasonik mesafeler [FC, FL, FR, CFL, CFR, CRL, CRR]
        self.us_data = [4.0] * 7
        
        # PID merkezleme
        self.kp = 1.5
        self.kd = 0.3
        self.prev_error = 0.0
        
        self.pub_cmd = self.create_publisher(Twist, '/cmd_vel', 10)
        self.create_subscription(String, '/robot_state', self.state_cb, 10)
        self.create_subscription(Float32MultiArray, '/sensors/ultrasonic', self.us_cb, 10)
        
        self.create_timer(0.1, self.control_loop)  # 10 Hz
        
        self.get_logger().info('Navigation Controller Started')
    
    def state_cb(self, msg):
        old = self.state
        self.state = msg.data
        if old != self.state:
            self.get_logger().info(f'State: {old} → {self.state}')
    
    def us_cb(self, msg):
        if msg.data and len(msg.data) >= 7:
            self.us_data = list(msg.data)
    
    def control_loop(self):
        cmd = Twist()
        
        if self.state in STOP_STATES:
            # Dur
            cmd.linear.x = 0.0
            cmd.angular.z = 0.0
        elif self.state in STATE_SPEEDS:
            speed = STATE_SPEEDS[self.state]
            
            # PID merkezleme: sol ve sağ ultrasonik mesafe farkı
            # us_data: [FC, FL, FR, CFL, CFR, CRL, CRR]
            left_dist = self.us_data[1]   # FL
            right_dist = self.us_data[2]  # FR
            
            # Hata: sağ-sol (pozitif = sağda daha uzak = sola kaymış)
            error = right_dist - left_dist
            
            # PD kontrolü
            derivative = error - self.prev_error
            steering = self.kp * error + self.kd * derivative
            self.prev_error = error
            
            # Sınırla
            steering = max(-0.5, min(0.5, steering))
            
            # Ön engel kontrolü
            front_dist = self.us_data[0]  # FC
            if front_dist < 0.3:
                speed = 0.0
                steering = 0.0
                self.get_logger().warn(f'FRONT OBSTACLE: {front_dist:.2f}m')
            elif front_dist < 0.8:
                speed *= 0.5
            
            cmd.linear.x = speed
            cmd.angular.z = steering
        else:
            # Bilinmeyen state → dur (güvenlik)
            cmd.linear.x = 0.0
            cmd.angular.z = 0.0
        
        self.pub_cmd.publish(cmd)

def main(args=None):
    rclpy.init(args=args)
    node = ControllerNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
PYEOF

# ═══════════════════════════════════════
# 3. LAUNCH FILE - controller ekle
# ═══════════════════════════════════════
cat > "$WS_DIR/src/ika_robot/ika_bringup/launch/system.launch.py" << 'PYEOF'
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(package='ika_decision', executable='state_machine', name='state_machine'),
        Node(package='ika_vision', executable='detector', name='detector',
             parameters=[{'camera_source': 'ros_topic'}]),
        Node(package='ika_control', executable='controller', name='controller'),
        Node(package='ika_control', executable='acceleration', name='acceleration'),
        Node(package='ika_web_bridge', executable='web_bridge', name='web_bridge'),
        Node(package='rosbridge_server', executable='rosbridge_websocket',
             name='rosbridge_websocket'),
    ])
PYEOF

# ═══════════════════════════════════════
# 4. Controller executable ekle
# ═══════════════════════════════════════
SETUP_FILE="$WS_DIR/src/ika_robot/ika_control/setup.py"
if ! grep -q "controller" "$SETUP_FILE"; then
    sed -i "s/'acceleration = ika_control.acceleration_node:main'/'acceleration = ika_control.acceleration_node:main',\n            'controller = ika_control.controller_node:main'/" "$SETUP_FILE"
fi

# BUILD
cd "$WS_DIR"
rm -rf build/ika_vision install/ika_vision build/ika_control install/ika_control build/ika_bringup install/ika_bringup
if [ -f /opt/ros/jazzy/setup.bash ]; then source /opt/ros/jazzy/setup.bash; else source /opt/ros/humble/setup.bash; fi
source install/setup.bash 2>/dev/null || true
colcon build --packages-select ika_interfaces
source install/setup.bash
colcon build --packages-select ika_vision ika_control ika_bringup

echo "============================================"
echo "  DETECTOR v5 + NAVIGATION CONTROLLER DONE! "
echo "============================================"
