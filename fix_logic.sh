#!/bin/bash
set -e
WS_DIR=~/ika_ws

echo "============================================"
echo "  FIXING STATE MACHINE + DETECTOR           "
echo "============================================"

# ═══════════════════════════════════════════
# 1. STATE MACHINE - Etap sırası zorunlu
# ═══════════════════════════════════════════
cat > "$WS_DIR/src/ika_robot/ika_decision/ika_decision/state_machine_node.py" << 'PYEOF'
"""
TEKNOFEST UGV State Machine

Etap Sırası (diyagramdan):
  0: NORMAL (Su Geçişi / Başlangıç)
  1: TASLI_YOL
  2: YAN_EGIM
  3: DIK_ENGEL
  4: TRAFIK_KONILERI
  5: KAYAR_ENGEL
  6: ENGEBELI_ARAZI
  7: DIK_EGIM_ATIS (son etap)

Kurallar:
  - NEXT_STAGE sadece mevcut etaptan bir sonrakine geçiş yapar
  - STOP sadece son etaptayken kabul edilir → araç durur
  - FINISH sadece son etaptayken kabul edilir → görev biter
  - Etap atlama YASAK (1'deyken 3'e gidemezsin)
"""

import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from ika_interfaces.msg import SignDetection

STAGES = [
    "NORMAL",
    "TASLI_YOL",
    "YAN_EGIM",
    "DIK_ENGEL",
    "TRAFIK_KONILERI",
    "KAYAR_ENGEL",
    "ENGEBELI_ARAZI",
    "DIK_EGIM_ATIS",
]

class StateMachineNode(Node):
    def __init__(self):
        super().__init__('state_machine')
        
        self.state = "IDLE"
        self.stage_index = 0
        self.mission_active = False
        
        self.pub_state = self.create_publisher(String, '/robot_state', 10)
        self.create_subscription(String, '/user_command', self.command_cb, 10)
        self.create_subscription(SignDetection, '/vision/sign', self.sign_cb, 10)
        
        self.create_timer(0.5, self.publish_state)
        
        self.get_logger().info('State Machine Started')
        self.get_logger().info(f'Stages: {" → ".join(STAGES)}')
    
    def command_cb(self, msg):
        cmd = msg.data.strip()
        self.get_logger().info(f'Command received: {cmd}')
        
        if cmd == "START_AUTO_NORMAL":
            self.stage_index = 0
            self.state = STAGES[0]
            self.mission_active = True
            self.get_logger().info(f'=== AUTOPILOT STARTED === Stage: {self.state} (1/{len(STAGES)})')
            
        elif cmd == "START_AUTO_ACCEL":
            self.state = "ACCEL_RUN"
            self.mission_active = True
            self.get_logger().info('=== ACCELERATION RUN STARTED ===')
            
        elif cmd == "STOP":
            self.state = "IDLE"
            self.mission_active = False
            self.stage_index = 0
            self.get_logger().info('=== STOPPED → IDLE ===')
            
        elif cmd == "MANUAL":
            self.state = "MANUAL"
            self.mission_active = False
            self.get_logger().info('=== MANUAL MODE ===')
    
    def sign_cb(self, msg):
        if not self.mission_active:
            return
        
        sign_type = msg.type
        conf = msg.confidence
        
        self.get_logger().info(
            f'Sign received: type={sign_type}, conf={conf:.2f} '
            f'(current stage: {self.state} [{self.stage_index+1}/{len(STAGES)}])'
        )
        
        # ═══ CONFIDENCE CHECK ═══
        if conf < 0.7:
            self.get_logger().info(f'Rejected: confidence too low ({conf:.2f} < 0.70)')
            return
        
        # ═══ NEXT_STAGE: Bir sonraki etaba geç ═══
        if sign_type == "NEXT_STAGE":
            if self.stage_index < len(STAGES) - 1:
                self.stage_index += 1
                self.state = STAGES[self.stage_index]
                self.get_logger().warn(
                    f'>>> STAGE ADVANCED → {self.state} '
                    f'({self.stage_index+1}/{len(STAGES)}) <<<'
                )
            else:
                self.get_logger().info('Already at last stage, NEXT_STAGE ignored')
        
        # ═══ STOP: Sadece son etapta kabul et ═══
        elif sign_type == "STOP":
            if self.stage_index >= len(STAGES) - 1:
                self.state = "IDLE"
                self.mission_active = False
                self.get_logger().warn('>>> STOP AT FINAL STAGE → MISSION COMPLETE <<<')
            else:
                self.get_logger().info(
                    f'STOP sign ignored: not at final stage '
                    f'({self.stage_index+1}/{len(STAGES)})'
                )
        
        # ═══ FINISH: Sadece son etapta kabul et ═══
        elif sign_type == "FINISH":
            if self.stage_index >= len(STAGES) - 1:
                self.state = "IDLE"
                self.mission_active = False
                self.get_logger().warn('>>> FINISH DETECTED → MISSION COMPLETE <<<')
            else:
                self.get_logger().info(
                    f'FINISH sign ignored: not at final stage '
                    f'({self.stage_index+1}/{len(STAGES)})'
                )
    
    def publish_state(self):
        msg = String()
        msg.data = self.state
        self.pub_state.publish(msg)

def main(args=None):
    rclpy.init(args=args)
    node = StateMachineNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
PYEOF

# ═══════════════════════════════════════════
# 2. DETECTOR - Hedef tespiti çok daha sıkı
# ═══════════════════════════════════════════
cat > "$WS_DIR/src/ika_robot/ika_vision/ika_vision/detector_node.py" << 'PYEOF'
"""
TEKNOFEST UGV - Gerçek Görüntü İşleme ile Tabela Algılama

Algılama:
  1. Kırmızı daire → Hough Circle Transform
  2. İç analiz → STOP (geniş siyah) vs SAYI (dar siyah)
  3. Hedef tabelası → sadece son etapta aranır, çok sıkı kontrol
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
        self.declare_parameter('min_circle_radius', 15)
        self.declare_parameter('max_circle_radius', 150)
        self.declare_parameter('stop_aspect_threshold', 1.5)
        self.declare_parameter('ring_red_ratio_min', 0.35)
        
        self.camera_source = self.get_parameter('camera_source').value
        self.cooldown = self.get_parameter('cooldown').value
        self.min_r = self.get_parameter('min_circle_radius').value
        self.max_r = self.get_parameter('max_circle_radius').value
        self.stop_aspect = self.get_parameter('stop_aspect_threshold').value
        self.ring_min = self.get_parameter('ring_red_ratio_min').value
        
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
        
        self.get_logger().info('=== VISION DETECTOR STARTED (Hough Circle) ===')

    def state_cb(self, msg):
        self.current_state = msg.data

    def image_cb(self, msg):
        if self.current_state == "IDLE" or self.current_state == "MANUAL":
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
            self.get_logger().warn(f'Frame decode error: {e}')
    
    def local_camera_loop(self):
        if self.current_state == "IDLE" or self.current_state == "MANUAL":
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
        # Sadece kırmızı daire tespiti (etap numarası + STOP)
        # Hedef tabelası SADECE son etapta aranır
        result = self.detect_red_circle_sign(frame)
        if result is not None:
            sign_type, confidence = result
            self.publish_detection(sign_type, confidence)
            self.last_detection_time = time.time()
            return
        
        # Hedef tabelası: sadece son etapta (DIK_EGIM_ATIS)
        if self.current_state == "DIK_EGIM_ATIS":
            if self.detect_target_sign(frame):
                self.publish_detection("FINISH", 0.90)
                self.last_detection_time = time.time()

    def detect_red_circle_sign(self, frame):
        hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
        
        # Kırmızı maske
        mask1 = cv2.inRange(hsv, np.array([0, 80, 80]), np.array([10, 255, 255]))
        mask2 = cv2.inRange(hsv, np.array([160, 80, 80]), np.array([180, 255, 255]))
        red_mask = mask1 | mask2
        
        # Morfolojik temizlik
        kernel = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (5, 5))
        red_mask = cv2.morphologyEx(red_mask, cv2.MORPH_CLOSE, kernel, iterations=2)
        red_mask = cv2.morphologyEx(red_mask, cv2.MORPH_OPEN, kernel, iterations=1)
        
        # Toplam kırmızı piksel kontrolü (minimum eşik - çok az kırmızıysa atla)
        total_red = cv2.countNonZero(red_mask)
        total_pixels = frame.shape[0] * frame.shape[1]
        red_percent = (total_red / total_pixels) * 100
        
        if red_percent < 0.5:
            return None  # Kırmızı neredeyse yok, daire aramaya gerek yok
        
        # Hough Circle Transform
        blurred = cv2.GaussianBlur(red_mask, (9, 9), 2)
        circles = cv2.HoughCircles(
            blurred,
            cv2.HOUGH_GRADIENT,
            dp=1.2,
            minDist=50,
            param1=50,
            param2=30,  # Biraz daha sıkı (25→30)
            minRadius=self.min_r,
            maxRadius=self.max_r
        )
        
        if circles is None:
            return None
        
        circles = np.uint16(np.around(circles))
        h, w = frame.shape[:2]
        
        for circle in circles[0]:
            cx, cy, r = int(circle[0]), int(circle[1]), int(circle[2])
            
            # Frame sınırları
            if cx - r < 0 or cy - r < 0 or cx + r >= w or cy + r >= h:
                continue
            
            if r < 15:
                continue
            
            # Halka doğrulama: halka bölgesinin kırmızı oranı
            circle_mask = np.zeros(red_mask.shape, dtype=np.uint8)
            cv2.circle(circle_mask, (cx, cy), r, 255, -1)
            inner_mask = np.zeros(red_mask.shape, dtype=np.uint8)
            cv2.circle(inner_mask, (cx, cy), int(r * 0.65), 255, -1)
            ring_mask = circle_mask - inner_mask
            
            ring_pixels = cv2.countNonZero(ring_mask)
            ring_red = cv2.countNonZero(cv2.bitwise_and(red_mask, ring_mask))
            
            if ring_pixels == 0:
                continue
            
            red_ratio = ring_red / ring_pixels
            
            if red_ratio < self.ring_min:
                continue  # Yeterince kırmızı halka değil
            
            # İÇ BEYAZLIK kontrolü: iç kısım ağırlıklı beyaz olmalı
            inner_r = int(r * 0.6)
            ix1, iy1 = max(0, cx - inner_r), max(0, cy - inner_r)
            ix2, iy2 = min(w, cx + inner_r), min(h, cy + inner_r)
            
            inner_roi = frame[iy1:iy2, ix1:ix2]
            if inner_roi.size == 0:
                continue
            
            # İç kısımdaki beyaz oranı kontrol et
            hsv_inner = cv2.cvtColor(inner_roi, cv2.COLOR_BGR2HSV)
            white_mask = cv2.inRange(hsv_inner, np.array([0, 0, 180]), np.array([180, 50, 255]))
            white_ratio = cv2.countNonZero(white_mask) / (white_mask.shape[0] * white_mask.shape[1])
            
            if white_ratio < 0.25:
                continue  # İç kısım yeterince beyaz değil → tabela değil
            
            # Siyah içerik analizi
            gray_inner = cv2.cvtColor(inner_roi, cv2.COLOR_BGR2GRAY)
            _, black_thresh = cv2.threshold(gray_inner, 80, 255, cv2.THRESH_BINARY_INV)
            
            coords = cv2.findNonZero(black_thresh)
            if coords is None or len(coords) < 10:
                continue
            
            bx, by, bw, bh = cv2.boundingRect(coords)
            if bh == 0:
                continue
            
            aspect = bw / bh
            black_fill = cv2.countNonZero(black_thresh) / (black_thresh.shape[0] * black_thresh.shape[1])
            
            self.get_logger().info(
                f'RED CIRCLE: center=({cx},{cy}) r={r} '
                f'ring_red={red_ratio:.2f} white={white_ratio:.2f} '
                f'aspect={aspect:.2f} black={black_fill:.2f}'
            )
            
            if black_fill < 0.05:
                continue  # Çok az siyah → yazı yok → tabela değil
            
            if aspect > self.stop_aspect and black_fill > 0.12:
                self.get_logger().warn(f'>>> STOP DETECTED (aspect={aspect:.2f}) <<<')
                return ("STOP", 0.95)
            else:
                self.get_logger().warn(f'>>> STAGE SIGN DETECTED (aspect={aspect:.2f}) <<<')
                return ("NEXT_STAGE", 0.90)
        
        return None

    def detect_target_sign(self, frame):
        """
        Hedef tabelası: siyah-beyaz iç içe daireler
        SADECE son etapta çağrılır (state machine'de de kontrol var)
        Çok sıkı kriterler - false positive önleme
        """
        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        
        # Çok keskin siyah-beyaz kontrast gerektir
        _, black_mask = cv2.threshold(gray, 50, 255, cv2.THRESH_BINARY_INV)
        
        kernel = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (7, 7))
        black_mask = cv2.morphologyEx(black_mask, cv2.MORPH_CLOSE, kernel)
        black_mask = cv2.morphologyEx(black_mask, cv2.MORPH_OPEN, kernel)
        
        blurred = cv2.GaussianBlur(black_mask, (9, 9), 2)
        
        circles = cv2.HoughCircles(
            blurred, cv2.HOUGH_GRADIENT,
            dp=1.2, minDist=20,
            param1=50, param2=35,  # Sıkı
            minRadius=20, maxRadius=self.max_r
        )
        
        if circles is None:
            return False
        
        circles = np.uint16(np.around(circles))
        
        if len(circles[0]) < 2:
            return False
        
        # İç içe daire kontrolü: merkezleri birbirine çok yakın olan 2+ daire
        centers = [(int(c[0]), int(c[1]), int(c[2])) for c in circles[0]]
        
        for i in range(len(centers)):
            for j in range(i + 1, len(centers)):
                dist = np.sqrt(
                    (centers[i][0] - centers[j][0])**2 + 
                    (centers[i][1] - centers[j][1])**2
                )
                # Yarıçaplar farklı olmalı (iç içe)
                r_diff = abs(centers[i][2] - centers[j][2])
                max_r = max(centers[i][2], centers[j][2])
                
                if dist < max_r * 0.2 and r_diff > max_r * 0.3:
                    # Merkezler yakın VE yarıçaplar farklı → iç içe
                    
                    # Ek kontrol: kırmızı olmamalı (hedef tabelasında kırmızı yok)
                    hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
                    cx, cy = centers[i][0], centers[i][1]
                    roi_r = max_r + 10
                    h, w = frame.shape[:2]
                    rx1, ry1 = max(0, cx - roi_r), max(0, cy - roi_r)
                    rx2, ry2 = min(w, cx + roi_r), min(h, cy + roi_r)
                    
                    hsv_roi = hsv[ry1:ry2, rx1:rx2]
                    if hsv_roi.size == 0:
                        continue
                    
                    red1 = cv2.inRange(hsv_roi, np.array([0, 80, 80]), np.array([10, 255, 255]))
                    red2 = cv2.inRange(hsv_roi, np.array([160, 80, 80]), np.array([180, 255, 255]))
                    red_pct = (cv2.countNonZero(red1 | red2) / (hsv_roi.shape[0] * hsv_roi.shape[1])) * 100
                    
                    if red_pct > 5:
                        continue  # Kırmızı var → etap tabelası, hedef değil
                    
                    self.get_logger().warn(
                        f'>>> TARGET DETECTED: concentric circles '
                        f'at ({cx},{cy}), radii={centers[i][2]},{centers[j][2]} <<<'
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

# ═══════════════════════════════════════════
# BUILD
# ═══════════════════════════════════════════
cd "$WS_DIR"
rm -rf build/ika_decision install/ika_decision build/ika_vision install/ika_vision
if [ -f /opt/ros/jazzy/setup.bash ]; then source /opt/ros/jazzy/setup.bash; else source /opt/ros/humble/setup.bash; fi
source install/setup.bash 2>/dev/null || true
colcon build --packages-select ika_interfaces
source install/setup.bash
colcon build --packages-select ika_decision ika_vision

echo "============================================"
echo "  STATE MACHINE + DETECTOR FIXED!           "
echo "============================================"
