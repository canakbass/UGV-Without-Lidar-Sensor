#!/bin/bash
set -e
WS_DIR=~/ika_ws

echo "============================================"
echo "  COMPLETE SYSTEM FIX v2                    "
echo "============================================"

# ═══════════════════════════════════════════════
# 1. STATE MACHINE - Tam etap mantığı
# ═══════════════════════════════════════════════
cat > "$WS_DIR/src/ika_robot/ika_decision/ika_decision/state_machine_node.py" << 'PYEOF'
"""
TEKNOFEST 2026 İKA - State Machine

Parkur Akışı:
  Tabela 1 → BASLA (PID merkezleme)
  Tabela 2 → TASLI_YOL
  Tabela 3 → YAN_EGIM
  Tabela 4 → DIK_ENGEL
  Tabela 5 → TRAFIK_KONILERI
  Tabela 6 → KAYAR_ENGEL
  Tabela 7 → ENGEBELI_ARAZI
  Tabela 8 → DIK_EGIM_CIKIS (yukarı eğim)
  STOP     → CIKIS_DURMA (eğimde dur - IMU doğrula)
  Tabela 9 → PLATFORM_ATIS (hedefe lazer)
  Tabela 10→ DIK_EGIM_INIS (aşağı eğim)
  STOP     → INIS_DURMA (eğimde dur - IMU doğrula)
  5 saniye → MISSION_COMPLETE → IDLE

STOP Kuralı:
  - Etap 1-7: STOP yok sayılır (false positive riski)
  - DIK_EGIM_CIKIS: STOP kabul → CIKIS_DURMA
  - DIK_EGIM_INIS:  STOP kabul → INIS_DURMA
  - Diğer: yok sayılır

FINISH (Hedef) Kuralı:
  - Sadece PLATFORM_ATIS'ta kabul edilir
  - Hedef tespit → lazer hizalama süreci başlar
"""

import rclpy
from rclpy.node import Node
from std_msgs.msg import String, Float32MultiArray
from geometry_msgs.msg import Twist
from ika_interfaces.msg import SignDetection
import time

# Sıralı etap listesi - NEXT_STAGE her seferinde bir sonrakine geçer
STAGES = [
    "BASLA",              # Tabela 1
    "TASLI_YOL",          # Tabela 2
    "YAN_EGIM",           # Tabela 3
    "DIK_ENGEL",          # Tabela 4
    "TRAFIK_KONILERI",    # Tabela 5
    "KAYAR_ENGEL",        # Tabela 6
    "ENGEBELI_ARAZI",     # Tabela 7
    "DIK_EGIM_CIKIS",    # Tabela 8
    # Buradan sonrası STOP/FINISH ile yönetilir
]

# STOP kabul edilen state'ler
STOP_ALLOWED = ["DIK_EGIM_CIKIS", "DIK_EGIM_INIS"]

# STOP sonrası geçiş haritası
STOP_TRANSITION = {
    "DIK_EGIM_CIKIS": "CIKIS_DURMA",
    "DIK_EGIM_INIS": "INIS_DURMA",
}

# NEXT_STAGE tabela gördüğünde geçiş haritası (son bölüm için)
SIGN_TRANSITION = {
    "CIKIS_DURMA": "PLATFORM_ATIS",     # Tabela 9
    "PLATFORM_ATIS": "DIK_EGIM_INIS",   # Tabela 10
}

class StateMachineNode(Node):
    def __init__(self):
        super().__init__('state_machine')
        
        self.state = "IDLE"
        self.stage_index = 0
        self.mission_active = False
        self.stop_time = None  # INIS_DURMA'da durma zamanı
        
        # IMU verileri (eğim kontrolü için)
        self.pitch = 0.0
        self.roll = 0.0
        
        # Publishers
        self.pub_state = self.create_publisher(String, '/robot_state', 10)
        self.pub_cmd = self.create_publisher(Twist, '/cmd_vel', 10)
        
        # Subscribers
        self.create_subscription(String, '/user_command', self.command_cb, 10)
        self.create_subscription(SignDetection, '/vision/sign', self.sign_cb, 10)
        self.create_subscription(Float32MultiArray, '/unity/telemetry', self.telemetry_cb, 10)
        
        self.create_timer(0.5, self.tick)
        
        self.get_logger().info('State Machine v2 Started')
        self.get_logger().info(f'Normal stages: {" → ".join(STAGES)}')
        self.get_logger().info(f'Last section: ...→ CIKIS_DURMA → PLATFORM_ATIS → DIK_EGIM_INIS → INIS_DURMA → END')
    
    def telemetry_cb(self, msg):
        """Unity telemetrisinden pitch/roll al"""
        if msg.data and len(msg.data) >= 7:
            # [posX, posY, posZ, speed, yaw, pitch, roll, ...]
            self.pitch = msg.data[5]
            self.roll = msg.data[6]
    
    def command_cb(self, msg):
        cmd = msg.data.strip()
        self.get_logger().info(f'Command: {cmd}')
        
        if cmd == "START_AUTO_NORMAL":
            self.stage_index = 0
            self.state = STAGES[0]
            self.mission_active = True
            self.stop_time = None
            self.get_logger().info(f'=== AUTOPILOT STARTED === Stage: {self.state}')
            
        elif cmd == "START_AUTO_ACCEL":
            self.state = "ACCEL_RUN"
            self.mission_active = True
            self.get_logger().info('=== ACCELERATION RUN ===')
            
        elif cmd == "STOP" or cmd == "MANUAL":
            self.state = "IDLE" if cmd == "STOP" else "MANUAL"
            self.mission_active = False
            self.stage_index = 0
            self.stop_time = None
            self.get_logger().info(f'=== {cmd} → {self.state} ===')
    
    def sign_cb(self, msg):
        if not self.mission_active:
            return
        
        sign_type = msg.type
        conf = msg.confidence
        
        self.get_logger().info(
            f'Sign: type={sign_type}, conf={conf:.2f}, '
            f'state={self.state}, pitch={self.pitch:.1f}°'
        )
        
        if conf < 0.7:
            self.get_logger().info(f'Rejected: low confidence')
            return
        
        # ═══════════ NEXT_STAGE (numara tabelası) ═══════════
        if sign_type == "NEXT_STAGE":
            self.handle_next_stage()
        
        # ═══════════ STOP tabelası ═══════════
        elif sign_type == "STOP":
            self.handle_stop()
        
        # ═══════════ FINISH (hedef tabelası) ═══════════
        elif sign_type == "FINISH":
            self.handle_finish()
    
    def handle_next_stage(self):
        """Numara tabelası görüldü → bir sonraki etaba geç"""
        
        # Normal etaplar arasında geçiş (STAGES listesinde)
        if self.state in [s for s in STAGES]:
            idx = STAGES.index(self.state) if self.state in STAGES else -1
            if idx >= 0 and idx < len(STAGES) - 1:
                self.stage_index = idx + 1
                self.state = STAGES[self.stage_index]
                self.get_logger().warn(
                    f'>>> STAGE → {self.state} ({self.stage_index+1}/{len(STAGES)}) <<<'
                )
                return
        
        # Son bölüm geçişleri (STOP sonrası tabela)
        if self.state in SIGN_TRANSITION:
            new_state = SIGN_TRANSITION[self.state]
            self.get_logger().warn(f'>>> {self.state} → {new_state} <<<')
            self.state = new_state
            return
        
        self.get_logger().info(f'NEXT_STAGE ignored in state {self.state}')
    
    def handle_stop(self):
        """STOP tabelası görüldü"""
        
        if self.state in STOP_ALLOWED:
            new_state = STOP_TRANSITION[self.state]
            self.get_logger().warn(
                f'>>> STOP ACCEPTED: {self.state} → {new_state} '
                f'(pitch={self.pitch:.1f}°) <<<'
            )
            self.state = new_state
            
            # Hemen dur
            self.send_stop_cmd()
            
            # INIS_DURMA ise 5 saniye sonra görev biter
            if new_state == "INIS_DURMA":
                self.stop_time = time.time()
                self.get_logger().warn('>>> FINAL STOP - Mission ending in 5s <<<')
        else:
            self.get_logger().info(
                f'STOP ignored in state {self.state} '
                f'(only allowed in: {STOP_ALLOWED})'
            )
    
    def handle_finish(self):
        """Hedef tabelası görüldü → sadece PLATFORM_ATIS'ta"""
        
        if self.state == "PLATFORM_ATIS":
            self.get_logger().warn('>>> TARGET DETECTED on platform! Laser aiming... <<<')
            # TODO: Lazer hizalama mantığı
        else:
            self.get_logger().info(
                f'FINISH/TARGET ignored in state {self.state} '
                f'(only allowed in PLATFORM_ATIS)'
            )
    
    def send_stop_cmd(self):
        """Aracı durdur"""
        msg = Twist()
        msg.linear.x = 0.0
        msg.angular.z = 0.0
        self.pub_cmd.publish(msg)
    
    def tick(self):
        """Ana döngü - state yayınla + INIS_DURMA zamanlayıcı"""
        
        # INIS_DURMA'da 5 saniye sonra görev biter
        if self.state == "INIS_DURMA" and self.stop_time is not None:
            elapsed = time.time() - self.stop_time
            if elapsed > 5.0:
                self.state = "IDLE"
                self.mission_active = False
                self.stop_time = None
                self.get_logger().warn('>>> MISSION COMPLETE! <<<')
        
        # CIKIS_DURMA ve INIS_DURMA'da sürekli dur komutu gönder
        if self.state in ["CIKIS_DURMA", "INIS_DURMA"]:
            self.send_stop_cmd()
        
        # State yayınla
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

# ═══════════════════════════════════════════════
# 2. DETECTOR - Hedef tespiti düzeltildi
# ═══════════════════════════════════════════════
cat > "$WS_DIR/src/ika_robot/ika_vision/ika_vision/detector_node.py" << 'PYEOF'
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
PYEOF

# ═══════════════════════════════════════════════
# 3. LAUNCH FILE - navigation ekle, mock kaldır
# ═══════════════════════════════════════════════
cat > "$WS_DIR/src/ika_robot/ika_bringup/launch/system.launch.py" << 'PYEOF'
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        # State Machine
        Node(
            package='ika_decision',
            executable='state_machine',
            name='state_machine'
        ),
        # Vision Detector
        Node(
            package='ika_vision',
            executable='detector',
            name='detector',
            parameters=[{'camera_source': 'ros_topic'}]
        ),
        # Acceleration
        Node(
            package='ika_control',
            executable='acceleration',
            name='acceleration'
        ),
        # Web Bridge
        Node(
            package='ika_web_bridge',
            executable='web_bridge',
            name='web_bridge'
        ),
        # Rosbridge WebSocket
        Node(
            package='rosbridge_server',
            executable='rosbridge_websocket',
            name='rosbridge_websocket'
        ),
    ])
PYEOF

# ═══════════════════════════════════════════════
# BUILD
# ═══════════════════════════════════════════════
cd "$WS_DIR"
rm -rf build/ika_decision install/ika_decision build/ika_vision install/ika_vision build/ika_bringup install/ika_bringup
if [ -f /opt/ros/jazzy/setup.bash ]; then source /opt/ros/jazzy/setup.bash; else source /opt/ros/humble/setup.bash; fi
source install/setup.bash 2>/dev/null || true
colcon build --packages-select ika_interfaces
source install/setup.bash
colcon build --packages-select ika_decision ika_vision ika_bringup

echo "============================================"
echo "  COMPLETE SYSTEM FIX v2 DONE!              "
echo "============================================"
