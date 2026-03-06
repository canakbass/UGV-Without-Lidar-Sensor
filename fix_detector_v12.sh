#!/bin/bash
set -e
WS_DIR=~/ika_ws

# Önce rakam şablonları oluştur (OpenCV ile programatik)
cat > /tmp/create_templates.py << 'PYEOF'
"""Rakam şablonları oluştur (1-9 + STOP)"""
import cv2
import numpy as np
import os

out_dir = os.path.expanduser("~/ika_ws/digit_templates")
os.makedirs(out_dir, exist_ok=True)

for digit in range(10):
    img = np.ones((60, 40), dtype=np.uint8) * 255  # Beyaz zemin
    text = str(digit)
    # Kalın siyah rakam
    cv2.putText(img, text, (5, 50), cv2.FONT_HERSHEY_SIMPLEX, 1.8, 0, 4)
    cv2.imwrite(f"{out_dir}/{digit}.png", img)

# STOP
img = np.ones((40, 80), dtype=np.uint8) * 255
cv2.putText(img, "STOP", (2, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.9, 0, 2)
cv2.imwrite(f"{out_dir}/STOP.png", img)

print(f"Templates saved to {out_dir}")
for f in sorted(os.listdir(out_dir)):
    print(f"  {f}")
PYEOF
python3 /tmp/create_templates.py

# Detector v12: OCR ile sayı okuma
cat > "$WS_DIR/src/ika_robot/ika_vision/ika_vision/detector_node.py" << 'PYEOF'
"""
Tabela Algılama v12 - SAYI OKUMA

Akış:
  1. Kırmızı dairesel kontur bul (circ > 0.45)
  2. En beyaz iç kısma sahip adayı seç
  3. İç ROI'yi al → grayscale → threshold
  4. Template matching ile 0-9 ve STOP karşılaştır
  5. En yüksek eşleşme → okunan sayı
  6. Okunan sayı = beklenen sonraki etap numarası → NEXT_STAGE
  7. Okunan = STOP → STOP sinyal

ETAP NUMARALARI:
  Tabela "1" → BASLA geçildi
  Tabela "2" → TASLI_YOL geçildi
  ...
  Tabela "8" → DIK_EGIM başlangıcı
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
import os
import glob

# Etap sırası ve hangi tabelada geçiş yapılacağı
STAGE_ORDER = [
    ("BASLA", 1),           # "1" tabelası görünce BASLA → TASLI_YOL
    ("TASLI_YOL", 2),       # "2" tabelası
    ("YAN_EGIM", 3),        # "3"
    ("DIK_ENGEL", 4),       # "4"
    ("TRAFIK_KONILERI", 5), # "5"
    ("KAYAR_ENGEL", 6),     # "6"
    ("ENGEBELI_ARAZI", 7),  # "7"
    ("DIK_EGIM_CIKIS", 8),  # "8"
]

class VisionDetector(Node):
    def __init__(self):
        super().__init__('vision_node')
        
        self.declare_parameter('camera_source', 'ros_topic')
        self.declare_parameter('local_camera_id', 0)
        self.declare_parameter('cooldown', 2.0)
        self.declare_parameter('min_radius', 15)
        self.declare_parameter('template_dir', os.path.expanduser('~/ika_ws/digit_templates'))
        
        self.camera_source = self.get_parameter('camera_source').value
        self.cooldown = self.get_parameter('cooldown').value
        self.min_r = self.get_parameter('min_radius').value
        template_dir = self.get_parameter('template_dir').value
        
        self.current_state = "IDLE"
        self.expected_digit = 1  # Sonraki beklenen tabela numarası
        self.last_detection_time = 0
        self.frame_count = 0
        self.detection_count = 0
        
        # Rakamlara ait şablonları yükle
        self.templates = {}
        self.load_templates(template_dir)
        
        self.save_dir = "/tmp/detector_debug"
        os.makedirs(self.save_dir, exist_ok=True)
        
        self.pub_sign = self.create_publisher(SignDetection, '/vision/sign', 10)
        self.create_subscription(String, '/robot_state', self.state_cb, 10)
        
        if self.camera_source == 'ros_topic':
            self.create_subscription(String, '/camera/image_base64', self.image_cb, 10)
            self.get_logger().info('Camera: /camera/image_base64')
        else:
            cam_id = self.get_parameter('local_camera_id').value
            self.cap = cv2.VideoCapture(cam_id)
            self.create_timer(0.2, self.local_camera_loop)
        
        self.create_timer(10.0, self.print_stats)
        self.get_logger().info(f'=== DETECTOR v12 (DIGIT READING) ===')
        self.get_logger().info(f'  Templates: {list(self.templates.keys())}')

    def load_templates(self, tdir):
        for f in glob.glob(os.path.join(tdir, "*.png")):
            name = os.path.splitext(os.path.basename(f))[0]
            t = cv2.imread(f, cv2.IMREAD_GRAYSCALE)
            if t is not None:
                self.templates[name] = t
                self.get_logger().info(f'  Template loaded: {name} ({t.shape})')

    def state_cb(self, msg):
        old = self.current_state
        self.current_state = msg.data
        if old != self.current_state:
            self.get_logger().info(f'State: {old} → {self.current_state}')
            # Hangi rakamı bekliyoruz?
            for i, (stage, digit) in enumerate(STAGE_ORDER):
                if stage == self.current_state:
                    self.expected_digit = digit
                    self.get_logger().info(f'  Expecting digit: {digit}')
                    break

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
                    self.get_logger().info(f'Frame #{self.frame_count}: {frame.shape[1]}x{frame.shape[0]}')
                self.process_frame(frame, now)
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
            self.process_frame(frame, now)

    def print_stats(self):
        self.get_logger().info(
            f'Stats: state={self.current_state}, '
            f'frames={self.frame_count}, OK={self.detection_count}, '
            f'expected={self.expected_digit}'
        )

    def process_frame(self, frame, now):
        result = self.detect_and_read(frame)
        if result is not None:
            sign_type, confidence, digit = result
            self.publish_detection(sign_type, confidence)
            self.last_detection_time = now
            # Debug kaydet
            path = f'{self.save_dir}/det_{self.detection_count}_{digit}.jpg'
            cv2.imwrite(path, frame)
            return
        if self.current_state == "PLATFORM_ATIS":
            if self.detect_target_sign(frame):
                self.publish_detection("FINISH", 0.90)
                self.last_detection_time = now

    def detect_and_read(self, frame):
        """Kırmızı daire bul → iç kısmı oku → sayı eşleştir"""
        hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
        m1 = cv2.inRange(hsv, np.array([0, 70, 70]), np.array([12, 255, 255]))
        m2 = cv2.inRange(hsv, np.array([160, 70, 70]), np.array([180, 255, 255]))
        red_mask = m1 | m2
        
        k = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (5, 5))
        red_mask = cv2.morphologyEx(red_mask, cv2.MORPH_CLOSE, k, iterations=2)
        red_mask = cv2.morphologyEx(red_mask, cv2.MORPH_OPEN, k, iterations=1)
        
        contours, _ = cv2.findContours(red_mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        h, w = frame.shape[:2]
        
        candidates = []
        for cnt in contours:
            area = cv2.contourArea(cnt)
            if area < 200 or area > 80000:
                continue
            peri = cv2.arcLength(cnt, True)
            if peri == 0:
                continue
            circ = (4 * math.pi * area) / (peri * peri)
            if circ < 0.45:
                continue
            (cx, cy), radius = cv2.minEnclosingCircle(cnt)
            r = int(radius)
            if r < self.min_r:
                continue
            candidates.append((int(cx), int(cy), r, circ))
        
        if not candidates:
            return None
        
        # Her adayın iç beyazlığını kontrol et
        best = None
        best_score = -1
        
        for cx, cy, r, circ in candidates:
            ir = int(r * 0.50)
            ix1, iy1 = max(0, cx-ir), max(0, cy-ir)
            ix2, iy2 = min(w, cx+ir), min(h, cy+ir)
            roi = frame[iy1:iy2, ix1:ix2]
            if roi.size == 0 or roi.shape[0] < 8 or roi.shape[1] < 8:
                continue
            
            hsv_roi = cv2.cvtColor(roi, cv2.COLOR_BGR2HSV)
            white = cv2.inRange(hsv_roi, np.array([0, 0, 160]), np.array([180, 50, 255]))
            wr = cv2.countNonZero(white) / (white.shape[0] * white.shape[1])
            
            if wr < 0.12:
                continue
            
            score = wr
            if score > best_score:
                best_score = score
                best = (cx, cy, r, circ, wr, roi)
        
        if best is None:
            return None
        
        cx, cy, r, circ, wr, roi = best
        
        # ═══ SAYI OKUMA ═══
        digit, confidence = self.read_digit(roi)
        
        if digit is None:
            return None
        
        self.get_logger().info(
            f'READ: ({cx},{cy}) r={r} white={wr:.0%} → "{digit}" ({confidence:.0%})'
        )
        
        # STOP kontrolü
        if digit == "STOP":
            self.get_logger().warn(f'>>> STOP READ at ({cx},{cy}) <<<')
            return ("STOP", 0.95, digit)
        
        # Rakam kontrolü
        try:
            num = int(digit)
        except ValueError:
            return None
        
        # Beklenen rakamla karşılaştır
        if num == self.expected_digit:
            self.get_logger().warn(
                f'>>> DIGIT {num} MATCHES expected {self.expected_digit} → NEXT_STAGE <<<'
            )
            return ("NEXT_STAGE", 0.95, digit)
        elif num == self.expected_digit + 1:
            # Bir sonraki de olabilir (bir tabela kaçırılmış)
            self.get_logger().warn(
                f'>>> DIGIT {num} (skip detected, expected {self.expected_digit}) → NEXT_STAGE <<<'
            )
            return ("NEXT_STAGE", 0.90, digit)
        else:
            self.get_logger().info(
                f'  digit={num} != expected={self.expected_digit}, ignoring'
            )
            return None
    
    def read_digit(self, roi):
        """Template matching ile ROI'deki rakamı oku"""
        if len(self.templates) == 0:
            return None, 0
        
        gray = cv2.cvtColor(roi, cv2.COLOR_BGR2GRAY)
        # Otsu thresholding (otomatik eşik)
        _, binary = cv2.threshold(gray, 0, 255, cv2.THRESH_BINARY_INV + cv2.THRESH_OTSU)
        
        # ROI'yi standart boyuta getir
        target_h = 60
        if roi.shape[0] < 8:
            return None, 0
        scale = target_h / roi.shape[0]
        target_w = max(int(roi.shape[1] * scale), 10)
        resized = cv2.resize(binary, (target_w, target_h), interpolation=cv2.INTER_AREA)
        
        best_match = None
        best_val = -1
        
        for name, template in self.templates.items():
            # Template'i ROI boyutuna uyarla
            th, tw = template.shape[:2]
            if resized.shape[0] < th or resized.shape[1] < tw:
                # Template büyükse küçült
                t_scaled = cv2.resize(template, (min(tw, resized.shape[1]-1), min(th, resized.shape[0]-1)))
            else:
                t_scaled = template
            
            if t_scaled.shape[0] > resized.shape[0] or t_scaled.shape[1] > resized.shape[1]:
                continue
            
            result = cv2.matchTemplate(resized, t_scaled, cv2.TM_CCOEFF_NORMED)
            _, max_val, _, _ = cv2.minMaxLoc(result)
            
            if max_val > best_val:
                best_val = max_val
                best_match = name
        
        if best_match is not None and best_val > 0.25:
            return best_match, best_val
        
        return None, 0

    def detect_target_sign(self, frame):
        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        _, bmask = cv2.threshold(gray, 50, 255, cv2.THRESH_BINARY_INV)
        k = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (7, 7))
        bmask = cv2.morphologyEx(bmask, cv2.MORPH_CLOSE, k)
        bmask = cv2.morphologyEx(bmask, cv2.MORPH_OPEN, k)
        contours, _ = cv2.findContours(bmask, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
        circles = []
        for cnt in contours:
            area = cv2.contourArea(cnt)
            if area < 500: continue
            peri = cv2.arcLength(cnt, True)
            if peri == 0: continue
            circ = (4*math.pi*area)/(peri*peri)
            if circ > 0.65:
                (cx,cy),r = cv2.minEnclosingCircle(cnt)
                circles.append((int(cx),int(cy),int(r)))
        for i in range(len(circles)):
            for j in range(i+1,len(circles)):
                d = np.sqrt((circles[i][0]-circles[j][0])**2+(circles[i][1]-circles[j][1])**2)
                mr = max(circles[i][2],circles[j][2])
                rd = abs(circles[i][2]-circles[j][2])
                if d < mr*0.3 and rd > mr*0.25:
                    cx,cy = circles[i][0],circles[i][1]
                    rr = mr+5
                    fh,fw = frame.shape[:2]
                    hr = cv2.cvtColor(frame[max(0,cy-rr):min(fh,cy+rr),max(0,cx-rr):min(fw,cx+rr)],cv2.COLOR_BGR2HSV)
                    if hr.size == 0: continue
                    r1 = cv2.inRange(hr,np.array([0,80,80]),np.array([10,255,255]))
                    r2 = cv2.inRange(hr,np.array([160,80,80]),np.array([180,255,255]))
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
        self.get_logger().warn(f'[#{self.detection_count}] >>> {sign_type} <<<')

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
echo "=== DETECTOR v12 (DIGIT READING) DONE ==="
