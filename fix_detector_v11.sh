#!/bin/bash
set -e
WS_DIR=~/ika_ws

cat > "$WS_DIR/src/ika_robot/ika_vision/ika_vision/detector_node.py" << 'PYEOF'
"""
Tabela Algılama v11

Veri analizi (önceki loglardan):
  GERÇEK TABELALAR: white=20-30%, black=60-70%
  BARİYERLER:       white= 0-10%, black=85-100%

Değişiklikler:
  - white >= 15% (önceki 20% tabelaları kaçırıyordu)
  - black <= 75% (önceki 55% tabelaları reddediyordu!)
  - Siyah threshold 80 → 100 (daha az agresif, gri pixeller siyah sayılmaz)
  - Debug: her 50 frame'de bir frame'i dosyaya kaydet (analiz için)
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

class VisionDetector(Node):
    def __init__(self):
        super().__init__('vision_node')
        
        self.declare_parameter('camera_source', 'ros_topic')
        self.declare_parameter('local_camera_id', 0)
        self.declare_parameter('cooldown', 2.0)
        self.declare_parameter('min_radius', 15)
        
        self.camera_source = self.get_parameter('camera_source').value
        self.cooldown = self.get_parameter('cooldown').value
        self.min_r = self.get_parameter('min_radius').value
        
        # Kalibrasyon eşikleri (loglardan elde edildi)
        self.min_white = 0.15    # Gerçek tabela min beyazlık
        self.max_black = 0.75    # Gerçek tabela max siyahlık
        self.black_thresh = 100  # Siyah pixel eşiği (önceki 80 çok agresifti)
        
        self.current_state = "IDLE"
        self.last_detection_time = 0
        self.frame_count = 0
        self.detection_count = 0
        self.reject_count = 0
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
        self.get_logger().info(
            f'=== DETECTOR v11 ===\n'
            f'  white>={self.min_white:.0%} black<={self.max_black:.0%} '
            f'thresh={self.black_thresh} r>={self.min_r}'
        )

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
                    self.get_logger().info(f'Frame #{self.frame_count}: {frame.shape[1]}x{frame.shape[0]}')
                # Her 100 frame'de bir kaydet (debug)
                if self.frame_count % 100 == 1:
                    path = f'{self.save_dir}/frame_{self.frame_count}.jpg'
                    cv2.imwrite(path, frame)
                    self.get_logger().info(f'Debug frame saved: {path}')
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
            f'frames={self.frame_count}, '
            f'OK={self.detection_count}, REJ={self.reject_count}'
        )

    def process_frame(self, frame, now):
        result = self.detect_sign(frame)
        if result is not None:
            sign_type, confidence = result
            self.publish_detection(sign_type, confidence)
            self.last_detection_time = now
            # Tespit anını kaydet
            path = f'{self.save_dir}/detection_{self.detection_count}.jpg'
            cv2.imwrite(path, frame)
            return
        if self.current_state == "PLATFORM_ATIS":
            if self.detect_target_sign(frame):
                self.publish_detection("FINISH", 0.90)
                self.last_detection_time = now

    def detect_sign(self, frame):
        hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
        m1 = cv2.inRange(hsv, np.array([0, 70, 70]), np.array([12, 255, 255]))
        m2 = cv2.inRange(hsv, np.array([160, 70, 70]), np.array([180, 255, 255]))
        red_mask = m1 | m2
        
        k = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (5, 5))
        red_mask = cv2.morphologyEx(red_mask, cv2.MORPH_CLOSE, k, iterations=2)
        red_mask = cv2.morphologyEx(red_mask, cv2.MORPH_OPEN, k, iterations=1)
        
        contours, _ = cv2.findContours(red_mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        h, w = frame.shape[:2]
        
        # Tüm adayları topla
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
        
        # Her adayın iç beyazlığını hesapla, en iyisini seç
        best_sign = None
        best_score = -1  # white - black farkı (tabela: yüksek, bariyer: düşük)
        
        for cx, cy, r, circ in candidates:
            ir = int(r * 0.50)
            ix1, iy1 = max(0, cx-ir), max(0, cy-ir)
            ix2, iy2 = min(w, cx+ir), min(h, cy+ir)
            roi = frame[iy1:iy2, ix1:ix2]
            if roi.size == 0 or roi.shape[0] < 6 or roi.shape[1] < 6:
                continue
            
            # Beyazlık
            hsv_roi = cv2.cvtColor(roi, cv2.COLOR_BGR2HSV)
            white = cv2.inRange(hsv_roi, np.array([0, 0, 160]), np.array([180, 50, 255]))
            wr = cv2.countNonZero(white) / (white.shape[0] * white.shape[1])
            
            # Siyahlık (threshold 100, daha yumuşak)
            gray = cv2.cvtColor(roi, cv2.COLOR_BGR2GRAY)
            _, bw = cv2.threshold(gray, self.black_thresh, 255, cv2.THRESH_BINARY_INV)
            bf = cv2.countNonZero(bw) / (bw.shape[0] * bw.shape[1])
            
            # Skor: beyaz/siyah oranı (tabela yüksek, bariyer düşük)
            score = wr - (bf * 0.3)
            
            if score > best_score:
                best_score = score
                best_sign = (cx, cy, r, circ, wr, bf, roi, bw)
        
        if best_sign is None:
            return None
        
        cx, cy, r, circ, wr, bf, roi, bw = best_sign
        
        self.get_logger().info(
            f'BEST: ({cx},{cy}) r={r} c={circ:.2f} '
            f'white={wr:.0%} black={bf:.0%} [{len(candidates)}]'
        )
        
        # Filter
        if wr < self.min_white:
            self.reject_count += 1
            return None
        
        if bf > self.max_black:
            self.get_logger().info(f'  ❌ black={bf:.0%} > {self.max_black:.0%}')
            self.reject_count += 1
            return None
        
        if bf < 0.03:
            self.reject_count += 1
            return None
        
        # Aspect ratio
        coords = cv2.findNonZero(bw)
        if coords is None or len(coords) < 3:
            return None
        bx, by, bw2, bh2 = cv2.boundingRect(coords)
        if bh2 == 0:
            return None
        aspect = bw2 / bh2
        
        self.get_logger().warn(
            f'  ✅ SIGN: w={wr:.0%} b={bf:.0%} a={aspect:.2f}'
        )
        
        # Debug: ROI'yi kaydet
        roi_path = f'{self.save_dir}/roi_{self.detection_count+1}.jpg'
        cv2.imwrite(roi_path, roi)
        bw_path = f'{self.save_dir}/bw_{self.detection_count+1}.jpg'
        cv2.imwrite(bw_path, bw)
        
        if aspect > 2.0 and bf > 0.08:
            self.get_logger().warn(f'>>> STOP <<<')
            return ("STOP", 0.95)
        else:
            self.get_logger().warn(f'>>> NUMBER <<<')
            return ("NEXT_STAGE", 0.90)

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
echo "  DETECTOR v11 DONE"
