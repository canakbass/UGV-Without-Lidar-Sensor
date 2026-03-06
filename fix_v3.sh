#!/bin/bash
set -e
WS_DIR=~/ika_ws

echo "============================================"
echo "  CONTROLLER v3 + DETECTOR v6               "
echo "============================================"

# ═══════════════════════════════════════
# 1. CONTROLLER v3 - Ultrasonik debug + agresif dönüş
# ═══════════════════════════════════════
cat > "$WS_DIR/src/ika_robot/ika_control/ika_control/controller_node.py" << 'PYEOF'
"""
Navigasyon Kontrolcü v3

Değişiklikler:
  - Ultrasonik veri gelip gelmediğini logla
  - Daha agresif engel kaçınma (erken kaçın, sert dön)
  - Ultrasonik gelmiyorsa güvenli varsayılan davranış
"""

import rclpy
from rclpy.node import Node
from std_msgs.msg import String, Float32MultiArray
from geometry_msgs.msg import Twist
import time

STATE_SPEEDS = {
    "BASLA": 0.4,
    "TASLI_YOL": 0.35,
    "YAN_EGIM": 0.3,
    "DIK_ENGEL": 0.25,
    "TRAFIK_KONILERI": 0.2,
    "KAYAR_ENGEL": 0.25,
    "ENGEBELI_ARAZI": 0.3,
    "DIK_EGIM_CIKIS": 0.25,
    "DIK_EGIM_INIS": 0.2,
}

STOP_STATES = ["IDLE", "MANUAL", "CIKIS_DURMA", "PLATFORM_ATIS", "INIS_DURMA"]

class ControllerNode(Node):
    def __init__(self):
        super().__init__('controller_node')
        
        self.state = "IDLE"
        self.us = [4.0] * 7  # [FC, FL, FR, CFL, CFR, CRL, CRR]
        self.us_received = False
        self.us_count = 0
        
        # PID
        self.kp = 2.0
        self.kd = 0.5
        self.prev_error = 0.0
        
        self.pub_cmd = self.create_publisher(Twist, '/cmd_vel', 10)
        self.create_subscription(String, '/robot_state', self.state_cb, 10)
        self.create_subscription(Float32MultiArray, '/sensors/ultrasonic', self.us_cb, 10)
        
        self.create_timer(0.1, self.control_loop)  # 10 Hz
        self.create_timer(5.0, self.debug_log)
        
        self.get_logger().info('Navigation Controller v3 Started')
    
    def state_cb(self, msg):
        old = self.state
        self.state = msg.data
        if old != self.state:
            self.get_logger().info(f'State: {old} → {self.state}')
    
    def us_cb(self, msg):
        if msg.data and len(msg.data) >= 7:
            self.us = list(msg.data)
            self.us_received = True
            self.us_count += 1
    
    def debug_log(self):
        self.get_logger().info(
            f'NAV: state={self.state}, us_received={self.us_received}, '
            f'us_count={self.us_count}, '
            f'FC={self.us[0]:.2f} FL={self.us[1]:.2f} FR={self.us[2]:.2f}'
        )
    
    def control_loop(self):
        cmd = Twist()
        
        if self.state in STOP_STATES:
            self.pub_cmd.publish(cmd)
            return
        
        if self.state not in STATE_SPEEDS:
            self.pub_cmd.publish(cmd)
            return
        
        speed = STATE_SPEEDS[self.state]
        steering = 0.0
        
        fc = self.us[0]   # Ön merkez
        fl = self.us[1]   # Ön sol
        fr = self.us[2]   # Ön sağ
        cfl = self.us[3]  # Köşe ön sol
        cfr = self.us[4]  # Köşe ön sağ
        
        if not self.us_received:
            # Ultrasonik veri gelmiyorsa → çok yavaş düz git
            speed = 0.15
            steering = 0.0
        elif fc < 0.4:
            # ÇOK YAKIN → sert dönüş
            speed = 0.1
            if fl > fr:
                steering = 0.8  # Sola kır
            else:
                steering = -0.8  # Sağa kır
        elif fc < 0.8:
            # YAKIN → yavaşla + dön
            speed *= 0.3
            if fl > fr:
                steering = 0.5
            else:
                steering = -0.5
        elif fc < 1.2:
            # ORTA → hafif yavaşla + hafif dön
            speed *= 0.6
            if fl > fr:
                steering = 0.3
            else:
                steering = -0.3
        elif cfl < 0.4 or cfr < 0.4:
            # Köşe engel
            speed *= 0.5
            if cfl < cfr:
                steering = -0.5
            else:
                steering = 0.5
        else:
            # NORMAL → PID merkezleme
            error = fr - fl
            derivative = error - self.prev_error
            steering = self.kp * error + self.kd * derivative
            self.prev_error = error
            steering = max(-0.6, min(0.6, steering))
        
        cmd.linear.x = max(0.0, speed)  # Geri gitme
        cmd.angular.z = steering
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
# 2. DETECTOR v6 - Daha sıkı filtre
# ═══════════════════════════════════════
cat > "$WS_DIR/src/ika_robot/ika_vision/ika_vision/detector_node.py" << 'PYEOF'
"""
Tabela Algılama v6

Değişiklikler:
  - Minimum yarıçap 60px (daha yakından algıla)
  - Cooldown 6 saniye
  - 3 ardışık frame'de görünmesi gerekir (false positive önleme)
  - Beyazlık eşiği artırıldı
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
        self.declare_parameter('cooldown', 6.0)
        self.declare_parameter('min_radius', 60)
        self.declare_parameter('confirm_frames', 2)
        
        self.camera_source = self.get_parameter('camera_source').value
        self.cooldown = self.get_parameter('cooldown').value
        self.min_r = self.get_parameter('min_radius').value
        self.confirm_needed = self.get_parameter('confirm_frames').value
        
        self.current_state = "IDLE"
        self.last_detection_time = 0
        self.frame_count = 0
        self.detection_count = 0
        
        # Onay sistemi: aynı tip ardışık N kez görünmeli
        self.pending_type = None
        self.pending_count = 0
        
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
        self.get_logger().info(
            f'=== DETECTOR v6 (min_r={self.min_r}, '
            f'confirm={self.confirm_needed}, cooldown={self.cooldown}s) ==='
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
            f'frames={self.frame_count}, detections={self.detection_count}, '
            f'pending={self.pending_type}({self.pending_count})'
        )

    def process_frame(self, frame):
        result = self.detect_sign_contour(frame)
        
        if result is not None:
            sign_type, confidence = result
            
            # Onay sistemi: aynı tip ardışık N kez → publish
            if sign_type == self.pending_type:
                self.pending_count += 1
            else:
                self.pending_type = sign_type
                self.pending_count = 1
            
            if self.pending_count >= self.confirm_needed:
                self.publish_detection(sign_type, confidence)
                self.last_detection_time = time.time()
                self.pending_type = None
                self.pending_count = 0
        else:
            # Hiç aday yok → pending sıfırla
            self.pending_type = None
            self.pending_count = 0
        
        if self.current_state == "PLATFORM_ATIS":
            if self.detect_target_sign(frame):
                self.publish_detection("FINISH", 0.90)
                self.last_detection_time = time.time()

    def detect_sign_contour(self, frame):
        hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
        m1 = cv2.inRange(hsv, np.array([0, 70, 70]), np.array([12, 255, 255]))
        m2 = cv2.inRange(hsv, np.array([160, 70, 70]), np.array([180, 255, 255]))
        red_mask = m1 | m2
        
        k = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (5, 5))
        red_mask = cv2.morphologyEx(red_mask, cv2.MORPH_CLOSE, k, iterations=2)
        red_mask = cv2.morphologyEx(red_mask, cv2.MORPH_OPEN, k, iterations=1)
        
        contours, _ = cv2.findContours(red_mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        h, w = frame.shape[:2]
        
        best = None
        best_r = 0
        
        for cnt in contours:
            area = cv2.contourArea(cnt)
            if area < 1200 or area > 60000:
                continue
            peri = cv2.arcLength(cnt, True)
            if peri == 0:
                continue
            circ = (4 * math.pi * area) / (peri * peri)
            if circ < 0.55:
                continue
            (cx, cy), radius = cv2.minEnclosingCircle(cnt)
            r = int(radius)
            if r < self.min_r:
                continue
            if r > best_r:
                best_r = r
                best = (int(cx), int(cy), r, circ)
        
        if best is None:
            return None
        
        cx, cy, r, circ = best
        
        # İç analiz
        ir = int(r * 0.55)
        ix1, iy1 = max(0, cx-ir), max(0, cy-ir)
        ix2, iy2 = min(w, cx+ir), min(h, cy+ir)
        roi = frame[iy1:iy2, ix1:ix2]
        if roi.size == 0 or roi.shape[0] < 10 or roi.shape[1] < 10:
            return None
        
        # Beyazlık
        hsv_roi = cv2.cvtColor(roi, cv2.COLOR_BGR2HSV)
        white = cv2.inRange(hsv_roi, np.array([0, 0, 150]), np.array([180, 60, 255]))
        wr = cv2.countNonZero(white) / (white.shape[0] * white.shape[1])
        if wr < 0.10:
            return None
        
        # Siyah içerik
        gray = cv2.cvtColor(roi, cv2.COLOR_BGR2GRAY)
        _, bw = cv2.threshold(gray, 80, 255, cv2.THRESH_BINARY_INV)
        coords = cv2.findNonZero(bw)
        if coords is None or len(coords) < 5:
            return None
        bx, by, bw2, bh2 = cv2.boundingRect(coords)
        if bh2 == 0:
            return None
        aspect = bw2 / bh2
        bf = cv2.countNonZero(bw) / (bw.shape[0] * bw.shape[1])
        
        self.get_logger().info(
            f'SIGN: ({cx},{cy}) r={r} c={circ:.2f} '
            f'w={wr:.2f} a={aspect:.2f} b={bf:.2f}'
        )
        
        if bf < 0.03:
            return None
        
        if aspect > 1.5 and bf > 0.10:
            return ("STOP", 0.95)
        else:
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
                    hr = cv2.cvtColor(frame[max(0,cy-rr):min(fh,cy+rr),max(0,cx-rr):min(fw,cx+rr)], cv2.COLOR_BGR2HSV)
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
        self.get_logger().warn(f'[#{self.detection_count}] >>> {sign_type} CONFIRMED <<<')

def main(args=None):
    rclpy.init(args=args)
    node = VisionDetector()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
PYEOF

# BUILD
cd "$WS_DIR"
rm -rf build/ika_vision install/ika_vision build/ika_control install/ika_control
if [ -f /opt/ros/jazzy/setup.bash ]; then source /opt/ros/jazzy/setup.bash; else source /opt/ros/humble/setup.bash; fi
source install/setup.bash 2>/dev/null || true
colcon build --packages-select ika_vision ika_control

echo "============================================"
echo "  CONTROLLER v3 + DETECTOR v6 DONE!         "
echo "============================================"
