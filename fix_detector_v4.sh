#!/bin/bash
set -e
WS_DIR=~/ika_ws

echo "============================================"
echo "  DETECTOR v4 - String Topic (guaranteed)   "
echo "============================================"

cat > "$WS_DIR/src/ika_robot/ika_vision/ika_vision/detector_node.py" << 'PYEOF'
"""
TEKNOFEST UGV - Tabela Algılama v4

Frame alma:
  Unity → CameraStreamer → rosbridge → /camera/image_base64 (std_msgs/String)
  String topic rosbridge'de %100 çalışıyor (robot_state gibi).
  data alanı: base64 encoded JPEG

Gerçek robotta: camera_source='local' → /dev/video0
"""

import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from ika_interfaces.msg import SignDetection
import cv2
import numpy as np
import time
import base64

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
            # std_msgs/String olarak base64 JPEG alıyoruz
            self.create_subscription(
                String, '/camera/image_base64',
                self.image_cb, 10
            )
            self.get_logger().info('Camera: /camera/image_base64 (String, base64 JPEG)')
        else:
            cam_id = self.get_parameter('local_camera_id').value
            self.cap = cv2.VideoCapture(cam_id)
            self.create_timer(0.2, self.local_camera_loop)
            self.get_logger().info(f'Camera: local /dev/video{cam_id}')
        
        self.create_timer(10.0, self.print_stats)
        self.get_logger().info('=== VISION DETECTOR v4 (String topic) ===')

    def state_cb(self, msg):
        old = self.current_state
        self.current_state = msg.data
        if old != self.current_state:
            self.get_logger().info(f'State: {old} → {self.current_state}')

    def image_cb(self, msg):
        """std_msgs/String callback - data = base64 encoded JPEG"""
        if self.current_state in ["IDLE", "MANUAL"]:
            return
        
        now = time.time()
        if now - self.last_detection_time < self.cooldown:
            return
        
        try:
            b64_str = msg.data
            if not b64_str or len(b64_str) < 100:
                return
            
            jpeg_bytes = base64.b64decode(b64_str)
            np_arr = np.frombuffer(jpeg_bytes, np.uint8)
            frame = cv2.imdecode(np_arr, cv2.IMREAD_COLOR)
            
            if frame is not None:
                self.frame_count += 1
                if self.frame_count <= 3:
                    h, w = frame.shape[:2]
                    self.get_logger().info(f'Frame #{self.frame_count}: {w}x{h} OK')
                self.process_frame(frame)
            else:
                if self.frame_count == 0:
                    self.get_logger().warn('imdecode returned None')
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
        result = self.detect_red_circle_sign(frame)
        if result is not None:
            sign_type, confidence = result
            self.publish_detection(sign_type, confidence)
            self.last_detection_time = time.time()
            return
        
        if self.current_state == "PLATFORM_ATIS":
            if self.detect_target_sign(frame):
                self.publish_detection("FINISH", 0.90)
                self.last_detection_time = time.time()

    def detect_red_circle_sign(self, frame):
        """Kırmızı daire → Hough Circle → iç analiz → STOP/NUMBER"""
        hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
        
        mask1 = cv2.inRange(hsv, np.array([0, 80, 80]), np.array([10, 255, 255]))
        mask2 = cv2.inRange(hsv, np.array([160, 80, 80]), np.array([180, 255, 255]))
        red_mask = mask1 | mask2
        
        kernel = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (5, 5))
        red_mask = cv2.morphologyEx(red_mask, cv2.MORPH_CLOSE, kernel, iterations=2)
        red_mask = cv2.morphologyEx(red_mask, cv2.MORPH_OPEN, kernel, iterations=1)
        
        total_red = cv2.countNonZero(red_mask)
        total_px = frame.shape[0] * frame.shape[1]
        if (total_red / total_px) < 0.005:
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
        
        for c in circles[0]:
            cx, cy, r = int(c[0]), int(c[1]), int(c[2])
            if cx-r < 0 or cy-r < 0 or cx+r >= w or cy+r >= h or r < 15:
                continue
            
            # Halka kırmızılık doğrulama
            cmask = np.zeros(red_mask.shape, np.uint8)
            cv2.circle(cmask, (cx, cy), r, 255, -1)
            imask = np.zeros(red_mask.shape, np.uint8)
            cv2.circle(imask, (cx, cy), int(r*0.65), 255, -1)
            rmask = cmask - imask
            rpx = cv2.countNonZero(rmask)
            rred = cv2.countNonZero(cv2.bitwise_and(red_mask, rmask))
            if rpx == 0 or (rred/rpx) < 0.35:
                continue
            
            # İç beyazlık
            ir = int(r*0.6)
            roi = frame[max(0,cy-ir):min(h,cy+ir), max(0,cx-ir):min(w,cx+ir)]
            if roi.size == 0:
                continue
            hsv_roi = cv2.cvtColor(roi, cv2.COLOR_BGR2HSV)
            wh = cv2.inRange(hsv_roi, np.array([0,0,170]), np.array([180,50,255]))
            wr = cv2.countNonZero(wh) / (wh.shape[0]*wh.shape[1])
            if wr < 0.20:
                continue
            
            # Siyah içerik analizi
            gray = cv2.cvtColor(roi, cv2.COLOR_BGR2GRAY)
            _, bw = cv2.threshold(gray, 80, 255, cv2.THRESH_BINARY_INV)
            coords = cv2.findNonZero(bw)
            if coords is None or len(coords) < 10:
                continue
            bx, by, bw2, bh2 = cv2.boundingRect(coords)
            if bh2 == 0:
                continue
            aspect = bw2/bh2
            bf = cv2.countNonZero(bw)/(bw.shape[0]*bw.shape[1])
            
            self.get_logger().info(
                f'CIRCLE ({cx},{cy}) r={r} red={rred/rpx:.2f} '
                f'white={wr:.2f} aspect={aspect:.2f} black={bf:.2f}'
            )
            
            if bf < 0.05:
                continue
            if aspect > 1.5 and bf > 0.12:
                self.get_logger().warn(f'>>> STOP (a={aspect:.1f}) <<<')
                return ("STOP", 0.95)
            else:
                self.get_logger().warn(f'>>> NUMBER (a={aspect:.1f}) <<<')
                return ("NEXT_STAGE", 0.90)
        return None

    def detect_target_sign(self, frame):
        """Hedef: siyah-beyaz bullseye (sadece PLATFORM_ATIS'ta)"""
        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        _, bmask = cv2.threshold(gray, 50, 255, cv2.THRESH_BINARY_INV)
        k = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (7,7))
        bmask = cv2.morphologyEx(bmask, cv2.MORPH_CLOSE, k)
        bmask = cv2.morphologyEx(bmask, cv2.MORPH_OPEN, k)
        bl = cv2.GaussianBlur(bmask, (9,9), 2)
        circles = cv2.HoughCircles(bl, cv2.HOUGH_GRADIENT, dp=1.2,
            minDist=20, param1=50, param2=40, minRadius=25, maxRadius=150)
        if circles is None or len(circles[0]) < 2:
            return False
        cl = [(int(x[0]),int(x[1]),int(x[2])) for x in circles[0]]
        for i in range(len(cl)):
            for j in range(i+1, len(cl)):
                d = np.sqrt((cl[i][0]-cl[j][0])**2+(cl[i][1]-cl[j][1])**2)
                rd = abs(cl[i][2]-cl[j][2])
                mr = max(cl[i][2], cl[j][2])
                if d < mr*0.2 and rd > mr*0.3:
                    hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
                    cx,cy = cl[i][0], cl[i][1]
                    rr = mr+10
                    fh,fw = frame.shape[:2]
                    hr = hsv[max(0,cy-rr):min(fh,cy+rr), max(0,cx-rr):min(fw,cx+rr)]
                    if hr.size == 0: continue
                    r1 = cv2.inRange(hr, np.array([0,80,80]), np.array([10,255,255]))
                    r2 = cv2.inRange(hr, np.array([160,80,80]), np.array([180,255,255]))
                    rp = (cv2.countNonZero(r1|r2)/(hr.shape[0]*hr.shape[1]))*100
                    if rp > 3: continue
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

cd "$WS_DIR"
rm -rf build/ika_vision install/ika_vision
if [ -f /opt/ros/jazzy/setup.bash ]; then source /opt/ros/jazzy/setup.bash; else source /opt/ros/humble/setup.bash; fi
source install/setup.bash 2>/dev/null || true
colcon build --packages-select ika_vision

echo "============================================"
echo "  DETECTOR v4 DEPLOYED (String topic)       "
echo "============================================"
