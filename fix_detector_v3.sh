#!/bin/bash
set -e
WS_DIR=~/ika_ws

echo "============================================"
echo "  DETECTOR FIX - WebSocket Bridge Mode      "
echo "============================================"

# Sorun: Unity → rosbridge WS → publish ediyor ama
# rosbridge bunu DDS'e bridge etmiyor (topic type uyumsuzluğu).
# Çözüm: Detector doğrudan rosbridge WebSocket'e bağlanıp frame alır.

pip3 install websocket-client 2>/dev/null || true

cat > "$WS_DIR/src/ika_robot/ika_vision/ika_vision/detector_node.py" << 'PYEOF'
"""
TEKNOFEST UGV - Tabela Algılama v3

FRAME ALMA:
  Unity → CameraStreamer → rosbridge WS → CompressedImage topic
  Bu node rosbridge WS üzerinden frame alır (subscribe via WS).
  
  Gerçek robotta: 'camera_source: local' parametresi ile /dev/video0

ALGILAMA:
  1. Kırmızı daire (Hough Circle) → iç analiz → STOP / NEXT_STAGE
  2. Hedef (siyah-beyaz bullseye) → sadece PLATFORM_ATIS'ta → FINISH
"""

import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from ika_interfaces.msg import SignDetection
import cv2
import numpy as np
import time
import base64
import json
import threading

class VisionDetector(Node):
    def __init__(self):
        super().__init__('vision_node')
        
        self.declare_parameter('camera_source', 'ros_topic')
        self.declare_parameter('local_camera_id', 0)
        self.declare_parameter('cooldown', 4.0)
        self.declare_parameter('rosbridge_url', 'ws://localhost:9090')
        
        self.camera_source = self.get_parameter('camera_source').value
        self.cooldown = self.get_parameter('cooldown').value
        self.ws_url = self.get_parameter('rosbridge_url').value
        
        self.current_state = "IDLE"
        self.last_detection_time = 0
        self.frame_count = 0
        self.detection_count = 0
        
        self.pub_sign = self.create_publisher(SignDetection, '/vision/sign', 10)
        self.create_subscription(String, '/robot_state', self.state_cb, 10)
        
        if self.camera_source == 'ros_topic':
            # WebSocket üzerinden frame al
            self._start_ws_listener()
            self.get_logger().info(f'Camera: rosbridge WS {self.ws_url} → /camera/compressed')
        else:
            cam_id = self.get_parameter('local_camera_id').value
            self.cap = cv2.VideoCapture(cam_id)
            self.create_timer(0.2, self.local_camera_loop)
            self.get_logger().info(f'Camera: local /dev/video{cam_id}')
        
        # Diagnostik timer
        self.create_timer(10.0, self.print_stats)
        
        self.get_logger().info('=== VISION DETECTOR v3 (Hough Circle + WS) ===')

    def _start_ws_listener(self):
        """Rosbridge WebSocket'e bağlanıp /camera/compressed dinle"""
        def ws_thread():
            import websocket
            import json as json_lib
            
            retry_count = 0
            while True:
                try:
                    self.get_logger().info(f'Connecting to rosbridge: {self.ws_url}')
                    ws = websocket.create_connection(self.ws_url, timeout=5)
                    
                    # Subscribe to /camera/compressed
                    sub_msg = json_lib.dumps({
                        "op": "subscribe",
                        "topic": "/camera/compressed",
                        "type": "sensor_msgs/msg/CompressedImage",
                        "throttle_rate": 200  # Max 5 FPS
                    })
                    ws.send(sub_msg)
                    self.get_logger().info('Subscribed to /camera/compressed via WS')
                    retry_count = 0
                    
                    while True:
                        raw = ws.recv()
                        if not raw:
                            continue
                        
                        try:
                            data = json_lib.loads(raw)
                            if data.get("topic") == "/camera/compressed":
                                self._handle_ws_frame(data)
                        except Exception as e:
                            pass

                except Exception as e:
                    retry_count += 1
                    wait = min(retry_count * 2, 10)
                    self.get_logger().warn(f'WS connection failed: {e}, retry in {wait}s')
                    time.sleep(wait)
        
        t = threading.Thread(target=ws_thread, daemon=True)
        t.start()
    
    def _handle_ws_frame(self, data):
        """WebSocket'ten gelen frame'i işle"""
        if self.current_state in ["IDLE", "MANUAL"]:
            return
        
        now = time.time()
        if now - self.last_detection_time < self.cooldown:
            return
        
        try:
            msg_data = data.get("msg", {})
            b64_data = msg_data.get("data", "")
            
            if not b64_data:
                return
            
            jpeg_bytes = base64.b64decode(b64_data)
            np_arr = np.frombuffer(jpeg_bytes, np.uint8)
            frame = cv2.imdecode(np_arr, cv2.IMREAD_COLOR)
            
            if frame is not None:
                self.frame_count += 1
                if self.frame_count <= 3:
                    h, w = frame.shape[:2]
                    self.get_logger().info(f'Frame #{self.frame_count}: {w}x{h} decoded OK')
                self.process_frame(frame)
        except Exception as e:
            self.get_logger().warn(f'Frame decode error: {e}')
    
    def state_cb(self, msg):
        old = self.current_state
        self.current_state = msg.data
        if old != self.current_state:
            self.get_logger().info(f'State changed: {old} → {self.current_state}')

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
            f'Stats: state={self.current_state}, frames={self.frame_count}, '
            f'detections={self.detection_count}'
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
        hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
        
        mask1 = cv2.inRange(hsv, np.array([0, 80, 80]), np.array([10, 255, 255]))
        mask2 = cv2.inRange(hsv, np.array([160, 80, 80]), np.array([180, 255, 255]))
        red_mask = mask1 | mask2
        
        kernel = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (5, 5))
        red_mask = cv2.morphologyEx(red_mask, cv2.MORPH_CLOSE, kernel, iterations=2)
        red_mask = cv2.morphologyEx(red_mask, cv2.MORPH_OPEN, kernel, iterations=1)
        
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
            
            # Halka kırmızılık
            circle_mask = np.zeros(red_mask.shape, np.uint8)
            cv2.circle(circle_mask, (cx, cy), r, 255, -1)
            inner_mask = np.zeros(red_mask.shape, np.uint8)
            cv2.circle(inner_mask, (cx, cy), int(r * 0.65), 255, -1)
            ring_mask = circle_mask - inner_mask
            
            ring_px = cv2.countNonZero(ring_mask)
            ring_red = cv2.countNonZero(cv2.bitwise_and(red_mask, ring_mask))
            if ring_px == 0 or (ring_red / ring_px) < 0.35:
                continue
            
            # İç beyazlık
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
                continue
            
            # Siyah içerik
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
                f'CIRCLE: ({cx},{cy}) r={r} red={red_ratio:.2f} '
                f'white={white_ratio:.2f} aspect={aspect:.2f} black={black_fill:.2f}'
            )
            
            if black_fill < 0.05:
                continue
            
            if aspect > 1.5 and black_fill > 0.12:
                self.get_logger().warn(f'>>> STOP (aspect={aspect:.1f}) <<<')
                return ("STOP", 0.95)
            else:
                self.get_logger().warn(f'>>> NUMBER (aspect={aspect:.1f}) <<<')
                return ("NEXT_STAGE", 0.90)
        
        return None

    def detect_target_sign(self, frame):
        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        _, black_mask = cv2.threshold(gray, 50, 255, cv2.THRESH_BINARY_INV)
        
        kernel = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (7, 7))
        black_mask = cv2.morphologyEx(black_mask, cv2.MORPH_CLOSE, kernel)
        black_mask = cv2.morphologyEx(black_mask, cv2.MORPH_OPEN, kernel)
        
        blurred = cv2.GaussianBlur(black_mask, (9, 9), 2)
        circles = cv2.HoughCircles(
            blurred, cv2.HOUGH_GRADIENT,
            dp=1.2, minDist=20,
            param1=50, param2=40,
            minRadius=25, maxRadius=150
        )
        
        if circles is None or len(circles[0]) < 2:
            return False
        
        cl = [(int(c[0]), int(c[1]), int(c[2])) for c in circles[0]]
        
        for i in range(len(cl)):
            for j in range(i+1, len(cl)):
                dist = np.sqrt((cl[i][0]-cl[j][0])**2 + (cl[i][1]-cl[j][1])**2)
                r_diff = abs(cl[i][2] - cl[j][2])
                max_r = max(cl[i][2], cl[j][2])
                
                if dist < max_r * 0.2 and r_diff > max_r * 0.3:
                    hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
                    cx, cy = cl[i][0], cl[i][1]
                    rr = max_r + 10
                    fh, fw = frame.shape[:2]
                    rx1, ry1 = max(0,cx-rr), max(0,cy-rr)
                    rx2, ry2 = min(fw,cx+rr), min(fh,cy+rr)
                    
                    hsv_r = hsv[ry1:ry2, rx1:rx2]
                    if hsv_r.size == 0:
                        continue
                    
                    r1 = cv2.inRange(hsv_r, np.array([0,80,80]), np.array([10,255,255]))
                    r2 = cv2.inRange(hsv_r, np.array([160,80,80]), np.array([180,255,255]))
                    red_pct = (cv2.countNonZero(r1|r2) / (hsv_r.shape[0]*hsv_r.shape[1])) * 100
                    
                    if red_pct > 3:
                        continue
                    
                    self.get_logger().warn(f'>>> TARGET at ({cx},{cy}) <<<')
                    return True
        return False

    def publish_detection(self, sign_type, confidence):
        self.detection_count += 1
        msg = SignDetection()
        msg.type = sign_type
        msg.distance = 2.0
        msg.confidence = confidence
        self.pub_sign.publish(msg)
        self.get_logger().info(f'[#{self.detection_count}] {sign_type} (conf={confidence:.2f})')

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
echo "  DETECTOR v3 DEPLOYED!                     "
echo "============================================"
