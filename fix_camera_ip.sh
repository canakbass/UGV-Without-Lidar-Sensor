#!/bin/bash
set -e
WS_DIR=~/ika_ws

# Doğru Windows host IP'yi bul
WIN_IP=$(ip route show default | awk '{print $3}')
echo "Windows Host IP: $WIN_IP"

cat > "$WS_DIR/src/ika_robot/ika_vision/ika_vision/detector_node.py" << PYEOF
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from ika_interfaces.msg import SignDetection
import cv2
import numpy as np
import threading
import time

WINDOWS_HOST_IP = "${WIN_IP}"

class VisionNode(Node):
    def __init__(self):
        super().__init__('vision_node')
        
        self.stream_url = f"http://{WINDOWS_HOST_IP}:8080/stream"
        self.current_state = "IDLE"
        self.last_detection_time = 0
        self.cooldown = 3.0

        self.pub_sign = self.create_publisher(SignDetection, '/vision/sign', 10)
        self.create_subscription(String, '/robot_state', self.state_callback, 10)

        self.yolo_model = None
        self.ocr_reader = None
        self.models_loaded = False
        
        self.load_thread = threading.Thread(target=self.load_models, daemon=True)
        self.load_thread.start()
        
        self.frame = None
        self.frame_lock = threading.Lock()
        self.cam_thread = threading.Thread(target=self.camera_loop, daemon=True)
        self.cam_thread.start()
        
        self.create_timer(0.2, self.process_frame)
        
        self.get_logger().info(f'Vision Node Started - Camera: {self.stream_url}')

    def load_models(self):
        try:
            from ultralytics import YOLO
            self.yolo_model = YOLO('yolov8n.pt')
            self.get_logger().info('YOLOv8n model loaded!')
        except Exception as e:
            self.get_logger().warn(f'YOLO load failed: {e}')
        
        try:
            import easyocr
            self.ocr_reader = easyocr.Reader(['en'], gpu=False)
            self.get_logger().info('EasyOCR loaded!')
        except Exception as e:
            self.get_logger().warn(f'EasyOCR load failed: {e}')
        
        self.models_loaded = True
        self.get_logger().info('=== ALL VISION MODELS READY ===')

    def state_callback(self, msg):
        self.current_state = msg.data

    def camera_loop(self):
        while True:
            try:
                cap = cv2.VideoCapture(self.stream_url)
                if not cap.isOpened():
                    self.get_logger().warn(f'Camera not available at {self.stream_url}, retrying in 3s...')
                    time.sleep(3)
                    continue
                
                self.get_logger().info('Connected to Unity camera!')
                
                while cap.isOpened():
                    ret, frame = cap.read()
                    if not ret:
                        break
                    with self.frame_lock:
                        self.frame = frame
                
                cap.release()
            except Exception as e:
                self.get_logger().warn(f'Camera error: {e}')
            time.sleep(2)

    def process_frame(self):
        if self.current_state == "IDLE":
            return
        if not self.models_loaded:
            return

        with self.frame_lock:
            frame = self.frame
        if frame is None:
            return
        
        now = time.time()
        if now - self.last_detection_time < self.cooldown:
            return

        stop_detected = self.detect_stop_sign(frame)
        if stop_detected:
            self.publish_detection("STOP", 0.95)
            self.last_detection_time = now
            return

        number = self.detect_number(frame)
        if number is not None:
            self.publish_detection("NEXT_STAGE", 0.90)
            self.last_detection_time = now
            return

    def detect_stop_sign(self, frame):
        if self.yolo_model is not None:
            try:
                results = self.yolo_model(frame, conf=0.5, verbose=False)
                for r in results:
                    for box in r.boxes:
                        cls_id = int(box.cls[0])
                        cls_name = self.yolo_model.names[cls_id]
                        if cls_name == "stop sign":
                            conf = float(box.conf[0])
                            self.get_logger().warn(f'>>> STOP SIGN (YOLO) conf={conf:.2f} <<<')
                            return True
            except Exception:
                pass
        
        hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
        mask1 = cv2.inRange(hsv, np.array([0, 100, 100]), np.array([10, 255, 255]))
        mask2 = cv2.inRange(hsv, np.array([160, 100, 100]), np.array([180, 255, 255]))
        red_mask = mask1 | mask2
        kernel = np.ones((5, 5), np.uint8)
        red_mask = cv2.morphologyEx(red_mask, cv2.MORPH_OPEN, kernel)
        red_mask = cv2.morphologyEx(red_mask, cv2.MORPH_CLOSE, kernel)
        contours, _ = cv2.findContours(red_mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        for cnt in contours:
            area = cv2.contourArea(cnt)
            if area > 2000:
                peri = cv2.arcLength(cnt, True)
                approx = cv2.approxPolyDP(cnt, 0.04 * peri, True)
                if 6 <= len(approx) <= 10:
                    self.get_logger().warn(f'>>> STOP SIGN (Color) area={area:.0f} <<<')
                    return True
        return False

    def detect_number(self, frame):
        hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
        blue_mask = cv2.inRange(hsv, np.array([100, 80, 80]), np.array([130, 255, 255]))
        white_mask = cv2.inRange(hsv, np.array([0, 0, 200]), np.array([180, 40, 255]))
        combined_mask = blue_mask | white_mask
        kernel = np.ones((5, 5), np.uint8)
        combined_mask = cv2.morphologyEx(combined_mask, cv2.MORPH_CLOSE, kernel)
        contours, _ = cv2.findContours(combined_mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        for cnt in contours:
            area = cv2.contourArea(cnt)
            if area > 1500:
                x, y, w, h = cv2.boundingRect(cnt)
                aspect = w / float(h) if h > 0 else 0
                if 0.3 < aspect < 3.0:
                    roi = frame[y:y+h, x:x+w]
                    if self.ocr_reader is not None:
                        try:
                            results = self.ocr_reader.readtext(roi, allowlist='0123456789')
                            for (_, text, conf) in results:
                                text = text.strip()
                                if text.isdigit() and 1 <= int(text) <= 6:
                                    self.get_logger().warn(f'>>> NUMBER: {text} (conf={conf:.2f}) <<<')
                                    return int(text)
                        except Exception:
                            pass
        return None

    def publish_detection(self, sign_type, confidence):
        msg = SignDetection()
        msg.type = sign_type
        msg.distance = 2.0
        msg.confidence = confidence
        self.pub_sign.publish(msg)
        self.get_logger().info(f'Published: {sign_type}, conf={confidence}')

def main(args=None):
    rclpy.init(args=args)
    node = VisionNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
PYEOF

cd "$WS_DIR"
rm -rf build/ika_vision install/ika_vision
if [ -f /opt/ros/jazzy/setup.bash ]; then source /opt/ros/jazzy/setup.bash; else source /opt/ros/humble/setup.bash; fi
source install/setup.bash 2>/dev/null || true
colcon build --packages-select ika_vision

echo "DONE - Using Windows IP: $WIN_IP"
