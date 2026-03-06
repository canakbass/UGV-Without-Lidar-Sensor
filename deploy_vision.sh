#!/bin/bash
set -e
WS_DIR=~/ika_ws

echo "============================================"
echo "  DEPLOYING REAL VISION DETECTOR            "
echo "============================================"

cat > "$WS_DIR/src/ika_robot/ika_vision/ika_vision/detector_node.py" << 'PYEOF'
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from ika_interfaces.msg import SignDetection
import cv2
import numpy as np
import threading
import time

class VisionNode(Node):
    def __init__(self):
        super().__init__('vision_node')
        
        self.stream_url = "http://localhost:8080/stream"
        self.current_state = "IDLE"
        self.last_detection_time = 0
        self.cooldown = 3.0  # Aynı tabelayı spam etmeyi önlemek için 3sn bekleme

        # Publishers
        self.pub_sign = self.create_publisher(SignDetection, '/vision/sign', 10)
        self.create_subscription(String, '/robot_state', self.state_callback, 10)

        # YOLO & OCR (lazy load - ilk kullanımda yükle)
        self.yolo_model = None
        self.ocr_reader = None
        self.models_loaded = False
        
        # Ayrı thread'de modelleri yükle (node'u bloklamasın)
        self.load_thread = threading.Thread(target=self.load_models, daemon=True)
        self.load_thread.start()
        
        # Ayrı thread'de kamerayı oku
        self.frame = None
        self.frame_lock = threading.Lock()
        self.cam_thread = threading.Thread(target=self.camera_loop, daemon=True)
        self.cam_thread.start()
        
        # Ana işleme timer'ı (5 FPS analiz)
        self.create_timer(0.2, self.process_frame)
        
        self.get_logger().info('Vision Node Started - Connecting to Unity Camera...')

    def load_models(self):
        """YOLO ve OCR modellerini arka planda yükle"""
        try:
            from ultralytics import YOLO
            self.yolo_model = YOLO('yolov8n.pt')  # 6MB nano model, otomatik indirir
            self.get_logger().info('YOLOv8n model loaded!')
        except Exception as e:
            self.get_logger().warn(f'YOLO load failed (will use color-only): {e}')
            self.yolo_model = None
        
        try:
            import easyocr
            self.ocr_reader = easyocr.Reader(['en'], gpu=False)
            self.get_logger().info('EasyOCR loaded!')
        except Exception as e:
            self.get_logger().warn(f'EasyOCR load failed (will use contour-only): {e}')
            self.ocr_reader = None
        
        self.models_loaded = True
        self.get_logger().info('=== ALL VISION MODELS READY ===')

    def state_callback(self, msg):
        self.current_state = msg.data

    def camera_loop(self):
        """Unity MJPEG stream'ini sürekli oku"""
        while True:
            try:
                cap = cv2.VideoCapture(self.stream_url)
                if not cap.isOpened():
                    self.get_logger().warn('Camera not available, retrying in 3s...')
                    time.sleep(3)
                    continue
                
                self.get_logger().info(f'Connected to Unity camera: {self.stream_url}')
                
                while cap.isOpened():
                    ret, frame = cap.read()
                    if not ret:
                        break
                    with self.frame_lock:
                        self.frame = frame
                
                cap.release()
            except Exception as e:
                self.get_logger().warn(f'Camera error: {e}')
            
            time.sleep(2)  # Reconnect delay

    def process_frame(self):
        """Ana tespit döngüsü"""
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

        # 1. STOP TABELASi TESPİTİ (Kırmızı renk + YOLO)
        stop_detected = self.detect_stop_sign(frame)
        if stop_detected:
            self.publish_detection("STOP", 0.95)
            self.last_detection_time = now
            return

        # 2. SAYI TESPİTİ (Mavi/Yeşil dikdörtgen tabela + OCR)
        number = self.detect_number(frame)
        if number is not None:
            self.publish_detection("NEXT_STAGE", 0.90)
            self.last_detection_time = now
            return

    def detect_stop_sign(self, frame):
        """Stop tabelası tespiti: YOLO + Kırmızı renk analizi"""
        
        # Yöntem 1: YOLO (en güvenilir)
        if self.yolo_model is not None:
            try:
                results = self.yolo_model(frame, conf=0.5, verbose=False)
                for r in results:
                    for box in r.boxes:
                        cls_id = int(box.cls[0])
                        cls_name = self.yolo_model.names[cls_id]
                        if cls_name == "stop sign":
                            conf = float(box.conf[0])
                            self.get_logger().warn(f'>>> STOP SIGN DETECTED (YOLO) conf={conf:.2f} <<<')
                            return True
            except Exception as e:
                self.get_logger().warn(f'YOLO error: {e}')
        
        # Yöntem 2: Renk bazlı (YOLO çalışmazsa yedek)
        hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
        
        # Kırmızı renk maskesi (iki aralık - kırmızı HSV'de sarmalanır)
        mask1 = cv2.inRange(hsv, np.array([0, 100, 100]), np.array([10, 255, 255]))
        mask2 = cv2.inRange(hsv, np.array([160, 100, 100]), np.array([180, 255, 255]))
        red_mask = mask1 | mask2
        
        # Gürültü temizle
        kernel = np.ones((5, 5), np.uint8)
        red_mask = cv2.morphologyEx(red_mask, cv2.MORPH_OPEN, kernel)
        red_mask = cv2.morphologyEx(red_mask, cv2.MORPH_CLOSE, kernel)
        
        contours, _ = cv2.findContours(red_mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        
        for cnt in contours:
            area = cv2.contourArea(cnt)
            if area > 2000:  # Minimum alan (piksel²)
                # Sekizgen mi kontrol et (stop tabelası)
                peri = cv2.arcLength(cnt, True)
                approx = cv2.approxPolyDP(cnt, 0.04 * peri, True)
                if 6 <= len(approx) <= 10:  # Sekizgene yakın
                    self.get_logger().warn(f'>>> STOP SIGN DETECTED (Color) area={area:.0f} <<<')
                    return True
        
        return False

    def detect_number(self, frame):
        """Etap numarası tespiti: Mavi/Beyaz tabela içindeki sayı"""
        hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
        
        # Mavi renk maskesi (etap tabelaları genelde mavi)
        blue_mask = cv2.inRange(hsv, np.array([100, 80, 80]), np.array([130, 255, 255]))
        
        # Beyaz renk maskesi (alternatif tabela rengi)
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
                
                # Dikdörtgene yakın şekiller (tabela)
                if 0.3 < aspect < 3.0:
                    roi = frame[y:y+h, x:x+w]
                    
                    if self.ocr_reader is not None:
                        try:
                            results = self.ocr_reader.readtext(roi, allowlist='0123456789')
                            for (_, text, conf) in results:
                                text = text.strip()
                                if text.isdigit() and 1 <= int(text) <= 6:
                                    self.get_logger().warn(f'>>> NUMBER DETECTED: {text} (conf={conf:.2f}) <<<')
                                    return int(text)
                        except Exception as e:
                            self.get_logger().warn(f'OCR error: {e}')
        
        return None

    def publish_detection(self, sign_type, confidence):
        msg = SignDetection()
        msg.type = sign_type
        msg.distance = 2.0
        msg.confidence = confidence
        self.pub_sign.publish(msg)
        self.get_logger().info(f'Published: type={sign_type}, conf={confidence}')

def main(args=None):
    rclpy.init(args=args)
    node = VisionNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
PYEOF

# Rebuild vision package
echo "  > Rebuilding ika_vision..."
cd "$WS_DIR"
rm -rf build/ika_vision install/ika_vision
if [ -f /opt/ros/jazzy/setup.bash ]; then source /opt/ros/jazzy/setup.bash; else source /opt/ros/humble/setup.bash; fi
source install/setup.bash 2>/dev/null || true
colcon build --packages-select ika_vision

echo "============================================"
echo "  REAL VISION DETECTOR DEPLOYED! ✅          "
echo "============================================"
