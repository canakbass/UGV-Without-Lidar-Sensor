#!/bin/bash
set -e
WS_DIR=~/ika_ws
TEMPLATE_DIR="$WS_DIR/sign_templates"

echo "============================================"
echo "  DETECTOR v13 - REAL TEMPLATE MATCHING      "
echo "============================================"

# ═══ 1. Gerçek tabela texture'larını kopyala ═══
mkdir -p "$TEMPLATE_DIR"

# Unity'deki gerçek tabela görsellerini WSL'e kopyala
PARTS_DIR="/mnt/c/projeler/Projects/unityProjects/arac/Assets/Parts3d"
for f in 1.png 2.png 3.png 4.png 5.png 6.png "7 1.png" 8.png 9.png 10.png stop.png hedef.png 11.png "11.2.png"; do
    if [ -f "$PARTS_DIR/$f" ]; then
        # Dosya adını normalize et
        dest=$(echo "$f" | sed 's/ /_/g')
        cp "$PARTS_DIR/$f" "$TEMPLATE_DIR/$dest"
        echo "  Copied: $f → $dest"
    fi
done

# ═══ 2. Template'leri işle (crop + resize) ═══
python3 << 'PYEOF'
import cv2
import numpy as np
import os, glob, math

tdir = os.path.expanduser("~/ika_ws/sign_templates")
outdir = os.path.join(tdir, "processed")
os.makedirs(outdir, exist_ok=True)

for f in sorted(glob.glob(os.path.join(tdir, "*.png"))):
    name = os.path.splitext(os.path.basename(f))[0]
    img = cv2.imread(f)
    if img is None:
        continue
    
    # Tabelanın iç kırmızı dairesini bul
    hsv = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)
    m1 = cv2.inRange(hsv, np.array([0, 100, 100]), np.array([10, 255, 255]))
    m2 = cv2.inRange(hsv, np.array([160, 100, 100]), np.array([180, 255, 255]))
    red = m1 | m2
    
    contours, _ = cv2.findContours(red, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    if not contours:
        print(f"  SKIP {name}: no red contour")
        continue
    
    # En büyük kırmızı kontur = tabela çemberi
    cnt = max(contours, key=cv2.contourArea)
    (cx, cy), radius = cv2.minEnclosingCircle(cnt)
    cx, cy, r = int(cx), int(cy), int(radius)
    
    # İç kısmı kırp (kırmızı halka hariç, beyaz+siyah)
    ir = int(r * 0.65)  # Kırmızı halkanın iç çapı
    x1, y1 = max(0, cx-ir), max(0, cy-ir)
    x2, y2 = min(img.shape[1], cx+ir), min(img.shape[0], cy+ir)
    roi = img[y1:y2, x1:x2]
    
    # 64x64'e yeniden boyutlandır (standart)
    resized = cv2.resize(roi, (64, 64))
    
    # Grayscale + binary (siyah rakam beyaz zemin)
    gray = cv2.cvtColor(resized, cv2.COLOR_BGR2GRAY)
    _, binary = cv2.threshold(gray, 0, 255, cv2.THRESH_BINARY_INV + cv2.THRESH_OTSU)
    
    cv2.imwrite(os.path.join(outdir, f"{name}_color.png"), resized)
    cv2.imwrite(os.path.join(outdir, f"{name}_binary.png"), binary)
    print(f"  Processed: {name} (r={r}, roi={roi.shape})")

print(f"\nTemplates in {outdir}:")
for f in sorted(os.listdir(outdir)):
    print(f"  {f}")
PYEOF

# ═══ 3. Detector v13 ═══
cat > "$WS_DIR/src/ika_robot/ika_vision/ika_vision/detector_node.py" << 'PYEOF'
"""
Tabela Algılama v13 - GERÇEK TEMPLATE MATCHING + DEBUG OVERLAY

Yaklaşım:
  1. Frame'de kırmızı daire ARA (ön filtre)
  2. En beyaz iç kısma sahip adayı seç (bariyer filtresi)
  3. ROI'yi 64x64'e resize et
  4. GERÇEK tabela şablonlarıyla karşılaştır (template matching)
  5. En yüksek eşleşme skoru → okunan numara
  6. Beklenen etap numarasıyla karşılaştır
  7. Debug: her frame'e bounding box + okunan sayı çiz, kaydet

Labeling: Her tespit/ret /tmp/detector_debug/ altına kaydedilir
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

STAGE_ORDER = [
    ("BASLA", "1"),
    ("TASLI_YOL", "2"),
    ("YAN_EGIM", "3"),
    ("DIK_ENGEL", "4"),
    ("TRAFIK_KONILERI", "5"),
    ("KAYAR_ENGEL", "6"),
    ("ENGEBELI_ARAZI", "7_1"),
    ("DIK_EGIM_CIKIS", "8"),
]

class VisionDetector(Node):
    def __init__(self):
        super().__init__('vision_node')
        
        self.declare_parameter('camera_source', 'ros_topic')
        self.declare_parameter('local_camera_id', 0)
        self.declare_parameter('cooldown', 3.0)
        self.declare_parameter('min_radius', 15)
        self.declare_parameter('match_threshold', 0.35)
        self.declare_parameter('template_dir',
            os.path.expanduser('~/ika_ws/sign_templates/processed'))
        
        self.camera_source = self.get_parameter('camera_source').value
        self.cooldown = self.get_parameter('cooldown').value
        self.min_r = self.get_parameter('min_radius').value
        self.match_thresh = self.get_parameter('match_threshold').value
        template_dir = self.get_parameter('template_dir').value
        
        self.current_state = "IDLE"
        self.expected_sign = "1"
        self.last_detection_time = 0
        self.frame_count = 0
        self.detection_count = 0
        
        self.save_dir = "/tmp/detector_debug"
        os.makedirs(self.save_dir, exist_ok=True)
        
        # Template'leri yükle
        self.templates = {}
        self.load_templates(template_dir)
        
        self.pub_sign = self.create_publisher(SignDetection, '/vision/sign', 10)
        self.pub_debug = self.create_publisher(String, '/vision/debug', 10)
        self.create_subscription(String, '/robot_state', self.state_cb, 10)
        
        if self.camera_source == 'ros_topic':
            self.create_subscription(String, '/camera/image_base64', self.image_cb, 10)
        else:
            cam_id = self.get_parameter('local_camera_id').value
            self.cap = cv2.VideoCapture(cam_id)
            self.create_timer(0.2, self.local_camera_loop)
        
        self.create_timer(10.0, self.print_stats)
        self.get_logger().info(f'=== DETECTOR v13 (REAL TEMPLATES) ===')
        self.get_logger().info(f'  Templates: {list(self.templates.keys())}')
        self.get_logger().info(f'  Threshold: {self.match_thresh}')

    def load_templates(self, tdir):
        if not os.path.exists(tdir):
            self.get_logger().error(f'Template dir not found: {tdir}')
            return
        for f in sorted(glob.glob(os.path.join(tdir, "*_binary.png"))):
            name = os.path.basename(f).replace("_binary.png", "")
            t = cv2.imread(f, cv2.IMREAD_GRAYSCALE)
            if t is not None:
                self.templates[name] = t
                self.get_logger().info(f'  Template: {name} ({t.shape})')

    def state_cb(self, msg):
        old = self.current_state
        self.current_state = msg.data
        if old != self.current_state:
            self.get_logger().info(f'State: {old} → {self.current_state}')
            for stage, sign in STAGE_ORDER:
                if stage == self.current_state:
                    self.expected_sign = sign
                    self.get_logger().info(f'  Expecting sign: "{sign}"')
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
            f'expected="{self.expected_sign}"'
        )

    def process_frame(self, frame, now):
        # Debug: annotated frame
        debug_frame = frame.copy()
        
        result = self.detect_and_match(frame, debug_frame)
        
        # Her 30 frame'de debug kaydet
        if self.frame_count % 30 == 0:
            path = f'{self.save_dir}/dbg_{self.frame_count}.jpg'
            cv2.imwrite(path, debug_frame)
        
        if result is not None:
            sign_type, confidence, matched_name = result
            self.publish_detection(sign_type, confidence)
            self.last_detection_time = now
            # Tespit frame'ini kaydet  
            path = f'{self.save_dir}/det_{self.detection_count}_{matched_name}.jpg'
            cv2.imwrite(path, debug_frame)
            # Debug mesajı publish et (web UI'da göster)
            dbg = String()
            dbg.data = f'{matched_name}|{confidence:.0%}|{sign_type}'
            self.pub_debug.publish(dbg)
            return
        
        if self.current_state == "PLATFORM_ATIS":
            if self.detect_target_sign(frame):
                self.publish_detection("FINISH", 0.90)
                self.last_detection_time = now

    def detect_and_match(self, frame, debug_frame):
        hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
        m1 = cv2.inRange(hsv, np.array([0, 70, 70]), np.array([12, 255, 255]))
        m2 = cv2.inRange(hsv, np.array([160, 70, 70]), np.array([180, 255, 255]))
        red_mask = m1 | m2
        
        k = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (5, 5))
        red_mask = cv2.morphologyEx(red_mask, cv2.MORPH_CLOSE, k, iterations=2)
        red_mask = cv2.morphologyEx(red_mask, cv2.MORPH_OPEN, k, iterations=1)
        
        contours, _ = cv2.findContours(red_mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        h, w = frame.shape[:2]
        
        # Adayları topla
        candidates = []
        for cnt in contours:
            area = cv2.contourArea(cnt)
            if area < 150 or area > 80000:
                continue
            peri = cv2.arcLength(cnt, True)
            if peri == 0:
                continue
            circ = (4 * math.pi * area) / (peri * peri)
            if circ < 0.40:
                continue
            (cx, cy), radius = cv2.minEnclosingCircle(cnt)
            r = int(radius)
            if r < self.min_r:
                continue
            candidates.append((int(cx), int(cy), r, circ))
        
        if not candidates:
            return None
        
        # Tüm adayların iç kısmını kontrol et
        best_match = None
        best_score = -1
        best_info = None
        
        for cx, cy, r, circ in candidates:
            # İç ROI al
            ir = int(r * 0.60)
            ix1, iy1 = max(0, cx-ir), max(0, cy-ir)
            ix2, iy2 = min(w, cx+ir), min(h, cy+ir)
            roi = frame[iy1:iy2, ix1:ix2]
            if roi.size == 0 or roi.shape[0] < 6 or roi.shape[1] < 6:
                continue
            
            # İç beyazlık kontrolü (min %12 - bariyerleri filtrele)
            hsv_roi = cv2.cvtColor(roi, cv2.COLOR_BGR2HSV)
            white = cv2.inRange(hsv_roi, np.array([0, 0, 160]), np.array([180, 50, 255]))
            wr = cv2.countNonZero(white) / (white.shape[0] * white.shape[1])
            
            if wr < 0.12:
                # Debug: bariyer olarak işaretle
                cv2.circle(debug_frame, (cx, cy), r, (0, 0, 255), 1)
                continue
            
            # Template matching
            gray = cv2.cvtColor(roi, cv2.COLOR_BGR2GRAY)
            _, binary = cv2.threshold(gray, 0, 255, 
                cv2.THRESH_BINARY_INV + cv2.THRESH_OTSU)
            resized = cv2.resize(binary, (64, 64))
            
            match_name, match_score = self.match_templates(resized)
            
            # Debug: aday olarak işaretle
            color = (0, 255, 255)  # Sarı
            cv2.circle(debug_frame, (cx, cy), r, color, 2)
            label = f'{match_name}:{match_score:.0%}' if match_name else '?'
            cv2.putText(debug_frame, label, (cx-r, cy-r-5),
                cv2.FONT_HERSHEY_SIMPLEX, 0.5, color, 1)
            
            if match_name and match_score > best_score:
                best_score = match_score
                best_match = match_name
                best_info = (cx, cy, r, wr, match_score)
        
        if best_match is None or best_score < self.match_thresh:
            return None
        
        cx, cy, r, wr, score = best_info
        
        self.get_logger().info(
            f'MATCH: "{best_match}" score={score:.0%} '
            f'at ({cx},{cy}) r={r} white={wr:.0%}'
        )
        
        # Debug: yeşil = onaylı
        cv2.circle(debug_frame, (cx, cy), r, (0, 255, 0), 3)
        cv2.putText(debug_frame, f'>>> {best_match} <<<', (cx-r, cy-r-15),
            cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 0), 2)
        
        # STOP mi?
        if best_match == "stop":
            self.get_logger().warn(f'>>> STOP (score={score:.0%}) <<<')
            return ("STOP", 0.95, best_match)
        
        # Beklenen sayıyla karşılaştır
        # Template isimleri: "1", "2", ..., "8", "9", "10", "7_1"
        # Expected: "1", "2", ..., "8"
        expected_num = self.expected_sign.replace("_1", "")
        match_num = best_match.replace("_1", "")
        
        try:
            exp_n = int(expected_num)
            mat_n = int(match_num)
        except ValueError:
            return None
        
        if mat_n == exp_n or mat_n == exp_n + 1:
            self.get_logger().warn(
                f'>>> SIGN "{best_match}" matches expected "{self.expected_sign}" '
                f'(score={score:.0%}) → NEXT_STAGE <<<'
            )
            return ("NEXT_STAGE", score, best_match)
        else:
            self.get_logger().info(
                f'  "{best_match}" != expected "{self.expected_sign}", skip'
            )
            return None
    
    def match_templates(self, binary_roi):
        """64x64 binary ROI'yi tüm şablonlarla karşılaştır"""
        if len(self.templates) == 0:
            return None, 0
        
        best_name = None
        best_score = -1
        
        for name, template in self.templates.items():
            # Template'i ROI boyutuna uyarla
            t_resized = cv2.resize(template, (64, 64))
            
            # Normalize cross-correlation
            result = cv2.matchTemplate(binary_roi, t_resized, cv2.TM_CCOEFF_NORMED)
            score = result[0][0]
            
            if score > best_score:
                best_score = score
                best_name = name
        
        return best_name, best_score

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
                    self.get_logger().warn(f'>>> TARGET <<<')
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

# BUILD
cd "$WS_DIR"
rm -rf build/ika_vision install/ika_vision
if [ -f /opt/ros/jazzy/setup.bash ]; then source /opt/ros/jazzy/setup.bash; else source /opt/ros/humble/setup.bash; fi
source install/setup.bash 2>/dev/null || true
colcon build --packages-select ika_vision

echo "============================================"
echo "  DETECTOR v13 (REAL TEMPLATES) DONE!        "
echo "  Debug images:  /tmp/detector_debug/        "
echo "  Templates:     ~/ika_ws/sign_templates/    "
echo "============================================"
