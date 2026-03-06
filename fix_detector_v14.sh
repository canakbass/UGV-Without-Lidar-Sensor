#!/bin/bash
set -e
WS_DIR=~/ika_ws

echo "============================================"
echo "  DETECTOR v14 - FULL FRAME TEMPLATE MATCH   "
echo "============================================"

# ═══ 1. Çok ölçekli şablonlar oluştur ═══
python3 << 'PYEOF'
import cv2
import numpy as np
import os, glob

src_dir = os.path.expanduser("~/ika_ws/sign_templates")
out_dir = os.path.expanduser("~/ika_ws/sign_templates/multiscale")
os.makedirs(out_dir, exist_ok=True)

# Her tabela texture'ı için farklı boyutlarda template oluştur
# Kameradan görünecek olası boyutlar: 20px - 120px
sizes = [24, 32, 48, 64, 80, 100]

for f in sorted(glob.glob(os.path.join(src_dir, "*.png"))):
    name = os.path.splitext(os.path.basename(f))[0]
    img = cv2.imread(f)
    if img is None:
        continue
    
    # Kare olarak kırp (tabela daire, kare frame'e sığdır)
    h, w = img.shape[:2]
    s = min(h, w)
    cx, cy = w//2, h//2
    crop = img[cy-s//2:cy+s//2, cx-s//2:cx+s//2]
    
    for sz in sizes:
        resized = cv2.resize(crop, (sz, sz), interpolation=cv2.INTER_AREA)
        outpath = os.path.join(out_dir, f"{name}_{sz}.png")
        cv2.imwrite(outpath, resized)
    
    print(f"  {name}: {len(sizes)} scales")

print(f"\nTotal templates: {len(sizes) * len(glob.glob(os.path.join(src_dir, '*.png')))}")
PYEOF

# ═══ 2. Detector v14 ═══
cat > "$WS_DIR/src/ika_robot/ika_vision/ika_vision/detector_node.py" << 'PYEOF'
"""
Tabela Algılama v14 - FULL FRAME MULTI-SCALE TEMPLATE MATCHING

TAMAMEN FARKLI yaklaşım:
  - Kırmızı daire ARAMIYORUZ
  - Tüm frame'de doğrudan tabela texture'larını arıyoruz
  - Çok ölçekli: 24px, 32px, 48px, 64px, 80px, 100px
  - Her frame'de HER template'i HER ölçekte ara
  - En yüksek eşleşme → tespit

Bu neden daha iyi:
  - Bariyerler tabela görüntüsüne benzemiyor
  - Kırmızı filtre yok = bariyerlerle karışma yok
  - Gerçek tabela texture'unu birebir arıyoruz
"""

import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from ika_interfaces.msg import SignDetection
import cv2
import numpy as np
import time
import base64
import os
import glob

STAGE_MAP = {
    "BASLA": "1",
    "TASLI_YOL": "2",
    "YAN_EGIM": "3",
    "DIK_ENGEL": "4",
    "TRAFIK_KONILERI": "5",
    "KAYAR_ENGEL": "6",
    "ENGEBELI_ARAZI": "7_1",
    "DIK_EGIM_CIKIS": "8",
    "CIKIS_DURMA": "9",
    "PLATFORM_ATIS": "10",
    "INIS_DURMA": "stop",
}

class VisionDetector(Node):
    def __init__(self):
        super().__init__('vision_node')
        
        self.declare_parameter('camera_source', 'ros_topic')
        self.declare_parameter('local_camera_id', 0)
        self.declare_parameter('cooldown', 3.0)
        self.declare_parameter('match_threshold', 0.55)
        self.declare_parameter('template_dir',
            os.path.expanduser('~/ika_ws/sign_templates/multiscale'))
        
        self.camera_source = self.get_parameter('camera_source').value
        self.cooldown = self.get_parameter('cooldown').value
        self.match_thresh = self.get_parameter('match_threshold').value
        tdir = self.get_parameter('template_dir').value
        
        self.current_state = "IDLE"
        self.expected_sign = "1"
        self.last_detection_time = 0
        self.frame_count = 0
        self.detection_count = 0
        
        self.save_dir = "/tmp/detector_debug"
        os.makedirs(self.save_dir, exist_ok=True)
        
        # Template'leri yükle: {isim: [(boyut, template), ...]}
        self.templates = {}
        self.load_templates(tdir)
        
        # Sadece beklenen + komşu template'leri aktif tut (hız için)
        self.active_templates = {}
        self.update_active_templates()
        
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
        self.get_logger().info(f'=== DETECTOR v14 (FULL FRAME SEARCH) ===')
        self.get_logger().info(f'  Signs: {list(self.templates.keys())}')
        self.get_logger().info(f'  Threshold: {self.match_thresh}')

    def load_templates(self, tdir):
        if not os.path.exists(tdir):
            self.get_logger().error(f'Template dir missing: {tdir}')
            return
        for f in sorted(glob.glob(os.path.join(tdir, "*.png"))):
            fname = os.path.basename(f)
            parts = fname.replace(".png", "").rsplit("_", 1)
            if len(parts) != 2:
                continue
            name, size_str = parts
            try:
                sz = int(size_str)
            except ValueError:
                continue
            t = cv2.imread(f)
            if t is None:
                continue
            if name not in self.templates:
                self.templates[name] = []
            self.templates[name].append((sz, t))
        
        for name in self.templates:
            self.templates[name].sort(key=lambda x: x[0])
            sizes = [s for s, _ in self.templates[name]]
            self.get_logger().info(f'  {name}: sizes={sizes}')

    def update_active_templates(self):
        """Sadece beklenen + STOP template'lerini aktif tut"""
        self.active_templates = {}
        
        # STOP her zaman aktif
        if "stop" in self.templates:
            self.active_templates["stop"] = self.templates["stop"]
        
        # Beklenen sayı
        if self.expected_sign in self.templates:
            self.active_templates[self.expected_sign] = self.templates[self.expected_sign]
        
        # +1 sonraki (kaçırma durumu)
        try:
            n = int(self.expected_sign.replace("_1", ""))
            next_names = [str(n+1), f"{n+1}_1"]
            for nn in next_names:
                if nn in self.templates:
                    self.active_templates[nn] = self.templates[nn]
        except ValueError:
            pass
        
        self.get_logger().info(f'  Active templates: {list(self.active_templates.keys())}')

    def state_cb(self, msg):
        old = self.current_state
        self.current_state = msg.data
        if old != self.current_state:
            self.get_logger().info(f'State: {old} → {self.current_state}')
            if self.current_state in STAGE_MAP:
                self.expected_sign = STAGE_MAP[self.current_state]
                self.get_logger().info(f'  Expecting: "{self.expected_sign}"')
                self.update_active_templates()

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
            f'expected="{self.expected_sign}", '
            f'active={list(self.active_templates.keys())}'
        )

    def process_frame(self, frame, now):
        result = self.search_signs(frame)
        
        if result is not None:
            sign_name, score, loc, size = result
            sign_type = "STOP" if sign_name == "stop" else "NEXT_STAGE"
            
            # Debug frame kaydet
            debug = frame.copy()
            x, y = loc
            cv2.rectangle(debug, (x, y), (x+size, y+size), (0, 255, 0), 3)
            cv2.putText(debug, f'{sign_name} ({score:.0%})',
                (x, y-10), cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0, 255, 0), 2)
            path = f'{self.save_dir}/det_{self.detection_count+1}_{sign_name}.jpg'
            cv2.imwrite(path, debug)
            
            self.publish_detection(sign_type, score)
            self.last_detection_time = now
            
            # Debug topic
            dbg = String()
            dbg.data = f'{sign_name}|{score:.0%}|{sign_type}'
            self.pub_debug.publish(dbg)
            return
        
        # Her 50 frame'de debug kaydet
        if self.frame_count % 50 == 0:
            path = f'{self.save_dir}/frame_{self.frame_count}.jpg'
            cv2.imwrite(path, frame)

    def search_signs(self, frame):
        """Frame boyunca tüm aktif template'leri ara"""
        best_name = None
        best_score = 0
        best_loc = None
        best_size = 0
        
        for name, scale_list in self.active_templates.items():
            for sz, template in scale_list:
                # Template frame'den büyükse atla
                if sz > frame.shape[0] or sz > frame.shape[1]:
                    continue
                
                result = cv2.matchTemplate(frame, template, cv2.TM_CCOEFF_NORMED)
                _, max_val, _, max_loc = cv2.minMaxLoc(result)
                
                if max_val > best_score:
                    best_score = max_val
                    best_name = name
                    best_loc = max_loc
                    best_size = sz
        
        if best_score >= self.match_thresh:
            self.get_logger().warn(
                f'FOUND: "{best_name}" score={best_score:.0%} '
                f'at {best_loc} size={best_size}px'
            )
            return (best_name, best_score, best_loc, best_size)
        elif best_score > 0.3:
            # Yakın ama yetersiz → logla
            self.get_logger().info(
                f'  weak: "{best_name}" score={best_score:.0%} '
                f'at {best_loc} sz={best_size}'
            )
        
        return None

    def publish_detection(self, sign_type, confidence):
        self.detection_count += 1
        msg = SignDetection()
        msg.type = sign_type
        msg.distance = 2.0
        msg.confidence = float(confidence)
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
echo "  DETECTOR v14 DONE!                         "
echo "  Debug:  /tmp/detector_debug/               "
echo "============================================"
