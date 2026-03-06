#!/bin/bash
set -e
WS_DIR=~/ika_ws

echo "=== CRITICAL FIX: PID + MIN SIZE ==="

# ═══ 1. Controller: PID yönünü düzelt ═══
cat > "$WS_DIR/src/ika_robot/ika_control/ika_control/controller_node.py" << 'PYEOF'
"""
Controller v4.2 - KANIT BAZLI PID DÜZELTMESİ

KANITLAR (loglardan):
  FL=0.54 (sol duvar yakın) FR=1.83 (sağ uzak)
  PID: error = fr - fl = +1.29 → steering = +0.5
  Robot SOLA gidiyor → sol duvara çarpıyor!
  
  Sonuç: Pozitif angular.z = SOLA dönüş (Unity'de)
  Düzeltme: error = fl - fr 
    FL=0.54 FR=1.83 → error = -1.29 → steering = -0.5 → SAĞA dön ✓
"""

import rclpy
from rclpy.node import Node
from std_msgs.msg import String, Float32MultiArray
from geometry_msgs.msg import Twist

STATE_SPEEDS = {
    "BASLA": 0.4,
    "TASLI_YOL": 0.35,
    "YAN_EGIM": 0.25,
    "DIK_ENGEL": 0.25,
    "TRAFIK_KONILERI": 0.20,
    "KAYAR_ENGEL": 0.25,
    "ENGEBELI_ARAZI": 0.30,
    "DIK_EGIM_CIKIS": 0.25,
    "DIK_EGIM_INIS": 0.20,
}

STOP_STATES = ["IDLE", "MANUAL", "CIKIS_DURMA", "PLATFORM_ATIS", "INIS_DURMA"]

class ControllerNode(Node):
    def __init__(self):
        super().__init__('controller_node')
        
        self.state = "IDLE"
        self.us = [4.0] * 7
        self.us_received = False
        self.us_count = 0
        self.roll = 0.0
        self.pitch = 0.0
        
        self.cone_steer = 0.0
        self.wall_steer = 0.0
        
        self.kp = 1.5
        self.kd = 0.3
        self.prev_error = 0.0
        
        self.pub_cmd = self.create_publisher(Twist, '/cmd_vel', 10)
        self.create_subscription(String, '/robot_state', self.state_cb, 10)
        self.create_subscription(Float32MultiArray, '/sensors/ultrasonic', self.us_cb, 10)
        self.create_subscription(Float32MultiArray, '/unity/telemetry', self.telemetry_cb, 10)
        self.create_subscription(String, '/vision/cone_steer', self.cone_cb, 10)
        self.create_subscription(String, '/vision/wall_steer', self.wall_cb, 10)
        
        self.create_timer(0.1, self.control_loop)
        self.create_timer(5.0, self.debug_log)
        self.get_logger().info('Controller v4.2 (PID: fl-fr)')
    
    def state_cb(self, msg):
        old = self.state
        self.state = msg.data
        if old != self.state:
            self.get_logger().info(f'State: {old} → {self.state}')
            self.prev_error = 0.0
    
    def us_cb(self, msg):
        if msg.data and len(msg.data) >= 7:
            self.us = list(msg.data)
            self.us_received = True
            self.us_count += 1
    
    def telemetry_cb(self, msg):
        if msg.data and len(msg.data) >= 7:
            self.pitch = msg.data[5]
            self.roll = msg.data[6]
    
    def cone_cb(self, msg):
        try: self.cone_steer = float(msg.data)
        except: pass
    
    def wall_cb(self, msg):
        try: self.wall_steer = float(msg.data)
        except: pass
    
    def debug_log(self):
        self.get_logger().info(
            f'NAV: state={self.state}, '
            f'FC={self.us[0]:.2f} FL={self.us[1]:.2f} FR={self.us[2]:.2f} '
            f'CFL={self.us[3]:.2f} CFR={self.us[4]:.2f} '
            f'roll={self.roll:.1f}'
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
        
        if not self.us_received:
            cmd.linear.x = 0.12
            self.pub_cmd.publish(cmd)
            return
        
        fc = self.us[0]
        fl = self.us[1]
        fr = self.us[2]
        cfl = self.us[3]
        cfr = self.us[4]
        
        # ═══ ENGEL KAÇINMA ═══
        obstacle = False
        
        if fc < 0.35:
            speed = 0.08
            # Sol daha uzak → SOLA kaç (negatif = sağ, pozitif = sol... 
            # AMA pozitif angular.z = sol dönüş!)
            # Sol uzak → sola itmek = pozitif steering
            if fl > fr:
                steering = 0.7  # Sola kaç (fl daha uzak)
            else:
                steering = -0.7  # Sağa kaç (fr daha uzak)
            obstacle = True
        elif fc < 0.7:
            speed *= 0.4
            if fl > fr:
                steering = 0.45
            else:
                steering = -0.45
            obstacle = True
        elif cfl < 0.30:
            speed *= 0.5
            steering = -0.5  # Sol köşe yakın → SAĞA kaç
            obstacle = True
        elif cfr < 0.30:
            speed *= 0.5
            steering = 0.5  # Sağ köşe yakın → SOLA kaç
            obstacle = True
        
        if not obstacle:
            if self.state == "YAN_EGIM":
                steering = self.navigate_yan_egim()
            elif self.state == "TRAFIK_KONILERI":
                steering = self.navigate_cones()
            elif self.state == "KAYAR_ENGEL":
                steering = self.navigate_wall()
            else:
                steering = self.navigate_pid()
        
        cmd.linear.x = max(0.0, speed)
        cmd.angular.z = max(-0.8, min(0.8, steering))
        self.pub_cmd.publish(cmd)
    
    def navigate_pid(self):
        """
        PID merkezleme
        
        Pozitif angular.z = SOLA dönüş (Unity'de kanıtlandı)
        
        FL yakın → SAĞA dön (negatif steering)
        FR yakın → SOLA dön (pozitif steering)
        
        error = fl - fr:
          FL=0.54 FR=1.83 → error = -1.29 → steering = NEGATİF → SAĞA ✓
          FL=1.83 FR=0.54 → error = +1.29 → steering = POZİTİF → SOLA ✓
        """
        fl = self.us[1]
        fr = self.us[2]
        
        if fl > 3.5 and fr > 3.5:
            self.prev_error = 0.0
            return 0.0
        
        # Tek taraf max ise
        if fl > 3.5 and fr < 3.5:
            return 0.2  # Sol boş, sağ yakın → SOLA it
        if fr > 3.5 and fl < 3.5:
            return -0.2  # Sağ boş, sol yakın → SAĞA it
        
        error = fl - fr
        derivative = error - self.prev_error
        self.prev_error = error
        
        steering = self.kp * error + self.kd * derivative
        return max(-0.5, min(0.5, steering))
    
    def navigate_yan_egim(self):
        """3. Etap: Sağa yaslan + roll kompanzasyonu"""
        fl = self.us[1]
        fr = self.us[2]
        
        # Sağa yaslanma = sağ duvara yakın dur = negatif bias
        # (negatif angular.z = sağa dönüş)
        right_bias = -0.12
        
        roll_correction = 0.0
        if abs(self.roll) > 3.0:
            roll_correction = -self.roll * 0.015
        
        if fl > 3.5 and fr > 3.5:
            return right_bias + roll_correction
        
        error = fl - fr
        derivative = error - self.prev_error
        self.prev_error = error
        pid = self.kp * error + self.kd * derivative
        pid = max(-0.4, min(0.4, pid))
        
        return pid + right_bias + roll_correction
    
    def navigate_cones(self):
        if abs(self.cone_steer) > 0.1:
            return self.cone_steer * 0.5
        return self.navigate_pid()
    
    def navigate_wall(self):
        if abs(self.wall_steer) > 0.1:
            return self.wall_steer * 0.5
        return self.navigate_pid()

def main(args=None):
    rclpy.init(args=args)
    node = ControllerNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
PYEOF

# ═══ 2. Detector: MIN BOYUT + SADECE beklenen tabela ═══
cat > "$WS_DIR/src/ika_robot/ika_vision/ika_vision/detector_node.py" << 'PYEOF'
"""
Tabela Algılama v14.2

Düzeltmeler:
  - MIN BOYUT: sadece size >= 40px kabul (yakın tabela)
  - SADECE beklenen tabela ara (expected+1 kaldırıldı)
  - Cooldown 3s → 5s
  - Uzak tabelalar (24px) artık reddedilecek
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
}

class VisionDetector(Node):
    def __init__(self):
        super().__init__('vision_node')
        
        self.declare_parameter('camera_source', 'ros_topic')
        self.declare_parameter('cooldown', 5.0)
        self.declare_parameter('match_threshold', 0.50)
        self.declare_parameter('min_detect_size', 40)
        self.declare_parameter('template_dir',
            os.path.expanduser('~/ika_ws/sign_templates/multiscale'))
        
        self.camera_source = self.get_parameter('camera_source').value
        self.cooldown = self.get_parameter('cooldown').value
        self.match_thresh = self.get_parameter('match_threshold').value
        self.min_detect_size = self.get_parameter('min_detect_size').value
        tdir = self.get_parameter('template_dir').value
        
        self.current_state = "IDLE"
        self.expected_sign = "1"
        self.last_detection_time = 0
        self.frame_count = 0
        self.detection_count = 0
        
        self.save_dir = "/tmp/detector_debug"
        os.makedirs(self.save_dir, exist_ok=True)
        
        self.templates = {}
        self.load_templates(tdir)
        self.active_templates = {}
        self.update_active_templates()
        
        self.pub_sign = self.create_publisher(SignDetection, '/vision/sign', 10)
        self.pub_debug = self.create_publisher(String, '/vision/debug', 10)
        self.create_subscription(String, '/robot_state', self.state_cb, 10)
        
        if self.camera_source == 'ros_topic':
            self.create_subscription(String, '/camera/image_base64', self.image_cb, 10)
        
        self.create_timer(10.0, self.print_stats)
        self.get_logger().info(f'=== DETECTOR v14.2 ===')
        self.get_logger().info(f'  min_size={self.min_detect_size}px, cd={self.cooldown}s')
        self.get_logger().info(f'  Templates: {list(self.templates.keys())}')

    def load_templates(self, tdir):
        if not os.path.exists(tdir): return
        for f in sorted(glob.glob(os.path.join(tdir, "*.png"))):
            fname = os.path.basename(f)
            parts = fname.replace(".png", "").rsplit("_", 1)
            if len(parts) != 2: continue
            name, size_str = parts
            try: sz = int(size_str)
            except: continue
            t = cv2.imread(f)
            if t is None: continue
            if name not in self.templates:
                self.templates[name] = []
            self.templates[name].append((sz, t))
        for name in self.templates:
            self.templates[name].sort(key=lambda x: x[0])

    def update_active_templates(self):
        """SADECE beklenen + stop (expected+1 YOK!)"""
        self.active_templates = {}
        if "stop" in self.templates:
            self.active_templates["stop"] = self.templates["stop"]
        if self.expected_sign in self.templates:
            self.active_templates[self.expected_sign] = self.templates[self.expected_sign]
        self.get_logger().info(f'  Active: {list(self.active_templates.keys())}')

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
        if self.current_state in ["IDLE", "MANUAL"]: return
        now = time.time()
        if now - self.last_detection_time < self.cooldown: return
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

    def print_stats(self):
        self.get_logger().info(
            f'Stats: state={self.current_state}, '
            f'frames={self.frame_count}, OK={self.detection_count}, '
            f'expected="{self.expected_sign}"'
        )

    def process_frame(self, frame, now):
        result = self.search_signs(frame)
        if result is not None:
            sign_name, score, loc, size = result
            sign_type = "STOP" if sign_name == "stop" else "NEXT_STAGE"
            
            debug = frame.copy()
            x, y = loc
            cv2.rectangle(debug, (x, y), (x+size, y+size), (0, 255, 0), 3)
            cv2.putText(debug, f'{sign_name} ({score:.0%}) {size}px',
                (x, y-10), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 0), 2)
            path = f'{self.save_dir}/det_{self.detection_count+1}_{sign_name}.jpg'
            cv2.imwrite(path, debug)
            
            self.publish_detection(sign_type, score)
            self.last_detection_time = now
        
        if self.frame_count % 50 == 0:
            cv2.imwrite(f'{self.save_dir}/frame_{self.frame_count}.jpg', frame)

    def search_signs(self, frame):
        best_name = None
        best_score = 0
        best_loc = None
        best_size = 0
        
        for name, scale_list in self.active_templates.items():
            for sz, template in scale_list:
                # ═══ MIN BOYUT FİLTRESİ ═══
                if sz < self.min_detect_size:
                    continue  # 24px, 32px → atla (çok uzak)
                
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
        elif best_score > 0.35:
            self.get_logger().info(
                f'  weak: "{best_name}" score={best_score:.0%} sz={best_size}'
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
rm -rf build/ika_vision install/ika_vision build/ika_control install/ika_control
if [ -f /opt/ros/jazzy/setup.bash ]; then source /opt/ros/jazzy/setup.bash; else source /opt/ros/humble/setup.bash; fi
source install/setup.bash 2>/dev/null || true
colcon build --packages-select ika_vision ika_control

echo "=== CRITICAL FIX DONE ==="
echo "  PID: fl-fr (sol yakın → sağa dön)"
echo "  MIN SIZE: >=40px (uzak tabela reddedilir)"
echo "  COOLDOWN: 5 saniye"
echo "  ACTIVE: sadece beklenen + stop"
