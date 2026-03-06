#!/bin/bash
set -e
WS_DIR=~/ika_ws

echo "============================================"
echo "  CONTROLLER v4 + CONE/WALL DETECTION        "
echo "============================================"

# ═══ CONTROLLER v4 ═══
cat > "$WS_DIR/src/ika_robot/ika_control/ika_control/controller_node.py" << 'PYEOF'
"""
Navigasyon Kontrolcü v4

Düzeltmeler:
  - Sola yaslanma DÜZELTME: PID yön kontrolü düzeltildi
  - Etap bazlı hız ayarı
  - Stage 5 (TRAFIK_KONILERI): Koni algılama + ultrasonik kaçınma
  - Stage 6 (KAYAR_ENGEL): Hareketli duvar algılama
  - Ultrasonik engel kaçınma tüm etaplarda aktif
"""

import rclpy
from rclpy.node import Node
from std_msgs.msg import String, Float32MultiArray
from geometry_msgs.msg import Twist
import time

STATE_SPEEDS = {
    "BASLA": 0.4,
    "TASLI_YOL": 0.35,
    "YAN_EGIM": 0.36,
    "DIK_ENGEL": 0.25,
    "TRAFIK_KONILERI": 0.20,
    "KAYAR_ENGEL": 0.25,
    "ENGEBELI_ARAZI": 0.30,
    "DIK_EGIM_CIKIS": 0.5,
    "DIK_EGIM_INIS": 0.20,
}

STOP_STATES = ["IDLE", "MANUAL", "CIKIS_DURMA", "PLATFORM_ATIS", "INIS_DURMA"]

class ControllerNode(Node):
    def __init__(self):
        super().__init__('controller_node')
        
        self.state = "IDLE"
        self.us = [4.0] * 7  # [FC, FL, FR, CFL, CFR, CRL, CRR]
        self.us_received = False
        self.us_count = 0
        
        # Koni/duvar algılama verileri (detector'dan gelecek)
        self.cone_steer = 0.0  # -1=sola git, +1=sağa git
        self.wall_steer = 0.0
        
        # PID
        self.kp = 1.5
        self.kd = 0.3
        self.prev_error = 0.0
        
        self.pub_cmd = self.create_publisher(Twist, '/cmd_vel', 10)
        self.create_subscription(String, '/robot_state', self.state_cb, 10)
        self.create_subscription(Float32MultiArray, '/sensors/ultrasonic', self.us_cb, 10)
        self.create_subscription(String, '/vision/cone_steer', self.cone_cb, 10)
        self.create_subscription(String, '/vision/wall_steer', self.wall_cb, 10)
        
        self.create_timer(0.1, self.control_loop)  # 10 Hz
        self.create_timer(5.0, self.debug_log)
        
        self.get_logger().info('Navigation Controller v4')
    
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
    
    def cone_cb(self, msg):
        try:
            self.cone_steer = float(msg.data)
        except:
            pass
    
    def wall_cb(self, msg):
        try:
            self.wall_steer = float(msg.data)
        except:
            pass
    
    def debug_log(self):
        self.get_logger().info(
            f'NAV: state={self.state}, us_received={self.us_received}, '
            f'us_count={self.us_count}, '
            f'FC={self.us[0]:.2f} FL={self.us[1]:.2f} FR={self.us[2]:.2f} '
            f'CFL={self.us[3]:.2f} CFR={self.us[4]:.2f}'
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
        cfl = self.us[3]  # Köşe ön sol (45°)
        cfr = self.us[4]  # Köşe ön sağ (45°)
        
        if not self.us_received:
            speed = 0.12
            steering = 0.0
            cmd.linear.x = speed
            cmd.angular.z = steering
            self.pub_cmd.publish(cmd)
            return
        
        # ═══ ENGEL KAÇINMA (en yüksek öncelik, tüm state'lerde) ═══
        obstacle_override = False
        
        if fc < 0.35:
            # Çok yakın → sert dönüş
            speed = 0.08
            if fl > fr:
                steering = 0.7
            else:
                steering = -0.7
            obstacle_override = True
        elif fc < 0.7:
            speed *= 0.35
            if fl > fr:
                steering = 0.45
            else:
                steering = -0.45
            obstacle_override = True
        elif cfl < 0.35:
            speed *= 0.5
            steering = -0.4  # Sağa kaç
            obstacle_override = True
        elif cfr < 0.35:
            speed *= 0.5
            steering = 0.4  # Sola kaç
            obstacle_override = True
        
        if not obstacle_override:
            # ═══ ETAP BAZLI NAVİGASYON ═══
            if self.state == "TRAFIK_KONILERI":
                steering = self.navigate_cones(speed)
            elif self.state == "KAYAR_ENGEL":
                steering = self.navigate_moving_wall(speed)
            else:
                steering = self.navigate_pid()
        
        cmd.linear.x = max(0.0, speed)
        cmd.angular.z = max(-0.8, min(0.8, steering))
        self.pub_cmd.publish(cmd)
    
    def navigate_pid(self):
        """Normal PID merkezleme (bariyer arası)"""
        fl = self.us[1]
        fr = self.us[2]
        
        # Her ikisi de max range ise → düz git
        if fl > 3.5 and fr > 3.5:
            self.prev_error = 0.0
            return 0.0
        
        # Tek taraf max → o tarafa UZAK, diğerine YAKIN → yakın tarafa kaç
        if fl > 3.5 and fr < 3.5:
            return -0.3  # Sol boş, sağda bariyer → sola kaç
        if fr > 3.5 and fl < 3.5:
            return 0.3  # Sağ boş, solda bariyer → sağa kaç
        
        # İkisi de menzilde → PID merkezleme
        error = fl - fr  # Pozitif = sola yakın → sağa dön
        derivative = error - self.prev_error
        self.prev_error = error
        
        steering = self.kp * error + self.kd * derivative
        return max(-0.5, min(0.5, steering))
    
    def navigate_cones(self, speed):
        """5. Etap: Koni slalom - vision + ultrasonik"""
        # Koni algılamadan gelen yön bilgisi
        if abs(self.cone_steer) > 0.1:
            return self.cone_steer * 0.5
        
        # Fallback: ultrasonik PID
        return self.navigate_pid()
    
    def navigate_moving_wall(self, speed):
        """6. Etap: Hareketli duvar - vision + ultrasonik"""
        # Duvar algılamadan gelen yön
        if abs(self.wall_steer) > 0.1:
            return self.wall_steer * 0.5
        
        # Fallback: ultrasonik PID
        return self.navigate_pid()

def main(args=None):
    rclpy.init(args=args)
    node = ControllerNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
PYEOF

# ═══ DETECTOR v14.1 - Koni + Duvar algılama ekle ═══
# Mevcut detector'a koni ve duvar algılama ekle
cat > "$WS_DIR/src/ika_robot/ika_vision/ika_vision/obstacle_detector_node.py" << 'PYEOF'
"""
Engel Algılama Node - Koni + Hareketli Duvar

Stage 5 (TRAFIK_KONILERI): Turuncu konileri algıla
  - HSV turuncu filtreleme
  - Konilerin sol/sağ dağılımına göre yön ver

Stage 6 (KAYAR_ENGEL): Beyaz hareketli duvarı algıla
  - Büyük beyaz bölge tespiti
  - Frame-to-frame pozisyon takibi
  - Duvarın hangi tarafta olduğuna göre yön ver
"""

import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import cv2
import numpy as np
import base64
import time

class ObstacleDetector(Node):
    def __init__(self):
        super().__init__('obstacle_detector')
        
        self.current_state = "IDLE"
        self.prev_wall_cx = None
        self.frame_count = 0
        
        self.pub_cone = self.create_publisher(String, '/vision/cone_steer', 10)
        self.pub_wall = self.create_publisher(String, '/vision/wall_steer', 10)
        self.create_subscription(String, '/robot_state', self.state_cb, 10)
        self.create_subscription(String, '/camera/image_base64', self.image_cb, 10)
        
        self.create_timer(10.0, self.stats)
        self.get_logger().info('=== OBSTACLE DETECTOR (Cone + Wall) ===')
    
    def state_cb(self, msg):
        old = self.current_state
        self.current_state = msg.data
        if old != self.current_state:
            self.get_logger().info(f'State: {old} → {self.current_state}')
            self.prev_wall_cx = None
    
    def stats(self):
        self.get_logger().info(f'Obstacle: state={self.current_state}, frames={self.frame_count}')
    
    def image_cb(self, msg):
        if self.current_state not in ["TRAFIK_KONILERI", "KAYAR_ENGEL"]:
            return
        
        try:
            jpeg_bytes = base64.b64decode(msg.data)
            np_arr = np.frombuffer(jpeg_bytes, np.uint8)
            frame = cv2.imdecode(np_arr, cv2.IMREAD_COLOR)
            if frame is None:
                return
            self.frame_count += 1
        except:
            return
        
        if self.current_state == "TRAFIK_KONILERI":
            self.detect_cones(frame)
        elif self.current_state == "KAYAR_ENGEL":
            self.detect_moving_wall(frame)
    
    def detect_cones(self, frame):
        """Turuncu trafik konilerini algıla"""
        hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
        
        # Turuncu renk (koniler H=5-25, yüksek satürasyon)
        mask = cv2.inRange(hsv, np.array([5, 100, 100]), np.array([25, 255, 255]))
        
        k = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (5, 5))
        mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, k)
        mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, k)
        
        contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        
        h, w = frame.shape[:2]
        mid_x = w // 2
        
        left_cones = 0
        right_cones = 0
        left_area = 0
        right_area = 0
        
        for cnt in contours:
            area = cv2.contourArea(cnt)
            if area < 100:
                continue
            M = cv2.moments(cnt)
            if M["m00"] == 0:
                continue
            cx = int(M["m10"] / M["m00"])
            
            if cx < mid_x:
                left_cones += 1
                left_area += area
            else:
                right_cones += 1
                right_area += area
        
        total = left_cones + right_cones
        if total == 0:
            return
        
        # Koniler hangi tarafta daha yoğun → o tarafa GİTME
        if left_area > right_area * 1.3:
            steer = -0.4  # Sola koni var → sağa git
        elif right_area > left_area * 1.3:
            steer = 0.4   # Sağa koni var → sola git
        else:
            steer = 0.0   # Eşit → düz
        
        msg = String()
        msg.data = str(steer)
        self.pub_cone.publish(msg)
        
        if self.frame_count % 10 == 0:
            self.get_logger().info(
                f'CONES: L={left_cones}({left_area}) R={right_cones}({right_area}) → steer={steer:.1f}'
            )
    
    def detect_moving_wall(self, frame):
        """Beyaz hareketli duvarı algıla ve yönünü belirle"""
        hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
        
        # Beyaz bölge (duvar)
        white_mask = cv2.inRange(hsv, np.array([0, 0, 180]), np.array([180, 40, 255]))
        
        # Alt yarıya odaklan (duvar yol seviyesinde)
        h, w = frame.shape[:2]
        white_mask[:h//3, :] = 0  # Üst 1/3'ü görmezden gel (gökyüzü)
        
        k = cv2.getStructuringElement(cv2.MORPH_RECT, (10, 10))
        white_mask = cv2.morphologyEx(white_mask, cv2.MORPH_CLOSE, k)
        
        contours, _ = cv2.findContours(white_mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        
        if not contours:
            self.prev_wall_cx = None
            return
        
        # En büyük beyaz bölge = duvar
        biggest = max(contours, key=cv2.contourArea)
        area = cv2.contourArea(biggest)
        
        if area < 1000:
            self.prev_wall_cx = None
            return
        
        M = cv2.moments(biggest)
        if M["m00"] == 0:
            return
        
        wall_cx = int(M["m10"] / M["m00"])
        mid_x = w // 2
        
        # Duvar nerede? → ters taraftan geç
        if wall_cx < mid_x - 30:
            steer = -0.5  # Duvar solda → sağa git
        elif wall_cx > mid_x + 30:
            steer = 0.5   # Duvar sağda → sola git
        else:
            # Duvar ortada → hangi tarafa kayıyor?
            if self.prev_wall_cx is not None:
                dx = wall_cx - self.prev_wall_cx
                if dx > 3:
                    steer = 0.4  # Duvar sağa gidiyor → sola git (ters)
                elif dx < -3:
                    steer = -0.4  # Duvar sola gidiyor → sağa git
                else:
                    steer = 0.0
            else:
                steer = 0.0
        
        self.prev_wall_cx = wall_cx
        
        msg = String()
        msg.data = str(steer)
        self.pub_wall.publish(msg)
        
        if self.frame_count % 10 == 0:
            self.get_logger().info(
                f'WALL: cx={wall_cx} mid={mid_x} area={area} → steer={steer:.1f}'
            )

def main(args=None):
    rclpy.init(args=args)
    node = ObstacleDetector()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
PYEOF

# setup.py'de yeni node'u ekle
SETUP_FILE="$WS_DIR/src/ika_robot/ika_vision/setup.py"
if ! grep -q "obstacle_detector" "$SETUP_FILE" 2>/dev/null; then
    sed -i "s|'detector = ika_vision.detector_node:main',|'detector = ika_vision.detector_node:main',\n            'obstacle_detector = ika_vision.obstacle_detector_node:main',|" "$SETUP_FILE"
    echo "  Added obstacle_detector to setup.py"
fi

# Launch dosyasına ekle
LAUNCH_FILE="$WS_DIR/src/ika_robot/ika_bringup/launch/system.launch.py"
if ! grep -q "obstacle_detector" "$LAUNCH_FILE" 2>/dev/null; then
    sed -i "/Node(package='ika_vision', executable='detector'/a\\
        ),\\
        Node(\\
            package='ika_vision',\\
            executable='obstacle_detector',\\
            name='obstacle_detector'" "$LAUNCH_FILE"
    echo "  Added obstacle_detector to launch file"
fi

# BUILD
cd "$WS_DIR"
rm -rf build/ika_vision install/ika_vision build/ika_control install/ika_control
if [ -f /opt/ros/jazzy/setup.bash ]; then source /opt/ros/jazzy/setup.bash; else source /opt/ros/humble/setup.bash; fi
source install/setup.bash 2>/dev/null || true
colcon build --packages-select ika_vision ika_control

echo "============================================"
echo "  CONTROLLER v4 + OBSTACLE DETECTOR DONE!    "
echo "============================================"
