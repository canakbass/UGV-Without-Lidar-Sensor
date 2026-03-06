#!/bin/bash
set -e
WS_DIR=~/ika_ws

cat > "$WS_DIR/src/ika_robot/ika_control/ika_control/controller_node.py" << 'PYEOF'
"""
Navigasyon Kontrolcü v4.1

Düzeltmeler:
  - PID yönü GERİ ALINDI: error = fr - fl (doğru)
  - YAN_EGIM: IMU roll ile sağa yaslanma eklendi
  - U dönüşü: köşe sensörleriyle erken algılama
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
        self.us = [4.0] * 7  # [FC, FL, FR, CFL, CFR, CRL, CRR]
        self.us_received = False
        self.us_count = 0
        
        # IMU verileri (telemetry'den)
        self.roll = 0.0
        self.pitch = 0.0
        self.yaw = 0.0
        
        # Koni/duvar algılama
        self.cone_steer = 0.0
        self.wall_steer = 0.0
        
        # PID
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
        
        self.get_logger().info('Navigation Controller v4.1')
    
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
            self.yaw = msg.data[4]
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
            f'NAV: state={self.state}, us_rx={self.us_received}, '
            f'FC={self.us[0]:.2f} FL={self.us[1]:.2f} FR={self.us[2]:.2f} '
            f'CFL={self.us[3]:.2f} CFR={self.us[4]:.2f} '
            f'roll={self.roll:.1f} pitch={self.pitch:.1f}'
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
        
        # ═══ ENGEL KAÇINMA (tüm state'lerde) ═══
        obstacle = False
        
        if fc < 0.35:
            speed = 0.08
            steering = 0.7 if fl > fr else -0.7
            obstacle = True
        elif fc < 0.7:
            speed *= 0.35
            steering = 0.45 if fl > fr else -0.45
            obstacle = True
        elif cfl < 0.35:
            speed *= 0.5
            steering = -0.4
            obstacle = True
        elif cfr < 0.35:
            speed *= 0.5
            steering = 0.4
            obstacle = True
        
        if not obstacle:
            # ═══ ETAP BAZLI NAVİGASYON ═══
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
        """Normal PID merkezleme - DOĞRU YÖN: error = fr - fl"""
        fl = self.us[1]
        fr = self.us[2]
        
        if fl > 3.5 and fr > 3.5:
            self.prev_error = 0.0
            return 0.0
        
        # fr - fl: sağ uzak = pozitif → sola dön (merkezle)
        error = fr - fl
        derivative = error - self.prev_error
        self.prev_error = error
        
        steering = self.kp * error + self.kd * derivative
        return max(-0.5, min(0.5, steering))
    
    def navigate_yan_egim(self):
        """3. Etap: Yan eğim - SAĞA YASLAN (IMU roll ile)"""
        fl = self.us[1]
        fr = self.us[2]
        
        # IMU roll: pozitif = sağa yatık
        # Yan eğimde sağa yaslanmak = sağ bariyere yakın durmak
        right_bias = -0.15  # Sabit sağa yaslanma (negatif = sağa dön)
        
        # Roll kompanzasyonu: eğim varsa düzeltme uygula  
        roll_correction = 0.0
        if abs(self.roll) > 3.0:
            # Roll eğiminin tersine düzelt (kaymamak için)
            roll_correction = -self.roll * 0.02
        
        # PID + sağ bias + roll düzeltme
        if fl > 3.5 and fr > 3.5:
            return right_bias + roll_correction
        
        error = fr - fl
        derivative = error - self.prev_error
        self.prev_error = error
        pid = self.kp * error + self.kd * derivative
        pid = max(-0.4, min(0.4, pid))
        
        return pid + right_bias + roll_correction
    
    def navigate_cones(self):
        """5. Etap: Koni slalom"""
        if abs(self.cone_steer) > 0.1:
            return self.cone_steer * 0.5
        return self.navigate_pid()
    
    def navigate_wall(self):
        """6. Etap: Hareketli duvar"""
        if abs(self.wall_steer) > 0.1:
            return self.wall_steer * 0.5
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

cd "$WS_DIR"
rm -rf build/ika_control install/ika_control
if [ -f /opt/ros/jazzy/setup.bash ]; then source /opt/ros/jazzy/setup.bash; else source /opt/ros/humble/setup.bash; fi
source install/setup.bash 2>/dev/null || true
colcon build --packages-select ika_control
echo "  CONTROLLER v4.1 DONE"
