"""
TEKNOFEST 2026 İKA - State Machine

Parkur Akışı:
  Tabela 1 → BASLA (PID merkezleme)
  Tabela 2 → TASLI_YOL
  Tabela 3 → YAN_EGIM
  Tabela 4 → DIK_ENGEL
  Tabela 5 → TRAFIK_KONILERI
  Tabela 6 → KAYAR_ENGEL
  Tabela 7 → ENGEBELI_ARAZI
  Tabela 8 → DIK_EGIM_CIKIS (yukarı eğim)
  STOP     → CIKIS_DURMA (eğimde dur - IMU doğrula)
  Tabela 9 → PLATFORM_ATIS (hedefe lazer)
  Tabela 10→ DIK_EGIM_INIS (aşağı eğim)
  STOP     → INIS_DURMA (eğimde dur - IMU doğrula)
  5 saniye → MISSION_COMPLETE → IDLE

STOP Kuralı:
  - Etap 1-7: STOP yok sayılır (false positive riski)
  - DIK_EGIM_CIKIS: STOP kabul → CIKIS_DURMA
  - DIK_EGIM_INIS:  STOP kabul → INIS_DURMA
  - Diğer: yok sayılır

FINISH (Hedef) Kuralı:
  - Sadece PLATFORM_ATIS'ta kabul edilir
  - Hedef tespit → lazer hizalama süreci başlar
"""

import rclpy
from rclpy.node import Node
from std_msgs.msg import String, Float32MultiArray
from geometry_msgs.msg import Twist
from ika_interfaces.msg import SignDetection
import time

# Sıralı etap listesi - NEXT_STAGE her seferinde bir sonrakine geçer
STAGES = [
    "BASLA",              # Tabela 1
    "TASLI_YOL",          # Tabela 2
    "YAN_EGIM",           # Tabela 3
    "DIK_ENGEL",          # Tabela 4
    "TRAFIK_KONILERI",    # Tabela 5
    "KAYAR_ENGEL",        # Tabela 6
    "ENGEBELI_ARAZI",     # Tabela 7
    "DIK_EGIM_CIKIS",    # Tabela 8
    # Buradan sonrası STOP/FINISH ile yönetilir
]

# STOP kabul edilen state'ler
STOP_ALLOWED = ["DIK_EGIM_CIKIS", "DIK_EGIM_INIS"]

# STOP sonrası geçiş haritası
STOP_TRANSITION = {
    "DIK_EGIM_CIKIS": "CIKIS_DURMA",
    "DIK_EGIM_INIS": "INIS_DURMA",
}

# NEXT_STAGE tabela gördüğünde geçiş haritası (son bölüm için)
SIGN_TRANSITION = {
    "CIKIS_DURMA": "PLATFORM_ATIS",     # Tabela 9
    "PLATFORM_ATIS": "DIK_EGIM_INIS",   # Tabela 10
}

class StateMachineNode(Node):
    def __init__(self):
        super().__init__('state_machine')
        
        self.state = "IDLE"
        self.stage_index = 0
        self.mission_active = False
        self.stop_time = None  # INIS_DURMA'da durma zamanı
        
        # IMU verileri (eğim kontrolü için)
        self.pitch = 0.0
        self.roll = 0.0
        
        # Publishers
        self.pub_state = self.create_publisher(String, '/robot_state', 10)
        self.pub_cmd = self.create_publisher(Twist, '/cmd_vel', 10)
        
        # Subscribers
        self.create_subscription(String, '/user_command', self.command_cb, 10)
        self.create_subscription(SignDetection, '/vision/sign', self.sign_cb, 10)
        self.create_subscription(Float32MultiArray, '/unity/telemetry', self.telemetry_cb, 10)
        
        self.create_timer(0.5, self.tick)
        
        self.get_logger().info('State Machine v2 Started')
        self.get_logger().info(f'Normal stages: {" → ".join(STAGES)}')
        self.get_logger().info(f'Last section: ...→ CIKIS_DURMA → PLATFORM_ATIS → DIK_EGIM_INIS → INIS_DURMA → END')
    
    def telemetry_cb(self, msg):
        """Unity telemetrisinden pitch/roll al"""
        if msg.data and len(msg.data) >= 7:
            # [posX, posY, posZ, speed, yaw, pitch, roll, ...]
            self.pitch = msg.data[5]
            self.roll = msg.data[6]
    
    def command_cb(self, msg):
        cmd = msg.data.strip()
        self.get_logger().info(f'Command: {cmd}')
        
        if cmd == "START_AUTO_NORMAL":
            self.stage_index = 0
            self.state = STAGES[0]
            self.mission_active = True
            self.stop_time = None
            self.get_logger().info(f'=== AUTOPILOT STARTED === Stage: {self.state}')
            
        elif cmd == "START_AUTO_ACCEL":
            self.state = "ACCEL_RUN"
            self.mission_active = True
            self.get_logger().info('=== ACCELERATION RUN ===')
            
        elif cmd == "STOP" or cmd == "MANUAL":
            self.state = "IDLE" if cmd == "STOP" else "MANUAL"
            self.mission_active = False
            self.stage_index = 0
            self.stop_time = None
            self.get_logger().info(f'=== {cmd} → {self.state} ===')
    
    def sign_cb(self, msg):
        if not self.mission_active:
            return
        
        sign_type = msg.type
        conf = msg.confidence
        
        self.get_logger().info(
            f'Sign: type={sign_type}, conf={conf:.2f}, '
            f'state={self.state}, pitch={self.pitch:.1f}°'
        )
        
        if conf < 0.7:
            self.get_logger().info(f'Rejected: low confidence')
            return
        
        # ═══════════ NEXT_STAGE (numara tabelası) ═══════════
        if sign_type == "NEXT_STAGE":
            self.handle_next_stage()
        
        # ═══════════ STOP tabelası ═══════════
        elif sign_type == "STOP":
            self.handle_stop()
        
        # ═══════════ FINISH (hedef tabelası) ═══════════
        elif sign_type == "FINISH":
            self.handle_finish()
    
    def handle_next_stage(self):
        """Numara tabelası görüldü → bir sonraki etaba geç"""
        
        # Normal etaplar arasında geçiş (STAGES listesinde)
        if self.state in [s for s in STAGES]:
            idx = STAGES.index(self.state) if self.state in STAGES else -1
            if idx >= 0 and idx < len(STAGES) - 1:
                self.stage_index = idx + 1
                self.state = STAGES[self.stage_index]
                self.get_logger().warn(
                    f'>>> STAGE → {self.state} ({self.stage_index+1}/{len(STAGES)}) <<<'
                )
                return
        
        # Son bölüm geçişleri (STOP sonrası tabela)
        if self.state in SIGN_TRANSITION:
            new_state = SIGN_TRANSITION[self.state]
            self.get_logger().warn(f'>>> {self.state} → {new_state} <<<')
            self.state = new_state
            return
        
        self.get_logger().info(f'NEXT_STAGE ignored in state {self.state}')
    
    def handle_stop(self):
        """STOP tabelası görüldü"""
        
        if self.state in STOP_ALLOWED:
            new_state = STOP_TRANSITION[self.state]
            self.get_logger().warn(
                f'>>> STOP ACCEPTED: {self.state} → {new_state} '
                f'(pitch={self.pitch:.1f}°) <<<'
            )
            self.state = new_state
            
            # Hemen dur
            self.send_stop_cmd()
            
            # INIS_DURMA ise 5 saniye sonra görev biter
            if new_state == "INIS_DURMA":
                self.stop_time = time.time()
                self.get_logger().warn('>>> FINAL STOP - Mission ending in 5s <<<')
        else:
            self.get_logger().info(
                f'STOP ignored in state {self.state} '
                f'(only allowed in: {STOP_ALLOWED})'
            )
    
    def handle_finish(self):
        """Hedef tabelası görüldü → sadece PLATFORM_ATIS'ta"""
        
        if self.state == "PLATFORM_ATIS":
            self.get_logger().warn('>>> TARGET DETECTED on platform! Laser aiming... <<<')
            # TODO: Lazer hizalama mantığı
        else:
            self.get_logger().info(
                f'FINISH/TARGET ignored in state {self.state} '
                f'(only allowed in PLATFORM_ATIS)'
            )
    
    def send_stop_cmd(self):
        """Aracı durdur"""
        msg = Twist()
        msg.linear.x = 0.0
        msg.angular.z = 0.0
        self.pub_cmd.publish(msg)
    
    def tick(self):
        """Ana döngü - state yayınla + INIS_DURMA zamanlayıcı"""
        
        # INIS_DURMA'da 5 saniye sonra görev biter
        if self.state == "INIS_DURMA" and self.stop_time is not None:
            elapsed = time.time() - self.stop_time
            if elapsed > 5.0:
                self.state = "IDLE"
                self.mission_active = False
                self.stop_time = None
                self.get_logger().warn('>>> MISSION COMPLETE! <<<')
        
        # CIKIS_DURMA ve INIS_DURMA'da sürekli dur komutu gönder
        if self.state in ["CIKIS_DURMA", "INIS_DURMA"]:
            self.send_stop_cmd()
        
        # State yayınla
        msg = String()
        msg.data = self.state
        self.pub_state.publish(msg)

def main(args=None):
    rclpy.init(args=args)
    node = StateMachineNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()