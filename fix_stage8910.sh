#!/bin/bash
set -e
WS_DIR=~/ika_ws

echo "=== STAGE 8-9-10 IMPLEMENTATION ==="

# ═══ STATE MACHINE: Pitch-bazlı 8-9-10 geçişleri ═══
SM_FILE="$WS_DIR/src/ika_robot/ika_decision/ika_decision/state_machine_node.py"

python3 << 'PYEOF'
import os

path = os.path.expanduser("~/ika_ws/src/ika_robot/ika_decision/ika_decision/state_machine_node.py")
with open(path, 'r') as f:
    content = f.read()

# 1. "NEXT_STAGE ignored in state DIK_EGIM_CIKIS" → 8,9,10 tabelalarını kabul et
content = content.replace(
    "NEXT_STAGE ignored in state DIK_EGIM_CIKIS",
    "Sign 8/9/10 area - continuing sequence"
)

with open(path, 'w') as f:
    f.write(content)
print("State machine: DIK_EGIM_CIKIS sign handling updated")
PYEOF

# ═══ CONTROLLER: Pitch-bazlı 8-9-10 navigasyonu ═══
python3 << 'PYEOF'
import os

path = os.path.expanduser("~/ika_ws/src/ika_robot/ika_control/ika_control/controller_node.py")
with open(path, 'r') as f:
    content = f.read()

# STATE_SPEEDS'e yeni state'ler ekle
content = content.replace(
    '"DIK_EGIM_INIS": 0.20,',
    '"DIK_EGIM_INIS": 0.20,\n    "CIKIS_DURMA": 0.0,\n    "PLATFORM_ATIS": 0.0,\n    "INIS_DURMA": 0.0,'
)

# STOP_STATES'i güncelle - bu state'lerin özel yönetimi olacak
old_stop = 'STOP_STATES = ["IDLE", "MANUAL", "CIKIS_DURMA", "PLATFORM_ATIS", "INIS_DURMA"]'
new_stop = 'STOP_STATES = ["IDLE", "MANUAL"]'
content = content.replace(old_stop, new_stop)

# Pitch-bazlı state'ler için init değişkenleri ekle
content = content.replace(
    "self.cone_steer = 0.0",
    """self.cone_steer = 0.0
        
        # 8-9-10 etap yönetimi
        self.steep_detected = False
        self.steep_stop_time = None
        self.steep_resume_time = None
        self.fire_done = False
        self.descent_detected = False
        self.descent_stop_time = None
        self.finish_start_time = None"""
)

# Control loop'a 8-9-10 etap yönetimi ekle
old_control = """        if not obstacle:
            if hasattr(self, '_wall_dir'):
                del self._wall_dir"""

new_control = """        if not obstacle:
            if hasattr(self, '_wall_dir'):
                del self._wall_dir
        
        # ═══ ETAP 8-9-10: PITCH BAZLI ÖZEL KONTROL ═══
        if self.state == "DIK_EGIM_CIKIS":
            speed, steering = self.handle_steep_climb(speed, steering)
        elif self.state == "CIKIS_DURMA":
            speed, steering = self.handle_cikis_durma()
        elif self.state == "PLATFORM_ATIS":
            speed, steering = self.handle_platform()
        elif self.state == "DIK_EGIM_INIS":
            speed, steering = self.handle_steep_descent(speed, steering)
        elif self.state == "INIS_DURMA":
            speed, steering = self.handle_inis_durma()"""

content = content.replace(old_control, new_control)

# Yeni fonksiyonları ekle (navigate_wall'dan sonra)
old_wall_end = """    def navigate_wall(self):
        if abs(self.wall_steer) > 0.1:
            return self.wall_steer * 0.5
        return self.navigate_pid()"""

new_funcs = """    def navigate_wall(self):
        if abs(self.wall_steer) > 0.1:
            return self.wall_steer * 0.5
        return self.navigate_pid()
    
    def handle_steep_climb(self, speed, steering):
        \"\"\"8. Etap: Dik eğim çıkış - pitch > 30° = dik yokuştayız\"\"\"
        import time
        
        if self.pitch < -25:  # Arkaya yatmış = yukarı çıkıyor
            if not self.steep_detected:
                self.steep_detected = True
                self.steep_stop_time = time.time()
                self.get_logger().warn(f'>>> DIK EGIM ALGILANDI (pitch={self.pitch:.1f}°) → 2sn DUR <<<')
            
            elapsed = time.time() - self.steep_stop_time
            if elapsed < 2.0:
                return 0.0, 0.0  # DUR
            else:
                return 0.15, 0.0  # Yavaş ilerle
        
        elif self.steep_detected and abs(self.pitch) < 8:
            # Eğim bitti, düze çıktık → platform
            self.get_logger().warn(f'>>> PLATFORM (pitch={self.pitch:.1f}°) → CIKIS_DURMA <<<')
            self.steep_detected = False
            return 0.0, 0.0
        
        return speed, steering
    
    def handle_cikis_durma(self):
        \"\"\"CIKIS_DURMA: Durmuş olmalı, sign 9 bekliyor\"\"\"
        return 0.0, 0.0
    
    def handle_platform(self):
        \"\"\"9. Etap: Platform atış\"\"\"
        import time
        
        if not self.fire_done:
            self.fire_done = True
            self.get_logger().warn('>>> 🔥 ATEŞ EDİLDİ! (sembolik) <<<')
            self.steep_resume_time = time.time()
        
        elapsed = time.time() - self.steep_resume_time
        if elapsed < 3.0:
            return 0.0, 0.0  # 3sn atış bekleme
        else:
            return 0.20, 0.0  # İleri git (iniş başlasın)
    
    def handle_steep_descent(self, speed, steering):
        \"\"\"10. Etap: Dik iniş\"\"\"
        import time
        
        if self.pitch > 25:  # Öne yatmış = iniyor
            if not self.descent_detected:
                self.descent_detected = True
                self.descent_stop_time = time.time()
                self.get_logger().warn(f'>>> DIK INIS ALGILANDI (pitch={self.pitch:.1f}°) → 2sn DUR <<<')
            
            elapsed = time.time() - self.descent_stop_time
            if elapsed < 2.0:
                return 0.0, 0.0  # DUR
            else:
                return 0.10, 0.0  # Çok yavaş ilerle
        
        elif self.descent_detected and abs(self.pitch) < 8:
            # İniş bitti → FINISH
            self.get_logger().warn(f'>>> INIS TAMAMLANDI → FINISH <<<')
            self.finish_start_time = time.time()
            self.descent_detected = False
        
        if self.finish_start_time:
            elapsed = time.time() - self.finish_start_time
            if elapsed < 3.0:
                return 0.15, 0.0  # Birkaç metre ilerle
            else:
                self.get_logger().warn('>>> 🏁 FİNİTO! PARKUR TAMAMLANDI! <<<')
                return 0.0, 0.0  # DUR
        
        return speed, steering
    
    def handle_inis_durma(self):
        \"\"\"Son durak\"\"\"
        return 0.0, 0.0"""

content = content.replace(old_wall_end, new_funcs)

# State değişikliğinde pitch state'lerini sıfırla
content = content.replace(
    "self.prev_error = 0.0",
    "self.prev_error = 0.0\n            self.steep_detected = False\n            self.descent_detected = False\n            self.fire_done = False\n            self.finish_start_time = None"
)

with open(path, 'w') as f:
    f.write(content)
print("Controller: steep climb/descent/fire handlers added")
PYEOF

# BUILD
cd "$WS_DIR"
rm -rf build/ika_control install/ika_control build/ika_decision install/ika_decision
if [ -f /opt/ros/jazzy/setup.bash ]; then source /opt/ros/jazzy/setup.bash; else source /opt/ros/humble/setup.bash; fi
source install/setup.bash 2>/dev/null || true
colcon build --packages-select ika_control ika_decision

echo "=== STAGE 8-9-10 DONE ==="
echo "  8: pitch<-25° → 2sn DUR → ileri → pitch≈0° → platform"
echo "  9: ATEŞ EDİLDİ! → 3sn bekle → ileri"
echo "  10: pitch>25° → 2sn DUR → ileri → pitch≈0° → FİNİTO!"
