import os

path = os.path.expanduser("~/ika_ws/src/ika_robot/ika_control/ika_control/controller_node.py")
with open(path, 'r') as f:
    c = f.read()

# ═══ 8. ETAP: Dik çıkış - güçlü motor ═══
old_climb = '''    def handle_steep_climb(self, speed, steering):
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
        
        return speed, steering'''

new_climb = '''    def handle_steep_climb(self, speed, steering):
        """8. Etap: Dik eğim çıkış - GÜÇLÜ MOTOR"""
        import time
        
        if self.pitch < -20:  # Arkaya yatmış = yukarı çıkıyor
            if not self.steep_detected:
                self.steep_detected = True
                self.steep_stop_time = time.time()
                self.get_logger().warn(f'>>> DIK EGIM ALGILANDI (pitch={self.pitch:.1f}°) → 2sn DUR <<<')
            
            elapsed = time.time() - self.steep_stop_time
            if elapsed < 2.0:
                return 0.0, 0.0  # DUR
            else:
                # GÜÇLÜ çıkış! 45° eğimde 0.15 yetmez
                climb_power = min(0.6, 0.3 + abs(self.pitch) * 0.01)
                self.get_logger().info(f'CLIMB: pitch={self.pitch:.1f}° power={climb_power:.2f}')
                return climb_power, 0.0
        
        elif self.steep_detected and abs(self.pitch) < 8:
            self.get_logger().warn(f'>>> PLATFORM (pitch={self.pitch:.1f}°) <<<')
            self.steep_detected = False
            return 0.0, 0.0
        
        return speed, steering'''

c = c.replace(old_climb, new_climb)

# ═══ 9. ETAP: Platform - çok yavaş ═══
old_platform = '''    def handle_platform(self):
        """9. Etap: Platform atış"""
        import time
        
        if not self.fire_done:
            self.fire_done = True
            self.get_logger().warn('>>> 🔥 ATEŞ EDİLDİ! (sembolik) <<<')
            self.steep_resume_time = time.time()
        
        elapsed = time.time() - self.steep_resume_time
        if elapsed < 3.0:
            return 0.0, 0.0  # 3sn atış bekleme
        else:
            return 0.20, 0.0  # İleri git (iniş başlasın)'''

new_platform = '''    def handle_platform(self):
        """9. Etap: Platform - ÇOK YAVAŞ"""
        import time
        
        if not self.fire_done:
            self.fire_done = True
            self.get_logger().warn('>>> 🔥 ATEŞ EDİLDİ! (sembolik) <<<')
            self.steep_resume_time = time.time()
        
        elapsed = time.time() - self.steep_resume_time
        if elapsed < 3.0:
            return 0.0, 0.0  # 3sn atış bekleme
        else:
            self.get_logger().info(f'PLATFORM: yavaş ileri, pitch={self.pitch:.1f}°')
            return 0.10, 0.0  # ÇOK yavaş ileri (iniş tehlikeli)'''

c = c.replace(old_platform, new_platform)

# ═══ 10. ETAP: Dik iniş - GERİ GİT + frenle ═══
old_descent = '''    def handle_steep_descent(self, speed, steering):
        """10. Etap: Dik iniş"""
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
        
        return speed, steering'''

new_descent = '''    def handle_steep_descent(self, speed, steering):
        """10. Etap: Dik iniş - GERİ VİTES + SERT FREN"""
        import time
        
        if self.pitch > 20:  # Öne yatmış = iniyor
            if not self.descent_detected:
                self.descent_detected = True
                self.descent_stop_time = time.time()
                self.get_logger().warn(f'>>> DIK INIS ALGILANDI (pitch={self.pitch:.1f}°) → FRENLE <<<')
            
            elapsed = time.time() - self.descent_stop_time
            
            if elapsed < 0.5:
                # Faz 1: GERİ GİT (0.5sn) - momentum kır
                self.get_logger().info(f'BRAKE: REVERSE (pitch={self.pitch:.1f}°)')
                return -0.15, 0.0
            elif elapsed < 2.5:
                # Faz 2: TAM FREN (2sn) - sabit dur
                return 0.0, 0.0
            else:
                # Faz 3: Çok yavaş iniş
                self.get_logger().info(f'DESCENT: slow (pitch={self.pitch:.1f}°)')
                return 0.08, 0.0
        
        elif self.descent_detected and abs(self.pitch) < 8:
            self.get_logger().warn(f'>>> INIS TAMAMLANDI → FINISH <<<')
            self.finish_start_time = time.time()
            self.descent_detected = False
        
        if self.finish_start_time:
            elapsed = time.time() - self.finish_start_time
            if elapsed < 4.0:
                return 0.12, 0.0  # Birkaç metre ilerle
            else:
                self.get_logger().warn('>>> 🏁 FİNİTO! PARKUR TAMAMLANDI! <<<')
                return 0.0, 0.0
        
        return speed, steering'''

c = c.replace(old_descent, new_descent)

with open(path, 'w') as f:
    f.write(c)

print("8: climb_power = 0.3 + pitch*0.01 (max 0.6)")
print("9: platform speed = 0.10 (çok yavaş)")
print("10: REVERSE 0.5s → BRAKE 2s → slow 0.08")
