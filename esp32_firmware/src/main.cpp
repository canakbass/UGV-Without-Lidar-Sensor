#include <Arduino.h>
#include <Wire.h>

// --- Configuration ---
#define BAUD_RATE 115200
#define MOTOR_L_PWM 12
#define MOTOR_L_DIR 13
#define MOTOR_R_PWM 14
#define MOTOR_R_DIR 27
#define ENC_L_A 25
#define ENC_L_B 26
#define ENC_R_A 32
#define ENC_R_B 33

// --- Protocol Definitions ---
const uint8_t START_BYTE = 0xAA;
const uint8_t END_BYTE = 0xFF;

// Commands from Jetson
enum CommandType {
  CMD_SET_SPEED = 0x01,
  CMD_BRAKE = 0x02,
  CMD_RESET_ODOM = 0x03
};

// Data Structure for Speed Command (Float L, Float R)
struct __attribute__((packed)) SpeedCommand {
  float left_speed;
  float right_speed;
};

// Data Structure to Send to Jetson
struct __attribute__((packed)) TelemetryData {
  uint8_t start_byte;
  long enc_left;
  long enc_right;
  float imu_roll;
  float imu_pitch;
  float us_front_center;
  float us_front_left;
  float us_front_right;
  float us_rear_left;
  float us_rear_right;
  uint8_t checksum;
  uint8_t end_byte;
};

// --- Globals ---
volatile long encoder_left_count = 0;
volatile long encoder_right_count = 0;
TelemetryData telemetry;
float target_speed_l = 0.0;
float target_speed_r = 0.0;
bool emergency_brake = false;

// --- Interrupts ---
void IRAM_ATTR isr_enc_l() {
  if (digitalRead(ENC_L_B)) encoder_left_count++; else encoder_left_count--;
}
void IRAM_ATTR isr_enc_r() {
  if (digitalRead(ENC_R_B)) encoder_right_count++; else encoder_right_count--;
}

void setup() {
  Serial.begin(BAUD_RATE);
  
  pinMode(MOTOR_L_PWM, OUTPUT);
  pinMode(MOTOR_L_DIR, OUTPUT);
  pinMode(MOTOR_R_PWM, OUTPUT);
  pinMode(MOTOR_R_DIR, OUTPUT);
  
  pinMode(ENC_L_A, INPUT_PULLUP);
  pinMode(ENC_L_B, INPUT_PULLUP);
  pinMode(ENC_R_A, INPUT_PULLUP);
  pinMode(ENC_R_B, INPUT_PULLUP);
  
  attachInterrupt(digitalPinToInterrupt(ENC_L_A), isr_enc_l, RISING);
  attachInterrupt(digitalPinToInterrupt(ENC_R_A), isr_enc_r, RISING);

  telemetry.start_byte = START_BYTE;
  telemetry.end_byte = END_BYTE;
}

void process_serial() {
  static uint8_t buffer[64];
  static int idx = 0;
  static bool receiving = false;

  while (Serial.available()) {
    uint8_t byte = Serial.read();
    
    if (byte == START_BYTE && !receiving) {
      receiving = true;
      idx = 0;
      buffer[idx++] = byte;
    } else if (receiving) {
      buffer[idx++] = byte;
      if (byte == END_BYTE) {
        // Full packet received
        // Format: [AA] [CMD] [DATA...] [CS] [FF]
        uint8_t cmd = buffer[1];
        if (cmd == CMD_SET_SPEED) {
           if (idx == 2 + sizeof(SpeedCommand) + 2) { // AA + CMD + DATA + CS + FF
             // Parse Speed
             SpeedCommand* spd = (SpeedCommand*)&buffer[2];
             target_speed_l = spd->left_speed;
             target_speed_r = spd->right_speed;
             emergency_brake = false;
           }
        } else if (cmd == CMD_BRAKE) {
           emergency_brake = true;
           target_speed_l = 0;
           target_speed_r = 0;
        } else if (cmd == CMD_RESET_ODOM) {
           encoder_left_count = 0;
           encoder_right_count = 0;
        }
        receiving = false;
      }
    }
  }
}

void drive_motors() {
  if (emergency_brake) {
    // Active Braking: Short circuit phases (Low-Low drive) or Reverse briefly
    digitalWrite(MOTOR_L_DIR, LOW);
    analogWrite(MOTOR_L_PWM, 255); // Full Reverse for instant stop logic needs tuning
    digitalWrite(MOTOR_R_DIR, LOW);
    analogWrite(MOTOR_R_PWM, 255); 
    // WARNING: Simplistic braking. Real implementation depends on driver (IBT-2 needs L_PWM/R_PWM both Low for coast, or Logic for brake).
    // Assuming L298N style: Enable High + IN1/IN2 Low = Brake? No. Custom logic needed.
    // For now: Set Output 0
    analogWrite(MOTOR_L_PWM, 0);
    analogWrite(MOTOR_R_PWM, 0); 
    return;
  }

  // Simple PWM Drive (Example)
  int pwm_l = (int)(target_speed_l); // Map float speed to PWM
  int pwm_r = (int)(target_speed_r);

  if (pwm_l >= 0) {
    digitalWrite(MOTOR_L_DIR, HIGH);
    analogWrite(MOTOR_L_PWM, min(pwm_l, 255));
  } else {
    digitalWrite(MOTOR_L_DIR, LOW);
    analogWrite(MOTOR_L_PWM, min(-pwm_l, 255));
  }
  
  if (pwm_r >= 0) {
    digitalWrite(MOTOR_R_DIR, HIGH);
    analogWrite(MOTOR_R_PWM, min(pwm_r, 255));
  } else {
    digitalWrite(MOTOR_R_DIR, LOW);
    analogWrite(MOTOR_R_PWM, min(-pwm_r, 255));
  }
}

void send_telemetry() {
  telemetry.enc_left = encoder_left_count;
  telemetry.enc_right = encoder_right_count;
  // TODO: Read IMU and US sensors
  telemetry.imu_roll = 0.0; 
  
  // Calc Checksum
  uint8_t cs = 0;
  uint8_t* ptr = (uint8_t*)&telemetry;
  for (unsigned int i=0; i<sizeof(TelemetryData)-2; i++) {
    cs ^= ptr[i];
  }
  telemetry.checksum = cs;
  
  Serial.write((uint8_t*)&telemetry, sizeof(TelemetryData));
}

void loop() {
  process_serial();
  drive_motors();
  
  static unsigned long last_telemetry = 0;
  if (millis() - last_telemetry > 50) { // 20Hz Telemetry
    send_telemetry();
    last_telemetry = millis();
  }
}
