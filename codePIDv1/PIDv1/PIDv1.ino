/*
 * ================================================================
 *  DRONE PID STABILIZER — Pitch & Roll
 *  ESP32 + MPU6050 + 4x ESC (X-Frame)
 * ================================================================
 *
 *  Layout motor (nhìn từ TRÊN xuống):
 *
 *           [MŨI - FRONT]
 *       M1 (FL)     M2 (FR)
 *       pin 26       pin 13
 *           \         /
 *            [THÂN]
 *           /         \
 *       M4 (BL)     M3 (BR)
 *       pin 27       pin 14
 *           [ĐUÔI - BACK]
 *
 *  Quy ước góc từ Complementary Filter:
 *    pitch > 0 → Mũi ngẩng LÊN
 *    pitch < 0 → Mũi chúc XUỐNG
 *    roll  > 0 → Cánh TRÁI xuống (nghiêng trái)
 *    roll  < 0 → Cánh PHẢI xuống (nghiêng phải)
 *
 *  ! Lưu ý thực tế:
 *    Nếu drone tự nghiêng THÊM thay vì hiệu chỉnh về 0,
 *    hãy đảo dấu của pitchOut hoặc rollOut trong phần Motor Mixing.
 * ================================================================
 */

#include <Wire.h>
#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>
#include <ESP32Servo.h>

// ================================================================
//  PIN ESC
// ================================================================
const int ESC_M1 = 13; // Front-Left  (FL)
const int ESC_M2 = 14; // Front-Right (FR)
const int ESC_M3 = 26; // Back-Right  (BR)
const int ESC_M4 = 27; // Back-Left   (BL)

Servo esc1, esc2, esc3, esc4;
Adafruit_MPU6050 mpu;

// ================================================================
//  THROTTLE (đơn vị: µs PWM)
// ================================================================
const int BASE_THROTTLE = 1200; // Tốc độ hover / cân bằng bình thường
const int MAX_THROTTLE  = 1400; // Tốc độ tối đa cho phép
const int MIN_THROTTLE  = 1100; // Tốc độ tối thiểu (không tắt hẳn)

// ================================================================
//  HỆ SỐ PID  ← TINH CHỈNH Ở ĐÂY
// ================================================================
//  Bước tinh chỉnh gợi ý:
//   1. Ki = 0, Kd = 0 → tăng Kp từ từ cho đến khi drone rung nhẹ
//   2. Lùi Kp lại ~30%, sau đó tăng Kd để dập rung
//   3. Cuối cùng thêm Ki nhỏ nếu còn bị lệch góc tĩnh
// ================================================================
float Kp_P = 1.5f, Ki_P = 0.02f, Kd_P = 0.8f; // Pitch
float Kp_R = 1.5f, Ki_R = 0.02f, Kd_R = 0.8f; // Roll

const float PID_CLAMP       = 200.0f; // Giới hạn output PID (µs)
const float INTEGRAL_CLAMP  =  50.0f; // Giới hạn tích phân (anti-windup)
const float SAFE_ANGLE      =  45.0f; // Góc nghiêng tối đa → tắt khẩn cấp

// ================================================================
//  SETPOINT (góc mục tiêu — bay nằm ngang)
// ================================================================
const float PITCH_SP = 0.0f;
const float ROLL_SP  = 0.0f;

// ================================================================
//  BIẾN TRẠNG THÁI
// ================================================================
float pitch = 0.0f, roll = 0.0f;
const float ALPHA = 0.96f;     // Hệ số lọc bù (96% Gyro, 4% Accel)
unsigned long lastTime = 0;

// Biến PID
float pitchIntegral = 0.0f, pitchPrevError = 0.0f;
float rollIntegral  = 0.0f, rollPrevError  = 0.0f;

// ================================================================
//  HÀM PID
// ================================================================
float computePID(float setpoint, float measured,
                 float &integral, float &prevError,
                 float Kp, float Ki, float Kd, float dt) {
  float error = setpoint - measured;

  // Cộng dồn tích phân + giới hạn anti-windup
  integral += error * dt;
  integral  = constrain(integral, -INTEGRAL_CLAMP, INTEGRAL_CLAMP);

  // Vi phân (tránh spike khi dt = 0)
  float derivative = (dt > 0.0f) ? (error - prevError) / dt : 0.0f;
  prevError = error;

  float output = (Kp * error) + (Ki * integral) + (Kd * derivative);
  return constrain(output, -PID_CLAMP, PID_CLAMP);
}

// ================================================================
//  DỪNG KHẨN CẤP
// ================================================================
void emergencyStop(const char* reason) {
  esc1.writeMicroseconds(1000);
  esc2.writeMicroseconds(1000);
  esc3.writeMicroseconds(1000);
  esc4.writeMicroseconds(1000);
  Serial.print("[EMERGENCY] ");
  Serial.println(reason);
  while (true) delay(500); // Treo vòng lặp, không tiếp tục
}

// ================================================================
//  SETUP
// ================================================================
void setup() {
  Serial.begin(115200);

  // ---- Khởi ESC, giữ ga = 0 để an toàn ----
  esc1.setPeriodHertz(50); esc1.attach(ESC_M1, 1000, 2000);
  esc2.setPeriodHertz(50); esc2.attach(ESC_M2, 1000, 2000);
  esc3.setPeriodHertz(50); esc3.attach(ESC_M3, 1000, 2000);
  esc4.setPeriodHertz(50); esc4.attach(ESC_M4, 1000, 2000);

  esc1.writeMicroseconds(1000);
  esc2.writeMicroseconds(1000);
  esc3.writeMicroseconds(1000);
  esc4.writeMicroseconds(1000);

  Serial.println("[BOOT] Giữ ga 0%. Chờ 5s để ESC hoàn tất khởi động...");
  delay(5000);

  // ---- Khởi MPU6050 ----
  Wire.begin(32, 33, 400000); // SDA=32, SCL=33
  if (!mpu.begin(0x68, &Wire)) {
    emergencyStop("Khong tim thay MPU6050!");
  }

  // Cấu hình dải đo phù hợp cho drone nhỏ
  mpu.setAccelerometerRange(MPU6050_RANGE_4_G);
  mpu.setGyroRange(MPU6050_RANGE_500_DEG);
  mpu.setFilterBandwidth(MPU6050_BAND_44_HZ);

  Serial.println("[OK] Vao vong lap PID 50Hz...");
  lastTime = millis();
}

// ================================================================
//  LOOP CHÍNH — 50Hz (20ms/chu kỳ)
// ================================================================
void loop() {
  sensors_event_t a, g, temp;
  mpu.getEvent(&a, &g, &temp);

  // ---- 1. Tính dt ----
  unsigned long now = millis();
  float dt = (now - lastTime) / 1000.0f;
  if (dt < 0.001f) dt = 0.001f; // Tránh chia cho 0
  lastTime = now;

  // ---- 2. Complementary Filter ----
  float pitchAcc = atan2f(a.acceleration.y,
                    sqrtf(a.acceleration.x * a.acceleration.x +
                          a.acceleration.z * a.acceleration.z)) * 57.29578f;
  float rollAcc  = atan2f(-a.acceleration.x,
                           a.acceleration.z) * 57.29578f;

  float gyroX = g.gyro.x * 57.29578f; // rad/s → °/s
  float gyroY = g.gyro.y * 57.29578f;

  pitch = ALPHA * (pitch + gyroX * dt) + (1.0f - ALPHA) * pitchAcc;
  roll  = ALPHA * (roll  + gyroY * dt) + (1.0f - ALPHA) * rollAcc;

  // ---- 3. Kiểm tra an toàn góc nghiêng ----
  if (fabsf(pitch) > SAFE_ANGLE || fabsf(roll) > SAFE_ANGLE) {
    emergencyStop("Nghieng qua 45 do! Tat khan cap.");
  }

  // ---- 4. Tính PID ----
  float pitchOut = computePID(PITCH_SP, pitch,
                              pitchIntegral, pitchPrevError,
                              Kp_P, Ki_P, Kd_P, dt);
  float rollOut  = computePID(ROLL_SP, roll,
                              rollIntegral, rollPrevError,
                              Kp_R, Ki_R, Kd_R, dt);

  // ---- 5. Motor Mixing (X-frame) ----
  //
  //  Logic vật lý:
  //   pitch > 0 (mũi lên)    → pitchOut âm → giảm M1,M2 (mũi) / tăng M3,M4 (đuôi)
  //   roll  < 0 (phải xuống) → rollOut dương → tăng M1,M4 (trái) / giảm M2,M3 (phải)
  //
  //  Công thức:
  //   M1 (FL) = Base + pitchOut + rollOut
  //   M2 (FR) = Base + pitchOut - rollOut
  //   M3 (BR) = Base - pitchOut - rollOut
  //   M4 (BL) = Base - pitchOut + rollOut
  //
  //  ! Nếu drone tự nghiêng thêm (feedback dương), đảo dấu:
  //    Pitch ngược: đổi +pitchOut ↔ -pitchOut
  //    Roll  ngược: đổi +rollOut  ↔ -rollOut

  int m1 = constrain((int)(BASE_THROTTLE + pitchOut + rollOut), MIN_THROTTLE, MAX_THROTTLE); // FL
  int m2 = constrain((int)(BASE_THROTTLE + pitchOut - rollOut), MIN_THROTTLE, MAX_THROTTLE); // FR
  int m3 = constrain((int)(BASE_THROTTLE - pitchOut - rollOut), MIN_THROTTLE, MAX_THROTTLE); // BR
  int m4 = constrain((int)(BASE_THROTTLE - pitchOut + rollOut), MIN_THROTTLE, MAX_THROTTLE); // BL

  // ---- 6. Xuất xung tới ESC ----
  esc1.writeMicroseconds(m1);
  esc2.writeMicroseconds(m2);
  esc3.writeMicroseconds(m3);
  esc4.writeMicroseconds(m4);

  // ---- 7. Debug Serial (50Hz — dùng Serial Monitor baud 115200) ----
  Serial.print("P:");     Serial.print(pitch,    1);
  Serial.print(" R:");    Serial.print(roll,     1);
  Serial.print(" | pOut:"); Serial.print(pitchOut, 1);
  Serial.print(" rOut:");  Serial.print(rollOut,  1);
  Serial.print(" | M1:"); Serial.print(m1);
  Serial.print(" M2:");   Serial.print(m2);
  Serial.print(" M3:");   Serial.print(m3);
  Serial.print(" M4:");   Serial.println(m4);

  delay(20); // Giữ tần số 50Hz
}
