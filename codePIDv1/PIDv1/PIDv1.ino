
#include <Wire.h>
#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>
#include <ESP32Servo.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>
#include <math.h>

// ================================================================
//  PIN ESC
// ================================================================
const int ESC_M1 = 26; // Front-Left  (FL)
const int ESC_M2 = 13; // Front-Right (FR)
const int ESC_M3 = 14; // Back-Right  (BR)
const int ESC_M4 = 27; // Back-Left   (BL)

Servo esc1, esc2, esc3, esc4;
Adafruit_MPU6050 mpu;

// ================================================================
//  OLED (SSD1306) — dùng I2C bus RIÊNG (bus số 1) để không đụng
//  với MPU6050 đang chạy trên Wire (bus 0, pin 32/33)
//
//  ! CẢNH BÁO: mỗi lần display.display() phải đẩy 1024 byte qua I2C.
//    Ở 400kHz mất ~23ms — trong lúc đó vòng điều khiển BỊ CHẶN.
//    Chỉ nên bật OLED khi hiệu chỉnh trên bàn. Khi bay thật:
//    đặt ENABLE_OLED = 0.
// ================================================================
#define ENABLE_OLED    1

#define SCREEN_WIDTH   128
#define SCREEN_HEIGHT  64
#define OLED_RESET     -1
#define OLED_SDA       21
#define OLED_SCL       22
#define OLED_ADDR      0x3C
#define OLED_FREQ      400000UL   // 100kHz -> 400kHz: giảm block từ ~92ms xuống ~23ms

TwoWire I2C_OLED = TwoWire(1);
Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &I2C_OLED, OLED_RESET);
bool oledOK = false;

// ================================================================
//  TẦN SỐ CÁC TÁC VỤ
// ================================================================
const unsigned long LOOP_US    = 5000;   // Vòng điều khiển: 200Hz
const unsigned long SERIAL_US  = 100000; // Debug Serial: 10Hz
const unsigned long OLED_US    = 250000; // Vẽ OLED: 4Hz

unsigned long nextLoopUs   = 0;
unsigned long lastSerialUs = 0;
unsigned long lastOledUs   = 0;

// ================================================================
//  THROTTLE (đơn vị: µs PWM)
// ================================================================
const int BASE_THROTTLE = 1300; // Tốc độ hover / cân bằng bình thường
const int MAX_THROTTLE  = 2000; // Tốc độ tối đa cho phép
const int MIN_THROTTLE  = 1100; // Tốc độ tối thiểu (không tắt hẳn)
const int IDLE_THROTTLE = 1000; // Xung "tắt" / arming

// ================================================================
//  TRIM ĐỘNG CƠ (bù chênh lệch ESC/motor/cánh) — đơn vị µs
// ================================================================
//  QUAN TRỌNG: trim phải là OFFSET µs CỐ ĐỊNH, không được nhân %.
//  Nhân % làm lượng bù thay đổi theo throttle và phá vỡ giới hạn
//  MIN/MAX. Trim được cộng TRƯỚC constrain nên không bao giờ đẩy
//  xung ra ngoài dải an toàn.
//
//  Cách chỉnh: tháo cánh, để BASE_THROTTLE, nghe/đo tốc độ 4 motor,
//  cộng/trừ từng chục µs cho tới khi đều nhau. Giữ |trim| < 60µs;
//  nếu cần hơn thì phần cứng đang có vấn đề thật.
// ================================================================
const int TRIM_M1 = 0;
const int TRIM_M2 = 0;
const int TRIM_M3 = 0;
const int TRIM_M4 = 0;

// ================================================================
//  HỆ SỐ PID  ← TINH CHỈNH Ở ĐÂY
// ================================================================
//  Bước tinh chỉnh gợi ý:
//   1. Ki = 0, Kd = 0 → tăng Kp từ từ cho đến khi drone rung nhẹ
//   2. Lùi Kp lại ~30%, sau đó tăng Kd để dập rung
//   3. Cuối cùng thêm Ki nhỏ nếu còn bị lệch góc tĩnh
//
//  Lưu ý: D-term nay lấy trực tiếp từ gyro (°/s) thay vì vi phân
//  góc, nên Kd KHÔNG cùng thang với bản cũ — bắt đầu lại từ ~0.3.
// ================================================================
float Kp_P = 1.5f, Ki_P = 0.02f, Kd_P = 0.3f; // Pitch
float Kp_R = 1.5f, Ki_R = 0.02f, Kd_R = 0.3f; // Roll
float Kp_Y = 2.0f, Ki_Y = 0.00f, Kd_Y = 0.0f; // Yaw (điều khiển theo TỐC ĐỘ quay)

const float PID_CLAMP       = 200.0f; // Giới hạn output PID (µs)
const float INTEGRAL_CLAMP  =  50.0f; // Giới hạn tích phân (anti-windup)
const float SAFE_ANGLE      =  45.0f; // Góc nghiêng tối đa → tắt khẩn cấp

// ================================================================
//  SETPOINT (góc mục tiêu — bay nằm ngang)
// ================================================================
const float PITCH_SP = 0.0f;
const float ROLL_SP  = 0.0f;
const float YAW_RATE_SP = 0.0f; // °/s — giữ hướng, không tự xoay

// ================================================================
//  DẤU TRỤC  ← KIỂM CHỨNG TRÊN BÀN TRƯỚC KHI LẮP CÁNH
// ================================================================
//  GYRO_*_SIGN: làm cho gyro CÙNG DẤU với accel trong bộ lọc bù.
//    Với MPU6050 lắp chuẩn: xoay +θ quanh trục X cho a.y ≈ -g·sin(θ),
//    tức pitchAcc ≈ -θ, trong khi tích phân gyro.x cho +θ → NGƯỢC.
//    Vì vậy mặc định để -1. Nếu góc hiển thị không hội tụ hoặc dao
//    động khi giữ nghiêng cố định → đảo thành +1.
//
//  MIX_*_SIGN: chiều phản hồi ra motor. Nếu drone TỰ NGHIÊNG THÊM
//    (phản hồi dương) thì đảo dấu tương ứng.
//
//  QUY TRÌNH TEST (THÁO CÁNH QUẠT):
//   B1. Nghiêng khung ~30° về phía trước, giữ yên 5 giây.
//       Serial phải hiện P: ổn định ở ~30 (hoặc ~-30) và KHÔNG trôi.
//       Nếu về 0 hoặc dao động → đảo GYRO_PITCH_SIGN.
//   B2. Lặp lại với nghiêng sang phải cho R: / GYRO_ROLL_SIGN.
//   B3. Xem M1..M4: khi nghiêng, motor ở phía THẤP phải TĂNG xung.
//       Nếu ngược → đảo MIX_PITCH_SIGN / MIX_ROLL_SIGN.
// ================================================================
const float GYRO_PITCH_SIGN = -1.0f;
const float GYRO_ROLL_SIGN  = -1.0f;

const float MIX_PITCH_SIGN  = +1.0f;
const float MIX_ROLL_SIGN   = +1.0f;
const float MIX_YAW_SIGN    = +1.0f;

// ================================================================
//  BIẾN TRẠNG THÁI
// ================================================================
float pitch = 0.0f, roll = 0.0f;

//  ALPHA: hằng số thời gian τ = ALPHA*dt/(1-ALPHA).
//  Với dt = 5ms, ALPHA = 0.995 → τ ≈ 1.0s (đủ lọc rung cánh,
//  vẫn đủ nhanh để chống trôi gyro). Bản cũ 0.96 cho τ chỉ 0.48s
//  ở 50Hz và τ = 0.12s ở 200Hz — quá thiên về accel, rất nhạy rung.
const float ALPHA = 0.995f;

unsigned long lastTimeUs = 0;

// Bias gyro đo lúc khởi động (rad/s)
float gyroBiasX = 0.0f, gyroBiasY = 0.0f, gyroBiasZ = 0.0f;

// Biến PID
float pitchIntegral = 0.0f, pitchPrevError = 0.0f;
float rollIntegral  = 0.0f, rollPrevError  = 0.0f;
float yawIntegral   = 0.0f, yawPrevError   = 0.0f;

// Xung đang xuất ra ESC (để emergencyStop hạ ga từ từ)
int curM1 = IDLE_THROTTLE, curM2 = IDLE_THROTTLE;
int curM3 = IDLE_THROTTLE, curM4 = IDLE_THROTTLE;

// Đếm lỗi đọc cảm biến liên tiếp
int sensorFailCount = 0;
const int SENSOR_FAIL_LIMIT = 10; // 10 chu kỳ 200Hz = 50ms mất cảm biến

// ================================================================
//  HÀM PID
// ================================================================
//  measured   : giá trị đo (góc ° hoặc tốc độ °/s)
//  rateForD   : tốc độ quay thô từ gyro (°/s) — dùng cho D-term.
//               Lấy D trực tiếp từ gyro thay vì vi phân sai số giúp
//               tránh khuếch đại nhiễu và không phụ thuộc dt.
//               Dấu âm vì d(error)/dt = -d(measured)/dt khi setpoint
//               không đổi.
//  saturated  : output vòng trước đã bị kẹp chưa (conditional
//               integration — ngừng tích lũy khi motor bão hòa)
// ================================================================
float computePID(float setpoint, float measured, float rateForD,
                 float &integral, float &prevError,
                 float Kp, float Ki, float Kd, float dt) {
  float error = setpoint - measured;

  // Tích phân thô để kiểm tra bão hòa trước khi cộng dồn
  float pTerm = Kp * error;
  float dTerm = -Kd * rateForD;

  // Conditional integration: chỉ cộng dồn khi output chưa bão hòa,
  // hoặc khi sai số có chiều kéo output ra khỏi vùng bão hòa.
  float preOut = pTerm + (Ki * integral) + dTerm;
  bool saturatedHigh = (preOut >  PID_CLAMP);
  bool saturatedLow  = (preOut < -PID_CLAMP);
  if ((!saturatedHigh && !saturatedLow) ||
      (saturatedHigh && error < 0.0f)   ||
      (saturatedLow  && error > 0.0f)) {
    integral += error * dt;
    integral  = constrain(integral, -INTEGRAL_CLAMP, INTEGRAL_CLAMP);
  }

  prevError = error;

  float output = pTerm + (Ki * integral) + dTerm;

  // constrain() với NaN trả về NaN (mọi so sánh đều false), rồi
  // (int)NaN là undefined behavior → phải chặn ở đây.
  if (isnan(output) || isinf(output)) {
    integral = 0.0f;
    return 0.0f;
  }
  return constrain(output, -PID_CLAMP, PID_CLAMP);
}

// ================================================================
//  HIỂN THỊ THÔNG TIN LÊN OLED
// ================================================================
void updateOled(float p, float r, float pOut, float rOut,
                 int m1, int m2, int m3, int m4) {
  if (!oledOK) return; // Không tìm thấy OLED lúc khởi động → bỏ qua

  display.clearDisplay();
  display.setCursor(0, 0);

  display.setTextSize(1);
  display.print("Pitch:"); display.print(p, 1);
  display.print(" Roll:"); display.println(r, 1);

  display.print("pOut:");  display.print(pOut, 1);
  display.print(" rOut:"); display.println(rOut, 1);

  display.println("--- ESC (us) ---");
  display.print("M1:"); display.print(m1);
  display.print("  M2:"); display.println(m2);
  display.print("M4:"); display.print(m4);
  display.print("  M3:"); display.println(m3);

  display.display();
}

// ================================================================
//  DỪNG KHẨN CẤP
// ================================================================
//  Cắt ga đột ngột khi đang bay = rơi tự do. Hạ ga tuyến tính về
//  IDLE trong ~1.5s để drone còn cơ hội tiếp đất mềm.
// ================================================================
void emergencyStop(const char* reason) {
  Serial.print("[EMERGENCY] ");
  Serial.println(reason);

  if (oledOK) {
    display.clearDisplay();
    display.setCursor(0, 0);
    display.setTextSize(1);
    display.println("!!! EMERGENCY !!!");
    display.println(reason);
    display.display();
  }

  // Hạ ga từ từ: 1.5s, bước 20ms
  const int STEPS = 75;
  int s1 = curM1, s2 = curM2, s3 = curM3, s4 = curM4;
  for (int i = 1; i <= STEPS; i++) {
    float k = 1.0f - (float)i / STEPS;
    esc1.writeMicroseconds(IDLE_THROTTLE + (int)((s1 - IDLE_THROTTLE) * k));
    esc2.writeMicroseconds(IDLE_THROTTLE + (int)((s2 - IDLE_THROTTLE) * k));
    esc3.writeMicroseconds(IDLE_THROTTLE + (int)((s3 - IDLE_THROTTLE) * k));
    esc4.writeMicroseconds(IDLE_THROTTLE + (int)((s4 - IDLE_THROTTLE) * k));
    delay(20);
  }

  esc1.writeMicroseconds(IDLE_THROTTLE);
  esc2.writeMicroseconds(IDLE_THROTTLE);
  esc3.writeMicroseconds(IDLE_THROTTLE);
  esc4.writeMicroseconds(IDLE_THROTTLE);

  while (true) delay(500); // Treo vòng lặp, không tiếp tục
}

// ================================================================
//  HIỆU CHUẨN BIAS GYRO
// ================================================================
//  MPU6050 xuất xưởng có offset gyro vài °/s. Không trừ bias thì
//  góc trôi liên tục và tích phân PID chạy theo.
//  ! Drone phải NẰM YÊN TUYỆT ĐỐI trong lúc này.
// ================================================================
void calibrateGyro() {
  const int SAMPLES = 1000;
  double sx = 0.0, sy = 0.0, sz = 0.0;

  Serial.println("[CAL] Dang hieu chuan gyro - GIU DRONE NAM YEN...");
  if (oledOK) {
    display.clearDisplay();
    display.setCursor(0, 0);
    display.println("Hieu chuan gyro...");
    display.println("GIU YEN 3 giay!");
    display.display();
  }

  sensors_event_t a, g, temp;
  for (int i = 0; i < SAMPLES; i++) {
    if (mpu.getEvent(&a, &g, &temp)) {
      sx += g.gyro.x;
      sy += g.gyro.y;
      sz += g.gyro.z;
    } else {
      i--; // Đọc lỗi → thử lại, không làm hỏng trung bình
    }
    delay(3);
  }

  gyroBiasX = (float)(sx / SAMPLES);
  gyroBiasY = (float)(sy / SAMPLES);
  gyroBiasZ = (float)(sz / SAMPLES);

  Serial.print("[CAL] Bias (deg/s) X:");
  Serial.print(gyroBiasX * 57.29578f, 3);
  Serial.print(" Y:"); Serial.print(gyroBiasY * 57.29578f, 3);
  Serial.print(" Z:"); Serial.println(gyroBiasZ * 57.29578f, 3);

  // Khởi tạo góc ban đầu từ accel để không phải chờ filter hội tụ
  if (mpu.getEvent(&a, &g, &temp)) {
    pitch = atan2f(a.acceleration.y,
                   sqrtf(a.acceleration.x * a.acceleration.x +
                         a.acceleration.z * a.acceleration.z)) * 57.29578f;
    roll  = atan2f(-a.acceleration.x, a.acceleration.z) * 57.29578f;
  }
}

// ================================================================
//  SETUP
// ================================================================
void setup() {
  Serial.begin(115200);

#if ENABLE_OLED
  // ---- Khởi OLED (bus I2C riêng, pin 21/22) ----
  I2C_OLED.begin(OLED_SDA, OLED_SCL, OLED_FREQ);
  if (!display.begin(SSD1306_SWITCHCAPVCC, OLED_ADDR)) {
    Serial.println("[WARN] Khong tim thay OLED! Bo qua hien thi.");
    oledOK = false;
  } else {
    oledOK = true;
    display.clearDisplay();
    display.setTextColor(SSD1306_WHITE);
    display.setTextSize(1);
    display.setCursor(0, 0);
    display.println("Dang khoi dong...");
    display.display();
  }
#else
  oledOK = false;
  Serial.println("[INFO] OLED da tat (ENABLE_OLED = 0).");
#endif

  // ---- Khởi ESC, giữ ga = 0 để an toàn ----
  esc1.setPeriodHertz(50); esc1.attach(ESC_M1, 1000, 2000);
  esc2.setPeriodHertz(50); esc2.attach(ESC_M2, 1000, 2000);
  esc3.setPeriodHertz(50); esc3.attach(ESC_M3, 1000, 2000);
  esc4.setPeriodHertz(50); esc4.attach(ESC_M4, 1000, 2000);

  esc1.writeMicroseconds(IDLE_THROTTLE);
  esc2.writeMicroseconds(IDLE_THROTTLE);
  esc3.writeMicroseconds(IDLE_THROTTLE);
  esc4.writeMicroseconds(IDLE_THROTTLE);

  Serial.println("[BOOT] Giu ga 0%. Cho 5s de ESC hoan tat khoi dong...");
  delay(5000);

  // ---- Khởi MPU6050 ----
  Wire.begin(32, 33, 400000); // SDA=32, SCL=33
  if (!mpu.begin(0x68, &Wire)) {
    emergencyStop("Khong tim thay MPU6050!");
  }

  // Cấu hình dải đo phù hợp cho drone nhỏ
  mpu.setAccelerometerRange(MPU6050_RANGE_4_G);
  mpu.setGyroRange(MPU6050_RANGE_500_DEG);
  mpu.setFilterBandwidth(MPU6050_BAND_44_HZ); // OK ở 200Hz (Nyquist = 100Hz)

  calibrateGyro();

  Serial.println("[OK] Vao vong lap PID 200Hz...");
  Serial.println("[!] THAO CANH QUAT truoc khi kiem chung dau truc.");

  lastTimeUs = micros();
  nextLoopUs = micros() + LOOP_US;
}

// ================================================================
//  LOOP CHÍNH — 200Hz (5ms/chu kỳ), KHÔNG dùng delay()
// ================================================================
//  delay(20) của bản cũ là thời gian NGHỈ THÊM, không phải chu kỳ.
//  Chu kỳ thực = 20ms + I2C + Serial (~6ms) + OLED (~92ms) → tần số
//  dao động 8–37Hz với jitter cực lớn. Nay dùng lịch theo micros().
// ================================================================
void loop() {
  // ---- 0. Chờ đến nhịp kế tiếp (không chặn) ----
  unsigned long nowUs = micros();
  if ((long)(nowUs - nextLoopUs) < 0) return;

  // Nếu bị trễ quá 1 chu kỳ (do OLED), bỏ qua các nhịp đã lỡ thay
  // vì cố chạy bù dồn dập.
  if ((long)(nowUs - nextLoopUs) > (long)LOOP_US) {
    nextLoopUs = nowUs + LOOP_US;
  } else {
    nextLoopUs += LOOP_US;
  }

  // ---- 1. Đọc cảm biến + phát hiện mất kết nối ----
  sensors_event_t a, g, temp;
  if (!mpu.getEvent(&a, &g, &temp)) {
    sensorFailCount++;
    if (sensorFailCount >= SENSOR_FAIL_LIMIT) {
      emergencyStop("Mat ket noi MPU6050!");
    }
    return; // Bỏ qua chu kỳ này, giữ nguyên xung ESC lần trước
  }
  sensorFailCount = 0;

  // ---- 2. Tính dt (micros — millis chỉ có do phan giai 5% o 200Hz) ----
  float dt = (nowUs - lastTimeUs) / 1000000.0f;
  lastTimeUs = nowUs;
  dt = constrain(dt, 0.0005f, 0.05f); // Chặn dt bất thường (tràn/trễ)

  // ---- 3. Complementary Filter ----
  float pitchAcc = atan2f(a.acceleration.y,
                    sqrtf(a.acceleration.x * a.acceleration.x +
                          a.acceleration.z * a.acceleration.z)) * 57.29578f;
  float rollAcc  = atan2f(-a.acceleration.x,
                           a.acceleration.z) * 57.29578f;

  // rad/s → °/s, đã trừ bias và chỉnh dấu cho khớp accel
  float gyroX = (g.gyro.x - gyroBiasX) * 57.29578f * GYRO_PITCH_SIGN;
  float gyroY = (g.gyro.y - gyroBiasY) * 57.29578f * GYRO_ROLL_SIGN;
  float gyroZ = (g.gyro.z - gyroBiasZ) * 57.29578f;

  pitch = ALPHA * (pitch + gyroX * dt) + (1.0f - ALPHA) * pitchAcc;
  roll  = ALPHA * (roll  + gyroY * dt) + (1.0f - ALPHA) * rollAcc;

  if (isnan(pitch) || isnan(roll)) {
    emergencyStop("Goc NaN - loi cam bien!");
  }

  // ---- 4. Kiểm tra an toàn góc nghiêng ----
  if (fabsf(pitch) > SAFE_ANGLE || fabsf(roll) > SAFE_ANGLE) {
    emergencyStop("Nghieng qua 45 do! Tat khan cap.");
  }

  // ---- 5. Tính PID ----
  //  D-term lấy trực tiếp từ gyro (°/s) thay vì vi phân góc đã lọc.
  float pitchOut = computePID(PITCH_SP, pitch, gyroX,
                              pitchIntegral, pitchPrevError,
                              Kp_P, Ki_P, Kd_P, dt);
  float rollOut  = computePID(ROLL_SP, roll, gyroY,
                              rollIntegral, rollPrevError,
                              Kp_R, Ki_R, Kd_R, dt);
  //  Yaw điều khiển theo TỐC ĐỘ quay (không có la bàn để giữ góc
  //  tuyệt đối). Không có yaw mixing thì drone tự xoay quanh trục
  //  đứng do mô-men phản lực không cân bằng.
  float yawOut   = computePID(YAW_RATE_SP, gyroZ, 0.0f,
                              yawIntegral, yawPrevError,
                              Kp_Y, Ki_Y, Kd_Y, dt);

  // ---- 6. Motor Mixing (X-frame) ----
  //
  //  Logic vật lý:
  //   pitch > 0 (mũi lên)    → pitchOut âm → giảm M1,M2 (mũi) / tăng M3,M4 (đuôi)
  //   roll  < 0 (phải xuống) → rollOut dương → tăng M1,M4 (trái) / giảm M2,M3 (phải)
  //
  //  Yaw: M1(FL) và M3(BR) quay cùng chiều; M2(FR) và M4(BL) chiều
  //  còn lại. Tăng cặp này / giảm cặp kia tạo mô-men xoay.
  //  Nếu drone xoay ngược ý muốn → đảo MIX_YAW_SIGN.
  //
  //  Công thức:
  //   M1 (FL) = Base + pitchOut + rollOut - yawOut
  //   M2 (FR) = Base + pitchOut - rollOut + yawOut
  //   M3 (BR) = Base - pitchOut - rollOut - yawOut
  //   M4 (BL) = Base - pitchOut + rollOut + yawOut
  //
  //  ! Nếu drone tự nghiêng thêm (feedback dương) → đảo MIX_*_SIGN
  //    ở phần khai báo, KHÔNG sửa công thức ở đây.

  float pOut = pitchOut * MIX_PITCH_SIGN;
  float rOut = rollOut  * MIX_ROLL_SIGN;
  float yOut = yawOut   * MIX_YAW_SIGN;

  // Trim cộng TRƯỚC constrain → không bao giờ vượt MIN/MAX_THROTTLE
  int m1 = constrain((int)(BASE_THROTTLE + pOut + rOut - yOut) + TRIM_M1, MIN_THROTTLE, MAX_THROTTLE); // FL
  int m2 = constrain((int)(BASE_THROTTLE + pOut - rOut + yOut) + TRIM_M2, MIN_THROTTLE, MAX_THROTTLE); // FR
  int m3 = constrain((int)(BASE_THROTTLE - pOut - rOut - yOut) + TRIM_M3, MIN_THROTTLE, MAX_THROTTLE); // BR
  int m4 = constrain((int)(BASE_THROTTLE - pOut + rOut + yOut) + TRIM_M4, MIN_THROTTLE, MAX_THROTTLE); // BL

  // ---- 7. Xuất xung tới ESC ----
  esc1.writeMicroseconds(m1);
  esc2.writeMicroseconds(m2);
  esc3.writeMicroseconds(m3);
  esc4.writeMicroseconds(m4);

  curM1 = m1; curM2 = m2; curM3 = m3; curM4 = m4;

  // ---- 8. Debug Serial (10Hz — Serial Monitor baud 115200) ----
  //  50Hz với ~70 ky tu o 115200 ton ~6ms/chu ky, du de pha nhip
  //  vong dieu khien 200Hz.
  if (nowUs - lastSerialUs >= SERIAL_US) {
    lastSerialUs = nowUs;
    Serial.print("P:");       Serial.print(pitch,    1);
    Serial.print(" R:");      Serial.print(roll,     1);
    Serial.print(" Yr:");     Serial.print(gyroZ,    1);
    Serial.print(" | pOut:"); Serial.print(pitchOut, 1);
    Serial.print(" rOut:");   Serial.print(rollOut,  1);
    Serial.print(" yOut:");   Serial.print(yawOut,   1);
    Serial.print(" | M1:");   Serial.print(m1);
    Serial.print(" M2:");     Serial.print(m2);
    Serial.print(" M3:");     Serial.print(m3);
    Serial.print(" M4:");     Serial.println(m4);
  }

  // ---- 9. Cập nhật OLED (4Hz — mỗi lần vẽ chặn loop ~23ms) ----
  if (oledOK && (nowUs - lastOledUs >= OLED_US)) {
    lastOledUs = nowUs;
    updateOled(pitch, roll, pitchOut, rollOut, m1, m2, m3, m4);
    // Vẽ xong đã trễ nhịp → đặt lại lịch để không chạy bù dồn dập
    nextLoopUs = micros() + LOOP_US;
    lastTimeUs = micros();
  }
}
