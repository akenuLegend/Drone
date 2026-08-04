/*
 * DRONE FLIGHT CONTROLLER — ESP32 + MPU6050 + 4x ESC (X-Frame)
 *
 * CHAN:  M1 (FL) = 26   M2 (FR) = 13
 *        M4 (BL) = 27   M3 (BR) = 14
 *        MPU6050 : SDA=32, SCL=33   |   OLED : SDA=21, SCL=22
 *
 * SERIAL (115200, gui 1 ky tu):
 *        a = ARM        d / SPACE = DISARM
 *        + / -          tang / giam ga 25us
 *        0              ga ve hover
 *        c              hieu chuan lai bias gyro (chi khi DISARMED)
 *
 * Quy uoc dau: pitch+ = mui ngang len, roll+ = nghieng phai, yaw+ = CCW.
 * THAO CANH QUAT truoc khi nap code va truoc khi kiem chung dau truc.
 */

#include <Wire.h>
#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>
#include <ESP32Servo.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>
#include <math.h>

// =============================================================================
//  1. CẤU HÌNH PHẦN CỨNG — chân ESC, OLED, MPU6050
// =============================================================================
const int ESC_M1 = 26; // Front-Left  (CW)
const int ESC_M2 = 13; // Front-Right (CCW)
const int ESC_M3 = 14; // Back-Right  (CW)
const int ESC_M4 = 27; // Back-Left   (CCW)

Servo esc1, esc2, esc3, esc4;
Adafruit_MPU6050 mpu;

#define ENABLE_OLED     1        // đặt 0 khi bay thật
#define SCREEN_WIDTH    128
#define SCREEN_HEIGHT   64
#define OLED_RESET      -1
#define OLED_SDA        21
#define OLED_SCL        22
#define OLED_ADDR       0x3C
#define OLED_FREQ       400000UL

TwoWire I2C_OLED = TwoWire(1);
Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &I2C_OLED, OLED_RESET);
bool oledOK = false;

#define MPU_SDA         32
#define MPU_SCL         33
#define MPU_FREQ        400000UL
#define MPU_ADDR        0x68

// =============================================================================
//  2. NHỊP HỆ THỐNG — chu kỳ vòng điều khiển và tần suất hiển thị
// =============================================================================
const uint32_t LOOP_HZ       = 250;
const uint32_t LOOP_US       = 1000000UL / LOOP_HZ;   // 4000µs
const uint32_t SERIAL_US     = 100000UL;              // 10Hz
const uint32_t OLED_US       = 250000UL;              // 4Hz

// =============================================================================
//  3. THROTTLE & NGƯỠNG AN TOÀN
// =============================================================================
const int   IDLE_THROTTLE   = 1000;  // xung "tắt"
const int   MIN_THROTTLE    = 1100;  // ga tối thiểu khi ARMED
const int   HOVER_THROTTLE  = 1300;  // ga hover mặc định
const int   MAX_THROTTLE    = 1500;  // chừa 100µs headroom cho PID
const int   THROTTLE_STEP   = 25;    // bước tăng/giảm qua Serial

const float SAFE_ANGLE      = 45.0f; // nghiêng quá → FAILSAFE
const float ARM_MAX_ANGLE   = 15.0f; // không cho ARM nếu nghiêng hơn mức này
const float MAX_RATE        = 150.0f;// giới hạn desired rate (°/s)
const float MAX_YAW_RATE    = 120.0f;// giới hạn desired yaw rate (°/s)

const int   SENSOR_FAIL_LIMIT = 12;  // ~48ms mất cảm biến → FAILSAFE

const float ACC_TRUST_LO    = 8.0f;  // m/s² — chỉ tin accel khi |a| gần 1g
const float ACC_TRUST_HI    = 11.5f; // m/s²

// =============================================================================
//  4. QUY ƯỚC TRỤC — hướng lắp chip và 6 hằng dấu
// =============================================================================
// MPU_MOUNT_Y_FORWARD: 1 = +Y chip về mũi   |   0 = +X chip về mũi
#define MPU_MOUNT_Y_FORWARD 1

// Dấu ĐO — kiểm chứng khi DISARMED, xem P/R/Yr trên Serial
const float GYRO_PITCH_SIGN = +1.0f;
const float GYRO_ROLL_SIGN  = +1.0f;
const float GYRO_YAW_SIGN   = +1.0f;

// Dấu ĐIỀU KHIỂN — kiểm chứng khi ARMED nhưng ĐÃ THÁO CÁNH, xem M1..M4
const float MIX_PITCH_SIGN  = +1.0f;
const float MIX_ROLL_SIGN   = +1.0f;
const float MIX_YAW_SIGN    = +1.0f;

// =============================================================================
//  5. TRIM ĐỘNG CƠ — offset µs cố định, bù chênh lệch ESC/motor/cánh
// =============================================================================
const int TRIM_M1 = 0;
const int TRIM_M2 = 0;
const int TRIM_M3 = 0;
const int TRIM_M4 = 0;

// =============================================================================
//  6. KHAI BÁO KIỂU DỮ LIỆU — bộ lọc, PID, cascaded, trạng thái bay
// =============================================================================
typedef enum { LPF_ORDER_1 = 1, LPF_ORDER_2 = 2 } LPFOrder;

typedef struct {
  float tau;
  float y1;          // state khâu 1
  float y2;          // state khâu 2 (output PT2)
  LPFOrder order;
} LowPassFilter;

typedef struct {
  float kp, ki, kd;
  float out_limit;
  float i_limit;
  float integral;
  float prev_meas;
  bool  has_prev;
  LowPassFilter d_lpf;
} PIDController;

typedef struct {
  PIDController angle;   // vòng ngoài: chỉ P
  PIDController rate;    // vòng trong: PID đầy đủ
  float max_rate;
} CascadedAxis;

typedef enum { STATE_DISARMED = 0, STATE_ARMED, STATE_FAILSAFE } FlightState;

// =============================================================================
//  7. BỘ LỌC THÔNG THẤP (PT1 / PT2)
// =============================================================================
const float GYRO_LPF_HZ     = 60.0f;  // pitch/roll rate
const float GYRO_YAW_LPF_HZ = 45.0f;  // yaw rate
const float DTERM_LPF_HZ    = 30.0f;  // lọc riêng cho khâu D

void lpf_init(LowPassFilter *f, float cutoff_hz,
              float init_val = 0.0f, LPFOrder order = LPF_ORDER_1) {
  f->order = order;
  f->tau   = 1.0f / (2.0f * PI * cutoff_hz);
  f->y1    = init_val;
  f->y2    = init_val;
}

float lpf_update(LowPassFilter *f, float x, float dt) {
  if (dt <= 0.0f)               return (f->order == LPF_ORDER_2) ? f->y2 : f->y1;
  if (isnan(x) || isinf(x))     return (f->order == LPF_ORDER_2) ? f->y2 : f->y1;

  float alpha = dt / (dt + f->tau);
  alpha = constrain(alpha, 0.0f, 1.0f);

  f->y1 = alpha * x + (1.0f - alpha) * f->y1;
  if (f->order == LPF_ORDER_2) {
    f->y2 = alpha * f->y1 + (1.0f - alpha) * f->y2;
    if (isnan(f->y2) || isinf(f->y2)) f->y2 = 0.0f;
    return f->y2;
  }
  if (isnan(f->y1) || isinf(f->y1)) f->y1 = 0.0f;
  return f->y1;
}

void lpf_reset(LowPassFilter *f, float val = 0.0f) {
  f->y1 = val;
  f->y2 = val;
}

// =============================================================================
//  8. BỘ LỌC BÙ — hằng số thời gian (α tính lại mỗi vòng theo dt)
// =============================================================================
const float CF_TAU = 1.2f;  // giây. Dải hợp lý: 0.8–3.0s

// =============================================================================
//  9. PID MỘT KHÂU — D trên measurement, anti-windup có điều kiện
// =============================================================================
void pid_init(PIDController *p, float kp, float ki, float kd,
              float out_limit, float i_limit, float dterm_hz) {
  p->kp = kp; p->ki = ki; p->kd = kd;
  p->out_limit = out_limit;
  p->i_limit   = i_limit;
  p->integral  = 0.0f;
  p->prev_meas = 0.0f;
  p->has_prev  = false;
  lpf_init(&p->d_lpf, dterm_hz, 0.0f, LPF_ORDER_1);
}

void pid_reset(PIDController *p) {
  p->integral  = 0.0f;
  p->prev_meas = 0.0f;
  p->has_prev  = false;
  lpf_reset(&p->d_lpf, 0.0f);
}

float pid_compute(PIDController *p, float setpoint, float meas, float dt) {
  if (dt <= 0.0f) return 0.0f;
  if (isnan(meas) || isinf(meas) || isnan(setpoint)) { pid_reset(p); return 0.0f; }

  float error  = setpoint - meas;
  float p_term = p->kp * error;

  // Khâu D: -kd · d(meas)/dt, đã lọc
  float d_term = 0.0f;
  if (p->kd != 0.0f) {
    if (p->has_prev) {
      float d_meas = (meas - p->prev_meas) / dt;
      d_term = -p->kd * lpf_update(&p->d_lpf, d_meas, dt);
    }
    p->prev_meas = meas;
    p->has_prev  = true;
  }

  // Khâu I: chỉ tích lũy khi không làm bão hoà thêm
  float pre_out = p_term + p->ki * p->integral + d_term;
  bool sat_hi = (pre_out >  p->out_limit);
  bool sat_lo = (pre_out < -p->out_limit);
  if ((!sat_hi && !sat_lo) || (sat_hi && error < 0.0f) || (sat_lo && error > 0.0f)) {
    p->integral = constrain(p->integral + error * dt, -p->i_limit, p->i_limit);
  }

  float out = p_term + p->ki * p->integral + d_term;
  if (isnan(out) || isinf(out)) { pid_reset(p); return 0.0f; }
  return constrain(out, -p->out_limit, p->out_limit);
}

// =============================================================================
//  10. CASCADED PID — vòng góc (P) lồng vòng tốc độ góc (PID)
// =============================================================================
void cascaded_init(CascadedAxis *c, float angle_kp,
                   float rate_kp, float rate_ki, float rate_kd,
                   float max_rate, float out_limit, float i_limit) {
  pid_init(&c->angle, angle_kp, 0.0f, 0.0f, max_rate, 0.0f, 100.0f);
  pid_init(&c->rate,  rate_kp, rate_ki, rate_kd, out_limit, i_limit, DTERM_LPF_HZ);
  c->max_rate = max_rate;
}

void cascaded_reset(CascadedAxis *c) { pid_reset(&c->angle); pid_reset(&c->rate); }

float cascaded_compute(CascadedAxis *c, float angle_sp, float angle_meas,
                       float rate_meas, float dt) {
  float desired_rate = pid_compute(&c->angle, angle_sp, angle_meas, dt);
  desired_rate = constrain(desired_rate, -c->max_rate, c->max_rate);
  return pid_compute(&c->rate, desired_rate, rate_meas, dt);
}

// =============================================================================
//  11. THAM SỐ PID  ←←← TINH CHỈNH Ở ĐÂY
// =============================================================================
const float PITCH_ANGLE_KP = 3.5f;
const float PITCH_RATE_KP  = 0.90f;
const float PITCH_RATE_KI  = 1.20f;
const float PITCH_RATE_KD  = 0.012f;

const float ROLL_ANGLE_KP  = 3.5f;
const float ROLL_RATE_KP   = 0.90f;
const float ROLL_RATE_KI   = 1.20f;
const float ROLL_RATE_KD   = 0.012f;

// Yaw: chỉ điều khiển TỐC ĐỘ, không điều khiển góc tuyệt đối
const float YAW_RATE_KP    = 2.00f;
const float YAW_RATE_KI    = 1.00f;
const float YAW_RATE_KD    = 0.0f;

const float RATE_OUT_LIMIT = 250.0f; // µs — biên mỗi trục
const float RATE_I_LIMIT   = 60.0f;  // ° (∫ của °/s)

// =============================================================================
//  12. TRỘN ĐỘNG CƠ (X-Frame) — bảng dấu + chuẩn hoá 2 chiều
// =============================================================================
//   M1 (FL, CW ) = T − P + R + Y      M2 (FR, CCW) = T − P − R − Y
//   M3 (BR, CW ) = T + P − R + Y      M4 (BL, CCW) = T + P + R − Y
void motor_mix(float throttle, float p_ctrl, float r_ctrl, float y_ctrl,
               int *m1, int *m2, int *m3, int *m4) {
  float f1 = -p_ctrl + r_ctrl + y_ctrl;   // FL, cánh CW
  float f2 = -p_ctrl - r_ctrl - y_ctrl;   // FR, cánh CCW
  float f3 = +p_ctrl - r_ctrl + y_ctrl;   // BR, cánh CW
  float f4 = +p_ctrl + r_ctrl - y_ctrl;   // BL, cánh CCW

  float hi = fmaxf(fmaxf(f1, f2), fmaxf(f3, f4));
  float lo = fminf(fminf(f1, f2), fminf(f3, f4));

  // (a) Thu nhỏ đều nếu biên vi sai vượt dải khả dụng
  float span  = hi - lo;
  float avail = (float)(MAX_THROTTLE - MIN_THROTTLE);
  if (span > avail && span > 0.0f) {
    float k = avail / span;
    f1 *= k; f2 *= k; f3 *= k; f4 *= k;
    hi *= k; lo *= k;
  }

  // (b) Tịnh tiến vào dải hợp lệ — hy sinh độ cao, giữ tư thế
  float base = throttle;
  if (base + hi > (float)MAX_THROTTLE) base = (float)MAX_THROTTLE - hi;
  if (base + lo < (float)MIN_THROTTLE) base = (float)MIN_THROTTLE - lo;

  *m1 = constrain((int)lroundf(base + f1) + TRIM_M1, MIN_THROTTLE, MAX_THROTTLE);
  *m2 = constrain((int)lroundf(base + f2) + TRIM_M2, MIN_THROTTLE, MAX_THROTTLE);
  *m3 = constrain((int)lroundf(base + f3) + TRIM_M3, MIN_THROTTLE, MAX_THROTTLE);
  *m4 = constrain((int)lroundf(base + f4) + TRIM_M4, MIN_THROTTLE, MAX_THROTTLE);
}

// =============================================================================
//  13. TRẠNG THÁI TOÀN CỤC
// =============================================================================
FlightState flightState = STATE_DISARMED;
const char* failReason = "";

LowPassFilter lpfPitchRate, lpfRollRate, lpfYawRate;
CascadedAxis  pitchAxis, rollAxis;
PIDController yawRatePid;

float pitch = 0.0f, roll = 0.0f;            // ° (ước lượng)
float pitchRate = 0.0f, rollRate = 0.0f, yawRate = 0.0f; // °/s (đã lọc)
float yawHeading = 0.0f;                    // ° — chỉ để hiển thị

float pitchSetpoint = 0.0f;                 // ° (giữ ngang)
float rollSetpoint  = 0.0f;
float yawRateSetpoint = 0.0f;               // °/s

float gyroBiasX = 0.0f, gyroBiasY = 0.0f, gyroBiasZ = 0.0f;
bool  gyroCalibrated = false;

int   throttleCmd = HOVER_THROTTLE;
int   curM1 = IDLE_THROTTLE, curM2 = IDLE_THROTTLE;
int   curM3 = IDLE_THROTTLE, curM4 = IDLE_THROTTLE;
float lastP = 0.0f, lastR = 0.0f, lastY = 0.0f; // output PID gần nhất

uint32_t nextLoopUs = 0, lastTimeUs = 0, lastSerialUs = 0, lastOledUs = 0;
int  sensorFailCount = 0;
bool loopOverrun = false;

// =============================================================================
//  14. ESC — điểm duy nhất ghi xung xuống động cơ
// =============================================================================
void escWriteAll(int v1, int v2, int v3, int v4) {
  esc1.writeMicroseconds(v1); esc2.writeMicroseconds(v2);
  esc3.writeMicroseconds(v3); esc4.writeMicroseconds(v4);
  curM1 = v1; curM2 = v2; curM3 = v3; curM4 = v4;
}

void escIdle() { escWriteAll(IDLE_THROTTLE, IDLE_THROTTLE, IDLE_THROTTLE, IDLE_THROTTLE); }

// =============================================================================
//  15. OLED — hiển thị trạng thái, góc, output PID, 4 xung motor
// =============================================================================
const char* stateName() {
  switch (flightState) {
    case STATE_ARMED:    return "ARMED";
    case STATE_FAILSAFE: return "FAILSAFE";
    default:             return "DISARMED";
  }
}

void updateOled() {
  if (!oledOK) return;
  display.clearDisplay();
  display.setCursor(0, 0);
  display.setTextSize(1);

  display.print(stateName());
  display.print(" T:"); display.println(throttleCmd);

  display.print("P:"); display.print(pitch, 1);
  display.print(" R:"); display.print(roll, 1);
  display.print(" Y:"); display.println(yawHeading, 0);

  display.print("o:"); display.print(lastP, 0);
  display.print(" "); display.print(lastR, 0);
  display.print(" "); display.println(lastY, 0);

  display.print("M1:"); display.print(curM1);
  display.print(" M2:"); display.println(curM2);
  display.print("M4:"); display.print(curM4);
  display.print(" M3:"); display.println(curM3);

  if (flightState == STATE_FAILSAFE) display.println(failReason);
  display.display();
}

// =============================================================================
//  16. MÁY TRẠNG THÁI — failsafe, disarm, arm
// =============================================================================
void enterFailsafe(const char* reason) {
  if (flightState == STATE_FAILSAFE) return;
  flightState = STATE_FAILSAFE;
  failReason  = reason;

  Serial.print("\n[FAILSAFE] "); Serial.println(reason);
  updateOled();

  const int STEPS = 75;                    // 75 × 20ms = 1.5s
  int s1 = curM1, s2 = curM2, s3 = curM3, s4 = curM4;
  for (int i = 1; i <= STEPS; i++) {
    float k = 1.0f - (float)i / (float)STEPS;
    escWriteAll(IDLE_THROTTLE + (int)((s1 - IDLE_THROTTLE) * k),
                IDLE_THROTTLE + (int)((s2 - IDLE_THROTTLE) * k),
                IDLE_THROTTLE + (int)((s3 - IDLE_THROTTLE) * k),
                IDLE_THROTTLE + (int)((s4 - IDLE_THROTTLE) * k));
    delay(20);
  }
  escIdle();

  cascaded_reset(&pitchAxis);
  cascaded_reset(&rollAxis);
  pid_reset(&yawRatePid);

  Serial.println("[FAILSAFE] Da ha ga. Gui 'd' de ve DISARMED.");
}

void disarm(const char* why) {
  escIdle();
  cascaded_reset(&pitchAxis);
  cascaded_reset(&rollAxis);
  pid_reset(&yawRatePid);
  throttleCmd = HOVER_THROTTLE;
  flightState = STATE_DISARMED;
  Serial.print("[DISARM] "); Serial.println(why);
}

bool tryArm() {
  if (!gyroCalibrated)                            { Serial.println("[ARM] Tu choi: chua hieu chuan gyro."); return false; }
  if (fabsf(pitch) > ARM_MAX_ANGLE ||
      fabsf(roll)  > ARM_MAX_ANGLE)               { Serial.println("[ARM] Tu choi: drone dang nghieng qua 15 do."); return false; }
  if (flightState == STATE_FAILSAFE)              { Serial.println("[ARM] Tu choi: dang FAILSAFE, gui 'd' truoc."); return false; }

  cascaded_reset(&pitchAxis);
  cascaded_reset(&rollAxis);
  pid_reset(&yawRatePid);
  throttleCmd = HOVER_THROTTLE;
  flightState = STATE_ARMED;
  Serial.println("[ARM] >>> ARMED — MOTOR SE QUAY. 'd' hoac SPACE de tat. <<<");
  return true;
}

// =============================================================================
//  17. HIỆU CHUẨN BIAS GYRO — phải chạy mỗi lần khởi động, giữ drone nằm yên
// =============================================================================
bool calibrateGyro() {
  const int SAMPLES = 800;
  double sx = 0, sy = 0, sz = 0;
  int    got = 0, tries = 0;

  Serial.println("[CAL] Hieu chuan gyro — GIU DRONE NAM YEN TUYET DOI (~3s)...");
  if (oledOK) {
    display.clearDisplay(); display.setCursor(0, 0); display.setTextSize(1);
    display.println("HIEU CHUAN GYRO");
    display.println("GIU YEN 3 GIAY!");
    display.display();
  }

  sensors_event_t a, g, t;
  while (got < SAMPLES && tries < SAMPLES * 3) {
    tries++;
    if (mpu.getEvent(&a, &g, &t)) { sx += g.gyro.x; sy += g.gyro.y; sz += g.gyro.z; got++; }
    delay(3);
  }
  if (got < SAMPLES / 2) { Serial.println("[CAL] THAT BAI: doc cam bien loi."); return false; }

  gyroBiasX = (float)(sx / got);
  gyroBiasY = (float)(sy / got);
  gyroBiasZ = (float)(sz / got);

  Serial.print("[CAL] Bias (deg/s)  X:"); Serial.print(gyroBiasX * RAD_TO_DEG, 3);
  Serial.print("  Y:");                   Serial.print(gyroBiasY * RAD_TO_DEG, 3);
  Serial.print("  Z:");                   Serial.println(gyroBiasZ * RAD_TO_DEG, 3);

  // Khởi tạo góc từ accel để bộ lọc bù không phải hội tụ từ 0
  if (mpu.getEvent(&a, &g, &t)) {
    float ax = a.acceleration.x, ay = a.acceleration.y, az = a.acceleration.z;
#if MPU_MOUNT_Y_FORWARD
    pitch = atan2f( ay, sqrtf(ax * ax + az * az)) * RAD_TO_DEG;
    roll  = atan2f(-ax, az) * RAD_TO_DEG;
#else
    pitch = atan2f( ax, sqrtf(ay * ay + az * az)) * RAD_TO_DEG;
    roll  = atan2f( ay, az) * RAD_TO_DEG;
#endif
  }
  lpf_reset(&lpfPitchRate); lpf_reset(&lpfRollRate); lpf_reset(&lpfYawRate);
  yawHeading = 0.0f;
  gyroCalibrated = true;
  Serial.println("[CAL] Xong.");
  return true;
}

// =============================================================================
//  18. LỆNH QUA SERIAL — không chặn
// =============================================================================
void handleSerial() {
  while (Serial.available() > 0) {
    char c = (char)Serial.read();
    switch (c) {
      case 'a': case 'A':
        tryArm();
        break;
      case 'd': case 'D': case ' ':
        disarm("Lenh nguoi dung.");
        break;
      case '+': case '=':
        throttleCmd = constrain(throttleCmd + THROTTLE_STEP, MIN_THROTTLE, MAX_THROTTLE);
        Serial.print("[THR] "); Serial.println(throttleCmd);
        break;
      case '-': case '_':
        throttleCmd = constrain(throttleCmd - THROTTLE_STEP, MIN_THROTTLE, MAX_THROTTLE);
        Serial.print("[THR] "); Serial.println(throttleCmd);
        break;
      case '0':
        throttleCmd = HOVER_THROTTLE;
        Serial.print("[THR] "); Serial.println(throttleCmd);
        break;
      case 'c': case 'C':
        if (flightState == STATE_DISARMED) calibrateGyro();
        else Serial.println("[CAL] Chi hieu chuan khi DISARMED.");
        break;
      default: break;
    }
  }
}

// =============================================================================
//  SETUP — khởi tạo OLED → ESC → MPU6050 → bộ lọc → PID → hiệu chuẩn gyro
// =============================================================================
void setup() {
  Serial.begin(115200);
  delay(200);
  Serial.println("\n=== DRONE FC (ESP32) — khoi dong ===");

#if ENABLE_OLED
  I2C_OLED.begin(OLED_SDA, OLED_SCL, OLED_FREQ);
  if (!display.begin(SSD1306_SWITCHCAPVCC, OLED_ADDR)) {
    Serial.println("[WARN] Khong tim thay OLED — bo qua hien thi.");
    oledOK = false;
  } else {
    oledOK = true;
    display.clearDisplay();
    display.setTextColor(SSD1306_WHITE);
    display.setTextSize(1);
    display.setCursor(0, 0);
    display.println("Khoi dong...");
    display.display();
  }
#else
  oledOK = false;
  Serial.println("[INFO] OLED tat (ENABLE_OLED = 0).");
#endif

  // ESC: giữ xung IDLE trong suốt quá trình ESC tự khởi động
  esc1.setPeriodHertz(50); esc1.attach(ESC_M1, 1000, 2000);
  esc2.setPeriodHertz(50); esc2.attach(ESC_M2, 1000, 2000);
  esc3.setPeriodHertz(50); esc3.attach(ESC_M3, 1000, 2000);
  esc4.setPeriodHertz(50); esc4.attach(ESC_M4, 1000, 2000);
  escIdle();
  Serial.println("[BOOT] Giu ga 0%. Cho 5s cho ESC khoi dong...");
  delay(5000);

  // MPU6050
  Wire.begin(MPU_SDA, MPU_SCL, MPU_FREQ);
  if (!mpu.begin(MPU_ADDR, &Wire)) {
    Serial.println("[FATAL] Khong tim thay MPU6050!");
    escIdle();
    if (oledOK) {
      display.clearDisplay(); display.setCursor(0, 0);
      display.println("KHONG THAY MPU6050"); display.display();
    }
    while (true) delay(500);
  }
  mpu.setAccelerometerRange(MPU6050_RANGE_4_G);
  mpu.setGyroRange(MPU6050_RANGE_500_DEG);
  mpu.setFilterBandwidth(MPU6050_BAND_44_HZ);

  // Bộ lọc gyro
  lpf_init(&lpfPitchRate, GYRO_LPF_HZ,     0.0f, LPF_ORDER_2);
  lpf_init(&lpfRollRate,  GYRO_LPF_HZ,     0.0f, LPF_ORDER_2);
  lpf_init(&lpfYawRate,   GYRO_YAW_LPF_HZ, 0.0f, LPF_ORDER_2);

  // PID
  cascaded_init(&pitchAxis, PITCH_ANGLE_KP,
                PITCH_RATE_KP, PITCH_RATE_KI, PITCH_RATE_KD,
                MAX_RATE, RATE_OUT_LIMIT, RATE_I_LIMIT);
  cascaded_init(&rollAxis,  ROLL_ANGLE_KP,
                ROLL_RATE_KP, ROLL_RATE_KI, ROLL_RATE_KD,
                MAX_RATE, RATE_OUT_LIMIT, RATE_I_LIMIT);
  pid_init(&yawRatePid, YAW_RATE_KP, YAW_RATE_KI, YAW_RATE_KD,
           RATE_OUT_LIMIT, RATE_I_LIMIT, DTERM_LPF_HZ);

  calibrateGyro();

  Serial.println("\n[OK] San sang. Vong dieu khien 250Hz.");
  Serial.println("     a=ARM  d/space=DISARM  +/-=ga  0=hover  c=hieu chuan lai");
  Serial.println("     !! THAO CANH QUAT truoc khi kiem chung dau truc (muc 4) !!\n");

  uint32_t t0 = micros();
  lastTimeUs = t0;
  nextLoopUs = t0 + LOOP_US;
}

// =============================================================================
//  LOOP — 250Hz, không chặn
// =============================================================================
void loop() {
  handleSerial();

  uint32_t nowUs = micros();
  if ((int32_t)(nowUs - nextLoopUs) < 0) return;   // chưa tới nhịp

  // Trễ quá 1 chu kỳ → bỏ nhịp lỡ, không chạy bù dồn dập
  if ((int32_t)(nowUs - nextLoopUs) > (int32_t)LOOP_US) {
    nextLoopUs = nowUs + LOOP_US;
    loopOverrun = true;
  } else {
    nextLoopUs += LOOP_US;
    loopOverrun = false;
  }

  // ---- 1. Đọc cảm biến + phát hiện mất kết nối ----
  sensors_event_t a, g, t;
  if (!mpu.getEvent(&a, &g, &t)) {
    if (++sensorFailCount >= SENSOR_FAIL_LIMIT && flightState == STATE_ARMED) {
      enterFailsafe("Mat ket noi MPU6050");
    }
    return;
  }
  sensorFailCount = 0;

  // ---- 2. Tính dt, chặn cả hai đầu ----
  float dt = (float)(nowUs - lastTimeUs) * 1e-6f;
  lastTimeUs = nowUs;
  dt = constrain(dt, 0.0005f, 0.05f);

  // ---- 3. Gyro: trừ bias → °/s → ánh xạ trục → chỉnh dấu → lọc ----
  float gcx = (g.gyro.x - gyroBiasX) * RAD_TO_DEG;
  float gcy = (g.gyro.y - gyroBiasY) * RAD_TO_DEG;
  float gcz = (g.gyro.z - gyroBiasZ) * RAD_TO_DEG;

#if MPU_MOUNT_Y_FORWARD
  float gxRaw =  gcx * GYRO_PITCH_SIGN;
  float gyRaw =  gcy * GYRO_ROLL_SIGN;
#else
  float gxRaw = -gcy * GYRO_PITCH_SIGN;
  float gyRaw =  gcx * GYRO_ROLL_SIGN;
#endif
  float gzRaw =  gcz * GYRO_YAW_SIGN;

  // Gyro ĐÃ LỌC cho vòng rate & khâu D; gyro THÔ cho tích phân góc
  pitchRate = lpf_update(&lpfPitchRate, gxRaw, dt);
  rollRate  = lpf_update(&lpfRollRate,  gyRaw, dt);
  yawRate   = lpf_update(&lpfYawRate,   gzRaw, dt);

  // ---- 4. Góc từ accel + kiểm tra độ tin cậy ----
  float ax = a.acceleration.x, ay = a.acceleration.y, az = a.acceleration.z;
  float aMag = sqrtf(ax * ax + ay * ay + az * az);
  bool  accTrust = (aMag > ACC_TRUST_LO && aMag < ACC_TRUST_HI);

#if MPU_MOUNT_Y_FORWARD
  float pitchAcc = atan2f( ay, sqrtf(ax * ax + az * az)) * RAD_TO_DEG;
  float rollAcc  = atan2f(-ax, az) * RAD_TO_DEG;
#else
  float pitchAcc = atan2f( ax, sqrtf(ay * ay + az * az)) * RAD_TO_DEG;
  float rollAcc  = atan2f( ay, az) * RAD_TO_DEG;
#endif

  // ---- 5. Bộ lọc bù ----
  float alpha = CF_TAU / (CF_TAU + dt);
  if (!accTrust) alpha = 1.0f;             // đang tăng tốc mạnh → chỉ tin gyro
  pitch = alpha * (pitch + gxRaw * dt) + (1.0f - alpha) * pitchAcc;
  roll  = alpha * (roll  + gyRaw * dt) + (1.0f - alpha) * rollAcc;

  yawHeading += yawRate * dt;              // chỉ để hiển thị — có trôi
  if (yawHeading >  180.0f) yawHeading -= 360.0f;
  if (yawHeading < -180.0f) yawHeading += 360.0f;

  if (isnan(pitch) || isnan(roll)) {
    pitch = pitchAcc; roll = rollAcc;
    if (flightState == STATE_ARMED) enterFailsafe("Goc NaN");
    return;
  }

  // ---- 6. An toàn ----
  if (flightState == STATE_ARMED &&
      (fabsf(pitch) > SAFE_ANGLE || fabsf(roll) > SAFE_ANGLE)) {
    enterFailsafe("Nghieng qua 45 do");
    return;
  }

  // ---- 7. Điều khiển ----
  float pOut = 0.0f, rOut = 0.0f, yOut = 0.0f;
  int m1 = IDLE_THROTTLE, m2 = IDLE_THROTTLE, m3 = IDLE_THROTTLE, m4 = IDLE_THROTTLE;

  if (flightState == STATE_ARMED) {
    pOut = cascaded_compute(&pitchAxis, pitchSetpoint, pitch, pitchRate, dt);
    rOut = cascaded_compute(&rollAxis,  rollSetpoint,  roll,  rollRate,  dt);
    yOut = pid_compute(&yawRatePid,
                       constrain(yawRateSetpoint, -MAX_YAW_RATE, MAX_YAW_RATE),
                       yawRate, dt);

    motor_mix((float)throttleCmd,
              pOut * MIX_PITCH_SIGN, rOut * MIX_ROLL_SIGN, yOut * MIX_YAW_SIGN,
              &m1, &m2, &m3, &m4);
    escWriteAll(m1, m2, m3, m4);
  } else {
    // DISARMED / FAILSAFE: giữ IDLE và xả tích phân
    escIdle();
    cascaded_reset(&pitchAxis);
    cascaded_reset(&rollAxis);
    pid_reset(&yawRatePid);
  }
  lastP = pOut; lastR = rOut; lastY = yOut;

  // ---- 8. Debug Serial (10Hz) ----
  if (nowUs - lastSerialUs >= SERIAL_US) {
    lastSerialUs = nowUs;
    Serial.print(stateName());
    Serial.print(" T:");  Serial.print(throttleCmd);
    Serial.print(" | P:"); Serial.print(pitch, 1);
    Serial.print(" R:");   Serial.print(roll, 1);
    Serial.print(" Yr:");  Serial.print(yawRate, 1);
    Serial.print(" | o:"); Serial.print(lastP, 0);
    Serial.print(",");     Serial.print(lastR, 0);
    Serial.print(",");     Serial.print(lastY, 0);
    Serial.print(" | M:"); Serial.print(curM1);
    Serial.print(" ");     Serial.print(curM2);
    Serial.print(" ");     Serial.print(curM3);
    Serial.print(" ");     Serial.print(curM4);
    if (!accTrust)  Serial.print("  [acc?]");
    if (loopOverrun) Serial.print("  [overrun]");
    Serial.println();
  }

  // ---- 9. OLED (4Hz) — chặn ~23ms, đặt lại lịch sau khi vẽ ----
  if (oledOK && (nowUs - lastOledUs >= OLED_US)) {
    lastOledUs = nowUs;
    updateOled();
    uint32_t after = micros();
    nextLoopUs = after + LOOP_US;
    lastTimeUs = after;        // tránh dt nhảy vọt ở chu kỳ kế tiếp
  }
}
