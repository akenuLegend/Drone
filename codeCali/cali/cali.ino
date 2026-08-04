#include <ESP32Servo.h>
#include <Wire.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>

// Định nghĩa các chân kết nối tín hiệu từ ESP32 đến 4 ESC
// const int escPin1 = 26;
// const int escPin2 = 13;
// const int escPin3 = 14;
// const int escPin4 = 27;

const int escPin1 = 13;
const int escPin2 = 26;
const int escPin3 = 27;
const int escPin4 = 14;

Servo esc1, esc2, esc3, esc4;

// ---- OLED: cấu hình giống hệt drone_complete.ino (I2C bus 1, tách khỏi MPU) ----
#define SCREEN_WIDTH   128
#define SCREEN_HEIGHT  64
#define OLED_RESET     -1
#define OLED_SDA       21
#define OLED_SCL       22
#define OLED_ADDR      0x3C
#define OLED_FREQ      400000UL

TwoWire I2C_OLED = TwoWire(1);
Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &I2C_OLED, OLED_RESET);
bool oledOK = false;

// OLED dùng font ASCII → mọi chữ hiển thị KHÔNG DẤU. Dùng chung chuỗi cho cả
// Serial để hai bên luôn nói cùng một điều, không lệch nhau khi sửa code.

void oledHeader(const char *left, const char *right) {
  display.clearDisplay();
  display.setTextSize(1);
  display.setCursor(0, 0);
  display.print(left);
  if (right[0]) {
    display.setCursor(SCREEN_WIDTH - strlen(right) * 6, 0);
    display.print(right);
  }
  display.drawFastHLine(0, 10, SCREEN_WIDTH, SSD1306_WHITE);
}

// Trạng thái thường: tiêu đề + 3 dòng nội dung
void showStatus(const char *step, const char *l1, const char *l2, const char *l3) {
  Serial.printf("[%s] %s | %s | %s\n", step, l1, l2, l3);
  if (!oledOK) return;

  oledHeader(step, "");
  display.setTextSize(1);
  display.setCursor(0, 18); display.print(l1);
  display.setCursor(0, 32); display.print(l2);
  display.setCursor(0, 46); display.print(l3);
  display.display();
}

// Đếm ngược: số giây in TO + thanh tiến trình, nhìn được khi hai tay đang bận
void showCountdown(const char *step, const char *action, const char *hint,
                   int secsLeft, int secsTotal) {
  Serial.printf("[%s] %s — con %ds\n", step, action, secsLeft);
  if (!oledOK) return;

  oledHeader(step, "");
  display.setTextSize(1);
  display.setCursor(0, 13);
  display.print(action);

  // Số giây cỡ 3 (18x24 px/ký tự), căn giữa theo số chữ số
  int digits = (secsLeft >= 10) ? 2 : 1;
  display.setTextSize(3);
  display.setCursor((SCREEN_WIDTH - digits * 18) / 2, 22);
  display.print(secsLeft);

  display.setTextSize(1);
  display.setCursor(0, 48);
  display.print(hint);

  int w = (SCREEN_WIDTH - 4) * secsLeft / secsTotal;
  display.drawRect(0, 56, SCREEN_WIDTH, 8, SSD1306_WHITE);
  display.fillRect(2, 58, w, 4, SSD1306_WHITE);
  display.display();
}

void countdown(int seconds, const char *step, const char *action, const char *hint) {
  for (int i = seconds; i > 0; i--) {
    showCountdown(step, action, hint, i, seconds);
    delay(1000);
  }
}

// Màn hình cảnh báo khi loop() quét ga — motor QUAY THẬT ở đoạn này
void oledMotorRun(int us, const char *dir) {
  if (!oledOK) return;
  oledHeader("MOTOR DANG CHAY", "");
  display.setTextSize(3);
  display.setCursor(14, 22);
  display.print(us);
  display.setTextSize(1);
  display.setCursor(104, 30);
  display.print("us");
  display.setCursor(0, 54);
  display.print(dir);
  display.display();
}

void writeAll(int us) {
  esc1.writeMicroseconds(us);
  esc2.writeMicroseconds(us);
  esc3.writeMicroseconds(us);
  esc4.writeMicroseconds(us);
}

void setup() {
  Serial.begin(115200);
  delay(200);

  // ---- OLED trước tiên, để mọi bước sau đều hiển thị được ----
  I2C_OLED.begin(OLED_SDA, OLED_SCL, OLED_FREQ);
  if (!display.begin(SSD1306_SWITCHCAPVCC, OLED_ADDR)) {
    Serial.println("[WARN] Khong thay OLED - chi con Serial.");
    oledOK = false;
  } else {
    oledOK = true;
    display.setTextColor(SSD1306_WHITE);
  }

  showStatus("CALIB ESC", "THAO CANH QUAT!", "Rut PIN khoi ESC", "Dang khoi dong...");
  delay(2000);

  // Cấp cho mỗi kênh PWM một timer LEDC RIÊNG, phải gọi TRƯỚC attach().
  // Thiếu bước này, 4 servo bị ép dùng chung timer nên độ rộng xung xuất ra
  // KHÔNG giống nhau giữa các chân — cùng một lệnh µs mà motor quay khác tốc độ.
  ESP32PWM::allocateTimer(0);
  ESP32PWM::allocateTimer(1);
  ESP32PWM::allocateTimer(2);
  ESP32PWM::allocateTimer(3);

  // Cấu hình tần số PWM cho ESC (thường dải 1000us - 2000us)
  esc1.setPeriodHertz(50);
  esc2.setPeriodHertz(50);
  esc3.setPeriodHertz(50);
  esc4.setPeriodHertz(50);

  // Đính kèm các chân pin
  esc1.attach(escPin1, 1000, 2000);
  esc2.attach(escPin2, 1000, 2000);
  esc3.attach(escPin3, 1000, 2000);
  esc4.attach(escPin4, 1000, 2000);

  showStatus("SAN SANG", "4 ESC da gan PWM", "50Hz 1000-2000us", "Bat dau calib...");
  delay(1500);

  // ---- Bước 1: gửi MAX trước khi ESC có điện ----
  writeAll(2000);
  showStatus("BUOC 1/3", "Da gui MAX", "2000us ra 4 ESC", "Chuan bi cam pin");
  delay(1500);

  countdown(10, "BUOC 1/3", ">> CAM PIN NGAY <<", "Bip = da nhan MAX");

  // ---- Bước 2: hạ về MIN, ESC chốt dải và ghi EEPROM ----
  writeAll(1000);
  showStatus("BUOC 2/3", "Da gui MIN", "1000us ra 4 ESC", "Cho tieng bip...");
  delay(3000);

  countdown(4, "BUOC 2/3", "DANG LUU EEPROM", "Cho bip + nhac");
  delay(3000);

  // ---- Bước 3: xong ----
  showStatus("BUOC 3/3", "CALIB HOAN TAT!", "4 ESC cung 1 dai", "1000 - 2000us");
  delay(3000);

  showStatus("CANH BAO", "Motor SAP QUAY", "Kiem tra canh quat", "Bat dau sau 3s");
  delay(3000);
}

void loop() {
  delay(1000);

  for (int i = 1000; i < 1400; i += 5) {
    writeAll(i);
    if (i % 100 == 0) oledMotorRun(i, "Dang TANG ga");
    delay(20);
  }

  delay(1000);

  for (int i = 1400; i > 1000; i -= 5) {
    writeAll(i);
    if (i % 100 == 0) oledMotorRun(i, "Dang GIAM ga");
    delay(20);
  }
}
