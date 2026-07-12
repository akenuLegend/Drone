// #include <Wire.h>
// #include <Adafruit_GFX.h>
// #include <Adafruit_SSD1306.h>
// #include <Adafruit_MPU6050.h>
// #include <Adafruit_Sensor.h>

// #define SCREEN_WIDTH 128
// #define SCREEN_HEIGHT 64
// #define OLED_RESET -1

// TwoWire I2C_OLED = TwoWire(1); 
// Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &I2C_OLED, OLED_RESET);
// Adafruit_MPU6050 mpu;

// // Biến phục vụ tính toán thời gian dt
// unsigned long mpuLastTime = 0;

// // Góc sau khi lọc bù
// float pitch = 0, roll = 0, yaw = 0;

// // Hệ số bộ lọc bù (96% tin tưởng Gyro đường dài, 4% tin tưởng Accel tức thời)
// const float alpha = 0.96; 

// void setup() {
//   Serial.begin(115200);

//   Wire.begin(32, 33, 400000);  
//   I2C_OLED.begin(21, 22, 100000); 

//   if (!mpu.begin(0x68, &Wire)) {
//     while (1);
//   }
//   if (!display.begin(SSD1306_SWITCHCAPVCC, 0x3C)) {
//     while (1);
//   }

//   display.clearDisplay();
//   display.setTextColor(SSD1306_WHITE);
//   display.setTextSize(1);
  
//   mpuLastTime = millis();
// }

// void loop() {
//   // sensors_event_t a, g, temp;
//   // mpu.getEvent(&a, &g, &temp);

//   // // 1. Tính khoảng thời gian dt (giây) giữa 2 lần đọc
//   // unsigned long currentTime = millis();
//   // float dt = (currentTime - mpuLastTime) / 1000.0;
//   // mpuLastTime = currentTime;

//   // // 2. Tính góc từ Gia tốc (Accelerometer) - đơn vị: Độ
//   // // Biến đổi đơn vị từ rad sang độ qua hệ số 180 / PI ~ 57.29578
//   // float pitchAcc = atan2(a.acceleration.y, sqrt(a.acceleration.x * a.acceleration.x + a.acceleration.z * a.acceleration.z)) * 57.29578;
//   // float rollAcc  = atan2(-a.acceleration.x, a.acceleration.z) * 57.29578;

//   // // 3. Tính vận tốc góc từ Gyro (đổi từ rad/s sang độ/s)
//   // float gyroXRate = g.gyro.x * 57.29578;
//   // float gyroYRate = g.gyro.y * 57.29578;
//   // float gyroZRate = g.gyro.z * 57.29578;

//   // // 4. Áp dụng Bộ lọc bù (Complementary Filter)
//   // pitch = alpha * (pitch + gyroXRate * dt) + (1.0 - alpha) * pitchAcc;
//   // roll  = alpha * (roll + gyroYRate * dt) + (1.0 - alpha) * rollAcc;
  
//   // // Yaw không có gốc tọa độ từ Accel nên tính thuần bằng tích phân Gyro
//   // yaw   = yaw + gyroZRate * dt; 

//   // // Giới hạn hiển thị góc Yaw trong khoảng -180 đến 180 độ cho đẹp
//   // if (yaw > 180) yaw -= 360;
//   // if (yaw < -180) yaw += 360;

//   // // 5. Hiển thị lên màn hình OLED
//   // display.clearDisplay();
  
//   // display.setCursor(0, 0);
//   // display.println("--- ORIENTATION ---");

//   // display.setCursor(0, 20);
//   // display.print("Pitch (X) : "); 
//   // display.print(pitch, 1);
//   // display.print((char)247); // Ký tự độ (°)

//   // display.setCursor(0, 35);
//   // display.print("Roll  (Y) : "); 
//   // display.print(roll, 1);
//   // display.print((char)247);

//   // display.setCursor(0, 50);
//   // display.print("Yaw   (Z) : "); 
//   // display.print(yaw, 1);
//   // display.print((char)247);

//   // display.display();

//   // delay(20); // Chu kỳ lấy mẫu nhỏ (20ms tương đương 50Hz) giúp tích phân chính xác hơn
//  // ... (các phần khai báo phía trên giữ nguyên)

// void loop() {
//   sensors_event_t a, g, temp;
//   mpu.getEvent(&a, &g, &temp);

//   // 1. Tính khoảng thời gian dt
//   unsigned long currentTime = millis();
//   float dt = (currentTime - mpuLastTime) / 1000.0;
//   mpuLastTime = currentTime;

//   // 2 & 3 & 4. Tính toán góc (giữ nguyên logic của bạn)
//   float pitchAcc = atan2(a.acceleration.y, sqrt(a.acceleration.x * a.acceleration.x + a.acceleration.z * a.acceleration.z)) * 57.29578;
//   float rollAcc  = atan2(-a.acceleration.x, a.acceleration.z) * 57.29578;
//   float gyroXRate = g.gyro.x * 57.29578;
//   float gyroYRate = g.gyro.y * 57.29578;
//   float gyroZRate = g.gyro.z * 57.29578;

//   pitch = alpha * (pitch + gyroXRate * dt) + (1.0 - alpha) * pitchAcc;
//   roll  = alpha * (roll + gyroYRate * dt) + (1.0 - alpha) * rollAcc;
//   yaw   = yaw + gyroZRate * dt; 

//   if (yaw > 180) yaw -= 360;
//   if (yaw < -180) yaw += 360;

//   // --- MỚI: Xuất dữ liệu ra Serial ---
//   // Cách này giúp bạn nhìn thấy số liệu rõ ràng trên Serial Monitor
//   Serial.print("Pitch:");
//   Serial.print(pitch);
//   Serial.print(" | Roll:");
//   Serial.print(roll);
//   Serial.print(" | Yaw:");
//   Serial.println(yaw);

//   /* GỢI Ý: Nếu bạn muốn dùng Serial Plotter, hãy comment (//) 3 dòng Serial.print ở trên 
//      và bỏ comment dòng dưới đây để đồ thị chạy mượt mà hơn:
//   */
//   // Serial.print(pitch); Serial.print(","); Serial.print(roll); Serial.print(","); Serial.println(yaw);
  
//   // 5. Hiển thị lên màn hình OLED (giữ nguyên)
//   display.clearDisplay();
//   display.setCursor(0, 0);
//   display.println("--- ORIENTATION ---");
//   display.setCursor(0, 20);
//   display.print("Pitch: "); display.print(pitch, 1);
//   display.setCursor(0, 35);
//   display.print("Roll : "); display.print(roll, 1);
//   display.setCursor(0, 50);
//   display.print("Yaw  : "); display.print(yaw, 1);
//   display.display();

//   delay(20);
// }
// }

#include <Wire.h>
#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>

Adafruit_MPU6050 mpu;

unsigned long mpuLastTime = 0;
float pitch = 0, roll = 0, yaw = 0;
const float alpha = 0.96;

void setup() {
  Serial.begin(115200);
  Wire.begin(32, 33, 400000); // SDA=32, SCL=33

  if (!mpu.begin(0x68, &Wire)) {
    Serial.println("Không tìm thấy MPU6050!");
    while (1);
  }

  Serial.println("MPU6050 sẵn sàng!");
  mpuLastTime = millis();
}

void loop() {
  sensors_event_t a, g, temp;
  mpu.getEvent(&a, &g, &temp);

  // 1. Tính dt (giây)
  unsigned long currentTime = millis();
  float dt = (currentTime - mpuLastTime) / 1000.0;
  mpuLastTime = currentTime;

  // 2. Góc từ Accelerometer
  float pitchAcc = atan2(a.acceleration.y,
                    sqrt(a.acceleration.x * a.acceleration.x +
                         a.acceleration.z * a.acceleration.z)) * 57.29578;
  float rollAcc  = atan2(-a.acceleration.x, a.acceleration.z) * 57.29578;

  // 3. Vận tốc góc từ Gyro (rad/s → °/s)
  float gyroXRate = g.gyro.x * 57.29578;
  float gyroYRate = g.gyro.y * 57.29578;
  float gyroZRate = g.gyro.z * 57.29578;

  // 4. Bộ lọc bù (Complementary Filter)
  pitch = alpha * (pitch + gyroXRate * dt) + (1.0 - alpha) * pitchAcc;
  roll  = alpha * (roll  + gyroYRate * dt) + (1.0 - alpha) * rollAcc;
  yaw   = yaw + gyroZRate * dt;

  if (yaw >  180) yaw -= 360;
  if (yaw < -180) yaw += 360;

  // 5. Xuất ra Serial Monitor
  Serial.print("Pitch: "); Serial.print(pitch, 2);
  Serial.print(" | Roll: ");  Serial.print(roll,  2);
  Serial.print(" | Yaw: ");   Serial.println(yaw,  2);

  // --- Bỏ comment dòng dưới nếu dùng Serial Plotter ---
  // Serial.print(pitch); Serial.print(",");
  // Serial.print(roll);  Serial.print(",");
  // Serial.println(yaw);

  delay(20); // 50Hz
}