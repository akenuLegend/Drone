#include <ESP32Servo.h>

// Định nghĩa các chân kết nối tín hiệu từ ESP32 đến 4 ESC
const int escPin1 = 13;
const int escPin2 = 14;
const int escPin3 = 26;
const int escPin4 = 27;

Servo esc1, esc2, esc3, esc4;

void setup() {
  Serial.begin(115200);
  
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

  Serial.println("--- BẮT ĐẦU QUY TRÌNH CALIB ESC ---");+
  Serial.println("1. Đang gửi tín hiệu MAX (2000us)...");
  
  // Gửi mức ga cao nhất (2000) đến cả 4 ESC cùng lúc
  esc1.writeMicroseconds(2000);
  esc2.writeMicroseconds(2000);
  esc3.writeMicroseconds(2000);
  esc4.writeMicroseconds(2000);

  Serial.println(" BÂY GIỜ: Hãy cắm nguồn PIN vào ESC!");
  Serial.println("Chờ nghe tiếng bíp báo Max Ga (khoảng 2-3 giây)...");
  
  // Cho bạn 8 giây để cắm pin vào ESC kể từ khi board khởi động
  delay(20000); 

  Serial.println("2. Đang gửi tín hiệu MIN (1000us)...");
  // Kéo ngay về mức thấp nhất
  esc1.writeMicroseconds(1000);
  esc2.writeMicroseconds(1000);
  esc3.writeMicroseconds(1000);
  esc4.writeMicroseconds(1000);

  Serial.println("Chờ tiếng bíp xác nhận Min Ga và nhạc khởi động xong.");
  delay(4000);
  
  Serial.println("--- CALIB HOÀN TẤT! ---");
}

void loop() {
  // Để trống, tuyệt đối không cho motor quay ở đây khi đang test
}