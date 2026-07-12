#include <ESP32Servo.h>

// Cấu hình 4 chân AN TOÀN (Đã thay GPIO 12 bằng GPIO 26)
const int escPin1 = 26; 
const int escPin2 = 13;
const int escPin3 = 14;
const int escPin4 = 27;

Servo esc1, esc2, esc3, esc4;

void setup() {
  Serial.begin(115200);
  
  // Thiết lập chu kỳ xung 50Hz tiêu chuẩn
  esc1.setPeriodHertz(50); 
  esc2.setPeriodHertz(50);
  esc3.setPeriodHertz(50);
  esc4.setPeriodHertz(50);

  // Khởi tạo chân xuất xung (Đồng bộ dải 1000us - 2000us đã calib)
  esc1.attach(escPin1, 1000, 2000);
  esc2.attach(escPin2, 1000, 2000);
  esc3.attach(escPin3, 1000, 2000);
  esc4.attach(escPin4, 1000, 2000);

  // BƯỚC AN TOÀN: Gửi mức ga 0% ngay khi vừa khởi động
  Serial.println("Hệ thống khởi động. Đang giữ ga ở mức 0% (An toàn)...");
  esc1.writeMicroseconds(1000);
  esc2.writeMicroseconds(1000);
  esc3.writeMicroseconds(1000);
  esc4.writeMicroseconds(1000);
  
  // Chờ 5 giây để bạn chuẩn bị tinh thần và ESC hoàn tất tiếng nhạc khởi động
  delay(5000); 

  // KÍCH HOẠT ĐỘNG CƠ 50%
  // Serial.println("CẢNH BÁO: Đang kích hoạt 4 động cơ chạy 50% ga!");
  esc1.writeMicroseconds(1100);
  esc2.writeMicroseconds(1100);
  esc3.writeMicroseconds(1100);
  esc4.writeMicroseconds(1100);
}

void loop() {
  for(int i = 1100; i<1400; i+=10){
    esc1.writeMicroseconds(i);
    esc2.writeMicroseconds(i);
    esc3.writeMicroseconds(i);
    esc4.writeMicroseconds(i);
    delay(100);
  }

  delay(1000);

  for(int i = 1400; i>1100; i-=10){
    esc1.writeMicroseconds(i);
    esc2.writeMicroseconds(i);
    esc3.writeMicroseconds(i);
    esc4.writeMicroseconds(i);
    delay(100);
  }

  delay(1000);



}