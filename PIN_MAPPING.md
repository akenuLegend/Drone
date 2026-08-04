# 🚁 ESP32 Pin Mapping - Drone Flight Controller

**Phiên bản:** v1.1 | **Cập nhật:** 2025-08-04 | **Board:** ESP32 Dev Kit V1

---

## 📑 Mục lục

1. [Tổng quan](#tổng-quan)
2. [I2C - Cảm biến](#1-i2c---cảm-biến-mpu6050--oled)
3. [PWM - Động cơ ESC](#2-pwm---điều-khiển-động-cơ-esc-ledc)
4. [SPI - RF Module](#3-spi---rf-module-nrf24l01)
5. [UART - GPS](#4-uart---gps-module)
6. [Tóm tắt GPIO](#5-tóm-tắt-gpio-được-sử-dụng)
7. [Diagram kết nối](#6-diagram-kết-nối)
8. [Cảnh báo & Ghi chú](#7-ghi-chú--cảnh-báo)

---

## 🎯 Tổng quan

| Thành phần | Chi tiết |
|-----------|---------|
| 🖥️ **Bộ vi xử lý** | ESP32 Classic (Dual Core, 240MHz) |
| 📡 **Cảm biến IMU** | MPU6050 (Gyro + Accel) |
| 🖼️ **Màn hình** | OLED SSD1306 128x64 |
| 📶 **RF Module** | NRF24L01 (2.4GHz) |
| 🛰️ **GPS** | u-blox (RX2/TX2 + PPS) |
| 🔌 **Động cơ** | 4x Brushless ESC (X-frame) |

---

## 1️⃣ I2C - Cảm biến (MPU6050 & OLED)

### Bảng kết nối

| Thiết bị | Chân | GPIO | Chức năng | Tần số | I2C Addr |
|---------|------|------|----------|--------|----------|
| **MPU6050** | SDA | **32** | Data Line | 400kHz | **0x68** |
| | SCL | **33** | Clock Line | 400kHz | — |
| **OLED SSD1306** | SDA | **21** | Data Line | 400kHz | **0x3C** |
| | SCL | **22** | Clock Line | 400kHz | — |

### ⚡ Đặc điểm

- **2 kênh I2C riêng biệt** (không chia sẻ bus)
- MPU6050 dùng `Wire.begin(32, 33)`
- OLED dùng `TwoWire(1)` với GPIO 21, 22
- Cả hai hoạt động độc lập, không xung đột

---

## 2️⃣ PWM - Điều khiển động cơ ESC (LEDC)

### Bảng motor configuration

| Motor | GPIO | Vị trí | Chiều xoay | LEDC CH | Tần số |
|-------|------|--------|-----------|---------|--------|
| **M1** | 26 | 🔴 FL (Trước-Trái) | ↻ CW | 0 | **200Hz** |
| **M2** | 13 | 🟢 FR (Trước-Phải) | ↺ CCW | 1 | **200Hz** |
| **M3** | 14 | 🔵 BR (Sau-Phải) | ↻ CW | 2 | **200Hz** |
| **M4** | 27 | 🟡 BL (Sau-Trái) | ↺ CCW | 3 | **200Hz** |

### ⚙️ Cấu hình PWM

```
PWM Frequency   : 200Hz (thay vì 50Hz để giảm trễ ~7.5ms)
Bit Resolution  : 16-bit (0-65535)
Signal Range    : 1000μs (IDLE) → 1550μs (MAX)
Hover Point     : 1350μs
Min Throttle    : 1150μs
Idle Throttle   : 1000μs
```

---

## 3️⃣ SPI - RF Module NRF24L01

### Bảng kết nối SPI

| Chân NRF24 | Chức năng | GPIO | Ghi chú |
|-----------|----------|------|---------|
| **SCK** | Serial Clock | **18** | SPI Clock Signal |
| **MOSI** | Out → In | **23** | Data: ESP32 → NRF24 |
| **MISO** | In ← Out | **19** | Data: NRF24 → ESP32 |
| **CSN** | Chip Select | **5** | Active LOW (Chọn chip) |
| **CE** | Chip Enable | **25** | Mode Transmit/Receive |

### 📡 RF Cấu hình

- **Giao thức:** SPI (3-wire data)
- **Tần số RF:** 2.4 GHz (ISM band)
- **Chế độ:** Remote Control (Điều khiển từ xa)
- **Thư viện:** `RF24` hoặc tương tự

---

## 4️⃣ UART - GPS Module (u-blox)

### Bảng kết nối UART

| GPS ← → ESP32 | GPIO ESP32 | Chức năng | Ghi chú |
|--------------|-----------|----------|---------|
| **GPS TX** | **16** (RX2) | ← Nhận GPS Data | UART2 |
| **GPS RX** | **17** (TX2) | → Gửi lệnh GPS | UART2 |
| **GPS PPS** | **4** (GPIO4) | Pulse Per Second | Đồng bộ hóa 1Hz |

### ⚠️ Kết nối chéo (Cross-wired)

```
┌──────────────┬──────────────────┐
│  ESP32       │  GPS Module      │
├──────────────┼──────────────────┤
│ GPIO16 (RX2) │ ← TX (dữ liệu từ GPS)
│ GPIO17 (TX2) │ → RX (lệnh đến GPS)
│ GPIO4 (PPS)  │ ← PPS (đồng bộ)
│ GND          │ ← GND (mass)
└──────────────┴──────────────────┘
```

### 🔧 Cấu hình UART

- **Baud Rate:** 9600 hoặc 115200 bps
- **Parity:** None
- **Stop Bits:** 1
- **Data Bits:** 8
- **Flow Control:** None

---

## 5️⃣ Tóm tắt GPIO được sử dụng

### 📍 I2C Pins
```
GPIO 21, 22    ← OLED (I2C #0)
GPIO 32, 33    ← MPU6050 (I2C #1)
```

### 📍 SPI Pins
```
GPIO 5         ← CSN (NRF24 Chip Select)
GPIO 18        ← SCK (SPI Clock)
GPIO 19        ← MISO (SPI Data In)
GPIO 23        ← MOSI (SPI Data Out)
GPIO 25        ← CE (NRF24 Transmit/Receive)
```

### 📍 UART Pins
```
GPIO 4         ← PPS (GPS Pulse)
GPIO 16        ← RX2 (GPS Receive)
GPIO 17        ← TX2 (GPS Transmit)
```

### 📍 PWM/LEDC Pins
```
GPIO 13        ← M2 ESC (Channel 1)
GPIO 14        ← M3 ESC (Channel 2)
GPIO 26        ← M1 ESC (Channel 0)
GPIO 27        ← M4 ESC (Channel 3)
```

### 📍 GPIO Sẵn có (Trống)
```
GPIO 0, 2, 3, 6-12, 15, 20, 24, 28-39
(tùy loại ESP32 và cấu hình)
```

---

## 6️⃣ Diagram kết nối

```
╔═════════════════════════════════════════════════════════╗
║                    ESP32 Dev Kit V1                     ║
╠═════════════════════════════════════════════════════════╣
║                                                          ║
║  ┌─────────────────────────────────────────────────┐   ║
║  │  I2C #0 (OLED)    │    I2C #1 (MPU6050)         │   ║
║  │  SDA: GPIO21      │    SDA: GPIO32              │   ║
║  │  SCL: GPIO22      │    SCL: GPIO33              │   ║
║  └─────────────────────────────────────────────────┘   ║
║                                                          ║
║  ┌─────────────────────────────────────────────────┐   ║
║  │  PWM/LEDC (ESC Motors - X-frame)                │   ║
║  │  M1(FL): GPIO26  │  M2(FR): GPIO13               │   ║
║  │  M3(BR): GPIO14  │  M4(BL): GPIO27               │   ║
║  └─────────────────────────────────────────────────┘   ║
║                                                          ║
║  ┌─────────────────────────────────────────────────┐   ║
║  │  SPI (NRF24L01 RF Module)                       │   ║
║  │  SCK:  GPIO18   │  MOSI: GPIO23                 │   ║
║  │  MISO: GPIO19   │  CSN:  GPIO5                  │   ║
║  │  CE:   GPIO25                                   │   ║
║  └─────────────────────────────────────────────────┘   ║
║                                                          ║
║  ┌─────────────────────────────────────────────────┐   ║
║  │  UART2 (GPS Module - u-blox)                    │   ║
║  │  RX: GPIO16 (← GPS TX) ← CHÉO!                  │   ║
║  │  TX: GPIO17 (→ GPS RX) ← CHÉO!                  │   ║
║  │  PPS: GPIO4                                     │   ║
║  └─────────────────────────────────────────────────┘   ║
║                                                          ║
╚═════════════════════════════════════════════════════════╝
```

---

## 7️⃣ Ghi chú & Cảnh báo

### ⛔ KHÔNG SỬ DỤNG

| GPIO | Lý do |
|------|-------|
| **6-11** | Dùng cho SPI Flash (không có) |
| **34-39** | Input-only (không PWM) |

### ⚠️ CẢN THẬN

| GPIO | Chú ý |
|------|-------|
| **0** | Pull-down, cẩn thận khi boot |
| **15** | Pull-up, cẩn thận khi boot |
| **4-13** | ADC2 conflict với WiFi |

### ✅ Đã Xác Minh

- ✓ Không xung đột I2C, SPI, UART
- ✓ PWM chỉ trên GPIO hỗ trợ LEDC
- ✓ PPS có thể trigger interrupt
- ✓ Tất cả thiết bị hoạt động độc lập

---

## 📚 Hướng dẫn sử dụng

### NRF24L01 (Điều khiển từ xa)
```cpp
#include <RF24.h>
RF24 radio(25, 5);  // CE, CSN
radio.begin();
radio.openReadingPipe(0, address);
```

### GPS u-blox (UART2)
```cpp
#include <HardwareSerial.h>
HardwareSerial gpsSerial(2);  // UART2
gpsSerial.begin(9600, SERIAL_8N1, 16, 17);  // RX, TX
```

### MPU6050 (I2C #1)
```cpp
Wire.begin(32, 33);  // SDA, SCL
Wire.setClock(400000);
```

### OLED SSD1306 (I2C #0)
```cpp
TwoWire I2C_OLED = TwoWire(1);
I2C_OLED.begin(21, 22, 400000);  // SDA, SCL, Freq
```

---

## 🔗 Liên kết nhanh

- **Tài liệu ESP32:** https://docs.espressif.com/
- **MPU6050 Datasheet:** Register Map & DMP
- **NRF24L01 Driver:** RF24 Library GitHub
- **u-blox GPS:** Protocol Reference

---

**Made with ❤️ for Drone FC** | Drone v2.0 Mapping Document
