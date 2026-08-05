/*
 * Sketch chan doan - chi in MAC cua board ra Serial moi 1 giay.
 * Nap vao board ESP32 30 chan de lay dung MAC dien vao peerMac ben 38 chan.
 *
 * MAC cua giao dien STA moi la cai ESP-NOW dung.
 * MAC cua SoftAP thuong lech byte cuoi -> dien nham la ESP-NOW FAIL.
 *
 * Board: "ESP32 Dev Module" - Serial 115200
 */

#include <WiFi.h>
#include <esp_wifi.h>

void setup() {
  Serial.begin(115200);
  delay(500);
  WiFi.mode(WIFI_STA);

  Serial.println();
  Serial.println("=== DOC MAC ESP32 ===");
}

void loop() {
  uint8_t sta[6], ap[6];
  esp_wifi_get_mac(WIFI_IF_STA, sta);
  esp_wifi_get_mac(WIFI_IF_AP,  ap);

  Serial.printf("STA (dung cho ESP-NOW) : %02X:%02X:%02X:%02X:%02X:%02X\n",
                sta[0], sta[1], sta[2], sta[3], sta[4], sta[5]);
  Serial.printf("SoftAP (KHONG dung)    : %02X:%02X:%02X:%02X:%02X:%02X\n",
                ap[0], ap[1], ap[2], ap[3], ap[4], ap[5]);
  Serial.println();

  delay(1000);
}
