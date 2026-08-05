




/*
 * ESP-NOW Serial bridge - NODE A (ESP32 30 chan) - BEN NHAN (RX)
 *
 * Nhan du lieu tu NODE B (ESP32 38 chan) qua ESP-NOW roi in nguyen xi ra Serial.
 *   MAC NODE A (board nay) : 3C:8A:1F:A7:8D:40
 *   MAC NODE B (nguon gui) : A0:B7:65:F6:43:10
 *
 * Du lieu nhan trong callback duoc day vao ring buffer, loop() moi in ra
 * Serial -> khong lam nghen tac vu WiFi.
 *
 * Board: "ESP32 Dev Module" - Serial 115200
 */

#include <WiFi.h>
#include <esp_now.h>
#include <esp_wifi.h>

#define NODE_NAME       "ESP32-30pin"
#define WIFI_CHANNEL    1

/* Noi xuat du lieu nhan duoc.
 * - Serial  : xem tren Serial Monitor / day len PC qua USB
 * - Serial2 : day tiep sang MCU khac - nho Serial2.begin(...) trong setup()
 */
#define OUT_SERIAL      Serial

#define RING_SIZE       2048
#define ONLY_FROM_PEER  1       // 1 = bo qua goi tu board la
#define SHOW_STATS      1       // 1 = in thong ke moi 5s (tat neu PC dang parse du lieu)

// MAC cua NODE B (ESP32 38 chan) - ben gui
static uint8_t peerMac[6] = {0xA0, 0xB7, 0x65, 0xF6, 0x43, 0x10};

static uint8_t  ring[RING_SIZE];
static volatile uint16_t ringHead = 0;
static volatile uint16_t ringTail = 0;

static volatile uint32_t rxPackets = 0;
static volatile uint32_t rxBytes   = 0;
static volatile uint32_t rxDropped = 0;

static void ringPush(const uint8_t *d, int len) {
  uint16_t head = ringHead;
  for (int i = 0; i < len; i++) {
    uint16_t next = (head + 1) % RING_SIZE;
    if (next == ringTail) {          // buffer day -> bo phan con lai
      rxDropped += (len - i);
      break;
    }
    ring[head] = d[i];
    head = next;
  }
  ringHead = head;
}

/* ---------- Callback khi nhan duoc goi ---------- */
#if ESP_ARDUINO_VERSION >= ESP_ARDUINO_VERSION_VAL(3, 0, 0)
void onDataRecv(const esp_now_recv_info_t *info, const uint8_t *data, int len) {
  const uint8_t *mac = info->src_addr;
#else
void onDataRecv(const uint8_t *mac, const uint8_t *data, int len) {
#endif
#if ONLY_FROM_PEER
  if (memcmp(mac, peerMac, 6) != 0) return;
#endif
  rxPackets++;
  rxBytes += len;
  ringPush(data, len);
}

void setup() {
  Serial.begin(115200);
  delay(500);

  WiFi.mode(WIFI_STA);
  WiFi.disconnect();
  esp_wifi_set_channel(WIFI_CHANNEL, WIFI_SECOND_CHAN_NONE);

  Serial.println();
  Serial.println("=== ESP-NOW BRIDGE - NODE A (" NODE_NAME ") - RX ===");
  Serial.print("MAC board nay : ");
  Serial.println(WiFi.macAddress());
  Serial.printf("Nhan tu       : %02X:%02X:%02X:%02X:%02X:%02X\n",
                peerMac[0], peerMac[1], peerMac[2],
                peerMac[3], peerMac[4], peerMac[5]);
  uint8_t ch; wifi_second_chan_t sec;
  esp_wifi_get_channel(&ch, &sec);
  Serial.printf("WiFi channel  : %d (dat=%d)\n", ch, WIFI_CHANNEL);

  if (esp_now_init() != ESP_OK) {
    Serial.println("LOI: esp_now_init that bai -> restart");
    delay(1000);
    ESP.restart();
  }

  esp_now_register_recv_cb(onDataRecv);

  // Them peer (khong bat buoc de nhan, nhung can neu sau nay muon gui nguoc lai)
  esp_now_peer_info_t peer;
  memset(&peer, 0, sizeof(peer));
  memcpy(peer.peer_addr, peerMac, 6);
  peer.channel = WIFI_CHANNEL;
  peer.encrypt = false;

  if (esp_now_add_peer(&peer) != ESP_OK) {
    Serial.println("CANH BAO: khong them duoc peer (van nhan duoc du lieu)");
  }

  Serial.println("San sang. Dang cho du lieu tu NODE B...");
  Serial.println();
}

void loop() {
  // Xa ring buffer ra Serial
  while (ringTail != ringHead) {
    OUT_SERIAL.write(ring[ringTail]);
    ringTail = (ringTail + 1) % RING_SIZE;
  }

}

