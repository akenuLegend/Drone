/*
 * ESP-NOW Serial bridge - NODE B (ESP32 38 chan) - BEN GUI (TX)
 *
 * Doc du lieu tu Serial roi day sang NODE A (ESP32 30 chan) qua ESP-NOW.
 *   MAC NODE A (dich) : 3C:8A:1F:A7:8D:40
 *   MAC NODE B (board nay) : A0:B7:65:F6:43:10
 *
 * Goi du lieu duoc "cat khuc" va gui khi thoa 1 trong 3 dieu kien:
 *   - gap ky tu '\n'  (gui het dong)
 *   - day 240 byte    (gioi han payload ESP-NOW la 250)
 *   - Serial im lang 20 ms (IDLE_FLUSH_MS) -> gui phan con lai
 *
 * Board: "ESP32 Dev Module" - Serial 115200
 */

#include <WiFi.h>
#include <esp_now.h>
#include <esp_wifi.h>

#define NODE_NAME       "ESP32-38pin"
#define WIFI_CHANNEL    1

/* Nguon du lieu can gui di.
 * - Serial  : go tay tren Serial Monitor / nhan tu PC qua USB
 * - Serial2 : nhan tu thiet bi ngoai (GPS, MCU khac...) - nho doi baud
 *             va goi Serial2.begin(9600, SERIAL_8N1, 16, 17) trong setup()
 */
#define SRC_SERIAL      Serial
#define SRC_BAUD        115200

#define CHUNK_MAX       240     // byte / goi ESP-NOW
#define IDLE_FLUSH_MS   20      // im lang bao lau thi day goi di
#define SEND_TIMEOUT_MS 50      // cho goi truoc gui xong
#define SHOW_STATS      1       // 1 = in thong ke moi 5s ra Serial

/* CHE DO TEST: 1 = gui broadcast thay vi gui dich danh.
 * Broadcast khong can ACK nen luon bao OK -> dung de kiem tra xem
 * ben nhan co thuc su nhan duoc khong, tach biet voi loi sai MAC.
 * Chay xong nho tra ve 0.
 */
#define USE_BROADCAST   0

// MAC cua NODE A (ESP32 30 chan) - ben nhan
static uint8_t peerMac[6] = {0x3C, 0x8A, 0x1F, 0xA7, 0x8D, 0x40};
static uint8_t bcastMac[6] = {0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF};

#if USE_BROADCAST
  #define DEST_MAC bcastMac
#else
  #define DEST_MAC peerMac
#endif

static uint8_t  txBuf[CHUNK_MAX];
static int      txLen = 0;
static unsigned long lastByteMs = 0;

static volatile bool sendPending = false;   // dang cho callback cua goi truoc
static uint32_t txPackets = 0;
static uint32_t txBytes   = 0;
static uint32_t txFailed  = 0;

/* ---------- Callback bao ket qua gui ---------- */
#if ESP_ARDUINO_VERSION >= ESP_ARDUINO_VERSION_VAL(3, 1, 0)
void onDataSent(const wifi_tx_info_t *info, esp_now_send_status_t status) {
#else
void onDataSent(const uint8_t *mac, esp_now_send_status_t status) {
#endif
  if (status != ESP_NOW_SEND_SUCCESS) {
    txFailed++;
    Serial.println("[TX] FAIL - khong nhan duoc ACK tu ben kia");
  }
  sendPending = false;
}

/* ---------- Day buffer hien tai di ---------- */
static void flushTx() {
  if (txLen <= 0) return;

  // Cho goi truoc hoan tat de khong lam tran hang doi cua ESP-NOW
  unsigned long t0 = millis();
  while (sendPending && millis() - t0 < SEND_TIMEOUT_MS) delay(1);

  esp_err_t r = esp_now_send(DEST_MAC, txBuf, txLen);
  if (r == ESP_OK) {
    sendPending = true;
    txPackets++;
    txBytes += txLen;
  } else {
    txFailed++;
  }
  txLen = 0;
}

void setup() {
  Serial.begin(115200);
#if SRC_BAUD != 115200
  // Neu SRC_SERIAL khong phai Serial thi mo cong nguon o day
  // vi du: Serial2.begin(SRC_BAUD, SERIAL_8N1, 16, 17);
#endif
  delay(500);

  WiFi.mode(WIFI_STA);
  WiFi.disconnect();
  esp_wifi_set_channel(WIFI_CHANNEL, WIFI_SECOND_CHAN_NONE);

  Serial.println();
  Serial.println("=== ESP-NOW BRIDGE - NODE B (" NODE_NAME ") - TX ===");
  Serial.print("MAC board nay : ");
  Serial.println(WiFi.macAddress());
  Serial.printf("Gui toi       : %02X:%02X:%02X:%02X:%02X:%02X %s\n",
                DEST_MAC[0], DEST_MAC[1], DEST_MAC[2],
                DEST_MAC[3], DEST_MAC[4], DEST_MAC[5],
                USE_BROADCAST ? "(BROADCAST - che do test)" : "");

  uint8_t ch; wifi_second_chan_t sec;
  esp_wifi_get_channel(&ch, &sec);
  Serial.printf("WiFi channel  : %d (dat=%d)\n", ch, WIFI_CHANNEL);

  if (esp_now_init() != ESP_OK) {
    Serial.println("LOI: esp_now_init that bai -> restart");
    delay(1000);
    ESP.restart();
  }

  esp_now_register_send_cb(onDataSent);

  esp_now_peer_info_t peer;
  memset(&peer, 0, sizeof(peer));
  memcpy(peer.peer_addr, DEST_MAC, 6);
  peer.channel = WIFI_CHANNEL;
  peer.encrypt = false;

  if (esp_now_add_peer(&peer) != ESP_OK) {
    Serial.println("LOI: khong them duoc peer");
    return;
  }

  Serial.println("San sang. Go du lieu roi Enter de gui sang NODE A.");
  Serial.println();
}

void loop() {
  // Gom byte tu cong nguon
  while (SRC_SERIAL.available() && txLen < CHUNK_MAX) {
    uint8_t c = SRC_SERIAL.read();
    txBuf[txLen++] = c;
    lastByteMs = millis();
    if (c == '\n') break;          // het 1 dong -> gui ngay
  }

  if (txLen > 0) {
    bool full    = (txLen >= CHUNK_MAX);
    bool endLine = (txBuf[txLen - 1] == '\n');
    bool idle    = (millis() - lastByteMs >= IDLE_FLUSH_MS);
    if (full || endLine || idle) flushTx();
  }

#if SHOW_STATS
  static unsigned long lastStat = 0;
  if (millis() - lastStat >= 5000) {
    lastStat = millis();
    if (txPackets > 0 || txFailed > 0) {
      Serial.printf("[STAT] goi=%lu  byte=%lu  loi=%lu\n",
                    (unsigned long)txPackets,
                    (unsigned long)txBytes,
                    (unsigned long)txFailed);
    }
  }
#endif
}
