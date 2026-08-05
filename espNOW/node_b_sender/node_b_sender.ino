/*
 * NODE B - TRAM MAT DAT cho drone_espnow_final
 *
 *   MAC board nay (NODE B) : A0:B7:65:F6:43:10
 *   MAC drone   (NODE A)   : 3C:8A:1F:A7:8D:40
 *
 * CHUC NANG
 *   1. Go mot so vao Serial Monitor roi Enter -> gui sang drone lam HOVER_THROTTLE.
 *   2. NHAN TOAN BO SO LIEU tu drone (10Hz) va hien thi - drone khong con in ra
 *      Serial cua no nua, day la cho DUY NHAT doc duoc thong so khi dang bay.
 *   3. Cu 5 giay in mot BANG TONG KET + CHAN DOAN nham thang vao hai trieu chung
 *      "truc yaw bi troi" va "drone do ve phia truoc du da can bang".
 *   4. Gui nhip tim "PING" moi 500ms va bao mat/khoi phuc lien lac.
 *
 * ============================================================================
 * DOC SO LIEU THE NAO - phan quan trong nhat cua file nay
 * ============================================================================
 *
 * Mot mo hinh duy nhat can nam: bo dieu khien LUON co gang dua goc do duoc ve
 * setpoint. Vi vay goc do duoc gan 0 KHONG co nghia la drone can bang - no chi
 * co nghia la bo dieu khien dang thang. Cai gia phai tra de thang nam o cho
 * khac: TICH PHAN (I) va DO LECH GIUA CAC DONG CO. Do moi la thuoc do "co khi
 * lech bao nhieu". Toan bo cach doc duoi day xoay quanh viec doi chieu ba thu:
 *
 *      GOC do duoc   <->   TICH PHAN I   <->   DO LECH DONG CO
 *
 * HAI DAI LUONG NODE B TU TINH (drone khong gui, vi tu no khong noi len gi):
 *   FB = (M1+M2)/2 - (M3+M4)/2   luc day TRUOC tru SAU.
 *        FB > 0 : hai dong co TRUOC dang phai chay manh hon -> drone NANG DAU
 *        TRUOC (trong tam lech ve truoc), bo dieu khien dang ghi mui len.
 *   CW = (M1+M3)/2 - (M2+M4)/2   luc day canh CW tru canh CCW.
 *        Khac 0 lien tuc = khung dang co MO-MEN XOAN DU, vong yaw phai bu mai.
 *
 * --- TRIEU CHUNG 1: DRONE DO / TROI VE PHIA TRUOC ---
 *   Doc avg P (goc pitch trung binh), I(P) va FB. Ba kha nang, ba cach sua khac
 *   han nhau - dung sua mu:
 *
 *   (a) avg P ~ 0  va  |FB| lon  va  |I(P)| CHUA cham tran
 *       Bo dieu khien dang giu duoc goc 0 va no lam duoc viec do. Nhung drone
 *       van troi ve truoc => "0 do" ma no dang giu KHONG PHAI la nam ngang that.
 *       Mat phang chuan bi lech tu luc hieu chuan (MPU dan hoi nghieng, chan de
 *       cao thap, ban khong that ngang).
 *       SUA: hieu chuan lai tren mat phang chac chan ngang. Neu van con, dat
 *            PITCH_TRIM_DEG trong file drone, TANG dan tung 0.5 do de mui ngoc
 *            len. Khong the tinh san con so nay tu so lieu - chinh cai thuoc do
 *            goc dang sai - nen bat buoc phai thu tung nac.
 *
 *   (b) avg P am ro (mui chui xuong)  va  |I(P)| BAM TRAN (~40)
 *       Bo dieu khien da dung het quyen luc ma van khong keo noi mui len. Day la
 *       loi CO KHI/CAU HINH, them trim chi lam no nang them.
 *       SUA: doi pin/board ve phia SAU cho can trong tam. Neu can khong duoc,
 *            tang THROTTLE_BAND tu 100 len 200 trong file drone (PID duoc gap
 *            doi du dia - dung y nhu ban v2 goc).
 *
 *   (c) avg P lech ngay khi CHUA ARM (dong co chua quay)
 *       Khong phai chuyen dieu khien - la uoc luong tu the sai. Xem offset LEVEL
 *       va bias trong bang tong ket, hieu chuan lai khi drone nam that yen.
 *
 * --- TRIEU CHUNG 2: TRUC YAW BI TROI ---
 *   Truoc het phai biet: vong yaw cua ban firmware nay giu TOC DO XOAY = 0, no
 *   KHONG giu huong. Huong (Y) troi cham la dung thiet ke, khong phai loi. Chi
 *   khi TOC DO xoay lech khoi 0 moi la co van de. Hai nguyen nhan:
 *
 *   (a) avg yr khac 0 ro rang (>3 do/s)  ->  drone dang thuc su xoay va vong yaw
 *       khong ghim noi. Xem I(Y):
 *         |I(Y)| bam tran (~20) : mo-men xoan du VUOT quyen luc vong yaw. Nguyen
 *              nhan co khi: mot dong co gan hoi nghieng, canh quat khong dong
 *              bo, hoac mot ESC yeu hon han.
 *              SUA: kiem tra do thang trang cua 4 truc dong co, doi canh quat
 *                   thanh mot bo dong nhat. Chi khi da lam roi ma van con moi
 *                   tang RATE_I_LIMIT (yaw) tu 20 len 30-40.
 *         |I(Y)| con nho : vong yaw phan ung yeu -> tang YAW_RATE_KP/KI.
 *
 *   (b) avg yr ~ 0 nhung MAT BAN nhin thay drone xoay  ->  day la TROI BIAS
 *       GYRO Z, KHONG phai loi PID. Con quay bao "dang dung yen" trong khi drone
 *       xoay that, nen PID khong co gi de sua. Doi chieu bias cal Z (do luc hieu
 *       chuan) voi bias bay Z (Madgwick hoc them luc bay): hai so lech nhau
 *       nhieu = con quay troi theo nhiet do.
 *       SUA: cho MPU am may 1-2 phut roi moi hieu chuan; hieu chuan lai ngay
 *            truoc moi lan bay; giu drone TUYET DOI yen luc hieu chuan. Khong
 *            co cach nao chua triet de bang phan mem - muon ghim huong that thi
 *            phai co la ban (magnetometer) hoac quang hoc.
 *
 * --- KIEM TRA SUC KHOE TRUOC KHI TIN VAO MOI KET LUAN TREN ---
 *   V (rung) > 0.3 hoac W < 0.90 hoac clip > 0  =>  uoc luong tu the dang bi
 *   rung lam sai lech. Can bang lai canh quat TRUOC, moi tinh chuyen tune - moi
 *   con so goc luc do deu khong dang tin.
 *
 * ============================================================================
 * LENH GO VAO SERIAL MONITOR
 *   <so>   dat HOVER_THROTTLE cho drone (1050..1750)
 *   h      bang lenh nay
 *   f      hien thi moi goi (10Hz)      n  hien thi thua (2Hz)      o  tat
 *   c      bat/tat che do CSV (de luu file / ve do thi)
 *   s      in tong ket ngay bay gio     r  xoa thong ke, dem lai tu dau
 *
 * Board: "ESP32 Dev Module" - Serial 115200
 */

#include <WiFi.h>
#include <esp_now.h>
#include <esp_wifi.h>
#include <ctype.h>

#define NODE_NAME        "NODE B"
#define WIFI_CHANNEL     1

#define HEARTBEAT_MS     500     // nhip tim khi khong go gi
#define LINK_FAIL_LIMIT  3       // truot lien tiep bao nhieu lan thi coi la dut
#define LINK_REPORT_MS   3000    // dang mat ket noi thi nhac lai moi bao lau
#define SEND_TIMEOUT_MS  50      // cho goi truoc gui xong
// KHONG dat ten LINE_MAX: <limits.h> da co macro do, trung ten se de bep dinh
// nghia chuan cua he thong (canh bao "redefined" luc bien dich).
#define CMD_LINE_MAX     32      // do dai toi da mot dong go vao

// Mien drone chap nhan - PHAI khop HOVER_MIN/MAX_ALLOWED trong file drone.
#define HOVER_MIN_ALLOWED 1050
#define HOVER_MAX_ALLOWED 1750
#define THROTTLE_BAND     100    // khop THROTTLE_BAND cua drone

// Tran tich phan cua drone - dung de biet khau I da bam tran hay chua.
// Khop RATE_I_LIMIT_PR va RATE_I_LIMIT trong file drone.
#define I_LIMIT_PR        40.0f
#define I_LIMIT_YAW       20.0f

// MAC cua NODE A (drone). Doi o day neu doi board.
static uint8_t peerMac[6] = {0x3C, 0x8A, 0x1F, 0xA7, 0x8D, 0x40};

/* ==========================================================================
 * GOI TELEMETRY - PHAI GIONG TUNG BYTE voi khoi cung ten trong
 * drone_espnow_final.ino. Sua mot ben ma quen ben kia thi goi van du do dai va
 * NODE B se in ra so rac ma khong bao loi gi - vi the moi lan doi truong deu
 * phai TANG TELEM_VER o CA HAI file, khi do ben nay tu choi va bao ro rang.
 * ========================================================================== */
#define TELEM_MAGIC 0xD1
#define TELEM_VER   1

typedef struct __attribute__((packed)) {
  uint8_t  magic;        // 0   TELEM_MAGIC
  uint8_t  ver;          // 1   TELEM_VER
  uint8_t  mode;         // 2   0=DISARMED 1=ARMED 2=TRIPPED
  uint8_t  flags;        // 3   bit0 linkUp | 1 overrun | 2 calibrated | 3 clip | 4 espnowReady

  uint32_t ms;           // 4   millis() tren drone
  uint16_t loopHz;       // 8   nhip dieu khien do duoc
  uint16_t seq;          // 10  dem goi -> tinh duoc ti le mat goi

  float pitch, roll, yaw;             // 12  do
  float pitchRate, rollRate, yawRate; // 24  do/s, da loc
  float outP, outR, outY;             // 36  output PID (us)
  float iPitch, iRoll, iYaw;          // 48  TICH PHAN
  float biasX, biasY, biasZ;          // 60  bias Madgwick hoc luc bay (do/s)
  float calBiasX, calBiasY, calBiasZ; // 72  bias do duoc luc hieu chuan (do/s)
  float pitchOffset, rollOffset;      // 84  offset LEVEL
  float vibX, vibY, vibZ;             // 92  bien do rung dinh trong ky (g)
  float accW;                         // 104 trong so tin cay accel trung binh

  int16_t  m1, m2, m3, m4;            // 108 xung 4 dong co (us)
  int16_t  throttle;                  // 116 ga lenh
  int16_t  hover;                     // 118 HOVER dang hieu luc
  int16_t  hoverTarget;               // 120 HOVER dang bo toi
  uint16_t clip;                      // 122 so mau bao hoa trong ky
  uint16_t rx, bad, lost;             // 124 dem goi lenh drone nhan duoc
  char     reason[20];                // 130 modeReason
} TelemPacket;                        // 150 byte

static_assert(sizeof(TelemPacket) == 150,
              "TelemPacket lech kich thuoc so voi drone_espnow_final.ino!");

/* ---------- Hang doi goi den ----------
 * Callback ESP-NOW chay TRONG task WiFi. In Serial o do se chan driver hang
 * milli giay va lam rot chinh nhung goi tiep theo. Nen callback chi chep byte
 * va day queue; moi viec dinh dang/in nam o loop().
 */
#define RX_QUEUE_LEN 6
#define RX_BUF_MAX   200

typedef struct {
  uint8_t len;
  uint8_t data[RX_BUF_MAX];
} RxMsg;

static QueueHandle_t rxQueue = NULL;

static char  line[CMD_LINE_MAX];
static int   lineLen = 0;
static unsigned long lastSendMs = 0;

// Sua trong callback (task WiFi), doc trong loop (core 1) -> phai volatile.
static volatile bool     sendPending = false;
static volatile uint32_t failStreak  = 0;   // so lan truot LIEN TIEP
static volatile uint32_t txOk = 0, txFail = 0;
static volatile uint32_t rxDrop = 0;        // goi rot vi queue day

static bool          linkUp       = false;
static bool          linkEverUp   = false;
static unsigned long downSinceMs  = 0;
static unsigned long lastReportMs = 0;

/* ---------- Trang thai hien thi ---------- */
static int  showEvery = 1;        // in 1 tren moi N goi; 0 = tat han
static bool csvMode   = false;
static bool csvHeaderDone = false;

/* ---------- Thong ke cua so ----------
 * Chi gom mau khi drone DANG ARM: so lieu luc nam yen tren ban khong noi len
 * dieu gi ve can bang khi bay, gop chung vao se pha loang trung binh.
 * Dung double vi cong don hang nghin mau float o quanh mot gia tri lech nho se
 * mat dan chu so - ma "lech nho keo dai" chinh la thu dang di tim.
 */
#define SUMMARY_MS 5000

typedef struct {
  uint32_t n;
  double   pitch, roll, yawRate, iP, iR, iY, fb, cw;
  float    pitchMin, pitchMax, rollMin, rollMax;
  float    yawFirst, yawLast;
  uint32_t msFirst, msLast;
  uint32_t clip, overrun;
  float    vibMax, accWMin;
} Stats;

static Stats       st;
static TelemPacket lastPkt;
static bool        havePkt   = false;
static uint32_t    pktSeen   = 0;   // so goi telemetry da nhan
static uint16_t    lastSeq   = 0;
static uint32_t    seqMissed = 0;   // so goi bi mat, suy ra tu truong seq
static unsigned long lastSummaryMs = 0;
static unsigned long lastTelemMs   = 0;

static void statsReset() {
  memset(&st, 0, sizeof(st));
  st.pitchMin = st.rollMin =  9999.0f;
  st.pitchMax = st.rollMax = -9999.0f;
  st.accWMin  = 9999.0f;
  st.vibMax   = 0.0f;
}

static void statsAdd(const TelemPacket *p) {
  float fb = ((float)p->m1 + p->m2) * 0.5f - ((float)p->m3 + p->m4) * 0.5f;
  float cw = ((float)p->m1 + p->m3) * 0.5f - ((float)p->m2 + p->m4) * 0.5f;

  if (st.n == 0) { st.yawFirst = p->yaw; st.msFirst = p->ms; }
  st.yawLast = p->yaw;
  st.msLast  = p->ms;

  st.n++;
  st.pitch   += p->pitch;
  st.roll    += p->roll;
  st.yawRate += p->yawRate;
  st.iP      += p->iPitch;
  st.iR      += p->iRoll;
  st.iY      += p->iYaw;
  st.fb      += fb;
  st.cw      += cw;

  if (p->pitch < st.pitchMin) st.pitchMin = p->pitch;
  if (p->pitch > st.pitchMax) st.pitchMax = p->pitch;
  if (p->roll  < st.rollMin)  st.rollMin  = p->roll;
  if (p->roll  > st.rollMax)  st.rollMax  = p->roll;

  float v = fmaxf(p->vibX, fmaxf(p->vibY, p->vibZ));
  if (v > st.vibMax)     st.vibMax  = v;
  if (p->accW < st.accWMin) st.accWMin = p->accW;

  st.clip += p->clip;
  if (p->flags & 0x02) st.overrun++;
}

static const char* modeName(uint8_t m) {
  return (m == 2) ? "SAFE" : (m == 1 ? "ARM" : "IDLE");
}

/* ---------- Callback bao ket qua gui: nguon tin ve lien lac ---------- */
#if ESP_ARDUINO_VERSION >= ESP_ARDUINO_VERSION_VAL(3, 1, 0)
void onDataSent(const wifi_tx_info_t *info, esp_now_send_status_t status) {
#else
void onDataSent(const uint8_t *mac, esp_now_send_status_t status) {
#endif
  if (status == ESP_NOW_SEND_SUCCESS) {
    txOk++;
    failStreak = 0;           // mot lan thanh cong la xoa sach chuoi truot
  } else {
    txFail++;
    failStreak++;
  }
  sendPending = false;
}

/* ---------- Callback nhan: chi chep va day queue ---------- */
#if ESP_ARDUINO_VERSION >= ESP_ARDUINO_VERSION_VAL(3, 0, 0)
void onDataRecv(const esp_now_recv_info_t *info, const uint8_t *data, int len) {
  const uint8_t *mac = info->src_addr;
#else
void onDataRecv(const uint8_t *mac, const uint8_t *data, int len) {
#endif
  if (memcmp(mac, peerMac, 6) != 0) return;    // chi nghe drone
  if (len <= 0 || rxQueue == NULL) return;

  RxMsg m;
  m.len = (len < RX_BUF_MAX) ? (uint8_t)len : (uint8_t)RX_BUF_MAX;
  memcpy(m.data, data, m.len);

  // Timeout 0: tuyet doi khong chan task WiFi. Queue day thi bo goi - mat mot
  // dong so lieu con hon lam nghen driver dang phai lo ca viec gui lenh ga.
  if (xQueueSend(rxQueue, &m, 0) != pdTRUE) rxDrop++;
}

/* ---------- Gui mot goi ---------- */
static bool sendPacket(const char *payload) {
  // Cho goi truoc hoan tat de khong lam tran hang doi cua ESP-NOW.
  unsigned long t0 = millis();
  while (sendPending && millis() - t0 < SEND_TIMEOUT_MS) delay(1);

  esp_err_t r = esp_now_send(peerMac, (const uint8_t *)payload, strlen(payload) + 1);
  lastSendMs = millis();
  if (r != ESP_OK) {
    Serial.print("[TX] esp_now_send loi: "); Serial.println((int)r);
    return false;
  }
  sendPending = true;
  return true;
}

/* ---------- Hien thi mot goi ---------- */
static void printCsvHeader() {
  Serial.println("ms,mode,loopHz,pitch,roll,yaw,pRate,rRate,yRate,outP,outR,outY,"
                 "iP,iR,iY,m1,m2,m3,m4,FB,CW,thr,hover,vibX,vibY,vibZ,accW,clip");
  csvHeaderDone = true;
}

static void printCsv(const TelemPacket *p) {
  int fb = (int)lroundf(((float)p->m1 + p->m2) * 0.5f - ((float)p->m3 + p->m4) * 0.5f);
  int cw = (int)lroundf(((float)p->m1 + p->m3) * 0.5f - ((float)p->m2 + p->m4) * 0.5f);
  char l[260];
  snprintf(l, sizeof(l),
    "%lu,%u,%u,%.2f,%.2f,%.1f,%.1f,%.1f,%.1f,%.1f,%.1f,%.1f,"
    "%.2f,%.2f,%.2f,%d,%d,%d,%d,%d,%d,%d,%d,%.3f,%.3f,%.3f,%.2f,%u",
    (unsigned long)p->ms, (unsigned)p->mode, (unsigned)p->loopHz,
    p->pitch, p->roll, p->yaw,
    p->pitchRate, p->rollRate, p->yawRate,
    p->outP, p->outR, p->outY,
    p->iPitch, p->iRoll, p->iYaw,
    p->m1, p->m2, p->m3, p->m4, fb, cw,
    p->throttle, p->hover,
    p->vibX, p->vibY, p->vibZ, p->accW, (unsigned)p->clip);
  Serial.println(l);
}

static void printLive(const TelemPacket *p) {
  int fb = (int)lroundf(((float)p->m1 + p->m2) * 0.5f - ((float)p->m3 + p->m4) * 0.5f);
  int cw = (int)lroundf(((float)p->m1 + p->m3) * 0.5f - ((float)p->m2 + p->m4) * 0.5f);

  char flag[8];
  int  fi = 0;
  if (p->flags & 0x02) flag[fi++] = '!';   // overrun vong dieu khien
  if (p->flags & 0x08) flag[fi++] = 'C';   // co mau bao hoa
  if (!(p->flags & 0x01)) flag[fi++] = 'X'; // drone dang khong nghe thay NODE B
  flag[fi] = '\0';

  char l[240];
  snprintf(l, sizeof(l),
    "%7.1fs %-4s%4uHz P%+6.1f R%+6.1f Y%+6.0f | r%+6.1f%+6.1f%+6.1f "
    "| o%+5.0f%+5.0f%+5.0f | I%+6.1f%+6.1f%+6.1f "
    "| M%5d%5d%5d%5d | FB%+5d CW%+5d | H%d T%d | V%.2f W%.2f %s%s",
    p->ms / 1000.0f, modeName(p->mode), (unsigned)p->loopHz,
    p->pitch, p->roll, p->yaw,
    p->pitchRate, p->rollRate, p->yawRate,
    p->outP, p->outR, p->outY,
    p->iPitch, p->iRoll, p->iYaw,
    p->m1, p->m2, p->m3, p->m4, fb, cw,
    p->hover, p->throttle,
    fmaxf(p->vibX, fmaxf(p->vibY, p->vibZ)), p->accW,
    flag, (p->mode == 1) ? "" : p->reason);
  Serial.println(l);
}

/* ---------- Bang tong ket + chan doan ----------
 * Moi ket luan duoi day deu duoc rut ra tu viec DOI CHIEU it nhat hai dai luong,
 * khong bao gio tu mot con so don le. Ly do o dau file: goc do duoc mot minh
 * khong phan biet duoc "can bang that" voi "bo dieu khien dang gong".
 */
static void printSummary() {
  if (st.n == 0) {
    Serial.println("[TONG KET] chua co mau nao trong luc ARM - chua ket luan duoc gi.");
    if (havePkt) {
      Serial.printf("           So lieu tinh: offset LEVEL P%+.2f R%+.2f | bias cal %+.3f %+.3f %+.3f do/s\n",
                    lastPkt.pitchOffset, lastPkt.rollOffset,
                    lastPkt.calBiasX, lastPkt.calBiasY, lastPkt.calBiasZ);
      if (fabsf(lastPkt.pitchOffset) > 3.0f || fabsf(lastPkt.rollOffset) > 3.0f)
        Serial.println("           * offset LEVEL > 3 do: drone da duoc hieu chuan tren mat KHONG ngang.");
    }
    return;
  }

  const float n   = (float)st.n;
  const float aP  = st.pitch   / n;
  const float aR  = st.roll    / n;
  const float aYr = st.yawRate / n;
  const float aIP = st.iP / n, aIR = st.iR / n, aIY = st.iY / n;
  const float aFB = st.fb / n, aCW = st.cw / n;
  const float secs = (st.msLast - st.msFirst) / 1000.0f;

  float yawDrift = st.yawLast - st.yawFirst;          // do, trong cua so
  while (yawDrift >  180.0f) yawDrift -= 360.0f;      // gap vong -180..180
  while (yawDrift < -180.0f) yawDrift += 360.0f;

  Serial.println();
  Serial.printf("========== TONG KET %.1fs / %lu mau ARM ==========\n",
                secs, (unsigned long)st.n);
  Serial.printf(" Goc TB       P%+7.2f (%.1f..%.1f)   R%+7.2f (%.1f..%.1f)\n",
                aP, st.pitchMin, st.pitchMax, aR, st.rollMin, st.rollMax);
  Serial.printf(" Toc do yaw   %+.2f do/s  ->  huong troi %+.0f do trong %.1fs\n",
                aYr, yawDrift, secs);
  Serial.printf(" Tich phan    I(P)%+7.2f/%.0f   I(R)%+7.2f/%.0f   I(Y)%+7.2f/%.0f\n",
                aIP, I_LIMIT_PR, aIR, I_LIMIT_PR, aIY, I_LIMIT_YAW);
  Serial.printf(" Dong co      FB(truoc-sau)%+6.1fus   CW(cw-ccw)%+6.1fus\n", aFB, aCW);
  Serial.printf(" Hieu chuan   offset LEVEL P%+.2f R%+.2f | bias cal %+.3f %+.3f %+.3f\n",
                lastPkt.pitchOffset, lastPkt.rollOffset,
                lastPkt.calBiasX, lastPkt.calBiasY, lastPkt.calBiasZ);
  Serial.printf(" Bias bay     %+.3f %+.3f %+.3f do/s (Madgwick hoc them)\n",
                lastPkt.biasX, lastPkt.biasY, lastPkt.biasZ);
  Serial.printf(" Suc khoe     rung dinh %.2f g | accW min %.2f | clip %lu | overrun %lu\n",
                st.vibMax, st.accWMin, (unsigned long)st.clip, (unsigned long)st.overrun);
  Serial.printf(" Duong truyen goi telem %lu (mat %lu) | drone nhan lenh %u bad %u lost %u\n",
                (unsigned long)pktSeen, (unsigned long)seqMissed,
                (unsigned)lastPkt.rx, (unsigned)lastPkt.bad, (unsigned)lastPkt.lost);

  Serial.println(" --- CHAN DOAN ---");
  bool said = false;

  // (0) Suc khoe cam bien phai xet TRUOC: rung manh lam moi con so goc mat nghia,
  //     ket luan tune dua tren so lieu do se dan di sai duong.
  if (st.vibMax > 0.30f || st.accWMin < 0.90f || st.clip > 0) {
    Serial.println(" * RUNG MANH / accel khong tin duoc. Can bang canh quat, siet gia do,");
    Serial.println("   lot dem MPU TRUOC DA. Moi ket luan ben duoi chua dang tin luc nay.");
    said = true;
  }

  // (1) Truc pitch - ba kha nang loai tru nhau, xem giai thich dau file.
  if (fabsf(aP) <= 1.0f && fabsf(aFB) >= 10.0f) {
    Serial.printf(" * PITCH: goc giu duoc ~0 nhung phai day lech %+.0fus giua truoc va sau.\n", aFB);
    Serial.printf("   %s. Neu drone VAN troi ve %s:\n",
                  aFB > 0 ? "Trong tam lech ve TRUOC" : "Trong tam lech ve SAU",
                  aFB > 0 ? "truoc" : "sau");
    Serial.println("   mat phang \"0 do\" bi lech tu luc hieu chuan -> hieu chuan lai tren mat");
    Serial.printf("   that ngang; con lai thi dat PITCH_TRIM_DEG %s dan tung 0.5 do.\n",
                  aFB > 0 ? "TANG" : "GIAM");
    said = true;
  } else if (fabsf(aP) > 1.0f && fabsf(aIP) >= 0.7f * I_LIMIT_PR) {
    Serial.printf(" * PITCH: goc lech %+.1f do MA khau I da bam tran (%+.1f/%.0f).\n",
                  aP, aIP, I_LIMIT_PR);
    Serial.println("   Bo dieu khien het quyen luc - day la loi CO KHI, them trim se lam nang them.");
    Serial.printf("   Doi pin/board ve phia %s cho can trong tam; hoac tang THROTTLE_BAND 100 -> 200.\n",
                  aP < 0 ? "SAU" : "TRUOC");
    said = true;
  } else if (fabsf(aP) > 1.0f) {
    Serial.printf(" * PITCH: goc lech %+.1f do nhung khau I moi o %+.1f/%.0f - chua dung het suc.\n",
                  aP, aIP, I_LIMIT_PR);
    Serial.println("   Vong toc do bu qua cham: tang PITCH_RATE_KI (0.30 -> 0.45), giu nguyen KP.");
    said = true;
  }

  // (2) Truc roll - cung logic, gon hon.
  if (fabsf(aR) > 1.5f || fabsf(aIR) >= 0.7f * I_LIMIT_PR) {
    Serial.printf(" * ROLL: goc TB %+.1f do, I(R) %+.1f/%.0f -> lech trong tam sang %s.\n",
                  aR, aIR, I_LIMIT_PR, aR < 0 ? "TRAI" : "PHAI");
    said = true;
  }

  // (3) Truc yaw - phan biet "PID khong ghim noi" voi "con quay troi bias".
  if (fabsf(aYr) >= 3.0f) {
    Serial.printf(" * YAW: drone dang xoay THAT %+.1f do/s va vong yaw khong ghim noi.\n", aYr);
    if (fabsf(aIY) >= 0.7f * I_LIMIT_YAW) {
      Serial.printf("   Khau I yaw bam tran (%+.1f/%.0f): mo-men xoan du VUOT quyen luc vong yaw.\n",
                    aIY, I_LIMIT_YAW);
      Serial.println("   Sua CO KHI: kiem tra 4 truc dong co co thang trang khong, doi ca 4 canh");
      Serial.println("   quat thanh mot bo dong nhat. Chi khi da lam roi moi tang RATE_I_LIMIT 20 -> 35.");
    } else {
      Serial.printf("   Khau I yaw moi o %+.1f/%.0f - chua dung het suc: vong yaw phan ung qua yeu.\n",
                    aIY, I_LIMIT_YAW);
      Serial.println("   Tang YAW_RATE_KP (2.00 -> 3.00) truoc, roi YAW_RATE_KI (0.50 -> 0.80).");
    }
    said = true;
  } else if (fabsf(aCW) >= 8.0f) {
    Serial.printf(" * YAW: toc do xoay dang duoc ghim tot, nhung phai day lech %+.0fus giua canh\n", aCW);
    Serial.println("   CW va CCW de lam duoc. Khung co mo-men xoan du - hien chua thanh trieu chung,");
    Serial.println("   nhung no an mat du dia dieu khien. Kiem tra do nghieng cua gia do dong co.");
    said = true;
  } else {
    Serial.printf(" * YAW: toc do xoay TB chi %+.2f do/s - vong yaw dang lam dung viec.\n", aYr);
    Serial.println("   Neu MAT BAN van thay drone xoay, do la TROI BIAS GYRO Z chu khong phai PID:");
    Serial.printf("   con quay bao dung yen trong khi drone xoay that (bias cal Z %+.3f, bay Z %+.3f).\n",
                  lastPkt.calBiasZ, lastPkt.biasZ);
    Serial.println("   Cho MPU am may 1-2 phut roi hieu chuan lai ngay truoc khi bay.");
    said = true;
  }

  // (4) Nhac lai thiet ke de khong di sua nham cho: huong troi cham ma toc do
  //     xoay van ~0 la dac tinh cua vong yaw chi giu TOC DO, khong phai loi.
  if (fabsf(yawDrift) > 15.0f && fabsf(aYr) < 3.0f) {
    Serial.printf(" * Luu y: huong da troi %+.0f do nhung toc do xoay chi %+.2f do/s. Vong yaw\n",
                  yawDrift, aYr);
    Serial.println("   chi giu TOC DO chu khong giu HUONG, nen troi cham la dung thiet ke.");
    Serial.println("   Muon ghim huong that thi phai them la ban hoac cam bien quang hoc.");
  }

  if (!said)
    Serial.println(" * Khong thay bat thuong nao dang ke trong cua so nay.");

  Serial.println("==================================================");
  Serial.println();
}

/* ---------- Xu ly goi telemetry vua lay ra khoi queue ---------- */
static void handleTelem(const TelemPacket *p) {
  lastTelemMs = millis();

  // Mat goi suy ra tu truong seq: neu con so nay tang deu thi duong truyen dang
  // qua tai (hoac qua xa), va cac trung binh ben duoi thua mau dan.
  if (pktSeen > 0) {
    uint16_t gap = (uint16_t)(p->seq - lastSeq);
    if (gap > 1) seqMissed += (gap - 1);
  }
  lastSeq = p->seq;
  pktSeen++;

  lastPkt = *p;
  havePkt = true;

  if (p->mode == 1) statsAdd(p);   // chi gom mau luc ARM

  if (csvMode) {
    if (!csvHeaderDone) printCsvHeader();
    printCsv(p);
  } else if (showEvery > 0 && (pktSeen % (uint32_t)showEvery) == 0) {
    printLive(p);
  }
}

static void printHelp() {
  Serial.println();
  Serial.println("--- LENH ---");
  Serial.printf("  <so>  dat HOVER_THROTTLE (%d..%d)\n", HOVER_MIN_ALLOWED, HOVER_MAX_ALLOWED);
  Serial.println("  f     hien thi moi goi (10Hz)");
  Serial.println("  n     hien thi thua (2Hz)");
  Serial.println("  o     tat hien thi truc tiep (van gom thong ke)");
  Serial.println("  c     bat/tat CSV (de luu file hoac ve do thi)");
  Serial.println("  s     in tong ket + chan doan ngay");
  Serial.println("  r     xoa thong ke, dem lai tu dau");
  Serial.println("  h     bang lenh nay");
  Serial.println();
}

/* ---------- Xu ly mot dong vua go xong ---------- */
static void handleLine() {
  line[lineLen] = '\0';
  lineLen = 0;

  // Lenh chu phai xet TRUOC strtol: strtol("f") tra 0 va se bi hieu thanh mot
  // lenh ga bang 0 gui di.
  if (isalpha((unsigned char)line[0]) || line[0] == '?') {
    switch (tolower((unsigned char)line[0])) {
      case 'f': showEvery = 1; csvMode = false;
                Serial.println("[VIEW] hien thi moi goi (10Hz)."); return;
      case 'n': showEvery = 5; csvMode = false;
                Serial.println("[VIEW] hien thi thua (2Hz)."); return;
      case 'o': showEvery = 0; csvMode = false;
                Serial.println("[VIEW] tat hien thi truc tiep - van gom thong ke."); return;
      case 'c': csvMode = !csvMode; csvHeaderDone = false;
                Serial.print("[VIEW] CSV "); Serial.println(csvMode ? "BAT" : "TAT"); return;
      case 's': printSummary(); statsReset(); lastSummaryMs = millis(); return;
      case 'r': statsReset(); pktSeen = 0; seqMissed = 0; lastSummaryMs = millis();
                Serial.println("[STAT] da xoa thong ke."); return;
      case 'h':
      case '?': printHelp(); return;
      default:
        Serial.print("[!] lenh la: \""); Serial.print(line);
        Serial.println("\" - go h de xem bang lenh."); return;
    }
  }

  char *end = NULL;
  long v = strtol(line, &end, 10);

  if (end == line) {
    Serial.print("[!] \""); Serial.print(line);
    Serial.println("\" khong phai so - bo qua, khong gui.");
    return;
  }
  if (v < HOVER_MIN_ALLOWED || v > HOVER_MAX_ALLOWED) {
    Serial.print("[!] "); Serial.print(v);
    Serial.print(" ngoai mien drone chap nhan (");
    Serial.print(HOVER_MIN_ALLOWED); Serial.print("..");
    Serial.print(HOVER_MAX_ALLOWED);
    Serial.println(") - van gui, nhung drone se tu choi.");
  }

  // Gui lai dung chuoi so da chuan hoa, tranh gui kem rac.
  char payload[16];
  snprintf(payload, sizeof(payload), "%ld", v);

  if (sendPacket(payload)) {
    Serial.print("[TX] gui \""); Serial.print(payload);
    Serial.print("\"  -> HOVER=");   Serial.print(v);
    Serial.print(" MIN=");           Serial.print(v - THROTTLE_BAND);
    Serial.print(" MAX=");           Serial.println(v + THROTTLE_BAND);
    if (!linkUp)
      Serial.println("     (dang MAT KET NOI - lenh nay nhieu kha nang khong toi noi)");
  }
}

/* ---------- Trang thai lien lac ----------
 * txOk > 0 la dieu kien bat buoc: luc moi khoi dong failStreak van bang 0 nhung
 * chua gui duoc goi nao, khong the coi do la "dang ket noi".
 */
static void updateLinkState() {
  bool up = (txOk > 0) && (failStreak < LINK_FAIL_LIMIT);

  if (up != linkUp) {
    linkUp = up;
    if (up) {
      if (linkEverUp) {
        Serial.print("[LINK] === DA KET NOI LAI voi NODE A === (mat ");
        Serial.print((millis() - downSinceMs) / 1000.0f, 1);
        Serial.println("s)");
      } else {
        Serial.println("[LINK] === DA KET NOI voi NODE A ===");
      }
      linkEverUp = true;
    } else {
      downSinceMs  = millis();
      lastReportMs = millis();
      Serial.println("[LINK] *** MAT KET NOI voi NODE A ***");
      Serial.println("       drone van giu HOVER cuoi cung no nhan duoc.");
    }
  }

  // Nhac lai deu dan trong luc dang mat, keo khong nhin man hinh lai tuong on.
  // Nhanh "chua bao gio ket noi" phai co rieng, neu khong truong hop sai MAC se
  // im lang tuyet doi - dung cai truong hop can bao nhat.
  if (!linkUp && failStreak >= LINK_FAIL_LIMIT &&
      millis() - lastReportMs >= LINK_REPORT_MS) {
    lastReportMs = millis();
    if (linkEverUp) {
      Serial.print("[LINK] van chua thay NODE A - da ");
      Serial.print((millis() - downSinceMs) / 1000);
      Serial.println("s");
    } else {
      Serial.print("[LINK] CHUA ket noi duoc lan nao (");
      Serial.print((millis() - downSinceMs) / 1000);
      Serial.println("s) - kiem tra MAC dich, nguon NODE A, va WiFi channel.");
    }
  }
}

void setup() {
  Serial.begin(115200);
  delay(500);

  statsReset();

  WiFi.mode(WIFI_STA);
  WiFi.disconnect();
  esp_wifi_set_channel(WIFI_CHANNEL, WIFI_SECOND_CHAN_NONE);

  Serial.println();
  Serial.println("=== " NODE_NAME " - tram mat dat drone (ESP-NOW) ===");
  Serial.print("MAC board nay : ");
  Serial.println(WiFi.macAddress());
  Serial.printf("Gui toi drone : %02X:%02X:%02X:%02X:%02X:%02X\n",
                peerMac[0], peerMac[1], peerMac[2],
                peerMac[3], peerMac[4], peerMac[5]);

  uint8_t ch; wifi_second_chan_t sec;
  esp_wifi_get_channel(&ch, &sec);
  Serial.printf("WiFi channel  : %d (dat=%d)\n", ch, WIFI_CHANNEL);
  Serial.printf("Goi telemetry : %d byte, ver %d\n", (int)sizeof(TelemPacket), TELEM_VER);

  // Queue phai san sang TRUOC khi dang ky callback, neu khong goi den som se roi
  // vao callback luc rxQueue con NULL.
  rxQueue = xQueueCreate(RX_QUEUE_LEN, sizeof(RxMsg));
  if (rxQueue == NULL) {
    Serial.println("LOI: khong cap duoc queue -> restart");
    delay(1000);
    ESP.restart();
  }

  if (esp_now_init() != ESP_OK) {
    Serial.println("LOI: esp_now_init that bai -> restart");
    delay(1000);
    ESP.restart();
  }
  esp_now_register_send_cb(onDataSent);
  esp_now_register_recv_cb(onDataRecv);

  esp_now_peer_info_t peer;
  memset(&peer, 0, sizeof(peer));
  memcpy(peer.peer_addr, peerMac, 6);
  peer.channel = WIFI_CHANNEL;
  peer.encrypt = false;
  if (esp_now_add_peer(&peer) != ESP_OK) {
    Serial.println("LOI: khong them duoc peer -> restart");
    delay(1000);
    ESP.restart();
  }

  Serial.print("San sang. Go mot so trong khoang ");
  Serial.print(HOVER_MIN_ALLOWED); Serial.print("..");
  Serial.print(HOVER_MAX_ALLOWED); Serial.println(" roi Enter, hoac go h de xem lenh.");
  Serial.println("Dang cho so lieu tu drone...");
  Serial.println();

  downSinceMs   = millis();   // moc de dem "chua ket noi duoc bao lau"
  lastReportMs  = millis();
  lastSummaryMs = millis();
}

void loop() {
  // 1. Gom ky tu go vao
  while (Serial.available()) {
    char c = (char)Serial.read();
    if (c == '\n' || c == '\r') {
      if (lineLen > 0) handleLine();
    } else if (lineLen < CMD_LINE_MAX - 1) {
      line[lineLen++] = c;
    }
  }

  // 2. Rut het goi da nhan. Lam o day chu khong trong callback: in Serial trong
  //    callback se chan task WiFi va lam rot chinh nhung goi tiep theo.
  RxMsg m;
  while (rxQueue && xQueueReceive(rxQueue, &m, 0) == pdTRUE) {
    if (m.len == sizeof(TelemPacket) && m.data[0] == TELEM_MAGIC) {
      TelemPacket p;
      memcpy(&p, m.data, sizeof(p));
      if (p.ver != TELEM_VER) {
        Serial.printf("[RX] goi telemetry ver %u, ben nay cho ver %d - NAP LAI FIRMWARE cho khop.\n",
                      (unsigned)p.ver, TELEM_VER);
        continue;
      }
      // Chan '\0' bang tay: goi bi nhieu lam hong co the den ma khong con byte
      // ket thuc chuoi, va reason[] duoc dua thang vao %s ben duoi.
      p.reason[sizeof(p.reason) - 1] = '\0';
      handleTelem(&p);
    } else {
      // Khong phai goi so lieu: in ra dang van ban de khong nuot mat thong bao.
      char t[RX_BUF_MAX + 1];
      int k = 0;
      for (int i = 0; i < m.len && k < RX_BUF_MAX; i++) {
        char c = (char)m.data[i];
        if (c == '\0') break;
        t[k++] = (c >= 32 && c < 127) ? c : '.';
      }
      t[k] = '\0';
      Serial.print("[RX] "); Serial.println(t);
    }
  }

  // 3. Nhip tim khi khong co gi de gui
  if (millis() - lastSendMs >= HEARTBEAT_MS) sendPacket("PING");

  // 4. Bao trang thai lien lac
  updateLinkState();

  // 5. Tong ket dinh ky. Chi in khi da co mau ARM hoac dang mat so lieu han -
  //    khong bat man hinh phai cuon mot bang trong moi 5 giay luc dang de yen.
  if (millis() - lastSummaryMs >= SUMMARY_MS) {
    lastSummaryMs = millis();
    if (st.n > 0) {
      printSummary();
      statsReset();     // moi cua so 5s la mot phep do doc lap
    }
  }

  // 6. Bao khi dot ngot mat dong so lieu (drone reset / het pin / ngoai tam).
  if (pktSeen > 0 && lastTelemMs && millis() - lastTelemMs > 3000) {
    Serial.println("[TELEM] *** 3s khong nhan duoc so lieu nao tu drone ***");
    lastTelemMs = millis();
  }
}
