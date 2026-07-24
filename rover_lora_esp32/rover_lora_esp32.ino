/*
 * Truck rover bridge: LC29H-DA + E22-900T30D LoRa (no WiFi) - slotted uplink.
 *
 *   corrections   E90-A ~~RF~~ E22 -> UART1 -> ESP32 -> UART2 -> LC29H
 *   telemetry     LC29H -> ESP32 -> E22 ~~RF~~ E90-C   (in this truck's slot)
 *
 * MEDIA ACCESS - beacon TDMA:
 *   The correction burst (every 2 s with --epoch-div 2) is the metronome.
 *   Every truck hears it; when it ENDS, each truck waits its own offset
 *   (SLOT_BASE + index*SLOT_WIDTH) and only then transmits its telemetry.
 *   No shared clock needed, no collisions by design:
 *
 *     t=0      [==corrections==]
 *     +200 ms   T1     +350 T2     +500 T3     +650 T4     +800 T5
 *
 *   Fallback: if no correction burst is heard for FALLBACK_AFTER_MS (the
 *   truck is shadowed), telemetry free-runs every ~2 s with random jitter
 *   so two shadowed trucks are unlikely to collide.
 *
 * On boot the ESP32 configures the E22 itself (channel/NetID/air rate) and
 * prints the register read-back over USB - verify once against the E90s.
 *
 * Wiring (ESP32 WROOM):
 *   LC29H TX -> GPIO16 (RX2)      E22 TXD -> GPIO26 (RX1)
 *   LC29H RX -> GPIO17 (TX2)      E22 RXD -> GPIO27 (TX1)
 *                                 E22 M0  -> GPIO32
 *                                 E22 M1  -> GPIO33
 *                                 E22 VCC -> 5V (VIN)  [1 W bursts ~600 mA]
 *                                 GND common everywhere
 */

#include <Arduino.h>

// ===========================================================================
//  CONFIGURATION
// ===========================================================================

#define TRUCK_ID        "T1"     // unique per truck: T1..T5
#define TRUCK_INDEX     1        // 1..5 - picks the uplink slot
#define DEBUG_ROVER     1        // USB logs (USB feeds nothing, safe to keep)

// Radio parameters - MUST equal the E90-DTU web config on A and C.
#define E22_CHANNEL     65       // 850.125 + 65 = 915.125 MHz
#define E22_NETID       20       // fleet standard - E90s must be set to 20 too
#define E22_ADDR        0x0000   // transparent broadcast group

#define E22_REG0        0xE6     // UART 115200 8N1 + air rate 38.4k
#define E22_REG1        0x03     // 240 B sub-packet + TX 21 dBm (bench);
                                 // field: 0x00 = 30 dBm
#define E22_REG3        0x03     // transparent, relay/LBT/WOR off

// Slot plan (milliseconds after the correction burst ends).
#define SLOT_BASE_MS        200
#define SLOT_WIDTH_MS       150
// A burst smaller than this is noise, not a correction epoch.
#define BURST_MIN_BYTES     150
// Burst is "ended" after this much RX silence.
#define BURST_GAP_MS         60
// No beacon for this long -> shadowed: free-run telemetry with jitter.
#define FALLBACK_AFTER_MS  5000
#define FALLBACK_PERIOD_MS 2000

const uint32_t GNSS_BAUD = 115200;
const uint32_t E22_BAUD_NORMAL = 115200;
const uint32_t E22_BAUD_CONFIG = 9600;    // E22 config mode is always 9600

const int PIN_GNSS_RX = 16, PIN_GNSS_TX = 17;
const int PIN_E22_RX  = 26, PIN_E22_TX  = 27;
const int PIN_M0 = 32, PIN_M1 = 33;
const int PIN_LED = 2;

// ===========================================================================

HardwareSerial GNSS(2);
HardwareSerial E22(1);

char nmeaLine[128];
size_t nmeaLen = 0;

// Latest GGA waiting for this truck's slot (newest always wins).
char pendingGga[128];
bool ggaPending = false;

uint32_t rtcmIn = 0, ggaSent = 0, helloSeq = 0;
uint32_t lastRxByte = 0, burstBytes = 0;
uint32_t beaconAt = 0;           // when the last correction burst ended
bool slotArmed = false;
uint32_t lastUplink = 0;

const uint32_t SLOT_OFFSET_MS = SLOT_BASE_MS + (TRUCK_INDEX - 1) * SLOT_WIDTH_MS;

void e22Mode(int m0, int m1) {
  digitalWrite(PIN_M0, m0);
  digitalWrite(PIN_M1, m1);
  delay(60);
}

bool e22Configure() {
  e22Mode(0, 1);
  E22.updateBaudRate(E22_BAUD_CONFIG);
  delay(100);
  while (E22.available()) E22.read();

  const uint8_t regs[9] = {
    (uint8_t)(E22_ADDR >> 8), (uint8_t)(E22_ADDR & 0xFF),
    E22_NETID, E22_REG0, E22_REG1, E22_CHANNEL, E22_REG3, 0x00, 0x00
  };
  uint8_t cmd[12] = {0xC0, 0x00, 0x09};
  memcpy(cmd + 3, regs, 9);
  E22.write(cmd, sizeof(cmd));
  E22.flush();
  delay(150);

#if DEBUG_ROVER
  Serial.print("[e22 ] set response:");
  while (E22.available()) Serial.printf(" %02X", E22.read());
  Serial.println();
#endif

  uint8_t query[3] = {0xC1, 0x00, 0x07};
  E22.write(query, sizeof(query));
  E22.flush();
  delay(150);

  uint8_t back[16];
  int n = 0;
  while (E22.available() && n < 16) back[n++] = E22.read();

#if DEBUG_ROVER
  Serial.print("[e22 ] read-back:");
  for (int i = 0; i < n; i++) Serial.printf(" %02X", back[i]);
  Serial.println("  (C1 00 07 ADDH ADDL NETID REG0 REG1 CH REG3)");
#endif

  bool ok = (n >= 10 && back[0] == 0xC1 &&
             back[5] == E22_NETID && back[8] == E22_CHANNEL);

  e22Mode(0, 0);
  E22.updateBaudRate(E22_BAUD_NORMAL);
  delay(100);
  return ok;
}

void handleNmeaLine() {
  nmeaLine[nmeaLen] = '\0';
  if (strstr(nmeaLine, "GGA")) {
    // Queue for the slot; a newer fix always replaces an unsent one.
    strncpy(pendingGga, nmeaLine, sizeof(pendingGga) - 1);
    pendingGga[sizeof(pendingGga) - 1] = '\0';
    ggaPending = true;
  }
  nmeaLen = 0;
}

void sendUplink(uint32_t now) {
  if (ggaPending) {
    E22.print(TRUCK_ID " ");
    E22.println(pendingGga);
    ggaPending = false;
    ggaSent++;
  } else {
    // Nothing fresh from the receiver: prove we are alive anyway.
    E22.printf("%s HELLO seq=%lu rtcm_in=%lu gga=%lu\r\n",
               TRUCK_ID, (unsigned long)++helloSeq,
               (unsigned long)rtcmIn, (unsigned long)ggaSent);
  }
  lastUplink = now;
#if DEBUG_ROVER
  Serial.printf("[slot] uplink at +%lums after beacon\n",
                (unsigned long)(now - beaconAt));
#endif
}

void setup() {
  pinMode(PIN_M0, OUTPUT);
  pinMode(PIN_M1, OUTPUT);
  pinMode(PIN_LED, OUTPUT);
  Serial.begin(115200);
  delay(400);

  E22.begin(E22_BAUD_CONFIG, SERIAL_8N1, PIN_E22_RX, PIN_E22_TX);
  GNSS.begin(GNSS_BAUD, SERIAL_8N1, PIN_GNSS_RX, PIN_GNSS_TX);
  GNSS.setRxBufferSize(2048);

#if DEBUG_ROVER
  Serial.println("\n[boot] truck " TRUCK_ID
                 " slot offset +" + String(SLOT_OFFSET_MS) + " ms");
#endif

  bool ok = e22Configure();
#if DEBUG_ROVER
  Serial.println(ok ? "[e22 ] configured: ch65 netid10 38.4k 240B transparent"
                    : "[e22 ] CONFIG READ-BACK MISMATCH - check wiring/manual");
#endif
  for (int i = 0; i < (ok ? 4 : 12); i++) {
    digitalWrite(PIN_LED, i % 2);
    delay(ok ? 250 : 90);
  }
  digitalWrite(PIN_LED, LOW);
  randomSeed(esp_random());
}

void loop() {
  uint32_t now = millis();

  // ── Corrections: E22 -> LC29H, tracking the burst for the beacon ────────
  while (E22.available()) {
    GNSS.write(E22.read());
    rtcmIn++;
    burstBytes++;
    lastRxByte = now;
  }

  // Burst ended? That silence edge is the shared metronome.
  if (burstBytes >= BURST_MIN_BYTES && now - lastRxByte > BURST_GAP_MS) {
    beaconAt = now;
    slotArmed = true;
    burstBytes = 0;
  } else if (burstBytes > 0 && now - lastRxByte > 500) {
    burstBytes = 0;              // stray bytes, not an epoch - discard count
  }

  // ── Position: LC29H -> queue the newest GGA ─────────────────────────────
  while (GNSS.available()) {
    char c = GNSS.read();
    if (c == '\n') handleNmeaLine();
    else if (c != '\r' && nmeaLen < sizeof(nmeaLine) - 1) nmeaLine[nmeaLen++] = c;
    else if (nmeaLen >= sizeof(nmeaLine) - 1) nmeaLen = 0;
  }

  // ── Uplink in OUR slot after the beacon ─────────────────────────────────
  if (slotArmed && now - beaconAt >= SLOT_OFFSET_MS) {
    slotArmed = false;
    sendUplink(now);
  }

  // ── Shadowed fallback: no beacon heard, free-run with jitter ────────────
  if (now - beaconAt > FALLBACK_AFTER_MS &&
      now - lastUplink > FALLBACK_PERIOD_MS + (uint32_t)random(0, 400)) {
    sendUplink(now);
  }
}
