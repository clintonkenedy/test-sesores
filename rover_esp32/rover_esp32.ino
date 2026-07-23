/*
 * RTK rover bridge for ESP32 + Quectel LC29H (rover mode) - verbose build.
 *
 * Pulls RTCM3 corrections from the base machine over WiFi and injects them into
 * the LC29H, which computes the RTK fix. Reads the receiver's NMEA back, prints
 * a detailed log over USB, and reports status to the base over the same TCP
 * link so the rover can be watched from the base PC with no computer at the rover.
 *
 * This is the WiFi stand-in for the final LoRa link.
 *
 * Data paths:
 *   corrections   WiFi (TCP) -> ESP32 -> Serial2 (TX2/GPIO17) -> LC29H
 *   position      LC29H -> Serial2 (RX2/GPIO16) -> ESP32 -> USB + TCP back to base
 *
 * Wiring (ESP32 DevKit / WROOM):
 *   LC29H TX  -> ESP32 GPIO16 (RX2)   <- the "listen" wire
 *   LC29H RX  -> ESP32 GPIO17 (TX2)   <- the "correction" wire
 *   GND       -> GND (common ground required)
 *
 * The base machine must be running:  python rtcm_to_lora.py --mode server
 */

#include <WiFi.h>

// ===========================================================================
//  CONFIGURATION
// ===========================================================================

const char* WIFI_SSID = "Starlink";
const char* WIFI_PASS = "starlinkmelvin";

const char* BASE_HOST = "192.168.1.162";
const uint16_t BASE_PORT = 8887;

const uint32_t GNSS_BAUD = 115200;   // must match the LC29H
const int PIN_RX2 = 16;              // ESP32 <- LC29H TX
const int PIN_TX2 = 17;              // ESP32 -> LC29H RX

const uint32_t RECONNECT_DELAY_MS = 3000;
const uint32_t HEARTBEAT_MS = 5000;      // "still here" to the base this often
const uint32_t HEALTH_MS = 2000;         // local health log this often

// ===========================================================================

WiFiClient base;
char nmeaLine[128];
size_t nmeaLen = 0;
float lastHSigma = -1.0f;

// Counters so the logs show whether data is actually moving.
uint32_t rtcmBytesIn = 0;      // corrections received from base
uint32_t gnssBytesIn = 0;      // bytes heard from the LC29H
uint32_t nmeaLines = 0;        // complete NMEA sentences parsed
uint32_t lastHealth = 0;
uint32_t lastHeartbeat = 0;
uint32_t heartbeatSeq = 0;

// ---------------------------------------------------------------------------

const char* fixName(int quality) {
  switch (quality) {
    case 0: return "NO FIX";
    case 1: return "SINGLE";
    case 2: return "DGPS";
    case 4: return "RTK FIXED";
    case 5: return "RTK FLOAT";
    case 6: return "DEAD RECKON";
    default: return "?";
  }
}

// Send one line to the base over TCP (if connected) and echo it to USB.
void toBase(const char* line) {
  Serial.print("[->base] ");
  Serial.println(line);
  if (base.connected()) {
    base.println(line);
  }
}

bool nmeaField(const char* line, int n, char* out, size_t outSize) {
  int field = 0;
  const char* p = line;
  while (field < n) {
    p = strchr(p, ',');
    if (!p) return false;
    p++;
    field++;
  }
  const char* end = strchr(p, ',');
  size_t len = end ? (size_t)(end - p) : strlen(p);
  if (len >= outSize) len = outSize - 1;
  memcpy(out, p, len);
  out[len] = '\0';
  return true;
}

// GGA carries the fix. Report it to USB and back to the base.
void reportGGA(const char* line) {
  char quality[8], sats[8], age[12];
  if (!nmeaField(line, 6, quality, sizeof(quality))) return;
  nmeaField(line, 7, sats, sizeof(sats));
  nmeaField(line, 13, age, sizeof(age));

  char out[96];
  int n = snprintf(out, sizeof(out), "FIX %s sats=%s age=%ss",
                   fixName(atoi(quality)),
                   sats[0] ? sats : "?",
                   age[0] ? age : "?");
  if (lastHSigma >= 0.0f && n > 0 && n < (int)sizeof(out)) {
    snprintf(out + n, sizeof(out) - n, " h+/-%.3fm", lastHSigma);
  }
  toBase(out);
}

// GST carries precision; store it to report on the next GGA.
void reportGST(const char* line) {
  char latSigma[16], lonSigma[16];
  if (!nmeaField(line, 6, latSigma, sizeof(latSigma))) return;
  if (!nmeaField(line, 7, lonSigma, sizeof(lonSigma))) return;
  if (!latSigma[0] || !lonSigma[0]) return;
  float la = atof(latSigma), lo = atof(lonSigma);
  lastHSigma = sqrtf(la * la + lo * lo);
}

void handleNmeaLine() {
  nmeaLine[nmeaLen] = '\0';
  nmeaLines++;
  // Log every sentence raw so it is obvious the LC29H is being heard.
  Serial.print("[gnss ] ");
  Serial.println(nmeaLine);
  if (strstr(nmeaLine, "GGA")) reportGGA(nmeaLine);
  else if (strstr(nmeaLine, "GST")) reportGST(nmeaLine);
  nmeaLen = 0;
}

// ---------------------------------------------------------------------------

void connectWiFi() {
  Serial.print("[wifi ] connecting to ");
  Serial.println(WIFI_SSID);
  WiFi.mode(WIFI_STA);
  WiFi.begin(WIFI_SSID, WIFI_PASS);
  uint32_t dots = 0;
  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
    if (++dots % 20 == 0) Serial.println();
  }
  Serial.print("\n[wifi ] connected  IP=");
  Serial.print(WiFi.localIP());
  Serial.print("  RSSI=");
  Serial.print(WiFi.RSSI());
  Serial.println(" dBm");
}

void connectBase() {
  while (!base.connected()) {
    Serial.print("[base ] connecting to ");
    Serial.print(BASE_HOST);
    Serial.print(":");
    Serial.println(BASE_PORT);
    if (base.connect(BASE_HOST, BASE_PORT)) {
      base.setNoDelay(true);
      Serial.println("[base ] CONNECTED - corrections flowing");
      toBase("HELLO from rover - connected to base over TCP");
      return;
    }
    Serial.println("[base ] unreachable, retrying");
    delay(RECONNECT_DELAY_MS);
  }
}

// Periodic "still here" so the base sees the rover is alive even before a fix.
void heartbeat() {
  uint32_t now = millis();
  if (now - lastHeartbeat < HEARTBEAT_MS) return;
  lastHeartbeat = now;
  char msg[96];
  snprintf(msg, sizeof(msg),
           "HELLO seq=%lu up=%lus rtcm_in=%luB gnss_in=%luB nmea=%lu",
           (unsigned long)++heartbeatSeq,
           (unsigned long)(now / 1000),
           (unsigned long)rtcmBytesIn,
           (unsigned long)gnssBytesIn,
           (unsigned long)nmeaLines);
  toBase(msg);
}

// Periodic local health log to USB.
void health() {
  uint32_t now = millis();
  if (now - lastHealth < HEALTH_MS) return;
  lastHealth = now;
  Serial.print("[health] wifi=");
  Serial.print(WiFi.status() == WL_CONNECTED ? "up" : "DOWN");
  Serial.print(" base=");
  Serial.print(base.connected() ? "up" : "DOWN");
  Serial.print(" rtcm_in=");
  Serial.print(rtcmBytesIn);
  Serial.print("B gnss_in=");
  Serial.print(gnssBytesIn);
  Serial.print("B nmea_lines=");
  Serial.println(nmeaLines);
  if (gnssBytesIn == 0) {
    Serial.println("[health] WARNING: nothing heard from LC29H - "
                   "check TX->GPIO16, GND and baud");
  }
}

void setup() {
  Serial.begin(115200);
  delay(300);
  Serial2.begin(GNSS_BAUD, SERIAL_8N1, PIN_RX2, PIN_TX2);
  Serial.println("\n========================================");
  Serial.println(" RTK rover bridge (verbose) starting");
  Serial.println(" watch the fix: SINGLE -> FLOAT -> FIXED");
  Serial.println("========================================\n");
  connectWiFi();
  connectBase();
}

void loop() {
  if (WiFi.status() != WL_CONNECTED) {
    Serial.println("[wifi ] dropped - reconnecting");
    connectWiFi();
  }
  if (!base.connected()) {
    Serial.println("[base ] link lost - reconnecting");
    connectBase();
  }

  // Corrections in: WiFi -> LC29H.
  while (base.available()) {
    Serial2.write(base.read());
    rtcmBytesIn++;
  }

  // Position out: LC29H -> USB + base, one NMEA line at a time.
  while (Serial2.available()) {
    char c = Serial2.read();
    gnssBytesIn++;
    if (c == '\n') {
      handleNmeaLine();
    } else if (c != '\r' && nmeaLen < sizeof(nmeaLine) - 1) {
      nmeaLine[nmeaLen++] = c;
    } else if (nmeaLen >= sizeof(nmeaLine) - 1) {
      nmeaLen = 0;
    }
  }

  heartbeat();
  health();
}
