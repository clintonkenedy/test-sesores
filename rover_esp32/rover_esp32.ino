/*
 * RTK rover bridge for ESP32 + Quectel LC29H (rover mode).
 *
 * Pulls RTCM3 corrections from the base machine over WiFi and injects them into
 * the LC29H, which computes the RTK fix. Reads the receiver's NMEA back and
 * prints a fix summary over USB so the solution can be watched during testing.
 *
 * This is the WiFi stand-in for the final LoRa link: WiFi here is later replaced
 * by the radio, and everything else stays the same.
 *
 * Data paths:
 *   corrections   WiFi (TCP) -> ESP32 -> Serial2 (TX2) -> LC29H
 *   position      LC29H -> Serial2 (RX2) -> ESP32 -> USB Serial (monitor)
 *
 * Wiring (ESP32 DevKit / WROOM):
 *   LC29H TX  -> ESP32 GPIO16 (RX2)
 *   LC29H RX  -> ESP32 GPIO17 (TX2)   <- the correction wire; easy to forget
 *   GND       -> GND (common ground required)
 *
 * The base machine must be running:  python rtcm_to_lora.py --mode server
 * The LC29H must be configured in ROVER mode and emitting GGA (GST optional).
 */

#include <WiFi.h>

// ===========================================================================
//  CONFIGURATION - edit these for your network and base machine.
// ===========================================================================

const char* WIFI_SSID = "Starlink";
const char* WIFI_PASS = "starlinkmelvin";

// The base PC running rtcm_to_lora.py in server mode. Find its IP with ipconfig.
const char* BASE_HOST = "192.168.1.162";
const uint16_t BASE_PORT = 8887;

const uint32_t GNSS_BAUD = 115200;   // must match the LC29H
const int PIN_RX2 = 16;              // ESP32 <- LC29H TX
const int PIN_TX2 = 17;              // ESP32 -> LC29H RX

const uint32_t RECONNECT_DELAY_MS = 3000;

// ===========================================================================

WiFiClient base;
char nmeaLine[128];
size_t nmeaLen = 0;
float lastHSigma = -1.0f;   // last horizontal precision seen, metres

const char* fixName(int quality) {
  switch (quality) {
    case 0: return "NO FIX";
    case 1: return "SINGLE";      // autonomous, a few metres
    case 2: return "DGPS";        // ~1 m
    case 4: return "RTK FIXED";   // ~1-2 cm  <- the goal
    case 5: return "RTK FLOAT";   // ~10-50 cm
    case 6: return "DEAD RECKON";
    default: return "?";
  }
}

// Pull comma-separated field n (0-based) out of an NMEA line into out.
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

// GGA carries the fix: quality (field 6), satellites (7), correction age (13).
// The summary goes to USB and back to the base over the same TCP link, so the
// fix can be watched on the base PC with no computer at the rover.
void reportGGA(const char* line) {
  char quality[8], sats[8], age[12];
  if (!nmeaField(line, 6, quality, sizeof(quality))) return;
  nmeaField(line, 7, sats, sizeof(sats));
  nmeaField(line, 13, age, sizeof(age));   // stays near 0-1 s if corrections arrive

  char out[96];
  int n = snprintf(out, sizeof(out), "FIX %s sats=%s age=%ss",
                   fixName(atoi(quality)),
                   sats[0] ? sats : "?",
                   age[0] ? age : "?");
  if (lastHSigma >= 0.0f && n > 0 && n < (int)sizeof(out)) {
    snprintf(out + n, sizeof(out) - n, " h+/-%.3fm", lastHSigma);
  }

  Serial.println(out);
  if (base.connected()) {
    base.println(out);
  }
}

// GST carries precision: lat/lon std deviations (fields 6 and 7), in metres.
void reportGST(const char* line) {
  char latSigma[16], lonSigma[16];
  if (!nmeaField(line, 6, latSigma, sizeof(latSigma))) return;
  if (!nmeaField(line, 7, lonSigma, sizeof(lonSigma))) return;
  if (!latSigma[0] || !lonSigma[0]) return;
  float la = atof(latSigma), lo = atof(lonSigma);
  lastHSigma = sqrtf(la * la + lo * lo);   // reported on the next GGA line
}

void handleNmeaLine() {
  nmeaLine[nmeaLen] = '\0';
  if (strstr(nmeaLine, "GGA")) reportGGA(nmeaLine);
  else if (strstr(nmeaLine, "GST")) reportGST(nmeaLine);
  nmeaLen = 0;
}

void connectWiFi() {
  Serial.print("WiFi: connecting to ");
  Serial.println(WIFI_SSID);
  WiFi.mode(WIFI_STA);
  WiFi.begin(WIFI_SSID, WIFI_PASS);
  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
  }
  Serial.print("\nWiFi: connected, IP ");
  Serial.println(WiFi.localIP());
}

void connectBase() {
  while (!base.connected()) {
    Serial.print("base: connecting to ");
    Serial.print(BASE_HOST);
    Serial.print(":");
    Serial.println(BASE_PORT);
    if (base.connect(BASE_HOST, BASE_PORT)) {
      base.setNoDelay(true);   // send corrections now, do not coalesce
      Serial.println("base: connected - corrections flowing");
      return;
    }
    Serial.println("base: unreachable, retrying");
    delay(RECONNECT_DELAY_MS);
  }
}

void setup() {
  Serial.begin(115200);
  Serial2.begin(GNSS_BAUD, SERIAL_8N1, PIN_RX2, PIN_TX2);
  Serial.println("\nRTK rover bridge starting");
  Serial.println("Watch the fix climb: SINGLE -> RTK FLOAT -> RTK FIXED\n");
  connectWiFi();
  connectBase();
}

void loop() {
  if (WiFi.status() != WL_CONNECTED) {
    Serial.println("WiFi: dropped");
    connectWiFi();
  }
  if (!base.connected()) {
    Serial.println("base: link lost");
    connectBase();
  }

  // Corrections in: WiFi -> LC29H.
  while (base.available()) {
    Serial2.write(base.read());
  }

  // Position out: LC29H -> USB, one NMEA line at a time.
  while (Serial2.available()) {
    char c = Serial2.read();
    if (c == '\n') {
      handleNmeaLine();
    } else if (c != '\r' && nmeaLen < sizeof(nmeaLine) - 1) {
      nmeaLine[nmeaLen++] = c;
    } else if (nmeaLen >= sizeof(nmeaLine) - 1) {
      nmeaLen = 0;   // overlong line, resync on the next newline
    }
  }
}
